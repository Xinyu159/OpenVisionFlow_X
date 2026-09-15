/**
 * @file audit_worker.cpp
 * @brief 父进程：起子进程、读管道、判死因、算档位
 *
 * ============================================================================
 * 这里的每一件事都围绕一个目标：**归因准确**
 * ============================================================================
 *
 * 子进程死掉之后没机会说话，所以父进程必须能从"它留下的最后一行"倒推出
 * 它是怎么死的。三条规矩：
 *
 *  1. **begin/end 必须配对。** 一个 begin 后面没有配对的 end = 子进程死在这一步。
 *     父进程因此能说出"`SurfaceRoughnessInspection` 死在 S5、参数
 *     `sampling_interval=0`"，而不是笼统的"这个算子会崩"。
 *
 *  2. **预算造成的失败必须和真缺陷分开。** 子进程的内存上限是体检自己设的，
 *     一个算子因为 1GB 上限而 OOM，那是**体检的预算**，不是它的缺陷。
 *     所以凡是按信号死掉的，都把那一步**单独**重跑一次、内存上限放大 4 倍：
 *     活了 → RESOURCE_LIMIT（C 档）；还是同样的死法 → 真缺陷（D 档）。
 *     见 `recheck_with_more_memory()`。
 *
 *  3. **一个算子只有一个写者。** 同一个 type_id 的多次 fork（重启、复检）
 *     全部串行进行，verdict 只在最后 `finalize()` 一次。并发写会让档位
 *     取决于谁先跑完 —— 那比没有体检报告还糟。
 *
 * 顺带一个自检：END 行里带着子进程认为的场景名，父进程拿它和自己那份
 * `plan_steps()` 对比。**对不上就记 parse_error** —— 因为"父进程和子进程
 * 枚举的计划必须逐项相等"是整个归因机制的假设，这个假设破了得有人知道。
 */

#include "audit.h"

#include "nlohmann/json.hpp"

#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>

namespace ovf {
namespace audit {

using Clock = std::chrono::steady_clock;

static int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now().time_since_epoch())
        .count();
}

int default_jobs() {
    long n = ::sysconf(_SC_NPROCESSORS_ONLN);
    if (n < 1) n = 1;
    if (n > 32) n = 32;   // 32 个并发 fork 已经远超收益，再多只是互相抢内存
    return static_cast<int>(n);
}

namespace {

String signal_name(int sig) {
    switch (sig) {
        case SIGSEGV: return "SIGSEGV";
        case SIGABRT: return "SIGABRT";
        case SIGBUS:  return "SIGBUS";
        case SIGFPE:  return "SIGFPE";
        case SIGILL:  return "SIGILL";
        case SIGKILL: return "SIGKILL";
        case SIGXCPU: return "SIGXCPU";
        case SIGXFSZ: return "SIGXFSZ";
        case SIGSYS:  return "SIGSYS";
        case SIGPIPE: return "SIGPIPE";
        case SIGTERM: return "SIGTERM";
        default: {
            const char* s = ::strsignal(sig);
            return s ? String(s) : ("signal " + std::to_string(sig));
        }
    }
}

Status parse_status(const String& s) {
    if (s == "ok")   return Status::Ok;
    if (s == "fail") return Status::Fail;
    if (s == "warn") return Status::Warn;
    return Status::Skip;
}

// ===========================================================================
// 一个子进程留下的全部痕迹
// ===========================================================================

struct ChildState {
    String type_id;
    int    start_step = 0;

    Vector<Evidence> evidence;

    /// 最后一个**没有配对 end** 的 begin —— 子进程就是死在它上面
    int last_begin = -1;
    int last_end   = -1;
    int last_step() const { return last_begin >= 0 ? last_begin : last_end; }

    bool    aborted = false;
    String  abort_why;
    bool    seen_done = false;
    int     done_steps = 0;
    int64_t budget_skipped = 0;

    String category;
    int    meta_in = 0, meta_out = 0, meta_par = 0;

    /// 子进程**实际**生效的内存上限（可能因为父进程自己占太多而被放大）
    int    effective_mem_mb = 0;
    String rlimit_warning;

    /// 父子两份计划对不上，或者行读坏了 —— 归因链断了，得报出来
    String parse_error;
};

/**
 * @brief 解析一行，填进 ChildState
 *
 * @param plan 父进程自己枚举的那份计划。步的场景/端口/参数**全部从这里取**，
 *             不听子进程的（子进程只在 END 行里回报一个场景名用于交叉核对）。
 */
void parse_line(ChildState& st, const Vector<Step>& plan, const String& line) {
    try {
        nlohmann::json j = nlohmann::json::parse(line);
        const String k = j.value("k", String(""));

        if (k == "begin") {
            st.last_begin = j.value("i", -1);
            return;
        }
        if (k == "end") {
            const int i = j.value("i", -1);
            Evidence e;
            if (i >= 0 && i < static_cast<int>(plan.size())) {
                const Step& step = plan[i];
                e.scenario = step.scenario;
                e.flavour  = step.flavour;
                e.port     = step.port;
                e.param    = step.param;
                e.value    = step.value_label;

                // 交叉核对：子进程说它跑的是哪个场景，和父进程以为的一致吗？
                const String got = j.value("s", String(""));
                if (!got.empty() && got != scenario_name(step.scenario) &&
                    st.parse_error.empty()) {
                    st.parse_error = "父子计划不一致：第 " + std::to_string(i) +
                                     " 步父进程以为是 " + scenario_name(step.scenario) +
                                     "，子进程说是 " + got;
                }
            } else if (st.parse_error.empty()) {
                st.parse_error = "end 行的步号 " + std::to_string(i) +
                                 " 超出计划范围(" + std::to_string(plan.size()) + ")";
            }
            e.status     = parse_status(j.value("st", String("")));
            e.error_code = j.value("ec", 0);
            e.ms         = j.value("ms", 0);
            e.detail     = j.value("m", String(""));

            st.evidence.push_back(e);
            st.last_begin = -1;
            st.last_end   = i;
            return;
        }
        if (k == "meta") {
            st.category = j.value("cat", String(""));
            st.meta_in  = j.value("in", 0);
            st.meta_out = j.value("out", 0);
            st.meta_par = j.value("par", 0);
            st.effective_mem_mb = j.value("mem", 0);
            return;
        }
        if (k == "budget") {
            if (j.contains("skipped")) st.budget_skipped = j.value("skipped", 0);
            const String rl = j.value("rlimit", String(""));
            if (!rl.empty()) st.rlimit_warning = rl;
            return;
        }
        if (k == "abort") {
            st.aborted   = true;
            st.abort_why = j.value("why", String(""));
            return;
        }
        if (k == "done") {
            st.seen_done  = true;
            st.done_steps = j.value("n", 0);
            return;
        }
    } catch (const std::exception& ex) {
        // 解析不出来 = 行写坏了或者漏了字段。**要报出来**，不能默默吞掉 ——
        // 归因链断了一环，这一轮的结论就不能全信了。
        if (st.parse_error.empty()) {
            st.parse_error = String("解析失败: ") + ex.what() +
                             " | 行: " + line.substr(0, 120);
        }
    }
}

/// 把缓冲区里完整的行都吃掉
void feed_lines(ChildState& st, const Vector<Step>& plan, String& buf) {
    size_t start = 0;
    for (;;) {
        const size_t nl = buf.find('\n', start);
        if (nl == String::npos) break;
        const String line = buf.substr(start, nl - start);
        start = nl + 1;
        if (!line.empty()) parse_line(st, plan, line);
    }
    buf.erase(0, start);
}

// ===========================================================================
// 起子进程
// ===========================================================================

/// 父进程手里代表"一个正在跑的子进程"的全部状态
struct Child {
    pid_t   pid = -1;
    int     fd  = -1;
    int64_t spawned_at = 0;

    /// 是我们按预算杀的（→ TIMEOUT），还是它自己死的
    bool killed_by_us = false;

    /// 读到了文件结尾 = 它正在退出
    bool eof = false;

    /// 行缓冲。子进程一次 write 一行（< PIPE_BUF 所以是原子的），
    /// 但 read 可能一次拿到半行，所以必须攒着。
    String buf;

    ChildState st;
};

struct SpawnRequest {
    String type_id;
    int    start_step = 0;
    int    stop_after_step = -1;
    int    mem_limit_mb = 0;   //!< 0 = 用 Options 里的
};

bool spawn_child(const SpawnRequest& req, const Options& opts, Child& out) {
    int p[2];
    if (::pipe(p) != 0) return false;

    // fork 之前把父进程 stdio 的缓冲冲干净。不冲的话，子进程会继承一份
    // 待输出的缓冲副本，然后（在 dup2 之后）把它冲进 /dev/null ——
    // 父进程自己那份不受影响，但重复输出/丢输出的组合很难查，索性冲干净。
    std::fflush(nullptr);

    const pid_t pid = ::fork();
    if (pid < 0) {
        ::close(p[0]);
        ::close(p[1]);
        return false;
    }

    if (pid == 0) {
        // ---------- 子进程 ----------
        ::close(p[0]);          // 子进程只写不读
        Options child_opts       = opts;
        child_opts.start_step    = req.start_step;
        child_opts.stop_after_step = req.stop_after_step;
        if (req.mem_limit_mb > 0) child_opts.mem_limit_mb = req.mem_limit_mb;

        run_child(req.type_id, child_opts, p[1]);
        ::_exit(0);             // run_child 不返回；这行只是保险
    }

    // ---------- 父进程 ----------
    // ★ 必须关掉写端。不关的话，即使所有子进程都退出了，管道也不会出现
    //   文件结尾（EOF）—— 因为内核看到"写端还有人持有"，父进程就会在 poll
    //   里永远等下去。这是这类程序最经典的一个死锁。
    ::close(p[1]);

    out.pid        = pid;
    out.fd         = p[0];
    out.spawned_at = now_ms();
    out.killed_by_us = false;
    out.eof        = false;
    out.buf.clear();
    out.st         = ChildState{};
    out.st.type_id    = req.type_id;
    out.st.start_step = req.start_step;
    return true;
}

/// 把管道里现成的内容读干；返回 false 表示读到 EOF
void drain(Child& c, const Vector<Step>& plan) {
    char tmp[8192];
    for (;;) {
        const ssize_t n = ::read(c.fd, tmp, sizeof(tmp));
        if (n > 0) {
            c.buf.append(tmp, static_cast<size_t>(n));
            feed_lines(c.st, plan, c.buf);
            if (static_cast<size_t>(n) < sizeof(tmp)) return;   // 大概是读完了
            continue;
        }
        if (n == 0) {   // EOF：所有写端都关了 = 子进程正在退出
            c.eof = true;
            return;
        }
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        c.eof = true;   // 读错了也当结束处理，别在这里卡住
        return;
    }
}

struct ExitInfo {
    bool   signaled = false;
    int    signal = 0;
    bool   exited = false;
    int    code = 0;
    String kind;      //!< "TIMEOUT" / "SIGSEGV" / "EXIT:1" / ""
};

ExitInfo wait_and_classify(Child& c) {
    int status = 0;
    for (;;) {
        const pid_t r = ::waitpid(c.pid, &status, 0);
        if (r == c.pid) break;
        if (r < 0 && errno == EINTR) continue;
        break;   // 不该发生；真发生了就当"没有退出信息"处理
    }
    ::close(c.fd);
    c.fd = -1;

    ExitInfo info;
    if (WIFSIGNALED(status)) {
        info.signaled = true;
        info.signal   = WTERMSIG(status);
        if (c.killed_by_us) {
            // 父进程按墙钟预算杀的 —— 这是**挂死**，不是它自己崩的
            info.kind = "TIMEOUT";
        } else if (info.signal == SIGKILL) {
            // 不是我们杀的 SIGKILL：多半是内核 OOM killer。
            // 它可能真的是因为算子吃太多内存，但也可能是机器本来就紧张 ——
            // 所以后面一定会走"放大内存复检"。
            info.kind = "SIGKILL(外部，疑似 OOM killer)";
        } else {
            info.kind = signal_name(info.signal);
        }
    } else if (WIFEXITED(status)) {
        info.exited = true;
        info.code   = WEXITSTATUS(status);
        info.kind   = info.code == 0 ? "" : ("EXIT:" + std::to_string(info.code));
    } else {
        info.kind = "UNKNOWN";
    }
    return info;
}

// ===========================================================================
// 复检：这一步到底是"算子有毛病"还是"体检给的内存太小"
// ===========================================================================

/**
 * @brief 用 4 倍内存上限，**只重跑死掉的那一步**
 *
 * 判据很干脆：
 *   这一步跑出了 end 行  →  RESOURCE_LIMIT，是体检的预算不够，不是算子缺陷
 *   还是同样的死法       →  真缺陷
 *
 * @return true 表示"确认是资源上限问题"
 */
bool recheck_with_more_memory(const String& type_id, int step, int old_mem_mb,
                              const Options& opts) {
    const int new_mem = old_mem_mb > 0 ? old_mem_mb * 4 : opts.mem_limit_mb * 4;

    SpawnRequest req;
    req.type_id          = type_id;
    req.start_step       = step;
    req.stop_after_step  = step;
    req.mem_limit_mb     = new_mem;

    Child c;
    if (!spawn_child(req, opts, c)) return false;

    // 阻塞地把管道读完 —— 只有一个算子、只有一步，不值得为它维护异步状态机。
    // 其余子进程会在这一小会儿里继续跑（它们有自己的进程，不受影响），
    // 只是父进程暂时不 poll 它们而已。
    const NodeInfo* info = NodeFactory::instance().get_info(type_id);
    const Vector<Step> plan = info ? plan_steps(*info, opts) : Vector<Step>{};

    const int64_t deadline = now_ms() + opts.budget_ms + opts.hard_grace_ms;
    for (;;) {
        struct pollfd pfd{c.fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 200);
        if (pr > 0) {
            drain(c, plan);
            if (c.eof) break;
        } else if (pr < 0 && errno != EINTR) {
            break;
        }
        if (now_ms() > deadline) {
            ::kill(c.pid, SIGKILL);
            c.killed_by_us = true;
            for (;;) {   // 杀完把剩下的读干，然后一定会 EOF
                drain(c, plan);
                if (c.eof) break;
                struct pollfd p2{c.fd, POLLIN, 0};
                if (::poll(&p2, 1, 500) <= 0) break;
            }
            break;
        }
    }
    wait_and_classify(c);

    // 判据：死掉**那一步**有没有跑出结论
    for (const auto& e : c.st.evidence) {
        if (e.scenario == plan[step].scenario && e.status != Status::Skip) return true;
    }
    return false;
}

// ===========================================================================
// 档位
// ===========================================================================

/**
 * @brief 定档。★ 完整判据见 docs/hardening/stage5_node_audit.md，这里是可执行版
 *
 *   D  崩了 / 挂了 / 构造不出来                    —— 别碰
 *   C  不崩不挂，但金丝雀这一档就跑不通、或声明了输出却不产出、
 *      或只在放大内存后才活下来（RESOURCE_LIMIT）      —— 用前先试
 *   B  不崩不挂、金丝雀能跑通，但元数据有 Error 级问题   —— 能用，元数据要修
 *   A  以上都没有                                    —— 放心用
 *
 * ★ 两处刻意的判断（和计划书的一行版不同，理由写在这）：
 *
 *  1. **S1/S2/S3/S5 的干净 `Result::failure` 不算问题。**
 *     "没接输入直接执行"失败、"喂了错类型"失败、"参数给 -1"失败 ——
 *     这些**正是应该发生的事**。把它们算成缺陷，会让 501 个算子里
 *     绝大多数掉进 C 档，档位就变成噪音了。
 *     只有 S4（金丝雀：输入齐、类型对、尺寸合法）跑不通才是真信号。
 *
 *  2. **只有 `Severity::Error` 级的元数据问题才影响档位。**
 *     Note 级是"没填作者/版本"这种，501 个上游算子里几乎人人都有 ——
 *     算进去的话 A 档会是空的。Note 级的条数在报告里单列一栏，信息不丢。
 */
void finalize(NodeVerdict& v) {
    bool canary_failed  = false;
    bool output_missing = false;
    bool meta_error     = false;
    int  meta_notes     = 0;

    for (const auto& e : v.evidence) {
        switch (e.scenario) {
            case Scenario::Canary:
                if (e.status == Status::Fail) canary_failed = true;
                break;
            case Scenario::Outputs:
                // Skip = 金丝雀就没跑通，这条判断不了（S4 已经记过了）
                if (e.status == Status::Fail || e.status == Status::Warn) output_missing = true;
                break;
            case Scenario::Metadata:
                if (e.status == Status::Warn) meta_error = true;
                break;
            case Scenario::Construct:
                if (e.status == Status::Warn) meta_error = true;   // info().id 对不上
                break;
            default:
                break;
        }
    }

    const bool fatal = (v.deaths > 0) || v.death_kind == "CONSTRUCT";

    if (fatal) {
        v.tier  = "D";
        v.score = node_health_score_for_tier(v.tier);
        String why = v.death_kind.empty() ? "崩溃" : v.death_kind;
        if (!v.death_scenario.empty()) why += " @ " + v.death_scenario;
        if (!v.death_param.empty())    why += " 参数 " + v.death_param +
                                              "=" + v.death_value;
        else if (!v.death_detail.empty()) why += " (" + v.death_detail + ")";
        v.note = why;
    } else if (v.resource_limit) {
        v.tier  = "C";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = "只在放大内存上限后才跑通 —— 疑似吃内存，也可能是体检预算不够。"
                  "原始死因: " + v.death_kind + " @" + v.death_scenario;
    } else if (v.death_kind == "SIGXCPU") {
        v.tier  = "C";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = "撞上 CPU 时间上限 —— 疑似死循环或极度耗时的算法";
    } else if (v.death_kind == "SIGXFSZ") {
        v.tier  = "C";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = "撞上单文件大小上限(16MB) —— 疑似把中间结果往磁盘上写";
    } else if (canary_failed) {
        v.tier  = "C";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = "金丝雀跑不通（输入齐全、类型正确、尺寸合法，仍然失败）";
    } else if (output_missing) {
        v.tier  = "C";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = "金丝雀执行成功，但声明过的输出端口没有产出";
    } else if (meta_error) {
        v.tier  = "B";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = "能跑通，元数据有 Error 级问题";
    } else {
        v.tier  = "A";
        v.score = node_health_score_for_tier(v.tier);
        v.note  = meta_notes > 0 ? "全过" : "全过";
    }

    if (v.steps_done < v.steps_planned) {
        v.note += "（跑了 " + std::to_string(v.steps_done) + "/" +
                  std::to_string(v.steps_planned) + " 步）";
    }
}

} // namespace

// ===========================================================================
// 主循环
// ===========================================================================

Vector<NodeVerdict> run_audit(const Vector<String>& type_ids, const Options& opts,
                              const ProgressFn& on_progress) {
    const int jobs = opts.jobs > 0 ? opts.jobs : default_jobs();

    Vector<NodeVerdict> out;
    out.reserve(type_ids.size());

    // 每个 type_id 一份计划 + 一份结论累加器。
    // 计划只算一次，父进程和子进程都用它 —— 这就是归因的"共同语言"。
    HashMap<String, Vector<Step>> plans;
    HashMap<String, NodeVerdict>  acc;
    HashMap<String, int>          restarts_used;

    std::deque<SpawnRequest> queue;
    for (const auto& id : type_ids) {
        const NodeInfo* info = NodeFactory::instance().get_info(id);

        NodeVerdict v;
        v.type_id = id;
        if (!info) {
            // 计划要体检一个没注册的 type_id：这是调用方的输入错误，不是缺陷
            v.tier = "D";
            v.score = node_health_score_for_tier(v.tier);
            v.note = "这个 type_id 没有注册（--only 写错了？）";
            acc[id] = v;
            continue;
        }
        plans[id] = plan_steps(*info, opts);
        v.category       = info->category;
        v.steps_planned  = static_cast<int>(plans[id].size());
        acc[id] = v;

        SpawnRequest req;
        req.type_id    = id;
        req.start_step = 0;
        queue.push_back(req);
    }

    size_t finished = 0;
    const size_t total = type_ids.size();

    Vector<Child> active;

    while (!queue.empty() || !active.empty()) {
        // ---- 补充并发数 ----
        while (static_cast<int>(active.size()) < jobs && !queue.empty()) {
            const SpawnRequest req = queue.front();
            queue.pop_front();

            Child c;
            const auto it = acc.find(req.type_id);
            if (it == acc.end() || plans.find(req.type_id) == plans.end()) continue;
            if (!spawn_child(req, opts, c)) {
                it->second.tier = "D";
                it->second.score = node_health_score_for_tier("D");
                it->second.note = "fork() 失败，体检工具自己起不了子进程";
                ++finished;
                if (on_progress) on_progress(it->second, finished, total);
                continue;
            }
            active.push_back(std::move(c));
        }
        if (active.empty()) break;

        // ---- 等一下 ----
        Vector<struct pollfd> pfds;
        pfds.reserve(active.size());
        for (auto& c : active) {
            struct pollfd p;
            p.fd = c.fd;
            p.events = POLLIN;
            p.revents = 0;
            pfds.push_back(p);
        }
        const int pr = ::poll(pfds.data(), pfds.size(), 200);
        if (pr > 0) {
            for (size_t i = 0; i < active.size(); ++i) {
                if (pfds[i].revents & (POLLIN | POLLHUP | POLLERR)) {
                    drain(active[i], plans[active[i].st.type_id]);
                }
            }
        } else if (pr < 0 && errno != EINTR) {
            // poll 本身出错了：不该发生。把活着的都杀了，避免无限循环。
            for (auto& c : active) { ::kill(c.pid, SIGKILL); c.killed_by_us = true; }
        }

        // ---- 墙钟预算到了就硬杀 ----
        const int64_t now = now_ms();
        for (auto& c : active) {
            if (c.fd >= 0 && !c.killed_by_us &&
                now - c.spawned_at > opts.budget_ms + opts.hard_grace_ms) {
                ::kill(c.pid, SIGKILL);
                c.killed_by_us = true;
            }
        }

        // ---- 收拾已经结束的 ----
        for (size_t i = 0; i < active.size();) {
            Child& c = active[i];
            if (!c.eof) {
                // 还没 EOF —— 它还在跑。**必须 ++i**，否则这个 for 永远原地打转。
                ++i;
                continue;
            }
            drain(c, plans[c.st.type_id]);   // 拿走最后几行
            const ExitInfo info = wait_and_classify(c);
            const String id = c.st.type_id;

            NodeVerdict& v = acc[id];
            const Vector<Step>& plan = plans[id];

            v.evidence.insert(v.evidence.end(),
                              c.st.evidence.begin(), c.st.evidence.end());
            v.steps_done  += static_cast<int>(c.st.evidence.size());
            if (!c.st.category.empty() && v.category.empty()) v.category = c.st.category;
            if (!c.st.parse_error.empty() && v.death_detail.empty()) {
                v.death_detail = c.st.parse_error;
            }

            if (c.st.aborted) {
                // 子进程自己说"没救了"：构造不出来。它走的是干净的 _exit(0)，
                // 所以信号和退出码都看不出来 —— 只能靠这行 abort。
                ++v.deaths;
                v.total_ms += static_cast<uint64_t>(
                    std::max<int64_t>(0, now_ms() - c.spawned_at));
                if (v.death_kind.empty()) {
                    v.death_kind     = "CONSTRUCT";
                    v.death_scenario = "S0";
                    v.death_detail   = c.st.abort_why;
                }
            } else if (info.signaled || !info.kind.empty()) {
                ++v.deaths;
                v.total_ms += static_cast<uint64_t>(
                    std::max<int64_t>(0, now_ms() - c.spawned_at));

                const int step = c.st.last_step();
                if (v.death_kind.empty()) {
                    v.death_kind = info.kind;
                    if (step >= 0 && step < static_cast<int>(plan.size())) {
                        v.death_scenario = scenario_name(plan[step].scenario);
                        v.death_param    = plan[step].param;
                        v.death_value    = plan[step].value_label;
                        if (plan[step].port.empty() == false) {
                            v.death_detail = "输入端口 " + plan[step].port;
                        } else if (!plan[step].flavour.empty()) {
                            v.death_detail = "金丝雀口味 " + plan[step].flavour;
                        }
                    } else {
                        v.death_scenario = "（还没开始就死了）";
                    }
                }

                // ★ 把这一处死亡**单独记成一条证据**。
                //   verdict 上的 death_* 只有一份，所以一个算子死在好几处时
                //   （SurfaceRoughnessInspection 实测死 4 处：小图金丝雀 +
                //   measure_length 的三个取值），报告只会报第一处 —— 读起来像是
                //   "修完这一个就没事了"，而事实正相反。每处死亡各占一行，
                //   才数得清有几个坑。
                //   注意：`end` 行才是"跑完了"，死亡是**没有配对的 begin**，
                //   所以这条证据父进程自己补，不来自子进程。
                if (step >= 0 && step < static_cast<int>(plan.size())) {
                    Evidence de;
                    de.scenario = plan[step].scenario;
                    de.flavour  = plan[step].flavour;
                    de.port     = plan[step].port;
                    de.param    = plan[step].param;
                    de.value    = plan[step].value_label;
                    de.status   = Status::Fail;
                    de.death    = true;
                    de.detail   = info.kind;
                    if (!plan[step].flavour.empty()) {
                        de.detail += " 金丝雀口味 " + plan[step].flavour;
                    } else if (!plan[step].port.empty()) {
                        de.detail += " 输入端口 " + plan[step].port;
                    }
                    de.ms = static_cast<int>(
                        std::max<int64_t>(0, now_ms() - c.spawned_at));
                    v.evidence.push_back(de);
                }

                // ---- 复检：这是算子的毛病，还是体检给的内存太小？----
                // 超时不用复检：加大内存治不好死循环。
                const bool worth_recheck =
                    (info.kind != "TIMEOUT") && !v.resource_limit &&
                    step >= 0 && step < static_cast<int>(plan.size()) &&
                    c.st.effective_mem_mb > 0;

                if (worth_recheck) {
                    if (recheck_with_more_memory(id, step, c.st.effective_mem_mb, opts)) {
                        v.resource_limit = true;
                        v.deaths = 0;      // 不是它的错，撤掉这次"死亡"记录
                        v.death_kind.clear();
                    }
                }
            } else {
                v.total_ms += static_cast<uint64_t>(
                    std::max<int64_t>(0, now_ms() - c.spawned_at));
            }

            if (c.st.budget_skipped > 0) {
                v.note = "预算用完，跳过 " + std::to_string(c.st.budget_skipped) + " 步";
            }

            // ---- 还要不要再 fork 一个接着跑？----
            const int step = c.st.last_step();
            const bool crashed = (v.deaths > 0);
            const bool more_steps = step >= 0 && step + 1 < static_cast<int>(plan.size());
            const int used = restarts_used.count(id) ? restarts_used[id] : 0;

            if (crashed && more_steps && !c.st.aborted && used < opts.max_restarts) {
                restarts_used[id] = used + 1;
                ++v.restarts;
                SpawnRequest req;
                req.type_id    = id;
                req.start_step = step + 1;
                queue.push_back(req);
            } else {
                // 到此为止 —— 归档
                if (crashed && more_steps && !c.st.aborted) {
                    v.note = "重启次数用尽(" + std::to_string(opts.max_restarts) +
                             ")，还有 " +
                             std::to_string(static_cast<int>(plan.size()) - step - 1) +
                             " 步没跑";
                }
                finalize(v);
                ++finished;
                if (on_progress) on_progress(v, finished, total);
            }

            active.erase(active.begin() + static_cast<long>(i));
        }
    }

    for (const auto& id : type_ids) out.push_back(acc[id]);
    return out;
}

} // namespace audit
} // namespace ovf
