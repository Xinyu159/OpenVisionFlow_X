/**
 * @file main.cpp
 * @brief ovf-node-audit 的命令行入口
 *
 * 用法速查：
 *
 *   ovf-node-audit --quick                     # 秒级，进 CTest
 *   ovf-node-audit --full                      # 全场景，几分钟
 *   ovf-node-audit --only Resize --verbose     # 盯一个算子
 *   ovf-node-audit --grep Morph                # 名字里带 Morph 的全部
 *   ovf-node-audit --full --update-annotations # 顺带刷新 health/node_health.json
 *   ovf-node-audit --list                      # 只列算子，不跑
 *
 * 退出码：默认**永远 0**（501 个老算子必然有 D 档，非零退出码会让 CTest 变红，
 * 那就没人看它了）。加了 `--fail-on-d` 才在出现 D 档时返回 1，给 CI 用。
 */

#include "audit.h"

#include <ovf/algorithm/algorithm.h>
#include <ovf/core/logger.h>

#ifdef OVF_WITH_USER_NODES
#include <ovf/nodes/nodes.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sys/stat.h>

using namespace ovf;
using namespace ovf::audit;

namespace {

void print_usage() {
    std::fprintf(stderr,
        "ovf-node-audit —— 把每个算子放进一次性子进程里跑一遍，扛得住崩溃和挂死\n"
        "\n"
        "用法: ovf-node-audit [选项]\n"
        "\n"
        "跑什么（默认 --full）:\n"
        "  --quick               只跑 S0/S0b/S1/S3，秒级；给 CTest 用\n"
        "  --full                S0–S6 全跑（含 256×256 的参数模糊测试）\n"
        "  --only <type_id>      只跑这个算子，可重复\n"
        "  --grep <子串>         type_id 里含这个子串的都跑，可重复\n"
        "  --list                只列出会跑哪些算子，不执行\n"
        "\n"
        "预算:\n"
        "  --jobs N              并行度，默认 = 核数\n"
        "  --budget-ms N         每个子进程的软预算，默认 2000\n"
        "  --mem-limit-mb N      子进程 RLIMIT_AS，默认 1024\n"
        "  --max-restarts N      崩溃后最多重启几次，默认 32（0 = 严格即停）\n"
        "  --fuzz-image-size N   S4/S5 金丝雀的图边长，默认 256\n"
        "  --source-dir DIR      ovf-algorithm/src，用于可中断性统计\n"
        "\n"
        "输出:\n"
        "  --reports-dir DIR     报告目录，默认 build/reports\n"
        "  --update-annotations  顺带写 health/node_health.json（进仓库的那份）\n"
        "  --json-out PATH       覆盖 JSON 报告路径\n"
        "  --md-out PATH         覆盖 Markdown 报告路径\n"
        "  --health-out PATH     覆盖标注表路径（隐含 --update-annotations）\n"
        "  --fail-on-d           出现 D 档时退出码 1（默认永远 0）\n"
        "  --verbose             逐算子打一行\n"
        "  --help\n");
}

bool mkdirs(const String& path) {
    // 够用就行：报告目录只有 build/reports 这种一层路径
    if (::mkdir(path.c_str(), 0755) == 0) return true;
    struct stat st;
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

int parse_int(const char* s, int fallback) {
    if (!s || !*s) return fallback;
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s) return fallback;
    return static_cast<int>(v);
}

} // namespace

int main(int argc, char** argv) {
    Options opts;
    opts.quick = false;   // 默认全量；--quick 才收窄
    bool list_only = false;
    Vector<String> greps;

    for (int i = 1; i < argc; ++i) {
        const String a = argv[i];
        auto next = [&](const char* what) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "错误: %s 后面缺参数\n", what);
                std::exit(2);
            }
            return argv[++i];
        };

        if      (a == "--quick")           opts.quick = true;
        else if (a == "--full")            opts.quick = false;
        else if (a == "--only")            opts.only.push_back(next("--only"));
        else if (a == "--grep")            greps.push_back(next("--grep"));
        else if (a == "--jobs")            opts.jobs = parse_int(next("--jobs"), 0);
        else if (a == "--budget-ms")       opts.budget_ms = parse_int(next("--budget-ms"), 2000);
        else if (a == "--mem-limit-mb")    opts.mem_limit_mb = parse_int(next("--mem-limit-mb"), 1024);
        else if (a == "--max-restarts")    opts.max_restarts = parse_int(next("--max-restarts"), 32);
        else if (a == "--fuzz-image-size") opts.fuzz_image_size = parse_int(next("--fuzz-image-size"), 256);
        else if (a == "--source-dir")      opts.source_dir = next("--source-dir");
        else if (a == "--reports-dir")     opts.reports_dir = next("--reports-dir");
        else if (a == "--json-out")        opts.json_out = next("--json-out");
        else if (a == "--md-out")          opts.md_out = next("--md-out");
        else if (a == "--health-out")    { opts.health_out = next("--health-out"); }
        else if (a == "--update-annotations") opts.health_out = "";   // 用默认路径，下面填
        else if (a == "--fail-on-d")       opts.fail_on_d = true;
        else if (a == "--verbose")         opts.verbose = true;
        else if (a == "--list")            list_only = true;
        else if (a == "--help" || a == "-h") { print_usage(); return 0; }
        else {
            std::fprintf(stderr, "错误: 不认识的选项 %s\n\n", a.c_str());
            print_usage();
            return 2;
        }
    }

    if (opts.budget_ms <= 0) opts.budget_ms = 2000;
    if (opts.jobs < 0) opts.jobs = 0;
    if (opts.max_restarts < 0) opts.max_restarts = 0;
    if (opts.fuzz_image_size < 8) opts.fuzz_image_size = 8;

    // ovf-algorithm/src：可中断性统计要扫它。编译期记下的仓库根优先，
    // 这样从任何 cwd 启动都找得到。
    if (opts.source_dir.empty()) {
#ifdef OVF_SOURCE_DIR
        opts.source_dir = String(OVF_SOURCE_DIR) + "/ovf-algorithm/src";
#else
        opts.source_dir = "ovf-algorithm/src";
#endif
    }

    if (opts.json_out.empty())   opts.json_out = opts.reports_dir + "/node_audit.json";
    if (opts.md_out.empty())     opts.md_out   = opts.reports_dir + "/node_audit.md";
    if (opts.health_out.empty()) {
#ifdef OVF_SOURCE_DIR
        opts.health_out = String(OVF_SOURCE_DIR) + "/health/node_health.json";
#else
        opts.health_out = "health/node_health.json";
#endif
    }

    // ---------- 先把 501 个算子弄进注册表 ----------
    //
    // 注册本身发生在静态初始化期（OVF_REGISTER_NODE 那套），main() 跑起来的
    // 时候其实已经满了。这两句调用的**真正作用不是注册，是 ODR-use** ——
    // 只要有一句引用到 libovf-algorithm.so / libovf-nodes.so 里的非弱符号，
    // 链接器的 --as-needed 就不敢把整个库当成"没人要"丢掉。
    // 库里一被丢，静态初始化不跑，注册表就是空的，体检报告会变成"0 个算子"
    // 而**链接期一个警告都没有**。这是阶段 4 花了一整个组合矩阵才证明的坑。
    ovf::algorithm::initialize_algorithm_module();
#ifdef OVF_WITH_USER_NODES
    ovf::nodes::initialize_user_nodes();
#endif

    auto& factory = NodeFactory::instance();
    Vector<String> all = factory.get_all_types();

    if (all.empty()) {
        std::fprintf(stderr,
            "致命: 注册表里一个算子都没有。这几乎总是意味着 libovf-algorithm.so\n"
            "      被链接器丢掉了（Ubuntu 默认开 --as-needed，且**链接期零警告**）。\n"
            "      检查 tools/node_audit/CMakeLists.txt 里的 ovf-algorithm 依赖还在不在。\n");
        return 3;
    }

    Vector<String> targets;
    if (!opts.only.empty()) {
        for (const auto& id : opts.only) targets.push_back(id);
    } else if (!greps.empty()) {
        for (const auto& id : all) {
            for (const auto& g : greps) {
                if (id.find(g) != String::npos) { targets.push_back(id); break; }
            }
        }
    } else {
        targets = all;
    }

    if (targets.empty()) {
        std::fprintf(stderr, "没有匹配到任何算子。用 --grep 换个关键词试试。\n");
        return 0;
    }

    if (list_only) {
        std::printf("%zu 个算子:\n", targets.size());
        for (const auto& id : targets) std::printf("  %s\n", id.c_str());
        return 0;
    }

    // 体检工具自己不需要日志刷屏 —— 每个算子的 INFO/WARN 已经在子进程里被沉掉了，
    // 父进程这边只要 ERROR 以上的。
    Logger::instance().set_level(LogLevel::Error);

    std::fprintf(stderr,
        "ovf-node-audit: %zu 个算子，模式 %s，并行 %d，每算子预算 %d ms，内存上限 %d MB\n",
        targets.size(), opts.quick ? "quick" : "full",
        opts.jobs > 0 ? opts.jobs : default_jobs(),
        opts.budget_ms, opts.mem_limit_mb);
    std::fprintf(stderr, "（--quick 只跑 S0/S0b/S1/S3；--full 含 256×256 参数模糊测试）\n\n");

    // ---------- 跑 ----------
    size_t last_print = 0;
    const auto t0 = std::chrono::steady_clock::now();

    const Vector<NodeVerdict> verdicts = run_audit(
        targets, opts,
        [&](const NodeVerdict& v, size_t done, size_t total) {
            if (!opts.verbose && done != total && done - last_print < std::max<size_t>(1, total / 20)) {
                return;   // 每 5% 打一次进度就够了
            }
            last_print = done;
            std::fprintf(stderr, "  [%zu/%zu] %-40s %s 档  %s\n",
                         done, total, v.type_id.c_str(), v.tier.c_str(),
                         v.note.empty() ? "" : v.note.substr(0, 60).c_str());
        });

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t0).count();

    // ---------- 可中断性 ----------
    int clean = 0, total_src = 0;
    const String interruptibility = scan_interruptibility(opts.source_dir, &clean, &total_src);

    // ---------- 写报告 ----------
    if (!mkdirs(opts.reports_dir)) {
        std::fprintf(stderr, "警告: 建不出报告目录 %s，报告只打到 stdout\n", opts.reports_dir.c_str());
    }

    auto write_file = [](const String& path, const String& body) -> bool {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out.is_open()) return false;
        out << body;
        return out.good();
    };

    const String json = verdicts_to_json(verdicts, opts, interruptibility);
    const String md   = verdicts_to_markdown(verdicts, opts, interruptibility);

    if (!write_file(opts.json_out, json)) {
        std::fprintf(stderr, "警告: 写不了 %s\n", opts.json_out.c_str());
    }
    if (!write_file(opts.md_out, md)) {
        std::fprintf(stderr, "警告: 写不了 %s\n", opts.md_out.c_str());
    }

    if (!opts.health_out.empty()) {
        // 标注表的目录可能还不存在（health/ 在仓库根）
        const size_t slash = opts.health_out.find_last_of('/');
        if (slash != String::npos) mkdirs(opts.health_out.substr(0, slash));
        if (!write_file(opts.health_out, build_health_json(verdicts))) {
            std::fprintf(stderr, "警告: 写不了标注表 %s\n", opts.health_out.c_str());
        } else {
            std::fprintf(stderr, "标注表已更新: %s\n", opts.health_out.c_str());
        }
    }

    // ---------- 收尾汇总 ----------
    int A = 0, B = 0, C = 0, D = 0;
    for (const auto& v : verdicts) {
        if      (v.tier == "A") ++A;
        else if (v.tier == "B") ++B;
        else if (v.tier == "C") ++C;
        else                    ++D;
    }

    std::fprintf(stderr, "\n");
    std::fprintf(stderr, "─────────────────────────────────────────────\n");
    std::fprintf(stderr, "  A %4d   B %4d   C %4d   D %4d      共 %zu 个算子\n",
                 A, B, C, D, verdicts.size());
    std::fprintf(stderr, "  耗时 %.1f s（%d 核并行）\n",
                 static_cast<double>(elapsed_ms) / 1000.0,
                 opts.jobs > 0 ? opts.jobs : default_jobs());
    std::fprintf(stderr, "  可中断性: %d/%d 个源文件一次都没轮询过 is_stopped()\n",
                 clean, total_src);
    std::fprintf(stderr, "─────────────────────────────────────────────\n");
    std::fprintf(stderr, "  报告: %s\n        %s\n", opts.json_out.c_str(), opts.md_out.c_str());

    if (D > 0) {
        std::fprintf(stderr, "\n  D 档（会崩/会挂，别往画布上放）:\n");
        int shown = 0;
        for (const auto& v : verdicts) {
            if (v.tier != "D") continue;
            std::fprintf(stderr, "    %-40s %s @ %s\n", v.type_id.c_str(),
                         v.death_kind.c_str(), v.death_scenario.c_str());
            if (++shown >= 20 && D > 20) {
                std::fprintf(stderr, "    ... 还有 %d 个，见报告\n", D - 20);
                break;
            }
        }
    }

    if (opts.fail_on_d && D > 0) return 1;
    return 0;
}
