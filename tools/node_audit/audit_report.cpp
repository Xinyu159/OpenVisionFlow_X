/**
 * @file audit_report.cpp
 * @brief 把体检结论写成 JSON / Markdown / 精简标注表
 *
 * JSON 是**手搓**的，没用 nlohmann 的 dump()。三个理由：
 *   1. 报告要能 `git diff`，字段顺序必须稳定 —— 手搓的顺序我说了算；
 *   2. 仓库自带的是一份 24KB 的简化实现，把 501 个算子 × 几十条证据
 *      构造成一棵 json 树再序列化，内存和不确定性都不划算；
 *   3. 全文只有字符串和整数，转义规则已经写好了（json_escape）。
 */

#include "audit.h"

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>

namespace ovf {
namespace audit {

namespace {

String json_escape(const String& s) {
    String o;
    o.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(c)));
                    o += buf;
                } else {
                    o += c;
                }
        }
    }
    return o;
}

/// Markdown 表格单元格：竖线要转义，换行要压平，否则表格会散架
String md_cell(const String& s) {
    String o;
    o.reserve(s.size());
    for (char c : s) {
        if (c == '|')       o += "\\|";
        else if (c == '\n' || c == '\r') o += ' ';
        else                o += c;
    }
    if (o.size() > 120) o = o.substr(0, 120) + "…";
    return o.empty() ? String("-") : o;
}

struct Counts {
    int A = 0, B = 0, C = 0, D = 0;
    int crashed = 0, hung = 0, construct_failed = 0;
    int resource_limit = 0, metadata_error = 0, output_missing = 0, canary_failed = 0;
    int total_deaths = 0, total_restarts = 0;
    uint64_t total_ms = 0;
    size_t planned_steps = 0, done_steps = 0;
};

Counts count_of(const Vector<NodeVerdict>& vs) {
    Counts c;
    for (const auto& v : vs) {
        if      (v.tier == "A") ++c.A;
        else if (v.tier == "B") ++c.B;
        else if (v.tier == "C") ++c.C;
        else                    ++c.D;

        if (v.death_kind == "TIMEOUT")        ++c.hung;
        else if (v.death_kind == "CONSTRUCT") ++c.construct_failed;
        else if (v.deaths > 0)                ++c.crashed;

        if (v.resource_limit) ++c.resource_limit;
        c.total_deaths   += v.deaths;
        c.total_restarts += v.restarts;
        c.total_ms       += v.total_ms;
        c.planned_steps  += static_cast<size_t>(v.steps_planned);
        c.done_steps     += static_cast<size_t>(v.steps_done);

        for (const auto& e : v.evidence) {
            if (e.scenario == Scenario::Metadata && e.status == Status::Warn) ++c.metadata_error;
            if (e.scenario == Scenario::Outputs  && e.status != Status::Ok &&
                e.status != Status::Skip) ++c.output_missing;
            if (e.scenario == Scenario::Canary   && e.status == Status::Fail) ++c.canary_failed;
        }
    }
    return c;
}

const char* tier_meaning(const String& tier) {
    if (tier == "A") return "全过：不崩不挂、金丝雀跑通、输出齐全、元数据干净";
    if (tier == "B") return "能用：跑得通，但元数据有 Error 级问题（画布上可能画不对）";
    if (tier == "C") return "有保留：不崩不挂，但金丝雀跑不通 / 声明了输出却不产出 / 吃内存";
    return "别碰：崩溃、挂死，或者根本构造不出来";
}

String fmt_pct(int n, size_t total) {
    if (total == 0) return "0.0%";
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.1f%%", 100.0 * n / static_cast<double>(total));
    return buf;
}

/// 按分类聚合：哪些 category 是重灾区
String category_table(const Vector<NodeVerdict>& vs) {
    // std::map 而不是 HashMap：报告要能 diff，遍历顺序必须稳定。
    std::map<String, std::array<int, 4>> tally;   // 下标 = A/B/C/D
    for (const auto& v : vs) {
        const String cat = v.category.empty() ? "（未分类）" : v.category;
        auto& t = tally[cat];
        if      (v.tier == "A") ++t[0];
        else if (v.tier == "B") ++t[1];
        else if (v.tier == "C") ++t[2];
        else                    ++t[3];
    }

    String s = "| 分类 | 总数 | A | B | C | D |\n|---|---|---|---|---|---|\n";
    for (const auto& kv : tally) {
        const int tot = kv.second[0] + kv.second[1] + kv.second[2] + kv.second[3];
        s += "| " + md_cell(kv.first) + " | " + std::to_string(tot);
        for (int i = 0; i < 4; ++i) s += " | " + std::to_string(kv.second[i]);
        s += " |\n";
    }
    return s;
}

} // namespace

// ===========================================================================
// 可中断性
// ===========================================================================

String scan_interruptibility(const String& src_dir, int* out_clean, int* out_total) {
    if (out_clean) *out_clean = 0;
    if (out_total) *out_total = 0;

    DIR* dir = ::opendir(src_dir.c_str());
    if (!dir) {
        return "（扫描不到源码目录 `" + src_dir + "` —— 用 --source-dir 指到 ovf-algorithm/src）";
    }

    Vector<String> clean_files;
    Vector<String> all_files;
    int total = 0;

    struct dirent* ent = nullptr;
    while ((ent = ::readdir(dir)) != nullptr) {
        const String name = ent->d_name;
        if (name.size() < 5 || name.substr(name.size() - 4) != ".cpp") continue;
        ++total;
        all_files.push_back(name);

        std::ifstream in(src_dir + "/" + name, std::ios::binary);
        if (!in.is_open()) continue;
        const String body((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());

        // 三个词中出现任何一个，就说明这个文件里有"能被叫停"的出口。
        // is_stopped 是 context 上的，is_paused 同理，step_ok 是阶段 3 给新算子
        // 准备的一行式检查。老算子三者皆无 = 一旦进循环就只能等它自己出来。
        const bool responds =
            body.find("is_stopped") != String::npos ||
            body.find("is_paused")  != String::npos ||
            body.find("step_ok")    != String::npos;
        if (!responds) clean_files.push_back(name);
    }
    ::closedir(dir);

    if (out_clean) *out_clean = static_cast<int>(clean_files.size());
    if (out_total) *out_total = total;

    String s = "扫了 " + std::to_string(total) + " 个 .cpp，其中 **" +
               std::to_string(clean_files.size()) +
               " 个一次都没有轮询过 `is_stopped()` / `is_paused()` / `step_ok()`**"
               " —— 这些文件里的算子一旦进入长循环，流程的\"停止\"按钮就是无效的，"
               "只能等它跑完或者杀死进程。\n";

    if (!clean_files.empty()) {
        std::sort(clean_files.begin(), clean_files.end());
        s += "\n<details><summary>展开这 " + std::to_string(clean_files.size()) +
             " 个文件</summary>\n\n```\n";
        for (const auto& f : clean_files) s += f + "\n";
        s += "```\n</details>\n";
    }
    return s;
}

// ===========================================================================
// JSON
// ===========================================================================

String verdicts_to_json(const Vector<NodeVerdict>& verdicts, const Options& opts,
                        const String& interruptibility_note) {
    const Counts c = count_of(verdicts);
    String s;
    s.reserve(verdicts.size() * 900 + 4096);

    s += "{\n";
    s += "  \"meta\": {\n";
    s += "    \"generated_by\": \"ovf-node-audit\",\n";
    s += "    \"nodes\": " + std::to_string(verdicts.size()) + ",\n";
    s += "    \"mode\": \"" + String(opts.quick ? "quick" : "full") + "\",\n";
    s += "    \"budget_ms\": " + std::to_string(opts.budget_ms) + ",\n";
    s += "    \"mem_limit_mb\": " + std::to_string(opts.mem_limit_mb) + ",\n";
    s += "    \"fuzz_image_size\": " + std::to_string(opts.fuzz_image_size) + ",\n";
    s += "    \"max_restarts\": " + std::to_string(opts.max_restarts) + ",\n";
    s += "    \"jobs\": " + std::to_string(opts.jobs > 0 ? opts.jobs : default_jobs()) + ",\n";
    s += "    \"interruptibility\": \"" + json_escape(interruptibility_note) + "\"\n";
    s += "  },\n";

    s += "  \"summary\": {\n";
    s += "    \"A\": " + std::to_string(c.A) + ",\n";
    s += "    \"B\": " + std::to_string(c.B) + ",\n";
    s += "    \"C\": " + std::to_string(c.C) + ",\n";
    s += "    \"D\": " + std::to_string(c.D) + ",\n";
    s += "    \"crashed\": " + std::to_string(c.crashed) + ",\n";
    s += "    \"hung\": " + std::to_string(c.hung) + ",\n";
    s += "    \"construct_failed\": " + std::to_string(c.construct_failed) + ",\n";
    s += "    \"resource_limit\": " + std::to_string(c.resource_limit) + ",\n";
    s += "    \"canary_failed\": " + std::to_string(c.canary_failed) + ",\n";
    s += "    \"output_missing\": " + std::to_string(c.output_missing) + ",\n";
    s += "    \"metadata_error\": " + std::to_string(c.metadata_error) + ",\n";
    s += "    \"total_deaths\": " + std::to_string(c.total_deaths) + ",\n";
    s += "    \"total_restarts\": " + std::to_string(c.total_restarts) + ",\n";
    s += "    \"steps_planned\": " + std::to_string(c.planned_steps) + ",\n";
    s += "    \"steps_done\": " + std::to_string(c.done_steps) + ",\n";
    // 这里报的是**执行步数**，不是墙钟（并行下墙钟不好归因到单算子）
    s += "    \"total_exec_ms\": " + std::to_string(c.total_ms) + "\n";
    s += "  },\n";

    s += "  \"nodes\": [\n";
    for (size_t i = 0; i < verdicts.size(); ++i) {
        const NodeVerdict& v = verdicts[i];
        s += "    {\n";
        s += "      \"type_id\": \"" + json_escape(v.type_id) + "\",\n";
        s += "      \"category\": \"" + json_escape(v.category) + "\",\n";
        s += "      \"tier\": \"" + json_escape(v.tier) + "\",\n";
        s += "      \"score\": " + std::to_string(v.score) + ",\n";
        s += "      \"note\": \"" + json_escape(v.note) + "\",\n";
        s += "      \"steps_planned\": " + std::to_string(v.steps_planned) + ",\n";
        s += "      \"steps_done\": " + std::to_string(v.steps_done) + ",\n";
        s += "      \"deaths\": " + std::to_string(v.deaths) + ",\n";
        s += "      \"restarts\": " + std::to_string(v.restarts) + ",\n";
        s += "      \"total_ms\": " + std::to_string(v.total_ms) + ",\n";
        s += "      \"resource_limit\": " + String(v.resource_limit ? "true" : "false") + ",\n";
        s += "      \"death\": {";
        s += "\"kind\": \"" + json_escape(v.death_kind) + "\"";
        s += ", \"scenario\": \"" + json_escape(v.death_scenario) + "\"";
        s += ", \"param\": \"" + json_escape(v.death_param) + "\"";
        s += ", \"value\": \"" + json_escape(v.death_value) + "\"";
        s += ", \"detail\": \"" + json_escape(v.death_detail) + "\"}";
        s += ",\n";

        s += "      \"evidence\": [";
        for (size_t k = 0; k < v.evidence.size(); ++k) {
            const Evidence& e = v.evidence[k];
            if (k) s += ",";
            s += "\n        {\"s\": \"" + String(scenario_name(e.scenario)) + "\"";
            s += ", \"st\": \"" + String(status_name(e.status)) + "\"";
            s += ", \"ms\": " + std::to_string(e.ms);
            s += ", \"ec\": " + std::to_string(e.error_code);
            if (!e.flavour.empty()) s += ", \"flavour\": \"" + json_escape(e.flavour) + "\"";
            if (!e.port.empty())    s += ", \"port\": \"" + json_escape(e.port) + "\"";
            if (!e.param.empty())   s += ", \"param\": \"" + json_escape(e.param) + "\"";
            if (!e.value.empty())   s += ", \"value\": \"" + json_escape(e.value) + "\"";
            if (!e.detail.empty())  s += ", \"detail\": \"" + json_escape(e.detail) + "\"";
            // ★ 这一条必须写出来。少了它，消费 JSON 的那一方就分不清
            //   "跑完了的一步"和"死在这里的一步" —— 阶段 6 要按缺陷清单
            //   逐个复核，靠的正是这个标记来定位每一处死亡。
            if (e.death)            s += ", \"death\": true";
            s += "}";
        }
        s += (v.evidence.empty() ? "" : "\n      ");
        s += "]\n";
        s += "    }";
        s += (i + 1 == verdicts.size() ? "\n" : ",\n");
    }
    s += "  ]\n}\n";
    return s;
}

// ===========================================================================
// Markdown
// ===========================================================================

String verdicts_to_markdown(const Vector<NodeVerdict>& verdicts, const Options& opts,
                            const String& interruptibility_note) {
    const Counts c = count_of(verdicts);
    const size_t total = verdicts.size();

    String s;
    s += "# OVF 算子体检报告\n\n";
    s += "这份报告是 `ovf-node-audit` 跑出来的。它把每个算子放进一个**用完即弃的\n";
    s += "子进程**里逐场景执行，所以能扛住崩溃和挂死，并把结论精确到\n";
    s += "\"死在哪一步、哪个参数\"。工具本身的用法见 `docs/hardening/stage5_node_audit.md`。\n\n";

    s += "## 一、这次是怎么跑的\n\n";
    s += "| 项 | 值 |\n|---|---|\n";
    s += "| 模式 | `" + String(opts.quick ? "--quick（只跑 S0/S0b/S1/S3）" : "--full（S0–S6 全跑）") + "` |\n";
    s += "| 算子数 | " + std::to_string(total) + " |\n";
    s += "| 每算子预算 | " + std::to_string(opts.budget_ms) + " ms |\n";
    s += "| 子进程内存上限 | " + std::to_string(opts.mem_limit_mb) + " MB (`RLIMIT_AS`) |\n";
    s += "| 参数 fuzz 用的图 | " + std::to_string(opts.fuzz_image_size) + "×" +
         std::to_string(opts.fuzz_image_size) + "（★ 比 S4 金丝雀大得多，理由见工具头注释）|\n";
    s += "| 崩溃后重启上限 | " + std::to_string(opts.max_restarts) + " 次 |\n";
    s += "| 并行度 | " + std::to_string(opts.jobs > 0 ? opts.jobs : default_jobs()) + " |\n";
    s += "| 计划步数 / 实际步数 | " + std::to_string(c.planned_steps) + " / " +
         std::to_string(c.done_steps) + " |\n\n";

    s += "## 二、分档结果\n\n";
    s += "| 档 | 数量 | 占比 | 含义 |\n|---|---|---|---|\n";
    const int ns[4] = {c.A, c.B, c.C, c.D};
    const char* tiers[4] = {"A", "B", "C", "D"};
    for (int i = 0; i < 4; ++i) {
        s += "| **" + String(tiers[i]) + "** | " + std::to_string(ns[i]) + " | " +
             fmt_pct(ns[i], total) + " | " + tier_meaning(tiers[i]) + " |\n";
    }
    s += "\n";
    s += "细项：崩溃 " + std::to_string(c.crashed) +
         "，挂死 " + std::to_string(c.hung) +
         "，构造不出 " + std::to_string(c.construct_failed) +
         "，疑似吃内存（放大上限后能跑）" + std::to_string(c.resource_limit) +
         "，金丝雀跑不通 " + std::to_string(c.canary_failed) +
         "，声明了输出却不产出 " + std::to_string(c.output_missing) +
         "，元数据 Error " + std::to_string(c.metadata_error) + "。\n\n";

    // ---- D 档明细 ----
    s += "## 三、D 档明细（崩溃 / 挂死 / 构造不出）\n\n";
    bool any_d = false;
    for (const auto& v : verdicts) {
        if (v.tier != "D") continue;
        if (!any_d) {
            s += "| type_id | 分类 | 死因 | 场景 | 参数 | 说明 |\n";
            s += "|---|---|---|---|---|---|\n";
            any_d = true;
        }

        // ★ 一个算子可能死在好几个地方，**每处各占一行**。
        //   verdict 上的 death_* 是"第一处"，只报它会让人以为修完那一个就完事了
        //   —— SurfaceRoughnessInspection 实测死 4 处，其中 3 处是同一个参数的
        //   三个取值，真正的坑数是 3 个而不是 1 个。
        bool rowed = false;
        for (const auto& e : v.evidence) {
            if (!e.death) continue;
            String where = scenario_name(e.scenario);
            if (!e.flavour.empty())  where += " (" + e.flavour + ")";
            if (!e.param.empty())    where += " / " + e.param + "=" + e.value;
            else if (!e.port.empty()) where += " / 端口 " + e.port;
            s += "| `" + md_cell(v.type_id) + "` | " + md_cell(v.category) + " | **" +
                 md_cell(e.detail.empty() ? v.death_kind : e.detail) + "** | " +
                 md_cell(where) + " | " + md_cell(e.param) + " | " +
                 md_cell(v.note) + " |\n";
            rowed = true;
        }
        // 兜底：万一这条死亡没配上计划里的步（比如"还没开始就死了"），
        // 也要有一行，不能让这个算子从 D 档明细里凭空消失。
        if (!rowed) {
            String where = v.death_scenario.empty() ? String("-") : v.death_scenario;
            if (!v.death_param.empty()) where += " / " + v.death_param + "=" + v.death_value;
            s += "| `" + md_cell(v.type_id) + "` | " + md_cell(v.category) + " | **" +
                 md_cell(v.death_kind) + "** | " + md_cell(where) + " | " +
                 md_cell(v.death_param) + " | " + md_cell(v.note) + " |\n";
        }
    }
    if (!any_d) s += "无。没有算子崩溃、挂死或构造不出来。\n";
    s += "\n";

    // ---- 三个已知坏种的核对 ----
    s += "## 四、已知坏种核对\n\n";
    s += "这三条是体检之前就通过读代码认定的缺陷。**它们的结论必须和代码分析一致，\n";
    s += "否则说明 fuzz 根本没打到代码，其余结论一律不可信。**\n\n";
    s += "| 算子 | 源码位置 | 预期 | 实际 | 结论 |\n|---|---|---|---|---|\n";
    struct Probe {
        const char* type_id;
        const char* where;
        const char* what;
        //! 代码分析预测的触发参数与场景。判"复现"必须命中**这个**，
        //! 光看档位会把"死在别处"误判成"预期缺陷已复现"。
        const char* expect_param;
        const char* expect_scen;
    };
    // 这三个 type_id 是从 OVF_REGISTER_NODE 那一行**读出来的**，不是猜的。
    // 将来改了名字，这里会明确报"注册表里没这个 type_id"，不会静默漏检。
    const Probe probes[] = {
        {"SurfaceRoughnessInspection", "automotive_inspection.cpp:1883",
         "`sampling_interval=0` → 循环变量不前进 = 死循环",
         "sampling_interval", "S5"},
        {"ThresholdMorph",             "morphology.cpp:709",
         "`kernel_size=0` → create_kernel 的返回值被丢弃，空核照用",
         "kernel_size", "S5"},
        {"Resize",                     "geometry.cpp:226",
         "宽高巨大值 → `dst_w*dst_h*channels` 在 int 下溢出",
         "width", "S5"},
    };
    auto& registry = NodeFactory::instance();

    for (const auto& p : probes) {
        // ① 先查**注册表**，而不是"这一趟跑了谁"。
        //    拿 verdicts 当存在性判据是错的：`--only X` 时 verdicts 里只有 X，
        //    另外两个会被无辜地报成"名字变了或被摘了"。注册表才是权威。
        if (!registry.has_type(p.type_id)) {
            s += "| `" + String(p.type_id) + "` | " + String(p.where) + " | " +
                 String(p.what) + " | **注册表里没有这个 type_id** | ❓ 名字变了或被摘了 |\n";
            continue;
        }

        // ② 这一次跑没跑它。
        const NodeVerdict* hit = nullptr;
        for (const auto& v : verdicts) {
            if (v.type_id == p.type_id) { hit = &v; break; }
        }
        if (!hit) {
            s += "| `" + String(p.type_id) + "` | " + String(p.where) + " | " +
                 String(p.what) + " | 注册了，但**这一趟没跑** | ⏭ 用 `--only " +
                 String(p.type_id) + "` 单独验 |\n";
            continue;
        }

        // ③ 判据是"**预测的那个触发点**有没有真的出事"，不是"这个算子是不是 D 档"。
        //    这一步很要紧：SurfaceRoughnessInspection 是 D 档，但把它打成 D 的
        //    是 S4 小图金丝雀，跟 `sampling_interval=0` 一点关系都没有。
        //    拿档位当判据会输出"✅ 复现"，而这条结论**根本没被这次运行验证过**。
        // ★ "出事"不等于"进程死了"。`sampling_interval=0` 的死循环是一边转
        //   一边往 profile 里塞数据，于是它**先撞上 RLIMIT_AS**、抛 bad_alloc，
        //   被 execute 外面那层 catch 接住 —— 子进程**活着**退出，只留一条
        //   status=fail。只看 death 会把它漏掉，然后对着一个已经复现的缺陷报 ⚠️。
        //   判据因此是：这一步**死了**，或者**抛了异常**（"抛出"是子进程记录
        //   被 catch 住的 C++ 异常时的原话）。干净的 Result::failure（比如
        //   "Input is not an image"）不含这个词，所以不会误判。
        auto blew_up = [](const Evidence& e) {
            return e.death || e.detail.find("抛出") != String::npos;
        };

        bool   expected_hit = false;
        String observed, expected_obs;
        for (const auto& e : hit->evidence) {
            if (e.param == p.expect_param &&
                scenario_name(e.scenario) == String(p.expect_scen) && blew_up(e)) {
                expected_hit = true;
                expected_obs = String(scenario_name(e.scenario)) + " / " + e.param + "=" +
                               e.value + " → " +
                               (e.death ? String("进程死亡") : String("抛异常")) +
                               "（" + e.detail.substr(0, 60) + "）";
            }
            if (observed.empty() && blew_up(e)) {
                observed = scenario_name(e.scenario);
                if (!e.flavour.empty()) observed += "(" + e.flavour + ")";
                if (!e.param.empty())   observed += "/" + e.param + "=" + e.value;
                observed += " " + (e.death ? hit->death_kind : String("抛异常"));
            }
        }

        String verdict_cell;
        if (expected_hit) {
            verdict_cell = "✅ **复现** —— " + md_cell(expected_obs);
        } else if (!observed.empty()) {
            // 崩了，但不是预测的那个点。如实说，别让它冒充成"预期缺陷已复现"。
            verdict_cell = "⚠️ 崩了，**但触发点不是预测的那个** —— 实际是 " +
                           md_cell(observed);
        } else {
            verdict_cell = "❌ **没复现**（" + hit->tier + " 档：" + md_cell(hit->note) +
                           "）—— 先修 fuzzer，其余结论不可信";
        }
        s += "| `" + md_cell(hit->type_id) + "` | " + md_cell(p.where) + " | " +
             md_cell(p.what) + " | " + md_cell(observed.empty() ? hit->note : observed) + " | " +
             verdict_cell + " |\n";
    }
    s += "\n";
    s += "这三个是**参数 fuzz 有没有真的打到代码**的判据。任何一个判 ❌，\n";
    s += "说明 fuzzer 构造的输入没进到出事的那条路径，整份报告都得打问号。\n";
    s += "判 ⚠️ 表示算子确实崩了，但**不是**这里预测的那个原因 —— 两回事，别混。\n";
    s += "（`Resize` 预测的是 `dst_w*dst_h*channels` 的 int 溢出，那属于\"一次只动一个\n";
    s += "参数够不着\"的情况：`use_scale_` 只在宽和高**同时** ≤0 时才为真，所以那条\n";
    s += "溢出路径靠单参数 fuzz 打不到，是已知覆盖盲区。它实测崩在另一个点上 ——\n";
    s += "`dst_w==1` 时 `image_utils.cpp` 的 `(src_w-1)/(dst_w-1)` 除零，见该行。）\n\n";

    // ---- C 档明细 ----
    s += "## 五、C 档明细（不崩，但用起来有保留）\n\n";
    bool any_c = false;
    for (const auto& v : verdicts) {
        if (v.tier != "C") continue;
        if (!any_c) {
            s += "| type_id | 分类 | 说明 |\n|---|---|---|\n";
            any_c = true;
        }
        s += "| `" + md_cell(v.type_id) + "` | " + md_cell(v.category) + " | " +
             md_cell(v.note) + " |\n";
    }
    if (!any_c) s += "无。\n";
    s += "\n";

    // ---- 按分类 ----
    s += "## 六、按分类统计\n\n";
    s += category_table(verdicts);
    s += "\n";

    // ---- 可中断性 ----
    s += "## 七、可中断性（全局只报一次）\n\n";
    s += interruptibility_note;
    s += "\n";

    s += "## 八、怎么复现这份报告\n\n";
    s += "```bash\n";
    s += "cd <仓库根>\n";
    s += "./build/bin/ovf-node-audit" + String(opts.quick ? " --quick" : " --full") +
         " --jobs " + std::to_string(opts.jobs > 0 ? opts.jobs : default_jobs()) +
         " --budget-ms " + std::to_string(opts.budget_ms) +
         " --mem-limit-mb " + std::to_string(opts.mem_limit_mb) + "\n";
    s += "```\n\n";
    s += "单看一个算子：\n\n```bash\n";
    s += "./build/bin/ovf-node-audit --only <type_id> --verbose\n";
    s += "```\n";

    return s;
}

// ===========================================================================
// 精简标注表（进仓库，给 NodeHealthRegistry 用）
// ===========================================================================

String build_health_json(const Vector<NodeVerdict>& verdicts) {
    const Counts c = count_of(verdicts);
    String s;
    s.reserve(verdicts.size() * 160 + 512);

    s += "{\n";
    s += "  \"_comment\": \"ovf-node-audit 生成。只标注、不改变行为：501 个算子全部照常注册可调用。\",\n";
    s += "  \"_schema\": \"type_id -> {tier, score, note}；tier 为空表示没体检过\",\n";
    s += "  \"_tiers\": {\"A\": \"全过\", \"B\": \"能跑通但元数据有 Error\", "
         "\"C\": \"不崩但金丝雀跑不通/输出缺失/吃内存\", \"D\": \"崩溃或挂死\"},\n";
    s += "  \"_counts\": {\"A\": " + std::to_string(c.A) +
         ", \"B\": " + std::to_string(c.B) +
         ", \"C\": " + std::to_string(c.C) +
         ", \"D\": " + std::to_string(c.D) + "},\n";
    s += "  \"nodes\": {\n";

    // 按 type_id 排序：报告要能 diff，字典序是唯一不会随机变的顺序
    Vector<const NodeVerdict*> sorted;
    sorted.reserve(verdicts.size());
    for (const auto& v : verdicts) sorted.push_back(&v);
    std::sort(sorted.begin(), sorted.end(),
              [](const NodeVerdict* a, const NodeVerdict* b) { return a->type_id < b->type_id; });

    for (size_t i = 0; i < sorted.size(); ++i) {
        const NodeVerdict& v = *sorted[i];
        s += "    \"" + json_escape(v.type_id) + "\": {\"tier\": \"" + json_escape(v.tier) +
             "\", \"score\": " + std::to_string(v.score) +
             ", \"note\": \"" + json_escape(v.note) + "\"}";
        s += (i + 1 == sorted.size() ? "\n" : ",\n");
    }
    s += "  }\n}\n";
    return s;
}

} // namespace audit
} // namespace ovf
