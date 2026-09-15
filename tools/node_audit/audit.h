/**
 * @file audit.h
 * @brief ovf-node-audit 的公共类型与接口
 *
 * ============================================================================
 * 为什么必须是 fork 子进程，而不是线程
 * ============================================================================
 *
 * 一个会挂死的算子**没法在进程内安全体检**：
 *   - C++ 标准里没有"杀线程"这回事（pthread_cancel 对不响应取消点的代码无效）；
 *   - 看门狗设 `context.stop()` 只对**配合的**算子有用，而实测 501 个上游
 *     算子里 0 个轮询 `is_stopped()`（见报告头部的"可中断性"统计）；
 *   - `detach()` 掉卡住的线程 = 泄漏它，它还在后台继续改全局状态。
 *
 * 所以每个算子跑在一个**用完即弃的子进程**里：
 *   挂死 → 父进程按墙钟预算 SIGKILL → TIMEOUT
 *   段错误 → WIFSIGNALED + SIGSEGV
 *   两者都伤不到父进程（子进程写的是 COW 副本）。
 *
 * ============================================================================
 * 子进程规矩（每一条都有理由，别删）
 * ============================================================================
 *
 *  1. **单线程**。父进程先 `initialize_algorithm_module()`，全程不起
 *     FlowRunner / Web 服务器。fork 之后子进程里只有一个线程，这才安全。
 *  2. **只准 `_exit()`，不准 `exit()`**。后者会跑 atexit、把**从父进程继承来的**
 *     stdio 缓冲冲出去（哪怕已经重定向，语义上也是错的）。
 *  3. fork 后第一件事 `Logger::clear_sinks()` —— 否则每个算子的日志都会
 *     和体检报告混在一起。
 *  4. stdout/stderr 重定向到 `/dev/null`；自己的结论走**专用管道 fd**，
 *     用 `::write()` 直接写（绕开 stdio，没有缓冲问题）。
 *  5. `RLIMIT_AS` / `RLIMIT_CPU` / `RLIMIT_FSIZE` / `RLIMIT_NOFILE` /
 *     `RLIMIT_NPROC=0`（算子不许自己 fork）/ **`RLIMIT_CORE=0`**
 *     —— 最后这条最关键：501 个子进程崩溃不能把磁盘写满 core 文件。
 *  6. `chdir()` 到该算子自己的临时目录，防止会写文件的算子互相污染。
 *  7. **每个"步"之前写一行 begin、之后写一行 end，都 flush。**
 *     这样子进程死在半路时，父进程仍然握着"是哪一步弄死的"——
 *     这是整个工具的**归因精度**所在。
 *
 * ============================================================================
 * 场景矩阵
 * ============================================================================
 *
 *   S0  构造 + `info().id` 与 type_id 一致           → CONSTRUCT / META_ID_MISMATCH
 *   S0b 跑元数据校验 V1–V15                          → 累积 META_WARN
 *   S1  **什么都不接，用声明的默认值直接 execute**     → ★ 单点价值最高：
 *                                                     这就是画布上连线连错的样子
 *   S2  每个输入端喂"类型对但尺寸为 0"的数据
 *   S3  每个输入端喂"错误类型"的数据
 *   S4  金丝雀：真实尺寸灰度 / 真实尺寸 RGB / 8×8 小图 / 小 Region / 小点云
 *   S5  **参数模糊测试**                              → 真正找崩的地方
 *   S6  金丝雀跑通之后，每个声明过的输出端口必须非 None
 *
 * ============================================================================
 * ★ 两处**故意偏离**计划书的地方（都有理由，写在这里免得以后被当成 bug）
 * ============================================================================
 *
 * (1) **S4/S5 用的图是 256×256，不是 8×8；8×8 降级成 S4 里一个单独的 flavour。**
 *     参数 fuzz 的意义是"把参数推到极端，看循环体里会发生什么"。8×8 的图
 *     会让大量算子的循环体**根本不执行**——极端参数就没机会生效，fuzz 变成
 *     空转。最典型的例子就是 `SurfaceRoughnessInspection`：它的
 *     `start_x = width/2 - measure_length/2`，measure_length 默认 100，
 *     width=8 时 start_x 在 uint32 下**回绕成 42.9 亿**，循环一次都不跑。
 *     要让它真的进循环体，图必须够大。
 *
 *     ★ 这条最早只用在 S5 上，实测证明不够：S4 的 8×8 金丝雀本身就会把这个
 *     算子打崩（同一个回绕），于是它**每轮都死、连吃两次重启预算**，
 *     后面的 S5 还没跑到 `sampling_interval` 就被 max-restarts 掐断了 ——
 *     验收门要的"必须复现 sampling_interval=0 死循环"因此复现不出来。
 *     现在 S4 基准也用 256，**小图作为一个独立的 flavour `tiny` 单列一步**：
 *     这样"小图崩"仍然会被抓到、如实记进报告，但它是它自己那一行证据，
 *     不会再冒充参数 fuzz 的结论、也不会再把 S5 挡在门外。
 *
 * (2) **硬失败之后会重启，而不是就此停手。**
 *     计划书写"硬失败即停"。但如果 S4 崩了就直接停，S5 的参数 fuzz 永远
 *     跑不到——恰恰把最有价值的结论丢掉了。改成：死在哪一步，就从**下一步**
 *     重新 fork 一次继续跑，最多 `--max-restarts` 次。
 *     每一步都是独立的 begin/end，所以**归因精度不变**（仍然是"死在这一步"），
 *     只是多问几个问题。`--max-restarts 0` 可以退回严格的"即停"。
 *     默认给 32：一份完整计划 ~25 步，每步都崩也不过 25 次，天然有界。
 *     太小（比如 4）会让"好几个参数都崩"的算子在中途就把额度烧光。
 */

#pragma once

#include <ovf/core/node.h>
#include <ovf/core/node_health.h>

#include <functional>
#include <string>
#include <vector>

namespace ovf {
namespace audit {

// ============================================================================
// 场景
// ============================================================================

enum class Scenario {
    Construct = 0,   //!< S0
    Metadata,        //!< S0b
    NoInput,         //!< S1
    EmptyInput,      //!< S2
    WrongType,       //!< S3
    Canary,          //!< S4
    ParamFuzz,       //!< S5
    Outputs,         //!< S6
    Count
};

const char* scenario_name(Scenario s);

// ============================================================================
// 一步的结论
// ============================================================================

enum class Status {
    Ok   = 0,   //!< 执行了，返回 success
    Fail = 1,   //!< 执行了，返回干净的 Result::failure
    Warn = 2,   //!< 执行了，但有问题（元数据瑕疵、输出端口没产出）
    Skip = 3,   //!< 没执行（合成不出这种输入 / 参数类型不支持 fuzz）
};

const char* status_name(Status s);

/**
 * @brief 一个"步"的证据
 *
 * 步 = 一次可独立归因的最小执行单位（一次 execute、一次校验、一个端口的
 * 一种喂法、一个参数的一种取值）。
 */
struct Evidence {
    Scenario scenario = Scenario::Construct;
    Status   status   = Status::Ok;

    String flavour;      //!< 金丝雀的口味："gray" / "rgb"
    String port;         //!< S2/S3：哪个输入端口
    String param;        //!< S5：哪个参数
    String value;        //!< S5：试的什么值

    String detail;       //!< 失败消息 / 说明
    int    error_code = 0;
    int    ms = 0;

    /// 这一步是不是"崩了"（父进程按信号/超时补出来的证据）
    bool death = false;
};

// ============================================================================
// 一个算子的总结论
// ============================================================================

struct NodeVerdict {
    String type_id;
    String category;

    String tier  = "D";   //!< A / B / C / D
    int    score = 10;
    String note;

    Vector<Evidence> evidence;

    int      deaths        = 0;
    int      restarts      = 0;
    int      steps_planned = 0;
    int      steps_done    = 0;
    uint64_t total_ms      = 0;

    /// 内存上限被撑爆（**重新跑一次、把上限放大之后就好了**）——
    /// 这是体检自己的预算造成的，不是算子缺陷。
    bool   resource_limit = false;

    String death_kind;      //!< "SIGSEGV" / "SIGABRT" / "TIMEOUT" / "CONSTRUCT" ...
    String death_scenario;  //!< 死在哪一步
    String death_param;
    String death_value;
    String death_detail;

    bool is_broken() const { return tier == "D"; }
};

// ============================================================================
// 选项
// ============================================================================

struct Options {
    int  jobs          = 0;      //!< 0 = nproc
    int  budget_ms     = 2000;   //!< 子进程软预算：过了就不再开新步
    int  hard_grace_ms = 3000;   //!< 父进程硬杀 = budget_ms + 这个
    int  mem_limit_mb  = 1024;   //!< RLIMIT_AS
    //! 硬失败后最多重启几次（0 = 严格即停）。
    //! ★ 默认 32 而不是 4：一份完整计划也就 ~25 步，**每步都崩**也不过 25 次重启，
    //!   所以 32 天然有界、不会失控。定成 4 的时候，一个"好几个参数都会崩"的算子
    //!   （比如 SurfaceRoughnessInspection：measure_length 三个取值全崩）会在跑到
    //!   sampling_interval 之前就把预算烧光 —— 于是最该被复现的那个死循环反而永远
    //!   看不到，体检报告的结论就变成了工具自身缺陷的投影。
    int  max_restarts  = 32;
    bool quick         = false;  //!< 只跑 S0/S0b/S1/S3，秒级
    bool fail_on_d     = false;  //!< 有 D 档就以退出码 1 结束（给 CI 用）
    bool verbose       = false;

    //! ★ S5 参数 fuzz 用的图边长。**故意比 S4 金丝雀大得多**，理由见文件头。
    int  fuzz_image_size = 256;

    Vector<String> only;        //!< 只跑这些 type_id（空 = 全部）
    String skip_prefix;         //!< 跳过这些前缀

    String reports_dir = "build/reports";
    String health_out;          //!< 非空 → 同时写一份 health/node_health.json
    String json_out;            //!< 空则默认 reports_dir/node_audit.json
    String md_out;              //!< 空则默认 reports_dir/node_audit.md
    String source_dir;          //!< ovf-algorithm/src，用于"可中断性"统计

    // ---- 以下两个由父进程在每次 fork 之前设置，命令行不直接暴露 ----

    /// 从第几步开始跑（崩溃重启时 = 死亡步 + 1）
    int start_step = 0;

    /// 跑到第几步为止（-1 = 跑到底）。放大内存上限复检单步时用它把范围收成一步。
    int stop_after_step = -1;
};

// ============================================================================
// 子进程侧（audit_child.cpp）
// ============================================================================

/**
 * @brief 计划里的**一步**
 *
 * 只装"怎么造这一步的输入"，不装结论 —— 结论走 Evidence。
 * 刻意不含任何运行期状态，这样父进程（只看 NodeInfo）和子进程（实际执行）
 * 枚举出的计划**逐项相等**。
 */
struct Step {
    Scenario scenario = Scenario::Construct;

    // ---- S4 / S6：金丝雀口味 ----
    String flavour;              //!< "gray" / "rgb"

    // ---- S2 / S3：喂哪个端口、端口声明的是什么类型 ----
    String   port;
    DataType port_type = DataType::None;

    // ---- S5：动哪个参数、动成什么 ----
    String param;
    String value_label;          //!< 给人看的（"0" / "\"\"" / "__ovf_invalid__"）
    Data   payload;              //!< 实际塞进去的
};

/**
 * @brief 枚举这个算子在当前选项下要跑的每一步
 *
 * ★ **父进程和子进程调的是同一个函数、同一个 `NodeInfo`** —— 这是"父进程
 * 说得出子进程死在第几步"的全部依据。两边枚举结果不一致，归因就是错的。
 * 所以这里只依赖 `NodeInfo`（从注册表读出来的，不是算子自己吐的）和
 * `Options`，**不依赖任何运行期状态**。
 */
Vector<Step> plan_steps(const NodeInfo& info, const Options& opts);

/**
 * @brief 在 fork 出来的子进程里跑第 `[opts.start_step, opts.stop_after_step]` 步
 *
 * **绝不返回** —— 一律以 `_exit()` 结束。
 */
void run_child(const String& type_id, const Options& opts, int out_fd);

// ============================================================================
// 父进程侧（audit_worker.cpp）
// ============================================================================

/// 每跑完一个算子回调一次（父进程用它打进度）
using ProgressFn = std::function<void(const NodeVerdict&, size_t done, size_t total)>;

/**
 * @brief 跑完一批算子
 *
 * @param type_ids 要体检的 type_id（顺序 = 返回值的顺序）
 * @return 每个算子的结论。输入里不认识 / 没注册的会得到一条 D 档。
 *
 * 并行模型：父进程是 `poll()` I/O 密集型，worker 之间零共享状态
 * —— 每个算子的全部状态都在它那个用完即弃的子进程里。
 */
Vector<NodeVerdict> run_audit(const Vector<String>& type_ids, const Options& opts,
                              const ProgressFn& on_progress);

/// 机器核数（`--jobs` 的默认值），已按 1..32 夹过
int default_jobs();

// ============================================================================
// 报告侧（audit_report.cpp）
// ============================================================================

String verdicts_to_json(const Vector<NodeVerdict>& verdicts, const Options& opts,
                        const String& interruptibility_note);

String verdicts_to_markdown(const Vector<NodeVerdict>& verdicts, const Options& opts,
                            const String& interruptibility_note);

/// 只含 type_id → {tier,score,note} 的精简版，进仓库、给 /api/nodes 用
String build_health_json(const Vector<NodeVerdict>& verdicts);

/**
 * @brief 可中断性统计 —— 全局报一次，不是逐算子
 *
 * 逐算子的"有没有轮询 is_stopped()"在运行期**测不出来**（得有个死循环才测得出来），
 * 它是源码属性。所以直接扫源码：哪些 .cpp 里一次都没有出现
 * `is_stopped` / `is_paused` / `step_ok`。
 */
String scan_interruptibility(const String& src_dir, int* out_clean, int* out_total);

int default_jobs();

}} // namespace ovf::audit
