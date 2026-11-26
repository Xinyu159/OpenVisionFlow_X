/**
 * @file audit_child.cpp
 * @brief fork 出来的子进程里做的事：把这个算子的一步一步跑过去，每步前后各写一行 JSON
 *
 * 这个文件里**没有一行 printf**：自己的结论全部走 `out_fd` 这条专用管道，
 * stdout/stderr 早被重定向到 /dev/null 了。理由见 audit.h 文件头。
 *
 * 归因机制（整个工具的命门）：
 *
 *     {"k":"begin","i":12,"s":"S5","p":"sampling_interval","v":"0"}
 *     ← 子进程死在这里的话，父进程手里最后一行就是这个 begin，没有配对的 end
 *     {"k":"end","i":12,"st":"ok","ms":3}
 *
 * 所以**写 begin 和写 end 都不能省、不能合并、不能缓冲**。
 */

#include "audit.h"

#include <ovf/core/flow.h>
#include <ovf/core/logger.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ovf {
namespace audit {

using Clock = std::chrono::steady_clock;

static int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now().time_since_epoch())
        .count();
}

// ===========================================================================
// 往管道写一行
// ===========================================================================

namespace {

/// JSON 字符串转义。手搓是因为子进程的内存预算很紧，不想为一行日志构造 JSON 树。
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
                    o += c;   // UTF-8 原样透传
                }
        }
    }
    return o;
}

/**
 * @brief 截断到 400 字节，并且**保证不切在 UTF-8 字符中间**
 *
 * 切一半的 UTF-8 会让父进程的 JSON 解析拿到非法字节 —— 报告里就是一堆乱码。
 */
String clip(const String& s, size_t max_bytes = 400) {
    if (s.size() <= max_bytes) return s;
    size_t cut = max_bytes;
    // 往回退到不是续字节（10xxxxxx）的位置
    while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0) == 0x80) --cut;
    return s.substr(0, cut) + "...";
}

/**
 * @brief 写一行并换行
 *
 * 单次 `::write`。行长 < PIPE_BUF(4096) 时管道写是原子的，所以父进程
 * 读到的永远是完整的行 —— 即使子进程被 SIGKILL 在 `write` 的瞬间。
 * **不要改成 stdio**：那会引入缓冲，子进程一死，缓冲里的 begin 行就没了。
 */
void emit(int fd, const String& json) {
    String line = json;
    line += '\n';
    ssize_t n = ::write(fd, line.data(), line.size());
    (void)n;   // 父进程不在了就写不出去，没有别的办法，也不值得处理
}

/// 把 Evidence 序列化成一行 end
void emit_end(int fd, int index, const Evidence& e) {
    String s = "{\"k\":\"end\",\"i\":" + std::to_string(index) +
               ",\"st\":\"" + status_name(e.status) + "\""
               ",\"ms\":" + std::to_string(e.ms) +
               ",\"ec\":" + std::to_string(e.error_code);
    if (!e.detail.empty()) s += ",\"m\":\"" + json_escape(clip(e.detail)) + "\"";
    s += "}";
    emit(fd, s);
}

void emit_begin(int fd, int index, const Step& st) {
    String s = "{\"k\":\"begin\",\"i\":" + std::to_string(index) +
               ",\"s\":\"" + scenario_name(st.scenario) + "\"";
    if (!st.flavour.empty())     s += ",\"fl\":\"" + json_escape(st.flavour) + "\"";
    if (!st.port.empty())        s += ",\"port\":\"" + json_escape(st.port) + "\"";
    if (!st.param.empty())       s += ",\"p\":\"" + json_escape(st.param) + "\"";
    if (!st.value_label.empty()) s += ",\"v\":\"" + json_escape(st.value_label) + "\"";
    s += "}";
    emit(fd, s);
}

// ===========================================================================
// 金丝雀
// ===========================================================================

ImageData make_image(uint32_t w, uint32_t h, uint32_t channels, ImageFormat fmt) {
    ImageData img;
    img.width    = w;
    img.height   = h;
    img.channels = channels;
    img.format   = fmt;
    img.data.resize(static_cast<size_t>(w) * h * channels);
    // 横向渐变：让"按 x 变化的算法"有东西可算，同时保证每行都不全同。
    // 单调递增，所以梯度/边缘类算子不会因为"整幅图一个色"而提前返回。
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint8_t v = w > 1 ? static_cast<uint8_t>((x * 255u) / (w - 1)) : 0;
            const size_t base = (static_cast<size_t>(y) * w + x) * channels;
            for (uint32_t c = 0; c < channels; ++c) {
                // RGB 三个通道给不同的斜坡，免得灰度类算子看不出是三通道
                img.data[base + c] = static_cast<uint8_t>(
                    (v + c * 37u) & 0xFF);
            }
        }
    }
    return img;
}

PointCloudData make_cloud(bool empty_cloud) {
    PointCloudData pc;
    if (empty_cloud) return pc;
    for (int i = 0; i < 12; ++i) {
        pc.add_point(static_cast<float>(i % 4) * 0.1f,
                     static_cast<float>(i / 4) * 0.1f,
                     0.5f);
    }
    pc.width = 4;
    pc.height = 3;
    pc.is_organized = true;
    return pc;
}

DepthImageData make_depth(bool empty_depth) {
    DepthImageData di;
    if (empty_depth) return di;
    di.depth = make_image(8, 8, 1, ImageFormat::Mono16);
    di.color = make_image(8, 8, 3, ImageFormat::RGB8);
    di.depth_scale = 0.001f;
    di.focal_length_x = di.focal_length_y = 100.0f;
    di.center_x = di.center_y = 4.0f;
    return di;
}

/**
 * @brief 造一份"声明成这个类型、内容合法"的数据
 *
 * @param image_size  S4 用 8（又小又脏），S5 用 256（见 audit.h 文件头的偏离说明）
 */
Data canary_of(DataType t, int image_size) {
    const uint32_t n = static_cast<uint32_t>(image_size);
    switch (t) {
        case DataType::Image:      return Data(make_image(n, n, 1, ImageFormat::Mono8));
        case DataType::Region:     return Data(Region(0, 0, 8, 8));
        case DataType::PointCloud: return Data(make_cloud(false));
        case DataType::DepthImage: return Data(make_depth(false));
        case DataType::Number:     return Data(1.0);
        case DataType::Boolean:    return Data(true);
        case DataType::String:     return Data(String("audit"));
        case DataType::Point:      return Data(Point3Df(1.0f, 2.0f, 3.0f));
        case DataType::Pose: {
            Pose p;
            p.x = 1.0; p.y = 2.0; p.z = 3.0;
            return Data(p);
        }
        // Array / Object / None / Any：没有"合法的空壳"这回事，给 None。
        default:                   return Data{};
    }
}

/// S2 用：类型**对**，但内容为空
Data empty_of(DataType t) {
    switch (t) {
        case DataType::Image: {
            ImageData img;
            img.width = 0; img.height = 0; img.channels = 1;
            img.format = ImageFormat::Mono8;
            return Data(img);   // data 是空的
        }
        case DataType::Region:     return Data(Region(0, 0, 0, 0));
        case DataType::PointCloud: return Data(make_cloud(true));
        case DataType::DepthImage: return Data(make_depth(true));
        case DataType::Number:     return Data(0.0);
        case DataType::Boolean:    return Data(false);
        case DataType::String:     return Data(String(""));
        default:                   return Data{};
    }
}

/// S3 用：类型**错**。String 端口特意给 Number —— 反着来才测得出对称性。
Data wrong_type_for(DataType t) {
    if (t == DataType::String) return Data(12345.0);
    return Data(String("ovf-audit-sentinel"));
}

// ===========================================================================
// 计划
// ===========================================================================

struct FuzzValue {
    String label;
    Data   data;
};

String num_label(double v) {
    char buf[32];
    if (std::isfinite(v) && v == std::floor(v) && std::fabs(v) < 1e15) {
        std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(v));
    } else if (std::isfinite(v)) {
        std::snprintf(buf, sizeof(buf), "%g", v);
    } else {
        // NaN / inf 也要能表达出来 —— 它们正是最容易把算子打崩的值
        std::snprintf(buf, sizeof(buf), "%s",
                      std::isnan(v) ? "nan" : (v > 0 ? "inf" : "-inf"));
    }
    return buf;
}

/**
 * @brief 一个参数该试哪些值
 *
 * 规矩：**一次只动一个参数，其余保持默认** —— 这样才能把崩溃归因到具体参数。
 * （`ResizeNode` 那种"单个参数到不了极值路径"的，由调用方在场景之外说明，
 * 不能靠同时动两个参数来掩盖归因。）
 */
Vector<FuzzValue> fuzz_values(const ParamDef& pd) {
    Vector<FuzzValue> out;
    auto push = [&out](const String& label, const Data& d) {
        for (const auto& e : out) {
            if (e.label == label) return;   // 按标签去重，重复的没意义
        }
        out.push_back({label, d});
    };

    switch (pd.type) {
        case DataType::Number: {
            push("0",  Data(0.0));
            push("-1", Data(-1.0));
            // 1e9 而不是 INT_MAX：很多算子会做 v*v，1e9 的平方就已经溢出 int64 了，
            // 再大只是更快地撞死在同一个地方，测不出新东西。
            push("1e9", Data(1e9));
            if (pd.min_value.type() == DataType::Number) {
                const double lo = pd.min_value.as_number();
                push("min:" + num_label(lo), Data(lo));
                push("min-1:" + num_label(lo - 1.0), Data(lo - 1.0));
            }
            if (pd.max_value.type() == DataType::Number) {
                const double hi = pd.max_value.as_number();
                push("max:" + num_label(hi), Data(hi));
                push("max+1:" + num_label(hi + 1.0), Data(hi + 1.0));
            }
            break;
        }
        case DataType::Boolean: {
            push("true",  Data(true));
            push("false", Data(false));
            break;
        }
        case DataType::String: {
            for (const auto& o : pd.options) push(o, Data(o));
            push("\"\"", Data(String("")));
            push("__ovf_invalid__", Data(String("__ovf_invalid__")));
            break;
        }
        default:
            // Image / Region / PointCloud / ... 参数：造不出有意义的极端值，
            // 硬塞一个空 Data 只会得到"类型不符"的噪音，不是缺陷信号。
            break;
    }
    return out;
}

bool has_image_input(const NodeInfo& info) {
    for (const auto& p : info.inputs) {
        if (p.data_type == DataType::Image) return true;
    }
    return false;
}

} // namespace

const char* scenario_name(Scenario s) {
    switch (s) {
        case Scenario::Construct:  return "S0";
        case Scenario::Metadata:   return "S0b";
        case Scenario::NoInput:    return "S1";
        case Scenario::EmptyInput: return "S2";
        case Scenario::WrongType:  return "S3";
        case Scenario::Canary:     return "S4";
        case Scenario::ParamFuzz:  return "S5";
        case Scenario::Outputs:    return "S6";
        default:                   return "?";
    }
}

const char* status_name(Status s) {
    switch (s) {
        case Status::Ok:   return "ok";
        case Status::Fail: return "fail";
        case Status::Warn: return "warn";
        default:           return "skip";
    }
}

Vector<Step> plan_steps(const NodeInfo& info, const Options& opts) {
    Vector<Step> plan;
    plan.reserve(8 + info.inputs.size() * 2 + info.params.size() * 4);

    // ---- S0 构造 ----
    {
        Step s;
        s.scenario = Scenario::Construct;
        plan.push_back(s);
    }

    // ---- S0b 元数据 ----
    {
        Step s;
        s.scenario = Scenario::Metadata;
        plan.push_back(s);
    }

    // ---- S1 什么都不接 ----
    {
        Step s;
        s.scenario = Scenario::NoInput;
        plan.push_back(s);
    }

    if (opts.quick) {
        // --quick 的清单：S0 / S0b / S1 / S3。刻意不含 S2/S4/S5/S6 ——
        // 它要在 CTest 里秒级跑完，就不能有 256×256 的模糊测试。
        for (const auto& p : info.inputs) {
            Step s;
            s.scenario  = Scenario::WrongType;
            s.port      = p.id;
            s.port_type = p.data_type;
            plan.push_back(s);
        }
        return plan;
    }

    // ---- S2 每个输入端口喂"类型对但空的" ----
    for (const auto& p : info.inputs) {
        Step s;
        s.scenario  = Scenario::EmptyInput;
        s.port      = p.id;
        s.port_type = p.data_type;
        plan.push_back(s);
    }

    // ---- S3 每个输入端口喂"错类型的" ----
    for (const auto& p : info.inputs) {
        Step s;
        s.scenario  = Scenario::WrongType;
        s.port      = p.id;
        s.port_type = p.data_type;
        plan.push_back(s);
    }

    // ---- S4 金丝雀 ----
    //
    // ★ 尺寸取 fuzz_image_size(256) 而不是 8，理由和 S5 一样（见 audit.h 文件头的
    //   偏离说明(1)）：8×8 对很多算子是**退化输入**而非"正常输入" —— 测量窗口
    //   比图还大，起点算出负数后按 uint 回绕。那是"小图"这个独立缺陷类，
    //   不是 S4 想测的东西；而它一崩就吃掉重启预算，把后面的 S5 全挡在门外，
    //   于是"复现不出 sampling_interval 死循环"就变成了工具自己的锅。
    //   所以 S4 基准用真实尺寸，"小图"降级成一个单独的 flavour 放到最后。
    {
        Step s;
        s.scenario = Scenario::Canary;
        s.flavour  = "gray";
        plan.push_back(s);
    }
    if (has_image_input(info)) {
        Step s;
        s.scenario = Scenario::Canary;
        s.flavour  = "rgb";
        plan.push_back(s);
    }
    // 小图。和上面两个是**互相独立**的一维：只改尺寸，不改通道数。
    // 8×8 刻意小于绝大多数测量窗口的默认边长，专门逼出"窗口比图大"这类回绕/越界。
    if (has_image_input(info)) {
        Step s;
        s.scenario = Scenario::Canary;
        s.flavour  = "tiny";
        plan.push_back(s);
    }

    // ---- S5 参数模糊测试 ----
    for (const auto& pd : info.params) {
        for (const auto& fv : fuzz_values(pd)) {
            Step s;
            s.scenario    = Scenario::ParamFuzz;
            s.param       = pd.id;
            s.value_label = fv.label;
            s.payload     = fv.data;
            plan.push_back(s);
        }
    }

    // ---- S6 输出端口检查 ----
    {
        Step s;
        s.scenario = Scenario::Outputs;
        s.flavour  = "gray";
        plan.push_back(s);
    }

    return plan;
}

// ===========================================================================
// 一步步执行
// ===========================================================================

namespace {

struct Outcome {
    Status status = Status::Ok;
    int    error_code = 0;
    String detail;
};

INode::Ptr make_node(const String& type_id) {
    // NodeFactory::create 会把构造函数抛出的异常吞掉并返回 nullptr。
    // 所以"构造抛异常"和"构造返回空"在这里是同一件事，都归 CONSTRUCT。
    return NodeFactory::instance().create(type_id, "audit." + type_id);
}

/**
 * @brief S0：只构造 + 核对 `info().id`，**不执行**
 *
 * 刻意不执行：S0 是"这个算子能不能被造出来"，S1 才是"造出来之后能不能跑"。
 * 混在一起的话，一个 execute 期的段错误会被归到 S0 头上，
 * 而 S0 的结论（"构造不出来"）会让父进程直接放弃重启 ——
 * 于是最该被 fuzz 的算子反而一步都跑不到。
 */
Outcome check_construct(const String& type_id) {
    Outcome o;
    auto node = make_node(type_id);
    if (!node) {
        o.status     = Status::Fail;
        o.error_code = static_cast<int>(ErrorCode::NodeNotFound);
        o.detail     = "构造失败：构造函数抛异常或返回了 nullptr";
        return o;
    }

    // 注册用的 type_id 和节点自己声明的 info().id 不一致时，编辑器按其中一个
    // 名字存流程、运行时按另一个查，只会得到 "Node type 'X' not found"，
    // 极难排查。V2 查过注册表那份，这里查的是**实例**那份，两码事。
    const String& declared = node->info().id;
    o.status = Status::Ok;
    if (declared != type_id) {
        o.status = Status::Warn;
        o.detail = "info().id = \"" + declared + "\"，和注册用的 type_id 不一致（V2）";
    }
    return o;
}

Outcome exec_node(const String& type_id, const NodeInfo& info, const Step& step,
                  const Options& opts) {
    FlowContext context;
    auto node = make_node(type_id);
    if (!node) {
        Outcome o;
        o.status     = Status::Fail;
        o.error_code = static_cast<int>(ErrorCode::NodeNotFound);
        o.detail     = "构造失败：构造函数抛异常或返回了 nullptr";
        return o;
    }

    // 连输入
    for (const auto& p : info.inputs) {
        Data d;
        bool set = false;
        switch (step.scenario) {
            case Scenario::NoInput:
                break;   // 一个都不设 —— 这正是 S1 要的
            case Scenario::EmptyInput:
                if (p.id == step.port) { d = empty_of(p.data_type); set = true; }
                break;
            case Scenario::WrongType:
                if (p.id == step.port) { d = wrong_type_for(p.data_type); set = true; }
                break;
            case Scenario::ParamFuzz:
                // S5 要给**全部**输入喂金丝雀，否则"参数极端值"根本没机会生效
                // —— 算子会先因为缺输入干净地失败退出，fuzz 就是空转。
                // 图用 256×256 而不是 8×8，理由见 audit.h 文件头的偏离说明(1)。
                d = canary_of(p.data_type, opts.fuzz_image_size);
                set = true;
                break;
            default: {
                // S4 / S6：金丝雀。每个 flavour 只改一个维度，其余端口照旧：
                //   gray —— 基准（真实尺寸、单通道）
                //   rgb  —— 只把**图像类**端口换成 3 通道，尺寸不变
                //   tiny —— 只把**图像类**端口缩到 8×8，通道数不变
                // 三者的尺寸基准都是 opts.fuzz_image_size，理由见 plan_steps 里 S4 那段。
                const int base = opts.fuzz_image_size;
                if (p.data_type == DataType::Image && step.flavour == "rgb") {
                    d = Data(make_image(base, base, 3, ImageFormat::RGB8));
                } else if (p.data_type == DataType::Image && step.flavour == "tiny") {
                    d = Data(make_image(8, 8, 1, ImageFormat::Mono8));
                } else {
                    d = canary_of(p.data_type, base);
                }
                set = true;
                break;
            }
        }
        if (set) node->set_input(p.id, d);
    }

    if (step.scenario == Scenario::ParamFuzz && !step.param.empty()) {
        node->set_param(step.param, step.payload);
    }

    const int64_t t0 = now_ms();

    // ★ 这一句就是所有崩溃的现场。异常由这一层的 catch 兜住，
    //   段错误/挂死则由父进程按信号/超时兜住 —— 两条路都会留下证据。
    Result<void> r = Result<void>::success();
    try {
        r = node->execute(context);
    } catch (const std::exception& e) {
        Outcome caught;
        caught.status     = Status::Fail;
        caught.error_code = static_cast<int>(ErrorCode::Unknown);
        caught.detail     = String("execute 抛出 std::exception: ") + e.what();
        return caught;
    } catch (...) {
        Outcome caught;
        caught.status     = Status::Fail;
        caught.error_code = static_cast<int>(ErrorCode::Unknown);
        caught.detail     = "execute 抛出了非 std::exception 的异常";
        return caught;
    }
    (void)t0;   // 计时由调用方的 emit_end 统一负责，免得两处各算一遍

    Outcome o;
    o.error_code = static_cast<int>(r.code());
    o.detail     = r.message();
    o.status = r.is_success() ? Status::Ok : Status::Fail;

    // S6 额外的活儿：声明过的输出端口必须真的有东西
    if (step.scenario == Scenario::Outputs) {
        if (!r.is_success()) {
            // 金丝雀本身就没跑通 → 这里判不了输出。设为 Skip 而不是 Fail：
            // "跑不通"和"跑通了但不产出"是两个信号，混在一起会让报告里的
            // note 说不清这个算子到底怎么了。
            o.status = Status::Skip;
            o.detail = "金丝雀没跑通，无法判断输出端口（这一条由 S4 负责）";
        } else {
            String missing;
            for (const auto& p : info.outputs) {
                const Data out = node->get_output(p.id);
                if (out.type() == DataType::None) {
                    if (!missing.empty()) missing += ", ";
                    missing += p.id;
                }
            }
            if (!missing.empty()) {
                o.status = Status::Warn;
                o.detail = "声明了的输出端口没有产出: " + missing;
            }
        }
    }

    return o;
}

/// 造 S0b 那一行：元数据校验
Outcome check_metadata(const String& type_id, const NodeInfo& info) {
    const Vector<MetadataIssue> issues = validate_node_info(type_id, info);
    int errors = 0;
    String first;
    for (const auto& i : issues) {
        if (i.severity == MetadataIssue::Severity::Error) {
            ++errors;
            if (first.empty()) first = i.rule + " " + i.message;
        }
    }
    Outcome o;
    if (errors == 0) {
        o.status = Status::Ok;
        if (!issues.empty()) {
            o.detail = "仅 Note 级 " + std::to_string(issues.size()) + " 条（作者/版本未填之类）";
        }
    } else {
        o.status = Status::Warn;
        o.error_code = errors;
        o.detail = std::to_string(errors) + " 条元数据问题；第一条: " + first;
    }
    return o;
}

// ===========================================================================
// 资源上限
// ===========================================================================

void apply_rlimits(const Options& opts, int* out_effective_mem_mb, String* out_warning) {
    struct rlimit rl;

    // --- 地址空间 ---
    // ⚠️ 坑：fork 出来的子进程**继承了父进程的整个地址空间**（COW 共享）。
    // RLIMIT_AS 限制的是这个进程的虚拟地址空间总大小，所以如果父进程自己
    // 就已经占了 800MB，设 1GB 只会让子进程**下一步分配就失败**，
    // 于是所有算子都会被误判成"撑爆内存"。
    // 遇到这种情况就放大到当前占用的 2 倍，并且**在报告里说出来** ——
    // 绝不让体检自己的预算伪装成算子缺陷，也不能反过来假装限制生效了。
    long vm_kb = 0;
    {
        FILE* f = std::fopen("/proc/self/statm", "r");
        if (f) {
            long total_pages = 0, resident = 0;
            if (std::fscanf(f, "%ld %ld", &total_pages, &resident) == 2) {
                vm_kb = total_pages * (static_cast<long>(::sysconf(_SC_PAGESIZE)) / 1024);
            }
            std::fclose(f);
        }
    }

    long long want_kb = static_cast<long long>(opts.mem_limit_mb) * 1024;
    long long eff_kb  = want_kb;
    if (vm_kb > 0 && vm_kb * 2 > want_kb) {
        eff_kb = vm_kb * 2;
        if (out_warning) {
            *out_warning = "内存上限 " + std::to_string(opts.mem_limit_mb) +
                           "MB 低于进程自身占用(" + std::to_string(vm_kb / 1024) +
                           "MB)的两倍，本次实际生效 " + std::to_string(eff_kb / 1024) +
                           "MB";
        }
    }
    if (out_effective_mem_mb) {
        *out_effective_mem_mb = static_cast<int>(eff_kb / 1024);
    }
    rl.rlim_cur = rl.rlim_max = static_cast<rlim_t>(eff_kb) * 1024;
    ::setrlimit(RLIMIT_AS, &rl);

    // --- CPU 秒数：父进程墙钟超时的兜底（墙钟到了先杀，这条几乎不会触发）---
    const long cpu_s = static_cast<long>((opts.budget_ms + opts.hard_grace_ms) / 1000) + 5;
    rl.rlim_cur = rl.rlim_max = static_cast<rlim_t>(cpu_s);
    ::setrlimit(RLIMIT_CPU, &rl);

    // --- 单文件最大 16MB：会写文件的算子不能把 /tmp 撑爆 ---
    rl.rlim_cur = rl.rlim_max = 16ull * 1024 * 1024;
    ::setrlimit(RLIMIT_FSIZE, &rl);

    // --- 打开的文件数 ---
    rl.rlim_cur = rl.rlim_max = 256;
    ::setrlimit(RLIMIT_NOFILE, &rl);

    // --- 不许自己 fork 进程 ---
    rl.rlim_cur = rl.rlim_max = 0;
    ::setrlimit(RLIMIT_NPROC, &rl);

    // --- ★ 绝不产生 core 文件 ---
    // 501 个子进程里凡是崩的都会试图 dump core，一个 core 几十 MB，
    // 跑一轮体检能把磁盘写满。这条不是可选项。
    rl.rlim_cur = rl.rlim_max = 0;
    ::setrlimit(RLIMIT_CORE, &rl);
}

} // namespace

// ===========================================================================
// 子进程入口
// ===========================================================================

void run_child(const String& type_id, const Options& opts, int out_fd) {
    // 1. 第一件事：把日志沉掉。否则每个算子的 INFO/WARN 都会和报告混在一起。
    Logger::instance().clear_sinks();

    // 2. stdout/stderr 到 /dev/null。算子里的 printf / std::cout 一律丢弃。
    //    ★ 重定向之后**必须**把 stdio 的缓冲清空，否则 fork 时从父进程
    //      带过来的那点内容会野到 /dev/null 里去 —— 无害，但脏。
    {
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            ::dup2(devnull, STDOUT_FILENO);
            ::dup2(devnull, STDERR_FILENO);
            if (devnull > STDERR_FILENO) ::close(devnull);
        }
        std::fflush(nullptr);
    }

    // 3. 自己的临时目录，防止会写文件的算子互相污染。
    {
        String dir = "/tmp/ovf-node-audit/" + type_id;
        ::mkdir("/tmp/ovf-node-audit", 0700);
        ::mkdir(dir.c_str(), 0700);
        if (::chdir(dir.c_str()) != 0) {
            // 建不出就退回 /tmp，不值得为此终止；/tmp 都进不去就硬着头皮跑。
            // 两处都取返回值，不然 glibc 的 warn_unused_result 会报。
            if (::chdir("/tmp") != 0) {
                emit(out_fd, "{\"k\":\"meta_note\",\"detail\":\"chdir 失败，cwd 未切换\"}");
            }
        }
    }

    int effective_mem_mb = opts.mem_limit_mb;
    String rlimit_warning;
    apply_rlimits(opts, &effective_mem_mb, &rlimit_warning);

    // 4. 拿元数据。拿不到就没什么可跑的了。
    const NodeInfo* info_ptr = NodeFactory::instance().get_info(type_id);
    if (!info_ptr) {
        emit(out_fd, "{\"k\":\"abort\",\"why\":\"get_info 返回空：这个 type_id 没注册\"}");
        emit(out_fd, "{\"k\":\"done\",\"n\":0}");
        _exit(0);
    }
    const NodeInfo info = *info_ptr;   // 拷一份：注册表在父进程里可能还会动

    emit(out_fd, "{\"k\":\"meta\",\"cat\":\"" + json_escape(info.category) +
                     "\",\"in\":" + std::to_string(info.inputs.size()) +
                     ",\"out\":" + std::to_string(info.outputs.size()) +
                     ",\"par\":" + std::to_string(info.params.size()) +
                     ",\"mem\":" + std::to_string(effective_mem_mb) + "}");
    if (!rlimit_warning.empty()) {
        emit(out_fd, "{\"k\":\"budget\",\"rlimit\":\"" + json_escape(rlimit_warning) + "\"}");
    }

    const Vector<Step> plan = plan_steps(info, opts);

    const int from = opts.start_step < 0 ? 0 : opts.start_step;
    const int to   = (opts.stop_after_step < 0 || opts.stop_after_step >= static_cast<int>(plan.size()))
                         ? static_cast<int>(plan.size()) - 1
                         : opts.stop_after_step;

    const int64_t t_start = now_ms();
    int executed = 0;

    for (int i = from; i <= to; ++i) {
        // 软预算：过了就不再开新步。父进程的硬杀在 budget + grace，
        // 比这里晚，所以正常情况下最后一行永远是 end 或 budget，不是被杀。
        if (i > from) {
            const int spent = static_cast<int>(now_ms() - t_start);
            if (spent > opts.budget_ms) {
                emit(out_fd, "{\"k\":\"budget\",\"skipped\":" +
                                 std::to_string(to - i + 1) + "}");
                break;
            }
        }

        const Step& step = plan[i];
        emit_begin(out_fd, i, step);

        const int64_t t0 = now_ms();
        Outcome o;
        if (step.scenario == Scenario::Metadata) {
            o = check_metadata(type_id, info);
        } else if (step.scenario == Scenario::Construct) {
            o = check_construct(type_id);
        } else {
            o = exec_node(type_id, info, step, opts);
        }

        Evidence ev;
        ev.scenario   = step.scenario;
        ev.status     = o.status;
        ev.error_code = o.error_code;
        ev.detail     = o.detail;
        ev.flavour    = step.flavour;
        ev.port       = step.port;
        ev.param      = step.param;
        ev.value      = step.value_label;
        ev.ms         = static_cast<int>(now_ms() - t0);
        emit_end(out_fd, i, ev);

        ++executed;

        // 构造都失败的话，后面每一步都会以同样的方式失败 —— 立刻停下来，
        // 让父进程知道"别再重启了"，省下几次无意义的重启。
        // 注意条件是 Fail 而不是 != Ok：S0 因为 info().id 不一致返回 Warn，
        // 那是元数据问题，不代表后面跑不动。
        if (step.scenario == Scenario::Construct && o.status == Status::Fail) {
            emit(out_fd, "{\"k\":\"abort\",\"why\":\"构造不出来，后面的步骤不会有新结论\"}");
            break;
        }
    }

    emit(out_fd, "{\"k\":\"done\",\"n\":" + std::to_string(executed) + "}");

    // ★ 只准 _exit。exit() 会跑 atexit、冲 stdio —— 那些东西是从父进程继承来的。
    _exit(0);
}

} // namespace audit
} // namespace ovf
