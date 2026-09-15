/**
 * @file test_node_helpers.cpp
 * @brief 写算子的安全带 API 测试（阶段 3）
 *
 * 被测对象是 INode 上那一组新接口：in_image / in_data / p_num / p_num_in /
 * p_str / step_ok / fail_node / warn_once，以及 OVF_TRY_IN / OVF_TRY 宏。
 *
 * 四条主线，对应计划书的验收门：
 *   1. 缺输入要**带节点名**报出真实原因，而不是静默返回空数据
 *   2. 类型不符要说 "expects Image, got Number"
 *   3. 参数越界会 clamp，并且**真的打了 WARN**（且同一个参数只打一次）
 *   4. step_ok 在超预算 / 被叫停之后返回 false
 *
 * 另外补了两条不变量：fail_node 要带节点名并写进 error_message()；
 * error_result() 要能转成**任意** Result<U>（这是 OVF_TRY_IN 能工作的前提）。
 */

#include "test_framework.h"
#include "ovf/core/node.h"
#include "ovf/core/flow.h"
#include "ovf/core/data.h"
#include "ovf/core/error.h"
#include "ovf/core/logger.h"
#include <memory>
#include <utility>

using namespace ovf;
using namespace ovf_test;

// ============================================================================
// 测试脚手架
// ============================================================================

/**
 * @brief 把日志收进内存的 sink，用来断言"确实打了 WARN"
 *
 * 断言日志内容不是形式主义：clamp 如果只夹了值却不报警，就是**静默改行为** ——
 * 那比崩溃更难查（结果悄悄变了，没人知道为什么）。
 */
class CapturingSink : public ILogSink {
public:
    void write(LogLevel level, const String& message,
               const String&, int, const String&) override {
        entries_.emplace_back(level, message);
    }

    /// 是否存在一条 level 级、且正文包含 needle 的日志
    bool has(LogLevel level, const String& needle) const {
        for (const auto& e : entries_) {
            if (e.first == level && e.second.find(needle) != String::npos) {
                return true;
            }
        }
        return false;
    }

    size_t count(LogLevel level) const {
        size_t n = 0;
        for (const auto& e : entries_) {
            if (e.first == level) ++n;
        }
        return n;
    }

    void clear() { entries_.clear(); }

private:
    Vector<std::pair<LogLevel, String>> entries_;
};

/**
 * @brief 探针节点 —— 把 INode 的 protected 安全带 API 摊开给测试
 *
 * 真实算子是在 execute() 内部直接调这些方法的；这里换个入口，
 * 免得每个断言都非要拼一整个流程才能验到。
 */
class ProbeNode : public INode {
public:
    ProbeNode() : INode("probe", make_info()) {}

    static NodeInfo make_info() {
        NodeInfo info;
        info.id          = "test.probe";
        info.name        = "探针";
        info.category    = "test";
        info.version     = "1.0";
        info.author      = "ovf-tests";

        info.inputs.push_back(DataPort("image", "输入图像", DataType::Image));
        info.inputs.push_back(DataPort("region", "输入区域", DataType::Region));
        info.inputs.push_back(DataPort("num", "输入数值", DataType::Number));
        // 声明了默认值的端口：不连线也应该拿到 7.0，与 get_input() 的既有语义一致
        info.inputs.push_back(
            DataPort("withdefault", "带默认值", DataType::Number).default_to(Data(7.0)));

        info.outputs.push_back(DataPort("out", "输出", DataType::Number));

        info.params.push_back(
            ParamDef("threshold", "阈值", DataType::Number, Data(128)).range(0, 255));
        info.params.push_back(
            ParamDef("kernel", "核大小", DataType::Number, Data(3)).range(1, 31));
        info.params.push_back(
            ParamDef("mode", "模式", DataType::String, Data(String("fast")))
                .choices({"fast", "accurate"}));
        // 故意**不声明范围** —— p_num_in 就是给这种参数就地加约束用的
        info.params.push_back(
            ParamDef("unbounded", "无范围参数", DataType::Number, Data(5)));
        return info;
    }

    Result<void> execute(FlowContext&) override { return Result<void>::success(); }

    // 走一遍真实算子会写的那条路：OVF_TRY_IN 取输入 + p_int_in 取参数 + step_ok 循环。
    // 顺便验证 OVF_TRY_IN 能把 Result<T> 的失败转成 Result<double> 的失败。
    Result<double> run_pipeline(FlowContext& ctx, uint64_t max_iter) {
        OVF_TRY_IN(v, in_number("num"));
        OVF_TRY_IN(k, p_int_in("kernel", 1, 31));

        double acc = v;
        for (uint64_t i = 0; i < 1000; ++i) {
            if (!step_ok(ctx, i, max_iter)) break;
            acc += static_cast<double>(k);
        }
        return Result<double>::success(acc);
    }

    // 把 protected 提到 public（只影响这个派生类）
    using INode::in_data;
    using INode::in_image;
    using INode::in_number;
    using INode::in_region;
    using INode::p_int;
    using INode::p_int_in;
    using INode::p_num;
    using INode::p_num_in;
    using INode::p_str;
    using INode::step_ok;
    using INode::fail_node;
    using INode::warn_once;
};

/// 造一张 w×h 单通道、像素值为 fill 的图
static ImageData make_image(uint32_t w, uint32_t h, uint8_t fill) {
    ImageData img;
    img.width    = w;
    img.height   = h;
    img.channels = 1;
    img.format   = ImageFormat::Mono8;
    img.data.assign(static_cast<size_t>(w) * h, fill);
    return img;
}

// ============================================================================
// 一、输入：缺输入 / 类型错都要报得清楚
// ============================================================================

TEST(NodeHelpers, InImageOnUnconnectedPortFailsWithNodeName) {
    ProbeNode node;

    // "image" 是声明过的，但没连线、也没有默认值
    auto r = node.in_image("image");

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::InvalidData, r.code());
    // 验收门原文：消息必须含节点名，否则画布上同类节点一多就不知道是谁报的
    ASSERT_NE(String::npos, r.message().find("probe"));
    ASSERT_NE(String::npos, r.message().find("image"));
    ASSERT_NE(String::npos, r.message().find("no data"));

    // 失败结果上取 value() 必须抛，不能返回一个空图让人继续算
    ASSERT_THROW(r.value(), Exception);
}

TEST(NodeHelpers, InImageOnUnknownPortReportsDeclaredPorts) {
    ProbeNode node;

    // 端口名拼错：这跟"没连线"是两回事，修法完全不同，必须分开报
    auto r = node.in_image("iamge");

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::NotFound, r.code());
    ASSERT_NE(String::npos, r.message().find("Declared inputs"));
    ASSERT_NE(String::npos, r.message().find("image"));
    ASSERT_NE(String::npos, r.message().find("num"));
}

TEST(NodeHelpers, InImageOnWrongTypeSaysExpectedAndGot) {
    ProbeNode node;

    // 验收门原文：喂 Data(3.0) 时失败且说 "expects Image, got Number"
    node.set_input("image", Data(3.0));

    auto r = node.in_image("image");

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::InvalidData, r.code());
    ASSERT_NE(String::npos, r.message().find("expects Image, got Number"));
}

TEST(NodeHelpers, InImageOnConnectedPortReturnsTheImage) {
    ProbeNode node;
    node.set_input("image", Data(make_image(4, 3, 7)));

    auto r = node.in_image("image");

    ASSERT_TRUE(r.is_success());
    ASSERT_EQ(4u, (*r).width);
    ASSERT_EQ(3u, (*r).height);
    ASSERT_EQ(12u, (*r).data.size());
    ASSERT_EQ(7, static_cast<int>((*r).data[0]));
}

TEST(NodeHelpers, InNumberFallsBackToDeclaredDefault) {
    ProbeNode node;

    // 没连线但声明了默认值 —— 这条路必须通，否则带默认值的端口全都变成"缺输入"
    auto r = node.in_number("withdefault");

    ASSERT_TRUE(r.is_success());
    ASSERT_NEAR(7.0, *r, 1e-9);
}

TEST(NodeHelpers, InRegionRejectsNumberInput) {
    ProbeNode node;
    node.set_input("region", Data(1.5));

    auto r = node.in_region("region");

    ASSERT_TRUE(r.is_failure());
    ASSERT_NE(String::npos, r.message().find("expects Region, got Number"));
}

// ============================================================================
// 二、参数：越界 clamp + 打 WARN + 拼错名字直接失败
// ============================================================================

TEST(NodeHelpers, PNumInClampsAndLogsOnce) {
    auto sink = std::make_shared<CapturingSink>();
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(sink);

    ProbeNode node;

    // 验收门原文：p_num_in 会 clamp 并记日志
    node.set_param("unbounded", Data(999.0));
    auto r = node.p_num_in("unbounded", 0.0, 10.0);

    ASSERT_TRUE(r.is_success());
    ASSERT_NEAR(10.0, *r, 1e-9);
    ASSERT_TRUE(sink->has(LogLevel::Warning, "unbounded"));
    ASSERT_TRUE(sink->has(LogLevel::Warning, "clamped to 10"));

    // 只警告一次：同一个参数越界一万次也不该刷爆日志
    const size_t before = sink->count(LogLevel::Warning);
    auto again = node.p_num_in("unbounded", 0.0, 10.0);
    ASSERT_TRUE(again.is_success());
    ASSERT_NEAR(10.0, *again, 1e-9);
    ASSERT_EQ(before, sink->count(LogLevel::Warning));
}

TEST(NodeHelpers, PNumUsesDeclaredRange) {
    auto sink = std::make_shared<CapturingSink>();
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(sink);

    ProbeNode node;

    // 声明过 .range(0, 255)：不用调用方再说一遍
    node.set_param("threshold", Data(300.0));
    auto hi = node.p_num("threshold");
    ASSERT_TRUE(hi.is_success());
    ASSERT_NEAR(255.0, *hi, 1e-9);
    ASSERT_TRUE(sink->has(LogLevel::Warning, "declared range"));

    // 范围内原样返回，不许乱动
    node.set_param("threshold", Data(100.0));
    auto ok = node.p_num("threshold");
    ASSERT_TRUE(ok.is_success());
    ASSERT_NEAR(100.0, *ok, 1e-9);

    // p_int_in 对 kernel 这类"0 就会算出垃圾"的参数特别有用
    node.set_param("kernel", Data(0.0));
    auto k = node.p_int_in("kernel", 1, 31);
    ASSERT_TRUE(k.is_success());
    ASSERT_EQ(1, *k);
}

TEST(NodeHelpers, PNumInWithInvertedRangeFails) {
    ProbeNode node;

    // 边界写反是算子代码自己的 bug，不能凑合夹
    auto r = node.p_num_in("unbounded", 10.0, 0.0);

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::InvalidParameter, r.code());
    ASSERT_NE(String::npos, r.message().find("inverted range"));
}

TEST(NodeHelpers, PNumOnUnknownParamFailsInsteadOfReturningZero) {
    ProbeNode node;

    // 老 API 这里会静默返回 Data{}，等于拿 0 去算 —— 拼错一个字母就查半天
    auto r = node.p_num("treshold");

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::NotFound, r.code());
    ASSERT_NE(String::npos, r.message().find("Declared params"));
    ASSERT_NE(String::npos, r.message().find("threshold"));
}

TEST(NodeHelpers, PNumRejectsWrongDeclaredType) {
    ProbeNode node;

    // "mode" 是 String，用数值去读它是算子代码写错了，必须报出来
    auto r = node.p_num("mode");

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::InvalidParameter, r.code());
    ASSERT_NE(String::npos, r.message().find("read as Number"));
}

TEST(NodeHelpers, PStrRejectsValueOutsideOptions) {
    ProbeNode node;

    node.set_param("mode", Data(String("turbo")));
    auto bad = node.p_str("mode");
    ASSERT_TRUE(bad.is_failure());
    ASSERT_NE(String::npos, bad.message().find("not one of"));
    ASSERT_NE(String::npos, bad.message().find("accurate"));

    node.set_param("mode", Data(String("accurate")));
    auto good = node.p_str("mode");
    ASSERT_TRUE(good.is_success());
    ASSERT_EQ(String("accurate"), *good);
}

// ============================================================================
// 三、循环：step_ok
// ============================================================================

TEST(NodeHelpers, StepOkStopsOnBudget) {
    auto sink = std::make_shared<CapturingSink>();
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(sink);

    ProbeNode node;
    FlowContext ctx;

    // 验收门原文：step_ok 在超预算后返回 false
    ASSERT_TRUE(node.step_ok(ctx, 0, 5));
    ASSERT_TRUE(node.step_ok(ctx, 4, 5));
    ASSERT_FALSE(node.step_ok(ctx, 5, 5));

    // 超预算要留下痕迹：结果可能是不完整的，不能悄悄少算
    ASSERT_TRUE(sink->has(LogLevel::Warning, "budget"));
}

TEST(NodeHelpers, StepOkStopsOnStopRequestWithoutWarning) {
    auto sink = std::make_shared<CapturingSink>();
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(sink);

    ProbeNode node;
    FlowContext ctx;
    ctx.stop();

    // max_iterations = 0 表示不限次数，只受"被叫停"约束
    ASSERT_FALSE(node.step_ok(ctx, 0, 0));

    // 停止是正常操作，不是异常 —— 不该刷警告
    ASSERT_EQ(0u, sink->count(LogLevel::Warning));
}

// ============================================================================
// 四、失败与错误传播
// ============================================================================

TEST(NodeHelpers, FailNodeCarriesNodeNameAndSetsErrorMessage) {
    ProbeNode node;

    auto r = node.fail_node(ErrorCode::AlgorithmExecFailed, "template matching blew up");

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::AlgorithmExecFailed, r.code());
    ASSERT_NE(String::npos, r.message().find("probe"));
    ASSERT_NE(String::npos, r.message().find("template matching blew up"));
    // 引擎的失败路径读的是 error_message()，这里必须同步写上
    ASSERT_NE(String::npos, node.error_message().find("template matching blew up"));

    // 模板版：换成别的结果类型也要成立
    auto r2 = node.fail_node<ImageData>(ErrorCode::InvalidImage, "bad image");
    ASSERT_TRUE(r2.is_failure());
    ASSERT_EQ(ErrorCode::InvalidImage, r2.code());
    ASSERT_NE(String::npos, r2.message().find("bad image"));
}

TEST(NodeHelpers, ErrorResultConvertsToAnyResultType) {
    ProbeNode node;
    auto src = node.in_image("image");       // 注定失败：没连线
    ASSERT_TRUE(src.is_failure());

    // 这是 OVF_TRY_IN 能工作的前提：Result<ImageData> 的失败要能变成
    // 外层函数的 Result<void> / Result<double> / ... 而 U 没法从 return 反推
    Result<void>    as_void   = src.error_result();
    Result<double>  as_double = src.error_result();
    Result<int32_t> as_int    = src.error_result();

    ASSERT_TRUE(as_void.is_failure());
    ASSERT_TRUE(as_double.is_failure());
    ASSERT_TRUE(as_int.is_failure());
    ASSERT_EQ(src.code(), as_void.code());
    ASSERT_EQ(src.message(), as_void.message());
    ASSERT_EQ(src.code(), as_int.code());
}

TEST(NodeHelpers, TryInMacroBindsValueAndPropagatesFailure) {
    ProbeNode node;
    FlowContext ctx;

    node.set_input("num", Data(2.0));
    node.set_param("kernel", Data(100.0));   // 超出 .range(1,31) → 夹到 31

    auto r = node.run_pipeline(ctx, 3);

    ASSERT_TRUE(r.is_success());
    // 2.0 + 3 次 × 31 = 95
    ASSERT_NEAR(95.0, *r, 1e-9);

    // 同样的调用，这次 "num" 没连线 → 失败必须原样冒出来，且带节点名
    ProbeNode bare;
    auto failed = bare.run_pipeline(ctx, 3);

    ASSERT_TRUE(failed.is_failure());
    ASSERT_NE(String::npos, failed.message().find("probe"));
    ASSERT_NE(String::npos, failed.message().find("num"));
}

TEST(NodeHelpers, TryMacroPropagatesWithoutBinding) {
    ProbeNode node;

    auto r = node.fail_node<void>(ErrorCode::NotSupported, "not today");
    ASSERT_TRUE(r.is_failure());

    // 下一行就是 OVF_TRY 的用法；这里直接调用宏，验证它把失败原样传出去
    auto propagate = [&]() -> Result<int32_t> {
        OVF_TRY(node.fail_node<void>(ErrorCode::NotSupported, "not today"));
        return Result<int32_t>::success(42);
    }();

    ASSERT_TRUE(propagate.is_failure());
    ASSERT_EQ(ErrorCode::NotSupported, propagate.code());
    ASSERT_EQ(String("Node 'probe': not today"), propagate.message());
}

// ============================================================================

int main() {
    // 只留内存 sink：控制台 sink 会把每条 WARN 都打到 stdout，塞满测试报告。
    // 本测试关心的是"日志有没有被写出来"，不是写到哪儿。
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(std::make_shared<CapturingSink>());

    TestStats stats = TestRunner::run_all_tests();
    TestRunner::save_report(stats, "test_node_helpers_report.json");
    return stats.failed_tests;
}
