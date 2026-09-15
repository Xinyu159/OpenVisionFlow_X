/**
 * @file test_user_nodes.cpp
 * @brief 用户算子模块的自检与 `--as-needed` 防御测试（阶段 4）
 *
 * 最要紧的一条在最前面：**这个文件从头到尾没有调用过 `initialize_user_nodes()`**。
 *
 * 那就是重点。它验证的是第 1 层防御（`ovf-nodes-link` 里的 `--no-as-needed`）
 * 单独也能让算子注册上 —— 少一层兜底都还是活的。如果有人把链接目标从
 * `ovf-nodes-link` 改回 `ovf-nodes`，这个测试会挂，而且挂的方式正好就是
 * 生产环境会发生的那个：算子全没了、链接期零警告。
 *
 * 其余几条覆盖第 3 层的判定逻辑（missing / hijacked / 消息是否可操作）
 * 和示例算子的实际行为。
 */

#include "test_framework.h"

#include "ovf/nodes/nodes.h"

#include <memory>

using namespace ovf;
using namespace ovf_test;
using namespace ovf::nodes;

// ============================================================================
// 测试脚手架
// ============================================================================

/// 把日志收进内存，用来断言"确实打了 WARN"（抄阶段 3 的写法）
class CapturingSink : public ILogSink {
public:
    void write(LogLevel level, const String& message,
               const String&, int, const String&) override {
        entries_.emplace_back(level, message);
    }

    bool has(LogLevel level, const String& needle) const {
        for (const auto& e : entries_) {
            if (e.first == level && e.second.find(needle) != String::npos) return true;
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

private:
    Vector<std::pair<LogLevel, String>> entries_;
};

static ImageData make_image(uint32_t w, uint32_t h, uint8_t fill) {
    ImageData img;
    img.width    = w;
    img.height   = h;
    img.channels = 1;
    img.format   = ImageFormat::Mono8;
    img.data.assign(static_cast<size_t>(w) * h, fill);
    return img;
}

/// 造一个最小的 NodeInfo，用来在测试里注册一个"别人的"节点
static NodeInfo minimal_info(const String& id) {
    NodeInfo info;
    info.id       = id;
    info.name     = id;
    info.category = "test";
    info.version  = "0.0.0";
    info.author   = "ovf-tests";
    info.outputs.push_back(DataPort("out", "输出", DataType::Number));
    return info;
}

// ============================================================================
// 一、第 1 层防御：不调 initialize_user_nodes() 也要有算子
// ============================================================================

TEST(UserNodes, ModuleAnchorIsPresent) {
    // 弱符号锚点解析到了定义 —— libovf-nodes.so 确实进了这个进程
    ASSERT_TRUE(user_node_module_loaded());
    ASSERT_EQ(String("ovf-nodes"), String(module_name()));

    // 反向确认一下这个断言不是恒真的：锚点确实解析到了，内容也对得上
    ASSERT_TRUE(ovf_nodes_module_anchor != nullptr);
    ASSERT_EQ(String("ovf-nodes"), String(ovf_nodes_module_anchor()));
}

TEST(UserNodes, NodesVisibleWithoutExplicitInit) {
    // ★ 本文件的 main() 里没有 initialize_user_nodes()。
    //   如果链接器把 libovf-nodes.so 丢了，静态初始化不跑，
    //   这个断言就会挂 —— 正是我们要在构建期抓住的失败。
    auto& factory = NodeFactory::instance();
    ASSERT_TRUE(factory.has_type("nodes.Threshold"));

    auto node = factory.create("nodes.Threshold", "probe_1");
    ASSERT_TRUE(node != nullptr);
    ASSERT_EQ(String("nodes.Threshold"), node->info().id);
}

TEST(UserNodes, ModuleIsRegisteredAsStrict) {
    // OVF_REGISTER_USER_NODE 等价于 OVF_REGISTER_NODE_STRICT + 记清单。
    // 严格模式意味着元数据问题会在 initialize_user_nodes() 里升级为致命，
    // 自己的算子从第一天起就得是干净的。
    const Vector<String> strict = NodeFactory::instance().strict_types();
    bool found = false;
    for (const auto& t : strict) {
        if (t == "nodes.Threshold") found = true;
    }
    ASSERT_TRUE(found);
}

// ============================================================================
// 二、第 3 层判定：missing / hijacked 要分得开
// ============================================================================

TEST(UserNodes, DeclaredMatchesRegistered) {
    const UserNodeReport report = check_user_nodes();

    ASSERT_TRUE(report.ok());
    ASSERT_EQ(1u, report.declared.size());
    ASSERT_EQ(String("nodes.Threshold"), report.declared[0]);
    ASSERT_EQ(1u, report.registered.size());
    ASSERT_EQ(0u, report.missing.size());
    ASSERT_EQ(0u, report.hijacked.size());

    // 真清单和注册表里的出处标签要对得上
    const Vector<String> tagged = tagged_type_ids();
    bool found = false;
    for (const auto& t : tagged) {
        if (t == "nodes.Threshold") found = true;
    }
    ASSERT_TRUE(found);
}

TEST(UserNodes, CheckReportsMissingTypeId) {
    // "根本没有"和"被抢先占了"是两回事 —— 修法完全不同，所以判定必须分开
    Vector<String> declared;
    declared.push_back("nodes.Threshold");
    declared.push_back("nodes.DoesNotExistAtAll");

    const UserNodeReport report = check_user_nodes_against(declared);

    ASSERT_FALSE(report.ok());
    ASSERT_EQ(2u, report.declared.size());
    ASSERT_EQ(1u, report.registered.size());
    ASSERT_EQ(1u, report.missing.size());
    ASSERT_EQ(String("nodes.DoesNotExistAtAll"), report.missing[0]);
    ASSERT_EQ(0u, report.hijacked.size());
}

TEST(UserNodes, CheckReportsHijackedTypeId) {
    // 造一个"别的模块先注册了同名 type_id"的局面：
    // FirstWins 策略下先注册的先赢，我们那份会被**静默丢弃**。
    auto& factory = NodeFactory::instance();
    const String before = factory.current_origin();

    factory.set_origin("some-other-module");
    factory.register_node("test.hijacked", [](const String&) -> INode::Ptr { return nullptr; },
                          minimal_info("test.hijacked"));
    factory.set_origin(before);

    Vector<String> declared;
    declared.push_back("test.hijacked");
    const UserNodeReport report = check_user_nodes_against(declared);

    ASSERT_FALSE(report.ok());
    ASSERT_TRUE(report.missing.empty());     // 它在注册表里……
    ASSERT_EQ(1u, report.hijacked.size());   // ……但不是我们的
    ASSERT_EQ(String("test.hijacked"), report.hijacked[0]);
}

TEST(UserNodes, ReportMessageIsActionable) {
    Vector<String> declared;
    declared.push_back("nodes.Threshold");
    declared.push_back("nodes.DoesNotExistAtAll");

    const String msg = format_user_node_report(check_user_nodes_against(declared));

    // 报错的价值全在"该改哪一行"。这三样缺一不可：
    ASSERT_NE(String::npos, msg.find("nodes.DoesNotExistAtAll"));  // 是谁没了
    ASSERT_NE(String::npos, msg.find("--as-needed"));              // 最可能的原因
    ASSERT_NE(String::npos, msg.find("ovf-nodes-link"));           // 具体怎么修

    // 全过的时候不该报成一团乱麻
    Vector<String> ok_declared;
    ok_declared.push_back("nodes.Threshold");
    const String ok_msg = format_user_node_report(check_user_nodes_against(ok_declared));
    ASSERT_EQ(String::npos, ok_msg.find("失败"));
}

TEST(UserNodes, RequirePassesOnHealthyModule) {
    // 当前模块是好的，硬断言不该抛。抛的那条路（库不在 / 算子被顶掉）
    // 由阶段 4 的**反向测试**实测：临时删掉 main.cpp 里那句调用重编，
    // 确认启动时报的是可操作错误而不是静默少节点。见 docs/hardening/stage4_nodes.md。
    bool threw = false;
    try {
        require_user_nodes();
        require_user_node_module_loaded("test_user_nodes");
    } catch (const Exception&) {
        threw = true;
    }
    ASSERT_FALSE(threw);
}

// ============================================================================
// 三、示例算子：安全带的实际效果
// ============================================================================

TEST(UserNodes, ThresholdBinarizesImage) {
    auto node = NodeFactory::instance().create("nodes.Threshold", "th_1");
    ASSERT_TRUE(node != nullptr);

    FlowContext ctx;
    node->set_input("image", Data(make_image(4, 4, 100)));

    // 默认 threshold = 128，100 < 128 → 全 0
    ASSERT_TRUE(node->execute(ctx).is_success());
    Data out = node->get_output("image");
    ASSERT_TRUE(out.is_image());
    const ImageData& dst0 = out.as_image();
    ASSERT_EQ(16u, dst0.data.size());
    ASSERT_EQ(0, static_cast<int>(dst0.data[0]));

    // 阈值降到 50，100 >= 50 → 全 255
    node->set_param("threshold", Data(50.0));
    ASSERT_TRUE(node->execute(ctx).is_success());
    Data out2 = node->get_output("image");
    const ImageData& dst1 = out2.as_image();
    ASSERT_EQ(255, static_cast<int>(dst1.data[0]));

    // 反相：前景变 0、背景变 255
    node->set_param("invert", Data(true));
    ASSERT_TRUE(node->execute(ctx).is_success());
    Data out3 = node->get_output("image");
    ASSERT_EQ(0, static_cast<int>(out3.as_image().data[0]));
}

TEST(UserNodes, ThresholdRejectsWrongInputType) {
    auto node = NodeFactory::instance().create("nodes.Threshold", "th_2");
    FlowContext ctx;
    node->set_input("image", Data(3.0));   // 数值，不是图

    const auto r = node->execute(ctx);

    ASSERT_TRUE(r.is_failure());
    ASSERT_NE(String::npos, r.message().find("expects Image, got Number"));
    // 消息里必须有节点名：画布上同类节点一多，"输入类型不对"等于没说
    ASSERT_NE(String::npos, r.message().find("th_2"));
}

TEST(UserNodes, ThresholdClampsOutOfRangeParam) {
    auto sink = std::make_shared<CapturingSink>();
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(sink);

    auto node = NodeFactory::instance().create("nodes.Threshold", "th_3");
    FlowContext ctx;
    node->set_input("image", Data(make_image(2, 2, 100)));

    // 999 超出 p_num_in("threshold", 0, 255) 的约束 → 夹到 255 并打一次 WARN。
    // 只夹不报就是**静默改行为**，比崩溃更难查。
    node->set_param("threshold", Data(999.0));
    ASSERT_TRUE(node->execute(ctx).is_success());

    Data out = node->get_output("image");
    ASSERT_EQ(0, static_cast<int>(out.as_image().data[0]));   // 100 < 255
    ASSERT_TRUE(sink->has(LogLevel::Warning, "threshold"));
    ASSERT_TRUE(sink->has(LogLevel::Warning, "clamped to 255"));
}

TEST(UserNodes, ThresholdStopsWhenFlowIsStopped) {
    auto sink = std::make_shared<CapturingSink>();
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(sink);

    auto node = NodeFactory::instance().create("nodes.Threshold", "th_4");
    FlowContext ctx;
    ctx.stop();                                  // 还没开始就被叫停
    node->set_input("image", Data(make_image(4, 4, 100)));
    node->set_param("threshold", Data(50.0));

    // step_ok 在第一行就返回 false → 整个循环体一次都不跑。
    // **不返回 failure**：被叫停是正常操作，不是错误（阶段 2.3 已统一口径）。
    ASSERT_TRUE(node->execute(ctx).is_success());

    // 但要留下痕迹：结果只有算过的行是准的，不能假装算完了
    ASSERT_TRUE(sink->has(LogLevel::Warning, "partial"));
    // 被叫停本身不该刷 WARN —— 那一支 step_ok 不打日志
    ASSERT_EQ(1u, sink->count(LogLevel::Warning));
}

TEST(UserNodes, ThresholdRejectsNonEightBitFormat) {
    auto node = NodeFactory::instance().create("nodes.Threshold", "th_5");
    FlowContext ctx;

    ImageData img = make_image(2, 2, 100);
    img.format = ImageFormat::Mono16;          // 16 位：不能按字节当像素处理
    node->set_input("image", Data(img));

    const auto r = node->execute(ctx);

    ASSERT_TRUE(r.is_failure());
    ASSERT_EQ(ErrorCode::NotSupported, r.code());
    ASSERT_NE(String::npos, r.message().find("8-bit"));
}

// ============================================================================

int main() {
    // 只留内存 sink：控制台 sink 会把每条 WARN 打到 stdout，塞满测试输出。
    Logger::instance().clear_sinks();
    Logger::instance().add_sink(std::make_shared<CapturingSink>());

    TestStats stats = TestRunner::run_all_tests();
    TestRunner::save_report(stats, "test_user_nodes_report.json");
    return stats.failed_tests;
}
