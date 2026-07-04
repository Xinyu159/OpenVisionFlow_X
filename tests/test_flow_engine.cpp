/**
 * @file test_flow_engine.cpp
 * @brief OpenVisionFlow 流程引擎测试
 *
 * 测试内容：
 * 1. FlowDef流程定义创建/加载
 * 2. FlowEngine流程执行/节点连接
 * 3. FlowContext上下文管理
 * 4. FlowRunner运行控制
 * 5. 节点注册/创建
 */

#include "test_framework.h"
#include "ovf/core/flow.h"
#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/error.h"
#include "ovf/core/logger.h"

using namespace ovf;
using namespace ovf_test;

// ============================================================================
// 测试节点类型定义
// ============================================================================

/**
 * @brief 简单测试节点 - 输入数值加1输出
 */
class AddOneNode : public INode {
public:
    AddOneNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> execute(FlowContext& context) override {
        Data input = get_input("input");
        if (input.is_number()) {
            double value = input.as_number() + 1.0;
            set_output("output", Data(value));
            return Result<void>::success();
        }
        return Result<void>::failure(ErrorCode::InvalidData, "Input is not a number");
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "test.add_one";
        info.name = "Add One";
        info.category = "Test";
        info.description = "Add 1 to input number";
        info.inputs = {DataPort("input", "Input Number", DataType::Number, true)};
        info.outputs = {DataPort("output", "Output Number", DataType::Number, false)};
        return info;
    }
};

/**
 * @brief 简单测试节点 - 输入数值乘以因子
 */
class MultiplyNode : public INode {
public:
    MultiplyNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> execute(FlowContext& context) override {
        Data input = get_input("input");
        double factor = get_param("factor", Data(2.0)).as_number();

        if (input.is_number()) {
            double value = input.as_number() * factor;
            set_output("output", Data(value));
            return Result<void>::success();
        }
        return Result<void>::failure(ErrorCode::InvalidData, "Input is not a number");
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "test.multiply";
        info.name = "Multiply";
        info.category = "Test";
        info.description = "Multiply input by factor";
        info.inputs = {DataPort("input", "Input Number", DataType::Number, true)};
        info.outputs = {DataPort("output", "Output Number", DataType::Number, false)};
        info.params = {ParamDef("factor", "Factor", DataType::Number, Data(2.0))};
        return info;
    }
};

/**
 * @brief 简单测试节点 - 输出固定值
 */
class ConstantNode : public INode {
public:
    ConstantNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> execute(FlowContext& context) override {
        double value = get_param("value", Data(10.0)).as_number();
        set_output("output", Data(value));
        return Result<void>::success();
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "test.constant";
        info.name = "Constant";
        info.category = "Test";
        info.description = "Output a constant value";
        info.inputs = {};
        info.outputs = {DataPort("output", "Output Number", DataType::Number, false)};
        info.params = {ParamDef("value", "Value", DataType::Number, Data(10.0))};
        return info;
    }
};

/**
 * @brief 简单测试节点 - 比较两个数值
 */
class CompareNode : public INode {
public:
    CompareNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> execute(FlowContext& context) override {
        Data input1 = get_input("input1");
        Data input2 = get_input("input2");

        if (input1.is_number() && input2.is_number()) {
            bool result = input1.as_number() > input2.as_number();
            set_output("result", Data(result));
            return Result<void>::success();
        }
        return Result<void>::failure(ErrorCode::InvalidData, "Inputs are not numbers");
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "test.compare";
        info.name = "Compare";
        info.category = "Test";
        info.description = "Compare two numbers (input1 > input2)";
        info.inputs = {
            DataPort("input1", "First Number", DataType::Number, true),
            DataPort("input2", "Second Number", DataType::Number, true)
        };
        info.outputs = {DataPort("result", "Result", DataType::Boolean, false)};
        return info;
    }
};

// 注册测试节点
namespace {
    struct TestNodeRegistrar {
        TestNodeRegistrar() {
            NodeFactory::instance().register_node("test.add_one",
                [](const String& id) -> INode::Ptr { return std::make_shared<AddOneNode>(id); },
                AddOneNode::make_info());
            NodeFactory::instance().register_node("test.multiply",
                [](const String& id) -> INode::Ptr { return std::make_shared<MultiplyNode>(id); },
                MultiplyNode::make_info());
            NodeFactory::instance().register_node("test.constant",
                [](const String& id) -> INode::Ptr { return std::make_shared<ConstantNode>(id); },
                ConstantNode::make_info());
            NodeFactory::instance().register_node("test.compare",
                [](const String& id) -> INode::Ptr { return std::make_shared<CompareNode>(id); },
                CompareNode::make_info());
        }
    } registrar_test_nodes;
}

// ============================================================================
// NodeFactory 测试
// ============================================================================

TEST(FlowEngine, NodeFactory_Register) {
    // 检查节点类型已注册
    ASSERT_TRUE(NodeFactory::instance().has_type("test.add_one"));
    ASSERT_TRUE(NodeFactory::instance().has_type("test.multiply"));
    ASSERT_TRUE(NodeFactory::instance().has_type("test.constant"));
    ASSERT_TRUE(NodeFactory::instance().has_type("test.compare"));
}

TEST(FlowEngine, NodeFactory_Create) {
    INode::Ptr node = NodeFactory::instance().create("test.add_one", "node_1");
    ASSERT_NOT_NULL(node);
    ASSERT_EQ("node_1", node->instance_id());
    ASSERT_EQ("test.add_one", node->info().id);
}

TEST(FlowEngine, NodeFactory_GetInfo) {
    const NodeInfo* info = NodeFactory::instance().get_info("test.add_one");
    ASSERT_NOT_NULL(info);
    ASSERT_EQ("Add One", info->name);
    ASSERT_EQ("Test", info->category);
}

TEST(FlowEngine, NodeFactory_GetAllTypes) {
    Vector<String> types = NodeFactory::instance().get_all_types();
    ASSERT_NOT_EMPTY(types);
    // 应包含测试节点类型
    bool has_add_one = false;
    for (const auto& type : types) {
        if (type == "test.add_one") {
            has_add_one = true;
            break;
        }
    }
    ASSERT_TRUE(has_add_one);
}

// ============================================================================
// INode 测试
// ============================================================================

TEST(FlowEngine, Node_Create) {
    AddOneNode node("test_node_01");
    ASSERT_EQ("test_node_01", node.instance_id());
    ASSERT_EQ(NodeState::Idle, node.state());
    ASSERT_TRUE(node.is_enabled());
}

TEST(FlowEngine, Node_SetParam) {
    MultiplyNode node("test_node_02");
    node.set_param("factor", Data(5.0));

    ASSERT_EQ(5.0, node.get_param("factor").as_number());
}

TEST(FlowEngine, Node_SetInput) {
    AddOneNode node("test_node_03");
    node.set_input("input", Data(10.0));

    ASSERT_TRUE(node.has_input("input"));
    ASSERT_EQ(10.0, node.get_input("input").as_number());
}

TEST(FlowEngine, Node_GetOutput) {
    AddOneNode node("test_node_04");
    node.set_output("output", Data(15.0));

    ASSERT_EQ(15.0, node.get_output("output").as_number());
}

TEST(FlowEngine, Node_EnableDisable) {
    AddOneNode node("test_node_05");
    ASSERT_TRUE(node.is_enabled());

    node.set_enabled(false);
    ASSERT_FALSE(node.is_enabled());

    node.set_enabled(true);
    ASSERT_TRUE(node.is_enabled());
}

TEST(FlowEngine, Node_Connect) {
    auto source = std::make_shared<AddOneNode>("source_node");
    auto target = std::make_shared<AddOneNode>("target_node");

    // 连接输出到输入
    source->connect_output("output", target, "input");

    // 验证连接
    source->set_output("output", Data(10.0));
}

// ============================================================================
// FlowContext 测试
// ============================================================================

TEST(FlowEngine, FlowContext_Create) {
    FlowContext context;
    ASSERT_FALSE(context.is_paused());
    ASSERT_FALSE(context.is_stopped());
    ASSERT_EQ(0u, context.current_frame());
}

TEST(FlowEngine, FlowContext_Variables) {
    FlowContext context;
    context.set_variable("test_var", Data(100.0));

    ASSERT_EQ(100.0, context.get_variable("test_var").as_number());
    ASSERT_EQ(0.0, context.get_variable("nonexistent", Data(0.0)).as_number());
}

TEST(FlowEngine, FlowContext_PauseResume) {
    FlowContext context;
    ASSERT_FALSE(context.is_paused());

    context.pause();
    ASSERT_TRUE(context.is_paused());

    context.resume();
    ASSERT_FALSE(context.is_paused());
}

TEST(FlowEngine, FlowContext_Stop) {
    FlowContext context;
    ASSERT_FALSE(context.is_stopped());

    context.stop();
    ASSERT_TRUE(context.is_stopped());
}

TEST(FlowEngine, FlowContext_FrameCount) {
    FlowContext context;
    ASSERT_EQ(0u, context.current_frame());

    context.increment_frame();
    ASSERT_EQ(1u, context.current_frame());

    context.increment_frame();
    context.increment_frame();
    ASSERT_EQ(3u, context.current_frame());
}

// ============================================================================
// FlowDef 测试
// ============================================================================

TEST(FlowEngine, FlowDef_Create) {
    FlowDef flow;
    flow.id = "test_flow_01";
    flow.name = "Test Flow";
    flow.description = "A simple test flow";
    flow.version = "1.0";

    ASSERT_EQ("test_flow_01", flow.id);
    ASSERT_EQ("Test Flow", flow.name);
    ASSERT_EQ("A simple test flow", flow.description);
    ASSERT_EQ("1.0", flow.version);
}

TEST(FlowEngine, FlowDef_AddNodeInstance) {
    FlowDef flow;
    flow.id = "test_flow_02";

    FlowDef::NodeInstance instance;
    instance.id = "node_01";
    instance.type_id = "test.add_one";
    instance.name = "AddOne Node";
    instance.x = 100;
    instance.y = 50;
    instance.enabled = true;

    flow.nodes.push_back(instance);
    ASSERT_EQ(1u, flow.nodes.size());
    ASSERT_EQ("node_01", flow.nodes[0].id);
}

TEST(FlowEngine, FlowDef_NodeConnection) {
    FlowDef flow;
    flow.id = "test_flow_03";

    // 创建两个节点实例
    FlowDef::NodeInstance source;
    source.id = "source_node";
    source.type_id = "test.constant";
    source.params.set("value", Data(5.0));

    FlowDef::NodeInstance target;
    target.id = "target_node";
    target.type_id = "test.add_one";

    // 设置连接
    FlowDef::NodeInstance::InputConnection conn;
    conn.source_node_id = "source_node";
    conn.source_port = "output";
    target.input_connections["input"] = conn;

    flow.nodes.push_back(source);
    flow.nodes.push_back(target);

    ASSERT_EQ(2u, flow.nodes.size());
    ASSERT_TRUE(flow.nodes[1].input_connections.find("input") != flow.nodes[1].input_connections.end());
}

TEST(FlowEngine, FlowDef_MultipleNodes) {
    FlowDef flow;
    flow.id = "test_flow_04";

    // 创建链式节点
    for (int i = 0; i < 5; ++i) {
        FlowDef::NodeInstance instance;
        instance.id = "node_" + std::to_string(i);
        instance.type_id = "test.add_one";
        flow.nodes.push_back(instance);
    }

    ASSERT_EQ(5u, flow.nodes.size());
}

// ============================================================================
// FlowResult 测试
// ============================================================================

TEST(FlowEngine, FlowResult_Success) {
    FlowResult result = FlowResult::ok();
    ASSERT_TRUE(result.success);
    ASSERT_TRUE(result.error_message.empty());
}

TEST(FlowEngine, FlowResult_Failure) {
    FlowResult result = FlowResult::fail("Test error", "failed_node");
    ASSERT_FALSE(result.success);
    ASSERT_EQ("Test error", result.error_message);
    ASSERT_EQ("failed_node", result.failed_node_id);
}

// ============================================================================
// FlowEngine 测试
// ============================================================================

TEST(FlowEngine, FlowEngine_Create) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    ASSERT_NOT_NULL(engine);
    ASSERT_EQ(FlowEngine::ExecutionMode::Sequential, engine->execution_mode());
}

TEST(FlowEngine, FlowEngine_SetExecutionMode) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    engine->set_execution_mode(FlowEngine::ExecutionMode::Parallel);
    ASSERT_EQ(FlowEngine::ExecutionMode::Parallel, engine->execution_mode());

    engine->set_execution_mode(FlowEngine::ExecutionMode::DataDriven);
    ASSERT_EQ(FlowEngine::ExecutionMode::DataDriven, engine->execution_mode());
}

TEST(FlowEngine, FlowEngine_LoadFlow) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_05";

    FlowDef::NodeInstance instance;
    instance.id = "const_node";
    instance.type_id = "test.constant";
    instance.params.set("value", Data(10.0));
    flow.nodes.push_back(instance);

    Result<void> result = engine->load_flow(flow);
    ASSERT_TRUE(result.is_success());
}

TEST(FlowEngine, FlowEngine_GetNode) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_06";

    FlowDef::NodeInstance instance;
    instance.id = "my_node";
    instance.type_id = "test.add_one";
    flow.nodes.push_back(instance);

    engine->load_flow(flow);

    INode::Ptr node = engine->get_node("my_node");
    ASSERT_NOT_NULL(node);
    ASSERT_EQ("my_node", node->instance_id());
}

TEST(FlowEngine, FlowEngine_GetAllNodes) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_07";

    for (int i = 0; i < 3; ++i) {
        FlowDef::NodeInstance instance;
        instance.id = "node_" + std::to_string(i);
        instance.type_id = "test.add_one";
        flow.nodes.push_back(instance);
    }

    engine->load_flow(flow);

    Vector<INode::Ptr> nodes = engine->get_all_nodes();
    ASSERT_EQ(3u, nodes.size());
}

TEST(FlowEngine, FlowEngine_Clear) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_08";
    FlowDef::NodeInstance instance;
    instance.id = "node_01";
    instance.type_id = "test.add_one";
    flow.nodes.push_back(instance);

    engine->load_flow(flow);
    ASSERT_NOT_NULL(engine->get_node("node_01"));

    engine->clear();
    ASSERT_NULL(engine->get_node("node_01"));
}

// ============================================================================
// FlowEngine 执行测试
// ============================================================================

TEST(FlowEngine, FlowEngine_RunSingleNode) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_09";

    FlowDef::NodeInstance instance;
    instance.id = "const_node";
    instance.type_id = "test.constant";
    instance.params.set("value", Data(42.0));
    flow.nodes.push_back(instance);

    engine->load_flow(flow);

    FlowResult result = engine->run(context);
    ASSERT_TRUE(result.success);

    INode::Ptr node = engine->get_node("const_node");
    ASSERT_EQ(42.0, node->get_output("output").as_number());
}

TEST(FlowEngine, FlowEngine_RunChain) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_10";

    // 常量节点 -> 加1节点 -> 加1节点
    FlowDef::NodeInstance const_inst;
    const_inst.id = "const";
    const_inst.type_id = "test.constant";
    const_inst.params.set("value", Data(10.0));

    FlowDef::NodeInstance add1_inst;
    add1_inst.id = "add1";
    add1_inst.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn1;
    conn1.source_node_id = "const";
    conn1.source_port = "output";
    add1_inst.input_connections["input"] = conn1;

    FlowDef::NodeInstance add2_inst;
    add2_inst.id = "add2";
    add2_inst.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn2;
    conn2.source_node_id = "add1";
    conn2.source_port = "output";
    add2_inst.input_connections["input"] = conn2;

    flow.nodes.push_back(const_inst);
    flow.nodes.push_back(add1_inst);
    flow.nodes.push_back(add2_inst);

    engine->load_flow(flow);
    FlowResult result = engine->run(context);

    ASSERT_TRUE(result.success);

    // 验证结果: 10 + 1 + 1 = 12
    INode::Ptr final_node = engine->get_node("add2");
    ASSERT_EQ(12.0, final_node->get_output("output").as_number());
}

TEST(FlowEngine, FlowEngine_RunMultiply) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_11";

    // 常量节点 -> 乘法节点
    FlowDef::NodeInstance const_inst;
    const_inst.id = "const";
    const_inst.type_id = "test.constant";
    const_inst.params.set("value", Data(5.0));

    FlowDef::NodeInstance mult_inst;
    mult_inst.id = "mult";
    mult_inst.type_id = "test.multiply";
    mult_inst.params.set("factor", Data(3.0));
    FlowDef::NodeInstance::InputConnection conn;
    conn.source_node_id = "const";
    conn.source_port = "output";
    mult_inst.input_connections["input"] = conn;

    flow.nodes.push_back(const_inst);
    flow.nodes.push_back(mult_inst);

    engine->load_flow(flow);
    FlowResult result = engine->run(context);

    ASSERT_TRUE(result.success);

    // 验证结果: 5 * 3 = 15
    INode::Ptr mult_node = engine->get_node("mult");
    ASSERT_EQ(15.0, mult_node->get_output("output").as_number());
}

TEST(FlowEngine, FlowEngine_RunCompare) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_12";

    // 两个常量节点 -> 比较节点
    FlowDef::NodeInstance const1_inst;
    const1_inst.id = "const1";
    const1_inst.type_id = "test.constant";
    const1_inst.params.set("value", Data(20.0));

    FlowDef::NodeInstance const2_inst;
    const2_inst.id = "const2";
    const2_inst.type_id = "test.constant";
    const2_inst.params.set("value", Data(10.0));

    FlowDef::NodeInstance comp_inst;
    comp_inst.id = "compare";
    comp_inst.type_id = "test.compare";
    FlowDef::NodeInstance::InputConnection conn1, conn2;
    conn1.source_node_id = "const1";
    conn1.source_port = "output";
    conn2.source_node_id = "const2";
    conn2.source_port = "output";
    comp_inst.input_connections["input1"] = conn1;
    comp_inst.input_connections["input2"] = conn2;

    flow.nodes.push_back(const1_inst);
    flow.nodes.push_back(const2_inst);
    flow.nodes.push_back(comp_inst);

    engine->load_flow(flow);
    FlowResult result = engine->run(context);

    ASSERT_TRUE(result.success);

    // 验证结果: 20 > 10 = true
    INode::Ptr comp_node = engine->get_node("compare");
    ASSERT_TRUE(comp_node->get_output("result").as_bool());
}

TEST(FlowEngine, FlowEngine_RunNodeById) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_13";

    FlowDef::NodeInstance instance;
    instance.id = "test_node";
    instance.type_id = "test.add_one";
    flow.nodes.push_back(instance);

    engine->load_flow(flow);

    // 设置输入
    INode::Ptr node = engine->get_node("test_node");
    node->set_input("input", Data(5.0));

    FlowResult result = engine->run_node("test_node", context);
    ASSERT_TRUE(result.success);

    ASSERT_EQ(6.0, node->get_output("output").as_number());
}

// ============================================================================
// FlowEngine 单步执行测试
// ============================================================================

TEST(FlowEngine, FlowEngine_StepExecution) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_14";

    for (int i = 0; i < 3; ++i) {
        FlowDef::NodeInstance instance;
        instance.id = "step_" + std::to_string(i);
        instance.type_id = "test.constant";
        instance.params.set("value", Data(i * 10.0));
        flow.nodes.push_back(instance);
    }

    engine->load_flow(flow);
    engine->step_begin();

    int step_count = 0;
    while (engine->step_has_more()) {
        Result<INode::Ptr> node_result = engine->step_next();
        ASSERT_TRUE(node_result.is_success());
        step_count++;
    }

    engine->step_end();
    ASSERT_EQ(3, step_count);
}

// ============================================================================
// FlowRunner 测试
// ============================================================================

TEST(FlowEngine, FlowRunner_Create) {
    FlowRunner::Ptr runner = std::make_shared<FlowRunner>();
    ASSERT_NOT_NULL(runner);
    ASSERT_FALSE(runner->is_running());
    ASSERT_EQ(FlowRunner::RunMode::Once, runner->run_mode());
}

TEST(FlowEngine, FlowRunner_SetEngine) {
    FlowRunner::Ptr runner = std::make_shared<FlowRunner>();
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    runner->set_engine(engine);
    ASSERT_EQ(engine, runner->engine());
}

TEST(FlowEngine, FlowRunner_Stats) {
    FlowRunner::Ptr runner = std::make_shared<FlowRunner>();

    ASSERT_EQ(0u, runner->total_runs());
    ASSERT_EQ(0u, runner->success_runs());
    ASSERT_EQ(0u, runner->failed_runs());
}

// ============================================================================
// NodeState 测试
// ============================================================================

TEST(FlowEngine, NodeState_Values) {
    ASSERT_EQ(0, static_cast<int>(NodeState::Idle));
    ASSERT_EQ(1, static_cast<int>(NodeState::Running));
    ASSERT_EQ(2, static_cast<int>(NodeState::Success));
    ASSERT_EQ(3, static_cast<int>(NodeState::Failed));
    ASSERT_EQ(4, static_cast<int>(NodeState::Disabled));
}

// ============================================================================
// DataType 测试
// ============================================================================

TEST(FlowEngine, DataType_Values) {
    ASSERT_EQ(0, static_cast<int>(DataType::None));
    ASSERT_EQ(1, static_cast<int>(DataType::Image));
    ASSERT_EQ(2, static_cast<int>(DataType::Number));
    ASSERT_EQ(3, static_cast<int>(DataType::String));
    ASSERT_EQ(4, static_cast<int>(DataType::Boolean));
    ASSERT_EQ(7, static_cast<int>(DataType::Point));
    ASSERT_EQ(10, static_cast<int>(DataType::PointCloud));
    ASSERT_EQ(255, static_cast<int>(DataType::Any));
}

// ============================================================================
// 拓扑分析测试
// ============================================================================

TEST(FlowEngine, FlowEngine_GetExecutionOrder) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_15";

    // 创建有依赖的节点
    FlowDef::NodeInstance n1;
    n1.id = "n1";
    n1.type_id = "test.constant";

    FlowDef::NodeInstance n2;
    n2.id = "n2";
    n2.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn;
    conn.source_node_id = "n1";
    conn.source_port = "output";
    n2.input_connections["input"] = conn;

    flow.nodes.push_back(n1);
    flow.nodes.push_back(n2);

    engine->load_flow(flow);

    Result<Vector<String>> order_result = engine->get_execution_order();
    ASSERT_TRUE(order_result.is_success());

    Vector<String> order = order_result.value();
    ASSERT_EQ(2u, order.size());
    // n1 应在 n2 之前
    ASSERT_EQ("n1", order[0]);
    ASSERT_EQ("n2", order[1]);
}

TEST(FlowEngine, FlowEngine_GetDependencies) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_16";

    FlowDef::NodeInstance n1;
    n1.id = "n1";
    n1.type_id = "test.constant";

    FlowDef::NodeInstance n2;
    n2.id = "n2";
    n2.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn;
    conn.source_node_id = "n1";
    conn.source_port = "output";
    n2.input_connections["input"] = conn;

    flow.nodes.push_back(n1);
    flow.nodes.push_back(n2);

    engine->load_flow(flow);

    Result<Vector<String>> deps_result = engine->get_dependencies("n2");
    ASSERT_TRUE(deps_result.is_success());

    Vector<String> deps = deps_result.value();
    ASSERT_EQ(1u, deps.size());
    ASSERT_EQ("n1", deps[0]);
}

TEST(FlowEngine, FlowEngine_GetDependents) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();

    FlowDef flow;
    flow.id = "test_flow_17";

    FlowDef::NodeInstance n1;
    n1.id = "n1";
    n1.type_id = "test.constant";

    FlowDef::NodeInstance n2;
    n2.id = "n2";
    n2.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn;
    conn.source_node_id = "n1";
    conn.source_port = "output";
    n2.input_connections["input"] = conn;

    flow.nodes.push_back(n1);
    flow.nodes.push_back(n2);

    engine->load_flow(flow);

    Result<Vector<String>> dependents_result = engine->get_dependents("n1");
    ASSERT_TRUE(dependents_result.is_success());

    Vector<String> dependents = dependents_result.value();
    ASSERT_EQ(1u, dependents.size());
    ASSERT_EQ("n2", dependents[0]);
}

// ============================================================================
// ImageFormat 测试
// ============================================================================

TEST(FlowEngine, ImageFormat_Values) {
    ASSERT_EQ(0, static_cast<int>(ImageFormat::Unknown));
    ASSERT_EQ(1, static_cast<int>(ImageFormat::Mono8));
    ASSERT_EQ(2, static_cast<int>(ImageFormat::Mono16));
    ASSERT_EQ(3, static_cast<int>(ImageFormat::RGB8));
    ASSERT_EQ(4, static_cast<int>(ImageFormat::RGBA8));
    ASSERT_EQ(5, static_cast<int>(ImageFormat::BGR8));
    ASSERT_EQ(7, static_cast<int>(ImageFormat::Float32));
}

// ============================================================================
// TriggerMode 测试
// ============================================================================

TEST(FlowEngine, TriggerMode_Values) {
    ASSERT_EQ(0, static_cast<int>(TriggerMode::Continuous));
    ASSERT_EQ(1, static_cast<int>(TriggerMode::Software));
    ASSERT_EQ(2, static_cast<int>(TriggerMode::Hardware));
    ASSERT_EQ(3, static_cast<int>(TriggerMode::External));
}

// ============================================================================
// 复杂流程测试
// ============================================================================

TEST(FlowEngine, FlowEngine_ComplexPipeline) {
    FlowEngine::Ptr engine = std::make_shared<FlowEngine>();
    FlowContext context;

    FlowDef flow;
    flow.id = "test_flow_complex";

    // 创建复杂流程: const1 -> add1 -> mult -> add2
    // const2 -> (mult的第二个输入通过参数)
    FlowDef::NodeInstance const_inst;
    const_inst.id = "const";
    const_inst.type_id = "test.constant";
    const_inst.params.set("value", Data(100.0));

    FlowDef::NodeInstance add1_inst;
    add1_inst.id = "add1";
    add1_inst.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn1;
    conn1.source_node_id = "const";
    conn1.source_port = "output";
    add1_inst.input_connections["input"] = conn1;

    FlowDef::NodeInstance mult_inst;
    mult_inst.id = "mult";
    mult_inst.type_id = "test.multiply";
    mult_inst.params.set("factor", Data(2.0));
    FlowDef::NodeInstance::InputConnection conn2;
    conn2.source_node_id = "add1";
    conn2.source_port = "output";
    mult_inst.input_connections["input"] = conn2;

    FlowDef::NodeInstance add2_inst;
    add2_inst.id = "add2";
    add2_inst.type_id = "test.add_one";
    FlowDef::NodeInstance::InputConnection conn3;
    conn3.source_node_id = "mult";
    conn3.source_port = "output";
    add2_inst.input_connections["input"] = conn3;

    flow.nodes.push_back(const_inst);
    flow.nodes.push_back(add1_inst);
    flow.nodes.push_back(mult_inst);
    flow.nodes.push_back(add2_inst);

    engine->load_flow(flow);
    FlowResult result = engine->run(context);

    ASSERT_TRUE(result.success);

    // 验证结果: 100 + 1 = 101, 101 * 2 = 202, 202 + 1 = 203
    INode::Ptr final_node = engine->get_node("add2");
    ASSERT_EQ(203.0, final_node->get_output("output").as_number());
}

// ============================================================================
// 主程序入口
// ============================================================================

int main() {
    // 运行所有测试
    TestStats stats = TestRunner::run_all_tests();

    // 保存测试报告
    TestRunner::save_report(stats, "test_flow_engine_report.json");

    // 返回失败测试数量作为退出码
    return stats.failed_tests;
}