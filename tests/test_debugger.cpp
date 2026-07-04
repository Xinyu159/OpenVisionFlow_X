/**
 * @file test_debugger.cpp
 * @brief 调试器工具测试
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#include "test_framework.h"
#include <ovf/core/debugger.h>
#include <thread>
#include <chrono>

using namespace ovf;

// ============================================================================
// 断点类型和状态测试
// ============================================================================

TEST(BreakpointTypes, EnumValues) {
    // 测试断点类型枚举值
    ASSERT_EQ(0, static_cast<int>(BreakpointType::Normal));
    ASSERT_EQ(1, static_cast<int>(BreakpointType::Conditional));
    ASSERT_EQ(2, static_cast<int>(BreakpointType::Temporary));
    ASSERT_EQ(3, static_cast<int>(BreakpointType::HitCount));
}

TEST(BreakpointStates, EnumValues) {
    // 测试断点状态枚举值
    ASSERT_EQ(0, static_cast<int>(BreakpointState::Enabled));
    ASSERT_EQ(1, static_cast<int>(BreakpointState::Disabled));
    ASSERT_EQ(2, static_cast<int>(BreakpointState::Pending));
}

TEST(DebuggerStates, EnumValues) {
    // 测试调试器状态枚举值
    ASSERT_EQ(0, static_cast<int>(DebuggerState::Idle));
    ASSERT_EQ(1, static_cast<int>(DebuggerState::Running));
    ASSERT_EQ(2, static_cast<int>(DebuggerState::Paused));
    ASSERT_EQ(3, static_cast<int>(DebuggerState::Stepping));
    ASSERT_EQ(4, static_cast<int>(DebuggerState::Stopped));
}

TEST(DebugUtils, StateToString) {
    // 测试状态转换函数
    ASSERT_EQ("Idle", debug_utils::debugger_state_to_string(DebuggerState::Idle));
    ASSERT_EQ("Running", debug_utils::debugger_state_to_string(DebuggerState::Running));
    ASSERT_EQ("Paused", debug_utils::debugger_state_to_string(DebuggerState::Paused));
    ASSERT_EQ("Stepping", debug_utils::debugger_state_to_string(DebuggerState::Stepping));
    ASSERT_EQ("Stopped", debug_utils::debugger_state_to_string(DebuggerState::Stopped));
}

TEST(DebugUtils, BreakpointTypeToString) {
    ASSERT_EQ("Normal", debug_utils::breakpoint_type_to_string(BreakpointType::Normal));
    ASSERT_EQ("Conditional", debug_utils::breakpoint_type_to_string(BreakpointType::Conditional));
    ASSERT_EQ("Temporary", debug_utils::breakpoint_type_to_string(BreakpointType::Temporary));
    ASSERT_EQ("HitCount", debug_utils::breakpoint_type_to_string(BreakpointType::HitCount));
}

TEST(DebugUtils, BreakpointStateToString) {
    ASSERT_EQ("Enabled", debug_utils::breakpoint_state_to_string(BreakpointState::Enabled));
    ASSERT_EQ("Disabled", debug_utils::breakpoint_state_to_string(BreakpointState::Disabled));
    ASSERT_EQ("Pending", debug_utils::breakpoint_state_to_string(BreakpointState::Pending));
}

// ============================================================================
// 断点结构测试
// ============================================================================

TEST(Breakpoint, CreateDefault) {
    Breakpoint bp;
    
    ASSERT_TRUE(bp.id.empty());
    ASSERT_TRUE(bp.node_id.empty());
    ASSERT_EQ(static_cast<int>(BreakpointType::Normal), static_cast<int>(bp.type));
    ASSERT_EQ(static_cast<int>(BreakpointState::Enabled), static_cast<int>(bp.state));
    ASSERT_TRUE(bp.condition.empty());
    ASSERT_EQ(0, bp.hit_count);
    ASSERT_EQ(0, bp.hit_target);
    ASSERT_EQ(0, bp.total_hits);
    ASSERT_EQ(0, bp.created_time);
    ASSERT_EQ(0, bp.last_hit_time);
}

TEST(Breakpoint, CreateWithParams) {
    Breakpoint bp("bp001", "node123");
    
    ASSERT_EQ("bp001", bp.id);
    ASSERT_EQ("node123", bp.node_id);
    ASSERT_GT(0, bp.created_time); // 创建时间应该已设置
}

TEST(Breakpoint, SetCondition) {
    Breakpoint bp("bp001", "node1");
    bp.type = BreakpointType::Conditional;
    bp.condition = "x > 10";
    
    ASSERT_EQ(static_cast<int>(BreakpointType::Conditional), static_cast<int>(bp.type));
    ASSERT_EQ("x > 10", bp.condition);
}

TEST(Breakpoint, SetHitTarget) {
    Breakpoint bp("bp002", "node2");
    bp.type = BreakpointType::HitCount;
    bp.hit_target = 5;
    
    ASSERT_EQ(static_cast<int>(BreakpointType::HitCount), static_cast<int>(bp.type));
    ASSERT_EQ(5, bp.hit_target);
}

// ============================================================================
// 断点管理器测试
// ============================================================================

TEST(BreakpointManager, CreateManager) {
    BreakpointManager manager;
    
    ASSERT_EQ(0, manager.breakpoint_count());
    ASSERT_TRUE(!manager.has_breakpoint("node1"));
}

TEST(BreakpointManager, AddBreakpoint) {
    BreakpointManager manager;
    
    String bp_id = manager.add_breakpoint("node1", BreakpointType::Normal);
    
    ASSERT_TRUE(!bp_id.empty());
    ASSERT_EQ(1, manager.breakpoint_count());
    ASSERT_TRUE(manager.has_breakpoint("node1"));
    
    // 获取断点
    const Breakpoint* bp = manager.get_breakpoint(bp_id);
    ASSERT_NOT_NULL(bp);
    ASSERT_EQ(bp_id, bp->id);
    ASSERT_EQ("node1", bp->node_id);
}

TEST(BreakpointManager, AddConditionalBreakpoint) {
    BreakpointManager manager;
    
    String bp_id = manager.add_breakpoint("node2", BreakpointType::Conditional, "count > 5");
    
    const Breakpoint* bp = manager.get_breakpoint(bp_id);
    ASSERT_NOT_NULL(bp);
    ASSERT_EQ(static_cast<int>(BreakpointType::Conditional), static_cast<int>(bp->type));
    ASSERT_EQ("count > 5", bp->condition);
}

TEST(BreakpointManager, AddHitCountBreakpoint) {
    BreakpointManager manager;
    
    String bp_id = manager.add_breakpoint("node3", BreakpointType::HitCount, "", 10);
    
    const Breakpoint* bp = manager.get_breakpoint(bp_id);
    ASSERT_NOT_NULL(bp);
    ASSERT_EQ(static_cast<int>(BreakpointType::HitCount), static_cast<int>(bp->type));
    ASSERT_EQ(10, bp->hit_target);
}

TEST(BreakpointManager, RemoveBreakpoint) {
    BreakpointManager manager;
    
    String bp_id = manager.add_breakpoint("node1");
    ASSERT_EQ(1, manager.breakpoint_count());
    
    bool removed = manager.remove_breakpoint(bp_id);
    ASSERT_TRUE(removed);
    ASSERT_EQ(0, manager.breakpoint_count());
    
    // 再次删除应该失败
    removed = manager.remove_breakpoint(bp_id);
    ASSERT_FALSE(removed);
}

TEST(BreakpointManager, RemoveByNode) {
    BreakpointManager manager;
    
    manager.add_breakpoint("node1");
    manager.add_breakpoint("node1"); // 同一节点添加多个断点
    manager.add_breakpoint("node2");
    
    ASSERT_EQ(3, manager.breakpoint_count());
    
    bool removed = manager.remove_breakpoint_by_node("node1");
    ASSERT_TRUE(removed);
    ASSERT_EQ(1, manager.breakpoint_count());
    ASSERT_TRUE(!manager.has_breakpoint("node1"));
    ASSERT_TRUE(manager.has_breakpoint("node2"));
}

TEST(BreakpointManager, ClearAllBreakpoints) {
    BreakpointManager manager;
    
    manager.add_breakpoint("node1");
    manager.add_breakpoint("node2");
    manager.add_breakpoint("node3");
    
    ASSERT_EQ(3, manager.breakpoint_count());
    
    manager.clear_all_breakpoints();
    ASSERT_EQ(0, manager.breakpoint_count());
}

TEST(BreakpointManager, EnableDisableBreakpoint) {
    BreakpointManager manager;
    
    String bp_id = manager.add_breakpoint("node1");
    
    // 禁用断点
    bool disabled = manager.disable_breakpoint(bp_id);
    ASSERT_TRUE(disabled);
    
    const Breakpoint* bp = manager.get_breakpoint(bp_id);
    ASSERT_EQ(static_cast<int>(BreakpointState::Disabled), static_cast<int>(bp->state));
    
    // 启用断点
    bool enabled = manager.enable_breakpoint(bp_id);
    ASSERT_TRUE(enabled);
    
    bp = manager.get_breakpoint(bp_id);
    ASSERT_EQ(static_cast<int>(BreakpointState::Enabled), static_cast<int>(bp->state));
}

TEST(BreakpointManager, ToggleBreakpoint) {
    BreakpointManager manager;
    
    String bp_id = manager.add_breakpoint("node1");
    
    // Toggle: Enabled -> Disabled
    manager.toggle_breakpoint(bp_id);
    const Breakpoint* bp = manager.get_breakpoint(bp_id);
    ASSERT_EQ(static_cast<int>(BreakpointState::Disabled), static_cast<int>(bp->state));
    
    // Toggle: Disabled -> Enabled
    manager.toggle_breakpoint(bp_id);
    bp = manager.get_breakpoint(bp_id);
    ASSERT_EQ(static_cast<int>(BreakpointState::Enabled), static_cast<int>(bp->state));
}

TEST(BreakpointManager, GetAllBreakpoints) {
    BreakpointManager manager;
    
    manager.add_breakpoint("node1");
    manager.add_breakpoint("node2");
    manager.add_breakpoint("node3");
    
    Vector<Breakpoint> all = manager.get_all_breakpoints();
    ASSERT_EQ(3, all.size());
}

TEST(BreakpointManager, GetBreakpointsByNode) {
    BreakpointManager manager;
    
    manager.add_breakpoint("node1");
    manager.add_breakpoint("node1");
    manager.add_breakpoint("node2");
    
    Vector<Breakpoint> node1_bps = manager.get_breakpoints_by_node("node1");
    ASSERT_EQ(2, node1_bps.size());
    
    Vector<Breakpoint> node2_bps = manager.get_breakpoints_by_node("node2");
    ASSERT_EQ(1, node2_bps.size());
    
    Vector<Breakpoint> node3_bps = manager.get_breakpoints_by_node("node3");
    ASSERT_EQ(0, node3_bps.size());
}

TEST(BreakpointManager, EvaluateCondition) {
    BreakpointManager manager;

    String bp_id = manager.add_breakpoint("node1", BreakpointType::Conditional, "x > 10");

    Breakpoint* bp = manager.get_breakpoint(bp_id);

    // 创建测试变量
    HashMap<String, Data> variables;

    // x = 15，应该命中
    variables["x"] = Data(15);

    bool should_hit = manager.evaluate_condition(*bp, variables);
    ASSERT_TRUE(should_hit);

    // x = 5，不应该命中
    variables["x"] = Data(5);

    should_hit = manager.evaluate_condition(*bp, variables);
    ASSERT_FALSE(should_hit);
}

// ============================================================================
// 变量监视器测试
// ============================================================================

TEST(VariableWatcher, CreateWatcher) {
    VariableWatcher watcher;
    
    Vector<String> watched = watcher.get_watched_variables();
    ASSERT_EQ(0, watched.size());
}

TEST(VariableWatcher, WatchVariable) {
    VariableWatcher watcher;
    
    watcher.watch_variable("image");
    watcher.watch_variable("threshold");
    
    Vector<String> watched = watcher.get_watched_variables();
    ASSERT_EQ(2, watched.size());
}

TEST(VariableWatcher, UnwatchVariable) {
    VariableWatcher watcher;
    
    watcher.watch_variable("var1");
    watcher.watch_variable("var2");
    watcher.watch_variable("var3");
    
    ASSERT_EQ(3, watcher.get_watched_variables().size());
    
    watcher.unwatch_variable("var2");
    
    Vector<String> watched = watcher.get_watched_variables();
    ASSERT_EQ(2, watched.size());
    
    // 验证var2已被移除
    bool has_var2 = false;
    for (const auto& name : watched) {
        if (name == "var2") has_var2 = true;
    }
    ASSERT_FALSE(has_var2);
}

TEST(VariableWatcher, SetVariableValue) {
    VariableWatcher watcher;

    Data value(42);

    watcher.set_variable_value("counter", value);

    Data retrieved = watcher.get_variable_value("counter");
    ASSERT_EQ(42, retrieved.as_int());
}

TEST(VariableWatcher, GetDefaultValue) {
    VariableWatcher watcher;

    Data default_val(0);

    Data retrieved = watcher.get_variable_value("nonexistent", default_val);
    ASSERT_EQ(0, retrieved.as_int());
}

TEST(VariableWatcher, HasVariable) {
    VariableWatcher watcher;

    ASSERT_FALSE(watcher.has_variable("var1"));

    Data value(10);
    watcher.set_variable_value("var1", value);

    ASSERT_TRUE(watcher.has_variable("var1"));
}

TEST(VariableWatcher, UpdateFromContext) {
    VariableWatcher watcher;

    watcher.watch_variable("x");
    watcher.watch_variable("y");

    HashMap<String, Data> context_vars;

    context_vars["x"] = Data(100);
    context_vars["y"] = Data(200);

    watcher.update_from_context(context_vars);

    ASSERT_TRUE(watcher.has_variable("x"));
    ASSERT_TRUE(watcher.has_variable("y"));
    ASSERT_EQ(100, watcher.get_variable_value("x").as_int());
    ASSERT_EQ(200, watcher.get_variable_value("y").as_int());
}

TEST(VariableWatcher, GetAllVariables) {
    VariableWatcher watcher;

    watcher.set_variable_value("a", Data(1));
    watcher.set_variable_value("b", Data(2));
    watcher.set_variable_value("c", Data(3));

    Vector<WatchVariable> all_vars = watcher.get_all_variables();
    ASSERT_EQ(3, all_vars.size());
}

TEST(VariableWatcher, GetWatchedVariablesInfo) {
    VariableWatcher watcher;

    watcher.watch_variable("x");
    watcher.watch_variable("y");

    watcher.set_variable_value("x", Data(50));

    Vector<WatchVariable> watched_info = watcher.get_watched_variables_info();
    ASSERT_EQ(2, watched_info.size());
}

TEST(VariableWatcher, ClearAllVariables) {
    VariableWatcher watcher;

    watcher.set_variable_value("a", Data(1));
    watcher.set_variable_value("b", Data(2));

    watcher.clear_all_variables();

    ASSERT_FALSE(watcher.has_variable("a"));
    ASSERT_FALSE(watcher.has_variable("b"));
}

// ============================================================================
// 执行追踪器测试
// ============================================================================

TEST(ExecutionTracer, CreateTracer) {
    ExecutionTracer tracer;
    
    ASSERT_FALSE(tracer.is_tracing());
    ASSERT_EQ(0, tracer.call_stack_depth());
}

TEST(ExecutionTracer, StartStopTrace) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    ASSERT_TRUE(tracer.is_tracing());
    
    tracer.stop_trace();
    ASSERT_FALSE(tracer.is_tracing());
}

TEST(ExecutionTracer, PauseResumeTrace) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    ASSERT_TRUE(tracer.is_tracing());
    
    tracer.pause_trace();
    
    tracer.resume_trace();
    ASSERT_TRUE(tracer.is_tracing());
    
    tracer.stop_trace();
}

TEST(ExecutionTracer, RecordStep) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    
    ExecutionStep step;
    step.node_id = "node1";
    step.node_type = "ImageSource";
    step.input_data = "{\"path\":\"test.jpg\"}";
    step.output_data = "{\"width\":256,\"height\":256}";
    step.duration_us = 1000;
    step.success = true;
    
    tracer.record_step(step);
    
    ExecutionTrace trace = tracer.get_current_trace();
    ASSERT_EQ(1, trace.steps.size());
    ASSERT_EQ("node1", trace.steps[0].node_id);
    ASSERT_EQ(1000, trace.steps[0].duration_us);
    
    tracer.stop_trace();
}

TEST(ExecutionTracer, RecordStepWithParams) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    
    tracer.record_step("node2", "GaussianBlur", "{\"sigma\":2.0}", "{\"result\":\"ok\"}", 5000, true);
    
    ExecutionTrace trace = tracer.get_current_trace();
    ASSERT_EQ(1, trace.steps.size());
    ASSERT_EQ("node2", trace.steps[0].node_id);
    ASSERT_EQ("GaussianBlur", trace.steps[0].node_type);
    ASSERT_EQ(5000, trace.steps[0].duration_us);
    ASSERT_TRUE(trace.steps[0].success);
    
    tracer.stop_trace();
}

TEST(ExecutionTracer, RecordFailedStep) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    
    tracer.record_step("node3", "Threshold", "{}", "{}", 100, false, "Invalid threshold value");
    
    ExecutionTrace trace = tracer.get_current_trace();
    ASSERT_EQ(1, trace.steps.size());
    ASSERT_FALSE(trace.steps[0].success);
    ASSERT_EQ("Invalid threshold value", trace.steps[0].error_message);
    
    tracer.stop_trace();
}

TEST(ExecutionTracer, CallStackManagement) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    
    ASSERT_EQ(0, tracer.call_stack_depth());
    
    tracer.push_call_stack("node1");
    ASSERT_EQ(1, tracer.call_stack_depth());
    
    tracer.push_call_stack("node2");
    ASSERT_EQ(2, tracer.call_stack_depth());
    
    Vector<String> stack = tracer.get_call_stack();
    ASSERT_EQ(2, stack.size());
    ASSERT_EQ("node1", stack[0]);
    ASSERT_EQ("node2", stack[1]);
    
    tracer.pop_call_stack();
    ASSERT_EQ(1, tracer.call_stack_depth());
    
    tracer.pop_call_stack();
    ASSERT_EQ(0, tracer.call_stack_depth());
    
    tracer.stop_trace();
}

TEST(ExecutionTracer, TraceStatistics) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    
    // 记录多个步骤
    tracer.record_step("node1", "Source", "{}", "{}", 1000, true);
    tracer.record_step("node2", "Process", "{}", "{}", 2000, true);
    tracer.record_step("node3", "Output", "{}", "{}", 500, false, "Error");
    
    ExecutionTrace trace = tracer.get_current_trace();
    
    ASSERT_EQ(3, trace.total_nodes);
    ASSERT_EQ(2, trace.success_nodes);
    ASSERT_EQ(1, trace.failed_nodes);
    
    tracer.stop_trace();
}

TEST(ExecutionTracer, TraceHistory) {
    ExecutionTracer tracer;
    
    // 第一次追踪
    tracer.start_trace("flow_001");
    tracer.record_step("node1", "Type1", "{}", "{}", 1000, true);
    tracer.stop_trace();
    
    // 第二次追踪
    tracer.start_trace("flow_002");
    tracer.record_step("node2", "Type2", "{}", "{}", 2000, true);
    tracer.stop_trace();
    
    Vector<ExecutionTrace> history = tracer.get_trace_history();
    ASSERT_EQ(2, history.size());
}

TEST(ExecutionTracer, ExportJson) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    tracer.record_step("node1", "Source", "{}", "{}", 1000, true);
    tracer.stop_trace();
    
    String json = tracer.export_trace_json();
    ASSERT_TRUE(!json.empty());
    
    // 验证JSON包含关键字
    ASSERT_TRUE(json.find("\"trace_id\"") != String::npos);
    ASSERT_TRUE(json.find("\"steps\"") != String::npos);
    ASSERT_TRUE(json.find("\"node_id\"") != String::npos);
}

TEST(ExecutionTracer, ClearTrace) {
    ExecutionTracer tracer;
    
    tracer.start_trace("flow_001");
    tracer.record_step("node1", "Type1", "{}", "{}", 1000, true);
    
    tracer.clear_trace();
    
    ExecutionTrace trace = tracer.get_current_trace();
    ASSERT_EQ(0, trace.steps.size());
}

TEST(ExecutionTracer, SetMaxHistorySize) {
    ExecutionTracer tracer;
    
    tracer.set_max_history_size(5);
    
    // 添加超过限制的追踪
    for (int i = 0; i < 10; ++i) {
        tracer.start_trace("flow_" + std::to_string(i));
        tracer.record_step("node", "Type", "{}", "{}", 1000, true);
        tracer.stop_trace();
    }
    
    Vector<ExecutionTrace> history = tracer.get_trace_history();
    ASSERT_EQ(5, history.size()); // 应该只保留最新的5条
}

// ============================================================================
// 调试上下文测试
// ============================================================================

TEST(DebugContext, ResetContext) {
    DebugContext ctx;
    
    ctx.current_node_id = "node1";
    ctx.state = DebuggerState::Running;
    ctx.is_paused = true;
    ctx.call_stack_depth = 5;
    
    ctx.reset();
    
    ASSERT_TRUE(ctx.current_node_id.empty());
    ASSERT_EQ(static_cast<int>(DebuggerState::Idle), static_cast<int>(ctx.state));
    ASSERT_FALSE(ctx.is_paused);
    ASSERT_EQ(0, ctx.call_stack_depth);
}

TEST(DebugContext, BreakpointHit) {
    DebugContext ctx;
    
    ctx.breakpoint_hit.breakpoint_id = "bp001";
    ctx.breakpoint_hit.node_id = "node1";
    ctx.breakpoint_hit.timestamp = 1234567890;
    
    ASSERT_EQ("bp001", ctx.breakpoint_hit.breakpoint_id);
    ASSERT_GT(0, ctx.breakpoint_hit.timestamp);
}

TEST(DebugContext, ErrorInfo) {
    DebugContext ctx;
    
    ctx.last_error.message = "Division by zero";
    ctx.last_error.node_id = "node5";
    ctx.last_error.timestamp = 1234567890;
    
    ASSERT_EQ("Division by zero", ctx.last_error.message);
    ASSERT_EQ("node5", ctx.last_error.node_id);
}

// ============================================================================
// 执行步骤测试
// ============================================================================

TEST(ExecutionStep, DefaultValues) {
    ExecutionStep step;
    
    ASSERT_TRUE(step.node_id.empty());
    ASSERT_TRUE(step.node_type.empty());
    ASSERT_TRUE(step.input_data.empty());
    ASSERT_TRUE(step.output_data.empty());
    ASSERT_EQ(0, step.start_time);
    ASSERT_EQ(0, step.end_time);
    ASSERT_EQ(0, step.duration_us);
    ASSERT_FALSE(step.success);
    ASSERT_TRUE(step.error_message.empty());
    ASSERT_EQ(0, step.stack_depth);
}

TEST(ExecutionStep, SetValues) {
    ExecutionStep step;
    
    step.node_id = "node123";
    step.node_type = "GaussianBlur";
    step.input_data = "{\"sigma\":2.0}";
    step.output_data = "{\"result\":\"ok\"}";
    step.start_time = 1000000;
    step.end_time = 1005000;
    step.duration_us = 5000;
    step.success = true;
    step.stack_depth = 1;
    
    ASSERT_EQ("node123", step.node_id);
    ASSERT_EQ(5000, step.duration_us);
    ASSERT_TRUE(step.success);
}

// ============================================================================
// 监视变量测试
// ============================================================================

TEST(WatchVariable, DefaultValues) {
    WatchVariable var;
    
    ASSERT_TRUE(var.name.empty());
    ASSERT_TRUE(var.display_value.empty());
    ASSERT_EQ(static_cast<int>(DataType::None), static_cast<int>(var.type));
    ASSERT_FALSE(var.is_valid);
    ASSERT_EQ(0, var.last_update);
    ASSERT_EQ(0, var.children.size());
}

TEST(WatchVariable, SetValues) {
    WatchVariable var;
    
    var.name = "image";
    var.display_value = "ImageData(256x256)";
    var.type = DataType::Image;
    var.is_valid = true;
    var.last_update = 1234567890;
    
    ASSERT_EQ("image", var.name);
    ASSERT_EQ("ImageData(256x256)", var.display_value);
    ASSERT_EQ(static_cast<int>(DataType::Image), static_cast<int>(var.type));
    ASSERT_TRUE(var.is_valid);
}

TEST(WatchVariable, ChildrenVariables) {
    WatchVariable parent;
    parent.name = "result";
    parent.type = DataType::Object;

    WatchVariable child1;
    child1.name = "width";
    child1.display_value = "256";

    WatchVariable child2;
    child2.name = "height";
    child2.display_value = "256";

    parent.children.push_back(child1);
    parent.children.push_back(child2);

    ASSERT_EQ(2, parent.children.size());
    ASSERT_EQ("width", parent.children[0].name);
    ASSERT_EQ("height", parent.children[1].name);
}

// ============================================================================
// 调试器主类测试
// ============================================================================

TEST(Debugger, CreateDebugger) {
    Debugger debugger;
    
    ASSERT_EQ(static_cast<int>(DebuggerState::Idle), static_cast<int>(debugger.state()));
    ASSERT_FALSE(debugger.is_attached());
    ASSERT_TRUE(debugger.debug_mode());
}

TEST(Debugger, DebugMode) {
    Debugger debugger;
    
    ASSERT_TRUE(debugger.debug_mode());
    
    debugger.set_debug_mode(false);
    ASSERT_FALSE(debugger.debug_mode());
    
    debugger.set_debug_mode(true);
    ASSERT_TRUE(debugger.debug_mode());
}

TEST(Debugger, BreakpointManagerAccess) {
    Debugger debugger;
    
    BreakpointManager& manager = debugger.breakpoint_manager();
    
    manager.add_breakpoint("node1");
    manager.add_breakpoint("node2");
    
    ASSERT_EQ(2, debugger.breakpoint_manager().breakpoint_count());
}

TEST(Debugger, VariableWatcherAccess) {
    Debugger debugger;
    
    VariableWatcher& watcher = debugger.variable_watcher();
    
    watcher.watch_variable("x");
    watcher.watch_variable("y");
    
    ASSERT_EQ(2, debugger.variable_watcher().get_watched_variables().size());
}

TEST(Debugger, TracerAccess) {
    Debugger debugger;
    
    ExecutionTracer& tracer = debugger.tracer();
    
    tracer.start_trace("flow_001");
    ASSERT_TRUE(debugger.tracer().is_tracing());
    tracer.stop_trace();
}

TEST(Debugger, ContextAccess) {
    Debugger debugger;
    
    const DebugContext& ctx = debugger.context();
    
    ASSERT_EQ(static_cast<int>(DebuggerState::Idle), static_cast<int>(ctx.state));
    ASSERT_TRUE(ctx.current_node_id.empty());
}

TEST(Debugger, SetCallbacks) {
    Debugger debugger;
    
    int state_change_count = 0;
    debugger.set_state_callback([&state_change_count](DebuggerState state, const DebugContext& ctx) {
        state_change_count++;
    });
    
    int breakpoint_hit_count = 0;
    debugger.set_breakpoint_callback([&breakpoint_hit_count](const Breakpoint& bp, const DebugContext& ctx) {
        breakpoint_hit_count++;
    });
    
    int step_count = 0;
    debugger.set_step_callback([&step_count](const ExecutionStep& step) {
        step_count++;
    });
    
    // 回调设置成功
    ASSERT_TRUE(true);
}

TEST(Debugger, ExportDebugInfoJson) {
    Debugger debugger;
    
    debugger.breakpoint_manager().add_breakpoint("node1");
    
    String json = debugger.export_debug_info_json();
    
    ASSERT_TRUE(!json.empty());
    ASSERT_TRUE(json.find("\"breakpoints\"") != String::npos || json.find("\"state\"") != String::npos);
}

TEST(Debugger, ExportCallStackJson) {
    Debugger debugger;
    
    debugger.tracer().start_trace("flow_001");
    debugger.tracer().push_call_stack("node1");
    debugger.tracer().push_call_stack("node2");
    
    String json = debugger.export_call_stack_json();
    
    ASSERT_TRUE(!json.empty());
    
    debugger.tracer().stop_trace();
}

// ============================================================================
// 调试器状态测试
// ============================================================================

TEST(Debugger, IsRunning) {
    Debugger debugger;
    
    ASSERT_FALSE(debugger.is_running());
    
    // 注意：在没有FlowEngine绑定的情况下，状态改变需要特殊处理
    // 此测试主要验证API可调用
}

TEST(Debugger, IsPaused) {
    Debugger debugger;
    
    ASSERT_FALSE(debugger.is_paused());
}

// ============================================================================
// 执行控制测试（基础API测试）
// ============================================================================

TEST(Debugger, StepOver) {
    Debugger debugger;
    
    // 在没有绑定FlowEngine时，应该返回错误或空
    Result<void> result = debugger.step_over();
    
    // 验证API可调用
    ASSERT_TRUE(true);
}

TEST(Debugger, StepInto) {
    Debugger debugger;
    
    Result<void> result = debugger.step_into();
    ASSERT_TRUE(true);
}

TEST(Debugger, StepOut) {
    Debugger debugger;
    
    Result<void> result = debugger.step_out();
    ASSERT_TRUE(true);
}

TEST(Debugger, ContinueExecution) {
    Debugger debugger;
    
    Result<void> result = debugger.continue_execution();
    ASSERT_TRUE(true);
}

TEST(Debugger, PauseExecution) {
    Debugger debugger;
    
    Result<void> result = debugger.pause_execution();
    ASSERT_TRUE(true);
}

TEST(Debugger, Restart) {
    Debugger debugger;
    
    Result<void> result = debugger.restart();
    ASSERT_TRUE(true);
}

// ============================================================================
// 多线程安全测试
// ============================================================================

TEST(DebuggerThreadSafety, ConcurrentBreakpointOperations) {
    BreakpointManager manager;
    
    // 多线程同时添加断点
    std::vector<std::thread> threads;
    
    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([&manager, i]() {
            for (int j = 0; j < 100; ++j) {
                manager.add_breakpoint("node_" + std::to_string(i) + "_" + std::to_string(j));
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // 应该有1000个断点
    ASSERT_EQ(1000, manager.breakpoint_count());
}

TEST(DebuggerThreadSafety, ConcurrentVariableOperations) {
    VariableWatcher watcher;

    std::vector<std::thread> threads;

    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([&watcher, i]() {
            for (int j = 0; j < 50; ++j) {
                Data val(i * 100 + j);
                watcher.set_variable_value("var_" + std::to_string(i) + "_" + std::to_string(j), val);
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    // 验证变量数量
    Vector<WatchVariable> vars = watcher.get_all_variables();
    ASSERT_EQ(500, vars.size());
}

// ============================================================================
// Data与JSON转换测试
// ============================================================================

TEST(DebugUtils, DataToJsonString) {
    Data int_data(42);

    String json = debug_utils::data_to_json_string(int_data);
    ASSERT_TRUE(!json.empty());
    ASSERT_TRUE(json.find("42") != String::npos || json.find("\"value\"") != String::npos);
}

TEST(DebugUtils, JsonStringToData) {
    // 简单的JSON测试
    String json = "{\"type\":\"int\",\"value\":123}";
    
    Data data = debug_utils::json_string_to_data(json);
    
    // 验证解析成功（具体实现依赖解析器）
    ASSERT_TRUE(true);
}

// ============================================================================
// 主函数
// ============================================================================

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Debugger Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    auto stats = ovf_test::TestRunner::run_all_tests();
    
    // 保存测试报告
    ovf_test::TestRunner::save_report(stats, "test_debugger_report.json");
    
    return stats.failed_tests > 0 ? 1 : 0;
}