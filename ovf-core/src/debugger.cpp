/**
 * @file debugger.cpp
 * @brief 调试器核心实现
 */

#include "ovf/core/debugger.h"
#include "ovf/core/flow.h"
#include <chrono>
#include <algorithm>
#include <sstream>

// nlohmann_json 头文件
#include "nlohmann/json.hpp"

namespace ovf {

using json = nlohmann::json;

// ============== BreakpointManager ==============

BreakpointManager::BreakpointManager() {}

String BreakpointManager::generate_breakpoint_id() {
    return "bp_" + std::to_string(next_breakpoint_id_++);
}

String BreakpointManager::add_breakpoint(const String& node_id,
                                         BreakpointType type,
                                         const String& condition,
                                         int32_t hit_target) {
    std::lock_guard<std::mutex> lock(mutex_);

    String bp_id = generate_breakpoint_id();
    Breakpoint bp(bp_id, node_id);
    bp.type = type;
    bp.condition = condition;
    bp.hit_target = hit_target;

    breakpoints_[bp_id] = bp;

    OVF_DEBUG() << "Breakpoint added: " << bp_id << " at node " << node_id;
    return bp_id;
}

bool BreakpointManager::remove_breakpoint(const String& breakpoint_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = breakpoints_.find(breakpoint_id);
    if (it == breakpoints_.end()) {
        return false;
    }

    OVF_DEBUG() << "Breakpoint removed: " << breakpoint_id;
    breakpoints_.erase(it);
    return true;
}

bool BreakpointManager::remove_breakpoint_by_node(const String& node_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    bool removed = false;
    for (auto it = breakpoints_.begin(); it != breakpoints_.end(); ) {
        if (it->second.node_id == node_id) {
            OVF_DEBUG() << "Breakpoint removed: " << it->first;
            it = breakpoints_.erase(it);
            removed = true;
        } else {
            ++it;
        }
    }
    return removed;
}

void BreakpointManager::clear_all_breakpoints() {
    std::lock_guard<std::mutex> lock(mutex_);
    breakpoints_.clear();
    OVF_DEBUG() << "All breakpoints cleared";
}

bool BreakpointManager::enable_breakpoint(const String& breakpoint_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = breakpoints_.find(breakpoint_id);
    if (it == breakpoints_.end()) {
        return false;
    }

    it->second.state = BreakpointState::Enabled;
    OVF_DEBUG() << "Breakpoint enabled: " << breakpoint_id;
    return true;
}

bool BreakpointManager::disable_breakpoint(const String& breakpoint_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = breakpoints_.find(breakpoint_id);
    if (it == breakpoints_.end()) {
        return false;
    }

    it->second.state = BreakpointState::Disabled;
    OVF_DEBUG() << "Breakpoint disabled: " << breakpoint_id;
    return true;
}

bool BreakpointManager::toggle_breakpoint(const String& breakpoint_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = breakpoints_.find(breakpoint_id);
    if (it == breakpoints_.end()) {
        return false;
    }

    it->second.state = (it->second.state == BreakpointState::Enabled)
                        ? BreakpointState::Disabled
                        : BreakpointState::Enabled;

    OVF_DEBUG() << "Breakpoint toggled: " << breakpoint_id;
    return true;
}

Breakpoint* BreakpointManager::get_breakpoint(const String& breakpoint_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = breakpoints_.find(breakpoint_id);
    return it != breakpoints_.end() ? &it->second : nullptr;
}

const Breakpoint* BreakpointManager::get_breakpoint(const String& breakpoint_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = breakpoints_.find(breakpoint_id);
    return it != breakpoints_.end() ? &it->second : nullptr;
}

Vector<Breakpoint> BreakpointManager::get_all_breakpoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<Breakpoint> result;
    for (const auto& pair : breakpoints_) {
        result.push_back(pair.second);
    }
    return result;
}

Vector<Breakpoint> BreakpointManager::get_breakpoints_by_node(const String& node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<Breakpoint> result;
    for (const auto& pair : breakpoints_) {
        if (pair.second.node_id == node_id) {
            result.push_back(pair.second);
        }
    }
    return result;
}

bool BreakpointManager::has_breakpoint(const String& node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& pair : breakpoints_) {
        if (pair.second.node_id == node_id &&
            pair.second.state == BreakpointState::Enabled) {
            return true;
        }
    }
    return false;
}

size_t BreakpointManager::breakpoint_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return breakpoints_.size();
}

bool BreakpointManager::evaluate_condition(const Breakpoint& bp,
                                          const HashMap<String, Data>& variables) {
    // 简化实现：条件断点评估（后续可集成脚本引擎）
    if (bp.condition.empty()) {
        return true;
    }

    // 基本条件评估示例
    // 格式: "变量名 运算符 值"
    // 例如: "x > 10", "count == 5"

    std::istringstream iss(bp.condition);
    String var_name, op, value_str;
    iss >> var_name >> op >> value_str;

    auto it = variables.find(var_name);
    if (it == variables.end()) {
        return false;
    }

    const Data& var = it->second;

    // 尝试数值比较
    if (var.is_number()) {
        double var_val = var.as_number();
        double target_val = std::stod(value_str);

        if (op == ">" || op == "gt") return var_val > target_val;
        if (op == "<" || op == "lt") return var_val < target_val;
        if (op == "==" || op == "eq") return var_val == target_val;
        if (op == ">=" || op == "ge") return var_val >= target_val;
        if (op == "<=" || op == "le") return var_val <= target_val;
        if (op == "!=" || op == "ne") return var_val != target_val;
    }

    // 尝试布尔比较
    if (var.is_bool()) {
        bool var_val = var.as_bool();
        bool target_val = (value_str == "true" || value_str == "1");

        if (op == "==" || op == "eq") return var_val == target_val;
        if (op == "!=" || op == "ne") return var_val != target_val;
    }

    return false;
}

// ============== VariableWatcher ==============

VariableWatcher::VariableWatcher() {}

void VariableWatcher::watch_variable(const String& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::find(watched_variables_.begin(), watched_variables_.end(), name);
    if (it == watched_variables_.end()) {
        watched_variables_.push_back(name);
        OVF_DEBUG() << "Variable watched: " << name;
    }
}

void VariableWatcher::unwatch_variable(const String& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::find(watched_variables_.begin(), watched_variables_.end(), name);
    if (it != watched_variables_.end()) {
        watched_variables_.erase(it);
        OVF_DEBUG() << "Variable unwatched: " << name;
    }
}

void VariableWatcher::clear_watched_variables() {
    std::lock_guard<std::mutex> lock(mutex_);
    watched_variables_.clear();
    OVF_DEBUG() << "All watched variables cleared";
}

Vector<String> VariableWatcher::get_watched_variables() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return watched_variables_;
}

void VariableWatcher::set_variable_value(const String& name, const Data& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    variables_[name] = value;
    OVF_DEBUG() << "Variable set: " << name << " = " << debug_utils::data_to_json_string(value);
}

Data VariableWatcher::get_variable_value(const String& name, const Data& default_val) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = variables_.find(name);
    return it != variables_.end() ? it->second : default_val;
}

bool VariableWatcher::has_variable(const String& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return variables_.find(name) != variables_.end();
}

void VariableWatcher::update_from_context(const HashMap<String, Data>& variables) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& pair : variables) {
        variables_[pair.first] = pair.second;
    }

    OVF_DEBUG() << "Variables updated from context: " << variables.size() << " items";
}

Vector<WatchVariable> VariableWatcher::get_all_variables() const {
    std::lock_guard<std::mutex> lock(mutex_);

    Vector<WatchVariable> result;
    uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    for (const auto& pair : variables_) {
        WatchVariable wv;
        wv.name = pair.first;
        wv.display_value = debug_utils::data_to_json_string(pair.second);
        wv.type = pair.second.type();
        wv.is_valid = true;
        wv.last_update = now_ms;
        result.push_back(wv);
    }

    return result;
}

Vector<WatchVariable> VariableWatcher::get_watched_variables_info() const {
    std::lock_guard<std::mutex> lock(mutex_);

    Vector<WatchVariable> result;
    uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    for (const auto& name : watched_variables_) {
        WatchVariable wv;
        wv.name = name;

        auto it = variables_.find(name);
        if (it != variables_.end()) {
            wv.display_value = debug_utils::data_to_json_string(it->second);
            wv.type = it->second.type();
            wv.is_valid = true;
        } else {
            wv.display_value = "<not found>";
            wv.type = DataType::None;
            wv.is_valid = false;
        }

        wv.last_update = now_ms;
        result.push_back(wv);
    }

    return result;
}

void VariableWatcher::clear_all_variables() {
    std::lock_guard<std::mutex> lock(mutex_);
    variables_.clear();
    OVF_DEBUG() << "All variables cleared";
}

// ============== ExecutionTracer ==============

ExecutionTracer::ExecutionTracer() {}

void ExecutionTracer::start_trace(const String& flow_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    current_trace_.trace_id = "trace_" + std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    current_trace_.flow_id = flow_id;
    current_trace_.start_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
    current_trace_.steps.clear();
    current_trace_.total_nodes = 0;
    current_trace_.success_nodes = 0;
    current_trace_.failed_nodes = 0;

    tracing_ = true;
    paused_ = false;

    OVF_DEBUG() << "Trace started: " << current_trace_.trace_id;
}

void ExecutionTracer::stop_trace() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!tracing_) return;

    current_trace_.end_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
    current_trace_.total_time_us = current_trace_.end_time - current_trace_.start_time;

    // 保存到历史
    trace_history_.push_back(current_trace_);

    // 限制历史大小
    if (trace_history_.size() > max_history_size_) {
        trace_history_.erase(trace_history_.begin());
    }

    tracing_ = false;

    OVF_DEBUG() << "Trace stopped: " << current_trace_.trace_id
                << " duration: " << current_trace_.total_time_us << "us";
}

void ExecutionTracer::pause_trace() {
    paused_ = true;
    OVF_DEBUG() << "Trace paused";
}

void ExecutionTracer::resume_trace() {
    paused_ = false;
    OVF_DEBUG() << "Trace resumed";
}

void ExecutionTracer::record_step(const ExecutionStep& step) {
    if (!tracing_ || paused_) return;

    std::lock_guard<std::mutex> lock(mutex_);
    current_trace_.steps.push_back(step);
    current_trace_.total_nodes++;

    if (step.success) {
        current_trace_.success_nodes++;
    } else {
        current_trace_.failed_nodes++;
    }

    OVF_DEBUG() << "Step recorded: node=" << step.node_id
                << " success=" << step.success
                << " duration=" << step.duration_us << "us";
}

void ExecutionTracer::record_step(const String& node_id,
                                  const String& node_type,
                                  const String& input_data,
                                  const String& output_data,
                                  uint64_t duration_us,
                                  bool success,
                                  const String& error) {
    ExecutionStep step;
    step.node_id = node_id;
    step.node_type = node_type;
    step.input_data = input_data;
    step.output_data = output_data;
    step.start_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count() - duration_us;
    step.end_time = step.start_time + duration_us;
    step.duration_us = duration_us;
    step.success = success;
    step.error_message = error;
    step.stack_depth = call_stack_depth();

    record_step(step);
}

void ExecutionTracer::push_call_stack(const String& node_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    call_stack_.push_back(node_id);
    OVF_DEBUG() << "Call stack pushed: " << node_id << " depth=" << call_stack_.size();
}

void ExecutionTracer::pop_call_stack() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!call_stack_.empty()) {
        String node_id = call_stack_.back();
        call_stack_.pop_back();
        OVF_DEBUG() << "Call stack popped: " << node_id << " depth=" << call_stack_.size();
    }
}

Vector<String> ExecutionTracer::get_call_stack() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return call_stack_;
}

int32_t ExecutionTracer::call_stack_depth() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int32_t>(call_stack_.size());
}

ExecutionTrace ExecutionTracer::get_current_trace() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_trace_;
}

Vector<ExecutionTrace> ExecutionTracer::get_trace_history() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return trace_history_;
}

String ExecutionTracer::export_trace_json() const {
    std::lock_guard<std::mutex> lock(mutex_);

    json j;
    j["trace_id"] = current_trace_.trace_id;
    j["flow_id"] = current_trace_.flow_id;
    j["start_time"] = static_cast<uint64_t>(current_trace_.start_time);
    j["end_time"] = static_cast<uint64_t>(current_trace_.end_time);
    j["total_time_us"] = static_cast<uint64_t>(current_trace_.total_time_us);
    j["total_nodes"] = static_cast<uint64_t>(current_trace_.total_nodes);
    j["success_nodes"] = static_cast<uint64_t>(current_trace_.success_nodes);
    j["failed_nodes"] = static_cast<uint64_t>(current_trace_.failed_nodes);

    j["steps"] = json::array();
    for (const auto& step : current_trace_.steps) {
        json step_json;
        step_json["node_id"] = step.node_id;
        step_json["node_type"] = step.node_type;
        step_json["input_data"] = step.input_data;
        step_json["output_data"] = step.output_data;
        step_json["start_time"] = step.start_time;
        step_json["end_time"] = step.end_time;
        step_json["duration_us"] = step.duration_us;
        step_json["success"] = step.success;
        step_json["error_message"] = step.error_message;
        step_json["stack_depth"] = step.stack_depth;
        j["steps"].push_back(step_json);
    }

    return j.dump(2);
}

String ExecutionTracer::export_trace_history_json() const {
    std::lock_guard<std::mutex> lock(mutex_);

    json j = json::array();
    for (const auto& trace : trace_history_) {
        json trace_json;
        trace_json["trace_id"] = trace.trace_id;
        trace_json["flow_id"] = trace.flow_id;
        trace_json["start_time"] = static_cast<uint64_t>(trace.start_time);
        trace_json["end_time"] = static_cast<uint64_t>(trace.end_time);
        trace_json["total_time_us"] = static_cast<uint64_t>(trace.total_time_us);
        trace_json["total_nodes"] = static_cast<uint64_t>(trace.total_nodes);
        trace_json["success_nodes"] = static_cast<uint64_t>(trace.success_nodes);
        trace_json["failed_nodes"] = static_cast<uint64_t>(trace.failed_nodes);

        trace_json["steps"] = json::array();
        for (const auto& step : trace.steps) {
            json step_json;
            step_json["node_id"] = step.node_id;
            step_json["node_type"] = step.node_type;
            step_json["duration_us"] = step.duration_us;
            step_json["success"] = step.success;
            step_json["error_message"] = step.error_message;
            trace_json["steps"].push_back(step_json);
        }

        j.push_back(trace_json);
    }

    return j.dump(2);
}

void ExecutionTracer::clear_trace() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_trace_.steps.clear();
    current_trace_.total_nodes = 0;
    current_trace_.success_nodes = 0;
    current_trace_.failed_nodes = 0;
    OVF_DEBUG() << "Current trace cleared";
}

void ExecutionTracer::clear_history() {
    std::lock_guard<std::mutex> lock(mutex_);
    trace_history_.clear();
    OVF_DEBUG() << "Trace history cleared";
}

void ExecutionTracer::set_max_history_size(size_t max_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_history_size_ = max_size;

    // 如果当前历史超过限制，裁剪
    if (trace_history_.size() > max_size) {
        trace_history_.erase(trace_history_.begin(),
                             trace_history_.begin() + (trace_history_.size() - max_size));
    }
}

// ============== Debugger ==============

Debugger::Debugger()
    : breakpoint_manager_(std::make_shared<BreakpointManager>())
    , variable_watcher_(std::make_shared<VariableWatcher>())
    , tracer_(std::make_shared<ExecutionTracer>()) {
    OVF_DEBUG() << "Debugger created";
}

Debugger::~Debugger() {
    detach();
    OVF_DEBUG() << "Debugger destroyed";
}

void Debugger::attach_to_engine(FlowEngine* engine) {
    if (attached_engine_) {
        detach();
    }

    attached_engine_ = engine;
    if (engine) {
        setup_engine_callbacks();
        OVF_DEBUG() << "Debugger attached to engine";
    }
}

void Debugger::detach() {
    if (attached_engine_) {
        // 清理回调
        attached_engine_->set_node_state_callback(nullptr);
        attached_engine_ = nullptr;

        // 重置状态
        context_.reset();
        update_state(DebuggerState::Idle);

        OVF_DEBUG() << "Debugger detached from engine";
    }
}

void Debugger::setup_engine_callbacks() {
    if (!attached_engine_) return;

    // 设置节点状态回调，以便调试器监控节点执行
    attached_engine_->set_node_state_callback(
        [this](const String& node_id, NodeState state) {
            if (state == NodeState::Running) {
                on_node_start(node_id);
            } else if (state == NodeState::Success || state == NodeState::Failed) {
                String error = (state == NodeState::Failed) ? "Execution failed" : "";
                on_node_end(node_id, state == NodeState::Success, error);
            }
        }
    );
}

void Debugger::on_node_start(const String& node_id) {
    if (!debug_mode_ || !attached_engine_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    context_.current_node_id = node_id;
    tracer_->push_call_stack(node_id);

    // 检查是否应该在当前节点暂停
    if (should_pause_at_node(node_id)) {
        should_pause_ = true;
        wait_for_continue();
    }
}

void Debugger::on_node_end(const String& node_id, bool success, const String& error) {
    if (!debug_mode_ || !attached_engine_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    tracer_->pop_call_stack();

    if (!success) {
        context_.last_error.message = error;
        context_.last_error.node_id = node_id;
        context_.last_error.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        // 执行失败时自动暂停
        should_pause_ = true;
        update_state(DebuggerState::Paused);
        wait_for_continue();
    }
}

bool Debugger::should_pause_at_node(const String& node_id) {
    // 检查是否有启用的断点
    auto breakpoints = breakpoint_manager_->get_breakpoints_by_node(node_id);
    for (const auto& bp : breakpoints) {
        if (bp.state != BreakpointState::Enabled) continue;

        // 检查条件断点
        if (bp.type == BreakpointType::Conditional) {
            // 获取当前变量（从FlowContext）
            // 简化实现：此处应该从FlowContext获取变量
            HashMap<String, Data> variables;
            if (!breakpoint_manager_->evaluate_condition(bp, variables)) {
                continue;
            }
        }

        // 检查命中次数断点
        if (bp.type == BreakpointType::HitCount) {
            Breakpoint* bp_ptr = breakpoint_manager_->get_breakpoint(bp.id);
            if (bp_ptr) {
                bp_ptr->hit_count++;
                bp_ptr->total_hits++;
                bp_ptr->last_hit_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();

                if (bp_ptr->hit_count < bp_ptr->hit_target) {
                    continue;
                }
                // 达到目标次数，重置计数
                bp_ptr->hit_count = 0;
            }
        }

        // 触发断点命中回调
        context_.breakpoint_hit.breakpoint_id = bp.id;
        context_.breakpoint_hit.node_id = node_id;
        context_.breakpoint_hit.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        if (breakpoint_callback_) {
            breakpoint_callback_(bp, context_);
        }

        // 临时断点命中后删除
        if (bp.type == BreakpointType::Temporary) {
            breakpoint_manager_->remove_breakpoint(bp.id);
        }

        update_state(DebuggerState::Paused);
        return true;
    }

    // 检查单步执行模式
    if (step_mode_ == StepMode::StepOver) {
        if (tracer_->call_stack_depth() <= step_depth_) {
            update_state(DebuggerState::Paused);
            return true;
        }
    } else if (step_mode_ == StepMode::StepInto) {
        update_state(DebuggerState::Paused);
        return true;
    } else if (step_mode_ == StepMode::StepOut) {
        if (tracer_->call_stack_depth() < step_depth_) {
            update_state(DebuggerState::Paused);
            return true;
        }
    }

    return should_pause_;
}

void Debugger::wait_for_continue() {
    if (!should_pause_) return;

    context_.is_paused = true;

    // 等待继续信号
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() {
        return !should_pause_ || should_stop_;
    });

    context_.is_paused = false;
}

void Debugger::update_state(DebuggerState new_state) {
    DebuggerState old_state = context_.state;
    context_.state = new_state;

    if (state_callback_ && old_state != new_state) {
        state_callback_(new_state, context_);
    }

    OVF_DEBUG() << "Debugger state changed: "
                << debug_utils::debugger_state_to_string(old_state)
                << " -> "
                << debug_utils::debugger_state_to_string(new_state);
}

Result<void> Debugger::step_over() {
    if (!attached_engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No engine attached");
    }

    if (!context_.is_paused) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Debugger not paused");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    step_mode_ = StepMode::StepOver;
    step_depth_ = tracer_->call_stack_depth();
    should_pause_ = false;
    update_state(DebuggerState::Stepping);

    cv_.notify_one();
    return Result<void>::success();
}

Result<void> Debugger::step_into() {
    if (!attached_engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No engine attached");
    }

    if (!context_.is_paused) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Debugger not paused");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    step_mode_ = StepMode::StepInto;
    should_pause_ = false;
    update_state(DebuggerState::Stepping);

    cv_.notify_one();
    return Result<void>::success();
}

Result<void> Debugger::step_out() {
    if (!attached_engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No engine attached");
    }

    if (!context_.is_paused) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Debugger not paused");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    step_mode_ = StepMode::StepOut;
    step_depth_ = tracer_->call_stack_depth();
    should_pause_ = false;
    update_state(DebuggerState::Stepping);

    cv_.notify_one();
    return Result<void>::success();
}

Result<void> Debugger::continue_execution() {
    if (!attached_engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No engine attached");
    }

    if (!context_.is_paused) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Debugger not paused");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    step_mode_ = StepMode::Continue;
    should_pause_ = false;
    update_state(DebuggerState::Running);

    cv_.notify_one();
    return Result<void>::success();
}

Result<void> Debugger::pause_execution() {
    if (!attached_engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No engine attached");
    }

    if (context_.state == DebuggerState::Idle || context_.state == DebuggerState::Stopped) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Debugger not running");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    should_pause_ = true;
    update_state(DebuggerState::Paused);

    return Result<void>::success();
}

Result<void> Debugger::restart() {
    if (!attached_engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No engine attached");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 重置状态
    context_.reset();
    tracer_->clear_trace();
    step_mode_ = StepMode::None;
    should_pause_ = false;
    should_stop_ = false;

    update_state(DebuggerState::Idle);

    OVF_DEBUG() << "Debugger restarted";
    return Result<void>::success();
}

bool Debugger::is_running() const {
    return context_.state == DebuggerState::Running ||
           context_.state == DebuggerState::Stepping;
}

String Debugger::export_debug_info_json() const {
    json j;

    // 调试上下文
    j["context"]["current_node_id"] = context_.current_node_id;
    j["context"]["state"] = debug_utils::debugger_state_to_string(context_.state);
    j["context"]["is_paused"] = context_.is_paused;
    j["context"]["call_stack_depth"] = context_.call_stack_depth;

    // 断点命中信息
    j["context"]["breakpoint_hit"]["breakpoint_id"] = context_.breakpoint_hit.breakpoint_id;
    j["context"]["breakpoint_hit"]["node_id"] = context_.breakpoint_hit.node_id;
    j["context"]["breakpoint_hit"]["timestamp"] = context_.breakpoint_hit.timestamp;

    // 错误信息
    j["context"]["last_error"]["message"] = context_.last_error.message;
    j["context"]["last_error"]["node_id"] = context_.last_error.node_id;
    j["context"]["last_error"]["timestamp"] = context_.last_error.timestamp;

    // 断点列表
    j["breakpoints"] = json::array();
    auto breakpoints = breakpoint_manager_->get_all_breakpoints();
    for (const auto& bp : breakpoints) {
        json bp_json;
        bp_json["id"] = bp.id;
        bp_json["node_id"] = bp.node_id;
        bp_json["type"] = debug_utils::breakpoint_type_to_string(bp.type);
        bp_json["state"] = debug_utils::breakpoint_state_to_string(bp.state);
        bp_json["condition"] = bp.condition;
        bp_json["hit_count"] = bp.hit_count;
        bp_json["hit_target"] = bp.hit_target;
        bp_json["total_hits"] = bp.total_hits;
        j["breakpoints"].push_back(bp_json);
    }

    // 监视变量
    j["watched_variables"] = json::array();
    auto watched_vars = variable_watcher_->get_watched_variables_info();
    for (const auto& var : watched_vars) {
        json var_json;
        var_json["name"] = var.name;
        var_json["display_value"] = var.display_value;
        var_json["type"] = static_cast<int>(var.type);
        var_json["is_valid"] = var.is_valid;
        j["watched_variables"].push_back(var_json);
    }

    // 调用栈
    j["call_stack"] = json::array();
    auto call_stack = tracer_->get_call_stack();
    for (const auto& node_id : call_stack) {
        j["call_stack"].push_back(node_id);
    }

    return j.dump(2);
}

String Debugger::export_call_stack_json() const {
    json j;
    j["call_stack"] = json::array();

    auto call_stack = tracer_->get_call_stack();
    for (size_t i = 0; i < call_stack.size(); ++i) {
        json frame;
        frame["depth"] = i;
        frame["node_id"] = call_stack[i];
        j["call_stack"].push_back(frame);
    }

    j["depth"] = call_stack.size();
    return j.dump(2);
}

// ============== debug_utils ==============

namespace debug_utils {

String data_to_json_string(const Data& data) {
    json j;

    switch (data.type()) {
        case DataType::None:
            j = nullptr;
            break;
        case DataType::Number:
            j = data.as_number();
            break;
        case DataType::String:
            j = data.as_string();
            break;
        case DataType::Boolean:
            j = data.as_bool();
            break;
        case DataType::Array:
            j = json::array();
            // 简化实现：Data的数组支持待完善
            break;
        case DataType::Object:
            j = json::object();
            // 简化实现：Data的对象支持待完善
            break;
        default:
            j = "<unsupported>";
            break;
    }

    return j.dump();
}

Data json_string_to_data(const String& json_str) {
    try {
        json j = json::parse(json_str);

        if (j.is_null()) {
            return Data();
        } else if (j.is_number()) {
            return Data(static_cast<double>(j));
        } else if (j.is_string()) {
            return Data(static_cast<String>(j));
        } else if (j.is_boolean()) {
            return Data(static_cast<bool>(j));
        }

        return Data();
    } catch (...) {
        return Data();
    }
}

} // namespace debug_utils

} // namespace ovf