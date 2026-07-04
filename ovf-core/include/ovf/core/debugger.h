/**
 * @file debugger.h
 * @brief OpenVisionFlow 调试器核心接口
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "types.h"
#include "error.h"
#include "data.h"
#include "logger.h"
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <chrono>

namespace ovf {

// 前向声明
class FlowEngine;
class FlowContext;

/**
 * @brief 断点类型
 */
enum class BreakpointType : uint8_t {
    Normal = 0,        // 普通断点
    Conditional = 1,   // 条件断点
    Temporary = 2,     // 临时断点（命中一次后自动删除）
    HitCount = 3       // 命中次数断点
};

/**
 * @brief 断点状态
 */
enum class BreakpointState : uint8_t {
    Enabled = 0,       // 启用
    Disabled = 1,      // 禁用
    Pending = 2        // 待绑定
};

/**
 * @brief 断点信息
 */
struct Breakpoint {
    String id;                         // 断点唯一ID
    String node_id;                    // 节点实例ID
    BreakpointType type = BreakpointType::Normal;
    BreakpointState state = BreakpointState::Enabled;

    // 条件断点
    String condition;                  // 条件表达式（如 "x > 10"）

    // 命中次数断点
    int32_t hit_count = 0;             // 当前命中次数
    int32_t hit_target = 0;            // 目标命中次数

    // 统计信息
    uint64_t total_hits = 0;           // 总命中次数

    // 时间戳
    uint64_t created_time = 0;         // 创建时间
    uint64_t last_hit_time = 0;        // 最后命中时间

    Breakpoint() = default;
    Breakpoint(const String& bp_id, const String& node)
        : id(bp_id), node_id(node) {
        created_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
    }
};

/**
 * @brief 调试器状态
 */
enum class DebuggerState : uint8_t {
    Idle = 0,          // 空闲
    Running = 1,       // 运行中
    Paused = 2,        // 已暂停（断点或用户暂停）
    Stepping = 3,      // 单步执行中
    Stopped = 4        // 已停止
};

/**
 * @brief 调试上下文
 */
struct DebugContext {
    String current_node_id;            // 当前执行的节点ID
    DebuggerState state = DebuggerState::Idle;
    bool is_paused = false;

    // 断点命中信息
    struct BreakpointHit {
        String breakpoint_id;
        String node_id;
        uint64_t timestamp = 0;
    } breakpoint_hit;

    // 最后错误信息
    struct ErrorInfo {
        String message;
        String node_id;
        uint64_t timestamp = 0;
    } last_error;

    // 调用栈深度
    int32_t call_stack_depth = 0;

    void reset() {
        current_node_id.clear();
        state = DebuggerState::Idle;
        is_paused = false;
        breakpoint_hit = BreakpointHit{};
        last_error = ErrorInfo{};
        call_stack_depth = 0;
    }
};

/**
 * @brief 执行步骤记录
 */
struct ExecutionStep {
    String node_id;                    // 节点ID
    String node_type;                  // 节点类型

    // 输入输出数据（JSON格式）
    String input_data;                 // 输入数据快照
    String output_data;                // 输出数据快照

    // 时间信息
    uint64_t start_time = 0;           // 开始时间戳
    uint64_t end_time = 0;             // 结束时间戳
    uint64_t duration_us = 0;          // 执行耗时（微秒）

    // 状态
    bool success = false;
    String error_message;

    // 调用栈深度
    int32_t stack_depth = 0;
};

/**
 * @brief 执行追踪记录
 */
struct ExecutionTrace {
    String trace_id;                   // 追踪ID
    String flow_id;                     // 流程ID
    uint64_t start_time = 0;            // 开始时间
    uint64_t end_time = 0;              // 结束时间
    uint64_t total_time_us = 0;         // 总耗时

    Vector<ExecutionStep> steps;        // 执行步骤

    // 统计信息
    uint32_t total_nodes = 0;           // 总节点数
    uint32_t success_nodes = 0;         // 成功节点数
    uint32_t failed_nodes = 0;          // 失败节点数
};

/**
 * @brief 变量监视项
 */
struct WatchVariable {
    String name;                        // 变量名
    String display_value;               // 显示值
    DataType type = DataType::None;     // 数据类型
    bool is_valid = false;              // 是否有效
    uint64_t last_update = 0;           // 最后更新时间

    // 子变量（用于对象/数组）
    Vector<WatchVariable> children;
};

/**
 * @brief 断点管理器
 */
class BreakpointManager {
public:
    using Ptr = std::shared_ptr<BreakpointManager>;

    BreakpointManager();
    ~BreakpointManager() = default;

    // 断点管理
    String add_breakpoint(const String& node_id,
                         BreakpointType type = BreakpointType::Normal,
                         const String& condition = "",
                         int32_t hit_target = 0);

    bool remove_breakpoint(const String& breakpoint_id);
    bool remove_breakpoint_by_node(const String& node_id);
    void clear_all_breakpoints();

    // 断点控制
    bool enable_breakpoint(const String& breakpoint_id);
    bool disable_breakpoint(const String& breakpoint_id);
    bool toggle_breakpoint(const String& breakpoint_id);

    // 断点查询
    Breakpoint* get_breakpoint(const String& breakpoint_id);
    const Breakpoint* get_breakpoint(const String& breakpoint_id) const;

    Vector<Breakpoint> get_all_breakpoints() const;
    Vector<Breakpoint> get_breakpoints_by_node(const String& node_id) const;

    bool has_breakpoint(const String& node_id) const;
    size_t breakpoint_count() const;

    // 条件断点评估
    bool evaluate_condition(const Breakpoint& bp, const HashMap<String, Data>& variables);

private:
    String generate_breakpoint_id();

private:
    HashMap<String, Breakpoint> breakpoints_;    // 断点列表
    mutable std::mutex mutex_;
    uint64_t next_breakpoint_id_ = 1;
};

/**
 * @brief 变量查看器
 */
class VariableWatcher {
public:
    using Ptr = std::shared_ptr<VariableWatcher>;

    VariableWatcher();
    ~VariableWatcher() = default;

    // 变量监视
    void watch_variable(const String& name);
    void unwatch_variable(const String& name);
    void clear_watched_variables();
    Vector<String> get_watched_variables() const;

    // 变量访问
    void set_variable_value(const String& name, const Data& value);
    Data get_variable_value(const String& name, const Data& default_val = Data{}) const;
    bool has_variable(const String& name) const;

    // 更新监视变量
    void update_from_context(const HashMap<String, Data>& variables);

    // 获取所有变量
    Vector<WatchVariable> get_all_variables() const;
    Vector<WatchVariable> get_watched_variables_info() const;

    // 清空
    void clear_all_variables();

private:
    Vector<String> watched_variables_;            // 监视的变量列表
    HashMap<String, Data> variables_;             // 变量存储
    mutable std::mutex mutex_;
};

/**
 * @brief 执行追踪器
 */
class ExecutionTracer {
public:
    using Ptr = std::shared_ptr<ExecutionTracer>;

    ExecutionTracer();
    ~ExecutionTracer() = default;

    // 追踪控制
    void start_trace(const String& flow_id);
    void stop_trace();
    void pause_trace();
    void resume_trace();
    bool is_tracing() const { return tracing_; }

    // 记录步骤
    void record_step(const ExecutionStep& step);
    void record_step(const String& node_id,
                     const String& node_type,
                     const String& input_data,
                     const String& output_data,
                     uint64_t duration_us,
                     bool success,
                     const String& error = "");

    // 调用栈管理
    void push_call_stack(const String& node_id);
    void pop_call_stack();
    Vector<String> get_call_stack() const;
    int32_t call_stack_depth() const;

    // 获取追踪信息
    ExecutionTrace get_current_trace() const;
    Vector<ExecutionTrace> get_trace_history() const;

    // 导出
    String export_trace_json() const;
    String export_trace_history_json() const;

    // 清空
    void clear_trace();
    void clear_history();
    void set_max_history_size(size_t max_size);

private:
    ExecutionTrace current_trace_;
    Vector<ExecutionTrace> trace_history_;
    Vector<String> call_stack_;
    size_t max_history_size_ = 100;

    std::atomic<bool> tracing_{false};
    std::atomic<bool> paused_{false};
    mutable std::mutex mutex_;
};

/**
 * @brief 调试器核心类
 */
class Debugger {
public:
    using Ptr = std::shared_ptr<Debugger>;

    // 回调类型
    using StateCallback = std::function<void(DebuggerState state, const DebugContext& context)>;
    using BreakpointCallback = std::function<void(const Breakpoint& bp, const DebugContext& context)>;
    using StepCallback = std::function<void(const ExecutionStep& step)>;

    Debugger();
    ~Debugger();

    // 断点管理
    BreakpointManager& breakpoint_manager() { return *breakpoint_manager_; }
    const BreakpointManager& breakpoint_manager() const { return *breakpoint_manager_; }

    // 变量监视
    VariableWatcher& variable_watcher() { return *variable_watcher_; }
    const VariableWatcher& variable_watcher() const { return *variable_watcher_; }

    // 执行追踪
    ExecutionTracer& tracer() { return *tracer_; }
    const ExecutionTracer& tracer() const { return *tracer_; }

    // 调试上下文
    const DebugContext& context() const { return context_; }

    // 流程绑定
    void attach_to_engine(FlowEngine* engine);
    void detach();
    bool is_attached() const { return attached_engine_ != nullptr; }

    // 执行控制
    Result<void> step_over();
    Result<void> step_into();
    Result<void> step_out();
    Result<void> continue_execution();
    Result<void> pause_execution();
    Result<void> restart();

    // 状态
    DebuggerState state() const { return context_.state; }
    bool is_paused() const { return context_.is_paused; }
    bool is_running() const;

    // 回调设置
    void set_state_callback(StateCallback callback) { state_callback_ = callback; }
    void set_breakpoint_callback(BreakpointCallback callback) { breakpoint_callback_ = callback; }
    void set_step_callback(StepCallback callback) { step_callback_ = callback; }

    // 调试模式
    void set_debug_mode(bool enabled) { debug_mode_ = enabled; }
    bool debug_mode() const { return debug_mode_; }

    // JSON导出
    String export_debug_info_json() const;
    String export_call_stack_json() const;

private:
    // 内部方法
    void on_node_start(const String& node_id);
    void on_node_end(const String& node_id, bool success, const String& error);
    bool should_pause_at_node(const String& node_id);
    void wait_for_continue();
    void update_state(DebuggerState new_state);

    // 引擎事件处理
    void setup_engine_callbacks();

private:
    BreakpointManager::Ptr breakpoint_manager_;
    VariableWatcher::Ptr variable_watcher_;
    ExecutionTracer::Ptr tracer_;

    DebugContext context_;
    FlowEngine* attached_engine_ = nullptr;
    bool debug_mode_ = true;

    // 同步控制
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> should_pause_{false};
    std::atomic<bool> should_stop_{false};

    // 单步控制
    enum class StepMode {
        None,
        StepOver,
        StepInto,
        StepOut,
        Continue
    };
    StepMode step_mode_ = StepMode::None;
    int32_t step_depth_ = 0;

    // 回调
    StateCallback state_callback_;
    BreakpointCallback breakpoint_callback_;
    StepCallback step_callback_;
};

/**
 * @brief 调试器辅助工具函数
 */
namespace debug_utils {

/**
 * @brief 将调试状态转换为字符串
 */
inline const char* debugger_state_to_string(DebuggerState state) {
    switch (state) {
        case DebuggerState::Idle: return "Idle";
        case DebuggerState::Running: return "Running";
        case DebuggerState::Paused: return "Paused";
        case DebuggerState::Stepping: return "Stepping";
        case DebuggerState::Stopped: return "Stopped";
        default: return "Unknown";
    }
}

/**
 * @brief 将断点类型转换为字符串
 */
inline const char* breakpoint_type_to_string(BreakpointType type) {
    switch (type) {
        case BreakpointType::Normal: return "Normal";
        case BreakpointType::Conditional: return "Conditional";
        case BreakpointType::Temporary: return "Temporary";
        case BreakpointType::HitCount: return "HitCount";
        default: return "Unknown";
    }
}

/**
 * @brief 将断点状态转换为字符串
 */
inline const char* breakpoint_state_to_string(BreakpointState state) {
    switch (state) {
        case BreakpointState::Enabled: return "Enabled";
        case BreakpointState::Disabled: return "Disabled";
        case BreakpointState::Pending: return "Pending";
        default: return "Unknown";
    }
}

/**
 * @brief 将Data转换为JSON字符串（用于调试输出）
 */
String data_to_json_string(const Data& data);

/**
 * @brief 从JSON字符串解析Data
 */
Data json_string_to_data(const String& json_str);

} // namespace debug_utils

} // namespace ovf