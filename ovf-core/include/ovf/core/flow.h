/**
 * @file flow.h
 * @brief OpenVisionFlow 流程引擎
 */

#pragma once

#include "types.h"
#include "error.h"
#include "node.h"
#include "logger.h"
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <functional>

namespace ovf {

/**
 * @brief 流程执行上下文
 */
class FlowContext {
public:
    FlowContext();
    ~FlowContext();
    
    // 状态控制
    void pause();
    void resume();
    void stop();
    bool is_paused() const { return paused_; }
    bool is_stopped() const { return stopped_; }
    
    // 全局变量
    void set_variable(const String& key, const Data& value);
    Data get_variable(const String& key, const Data& default_val = Data{}) const;
    
    // 流程信息
    uint64_t current_frame() const { return current_frame_; }
    void increment_frame() { ++current_frame_; }
    
    // 回调
    using NodeCallback = std::function<void(const String& node_id, NodeState state)>;
    void set_node_callback(NodeCallback callback) { node_callback_ = callback; }
    
    void notify_node_state(const String& node_id, NodeState state) {
        if (node_callback_) {
            node_callback_(node_id, state);
        }
    }

private:
    std::atomic<bool> paused_{false};
    std::atomic<bool> stopped_{false};
    std::atomic<uint64_t> current_frame_{0};
    HashMap<String, Data> variables_;
    mutable std::mutex mutex_;
    NodeCallback node_callback_;
};

/**
 * @brief 流程定义
 */
struct FlowDef {
    String id;                          // 流程ID
    String name;                        // 流程名称
    String description;                 // 描述
    String version = "1.0";             // 版本
    
    struct NodeInstance {
        String id;                       // 实例ID
        String type_id;                  // 节点类型ID
        String name;                     // 显示名称
        int32_t x = 0;                   // UI位置X
        int32_t y = 0;                   // UI位置Y
        bool enabled = true;             // 是否启用
        ParamSet params;                 // 参数值
        
        // 输入连接
        struct InputConnection {
            String source_node_id;      // 源节点ID
            String source_port;          // 源端口
        };
        HashMap<String, InputConnection> input_connections;
    };
    
    Vector<NodeInstance> nodes;         // 节点列表
    
    // 元数据
    String created_time;
    String modified_time;
    String author;
};

/**
 * @brief 流程执行结果
 */
struct FlowResult {
    bool success = false;
    String error_message;
    String failed_node_id;
    uint64_t total_time_us = 0;
    HashMap<String, uint64_t> node_times;
    
    static FlowResult ok() {
        FlowResult r;
        r.success = true;
        return r;
    }
    
    static FlowResult fail(const String& msg, const String& node_id = "") {
        FlowResult r;
        r.success = false;
        r.error_message = msg;
        r.failed_node_id = node_id;
        return r;
    }
};

/**
 * @brief 流程引擎
 */
class FlowEngine {
public:
    using Ptr = std::shared_ptr<FlowEngine>;
    
    FlowEngine();
    ~FlowEngine();
    
    // 流程管理
    Result<void> load_flow(const FlowDef& flow_def);
    Result<void> load_from_file(const String& filepath);
    Result<void> save_to_file(const String& filepath);
    void clear();
    
    // 节点访问
    INode::Ptr get_node(const String& instance_id) const;
    Vector<INode::Ptr> get_all_nodes() const;
    
    // 执行
    FlowResult run(FlowContext& context);
    FlowResult run_node(const String& node_id, FlowContext& context);
    
    // 单步执行
    void step_begin();
    Result<INode::Ptr> step_next();
    bool step_has_more() const;
    void step_end();
    
    // 执行模式
    enum class ExecutionMode {
        Sequential,     // 顺序执行
        Parallel,        // 并行执行
        DataDriven       // 数据驱动
    };
    
    void set_execution_mode(ExecutionMode mode) { execution_mode_ = mode; }
    ExecutionMode execution_mode() const { return execution_mode_; }
    
    // 回调
    using NodeStateCallback = std::function<void(const String& node_id, NodeState state)>;
    void set_node_state_callback(NodeStateCallback callback) { node_state_callback_ = callback; }
    
    // 拓扑分析
    Result<Vector<String>> get_execution_order() const;
    Result<Vector<String>> get_dependencies(const String& node_id) const;
    Result<Vector<String>> get_dependents(const String& node_id) const;
    
private:
    // 拓扑排序
    Result<Vector<String>> topological_sort() const;
    
    // 执行节点
    FlowResult execute_node(INode::Ptr node, FlowContext& context);
    
    // 并行执行
    FlowResult execute_parallel(FlowContext& context);
    
    // 数据驱动执行
    FlowResult execute_data_driven(FlowContext& context);
    
private:
    FlowDef flow_def_;
    HashMap<String, INode::Ptr> nodes_;
    HashMap<String, Vector<String>> adjacency_list_;  // 邻接表（依赖关系）
    ExecutionMode execution_mode_ = ExecutionMode::Sequential;
    NodeStateCallback node_state_callback_;
    
    // 单步执行状态
    Vector<String> execution_order_;
    size_t current_step_ = 0;
    bool stepping_ = false;
    
    mutable std::mutex mutex_;
};

/**
 * @brief 流程运行器 - 支持连续运行、触发运行
 */
class FlowRunner {
public:
    using Ptr = std::shared_ptr<FlowRunner>;
    
    enum class RunMode {
        Once,           // 单次执行
        Continuous,     // 连续执行
        Triggered       // 触发执行
    };
    
    FlowRunner();
    ~FlowRunner();
    
    // 设置引擎
    void set_engine(FlowEngine::Ptr engine);
    FlowEngine::Ptr engine() const { return engine_; }
    
    // 运行控制
    Result<void> start(RunMode mode);
    void stop();
    void trigger();  // 触发执行
    
    // 状态
    bool is_running() const { return running_; }
    RunMode run_mode() const { return run_mode_; }
    
    // 回调
    using ResultCallback = std::function<void(const FlowResult&)>;
    void set_result_callback(ResultCallback callback) { result_callback_ = callback; }
    
    // 统计
    uint64_t total_runs() const { return total_runs_; }
    uint64_t success_runs() const { return success_runs_; }
    uint64_t failed_runs() const { return failed_runs_; }
    double average_time_ms() const;

private:
    void run_thread();
    
private:
    FlowEngine::Ptr engine_;
    FlowContext context_;
    
    std::atomic<bool> running_{false};
    std::atomic<bool> trigger_{false};
    RunMode run_mode_ = RunMode::Once;
    
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    
    ResultCallback result_callback_;
    
    std::atomic<uint64_t> total_runs_{0};
    std::atomic<uint64_t> success_runs_{0};
    std::atomic<uint64_t> failed_runs_{0};
    std::atomic<uint64_t> total_time_us_{0};
};

// ============================================================================
// 枚举类型输出运算符（用于调试和测试输出）
// ============================================================================

// FlowEngine::ExecutionMode 输出运算符
inline std::ostream& operator<<(std::ostream& os, FlowEngine::ExecutionMode mode) {
    switch (mode) {
        case FlowEngine::ExecutionMode::Sequential: os << "Sequential(0)"; break;
        case FlowEngine::ExecutionMode::Parallel: os << "Parallel(1)"; break;
        case FlowEngine::ExecutionMode::DataDriven: os << "DataDriven(2)"; break;
        default: os << "ExecutionMode(" << static_cast<int>(mode) << ")"; break;
    }
    return os;
}

// FlowRunner::RunMode 输出运算符
inline std::ostream& operator<<(std::ostream& os, FlowRunner::RunMode mode) {
    switch (mode) {
        case FlowRunner::RunMode::Once: os << "Once(0)"; break;
        case FlowRunner::RunMode::Continuous: os << "Continuous(1)"; break;
        case FlowRunner::RunMode::Triggered: os << "Triggered(2)"; break;
        default: os << "RunMode(" << static_cast<int>(mode) << ")"; break;
    }
    return os;
}

} // namespace ovf