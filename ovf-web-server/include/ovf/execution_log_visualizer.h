/**
 * @file execution_log_visualizer.h
 * @brief 执行日志可视化器 - 实时推送节点执行状态
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/node.h"
#include "nlohmann/json.hpp"
#include <chrono>
#include <map>
#include <memory>
#include <functional>
#include <mutex>

namespace ovf {
namespace web {

/**
 * @brief 节点执行状态枚举
 */
enum class NodeExecutionState {
    Pending,    // 等待执行
    Running,    // 正在执行
    Success,    // 执行成功
    Error,      // 执行错误
    Skipped     // 跳过执行
};

/**
 * @brief 节点执行记录
 */
struct NodeExecutionRecord {
    String node_id;                     // 节点实例ID
    String node_type;                   // 节点类型
    String node_name;                   // 节点名称
    NodeExecutionState state;          // 执行状态
    uint64_t start_timestamp;           // 开始时间戳（微秒）
    uint64_t end_timestamp;             // 结束时间戳（微秒）
    uint64_t duration_us;               // 执行耗时（微秒）
    int error_code;                      // 错误码
    String error_message;               // 错误信息
    std::map<String, String> input_summary;   // 输入数据摘要
    std::map<String, String> output_summary;  // 输出数据摘要
    size_t memory_before_bytes;         // 执行前内存
    size_t memory_after_bytes;          // 执行后内存
};

/**
 * @brief 流程执行会话
 */
struct FlowExecutionSession {
    String session_id;                  // 会话ID
    String flow_id;                     // 流程ID
    String flow_name;                   // 流程名称
    uint64_t start_timestamp;           // 开始时间戳
    uint64_t end_timestamp;             // 结束时间戳（0表示未结束）
    bool is_running;                    // 是否正在运行
    bool success;                       // 是否成功完成
    int total_nodes;                    // 总节点数
    int completed_nodes;                // 已完成节点数
    int failed_nodes;                   // 失败节点数
    Vector<NodeExecutionRecord> node_records;  // 节点执行记录
    std::map<String, uint64_t> node_times;  // 节点耗时统计
};

/**
 * @brief WebSocket消息类型
 */
enum class WSMessageType {
    NodeStarted,        // 节点开始执行
    NodeCompleted,      // 节点执行完成
    NodeError,          // 节点执行错误
    FlowProgress,        // 流程进度更新
    PerformanceReport,  // 性能报告
    FlowStarted,        // 流程开始执行
    FlowCompleted,      // 流程执行完成
    MemoryUpdate        // 内存使用更新
};

/**
 * @brief WebSocket消息推送回调
 */
using WSPushCallback = std::function<void(const String& message)>;

/**
 * @brief 执行日志可视化器
 * 
 * 负责记录节点执行过程，并通过WebSocket实时推送执行状态
 */
class ExecutionLogVisualizer {
public:
    using Ptr = std::shared_ptr<ExecutionLogVisualizer>;
    
    ExecutionLogVisualizer();
    ~ExecutionLogVisualizer();
    
    /**
     * @brief 设置WebSocket推送回调
     */
    void set_ws_push_callback(WSPushCallback callback);
    
    /**
     * @brief 开始新的执行会话
     */
    void begin_session(const String& flow_id, const String& flow_name, int total_nodes);
    
    /**
     * @brief 结束当前执行会话
     */
    void end_session(bool success);
    
    /**
     * @brief 记录节点开始执行
     */
    void on_node_started(const String& node_id, const String& node_type, const String& node_name);
    
    /**
     * @brief 记录节点执行完成
     */
    void on_node_completed(const String& node_id, 
                          uint64_t duration_us,
                          const std::map<String, String>& output_summary);
    
    /**
     * @brief 记录节点执行错误
     */
    void on_node_error(const String& node_id,
                      int error_code,
                      const String& error_message);
    
    /**
     * @brief 记录节点被跳过
     */
    void on_node_skipped(const String& node_id);
    
    /**
     * @brief 更新进度
     */
    void update_progress(int completed_nodes);
    
    /**
     * @brief 获取当前会话
     */
    const FlowExecutionSession& current_session() const { return current_session_; }
    
    /**
     * @brief 获取执行历史
     */
    Vector<FlowExecutionSession> get_history(int limit = 100) const;
    
    /**
     * @brief 获取指定会话详情
     */
    bool get_session(const String& session_id, FlowExecutionSession& session) const;
    
    /**
     * @brief 导出执行日志为JSON格式
     */
    String export_to_json(const String& session_id = "") const;
    
    /**
     * @brief 导出执行日志为CSV格式
     */
    String export_to_csv(const String& session_id = "") const;
    
    /**
     * @brief 清除历史记录
     */
    void clear_history();
    
private:
    /**
     * @brief 推送WebSocket消息
     */
    void push_message(WSMessageType type, const nlohmann::json& data);
    
    /**
     * @brief 生成节点执行摘要
     */
    std::map<String, String> summarize_data(const std::map<String, Data>& data);
    
    /**
     * @brief 生成唯一会话ID
     */
    String generate_session_id();
    
private:
    WSPushCallback ws_push_callback_;
    FlowExecutionSession current_session_;
    Vector<FlowExecutionSession> history_;
    mutable std::mutex mutex_;
    std::map<String, NodeExecutionRecord> running_nodes_;  // 正在执行的节点
};

} // namespace web
} // namespace ovf