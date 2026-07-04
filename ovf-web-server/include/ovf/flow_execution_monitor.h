/**
 * @file flow_execution_monitor.h
 * @brief 流程执行监控 - 实时状态监控、历史记录、统计分析
 */

#pragma once

#include "ovf/core/types.h"
#include "execution_log_visualizer.h"
#include "performance_analyzer.h"
#include "nlohmann/json.hpp"
#include <chrono>
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <functional>

namespace ovf {
namespace web {

/**
 * @brief 流程执行状态枚举
 */
enum class FlowState {
    Idle,           // 空闲
    Running,        // 正在运行
    Paused,         // 已暂停
    Completed,      // 已完成
    Failed,         // 失败
    Cancelled       // 已取消
};

/**
 * @brief 异常日志条目
 */
struct ExceptionLog {
    uint64_t timestamp;                // 时间戳
    String node_id;                    // 节点ID
    String node_type;                  // 节点类型
    int error_code;                    // 错误码
    String error_message;              // 错误信息
    String stack_trace;                // 堆栈跟踪（如果可用）
    nlohmann::json context;            // 上下文信息
};

/**
 * @brief 执行统计摘要
 */
struct ExecutionStatistics {
    int total_executions;              // 总执行次数
    int successful_executions;         // 成功次数
    int failed_executions;             // 失败次数
    double success_rate;               // 成功率（百分比）
    double avg_execution_time_ms;      // 平均执行时间
    double min_execution_time_ms;      // 最小执行时间
    double max_execution_time_ms;      // 最大执行时间
    uint64_t total_execution_time_us;  // 总执行时间
    int total_nodes_executed;          // 总执行节点数
    size_t peak_memory_mb;             // 峰值内存
    double avg_fps;                    // 平均帧率
    
    // 按节点类型统计
    std::map<String, int> executions_by_node_type;
    std::map<String, double> avg_time_by_node_type;
    std::map<String, int> errors_by_node_type;
};

/**
 * @brief 实时监控数据
 */
struct RealtimeMonitorData {
    FlowState state;                   // 当前状态
    String current_node;               // 当前执行的节点
    double progress_percent;           // 进度百分比
    uint64_t elapsed_time_ms;          // 已执行时间
    uint64_t estimated_remaining_ms;   // 预计剩余时间
    
    // 当前会话信息
    String session_id;                 // 会话ID
    String flow_id;                    // 流程ID
    String flow_name;                  // 流程名称
    
    // 实时性能数据
    double current_fps;                // 当前帧率
    size_t current_memory_mb;          // 当前内存使用
    double throughput;                 // 吞吐量
    
    // 最近异常
    std::vector<ExceptionLog> recent_exceptions;
};

/**
 * @brief 流程执行监控器
 * 
 * 集成ExecutionLogVisualizer和PerformanceAnalyzer，
 * 提供完整的流程执行监控功能
 */
class FlowExecutionMonitor {
public:
    using Ptr = std::shared_ptr<FlowExecutionMonitor>;
    using StateChangeCallback = std::function<void(FlowState old_state, FlowState new_state)>;
    using ProgressCallback = std::function<void(double percent, const String& current_node)>;
    using ExceptionCallback = std::function<void(const ExceptionLog& log)>;
    
    FlowExecutionMonitor();
    ~FlowExecutionMonitor();
    
    /**
     * @brief 设置日志可视化器
     */
    void set_log_visualizer(ExecutionLogVisualizer::Ptr visualizer);
    
    /**
     * @brief 设置性能分析器
     */
    void set_performance_analyzer(PerformanceAnalyzer::Ptr analyzer);
    
    /**
     * @brief 设置WebSocket推送回调
     */
    void set_ws_push_callback(WSPushCallback callback);
    
    // ========== 状态管理 ==========
    
    /**
     * @brief 获取当前状态
     */
    FlowState get_state() const;
    
    /**
     * @brief 开始执行流程
     */
    void start_execution(const String& flow_id, const String& flow_name, int total_nodes);
    
    /**
     * @brief 暂停执行
     */
    void pause_execution();
    
    /**
     * @brief 恢复执行
     */
    void resume_execution();
    
    /**
     * @brief 取消执行
     */
    void cancel_execution();
    
    /**
     * @brief 完成执行
     */
    void complete_execution(bool success);
    
    // ========== 节点事件 ==========
    
    /**
     * @brief 节点开始执行
     */
    void on_node_started(const String& node_id, 
                        const String& node_type, 
                        const String& node_name);
    
    /**
     * @brief 节点执行完成
     */
    void on_node_completed(const String& node_id,
                          uint64_t duration_us,
                          const std::map<String, String>& output_summary);
    
    /**
     * @brief 节点执行错误
     */
    void on_node_error(const String& node_id,
                      int error_code,
                      const String& error_message);
    
    /**
     * @brief 节点被跳过
     */
    void on_node_skipped(const String& node_id);
    
    // ========== 帧处理 ==========
    
    /**
     * @brief 记录帧处理完成
     */
    void on_frame_processed();
    
    /**
     * @brief 记录内存快照
     */
    void on_memory_snapshot(size_t memory_bytes);
    
    // ========== 监控数据获取 ==========
    
    /**
     * @brief 获取实时监控数据
     */
    RealtimeMonitorData get_realtime_data() const;
    
    /**
     * @brief 获取执行统计
     */
    ExecutionStatistics get_statistics() const;
    
    /**
     * @brief 获取执行历史
     */
    std::vector<FlowExecutionSession> get_execution_history(int limit = 100) const;
    
    /**
     * @brief 获取指定会话详情
     */
    bool get_session_detail(const String& session_id, FlowExecutionSession& session) const;
    
    /**
     * @brief 获取异常日志
     */
    std::vector<ExceptionLog> get_exception_logs(int limit = 100) const;
    
    /**
     * @brief 获取性能报告
     */
    PerformanceReport get_performance_report(int history_seconds = 60) const;
    
    /**
     * @brief 获取性能报告（JSON格式，Chart.js）
     */
    nlohmann::json get_performance_report_json(int history_seconds = 60) const;
    
    // ========== 导出功能 ==========
    
    /**
     * @brief 导出执行日志为JSON
     */
    String export_logs_json(const String& session_id = "") const;
    
    /**
     * @brief 导出执行日志为CSV
     */
    String export_logs_csv(const String& session_id = "") const;
    
    /**
     * @brief 导出性能报告为JSON
     */
    String export_performance_json() const;
    
    // ========== 回调设置 ==========
    
    /**
     * @brief 设置状态变化回调
     */
    void set_state_change_callback(StateChangeCallback callback);
    
    /**
     * @brief 设置进度回调
     */
    void set_progress_callback(ProgressCallback callback);
    
    /**
     * @brief 设置异常回调
     */
    void set_exception_callback(ExceptionCallback callback);
    
    // ========== 清理 ==========
    
    /**
     * @brief 清除历史记录
     */
    void clear_history();
    
    /**
     * @brief 重置统计
     */
    void reset_statistics();
    
private:
    /**
     * @brief 更新状态
     */
    void update_state(FlowState new_state);
    
    /**
     * @brief 推送监控数据到WebSocket
     */
    void push_monitor_data();
    
    /**
     * @brief 计算进度
     */
    double calculate_progress() const;
    
    /**
     * @brief 预估剩余时间
     */
    uint64_t estimate_remaining_time() const;
    
    /**
     * @brief 添加异常日志
     */
    void add_exception_log(const ExceptionLog& log);
    
    /**
     * @brief 生成Chart.js格式的时间戳标签
     */
    String format_timestamp(uint64_t timestamp_us) const;
    
private:
    mutable std::mutex mutex_;
    
    // 状态
    FlowState state_;
    String current_flow_id_;
    String current_flow_name_;
    String current_session_id_;
    uint64_t execution_start_time_;
    uint64_t execution_end_time_;
    int total_nodes_;
    int completed_nodes_;
    int failed_nodes_;
    
    // 组件
    ExecutionLogVisualizer::Ptr log_visualizer_;
    PerformanceAnalyzer::Ptr performance_analyzer_;
    
    // 统计
    ExecutionStatistics statistics_;
    std::vector<ExceptionLog> exception_logs_;
    
    // 回调
    WSPushCallback ws_push_callback_;
    StateChangeCallback state_change_callback_;
    ProgressCallback progress_callback_;
    ExceptionCallback exception_callback_;
    
    // 配置
    static constexpr int MAX_EXCEPTION_LOGS = 1000;  // 最多保留1000条异常日志
};

} // namespace web
} // namespace ovf