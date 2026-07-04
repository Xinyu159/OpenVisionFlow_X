/**
 * @file flow_execution_monitor.cpp
 * @brief 流程执行监控器实现
 */

#include "ovf/flow_execution_monitor.h"
#include "ovf/core/logger.h"
#include <chrono>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace ovf {
namespace web {

using json = nlohmann::json;

// 获取当前时间戳（微秒）
static uint64_t get_timestamp_us() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

// 状态转换为字符串
static String state_to_string(FlowState state) {
    switch (state) {
        case FlowState::Idle: return "idle";
        case FlowState::Running: return "running";
        case FlowState::Paused: return "paused";
        case FlowState::Completed: return "completed";
        case FlowState::Failed: return "failed";
        case FlowState::Cancelled: return "cancelled";
        default: return "unknown";
    }
}

FlowExecutionMonitor::FlowExecutionMonitor()
    : state_(FlowState::Idle)
    , execution_start_time_(0)
    , execution_end_time_(0)
    , total_nodes_(0)
    , completed_nodes_(0)
    , failed_nodes_(0) {
    
    // 初始化统计
    statistics_.total_executions = 0;
    statistics_.successful_executions = 0;
    statistics_.failed_executions = 0;
    statistics_.success_rate = 0.0;
    statistics_.avg_execution_time_ms = 0.0;
    statistics_.min_execution_time_ms = 0.0;
    statistics_.max_execution_time_ms = 0.0;
    statistics_.total_execution_time_us = 0;
    statistics_.total_nodes_executed = 0;
    statistics_.peak_memory_mb = 0;
    statistics_.avg_fps = 0.0;
    
    // 创建默认组件
    log_visualizer_ = std::make_shared<ExecutionLogVisualizer>();
    performance_analyzer_ = std::make_shared<PerformanceAnalyzer>();
}

FlowExecutionMonitor::~FlowExecutionMonitor() {
}

void FlowExecutionMonitor::set_log_visualizer(ExecutionLogVisualizer::Ptr visualizer) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_visualizer_ = visualizer;
}

void FlowExecutionMonitor::set_performance_analyzer(PerformanceAnalyzer::Ptr analyzer) {
    std::lock_guard<std::mutex> lock(mutex_);
    performance_analyzer_ = analyzer;
}

void FlowExecutionMonitor::set_ws_push_callback(WSPushCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    ws_push_callback_ = callback;
    
    if (log_visualizer_) {
        log_visualizer_->set_ws_push_callback(callback);
    }
}

FlowState FlowExecutionMonitor::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void FlowExecutionMonitor::update_state(FlowState new_state) {
    FlowState old_state = state_;
    state_ = new_state;
    
    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }
    
    // 推送状态变化消息
    if (ws_push_callback_) {
        json msg = json::object();
        msg["type"] = "flow_state_changed";
        msg["old_state"] = state_to_string(old_state);
        msg["new_state"] = state_to_string(new_state);
        msg["timestamp"] = get_timestamp_us();
        ws_push_callback_(msg.dump());
    }
    
    OVF_DEBUG() << "Flow state changed: " << state_to_string(old_state) 
                << " -> " << state_to_string(new_state);
}

void FlowExecutionMonitor::start_execution(const String& flow_id, const String& flow_name, int total_nodes) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_flow_id_ = flow_id;
    current_flow_name_ = flow_name;
    total_nodes_ = total_nodes;
    completed_nodes_ = 0;
    failed_nodes_ = 0;
    execution_start_time_ = get_timestamp_us();
    execution_end_time_ = 0;
    
    update_state(FlowState::Running);
    
    if (log_visualizer_) {
        log_visualizer_->begin_session(flow_id, flow_name, total_nodes);
        current_session_id_ = log_visualizer_->current_session().session_id;
    }
    
    OVF_INFO() << "Flow execution started: " << flow_name << " (" << total_nodes << " nodes)";
}

void FlowExecutionMonitor::pause_execution() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ != FlowState::Running) {
        OVF_WARN() << "Cannot pause flow that is not running";
        return;
    }
    
    update_state(FlowState::Paused);
    
    OVF_INFO() << "Flow execution paused";
}

void FlowExecutionMonitor::resume_execution() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ != FlowState::Paused) {
        OVF_WARN() << "Cannot resume flow that is not paused";
        return;
    }
    
    update_state(FlowState::Running);
    
    OVF_INFO() << "Flow execution resumed";
}

void FlowExecutionMonitor::cancel_execution() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ != FlowState::Running && state_ != FlowState::Paused) {
        OVF_WARN() << "Cannot cancel flow that is not running or paused";
        return;
    }
    
    update_state(FlowState::Cancelled);
    
    if (log_visualizer_) {
        log_visualizer_->end_session(false);
    }
    
    OVF_INFO() << "Flow execution cancelled";
}

void FlowExecutionMonitor::complete_execution(bool success) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    execution_end_time_ = get_timestamp_us();
    
    if (success) {
        update_state(FlowState::Completed);
    } else {
        update_state(FlowState::Failed);
    }
    
    if (log_visualizer_) {
        log_visualizer_->end_session(success);
    }
    
    // 更新统计
    statistics_.total_executions++;
    if (success) {
        statistics_.successful_executions++;
    } else {
        statistics_.failed_executions++;
    }
    
    if (statistics_.total_executions > 0) {
        statistics_.success_rate = statistics_.successful_executions * 100.0 / statistics_.total_executions;
    }
    
    uint64_t execution_time = execution_end_time_ - execution_start_time_;
    statistics_.total_execution_time_us += execution_time;
    
    double time_ms = execution_time / 1000.0;
    if (statistics_.avg_execution_time_ms == 0) {
        statistics_.avg_execution_time_ms = time_ms;
        statistics_.min_execution_time_ms = time_ms;
        statistics_.max_execution_time_ms = time_ms;
    } else {
        statistics_.avg_execution_time_ms = 
            (statistics_.avg_execution_time_ms * (statistics_.total_executions - 1) + time_ms) 
            / statistics_.total_executions;
        statistics_.min_execution_time_ms = std::min(statistics_.min_execution_time_ms, time_ms);
        statistics_.max_execution_time_ms = std::max(statistics_.max_execution_time_ms, time_ms);
    }
    
    statistics_.total_nodes_executed += completed_nodes_;
    
    if (performance_analyzer_) {
        statistics_.peak_memory_mb = performance_analyzer_->get_peak_memory() / (1024.0 * 1024.0);
        statistics_.avg_fps = performance_analyzer_->get_current_fps();
    }
    
    // 推送执行完成消息
    push_monitor_data();
    
    OVF_INFO() << "Flow execution completed: " << (success ? "success" : "failed")
               << " (time: " << time_ms << " ms)";
}

void FlowExecutionMonitor::on_node_started(const String& node_id, 
                                           const String& node_type, 
                                           const String& node_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        log_visualizer_->on_node_started(node_id, node_type, node_name);
    }
    
    // 更新统计
    statistics_.executions_by_node_type[node_type]++;
    
    // 推送进度更新
    push_monitor_data();
}

void FlowExecutionMonitor::on_node_completed(const String& node_id,
                                             uint64_t duration_us,
                                             const std::map<String, String>& output_summary) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    completed_nodes_++;
    
    if (log_visualizer_) {
        log_visualizer_->on_node_completed(node_id, duration_us, output_summary);
    }
    
    if (performance_analyzer_) {
        // 从输出摘要推断节点类型和名称
        String node_type = "";
        String node_name = "";
        
        // 如果有日志可视化器，从当前会话获取节点信息
        if (log_visualizer_) {
            const auto& session = log_visualizer_->current_session();
            for (const auto& record : session.node_records) {
                if (record.node_id == node_id) {
                    node_type = record.node_type;
                    node_name = record.node_name;
                    break;
                }
            }
        }
        
        performance_analyzer_->record_node_execution(node_id, node_type, node_name,
                                                    duration_us, true, 0);
    }
    
    // 触发进度回调
    if (progress_callback_) {
        double percent = calculate_progress();
        progress_callback_(percent, node_id);
    }
    
    // 推送进度更新
    push_monitor_data();
}

void FlowExecutionMonitor::on_node_error(const String& node_id,
                                         int error_code,
                                         const String& error_message) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    failed_nodes_++;
    
    if (log_visualizer_) {
        log_visualizer_->on_node_error(node_id, error_code, error_message);
    }
    
    // 记录异常日志
    ExceptionLog ex_log;
    ex_log.timestamp = get_timestamp_us();
    ex_log.node_id = node_id;
    ex_log.error_code = error_code;
    ex_log.error_message = error_message;
    
    // 从日志可视化器获取节点类型
    if (log_visualizer_) {
        const auto& session = log_visualizer_->current_session();
        for (const auto& record : session.node_records) {
            if (record.node_id == node_id) {
                ex_log.node_type = record.node_type;
                
                if (performance_analyzer_) {
                    performance_analyzer_->record_node_execution(node_id, record.node_type,
                                                               record.node_name, 0, false, 0);
                }
                
                statistics_.errors_by_node_type[record.node_type]++;
                break;
            }
        }
    }
    
    add_exception_log(ex_log);
    
    // 触发异常回调
    if (exception_callback_) {
        exception_callback_(ex_log);
    }
    
    // 推送错误消息
    push_monitor_data();
}

void FlowExecutionMonitor::on_node_skipped(const String& node_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        log_visualizer_->on_node_skipped(node_id);
    }
}

void FlowExecutionMonitor::on_frame_processed() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (performance_analyzer_) {
        performance_analyzer_->record_frame_processed();
    }
}

void FlowExecutionMonitor::on_memory_snapshot(size_t memory_bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (performance_analyzer_) {
        performance_analyzer_->record_memory_snapshot(memory_bytes);
    }
    
    // 推送内存更新消息
    if (ws_push_callback_) {
        json msg = json::object();
        msg["type"] = "memory_update";
        msg["memory_mb"] = memory_bytes / (1024.0 * 1024.0);
        msg["timestamp"] = get_timestamp_us();
        ws_push_callback_(msg.dump());
    }
}

RealtimeMonitorData FlowExecutionMonitor::get_realtime_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    RealtimeMonitorData data;
    data.state = state_;
    data.progress_percent = calculate_progress();
    
    if (state_ == FlowState::Running || state_ == FlowState::Paused) {
        data.elapsed_time_ms = (get_timestamp_us() - execution_start_time_) / 1000;
        data.estimated_remaining_ms = estimate_remaining_time();
    } else if (execution_end_time_ > 0) {
        data.elapsed_time_ms = (execution_end_time_ - execution_start_time_) / 1000;
        data.estimated_remaining_ms = 0;
    } else {
        data.elapsed_time_ms = 0;
        data.estimated_remaining_ms = 0;
    }
    
    data.session_id = current_session_id_;
    data.flow_id = current_flow_id_;
    data.flow_name = current_flow_name_;
    
    if (performance_analyzer_) {
        data.current_fps = performance_analyzer_->get_current_fps();
        data.current_memory_mb = performance_analyzer_->get_current_memory() / (1024.0 * 1024.0);
        data.throughput = performance_analyzer_->get_throughput();
    } else {
        data.current_fps = 0;
        data.current_memory_mb = 0;
        data.throughput = 0;
    }
    
    // 最近异常（最多10条）
    int count = std::min(10, static_cast<int>(exception_logs_.size()));
    for (int i = static_cast<int>(exception_logs_.size()) - 1; i >= 0 && count > 0; --i, --count) {
        data.recent_exceptions.push_back(exception_logs_[i]);
    }
    
    return data;
}

ExecutionStatistics FlowExecutionMonitor::get_statistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return statistics_;
}

std::vector<FlowExecutionSession> FlowExecutionMonitor::get_execution_history(int limit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        return log_visualizer_->get_history(limit);
    }
    
    return std::vector<FlowExecutionSession>();
}

bool FlowExecutionMonitor::get_session_detail(const String& session_id, FlowExecutionSession& session) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        return log_visualizer_->get_session(session_id, session);
    }
    
    return false;
}

std::vector<ExceptionLog> FlowExecutionMonitor::get_exception_logs(int limit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<ExceptionLog> result;
    int count = std::min(limit, static_cast<int>(exception_logs_.size()));
    
    for (int i = static_cast<int>(exception_logs_.size()) - 1; i >= 0 && count > 0; --i, --count) {
        result.push_back(exception_logs_[i]);
    }
    
    return result;
}

PerformanceReport FlowExecutionMonitor::get_performance_report(int history_seconds) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (performance_analyzer_) {
        return performance_analyzer_->generate_report(history_seconds);
    }
    
    return PerformanceReport();
}

json FlowExecutionMonitor::get_performance_report_json(int history_seconds) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (performance_analyzer_) {
        return performance_analyzer_->generate_chartjs_report(history_seconds);
    }
    
    return json::object();
}

String FlowExecutionMonitor::export_logs_json(const String& session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        return log_visualizer_->export_to_json(session_id);
    }
    
    return "{}";
}

String FlowExecutionMonitor::export_logs_csv(const String& session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        return log_visualizer_->export_to_csv(session_id);
    }
    
    return "";
}

String FlowExecutionMonitor::export_performance_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (performance_analyzer_) {
        return performance_analyzer_->generate_chartjs_report().dump();
    }
    
    return "{}";
}

void FlowExecutionMonitor::set_state_change_callback(StateChangeCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_change_callback_ = callback;
}

void FlowExecutionMonitor::set_progress_callback(ProgressCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_callback_ = callback;
}

void FlowExecutionMonitor::set_exception_callback(ExceptionCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    exception_callback_ = callback;
}

void FlowExecutionMonitor::clear_history() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (log_visualizer_) {
        log_visualizer_->clear_history();
    }
    
    if (performance_analyzer_) {
        performance_analyzer_->clear_history();
    }
    
    exception_logs_.clear();
    
    OVF_INFO() << "Execution history cleared";
}

void FlowExecutionMonitor::reset_statistics() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    statistics_.total_executions = 0;
    statistics_.successful_executions = 0;
    statistics_.failed_executions = 0;
    statistics_.success_rate = 0.0;
    statistics_.avg_execution_time_ms = 0.0;
    statistics_.min_execution_time_ms = 0.0;
    statistics_.max_execution_time_ms = 0.0;
    statistics_.total_execution_time_us = 0;
    statistics_.total_nodes_executed = 0;
    statistics_.peak_memory_mb = 0;
    statistics_.avg_fps = 0.0;
    statistics_.executions_by_node_type.clear();
    statistics_.avg_time_by_node_type.clear();
    statistics_.errors_by_node_type.clear();
    
    if (performance_analyzer_) {
        performance_analyzer_->reset();
    }
    
    OVF_INFO() << "Statistics reset";
}

void FlowExecutionMonitor::push_monitor_data() {
    if (ws_push_callback_) {
        auto data = get_realtime_data();
        
        json msg = json::object();
        msg["type"] = "monitor_update";
        msg["state"] = state_to_string(data.state);
        msg["current_node"] = data.current_node;
        msg["progress"] = data.progress_percent;
        msg["elapsed_time_ms"] = data.elapsed_time_ms;
        msg["estimated_remaining_ms"] = data.estimated_remaining_ms;
        msg["session_id"] = data.session_id;
        msg["fps"] = data.current_fps;
        msg["memory_mb"] = data.current_memory_mb;
        msg["throughput"] = data.throughput;
        msg["timestamp"] = get_timestamp_us();
        
        ws_push_callback_(msg.dump());
    }
}

double FlowExecutionMonitor::calculate_progress() const {
    if (total_nodes_ == 0) return 0.0;
    return completed_nodes_ * 100.0 / total_nodes_;
}

uint64_t FlowExecutionMonitor::estimate_remaining_time() const {
    if (completed_nodes_ == 0) return 0;
    
    uint64_t elapsed = get_timestamp_us() - execution_start_time_;
    double avg_time_per_node = elapsed / completed_nodes_;
    int remaining_nodes = total_nodes_ - completed_nodes_ - failed_nodes_;
    
    return static_cast<uint64_t>(avg_time_per_node * remaining_nodes);
}

void FlowExecutionMonitor::add_exception_log(const ExceptionLog& log) {
    exception_logs_.push_back(log);
    
    // 限制异常日志数量
    if (exception_logs_.size() > MAX_EXCEPTION_LOGS) {
        exception_logs_.erase(exception_logs_.begin());
    }
}

String FlowExecutionMonitor::format_timestamp(uint64_t timestamp_us) const {
    auto seconds = timestamp_us / 1000000;
    auto ms = (timestamp_us % 1000000) / 1000;
    
    std::ostringstream oss;
    oss << seconds << "." << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

} // namespace web
} // namespace ovf