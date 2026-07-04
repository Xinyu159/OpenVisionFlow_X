/**
 * @file execution_log_visualizer.cpp
 * @brief 执行日志可视化器实现
 */

#include "ovf/execution_log_visualizer.h"
#include "ovf/core/logger.h"
#include <sstream>
#include <iomanip>
#include <chrono>

namespace ovf {
namespace web {

using json = nlohmann::json;

// 获取当前时间戳（微秒）
static uint64_t get_timestamp_us() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

// 节点执行状态转换为字符串
static String state_to_string(NodeExecutionState state) {
    switch (state) {
        case NodeExecutionState::Pending: return "pending";
        case NodeExecutionState::Running: return "running";
        case NodeExecutionState::Success: return "success";
        case NodeExecutionState::Error: return "error";
        case NodeExecutionState::Skipped: return "skipped";
        default: return "unknown";
    }
}

ExecutionLogVisualizer::ExecutionLogVisualizer() {
}

ExecutionLogVisualizer::~ExecutionLogVisualizer() {
}

void ExecutionLogVisualizer::set_ws_push_callback(WSPushCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    ws_push_callback_ = callback;
}

String ExecutionLogVisualizer::generate_session_id() {
    auto timestamp = get_timestamp_us();
    std::ostringstream oss;
    oss << "session_" << std::hex << timestamp;
    return oss.str();
}

void ExecutionLogVisualizer::begin_session(const String& flow_id, const String& flow_name, int total_nodes) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_session_.session_id = generate_session_id();
    current_session_.flow_id = flow_id;
    current_session_.flow_name = flow_name;
    current_session_.start_timestamp = get_timestamp_us();
    current_session_.end_timestamp = 0;
    current_session_.is_running = true;
    current_session_.success = false;
    current_session_.total_nodes = total_nodes;
    current_session_.completed_nodes = 0;
    current_session_.failed_nodes = 0;
    current_session_.node_records.clear();
    current_session_.node_times.clear();
    
    running_nodes_.clear();
    
    // 推送flow_started消息
    json msg = json::object();
    msg["type"] = "flow_started";
    msg["session_id"] = current_session_.session_id;
    msg["flow_id"] = flow_id;
    msg["flow_name"] = flow_name;
    msg["total_nodes"] = total_nodes;
    msg["timestamp"] = current_session_.start_timestamp;
    push_message(WSMessageType::FlowStarted, msg);
    
    OVF_DEBUG() << "Execution session started: " << current_session_.session_id;
}

void ExecutionLogVisualizer::end_session(bool success) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_session_.end_timestamp = get_timestamp_us();
    current_session_.is_running = false;
    current_session_.success = success;
    
    // 计算总执行时间
    uint64_t total_time = current_session_.end_timestamp - current_session_.start_timestamp;
    
    // 推送flow_completed消息
    json msg = json::object();
    msg["type"] = "flow_completed";
    msg["session_id"] = current_session_.session_id;
    msg["success"] = success;
    msg["total_nodes"] = current_session_.total_nodes;
    msg["completed_nodes"] = current_session_.completed_nodes;
    msg["failed_nodes"] = current_session_.failed_nodes;
    msg["total_time_ms"] = static_cast<double>(total_time) / 1000.0;
    msg["timestamp"] = current_session_.end_timestamp;
    push_message(WSMessageType::FlowCompleted, msg);
    
    // 将当前会话添加到历史
    history_.push_back(current_session_);
    
    // 限制历史记录数量
    if (history_.size() > 100) {
        history_.erase(history_.begin());
    }
    
    running_nodes_.clear();
    
    OVF_INFO() << "Execution session ended: " << current_session_.session_id 
               << " (success: " << success << ")";
}

void ExecutionLogVisualizer::on_node_started(const String& node_id, const String& node_type, const String& node_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    NodeExecutionRecord record;
    record.node_id = node_id;
    record.node_type = node_type;
    record.node_name = node_name;
    record.state = NodeExecutionState::Running;
    record.start_timestamp = get_timestamp_us();
    record.end_timestamp = 0;
    record.duration_us = 0;
    record.error_code = 0;
    record.error_message = "";
    record.memory_before_bytes = 0;
    record.memory_after_bytes = 0;
    
    running_nodes_[node_id] = record;
    
    // 推送node_started消息
    json msg = json::object();
    msg["type"] = "node_started";
    msg["node_id"] = node_id;
    msg["node_type"] = node_type;
    msg["node_name"] = node_name;
    msg["timestamp"] = record.start_timestamp;
    push_message(WSMessageType::NodeStarted, msg);
    
    OVF_DEBUG() << "Node started: " << node_id << " (" << node_type << ")";
}

void ExecutionLogVisualizer::on_node_completed(const String& node_id, 
                                               uint64_t duration_us,
                                               const std::map<String, String>& output_summary) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = running_nodes_.find(node_id);
    if (it == running_nodes_.end()) {
        OVF_WARN() << "Node completed but not in running list: " << node_id;
        return;
    }
    
    NodeExecutionRecord& record = it->second;
    record.state = NodeExecutionState::Success;
    record.end_timestamp = get_timestamp_us();
    record.duration_us = duration_us;
    record.output_summary = output_summary;
    
    current_session_.node_records.push_back(record);
    current_session_.node_times[node_id] = duration_us;
    current_session_.completed_nodes++;
    
    running_nodes_.erase(it);
    
    // 推送node_completed消息
    json msg = json::object();
    msg["type"] = "node_completed";
    msg["node_id"] = node_id;
    msg["duration_ms"] = static_cast<double>(duration_us) / 1000.0;
    
    // 输出数据摘要（用于前端可视化）
    json outputs = json::object();
    for (const auto& pair : output_summary) {
        outputs[pair.first] = pair.second;
    }
    msg["outputs"] = outputs;
    msg["timestamp"] = record.end_timestamp;
    
    push_message(WSMessageType::NodeCompleted, msg);
    
    // 更新进度并推送
    update_progress(current_session_.completed_nodes);
    
    OVF_DEBUG() << "Node completed: " << node_id << " (" << duration_us << " us)";
}

void ExecutionLogVisualizer::on_node_error(const String& node_id,
                                          int error_code,
                                          const String& error_message) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = running_nodes_.find(node_id);
    if (it == running_nodes_.end()) {
        OVF_WARN() << "Node error but not in running list: " << node_id;
        return;
    }
    
    NodeExecutionRecord& record = it->second;
    record.state = NodeExecutionState::Error;
    record.end_timestamp = get_timestamp_us();
    record.duration_us = record.end_timestamp - record.start_timestamp;
    record.error_code = error_code;
    record.error_message = error_message;
    
    current_session_.node_records.push_back(record);
    current_session_.failed_nodes++;
    
    running_nodes_.erase(it);
    
    // 推送node_error消息
    json msg = json::object();
    msg["type"] = "node_error";
    msg["node_id"] = node_id;
    msg["error_code"] = error_code;
    msg["message"] = error_message;
    msg["timestamp"] = record.end_timestamp;
    push_message(WSMessageType::NodeError, msg);
    
    OVF_ERROR() << "Node error: " << node_id << " (code: " << error_code << ")";
}

void ExecutionLogVisualizer::on_node_skipped(const String& node_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    NodeExecutionRecord record;
    record.node_id = node_id;
    record.state = NodeExecutionState::Skipped;
    record.start_timestamp = get_timestamp_us();
    record.end_timestamp = record.start_timestamp;
    record.duration_us = 0;
    
    current_session_.node_records.push_back(record);
    
    // 推送node_skipped消息（可选，可以归类到node_completed）
    json msg = json::object();
    msg["type"] = "node_skipped";
    msg["node_id"] = node_id;
    msg["timestamp"] = record.start_timestamp;
    push_message(WSMessageType::NodeCompleted, msg);  // 使用Completed类型
    
    OVF_DEBUG() << "Node skipped: " << node_id;
}

void ExecutionLogVisualizer::update_progress(int completed_nodes) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_session_.completed_nodes = completed_nodes;
    
    double percent = 0.0;
    if (current_session_.total_nodes > 0) {
        percent = static_cast<double>(completed_nodes) / current_session_.total_nodes * 100.0;
    }
    
    // 推送flow_progress消息
    json msg = json::object();
    msg["type"] = "flow_progress";
    msg["percent"] = percent;
    msg["completed_nodes"] = completed_nodes;
    msg["total_nodes"] = current_session_.total_nodes;
    msg["failed_nodes"] = current_session_.failed_nodes;
    msg["timestamp"] = get_timestamp_us();
    
    push_message(WSMessageType::FlowProgress, msg);
}

void ExecutionLogVisualizer::push_message(WSMessageType type, const json& data) {
    if (ws_push_callback_) {
        String message = data.dump();
        ws_push_callback_(message);
    }
}

std::vector<FlowExecutionSession> ExecutionLogVisualizer::get_history(int limit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<FlowExecutionSession> result;
    int count = std::min(limit, static_cast<int>(history_.size()));
    
    // 返回最近的记录（从后往前）
    for (int i = static_cast<int>(history_.size()) - 1; i >= 0 && count > 0; --i, --count) {
        result.push_back(history_[i]);
    }
    
    return result;
}

bool ExecutionLogVisualizer::get_session(const String& session_id, FlowExecutionSession& session) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (const auto& s : history_) {
        if (s.session_id == session_id) {
            session = s;
            return true;
        }
    }
    
    return false;
}

String ExecutionLogVisualizer::export_to_json(const String& session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    json export_data = json::object();
    
    if (session_id.empty()) {
        // 导出所有历史
        json sessions = json::array();
        for (const auto& s : history_) {
            json session_json = json::object();
            session_json["session_id"] = s.session_id;
            session_json["flow_id"] = s.flow_id;
            session_json["flow_name"] = s.flow_name;
            session_json["start_timestamp"] = s.start_timestamp;
            session_json["end_timestamp"] = s.end_timestamp;
            session_json["success"] = s.success;
            session_json["total_nodes"] = s.total_nodes;
            session_json["completed_nodes"] = s.completed_nodes;
            session_json["failed_nodes"] = s.failed_nodes;
            
            // 节点记录
            json nodes = json::array();
            for (const auto& record : s.node_records) {
                json node = json::object();
                node["node_id"] = record.node_id;
                node["node_type"] = record.node_type;
                node["node_name"] = record.node_name;
                node["state"] = state_to_string(record.state);
                node["start_timestamp"] = record.start_timestamp;
                node["end_timestamp"] = record.end_timestamp;
                node["duration_us"] = record.duration_us;
                node["error_code"] = record.error_code;
                node["error_message"] = record.error_message;
                
                json input_summary = json::object();
                for (const auto& pair : record.input_summary) {
                    input_summary[pair.first] = pair.second;
                }
                node["input_summary"] = input_summary;
                
                json output_summary = json::object();
                for (const auto& pair : record.output_summary) {
                    output_summary[pair.first] = pair.second;
                }
                node["output_summary"] = output_summary;
                
                nodes.push_back(node);
            }
            session_json["node_records"] = nodes;
            
            sessions.push_back(session_json);
        }
        export_data["sessions"] = sessions;
        export_data["count"] = static_cast<int>(history_.size());
    } else {
        // 导出指定会话
        FlowExecutionSession target_session;
        if (get_session(session_id, target_session)) {
            export_data["session_id"] = target_session.session_id;
            export_data["flow_id"] = target_session.flow_id;
            export_data["flow_name"] = target_session.flow_name;
            export_data["start_timestamp"] = target_session.start_timestamp;
            export_data["end_timestamp"] = target_session.end_timestamp;
            export_data["success"] = target_session.success;
            export_data["total_nodes"] = target_session.total_nodes;
            export_data["completed_nodes"] = target_session.completed_nodes;
            export_data["failed_nodes"] = target_session.failed_nodes;
            
            json nodes = json::array();
            for (const auto& record : target_session.node_records) {
                json node = json::object();
                node["node_id"] = record.node_id;
                node["node_type"] = record.node_type;
                node["node_name"] = record.node_name;
                node["state"] = state_to_string(record.state);
                node["duration_us"] = record.duration_us;
                node["error_code"] = record.error_code;
                node["error_message"] = record.error_message;
                nodes.push_back(node);
            }
            export_data["node_records"] = nodes;
        }
    }
    
    return export_data.dump();
}

String ExecutionLogVisualizer::export_to_csv(const String& session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ostringstream oss;
    
    // CSV头
    oss << "session_id,flow_id,flow_name,node_id,node_type,node_name,state,start_timestamp,end_timestamp,duration_ms,error_code,error_message\n";
    
    if (session_id.empty()) {
        // 导出所有历史
        for (const auto& s : history_) {
            for (const auto& record : s.node_records) {
                oss << s.session_id << ","
                    << s.flow_id << ","
                    << s.flow_name << ","
                    << record.node_id << ","
                    << record.node_type << ","
                    << record.node_name << ","
                    << state_to_string(record.state) << ","
                    << record.start_timestamp << ","
                    << record.end_timestamp << ","
                    << static_cast<double>(record.duration_us) / 1000.0 << ","
                    << record.error_code << ","
                    << record.error_message << "\n";
            }
        }
    } else {
        // 导出指定会话
        FlowExecutionSession target_session;
        if (get_session(session_id, target_session)) {
            for (const auto& record : target_session.node_records) {
                oss << target_session.session_id << ","
                    << target_session.flow_id << ","
                    << target_session.flow_name << ","
                    << record.node_id << ","
                    << record.node_type << ","
                    << record.node_name << ","
                    << state_to_string(record.state) << ","
                    << record.start_timestamp << ","
                    << record.end_timestamp << ","
                    << static_cast<double>(record.duration_us) / 1000.0 << ","
                    << record.error_code << ","
                    << record.error_message << "\n";
            }
        }
    }
    
    return oss.str();
}

void ExecutionLogVisualizer::clear_history() {
    std::lock_guard<std::mutex> lock(mutex_);
    history_.clear();
    OVF_INFO() << "Execution history cleared";
}

} // namespace web
} // namespace ovf