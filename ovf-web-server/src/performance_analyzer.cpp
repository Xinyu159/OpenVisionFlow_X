/**
 * @file performance_analyzer.cpp
 * @brief 性能分析器实现
 */

#include "ovf/performance_analyzer.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace ovf {
namespace web {

using json = nlohmann::json;

PerformanceAnalyzer::PerformanceAnalyzer()
    : total_frames_(0)
    , start_timestamp_(0)
    , total_nodes_executed_(0)
    , current_memory_(0)
    , peak_memory_(0) {
    start_timestamp_ = get_timestamp_us();
}

PerformanceAnalyzer::~PerformanceAnalyzer() {
}

uint64_t PerformanceAnalyzer::get_timestamp_us() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

void PerformanceAnalyzer::record_node_execution(const String& node_id,
                                                const String& node_type,
                                                const String& node_name,
                                                uint64_t duration_us,
                                                bool success,
                                                size_t memory_delta_bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新节点统计
    auto& stats = node_stats_[node_id];
    stats.node_id = node_id;
    stats.node_type = node_type;
    stats.node_name = node_name;
    stats.total_duration_us += duration_us;
    
    if (stats.execution_count == 0) {
        stats.min_duration_us = duration_us;
        stats.max_duration_us = duration_us;
        stats.avg_duration_us = duration_us;
        stats.execution_count = 1;
        stats.error_count = success ? 0 : 1;
    } else {
        stats.min_duration_us = std::min(stats.min_duration_us, duration_us);
        stats.max_duration_us = std::max(stats.max_duration_us, duration_us);
        stats.execution_count++;
        if (!success) stats.error_count++;
        stats.avg_duration_us = stats.total_duration_us / stats.execution_count;
    }
    
    stats.success_rate = (stats.execution_count - stats.error_count) * 100.0 / stats.execution_count;
    
    // 内存统计
    size_t total_memory = memory_delta_bytes;
    if (stats.avg_memory_bytes > 0) {
        stats.avg_memory_bytes = (stats.avg_memory_bytes + memory_delta_bytes) / 2;
    } else {
        stats.avg_memory_bytes = memory_delta_bytes;
    }
    stats.peak_memory_bytes = std::max(stats.peak_memory_bytes, memory_delta_bytes);
    
    // 更新吞吐量历史
    total_nodes_executed_++;
    auto now = get_timestamp_us();
    node_execution_history_.push_back({now, total_nodes_executed_});
    
    // 清理过期历史
    while (!node_execution_history_.empty() &&
           node_execution_history_.front().first < now - MAX_HISTORY_SECONDS * 1000000) {
        node_execution_history_.pop_front();
    }
    
    // 更新执行时间历史
    execution_time_history_.push_back({now, static_cast<double>(duration_us) / 1000.0});
    while (!execution_time_history_.empty() &&
           execution_time_history_.front().first < now - MAX_HISTORY_SECONDS * 1000000) {
        execution_time_history_.pop_front();
    }
    
    OVF_DEBUG() << "Node execution recorded: " << node_id << " (" << duration_us << " us)";
}

void PerformanceAnalyzer::record_frame_processed() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    total_frames_++;
    auto now = get_timestamp_us();
    frame_history_.push_back({now, total_frames_});
    
    // 清理过期历史（只保留最近的数据）
    while (!frame_history_.empty() &&
           frame_history_.front().first < now - MAX_HISTORY_SECONDS * 1000000) {
        frame_history_.pop_front();
    }
    
    update_fps();
}

void PerformanceAnalyzer::record_memory_snapshot(size_t memory_bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_memory_ = memory_bytes;
    peak_memory_ = std::max(peak_memory_, memory_bytes);
    
    auto now = get_timestamp_us();
    memory_history_.push_back({now, memory_bytes});
    memory_timeline_.push_back({now, memory_bytes});
    
    // 清理过期历史
    while (!memory_history_.empty() &&
           memory_history_.front().first < now - MAX_HISTORY_SECONDS * 1000000) {
        memory_history_.pop_front();
    }
    while (!memory_timeline_.empty() &&
           memory_timeline_.front().first < now - MAX_HISTORY_SECONDS * 1000000) {
        memory_timeline_.pop_front();
    }
}

void PerformanceAnalyzer::update_fps() {
    // FPS计算基于最近FPS_WINDOW_SECONDS的数据
}

NodePerformanceStats PerformanceAnalyzer::get_node_stats(const String& node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = node_stats_.find(node_id);
    if (it != node_stats_.end()) {
        return it->second;
    }
    
    return NodePerformanceStats();
}

std::vector<NodePerformanceStats> PerformanceAnalyzer::get_all_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<NodePerformanceStats> result;
    for (const auto& pair : node_stats_) {
        result.push_back(pair.second);
    }
    return result;
}

std::vector<NodePerformanceStats> PerformanceAnalyzer::get_slowest_nodes(int top_n) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<NodePerformanceStats> all_stats;
    for (const auto& pair : node_stats_) {
        all_stats.push_back(pair.second);
    }
    
    // 按平均耗时排序（降序）
    std::sort(all_stats.begin(), all_stats.end(),
              [](const NodePerformanceStats& a, const NodePerformanceStats& b) {
                  return a.avg_duration_us > b.avg_duration_us;
              });
    
    // 返回前N个
    if (top_n > 0 && all_stats.size() > top_n) {
        all_stats.resize(top_n);
    }
    
    return all_stats;
}

std::vector<PerformanceBottleneck> PerformanceAnalyzer::identify_bottlenecks(double threshold_percent) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<PerformanceBottleneck> bottlenecks;
    
    // 计算总执行时间
    uint64_t total_time = 0;
    for (const auto& pair : node_stats_) {
        total_time += pair.second.total_duration_us;
    }
    
    if (total_time == 0) return bottlenecks;
    
    // 识别耗时超过阈值的节点
    for (const auto& pair : node_stats_) {
        const auto& stats = pair.second;
        double percent = stats.total_duration_us * 100.0 / total_time;
        
        if (percent >= threshold_percent) {
            PerformanceBottleneck bn;
            bn.node_id = stats.node_id;
            bn.node_type = stats.node_type;
            bn.bottleneck_type = "slow_execution";
            bn.severity = percent / 100.0;
            bn.impact_score = percent;
            
            std::ostringstream desc;
            desc << "Node consumes " << std::fixed << std::setprecision(1) << percent 
                 << "% of execution time (avg: " << stats.avg_duration_us / 1000.0 << " ms)";
            bn.description = desc.str();
            
            bn.details = json::object();
            bn.details["avg_duration_ms"] = stats.avg_duration_us / 1000.0;
            bn.details["execution_count"] = stats.execution_count;
            bn.details["percent_of_total"] = percent;
            
            bottlenecks.push_back(bn);
        }
        
        // 识别高内存使用节点
        if (stats.peak_memory_bytes > 50 * 1024 * 1024) {  // > 50MB
            PerformanceBottleneck bn;
            bn.node_id = stats.node_id;
            bn.node_type = stats.node_type;
            bn.bottleneck_type = "high_memory";
            bn.severity = stats.peak_memory_bytes / (100.0 * 1024 * 1024);
            bn.impact_score = stats.peak_memory_bytes / (1024.0 * 1024.0);
            
            std::ostringstream desc;
            desc << "Node uses peak memory of " << std::fixed << std::setprecision(1)
                 << stats.peak_memory_bytes / (1024.0 * 1024.0) << " MB";
            bn.description = desc.str();
            
            bn.details = json::object();
            bn.details["peak_memory_mb"] = stats.peak_memory_bytes / (1024.0 * 1024.0);
            bn.details["avg_memory_mb"] = stats.avg_memory_bytes / (1024.0 * 1024.0);
            
            bottlenecks.push_back(bn);
        }
        
        // 识别频繁错误节点
        if (stats.error_count > 0 && stats.error_count * 100.0 / stats.execution_count > 10.0) {
            PerformanceBottleneck bn;
            bn.node_id = stats.node_id;
            bn.node_type = stats.node_type;
            bn.bottleneck_type = "frequent_error";
            bn.severity = 1.0 - stats.success_rate / 100.0;
            bn.impact_score = stats.error_count;
            
            std::ostringstream desc;
            desc << "Node has " << stats.error_count << " errors (" 
                 << std::fixed << std::setprecision(1) << (100.0 - stats.success_rate) << "% failure rate)";
            bn.description = desc.str();
            
            bn.details = json::object();
            bn.details["error_count"] = stats.error_count;
            bn.details["success_rate"] = stats.success_rate;
            
            bottlenecks.push_back(bn);
        }
    }
    
    // 按影响分数排序
    std::sort(bottlenecks.begin(), bottlenecks.end(),
              [](const PerformanceBottleneck& a, const PerformanceBottleneck& b) {
                  return a.impact_score > b.impact_score;
              });
    
    return bottlenecks;
}

double PerformanceAnalyzer::get_current_fps() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (frame_history_.empty()) return 0.0;
    
    auto now = get_timestamp_us();
    uint64_t window_start = now - FPS_WINDOW_SECONDS * 1000000;
    
    int frames_in_window = 0;
    for (const auto& pair : frame_history_) {
        if (pair.first >= window_start) {
            frames_in_window = pair.second;
            break;
        }
    }
    
    // 查找窗口开始时的帧数
    int frames_at_start = 0;
    for (const auto& pair : frame_history_) {
        if (pair.first < window_start) {
            frames_at_start = pair.second;
        }
    }
    
    int delta_frames = frames_in_window - frames_at_start;
    if (delta_frames <= 0) return 0.0;
    
    return delta_frames / static_cast<double>(FPS_WINDOW_SECONDS);
}

double PerformanceAnalyzer::get_throughput() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (node_execution_history_.empty()) return 0.0;
    
    auto now = get_timestamp_us();
    uint64_t window_start = now - FPS_WINDOW_SECONDS * 1000000;
    
    int nodes_in_window = 0;
    for (const auto& pair : node_execution_history_) {
        if (pair.first >= window_start) {
            nodes_in_window = pair.second;
            break;
        }
    }
    
    int nodes_at_start = 0;
    for (const auto& pair : node_execution_history_) {
        if (pair.first < window_start) {
            nodes_at_start = pair.second;
        }
    }
    
    int delta_nodes = nodes_in_window - nodes_at_start;
    if (delta_nodes <= 0) return 0.0;
    
    return delta_nodes / static_cast<double>(FPS_WINDOW_SECONDS);
}

size_t PerformanceAnalyzer::get_current_memory() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_memory_;
}

size_t PerformanceAnalyzer::get_peak_memory() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return peak_memory_;
}

PerformanceReport PerformanceAnalyzer::generate_report(int history_seconds) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    PerformanceReport report;
    
    report.slowest_nodes = get_slowest_nodes(10);
    report.bottlenecks = identify_bottlenecks(10.0);
    
    // 计算总执行时间
    uint64_t total_time = 0;
    for (const auto& pair : node_stats_) {
        total_time += pair.second.total_duration_us;
    }
    report.total_execution_time_ms = total_time / 1000.0;
    
    report.fps = get_current_fps();
    report.throughput = get_throughput();
    report.peak_memory_mb = peak_memory_ / (1024.0 * 1024.0);
    report.current_memory_mb = current_memory_ / (1024.0 * 1024.0);
    
    // 时间序列数据
    auto now = get_timestamp_us();
    uint64_t window_start = now - history_seconds * 1000000;
    
    for (const auto& pair : execution_time_history_) {
        if (pair.first >= window_start) {
            report.execution_times.push_back(pair.second);
        }
    }
    
    for (const auto& pair : memory_timeline_) {
        if (pair.first >= window_start) {
            report.memory_timeline.push_back(pair.second / (1024.0 * 1024.0));
        }
    }
    
    return report;
}

json PerformanceAnalyzer::generate_chartjs_report(int history_seconds) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto report = generate_report(history_seconds);
    
    json chartjs = json::object();
    
    // 最慢节点图表数据（Bar Chart）
    json slowest_chart = json::object();
    slowest_chart["type"] = "bar";
    
    json slowest_data = json::object();
    json labels = json::array();
    json values = json::array();
    for (const auto& stats : report.slowest_nodes) {
        labels.push_back(stats.node_name.empty() ? stats.node_id : stats.node_name);
        values.push_back(stats.avg_duration_us / 1000.0);
    }
    slowest_data["labels"] = labels;
    
    json dataset = json::object();
    dataset["label"] = "Average Duration (ms)";
    dataset["data"] = values;
    dataset["backgroundColor"] = "rgba(255, 99, 132, 0.5)";
    
    slowest_data["datasets"] = json::array();
    slowest_data["datasets"].push_back(dataset);
    slowest_chart["data"] = slowest_data;
    
    chartjs["slowest_nodes_chart"] = slowest_chart;
    
    // 执行时间时间序列（Line Chart）
    json timeline_chart = json::object();
    timeline_chart["type"] = "line";
    
    json timeline_data = json::object();
    timeline_data["labels"] = json::array();  // 时间戳标签
    for (size_t i = 0; i < report.execution_times.size(); ++i) {
        timeline_data["labels"].push_back(std::to_string(i));
    }
    
    json timeline_dataset = json::object();
    timeline_dataset["label"] = "Execution Time (ms)";
    timeline_dataset["data"] = json::array();
    for (const auto& time : report.execution_times) {
        timeline_dataset["data"].push_back(time);
    }
    timeline_dataset["fill"] = false;
    timeline_dataset["borderColor"] = "rgb(75, 192, 192)";
    timeline_dataset["tension"] = 0.1;
    
    timeline_data["datasets"] = json::array();
    timeline_data["datasets"].push_back(timeline_dataset);
    timeline_chart["data"] = timeline_data;
    
    chartjs["execution_timeline_chart"] = timeline_chart;
    
    // 内存使用时间序列（Line Chart）
    json memory_chart = json::object();
    memory_chart["type"] = "line";
    
    json memory_data = json::object();
    memory_data["labels"] = json::array();
    for (size_t i = 0; i < report.memory_timeline.size(); ++i) {
        memory_data["labels"].push_back(std::to_string(i));
    }
    
    json memory_dataset = json::object();
    memory_dataset["label"] = "Memory (MB)";
    memory_dataset["data"] = json::array();
    for (const auto& mem : report.memory_timeline) {
        memory_dataset["data"].push_back(mem);
    }
    memory_dataset["fill"] = false;
    memory_dataset["borderColor"] = "rgb(153, 102, 255)";
    memory_dataset["tension"] = 0.1;
    
    memory_data["datasets"] = json::array();
    memory_data["datasets"].push_back(memory_dataset);
    memory_chart["data"] = memory_data;
    
    chartjs["memory_timeline_chart"] = memory_chart;
    
    // 瓶颈列表
    json bottlenecks = json::array();
    for (const auto& bn : report.bottlenecks) {
        json bn_json = json::object();
        bn_json["node_id"] = bn.node_id;
        bn_json["node_type"] = bn.node_type;
        bn_json["bottleneck_type"] = bn.bottleneck_type;
        bn_json["description"] = bn.description;
        bn_json["severity"] = bn.severity;
        bn_json["impact_score"] = bn.impact_score;
        bottlenecks.push_back(bn_json);
    }
    chartjs["bottlenecks"] = bottlenecks;
    
    // 性能摘要
    chartjs["summary"] = json::object();
    chartjs["summary"]["fps"] = report.fps;
    chartjs["summary"]["throughput"] = report.throughput;
    chartjs["summary"]["peak_memory_mb"] = report.peak_memory_mb;
    chartjs["summary"]["current_memory_mb"] = report.current_memory_mb;
    chartjs["summary"]["total_execution_time_ms"] = report.total_execution_time_ms;
    
    return chartjs;
}

void PerformanceAnalyzer::clear_history() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    frame_history_.clear();
    node_execution_history_.clear();
    memory_history_.clear();
    execution_time_history_.clear();
    memory_timeline_.clear();
    
    OVF_INFO() << "Performance history cleared";
}

void PerformanceAnalyzer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    node_stats_.clear();
    frame_history_.clear();
    node_execution_history_.clear();
    memory_history_.clear();
    execution_time_history_.clear();
    memory_timeline_.clear();
    
    total_frames_ = 0;
    total_nodes_executed_ = 0;
    current_memory_ = 0;
    peak_memory_ = 0;
    start_timestamp_ = get_timestamp_us();
    
    OVF_INFO() << "Performance analyzer reset";
}

} // namespace web
} // namespace ovf