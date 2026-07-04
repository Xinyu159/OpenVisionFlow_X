/**
 * @file performance_analyzer.h
 * @brief 性能分析器 - 节点耗时分析、瓶颈识别、内存监控
 */

#pragma once

#include "ovf/core/types.h"
#include "nlohmann/json.hpp"
#include <chrono>
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <deque>

namespace ovf {
namespace web {

/**
 * @brief 节点性能统计
 */
struct NodePerformanceStats {
    String node_id;                     // 节点ID
    String node_type;                   // 节点类型
    String node_name;                   // 节点名称
    uint64_t total_duration_us;         // 总耗时（微秒）
    uint64_t min_duration_us;           // 最小耗时
    uint64_t max_duration_us;           // 最大耗时
    uint64_t avg_duration_us;           // 平均耗时
    int execution_count;                 // 执行次数
    int error_count;                     // 错误次数
    double success_rate;                // 成功率（百分比）
    size_t peak_memory_bytes;           // 峰值内存使用
    size_t avg_memory_bytes;            // 平均内存使用
};

/**
 * @brief 性能瓶颈信息
 */
struct PerformanceBottleneck {
    String node_id;                     // 瓶颈节点ID
    String node_type;                   // 节点类型
    String bottleneck_type;             // 瓶颈类型（slow_execution, high_memory, frequent_error）
    String description;                 // 详细描述
    double severity;                     // 严重程度（0-1）
    double impact_score;                // 影响分数
    nlohmann::json details;            // 详细信息
};

/**
 * @brief 性能快照（用于FPS/吞吐量计算）
 */
struct PerformanceSnapshot {
    uint64_t timestamp_us;              // 时间戳
    int frames_processed;               // 已处理帧数
    int nodes_executed;                 // 已执行节点数
    size_t memory_used_bytes;           // 内存使用
    double cpu_usage_percent;          // CPU使用率（如果可获取）
};

/**
 * @brief 性能报告（Chart.js格式）
 */
struct PerformanceReport {
    // 最慢节点排行
    std::vector<NodePerformanceStats> slowest_nodes;
    
    // 性能瓶颈列表
    std::vector<PerformanceBottleneck> bottlenecks;
    
    // 整体统计
    double total_execution_time_ms;     // 总执行时间
    double fps;                          // 帧率
    double throughput;                   // 吞吐量（节点/秒）
    size_t peak_memory_mb;              // 峰值内存（MB）
    size_t current_memory_mb;           // 当前内存（MB）
    
    // 时间序列数据（用于Chart.js绘图）
    std::vector<double> execution_times; // 执行时间序列
    std::vector<double> memory_timeline; // 内存使用时间序列
    std::vector<String> timestamps;      // 时间戳标签
};

/**
 * @brief 性能分析器
 * 
 * 提供节点耗时分析、瓶颈识别、内存监控等功能
 * 支持Chart.js格式的输出，便于前端绘图
 */
class PerformanceAnalyzer {
public:
    using Ptr = std::shared_ptr<PerformanceAnalyzer>;
    
    PerformanceAnalyzer();
    ~PerformanceAnalyzer();
    
    /**
     * @brief 记录节点执行性能数据
     */
    void record_node_execution(const String& node_id, 
                              const String& node_type,
                              const String& node_name,
                              uint64_t duration_us,
                              bool success,
                              size_t memory_delta_bytes);
    
    /**
     * @brief 记录帧处理完成（用于FPS计算）
     */
    void record_frame_processed();
    
    /**
     * @brief 记录内存快照
     */
    void record_memory_snapshot(size_t memory_bytes);
    
    /**
     * @brief 获取节点性能统计
     */
    NodePerformanceStats get_node_stats(const String& node_id) const;
    
    /**
     * @brief 获取所有节点性能统计
     */
    std::vector<NodePerformanceStats> get_all_stats() const;
    
    /**
     * @brief 获取最慢的N个节点
     */
    std::vector<NodePerformanceStats> get_slowest_nodes(int top_n = 10) const;
    
    /**
     * @brief 识别性能瓶颈
     */
    std::vector<PerformanceBottleneck> identify_bottlenecks(double threshold_percent = 10.0) const;
    
    /**
     * @brief 获取当前FPS
     */
    double get_current_fps() const;
    
    /**
     * @brief 获取吞吐量（节点/秒）
     */
    double get_throughput() const;
    
    /**
     * @brief 获取当前内存使用（字节）
     */
    size_t get_current_memory() const;
    
    /**
     * @brief 获取峰值内存使用（字节）
     */
    size_t get_peak_memory() const;
    
    /**
     * @brief 生成性能报告（Chart.js格式）
     */
    PerformanceReport generate_report(int history_seconds = 60) const;
    
    /**
     * @brief 生成Chart.js格式的JSON报告
     */
    nlohmann::json generate_chartjs_report(int history_seconds = 60) const;
    
    /**
     * @brief 清除历史数据
     */
    void clear_history();
    
    /**
     * @brief 重置统计
     */
    void reset();
    
private:
    /**
     * @brief 更新FPS计算
     */
    void update_fps();
    
    /**
     * @brief 计算时间窗口内的平均值
     */
    double calculate_window_average(const std::deque<std::pair<uint64_t, double>>& data,
                                   int window_seconds) const;
    
    /**
     * @brief 获取当前时间戳（微秒）
     */
    uint64_t get_timestamp_us() const;
    
private:
    mutable std::mutex mutex_;
    
    // 节点性能统计
    std::map<String, NodePerformanceStats> node_stats_;
    
    // FPS计算相关
    std::deque<std::pair<uint64_t, int>> frame_history_;  // <timestamp, frame_count>
    int total_frames_;
    uint64_t start_timestamp_;
    
    // 吞吐量计算
    std::deque<std::pair<uint64_t, int>> node_execution_history_;
    int total_nodes_executed_;
    
    // 内存监控
    std::deque<std::pair<uint64_t, size_t>> memory_history_;
    size_t current_memory_;
    size_t peak_memory_;
    
    // 时间序列数据（用于绘图）
    std::deque<std::pair<uint64_t, double>> execution_time_history_;
    std::deque<std::pair<uint64_t, size_t>> memory_timeline_;
    
    // 配置
    static constexpr int MAX_HISTORY_SECONDS = 3600;  // 最多保留1小时的历史数据
    static constexpr int FPS_WINDOW_SECONDS = 5;      // FPS计算窗口
};

} // namespace web
} // namespace ovf