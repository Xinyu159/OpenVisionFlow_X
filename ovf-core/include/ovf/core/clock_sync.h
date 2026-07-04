/**
 * @file clock_sync.h
 * @brief OpenVisionFlow 全局时钟同步模块
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "types.h"
#include "error.h"
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <chrono>
#include <deque>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ovf {
namespace core {

// 时间戳类型（微秒级）
using ClockTimestamp = uint64_t;

/**
 * @brief 全局时钟 - 提统一的时间基准
 */
class GlobalClock {
public:
    static GlobalClock& instance();
    
    // 获取当前时间戳（微秒）
    ClockTimestamp now() const;
    
    // 设置时钟源
    void set_clock_source(ClockSource source);
    ClockSource get_clock_source() const { return source_; }
    
    // 同步到外部时钟
    ErrorCode sync_to_external(ClockTimestamp external_time);
    
    // 获取时钟精度（微秒）
    float get_accuracy() const;
    
    // 获取时钟偏差（微秒）
    float get_offset() const { return offset_us_; }
    
    // 获取时钟频率
    float get_frequency() const { return frequency_; }
    
private:
    GlobalClock();
    ~GlobalClock() = default;
    
    ClockSource source_ = ClockSource::System;
    float offset_us_ = 0.0f;
    float accuracy_us_ = 10.0f;  // 默认精度10微秒
    float frequency_ = 0.0f;     // 时钟频率
    
    #ifdef _WIN32
    LARGE_INTEGER freq_;
    LARGE_INTEGER base_counter_;
    #endif
    
    std::atomic<ClockTimestamp> base_time_{0};
};

/**
 * @brief 同步帧结构 - 带时间戳的帧数据
 */
struct SyncFrame {
    ImageData image;
    ClockTimestamp capture_time;  // 捕获时间戳（微秒）
    uint32_t camera_id;           // 相机ID
    uint32_t frame_id;            // 帧ID
    float exposure_us;            // 曝光时间（微秒）
    bool is_synced;               // 是否已同步
    
    SyncFrame() : capture_time(0), camera_id(0), frame_id(0), exposure_us(0.0f), is_synced(false) {}
};

/**
 * @brief 同步状态
 */
struct SyncStatus {
    uint32_t synced_count;        // 已同步帧数
    uint32_t dropped_count;       // 丢帧数
    float sync_accuracy;          // 同步精度（微秒）
    float current_fps;            // 当前帧率
    float target_fps;             // 目标帧率
    uint32_t active_cameras;      // 活动相机数
    
    SyncStatus() : synced_count(0), dropped_count(0), sync_accuracy(0.0f), 
                   current_fps(0.0f), target_fps(30.0f), active_cameras(0) {}
};

/**
 * @brief 多相机同步管理器
 */
class MultiCameraSync {
public:
    MultiCameraSync();
    ~MultiCameraSync();
    
    // 添加相机到同步组
    void add_camera(uint32_t camera_id, float offset_us = 0.0f);
    void remove_camera(uint32_t camera_id);
    
    // 设置同步模式
    void set_sync_mode(SyncMode mode);
    SyncMode get_sync_mode() const { return mode_; }
    
    // 设置目标帧率
    void set_frame_rate(float fps);
    float get_frame_rate() const { return target_fps_; }
    
    // 设置同步容差（微秒）
    void set_tolerance(float tolerance_us);
    float get_tolerance() const { return tolerance_us_; }
    
    // 同步采集
    ErrorCode sync_capture(Vector<SyncFrame>& frames, uint32_t timeout_ms = 1000);
    
    // 获取同步状态
    SyncStatus get_status() const;
    
    // 启动/停止持续同步采集
    ErrorCode start_continuous();
    ErrorCode stop_continuous();
    
    // 设置同步回调
    void set_sync_callback(std::function<void(const Vector<SyncFrame>&)> callback);
    
    // 设置触发信号接口（硬同步）
    void set_trigger_signal(std::function<void()> trigger_func);
    
    // 获取相机列表
    const Vector<uint32_t>& get_camera_ids() const { return camera_ids_; }
    
    // 是否正在运行
    bool is_running() const { return running_.load(); }

private:
    Vector<uint32_t> camera_ids_;
    Vector<float> camera_offsets_;
    SyncMode mode_ = SyncMode::Software;
    float target_fps_ = 30.0f;
    float tolerance_us_ = 1000.0f;  // 默认容差1毫秒
    
    std::atomic<bool> running_{false};
    std::thread sync_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    
    std::function<void(const Vector<SyncFrame>&)> callback_;
    std::function<void()> trigger_func_;
    
    // 统计数据
    std::atomic<uint32_t> synced_count_{0};
    std::atomic<uint32_t> dropped_count_{0};
    std::atomic<uint32_t> frame_counter_{0};
    
    ClockTimestamp last_sync_time_{0};
    
    // 同步采集线程函数
    void sync_thread_func();
    
    // 单次同步采集
    ErrorCode do_sync_capture(Vector<SyncFrame>& frames, uint32_t timeout_ms);
};

/**
 * @brief 带时间戳的环形缓冲区
 */
class TimestampedBuffer {
public:
    explicit TimestampedBuffer(size_t capacity = 1000);
    ~TimestampedBuffer() = default;
    
    // 推入帧
    void push(const SyncFrame& frame);
    
    // 弹出帧（按时间）
    bool pop(SyncFrame& frame, ClockTimestamp before_time = 0);
    
    // 弹出最新帧
    bool pop_latest(SyncFrame& frame);
    
    // 弹出最旧帧
    bool pop_oldest(SyncFrame& frame);
    
    // 获取大小
    size_t size() const;
    size_t capacity() const { return capacity_; }
    
    // 清空
    void clear();
    
    // 是否为空
    bool empty() const { return size() == 0; }
    
    // 是否已满
    bool full() const { return size() >= capacity_; }
    
    // 按时间范围查询
    Vector<SyncFrame> query_range(ClockTimestamp start, ClockTimestamp end);
    
    // 获取最新帧时间戳
    ClockTimestamp get_latest_time() const;
    
    // 获取最旧帧时间戳
    ClockTimestamp get_oldest_time() const;

private:
    size_t capacity_;
    size_t head_{0};    // 写入位置
    size_t tail_{0};    // 读取位置
    size_t count_{0};   // 当前数量
    Vector<SyncFrame> buffer_;
    std::mutex mutex_;
};

/**
 * @brief 高精度计时器
 */
class HighResTimer {
public:
    HighResTimer();
    
    // 开始计时
    void start();
    
    // 停止计时
    void stop();
    
    // 重置
    void reset();
    
    // 获取 elapsed 时间（微秒）
    ClockTimestamp elapsed_us() const;
    
    // 获取 elapsed 时间（毫秒）
    float elapsed_ms() const;
    
    // 获取 elapsed 时间（秒）
    float elapsed_sec() const;

private:
    #ifdef _WIN32
    LARGE_INTEGER start_time_;
    LARGE_INTEGER end_time_;
    LARGE_INTEGER frequency_;
    #else
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    #endif
    
    bool running_{false};
};

/**
 * @brief 时钟同步工具函数
 */

// 将微秒时间戳转换为字符串
String timestamp_to_string(ClockTimestamp ts);

// 计算两个时间戳之间的时间差（微秒）
ClockTimestamp time_diff_us(ClockTimestamp t1, ClockTimestamp t2);

// 检查帧是否在时间容差内同步
bool is_frame_synced(const Vector<SyncFrame>& frames, float tolerance_us);

// 计算帧组的平均同步偏差
float calculate_sync_offset(const Vector<SyncFrame>& frames);

} // namespace core
} // namespace ovf