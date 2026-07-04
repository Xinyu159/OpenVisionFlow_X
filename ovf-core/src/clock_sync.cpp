/**
 * @file clock_sync.cpp
 * @brief OpenVisionFlow 全局时钟同步模块实现
 */

#include "ovf/core/clock_sync.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace ovf {
namespace core {

//==============================================================================
// GlobalClock 实现
//==============================================================================

GlobalClock& GlobalClock::instance() {
    static GlobalClock instance;
    return instance;
}

GlobalClock::GlobalClock() {
#ifdef _WIN32
    QueryPerformanceFrequency(&freq_);
    QueryPerformanceCounter(&base_counter_);
    frequency_ = static_cast<float>(freq_.QuadPart);
#else
    frequency_ = 1000000000.0f; // nanoseconds
#endif
}

ClockTimestamp GlobalClock::now() const {
#ifdef _WIN32
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    
    // 转换为微秒
    uint64_t microseconds = static_cast<uint64_t>(
        (counter.QuadPart - base_counter_.QuadPart) * 1000000ULL / freq_.QuadPart
    );
    
    // 应用偏移
    microseconds += static_cast<uint64_t>(offset_us_);
    
    return microseconds;
#else
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    return microseconds.count() + static_cast<uint64_t>(offset_us_);
#endif
}

void GlobalClock::set_clock_source(ClockSource source) {
    source_ = source;
    
    // 根据时钟源设置精度
    switch (source) {
        case ClockSource::System:
            accuracy_us_ = 10.0f;  // 10微秒
            break;
        case ClockSource::Hardware:
            accuracy_us_ = 1.0f;   // 1微秒
            break;
        case ClockSource::PTP:
            accuracy_us_ = 0.001f; // 1纳秒
            break;
        case ClockSource::NTP:
            accuracy_us_ = 100.0f; // 100微秒
            break;
    }
}

ErrorCode GlobalClock::sync_to_external(ClockTimestamp external_time) {
    ClockTimestamp current_time = now();
    offset_us_ = static_cast<float>(external_time) - static_cast<float>(current_time);
    
    OVF_INFO() << "Clock synchronized to external time. Offset: " << offset_us_ << " us";
    
    return ErrorCode::Success;
}

float GlobalClock::get_accuracy() const {
    return accuracy_us_;
}

//==============================================================================
// MultiCameraSync 实现
//==============================================================================

MultiCameraSync::MultiCameraSync() {
}

MultiCameraSync::~MultiCameraSync() {
    stop_continuous();
}

void MultiCameraSync::add_camera(uint32_t camera_id, float offset_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = std::find(camera_ids_.begin(), camera_ids_.end(), camera_id);
    if (it == camera_ids_.end()) {
        camera_ids_.push_back(camera_id);
        camera_offsets_.push_back(offset_us);
        OVF_INFO() << "Camera " << camera_id << " added to sync group with offset " << offset_us << " us";
    } else {
        size_t index = std::distance(camera_ids_.begin(), it);
        camera_offsets_[index] = offset_us;
        OVF_INFO() << "Camera " << camera_id << " offset updated to " << offset_us << " us";
    }
}

void MultiCameraSync::remove_camera(uint32_t camera_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = std::find(camera_ids_.begin(), camera_ids_.end(), camera_id);
    if (it != camera_ids_.end()) {
        size_t index = std::distance(camera_ids_.begin(), it);
        camera_ids_.erase(it);
        camera_offsets_.erase(camera_offsets_.begin() + index);
        OVF_INFO() << "Camera " << camera_id << " removed from sync group";
    }
}

void MultiCameraSync::set_sync_mode(SyncMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
    OVF_INFO() << "Sync mode changed to " << static_cast<int>(mode);
}

void MultiCameraSync::set_frame_rate(float fps) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_fps_ = fps;
    OVF_INFO() << "Target frame rate set to " << fps << " fps";
}

void MultiCameraSync::set_tolerance(float tolerance_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    tolerance_us_ = tolerance_us;
    OVF_INFO() << "Sync tolerance set to " << tolerance_us << " us";
}

ErrorCode MultiCameraSync::sync_capture(Vector<SyncFrame>& frames, uint32_t timeout_ms) {
    return do_sync_capture(frames, timeout_ms);
}

ErrorCode MultiCameraSync::do_sync_capture(Vector<SyncFrame>& frames, uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (camera_ids_.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    frames.clear();
    frames.reserve(camera_ids_.size());
    
    ClockTimestamp start_time = GlobalClock::instance().now();
    ClockTimestamp timeout_us = static_cast<ClockTimestamp>(timeout_ms) * 1000;
    
    // 这里需要实际的相机接口来获取帧数据
    // 目前返回空帧作为占位符
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
        SyncFrame frame;
        frame.camera_id = camera_ids_[i];
        frame.capture_time = GlobalClock::instance().now();
        frame.frame_id = frame_counter_.fetch_add(1);
        frame.exposure_us = 10000.0f; // 默认曝光10ms
        frame.is_synced = true;
        
        frames.push_back(frame);
    }
    
    // 检查是否超时
    ClockTimestamp elapsed = GlobalClock::instance().now() - start_time;
    if (elapsed > timeout_us) {
        OVF_WARN() << "Sync capture timeout: " << elapsed << " us > " << timeout_us << " us";
        return ErrorCode::Timeout;
    }
    
    // 检查同步精度
    if (!is_frame_synced(frames, tolerance_us_)) {
        dropped_count_.fetch_add(1);
        OVF_WARN() << "Frame sync failed, tolerance exceeded";
        return ErrorCode::FrameSyncError;
    }
    
    synced_count_.fetch_add(1);
    last_sync_time_ = GlobalClock::instance().now();
    
    return ErrorCode::Success;
}

SyncStatus MultiCameraSync::get_status() const {
    SyncStatus status;
    status.synced_count = synced_count_.load();
    status.dropped_count = dropped_count_.load();
    status.sync_accuracy = GlobalClock::instance().get_accuracy();
    status.target_fps = target_fps_;
    status.active_cameras = static_cast<uint32_t>(camera_ids_.size());
    
    // 计算当前帧率
    if (last_sync_time_ > 0) {
        ClockTimestamp now = GlobalClock::instance().now();
        ClockTimestamp elapsed = now - last_sync_time_;
        if (elapsed > 0) {
            status.current_fps = 1000000.0f / static_cast<float>(elapsed);
        }
    }
    
    return status;
}

ErrorCode MultiCameraSync::start_continuous() {
    if (running_.load()) {
        return ErrorCode::Success;
    }
    
    running_.store(true);
    sync_thread_ = std::thread(&MultiCameraSync::sync_thread_func, this);
    
    OVF_INFO() << "Continuous sync capture started";
    return ErrorCode::Success;
}

ErrorCode MultiCameraSync::stop_continuous() {
    if (!running_.load()) {
        return ErrorCode::Success;
    }
    
    running_.store(false);
    cv_.notify_all();
    
    if (sync_thread_.joinable()) {
        sync_thread_.join();
    }
    
    OVF_INFO() << "Continuous sync capture stopped";
    return ErrorCode::Success;
}

void MultiCameraSync::set_sync_callback(std::function<void(const Vector<SyncFrame>&)> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = callback;
}

void MultiCameraSync::set_trigger_signal(std::function<void()> trigger_func) {
    std::lock_guard<std::mutex> lock(mutex_);
    trigger_func_ = trigger_func;
}

void MultiCameraSync::sync_thread_func() {
    OVF_INFO() << "Sync thread started";
    
    float interval_us = 1000000.0f / target_fps_;
    ClockTimestamp next_capture_time = GlobalClock::instance().now();
    
    while (running_.load()) {
        ClockTimestamp current_time = GlobalClock::instance().now();
        
        if (current_time >= next_capture_time) {
            Vector<SyncFrame> frames;
            
            // 硬同步模式：发送触发信号
            if (mode_ == SyncMode::Hardware && trigger_func_) {
                trigger_func_();
            }
            
            ErrorCode result = do_sync_capture(frames, 1000);
            
            if (result == ErrorCode::Success && callback_) {
                callback_(frames);
            }
            
            next_capture_time += static_cast<ClockTimestamp>(interval_us);
        } else {
            // 等待下一次采集时间
            ClockTimestamp wait_time = next_capture_time - current_time;
            if (wait_time > 0) {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::microseconds(wait_time));
            }
        }
    }
    
    OVF_INFO() << "Sync thread stopped";
}

//==============================================================================
// TimestampedBuffer 实现
//==============================================================================

TimestampedBuffer::TimestampedBuffer(size_t capacity) 
    : capacity_(capacity) {
    buffer_.resize(capacity_);
}

void TimestampedBuffer::push(const SyncFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    buffer_[head_] = frame;
    head_ = (head_ + 1) % capacity_;
    
    if (count_ < capacity_) {
        count_++;
    } else {
        tail_ = (tail_ + 1) % capacity_;
    }
}

bool TimestampedBuffer::pop(SyncFrame& frame, ClockTimestamp before_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (count_ == 0) {
        return false;
    }
    
    if (before_time == 0) {
        // 弹出最旧的帧
        frame = buffer_[tail_];
        tail_ = (tail_ + 1) % capacity_;
        count_--;
        return true;
    }
    
    // 查找指定时间之前的帧
    size_t current = tail_;
    for (size_t i = 0; i < count_; ++i) {
        if (buffer_[current].capture_time <= before_time) {
            frame = buffer_[current];
            // 移除该帧（标记为已弹出）
            // 简化实现：只弹出最旧的
            if (current == tail_) {
                tail_ = (tail_ + 1) % capacity_;
                count_--;
            }
            return true;
        }
        current = (current + 1) % capacity_;
    }
    
    return false;
}

bool TimestampedBuffer::pop_latest(SyncFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (count_ == 0) {
        return false;
    }
    
    // 最新帧在 head_ - 1 位置
    size_t latest = (head_ + capacity_ - 1) % capacity_;
    frame = buffer_[latest];
    head_ = latest;
    count_--;
    
    return true;
}

bool TimestampedBuffer::pop_oldest(SyncFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (count_ == 0) {
        return false;
    }
    
    frame = buffer_[tail_];
    tail_ = (tail_ + 1) % capacity_;
    count_--;
    
    return true;
}

size_t TimestampedBuffer::size() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex_));
    return count_;
}

void TimestampedBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    tail_ = 0;
    count_ = 0;
}

Vector<SyncFrame> TimestampedBuffer::query_range(ClockTimestamp start, ClockTimestamp end) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Vector<SyncFrame> result;
    if (count_ == 0) {
        return result;
    }
    
    size_t current = tail_;
    for (size_t i = 0; i < count_; ++i) {
        if (buffer_[current].capture_time >= start && buffer_[current].capture_time <= end) {
            result.push_back(buffer_[current]);
        }
        current = (current + 1) % capacity_;
    }
    
    return result;
}

ClockTimestamp TimestampedBuffer::get_latest_time() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex_));
    
    if (count_ == 0) {
        return 0;
    }
    
    size_t latest = (head_ + capacity_ - 1) % capacity_;
    return buffer_[latest].capture_time;
}

ClockTimestamp TimestampedBuffer::get_oldest_time() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex_));
    
    if (count_ == 0) {
        return 0;
    }
    
    return buffer_[tail_].capture_time;
}

//==============================================================================
// HighResTimer 实现
//==============================================================================

HighResTimer::HighResTimer() {
#ifdef _WIN32
    QueryPerformanceFrequency(&frequency_);
#endif
    reset();
}

void HighResTimer::start() {
#ifdef _WIN32
    QueryPerformanceCounter(&start_time_);
#else
    start_time_ = std::chrono::steady_clock::now();
#endif
    running_ = true;
}

void HighResTimer::stop() {
#ifdef _WIN32
    QueryPerformanceCounter(&end_time_);
#else
    end_time_ = std::chrono::steady_clock::now();
#endif
    running_ = false;
}

void HighResTimer::reset() {
#ifdef _WIN32
    start_time_.QuadPart = 0;
    end_time_.QuadPart = 0;
#else
    start_time_ = std::chrono::steady_clock::time_point();
    end_time_ = std::chrono::steady_clock::time_point();
#endif
    running_ = false;
}

ClockTimestamp HighResTimer::elapsed_us() const {
#ifdef _WIN32
    LARGE_INTEGER end = running_ ? start_time_ : end_time_;
    if (start_time_.QuadPart == 0) {
        return 0;
    }
    
    LARGE_INTEGER elapsed;
    elapsed.QuadPart = end.QuadPart - start_time_.QuadPart;
    
    // 转换为微秒
    return static_cast<ClockTimestamp>(
        (elapsed.QuadPart * 1000000ULL) / frequency_.QuadPart
    );
#else
    auto end = running_ ? std::chrono::steady_clock::now() : end_time_;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_time_);
    return duration.count();
#endif
}

float HighResTimer::elapsed_ms() const {
    return static_cast<float>(elapsed_us()) / 1000.0f;
}

float HighResTimer::elapsed_sec() const {
    return static_cast<float>(elapsed_us()) / 1000000.0f;
}

//==============================================================================
// 工具函数实现
//==============================================================================

String timestamp_to_string(ClockTimestamp ts) {
    // 将微秒时间戳转换为可读字符串
    uint64_t total_seconds = ts / 1000000ULL;
    uint64_t microseconds = ts % 1000000ULL;
    
    uint64_t hours = total_seconds / 3600;
    uint64_t minutes = (total_seconds % 3600) / 60;
    uint64_t seconds = total_seconds % 60;
    
    std::ostringstream oss;
    oss << std::setfill('0') 
        << std::setw(2) << hours << ":"
        << std::setw(2) << minutes << ":"
        << std::setw(2) << seconds << "."
        << std::setw(6) << microseconds;
    
    return oss.str();
}

ClockTimestamp time_diff_us(ClockTimestamp t1, ClockTimestamp t2) {
    if (t1 >= t2) {
        return t1 - t2;
    } else {
        return t2 - t1;
    }
}

bool is_frame_synced(const Vector<SyncFrame>& frames, float tolerance_us) {
    if (frames.empty()) {
        return true;
    }
    
    // 找到最早和最晚的时间戳
    ClockTimestamp min_time = frames[0].capture_time;
    ClockTimestamp max_time = frames[0].capture_time;
    
    for (const auto& frame : frames) {
        if (frame.capture_time < min_time) {
            min_time = frame.capture_time;
        }
        if (frame.capture_time > max_time) {
            max_time = frame.capture_time;
        }
    }
    
    // 计算时间差
    ClockTimestamp diff = max_time - min_time;
    return static_cast<float>(diff) <= tolerance_us;
}

float calculate_sync_offset(const Vector<SyncFrame>& frames) {
    if (frames.size() < 2) {
        return 0.0f;
    }
    
    // 计算平均时间戳
    ClockTimestamp sum = 0;
    for (const auto& frame : frames) {
        sum += frame.capture_time;
    }
    ClockTimestamp avg = sum / frames.size();
    
    // 计算平均偏差
    float total_offset = 0.0f;
    for (const auto& frame : frames) {
        total_offset += static_cast<float>(
            frame.capture_time > avg ? frame.capture_time - avg : avg - frame.capture_time
        );
    }
    
    return total_offset / frames.size();
}

} // namespace core
} // namespace ovf