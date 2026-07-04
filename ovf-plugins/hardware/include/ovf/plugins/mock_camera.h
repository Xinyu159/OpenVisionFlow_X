/**
 * @file mock_camera.h
 * @brief 模拟相机插件（用于测试）
 */

#pragma once

#include "ovf/hal/hal.h"
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>

namespace ovf {
namespace plugins {

/**
 * @brief 模拟相机 - 用于无真实硬件环境下的测试
 */
class MockCamera : public hal::ICamera {
public:
    explicit MockCamera(const hal::DeviceInfo& info);
    ~MockCamera();
    
    // IDevice 接口
    const hal::DeviceInfo& info() const override { return info_; }
    hal::DeviceState state() const override { return state_; }
    
    Result<void> open() override;
    Result<void> close() override;
    bool is_open() const override { return is_open_; }
    
    Result<void> set_param(const String& key, const Data& value) override;
    Result<Data> get_param(const String& key) const override;
    String last_error() const override { return last_error_; }
    
    // ICamera 接口 - 基础参数
    Result<void> set_camera_params(const CameraParams& params) override;
    Result<CameraParams> get_camera_params() const override;
    
    // ICamera 接口 - 采集控制
    Result<void> start_capture() override;
    Result<void> stop_capture() override;
    Result<ImageData> capture_frame(uint32_t timeout_ms = 1000) override;
    
    // ICamera 接口 - 触发控制
    Result<void> set_trigger_mode(TriggerMode mode) override;
    Result<void> send_soft_trigger() override;
    TriggerMode get_trigger_mode() const override;
    
    // ICamera 接口 - 回调
    void set_image_callback(ImageCallback callback) override;
    
    // ICamera 接口 - 相机能力
    bool has_feature(const String& feature) const override;
    Vector<String> get_features() const override;
    
    // ICamera 接口 - GenICam特性
    Vector<hal::GenICamFeatureInfo> get_genicam_features() const override;
    hal::GenICamFeatureInfo get_feature_info(const String& feature_name) const override;
    Result<void> set_feature_value(const String& feature_name, const Data& value) override;
    Result<Data> get_feature_value(const String& feature_name) const override;
    
    // ICamera 接口 - 相机参数便捷接口
    Result<void> set_exposure(double us) override;
    Result<double> get_exposure() const override;
    Result<void> set_gain(double gain) override;
    Result<double> get_gain() const override;
    Result<void> set_white_balance(double r, double g, double b) override;
    Result<hal::WhiteBalanceParams> get_white_balance() const override;
    
    // ICamera 接口 - ROI控制
    Result<void> set_roi(const hal::ROIConfig& roi) override;
    Result<hal::ROIConfig> get_roi() const override;
    
    // ICamera 接口 - 缓冲区管理
    Result<void> set_buffer_config(const hal::BufferConfig& config) override;
    Result<hal::BufferConfig> get_buffer_config() const override;
    void clear_buffer_queue() override;
    uint32_t get_buffer_queue_size() const override;
    
    // ICamera 接口 - 帧率控制
    Result<void> set_frame_rate(double fps) override;
    Result<double> get_frame_rate() const override;
    
    // ICamera 接口 - 黑电平控制
    Result<void> set_black_level(double level) override;
    Result<double> get_black_level() const override;
    
    // ICamera 接口 - 自动功能
    Result<void> set_auto_exposure(bool enable) override;
    Result<void> set_auto_gain(bool enable) override;
    Result<void> set_auto_white_balance(bool enable) override;
    
    // ICamera 接口 - 特性回调
    void set_feature_callback(FeatureCallback callback) override;

private:
    void generate_frame();
    void capture_thread();
    
private:
    hal::DeviceInfo info_;
    hal::DeviceState state_ = hal::DeviceState::Offline;
    bool is_open_ = false;
    String last_error_;
    
    CameraParams params_;
    hal::BufferConfig buffer_config_;
    hal::WhiteBalanceParams white_balance_;
    hal::ROIConfig roi_config_;
    
    std::atomic<bool> capturing_{false};
    std::thread capture_thread_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<ImageData> frame_queue_;
    ImageCallback image_callback_;
    FeatureCallback feature_callback_;
    
    uint32_t frame_counter_ = 0;
    double exposure_us_ = 10000.0;
    double gain_ = 1.0;
    double black_level_ = 0.0;
    bool auto_exposure_ = false;
    bool auto_gain_ = false;
    
    Vector<hal::GenICamFeatureInfo> genicam_features_;
};

} // namespace plugins
} // namespace ovf