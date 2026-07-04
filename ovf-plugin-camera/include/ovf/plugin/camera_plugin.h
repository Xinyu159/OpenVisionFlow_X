/**
 * @file camera_plugin.h
 * @brief 相机驱动插件 - 基于GenICam标准的通用相机驱动
 * 参考GB/T 33478-2016 机器视觉相机接口规范
 */

#pragma once

#include "ovf/hal/hal.h"
#include "ovf/core/node.h"
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <mutex>

namespace ovf {
namespace plugin {

/**
 * @brief 相机参数结构体 - 符合任务要求
 */
struct CameraParamsData {
    double exposure_time;  // 曝光时间(us)
    double gain;           // 增益
    int width;             // 分辨率宽度
    int height;            // 分辨率高度
    int pixel_format;      // 像素格式 (对应ImageFormat枚举值)
    int trigger_mode;      // 触发模式 (对应TriggerMode枚举值)
    
    CameraParamsData()
        : exposure_time(10000.0)
        , gain(1.0)
        , width(1280)
        , height(720)
        , pixel_format(static_cast<int>(ImageFormat::BGR8))
        , trigger_mode(static_cast<int>(TriggerMode::Continuous))
    {}
};

/**
 * @brief 通用相机驱动 - 实现GenICam标准特性
 */
class GenericCameraDriver : public hal::ICamera {
public:
    explicit GenericCameraDriver(const hal::DeviceInfo& info);
    ~GenericCameraDriver();
    
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
    void generate_test_frame();
    void capture_thread_func();
    void initialize_genicam_features();
    
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
    mutable std::mutex mutex_;  // mutable for const methods
    std::condition_variable cv_;
    std::queue<ImageData> frame_queue_;
    ImageCallback image_callback_;
    FeatureCallback feature_callback_;
    
    uint32_t frame_counter_ = 0;
    double exposure_us_ = 10000.0;
    double gain_ = 1.0;
    double black_level_ = 0.0;
    double frame_rate_ = 30.0;
    bool auto_exposure_ = false;
    bool auto_gain_ = false;
    
    Vector<hal::GenICamFeatureInfo> genicam_features_;
};

/**
 * @brief 相机采集节点 - CameraAcquireNode
 */
class CameraAcquireNode : public INode {
public:
    explicit CameraAcquireNode(const String& instance_id);
    ~CameraAcquireNode();
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    Result<void> reset() override;
    
    // 设置相机
    void set_camera(hal::ICamera::Ptr camera);
    hal::ICamera::Ptr get_camera() const;
    
    // 采集控制
    Result<void> start_continuous_capture();
    Result<void> stop_capture();
    Result<ImageData> capture_single_frame(uint32_t timeout_ms = 1000);
    
private:
    hal::ICamera::Ptr camera_;
    std::atomic<bool> is_capturing_{false};
};

/**
 * @brief 相机配置节点 - CameraConfigNode
 */
class CameraConfigNode : public INode {
public:
    explicit CameraConfigNode(const String& instance_id);
    ~CameraConfigNode();
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    Result<void> reset() override;
    
    // 设置相机
    void set_camera(hal::ICamera::Ptr camera);
    hal::ICamera::Ptr get_camera() const;
    
    // 配置方法
    Result<void> configure_exposure(double us);
    Result<void> configure_gain(double gain);
    Result<void> configure_trigger_mode(TriggerMode mode);
    Result<void> configure_roi(uint32_t x, uint32_t y, uint32_t width, uint32_t height);
    Result<void> configure_white_balance(double r, double g, double b);
    Result<void> configure_frame_rate(double fps);
    Result<void> configure_pixel_format(ImageFormat format);
    
    // 应用参数
    Result<void> apply_params(const CameraParamsData& params);
    Result<CameraParamsData> get_current_params() const;
    
private:
    hal::ICamera::Ptr camera_;
    CameraParamsData current_params_;
};

// DLL导出函数
#ifdef WIN32
    #ifdef OVF_PLUGIN_CAMERA_EXPORTS
        #define OVF_CAMERA_API __declspec(dllexport)
    #else
        #define OVF_CAMERA_API
    #endif
#else
    #define OVF_CAMERA_API
#endif

// 插件初始化函数
OVF_CAMERA_API void init_camera_plugin();

// 获取相机列表
OVF_CAMERA_API Vector<hal::DeviceInfo> get_available_cameras();

// 创建相机实例
OVF_CAMERA_API hal::ICamera::Ptr create_camera(const String& driver_type, const hal::DeviceInfo& info);

} // namespace plugin
} // namespace ovf