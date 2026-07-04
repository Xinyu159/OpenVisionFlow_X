/**
 * @file hal.h
 * @brief OpenVisionFlow 硬件抽象层
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/error.h"
#include "ovf/core/data.h"
#include <memory>
#include <functional>
#include <mutex>

namespace ovf {
namespace hal {

/**
 * @brief 设备状态
 */
enum class DeviceState : uint8_t {
    Offline = 0,        // 离线
    Ready = 1,          // 就绪
    Running = 2,        // 运行中
    Error = 3,          // 错误
    Busy = 4            // 繁忙
};

/**
 * @brief 设备信息
 */
struct DeviceInfo {
    String id;              // 设备唯一ID
    String name;            // 设备名称
    String vendor;          // 制造商
    String model;           // 型号
    String serial_number;   // 序列号
    String driver_type;     // 驱动类型
    String firmware_version; // 固件版本
    String description;     // 描述
};

/**
 * @brief 设备基类 - 所有硬件设备的抽象接口
 */
class IDevice {
public:
    using Ptr = std::shared_ptr<IDevice>;
    
    virtual ~IDevice() = default;
    
    // 基本信息
    virtual const DeviceInfo& info() const = 0;
    virtual DeviceState state() const = 0;
    
    // 连接管理
    virtual Result<void> open() = 0;
    virtual Result<void> close() = 0;
    virtual bool is_open() const = 0;
    
    // 参数设置
    virtual Result<void> set_param(const String& key, const Data& value) = 0;
    virtual Result<Data> get_param(const String& key) const = 0;
    
    // 错误信息
    virtual String last_error() const = 0;
};

/**
 * @brief GenICam标准特性节点类型
 * 参考GB/T 33478-2016 机器视觉相机接口规范和GenICam SFNC
 */
enum class GenICamNodeType : uint8_t {
    Integer = 0,    // 整数节点
    Float = 1,      // 浮点节点
    String = 2,     // 字符串节点
    Boolean = 3,    // 布尔节点
    Command = 4,    // 命令节点
    Enumeration = 5, // 枚举节点
    Category = 6    // 分类节点
};

/**
 * @brief GenICam特性节点信息
 */
struct GenICamFeatureInfo {
    String name;               // 特性名称 (GenICam SFNC标准名称)
    String display_name;       // 显示名称
    GenICamNodeType node_type; // 节点类型
    String category;           // 所属分类
    String description;        // 描述
    bool is_readable = true;   // 是否可读
    bool is_writable = true;   // 是否可写
    double min_value = 0.0;    // 最小值 (数值类型)
    double max_value = 0.0;    // 最大值 (数值类型)
    double step_value = 1.0;   // 步进值 (数值类型)
    String unit;               // 单位
    Vector<String> entries;    // 枚举项列表 (枚举类型)
    String current_value;      // 当前值字符串表示
};

/**
 * @brief 图像缓冲区配置
 */
struct BufferConfig {
    uint32_t buffer_count = 5;     // 缓冲区数量
    uint32_t max_queue_size = 10;  // 最大队列大小
    bool use_ring_buffer = true;   // 使用环形缓冲区
    uint32_t timeout_ms = 1000;    // 获取超时时间
};

/**
 * @brief 白平衡参数
 */
struct WhiteBalanceParams {
    double red_ratio = 1.0;   // 红色通道比率
    double green_ratio = 1.0; // 绿色通道比率
    double blue_ratio = 1.0;  // 蓝色通道比率
    bool auto_white_balance = false; // 自动白平衡
};

/**
 * @brief ROI区域配置
 */
struct ROIConfig {
    uint32_t offset_x = 0;     // X偏移
    uint32_t offset_y = 0;     // Y偏移
    uint32_t width = 0;        // ROI宽度
    uint32_t height = 0;       // ROI高度
};

/**
 * @brief 相机接口
 */
class ICamera : public IDevice {
public:
    using Ptr = std::shared_ptr<ICamera>;
    
    // 相机参数
    struct CameraParams {
        uint32_t width = 0;         // 图像宽度
        uint32_t height = 0;        // 图像高度
        uint32_t offset_x = 0;      // X偏移
        uint32_t offset_y = 0;      // Y偏移
        ImageFormat format = ImageFormat::Mono8;
        TriggerMode trigger_mode = TriggerMode::Continuous;
        double exposure_time = 0.0; // 曝光时间(us)
        double gain = 0.0;          // 增益
        double frame_rate = 0.0;    // 帧率
        bool auto_exposure = false; // 自动曝光
        bool auto_gain = false;     // 自动增益
        double black_level = 0.0;   // 黑电平
        WhiteBalanceParams white_balance; // 白平衡参数
    };
    
    // 图像回调
    using ImageCallback = std::function<void(const ImageData&)>;
    
    // GenICam特性回调
    using FeatureCallback = std::function<void(const String& feature_name, const Data& value)>;
    
    virtual ~ICamera() = default;
    
    // 相机参数
    virtual Result<void> set_camera_params(const CameraParams& params) = 0;
    virtual Result<CameraParams> get_camera_params() const = 0;
    
    // 采集控制
    virtual Result<void> start_capture() = 0;
    virtual Result<void> stop_capture() = 0;
    virtual Result<ImageData> capture_frame(uint32_t timeout_ms = 1000) = 0;
    
    // 触发控制
    virtual Result<void> set_trigger_mode(TriggerMode mode) = 0;
    virtual Result<void> send_soft_trigger() = 0;
    virtual TriggerMode get_trigger_mode() const = 0;
    
    // 回调采集
    virtual void set_image_callback(ImageCallback callback) = 0;
    
    // 相机能力
    virtual bool has_feature(const String& feature) const = 0;
    virtual Vector<String> get_features() const = 0;
    
    // GenICam标准特性访问
    virtual Vector<GenICamFeatureInfo> get_genicam_features() const = 0;
    virtual GenICamFeatureInfo get_feature_info(const String& feature_name) const = 0;
    
    // GenICam特性读写 (通用接口)
    virtual Result<void> set_feature_value(const String& feature_name, const Data& value) = 0;
    virtual Result<Data> get_feature_value(const String& feature_name) const = 0;
    
    // 相机特定参数 (便捷接口)
    virtual Result<void> set_exposure(double us) = 0;
    virtual Result<double> get_exposure() const = 0;
    virtual Result<void> set_gain(double gain) = 0;
    virtual Result<double> get_gain() const = 0;
    virtual Result<void> set_white_balance(double r, double g, double b) = 0;
    virtual Result<WhiteBalanceParams> get_white_balance() const = 0;
    
    // ROI控制
    virtual Result<void> set_roi(const ROIConfig& roi) = 0;
    virtual Result<ROIConfig> get_roi() const = 0;
    
    // 图像缓冲区管理
    virtual Result<void> set_buffer_config(const BufferConfig& config) = 0;
    virtual Result<BufferConfig> get_buffer_config() const = 0;
    virtual void clear_buffer_queue() = 0;
    virtual uint32_t get_buffer_queue_size() const = 0;
    
    // 帧率控制
    virtual Result<void> set_frame_rate(double fps) = 0;
    virtual Result<double> get_frame_rate() const = 0;
    
    // 黑电平控制
    virtual Result<void> set_black_level(double level) = 0;
    virtual Result<double> get_black_level() const = 0;
    
    // 自动功能控制
    virtual Result<void> set_auto_exposure(bool enable) = 0;
    virtual Result<void> set_auto_gain(bool enable) = 0;
    virtual Result<void> set_auto_white_balance(bool enable) = 0;
    
    // 特性变更回调
    virtual void set_feature_callback(FeatureCallback callback) = 0;
};

/**
 * @brief 光源接口
 */
class ILightSource : public IDevice {
public:
    using Ptr = std::shared_ptr<ILightSource>;
    
    struct LightParams {
        double intensity = 100.0;      // 亮度百分比
        bool strobe_enabled = false;    // 频闪启用
        double strobe_delay = 0.0;      // 频闪延迟(us)
        double strobe_duration = 0.0;   // 频闪持续时间(us)
        String color;                   // 颜色
    };
    
    virtual ~ILightSource() = default;
    
    virtual Result<void> set_light_params(const LightParams& params) = 0;
    virtual Result<LightParams> get_light_params() const = 0;
    
    virtual Result<void> turn_on() = 0;
    virtual Result<void> turn_off() = 0;
    virtual Result<void> set_intensity(double percent) = 0;
    virtual Result<double> get_intensity() const = 0;
    
    virtual Result<void> enable_strobe(bool enable) = 0;
    virtual Result<void> trigger_strobe() = 0;
};

/**
 * @brief 运动控制接口
 */
class IMotionController : public IDevice {
public:
    using Ptr = std::shared_ptr<IMotionController>;
    
    struct AxisParams {
        uint32_t axis_id;
        double position;
        double velocity;
        double acceleration;
        bool enabled;
        bool homed;
    };
    
    enum MotionType {
        Linear,         // 线性运动
        Rotary,         // 旋转运动
        Continuous      // 连续运动
    };
    
    virtual ~IMotionController() = default;
    
    // 轴管理
    virtual uint32_t axis_count() const = 0;
    virtual Result<void> enable_axis(uint32_t axis) = 0;
    virtual Result<void> disable_axis(uint32_t axis) = 0;
    
    // 位置控制
    virtual Result<double> get_position(uint32_t axis) const = 0;
    virtual Result<void> set_position(uint32_t axis, double pos) = 0;
    virtual Result<void> move_to(uint32_t axis, double pos, double velocity = 0) = 0;
    virtual Result<void> move_relative(uint32_t axis, double distance, double velocity = 0) = 0;
    
    // 原点
    virtual Result<void> home(uint32_t axis) = 0;
    virtual Result<bool> is_homed(uint32_t axis) const = 0;
    
    // 状态
    virtual Result<bool> is_moving(uint32_t axis) const = 0;
    virtual Result<void> stop(uint32_t axis) = 0;
    virtual Result<void> emergency_stop() = 0;
    
    // 参数
    virtual Result<void> set_velocity(uint32_t axis, double velocity) = 0;
    virtual Result<void> set_acceleration(uint32_t axis, double accel) = 0;
};

/**
 * @brief IO接口
 */
class IIODevice : public IDevice {
public:
    using Ptr = std::shared_ptr<IIODevice>;
    
    enum PortType {
        DigitalInput,
        DigitalOutput,
        AnalogInput,
        AnalogOutput
    };
    
    struct PortInfo {
        uint32_t port_id;
        PortType type;
        String name;
        double min_value;
        double max_value;
    };
    
    virtual ~IIODevice() = default;
    
    // 端口管理
    virtual uint32_t port_count() const = 0;
    virtual Vector<PortInfo> get_ports() const = 0;
    
    // 数字IO
    virtual Result<bool> read_digital(uint32_t port) const = 0;
    virtual Result<void> write_digital(uint32_t port, bool value) = 0;
    
    // 模拟IO
    virtual Result<double> read_analog(uint32_t port) const = 0;
    virtual Result<void> write_analog(uint32_t port, double value) = 0;
};

/**
 * @brief 硬件抽象层管理器
 */
class HALManager {
public:
    static HALManager& instance() {
        static HALManager manager;
        return manager;
    }
    
    // 设备注册
    template<typename T>
    void register_driver(const String& driver_type, 
                         std::function<typename T::Ptr(const DeviceInfo&)> creator) {
        creators_[driver_type] = [creator](const DeviceInfo& info) -> IDevice::Ptr {
            return creator(info);
        };
    }
    
    // 设备创建
    ICamera::Ptr create_camera(const String& driver_type, const DeviceInfo& info);
    ILightSource::Ptr create_light_source(const String& driver_type, const DeviceInfo& info);
    IMotionController::Ptr create_motion_controller(const String& driver_type, const DeviceInfo& info);
    IIODevice::Ptr create_io_device(const String& driver_type, const DeviceInfo& info);
    
    // 设备枚举
    Vector<DeviceInfo> enumerate_cameras(const String& driver_type = "");
    Vector<DeviceInfo> enumerate_devices(const String& device_type);
    
    // 设备管理
    void add_device(IDevice::Ptr device);
    void remove_device(const String& device_id);
    IDevice::Ptr get_device(const String& device_id) const;
    ICamera::Ptr get_camera(const String& device_id) const;
    Vector<IDevice::Ptr> get_all_devices() const;
    
    // 设备查找
    Vector<ICamera::Ptr> get_all_cameras() const;
    Vector<ILightSource::Ptr> get_all_light_sources() const;
    Vector<IMotionController::Ptr> get_all_motion_controllers() const;
    Vector<IIODevice::Ptr> get_all_io_devices() const;
    
private:
    HALManager() = default;
    
    HashMap<String, std::function<IDevice::Ptr(const DeviceInfo&)>> creators_;
    HashMap<String, IDevice::Ptr> devices_;
    mutable std::mutex mutex_;
};

// 驱动注册宏
#define OVF_REGISTER_CAMERA_DRIVER(DriverClass, driver_type) \
    namespace { \
        struct DriverClass##Registrar { \
            DriverClass##Registrar() { \
                ovf::hal::HALManager::instance().register_driver<ovf::hal::ICamera>( \
                    driver_type, [](const ovf::hal::DeviceInfo& info) -> ovf::hal::ICamera::Ptr { \
                        return std::make_shared<DriverClass>(info); \
                    }); \
            } \
        } registrar_##DriverClass; \
    }

} // namespace hal
} // namespace ovf