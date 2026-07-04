/**
 * @file camera_node.cpp
 * @brief 相机节点实现 - CameraAcquireNode和CameraConfigNode
 */

#include "ovf/plugin/camera_plugin.h"
#include "ovf/core/logger.h"
#include "ovf/core/flow.h"

namespace ovf {
namespace plugin {

// ============================================================================
// CameraAcquireNode - 相机采集节点
// ============================================================================

CameraAcquireNode::CameraAcquireNode(const String& instance_id)
    : INode(instance_id, NodeInfo{
        "camera_acquire", 
        "Camera Acquire", 
        "Camera",
        "Acquire images from camera device",
        "0.1.0",
        "OpenVisionFlow Team",
        {
            // 输入端口 - 使用5参数构造
            DataPort{"trigger", "Trigger", DataType::Boolean, false, Data(false)},
            DataPort{"camera", "Camera", DataType::Any, false, Data{}},
        },
        {
            // 输出端口
            DataPort{"image", "Image", DataType::Image, true, Data{}},
            DataPort{"frame_id", "Frame ID", DataType::Number, false, Data(0)},
            DataPort{"timestamp", "Timestamp", DataType::Number, false, Data(0)},
        },
        {
            // 参数定义
            ParamDef{"timeout_ms", "Timeout (ms)", DataType::Number, Data(1000)},
            ParamDef{"continuous", "Continuous Mode", DataType::Boolean, Data(false)},
            ParamDef{"trigger_soft", "Send Soft Trigger", DataType::Boolean, Data(false)},
        }
    }) {
}

CameraAcquireNode::~CameraAcquireNode() {
    stop_capture();
}

Result<void> CameraAcquireNode::init() {
    OVF_INFO() << "CameraAcquireNode initialized: " << instance_id_;
    return Result<void>::success();
}

Result<void> CameraAcquireNode::execute(FlowContext& context) {
    // 检查是否有相机输入
    if (has_input("camera")) {
        Data camera_data = get_input("camera");
        // 这里需要从Data中提取相机对象，简化处理
        // 实际应用中可能需要使用std::any或其他机制
    }
    
    if (!camera_) {
        set_error("No camera device set");
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device set");
    }
    
    // 检查相机状态
    if (!camera_->is_open()) {
        auto open_result = camera_->open();
        if (open_result.is_failure()) {
            set_error("Failed to open camera: " + open_result.message());
            return open_result;
        }
    }
    
    // 获取参数
    uint32_t timeout_ms = static_cast<uint32_t>(get_param("timeout_ms", Data(1000)).as_int());
    bool continuous = get_param("continuous", Data(false)).as_bool();
    bool trigger_soft = get_param("trigger_soft", Data(false)).as_bool();
    
    // 处理触发输入
    if (has_input("trigger")) {
        bool trigger_signal = get_input("trigger").as_bool();
        if (trigger_signal && camera_->get_trigger_mode() == TriggerMode::Software) {
            auto trigger_result = camera_->send_soft_trigger();
            if (trigger_result.is_failure()) {
                OVF_WARN() << "Soft trigger failed: " << trigger_result.message();
            }
        }
    }
    
    if (trigger_soft && camera_->get_trigger_mode() == TriggerMode::Software) {
        // 发送软触发
        auto trigger_result = camera_->send_soft_trigger();
        if (trigger_result.is_failure()) {
            set_error("Failed to send soft trigger");
            return trigger_result;
        }
    }
    
    // 获取帧
    auto capture_result = capture_single_frame(timeout_ms);
    
    if (capture_result.is_failure()) {
        set_error("Capture failed: " + capture_result.message());
        return Result<void>::failure(capture_result.code(), capture_result.message());
    }
    
    // 输出图像
    ImageData image = capture_result.value();
    set_output("image", Data(image));
    set_output("frame_id", Data(static_cast<int>(image.frame_id)));
    set_output("timestamp", Data(static_cast<double>(image.timestamp)));
    
    OVF_INFO() << "CameraAcquireNode captured frame: " << image.frame_id;
    return Result<void>::success();
}

Result<void> CameraAcquireNode::reset() {
    stop_capture();
    clear_error();
    return Result<void>::success();
}

void CameraAcquireNode::set_camera(hal::ICamera::Ptr camera) {
    camera_ = camera;
}

hal::ICamera::Ptr CameraAcquireNode::get_camera() const {
    return camera_;
}

Result<void> CameraAcquireNode::start_continuous_capture() {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    
    if (is_capturing_) {
        return Result<void>::success();
    }
    
    auto result = camera_->start_capture();
    if (result.is_success()) {
        is_capturing_ = true;
    }
    return result;
}

Result<void> CameraAcquireNode::stop_capture() {
    if (!camera_) {
        return Result<void>::success();
    }
    
    if (is_capturing_) {
        auto result = camera_->stop_capture();
        is_capturing_ = false;
        return result;
    }
    return Result<void>::success();
}

Result<ImageData> CameraAcquireNode::capture_single_frame(uint32_t timeout_ms) {
    if (!camera_) {
        return Result<ImageData>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    return camera_->capture_frame(timeout_ms);
}

// ============================================================================
// CameraConfigNode - 相机配置节点
// ============================================================================

CameraConfigNode::CameraConfigNode(const String& instance_id)
    : INode(instance_id, NodeInfo{
        "camera_config", 
        "Camera Config", 
        "Camera",
        "Configure camera parameters",
        "0.1.0",
        "OpenVisionFlow Team",
        {
            // 输入端口 - 使用5参数构造
            DataPort{"camera", "Camera", DataType::Any, false, Data{}},
            DataPort{"exposure", "Exposure", DataType::Number, false, Data(10000.0)},
            DataPort{"gain", "Gain", DataType::Number, false, Data(1.0)},
            DataPort{"trigger_mode", "Trigger Mode", DataType::Number, false, Data(0)},
        },
        {
            // 输出端口
            DataPort{"camera", "Camera", DataType::Any, true, Data{}},
            DataPort{"config_success", "Success", DataType::Boolean, true, Data(false)},
        },
        {
            // 参数定义
            ParamDef{"exposure_time", "Exposure Time (us)", DataType::Number, Data(10000.0)},
            ParamDef{"gain", "Gain", DataType::Number, Data(1.0)},
            ParamDef{"width", "Width", DataType::Number, Data(1280)},
            ParamDef{"height", "Height", DataType::Number, Data(720)},
            ParamDef{"trigger_mode", "Trigger Mode", DataType::Number, Data(0)},
            ParamDef{"pixel_format", "Pixel Format", DataType::Number, Data(5)},
            ParamDef{"frame_rate", "Frame Rate", DataType::Number, Data(30.0)},
            ParamDef{"auto_exposure", "Auto Exposure", DataType::Boolean, Data(false)},
            ParamDef{"auto_gain", "Auto Gain", DataType::Boolean, Data(false)},
            ParamDef{"roi_x", "ROI X", DataType::Number, Data(0)},
            ParamDef{"roi_y", "ROI Y", DataType::Number, Data(0)},
            ParamDef{"roi_width", "ROI Width", DataType::Number, Data(0)},
            ParamDef{"roi_height", "ROI Height", DataType::Number, Data(0)},
            ParamDef{"wb_r", "White Balance R", DataType::Number, Data(1.0)},
            ParamDef{"wb_g", "White Balance G", DataType::Number, Data(1.0)},
            ParamDef{"wb_b", "White Balance B", DataType::Number, Data(1.0)},
            ParamDef{"black_level", "Black Level", DataType::Number, Data(0.0)},
        }
    }) {
    
    // 初始化默认参数
    current_params_.exposure_time = 10000.0;
    current_params_.gain = 1.0;
    current_params_.width = 1280;
    current_params_.height = 720;
    current_params_.pixel_format = static_cast<int>(ImageFormat::BGR8);
    current_params_.trigger_mode = static_cast<int>(TriggerMode::Continuous);
}

CameraConfigNode::~CameraConfigNode() {
}

Result<void> CameraConfigNode::init() {
    OVF_INFO() << "CameraConfigNode initialized: " << instance_id_;
    return Result<void>::success();
}

Result<void> CameraConfigNode::execute(FlowContext& context) {
    // 检查是否有相机输入
    if (has_input("camera")) {
        // 从输入获取相机引用
    }
    
    if (!camera_) {
        set_error("No camera device set");
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device set");
    }
    
    // 检查相机状态
    if (!camera_->is_open()) {
        auto open_result = camera_->open();
        if (open_result.is_failure()) {
            set_error("Failed to open camera: " + open_result.message());
            return open_result;
        }
    }
    
    bool success = true;
    
    // 从输入端口获取配置值（优先使用输入）
    double exposure = has_input("exposure") ? 
        get_input("exposure").as_number() : 
        get_param("exposure_time", Data(10000.0)).as_number();
    
    double gain_val = has_input("gain") ?
        get_input("gain").as_number() :
        get_param("gain", Data(1.0)).as_number();
    
    int trigger_mode_val = has_input("trigger_mode") ?
        get_input("trigger_mode").as_int() :
        get_param("trigger_mode", Data(0)).as_int();
    
    // 从参数获取其他配置
    int width = get_param("width", Data(1280)).as_int();
    int height = get_param("height", Data(720)).as_int();
    int pixel_format = get_param("pixel_format", Data(5)).as_int();
    double frame_rate = get_param("frame_rate", Data(30.0)).as_number();
    bool auto_exposure = get_param("auto_exposure", Data(false)).as_bool();
    bool auto_gain = get_param("auto_gain", Data(false)).as_bool();
    
    // ROI配置
    int roi_x = get_param("roi_x", Data(0)).as_int();
    int roi_y = get_param("roi_y", Data(0)).as_int();
    int roi_width = get_param("roi_width", Data(0)).as_int();
    int roi_height = get_param("roi_height", Data(0)).as_int();
    
    // 白平衡配置
    double wb_r = get_param("wb_r", Data(1.0)).as_number();
    double wb_g = get_param("wb_g", Data(1.0)).as_number();
    double wb_b = get_param("wb_b", Data(1.0)).as_number();
    
    // 黑电平
    double black_level = get_param("black_level", Data(0.0)).as_number();
    
    // 应用配置
    auto exposure_result = configure_exposure(exposure);
    if (exposure_result.is_failure()) {
        OVF_WARN() << "Failed to set exposure: " << exposure_result.message();
        success = false;
    }
    
    auto gain_result = configure_gain(gain_val);
    if (gain_result.is_failure()) {
        OVF_WARN() << "Failed to set gain: " << gain_result.message();
        success = false;
    }
    
    auto trigger_result = configure_trigger_mode(static_cast<TriggerMode>(trigger_mode_val));
    if (trigger_result.is_failure()) {
        OVF_WARN() << "Failed to set trigger mode: " << trigger_result.message();
        success = false;
    }
    
    // 设置分辨率
    if (width > 0 && height > 0) {
        hal::ROIConfig roi;
        roi.offset_x = roi_x;
        roi.offset_y = roi_y;
        roi.width = width;
        roi.height = height;
        
        auto roi_result = configure_roi(roi_x, roi_y, roi_width > 0 ? roi_width : width, 
                                         roi_height > 0 ? roi_height : height);
        if (roi_result.is_failure()) {
            OVF_WARN() << "Failed to set ROI: " + roi_result.message();
            success = false;
        }
    }
    
    // 设置白平衡
    auto wb_result = configure_white_balance(wb_r, wb_g, wb_b);
    if (wb_result.is_failure()) {
        OVF_WARN() << "Failed to set white balance: " << wb_result.message();
        success = false;
    }
    
    // 设置帧率
    auto fps_result = configure_frame_rate(frame_rate);
    if (fps_result.is_failure()) {
        OVF_WARN() << "Failed to set frame rate: " << fps_result.message();
        success = false;
    }
    
    // 设置像素格式
    auto format_result = configure_pixel_format(static_cast<ImageFormat>(pixel_format));
    if (format_result.is_failure()) {
        OVF_WARN() << "Failed to set pixel format: " << format_result.message();
        success = false;
    }
    
    // 设置自动功能
    auto auto_exp_result = camera_->set_auto_exposure(auto_exposure);
    auto auto_gain_result = camera_->set_auto_gain(auto_gain);
    
    // 设置黑电平
    auto black_result = camera_->set_black_level(black_level);
    
    // 更新当前参数
    current_params_.exposure_time = exposure;
    current_params_.gain = gain_val;
    current_params_.width = width;
    current_params_.height = height;
    current_params_.pixel_format = pixel_format;
    current_params_.trigger_mode = trigger_mode_val;
    
    // 输出配置结果
    set_output("config_success", Data(success));
    
    OVF_INFO() << "CameraConfigNode executed, success: " << success;
    return Result<void>::success();
}

Result<void> CameraConfigNode::reset() {
    clear_error();
    return Result<void>::success();
}

void CameraConfigNode::set_camera(hal::ICamera::Ptr camera) {
    camera_ = camera;
}

hal::ICamera::Ptr CameraConfigNode::get_camera() const {
    return camera_;
}

Result<void> CameraConfigNode::configure_exposure(double us) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    return camera_->set_exposure(us);
}

Result<void> CameraConfigNode::configure_gain(double gain) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    return camera_->set_gain(gain);
}

Result<void> CameraConfigNode::configure_trigger_mode(TriggerMode mode) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    return camera_->set_trigger_mode(mode);
}

Result<void> CameraConfigNode::configure_roi(uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    
    hal::ROIConfig roi;
    roi.offset_x = x;
    roi.offset_y = y;
    roi.width = width;
    roi.height = height;
    
    return camera_->set_roi(roi);
}

Result<void> CameraConfigNode::configure_white_balance(double r, double g, double b) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    return camera_->set_white_balance(r, g, b);
}

Result<void> CameraConfigNode::configure_frame_rate(double fps) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    return camera_->set_frame_rate(fps);
}

Result<void> CameraConfigNode::configure_pixel_format(ImageFormat format) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    
    auto params_result = camera_->get_camera_params();
    if (params_result.is_failure()) {
        return Result<void>::failure(params_result.code(), params_result.message());
    }
    
    auto params = params_result.value();
    params.format = format;
    
    return camera_->set_camera_params(params);
}

Result<void> CameraConfigNode::apply_params(const CameraParamsData& params) {
    if (!camera_) {
        return Result<void>::failure(ErrorCode::DeviceNotFound, "No camera device");
    }
    
    bool success = true;
    
    auto exp_result = configure_exposure(params.exposure_time);
    if (exp_result.is_failure()) success = false;
    
    auto gain_result = configure_gain(params.gain);
    if (gain_result.is_failure()) success = false;
    
    auto trigger_result = configure_trigger_mode(static_cast<TriggerMode>(params.trigger_mode));
    if (trigger_result.is_failure()) success = false;
    
    auto format_result = configure_pixel_format(static_cast<ImageFormat>(params.pixel_format));
    if (format_result.is_failure()) success = false;
    
    auto roi_result = configure_roi(0, 0, params.width, params.height);
    if (roi_result.is_failure()) success = false;
    
    current_params_ = params;
    
    return success ? Result<void>::success() : 
        Result<void>::failure(ErrorCode::DeviceError, "Some parameters failed to apply");
}

Result<CameraParamsData> CameraConfigNode::get_current_params() const {
    return Result<CameraParamsData>::success(current_params_);
}

// ============================================================================
// 插件导出函数
// ============================================================================

OVF_CAMERA_API void init_camera_plugin() {
    // 注册节点
    NodeInfo acquire_info{
        "camera_acquire", 
        "Camera Acquire", 
        "Camera",
        "Acquire images from camera device",
        "0.1.0",
        "OpenVisionFlow Team",
        {},
        {},
        {}
    };
    
    NodeInfo config_info{
        "camera_config", 
        "Camera Config", 
        "Camera",
        "Configure camera parameters",
        "0.1.0",
        "OpenVisionFlow Team",
        {},
        {},
        {}
    };
    
    OVF_INFO() << "Camera plugin initialized";
}

OVF_CAMERA_API Vector<hal::DeviceInfo> get_available_cameras() {
    Vector<hal::DeviceInfo> cameras;
    
    // 返回一个模拟相机信息
    hal::DeviceInfo info;
    info.id = "generic_camera_0";
    info.name = "Generic Camera 0";
    info.vendor = "OpenVisionFlow";
    info.model = "Generic";
    info.serial_number = "SN001";
    info.driver_type = "generic_camera";
    info.firmware_version = "1.0.0";
    info.description = "Generic camera driver for testing";
    
    cameras.push_back(info);
    
    return cameras;
}

OVF_CAMERA_API hal::ICamera::Ptr create_camera(const String& driver_type, const hal::DeviceInfo& info) {
    if (driver_type == "generic_camera") {
        return std::make_shared<GenericCameraDriver>(info);
    }
    return nullptr;
}

// 注册节点到工厂
namespace {
    struct CameraNodeRegistrar {
        CameraNodeRegistrar() {
            // 注册CameraAcquireNode
            ovf::NodeFactory::instance().register_node("camera_acquire",
                [](const ovf::String& id) -> ovf::INode::Ptr {
                    return std::make_shared<CameraAcquireNode>(id);
                },
                ovf::NodeInfo{
                    "camera_acquire", 
                    "Camera Acquire", 
                    "Camera",
                    "Acquire images from camera device",
                    "0.1.0",
                    "OpenVisionFlow Team",
                    {},
                    {},
                    {}
                });
            
            // 注册CameraConfigNode
            ovf::NodeFactory::instance().register_node("camera_config",
                [](const ovf::String& id) -> ovf::INode::Ptr {
                    return std::make_shared<CameraConfigNode>(id);
                },
                ovf::NodeInfo{
                    "camera_config", 
                    "Camera Config", 
                    "Camera",
                    "Configure camera parameters",
                    "0.1.0",
                    "OpenVisionFlow Team",
                    {},
                    {},
                    {}
                });
        }
    } camera_node_registrar;
}

} // namespace plugin
} // namespace ovf