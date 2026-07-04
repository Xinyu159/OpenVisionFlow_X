/**
 * @file generic_camera_driver.cpp
 * @brief 通用相机驱动实现 - 基于GenICam标准
 * 参考GB/T 33478-2016 机器视觉相机接口规范
 */

#include "ovf/plugin/camera_plugin.h"
#include "ovf/core/logger.h"
#include <chrono>
#include <cstring>
#include <algorithm>

namespace ovf {
namespace plugin {

GenericCameraDriver::GenericCameraDriver(const hal::DeviceInfo& info)
    : info_(info) {
    // 设置默认参数
    params_.width = 1280;
    params_.height = 720;
    params_.offset_x = 0;
    params_.offset_y = 0;
    params_.format = ImageFormat::BGR8;
    params_.trigger_mode = TriggerMode::Continuous;
    params_.exposure_time = 10000.0;
    params_.gain = 1.0;
    params_.frame_rate = 30.0;
    params_.auto_exposure = false;
    params_.auto_gain = false;
    params_.black_level = 0.0;
    
    // 默认缓冲区配置
    buffer_config_.buffer_count = 5;
    buffer_config_.max_queue_size = 10;
    buffer_config_.use_ring_buffer = true;
    buffer_config_.timeout_ms = 1000;
    
    // 初始化GenICam特性列表
    initialize_genicam_features();
}

GenericCameraDriver::~GenericCameraDriver() {
    stop_capture();
    close();
}

void GenericCameraDriver::initialize_genicam_features() {
    // GenICam SFNC标准特性 - 参考：GB/T 33478-2016
    
    // Root分类
    genicam_features_.push_back({
        "Root", "Root", hal::GenICamNodeType::Category,
        "Root", "Root category for all features", true, false
    });
    
    // Image Control分类
    genicam_features_.push_back({
        "ImageControl", "Image Control", hal::GenICamNodeType::Category,
        "Root", "Image control features", true, false
    });
    
    // Width
    genicam_features_.push_back({
        "Width", "Width", hal::GenICamNodeType::Integer,
        "ImageControl", "Width of the image in pixels", true, true,
        1.0, 4096.0, 1.0, "px", {}, std::to_string(params_.width)
    });
    
    // Height
    genicam_features_.push_back({
        "Height", "Height", hal::GenICamNodeType::Integer,
        "ImageControl", "Height of the image in pixels", true, true,
        1.0, 4096.0, 1.0, "px", {}, std::to_string(params_.height)
    });
    
    // OffsetX
    genicam_features_.push_back({
        "OffsetX", "Offset X", hal::GenICamNodeType::Integer,
        "ImageControl", "Horizontal offset from the origin to the ROI", true, true,
        0.0, 4096.0, 1.0, "px", {}, std::to_string(params_.offset_x)
    });
    
    // OffsetY
    genicam_features_.push_back({
        "OffsetY", "Offset Y", hal::GenICamNodeType::Integer,
        "ImageControl", "Vertical offset from the origin to the ROI", true, true,
        0.0, 4096.0, 1.0, "px", {}, std::to_string(params_.offset_y)
    });
    
    // PixelFormat枚举
    genicam_features_.push_back({
        "PixelFormat", "Pixel Format", hal::GenICamNodeType::Enumeration,
        "ImageControl", "Format of the pixels provided by the camera", true, true,
        0.0, 0.0, 1.0, "", 
        {"Mono8", "Mono16", "RGB8", "BGR8", "RGBA8", "BGRA8"},
        "BGR8"
    });
    
    // Acquisition Control分类
    genicam_features_.push_back({
        "AcquisitionControl", "Acquisition Control", hal::GenICamNodeType::Category,
        "Root", "Acquisition control features", true, false
    });
    
    // AcquisitionMode
    genicam_features_.push_back({
        "AcquisitionMode", "Acquisition Mode", hal::GenICamNodeType::Enumeration,
        "AcquisitionControl", "Acquisition mode of the device", true, true,
        0.0, 0.0, 1.0, "",
        {"Continuous", "SingleFrame"},
        "Continuous"
    });
    
    // TriggerSelector
    genicam_features_.push_back({
        "TriggerSelector", "Trigger Selector", hal::GenICamNodeType::Enumeration,
        "AcquisitionControl", "Selects the trigger type", true, true,
        0.0, 0.0, 1.0, "",
        {"FrameStart", "AcquisitionStart"},
        "FrameStart"
    });
    
    // TriggerMode
    genicam_features_.push_back({
        "TriggerMode", "Trigger Mode", hal::GenICamNodeType::Enumeration,
        "AcquisitionControl", "Trigger mode", true, true,
        0.0, 0.0, 1.0, "",
        {"Off", "On"},
        "Off"
    });
    
    // TriggerSource
    genicam_features_.push_back({
        "TriggerSource", "Trigger Source", hal::GenICamNodeType::Enumeration,
        "AcquisitionControl", "Trigger source", true, true,
        0.0, 0.0, 1.0, "",
        {"Software", "HardwareLine0", "HardwareLine1"},
        "Software"
    });
    
    // Analog Control分类
    genicam_features_.push_back({
        "AnalogControl", "Analog Control", hal::GenICamNodeType::Category,
        "Root", "Analog control features", true, false
    });
    
    // ExposureTime
    genicam_features_.push_back({
        "ExposureTime", "Exposure Time", hal::GenICamNodeType::Float,
        "AnalogControl", "Exposure time in microseconds", true, true,
        10.0, 10000000.0, 1.0, "us", {}, std::to_string(exposure_us_)
    });
    
    // ExposureAuto
    genicam_features_.push_back({
        "ExposureAuto", "Exposure Auto", hal::GenICamNodeType::Enumeration,
        "AnalogControl", "Automatic exposure mode", true, true,
        0.0, 0.0, 1.0, "",
        {"Off", "Once", "Continuous"},
        "Off"
    });
    
    // Gain
    genicam_features_.push_back({
        "Gain", "Gain", hal::GenICamNodeType::Float,
        "AnalogControl", "Gain value", true, true,
        0.0, 100.0, 0.1, "dB", {}, std::to_string(gain_)
    });
    
    // GainAuto
    genicam_features_.push_back({
        "GainAuto", "Gain Auto", hal::GenICamNodeType::Enumeration,
        "AnalogControl", "Automatic gain mode", true, true,
        0.0, 0.0, 1.0, "",
        {"Off", "Once", "Continuous"},
        "Off"
    });
    
    // BlackLevel
    genicam_features_.push_back({
        "BlackLevel", "Black Level", hal::GenICamNodeType::Float,
        "AnalogControl", "Black level offset", true, true,
        0.0, 255.0, 1.0, "", {}, std::to_string(black_level_)
    });
    
    // BalanceWhiteControl
    genicam_features_.push_back({
        "BalanceWhiteControl", "Balance White Control", hal::GenICamNodeType::Category,
        "Root", "White balance control features", true, false
    });
    
    // BalanceRatio
    genicam_features_.push_back({
        "BalanceRatio", "Balance Ratio", hal::GenICamNodeType::Float,
        "BalanceWhiteControl", "White balance ratio", true, true,
        0.0, 10.0, 0.01, "", {}, "1.0"
    });
    
    // BalanceWhiteAuto
    genicam_features_.push_back({
        "BalanceWhiteAuto", "Balance White Auto", hal::GenICamNodeType::Enumeration,
        "BalanceWhiteControl", "Automatic white balance mode", true, true,
        0.0, 0.0, 1.0, "",
        {"Off", "Once", "Continuous"},
        "Off"
    });
    
    // Transport Layer分类
    genicam_features_.push_back({
        "TransportLayer", "Transport Layer", hal::GenICamNodeType::Category,
        "Root", "Transport layer features", true, false
    });
    
    // PayloadSize
    genicam_features_.push_back({
        "PayloadSize", "Payload Size", hal::GenICamNodeType::Integer,
        "TransportLayer", "Size of the payload in bytes", true, false,
        0.0, 0.0, 1.0, "B", {}, std::to_string(params_.width * params_.height * 3)
    });
}

Result<void> GenericCameraDriver::open() {
    if (is_open_) {
        return Result<void>::success();
    }
    
    is_open_ = true;
    state_ = hal::DeviceState::Ready;
    OVF_INFO() << "GenericCameraDriver opened: " << info_.name;
    return Result<void>::success();
}

Result<void> GenericCameraDriver::close() {
    stop_capture();
    is_open_ = false;
    state_ = hal::DeviceState::Offline;
    return Result<void>::success();
}

Result<void> GenericCameraDriver::set_param(const String& key, const Data& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (key == "width") params_.width = value.as_int();
    else if (key == "height") params_.height = value.as_int();
    else if (key == "offset_x") params_.offset_x = value.as_int();
    else if (key == "offset_y") params_.offset_y = value.as_int();
    else if (key == "exposure") exposure_us_ = value.as_number();
    else if (key == "gain") gain_ = value.as_number();
    else if (key == "frame_rate") {
        frame_rate_ = value.as_number();
        params_.frame_rate = frame_rate_;
    }
    else if (key == "black_level") black_level_ = value.as_number();
    else if (key == "trigger_mode") params_.trigger_mode = static_cast<TriggerMode>(value.as_int());
    else if (key == "pixel_format") params_.format = static_cast<ImageFormat>(value.as_int());
    else if (key == "auto_exposure") {
        auto_exposure_ = value.as_bool();
        params_.auto_exposure = auto_exposure_;
    }
    else if (key == "auto_gain") {
        auto_gain_ = value.as_bool();
        params_.auto_gain = auto_gain_;
    }
    else {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Unknown parameter: " + key);
    }
    
    return Result<void>::success();
}

Result<Data> GenericCameraDriver::get_param(const String& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (key == "width") return Result<Data>::success(Data(static_cast<int>(params_.width)));
    if (key == "height") return Result<Data>::success(Data(static_cast<int>(params_.height)));
    if (key == "offset_x") return Result<Data>::success(Data(static_cast<int>(params_.offset_x)));
    if (key == "offset_y") return Result<Data>::success(Data(static_cast<int>(params_.offset_y)));
    if (key == "exposure") return Result<Data>::success(Data(exposure_us_));
    if (key == "gain") return Result<Data>::success(Data(gain_));
    if (key == "frame_rate") return Result<Data>::success(Data(frame_rate_));
    if (key == "black_level") return Result<Data>::success(Data(black_level_));
    if (key == "trigger_mode") return Result<Data>::success(Data(static_cast<int>(params_.trigger_mode)));
    if (key == "pixel_format") return Result<Data>::success(Data(static_cast<int>(params_.format)));
    if (key == "auto_exposure") return Result<Data>::success(Data(auto_exposure_));
    if (key == "auto_gain") return Result<Data>::success(Data(auto_gain_));
    
    return Result<Data>::failure(ErrorCode::InvalidParameter, "Unknown parameter: " + key);
}

Result<void> GenericCameraDriver::set_camera_params(const CameraParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    exposure_us_ = params.exposure_time;
    gain_ = params.gain;
    frame_rate_ = params.frame_rate;
    black_level_ = params.black_level;
    auto_exposure_ = params.auto_exposure;
    auto_gain_ = params.auto_gain;
    return Result<void>::success();
}

Result<GenericCameraDriver::CameraParams> GenericCameraDriver::get_camera_params() const {
    std::lock_guard<std::mutex> lock(mutex_);
    CameraParams params = params_;
    params.exposure_time = exposure_us_;
    params.gain = gain_;
    params.frame_rate = frame_rate_;
    params.black_level = black_level_;
    params.auto_exposure = auto_exposure_;
    params.auto_gain = auto_gain_;
    params.white_balance = white_balance_;
    return Result<CameraParams>::success(params);
}

Result<void> GenericCameraDriver::start_capture() {
    if (!is_open_) {
        return Result<void>::failure(ErrorCode::DeviceOpenFailed, "Camera not opened");
    }
    
    if (capturing_) {
        return Result<void>::success();
    }
    
    capturing_ = true;
    state_ = hal::DeviceState::Running;
    capture_thread_ = std::thread(&GenericCameraDriver::capture_thread_func, this);
    
    OVF_INFO() << "GenericCameraDriver started capturing";
    return Result<void>::success();
}

Result<void> GenericCameraDriver::stop_capture() {
    capturing_ = false;
    state_ = hal::DeviceState::Ready;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    clear_buffer_queue();
    
    return Result<void>::success();
}

Result<ImageData> GenericCameraDriver::capture_frame(uint32_t timeout_ms) {
    if (!is_open_) {
        return Result<ImageData>::failure(ErrorCode::CameraCaptureFailed, "Camera not opened");
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (params_.trigger_mode == TriggerMode::Software) {
        // 软触发模式下等待触发
        if (frame_queue_.empty()) {
            cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
        }
    } else if (params_.trigger_mode == TriggerMode::Continuous) {
        // 连续模式下等待帧
        if (frame_queue_.empty()) {
            cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
        }
    }
    
    if (frame_queue_.empty()) {
        return Result<ImageData>::failure(ErrorCode::TriggerTimeout, "Frame timeout");
    }
    
    ImageData frame = frame_queue_.front();
    frame_queue_.pop();
    
    return Result<ImageData>::success(frame);
}

Result<void> GenericCameraDriver::set_trigger_mode(TriggerMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_.trigger_mode = mode;
    return Result<void>::success();
}

Result<void> GenericCameraDriver::send_soft_trigger() {
    if (params_.trigger_mode != TriggerMode::Software) {
        return Result<void>::failure(ErrorCode::NotSupported, "Not in software trigger mode");
    }
    
    generate_test_frame();
    return Result<void>::success();
}

TriggerMode GenericCameraDriver::get_trigger_mode() const {
    return params_.trigger_mode;
}

void GenericCameraDriver::set_image_callback(ImageCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    image_callback_ = callback;
}

bool GenericCameraDriver::has_feature(const String& feature) const {
    for (const auto& f : genicam_features_) {
        if (f.name == feature) return true;
    }
    return false;
}

Vector<String> GenericCameraDriver::get_features() const {
    Vector<String> features;
    for (const auto& f : genicam_features_) {
        if (f.node_type != hal::GenICamNodeType::Category) {
            features.push_back(f.name);
        }
    }
    return features;
}

Vector<hal::GenICamFeatureInfo> GenericCameraDriver::get_genicam_features() const {
    return genicam_features_;
}

hal::GenICamFeatureInfo GenericCameraDriver::get_feature_info(const String& feature_name) const {
    for (const auto& f : genicam_features_) {
        if (f.name == feature_name) return f;
    }
    return hal::GenICamFeatureInfo{};
}

Result<void> GenericCameraDriver::set_feature_value(const String& feature_name, const Data& value) {
    // 根据GenICam特性名设置参数
    if (feature_name == "Width") {
        params_.width = value.as_int();
    } else if (feature_name == "Height") {
        params_.height = value.as_int();
    } else if (feature_name == "OffsetX") {
        params_.offset_x = value.as_int();
        roi_config_.offset_x = params_.offset_x;
    } else if (feature_name == "OffsetY") {
        params_.offset_y = value.as_int();
        roi_config_.offset_y = params_.offset_y;
    } else if (feature_name == "ExposureTime") {
        exposure_us_ = value.as_number();
        params_.exposure_time = exposure_us_;
    } else if (feature_name == "Gain") {
        gain_ = value.as_number();
        params_.gain = gain_;
    } else if (feature_name == "BlackLevel") {
        black_level_ = value.as_number();
        params_.black_level = black_level_;
    } else if (feature_name == "TriggerMode") {
        String mode_str = value.as_string();
        if (mode_str == "On") {
            // TriggerMode On，需要设置TriggerSource
        } else if (mode_str == "Off") {
            params_.trigger_mode = TriggerMode::Continuous;
        }
    } else if (feature_name == "TriggerSource") {
        String source_str = value.as_string();
        if (source_str == "Software") {
            params_.trigger_mode = TriggerMode::Software;
        } else if (source_str.find("Hardware") != String::npos) {
            params_.trigger_mode = TriggerMode::Hardware;
        }
    } else if (feature_name == "ExposureAuto") {
        String auto_str = value.as_string();
        auto_exposure_ = (auto_str != "Off");
        params_.auto_exposure = auto_exposure_;
    } else if (feature_name == "GainAuto") {
        String auto_str = value.as_string();
        auto_gain_ = (auto_str != "Off");
        params_.auto_gain = auto_gain_;
    } else if (feature_name == "PixelFormat") {
        String format_str = value.as_string();
        if (format_str == "Mono8") params_.format = ImageFormat::Mono8;
        else if (format_str == "Mono16") params_.format = ImageFormat::Mono16;
        else if (format_str == "RGB8") params_.format = ImageFormat::RGB8;
        else if (format_str == "BGR8") params_.format = ImageFormat::BGR8;
        else if (format_str == "RGBA8") params_.format = ImageFormat::RGBA8;
        else if (format_str == "BGRA8") params_.format = ImageFormat::BGRA8;
    } else if (feature_name == "BalanceRatio") {
        // BalanceRatio通常需要先选择BalanceRatioSelector
    } else if (feature_name == "BalanceWhiteAuto") {
        String auto_str = value.as_string();
        white_balance_.auto_white_balance = (auto_str != "Off");
    } else {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Unknown or read-only feature: " + feature_name);
    }
    
    // 触发特性变更回调
    if (feature_callback_) {
        feature_callback_(feature_name, value);
    }
    
    return Result<void>::success();
}

Result<Data> GenericCameraDriver::get_feature_value(const String& feature_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (feature_name == "Width") return Result<Data>::success(Data(static_cast<int>(params_.width)));
    if (feature_name == "Height") return Result<Data>::success(Data(static_cast<int>(params_.height)));
    if (feature_name == "OffsetX") return Result<Data>::success(Data(static_cast<int>(params_.offset_x)));
    if (feature_name == "OffsetY") return Result<Data>::success(Data(static_cast<int>(params_.offset_y)));
    if (feature_name == "ExposureTime") return Result<Data>::success(Data(exposure_us_));
    if (feature_name == "Gain") return Result<Data>::success(Data(gain_));
    if (feature_name == "BlackLevel") return Result<Data>::success(Data(black_level_));
    if (feature_name == "TriggerMode") {
        return Result<Data>::success(Data(params_.trigger_mode == TriggerMode::Continuous ? "Off" : "On"));
    }
    if (feature_name == "TriggerSource") {
        switch (params_.trigger_mode) {
            case TriggerMode::Software: return Result<Data>::success(Data("Software"));
            case TriggerMode::Hardware: return Result<Data>::success(Data("HardwareLine0"));
            default: return Result<Data>::success(Data("Software"));
        }
    }
    if (feature_name == "ExposureAuto") {
        return Result<Data>::success(Data(auto_exposure_ ? "Continuous" : "Off"));
    }
    if (feature_name == "GainAuto") {
        return Result<Data>::success(Data(auto_gain_ ? "Continuous" : "Off"));
    }
    if (feature_name == "PixelFormat") {
        switch (params_.format) {
            case ImageFormat::Mono8: return Result<Data>::success(Data("Mono8"));
            case ImageFormat::Mono16: return Result<Data>::success(Data("Mono16"));
            case ImageFormat::RGB8: return Result<Data>::success(Data("RGB8"));
            case ImageFormat::BGR8: return Result<Data>::success(Data("BGR8"));
            case ImageFormat::RGBA8: return Result<Data>::success(Data("RGBA8"));
            case ImageFormat::BGRA8: return Result<Data>::success(Data("BGRA8"));
            default: return Result<Data>::success(Data("BGR8"));
        }
    }
    if (feature_name == "PayloadSize") {
        uint32_t channels = 3;
        if (params_.format == ImageFormat::Mono8 || params_.format == ImageFormat::Mono16) channels = 1;
        return Result<Data>::success(Data(static_cast<int>(params_.width * params_.height * channels)));
    }
    
    return Result<Data>::failure(ErrorCode::InvalidParameter, "Unknown feature: " + feature_name);
}

Result<void> GenericCameraDriver::set_exposure(double us) {
    std::lock_guard<std::mutex> lock(mutex_);
    exposure_us_ = us;
    params_.exposure_time = us;
    
    if (feature_callback_) {
        feature_callback_("ExposureTime", Data(us));
    }
    
    return Result<void>::success();
}

Result<double> GenericCameraDriver::get_exposure() const {
    return Result<double>::success(exposure_us_);
}

Result<void> GenericCameraDriver::set_gain(double gain) {
    std::lock_guard<std::mutex> lock(mutex_);
    gain_ = gain;
    params_.gain = gain;
    
    if (feature_callback_) {
        feature_callback_("Gain", Data(gain));
    }
    
    return Result<void>::success();
}

Result<double> GenericCameraDriver::get_gain() const {
    return Result<double>::success(gain_);
}

Result<void> GenericCameraDriver::set_white_balance(double r, double g, double b) {
    std::lock_guard<std::mutex> lock(mutex_);
    white_balance_.red_ratio = r;
    white_balance_.green_ratio = g;
    white_balance_.blue_ratio = b;
    params_.white_balance = white_balance_;
    
    if (feature_callback_) {
        feature_callback_("BalanceRatio", Data(r)); // 简化处理
    }
    
    return Result<void>::success();
}

Result<hal::WhiteBalanceParams> GenericCameraDriver::get_white_balance() const {
    return Result<hal::WhiteBalanceParams>::success(white_balance_);
}

Result<void> GenericCameraDriver::set_roi(const hal::ROIConfig& roi) {
    std::lock_guard<std::mutex> lock(mutex_);
    roi_config_ = roi;
    params_.offset_x = roi.offset_x;
    params_.offset_y = roi.offset_y;
    params_.width = roi.width;
    params_.height = roi.height;
    return Result<void>::success();
}

Result<hal::ROIConfig> GenericCameraDriver::get_roi() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Result<hal::ROIConfig>::success(roi_config_);
}

Result<void> GenericCameraDriver::set_buffer_config(const hal::BufferConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_config_ = config;
    return Result<void>::success();
}

Result<hal::BufferConfig> GenericCameraDriver::get_buffer_config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Result<hal::BufferConfig>::success(buffer_config_);
}

void GenericCameraDriver::clear_buffer_queue() {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_queue_ = std::queue<ImageData>();
}

uint32_t GenericCameraDriver::get_buffer_queue_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(frame_queue_.size());
}

Result<void> GenericCameraDriver::set_frame_rate(double fps) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_rate_ = fps;
    params_.frame_rate = fps;
    return Result<void>::success();
}

Result<double> GenericCameraDriver::get_frame_rate() const {
    return Result<double>::success(frame_rate_);
}

Result<void> GenericCameraDriver::set_black_level(double level) {
    std::lock_guard<std::mutex> lock(mutex_);
    black_level_ = level;
    params_.black_level = level;
    return Result<void>::success();
}

Result<double> GenericCameraDriver::get_black_level() const {
    return Result<double>::success(black_level_);
}

Result<void> GenericCameraDriver::set_auto_exposure(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_exposure_ = enable;
    params_.auto_exposure = enable;
    return Result<void>::success();
}

Result<void> GenericCameraDriver::set_auto_gain(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_gain_ = enable;
    params_.auto_gain = enable;
    return Result<void>::success();
}

Result<void> GenericCameraDriver::set_auto_white_balance(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    white_balance_.auto_white_balance = enable;
    params_.white_balance.auto_white_balance = enable;
    return Result<void>::success();
}

void GenericCameraDriver::set_feature_callback(FeatureCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    feature_callback_ = callback;
}

void GenericCameraDriver::generate_test_frame() {
    // 生成测试图像
    ImageData img_data;
    img_data.width = params_.width;
    img_data.height = params_.height;
    
    uint32_t channels = 1;
    switch (params_.format) {
        case ImageFormat::Mono8: 
            img_data.format = ImageFormat::Mono8; 
            channels = 1;
            break;
        case ImageFormat::Mono16: 
            img_data.format = ImageFormat::Mono16; 
            channels = 2;
            break;
        case ImageFormat::RGB8: 
            img_data.format = ImageFormat::RGB8; 
            channels = 3;
            break;
        case ImageFormat::BGR8: 
            img_data.format = ImageFormat::BGR8; 
            channels = 3;
            break;
        case ImageFormat::RGBA8: 
            img_data.format = ImageFormat::RGBA8; 
            channels = 4;
            break;
        case ImageFormat::BGRA8: 
            img_data.format = ImageFormat::BGRA8; 
            channels = 4;
            break;
        default:
            img_data.format = ImageFormat::BGR8;
            channels = 3;
    }
    
    img_data.channels = channels;
    img_data.data.resize(img_data.width * img_data.height * channels);
    
    // 创建渐变背景
    for (uint32_t y = 0; y < img_data.height; ++y) {
        for (uint32_t x = 0; x < img_data.width; ++x) {
            double brightness = static_cast<double>(y + x) / (img_data.height + img_data.width);
            brightness *= gain_;
            
            // 模拟曝光影响
            uint8_t b = static_cast<uint8_t>(brightness * exposure_us_ / 10000.0 * 255);
            b = static_cast<uint8_t>(std::min(255.0, static_cast<double>(b + black_level_)));
            
            if (channels == 1) {
                img_data.data[y * img_data.width + x] = b;
            } else if (channels >= 3) {
                size_t idx = (y * img_data.width + x) * channels;
                img_data.data[idx] = b;
                img_data.data[idx + 1] = b;
                img_data.data[idx + 2] = b;
                if (channels == 4) {
                    img_data.data[idx + 3] = 255;
                }
            }
        }
    }
    
    // 添加圆形标记（便于测试）
    uint32_t cx = img_data.width / 2;
    uint32_t cy = img_data.height / 2;
    uint32_t r = std::min(img_data.width, img_data.height) / 4;
    
    for (uint32_t y = 0; y < img_data.height; ++y) {
        for (uint32_t x = 0; x < img_data.width; ++x) {
            int32_t dx = static_cast<int32_t>(x) - static_cast<int32_t>(cx);
            int32_t dy = static_cast<int32_t>(y) - static_cast<int32_t>(cy);
            if (dx * dx + dy * dy <= static_cast<int32_t>(r * r)) {
                if (channels == 1) {
                    img_data.data[y * img_data.width + x] = 200;
                } else if (channels >= 3) {
                    size_t idx = (y * img_data.width + x) * channels;
                    img_data.data[idx] = 0;
                    img_data.data[idx + 1] = 0;
                    img_data.data[idx + 2] = 255;
                    if (channels == 4) img_data.data[idx + 3] = 255;
                }
            }
        }
    }
    
    img_data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    img_data.frame_id = frame_counter_++;
    img_data.source_id = info_.id;
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_queue_.push(img_data);
        
        // 限制队列大小
        while (frame_queue_.size() > buffer_config_.max_queue_size) {
            frame_queue_.pop();
        }
    }
    
    cv_.notify_one();
    
    if (image_callback_) {
        image_callback_(img_data);
    }
}

void GenericCameraDriver::capture_thread_func() {
    double frame_interval_ms = 1000.0 / frame_rate_;
    
    while (capturing_) {
        if (params_.trigger_mode == TriggerMode::Continuous) {
            generate_test_frame();
        }
        
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int64_t>(frame_interval_ms)));
    }
}

// 注册驱动
OVF_REGISTER_CAMERA_DRIVER(GenericCameraDriver, "generic_camera");

} // namespace plugin
} // namespace ovf