/**
 * @file mock_camera.cpp
 * @brief 模拟相机实现（不依赖OpenCV）
 */

#include "ovf/plugins/mock_camera.h"
#include "ovf/core/logger.h"
#include <chrono>
#include <cstring>
#include <algorithm>

namespace ovf {
namespace plugins {

MockCamera::MockCamera(const hal::DeviceInfo& info)
    : info_(info) {
    // 设置默认参数
    params_.width = 1280;
    params_.height = 720;
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
}

MockCamera::~MockCamera() {
    stop_capture();
    close();
}

Result<void> MockCamera::open() {
    if (is_open_) {
        return Result<void>::success();
    }
    
    is_open_ = true;
    state_ = hal::DeviceState::Ready;
    OVF_INFO() << "MockCamera opened: " << info_.name;
    return Result<void>::success();
}

Result<void> MockCamera::close() {
    stop_capture();
    is_open_ = false;
    state_ = hal::DeviceState::Offline;
    return Result<void>::success();
}

Result<void> MockCamera::set_param(const String& key, const Data& value) {
    if (key == "width") params_.width = value.as_int();
    else if (key == "height") params_.height = value.as_int();
    else if (key == "exposure") exposure_us_ = value.as_number();
    else if (key == "gain") gain_ = value.as_number();
    else if (key == "frame_rate") params_.frame_rate = value.as_number();
    else if (key == "black_level") black_level_ = value.as_number();
    else if (key == "auto_exposure") {
        auto_exposure_ = value.as_bool();
        params_.auto_exposure = auto_exposure_;
    }
    else if (key == "auto_gain") {
        auto_gain_ = value.as_bool();
        params_.auto_gain = auto_gain_;
    }
    return Result<void>::success();
}

Result<Data> MockCamera::get_param(const String& key) const {
    if (key == "width") return Result<Data>::success(Data(static_cast<int>(params_.width)));
    if (key == "height") return Result<Data>::success(Data(static_cast<int>(params_.height)));
    if (key == "exposure") return Result<Data>::success(Data(exposure_us_));
    if (key == "gain") return Result<Data>::success(Data(gain_));
    if (key == "frame_rate") return Result<Data>::success(Data(params_.frame_rate));
    if (key == "black_level") return Result<Data>::success(Data(black_level_));
    if (key == "auto_exposure") return Result<Data>::success(Data(auto_exposure_));
    if (key == "auto_gain") return Result<Data>::success(Data(auto_gain_));
    return Result<Data>::failure(ErrorCode::InvalidParameter, "Unknown parameter: " + key);
}

Result<void> MockCamera::set_camera_params(const CameraParams& params) {
    params_ = params;
    exposure_us_ = params.exposure_time;
    gain_ = params.gain;
    return Result<void>::success();
}

Result<MockCamera::CameraParams> MockCamera::get_camera_params() const {
    CameraParams params = params_;
    params.exposure_time = exposure_us_;
    params.gain = gain_;
    params.black_level = black_level_;
    params.auto_exposure = auto_exposure_;
    params.auto_gain = auto_gain_;
    params.white_balance = white_balance_;
    return Result<CameraParams>::success(params);
}

Result<void> MockCamera::start_capture() {
    if (!is_open_) {
        return Result<void>::failure(ErrorCode::DeviceOpenFailed, "Camera not opened");
    }
    
    if (capturing_) {
        return Result<void>::success();
    }
    
    capturing_ = true;
    state_ = hal::DeviceState::Running;
    capture_thread_ = std::thread(&MockCamera::capture_thread, this);
    
    OVF_INFO() << "MockCamera started capturing";
    return Result<void>::success();
}

Result<void> MockCamera::stop_capture() {
    capturing_ = false;
    state_ = hal::DeviceState::Ready;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    // 清空队列
    std::lock_guard<std::mutex> lock(mutex_);
    frame_queue_ = std::queue<ImageData>();
    
    return Result<void>::success();
}

Result<ImageData> MockCamera::capture_frame(uint32_t timeout_ms) {
    if (!is_open_ || !capturing_) {
        return Result<ImageData>::failure(ErrorCode::CameraCaptureFailed, "Not capturing");
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (params_.trigger_mode == TriggerMode::Software) {
        // 等待触发
        return Result<ImageData>::failure(ErrorCode::TriggerTimeout, "Waiting for trigger");
    }
    
    // 等待帧
    if (frame_queue_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    }
    
    if (frame_queue_.empty()) {
        return Result<ImageData>::failure(ErrorCode::TriggerTimeout, "Frame timeout");
    }
    
    ImageData frame = frame_queue_.front();
    frame_queue_.pop();
    
    return Result<ImageData>::success(frame);
}

Result<void> MockCamera::set_trigger_mode(TriggerMode mode) {
    params_.trigger_mode = mode;
    return Result<void>::success();
}

Result<void> MockCamera::send_soft_trigger() {
    if (params_.trigger_mode != TriggerMode::Software) {
        return Result<void>::failure(ErrorCode::NotSupported, "Not in software trigger mode");
    }
    
    // 生成一帧
    generate_frame();
    
    return Result<void>::success();
}

TriggerMode MockCamera::get_trigger_mode() const {
    return params_.trigger_mode;
}

void MockCamera::set_image_callback(ImageCallback callback) {
    image_callback_ = callback;
}

bool MockCamera::has_feature(const String& feature) const {
    static const Vector<String> features = {
        "width", "height", "exposure", "gain", "frame_rate",
        "trigger_mode", "trigger_source", "auto_exposure", "auto_gain",
        "black_level", "white_balance"
    };
    return std::find(features.begin(), features.end(), feature) != features.end();
}

Vector<String> MockCamera::get_features() const {
    return {"width", "height", "exposure", "gain", "frame_rate",
            "trigger_mode", "trigger_source", "black_level", "white_balance"};
}

Vector<hal::GenICamFeatureInfo> MockCamera::get_genicam_features() const {
    return genicam_features_;
}

hal::GenICamFeatureInfo MockCamera::get_feature_info(const String& feature_name) const {
    for (const auto& f : genicam_features_) {
        if (f.name == feature_name) return f;
    }
    return hal::GenICamFeatureInfo{};
}

Result<void> MockCamera::set_feature_value(const String& feature_name, const Data& value) {
    if (feature_name == "ExposureTime") {
        exposure_us_ = value.as_number();
        params_.exposure_time = exposure_us_;
    } else if (feature_name == "Gain") {
        gain_ = value.as_number();
        params_.gain = gain_;
    } else if (feature_name == "BlackLevel") {
        black_level_ = value.as_number();
        params_.black_level = black_level_;
    } else if (feature_name == "Width") {
        params_.width = value.as_int();
    } else if (feature_name == "Height") {
        params_.height = value.as_int();
    } else {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Unknown or read-only feature: " + feature_name);
    }
    
    if (feature_callback_) {
        feature_callback_(feature_name, value);
    }
    
    return Result<void>::success();
}

Result<Data> MockCamera::get_feature_value(const String& feature_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (feature_name == "ExposureTime") return Result<Data>::success(Data(exposure_us_));
    if (feature_name == "Gain") return Result<Data>::success(Data(gain_));
    if (feature_name == "BlackLevel") return Result<Data>::success(Data(black_level_));
    if (feature_name == "Width") return Result<Data>::success(Data(static_cast<int>(params_.width)));
    if (feature_name == "Height") return Result<Data>::success(Data(static_cast<int>(params_.height)));
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
    
    return Result<Data>::failure(ErrorCode::InvalidParameter, "Unknown feature: " + feature_name);
}

Result<void> MockCamera::set_exposure(double us) {
    exposure_us_ = us;
    params_.exposure_time = us;
    
    if (feature_callback_) {
        feature_callback_("ExposureTime", Data(us));
    }
    
    return Result<void>::success();
}

Result<double> MockCamera::get_exposure() const {
    return Result<double>::success(exposure_us_);
}

Result<void> MockCamera::set_gain(double gain) {
    gain_ = gain;
    params_.gain = gain;
    
    if (feature_callback_) {
        feature_callback_("Gain", Data(gain));
    }
    
    return Result<void>::success();
}

Result<double> MockCamera::get_gain() const {
    return Result<double>::success(gain_);
}

Result<void> MockCamera::set_white_balance(double r, double g, double b) {
    white_balance_.red_ratio = r;
    white_balance_.green_ratio = g;
    white_balance_.blue_ratio = b;
    params_.white_balance = white_balance_;
    return Result<void>::success();
}

Result<hal::WhiteBalanceParams> MockCamera::get_white_balance() const {
    return Result<hal::WhiteBalanceParams>::success(white_balance_);
}

Result<void> MockCamera::set_roi(const hal::ROIConfig& roi) {
    roi_config_ = roi;
    params_.offset_x = roi.offset_x;
    params_.offset_y = roi.offset_y;
    params_.width = roi.width;
    params_.height = roi.height;
    return Result<void>::success();
}

Result<hal::ROIConfig> MockCamera::get_roi() const {
    return Result<hal::ROIConfig>::success(roi_config_);
}

Result<void> MockCamera::set_buffer_config(const hal::BufferConfig& config) {
    buffer_config_ = config;
    return Result<void>::success();
}

Result<hal::BufferConfig> MockCamera::get_buffer_config() const {
    return Result<hal::BufferConfig>::success(buffer_config_);
}

void MockCamera::clear_buffer_queue() {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_queue_ = std::queue<ImageData>();
}

uint32_t MockCamera::get_buffer_queue_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(frame_queue_.size());
}

Result<void> MockCamera::set_frame_rate(double fps) {
    params_.frame_rate = fps;
    return Result<void>::success();
}

Result<double> MockCamera::get_frame_rate() const {
    return Result<double>::success(params_.frame_rate);
}

Result<void> MockCamera::set_black_level(double level) {
    black_level_ = level;
    params_.black_level = level;
    return Result<void>::success();
}

Result<double> MockCamera::get_black_level() const {
    return Result<double>::success(black_level_);
}

Result<void> MockCamera::set_auto_exposure(bool enable) {
    auto_exposure_ = enable;
    params_.auto_exposure = enable;
    return Result<void>::success();
}

Result<void> MockCamera::set_auto_gain(bool enable) {
    auto_gain_ = enable;
    params_.auto_gain = enable;
    return Result<void>::success();
}

Result<void> MockCamera::set_auto_white_balance(bool enable) {
    white_balance_.auto_white_balance = enable;
    params_.white_balance.auto_white_balance = enable;
    return Result<void>::success();
}

void MockCamera::set_feature_callback(FeatureCallback callback) {
    feature_callback_ = callback;
}

void MockCamera::generate_frame() {
    // 生成模拟图像（纯C++实现，不依赖OpenCV）
    ImageData img_data;
    img_data.width = params_.width;
    img_data.height = params_.height;
    img_data.channels = 3;
    img_data.format = ImageFormat::BGR8;
    img_data.data.resize(img_data.width * img_data.height * 3);
    
    // 创建渐变背景
    for (uint32_t y = 0; y < img_data.height; ++y) {
        for (uint32_t x = 0; x < img_data.width; ++x) {
            double brightness = static_cast<double>(y + x) / (img_data.height + img_data.width);
            brightness *= gain_;
            
            // 模拟曝光影响
            uint8_t b = static_cast<uint8_t>(brightness * exposure_us_ / 10000.0 * 255);
            b = static_cast<uint8_t>(std::min(255.0, static_cast<double>(b + black_level_)));
            
            size_t idx = (y * img_data.width + x) * 3;
            img_data.data[idx] = b;     // B
            img_data.data[idx + 1] = b; // G
            img_data.data[idx + 2] = b; // R
        }
    }
    
    // 添加圆形（红色）
    uint32_t cx = img_data.width / 2;
    uint32_t cy = img_data.height / 2;
    uint32_t r = std::min(img_data.width, img_data.height) / 4;
    
    for (uint32_t y = 0; y < img_data.height; ++y) {
        for (uint32_t x = 0; x < img_data.width; ++x) {
            int32_t dx = static_cast<int32_t>(x) - static_cast<int32_t>(cx);
            int32_t dy = static_cast<int32_t>(y) - static_cast<int32_t>(cy);
            if (dx * dx + dy * dy <= static_cast<int32_t>(r * r)) {
                size_t idx = (y * img_data.width + x) * 3;
                img_data.data[idx] = 0;      // B
                img_data.data[idx + 1] = 0;  // G
                img_data.data[idx + 2] = 255; // R
            }
        }
    }
    
    // 添加矩形（绿色）
    uint32_t rect_x1 = img_data.width / 4;
    uint32_t rect_y1 = img_data.height / 4;
    uint32_t rect_x2 = rect_x1 + 100;
    uint32_t rect_y2 = rect_y1 + 100;
    
    for (uint32_t y = rect_y1; y < rect_y2 && y < img_data.height; ++y) {
        for (uint32_t x = rect_x1; x < rect_x2 && x < img_data.width; ++x) {
            // 边框
            if (x == rect_x1 || x == rect_x2 - 1 || y == rect_y1 || y == rect_y2 - 1) {
                size_t idx = (y * img_data.width + x) * 3;
                img_data.data[idx] = 0;      // B
                img_data.data[idx + 1] = 255; // G
                img_data.data[idx + 2] = 0;  // R
            }
        }
    }
    
    img_data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    img_data.frame_id = frame_counter_++;
    img_data.source_id = info_.id;
    
    // 添加到队列
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_queue_.push(img_data);
        
        // 限制队列大小
        while (frame_queue_.size() > 10) {
            frame_queue_.pop();
        }
    }
    
    cv_.notify_one();
    
    // 调用回调
    if (image_callback_) {
        image_callback_(img_data);
    }
}

void MockCamera::capture_thread() {
    double frame_interval_ms = 1000.0 / params_.frame_rate;
    
    while (capturing_) {
        if (params_.trigger_mode == TriggerMode::Continuous) {
            generate_frame();
        }
        
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int64_t>(frame_interval_ms)));
    }
}

// 注册驱动
OVF_REGISTER_CAMERA_DRIVER(MockCamera, "mock_camera");

} // namespace plugins
} // namespace ovf