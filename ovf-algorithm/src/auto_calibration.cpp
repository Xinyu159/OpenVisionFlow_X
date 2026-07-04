/**
 * @file auto_calibration.cpp
 * @brief 自动标定补偿流程模块实现
 */

#include "ovf/algorithm/auto_calibration.h"
#include "ovf/algorithm/template_matching.h"
#include "ovf/core/logger.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>

namespace ovf {
namespace algorithm {

// ========== 全局状态（模块内部） ==========

namespace {
    // 自动标定系统状态
    auto_calibration::AutoCalibConfig g_auto_calib_config;
    std::vector<auto_calibration::CalibCompensation> g_calibration_history;
    auto_calibration::CalibCompensation g_current_compensation;
    int g_correction_count = 0;
    bool g_initialized = false;

    // 基准温度（用于检测温度变化）
    float g_reference_temperature = 25.0f;
}

// ========== calibration_monitor 实现 ==========

namespace calibration_monitor {

ErrorCode monitor_environment(EnvironmentStatus& status) {
    // 获取当前时间戳
    auto now = std::chrono::steady_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();

    // 模拟环境监测（实际应用中应从传感器读取）
    // 这里使用默认值或从参数获取
    status.temperature = 25.0f;       // 默认温度
    status.humidity = 50.0f;          // 默认湿度
    status.vibration_level = 0.0f;    // 默认振动
    status.light_intensity = 100.0f;  // 默认光照

    OVF_DEBUG() << "Environment monitored: temp=" << status.temperature
                << ", humidity=" << status.humidity;

    return ErrorCode::Success;
}

ErrorCode calculate_drift(const ImageData& reference, const ImageData& current,
                          float& drift_x, float& drift_y) {
    if (reference.empty() || current.empty()) {
        return ErrorCode::InvalidImage;
    }

    // 使用模板匹配计算漂移
    // 将基准图像作为模板，在当前图像中搜索匹配位置

    // 转换为灰度图像
    std::vector<uint8_t> ref_gray, cur_gray;
    int ref_w, ref_h, cur_w, cur_h;

    // 处理基准图像
    if (reference.channels == 1) {
        ref_gray = reference.data;
        ref_w = reference.width;
        ref_h = reference.height;
    } else {
        ref_w = reference.width;
        ref_h = reference.height;
        ref_gray.resize(ref_w * ref_h);
        for (size_t i = 0; i < ref_gray.size(); ++i) {
            size_t idx = i * reference.channels;
            ref_gray[i] = static_cast<uint8_t>(
                (reference.data[idx] + reference.data[idx + 1] + reference.data[idx + 2]) / 3);
        }
    }

    // 处理当前图像
    if (current.channels == 1) {
        cur_gray = current.data;
        cur_w = current.width;
        cur_h = current.height;
    } else {
        cur_w = current.width;
        cur_h = current.height;
        cur_gray.resize(cur_w * cur_h);
        for (size_t i = 0; i < cur_gray.size(); ++i) {
            size_t idx = i * current.channels;
            cur_gray[i] = static_cast<uint8_t>(
                (current.data[idx] + current.data[idx + 1] + current.data[idx + 2]) / 3);
        }
    }

    // 使用NCC模板匹配
    MatchResult result;
    ErrorCode err = template_utils::match_ncc(cur_gray.data(), cur_w, cur_h,
                                               ref_gray.data(), ref_w, ref_h, result);

    if (err != ErrorCode::Success) {
        return err;
    }

    // 假设基准图像在当前图像的期望位置是 (0, 0)
    // 漂移量 = 实际匹配位置 - 期望位置
    drift_x = static_cast<float>(result.x);
    drift_y = static_cast<float>(result.y);

    OVF_DEBUG() << "Drift calculated: dx=" << drift_x << ", dy=" << drift_y
                << ", score=" << result.score;

    return ErrorCode::Success;
}

bool check_calibration_needed(const EnvironmentStatus& status,
                              float temp_threshold,
                              float drift_threshold) {
    // 检查温度变化
    float temp_change = std::abs(status.temperature - g_reference_temperature);
    if (temp_change > temp_threshold) {
        OVF_WARN() << "Temperature change exceeds threshold: " << temp_change
                   << " > " << temp_threshold;
        return true;
    }

    // 检查漂移量
    float drift_magnitude = std::sqrt(status.drift_x * status.drift_x +
                                       status.drift_y * status.drift_y);
    if (drift_magnitude > drift_threshold) {
        OVF_WARN() << "Drift exceeds threshold: " << drift_magnitude
                   << " > " << drift_threshold;
        return true;
    }

    return false;
}

} // namespace calibration_monitor

// ========== auto_calibration 实现 ==========

namespace auto_calibration {

ErrorCode init_auto_calibration(const AutoCalibConfig& config) {
    g_auto_calib_config = config;
    g_calibration_history.clear();
    g_correction_count = 0;
    g_initialized = true;

    // 记录初始温度参考
    calibration_monitor::EnvironmentStatus status;
    calibration_monitor::monitor_environment(status);
    g_reference_temperature = status.temperature;

    OVF_INFO() << "Auto calibration system initialized. "
               << "Interval=" << config.check_interval_sec << "s, "
               << "Drift threshold=" << config.drift_threshold << "px";

    return ErrorCode::Success;
}

ErrorCode run_calibration_check(const ImageData& reference,
                                CalibCompensation& compensation) {
    if (!g_initialized) {
        // 使用默认配置初始化
        AutoCalibConfig default_config;
        init_auto_calibration(default_config);
    }

    if (reference.empty()) {
        return ErrorCode::InvalidImage;
    }

    // 获取当前时间戳
    auto now = std::chrono::steady_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();

    // 计算补偿参数（基于基准图像）
    // 这里使用增量式参数调整策略

    // 计算与上次补偿的增量
    float delta_x = g_current_compensation.offset_x;
    float delta_y = g_current_compensation.offset_y;

    // 应用补偿参数
    compensation.offset_x = delta_x;
    compensation.offset_y = delta_y;
    compensation.rotation_comp = 0.0f;
    compensation.scale_comp_x = 1.0f;
    compensation.scale_comp_y = 1.0f;
    compensation.timestamp = timestamp;
    compensation.valid = true;

    // 记录历史
    log_calibration_history(compensation);

    OVF_INFO() << "Calibration check completed. Compensation: "
               << "offset=(" << compensation.offset_x << "," << compensation.offset_y << ")";

    return ErrorCode::Success;
}

ErrorCode apply_compensation(const CalibCompensation& comp) {
    if (!comp.valid) {
        return ErrorCode::InvalidParameter;
    }

    // 应用补偿参数到当前系统状态
    g_current_compensation = comp;
    g_correction_count++;

    OVF_INFO() << "Compensation applied: offset=(" << comp.offset_x << "," << comp.offset_y
               << "), correction count=" << g_correction_count;

    // 检查是否超过最大校正次数
    if (g_correction_count > g_auto_calib_config.max_auto_corrections) {
        OVF_WARN() << "Max auto corrections reached. Manual calibration may be required.";
    }

    return ErrorCode::Success;
}

ErrorCode log_calibration_history(const CalibCompensation& comp) {
    g_calibration_history.push_back(comp);

    // 限制历史记录数量
    if (g_calibration_history.size() > 100) {
        g_calibration_history.erase(g_calibration_history.begin());
    }

    OVF_DEBUG() << "Calibration history logged. Total records: " << g_calibration_history.size();

    return ErrorCode::Success;
}

ErrorCode get_calibration_history(std::vector<CalibCompensation>& history, int max_count) {
    history.clear();

    int count = std::min(max_count, static_cast<int>(g_calibration_history.size()));
    int start_idx = static_cast<int>(g_calibration_history.size()) - count;

    for (int i = start_idx; i < static_cast<int>(g_calibration_history.size()); ++i) {
        history.push_back(g_calibration_history[i]);
    }

    return ErrorCode::Success;
}

} // namespace auto_calibration

// ========== adaptive_params 实现 ==========

namespace adaptive_params {

ErrorCode evaluate_image_quality(const ImageData& image,
                                 float& sharpness,
                                 float& contrast,
                                 float& brightness) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    // 转换为灰度图像
    std::vector<uint8_t> gray;
    int width, height;

    if (image.channels == 1) {
        gray = image.data;
        width = image.width;
        height = image.height;
    } else {
        width = image.width;
        height = image.height;
        gray.resize(width * height);
        for (size_t i = 0; i < gray.size(); ++i) {
            size_t idx = i * image.channels;
            gray[i] = static_cast<uint8_t>(
                (image.data[idx] + image.data[idx + 1] + image.data[idx + 2]) / 3);
        }
    }

    // 计算亮度（平均像素值）
    double sum = 0.0;
    for (auto val : gray) {
        sum += val;
    }
    brightness = static_cast<float>(sum / gray.size());

    // 计算对比度（标准差）
    double variance = 0.0;
    double mean = brightness;
    for (auto val : gray) {
        double diff = val - mean;
        variance += diff * diff;
    }
    contrast = static_cast<float>(std::sqrt(variance / gray.size()));

    // 计算清晰度（梯度能量）
    // 使用Sobel梯度计算
    double gradient_energy = 0.0;
    int count = 0;

    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            // Sobel梯度
            int gx = -gray[(y-1)*width + (x-1)] + gray[(y-1)*width + (x+1)]
                     - 2*gray[y*width + (x-1)] + 2*gray[y*width + (x+1)]
                     - gray[(y+1)*width + (x-1)] + gray[(y+1)*width + (x+1)];

            int gy = -gray[(y-1)*width + (x-1)] - 2*gray[(y-1)*width + x] - gray[(y-1)*width + (x+1)]
                     + gray[(y+1)*width + (x-1)] + 2*gray[(y+1)*width + x] + gray[(y+1)*width + (x+1)];

            gradient_energy += gx * gx + gy * gy;
            count++;
        }
    }

    if (count > 0) {
        sharpness = static_cast<float>(std::sqrt(gradient_energy / count));
    } else {
        sharpness = 0.0f;
    }

    OVF_DEBUG() << "Image quality: sharpness=" << sharpness
                << ", contrast=" << contrast
                << ", brightness=" << brightness;

    return ErrorCode::Success;
}

ErrorCode adjust_params_for_quality(const ImageData& image,
                                    CameraParams& params) {
    float sharpness, contrast, brightness;
    ErrorCode err = evaluate_image_quality(image, sharpness, contrast, brightness);

    if (err != ErrorCode::Success) {
        return err;
    }

    // 根据亮度调整曝光
    double target_brightness = 128.0;  // 目标亮度值
    double brightness_diff = target_brightness - brightness;

    // 调整曝光时间（亮度偏低增加曝光，偏高减少曝光）
    double exposure_adjust = brightness_diff * 100.0;  // 每亮度差调整100us
    params.exposure_us = 10000.0 + exposure_adjust;
    params.exposure_us = std::max(1000.0, std::min(100000.0, params.exposure_us));

    // 根据对比度调整增益
    if (contrast < 30.0) {
        params.gain = 2.0;  // 低对比度时增加增益
    } else if (contrast > 70.0) {
        params.gain = 1.0;  // 高对比度时保持正常增益
    } else {
        params.gain = 1.5;  // 中等对比度
    }

    // 设置亮度和对比度参数
    params.brightness = brightness_diff / 255.0;  // 归一化到 -1 ~ 1
    params.contrast = contrast / 128.0;           // 归一化

    OVF_INFO() << "Camera params adjusted: exposure=" << params.exposure_us
               << "us, gain=" << params.gain;

    return ErrorCode::Success;
}

ErrorCode adaptive_exposure(const ImageData& image, double& exposure_us) {
    float sharpness, contrast, brightness;
    ErrorCode err = evaluate_image_quality(image, sharpness, contrast, brightness);

    if (err != ErrorCode::Success) {
        return err;
    }

    // 根据亮度自适应调整曝光
    double target_brightness = 128.0;
    double current_exposure = 10000.0;  // 基准曝光

    // 计算曝光调整比例
    double ratio = target_brightness / std::max(brightness, 1.0f);
    exposure_us = current_exposure * ratio;

    // 限制曝光范围
    exposure_us = std::max(1000.0, std::min(100000.0, exposure_us));

    OVF_DEBUG() << "Adaptive exposure: " << exposure_us << "us (brightness=" << brightness << ")";

    return ErrorCode::Success;
}

ErrorCode adaptive_gain(const ImageData& image, double& gain) {
    float sharpness, contrast, brightness;
    ErrorCode err = evaluate_image_quality(image, sharpness, contrast, brightness);

    if (err != ErrorCode::Success) {
        return err;
    }

    // 根据对比度和清晰度调整增益
    // 低对比度时增加增益以提高对比度
    // 但清晰度低时减少增益以避免噪声放大

    if (contrast < 30.0) {
        gain = 2.0;
    } else if (contrast > 70.0) {
        gain = 1.0;
    } else {
        gain = 1.0 + (70.0 - contrast) / 40.0;
    }

    // 如果清晰度很低，降低增益避免噪声
    if (sharpness < 10.0) {
        gain = std::min(gain, 1.5);
    }

    OVF_DEBUG() << "Adaptive gain: " << gain << " (contrast=" << contrast << ")";

    return ErrorCode::Success;
}

} // namespace adaptive_params

// ========== 节点实现 ==========

// CalibrationMonitorNode
CalibrationMonitorNode::CalibrationMonitorNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CalibrationMonitorNode::make_info() {
    NodeInfo info;
    info.id = "CalibrationMonitor";
    info.name = "标定监测";
    info.category = "自动标定";
    info.description = "监测环境变化，计算漂移量，判断是否需要重新标定";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("reference_image", "基准图像", DataType::Image, true));
    info.inputs.push_back(DataPort("current_image", "当前图像", DataType::Image, true));

    info.outputs.push_back(DataPort("temperature", "温度", DataType::Number));
    info.outputs.push_back(DataPort("humidity", "湿度", DataType::Number));
    info.outputs.push_back(DataPort("drift_x", "漂移X", DataType::Number));
    info.outputs.push_back(DataPort("drift_y", "漂移Y", DataType::Number));
    info.outputs.push_back(DataPort("needs_calibration", "需要标定", DataType::Boolean));

    info.params.push_back(ParamDef("temp_threshold", "温度阈值", DataType::Number, Data(5.0f)));
    info.params.push_back(ParamDef("drift_threshold", "漂移阈值", DataType::Number, Data(0.5f)));

    return info;
}

Result<void> CalibrationMonitorNode::execute(FlowContext& context) {
    auto ref_data = get_input("reference_image");
    auto cur_data = get_input("current_image");

    if (!ref_data.is_image() || !cur_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Reference and current images are required");
    }

    ImageData ref_image = ref_data.as_image();
    ImageData cur_image = cur_data.as_image();

    if (ref_image.empty() || cur_image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Images cannot be empty");
    }

    // 监测环境
    ErrorCode err = calibration_monitor::monitor_environment(status_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Environment monitoring failed");
    }

    // 计算漂移
    err = calibration_monitor::calculate_drift(ref_image, cur_image,
                                                status_.drift_x, status_.drift_y);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Drift calculation failed");
    }

    // 获取阈值参数
    float temp_threshold = get_param("temp_threshold", Data(5.0f)).as_number();
    float drift_threshold = get_param("drift_threshold", Data(0.5f)).as_number();

    // 判断是否需要标定
    status_.needs_calibration = calibration_monitor::check_calibration_needed(
        status_, temp_threshold, drift_threshold);

    // 设置输出
    set_output("temperature", Data(static_cast<double>(status_.temperature)));
    set_output("humidity", Data(static_cast<double>(status_.humidity)));
    set_output("drift_x", Data(static_cast<double>(status_.drift_x)));
    set_output("drift_y", Data(static_cast<double>(status_.drift_y)));
    set_output("needs_calibration", Data(status_.needs_calibration));

    OVF_INFO() << "Calibration monitor: drift=(" << status_.drift_x << "," << status_.drift_y
               << "), needs_calib=" << status_.needs_calibration;

    return Result<void>::success();
}

// AutoCalibrationNode
AutoCalibrationNode::AutoCalibrationNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AutoCalibrationNode::make_info() {
    NodeInfo info;
    info.id = "AutoCalibration";
    info.name = "自动标定";
    info.category = "自动标定";
    info.description = "执行自动标定检查，计算补偿参数";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("reference_image", "基准图像", DataType::Image, true));
    info.inputs.push_back(DataPort("drift_x", "漂移X", DataType::Number));
    info.inputs.push_back(DataPort("drift_y", "漂移Y", DataType::Number));

    info.outputs.push_back(DataPort("offset_x", "补偿X", DataType::Number));
    info.outputs.push_back(DataPort("offset_y", "补偿Y", DataType::Number));
    info.outputs.push_back(DataPort("rotation", "旋转补偿", DataType::Number));
    info.outputs.push_back(DataPort("scale_x", "缩放X", DataType::Number));
    info.outputs.push_back(DataPort("scale_y", "缩放Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "有效", DataType::Boolean));

    info.params.push_back(ParamDef("check_interval", "检查间隔", DataType::Number, Data(60.0f)));
    info.params.push_back(ParamDef("drift_threshold", "漂移阈值", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("max_corrections", "最大校正次数", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("enable_auto", "启用自动", DataType::Boolean, Data(true)));

    return info;
}

Result<void> AutoCalibrationNode::execute(FlowContext& context) {
    auto ref_data = get_input("reference_image");
    if (!ref_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Reference image is required");
    }

    reference_image_ = ref_data.as_image();
    if (reference_image_.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Reference image is empty");
    }

    // 从输入获取漂移量（如果有）
    auto drift_x_data = get_input("drift_x");
    auto drift_y_data = get_input("drift_y");

    float drift_x = drift_x_data.is_valid() ? drift_x_data.as_number() : 0.0f;
    float drift_y = drift_y_data.is_valid() ? drift_y_data.as_number() : 0.0f;

    // 配置参数
    config_.check_interval_sec = get_param("check_interval", Data(60.0f)).as_number();
    config_.drift_threshold = get_param("drift_threshold", Data(0.5f)).as_number();
    config_.max_auto_corrections = get_param("max_corrections", Data(3)).as_int();
    config_.enable_auto_correction = get_param("enable_auto", Data(true)).as_bool();

    // 初始化系统
    auto_calibration::init_auto_calibration(config_);

    // 运行标定检查
    ErrorCode err = auto_calibration::run_calibration_check(reference_image_, compensation_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Calibration check failed");
    }

    // 设置补偿参数（根据漂移量）
    compensation_.offset_x = -drift_x;  // 反向补偿
    compensation_.offset_y = -drift_y;
    compensation_.valid = true;

    // 自动应用补偿
    if (config_.enable_auto_correction) {
        err = auto_calibration::apply_compensation(compensation_);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "Apply compensation failed");
        }
    }

    // 设置输出
    set_output("offset_x", Data(static_cast<double>(compensation_.offset_x)));
    set_output("offset_y", Data(static_cast<double>(compensation_.offset_y)));
    set_output("rotation", Data(static_cast<double>(compensation_.rotation_comp)));
    set_output("scale_x", Data(static_cast<double>(compensation_.scale_comp_x)));
    set_output("scale_y", Data(static_cast<double>(compensation_.scale_comp_y)));
    set_output("valid", Data(compensation_.valid));

    OVF_INFO() << "Auto calibration: compensation=(" << compensation_.offset_x
               << "," << compensation_.offset_y << ")";

    return Result<void>::success();
}

// DriftCorrectionNode
DriftCorrectionNode::DriftCorrectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DriftCorrectionNode::make_info() {
    NodeInfo info;
    info.id = "DriftCorrection";
    info.name = "漂移校正";
    info.category = "自动标定";
    info.description = "应用漂移校正参数到图像";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("drift_x", "漂移X", DataType::Number));
    info.inputs.push_back(DataPort("drift_y", "漂移Y", DataType::Number));

    info.outputs.push_back(DataPort("image", "校正后图像", DataType::Image));
    info.outputs.push_back(DataPort("corrected_x", "校正X", DataType::Number));
    info.outputs.push_back(DataPort("corrected_y", "校正Y", DataType::Number));

    info.params.push_back(ParamDef("apply_offset", "应用偏移", DataType::Boolean, Data(true)));

    return info;
}

Result<void> DriftCorrectionNode::execute(FlowContext& context) {
    auto image_data = get_input("image");
    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is required");
    }

    ImageData image = image_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取漂移量
    auto drift_x_data = get_input("drift_x");
    auto drift_y_data = get_input("drift_y");

    drift_x_ = drift_x_data.is_valid() ? drift_x_data.as_number() : 0.0f;
    drift_y_ = drift_y_data.is_valid() ? drift_y_data.as_number() : 0.0f;

    bool apply_offset = get_param("apply_offset", Data(true)).as_bool();

    // 计算校正后的位置（反向补偿）
    float corrected_x = -drift_x_;
    float corrected_y = -drift_y_;

    // 设置输出
    set_output("image", Data(image));  // 图像数据本身不变，输出位置信息
    set_output("corrected_x", Data(static_cast<double>(corrected_x)));
    set_output("corrected_y", Data(static_cast<double>(corrected_y)));

    OVF_INFO() << "Drift correction: original=(" << drift_x_ << "," << drift_y_
               << "), corrected=(" << corrected_x << "," << corrected_y << ")";

    return Result<void>::success();
}

// AdaptiveParamNode
AdaptiveParamNode::AdaptiveParamNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AdaptiveParamNode::make_info() {
    NodeInfo info;
    info.id = "AdaptiveParam";
    info.name = "参数自适应";
    info.category = "自动标定";
    info.description = "根据图像质量自动调整相机参数";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("exposure_us", "曝光时间", DataType::Number));
    info.outputs.push_back(DataPort("gain", "增益", DataType::Number));
    info.outputs.push_back(DataPort("brightness", "亮度", DataType::Number));
    info.outputs.push_back(DataPort("contrast", "对比度", DataType::Number));

    info.params.push_back(ParamDef("target_brightness", "目标亮度", DataType::Number, Data(128.0f)));
    info.params.push_back(ParamDef("enable_adaptive", "启用自适应", DataType::Boolean, Data(true)));

    return info;
}

Result<void> AdaptiveParamNode::execute(FlowContext& context) {
    auto image_data = get_input("image");
    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is required");
    }

    ImageData image = image_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    config_.enable_adaptive = get_param("enable_adaptive", Data(true)).as_bool();

    if (!config_.enable_adaptive) {
        // 不启用自适应，使用默认参数
        params_.exposure_us = 10000.0;
        params_.gain = 1.0;
        params_.brightness = 0.0;
        params_.contrast = 1.0;
    } else {
        // 执行参数自适应调整
        ErrorCode err = adaptive_params::adjust_params_for_quality(image, params_);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "Parameter adjustment failed");
        }
    }

    // 设置输出
    set_output("exposure_us", Data(params_.exposure_us));
    set_output("gain", Data(params_.gain));
    set_output("brightness", Data(params_.brightness));
    set_output("contrast", Data(params_.contrast));

    OVF_INFO() << "Adaptive params: exposure=" << params_.exposure_us
               << "us, gain=" << params_.gain;

    return Result<void>::success();
}

// QualityEvaluateNode
QualityEvaluateNode::QualityEvaluateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo QualityEvaluateNode::make_info() {
    NodeInfo info;
    info.id = "QualityEvaluate";
    info.name = "质量评估";
    info.category = "自动标定";
    info.description = "评估图像质量（清晰度、对比度、亮度）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("sharpness", "清晰度", DataType::Number));
    info.outputs.push_back(DataPort("contrast", "对比度", DataType::Number));
    info.outputs.push_back(DataPort("brightness", "亮度", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "综合质量分数", DataType::Number));

    info.params.push_back(ParamDef("sharpness_threshold", "清晰度阈值", DataType::Number, Data(10.0f)));
    info.params.push_back(ParamDef("contrast_threshold", "对比度阈值", DataType::Number, Data(30.0f)));

    return info;
}

Result<void> QualityEvaluateNode::execute(FlowContext& context) {
    auto image_data = get_input("image");
    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is required");
    }

    ImageData image = image_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 评估图像质量
    ErrorCode err = adaptive_params::evaluate_image_quality(image, sharpness_, contrast_, brightness_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Quality evaluation failed");
    }

    // 计算综合质量分数（0-100）
    // 清晰度权重 40%, 对比度权重 40%, 亮度权重 20%
    float sharpness_score = std::min(sharpness_ / 50.0f, 1.0f) * 100.0f;
    float contrast_score = std::min(contrast_ / 70.0f, 1.0f) * 100.0f;
    float brightness_score = 100.0f - std::abs(brightness_ - 128.0f) / 128.0f * 100.0f;

    float quality_score = sharpness_score * 0.4f + contrast_score * 0.4f + brightness_score * 0.2f;

    // 设置输出
    set_output("sharpness", Data(static_cast<double>(sharpness_)));
    set_output("contrast", Data(static_cast<double>(contrast_)));
    set_output("brightness", Data(static_cast<double>(brightness_)));
    set_output("quality_score", Data(static_cast<double>(quality_score)));

    OVF_INFO() << "Quality evaluation: sharpness=" << sharpness_
               << ", contrast=" << contrast_
               << ", brightness=" << brightness_
               << ", score=" << quality_score;

    return Result<void>::success();
}

// CalibrationHistoryNode
CalibrationHistoryNode::CalibrationHistoryNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CalibrationHistoryNode::make_info() {
    NodeInfo info;
    info.id = "CalibrationHistory";
    info.name = "标定历史";
    info.category = "自动标定";
    info.description = "记录和管理标定历史数据";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("compensation", "补偿参数", DataType::Object, false));
    info.inputs.push_back(DataPort("record", "记录新数据", DataType::Boolean));

    info.outputs.push_back(DataPort("history_count", "历史记录数", DataType::Number));
    info.outputs.push_back(DataPort("last_offset_x", "最近补偿X", DataType::Number));
    info.outputs.push_back(DataPort("last_offset_y", "最近补偿Y", DataType::Number));

    info.params.push_back(ParamDef("max_history", "最大历史数", DataType::Number, Data(100)));

    return info;
}

Result<void> CalibrationHistoryNode::execute(FlowContext& context) {
    max_history_count_ = get_param("max_history", Data(100)).as_int();

    // 检查是否需要记录新数据
    auto record_data = get_input("record");
    bool should_record = record_data.is_valid() ? record_data.as_bool() : false;

    if (should_record) {
        // 从全局历史获取最新的补偿参数
        std::vector<auto_calibration::CalibCompensation> global_history;
        auto_calibration::get_calibration_history(global_history, 1);

        if (!global_history.empty()) {
            history_.push_back(global_history.back());

            // 限制历史记录数量
            while (history_.size() > max_history_count_) {
                history_.pop_front();
            }
        }
    }

    // 设置输出
    set_output("history_count", Data(static_cast<int>(history_.size())));

    if (!history_.empty()) {
        const auto& last = history_.back();
        set_output("last_offset_x", Data(static_cast<double>(last.offset_x)));
        set_output("last_offset_y", Data(static_cast<double>(last.offset_y)));
    } else {
        set_output("last_offset_x", Data(0.0));
        set_output("last_offset_y", Data(0.0));
    }

    OVF_INFO() << "Calibration history: count=" << history_.size();

    return Result<void>::success();
}

// ========== 节点注册 ==========

OVF_REGISTER_NODE(CalibrationMonitorNode, "CalibrationMonitor", CalibrationMonitorNode::make_info())
OVF_REGISTER_NODE(AutoCalibrationNode, "AutoCalibration", AutoCalibrationNode::make_info())
OVF_REGISTER_NODE(DriftCorrectionNode, "DriftCorrection", DriftCorrectionNode::make_info())
OVF_REGISTER_NODE(AdaptiveParamNode, "AdaptiveParam", AdaptiveParamNode::make_info())
OVF_REGISTER_NODE(QualityEvaluateNode, "QualityEvaluate", QualityEvaluateNode::make_info())
OVF_REGISTER_NODE(CalibrationHistoryNode, "CalibrationHistory", CalibrationHistoryNode::make_info())

} // namespace algorithm
} // namespace ovf