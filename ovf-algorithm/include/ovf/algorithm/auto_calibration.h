/**
 * @file auto_calibration.h
 * @brief 自动标定补偿流程模块
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/template_matching.h"
#include <vector>
#include <deque>
#include <chrono>

namespace ovf {
namespace algorithm {

// ========== 环境监测命名空间 ==========

namespace calibration_monitor {

/**
 * @brief 环境状态结构
 */
struct EnvironmentStatus {
    float temperature;          // 温度（°C）
    float humidity;             // 湿度（%）
    float vibration_level;      // 振动等级
    float light_intensity;      // 光照强度
    float drift_x;              // 漂移量X（像素）
    float drift_y;              // 漂移量Y（像素）
    bool needs_calibration;     // 是否需要标定

    EnvironmentStatus()
        : temperature(25.0f)
        , humidity(50.0f)
        , vibration_level(0.0f)
        , light_intensity(100.0f)
        , drift_x(0.0f)
        , drift_y(0.0f)
        , needs_calibration(false) {}
};

/**
 * @brief 监测环境变化
 * @param status 输出的环境状态
 * @return 错误码
 */
ErrorCode monitor_environment(EnvironmentStatus& status);

/**
 * @brief 计算漂移量（对比基准图像）
 * @param reference 基准图像
 * @param current 当前图像
 * @param drift_x 输出的X方向漂移量
 * @param drift_y 输出的Y方向漂移量
 * @return 错误码
 */
ErrorCode calculate_drift(const ImageData& reference, const ImageData& current,
                          float& drift_x, float& drift_y);

/**
 * @brief 判断是否需要重新标定
 * @param status 环境状态
 * @param temp_threshold 温度变化阈值
 * @param drift_threshold 漂移阈值
 * @return 是否需要标定
 */
bool check_calibration_needed(const EnvironmentStatus& status,
                              float temp_threshold = 5.0f,
                              float drift_threshold = 0.5f);

} // namespace calibration_monitor

// ========== 自动标定命名空间 ==========

namespace auto_calibration {

/**
 * @brief 自动标定配置结构
 */
struct AutoCalibConfig {
    float check_interval_sec;       // 检查间隔（秒）
    float drift_threshold;          // 漂移阈值（像素）
    float temp_threshold;           // 温度阈值（°C）
    int max_auto_corrections;       // 最大自动校正次数
    bool enable_auto_correction;    // 启用自动校正

    AutoCalibConfig()
        : check_interval_sec(60.0f)
        , drift_threshold(0.5f)
        , temp_threshold(5.0f)
        , max_auto_corrections(3)
        , enable_auto_correction(true) {}
};

/**
 * @brief 标定补偿参数结构
 */
struct CalibCompensation {
    float offset_x;             // 位置补偿X
    float offset_y;             // 位置补偿Y
    float rotation_comp;        // 旋转补偿（度）
    float scale_comp_x;         // 缩放补偿X
    float scale_comp_y;         // 缩放补偿Y
    uint64_t timestamp;         // 时间戳
    bool valid;                 // 是否有效

    CalibCompensation()
        : offset_x(0.0f)
        , offset_y(0.0f)
        , rotation_comp(0.0f)
        , scale_comp_x(1.0f)
        , scale_comp_y(1.0f)
        , timestamp(0)
        , valid(false) {}
};

/**
 * @brief 初始化自动标定系统
 * @param config 配置参数
 * @return 错误码
 */
ErrorCode init_auto_calibration(const AutoCalibConfig& config);

/**
 * @brief 执行自动标定检查
 * @param reference 基准图像
 * @param compensation 输出的补偿参数
 * @return 错误码
 */
ErrorCode run_calibration_check(const ImageData& reference,
                                CalibCompensation& compensation);

/**
 * @brief 应用补偿参数
 * @param comp 补偿参数
 * @return 错误码
 */
ErrorCode apply_compensation(const CalibCompensation& comp);

/**
 * @brief 记录标定历史
 * @param comp 补偿参数
 * @return 错误码
 */
ErrorCode log_calibration_history(const CalibCompensation& comp);

/**
 * @brief 获取标定历史记录
 * @param history 输出的历史记录列表
 * @param max_count 最大记录数
 * @return 错误码
 */
ErrorCode get_calibration_history(std::vector<CalibCompensation>& history, int max_count = 100);

} // namespace auto_calibration

// ========== 参数自适应命名空间 ==========

namespace adaptive_params {

/**
 * @brief 相机参数结构
 */
struct CameraParams {
    double exposure_us;         // 曝光时间（微秒）
    double gain;                // 增益
    double brightness;          // 亮度
    double contrast;            // 对比度

    CameraParams()
        : exposure_us(10000.0)
        , gain(1.0)
        , brightness(0.0)
        , contrast(1.0) {}
};

/**
 * @brief 自适应配置结构
 */
struct AdaptiveConfig {
    float brightness_range;     // 亮度范围
    float contrast_range;       // 对比度范围
    float exposure_auto_range;  // 自动曝光范围
    bool enable_adaptive;       // 启用自适应

    AdaptiveConfig()
        : brightness_range(0.2f)
        , contrast_range(0.3f)
        , exposure_auto_range(5000.0f)
        , enable_adaptive(true) {}
};

/**
 * @brief 根据图像质量调整参数
 * @param image 输入图像
 * @param params 输出的调整后参数
 * @return 错误码
 */
ErrorCode adjust_params_for_quality(const ImageData& image,
                                    CameraParams& params);

/**
 * @brief 计算图像质量指标
 * @param image 输入图像
 * @param sharpness 输出的清晰度（梯度）
 * @param contrast 输出的对比度
 * @param brightness 输出的亮度
 * @return 错误码
 */
ErrorCode evaluate_image_quality(const ImageData& image,
                                 float& sharpness,
                                 float& contrast,
                                 float& brightness);

/**
 * @brief 自适应曝光调整
 * @param image 输入图像
 * @param exposure_us 输出的曝光时间
 * @return 错误码
 */
ErrorCode adaptive_exposure(const ImageData& image, double& exposure_us);

/**
 * @brief 自适应增益调整
 * @param image 输入图像
 * @param gain 输出的增益值
 * @return 错误码
 */
ErrorCode adaptive_gain(const ImageData& image, double& gain);

} // namespace adaptive_params

// ========== 节点类定义 ==========

/**
 * @brief 标定监测节点
 */
class CalibrationMonitorNode : public INode {
public:
    CalibrationMonitorNode(const String& instance_id);
    ~CalibrationMonitorNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void set_reference_image(const ImageData& ref) { reference_image_ = ref; }
    const calibration_monitor::EnvironmentStatus& get_status() const { return status_; }

private:
    ImageData reference_image_;
    calibration_monitor::EnvironmentStatus status_;
};

/**
 * @brief 自动标定节点
 */
class AutoCalibrationNode : public INode {
public:
    AutoCalibrationNode(const String& instance_id);
    ~AutoCalibrationNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void set_config(const auto_calibration::AutoCalibConfig& config) { config_ = config; }
    const auto_calibration::CalibCompensation& get_compensation() const { return compensation_; }

private:
    auto_calibration::AutoCalibConfig config_;
    auto_calibration::CalibCompensation compensation_;
    ImageData reference_image_;
};

/**
 * @brief 漂移校正节点
 */
class DriftCorrectionNode : public INode {
public:
    DriftCorrectionNode(const String& instance_id);
    ~DriftCorrectionNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float drift_x_ = 0.0f;
    float drift_y_ = 0.0f;
};

/**
 * @brief 参数自适应节点
 */
class AdaptiveParamNode : public INode {
public:
    AdaptiveParamNode(const String& instance_id);
    ~AdaptiveParamNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    const adaptive_params::CameraParams& get_params() const { return params_; }

private:
    adaptive_params::AdaptiveConfig config_;
    adaptive_params::CameraParams params_;
};

/**
 * @brief 质量评估节点
 */
class QualityEvaluateNode : public INode {
public:
    QualityEvaluateNode(const String& instance_id);
    ~QualityEvaluateNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    float get_sharpness() const { return sharpness_; }
    float get_contrast() const { return contrast_; }
    float get_brightness() const { return brightness_; }

private:
    float sharpness_ = 0.0f;
    float contrast_ = 0.0f;
    float brightness_ = 0.0f;
};

/**
 * @brief 标定历史记录节点
 */
class CalibrationHistoryNode : public INode {
public:
    CalibrationHistoryNode(const String& instance_id);
    ~CalibrationHistoryNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::deque<auto_calibration::CalibCompensation> history_;
    int max_history_count_ = 100;
};

} // namespace algorithm
} // namespace ovf