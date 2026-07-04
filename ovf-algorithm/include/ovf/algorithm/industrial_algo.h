/**
 * @file industrial_algo.h
 * @brief 工业专用算子模块 - 微缺陷检测、反光处理、频域分析、图像增强
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/types.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

// ========== 缺陷检测模块 ==========

namespace defect_utils {

/**
 * @brief 缺陷类型枚举
 */
enum class DefectType {
    Scratch,       // 划痕
    Crack,         // 裂纹
    Spot,          // 斑点
    Burr,          // 毛刺
    Pinhole,       // 针孔
    Contamination, // 污染
    Unevenness,     // 不平整
    Missing        // 缺失
};

/**
 * @brief 缺陷信息结构
 */
struct DefectInfo {
    DefectType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float severity = 0.0f;      // 严重程度 0-1
    float confidence = 0.0f;    // 置信度
    String description;

    DefectInfo() = default;
    DefectInfo(DefectType t, int px, int py, int w, int h, 
               float sev, float conf, const String& desc = "")
        : type(t), x(px), y(py), width(w), height(h), 
          severity(sev), confidence(conf), description(desc) {}
};

/**
 * @brief 获取缺陷类型名称
 */
inline String defect_type_name(DefectType type) {
    switch (type) {
        case DefectType::Scratch: return "划痕";
        case DefectType::Crack: return "裂纹";
        case DefectType::Spot: return "斑点";
        case DefectType::Burr: return "毛刺";
        case DefectType::Pinhole: return "针孔";
        case DefectType::Contamination: return "污染";
        case DefectType::Unevenness: return "不平整";
        case DefectType::Missing: return "缺失";
        default: return "未知";
    }
}

/**
 * @brief 微缺陷检测（多尺度方法）
 * @param image 输入图像
 * @param defects 输出缺陷列表
 * @param sensitivity 灵敏度 0-1
 * @param min_size 最小缺陷尺寸
 * @return 错误码
 */
ErrorCode detect_micro_defects(const ImageData& image, Vector<DefectInfo>& defects,
                               float sensitivity = 0.5f, int min_size = 3);

/**
 * @brief 划痕检测（方向性分析）
 * @param image 输入图像
 * @param scratches 输出划痕列表
 * @param min_length 最小长度
 * @param angle_tolerance 角度容差（度）
 * @return 错误码
 */
ErrorCode detect_scratches(const ImageData& image, Vector<DefectInfo>& scratches,
                          int min_length = 10, float angle_tolerance = 15.0f);

/**
 * @brief 斑点检测（局部对比度）
 * @param image 输入图像
 * @param spots 输出斑点列表
 * @param min_area 最小面积
 * @param contrast_threshold 对比度阈值
 * @return 错误码
 */
ErrorCode detect_spots(const ImageData& image, Vector<DefectInfo>& spots,
                      int min_area = 5, float contrast_threshold = 0.1f);

} // namespace defect_utils

// ========== 反光处理模块 ==========

namespace reflection_utils {

/**
 * @brief 反光检测
 * @param image 输入图像
 * @param reflection_points 输出反光点列表
 * @param threshold 亮度阈值
 * @return 错误码
 */
ErrorCode detect_reflection(const ImageData& image, Vector<Point2D<int>>& reflection_points,
                            float threshold = 250.0f);

/**
 * @brief 反光抑制（多方法）
 * @param image 输入/输出图像
 * @param method 方法：0=滤波 1=插值 2=偏振模拟
 * @return 错误码
 */
ErrorCode remove_reflection(ImageData& image, int method = 0);

/**
 * @brief 高光区域修复
 * @param image 输入/输出图像
 * @param highlights 高光点列表
 * @return 错误码
 */
ErrorCode repair_highlights(ImageData& image, const Vector<Point2D<int>>& highlights);

/**
 * @brief 均匀光照校正
 * @param image 输入/输出图像
 * @return 错误码
 */
ErrorCode correct_uneven_lighting(ImageData& image);

/**
 * @brief 自适应阈值（处理反光区域）
 * @param image 输入图像
 * @param binary 输出二值图像
 * @return 错误码
 */
ErrorCode adaptive_threshold_reflection(const ImageData& image, ImageData& binary);

} // namespace reflection_utils

// ========== 频域分析模块 ==========

namespace frequency_utils {

/**
 * @brief FFT 2D（简化实现，仅计算幅度谱和相位谱）
 * @param image 输入图像
 * @param magnitude 输出幅度谱
 * @param phase 输出相位谱
 * @return 错误码
 */
ErrorCode fft2d(const ImageData& image, Vector<float>& magnitude, Vector<float>& phase);

/**
 * @brief 频域滤波
 * @param image 输入/输出图像
 * @param filter_type 滤波类型：0=低通 1=高通 2=带通
 * @param cutoff_freq 截止频率
 * @return 错误码
 */
ErrorCode frequency_filter(ImageData& image, int filter_type, float cutoff_freq);

/**
 * @brief 周期性噪声去除
 * @param image 输入/输出图像
 * @return 错误码
 */
ErrorCode remove_periodic_noise(ImageData& image);

/**
 * @brief 纹理分析
 * @param image 输入图像
 * @param uniformity 输出均匀度
 * @param coarseness 输出粗糙度
 * @return 错误码
 */
ErrorCode analyze_texture(const ImageData& image, float& uniformity, float& coarseness);

/**
 * @brief 频域缺陷检测
 * @param image 输入图像
 * @param defects 输出缺陷列表
 * @return 错误码
 */
ErrorCode detect_defects_frequency(const ImageData& image, Vector<defect_utils::DefectInfo>& defects);

} // namespace frequency_utils

// ========== 图像增强模块 ==========

namespace enhance_utils {

/**
 * @brief 低对比度增强
 * @param image 输入/输出图像
 * @param strength 增强强度
 * @return 错误码
 */
ErrorCode enhance_low_contrast(ImageData& image, float strength = 1.0f);

/**
 * @brief 暗图增强
 * @param image 输入/输出图像
 * @param gamma Gamma值
 * @return 错误码
 */
ErrorCode enhance_dark_image(ImageData& image, float gamma = 1.5f);

/**
 * @brief 噪声抑制保边缘
 * @param image 输入/输出图像
 * @param iterations 迭代次数
 * @return 错误码
 */
ErrorCode denoise_preserve_edge(ImageData& image, int iterations = 3);

/**
 * @brief 锐化增强
 * @param image 输入/输出图像
 * @param strength 锐化强度
 * @return 错误码
 */
ErrorCode sharpen(ImageData& image, float strength = 0.5f);

/**
 * @brief 自适应增强
 * @param image 输入/输出图像
 * @return 错误码
 */
ErrorCode adaptive_enhance(ImageData& image);

} // namespace enhance_utils

// ========== 工业算子节点 ==========

/**
 * @brief 微缺陷检测节点
 */
class MicroDefectDetectNode : public INode {
public:
    MicroDefectDetectNode(const String& instance_id);
    ~MicroDefectDetectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float sensitivity_ = 0.5f;
    int min_size_ = 3;
};

/**
 * @brief 划痕检测节点
 */
class ScratchDetectNode : public INode {
public:
    ScratchDetectNode(const String& instance_id);
    ~ScratchDetectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int min_length_ = 10;
    float angle_tolerance_ = 15.0f;
};

/**
 * @brief 反光抑制节点
 */
class ReflectionRemoveNode : public INode {
public:
    ReflectionRemoveNode(const String& instance_id);
    ~ReflectionRemoveNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int method_ = 0;
    float threshold_ = 250.0f;
};

/**
 * @brief 频域滤波节点
 */
class FrequencyFilterNode : public INode {
public:
    FrequencyFilterNode(const String& instance_id);
    ~FrequencyFilterNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int filter_type_ = 0;
    float cutoff_freq_ = 0.5f;
};

/**
 * @brief 纹理分析节点
 */
class TextureAnalysisNode : public INode {
public:
    TextureAnalysisNode(const String& instance_id);
    ~TextureAnalysisNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 自适应增强节点
 */
class AdaptiveEnhanceNode : public INode {
public:
    AdaptiveEnhanceNode(const String& instance_id);
    ~AdaptiveEnhanceNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf