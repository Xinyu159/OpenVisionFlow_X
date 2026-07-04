/**
 * @file image_enhance.h
 * @brief 图像增强算子（纯C++实现，不依赖OpenCV）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 图像增强工具函数
 */
namespace enhance_utils {

// ========== 滤波器工具函数 ==========

/**
 * @brief 快速中值滤波 - 使用优化算法
 */
void median_filter(const uint8_t* src, uint8_t* dst, int width, int height, 
                   int kernel_size, int channels);

/**
 * @brief 双边滤波 - 保边缘去噪
 */
void bilateral_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                      int channels, int d, double sigma_color, double sigma_space);

/**
 * @brief 导向滤波
 */
void guided_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                   int channels, int radius, double epsilon);

/**
 * @brief 高斯滤波
 */
void gaussian_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                     int channels, int kernel_size, double sigma);

/**
 * @brief 方框滤波/均值滤波
 */
void box_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                int channels, int kernel_size);

/**
 * @brief 拉普拉斯滤波
 */
void laplacian_filter(const uint8_t* src, uint8_t* dst, int width, int height, int channels);

/**
 * @brief 锐化滤波
 */
void sharpen_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                    int channels, double strength);

/**
 * @brief 反锐化掩蔽
 */
void unsharp_mask(const uint8_t* src, uint8_t* dst, int width, int height,
                  int channels, int radius, double amount, double threshold);

// ========== 去噪工具函数 ==========

/**
 * @brief 非局部均值去噪（简化版）
 */
void nlm_denoise(const uint8_t* src, uint8_t* dst, int width, int height,
                 int channels, int search_window, int template_size, double h);

/**
 * @brief 自适应去噪 - 根据局部方差调整
 */
void adaptive_denoise(const uint8_t* src, uint8_t* dst, int width, int height,
                      int channels, int window_size, double noise_level);

// ========== 增强工具函数 ==========

/**
 * @brief 对比度增强 - 线性拉伸
 */
void contrast_enhance(const uint8_t* src, uint8_t* dst, int width, int height,
                      int channels, double factor);

/**
 * @brief 亮度调整
 */
void brightness_adjust(const uint8_t* src, uint8_t* dst, int width, int height,
                       int channels, int delta);

/**
 * @brief Gamma校正
 */
void gamma_correct(const uint8_t* src, uint8_t* dst, int width, int height,
                   int channels, double gamma);

/**
 * @brief 直方图均衡化（全局）
 */
void histogram_equalize(const uint8_t* src, uint8_t* dst, int width, int height, int channels);

/**
 * @brief 自适应直方图均衡（CLAHE简化）
 */
void adaptive_histogram_equalize(const uint8_t* src, uint8_t* dst, int width, int height,
                                  int channels, int clip_limit, int tile_size);

// ========== 色彩调整工具函数 ==========

/**
 * @brief RGB到HSV转换
 */
void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, double& h, double& s, double& v);

/**
 * @brief HSV到RGB转换
 */
void hsv_to_rgb(double h, double s, double v, uint8_t& r, uint8_t& g, uint8_t& b);

/**
 * @brief 饱和度调整
 */
void saturation_adjust(const uint8_t* src, uint8_t* dst, int width, int height, double factor);

/**
 * @brief 色相偏移
 */
void hue_shift(const uint8_t* src, uint8_t* dst, int width, int height, double delta_h);

/**
 * @brief 白平衡校正
 */
void white_balance(const uint8_t* src, uint8_t* dst, int width, int height, const String& method);

// ========== 辅助函数 ==========

/**
 * @brief 创建高斯核
 */
std::vector<double> create_gaussian_kernel(int size, double sigma);

/**
 * @brief 计算直方图
 */
std::vector<int> compute_histogram(const uint8_t* data, int size);

/**
 * @brief 计算累积分布函数
 */
std::vector<double> compute_cdf(const std::vector<int>& histogram, int total_pixels);

} // namespace enhance_utils

// ========== 滤波器类节点（8个） ==========

/**
 * @brief 中值滤波节点
 */
class MedianFilterNode : public INode {
public:
    MedianFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 双边滤波节点
 */
class BilateralFilterNode : public INode {
public:
    BilateralFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 导向滤波节点
 */
class GuidedFilterNode : public INode {
public:
    GuidedFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 高斯滤波节点
 */
class GaussianFilterNode : public INode {
public:
    GaussianFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 方框滤波/均值滤波节点
 */
class BoxFilterNode : public INode {
public:
    BoxFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 拉普拉斯滤波节点
 */
class LaplacianFilterNode : public INode {
public:
    LaplacianFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 锐化滤波节点
 */
class SharpenFilterNode : public INode {
public:
    SharpenFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 反锐化掩蔽节点
 */
class UnsharpMaskNode : public INode {
public:
    UnsharpMaskNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ========== 去噪类节点（4个） ==========

/**
 * @brief 双边去噪节点
 */
class DenoiseBilateralNode : public INode {
public:
    DenoiseBilateralNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 非局部均值去噪节点（简化）
 */
class DenoiseNLMNode : public INode {
public:
    DenoiseNLMNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 小波去噪节点（简化）
 */
class DenoiseWaveletNode : public INode {
public:
    DenoiseWaveletNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 自适应去噪节点
 */
class DenoiseAdaptiveNode : public INode {
public:
    DenoiseAdaptiveNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ========== 增强类节点（5个） ==========

/**
 * @brief 对比度增强节点
 */
class ContrastEnhanceNode : public INode {
public:
    ContrastEnhanceNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 亮度调整节点
 */
class BrightnessAdjustNode : public INode {
public:
    BrightnessAdjustNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief Gamma校正节点
 */
class GammaCorrectNode : public INode {
public:
    GammaCorrectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 直方图均衡化节点（全局）
 */
class HistogramEqualizeNode : public INode {
public:
    HistogramEqualizeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 自适应直方图均衡节点（CLAHE简化）
 */
class AdaptiveHistogramEqualizeNode : public INode {
public:
    AdaptiveHistogramEqualizeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ========== 色彩调整节点（3个） ==========

/**
 * @brief 鸣和度调整节点
 */
class SaturationAdjustNode : public INode {
public:
    SaturationAdjustNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 色相偏移节点
 */
class HueShiftNode : public INode {
public:
    HueShiftNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 白平衡校正节点
 */
class WhiteBalanceNode : public INode {
public:
    WhiteBalanceNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf