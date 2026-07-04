/**
 * @file image_arithmetic.h
 * @brief 图像运算算子（纯C++实现，参考VisionPro CogIPTwoImage系列）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <algorithm>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 图像运算工具函数
 */
namespace arithmetic_utils {

/**
 * @brief 验证两图像是否兼容运算
 * @return true 如果尺寸和通道数匹配
 */
bool validate_images(const ImageData& a, const ImageData& b);

/**
 * @brief 饱和截断到[0, 255]
 */
inline uint8_t saturate_cast(double value) {
    return static_cast<uint8_t>(std::clamp(value, 0.0, 255.0));
}

/**
 * @brief 保持浮点值（不截断）
 */
inline float keep_float(double value) {
    return static_cast<float>(value);
}

/**
 * @brief 图像加法
 * @param scale_factor 缩放因子（避免溢出）
 * @param offset 偏移值
 * @param saturate 是否饱和截断
 */
void image_add(const ImageData& a, const ImageData& b, ImageData& result,
               double scale_factor = 1.0, double offset = 0.0, bool saturate = true);

/**
 * @brief 图像减法
 */
void image_subtract(const ImageData& a, const ImageData& b, ImageData& result,
                    double scale_factor = 1.0, double offset = 0.0, bool saturate = true);

/**
 * @brief 图像乘法
 */
void image_multiply(const ImageData& a, const ImageData& b, ImageData& result,
                    double scale_factor = 1.0, double offset = 0.0, bool saturate = true);

/**
 * @brief 图像除法
 */
void image_divide(const ImageData& a, const ImageData& b, ImageData& result,
                  double scale_factor = 1.0, double offset = 0.0, bool saturate = true);

/**
 * @brief 绝对差值
 */
void image_abs_diff(const ImageData& a, const ImageData& b, ImageData& result,
                    double scale_factor = 1.0, double offset = 0.0, bool saturate = true);

/**
 * @brief 两图像最小值
 */
void image_min(const ImageData& a, const ImageData& b, ImageData& result);

/**
 * @brief 两图像最大值
 */
void image_max(const ImageData& a, const ImageData& b, ImageData& result);

/**
 * @brief 位与运算
 */
void image_and(const ImageData& a, const ImageData& b, ImageData& result);

/**
 * @brief 位或运算
 */
void image_or(const ImageData& a, const ImageData& b, ImageData& result);

/**
 * @brief 位异或运算
 */
void image_xor(const ImageData& a, const ImageData& b, ImageData& result);

/**
 * @brief 位非运算（单图像）
 */
void image_not(const ImageData& a, ImageData& result);

/**
 * @brief 图像混合（加权）
 * @param blend_weight 第一图像权重 (0.0-1.0)
 */
void image_blend(const ImageData& a, const ImageData& b, ImageData& result,
                 double blend_weight = 0.5, bool saturate = true);

} // namespace arithmetic_utils

// ========== 图像算术运算节点（6个） ==========

/**
 * @brief 图像加法节点
 */
class ImageAddNode : public INode {
public:
    ImageAddNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 图像减法节点
 */
class ImageSubtractNode : public INode {
public:
    ImageSubtractNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 图像乘法节点
 */
class ImageMultiplyNode : public INode {
public:
    ImageMultiplyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 图像除法节点
 */
class ImageDivideNode : public INode {
public:
    ImageDivideNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 绝对差值节点
 */
class ImageAbsDiffNode : public INode {
public:
    ImageAbsDiffNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 图像最小值节点
 */
class ImageMinNode : public INode {
public:
    ImageMinNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 图像最大值节点
 */
class ImageMaxNode : public INode {
public:
    ImageMaxNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ========== 位运算节点（4个） ==========

/**
 * @brief 位与运算节点
 */
class ImageAndNode : public INode {
public:
    ImageAndNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 位或运算节点
 */
class ImageOrNode : public INode {
public:
    ImageOrNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 位异或运算节点
 */
class ImageXorNode : public INode {
public:
    ImageXorNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 位非运算节点
 */
class ImageNotNode : public INode {
public:
    ImageNotNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ========== 混合运算节点（1个） ==========

/**
 * @brief 图像混合节点
 */
class ImageBlendNode : public INode {
public:
    ImageBlendNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf