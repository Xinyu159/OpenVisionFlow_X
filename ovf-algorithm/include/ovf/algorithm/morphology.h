#pragma once

#include "ovf/core/node.h"
#include <vector>

namespace ovf {
namespace algorithm {

/**
 * @brief 形态学操作类型
 */
enum class MorphOp {
    Erode,      // 腐蚀
    Dilate,     // 膨胀
    Open,       // 开运算（先腐蚀后膨胀）
    Close,      // 闭运算（先膨胀后腐蚀）
    Gradient,   // 形态学梯度（膨胀减腐蚀）
    TopHat,     // 顶帽（原图减开运算）
    BlackHat    // 黑帽（闭运算减原图）
};

/**
 * @brief 结构元素形状
 */
enum class KernelShape {
    Rect,       // 矩形
    Ellipse,    // 椭圆/圆形
    Cross,      // 十字
    Diamond,    // 菱形
    Custom      // 自定义
};

/**
 * @brief 形态学处理工具函数
 */
namespace morph_utils {

/**
 * @brief 创建结构元素（核）
 * @param shape 形状
 * @param ksize 大小
 * @param kernel 输出核数据
 * @return 错误码
 */
ErrorCode create_kernel(KernelShape shape, int ksize, std::vector<uint8_t>& kernel);

/**
 * @brief 创建自定义结构元素
 * @param data 自定义数据
 * @param width 核宽度
 * @param height 核高度
 * @param kernel 输出核数据
 * @return 错误码
 */
ErrorCode create_custom_kernel(const uint8_t* data, int width, int height,
                               std::vector<uint8_t>& kernel);

/**
 * @brief 腐蚀操作
 * @param src 源图像
 * @param dst 目标图像
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 结构元素
 * @param kernel_width 核宽度
 * @param kernel_height 核高度
 * @param iterations 迭代次数
 * @return 错误码
 */
ErrorCode erode(const uint8_t* src, uint8_t* dst, int width, int height,
                const uint8_t* kernel, int kernel_width, int kernel_height,
                int iterations = 1);

/**
 * @brief 膨胀操作
 */
ErrorCode dilate(const uint8_t* src, uint8_t* dst, int width, int height,
                 const uint8_t* kernel, int kernel_width, int kernel_height,
                 int iterations = 1);

/**
 * @brief 开运算
 */
ErrorCode open(const uint8_t* src, uint8_t* dst, int width, int height,
               const uint8_t* kernel, int kernel_width, int kernel_height,
               int iterations = 1);

/**
 * @brief 闭运算
 */
ErrorCode close(const uint8_t* src, uint8_t* dst, int width, int height,
                const uint8_t* kernel, int kernel_width, int kernel_height,
                int iterations = 1);

/**
 * @brief 形态学梯度
 */
ErrorCode gradient(const uint8_t* src, uint8_t* dst, int width, int height,
                   const uint8_t* kernel, int kernel_width, int kernel_height);

/**
 * @brief 顶帽运算
 */
ErrorCode top_hat(const uint8_t* src, uint8_t* dst, int width, int height,
                   const uint8_t* kernel, int kernel_width, int kernel_height,
                   int iterations = 1);

/**
 * @brief 黑帽运算
 */
ErrorCode black_hat(const uint8_t* src, uint8_t* dst, int width, int height,
                    const uint8_t* kernel, int kernel_width, int kernel_height,
                    int iterations = 1);

/**
 * @brief 骨架提取
 */
ErrorCode skeleton(const uint8_t* src, uint8_t* dst, int width, int height);

/**
 * @brief 距离变换
 */
ErrorCode distance_transform(const uint8_t* src, uint8_t* dst, int width, int height);

/**
 * @brief 连通区域标记后的形态学重建
 */
ErrorCode reconstruct(const uint8_t* marker, const uint8_t* mask,
                      uint8_t* dst, int width, int height);

} // namespace morph_utils

/**
 * @brief 形态学处理节点
 */
class MorphologyNode : public INode {
public:
    MorphologyNode(const String& instance_id);
    ~MorphologyNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

    void set_operation(MorphOp op) { operation_ = op; }
    void set_kernel(KernelShape shape, int ksize);
    void set_iterations(int iter) { iterations_ = iter; }

private:
    MorphOp operation_ = MorphOp::Erode;
    KernelShape kernel_shape_ = KernelShape::Rect;
    int kernel_size_ = 3;
    int iterations_ = 1;
    std::vector<uint8_t> kernel_;
    std::vector<uint8_t> custom_kernel_data_;
};

/**
 * @brief 阈值形态学节点（先阈值后形态学）
 */
class ThresholdMorphNode : public INode {
public:
    ThresholdMorphNode(const String& instance_id);
    ~ThresholdMorphNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    int threshold_value_ = 128;
    MorphOp morph_op_ = MorphOp::Erode;
    KernelShape kernel_shape_ = KernelShape::Rect;
    int kernel_size_ = 3;
    int iterations_ = 1;
};

/**
 * @brief 孔洞填充节点
 */
class FillHolesNode : public INode {
public:
    FillHolesNode(const String& instance_id);
    ~FillHolesNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 边界提取节点
 */
class ExtractBoundaryNode : public INode {
public:
    ExtractBoundaryNode(const String& instance_id);
    ~ExtractBoundaryNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 细化节点
 */
class ThinningNode : public INode {
public:
    ThinningNode(const String& instance_id);
    ~ThinningNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    int max_iterations_ = 100;
};

} // namespace algorithm
} // namespace ovf