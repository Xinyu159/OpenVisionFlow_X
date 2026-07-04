/**
 * @file subpixel_precision.h
 * @brief 亚像素精度验证与优化模块（对标Halcon edges_sub_pix）
 *
 * 提供亚像素边缘定位、角点定位、直线/圆定位、亚像素测量与轮廓提取，
 * 以及精度验证工具（合成测试图、基准测试、报告生成）。
 * 目标精度：±0.01像素（边缘），±0.03像素（角点）。
 *
 * 节点列表（10个）：
 *   亚像素边缘定位：SubpixelEdgeNode / SubpixelCornerNode / SubpixelLineNode / SubpixelCircleNode
 *   亚像素测量：    SubpixelMeasureNode / SubpixelCaliperNode / SubpixelContourNode
 *   精度验证工具：  PrecisionTestNode / PrecisionBenchmarkNode / PrecisionReportNode
 *
 * 纯C++实现，使用 ovf::core::INode 基类，OVF_REGISTER_NODE 宏注册。
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/measurement.h"  // Point2Df / Line2D / Circle2D / fit_line / fit_circle
#include <vector>
#include <cmath>
#include <string>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

/**
 * @brief 亚像素定位方法枚举
 */
enum class SubpixelMethod {
    Taylor = 0,    // 二阶泰勒展开（参考Halcon edges_sub_pix）
    Parabola = 1,  // 三点抛物线拟合
    Gaussian = 2,  // 高斯函数拟合
    Saddle = 3,    // Saddle点（角点）
    Zernike = 4    // Zernike矩法
};

/**
 * @brief 亚像素边缘点结果
 */
struct SubpixelEdgePoint {
    float x = 0.0f;          // 边缘亚像素位置X
    float y = 0.0f;          // 边缘亚像素位置Y
    float strength = 0.0f;   // 边缘强度（梯度幅值）
    float angle = 0.0f;      // 梯度方向（弧度，沿梯度方向）
    float subpixel_offset = 0.0f;  // 沿梯度方向的亚像素偏移（像素）
    bool valid = false;
};

/**
 * @brief 亚像素角点结果
 */
struct SubpixelCornerPoint {
    float x = 0.0f;          // 角点亚像素位置X
    float y = 0.0f;          // 角点亚像素位置Y
    float response = 0.0f;  // 角点响应值
    float angle = 0.0f;      // 角点张角（度）
    bool valid = false;
};

/**
 * @brief 亚像素直线结果
 */
struct SubpixelLineResult {
    Line2D line;             // 拟合直线
    float rms_error = 0.0f;  // 拟合均方根误差（像素）
    uint32_t point_count = 0; // 参与拟合的边缘点数
    bool valid = false;
};

/**
 * @brief 亚像素圆结果
 */
struct SubpixelCircleResult {
    Circle2D circle;         // 拟合圆
    float rms_error = 0.0f;  // 拟合均方根误差（像素）
    uint32_t point_count = 0; // 参与拟合的边缘点数
    bool valid = false;
};

/**
 * @brief 精度统计信息
 */
struct PrecisionStats {
    float mean_error = 0.0f;    // 平均误差（像素）
    float std_error = 0.0f;     // 标准差（像素）
    float max_error = 0.0f;     // 最大误差（像素）
    float rms_error = 0.0f;     // 均方根误差（像素）
    uint32_t sample_count = 0;  // 样本数量
    float target_precision = 0.01f; // 目标精度（像素）
    bool pass = false;          // 是否达标

    // 计算达标判定
    void evaluate() {
        pass = (max_error <= target_precision) && (mean_error <= target_precision);
    }
};

/**
 * @brief 合成测试图类型
 */
enum class TestPatternType {
    VerticalEdge = 0,    // 垂直边缘
    HorizontalEdge = 1,  // 水平边缘
    Circle = 2,          // 圆形
    Line = 3,            // 倾斜直线
    Corner = 4,          // 角点
    Square = 5           // 矩形（含4个角点）
};

/**
 * @brief 亚像素算法核心工具
 */
namespace subpixel_utils {

// ------------------------------------------------------------------------
// 图像基础工具
// ------------------------------------------------------------------------

/**
 * @brief 转换为灰度图（uint8 单通道）
 */
ImageData to_gray(const ImageData& input);

/**
 * @brief 双线性插值采样
 * @param image 灰度图像
 * @param x X坐标（亚像素）
 * @param y Y坐标（亚像素）
 * @return 插值后的灰度值（0-255）
 */
float bilinear_sample(const ImageData& image, float x, float y);

/**
 * @brief 高斯平滑（可分离卷积）
 * @param input 输入灰度图
 * @param sigma 平滑系数
 * @return 平滑后图像
 */
ImageData gaussian_smooth(const ImageData& input, float sigma);

/**
 * @brief 一维高斯核
 */
std::vector<float> gaussian_kernel_1d(float sigma, int kernel_size = 0);

// ------------------------------------------------------------------------
// 梯度计算
// ------------------------------------------------------------------------

/**
 * @brief Sobel梯度计算（带亚像素精度）
 * @param image 输入灰度图
 * @param grad_x 输出X方向梯度（float缓冲，size = w*h）
 * @param grad_y 输出Y方向梯度
 * @param magnitude 输出梯度幅值
 * @param angle 输出梯度方向（弧度）
 */
void sobel_gradient(const ImageData& image,
                   std::vector<float>& grad_x,
                   std::vector<float>& grad_y,
                   std::vector<float>& magnitude,
                   std::vector<float>& angle);

// ------------------------------------------------------------------------
// 亚像素定位核心算法
// ------------------------------------------------------------------------

/**
 * @brief 二阶泰勒展开亚像素边缘定位（参考Halcon edges_sub_pix）
 *
 * 在梯度极大值点附近沿梯度方向取三点，使用二阶泰勒展开拟合
 * 一阶导数极值点位置，精度：±0.01像素。
 *
 * @param g0 前一点的梯度幅值
 * @param g1 中心点（极大值）的梯度幅值
 * @param g2 后一点的梯度幅值
 * @return 亚像素偏移量（相对于中心点，范围 [-0.5, 0.5]）
 */
float taylor_subpixel_offset(float g0, float g1, float g2);

/**
 * @brief 三点抛物线拟合亚像素定位
 *
 * y = ax^2 + bx + c，极值点 x = -b/(2a)
 * 精度：±0.02像素。
 *
 * @param y0 y(-1)
 * @param y1 y(0)
 * @param y2 y(+1)
 * @return 亚像素偏移量（范围 [-0.5, 0.5]）
 */
float parabola_subpixel_offset(float y0, float y1, float y2);

/**
 * @brief 高斯函数拟合亚像素定位
 *
 * 假设响应为高斯分布，对三点取对数后线性拟合，求极值点。
 * 精度：±0.01像素。
 *
 * @param y0 y(-1)
 * @param y1 y(0)
 * @param y2 y(+1)
 * @return 亚像素偏移量（范围 [-0.5, 0.5]）
 */
float gaussian_subpixel_offset(float y0, float y1, float y2);

/**
 * @brief Saddle点亚像素角点定位
 *
 * 在角点响应函数R局部极大值处拟合二次曲面
 *   R(x,y) = a*x^2 + b*y^2 + c*x*y + d*x + e*y + f
 * 求解Saddle点（梯度=0）位置，精度：±0.03像素。
 *
 * @param values 3x3邻域响应值（行优先：v00, v01, v02, v10, v11, v12, v20, v21, v22）
 * @param dx 输出亚像素X偏移（相对中心，范围 [-0.5, 0.5]）
 * @param dy 输出亚像素Y偏移
 * @return 是否成功（ False 表示奇异）
 */
bool saddle_subpixel_offset(const float values[9], float& dx, float& dy);

/**
 * @brief Zernike矩亚像素边缘定位（高级算法）
 *
 * 计算 Z20、Z22、Z11 矩，利用矩的旋转不变性推导亚像素边缘位置。
 * 精度：±0.005像素。
 *
 * @param values 5x5邻域灰度值（行优先，共25个元素）
 * @param sub_x 输出亚像素X偏移（相对中心，范围 [-0.5, 0.5]）
 * @param sub_y 输出亚像素Y偏移
 * @param angle 输出边缘方向（弧度）
 * @param strength 输出边缘强度
 * @return 是否成功
 */
bool zernike_subpixel_edge(const float values[25], float& sub_x, float& sub_y,
                           float& angle, float& strength);

// ------------------------------------------------------------------------
// 角点响应计算
// ------------------------------------------------------------------------

/**
 * @brief Harris角点响应
 * @param grad_x X方向梯度
 * @param grad_y Y方向梯度
 * @param width 图像宽度
 * @param height 图像高度
 * @param x 当前像素X
 * @param y 当前像素Y
 * @param k Harris灵敏度系数
 * @param window_size 邻域窗口大小
 * @return 角点响应值
 */
float harris_response(const std::vector<float>& grad_x,
                      const std::vector<float>& grad_y,
                      uint32_t width, uint32_t height,
                      uint32_t x, uint32_t y,
                      float k = 0.04f, int window_size = 3);

// ------------------------------------------------------------------------
// 测试图生成与噪声
// ------------------------------------------------------------------------

/**
 * @brief 生成已知参数的合成测试图
 * @param width 图像宽度
 * @param height 图像高度
 * @param type 测试图类型
 * @param param1 主参数（如圆心X、直线斜率、边缘位置等）
 * @param param2 副参数（如圆心Y、直线截距、圆半径等）
 * @param fg_value 前景灰度值
 * @param bg_value 背景灰度值
 * @return 合成图像
 */
ImageData generate_test_pattern(uint32_t width, uint32_t height,
                                TestPatternType type,
                                float param1, float param2,
                                uint8_t fg_value = 200, uint8_t bg_value = 50);

/**
 * @brief 添加高斯白噪声
 * @param image 输入图像（将被修改）
 * @param snr_db 信噪比（dB），常用 20/40/60
 */
void add_gaussian_noise(ImageData& image, float snr_db);

/**
 * @brief 计算精度统计
 * @param errors 误差样本列表
 * @param target_precision 目标精度
 * @return 统计结果
 */
PrecisionStats compute_precision_stats(const std::vector<float>& errors,
                                        float target_precision = 0.01f);

} // namespace subpixel_utils

// ============================================================================
// 节点类声明
// ============================================================================

/**
 * @brief 亚像素边缘定位节点（泰勒展开法，对标Halcon edges_sub_pix）
 *
 * 算法流程：Sobel梯度 -> 沿梯度方向插值 -> 二阶泰勒展开求极值点
 * 支持方法：taylor / parabola / gaussian / zernike
 */
class SubpixelEdgeNode : public INode {
public:
    SubpixelEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 亚像素角点定位节点（Saddle点法）
 *
 * 角点响应函数 R = det(M) - k*trace(M)^2，
 * 在R局部最大值处拟合二次曲面，求解Saddle点位置。
 */
class SubpixelCornerNode : public INode {
public:
    SubpixelCornerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 亚像素直线定位节点
 *
 * 提取亚像素边缘点后，使用最小二乘法拟合直线，并计算RMS误差。
 */
class SubpixelLineNode : public INode {
public:
    SubpixelLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 亚像素圆定位节点
 *
 * 提取圆弧上的亚像素边缘点后，使用最小二乘法拟合圆，并计算RMS误差。
 */
class SubpixelCircleNode : public INode {
public:
    SubpixelCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 亚像素测量节点（高斯插值）
 *
 * 沿给定路径提取灰度轮廓，使用高斯函数拟合边缘响应极值，
 * 精度：±0.01像素。
 */
class SubpixelMeasureNode : public INode {
public:
    SubpixelMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 亚像素卡尺节点（抛物线拟合）
 *
 * 在梯度最大值附近取3点，抛物线拟合 y = ax^2 + bx + c，
 * 极值点 x = -b/(2a)，精度：±0.02像素。
 */
class SubpixelCaliperNode : public INode {
public:
    SubpixelCaliperNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 亚像素轮廓提取节点
 *
 * 在边缘点列表基础上，沿梯度方向细化得到亚像素级轮廓。
 */
class SubpixelContourNode : public INode {
public:
    SubpixelContourNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 精度测试节点（生成标准测试图）
 *
 * 生成已知参数的合成测试图（圆、直线、边缘），用于精度验证。
 */
class PrecisionTestNode : public INode {
public:
    PrecisionTestNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 精度基准测试节点（与理论值对比）
 *
 * 在测试图上运行亚像素定位算法，与已知理论值对比，输出统计误差。
 */
class PrecisionBenchmarkNode : public INode {
public:
    PrecisionBenchmarkNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 精度报告生成节点
 *
 * 汇总多次精度测试结果，输出均值误差、标准差、最大误差，并判定达标。
 */
class PrecisionReportNode : public INode {
public:
    PrecisionReportNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf
