/**
 * @file shape_descriptor.h
 * @brief 形状描述符和形状特征算子（纯C++实现）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <complex>
#include <algorithm>

namespace ovf {
namespace algorithm {

/**
 * @brief 轮廓点结构
 */
struct ContourPoint {
    float x = 0.0f;
    float y = 0.0f;
    
    ContourPoint() = default;
    ContourPoint(float x_, float y_) : x(x_), y(y_) {}
    
    float distance_to(const ContourPoint& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

/**
 * @brief 形状描述符结构
 */
struct ShapeDescriptor {
    std::vector<double> hu_moments;       // Hu不变矩（7个）
    std::vector<double> flusser_moments;  // Flusser不变矩
    double area = 0.0;                    // 面积
    double perimeter = 0.0;               // 周长
    double centroid_x = 0.0;              // 质心X
    double centroid_y = 0.0;              // 质心Y
    double circularity = 0.0;             // 圆度
    double rectangularity = 0.0;          // 矩形度
    double convexity = 0.0;               // 凸度
    double compactness = 0.0;             // 紧凑度
    double anisometry = 0.0;              // 各向异性
    double orientation = 0.0;             // 方向角
    double major_axis = 0.0;              // 主轴长度
    double minor_axis = 0.0;              // 次轴长度
    double eccentricity = 0.0;            // 偏心率
};

/**
 * @brief 形状描述符工具函数
 */
namespace shape_utils {

/**
 * @brief 从图像提取轮廓（边界跟踪算法）
 */
std::vector<ContourPoint> extract_contour(const ImageData& binary_img);

/**
 * @brief 计算轮廓面积
 */
double contour_area(const std::vector<ContourPoint>& contour);

/**
 * @brief 计算轮廓周长
 */
double contour_perimeter(const std::vector<ContourPoint>& contour);

/**
 * @brief 计算轮廓质心
 */
void contour_centroid(const std::vector<ContourPoint>& contour, double& cx, double& cy);

/**
 * @brief 计算几何矩 m_pq
 */
double geometric_moment(const std::vector<ContourPoint>& contour, int p, int q, double cx = 0, double cy = 0);

/**
 * @brief 计算中心矩 mu_pq
 */
double central_moment(const std::vector<ContourPoint>& contour, int p, int q);

/**
 * @brief 计算归一化中心矩 nu_pq
 */
double normalized_central_moment(const std::vector<ContourPoint>& contour, int p, int q);

/**
 * @brief 计算Hu不变矩（7个）
 */
std::vector<double> hu_moments(const std::vector<ContourPoint>& contour);

/**
 * @brief 计算Flusser不变矩
 */
std::vector<double> flusser_moments(const std::vector<ContourPoint>& contour);

/**
 * @brief 计算形状的凸包
 */
std::vector<ContourPoint> convex_hull(const std::vector<ContourPoint>& contour);

/**
 * @brief 计算圆度 circularity = 4*pi*A/P^2
 */
double circularity(double area, double perimeter);

/**
 * @brief 计算矩形度 rectangularity = A/(w*h)
 */
double rectangularity(double area, double width, double height);

/**
 * @brief 计算凸度 convexity = A_hull/A
 */
double convexity(double area, double hull_area);

/**
 * @brief 计算紧凑度 compactness = sqrt(4*A/pi)/P
 */
double compactness(double area, double perimeter);

/**
 * @brief 计算各向异性 anisometry = major_axis/minor_axis
 */
void compute_axes(const std::vector<ContourPoint>& contour, 
                  double& major_axis, double& minor_axis, double& orientation);

/**
 * @brief 计算Fourier描述符
 */
std::vector<std::complex<double>> fourier_descriptors(const std::vector<ContourPoint>& contour, int num_descriptors = 32);

/**
 * @brief 计算曲率描述符
 */
std::vector<double> curvature_descriptor(const std::vector<ContourPoint>& contour, int num_points = 64);

/**
 * @brief 计算轮廓签名（距离签名）
 */
std::vector<double> contour_signature(const std::vector<ContourPoint>& contour, int num_samples = 64);

/**
 * @brief 计算形状上下文描述符（简化版）
 */
std::vector<std::vector<int>> shape_context(const std::vector<ContourPoint>& contour, 
                                             int num_points = 64, 
                                             int num_r_bins = 5, 
                                             int num_theta_bins = 12);

} // namespace shape_utils

// ==================== 不变矩节点 ====================

/**
 * @brief Hu不变矩节点
 */
class HuMomentsNode : public INode {
public:
    HuMomentsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 形状矩节点
 */
class ShapeMomentsNode : public INode {
public:
    ShapeMomentsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief Flusser不变矩节点
 */
class FlusserMomentsNode : public INode {
public:
    FlusserMomentsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 矩不变量节点
 */
class MomentInvariantsNode : public INode {
public:
    MomentInvariantsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

// ==================== 形状特征节点 ====================

/**
 * @brief 形状特征节点
 */
class ShapeFeaturesNode : public INode {
public:
    ShapeFeaturesNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 紧凑度节点
 */
class CompactnessNode : public INode {
public:
    CompactnessNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 凸度节点
 */
class ConvexityNode : public INode {
public:
    ConvexityNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 各向异性节点
 */
class AnisometryNode : public INode {
public:
    AnisometryNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

// ==================== 轮廓描述符节点 ====================

/**
 * @brief Fourier轮廓描述符节点
 */
class FourierDescriptorNode : public INode {
public:
    FourierDescriptorNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 形状上下文描述符节点
 */
class ShapeContextNode : public INode {
public:
    ShapeContextNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 曲率描述符节点
 */
class CurvatureDescriptorNode : public INode {
public:
    CurvatureDescriptorNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 轮廓签名节点
 */
class ContourSignatureNode : public INode {
public:
    ContourSignatureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf