/**
 * @file position_guiding.h
 * @brief 定位引导流程模块
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 标定结果结构（与calibration.h一致）
 */
struct CalibResult {
    float a = 0.0f;  // 仿射变换参数：world_x = a * image_x + b * image_y + c
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;  // 仿射变换参数：world_y = d * image_x + e * image_y + f
    float e = 0.0f;
    float f = 0.0f;
    bool valid = false;
    
    // 应用变换：图像坐标 -> 物理坐标
    void transform(float image_x, float image_y, float& world_x, float& world_y) const {
        world_x = a * image_x + b * image_y + c;
        world_y = d * image_x + e * image_y + f;
    }
};

/**
 * @brief 2D点结构
 */
struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
    
    Point2D() = default;
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

/**
 * @brief 对位补正结果
 */
struct AlignmentResult {
    float offset_x = 0.0f;     // 平移偏差X
    float offset_y = 0.0f;     // 平移偏差Y
    float rotation = 0.0f;     // 旋转偏差（度）
    float scale_x = 1.0f;      // 缩放偏差X
    float scale_y = 1.0f;      // 缩放偏差Y
    bool valid = false;
};

/**
 * @brief 运动指令
 */
struct MotionCommand {
    float x = 0.0f;            // 目标位置X
    float y = 0.0f;            // 目标位置Y
    float z = 0.0f;            // 目标位置Z
    float rx = 0.0f;           // 目标姿态RX
    float ry = 0.0f;           // 目标姿态RY
    float rz = 0.0f;           // 目标姿态RZ
    int motion_type = 0;        // 运动类型(0:PTP, 1:直线)
    bool valid = false;
};

/**
 * @brief 匹配点对（用于仿射变换计算）
 */
struct MatchPointPair {
    Point2D source_point;      // 源点（模板坐标）
    Point2D target_point;      // 目标点（当前图像坐标）
};

/**
 * @brief 仿射变换矩阵（2x3）
 */
struct AffineTransform {
    float a = 1.0f;  // 缩放/旋转
    float b = 0.0f;  // 旋转/缩放
    float c = 0.0f;  // 平移X
    float d = 0.0f;  // 旋转/缩放
    float e = 1.0f;  // 缩放/旋转
    float f = 0.0f;  // 平移Y
    bool valid = false;
    
    // 应用变换
    void transform(float in_x, float in_y, float& out_x, float& out_y) const {
        out_x = a * in_x + b * in_y + c;
        out_y = d * in_x + e * in_y + f;
    }
    
    // 获取平移量
    float get_translation_x() const { return c; }
    float get_translation_y() const { return f; }
    
    // 获取旋转角度（弧度）
    float get_rotation_rad() const {
        return std::atan2(b, a);
    }
    
    // 获取旋转角度（度）
    float get_rotation_deg() const {
        return get_rotation_rad() * 180.0f / static_cast<float>(M_PI);
    }
    
    // 获取缩放
    float get_scale_x() const {
        return std::sqrt(a * a + d * d);
    }
    
    float get_scale_y() const {
        return std::sqrt(b * b + e * e);
    }
};

/**
 * @brief 坐标转换工具函数
 */
namespace coord_transform_utils {

/**
 * @brief 使用标定结果进行坐标转换
 */
inline Point2D transform(const CalibResult& calib, const Point2D& image_pt) {
    Point2D world_pt;
    calib.transform(image_pt.x, image_pt.y, world_pt.x, world_pt.y);
    return world_pt;
}

/**
 * @brief 批量坐标转换
 */
inline std::vector<Point2D> transform_batch(const CalibResult& calib, 
                                             const std::vector<Point2D>& image_points) {
    std::vector<Point2D> world_points;
    world_points.reserve(image_points.size());
    
    for (const auto& pt : image_points) {
        world_points.push_back(transform(calib, pt));
    }
    
    return world_points;
}

} // namespace coord_transform_utils

/**
 * @brief 对位补正工具函数
 */
namespace alignment_utils {

/**
 * @brief 计算对位偏差
 * @param target 目标位置
 * @param actual 实际位置
 * @param target_angle 目标角度（度）
 * @param actual_angle 实际角度（度）
 * @return 对位结果
 */
inline AlignmentResult calculate_alignment(const Point2D& target, const Point2D& actual,
                                           float target_angle = 0.0f, float actual_angle = 0.0f) {
    AlignmentResult result;
    
    // 计算平移偏差
    result.offset_x = target.x - actual.x;
    result.offset_y = target.y - actual.y;
    
    // 计算旋转偏差
    result.rotation = target_angle - actual_angle;
    
    // 归一化角度到 -180 ~ 180
    while (result.rotation > 180.0f) result.rotation -= 360.0f;
    while (result.rotation < -180.0f) result.rotation += 360.0f;
    
    // 缩放偏差默认为1.0
    result.scale_x = 1.0f;
    result.scale_y = 1.0f;
    
    result.valid = true;
    return result;
}

/**
 * @brief 从仿射变换提取对位偏差
 */
inline AlignmentResult extract_alignment_from_transform(const AffineTransform& transform) {
    AlignmentResult result;
    
    result.offset_x = transform.get_translation_x();
    result.offset_y = transform.get_translation_y();
    result.rotation = transform.get_rotation_deg();
    result.scale_x = transform.get_scale_x();
    result.scale_y = transform.get_scale_y();
    result.valid = transform.valid;
    
    return result;
}

} // namespace alignment_utils

/**
 * @brief 仿射变换计算工具函数
 */
namespace affine_utils {

/**
 * @brief 根据匹配点对计算仿射变换（最少3个点）
 * @param point_pairs 匹配点对列表
 * @return 仿射变换矩阵
 */
inline AffineTransform calculate_affine_transform(const std::vector<MatchPointPair>& point_pairs) {
    AffineTransform result;
    
    if (point_pairs.size() < 3) {
        result.valid = false;
        return result;
    }
    
    size_t n = point_pairs.size();
    
    // 使用最小二乘法计算仿射变换
    // 方程组：[x'] = [a b c] [x]
    //        [y']   [d e f] [y]
    //                         [1]
    
    // 构建矩阵方程 AX = B
    // 对于每个点对：
    // x' = a*x + b*y + c
    // y' = d*x + e*y + f
    
    double sum_xx = 0.0, sum_xy = 0.0, sum_x = 0.0;
    double sum_yy = 0.0, sum_y = 0.0, sum_n = static_cast<double>(n);
    double sum_x_prime = 0.0, sum_y_prime = 0.0;
    double sum_xx_prime = 0.0, sum_xy_prime = 0.0, sum_x_y_prime = 0.0;
    double sum_xx__y_prime = 0.0, sum_xy__y_prime = 0.0, sum_x__y_prime = 0.0;
    
    for (const auto& pair : point_pairs) {
        double sx = pair.source_point.x;
        double sy = pair.source_point.y;
        double tx = pair.target_point.x;
        double ty = pair.target_point.y;
        
        sum_xx += sx * sx;
        sum_xy += sx * sy;
        sum_x += sx;
        sum_yy += sy * sy;
        sum_y += sy;
        
        sum_x_prime += tx;
        sum_y_prime += ty;
        
        sum_xx_prime += sx * sx * tx;
        sum_xy_prime += sx * sy * tx;
        sum_x_y_prime += sx * tx;
        
        sum_xx__y_prime += sx * sx * ty;
        sum_xy__y_prime += sx * sy * ty;
        sum_x__y_prime += sx * ty;
    }
    
    // 求解 a, b, c
    double det = sum_xx * (sum_yy * sum_n - sum_y * sum_y)
               - sum_xy * (sum_xy * sum_n - sum_y * sum_x)
               + sum_x * (sum_xy * sum_y - sum_yy * sum_x);
    
    if (std::abs(det) < 1e-10) {
        result.valid = false;
        return result;
    }
    
    double det_a = sum_xx_prime * (sum_yy * sum_n - sum_y * sum_y)
                 - sum_xy_prime * (sum_xy * sum_n - sum_y * sum_x)
                 + sum_x_y_prime * (sum_xy * sum_y - sum_yy * sum_x);
    
    double det_b = sum_xx * (sum_xy_prime * sum_n - sum_y * sum_x_y_prime)
                 - sum_xy * (sum_xx_prime * sum_n - sum_y * sum_x_y_prime)
                 + sum_x * (sum_xx_prime * sum_y - sum_xy_prime * sum_x);
    
    double det_c = sum_xx * (sum_yy * sum_x_y_prime - sum_xy_prime * sum_y)
                 - sum_xy * (sum_xy * sum_x_y_prime - sum_xx_prime * sum_y)
                 + sum_x * (sum_xy * sum_xy_prime - sum_yy * sum_xx_prime);
    
    result.a = static_cast<float>(det_a / det);
    result.b = static_cast<float>(det_b / det);
    result.c = static_cast<float>(det_c / det);
    
    // 求解 d, e, f
    double det_d = sum_xx__y_prime * (sum_yy * sum_n - sum_y * sum_y)
                 - sum_xy__y_prime * (sum_xy * sum_n - sum_y * sum_x)
                 + sum_x__y_prime * (sum_xy * sum_y - sum_yy * sum_x);
    
    double det_e = sum_xx * (sum_xy__y_prime * sum_n - sum_y * sum_x__y_prime)
                 - sum_xy * (sum_xx__y_prime * sum_n - sum_y * sum_x__y_prime)
                 + sum_x * (sum_xx__y_prime * sum_y - sum_xy__y_prime * sum_x);
    
    double det_f = sum_xx * (sum_yy * sum_x__y_prime - sum_xy__y_prime * sum_y)
                 - sum_xy * (sum_xy * sum_x__y_prime - sum_xx__y_prime * sum_y)
                 + sum_x * (sum_xy * sum_xy__y_prime - sum_yy * sum_xx__y_prime);
    
    result.d = static_cast<float>(det_d / det);
    result.e = static_cast<float>(det_e / det);
    result.f = static_cast<float>(det_f / det);
    
    result.valid = true;
    return result;
}

} // namespace affine_utils

/**
 * @brief 坐标转换节点
 * 使用标定结果将图像坐标转换为物理坐标
 */
class CoordinateTransformNode : public INode {
public:
    CoordinateTransformNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 对位补正节点
 * 计算目标位置与实际位置的偏差
 */
class AlignmentCorrectionNode : public INode {
public:
    AlignmentCorrectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 机器人引导节点
 * 输出运动指令
 */
class RobotGuideNode : public INode {
public:
    RobotGuideNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 位置匹配节点
 * 模板匹配+坐标输出
 */
class PositionMatchNode : public INode {
public:
    PositionMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 仿射变换计算节点
 * 根据匹配点对计算变换
 */
class AffineTransformCalcNode : public INode {
public:
    AffineTransformCalcNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf