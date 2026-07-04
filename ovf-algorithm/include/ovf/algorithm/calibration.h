/**
 * @file calibration.h
 * @brief 标定算法模块（九点标定）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 标定点结构
 */
struct CalibrationPoint {
    float image_x = 0.0f;   // 图像X坐标
    float image_y = 0.0f;  // 图像Y坐标
    float world_x = 0.0f;  // 物理X坐标
    float world_y = 0.0f;  // 物理Y坐标
    
    CalibrationPoint() = default;
    CalibrationPoint(float ix, float iy, float wx, float wy)
        : image_x(ix), image_y(iy), world_x(wx), world_y(wy) {}
};

/**
 * @brief 标定结果结构
 */
struct CalibrationResult {
    float a = 0.0f;  // 仿射变换参数：world_x = a * image_x + b * image_y + c
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;  // 仿射变换参数：world_y = d * image_x + e * image_y + f
    float e = 0.0f;
    float f = 0.0f;
    float error = 0.0f;     // 平均误差
    float max_error = 0.0f; // 最大误差
    bool valid = false;
    
    // 应用变换：图像坐标 -> 物理坐标
    void transform(float image_x, float image_y, float& world_x, float& world_y) const {
        world_x = a * image_x + b * image_y + c;
        world_y = d * image_x + e * image_y + f;
    }
    
    // 逆变换：物理坐标 -> 图像坐标
    void inverse_transform(float world_x, float world_y, float& image_x, float& image_y) const {
        // 计算逆矩阵
        float det = a * e - b * d;
        if (std::abs(det) < 1e-10f) {
            image_x = world_x;
            image_y = world_y;
            return;
        }
        
        float inv_a = e / det;
        float inv_b = -b / det;
        float inv_d = -d / det;
        float inv_e = a / det;
        
        float tx = world_x - c;
        float ty = world_y - f;
        
        image_x = inv_a * tx + inv_b * ty;
        image_y = inv_d * tx + inv_e * ty;
    }
};

/**
 * @brief 标定工具函数
 */
namespace calibration_utils {

/**
 * @brief 九点标定算法（最小二乘法求解仿射变换）
 * @param points 标定点集合（至少需要3个点）
 * @return 标定结果
 */
inline CalibrationResult nine_point_calibration(const std::vector<CalibrationPoint>& points) {
    CalibrationResult result;
    
    if (points.size() < 3) {
        result.valid = false;
        return result;
    }
    
    size_t n = points.size();
    
    // 构建矩阵方程 AX = B（使用最小二乘法）
    // 对于仿射变换：
    // world_x = a * image_x + b * image_y + c
    // world_y = d * image_x + e * image_y + f
    
    // 计算矩阵 A^T * A 和 A^T * B
    double sum_xx = 0.0, sum_xy = 0.0, sum_x = 0.0;
    double sum_yy = 0.0, sum_y = 0.0, sum_n = static_cast<double>(n);
    double sum_wx = 0.0, sum_wy = 0.0;
    double sum_xx_wx = 0.0, sum_xy_wx = 0.0, sum_x_wx = 0.0;
    double sum_xx_wy = 0.0, sum_xy_wy = 0.0, sum_x_wy = 0.0;
    
    for (const auto& p : points) {
        double ix = p.image_x;
        double iy = p.image_y;
        double wx = p.world_x;
        double wy = p.world_y;
        
        sum_xx += ix * ix;
        sum_xy += ix * iy;
        sum_x += ix;
        sum_yy += iy * iy;
        sum_y += iy;
        
        sum_wx += wx;
        sum_wy += wy;
        
        sum_xx_wx += ix * ix * wx;
        sum_xy_wx += ix * iy * wx;
        sum_x_wx += ix * wx;
        
        sum_xx_wy += ix * ix * wy;
        sum_xy_wy += ix * iy * wy;
        sum_x_wy += ix * wy;
    }
    
    // 计算 (A^T * A)^-1 * A^T * B
    // 矩阵形式：
    // [sum_xx  sum_xy  sum_x ] [a]   [sum_xx_wx]
    // [sum_xy  sum_yy  sum_y ] [b] = [sum_xy_wx]
    // [sum_x   sum_y   sum_n ] [c]   [sum_x_wx ]
    
    // 使用克拉默法则求解 3x3 线性方程组
    double det = sum_xx * (sum_yy * sum_n - sum_y * sum_y)
               - sum_xy * (sum_xy * sum_n - sum_y * sum_x)
               + sum_x * (sum_xy * sum_y - sum_yy * sum_x);
    
    if (std::abs(det) < 1e-10) {
        result.valid = false;
        return result;
    }
    
    // 求解 a, b, c
    double det_a = sum_xx_wx * (sum_yy * sum_n - sum_y * sum_y)
                 - sum_xy_wx * (sum_xy * sum_n - sum_y * sum_x)
                 + sum_x_wx * (sum_xy * sum_y - sum_yy * sum_x);
    
    double det_b = sum_xx * (sum_xy_wx * sum_n - sum_y * sum_x_wx)
                 - sum_xy * (sum_xx_wx * sum_n - sum_y * sum_x_wx)
                 + sum_x * (sum_xx_wx * sum_y - sum_xy_wx * sum_x);
    
    double det_c = sum_xx * (sum_yy * sum_x_wx - sum_xy_wx * sum_y)
                 - sum_xy * (sum_xy * sum_x_wx - sum_xx_wx * sum_y)
                 + sum_x * (sum_xy * sum_xy_wx - sum_yy * sum_xx_wx);
    
    result.a = static_cast<float>(det_a / det);
    result.b = static_cast<float>(det_b / det);
    result.c = static_cast<float>(det_c / det);
    
    // 求解 d, e, f
    double det_d = sum_xx_wy * (sum_yy * sum_n - sum_y * sum_y)
                 - sum_xy_wy * (sum_xy * sum_n - sum_y * sum_x)
                 + sum_x_wy * (sum_xy * sum_y - sum_yy * sum_x);
    
    double det_e = sum_xx * (sum_xy_wy * sum_n - sum_y * sum_x_wy)
                 - sum_xy * (sum_xx_wy * sum_n - sum_y * sum_x_wy)
                 + sum_x * (sum_xx_wy * sum_y - sum_xy_wy * sum_x);
    
    double det_f = sum_xx * (sum_yy * sum_x_wy - sum_xy_wy * sum_y)
                 - sum_xy * (sum_xy * sum_x_wy - sum_xx_wy * sum_y)
                 + sum_x * (sum_xy * sum_xy_wy - sum_yy * sum_xx_wy);
    
    result.d = static_cast<float>(det_d / det);
    result.e = static_cast<float>(det_e / det);
    result.f = static_cast<float>(det_f / det);
    
    // 计算误差
    double total_error = 0.0;
    double max_err = 0.0;
    
    for (const auto& p : points) {
        float pred_wx, pred_wy;
        result.transform(p.image_x, p.image_y, pred_wx, pred_wy);
        
        double err_x = pred_wx - p.world_x;
        double err_y = pred_wy - p.world_y;
        double err = std::sqrt(err_x * err_x + err_y * err_y);
        
        total_error += err;
        max_err = std::max(max_err, err);
    }
    
    result.error = static_cast<float>(total_error / n);
    result.max_error = static_cast<float>(max_err);
    result.valid = true;
    
    return result;
}

/**
 * @brief 验证标定结果
 * @param result 标定结果
 * @param points 验证点集
 * @return 平均误差
 */
inline float verify_calibration(const CalibrationResult& result, 
                                const std::vector<CalibrationPoint>& points) {
    if (!result.valid || points.empty()) {
        return -1.0f;
    }
    
    double total_error = 0.0;
    for (const auto& p : points) {
        float pred_wx, pred_wy;
        result.transform(p.image_x, p.image_y, pred_wx, pred_wy);
        
        double err_x = pred_wx - p.world_x;
        double err_y = pred_wy - p.world_y;
        total_error += std::sqrt(err_x * err_x + err_y * err_y);
    }
    
    return static_cast<float>(total_error / points.size());
}

} // namespace calibration_utils

/**
 * @brief 九点标定节点
 */
class NinePointCalibrationNode : public INode {
public:
    NinePointCalibrationNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    CalibrationResult result_;
};

} // namespace algorithm
} // namespace ovf