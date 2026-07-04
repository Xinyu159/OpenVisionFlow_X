/**
 * @file measurement.h
 * @brief 测量模块（卡尺工具、拟合算法、距离和角度测量）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 二维点结构
 */
struct Point2Df {
    float x = 0.0f;
    float y = 0.0f;
    
    Point2Df() = default;
    Point2Df(float x_, float y_) : x(x_), y(y_) {}
    
    float distance_to(const Point2Df& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

/**
 * @brief 直线结构（参数方程：ax + by + c = 0）
 */
struct Line2D {
    float a = 0.0f;  // x 系数
    float b = 0.0f;  // y 系数
    float c = 0.0f;  // 常数项
    
    // 归一化法向量
    void normalize() {
        float len = std::sqrt(a * a + b * b);
        if (len > 1e-10f) {
            a /= len;
            b /= len;
            c /= len;
        }
    }
    
    // 点到直线距离
    float distance_to_point(float x, float y) const {
        return std::abs(a * x + b * y + c) / std::sqrt(a * a + b * b);
    }
    
    float distance_to_point(const Point2Df& p) const {
        return distance_to_point(p.x, p.y);
    }
};

/**
 * @brief 圆结构
 */
struct Circle2D {
    float center_x = 0.0f;
    float center_y = 0.0f;
    float radius = 0.0f;
    float error = 0.0f;  // 拟合误差
    bool valid = false;
};

/**
 * @brief 测量工具函数
 */
namespace measurement_utils {

/**
 * @brief 直线拟合（最小二乘法）
 * @param points 输入点集
 * @return 拟合直线
 */
inline Line2D fit_line(const std::vector<Point2Df>& points) {
    Line2D line;
    
    if (points.size() < 2) {
        return line;
    }
    
    size_t n = points.size();
    
    // 计算均值
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;
    
    // 计算协方差矩阵
    double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
    for (const auto& p : points) {
        double dx = p.x - mean_x;
        double dy = p.y - mean_y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }
    
    // 主成分分析（PCA）- 找最小特征值对应的特征向量
    // 对于直线 ax + by + c = 0，(a, b) 是协方差矩阵的最小特征向量
    
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    
    // 特征值
    double lambda1 = (trace + std::sqrt(trace * trace - 4 * det)) / 2.0;
    double lambda2 = (trace - std::sqrt(trace * trace - 4 * det)) / 2.0;
    
    // 使用最小特征值对应的特征向量
    if (std::abs(cov_xy) > 1e-10) {
        line.a = static_cast<float>(lambda2 - cov_yy);
        line.b = static_cast<float>(-cov_xy);
    } else if (cov_xx > cov_yy) {
        line.a = 0.0f;
        line.b = 1.0f;
    } else {
        line.a = 1.0f;
        line.b = 0.0f;
    }
    
    // 计算常数项
    line.c = -(line.a * mean_x + line.b * mean_y);
    
    // 归一化
    line.normalize();
    
    return line;
}

/**
 * @brief 圆拟合（最小二乘法）
 * @param points 输入点集
 * @return 拟合圆
 */
inline Circle2D fit_circle(const std::vector<Point2Df>& points) {
    Circle2D circle;
    
    if (points.size() < 3) {
        return circle;
    }
    
    size_t n = points.size();
    
    // 使用 Kasa 方法（代数拟合）
    // 最小化 sum((x - cx)^2 + (y - cy)^2 - R^2)^2
    
    // 构建线性方程组 A*X = B
    // 其中 X = [cx, cy, cx^2 + cy^2 - R^2]
    
    double sum_x = 0.0, sum_y = 0.0;
    double sum_x2 = 0.0, sum_y2 = 0.0;
    double sum_x3 = 0.0, sum_y3 = 0.0;
    double sum_xy = 0.0, sum_x2y = 0.0, sum_xy2 = 0.0;
    
    for (const auto& p : points) {
        double x = p.x, y = p.y;
        double x2 = x * x, y2 = y * y;
        
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x2y += x2 * y;
        sum_xy2 += x * y2;
    }
    
    // 求解 3x3 线性方程组
    double A = n * sum_x2 - sum_x * sum_x;
    double B = n * sum_xy - sum_x * sum_y;
    double C = n * sum_y2 - sum_y * sum_y;
    
    double D = n * sum_x3 + n * sum_xy2 - (sum_x2 + sum_y2) * sum_x;
    double E = n * sum_x2y + n * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    
    double det = A * C - B * B;
    
    if (std::abs(det) < 1e-10) {
        circle.valid = false;
        return circle;
    }
    
    double cx = (D * C - B * E) / det / 2.0;
    double cy = (A * E - B * D) / det / 2.0;
    
    double r_sum = 0.0;
    for (const auto& p : points) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        r_sum += std::sqrt(dx * dx + dy * dy);
    }
    double r = r_sum / n;
    
    circle.center_x = static_cast<float>(cx);
    circle.center_y = static_cast<float>(cy);
    circle.radius = static_cast<float>(r);
    
    // 计算拟合误差
    double total_error = 0.0;
    for (const auto& p : points) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        double dist = std::sqrt(dx * dx + dy * dy);
        total_error += std::abs(dist - r);
    }
    circle.error = static_cast<float>(total_error / n);
    circle.valid = true;
    
    return circle;
}

/**
 * @brief 计算两点距离
 */
inline float point_distance(const Point2Df& p1, const Point2Df& p2) {
    return p1.distance_to(p2);
}

/**
 * @brief 计算点到直线距离
 */
inline float point_to_line_distance(const Point2Df& point, const Line2D& line) {
    return line.distance_to_point(point);
}

/**
 * @brief 计算两条直线的夹角（弧度）
 */
inline float line_angle(const Line2D& l1, const Line2D& l2) {
    // 计算法向量的夹角
    float dot = l1.a * l2.a + l1.b * l2.b;
    float cross = l1.a * l2.b - l1.b * l2.a;
    
    // 返回较小的夹角
    float angle = std::atan2(std::abs(cross), std::abs(dot));
    return angle;
}

/**
 * @brief 卡尺工具：沿投影方向搜索边缘跳变点
 * @param image 图像数据（灰度）
 * @param start_x 起始点X
 * @param start_y 起始点Y
 * @param end_x 结束点X
 * @param end_y 结束点Y
 * @param width 搜索宽度（垂直于投影方向的宽度）
 * @param threshold 边缘阈值
 * @param polarity 极性：1=亮到暗，-1=暗到亮，0=双向
 * @return 边缘点集合
 */
inline std::vector<Point2Df> caliper_search(const ImageData& image,
                                             float start_x, float start_y,
                                             float end_x, float end_y,
                                             int width,
                                             float threshold,
                                             int polarity = 0) {
    std::vector<Point2Df> edge_points;
    
    if (image.empty() || image.channels != 1) {
        return edge_points;
    }
    
    // 计算投影方向
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float len = std::sqrt(dx * dx + dy * dy);
    
    if (len < 1.0f) {
        return edge_points;
    }
    
    // 归一化方向向量
    float dir_x = dx / len;
    float dir_y = dy / len;
    
    // 垂直方向（用于计算搜索宽度）
    float perp_x = -dir_y;
    float perp_y = dir_x;
    
    // 沿投影方向采样
    int num_samples = static_cast<int>(len);
    if (num_samples < 2) num_samples = 2;
    
    std::vector<float> profile(num_samples);
    std::vector<float> sample_x(num_samples);
    std::vector<float> sample_y(num_samples);
    
    for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / (num_samples - 1);
        float cx = start_x + t * dx;
        float cy = start_y + t * dy;
        sample_x[i] = cx;
        sample_y[i] = cy;
        
        // 在垂直方向上平均采样
        float sum = 0.0f;
        int count = 0;
        
        int half_width = width / 2;
        for (int w = -half_width; w <= half_width; ++w) {
            float px = cx + w * perp_x;
            float py = cy + w * perp_y;
            
            int ix = static_cast<int>(px);
            int iy = static_cast<int>(py);
            
            if (ix >= 0 && ix < static_cast<int>(image.width) &&
                iy >= 0 && iy < static_cast<int>(image.height)) {
                sum += image.data[iy * image.width + ix];
                ++count;
            }
        }
        
        profile[i] = (count > 0) ? sum / count : 0.0f;
    }
    
    // 在轮廓中寻找边缘跳变
    for (int i = 1; i < num_samples; ++i) {
        float diff = profile[i] - profile[i - 1];
        
        bool is_edge = false;
        if (polarity > 0 && diff > threshold) {
            is_edge = true;  // 亮到暗
        } else if (polarity < 0 && diff < -threshold) {
            is_edge = true;  // 暗到亮
        } else if (polarity == 0 && std::abs(diff) > threshold) {
            is_edge = true;  // 双向
        }
        
        if (is_edge) {
            // 亚像素边缘定位
            float edge_x = (sample_x[i] + sample_x[i - 1]) / 2.0f;
            float edge_y = (sample_y[i] + sample_y[i - 1]) / 2.0f;
            edge_points.emplace_back(edge_x, edge_y);
        }
    }
    
    return edge_points;
}

/**
 * @brief 计算两条直线的交点
 */
inline Point2Df line_intersection(const Line2D& l1, const Line2D& l2) {
    float det = l1.a * l2.b - l2.a * l1.b;
    
    if (std::abs(det) < 1e-10f) {
        return Point2Df(0, 0);  // 平行线无交点
    }
    
    float x = (l1.b * l2.c - l2.b * l1.c) / det;
    float y = (l2.a * l1.c - l1.a * l2.c) / det;
    
    return Point2Df(x, y);
}

} // namespace measurement_utils

/**
 * @brief 卡尺测量节点
 */
class CaliperToolNode : public INode {
public:
    CaliperToolNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 直线拟合节点
 */
class LineFitNode : public INode {
public:
    LineFitNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 圆拟合节点
 */
class CircleFitNode : public INode {
public:
    CircleFitNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 距离测量节点
 */
class DistanceMeasureNode : public INode {
public:
    DistanceMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 角度测量节点
 */
class AngleMeasureNode : public INode {
public:
    AngleMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf