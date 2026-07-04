/**
 * @file geometry_measure.h
 * @brief 几何测量模块 - 包含基本测量、形状测量和拟合测量节点
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 椭圆结构
 */
struct Ellipse2D {
    float center_x = 0.0f;      // 中心X
    float center_y = 0.0f;      // 中心Y
    float major_axis = 0.0f;    // 长轴
    float minor_axis = 0.0f;    // 短轴
    float angle = 0.0f;         // 旋转角度（弧度）
    float error = 0.0f;         // 拟合误差
    bool valid = false;
};

/**
 * @brief 矩形结构
 */
struct Rectangle2D {
    float x = 0.0f;             // 左上角X
    float y = 0.0f;             // 左上角Y
    float width = 0.0f;        // 宽度
    float height = 0.0f;        // 高度
    float angle = 0.0f;         // 旋转角度（弧度）
};

/**
 * @brief 多边形结构
 */
struct Polygon2D {
    std::vector<float> x_coords;  // X坐标
    std::vector<float> y_coords;  // Y坐标
    int vertex_count = 0;
};

/**
 * @brief 几何测量工具函数
 */
namespace geometry_measure_utils {

/**
 * @brief 计算多边形面积（Shoelace公式）
 */
inline float polygon_area(const std::vector<float>& x, const std::vector<float>& y) {
    if (x.size() < 3 || y.size() != x.size()) return 0.0f;
    
    float area = 0.0f;
    size_t n = x.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += x[i] * y[j];
        area -= x[j] * y[i];
    }
    
    return std::abs(area) / 2.0f;
}

/**
 * @brief 计算多边形周长
 */
inline float polygon_perimeter(const std::vector<float>& x, const std::vector<float>& y) {
    if (x.size() < 2 || y.size() != x.size()) return 0.0f;
    
    float perimeter = 0.0f;
    size_t n = x.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        float dx = x[j] - x[i];
        float dy = y[j] - y[i];
        perimeter += std::sqrt(dx * dx + dy * dy);
    }
    
    return perimeter;
}

/**
 * @brief 计算多边形中心点
 */
inline void polygon_center(const std::vector<float>& x, const std::vector<float>& y,
                          float& center_x, float& center_y) {
    if (x.empty() || y.empty() || x.size() != y.size()) {
        center_x = 0.0f;
        center_y = 0.0f;
        return;
    }
    
    float sum_x = 0.0f, sum_y = 0.0f;
    for (size_t i = 0; i < x.size(); ++i) {
        sum_x += x[i];
        sum_y += y[i];
    }
    
    center_x = sum_x / x.size();
    center_y = sum_y / y.size();
}

/**
 * @brief 计算圆度（面积与周长的关系）
 */
inline float circularity(float area, float perimeter) {
    if (perimeter < 1e-6f) return 0.0f;
    return 4.0f * static_cast<float>(M_PI) * area / (perimeter * perimeter);
}

/**
 * @brief 计算矩形度
 */
inline float rectangularity(float area, float bounding_area) {
    if (bounding_area < 1e-6f) return 0.0f;
    return area / bounding_area;
}

/**
 * @brief 椭圆拟合（最小二乘法）
 */
inline Ellipse2D fit_ellipse(const std::vector<float>& x, const std::vector<float>& y) {
    Ellipse2D ellipse;
    
    if (x.size() < 5 || y.size() != x.size()) {
        return ellipse;
    }
    
    size_t n = x.size();
    
    // 使用直接最小二乘法拟合椭圆
    // 基于代数距离最小化
    
    // 计算均值
    double mean_x = 0.0, mean_y = 0.0;
    for (size_t i = 0; i < n; ++i) {
        mean_x += x[i];
        mean_y += y[i];
    }
    mean_x /= n;
    mean_y /= n;
    
    // 中心化坐标
    std::vector<double> xc(n), yc(n);
    double max_dist = 0.0;
    for (size_t i = 0; i < n; ++i) {
        xc[i] = x[i] - mean_x;
        yc[i] = y[i] - mean_y;
        double dist = std::sqrt(xc[i] * xc[i] + yc[i] * yc[i]);
        if (dist > max_dist) max_dist = dist;
    }
    
    // 归一化
    if (max_dist > 1e-6) {
        for (size_t i = 0; i < n; ++i) {
            xc[i] /= max_dist;
            yc[i] /= max_dist;
        }
    }
    
    // 构建设计矩阵
    // 最小化 sum(D(x,y)^2) 其中 D = Ax^2 + Bxy + Cy^2 + Dx + Ey + F
    
    double S1 = 0, S2 = 0, S3 = 0, S4 = 0, S5 = 0;
    double S6 = 0, S7 = 0, S8 = 0, S9 = 0;
    
    for (size_t i = 0; i < n; ++i) {
        double xi = xc[i], yi = yc[i];
        double xi2 = xi * xi, yi2 = yi * yi;
        double xiyi = xi * yi;
        
        S1 += xi2 * xi2;
        S2 += xi2 * xiyi;
        S3 += xi2 * yi2;
        S4 += xiyi * xiyi;
        S5 += xiyi * yi2;
        S6 += yi2 * yi2;
        S7 += xi2;
        S8 += xiyi;
        S9 += yi2;
    }
    
    // 简化拟合：使用协方差矩阵的特征值分解
    double cov_xx = S7 / n;
    double cov_yy = S9 / n;
    double cov_xy = S8 / n;
    
    // 计算主轴方向
    double angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    
    // 计算长短轴
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double sqrt_term = std::sqrt(std::max(0.0, trace * trace / 4.0 - det));
    
    double lambda1 = trace / 2.0 + sqrt_term;
    double lambda2 = trace / 2.0 - sqrt_term;
    
    // 转换回原始尺度
    double major = std::sqrt(std::abs(lambda1)) * max_dist * 2.0;
    double minor = std::sqrt(std::abs(lambda2)) * max_dist * 2.0;
    
    ellipse.center_x = static_cast<float>(mean_x);
    ellipse.center_y = static_cast<float>(mean_y);
    ellipse.major_axis = static_cast<float>(std::max(major, minor));
    ellipse.minor_axis = static_cast<float>(std::min(major, minor));
    ellipse.angle = static_cast<float>(angle);
    ellipse.valid = true;
    
    return ellipse;
}

/**
 * @brief 计算点到椭圆的距离
 */
inline float point_to_ellipse_distance(float px, float py,
                                       float cx, float cy,
                                       float a, float b, float angle) {
    // 转换到椭圆坐标系
    float dx = px - cx;
    float dy = py - cy;
    
    float cos_a = std::cos(-angle);
    float sin_a = std::sin(-angle);
    
    float xr = dx * cos_a - dy * sin_a;
    float yr = dx * sin_a + dy * cos_a;
    
    // 归一化坐标
    float xn = xr / a;
    float yn = yr / b;
    
    // 计算椭圆上的最近点
    float dist_to_center = std::sqrt(xn * xn + yn * yn);
    
    if (dist_to_center < 1e-6f) {
        return b; // 在中心
    }
    
    // 归一化距离
    float t = 1.0f / dist_to_center;
    float nearest_x = xn * t;
    float nearest_y = yn * t;
    
    // 实际距离
    float dist_on_ellipse = std::sqrt((xr - nearest_x * a) * (xr - nearest_x * a) +
                                      (yr - nearest_y * b) * (yr - nearest_y * b));
    
    return dist_on_ellipse;
}

/**
 * @brief 计算最小外接矩形
 */
inline Rectangle2D min_bounding_rect(const std::vector<float>& x, const std::vector<float>& y) {
    Rectangle2D rect;
    
    if (x.empty() || y.empty() || x.size() != y.size()) {
        return rect;
    }
    
    // 找到轴对齐边界框
    float min_x = x[0], max_x = x[0];
    float min_y = y[0], max_y = y[0];
    
    for (size_t i = 1; i < x.size(); ++i) {
        min_x = std::min(min_x, x[i]);
        max_x = std::max(max_x, x[i]);
        min_y = std::min(min_y, y[i]);
        max_y = std::max(max_y, y[i]);
    }
    
    rect.x = min_x;
    rect.y = min_y;
    rect.width = max_x - min_x;
    rect.height = max_y - min_y;
    rect.angle = 0.0f;
    
    return rect;
}

/**
 * @brief 计算最小外接圆
 */
inline void min_bounding_circle(const std::vector<float>& x, const std::vector<float>& y,
                                float& center_x, float& center_y, float& radius) {
    if (x.empty() || y.empty() || x.size() != y.size()) {
        center_x = center_y = radius = 0.0f;
        return;
    }
    
    // 简单实现：使用质心和最大距离
    float sum_x = 0.0f, sum_y = 0.0f;
    for (size_t i = 0; i < x.size(); ++i) {
        sum_x += x[i];
        sum_y += y[i];
    }
    center_x = sum_x / x.size();
    center_y = sum_y / y.size();
    
    radius = 0.0f;
    for (size_t i = 0; i < x.size(); ++i) {
        float dx = x[i] - center_x;
        float dy = y[i] - center_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        radius = std::max(radius, dist);
    }
}

} // namespace geometry_measure_utils

// ==================== 基本测量节点 ====================

/**
 * @brief 面积测量节点
 */
class AreaMeasureNode : public INode {
public:
    AreaMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 周长测量节点
 */
class PerimeterMeasureNode : public INode {
public:
    PerimeterMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 中心点测量节点
 */
class CenterMeasureNode : public INode {
public:
    CenterMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

// ==================== 形状测量节点 ====================

/**
 * @brief 圆测量节点（圆度、直径）
 */
class CircleMeasureNode : public INode {
public:
    CircleMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 直线测量节点（长度、角度）
 */
class LineMeasureNode : public INode {
public:
    LineMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 矩形测量节点（长宽、面积）
 */
class RectangleMeasureNode : public INode {
public:
    RectangleMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 椭圆测量节点
 */
class EllipseMeasureNode : public INode {
public:
    EllipseMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 多边形测量节点
 */
class PolygonMeasureNode : public INode {
public:
    PolygonMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

// ==================== 拟合测量节点 ====================

/**
 * @brief 圆拟合测量节点
 */
class CircleFitMeasureNode : public INode {
public:
    CircleFitMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 直线拟合测量节点
 */
class LineFitMeasureNode : public INode {
public:
    LineFitMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 椭圆拟合测量节点
 */
class EllipseFitMeasureNode : public INode {
public:
    EllipseFitMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 轮廓拟合测量节点
 */
class ContourFitMeasureNode : public INode {
public:
    ContourFitMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 点云拟合测量节点
 */
class PointCloudFitMeasureNode : public INode {
public:
    PointCloudFitMeasureNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf