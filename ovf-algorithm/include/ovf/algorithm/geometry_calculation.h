/**
 * @file geometry_calculation.h
 * @brief 几何计算工具库 - 参考VisionPro的CogDistance/CogAngle/CogIntersect工具系列
 *        包含距离计算、角度计算、交点计算、几何创建等25个节点
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 几何计算工具函数
 */
namespace geometry_calc_utils {

/**
 * @brief 计算两点距离
 */
inline float distance_point_point(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 计算点到直线距离（直线方程: ax + by + c = 0）
 */
inline float distance_point_line(float px, float py, float a, float b, float c) {
    return std::abs(a * px + b * py + c) / std::sqrt(a * a + b * b);
}

/**
 * @brief 计算点到圆距离（返回点到圆边界的最短距离）
 * @return 正值表示点在圆外，负值表示点在圆内
 */
inline float distance_point_circle(float px, float py, float cx, float cy, float radius) {
    float dist_to_center = distance_point_point(px, py, cx, cy);
    return dist_to_center - radius;
}

/**
 * @brief 计算点到椭圆距离（近似）
 */
inline float distance_point_ellipse(float px, float py,
                                    float cx, float cy,
                                    float a, float b, float angle_rad) {
    // 转换到椭圆坐标系
    float dx = px - cx;
    float dy = py - cy;

    float cos_a = std::cos(-angle_rad);
    float sin_a = std::sin(-angle_rad);

    float xr = dx * cos_a - dy * sin_a;
    float yr = dx * sin_a + dy * cos_a;

    // 归一化坐标
    float xn = xr / a;
    float yn = yr / b;

    float dist_to_center = std::sqrt(xn * xn + yn * yn);

    if (dist_to_center < 1e-6f) {
        return b; // 在中心
    }

    // 归一化距离
    float t = 1.0f / dist_to_center;
    float nearest_xn = xn * t;
    float nearest_yn = yn * t;

    // 实际距离
    float nearest_x = nearest_xn * a;
    float nearest_y = nearest_yn * b;

    float dist = std::sqrt((xr - nearest_x) * (xr - nearest_x) +
                           (yr - nearest_y) * (yr - nearest_y));

    // 判断点在椭圆内还是外
    if (dist_to_center > 1.0f) {
        return dist; // 在椭圆外
    } else {
        return -dist; // 在椭圆内
    }
}

/**
 * @brief 计算点到线段距离
 */
inline float distance_point_segment(float px, float py,
                                    float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float len_sq = dx * dx + dy * dy;

    if (len_sq < 1e-10f) {
        // 线段长度为0
        return distance_point_point(px, py, x1, y1);
    }

    // 投影参数 t
    float t = ((px - x1) * dx + (py - y1) * dy) / len_sq;

    // 限制 t 在 [0, 1] 范围内
    t = std::max(0.0f, std::min(1.0f, t));

    // 最近点
    float nearest_x = x1 + t * dx;
    float nearest_y = y1 + t * dy;

    return distance_point_point(px, py, nearest_x, nearest_y);
}

/**
 * @brief 计算直线到圆距离（圆心到直线距离减半径）
 */
inline float distance_line_circle(float a, float b, float c,
                                  float cx, float cy, float radius) {
    float dist_to_center = distance_point_line(cx, cy, a, b, c);
    return std::abs(dist_to_center) - radius;
}

/**
 * @brief 计算直线到椭圆距离（近似，使用中心点）
 */
inline float distance_line_ellipse(float a, float b, float c,
                                   float cx, float cy,
                                   float ea, float eb, float angle_rad) {
    // 简化：计算中心到直线距离减去椭圆最大半径
    float dist_center = distance_point_line(cx, cy, a, b, c);
    float max_radius = std::max(ea, eb);
    return std::abs(dist_center) - max_radius;
}

/**
 * @brief 计算两条直线距离（平行线距离，不平行返回0）
 */
inline float distance_line_line(float a1, float b1, float c1,
                                float a2, float b2, float c2) {
    // 归一化法向量
    float len1 = std::sqrt(a1 * a1 + b1 * b1);
    float len2 = std::sqrt(a2 * a2 + b2 * b2);

    if (len1 < 1e-10f || len2 < 1e-10f) return 0.0f;

    float n1x = a1 / len1, n1y = b1 / len1;
    float n2x = a2 / len2, n2y = b2 / len2;

    // 检查是否平行（法向量平行）
    float dot = std::abs(n1x * n2x + n1y * n2y);

    if (dot > 0.999f) {
        // 平行线，计算距离
        float d1 = std::abs(c1) / len1;
        float d2 = std::abs(c2) / len2;
        return std::abs(d1 - d2);
    }

    return 0.0f; // 不平行
}

/**
 * @brief 计算线段到圆距离
 */
inline float distance_segment_circle(float x1, float y1, float x2, float y2,
                                      float cx, float cy, float radius) {
    // 线段到圆心的距离
    float dist = distance_point_segment(cx, cy, x1, y1, x2, y2);
    return dist - radius;
}

/**
 * @brief 计算两条线段距离
 */
inline float distance_segment_segment(float x1, float y1, float x2, float y2,
                                       float x3, float y3, float x4, float y4) {
    // 简化实现：计算四个端点到另一线段的距离的最小值
    float d1 = distance_point_segment(x1, y1, x3, y3, x4, y4);
    float d2 = distance_point_segment(x2, y2, x3, y3, x4, y4);
    float d3 = distance_point_segment(x3, y3, x1, y1, x2, y2);
    float d4 = distance_point_segment(x4, y4, x1, y1, x2, y2);

    return std::min({d1, d2, d3, d4});
}

/**
 * @brief 计算两直线夹角（弧度）
 */
inline float angle_line_line(float a1, float b1, float a2, float b2) {
    // 计算方向向量
    float len1 = std::sqrt(a1 * a1 + b1 * b1);
    float len2 = std::sqrt(a2 * a2 + b2 * b2);

    if (len1 < 1e-10f || len2 < 1e-10f) return 0.0f;

    // 法向量夹角（转换为方向向量）
    float dot = (a1 * a2 + b1 * b2) / (len1 * len2);
    float cross = (a1 * b2 - b1 * a2) / (len1 * len2);

    float angle = std::atan2(std::abs(cross), std::abs(dot));

    // 返回锐角（0-90度）
    if (angle > static_cast<float>(M_PI) / 2.0f) {
        angle = static_cast<float>(M_PI) - angle;
    }

    return angle;
}

/**
 * @brief 计算两点连线角度（相对于X轴）
 */
inline float angle_point_point(float x1, float y1, float x2, float y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

/**
 * @brief 计算三点角度（在点2处的角度）
 */
inline float angle_three_points(float x1, float y1, float x2, float y2,
                                float x3, float y3) {
    float angle1 = angle_point_point(x2, y2, x1, y1);
    float angle2 = angle_point_point(x2, y2, x3, y3);

    float diff = angle2 - angle1;

    // 归一化到 [0, PI]
    while (diff < 0) diff += static_cast<float>(2.0f * M_PI);
    while (diff > static_cast<float>(M_PI)) diff = static_cast<float>(2.0f * M_PI) - diff;

    return diff;
}

/**
 * @brief 计算两向量夹角
 */
inline float angle_vector_vector(float vx1, float vy1, float vx2, float vy2) {
    float len1 = std::sqrt(vx1 * vx1 + vy1 * vy1);
    float len2 = std::sqrt(vx2 * vx2 + vy2 * vy2);

    if (len1 < 1e-10f || len2 < 1e-10f) return 0.0f;

    float dot = (vx1 * vx2 + vy1 * vy2) / (len1 * len2);
    float cross = (vx1 * vy2 - vy1 * vx2) / (len1 * len2);

    return std::atan2(std::abs(cross), std::abs(dot));
}

/**
 * @brief 计算两直线交点
 */
inline bool intersect_line_line(float a1, float b1, float c1,
                                float a2, float b2, float c2,
                                float& x, float& y) {
    float det = a1 * b2 - a2 * b1;

    if (std::abs(det) < 1e-10f) {
        return false; // 平行线
    }

    x = (b1 * c2 - b2 * c1) / det;
    y = (a2 * c1 - a1 * c2) / det;

    return true;
}

/**
 * @brief 计算直线与圆交点
 * @return 交点数量（0, 1, 或 2）
 */
inline int intersect_line_circle(float a, float b, float c,
                                 float cx, float cy, float radius,
                                 float& x1, float& y1, float& x2, float& y2) {
    // 归一化直线方程
    float len = std::sqrt(a * a + b * b);
    if (len < 1e-10f) return 0;

    float na = a / len;
    float nb = b / len;
    float nc = c / len;

    // 圆心到直线距离
    float dist = na * cx + nb * cy + nc;

    if (std::abs(dist) > radius) {
        return 0; // 无交点
    }

    // 投影点
    float px = cx - dist * na;
    float py = cy - dist * nb;

    float offset = std::sqrt(radius * radius - dist * dist);

    // 沿直线方向
    float dx = -nb;
    float dy = na;

    x1 = px + offset * dx;
    y1 = py + offset * dy;
    x2 = px - offset * dx;
    y2 = py - offset * dy;

    if (std::abs(dist) == radius) {
        return 1; // 一个交点
    }

    return 2; // 两个交点
}

/**
 * @brief 计算直线与椭圆交点（近似）
 */
inline int intersect_line_ellipse(float a, float b, float c,
                                  float cx, float cy,
                                  float ea, float eb, float angle_rad,
                                  float& x1, float& y1, float& x2, float& y2) {
    // 转换到椭圆坐标系
    // 旋转椭圆到标准位置
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);

    // 旋转直线方程
    // ax + by + c = 0 在旋转坐标系中变为 a'(x'+cx) + b'(y'+cy) + c = 0
    float na = a * cos_a + b * sin_a;
    float nb = -a * sin_a + b * cos_a;
    float nc = a * cx + b * cy + c;

    // 归一化
    float len = std::sqrt(na * na + nb * nb);
    if (len < 1e-10f) return 0;

    na /= len;
    nb /= len;
    nc /= len;

    // 圆心到直线距离
    float dist = std::abs(nc);

    if (dist > std::max(ea, eb)) {
        return 0; // 近似判断无交点
    }

    // 简化计算：使用数值方法
    // 求解椭圆方程 x'^2/ea^2 + y'^2/eb^2 = 1 与直线 na*x' + nb*y' + nc = 0 的交点

    float offset = std::sqrt(std::max(0.0f, ea * ea - nc * nc));

    float x1r = offset;
    float y1r = -nc / nb; // 简化
    float x2r = -offset;
    float y2r = -nc / nb;

    // 旋转回原坐标系
    x1 = x1r * cos_a - y1r * sin_a + cx;
    y1 = x1r * sin_a + y1r * cos_a + cy;
    x2 = x2r * cos_a - y2r * sin_a + cx;
    y2 = x2r * sin_a + y2r * cos_a + cy;

    return 2;
}

/**
 * @brief 计算两圆交点
 */
inline int intersect_circle_circle(float c1x, float c1y, float r1,
                                   float c2x, float c2y, float r2,
                                   float& x1, float& y1, float& x2, float& y2) {
    float dx = c2x - c1x;
    float dy = c2y - c1y;
    float dist = std::sqrt(dx * dx + dy * dy);

    // 检查是否有交点
    if (dist > r1 + r2) return 0; // 相离
    if (dist < std::abs(r1 - r2)) return 0; // 内含
    if (dist < 1e-10f && std::abs(r1 - r2) < 1e-10f) return 0; // 同心圆

    // 计算交点
    float a = (r1 * r1 - r2 * r2 + dist * dist) / (2.0f * dist);
    float h = std::sqrt(r1 * r1 - a * a);

    float px = c1x + a * dx / dist;
    float py = c1y + a * dy / dist;

    float perp_x = -dy / dist;
    float perp_y = dx / dist;

    x1 = px + h * perp_x;
    y1 = py + h * perp_y;
    x2 = px - h * perp_x;
    y2 = py - h * perp_y;

    if (h < 1e-6f) {
        return 1; // 一个交点
    }

    return 2;
}

/**
 * @brief 计算线段与直线交点
 */
inline bool intersect_segment_line(float x1, float y1, float x2, float y2,
                                   float a, float b, float c,
                                   float& x, float& y) {
    // 线段所在直线方程
    float la = y1 - y2;
    float lb = x2 - x1;
    float lc = x1 * y2 - x2 * y1;

    float det = la * b - lb * a;

    if (std::abs(det) < 1e-10f) {
        return false; // 平行
    }

    float ix = (lb * c - b * lc) / det;
    float iy = (la * c - a * lc) / det;

    // 检查交点是否在线段上
    float t = ((ix - x1) * (x2 - x1) + (iy - y1) * (y2 - y1)) /
              ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

    if (t >= 0.0f && t <= 1.0f) {
        x = ix;
        y = iy;
        return true;
    }

    return false;
}

/**
 * @brief 计算线段与圆交点
 */
inline int intersect_segment_circle(float x1, float y1, float x2, float y2,
                                    float cx, float cy, float radius,
                                    float& x_out1, float& y_out1,
                                    float& x_out2, float& y_out2) {
    float tx1, ty1, tx2, ty2;

    // 线段所在直线方程
    float a = y1 - y2;
    float b = x2 - x1;
    float c = x1 * y2 - x2 * y1;

    int count = intersect_line_circle(a, b, c, cx, cy, radius, tx1, ty1, tx2, ty2);

    if (count == 0) return 0;

    int out_count = 0;

    // 检查交点是否在线段上
    float len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    auto check_point = [&](float px, float py) {
        float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / len_sq;
        if (t >= 0.0f && t <= 1.0f) {
            if (out_count == 0) {
                x_out1 = px;
                y_out1 = py;
            } else {
                x_out2 = px;
                y_out2 = py;
            }
            out_count++;
        }
    };

    check_point(tx1, ty1);
    if (count == 2) {
        check_point(tx2, ty2);
    }

    return out_count;
}

/**
 * @brief 计算两线段交点
 */
inline bool intersect_segment_segment(float x1, float y1, float x2, float y2,
                                      float x3, float y3, float x4, float y4,
                                      float& x, float& y) {
    float d1x = x2 - x1, d1y = y2 - y1;
    float d2x = x4 - x3, d2y = y4 - y3;

    float cross = d1x * d2y - d1y * d2x;

    if (std::abs(cross) < 1e-10f) {
        return false; // 平行
    }

    float dx = x3 - x1;
    float dy = y3 - y1;

    float t1 = (dx * d2y - dy * d2x) / cross;
    float t2 = (dx * d1y - dy * d1x) / cross;

    if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f) {
        x = x1 + t1 * d1x;
        y = y1 + t1 * d1y;
        return true;
    }

    return false;
}

/**
 * @brief 计算线段与椭圆交点（近似）
 */
inline int intersect_segment_ellipse(float x1, float y1, float x2, float y2,
                                     float cx, float cy,
                                     float ea, float eb, float angle_rad,
                                     float& x_out1, float& y_out1,
                                     float& x_out2, float& y_out2) {
    // 线段所在直线方程
    float a = y1 - y2;
    float b = x2 - x1;
    float c = x1 * y2 - x2 * y1;

    float tx1, ty1, tx2, ty2;
    int count = intersect_line_ellipse(a, b, c, cx, cy, ea, eb, angle_rad,
                                       tx1, ty1, tx2, ty2);

    if (count == 0) return 0;

    int out_count = 0;
    float len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    auto check_point = [&](float px, float py) {
        float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / len_sq;
        if (t >= 0.0f && t <= 1.0f) {
            if (out_count == 0) {
                x_out1 = px;
                y_out1 = py;
            } else {
                x_out2 = px;
                y_out2 = py;
            }
            out_count++;
        }
    };

    check_point(tx1, ty1);
    if (count == 2) {
        check_point(tx2, ty2);
    }

    return out_count;
}

/**
 * @brief 创建平行线
 * @param distance 距离（正值在左侧，负值在右侧）
 */
inline void create_line_parallel(float a, float b, float c, float distance,
                                 float& pa, float& pb, float& pc) {
    float len = std::sqrt(a * a + b * b);
    if (len < 1e-10f) {
        pa = a;
        pb = b;
        pc = c;
        return;
    }

    pa = a;
    pb = b;
    pc = c + distance * len;
}

/**
 * @brief 创建过点的垂线
 */
inline void create_line_perpendicular(float a, float b, float c,
                                       float px, float py,
                                       float& pa, float& pb, float& pc) {
    // 原直线的方向向量 (-b, a)
    // 垂线的方向向量 (a, b)，即垂线的法向量

    pa = -b;
    pb = a;
    pc = b * px - a * py;
}

/**
 * @brief 创建两点的平分线（过两点的垂直平分线）
 */
inline void create_line_bisect_points(float x1, float y1, float x2, float y2,
                                      float& a, float& b, float& c) {
    // 中点
    float mx = (x1 + x2) / 2.0f;
    float my = (y1 + y2) / 2.0f;

    // 连线方向
    float dx = x2 - x1;
    float dy = y2 - y1;

    // 垂直方向
    a = dx;
    b = dy;
    c = -a * mx - b * my;
}

} // namespace geometry_calc_utils

// ============================================================================
// 距离计算节点（10个）
// ============================================================================

/**
 * @brief 点到点距离节点
 */
class DistancePointPointNode : public INode {
public:
    DistancePointPointNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 点到直线距离节点
 */
class DistancePointLineNode : public INode {
public:
    DistancePointLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 点到圆距离节点
 */
class DistancePointCircleNode : public INode {
public:
    DistancePointCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 点到椭圆距离节点
 */
class DistancePointEllipseNode : public INode {
public:
    DistancePointEllipseNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 点到线段距离节点
 */
class DistancePointSegmentNode : public INode {
public:
    DistancePointSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线到圆距离节点
 */
class DistanceLineCircleNode : public INode {
public:
    DistanceLineCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线到椭圆距离节点
 */
class DistanceLineEllipseNode : public INode {
public:
    DistanceLineEllipseNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线到线距离节点
 */
class DistanceLineLineNode : public INode {
public:
    DistanceLineLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线段到圆距离节点
 */
class DistanceSegmentCircleNode : public INode {
public:
    DistanceSegmentCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线段到线段距离节点
 */
class DistanceSegmentSegmentNode : public INode {
public:
    DistanceSegmentSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ============================================================================
// 角度计算节点（4个）
// ============================================================================

/**
 * @brief 两直线夹角节点
 */
class AngleLineLineNode : public INode {
public:
    AngleLineLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 两点连线角度节点
 */
class AnglePointPointNode : public INode {
public:
    AnglePointPointNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 三点角度节点
 */
class AngleThreePointsNode : public INode {
public:
    AngleThreePointsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 两向量夹角节点
 */
class AngleVectorVectorNode : public INode {
public:
    AngleVectorVectorNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ============================================================================
// 交点计算节点（8个）
// ============================================================================

/**
 * @brief 线线交点节点
 */
class IntersectLineLineNode : public INode {
public:
    IntersectLineLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线圆交点节点
 */
class IntersectLineCircleNode : public INode {
public:
    IntersectLineCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线椭圆交点节点
 */
class IntersectLineEllipseNode : public INode {
public:
    IntersectLineEllipseNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 圆圆交点节点
 */
class IntersectCircleCircleNode : public INode {
public:
    IntersectCircleCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线段线交点节点
 */
class IntersectSegmentLineNode : public INode {
public:
    IntersectSegmentLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线段圆交点节点
 */
class IntersectSegmentCircleNode : public INode {
public:
    IntersectSegmentCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线段线段交点节点
 */
class IntersectSegmentSegmentNode : public INode {
public:
    IntersectSegmentSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 线段椭圆交点节点
 */
class IntersectSegmentEllipseNode : public INode {
public:
    IntersectSegmentEllipseNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ============================================================================
// 几何创建节点（3个）
// ============================================================================

/**
 * @brief 创建平行线节点
 */
class CreateLineParallelNode : public INode {
public:
    CreateLineParallelNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 创建垂线节点
 */
class CreateLinePerpendicularNode : public INode {
public:
    CreateLinePerpendicularNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 创建两点平分线节点
 */
class CreateLineBisectPointsNode : public INode {
public:
    CreateLineBisectPointsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf