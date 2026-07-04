/**
 * @file xld_processing.h
 * @brief XLD轮廓处理和亚像素边缘检测算子（Halcon风格，纯C++实现）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/measurement.h"

namespace ovf {
namespace algorithm {

/**
 * @brief XLD轮廓点（亚像素精度）
 */
struct XLDPoint {
    float x = 0.0f;
    float y = 0.0f;

    XLDPoint() = default;
    XLDPoint(float x_, float y_) : x(x_), y(y_) {}

    // 转换为Point2Df
    operator Point2Df() const {
        return Point2Df(x, y);
    }

    float distance_to(const XLDPoint& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    float length() const {
        return std::sqrt(x * x + y * y);
    }

    XLDPoint operator+(const XLDPoint& other) const {
        return XLDPoint(x + other.x, y + other.y);
    }

    XLDPoint operator-(const XLDPoint& other) const {
        return XLDPoint(x - other.x, y - other.y);
    }

    XLDPoint operator*(float s) const {
        return XLDPoint(x * s, y * s);
    }

    // 点积
    float dot(const XLDPoint& other) const {
        return x * other.x + y * other.y;
    }

    // 叉积（返回z分量）
    float cross(const XLDPoint& other) const {
        return x * other.y - y * other.x;
    }

    // 归一化
    XLDPoint normalized() const {
        float len = length();
        if (len > 1e-10f) {
            return XLDPoint(x / len, y / len);
        }
        return *this;
    }
};

/**
 * @brief XLD轮廓结构
 */
struct XLDContour {
    std::vector<XLDPoint> points;     // 轮廓点
    bool is_closed = false;           // 是否闭合
    float length = 0.0f;              // 轮廓长度
    float area = 0.0f;                // 面积（闭合轮廓）
    XLDPoint centroid;                // 质心
    float angle = 0.0f;               // 主方向角（弧度）
    float circularity = 0.0f;         // 圆度
    float compactness = 0.0f;         // 紧凑度

    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }

    void clear() {
        points.clear();
        is_closed = false;
        length = 0.0f;
        area = 0.0f;
        angle = 0.0f;
        circularity = 0.0f;
        compactness = 0.0f;
    }

    // 计算轮廓属性
    void compute_properties() {
        if (points.empty()) return;

        // 计算长度
        length = 0.0f;
        for (size_t i = 1; i < points.size(); ++i) {
            length += points[i].distance_to(points[i - 1]);
        }
        if (is_closed && points.size() > 2) {
            length += points.back().distance_to(points.front());
        }

        // 计算质心和面积
        centroid = XLDPoint(0, 0);
        area = 0.0f;

        if (is_closed && points.size() > 2) {
            // 使用鞋带公式计算面积和质心
            float cx = 0.0f, cy = 0.0f;
            for (size_t i = 0; i < points.size(); ++i) {
                size_t j = (i + 1) % points.size();
                float cross = points[i].cross(points[j]);
                area += cross;
                cx += (points[i].x + points[j].x) * cross;
                cy += (points[i].y + points[j].y) * cross;
            }
            area = std::abs(area) * 0.5f;
            if (area > 1e-10f) {
                centroid.x = cx / (6.0f * area);
                centroid.y = cy / (6.0f * area);
            }
        } else {
            // 开放轮廓质心
            for (const auto& pt : points) {
                centroid.x += pt.x;
                centroid.y += pt.y;
            }
            centroid.x /= static_cast<float>(points.size());
            centroid.y /= static_cast<float>(points.size());
        }

        // 计算圆度
        if (area > 1e-10f && length > 1e-10f) {
            circularity = 4.0f * static_cast<float>(M_PI) * area / (length * length);
        }

        // 计算主方向
        if (points.size() > 1) {
            float sum_xx = 0, sum_yy = 0, sum_xy = 0;
            for (const auto& pt : points) {
                float dx = pt.x - centroid.x;
                float dy = pt.y - centroid.y;
                sum_xx += dx * dx;
                sum_yy += dy * dy;
                sum_xy += dx * dy;
            }

            // 使用图像矩计算主轴方向
            float trace = sum_xx + sum_yy;
            float det = sum_xx * sum_yy - sum_xy * sum_xy;
            float diff = trace * trace - 4 * det;
            if (diff >= 0) {
                float lambda = (trace + std::sqrt(diff)) * 0.5f;
                if (std::abs(sum_xy) > 1e-10f) {
                    angle = std::atan2(lambda - sum_yy, sum_xy);
                } else if (sum_xx > sum_yy) {
                    angle = 0.0f;
                } else {
                    angle = static_cast<float>(M_PI) * 0.5f;
                }
            }
        }
    }
};

/**
 * @brief 亚像素边缘点
 */
struct SubPixelEdge {
    XLDPoint position;       // 边缘位置
    XLDPoint direction;      // 边缘方向（单位向量）
    float amplitude = 0.0f;  // 边缘幅值（梯度强度）
    float angle = 0.0f;       // 边缘角度（弧度）
    int polarity = 0;        // 极性：1=亮到暗，-1=暗到亮

    SubPixelEdge() = default;
    SubPixelEdge(const XLDPoint& pos, const XLDPoint& dir, float amp, float ang, int pol)
        : position(pos), direction(dir), amplitude(amp), angle(ang), polarity(pol) {}
};

/**
 * @brief 边缘对测量结果
 */
struct EdgePair {
    XLDPoint first_edge;      // 第一条边缘位置
    XLDPoint second_edge;     // 第二条边缘位置
    float distance = 0.0f;    // 两边缘间距
    float center_x = 0.0f;    // 边缘对中心X
    float center_y = 0.0f;    // 边缘对中心Y
    int polarity = 0;         // 极性模式
    bool valid = false;

    void compute_center() {
        center_x = (first_edge.x + second_edge.x) * 0.5f;
        center_y = (first_edge.y + second_edge.y) * 0.5f;
        distance = first_edge.distance_to(second_edge);
    }
};

/**
 * @brief XLD处理工具函数
 */
namespace xld_utils {

/**
 * @brief 计算轮廓链码方向
 */
inline float chain_direction(const XLDPoint& p1, const XLDPoint& p2) {
    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

/**
 * @brief 计算轮廓曲率
 */
inline float curvature(const XLDPoint& p0, const XLDPoint& p1, const XLDPoint& p2) {
    XLDPoint v1 = p1 - p0;
    XLDPoint v2 = p2 - p1;

    float cross = v1.cross(v2);
    float dot = v1.dot(v2);
    float len1 = v1.length();
    float len2 = v2.length();

    if (len1 < 1e-10f || len2 < 1e-10f) return 0.0f;

    return cross / (len1 * len2);
}

/**
 * @brief 高斯平滑滤波
 */
inline void gaussian_smooth(std::vector<float>& data, float sigma) {
    if (data.empty() || sigma < 0.1f) return;

    int radius = static_cast<int>(3.0f * sigma);
    if (radius < 1) radius = 1;

    std::vector<float> kernel(2 * radius + 1);
    float sum = 0.0f;

    for (int i = -radius; i <= radius; ++i) {
        float val = std::exp(-static_cast<float>(i * i) / (2.0f * sigma * sigma));
        kernel[i + radius] = val;
        sum += val;
    }

    // 归一化
    for (auto& k : kernel) k /= sum;

    // 卷积
    std::vector<float> result(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        float val = 0.0f;
        for (int j = -radius; j <= radius; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(data.size())) {
                val += data[idx] * kernel[j + radius];
            }
        }
        result[i] = val;
    }
    data = std::move(result);
}

/**
 * @brief 双线性插值
 */
inline float bilinear_interpolate(const ImageData& img, float x, float y) {
    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x0 < 0 || x1 >= static_cast<int>(img.width) ||
        y0 < 0 || y1 >= static_cast<int>(img.height)) {
        return 0.0f;
    }

    float dx = x - x0;
    float dy = y - y0;

    float v00 = static_cast<float>(img.data[y0 * img.width + x0]);
    float v01 = static_cast<float>(img.data[y0 * img.width + x1]);
    float v10 = static_cast<float>(img.data[y1 * img.width + x0]);
    float v11 = static_cast<float>(img.data[y1 * img.width + x1]);

    return v00 * (1 - dx) * (1 - dy) +
           v01 * dx * (1 - dy) +
           v10 * (1 - dx) * dy +
           v11 * dx * dy;
}

/**
 * @brief 计算图像梯度（带亚像素精度）
 */
inline void compute_gradient(const ImageData& img, float x, float y,
                            float& gx, float& gy, float& magnitude) {
    // 使用Sobel算子计算梯度
    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);

    gx = 0.0f;
    gy = 0.0f;

    if (x0 < 1 || x0 >= static_cast<int>(img.width) - 1 ||
        y0 < 1 || y0 >= static_cast<int>(img.height) - 1) {
        magnitude = 0.0f;
        return;
    }

    // Sobel算子
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            float val = static_cast<float>(img.data[(y0 + dy) * img.width + (x0 + dx)]);
            gx += val * dx * (dy == 0 ? 2 : 1);
            gy += val * dy * (dx == 0 ? 2 : 1);
        }
    }

    magnitude = std::sqrt(gx * gx + gy * gy);
}

/**
 * @brief 亚像素边缘定位（梯度方向极值插值）
 */
inline bool subpixel_edge_position(const ImageData& img, float x, float y,
                                   XLDPoint& edge_pos, float& amplitude,
                                   XLDPoint& direction, int& polarity) {
    // 计算梯度
    float gx, gy, mag;
    compute_gradient(img, x, y, gx, gy, mag);

    if (mag < 1.0f) return false;

    // 梯度方向（归一化）
    float grad_len = std::sqrt(gx * gx + gy * gy);
    if (grad_len < 1e-10f) return false;

    float nx = gx / grad_len;  // 梯度方向单位向量
    float ny = gy / grad_len;

    // 沿梯度方向寻找极值点（泰勒展开）
    // 在梯度方向上采样
    float step = 0.5f;  // 采样步长
    float v_center = bilinear_interpolate(img, x, y);
    float v_plus = bilinear_interpolate(img, x + nx * step, y + ny * step);
    float v_minus = bilinear_interpolate(img, x - nx * step, y - ny * step);

    // 二次插值找极值
    float a = (v_plus + v_minus - 2 * v_center) / (step * step);
    float b = (v_plus - v_minus) / (2 * step);

    if (std::abs(a) < 1e-10f) return false;

    float offset = -b / a;  // 极值点偏移

    // 限制偏移范围
    if (std::abs(offset) > step) {
        offset = (offset > 0) ? step : -step;
    }

    // 亚像素边缘位置
    edge_pos.x = x + nx * offset;
    edge_pos.y = y + ny * offset;

    // 边缘强度和方向
    amplitude = mag;
    direction.x = -ny;  // 边缘方向垂直于梯度方向
    direction.y = nx;

    // 极性判断
    polarity = (v_minus > v_plus) ? 1 : -1;  // 1=亮到暗，-1=暗到亮

    return true;
}

/**
 * @brief Douglas-Peucker算法简化轮廓
 */
inline std::vector<XLDPoint> simplify_contour(const std::vector<XLDPoint>& points, float epsilon) {
    if (points.size() < 3) return points;

    // 找到距离最远的点
    float max_dist = 0.0f;
    size_t max_idx = 0;

    XLDPoint start = points.front();
    XLDPoint end = points.back();

    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float len = std::sqrt(dx * dx + dy * dy);

    if (len < 1e-10f) return points;

    for (size_t i = 1; i < points.size() - 1; ++i) {
        float dist = std::abs((points[i].x - start.x) * dy -
                             (points[i].y - start.y) * dx) / len;
        if (dist > max_dist) {
            max_dist = dist;
            max_idx = i;
        }
    }

    std::vector<XLDPoint> result;

    if (max_dist > epsilon) {
        // 递归简化
        auto left = simplify_contour(
            std::vector<XLDPoint>(points.begin(), points.begin() + max_idx + 1), epsilon);
        auto right = simplify_contour(
            std::vector<XLDPoint>(points.begin() + max_idx, points.end()), epsilon);

        result.insert(result.end(), left.begin(), left.end() - 1);
        result.insert(result.end(), right.begin(), right.end());
    } else {
        result.push_back(start);
        result.push_back(end);
    }

    return result;
}

/**
 * @brief 从区域生成轮廓（边界跟踪）
 */
inline std::vector<XLDContour> contours_from_region(const ImageData& binary_img) {
    std::vector<XLDContour> contours;

    if (binary_img.empty() || binary_img.channels != 1) {
        return contours;
    }

    uint32_t width = binary_img.width;
    uint32_t height = binary_img.height;

    // 创建标记数组
    std::vector<bool> visited(width * height, false);

    // 8邻域方向
    const int dx[] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    const int dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

    // 边界跟踪（摩尔邻域跟踪算法）
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;

            // 寻找起始点（前景像素且未被访问，且左边是背景）
            if (binary_img.data[idx] == 0 || visited[idx]) continue;
            if (x > 0 && binary_img.data[idx - 1] != 0) continue;

            // 开始边界跟踪
            XLDContour contour;
            contour.is_closed = true;

            int cx = static_cast<int>(x);
            int cy = static_cast<int>(y);
            int start_dir = 0;  // 起始搜索方向

            // 亚像素修正：边界位于像素边缘
            contour.points.push_back(XLDPoint(static_cast<float>(cx) - 0.5f,
                                              static_cast<float>(cy) - 0.5f));

            bool first = true;

            do {
                // 在摩尔邻域中搜索
                bool found = false;
                int search_dir = first ? 0 : (start_dir + 6) % 8;  // 回溯搜索

                for (int i = 0; i < 8; ++i) {
                    int dir = (search_dir + i) % 8;
                    int nx = cx + dx[dir];
                    int ny = cy + dy[dir];

                    if (nx < 0 || nx >= static_cast<int>(width) ||
                        ny < 0 || ny >= static_cast<int>(height)) {
                        continue;
                    }

                    size_t nidx = ny * width + nx;

                    if (binary_img.data[nidx] != 0) {
                        // 找到下一个边界点
                        cx = nx;
                        cy = ny;
                        start_dir = dir;
                        found = true;
                        visited[static_cast<size_t>(cy) * width + static_cast<size_t>(cx)] = true;

                        // 亚像素边界点
                        contour.points.push_back(
                            XLDPoint(static_cast<float>(cx) - 0.5f,
                                    static_cast<float>(cy) - 0.5f));
                        first = false;
                        break;
                    }
                }

                if (!found) break;

            } while (cx != static_cast<int>(x) || cy != static_cast<int>(y));

            if (contour.points.size() > 3) {
                contour.compute_properties();
                contours.push_back(std::move(contour));
            }
        }
    }

    return contours;
}

/**
 * @brief 从骨架生成轮廓
 */
inline std::vector<XLDContour> contours_from_skeleton(const ImageData& skeleton) {
    std::vector<XLDContour> contours;

    if (skeleton.empty() || skeleton.channels != 1) {
        return contours;
    }

    uint32_t width = skeleton.width;
    uint32_t height = skeleton.height;

    std::vector<bool> visited(width * height, false);

    // 找端点和分叉点
    for (uint32_t y = 1; y < height - 1; ++y) {
        for (uint32_t x = 1; x < width - 1; ++x) {
            size_t idx = y * width + x;
            if (skeleton.data[idx] == 0 || visited[idx]) continue;

            // 计算邻域非零像素数
            int neighbors = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (skeleton.data[(y + dy) * width + (x + dx)] != 0) {
                        neighbors++;
                    }
                }
            }

            // 端点（1个邻居）或分叉点（>2个邻居）
            bool is_endpoint = (neighbors == 1);
            bool is_branch = (neighbors > 2);

            if (is_endpoint) {
                // 从端点开始跟踪
                XLDContour contour;
                contour.is_closed = false;

                int cx = static_cast<int>(x);
                int cy = static_cast<int>(y);
                int prev_x = -1, prev_y = -1;

                while (true) {
                    contour.points.push_back(XLDPoint(static_cast<float>(cx),
                                                      static_cast<float>(cy)));
                    visited[static_cast<size_t>(cy) * width + static_cast<size_t>(cx)] = true;

                    // 找下一个点
                    bool found_next = false;
                    for (int dy = -1; dy <= 1 && !found_next; ++dy) {
                        for (int dx = -1; dx <= 1 && !found_next; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = cx + dx;
                            int ny = cy + dy;
                            if (nx < 0 || nx >= static_cast<int>(width) ||
                                ny < 0 || ny >= static_cast<int>(height)) continue;
                            if (nx == prev_x && ny == prev_y) continue;

                            size_t nidx = static_cast<size_t>(ny) * width + static_cast<size_t>(nx);
                            if (skeleton.data[nidx] != 0 && !visited[nidx]) {
                                prev_x = cx;
                                prev_y = cy;
                                cx = nx;
                                cy = ny;
                                found_next = true;
                            }
                        }
                    }

                    if (!found_next) break;
                }

                if (contour.points.size() > 1) {
                    contour.compute_properties();
                    contours.push_back(std::move(contour));
                }
            }
        }
    }

    return contours;
}

/**
 * @brief RANSAC直线拟合
 */
inline bool ransac_fit_line(const std::vector<XLDPoint>& points, Line2D& line,
                           float threshold = 2.0f, int max_iterations = 100) {
    if (points.size() < 2) return false;

    size_t best_inliers = 0;
    Line2D best_line;

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择两点
        size_t i1 = std::rand() % points.size();
        size_t i2 = std::rand() % points.size();
        if (i1 == i2) continue;

        const XLDPoint& p1 = points[i1];
        const XLDPoint& p2 = points[i2];

        // 计算直线参数
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-10f) continue;

        Line2D test_line;
        test_line.a = -dy / len;
        test_line.b = dx / len;
        test_line.c = -(test_line.a * p1.x + test_line.b * p1.y);

        // 计算内点数
        size_t inliers = 0;
        for (const auto& pt : points) {
            float dist = test_line.distance_to_point(pt);
            if (dist < threshold) {
                inliers++;
            }
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_line = test_line;
        }
    }

    if (best_inliers > points.size() / 2) {
        line = best_line;
        line.normalize();
        return true;
    }

    return false;
}

/**
 * @brief RANSAC圆拟合
 */
inline bool ransac_fit_circle(const std::vector<XLDPoint>& points, Circle2D& circle,
                              float threshold = 2.0f, int max_iterations = 100) {
    if (points.size() < 3) return false;

    size_t best_inliers = 0;
    Circle2D best_circle;

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择三点
        size_t i1 = std::rand() % points.size();
        size_t i2 = std::rand() % points.size();
        size_t i3 = std::rand() % points.size();
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        const XLDPoint& p1 = points[i1];
        const XLDPoint& p2 = points[i2];
        const XLDPoint& p3 = points[i3];

        // 三点定圆
        float x1 = p1.x, y1 = p1.y;
        float x2 = p2.x, y2 = p2.y;
        float x3 = p3.x, y3 = p3.y;

        float a = 2.0f * (x2 - x1);
        float b = 2.0f * (y2 - y1);
        float c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
        float d = 2.0f * (x3 - x2);
        float e = 2.0f * (y3 - y2);
        float f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;

        float det = a * e - b * d;
        if (std::abs(det) < 1e-10f) continue;

        float cx = (c * e - b * f) / det;
        float cy = (a * f - c * d) / det;
        float r = std::sqrt((x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy));

        if (r < 1.0f || r > 10000.0f) continue;  // 半径限制

        // 计算内点数
        size_t inliers = 0;
        for (const auto& pt : points) {
            float dx = pt.x - cx;
            float dy = pt.y - cy;
            float dist = std::abs(std::sqrt(dx * dx + dy * dy) - r);
            if (dist < threshold) {
                inliers++;
            }
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_circle.center_x = cx;
            best_circle.center_y = cy;
            best_circle.radius = r;
            best_circle.valid = true;
        }
    }

    if (best_inliers > points.size() / 3) {
        circle = best_circle;
        return true;
    }

    return false;
}

/**
 * @brief 沿投影线采样边缘点
 */
inline std::vector<SubPixelEdge> sample_edges_along_line(
    const ImageData& img,
    float start_x, float start_y,
    float end_x, float end_y,
    float threshold, int polarity = 0) {

    std::vector<SubPixelEdge> edges;

    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float len = std::sqrt(dx * dx + dy * dy);

    if (len < 1.0f) return edges;

    // 归一化方向
    float dir_x = dx / len;
    float dir_y = dy / len;

    // 采样步长
    float step = 1.0f;
    int num_samples = static_cast<int>(len / step) + 1;

    // 采样灰度值
    std::vector<float> profile(num_samples);
    std::vector<float> sample_x(num_samples);
    std::vector<float> sample_y(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / (num_samples - 1);
        float x = start_x + t * dx;
        float y = start_y + t * dy;
        sample_x[i] = x;
        sample_y[i] = y;
        profile[i] = bilinear_interpolate(img, x, y);
    }

    // 找边缘跳变
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
            // 亚像素定位
            float edge_x = (sample_x[i] + sample_x[i - 1]) * 0.5f;
            float edge_y = (sample_y[i] + sample_y[i - 1]) * 0.5f;

            SubPixelEdge edge;
            if (subpixel_edge_position(img, edge_x, edge_y,
                                       edge.position, edge.amplitude,
                                       edge.direction, edge.polarity)) {
                edge.angle = std::atan2(edge.direction.y, edge.direction.x);
                edges.push_back(edge);
            }
        }
    }

    return edges;
}

/**
 * @brief 计算轮廓包围盒
 */
inline void contour_bounding_box(const XLDContour& contour,
                                 float& min_x, float& min_y,
                                 float& max_x, float& max_y) {
    if (contour.points.empty()) {
        min_x = min_y = max_x = max_y = 0;
        return;
    }

    min_x = max_x = contour.points[0].x;
    min_y = max_y = contour.points[0].y;

    for (const auto& pt : contour.points) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }
}

} // namespace xld_utils

//==============================================================================
// XLD轮廓生成节点（6个）
//==============================================================================

/**
 * @brief 从区域生成轮廓节点
 */
class GenContourRegionNode : public INode {
public:
    GenContourRegionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 从骨架生成轮廓节点
 */
class GenContoursSkeletonNode : public INode {
public:
    GenContoursSkeletonNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 生成十字形轮廓节点
 */
class GenCrossContourNode : public INode {
public:
    GenCrossContourNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 生成多边形轮廓节点
 */
class GenPolygonContourNode : public INode {
public:
    GenPolygonContourNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 生成弧形轮廓节点
 */
class GenArcContourNode : public INode {
public:
    GenArcContourNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 生成椭圆轮廓节点
 */
class GenEllipseContourNode : public INode {
public:
    GenEllipseContourNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

//==============================================================================
// XLD轮廓处理节点（6个）
//==============================================================================

/**
 * @brief 轮廓选择节点
 */
class SelectContoursNode : public INode {
public:
    SelectContoursNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓分割节点
 */
class SegmentContoursNode : public INode {
public:
    SegmentContoursNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓近似节点（链码简化）
 */
class ApproxChainNode : public INode {
public:
    ApproxChainNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓平滑节点
 */
class SmoothContoursNode : public INode {
public:
    SmoothContoursNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓合并节点
 */
class MergeContoursNode : public INode {
public:
    MergeContoursNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓裁剪节点
 */
class ClipContoursNode : public INode {
public:
    ClipContoursNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

//==============================================================================
// 亚像素边缘检测节点（4个）
//==============================================================================

/**
 * @brief 亚像素边缘检测节点（梯度最大值定位）
 */
class EdgesSubPixNode : public INode {
public:
    EdgesSubPixNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief Sobel亚像素边缘检测节点
 */
class EdgesSubPixSobelNode : public INode {
public:
    EdgesSubPixSobelNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief Canny亚像素边缘检测节点
 */
class EdgesSubPixCannyNode : public INode {
public:
    EdgesSubPixCannyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 亚像素直线检测节点
 */
class LinesSubPixNode : public INode {
public:
    LinesSubPixNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

//==============================================================================
// 边缘对测量节点（4个）
//==============================================================================

/**
 * @brief 边缘对测量节点（卡尺找边对）
 */
class MeasurePairsNode : public INode {
public:
    MeasurePairsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 位置测量节点（单边定位）
 */
class MeasurePosNode : public INode {
public:
    MeasurePosNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 找直线节点（边缘拟合）
 */
class FindLinesNode : public INode {
public:
    FindLinesNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 找圆节点（边缘拟合）
 */
class FindCirclesNode : public INode {
public:
    FindCirclesNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf