/**
 * @file feature_detection.h
 * @brief 特征检测算子（纯C++实现，不依赖OpenCV）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 角点结构
 */
struct Corner {
    uint32_t x = 0;           // X坐标
    uint32_t y = 0;           // Y坐标
    double response = 0.0;    // 响应值
    bool valid = false;
};

/**
 * @brief 关键点结构
 */
struct KeyPoint {
    uint32_t x = 0;           // X坐标
    uint32_t y = 0;           // Y坐标
    double size = 1.0;        // 尺度
    double angle = 0.0;       // 方向
    double response = 0.0;    // 响应值
    uint32_t id = 0;          // ID
    bool valid = false;
};

/**
 * @brief 描述符结构
 */
struct Descriptor {
    std::vector<float> data;  // 描述符数据
    uint32_t keypoint_id = 0; // 对应关键点ID
};

/**
 * @brief 轮廓结构
 */
struct Contour {
    uint32_t id = 0;                    // 轮廓ID
    std::vector<std::pair<int, int>> points;  // 轮廓点
    uint32_t area = 0;                  // 面积
    double perimeter = 0.0;             // 周长
    int32_t parent = -1;                // 父轮廓索引
    std::vector<uint32_t> children;     // 子轮廓索引
    bool is_hole = false;               // 是否为孔洞
    bool valid = false;
};

/**
 * @brief 轮廓属性结构
 */
struct ContourProperty {
    uint32_t id = 0;          // 轮廓ID
    double area = 0.0;        // 面积
    double perimeter = 0.0;   // 周长
    double circularity = 0.0; // 圆度
    double convexity = 0.0;   // 凸性
    double aspect_ratio = 0.0; // 长宽比
    double extent = 0.0;      // 占空比
    uint32_t min_x = 0;       // 边界框
    uint32_t min_y = 0;
    uint32_t max_x = 0;
    uint32_t max_y = 0;
    uint32_t centroid_x = 0;  // 质心
    uint32_t centroid_y = 0;
    std::vector<std::pair<int, int>> convex_hull; // 凸包
};

/**
 * @brief 匹配结果结构
 */
struct FeatureMatch {
    uint32_t query_id = 0;     // 查询特征ID
    uint32_t train_id = 0;     // 训练特征ID
    double distance = 0.0;      // 匹配距离
    bool valid = false;
};

/**
 * @brief 特征跟踪结果
 */
struct TrackResult {
    uint32_t prev_id = 0;      // 前一帧特征ID
    uint32_t curr_id = 0;      // 当前帧特征ID
    int32_t dx = 0;            // X方向位移
    int32_t dy = 0;            // Y方向位移
    double confidence = 0.0;   // 置信度
    bool found = false;
};

/**
 * @brief 特征检测工具函数
 */
namespace feature_utils {

// ==================== 角点检测算法 ====================

/**
 * @brief Harris角点检测
 */
inline void harris_corner(const ImageData& gray, std::vector<Corner>& corners,
                          double k = 0.04, int threshold = 50,
                          int block_size = 3) {
    corners.clear();
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;
    int half = block_size / 2;

    // 计算梯度
    std::vector<double> Ix(w * h, 0.0);
    std::vector<double> Iy(w * h, 0.0);

    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            size_t idx = y * w + x;
            // Sobel梯度
            Ix[idx] = (gray.data[(y) * w + (x + 1)] - gray.data[(y) * w + (x - 1)]) / 2.0;
            Iy[idx] = (gray.data[(y + 1) * w + (x)] - gray.data[(y - 1) * w + (x)]) / 2.0;
        }
    }

    // 计算Harris响应
    std::vector<double> response(w * h, 0.0);

    for (uint32_t y = half; y < h - half; ++y) {
        for (uint32_t x = half; x < w - half; ++x) {
            double Ixx = 0, Iyy = 0, Ixy = 0;

            // 在窗口内累加
            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    size_t idx = (y + dy) * w + (x + dx);
                    Ixx += Ix[idx] * Ix[idx];
                    Iyy += Iy[idx] * Iy[idx];
                    Ixy += Ix[idx] * Iy[idx];
                }
            }

            // Harris响应: R = det(M) - k * trace(M)^2
            double det = Ixx * Iyy - Ixy * Ixy;
            double trace = Ixx + Iyy;
            response[y * w + x] = det - k * trace * trace;
        }
    }

    // 非极大值抑制
    for (uint32_t y = half + 1; y < h - half - 1; ++y) {
        for (uint32_t x = half + 1; x < w - half - 1; ++x) {
            double r = response[y * w + x];
            if (r < threshold) continue;

            // 检查是否为局部最大值
            bool is_max = true;
            for (int dy = -1; dy <= 1 && is_max; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (response[(y + dy) * w + (x + dx)] > r) {
                        is_max = false;
                        break;
                    }
                }
            }

            if (is_max) {
                Corner c;
                c.x = x;
                c.y = y;
                c.response = r;
                c.valid = true;
                corners.push_back(c);
            }
        }
    }

    // 按响应值排序
    std::sort(corners.begin(), corners.end(),
              [](const Corner& a, const Corner& b) { return a.response > b.response; });
}

/**
 * @brief FAST角点检测（简化版，9点圆周比较）
 */
inline void fast_corner(const ImageData& gray, std::vector<Corner>& corners,
                        int threshold = 20) {
    corners.clear();
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;

    // FAST-9 圆周偏移（简化版，使用9个点）
    const int circle_r = 3;
    const int circle_dx[] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
    const int circle_dy[] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};
    const int n_points = 16;

    for (uint32_t y = 4; y < h - 4; ++y) {
        for (uint32_t x = 4; x < w - 4; ++x) {
            uint8_t center = gray.data[y * w + x];
            int brighter = 0;
            int darker = 0;

            // 检查圆周上的点
            for (int i = 0; i < n_points; ++i) {
                int px = static_cast<int>(x) + circle_dx[i];
                int py = static_cast<int>(y) + circle_dy[i];
                uint8_t val = gray.data[py * w + px];

                if (val > center + threshold) {
                    brighter++;
                } else if (val < center - threshold) {
                    darker++;
                }
            }

            // 如果连续9个点都比中心亮或暗
            if (brighter >= 9 || darker >= 9) {
                Corner c;
                c.x = x;
                c.y = y;
                c.response = static_cast<double>(std::max(brighter, darker));
                c.valid = true;
                corners.push_back(c);
            }
        }
    }

    // 按响应值排序
    std::sort(corners.begin(), corners.end(),
              [](const Corner& a, const Corner& b) { return a.response > b.response; });
}

/**
 * @brief Sobel角点检测（基于梯度方向变化）
 */
inline void sobel_corner(const ImageData& gray, std::vector<Corner>& corners,
                         int threshold = 50) {
    corners.clear();
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;

    // 计算梯度幅值和方向
    std::vector<double> magnitude(w * h, 0.0);
    std::vector<double> angle(w * h, 0.0);

    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            int gx = -gray.data[(y - 1) * w + (x - 1)] + gray.data[(y - 1) * w + (x + 1)]
                     - 2 * gray.data[y * w + (x - 1)] + 2 * gray.data[y * w + (x + 1)]
                     - gray.data[(y + 1) * w + (x - 1)] + gray.data[(y + 1) * w + (x + 1)];

            int gy = -gray.data[(y - 1) * w + (x - 1)] - 2 * gray.data[(y - 1) * w + x]
                     - gray.data[(y - 1) * w + (x + 1)]
                     + gray.data[(y + 1) * w + (x - 1)] + 2 * gray.data[(y + 1) * w + x]
                     + gray.data[(y + 1) * w + (x + 1)];

            size_t idx = y * w + x;
            magnitude[idx] = std::sqrt(gx * gx + gy * gy);
            angle[idx] = std::atan2(gy, gx);
        }
    }

    // 计算角点响应（梯度方向变化）
    for (uint32_t y = 2; y < h - 2; ++y) {
        for (uint32_t x = 2; x < w - 2; ++x) {
            double angle_sum = 0;
            double mag_sum = 0;
            int count = 0;

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    size_t idx = (y + dy) * w + (x + dx);
                    if (magnitude[idx] > 30) {
                        angle_sum += angle[idx];
                        mag_sum += magnitude[idx];
                        count++;
                    }
                }
            }

            if (count >= 3) {
                // 计算方向变化程度
                double angle_var = 0;
                double mean_angle = angle_sum / count;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        size_t idx = (y + dy) * w + (x + dx);
                        if (magnitude[idx] > 30) {
                            double diff = angle[idx] - mean_angle;
                            while (diff > M_PI) diff -= 2 * M_PI;
                            while (diff < -M_PI) diff += 2 * M_PI;
                            angle_var += diff * diff;
                        }
                    }
                }
                angle_var = std::sqrt(angle_var / count);

                if (angle_var > threshold / 100.0) {
                    Corner c;
                    c.x = x;
                    c.y = y;
                    c.response = angle_var * mag_sum;
                    c.valid = true;
                    corners.push_back(c);
                }
            }
        }
    }

    // 按响应值排序
    std::sort(corners.begin(), corners.end(),
              [](const Corner& a, const Corner& b) { return a.response > b.response; });
}

/**
 * @brief Moravec角点检测
 */
inline void moravec_corner(const ImageData& gray, std::vector<Corner>& corners,
                           int threshold = 10000, int window_size = 3) {
    corners.clear();
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;
    int half = window_size / 2;

    std::vector<double> response(w * h, 0.0);

    // 计算每个点的最小变化值
    const int dx[] = {1, 1, 0, -1};
    const int dy[] = {0, 1, 1, 1};
    const int n_dirs = 4;

    for (uint32_t y = half + 1; y < h - half - 1; ++y) {
        for (uint32_t x = half + 1; x < w - half - 1; ++x) {
            double min_var = std::numeric_limits<double>::max();

            // 计算各方向的平方差和
            for (int d = 0; d < n_dirs; ++d) {
                double sum = 0;
                for (int wy = -half; wy <= half; ++wy) {
                    for (int wx = -half; wx <= half; ++wx) {
                        int x1 = static_cast<int>(x) + wx;
                        int y1 = static_cast<int>(y) + wy;
                        int x2 = x1 + dx[d];
                        int y2 = y1 + dy[d];

                        if (x2 >= 0 && x2 < static_cast<int>(w) &&
                            y2 >= 0 && y2 < static_cast<int>(h)) {
                            double diff = gray.data[y1 * w + x1] - gray.data[y2 * w + x2];
                            sum += diff * diff;
                        }
                    }
                }
                min_var = std::min(min_var, sum);
            }

            response[y * w + x] = min_var;
        }
    }

    // 非极大值抑制并阈值化
    for (uint32_t y = half + 2; y < h - half - 2; ++y) {
        for (uint32_t x = half + 2; x < w - half - 2; ++x) {
            double r = response[y * w + x];
            if (r < threshold) continue;

            // 检查是否为局部最大值
            bool is_max = true;
            for (int dy = -1; dy <= 1 && is_max; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (response[(y + dy) * w + (x + dx)] > r) {
                        is_max = false;
                        break;
                    }
                }
            }

            if (is_max) {
                Corner c;
                c.x = x;
                c.y = y;
                c.response = r;
                c.valid = true;
                corners.push_back(c);
            }
        }
    }

    // 按响应值排序
    std::sort(corners.begin(), corners.end(),
              [](const Corner& a, const Corner& b) { return a.response > b.response; });
}

// ==================== 轮廓检测算法 ====================

/**
 * @brief 轮廓检测（基于边界跟踪）
 */
inline void find_contours(const ImageData& binary, std::vector<Contour>& contours,
                          bool find_hierarchy = true) {
    contours.clear();
    if (binary.empty() || binary.channels != 1) return;

    uint32_t w = binary.width;
    uint32_t h = binary.height;

    // 复制图像用于标记
    std::vector<uint8_t> img = binary.data;
    std::vector<int> label(w * h, -1);  // -1: 未处理, >=0: 轮廓ID

    // 链码方向（8方向）
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

    // 外轮廓起点查找
    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            size_t idx = y * w + x;

            // 查找外轮廓起点：当前为前景，上方为背景
            if (img[idx] > 0 && (y == 0 || img[idx - w] == 0) && label[idx] < 0) {
                Contour contour;
                contour.id = static_cast<uint32_t>(contours.size());
                contour.is_hole = false;

                // 边界跟踪
                int cx = static_cast<int>(x);
                int cy = static_cast<int>(y);
                int start_x = cx, start_y = cy;
                int dir = 0;  // 起始方向

                do {
                    // 添加点
                    contour.points.push_back({cx, cy});
                    label[cy * w + cx] = static_cast<int>(contour.id);

                    // 查找下一个边界点
                    bool found = false;
                    int start_dir = (dir + 5) % 8;  // 回溯方向

                    for (int i = 0; i < 8; ++i) {
                        int new_dir = (start_dir + i) % 8;
                        int nx = cx + dx[new_dir];
                        int ny = cy + dy[new_dir];

                        if (nx >= 0 && nx < static_cast<int>(w) &&
                            ny >= 0 && ny < static_cast<int>(h) &&
                            img[ny * w + nx] > 0) {
                            cx = nx;
                            cy = ny;
                            dir = new_dir;
                            found = true;
                            break;
                        }
                    }

                    if (!found) break;

                } while (cx != start_x || cy != start_y);

                // 计算面积
                contour.area = static_cast<uint32_t>(contour.points.size());
                contour.valid = true;
                contours.push_back(contour);
            }
        }
    }

    // 查找孔洞轮廓
    if (find_hierarchy) {
        for (uint32_t y = 1; y < h - 1; ++y) {
            for (uint32_t x = 1; x < w - 1; ++x) {
                size_t idx = y * w + x;

                // 查找孔洞起点：当前为前景，上方为前景（内部）
                if (img[idx] > 0 && img[idx - w] > 0 && label[idx] < 0) {
                    // 检查是否在某个轮廓内部
                    bool in_contour = false;
                    for (const auto& c : contours) {
                        // 简化的点在多边形内判断
                        int inside = 0;
                        for (size_t i = 0; i < c.points.size(); ++i) {
                            const auto& p1 = c.points[i];
                            const auto& p2 = c.points[(i + 1) % c.points.size()];
                            if ((p1.second > static_cast<int>(y)) !=
                                (p2.second > static_cast<int>(y))) {
                                double x_intersect = p1.first +
                                    (static_cast<double>(y) - p1.second) /
                                    (p2.second - p1.second) * (p2.first - p1.first);
                                if (static_cast<int>(x) < x_intersect) {
                                    inside = !inside;
                                }
                            }
                        }
                        if (inside) {
                            in_contour = true;
                            break;
                        }
                    }

                    if (in_contour) {
                        Contour contour;
                        contour.id = static_cast<uint32_t>(contours.size());
                        contour.is_hole = true;

                        // 边界跟踪（类似外轮廓）
                        int cx = static_cast<int>(x);
                        int cy = static_cast<int>(y);
                        int start_x = cx, start_y = cy;
                        int dir = 0;

                        do {
                            contour.points.push_back({cx, cy});
                            label[cy * w + cx] = static_cast<int>(contour.id);

                            bool found = false;
                            int start_dir = (dir + 5) % 8;

                            for (int i = 0; i < 8; ++i) {
                                int new_dir = (start_dir + i) % 8;
                                int nx = cx + dx[new_dir];
                                int ny = cy + dy[new_dir];

                                if (nx >= 0 && nx < static_cast<int>(w) &&
                                    ny >= 0 && ny < static_cast<int>(h) &&
                                    img[ny * w + nx] > 0) {
                                    cx = nx;
                                    cy = ny;
                                    dir = new_dir;
                                    found = true;
                                    break;
                                }
                            }

                            if (!found) break;

                        } while (cx != start_x || cy != start_y);

                        contour.area = static_cast<uint32_t>(contour.points.size());
                        contour.valid = true;
                        contours.push_back(contour);
                    }
                }
            }
        }
    }
}

/**
 * @brief 轮廓近似（多边形拟合）
 */
inline void approximate_contour(const Contour& contour, Contour& approx,
                                double epsilon = 2.0) {
    approx = contour;
    approx.points.clear();

    if (contour.points.size() < 3) {
        approx.points = contour.points;
        return;
    }

    // Douglas-Peucker算法
    std::vector<bool> keep(contour.points.size(), false);
    keep[0] = true;
    keep[contour.points.size() - 1] = true;

    std::function<void(size_t, size_t)> dp = [&](size_t start, size_t end) {
        if (end <= start + 1) return;

        double max_dist = 0;
        size_t max_idx = start;

        for (size_t i = start + 1; i < end; ++i) {
            // 点到线段的距离
            const auto& p = contour.points[i];
            const auto& p1 = contour.points[start];
            const auto& p2 = contour.points[end];

            double dx = p2.first - p1.first;
            double dy = p2.second - p1.second;
            double len = std::sqrt(dx * dx + dy * dy);

            double dist = 0;
            if (len < 1e-6) {
                dist = std::sqrt((p.first - p1.first) * (p.first - p1.first) +
                                (p.second - p1.second) * (p.second - p1.second));
            } else {
                double t = ((p.first - p1.first) * dx + (p.second - p1.second) * dy) / (len * len);
                t = std::max(0.0, std::min(1.0, t));
                double proj_x = p1.first + t * dx;
                double proj_y = p1.second + t * dy;
                dist = std::sqrt((p.first - proj_x) * (p.first - proj_x) +
                                (p.second - proj_y) * (p.second - proj_y));
            }

            if (dist > max_dist) {
                max_dist = dist;
                max_idx = i;
            }
        }

        if (max_dist > epsilon) {
            keep[max_idx] = true;
            dp(start, max_idx);
            dp(max_idx, end);
        }
    };

    dp(0, contour.points.size() - 1);

    for (size_t i = 0; i < contour.points.size(); ++i) {
        if (keep[i]) {
            approx.points.push_back(contour.points[i]);
        }
    }
}

/**
 * @brief 计算轮廓属性
 */
inline ContourProperty compute_contour_property(const Contour& contour) {
    ContourProperty prop;
    prop.id = contour.id;

    if (contour.points.empty()) return prop;

    // 计算边界框
    prop.min_x = std::numeric_limits<uint32_t>::max();
    prop.min_y = std::numeric_limits<uint32_t>::max();
    prop.max_x = 0;
    prop.max_y = 0;

    double sum_x = 0, sum_y = 0;
    for (const auto& p : contour.points) {
        prop.min_x = std::min(prop.min_x, static_cast<uint32_t>(std::max(0, p.first)));
        prop.min_y = std::min(prop.min_y, static_cast<uint32_t>(std::max(0, p.second)));
        prop.max_x = std::max(prop.max_x, static_cast<uint32_t>(std::max(0, p.first)));
        prop.max_y = std::max(prop.max_y, static_cast<uint32_t>(std::max(0, p.second)));
        sum_x += p.first;
        sum_y += p.second;
    }

    // 质心
    prop.centroid_x = static_cast<uint32_t>(sum_x / contour.points.size());
    prop.centroid_y = static_cast<uint32_t>(sum_y / contour.points.size());

    // 面积（格林公式）
    double area = 0;
    for (size_t i = 0; i < contour.points.size(); ++i) {
        const auto& p1 = contour.points[i];
        const auto& p2 = contour.points[(i + 1) % contour.points.size()];
        area += (p1.first * p2.second - p2.first * p1.second);
    }
    prop.area = std::abs(area) / 2.0;

    // 周长
    for (size_t i = 0; i < contour.points.size(); ++i) {
        const auto& p1 = contour.points[i];
        const auto& p2 = contour.points[(i + 1) % contour.points.size()];
        prop.perimeter += std::sqrt((p2.first - p1.first) * (p2.first - p1.first) +
                                    (p2.second - p1.second) * (p2.second - p1.second));
    }

    // 圆度
    if (prop.perimeter > 0) {
        prop.circularity = 4 * M_PI * prop.area / (prop.perimeter * prop.perimeter);
    }

    // 长宽比
    uint32_t width = prop.max_x - prop.min_x + 1;
    uint32_t height = prop.max_y - prop.min_y + 1;
    if (height > 0) {
        prop.aspect_ratio = static_cast<double>(width) / height;
    }

    // 占空比
    double bbox_area = width * height;
    if (bbox_area > 0) {
        prop.extent = prop.area / bbox_area;
    }

    // 凸包（简化实现）
    if (contour.points.size() >= 3) {
        // Graham scan算法
        auto cross = [](const std::pair<int, int>& O, const std::pair<int, int>& A,
                       const std::pair<int, int>& B) -> double {
            return (A.first - O.first) * (B.second - O.second) -
                   (A.second - O.second) * (B.first - O.first);
        };

        std::vector<std::pair<int, int>> pts = contour.points;

        // 找最低点
        size_t lowest = 0;
        for (size_t i = 1; i < pts.size(); ++i) {
            if (pts[i].second > pts[lowest].second ||
                (pts[i].second == pts[lowest].second && pts[i].first < pts[lowest].first)) {
                lowest = i;
            }
        }
        std::swap(pts[0], pts[lowest]);

        // 按极角排序
        auto pivot = pts[0];
        std::sort(pts.begin() + 1, pts.end(), [&pivot, &cross](const auto& a, const auto& b) {
            double cr = cross(pivot, a, b);
            if (cr != 0) return cr > 0;
            double da = (a.first - pivot.first) * (a.first - pivot.first) +
                       (a.second - pivot.second) * (a.second - pivot.second);
            double db = (b.first - pivot.first) * (b.first - pivot.first) +
                       (b.second - pivot.second) * (b.second - pivot.second);
            return da < db;
        });

        // 构建凸包
        std::vector<std::pair<int, int>> hull;
        for (const auto& p : pts) {
            while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0) {
                hull.pop_back();
            }
            hull.push_back(p);
        }

        prop.convex_hull = hull;

        // 凸性
        if (prop.area > 0) {
            double hull_area = 0;
            for (size_t i = 0; i < hull.size(); ++i) {
                const auto& p1 = hull[i];
                const auto& p2 = hull[(i + 1) % hull.size()];
                hull_area += (p1.first * p2.second - p2.first * p1.second);
            }
            hull_area = std::abs(hull_area) / 2.0;
            if (hull_area > 0) {
                prop.convexity = prop.area / hull_area;
            }
        }
    }

    return prop;
}

// ==================== 关键点检测算法 ====================

/**
 * @brief Blob检测（基于连通区域）
 */
inline void detect_blobs(const ImageData& binary, std::vector<KeyPoint>& keypoints,
                         uint32_t min_area = 10, uint32_t max_area = 100000) {
    keypoints.clear();
    if (binary.empty() || binary.channels != 1) return;

    uint32_t w = binary.width;
    uint32_t h = binary.height;

    // 标记连通区域
    std::vector<int> labels(w * h, 0);
    std::unordered_map<int, std::vector<std::pair<uint32_t, uint32_t>>> regions;
    int current_label = 0;
    std::vector<int> label_equiv(1, 0);

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx = y * w + x;
            if (binary.data[idx] == 0) continue;

            int left = (x > 0) ? labels[idx - 1] : 0;
            int top = (y > 0) ? labels[idx - w] : 0;

            if (left == 0 && top == 0) {
                current_label++;
                label_equiv.push_back(current_label);
                labels[idx] = current_label;
            } else if (left != 0 && top == 0) {
                labels[idx] = left;
            } else if (left == 0 && top != 0) {
                labels[idx] = top;
            } else {
                int min_l = std::min(left, top);
                int max_l = std::max(left, top);
                labels[idx] = min_l;
                if (max_l < static_cast<int>(label_equiv.size())) {
                    label_equiv[max_l] = min_l;
                }
            }
        }
    }

    // 合并等价标签
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx = y * w + x;
            int label = labels[idx];
            if (label == 0) continue;

            while (label_equiv[label] != label && label < static_cast<int>(label_equiv.size())) {
                label = label_equiv[label];
            }

            regions[label].push_back({x, y});
        }
    }

    // 提取关键点
    uint32_t kp_id = 1;
    for (const auto& pair : regions) {
        const auto& points = pair.second;
        if (points.size() < min_area || points.size() > max_area) continue;

        KeyPoint kp;
        kp.id = kp_id++;
        kp.size = std::sqrt(static_cast<double>(points.size()) * 4 / M_PI);

        // 计算质心
        double sum_x = 0, sum_y = 0;
        uint32_t min_x = w, min_y = h, max_x = 0, max_y = 0;

        for (const auto& p : points) {
            sum_x += p.first;
            sum_y += p.second;
            min_x = std::min(min_x, p.first);
            min_y = std::min(min_y, p.second);
            max_x = std::max(max_x, p.first);
            max_y = std::max(max_y, p.second);
        }

        kp.x = static_cast<uint32_t>(sum_x / points.size());
        kp.y = static_cast<uint32_t>(sum_y / points.size());
        kp.response = static_cast<double>(points.size());
        kp.valid = true;

        keypoints.push_back(kp);
    }
}

/**
 * @brief SIFT关键点检测（简化版，基于DoG）
 */
inline void sift_keypoints(const ImageData& gray, std::vector<KeyPoint>& keypoints,
                           int n_octaves = 4, int threshold = 10) {
    keypoints.clear();
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;

    // 高斯模糊
    auto gaussian_blur = [](const std::vector<uint8_t>& input, uint32_t width, uint32_t height,
                           double sigma) -> std::vector<double> {
        std::vector<double> output(width * height);
        int ksize = static_cast<int>(sigma * 3) * 2 + 1;
        int half = ksize / 2;

        // 创建高斯核
        std::vector<double> kernel(ksize);
        double sum = 0;
        for (int i = 0; i < ksize; ++i) {
            double x = i - half;
            kernel[i] = std::exp(-(x * x) / (2 * sigma * sigma));
            sum += kernel[i];
        }
        for (int i = 0; i < ksize; ++i) kernel[i] /= sum;

        // 水平模糊
        std::vector<double> temp(width * height);
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                double val = 0;
                for (int k = -half; k <= half; ++k) {
                    int nx = static_cast<int>(x) + k;
                    if (nx < 0) nx = -nx;
                    if (nx >= static_cast<int>(width)) nx = 2 * width - nx - 2;
                    val += input[y * width + nx] * kernel[k + half];
                }
                temp[y * width + x] = val;
            }
        }

        // 垂直模糊
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                double val = 0;
                for (int k = -half; k <= half; ++k) {
                    int ny = static_cast<int>(y) + k;
                    if (ny < 0) ny = -ny;
                    if (ny >= static_cast<int>(height)) ny = 2 * height - ny - 2;
                    val += temp[ny * width + x] * kernel[k + half];
                }
                output[y * width + x] = val;
            }
        }

        return output;
    };

    // 不同尺度的高斯模糊
    std::vector<std::vector<double>> gaussians;
    double sigma = 1.6;

    for (int o = 0; o < n_octaves && (w >> o) > 16 && (h >> o) > 16; ++o) {
        uint32_t ow = w >> o;
        uint32_t oh = h >> o;

        // 下采样
        std::vector<uint8_t> octave_img(ow * oh);
        if (o == 0) {
            octave_img = gray.data;
        } else {
            for (uint32_t y = 0; y < oh; ++y) {
                for (uint32_t x = 0; x < ow; ++x) {
                    octave_img[y * ow + x] = gray.data[(y << o) * w + (x << o)];
                }
            }
        }

        // 多尺度高斯
        for (int s = 0; s < 3; ++s) {
            gaussians.push_back(gaussian_blur(octave_img, ow, oh, sigma * std::pow(2.0, s / 3.0)));
        }
    }

    // DoG检测极值点
    uint32_t kp_id = 1;
    for (size_t g = 1; g + 1 < gaussians.size(); g += 2) {
        const auto& prev = gaussians[g - 1];
        const auto& curr = gaussians[g];
        const auto& next = gaussians[g + 1];

        int octave = g / 3;
        uint32_t ow = w >> octave;
        uint32_t oh = h >> octave;

        for (uint32_t y = 1; y < oh - 1; ++y) {
            for (uint32_t x = 1; x < ow - 1; ++x) {
                size_t idx = y * ow + x;
                double val = curr[idx];

                // 检查是否为极值
                bool is_extremum = true;

                for (int dy = -1; dy <= 1 && is_extremum; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dy == 0 && dx == 0) continue;

                        size_t nidx = (y + dy) * ow + (x + dx);
                        if (prev[nidx] >= val || curr[nidx] >= val || next[nidx] >= val) {
                            if (val > 0) {
                                is_extremum = false;
                                break;
                            }
                        }
                        if (prev[nidx] <= val || curr[nidx] <= val || next[nidx] <= val) {
                            if (val < 0) {
                                is_extremum = false;
                                break;
                            }
                        }
                    }
                }

                if (is_extremum && std::abs(val) > threshold) {
                    KeyPoint kp;
                    kp.id = kp_id++;
                    kp.x = x << octave;
                    kp.y = y << octave;
                    kp.response = std::abs(val);
                    kp.size = sigma * std::pow(2.0, (g % 3) / 3.0) * (1 << octave);
                    kp.valid = true;
                    keypoints.push_back(kp);
                }
            }
        }
    }
}

/**
 * @brief 简单描述符计算（基于梯度直方图）
 */
inline void compute_descriptor(const ImageData& gray, const KeyPoint& kp,
                               Descriptor& desc, int desc_size = 8) {
    desc.data.clear();
    desc.keypoint_id = kp.id;

    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;
    int half = desc_size / 2;

    desc.data.resize(desc_size * desc_size);

    // 提取局部区域
    for (int dy = -half; dy < half; ++dy) {
        for (int dx = -half; dx < half; ++dx) {
            int x = static_cast<int>(kp.x) + dx;
            int y = static_cast<int>(kp.y) + dy;

            if (x >= 0 && x < static_cast<int>(w) && y >= 0 && y < static_cast<int>(h)) {
                // 计算梯度
                int gx = 0, gy = 0;
                if (x > 0 && x < static_cast<int>(w) - 1) {
                    gx = gray.data[y * w + (x + 1)] - gray.data[y * w + (x - 1)];
                }
                if (y > 0 && y < static_cast<int>(h) - 1) {
                    gy = gray.data[(y + 1) * w + x] - gray.data[(y - 1) * w + x];
                }

                double mag = std::sqrt(gx * gx + gy * gy);
                desc.data[(dy + half) * desc_size + (dx + half)] = static_cast<float>(mag);
            }
        }
    }

    // 归一化
    float norm = 0;
    for (float v : desc.data) norm += v * v;
    norm = std::sqrt(norm) + 1e-7f;
    for (float& v : desc.data) v /= norm;
}

// ==================== 匹配算法 ====================

/**
 * @brief 关键点匹配（基于描述符）
 */
inline void match_keypoints(const std::vector<KeyPoint>& kp1,
                           const std::vector<Descriptor>& desc1,
                           const std::vector<KeyPoint>& kp2,
                           const std::vector<Descriptor>& desc2,
                           std::vector<FeatureMatch>& matches,
                           double ratio_threshold = 0.75) {
    matches.clear();

    for (size_t i = 0; i < desc1.size(); ++i) {
        double best_dist = std::numeric_limits<double>::max();
        double second_dist = std::numeric_limits<double>::max();
        int best_j = -1;

        for (size_t j = 0; j < desc2.size(); ++j) {
            // 计算欧氏距离
            double dist = 0;
            size_t min_size = std::min(desc1[i].data.size(), desc2[j].data.size());
            for (size_t k = 0; k < min_size; ++k) {
                double diff = desc1[i].data[k] - desc2[j].data[k];
                dist += diff * diff;
            }
            dist = std::sqrt(dist);

            if (dist < best_dist) {
                second_dist = best_dist;
                best_dist = dist;
                best_j = static_cast<int>(j);
            } else if (dist < second_dist) {
                second_dist = dist;
            }
        }

        // 比率测试
        if (best_j >= 0 && best_dist < ratio_threshold * second_dist) {
            FeatureMatch m;
            m.query_id = desc1[i].keypoint_id;
            m.train_id = desc2[best_j].keypoint_id;
            m.distance = best_dist;
            m.valid = true;
            matches.push_back(m);
        }
    }
}

/**
 * @brief 角点匹配（简化版，基于位置和局部模式）
 */
inline void match_corners(const std::vector<Corner>& corners1,
                          const std::vector<Corner>& corners2,
                          std::vector<FeatureMatch>& matches,
                          int search_radius = 50) {
    matches.clear();

    for (size_t i = 0; i < corners1.size(); ++i) {
        double best_dist = std::numeric_limits<double>::max();
        int best_j = -1;

        for (size_t j = 0; j < corners2.size(); ++j) {
            // 位置距离
            double dx = static_cast<double>(corners1[i].x) - corners2[j].x;
            double dy = static_cast<double>(corners1[i].y) - corners2[j].y;
            double pos_dist = std::sqrt(dx * dx + dy * dy);

            if (pos_dist > search_radius) continue;

            // 响应值差异
            double resp_diff = std::abs(corners1[i].response - corners2[j].response);

            double total_dist = pos_dist + resp_diff * 0.01;

            if (total_dist < best_dist) {
                best_dist = total_dist;
                best_j = static_cast<int>(j);
            }
        }

        if (best_j >= 0 && best_dist < search_radius) {
            FeatureMatch m;
            m.query_id = static_cast<uint32_t>(i);
            m.train_id = static_cast<uint32_t>(best_j);
            m.distance = best_dist;
            m.valid = true;
            matches.push_back(m);
        }
    }
}

// ==================== 跟踪算法 ====================

/**
 * @brief 特征跟踪（Lucas-Kanade光流）
 */
inline void track_features(const ImageData& prev_gray, const ImageData& curr_gray,
                          const std::vector<KeyPoint>& prev_kp,
                          std::vector<TrackResult>& tracks,
                          int window_size = 15, int max_iter = 20) {
    tracks.clear();
    if (prev_gray.empty() || curr_gray.empty() ||
        prev_gray.channels != 1 || curr_gray.channels != 1) return;

    uint32_t w = prev_gray.width;
    uint32_t h = prev_gray.height;
    int half = window_size / 2;

    for (const auto& kp : prev_kp) {
        TrackResult tr;
        tr.prev_id = kp.id;
        tr.found = false;

        int x0 = static_cast<int>(kp.x);
        int y0 = static_cast<int>(kp.y);

        if (x0 < half || x0 >= static_cast<int>(w) - half ||
            y0 < half || y0 >= static_cast<int>(h) - half) {
            tracks.push_back(tr);
            continue;
        }

        // 计算前一帧窗口内的梯度和
        double sum_Ix = 0, sum_Iy = 0, sum_IxIx = 0, sum_IyIy = 0, sum_IxIy = 0;

        for (int dy = -half; dy <= half; ++dy) {
            for (int dx = -half; dx <= half; ++dx) {
                int x = x0 + dx;
                int y = y0 + dy;

                double Ix = (prev_gray.data[y * w + (x + 1)] - prev_gray.data[y * w + (x - 1)]) / 2.0;
                double Iy = (prev_gray.data[(y + 1) * w + x] - prev_gray.data[(y - 1) * w + x]) / 2.0;

                sum_IxIx += Ix * Ix;
                sum_IyIy += Iy * Iy;
                sum_IxIy += Ix * Iy;
            }
        }

        // 检查是否可逆
        double det = sum_IxIx * sum_IyIy - sum_IxIy * sum_IxIy;
        if (std::abs(det) < 1e-6) {
            tracks.push_back(tr);
            continue;
        }

        // 迭代求解光流
        double vx = 0, vy = 0;

        for (int iter = 0; iter < max_iter; ++iter) {
            double sum_It = 0, sum_IxIt = 0, sum_IyIt = 0;

            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    int x = x0 + dx;
                    int y = y0 + dy;
                    int nx = static_cast<int>(x + vx + 0.5);
                    int ny = static_cast<int>(y + vy + 0.5);

                    if (nx < 0 || nx >= static_cast<int>(w) ||
                        ny < 0 || ny >= static_cast<int>(h)) {
                        continue;
                    }

                    double Ix = (prev_gray.data[y * w + (x + 1)] - prev_gray.data[y * w + (x - 1)]) / 2.0;
                    double Iy = (prev_gray.data[(y + 1) * w + x] - prev_gray.data[(y - 1) * w + x]) / 2.0;
                    double It = curr_gray.data[ny * w + nx] - prev_gray.data[y * w + x];

                    sum_IxIt += Ix * It;
                    sum_IyIt += Iy * It;
                }
            }

            // 求解2x2系统
            double dvx = (-sum_IyIy * sum_IxIt + sum_IxIy * sum_IyIt) / det;
            double dvy = (sum_IxIy * sum_IxIt - sum_IxIx * sum_IyIt) / det;

            vx += dvx;
            vy += dvy;

            if (std::abs(dvx) < 0.01 && std::abs(dvy) < 0.01) break;
        }

        // 检查新位置是否在图像范围内
        int nx = static_cast<int>(x0 + vx + 0.5);
        int ny = static_cast<int>(y0 + vy + 0.5);

        if (nx >= 0 && nx < static_cast<int>(w) &&
            ny >= 0 && ny < static_cast<int>(h)) {
            tr.found = true;
            tr.dx = static_cast<int32_t>(vx);
            tr.dy = static_cast<int32_t>(vy);
            tr.confidence = 1.0 / (1.0 + std::abs(vx) + std::abs(vy));
            tracks.push_back(tr);
        } else {
            tracks.push_back(tr);
        }
    }
}

/**
 * @brief 特征聚类（K-means）
 */
inline void cluster_features(const std::vector<KeyPoint>& keypoints,
                            std::vector<std::vector<uint32_t>>& clusters,
                            int k = 4, int max_iter = 10) {
    clusters.clear();
    if (keypoints.empty() || k <= 0) return;

    clusters.resize(k);

    // 初始化聚类中心
    std::vector<std::pair<double, double>> centers(k);
    for (int i = 0; i < k; ++i) {
        centers[i] = {static_cast<double>(keypoints[i % keypoints.size()].x),
                      static_cast<double>(keypoints[i % keypoints.size()].y)};
    }

    // 迭代
    for (int iter = 0; iter < max_iter; ++iter) {
        // 清空聚类
        for (auto& c : clusters) c.clear();

        // 分配点到最近的聚类中心
        for (size_t i = 0; i < keypoints.size(); ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int min_cluster = 0;

            for (int j = 0; j < k; ++j) {
                double dx = keypoints[i].x - centers[j].first;
                double dy = keypoints[i].y - centers[j].second;
                double dist = dx * dx + dy * dy;

                if (dist < min_dist) {
                    min_dist = dist;
                    min_cluster = j;
                }
            }

            clusters[min_cluster].push_back(static_cast<uint32_t>(i));
        }

        // 更新聚类中心
        for (int j = 0; j < k; ++j) {
            if (clusters[j].empty()) continue;

            double sum_x = 0, sum_y = 0;
            for (uint32_t idx : clusters[j]) {
                sum_x += keypoints[idx].x;
                sum_y += keypoints[idx].y;
            }
            centers[j] = {sum_x / clusters[j].size(), sum_y / clusters[j].size()};
        }
    }
}

/**
 * @brief 绘制角点
 */
inline void draw_corners(ImageData& img, const std::vector<Corner>& corners,
                         uint8_t r = 255, uint8_t g = 0, uint8_t b = 0, int radius = 3) {
    if (img.channels != 3) return;

    for (const auto& c : corners) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx * dx + dy * dy <= radius * radius) {
                    int px = static_cast<int>(c.x) + dx;
                    int py = static_cast<int>(c.y) + dy;

                    if (px >= 0 && px < static_cast<int>(img.width) &&
                        py >= 0 && py < static_cast<int>(img.height)) {
                        size_t idx = (py * img.width + px) * 3;
                        img.data[idx] = b;
                        img.data[idx + 1] = g;
                        img.data[idx + 2] = r;
                    }
                }
            }
        }
    }
}

/**
 * @brief 绘制关键点
 */
inline void draw_keypoints(ImageData& img, const std::vector<KeyPoint>& keypoints,
                          uint8_t r = 0, uint8_t g = 255, uint8_t b = 0) {
    if (img.channels != 3) return;

    for (const auto& kp : keypoints) {
        int x = static_cast<int>(kp.x);
        int y = static_cast<int>(kp.y);
        int radius = static_cast<int>(kp.size / 2 + 1);

        // 绘制圆
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx * dx + dy * dy <= radius * radius) {
                    int px = x + dx;
                    int py = y + dy;

                    if (px >= 0 && px < static_cast<int>(img.width) &&
                        py >= 0 && py < static_cast<int>(img.height)) {
                        size_t idx = (py * img.width + px) * 3;
                        img.data[idx] = b;
                        img.data[idx + 1] = g;
                        img.data[idx + 2] = r;
                    }
                }
            }
        }

        // 绘制方向线
        if (kp.angle != 0) {
            int len = radius + 3;
            int ex = x + static_cast<int>(len * std::cos(kp.angle));
            int ey = y + static_cast<int>(len * std::sin(kp.angle));

            for (int t = 0; t <= len * 2; ++t) {
                int px = x + (ex - x) * t / (len * 2);
                int py = y + (ey - y) * t / (len * 2);

                if (px >= 0 && px < static_cast<int>(img.width) &&
                    py >= 0 && py < static_cast<int>(img.height)) {
                    size_t idx = (py * img.width + px) * 3;
                    img.data[idx] = b;
                    img.data[idx + 1] = g;
                    img.data[idx + 2] = r;
                }
            }
        }
    }
}

/**
 * @brief 绘制轮廓
 */
inline void draw_contours(ImageData& img, const std::vector<Contour>& contours,
                         uint8_t r = 255, uint8_t g = 255, uint8_t b = 0) {
    if (img.channels != 3) return;

    for (const auto& contour : contours) {
        for (size_t i = 0; i < contour.points.size(); ++i) {
            const auto& p1 = contour.points[i];
            const auto& p2 = contour.points[(i + 1) % contour.points.size()];

            // Bresenham直线算法
            int x1 = p1.first, y1 = p1.second;
            int x2 = p2.first, y2 = p2.second;
            int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
            int sx = (x1 < x2) ? 1 : -1, sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy;

            while (true) {
                if (x1 >= 0 && x1 < static_cast<int>(img.width) &&
                    y1 >= 0 && y1 < static_cast<int>(img.height)) {
                    size_t idx = (y1 * img.width + x1) * 3;
                    img.data[idx] = b;
                    img.data[idx + 1] = g;
                    img.data[idx + 2] = r;
                }

                if (x1 == x2 && y1 == y2) break;

                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x1 += sx; }
                if (e2 < dx) { err += dx; y1 += sy; }
            }
        }
    }
}

} // namespace feature_utils

// ==================== 角点检测节点 ====================

/**
 * @brief Harris角点检测节点
 */
class HarrisCornerNode : public INode {
public:
    HarrisCornerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief FAST角点检测节点
 */
class FastCornerNode : public INode {
public:
    FastCornerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief Sobel角点检测节点
 */
class SobelCornerNode : public INode {
public:
    SobelCornerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief Moravec角点检测节点
 */
class MoravecCornerNode : public INode {
public:
    MoravecCornerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 轮廓检测节点 ====================

/**
 * @brief 轮廓检测节点
 */
class ContourDetectNode : public INode {
public:
    ContourDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓近似节点
 */
class ContourApproxNode : public INode {
public:
    ContourApproxNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓层次分析节点
 */
class ContourHierarchyNode : public INode {
public:
    ContourHierarchyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 轮廓属性计算节点
 */
class ContourPropertyNode : public INode {
public:
    ContourPropertyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 关键点检测节点 ====================

/**
 * @brief Blob检测节点
 */
class BlobDetectNode : public INode {
public:
    BlobDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 关键点匹配节点
 */
class KeyPointMatchNode : public INode {
public:
    KeyPointMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief SIFT关键点检测节点（简化版）
 */
class SIFTNode : public INode {
public:
    SIFTNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 特征描述节点 ====================

/**
 * @brief 角点匹配节点
 */
class CornerMatchNode : public INode {
public:
    CornerMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 特征匹配节点
 */
class FeatureMatchNode : public INode {
public:
    FeatureMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 特征跟踪节点
 */
class FeatureTrackNode : public INode {
public:
    FeatureTrackNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 特征聚类节点
 */
class FeatureClusterNode : public INode {
public:
    FeatureClusterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf