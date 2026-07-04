/**
 * @file calibration_pattern.h
 * @brief 标定板检测节点（纯C++实现，不依赖OpenCV）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 标定角点结构
 */
struct CalibCorner {
    float x = 0.0f;           // X坐标（亚像素精度）
    float y = 0.0f;           // Y坐标（亚像素精度）
    double response = 0.0;    // 响应值
    int row = -1;             // 在标定板上的行索引
    int col = -1;             // 在标定板上的列索引
    int id = -1;              // 序号（按行优先顺序）
    bool valid = false;
};

/**
 * @brief 圆点结构
 */
struct CirclePoint {
    float center_x = 0.0f;    // 圆心X坐标
    float center_y = 0.0f;    // 圆心Y坐标
    float radius = 0.0f;      // 半径
    double score = 0.0;       // 圆度评分
    int row = -1;             // 在标定板上的行索引
    int col = -1;             // 在标定板上的列索引
    int id = -1;              // 序号
    bool valid = false;
};

/**
 * @brief 标定板检测结果
 */
struct PatternResult {
    bool found = false;                    // 是否检测到标定板
    int rows = 0;                          // 检测到的行数
    int cols = 0;                          // 检测到的列数
    std::vector<CalibCorner> corners;      // 角点列表
    std::vector<CirclePoint> circles;      // 圆点列表
    float avg_error = 0.0f;                // 平均重投影误差
    float max_error = 0.0f;                // 最大重投影误差
    double confidence = 0.0;               // 置信度
};

/**
 * @brief 标定质量评估结果
 */
struct QualityResult {
    bool valid = false;                    // 是否有效
    float rms_error = 0.0f;                // RMS误差
    float max_error = 0.0f;                // 最大误差
    float min_error = 0.0f;                // 最小误差
    float coverage = 0.0f;                 // 覆盖率（检测到的点数/预期点数）
    float uniformity = 0.0f;               // 分布均匀性
    float symmetry = 0.0f;                 // 对称性评分
    int total_points = 0;                  // 总点数
    int valid_points = 0;                  // 有效点数
    std::string quality_level;             // 质量等级（Excellent/Good/Average/Poor）
};

/**
 * @brief 标定板检测工具函数
 */
namespace calibration_pattern_utils {

// ==================== 图像预处理 ====================

/**
 * @brief 转换为灰度图像
 */
inline void to_gray(const ImageData& input, ImageData& gray) {
    if (input.channels == 1) {
        gray = input;
        return;
    }

    gray.width = input.width;
    gray.height = input.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(gray.width * gray.height);
    gray.timestamp = input.timestamp;
    gray.frame_id = input.frame_id;
    gray.source_id = input.source_id;

    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            gray.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i];
        }
    }
}

/**
 * @brief 高斯模糊
 */
inline void gaussian_blur(const ImageData& input, ImageData& output, double sigma = 1.0) {
    output = input;
    if (input.empty()) return;

    uint32_t w = input.width;
    uint32_t h = input.height;

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
    std::vector<double> temp(w * h);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            double val = 0;
            for (int k = -half; k <= half; ++k) {
                int nx = static_cast<int>(x) + k;
                if (nx < 0) nx = -nx;
                if (nx >= static_cast<int>(w)) nx = 2 * static_cast<int>(w) - nx - 2;
                val += input.data[y * w + nx] * kernel[k + half];
            }
            temp[y * w + x] = val;
        }
    }

    // 垂直模糊
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            double val = 0;
            for (int k = -half; k <= half; ++k) {
                int ny = static_cast<int>(y) + k;
                if (ny < 0) ny = -ny;
                if (ny >= static_cast<int>(h)) ny = 2 * static_cast<int>(h) - ny - 2;
                val += temp[ny * w + x] * kernel[k + half];
            }
            output.data[y * w + x] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
        }
    }
}

/**
 * @brief 二值化
 */
inline void threshold(const ImageData& gray, ImageData& binary, int thresh = 128) {
    binary = gray;
    for (size_t i = 0; i < gray.data.size(); ++i) {
        binary.data[i] = (gray.data[i] > thresh) ? 255 : 0;
    }
}

/**
 * @brief 自适应阈值二值化（局部均值）
 */
inline void adaptive_threshold(const ImageData& gray, ImageData& binary,
                               int block_size = 11, int offset = 2) {
    binary.width = gray.width;
    binary.height = gray.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(gray.width * gray.height);

    uint32_t w = gray.width;
    uint32_t h = gray.height;
    int half = block_size / 2;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            // 计算局部均值
            double sum = 0;
            int count = 0;

            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    int nx = static_cast<int>(x) + dx;
                    int ny = static_cast<int>(y) + dy;

                    if (nx >= 0 && nx < static_cast<int>(w) &&
                        ny >= 0 && ny < static_cast<int>(h)) {
                        sum += gray.data[ny * w + nx];
                        count++;
                    }
                }
            }

            int local_mean = static_cast<int>(sum / count);
            binary.data[y * w + x] = (gray.data[y * w + x] > local_mean - offset) ? 255 : 0;
        }
    }
}

// ==================== 角点检测算法 ====================

/**
 * @brief Harris角点检测（返回亚像素精度）
 */
inline void harris_corners(const ImageData& gray, std::vector<CalibCorner>& corners,
                           double k = 0.04, int threshold = 50, int block_size = 3) {
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

            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    size_t idx = (y + dy) * w + (x + dx);
                    Ixx += Ix[idx] * Ix[idx];
                    Iyy += Iy[idx] * Iy[idx];
                    Ixy += Ix[idx] * Iy[idx];
                }
            }

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
                CalibCorner c;
                c.x = static_cast<float>(x);
                c.y = static_cast<float>(y);
                c.response = r;
                c.valid = true;
                corners.push_back(c);
            }
        }
    }

    // 按响应值排序
    std::sort(corners.begin(), corners.end(),
              [](const CalibCorner& a, const CalibCorner& b) { return a.response > b.response; });
}

/**
 * @brief 亚像素角点定位（泰勒展开优化）
 */
inline void refine_corner_location(const ImageData& gray, CalibCorner& corner,
                                   int window_size = 5, int max_iter = 10) {
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;
    int half = window_size / 2;

    float x = corner.x;
    float y = corner.y;

    for (int iter = 0; iter < max_iter; ++iter) {
        int ix = static_cast<int>(x);
        int iy = static_cast<int>(y);

        if (ix < half + 1 || ix >= static_cast<int>(w) - half - 1 ||
            iy < half + 1 || iy >= static_cast<int>(h) - half - 1) break;

        // 计算梯度
        double A = 0, B = 0, C = 0;
        double bx = 0, by = 0;

        for (int dy = -half; dy <= half; ++dy) {
            for (int dx = -half; dx <= half; ++dx) {
                int px = ix + dx;
                int py = iy + dy;

                double gx = (gray.data[py * w + (px + 1)] - gray.data[py * w + (px - 1)]) / 2.0;
                double gy = (gray.data[(py + 1) * w + px] - gray.data[(py - 1) * w + px]) / 2.0;

                A += gx * gx;
                B += gx * gy;
                C += gy * gy;

                // 计算二阶导数近似
                double dx_val = gray.data[py * w + px] - gray.data[iy * w + ix];
                bx += gx * dx_val;
                by += gy * dx_val;
            }
        }

        // 求解线性方程组
        double det = A * C - B * B;
        if (std::abs(det) < 1e-10) break;

        float dx_new = static_cast<float>((C * bx - B * by) / det);
        float dy_new = static_cast<float>((A * by - B * bx) / det);

        x += dx_new;
        y += dy_new;

        // 收敛判断
        if (std::abs(dx_new) < 0.01f && std::abs(dy_new) < 0.01f) break;
    }

    corner.x = x;
    corner.y = y;
}

/**
 * @brief 批量亚像素角点定位
 */
inline void refine_corners(const ImageData& gray, std::vector<CalibCorner>& corners,
                           int window_size = 5) {
    for (auto& corner : corners) {
        refine_corner_location(gray, corner, window_size);
    }
}

// ==================== 棋盘格检测 ====================

/**
 * @brief 检测棋盘格结构
 */
inline bool detect_checkerboard(const ImageData& gray, PatternResult& result,
                                int pattern_rows, int pattern_cols,
                                int threshold = 50, int min_corners = 100) {
    result.found = false;
    result.rows = pattern_rows;
    result.cols = pattern_cols;
    result.corners.clear();

    if (gray.empty()) return false;

    // 1. Harris角点检测
    std::vector<CalibCorner> all_corners;
    harris_corners(gray, all_corners, 0.04, threshold, 5);

    if (all_corners.size() < min_corners) return false;

    // 2. 亚像素定位
    refine_corners(gray, all_corners, 5);

    // 3. 网格结构验证和排序
    int expected_corners = pattern_rows * pattern_cols;

    // 计算角点的统计分布
    if (all_corners.empty()) return false;

    // 计算平均间距（基于最近的预期角点数）
    std::vector<float> distances;
    for (size_t i = 0; i < std::min(all_corners.size(), static_cast<size_t>(expected_corners * 2)); ++i) {
        for (size_t j = i + 1; j < std::min(all_corners.size(), static_cast<size_t>(expected_corners * 2)); ++j) {
            float dx = all_corners[i].x - all_corners[j].x;
            float dy = all_corners[i].y - all_corners[j].y;
            distances.push_back(std::sqrt(dx * dx + dy * dy));
        }
    }

    if (distances.empty()) return false;

    // 排序并取中位数作为估计间距
    std::sort(distances.begin(), distances.end());
    float median_dist = distances[distances.size() / 2];
    float cell_size = median_dist;

    // 4. 网格组织 - 基于棋盘格的规则结构
    // 找到最左上角的角点作为起点
    CalibCorner top_left = all_corners[0];
    for (const auto& c : all_corners) {
        if (c.x + c.y < top_left.x + top_left.y) {
            top_left = c;
        }
    }

    // 尝试构建网格
    std::vector<std::vector<CalibCorner>> grid(pattern_rows, std::vector<CalibCorner>(pattern_cols));

    // 简化实现：基于距离聚类
    float tolerance = cell_size * 0.4f;

    // 按行分组
    std::vector<std::vector<CalibCorner>> row_groups;
    for (const auto& c : all_corners) {
        bool found_row = false;
        for (auto& row : row_groups) {
            if (!row.empty()) {
                float dy = std::abs(c.y - row[0].y);
                if (dy < tolerance) {
                    row.push_back(c);
                    found_row = true;
                    break;
                }
            }
        }
        if (!found_row) {
            row_groups.push_back({c});
        }
    }

    // 检查是否有足够的行
    if (row_groups.size() < pattern_rows) return false;

    // 按Y坐标排序行
    std::sort(row_groups.begin(), row_groups.end(),
              [](const std::vector<CalibCorner>& a, const std::vector<CalibCorner>& b) {
                  return a[0].y < b[0].y;
              });

    // 对每行按X坐标排序
    for (auto& row : row_groups) {
        std::sort(row.begin(), row.end(),
                  [](const CalibCorner& a, const CalibCorner& b) { return a.x < b.x; });
    }

    // 验证每行的列数
    for (const auto& row : row_groups) {
        if (row.size() < pattern_cols) return false;
    }

    // 提取网格角点
    result.corners.clear();
    for (int r = 0; r < pattern_rows; ++r) {
        for (int c = 0; c < pattern_cols; ++c) {
            CalibCorner corner = row_groups[r][c];
            corner.row = r;
            corner.col = c;
            corner.id = r * pattern_cols + c;
            result.corners.push_back(corner);
        }
    }

    // 5. 验证棋盘格结构（相邻角点间距一致性）
    float avg_spacing = 0;
    int spacing_count = 0;

    for (int r = 0; r < pattern_rows; ++r) {
        for (int c = 0; c < pattern_cols - 1; ++c) {
            float dx = result.corners[r * pattern_cols + c + 1].x -
                      result.corners[r * pattern_cols + c].x;
            float dy = result.corners[r * pattern_cols + c + 1].y -
                      result.corners[r * pattern_cols + c].y;
            avg_spacing += std::sqrt(dx * dx + dy * dy);
            spacing_count++;
        }
    }

    for (int r = 0; r < pattern_rows - 1; ++r) {
        for (int c = 0; c < pattern_cols; ++c) {
            float dx = result.corners[(r + 1) * pattern_cols + c].x -
                      result.corners[r * pattern_cols + c].x;
            float dy = result.corners[(r + 1) * pattern_cols + c].y -
                      result.corners[r * pattern_cols + c].y;
            avg_spacing += std::sqrt(dx * dx + dy * dy);
            spacing_count++;
        }
    }

    avg_spacing /= spacing_count;

    // 检查间距一致性
    float spacing_variance = 0;
    for (int r = 0; r < pattern_rows; ++r) {
        for (int c = 0; c < pattern_cols - 1; ++c) {
            float dx = result.corners[r * pattern_cols + c + 1].x -
                      result.corners[r * pattern_cols + c].x;
            float dy = result.corners[r * pattern_cols + c + 1].y -
                      result.corners[r * pattern_cols + c].y;
            float spacing = std::sqrt(dx * dx + dy * dy);
            spacing_variance += (spacing - avg_spacing) * (spacing - avg_spacing);
        }
    }

    float spacing_std = std::sqrt(spacing_variance / spacing_count);
    if (spacing_std > avg_spacing * 0.2f) return false;  // 间距差异超过20%，失败

    result.found = true;
    result.confidence = 1.0 - spacing_std / avg_spacing;

    return true;
}

// ==================== 圆点检测 ====================

/**
 * @brief Hough圆检测（简化版）
 */
inline void detect_circles_hough(const ImageData& gray, std::vector<CirclePoint>& circles,
                                 float min_radius = 5, float max_radius = 50,
                                 int threshold = 50, int min_distance = 10) {
    circles.clear();
    if (gray.empty() || gray.channels != 1) return;

    uint32_t w = gray.width;
    uint32_t h = gray.height;

    // 边缘检测（Sobel）
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

    // 简化的Hough变换：累加器
    int r_range = static_cast<int>(max_radius - min_radius);
    std::vector<std::vector<int>> accumulator(w * h, std::vector<int>(r_range + 1, 0));

    // 对边缘点投票
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx = y * w + x;
            if (magnitude[idx] < threshold) continue;

            double theta = angle[idx];

            // 对可能的半径投票
            for (int r = static_cast<int>(min_radius); r <= static_cast<int>(max_radius); ++r) {
                int cx = static_cast<int>(x) - static_cast<int>(r * std::cos(theta));
                int cy = static_cast<int>(y) - static_cast<int>(r * std::sin(theta));

                if (cx >= 0 && cx < static_cast<int>(w) &&
                    cy >= 0 && cy < static_cast<int>(h)) {
                    accumulator[cy * w + cx][r - static_cast<int>(min_radius)]++;
                }
            }
        }
    }

    // 找峰值
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            for (int r = static_cast<int>(min_radius); r <= static_cast<int>(max_radius); ++r) {
                int votes = accumulator[y * w + x][r - static_cast<int>(min_radius)];
                if (votes < threshold * 2) continue;

                // 检查是否为局部最大
                bool is_max = true;
                int check_range = min_distance / 2;

                for (int dy = -check_range; dy <= check_range && is_max; ++dy) {
                    for (int dx = -check_range; dx <= check_range; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = static_cast<int>(x) + dx;
                        int ny = static_cast<int>(y) + dy;
                        if (nx >= 0 && nx < static_cast<int>(w) &&
                            ny >= 0 && ny < static_cast<int>(h)) {
                            for (int nr = r - 1; nr <= r + 1; ++nr) {
                                if (nr >= static_cast<int>(min_radius) && nr <= static_cast<int>(max_radius)) {
                                    if (accumulator[ny * w + nx][nr - static_cast<int>(min_radius)] > votes) {
                                        is_max = false;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                if (is_max) {
                    CirclePoint cp;
                    cp.center_x = static_cast<float>(x);
                    cp.center_y = static_cast<float>(y);
                    cp.radius = static_cast<float>(r);
                    cp.score = static_cast<double>(votes);
                    cp.valid = true;
                    circles.push_back(cp);
                }
            }
        }
    }

    // 按分数排序
    std::sort(circles.begin(), circles.end(),
              [](const CirclePoint& a, const CirclePoint& b) { return a.score > b.score; });

    // 过滤距离太近的圆
    std::vector<CirclePoint> filtered;
    for (const auto& c : circles) {
        bool too_close = false;
        for (const auto& fc : filtered) {
            float dx = c.center_x - fc.center_x;
            float dy = c.center_y - fc.center_y;
            if (std::sqrt(dx * dx + dy * dy) < min_distance) {
                too_close = true;
                break;
            }
        }
        if (!too_close) filtered.push_back(c);
    }

    circles = filtered;
}

/**
 * @brief 椭圆拟合（最小二乘）
 */
inline void fit_ellipse(const std::vector<std::pair<int, int>>& points,
                        float& center_x, float& center_y, float& a, float& b) {
    if (points.size() < 5) {
        center_x = center_y = a = b = 0;
        return;
    }

    // 简化实现：使用二阶矩估计
    double sum_x = 0, sum_y = 0;
    for (const auto& p : points) {
        sum_x += p.first;
        sum_y += p.second;
    }

    center_x = static_cast<float>(sum_x / points.size());
    center_y = static_cast<float>(sum_y / points.size());

    // 计算协方差矩阵
    double cov_xx = 0, cov_yy = 0, cov_xy = 0;
    for (const auto& p : points) {
        double dx = p.first - center_x;
        double dy = p.second - center_y;
        cov_xx += dx * dx;
        cov_yy += dy * dy;
        cov_xy += dx * dy;
    }

    cov_xx /= points.size();
    cov_yy /= points.size();
    cov_xy /= points.size();

    // 简化：取主轴方向的特征值作为半轴长度
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;

    double lambda1 = (trace + std::sqrt(trace * trace - 4 * det)) / 2;
    double lambda2 = (trace - std::sqrt(trace * trace - 4 * det)) / 2;

    a = static_cast<float>(2 * std::sqrt(std::abs(lambda1)));
    b = static_cast<float>(2 * std::sqrt(std::abs(lambda2)));
}

/**
 * @brief 圆点阵列检测
 */
inline bool detect_circle_grid(const ImageData& gray, PatternResult& result,
                               int pattern_rows, int pattern_cols,
                               float min_radius = 5, float max_radius = 50,
                               int threshold = 50) {
    result.found = false;
    result.rows = pattern_rows;
    result.cols = pattern_cols;
    result.circles.clear();

    if (gray.empty()) return false;

    // 1. 检测圆
    std::vector<CirclePoint> all_circles;
    detect_circles_hough(gray, all_circles, min_radius, max_radius, threshold);

    int expected_circles = pattern_rows * pattern_cols;
    if (all_circles.size() < expected_circles) return false;

    // 2. 组织网格结构
    // 计算平均间距
    std::vector<float> distances;
    for (size_t i = 0; i < all_circles.size(); ++i) {
        for (size_t j = i + 1; j < all_circles.size(); ++j) {
            float dx = all_circles[i].center_x - all_circles[j].center_x;
            float dy = all_circles[i].center_y - all_circles[j].center_y;
            distances.push_back(std::sqrt(dx * dx + dy * dy));
        }
    }

    std::sort(distances.begin(), distances.end());
    float cell_size = distances[distances.size() / 4];  // 取较小的四分位数

    // 按行分组
    float tolerance = cell_size * 0.4f;
    std::vector<std::vector<CirclePoint>> row_groups;

    for (const auto& c : all_circles) {
        bool found_row = false;
        for (auto& row : row_groups) {
            if (!row.empty()) {
                float dy = std::abs(c.center_y - row[0].center_y);
                if (dy < tolerance) {
                    row.push_back(c);
                    found_row = true;
                    break;
                }
            }
        }
        if (!found_row) {
            row_groups.push_back({c});
        }
    }

    // 检查行数
    if (row_groups.size() < pattern_rows) return false;

    // 按Y排序行
    std::sort(row_groups.begin(), row_groups.end(),
              [](const std::vector<CirclePoint>& a, const std::vector<CirclePoint>& b) {
                  return a[0].center_y < b[0].center_y;
              });

    // 按X排序每行
    for (auto& row : row_groups) {
        std::sort(row.begin(), row.end(),
                  [](const CirclePoint& a, const CirclePoint& b) {
                      return a.center_x < b.center_x;
                  });
    }

    // 验证列数
    for (const auto& row : row_groups) {
        if (row.size() < pattern_cols) return false;
    }

    // 提取网格圆点
    result.circles.clear();
    for (int r = 0; r < pattern_rows; ++r) {
        for (int c = 0; c < pattern_cols; ++c) {
            CirclePoint cp = row_groups[r][c];
            cp.row = r;
            cp.col = c;
            cp.id = r * pattern_cols + c;
            result.circles.push_back(cp);
        }
    }

    // 验证间距一致性
    float avg_spacing = 0;
    int spacing_count = 0;

    for (int r = 0; r < pattern_rows; ++r) {
        for (int c = 0; c < pattern_cols - 1; ++c) {
            float dx = result.circles[r * pattern_cols + c + 1].center_x -
                      result.circles[r * pattern_cols + c].center_x;
            float dy = result.circles[r * pattern_cols + c + 1].center_y -
                      result.circles[r * pattern_cols + c].center_y;
            avg_spacing += std::sqrt(dx * dx + dy * dy);
            spacing_count++;
        }
    }

    avg_spacing /= spacing_count;

    // 检查间距一致性
    float spacing_variance = 0;
    for (int r = 0; r < pattern_rows; ++r) {
        for (int c = 0; c < pattern_cols - 1; ++c) {
            float dx = result.circles[r * pattern_cols + c + 1].center_x -
                      result.circles[r * pattern_cols + c].center_x;
            float dy = result.circles[r * pattern_cols + c + 1].center_y -
                      result.circles[r * pattern_cols + c].center_y;
            float spacing = std::sqrt(dx * dx + dy * dy);
            spacing_variance += (spacing - avg_spacing) * (spacing - avg_spacing);
        }
    }

    float spacing_std = std::sqrt(spacing_variance / spacing_count);
    if (spacing_std > avg_spacing * 0.2f) return false;

    result.found = true;
    result.confidence = 1.0 - spacing_std / avg_spacing;

    return true;
}

// ==================== 标定点排序和编号 ====================

/**
 * @brief 标定角点排序（按行优先）
 */
inline void sort_calibration_points(std::vector<CalibCorner>& corners, int rows, int cols) {
    if (corners.size() != rows * cols) return;

    // 计算边界
    float min_x = corners[0].x, max_x = corners[0].x;
    float min_y = corners[0].y, max_y = corners[0].y;

    for (const auto& c : corners) {
        min_x = std::min(min_x, c.x);
        max_x = std::max(max_x, c.x);
        min_y = std::min(min_y, c.y);
        max_y = std::max(max_y, c.y);
    }

    // 计算单元格大小
    float cell_w = (max_x - min_x) / (cols - 1);
    float cell_h = (max_y - min_y) / (rows - 1);

    // 分配行列
    for (auto& c : corners) {
        int col = static_cast<int>((c.x - min_x) / cell_w + 0.5f);
        int row = static_cast<int>((c.y - min_y) / cell_h + 0.5f);
        col = std::max(0, std::min(cols - 1, col));
        row = std::max(0, std::min(rows - 1, row));
        c.col = col;
        c.row = row;
        c.id = row * cols + col;
    }

    // 按ID排序
    std::sort(corners.begin(), corners.end(),
              [](const CalibCorner& a, const CalibCorner& b) { return a.id < b.id; });
}

/**
 * @brief 标定圆点排序（按行优先）
 */
inline void sort_circle_points(std::vector<CirclePoint>& circles, int rows, int cols) {
    if (circles.size() != rows * cols) return;

    float min_x = circles[0].center_x, max_x = circles[0].center_x;
    float min_y = circles[0].center_y, max_y = circles[0].center_y;

    for (const auto& c : circles) {
        min_x = std::min(min_x, c.center_x);
        max_x = std::max(max_x, c.center_x);
        min_y = std::min(min_y, c.center_y);
        max_y = std::max(max_y, c.center_y);
    }

    float cell_w = (max_x - min_x) / (cols - 1);
    float cell_h = (max_y - min_y) / (rows - 1);

    for (auto& c : circles) {
        int col = static_cast<int>((c.center_x - min_x) / cell_w + 0.5f);
        int row = static_cast<int>((c.center_y - min_y) / cell_h + 0.5f);
        col = std::max(0, std::min(cols - 1, col));
        row = std::max(0, std::min(rows - 1, row));
        c.col = col;
        c.row = row;
        c.id = row * cols + col;
    }

    std::sort(circles.begin(), circles.end(),
              [](const CirclePoint& a, const CirclePoint& b) { return a.id < b.id; });
}

// ==================== 质量评估 ====================

/**
 * @brief 计算标定质量
 */
inline QualityResult compute_quality(const PatternResult& pattern,
                                     int expected_rows, int expected_cols) {
    QualityResult result;
    result.valid = false;
    result.total_points = expected_rows * expected_cols;

    if (!pattern.found) {
        result.valid_points = 0;
        result.coverage = 0;
        result.quality_level = "Poor";
        return result;
    }

    int detected_points = 0;
    if (!pattern.corners.empty()) {
        detected_points = static_cast<int>(pattern.corners.size());
        result.valid_points = detected_points;
    } else if (!pattern.circles.empty()) {
        detected_points = static_cast<int>(pattern.circles.size());
        result.valid_points = detected_points;
    }

    result.coverage = static_cast<float>(detected_points) / result.total_points;

    if (result.coverage < 0.5f) {
        result.quality_level = "Poor";
        return result;
    }

    // 计算间距一致性
    float avg_spacing = 0;
    float spacing_variance = 0;
    int spacing_count = 0;

    if (!pattern.corners.empty()) {
        for (int r = 0; r < pattern.rows; ++r) {
            for (int c = 0; c < pattern.cols - 1; ++c) {
                float dx = pattern.corners[r * pattern.cols + c + 1].x -
                          pattern.corners[r * pattern.cols + c].x;
                float dy = pattern.corners[r * pattern.cols + c + 1].y -
                          pattern.corners[r * pattern.cols + c].y;
                avg_spacing += std::sqrt(dx * dx + dy * dy);
                spacing_count++;
            }
        }
    }

    if (spacing_count > 0) {
        avg_spacing /= spacing_count;

        for (int r = 0; r < pattern.rows; ++r) {
            for (int c = 0; c < pattern.cols - 1; ++c) {
                float dx = pattern.corners[r * pattern.cols + c + 1].x -
                          pattern.corners[r * pattern.cols + c].x;
                float dy = pattern.corners[r * pattern.cols + c + 1].y -
                          pattern.corners[r * pattern.cols + c].y;
                float spacing = std::sqrt(dx * dx + dy * dy);
                spacing_variance += (spacing - avg_spacing) * (spacing - avg_spacing);
            }
        }

        result.uniformity = 1.0f - static_cast<float>(std::sqrt(spacing_variance / spacing_count) / avg_spacing);
    }

    // 综合评估
    float score = result.coverage * 0.5f + result.uniformity * 0.3f +
                  static_cast<float>(pattern.confidence) * 0.2f;

    if (score >= 0.9f) result.quality_level = "Excellent";
    else if (score >= 0.75f) result.quality_level = "Good";
    else if (score >= 0.5f) result.quality_level = "Average";
    else result.quality_level = "Poor";

    result.valid = true;

    return result;
}

// ==================== 绘制函数 ====================

/**
 * @brief 绘制标定角点
 */
inline void draw_calibration_corners(ImageData& output, const std::vector<CalibCorner>& corners,
                                      uint8_t r = 255, uint8_t g = 0, uint8_t b = 0) {
    if (output.channels != 3) return;

    for (const auto& c : corners) {
        int x = static_cast<int>(c.x);
        int y = static_cast<int>(c.y);

        // 绘制十字标记
        for (int dy = -5; dy <= 5; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < static_cast<int>(output.width) &&
                    py >= 0 && py < static_cast<int>(output.height)) {
                    size_t idx = (py * output.width + px) * 3;
                    output.data[idx] = b;
                    output.data[idx + 1] = g;
                    output.data[idx + 2] = r;
                }
            }
        }

        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < static_cast<int>(output.width) &&
                    py >= 0 && py < static_cast<int>(output.height)) {
                    size_t idx = (py * output.width + px) * 3;
                    output.data[idx] = b;
                    output.data[idx + 1] = g;
                    output.data[idx + 2] = r;
                }
            }
        }
    }
}

/**
 * @brief 绘制标定圆点
 */
inline void draw_calibration_circles(ImageData& output, const std::vector<CirclePoint>& circles,
                                      uint8_t r = 0, uint8_t g = 255, uint8_t b = 0) {
    if (output.channels != 3) return;

    for (const auto& c : circles) {
        int cx = static_cast<int>(c.center_x);
        int cy = static_cast<int>(c.center_y);
        int radius = static_cast<int>(c.radius + 2);

        // 绘制圆
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx * dx + dy * dy <= radius * radius) {
                    int px = cx + dx;
                    int py = cy + dy;
                    if (px >= 0 && px < static_cast<int>(output.width) &&
                        py >= 0 && py < static_cast<int>(output.height)) {
                        size_t idx = (py * output.width + px) * 3;
                        output.data[idx] = b;
                        output.data[idx + 1] = g;
                        output.data[idx + 2] = r;
                    }
                }
            }
        }
    }
}

/**
 * @brief 绘制编号
 */
inline void draw_point_numbers(ImageData& output, const std::vector<CalibCorner>& corners) {
    // 简化实现：只绘制小标记
    if (output.channels != 3) return;

    uint8_t colors[][3] = {
        {255, 255, 255},  // 白
        {255, 0, 0},      // 红
        {0, 255, 0},      // 绿
        {0, 0, 255},      // 蓝
        {255, 255, 0}     // 黄
    };

    for (const auto& c : corners) {
        if (c.id < 0) continue;

        int x = static_cast<int>(c.x) + 8;
        int y = static_cast<int>(c.y) - 8;

        uint8_t color_idx = c.id % 5;

        // 绘制小方块表示编号
        for (int dy = 0; dy < 5; ++dy) {
            for (int dx = 0; dx < 5; ++dx) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < static_cast<int>(output.width) &&
                    py >= 0 && py < static_cast<int>(output.height)) {
                    size_t idx = (py * output.width + px) * 3;
                    output.data[idx] = colors[color_idx][0];
                    output.data[idx + 1] = colors[color_idx][1];
                    output.data[idx + 2] = colors[color_idx][2];
                }
            }
        }
    }
}

} // namespace calibration_pattern_utils

// ==================== 标定板检测节点 ====================

/**
 * @brief 棋盘格检测节点
 */
class CheckerboardDetectNode : public INode {
public:
    CheckerboardDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 圆点阵列检测节点
 */
class CircleGridDetectNode : public INode {
public:
    CircleGridDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 标定点提取节点
 */
class CalibrationPointsExtractNode : public INode {
public:
    CalibrationPointsExtractNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 标定板匹配节点
 */
class CalibrationPatternMatchNode : public INode {
public:
    CalibrationPatternMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 标定质量检查节点
 */
class CalibrationQualityCheckNode : public INode {
public:
    CalibrationQualityCheckNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf