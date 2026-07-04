/**
 * @file segmentation.cpp
 * @brief 图像分割算法实现（纯C++实现，不依赖OpenCV）
 */

#include "ovf/algorithm/segmentation.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <queue>
#include <random>
#include <limits>
#include <map>
#include <set>

namespace ovf {
namespace algorithm {

namespace seg_utils {

// ==================== 基础滤波函数 ====================

void gaussian_blur(const uint8_t* src, uint8_t* dst, int width, int height, int channels, int ksize) {
    if (!src || !dst || ksize < 1) return;

    // 简化版：使用均值滤波代替高斯
    int half = ksize / 2;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                int sum = 0;
                int count = 0;

                for (int ky = -half; ky <= half; ++ky) {
                    for (int kx = -half; kx <= half; ++kx) {
                        int ny = std::clamp(y + ky, 0, height - 1);
                        int nx = std::clamp(x + kx, 0, width - 1);
                        sum += src[(ny * width + nx) * channels + c];
                        count++;
                    }
                }

                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(sum / count);
            }
        }
    }
}

void median_blur(const uint8_t* src, uint8_t* dst, int width, int height, int channels, int ksize) {
    if (!src || !dst || ksize < 1) return;

    int half = ksize / 2;
    std::vector<uint8_t> window(ksize * ksize);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                int idx = 0;
                for (int ky = -half; ky <= half; ++ky) {
                    for (int kx = -half; kx <= half; ++kx) {
                        int ny = std::clamp(y + ky, 0, height - 1);
                        int nx = std::clamp(x + kx, 0, width - 1);
                        window[idx++] = src[(ny * width + nx) * channels + c];
                    }
                }

                std::sort(window.begin(), window.end());
                dst[(y * width + x) * channels + c] = window[idx / 2];
            }
        }
    }
}

void compute_gradient(const uint8_t* src, uint8_t* grad_x, uint8_t* grad_y,
                      uint8_t* magnitude, int width, int height) {
    if (!src) return;

    // Sobel算子
    const int gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    const int gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int sum_x = 0, sum_y = 0;

            for (int ky = -1; ky <= 1; ++ky) {
                for (int kx = -1; kx <= 1; ++kx) {
                    int val = src[(y + ky) * width + (x + kx)];
                    sum_x += val * gx[ky + 1][kx + 1];
                    sum_y += val * gy[ky + 1][kx + 1];
                }
            }

            if (grad_x) grad_x[y * width + x] = static_cast<uint8_t>(std::clamp(std::abs(sum_x), 0, 255));
            if (grad_y) grad_y[y * width + x] = static_cast<uint8_t>(std::clamp(std::abs(sum_y), 0, 255));
            if (magnitude) {
                int mag = static_cast<int>(std::sqrt(sum_x * sum_x + sum_y * sum_y));
                magnitude[y * width + x] = static_cast<uint8_t>(std::clamp(mag, 0, 255));
            }
        }
    }
}

// ==================== 阈值分割 ====================

void threshold_segment(const uint8_t* src, uint8_t* dst, int width, int height,
                       int thresh, int max_val, ThresholdType type) {
    if (!src || !dst) return;

    for (int i = 0; i < width * height; ++i) {
        uint8_t val = src[i];
        switch (type) {
            case ThresholdType::Binary:
                dst[i] = (val > thresh) ? max_val : 0;
                break;
            case ThresholdType::BinaryInv:
                dst[i] = (val > thresh) ? 0 : max_val;
                break;
            case ThresholdType::Trunc:
                dst[i] = (val > thresh) ? thresh : val;
                break;
            case ThresholdType::ToZero:
                dst[i] = (val > thresh) ? val : 0;
                break;
            case ThresholdType::ToZeroInv:
                dst[i] = (val > thresh) ? 0 : val;
                break;
        }
    }
}

void adaptive_threshold(const uint8_t* src, uint8_t* dst, int width, int height,
                        int block_size, int c, AdaptiveMethod method, ThresholdType type) {
    if (!src || !dst || block_size < 3) return;

    int half = block_size / 2;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 计算局部均值
            int sum = 0;
            int count = 0;

            for (int ky = -half; ky <= half; ++ky) {
                for (int kx = -half; kx <= half; ++kx) {
                    int ny = std::clamp(y + ky, 0, height - 1);
                    int nx = std::clamp(x + kx, 0, width - 1);
                    sum += src[ny * width + nx];
                    count++;
                }
            }

            int local_thresh = sum / count - c;
            uint8_t val = src[y * width + x];

            switch (type) {
                case ThresholdType::Binary:
                    dst[y * width + x] = (val > local_thresh) ? 255 : 0;
                    break;
                case ThresholdType::BinaryInv:
                    dst[y * width + x] = (val > local_thresh) ? 0 : 255;
                    break;
                default:
                    dst[y * width + x] = (val > local_thresh) ? 255 : 0;
                    break;
            }
        }
    }
}

// ==================== 颜色分割 ====================

void color_segment_rgb(const uint8_t* src, uint8_t* mask, int width, int height,
                       uint8_t r, uint8_t g, uint8_t b, int tolerance) {
    if (!src || !mask) return;

    for (int i = 0; i < width * height; ++i) {
        uint8_t pr = src[i * 3];
        uint8_t pg = src[i * 3 + 1];
        uint8_t pb = src[i * 3 + 2];

        int dr = std::abs(pr - r);
        int dg = std::abs(pg - g);
        int db = std::abs(pb - b);

        mask[i] = (dr <= tolerance && dg <= tolerance && db <= tolerance) ? 255 : 0;
    }
}

// ==================== 边缘分割 ====================

void edge_segment(const uint8_t* src, uint8_t* dst, int width, int height,
                  EdgeSegmentMethod method, int low_thresh, int high_thresh) {
    if (!src || !dst) return;

    std::vector<uint8_t> grad(width * height);
    std::fill(dst, dst + width * height, 0);

    switch (method) {
        case EdgeSegmentMethod::Sobel:
        case EdgeSegmentMethod::Gradient:
            compute_gradient(src, nullptr, nullptr, grad.data(), width, height);
            for (int i = 0; i < width * height; ++i) {
                dst[i] = (grad[i] > low_thresh) ? 255 : 0;
            }
            break;

        case EdgeSegmentMethod::Canny:
            // 简化版Canny
            compute_gradient(src, nullptr, nullptr, grad.data(), width, height);
            for (int y = 1; y < height - 1; ++y) {
                for (int x = 1; x < width - 1; ++x) {
                    int idx = y * width + x;
                    if (grad[idx] >= high_thresh) {
                        dst[idx] = 255;
                    } else if (grad[idx] >= low_thresh) {
                        // 检查邻居是否有强边缘
                        bool has_strong = false;
                        for (int dy = -1; dy <= 1 && !has_strong; ++dy) {
                            for (int dx = -1; dx <= 1 && !has_strong; ++dx) {
                                if (grad[(y + dy) * width + (x + dx)] >= high_thresh) {
                                    has_strong = true;
                                }
                            }
                        }
                        dst[idx] = has_strong ? 255 : 0;
                    }
                }
            }
            break;

        case EdgeSegmentMethod::Laplacian:
            for (int y = 1; y < height - 1; ++y) {
                for (int x = 1; x < width - 1; ++x) {
                    int lap = -4 * src[y * width + x];
                    lap += src[(y - 1) * width + x];
                    lap += src[(y + 1) * width + x];
                    lap += src[y * width + (x - 1)];
                    lap += src[y * width + (x + 1)];
                    dst[y * width + x] = (std::abs(lap) > low_thresh) ? 255 : 0;
                }
            }
            break;
    }
}

// ==================== 连通区域标记 ====================

int connected_components(const uint8_t* binary, int32_t* labels, int width, int height, bool use_8connectivity) {
    if (!binary || !labels) return 0;

    std::fill(labels, labels + width * height, 0);
    int current_label = 0;

    // 使用洪水填充算法
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (binary[y * width + x] > 0 && labels[y * width + x] == 0) {
                current_label++;
                std::queue<std::pair<int, int>> q;
                q.push({x, y});
                labels[y * width + x] = current_label;

                while (!q.empty()) {
                    auto [cx, cy] = q.front();
                    q.pop();

                    // 4邻域或8邻域
                    static const int dx4[] = {-1, 0, 0, 1};
                    static const int dy4[] = {0, -1, 1, 0};
                    static const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
                    static const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
                    const int* dx = use_8connectivity ? dx8 : dx4;
                    const int* dy = use_8connectivity ? dy8 : dy4;
                    int n_count = use_8connectivity ? 8 : 4;

                    for (int i = 0; i < n_count; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            if (binary[ny * width + nx] > 0 && labels[ny * width + nx] == 0) {
                                labels[ny * width + nx] = current_label;
                                q.push({nx, ny});
                            }
                        }
                    }
                }
            }
        }
    }

    return current_label;
}

// ==================== 区域生长 ====================

void region_growing(const uint8_t* src, uint8_t* dst, int width, int height,
                    int seed_x, int seed_y, int threshold, bool use_8connectivity) {
    if (!src || !dst) return;

    std::fill(dst, dst + width * height, 0);

    if (seed_x < 0 || seed_x >= width || seed_y < 0 || seed_y >= height) return;

    uint8_t seed_val = src[seed_y * width + seed_x];
    std::queue<std::pair<int, int>> q;
    q.push({seed_x, seed_y});
    dst[seed_y * width + seed_x] = 255;

    const int dx4[] = {-1, 0, 0, 1};
    const int dy4[] = {0, -1, 1, 0};
    const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();

        int n_count = use_8connectivity ? 8 : 4;
        const int* dx = use_8connectivity ? dx8 : dx4;
        const int* dy = use_8connectivity ? dy8 : dy4;

        for (int i = 0; i < n_count; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                if (dst[ny * width + nx] == 0) {
                    int diff = std::abs(static_cast<int>(src[ny * width + nx]) - seed_val);
                    if (diff <= threshold) {
                        dst[ny * width + nx] = 255;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }
}

// ==================== 区域分裂合并 ====================

void region_split_merge(const uint8_t* src, uint8_t* dst, int width, int height,
                        int min_size, int threshold) {
    if (!src || !dst) return;

    // 初始化：将整个图像标记为区域1
    std::vector<int32_t> labels(width * height, 1);

    // 递归分裂
    std::function<void(int, int, int, int, int)> split_region = [&](int x1, int y1, int x2, int y2, int label) {
        int w = x2 - x1;
        int h = y2 - y1;

        if (w <= min_size || h <= min_size) return;

        // 计算当前区域的均值和方差
        int sum = 0, count = 0;
        int min_val = 255, max_val = 0;

        for (int y = y1; y < y2; ++y) {
            for (int x = x1; x < x2; ++x) {
                uint8_t val = src[y * width + x];
                sum += val;
                count++;
                min_val = std::min(min_val, (int)val);
                max_val = std::max(max_val, (int)val);
            }
        }

        int range = max_val - min_val;

        if (range > threshold) {
            // 分裂为4个子区域
            int mx = (x1 + x2) / 2;
            int my = (y1 + y2) / 2;

            split_region(x1, y1, mx, my, label * 4 + 1);
            split_region(mx, y1, x2, my, label * 4 + 2);
            split_region(x1, my, mx, y2, label * 4 + 3);
            split_region(mx, my, x2, y2, label * 4 + 4);

            // 更新标签
            for (int y = y1; y < y2; ++y) {
                for (int x = x1; x < x2; ++x) {
                    if (x < mx && y < my) labels[y * width + x] = label * 4 + 1;
                    else if (x >= mx && y < my) labels[y * width + x] = label * 4 + 2;
                    else if (x < mx && y >= my) labels[y * width + x] = label * 4 + 3;
                    else labels[y * width + x] = label * 4 + 4;
                }
            }
        }
    };

    split_region(0, 0, width, height, 1);

    // 计算每个区域的均值颜色
    std::map<int32_t, int> region_mean;
    std::map<int32_t, int> region_count;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int32_t lbl = labels[y * width + x];
            region_mean[lbl] += src[y * width + x];
            region_count[lbl]++;
        }
    }

    for (auto& [lbl, sum] : region_mean) {
        region_mean[lbl] = sum / region_count[lbl];
    }

    // 合并相邻相似区域
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int32_t lbl = labels[y * width + x];
            dst[y * width + x] = static_cast<uint8_t>(region_mean[lbl]);
        }
    }
}

// ==================== K均值聚类 ====================

void kmeans_segment(const uint8_t* src, uint8_t* dst, int width, int height, int channels,
                    int k, int max_iterations) {
    if (!src || !dst || k < 1) return;

    int total_pixels = width * height;

    // 初始化聚类中心（随机选择）
    std::vector<std::vector<float>> centers(k, std::vector<float>(channels));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, total_pixels - 1);

    for (int i = 0; i < k; ++i) {
        int idx = dis(gen);
        for (int c = 0; c < channels; ++c) {
            centers[i][c] = src[idx * channels + c];
        }
    }

    std::vector<int> labels(total_pixels);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 分配像素到最近的聚类中心
        for (int p = 0; p < total_pixels; ++p) {
            float min_dist = std::numeric_limits<float>::max();
            int best_center = 0;

            for (int i = 0; i < k; ++i) {
                float dist = 0;
                for (int c = 0; c < channels; ++c) {
                    float diff = src[p * channels + c] - centers[i][c];
                    dist += diff * diff;
                }

                if (dist < min_dist) {
                    min_dist = dist;
                    best_center = i;
                }
            }

            labels[p] = best_center;
        }

        // 更新聚类中心
        std::vector<std::vector<float>> new_centers(k, std::vector<float>(channels, 0));
        std::vector<int> counts(k, 0);

        for (int p = 0; p < total_pixels; ++p) {
            int lbl = labels[p];
            for (int c = 0; c < channels; ++c) {
                new_centers[lbl][c] += src[p * channels + c];
            }
            counts[lbl]++;
        }

        for (int i = 0; i < k; ++i) {
            if (counts[i] > 0) {
                for (int c = 0; c < channels; ++c) {
                    centers[i][c] = new_centers[i][c] / counts[i];
                }
            }
        }
    }

    // 生成输出图像
    for (int p = 0; p < total_pixels; ++p) {
        int lbl = labels[p];
        for (int c = 0; c < channels; ++c) {
            dst[p * channels + c] = static_cast<uint8_t>(centers[lbl][c]);
        }
    }
}

// ==================== 分水岭分割 ====================

void watershed_segment(const uint8_t* src, int32_t* markers, int32_t* result, int width, int height) {
    if (!src || !markers || !result) return;

    // 复制初始标记
    std::copy(markers, markers + width * height, result);

    // 计算梯度作为分割依据
    std::vector<uint8_t> gradient(width * height);
    compute_gradient(src, nullptr, nullptr, gradient.data(), width, height);

    // 按梯度值从小到大处理像素
    std::vector<std::pair<int, int>> pixels;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            pixels.push_back({gradient[y * width + x], y * width + x});
        }
    }
    std::sort(pixels.begin(), pixels.end());

    // 分水岭算法
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (auto& [grad, idx] : pixels) {
        if (result[idx] != 0) continue; // 已经标记的跳过

        int x = idx % width;
        int y = idx / width;

        // 检查邻居的标记
        std::set<int32_t> neighbor_labels;
        bool has_conflict = false;

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int32_t nl = result[ny * width + nx];
                if (nl > 0) {
                    neighbor_labels.insert(nl);
                }
            }
        }

        if (neighbor_labels.size() == 1) {
            // 所有邻居属于同一区域，标记为该区域
            result[idx] = *neighbor_labels.begin();
        } else if (neighbor_labels.size() > 1) {
            // 邻居属于不同区域，标记为边界（-1）
            result[idx] = -1;
        }
    }
}

// ==================== 图割分割（简化） ====================

void graphcut_segment(const uint8_t* src, uint8_t* mask, int width, int height,
                      int seed_x1, int seed_y1, int seed_x2, int seed_y2) {
    if (!src || !mask) return;

    std::fill(mask, mask + width * height, 0);

    // 简化版：使用区域生长模拟图割效果
    // 前景种子点
    uint8_t fg_val = src[seed_y1 * width + seed_x1];
    uint8_t bg_val = src[seed_y2 * width + seed_x2];

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t val = src[y * width + x];
            int fg_dist = std::abs(val - fg_val);
            int bg_dist = std::abs(val - bg_val);

            // 根据距离判断前景或背景
            mask[y * width + x] = (fg_dist < bg_dist) ? 255 : 0;
        }
    }

    // 使用连通性约束进行优化
    // 从前景种子开始生长
    std::queue<std::pair<int, int>> q;
    std::vector<bool> visited(width * height, false);

    q.push({seed_x1, seed_y1});
    visited[seed_y1 * width + seed_x1] = true;

    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();

        mask[cy * width + cx] = 255;

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                int nx = cx + dx;
                int ny = cy + dy;

                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int idx = ny * width + nx;
                    if (!visited[idx] && mask[idx] == 255) {
                        visited[idx] = true;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }
}

// ==================== GrabCut分割（简化） ====================

void grabcut_segment(const uint8_t* src, uint8_t* mask, int width, int height, int channels,
                     int rect_x, int rect_y, int rect_w, int rect_h, int iterations) {
    if (!src || !mask) return;

    // 初始化掩码：矩形内为可能前景(3)，矩形外为确定背景(0)
    std::fill(mask, mask + width * height, 0);

    for (int y = rect_y; y < rect_y + rect_h && y < height; ++y) {
        for (int x = rect_x; x < rect_x + rect_w && x < width; ++x) {
            mask[y * width + x] = 3; // 可能前景
        }
    }

    // 简化版：使用迭代聚类方法
    // 计算背景和前景的颜色模型

    for (int iter = 0; iter < iterations; ++iter) {
        // 计算背景均值（掩码为0的区域）
        std::vector<float> bg_mean(channels, 0);
        int bg_count = 0;

        // 计算前景均值（掩码>0的区域）
        std::vector<float> fg_mean(channels, 0);
        int fg_count = 0;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                if (mask[idx] > 0) {
                    for (int c = 0; c < channels; ++c) {
                        fg_mean[c] += src[idx * channels + c];
                    }
                    fg_count++;
                } else {
                    for (int c = 0; c < channels; ++c) {
                        bg_mean[c] += src[idx * channels + c];
                    }
                    bg_count++;
                }
            }
        }

        if (fg_count > 0) {
            for (int c = 0; c < channels; ++c) fg_mean[c] /= fg_count;
        }
        if (bg_count > 0) {
            for (int c = 0; c < channels; ++c) bg_mean[c] /= bg_count;
        }

        // 更新掩码
        for (int y = rect_y; y < rect_y + rect_h && y < height; ++y) {
            for (int x = rect_x; x < rect_x + rect_w && x < width; ++x) {
                int idx = y * width + x;

                float fg_dist = 0, bg_dist = 0;
                for (int c = 0; c < channels; ++c) {
                    fg_dist += std::pow(src[idx * channels + c] - fg_mean[c], 2);
                    bg_dist += std::pow(src[idx * channels + c] - bg_mean[c], 2);
                }

                mask[idx] = (fg_dist < bg_dist * 1.5) ? 255 : 0;
            }
        }
    }
}

// ==================== MeanShift分割 ====================

void meanshift_segment(const uint8_t* src, uint8_t* dst, int width, int height, int channels,
                       float spatial_radius, float color_radius, int min_density) {
    if (!src || !dst) return;

    int total_pixels = width * height;

    // MeanShift聚类
    std::vector<std::vector<float>> modes; // 存储模式点
    std::vector<int> labels(total_pixels, -1);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;

            // 初始点
            std::vector<float> point(channels + 2);
            point[0] = x; // 空间坐标
            point[1] = y;
            for (int c = 0; c < channels; ++c) {
                point[c + 2] = src[idx * channels + c];
            }

            // MeanShift迭代
            for (int iter = 0; iter < 20; ++iter) {
                std::vector<float> mean(channels + 2, 0);
                int count = 0;

                // 在空间和颜色范围内收集邻居
                for (int ny = 0; ny < height; ++ny) {
                    for (int nx = 0; nx < width; ++nx) {
                        float dx = nx - point[0];
                        float dy = ny - point[1];
                        float spatial_dist = std::sqrt(dx * dx + dy * dy);

                        if (spatial_dist <= spatial_radius) {
                            int nidx = ny * width + nx;
                            float color_dist = 0;
                            for (int c = 0; c < channels; ++c) {
                                color_dist += std::pow(src[nidx * channels + c] - point[c + 2], 2);
                            }
                            color_dist = std::sqrt(color_dist);

                            if (color_dist <= color_radius) {
                                mean[0] += nx;
                                mean[1] += ny;
                                for (int c = 0; c < channels; ++c) {
                                    mean[c + 2] += src[nidx * channels + c];
                                }
                                count++;
                            }
                        }
                    }
                }

                if (count > 0) {
                    for (int i = 0; i < channels + 2; ++i) {
                        mean[i] /= count;
                    }
                }

                // 检查收敛
                float shift = 0;
                for (int i = 0; i < channels + 2; ++i) {
                    shift += std::pow(mean[i] - point[i], 2);
                }

                if (shift < 1.0f) break;

                point = mean;
            }

            // 查找或创建模式
            int found_mode = -1;
            for (int m = 0; m < modes.size(); ++m) {
                float dist = 0;
                for (int i = 0; i < channels + 2; ++i) {
                    dist += std::pow(modes[m][i] - point[i], 2);
                }
                if (std::sqrt(dist) < color_radius) {
                    found_mode = m;
                    break;
                }
            }

            if (found_mode >= 0) {
                labels[idx] = found_mode;
            } else {
                labels[idx] = modes.size();
                modes.push_back(point);
            }
        }
    }

    // 应用模式颜色到输出
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            int lbl = labels[idx];
            for (int c = 0; c < channels; ++c) {
                dst[idx * channels + c] = static_cast<uint8_t>(modes[lbl][c + 2]);
            }
        }
    }
}

// ==================== 超像素分割 ====================

void superpixel_segment(const uint8_t* src, int32_t* labels, int width, int height, int channels,
                        int num_superpixels, int compactness, int iterations) {
    if (!src || !labels) return;

    std::fill(labels, labels + width * height, 0);

    // 计算网格大小
    int grid_size = static_cast<int>(std::sqrt(width * height / num_superpixels));
    if (grid_size < 1) grid_size = 1;

    // 初始化聚类中心
    std::vector<std::vector<float>> centers(num_superpixels, std::vector<float>(channels + 2));

    int center_idx = 0;
    for (int y = grid_size / 2; y < height && center_idx < num_superpixels; y += grid_size) {
        for (int x = grid_size / 2; x < width && center_idx < num_superpixels; x += grid_size) {
            centers[center_idx][0] = x;
            centers[center_idx][1] = y;
            for (int c = 0; c < channels; ++c) {
                centers[center_idx][c + 2] = src[(y * width + x) * channels + c];
            }
            center_idx++;
        }
    }

    // 实际聚类数量
    int actual_k = center_idx;

    // 距离数组
    std::vector<float> distances(width * height, std::numeric_limits<float>::max());

    for (int iter = 0; iter < iterations; ++iter) {
        std::fill(distances.begin(), distances.end(), std::numeric_limits<float>::max());
        std::fill(labels, labels + width * height, -1);

        // 分配像素到最近的中心
        for (int k = 0; k < actual_k; ++k) {
            int cx = static_cast<int>(centers[k][0]);
            int cy = static_cast<int>(centers[k][1]);

            // 搜索范围：2S x 2S
            int x1 = std::max(0, cx - grid_size * 2);
            int x2 = std::min(width, cx + grid_size * 2);
            int y1 = std::max(0, cy - grid_size * 2);
            int y2 = std::min(height, cy + grid_size * 2);

            for (int y = y1; y < y2; ++y) {
                for (int x = x1; x < x2; ++x) {
                    int idx = y * width + x;

                    // 计算距离
                    float spatial_dist = std::pow(x - centers[k][0], 2) + std::pow(y - centers[k][1], 2);
                    float color_dist = 0;
                    for (int c = 0; c < channels; ++c) {
                        color_dist += std::pow(src[idx * channels + c] - centers[k][c + 2], 2);
                    }

                    // 综合距离
                    float D = color_dist + (spatial_dist * compactness * compactness) / (grid_size * grid_size);

                    if (D < distances[idx]) {
                        distances[idx] = D;
                        labels[idx] = k;
                    }
                }
            }
        }

        // 更新中心
        std::vector<std::vector<float>> new_centers(actual_k, std::vector<float>(channels + 2, 0));
        std::vector<int> counts(actual_k, 0);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int k = labels[y * width + x];
                if (k >= 0) {
                    new_centers[k][0] += x;
                    new_centers[k][1] += y;
                    for (int c = 0; c < channels; ++c) {
                        new_centers[k][c + 2] += src[(y * width + x) * channels + c];
                    }
                    counts[k]++;
                }
            }
        }

        for (int k = 0; k < actual_k; ++k) {
            if (counts[k] > 0) {
                for (int i = 0; i < channels + 2; ++i) {
                    centers[k][i] = new_centers[k][i] / counts[k];
                }
            }
        }
    }

    // 处理孤立像素
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (labels[y * width + x] < 0) {
                // 分配到最近的中心
                float min_dist = std::numeric_limits<float>::max();
                int best_k = 0;

                for (int k = 0; k < actual_k; ++k) {
                    float dist = std::pow(x - centers[k][0], 2) + std::pow(y - centers[k][1], 2);
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_k = k;
                    }
                }

                labels[y * width + x] = best_k;
            }
        }
    }
}

void flood_fill(const uint8_t* src, uint8_t* dst, int width, int height,
                int seed_x, int seed_y, uint8_t fill_value, int tolerance) {
    if (!src || !dst || seed_x < 0 || seed_x >= width || seed_y < 0 || seed_y >= height) return;

    uint8_t target_value = src[seed_y * width + seed_x];
    std::queue<std::pair<int, int>> q;
    q.push({seed_x, seed_y});
    dst[seed_y * width + seed_x] = fill_value;

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        const int dx[] = {-1, 0, 0, 1};
        const int dy[] = {0, -1, 1, 0};

        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                if (dst[ny * width + nx] != fill_value) {
                    int diff = std::abs(static_cast<int>(src[ny * width + nx]) - target_value);
                    if (diff <= tolerance) {
                        dst[ny * width + nx] = fill_value;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }
}

void region_statistics(const uint8_t* src, const int32_t* labels, int width, int height,
                       int label, int& area, int& min_val, int& max_val, float& mean_val) {
    area = 0;
    min_val = 255;
    max_val = 0;
    int sum = 0;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (labels[y * width + x] == label) {
                uint8_t val = src[y * width + x];
                area++;
                sum += val;
                min_val = std::min(min_val, (int)val);
                max_val = std::max(max_val, (int)val);
            }
        }
    }

    mean_val = (area > 0) ? sum / area : 0;
}

} // namespace seg_utils

// ==================== ThresholdSegmentNode ====================

ThresholdSegmentNode::ThresholdSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ThresholdSegmentNode::make_info() {
    NodeInfo info;
    info.id = "ThresholdSegment";
    info.name = "阈值分割";
    info.category = "图像分割";
    info.description = "固定阈值或自适应阈值分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("binary", "二值图像", DataType::Image));
    info.outputs.push_back(DataPort("threshold_value", "使用的阈值", DataType::Number));

    info.params.push_back(ParamDef("threshold_type", "阈值类型", DataType::String, Data("Binary")));
    info.params.push_back(ParamDef("threshold_value", "阈值值", DataType::Number, Data(128)));
    info.params.push_back(ParamDef("max_value", "最大值", DataType::Number, Data(255)));
    info.params.push_back(ParamDef("auto_threshold", "自动阈值(Otsu)", DataType::Boolean, Data(false)));

    return info;
}

int ThresholdSegmentNode::compute_otsu_threshold(const uint8_t* data, int size) {
    // 计算直方图
    std::vector<int> hist(256, 0);
    for (int i = 0; i < size; ++i) {
        hist[data[i]]++;
    }

    // Otsu算法
    float sum = 0;
    for (int i = 0; i < 256; ++i) sum += i * hist[i];

    int total = size;
    float sumB = 0;
    int wB = 0;
    int wF = 0;

    float max_var = 0;
    int threshold = 0;

    for (int t = 0; t < 256; ++t) {
        wB += hist[t];
        if (wB == 0) continue;

        wF = total - wB;
        if (wF == 0) break;

        sumB += t * hist[t];

        float mB = sumB / wB;
        float mF = (sum - sumB) / wF;

        float var = wB * wF * (mB - mF) * (mB - mF);

        if (var > max_var) {
            max_var = var;
            threshold = t;
        }
    }

    return threshold;
}

Result<void> ThresholdSegmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (input.channels == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    String type_str = get_param("threshold_type", Data("Binary")).as_string();
    ThresholdType type = ThresholdType::Binary;
    if (type_str == "BinaryInv") type = ThresholdType::BinaryInv;
    else if (type_str == "Trunc") type = ThresholdType::Trunc;
    else if (type_str == "ToZero") type = ThresholdType::ToZero;
    else if (type_str == "ToZeroInv") type = ThresholdType::ToZeroInv;

    int thresh = static_cast<int>(get_param("threshold_value", Data(128)).as_int());
    int max_val = static_cast<int>(get_param("max_value", Data(255)).as_int());
    bool auto_thresh = get_param("auto_threshold", Data(false)).as_bool();

    if (auto_thresh) {
        thresh = compute_otsu_threshold(gray.data(), w * h);
    }

    // 阈值分割
    std::vector<uint8_t> binary(w * h);
    seg_utils::threshold_segment(gray.data(), binary.data(), w, h, thresh, max_val, type);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = binary;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("binary", Data(output));
    set_output("threshold_value", Data(thresh));

    OVF_INFO() << "ThresholdSegment completed: threshold=" << thresh;

    return Result<void>::success();
}

// ==================== AdaptiveThresholdNode ====================

AdaptiveThresholdNode::AdaptiveThresholdNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo AdaptiveThresholdNode::make_info() {
    NodeInfo info;
    info.id = "AdaptiveThreshold";
    info.name = "自适应阈值分割";
    info.category = "图像分割";
    info.description = "局部自适应阈值分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("binary", "二值图像", DataType::Image));

    info.params.push_back(ParamDef("method", "自适应方法", DataType::String, Data("Mean")));
    info.params.push_back(ParamDef("block_size", "块大小", DataType::Number, Data(31)));
    info.params.push_back(ParamDef("c", "常数偏移", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("threshold_type", "阈值类型", DataType::String, Data("Binary")));

    return info;
}

Result<void> AdaptiveThresholdNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (input.channels == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    String method_str = get_param("method", Data("Mean")).as_string();
    AdaptiveMethod method = (method_str == "Gaussian") ? AdaptiveMethod::Gaussian : AdaptiveMethod::Mean;

    int block_size = static_cast<int>(get_param("block_size", Data(31)).as_int());
    if (block_size % 2 == 0) block_size++;
    if (block_size < 3) block_size = 3;

    int c = static_cast<int>(get_param("c", Data(5)).as_int());

    String type_str = get_param("threshold_type", Data("Binary")).as_string();
    ThresholdType type = (type_str == "BinaryInv") ? ThresholdType::BinaryInv : ThresholdType::Binary;

    // 自适应阈值分割
    std::vector<uint8_t> binary(w * h);
    seg_utils::adaptive_threshold(gray.data(), binary.data(), w, h, block_size, c, method, type);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = binary;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("binary", Data(output));

    OVF_INFO() << "AdaptiveThreshold completed: block_size=" << block_size << ", c=" << c;

    return Result<void>::success();
}

// ==================== EdgeSegmentNode ====================

EdgeSegmentNode::EdgeSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo EdgeSegmentNode::make_info() {
    NodeInfo info;
    info.id = "EdgeSegment";
    info.name = "边缘分割";
    info.category = "图像分割";
    info.description = "基于边缘检测的分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("edges", "边缘图像", DataType::Image));
    info.outputs.push_back(DataPort("edge_mask", "边缘掩码", DataType::Image));

    info.params.push_back(ParamDef("method", "边缘检测方法", DataType::String, Data("Sobel")));
    info.params.push_back(ParamDef("low_threshold", "低阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("high_threshold", "高阈值", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(50)));

    return info;
}

Result<void> EdgeSegmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (input.channels == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    String method_str = get_param("method", Data("Sobel")).as_string();
    EdgeSegmentMethod method = EdgeSegmentMethod::Sobel;
    if (method_str == "Canny") method = EdgeSegmentMethod::Canny;
    else if (method_str == "Laplacian") method = EdgeSegmentMethod::Laplacian;
    else if (method_str == "Gradient") method = EdgeSegmentMethod::Gradient;

    int low_thresh = static_cast<int>(get_param("low_threshold", Data(50)).as_int());
    int high_thresh = static_cast<int>(get_param("high_threshold", Data(100)).as_int());

    // 边缘分割
    std::vector<uint8_t> edges(w * h);
    seg_utils::edge_segment(gray.data(), edges.data(), w, h, method, low_thresh, high_thresh);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = edges;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("edges", Data(output));
    set_output("edge_mask", Data(output));

    OVF_INFO() << "EdgeSegment completed: method=" << method_str;

    return Result<void>::success();
}

// ==================== WatershedNode ====================

WatershedNode::WatershedNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo WatershedNode::make_info() {
    NodeInfo info;
    info.id = "Watershed";
    info.name = "分水岭分割";
    info.category = "图像分割";
    info.description = "基于标记的分水岭分割算法";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("markers", "标记图像", DataType::Image, false));
    info.outputs.push_back(DataPort("segmented", "分割结果", DataType::Image));
    info.outputs.push_back(DataPort("boundaries", "分割边界", DataType::Image));
    info.outputs.push_back(DataPort("num_regions", "区域数量", DataType::Number));

    info.params.push_back(ParamDef("marker_method", "标记生成方法", DataType::String, Data("Auto")));
    info.params.push_back(ParamDef("min_region_size", "最小区域大小", DataType::Number, Data(10)));

    return info;
}

void WatershedNode::generate_markers_from_gradient(const uint8_t* src, int32_t* markers, int w, int h) {
    // 计算梯度
    std::vector<uint8_t> grad(w * h);
    seg_utils::compute_gradient(src, nullptr, nullptr, grad.data(), w, h);

    // 基于梯度阈值生成标记
    int threshold = 20;
    std::fill(markers, markers + w * h, 0);

    // 低梯度区域为前景标记(1)，高梯度区域为背景标记(2)
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grad[y * w + x] < threshold) {
                markers[y * w + x] = 1; // 可能的前景
            } else if (grad[y * w + x] > threshold * 3) {
                markers[y * w + x] = 2; // 背景
            }
        }
    }

    // 使用形态学腐蚀提取确定的前景
    // 简化：直接使用中心区域作为标记
}

Result<void> WatershedNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (input.channels == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取或生成标记
    std::vector<int32_t> markers(w * h, 0);

    auto markers_data = get_input("markers");
    if (markers_data.is_image()) {
        ImageData markers_img = markers_data.as_image();
        for (int i = 0; i < w * h && i < markers_img.data.size(); ++i) {
            markers[i] = static_cast<int32_t>(markers_img.data[i]);
        }
    } else {
        generate_markers_from_gradient(gray.data(), markers.data(), w, h);
    }

    // 分水岭分割
    std::vector<int32_t> result(w * h);
    seg_utils::watershed_segment(gray.data(), markers.data(), result.data(), w, h);

    // 统计区域数量
    int num_regions = 0;
    std::set<int32_t> region_set;
    for (int i = 0; i < w * h; ++i) {
        if (result[i] > 0) {
            region_set.insert(result[i]);
        }
    }
    num_regions = region_set.size();

    // 创建分割结果图像
    std::vector<uint8_t> segmented(w * h);
    std::vector<uint8_t> boundaries(w * h, 0);

    // 为每个区域分配颜色
    std::map<int32_t, uint8_t> region_colors;
    for (auto& lbl : region_set) {
        region_colors[lbl] = static_cast<uint8_t>((lbl * 37) % 256);
    }

    for (int i = 0; i < w * h; ++i) {
        if (result[i] > 0) {
            segmented[i] = region_colors[result[i]];
        } else if (result[i] == -1) {
            boundaries[i] = 255;
            segmented[i] = 0;
        }
    }

    // 输出分割结果
    ImageData seg_output;
    seg_output.width = w;
    seg_output.height = h;
    seg_output.channels = 1;
    seg_output.format = ImageFormat::Mono8;
    seg_output.data = segmented;

    set_output("segmented", Data(seg_output));

    // 输出边界
    ImageData bound_output;
    bound_output.width = w;
    bound_output.height = h;
    bound_output.channels = 1;
    bound_output.format = ImageFormat::Mono8;
    bound_output.data = boundaries;

    set_output("boundaries", Data(bound_output));
    set_output("num_regions", Data(num_regions));

    OVF_INFO() << "Watershed completed: num_regions=" << num_regions;

    return Result<void>::success();
}

// ==================== RegionGrowingNode ====================

RegionGrowingNode::RegionGrowingNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo RegionGrowingNode::make_info() {
    NodeInfo info;
    info.id = "RegionGrowing";
    info.name = "区域生长分割";
    info.category = "图像分割";
    info.description = "基于种子点的区域生长分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("seed_point", "种子点", DataType::Point, false));
    info.outputs.push_back(DataPort("region", "生长区域", DataType::Image));
    info.outputs.push_back(DataPort("region_area", "区域面积", DataType::Number));
    info.outputs.push_back(DataPort("region_mean", "区域平均值", DataType::Number));

    info.params.push_back(ParamDef("seed_x", "种子点X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("seed_y", "种子点Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("threshold", "生长阈值", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("connectivity", "连通性(4/8)", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("auto_seed", "自动种子点", DataType::Boolean, Data(true)));

    return info;
}

void RegionGrowingNode::find_auto_seed(const uint8_t* src, int w, int h, int& seed_x, int& seed_y) {
    // 自动选择：选择图像中心
    seed_x = w / 2;
    seed_y = h / 2;
}

Result<void> RegionGrowingNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (input.channels == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    int seed_x = static_cast<int>(get_param("seed_x", Data(0)).as_int());
    int seed_y = static_cast<int>(get_param("seed_y", Data(0)).as_int());
    int threshold = static_cast<int>(get_param("threshold", Data(10)).as_int());
    int connectivity = static_cast<int>(get_param("connectivity", Data(8)).as_int());
    bool auto_seed = get_param("auto_seed", Data(true)).as_bool();

    if (auto_seed) {
        find_auto_seed(gray.data(), w, h, seed_x, seed_y);
    }

    // 检查种子点有效性
    if (seed_x < 0 || seed_x >= w || seed_y < 0 || seed_y >= h) {
        seed_x = w / 2;
        seed_y = h / 2;
    }

    bool use_8 = (connectivity == 8);

    // 区域生长
    std::vector<uint8_t> region(w * h);
    seg_utils::region_growing(gray.data(), region.data(), w, h, seed_x, seed_y, threshold, use_8);

    // 计算区域统计
    int area = 0;
    int sum = 0;
    for (int i = 0; i < w * h; ++i) {
        if (region[i] > 0) {
            area++;
            sum += gray[i];
        }
    }
    float mean_val = (area > 0) ? sum / area : 0;

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = region;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("region", Data(output));
    set_output("region_area", Data(area));
    set_output("region_mean", Data(mean_val));

    OVF_INFO() << "RegionGrowing completed: seed=(" << seed_x << "," << seed_y << "), area=" << area;

    return Result<void>::success();
}

// ==================== RegionSplitMergeNode ====================

RegionSplitMergeNode::RegionSplitMergeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo RegionSplitMergeNode::make_info() {
    NodeInfo info;
    info.id = "RegionSplitMerge";
    info.name = "区域分裂合并";
    info.category = "图像分割";
    info.description = "基于四叉树的区域分裂合并分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("segmented", "分割结果", DataType::Image));
    info.outputs.push_back(DataPort("region_labels", "区域标签", DataType::Image));
    info.outputs.push_back(DataPort("num_regions", "区域数量", DataType::Number));

    info.params.push_back(ParamDef("min_size", "最小区域大小", DataType::Number, Data(16)));
    info.params.push_back(ParamDef("threshold", "分裂阈值", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("merge_threshold", "合并阈值", DataType::Number, Data(10)));

    return info;
}

Result<void> RegionSplitMergeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (input.channels == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    int min_size = static_cast<int>(get_param("min_size", Data(16)).as_int());
    int threshold = static_cast<int>(get_param("threshold", Data(20)).as_int());

    // 区域分裂合并
    std::vector<uint8_t> segmented(w * h);
    seg_utils::region_split_merge(gray.data(), segmented.data(), w, h, min_size, threshold);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = segmented;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("segmented", Data(output));

    OVF_INFO() << "RegionSplitMerge completed: min_size=" << min_size << ", threshold=" << threshold;

    return Result<void>::success();
}

// ==================== KMeansSegmentNode ====================

KMeansSegmentNode::KMeansSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo KMeansSegmentNode::make_info() {
    NodeInfo info;
    info.id = "KMeansSegment";
    info.name = "K均值聚类分割";
    info.category = "图像分割";
    info.description = "基于颜色K均值聚类的分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("segmented", "分割结果", DataType::Image));
    info.outputs.push_back(DataPort("labels", "像素标签", DataType::Image));
    info.outputs.push_back(DataPort("centers", "聚类中心颜色", DataType::String));

    info.params.push_back(ParamDef("k", "聚类数量", DataType::Number, Data(4)));
    info.params.push_back(ParamDef("max_iterations", "最大迭代次数", DataType::Number, Data(100)));

    return info;
}

Result<void> KMeansSegmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 获取参数
    int k = static_cast<int>(get_param("k", Data(4)).as_int());
    if (k < 2) k = 2;
    if (k > 16) k = 16;

    int max_iter = static_cast<int>(get_param("max_iterations", Data(100)).as_int());

    // K均值聚类分割
    std::vector<uint8_t> segmented(w * h * ch);
    seg_utils::kmeans_segment(input.data.data(), segmented.data(), w, h, ch, k, max_iter);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = ch;
    output.format = input.format;
    output.data = segmented;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("segmented", Data(output));

    OVF_INFO() << "KMeansSegment completed: k=" << k;

    return Result<void>::success();
}

// ==================== GraphCutNode ====================

GraphCutNode::GraphCutNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo GraphCutNode::make_info() {
    NodeInfo info;
    info.id = "GraphCut";
    info.name = "图割分割";
    info.category = "图像分割";
    info.description = "简化版图割分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("foreground_seed", "前景种子", DataType::Point, false));
    info.inputs.push_back(DataPort("background_seed", "背景种子", DataType::Point, false));
    info.outputs.push_back(DataPort("mask", "分割掩码", DataType::Image));
    info.outputs.push_back(DataPort("foreground_image", "前景图像", DataType::Image));
    info.outputs.push_back(DataPort("background_image", "背景图像", DataType::Image));

    info.params.push_back(ParamDef("fg_seed_x", "前景种子X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("fg_seed_y", "前景种子Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("bg_seed_x", "背景种子X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("bg_seed_y", "背景种子Y", DataType::Number, Data(0)));

    return info;
}

Result<void> GraphCutNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 转换为灰度
    std::vector<uint8_t> gray(w * h);
    if (ch == 1) {
        gray = input.data;
    } else {
        for (int i = 0; i < w * h; ++i) {
            int idx = i * ch;
            gray[i] = static_cast<uint8_t>((input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    int fg_x = static_cast<int>(get_param("fg_seed_x", Data(w / 4)).as_int());
    int fg_y = static_cast<int>(get_param("fg_seed_y", Data(h / 4)).as_int());
    int bg_x = static_cast<int>(get_param("bg_seed_x", Data(3 * w / 4)).as_int());
    int bg_y = static_cast<int>(get_param("bg_seed_y", Data(3 * h / 4)).as_int());

    // 图割分割
    std::vector<uint8_t> mask(w * h);
    seg_utils::graphcut_segment(gray.data(), mask.data(), w, h, fg_x, fg_y, bg_x, bg_y);

    // 创建前景和背景图像
    std::vector<uint8_t> foreground(w * h * ch);
    std::vector<uint8_t> background(w * h * ch);

    for (int i = 0; i < w * h; ++i) {
        for (int c = 0; c < ch; ++c) {
            if (mask[i] > 0) {
                foreground[i * ch + c] = input.data[i * ch + c];
                background[i * ch + c] = 0;
            } else {
                foreground[i * ch + c] = 0;
                background[i * ch + c] = input.data[i * ch + c];
            }
        }
    }

    // 输出掩码
    ImageData mask_output;
    mask_output.width = w;
    mask_output.height = h;
    mask_output.channels = 1;
    mask_output.format = ImageFormat::Mono8;
    mask_output.data = mask;

    set_output("mask", Data(mask_output));

    // 输出前景
    ImageData fg_output;
    fg_output.width = w;
    fg_output.height = h;
    fg_output.channels = ch;
    fg_output.format = input.format;
    fg_output.data = foreground;

    set_output("foreground_image", Data(fg_output));

    // 输出背景
    ImageData bg_output;
    bg_output.width = w;
    bg_output.height = h;
    bg_output.channels = ch;
    bg_output.format = input.format;
    bg_output.data = background;

    set_output("background_image", Data(bg_output));

    OVF_INFO() << "GraphCut completed";

    return Result<void>::success();
}

// ==================== GrabCutNode ====================

GrabCutNode::GrabCutNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo GrabCutNode::make_info() {
    NodeInfo info;
    info.id = "GrabCut";
    info.name = "GrabCut分割";
    info.category = "图像分割";
    info.description = "简化版GrabCut分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入RGB图像", DataType::Image, true));
    info.inputs.push_back(DataPort("rect", "矩形区域", DataType::Region, false));
    info.outputs.push_back(DataPort("mask", "分割掩码", DataType::Image));
    info.outputs.push_back(DataPort("foreground", "前景图像", DataType::Image));
    info.outputs.push_back(DataPort("background", "背景图像", DataType::Image));

    info.params.push_back(ParamDef("rect_x", "矩形X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("rect_y", "矩形Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("rect_w", "矩形宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("rect_h", "矩形高度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("iterations", "迭代次数", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("auto_rect", "自动矩形", DataType::Boolean, Data(true)));

    return info;
}

void GrabCutNode::init_mask_from_rect(uint8_t* mask, int w, int h, int rx, int ry, int rw, int rh) {
    std::fill(mask, mask + w * h, 0);

    for (int y = ry; y < ry + rh && y < h; ++y) {
        for (int x = rx; x < rx + rw && x < w; ++x) {
            mask[y * w + x] = 3; // 可能前景
        }
    }
}

void GrabCutNode::refine_segmentation(const uint8_t* src, uint8_t* mask, int w, int h, int channels, int iterations) {
    // 简化版GrabCut迭代优化
    for (int iter = 0; iter < iterations; ++iter) {
        // 计算前景和背景颜色分布
        std::vector<float> fg_mean(channels, 0);
        std::vector<float> bg_mean(channels, 0);
        int fg_count = 0, bg_count = 0;

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int idx = y * w + x;
                if (mask[idx] > 0) {
                    for (int c = 0; c < channels; ++c) {
                        fg_mean[c] += src[idx * channels + c];
                    }
                    fg_count++;
                } else {
                    for (int c = 0; c < channels; ++c) {
                        bg_mean[c] += src[idx * channels + c];
                    }
                    bg_count++;
                }
            }
        }

        if (fg_count > 0) {
            for (int c = 0; c < channels; ++c) fg_mean[c] /= fg_count;
        }
        if (bg_count > 0) {
            for (int c = 0; c < channels; ++c) bg_mean[c] /= bg_count;
        }

        // 更新掩码
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int idx = y * w + x;
                if (mask[idx] == 0 || mask[idx] == 3) {
                    float fg_dist = 0, bg_dist = 0;
                    for (int c = 0; c < channels; ++c) {
                        fg_dist += std::pow(src[idx * channels + c] - fg_mean[c], 2);
                        bg_dist += std::pow(src[idx * channels + c] - bg_mean[c], 2);
                    }

                    mask[idx] = (fg_dist < bg_dist * 2) ? 255 : 0;
                }
            }
        }
    }
}

Result<void> GrabCutNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty() || input.channels < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty or not RGB");
    }

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 获取参数
    int rx = static_cast<int>(get_param("rect_x", Data(0)).as_int());
    int ry = static_cast<int>(get_param("rect_y", Data(0)).as_int());
    int rw = static_cast<int>(get_param("rect_w", Data(w / 2)).as_int());
    int rh = static_cast<int>(get_param("rect_h", Data(h / 2)).as_int());
    int iterations = static_cast<int>(get_param("iterations", Data(5)).as_int());
    bool auto_rect = get_param("auto_rect", Data(true)).as_bool();

    if (auto_rect) {
        rx = w / 4;
        ry = h / 4;
        rw = w / 2;
        rh = h / 2;
    }

    // GrabCut分割
    std::vector<uint8_t> mask(w * h);
    seg_utils::grabcut_segment(input.data.data(), mask.data(), w, h, ch, rx, ry, rw, rh, iterations);

    // 创建前景和背景图像
    std::vector<uint8_t> foreground(w * h * ch);
    std::vector<uint8_t> background(w * h * ch);

    for (int i = 0; i < w * h; ++i) {
        for (int c = 0; c < ch; ++c) {
            if (mask[i] > 0) {
                foreground[i * ch + c] = input.data[i * ch + c];
                background[i * ch + c] = 0;
            } else {
                foreground[i * ch + c] = 0;
                background[i * ch + c] = input.data[i * ch + c];
            }
        }
    }

    // 输出掩码
    ImageData mask_output;
    mask_output.width = w;
    mask_output.height = h;
    mask_output.channels = 1;
    mask_output.format = ImageFormat::Mono8;
    mask_output.data = mask;

    set_output("mask", Data(mask_output));

    // 输出前景
    ImageData fg_output;
    fg_output.width = w;
    fg_output.height = h;
    fg_output.channels = ch;
    fg_output.format = input.format;
    fg_output.data = foreground;

    set_output("foreground", Data(fg_output));

    // 输出背景
    ImageData bg_output;
    bg_output.width = w;
    bg_output.height = h;
    bg_output.channels = ch;
    bg_output.format = input.format;
    bg_output.data = background;

    set_output("background", Data(bg_output));

    OVF_INFO() << "GrabCut completed: iterations=" << iterations;

    return Result<void>::success();
}

// ==================== MeanShiftSegmentNode ====================

MeanShiftSegmentNode::MeanShiftSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MeanShiftSegmentNode::make_info() {
    NodeInfo info;
    info.id = "MeanShiftSegment";
    info.name = "MeanShift分割";
    info.category = "图像分割";
    info.description = "基于MeanShift聚类的分割";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("segmented", "分割结果", DataType::Image));
    info.outputs.push_back(DataPort("labels", "区域标签", DataType::Image));
    info.outputs.push_back(DataPort("num_regions", "区域数量", DataType::Number));

    info.params.push_back(ParamDef("spatial_radius", "空间半径", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("color_radius", "颜色半径", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("min_density", "最小密度", DataType::Number, Data(50)));

    return info;
}

Result<void> MeanShiftSegmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 获取参数
    float spatial_radius = static_cast<float>(get_param("spatial_radius", Data(10)).as_number());
    float color_radius = static_cast<float>(get_param("color_radius", Data(20)).as_number());
    int min_density = static_cast<int>(get_param("min_density", Data(50)).as_int());

    // MeanShift分割
    std::vector<uint8_t> segmented(w * h * ch);
    seg_utils::meanshift_segment(input.data.data(), segmented.data(), w, h, ch, spatial_radius, color_radius, min_density);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = ch;
    output.format = input.format;
    output.data = segmented;
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    set_output("segmented", Data(output));

    OVF_INFO() << "MeanShiftSegment completed";

    return Result<void>::success();
}

// ==================== SuperpixelNode ====================

SuperpixelNode::SuperpixelNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo SuperpixelNode::make_info() {
    NodeInfo info;
    info.id = "Superpixel";
    info.name = "超像素分割";
    info.category = "图像分割";
    info.description = "SLIC超像素分割（简化版）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("labels", "超像素标签", DataType::Image));
    info.outputs.push_back(DataPort("boundaries", "超像素边界", DataType::Image));
    info.outputs.push_back(DataPort("num_superpixels", "超像素数量", DataType::Number));
    info.outputs.push_back(DataPort("superpixel_image", "超像素平均颜色图像", DataType::Image));

    info.params.push_back(ParamDef("num_superpixels", "超像素数量", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("compactness", "紧致度", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("iterations", "迭代次数", DataType::Number, Data(10)));

    return info;
}

void SuperpixelNode::draw_superpixel_boundaries(const int32_t* labels, uint8_t* boundaries, int w, int h) {
    std::fill(boundaries, boundaries + w * h, 0);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            int32_t lbl = labels[y * w + x];

            // 检查四邻域是否有不同标签
            if (labels[y * w + (x - 1)] != lbl ||
                labels[y * w + (x + 1)] != lbl ||
                labels[(y - 1) * w + x] != lbl ||
                labels[(y + 1) * w + x] != lbl) {
                boundaries[y * w + x] = 255;
            }
        }
    }
}

void SuperpixelNode::create_superpixel_image(const uint8_t* src, const int32_t* labels, uint8_t* dst,
                                              int w, int h, int channels) {
    // 计算每个超像素的平均颜色
    std::map<int32_t, std::vector<float>> color_sum;
    std::map<int32_t, int> counts;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int32_t lbl = labels[y * w + x];
            for (int c = 0; c < channels; ++c) {
                color_sum[lbl].push_back(src[(y * w + x) * channels + c]);
            }
            counts[lbl]++;
        }
    }

    std::map<int32_t, std::vector<uint8_t>> avg_colors;
    for (auto& [lbl, sum] : color_sum) {
        avg_colors[lbl].resize(channels);
        for (int c = 0; c < channels; ++c) {
            avg_colors[lbl][c] = static_cast<uint8_t>(sum[c] / counts[lbl]);
        }
    }

    // 应用平均颜色
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int32_t lbl = labels[y * w + x];
            for (int c = 0; c < channels; ++c) {
                dst[(y * w + x) * channels + c] = avg_colors[lbl][c];
            }
        }
    }
}

Result<void> SuperpixelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 获取参数
    int num_sp = static_cast<int>(get_param("num_superpixels", Data(100)).as_int());
    int compactness = static_cast<int>(get_param("compactness", Data(10)).as_int());
    int iterations = static_cast<int>(get_param("iterations", Data(10)).as_int());

    // 超像素分割
    std::vector<int32_t> labels(w * h);
    seg_utils::superpixel_segment(input.data.data(), labels.data(), w, h, ch, num_sp, compactness, iterations);

    // 统计超像素数量
    std::set<int32_t> sp_set;
    for (int i = 0; i < w * h; ++i) {
        sp_set.insert(labels[i]);
    }
    int num_superpixels = sp_set.size();

    // 绘制边界
    std::vector<uint8_t> boundaries(w * h);
    draw_superpixel_boundaries(labels.data(), boundaries.data(), w, h);

    // 创建超像素平均颜色图像
    std::vector<uint8_t> sp_image(w * h * ch);
    create_superpixel_image(input.data.data(), labels.data(), sp_image.data(), w, h, ch);

    // 输出标签
    ImageData labels_output;
    labels_output.width = w;
    labels_output.height = h;
    labels_output.channels = 1;
    labels_output.format = ImageFormat::Mono8;
    // 将标签映射到0-255范围
    labels_output.data.resize(w * h);
    for (int i = 0; i < w * h; ++i) {
        labels_output.data[i] = static_cast<uint8_t>((labels[i] * 37) % 256);
    }

    set_output("labels", Data(labels_output));

    // 输出边界
    ImageData bound_output;
    bound_output.width = w;
    bound_output.height = h;
    bound_output.channels = 1;
    bound_output.format = ImageFormat::Mono8;
    bound_output.data = boundaries;

    set_output("boundaries", Data(bound_output));

    // 输出数量
    set_output("num_superpixels", Data(num_superpixels));

    // 输出超像素图像
    ImageData sp_output;
    sp_output.width = w;
    sp_output.height = h;
    sp_output.channels = ch;
    sp_output.format = input.format;
    sp_output.data = sp_image;

    set_output("superpixel_image", Data(sp_output));

    OVF_INFO() << "Superpixel completed: num_superpixels=" << num_superpixels;

    return Result<void>::success();
}

// ==================== Node Registration ====================

OVF_REGISTER_NODE(ThresholdSegmentNode, "ThresholdSegment", ThresholdSegmentNode::make_info())
OVF_REGISTER_NODE(AdaptiveThresholdNode, "AdaptiveThreshold", AdaptiveThresholdNode::make_info())
OVF_REGISTER_NODE(EdgeSegmentNode, "EdgeSegment", EdgeSegmentNode::make_info())
OVF_REGISTER_NODE(WatershedNode, "Watershed", WatershedNode::make_info())
OVF_REGISTER_NODE(RegionGrowingNode, "RegionGrowing", RegionGrowingNode::make_info())
OVF_REGISTER_NODE(RegionSplitMergeNode, "RegionSplitMerge", RegionSplitMergeNode::make_info())
OVF_REGISTER_NODE(KMeansSegmentNode, "KMeansSegment", KMeansSegmentNode::make_info())
OVF_REGISTER_NODE(GraphCutNode, "GraphCut", GraphCutNode::make_info())
OVF_REGISTER_NODE(GrabCutNode, "GrabCut", GrabCutNode::make_info())
OVF_REGISTER_NODE(MeanShiftSegmentNode, "MeanShiftSegment", MeanShiftSegmentNode::make_info())
OVF_REGISTER_NODE(SuperpixelNode, "Superpixel", SuperpixelNode::make_info())

} // namespace algorithm
} // namespace ovf