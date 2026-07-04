/**
 * @file image_stitch.h
 * @brief 图像拼接和融合算子（纯C++实现，不依赖OpenCV）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 单应性矩阵结构（3x3）
 */
struct Homography {
    double h[9];  // 3x3矩阵，行优先存储
    
    Homography() {
        // 默认为单位矩阵
        h[0] = 1; h[1] = 0; h[2] = 0;
        h[3] = 0; h[4] = 1; h[5] = 0;
        h[6] = 0; h[7] = 0; h[8] = 1;
    }
    
    // 应用变换到点
    inline void transform(double x, double y, double& tx, double& ty) const {
        double w = h[6] * x + h[7] * y + h[8];
        tx = (h[0] * x + h[1] * y + h[2]) / w;
        ty = (h[3] * x + h[4] * y + h[5]) / w;
    }
    
    // 逆变换
    Homography inverse() const {
        Homography inv;
        double det = h[0] * (h[4] * h[8] - h[5] * h[7]) -
                     h[1] * (h[3] * h[8] - h[5] * h[6]) +
                     h[2] * (h[3] * h[7] - h[4] * h[6]);
        
        if (std::abs(det) < 1e-10) return inv;
        
        double inv_det = 1.0 / det;
        
        inv.h[0] = (h[4] * h[8] - h[5] * h[7]) * inv_det;
        inv.h[1] = (h[2] * h[7] - h[1] * h[8]) * inv_det;
        inv.h[2] = (h[1] * h[5] - h[2] * h[4]) * inv_det;
        inv.h[3] = (h[5] * h[6] - h[3] * h[8]) * inv_det;
        inv.h[4] = (h[0] * h[8] - h[2] * h[6]) * inv_det;
        inv.h[5] = (h[2] * h[3] - h[0] * h[5]) * inv_det;
        inv.h[6] = (h[3] * h[7] - h[4] * h[6]) * inv_det;
        inv.h[7] = (h[1] * h[6] - h[0] * h[7]) * inv_det;
        inv.h[8] = (h[0] * h[4] - h[1] * h[3]) * inv_det;
        
        return inv;
    }
};

/**
 * @brief 匹配点对结构
 */
struct MatchPair {
    double x1, y1;  // 图像1中的点
    double x2, y2;  // 图像2中的点
    bool valid = true;
};

/**
 * @brief 图像拼接工具函数
 */
namespace stitch_utils {

// ==================== 特征检测 ====================

/**
 * @brief Harris角点检测（用于配准）
 */
inline void detect_harris_corners(const ImageData& gray, std::vector<std::pair<double, double>>& corners,
                                  double k = 0.04, int threshold = 50, int block_size = 3,
                                  int max_corners = 500) {
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
    std::vector<std::pair<double, std::pair<double, double>>> scored_corners;
    
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
                scored_corners.push_back({r, {static_cast<double>(x), static_cast<double>(y)}});
            }
        }
    }
    
    // 按响应值排序并取前N个
    std::sort(scored_corners.begin(), scored_corners.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    for (int i = 0; i < std::min(max_corners, static_cast<int>(scored_corners.size())); ++i) {
        corners.push_back(scored_corners[i].second);
    }
}

/**
 * @brief 计算简化的SIFT描述符
 */
inline void compute_sift_descriptor(const ImageData& gray, double x, double y,
                                    std::vector<float>& descriptor, int desc_size = 8) {
    descriptor.clear();
    descriptor.resize(desc_size * desc_size, 0.0f);
    
    if (gray.empty() || gray.channels != 1) return;
    
    uint32_t w = gray.width;
    uint32_t h = gray.height;
    int half = desc_size / 2;
    
    for (int dy = -half; dy < half; ++dy) {
        for (int dx = -half; dx < half; ++dx) {
            int px = static_cast<int>(x) + dx;
            int py = static_cast<int>(y) + dy;
            
            if (px >= 1 && px < static_cast<int>(w) - 1 &&
                py >= 1 && py < static_cast<int>(h) - 1) {
                // 计算梯度幅值
                double gx = gray.data[py * w + (px + 1)] - gray.data[py * w + (px - 1)];
                double gy = gray.data[(py + 1) * w + px] - gray.data[(py - 1) * w + px];
                double mag = std::sqrt(gx * gx + gy * gy);
                descriptor[(dy + half) * desc_size + (dx + half)] = static_cast<float>(mag);
            }
        }
    }
    
    // 归一化
    float norm = 0;
    for (float v : descriptor) norm += v * v;
    norm = std::sqrt(norm) + 1e-7f;
    for (float& v : descriptor) v /= norm;
}

/**
 * @brief 特征点匹配（基于描述符）
 */
inline void match_features(const std::vector<std::pair<double, double>>& corners1,
                           const std::vector<std::vector<float>>& desc1,
                           const std::vector<std::pair<double, double>>& corners2,
                           const std::vector<std::vector<float>>& desc2,
                           std::vector<MatchPair>& matches,
                           double ratio_threshold = 0.75) {
    matches.clear();
    
    for (size_t i = 0; i < desc1.size(); ++i) {
        double best_dist = std::numeric_limits<double>::max();
        double second_dist = std::numeric_limits<double>::max();
        int best_j = -1;
        
        for (size_t j = 0; j < desc2.size(); ++j) {
            double dist = 0;
            size_t min_size = std::min(desc1[i].size(), desc2[j].size());
            for (size_t k = 0; k < min_size; ++k) {
                double diff = desc1[i][k] - desc2[j][k];
                dist += diff * diff;
            }
            
            if (dist < best_dist) {
                second_dist = best_dist;
                best_dist = dist;
                best_j = static_cast<int>(j);
            } else if (dist < second_dist) {
                second_dist = dist;
            }
        }
        
        if (best_j >= 0 && best_dist < ratio_threshold * second_dist) {
            MatchPair mp;
            mp.x1 = corners1[i].first;
            mp.y1 = corners1[i].second;
            mp.x2 = corners2[best_j].first;
            mp.y2 = corners2[best_j].second;
            matches.push_back(mp);
        }
    }
}

// ==================== 单应性矩阵计算 ====================

/**
 * @brief 从4对点计算单应性矩阵（DLT算法）
 */
inline Homography compute_homography_4pts(const MatchPair* pairs) {
    Homography H;
    
    // 构建矩阵A（8x9）
    double A[8][9] = {0};
    
    for (int i = 0; i < 4; ++i) {
        double x1 = pairs[i].x1, y1 = pairs[i].y1;
        double x2 = pairs[i].x2, y2 = pairs[i].y2;
        
        A[i * 2][0] = -x1;
        A[i * 2][1] = -y1;
        A[i * 2][2] = -1;
        A[i * 2][6] = x2 * x1;
        A[i * 2][7] = x2 * y1;
        A[i * 2][8] = x2;
        
        A[i * 2 + 1][3] = -x1;
        A[i * 2 + 1][4] = -y1;
        A[i * 2 + 1][5] = -1;
        A[i * 2 + 1][6] = y2 * x1;
        A[i * 2 + 1][7] = y2 * y1;
        A[i * 2 + 1][8] = y2;
    }
    
    // SVD求解（简化版：使用奇异值分解近似）
    // 这里使用简化的方法：直接解线性方程组
    // 对A进行奇异值分解的最小奇异值对应的向量
    
    // 简化实现：使用迭代法求解
    double h[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    
    for (int iter = 0; iter < 100; ++iter) {
        double b[8], J[8][9];
        
        for (int i = 0; i < 8; ++i) {
            double sum = 0;
            for (int j = 0; j < 9; ++j) {
                sum += A[i][j] * h[j];
            }
            b[i] = -sum;
            
            for (int j = 0; j < 9; ++j) {
                J[i][j] = A[i][j];
            }
        }
        
        // 求解 J * delta = b（简化版）
        // 使用最小二乘近似
        double delta[9] = {0};
        for (int j = 0; j < 9; ++j) {
            for (int i = 0; i < 8; ++i) {
                delta[j] += J[i][j] * b[i];
            }
        }
        
        // 更新h
        for (int j = 0; j < 9; ++j) {
            h[j] += delta[j] * 0.001;
        }
        
        // 归一化
        double norm = std::sqrt(h[0]*h[0] + h[1]*h[1] + h[2]*h[2] +
                                h[3]*h[3] + h[4]*h[4] + h[5]*h[5] +
                                h[6]*h[6] + h[7]*h[7] + h[8]*h[8]);
        if (norm > 1e-6) {
            for (int j = 0; j < 9; ++j) h[j] /= norm;
        }
    }
    
    for (int i = 0; i < 9; ++i) H.h[i] = h[i];
    
    return H;
}

/**
 * @brief 计算单应性矩阵误差
 */
inline double compute_homography_error(const Homography& H, const MatchPair& pair) {
    double tx, ty;
    H.transform(pair.x1, pair.y1, tx, ty);
    double dx = tx - pair.x2;
    double dy = ty - pair.y2;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief RANSAC估计单应性矩阵
 */
inline Homography ransac_homography(const std::vector<MatchPair>& matches,
                                    int max_iter = 1000, double threshold = 3.0,
                                    double confidence = 0.99) {
    Homography best_H;
    int best_count = 0;
    
    if (matches.size() < 4) return best_H;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, static_cast<int>(matches.size() - 1));
    
    for (int iter = 0; iter < max_iter; ++iter) {
        // 随机选择4个点
        MatchPair samples[4];
        std::vector<int> indices;
        
        for (int i = 0; i < 4; ++i) {
            int idx;
            do {
                idx = dis(gen);
            } while (std::find(indices.begin(), indices.end(), idx) != indices.end());
            indices.push_back(idx);
            samples[i] = matches[idx];
        }
        
        // 计算单应性矩阵
        Homography H = compute_homography_4pts(samples);
        
        // 计算内点数
        int count = 0;
        for (const auto& m : matches) {
            double err = compute_homography_error(H, m);
            if (err < threshold) {
                count++;
            }
        }
        
        if (count > best_count) {
            best_count = count;
            best_H = H;
            
            // 更新迭代次数
            double inlier_ratio = static_cast<double>(count) / matches.size();
            if (inlier_ratio > 0) {
                double new_max_iter = std::log(1.0 - confidence) / 
                                     std::log(1.0 - std::pow(inlier_ratio, 4));
                max_iter = std::min(max_iter, static_cast<int>(new_max_iter));
            }
        }
    }
    
    // 使用所有内点重新估计
    std::vector<MatchPair> inliers;
    for (const auto& m : matches) {
        double err = compute_homography_error(best_H, m);
        if (err < threshold) {
            inliers.push_back(m);
        }
    }
    
    if (inliers.size() >= 4) {
        // 使用最小二乘重新估计
        // 简化版：多次迭代优化
        for (int refine_iter = 0; refine_iter < 10 && inliers.size() >= 4; ++refine_iter) {
            MatchPair samples[4];
            for (int i = 0; i < 4; ++i) {
                samples[i] = inliers[i % inliers.size()];
            }
            best_H = compute_homography_4pts(samples);
            
            // 更新内点
            inliers.clear();
            for (const auto& m : matches) {
                double err = compute_homography_error(best_H, m);
                if (err < threshold) {
                    inliers.push_back(m);
                }
            }
        }
    }
    
    return best_H;
}

// ==================== 图像变换 ====================

/**
 * @brief 计算变换后的图像边界
 */
inline void compute_transformed_bounds(const ImageData& img, const Homography& H,
                                       double& min_x, double& max_x,
                                       double& min_y, double& max_y) {
    min_x = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    min_y = std::numeric_limits<double>::max();
    max_y = std::numeric_limits<double>::lowest();
    
    // 检查四个角点
    double corners[4][2] = {
        {0, 0},
        {static_cast<double>(img.width), 0},
        {static_cast<double>(img.width), static_cast<double>(img.height)},
        {0, static_cast<double>(img.height)}
    };
    
    for (int i = 0; i < 4; ++i) {
        double tx, ty;
        H.transform(corners[i][0], corners[i][1], tx, ty);
        min_x = std::min(min_x, tx);
        max_x = std::max(max_x, tx);
        min_y = std::min(min_y, ty);
        max_y = std::max(max_y, ty);
    }
}

/**
 * @brief 使用单应性矩阵变换图像
 */
inline void warp_image(const ImageData& src, ImageData& dst, const Homography& H,
                       int dst_width, int dst_height, int offset_x = 0, int offset_y = 0) {
    dst.width = dst_width;
    dst.height = dst_height;
    dst.channels = src.channels;
    dst.format = src.format;
    dst.data.resize(dst_width * dst_height * dst.channels, 0);
    
    Homography H_inv = H.inverse();
    
    for (int y = 0; y < dst_height; ++y) {
        for (int x = 0; x < dst_width; ++x) {
            double sx, sy;
            H_inv.transform(x - offset_x, y - offset_y, sx, sy);
            
            int src_x = static_cast<int>(sx);
            int src_y = static_cast<int>(sy);
            
            if (src_x >= 0 && src_x < static_cast<int>(src.width) &&
                src_y >= 0 && src_y < static_cast<int>(src.height)) {
                // 双线性插值
                double dx = sx - src_x;
                double dy = sy - src_y;
                
                int x0 = src_x, x1 = std::min(src_x + 1, static_cast<int>(src.width) - 1);
                int y0 = src_y, y1 = std::min(src_y + 1, static_cast<int>(src.height) - 1);
                
                for (int c = 0; c < src.channels; ++c) {
                    double v00 = src.data[(y0 * src.width + x0) * src.channels + c];
                    double v01 = src.data[(y0 * src.width + x1) * src.channels + c];
                    double v10 = src.data[(y1 * src.width + x0) * src.channels + c];
                    double v11 = src.data[(y1 * src.width + x1) * src.channels + c];
                    
                    double val = v00 * (1 - dx) * (1 - dy) +
                                 v01 * dx * (1 - dy) +
                                 v10 * (1 - dx) * dy +
                                 v11 * dx * dy;
                    
                    dst.data[(y * dst_width + x) * dst.channels + c] = 
                        static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
                }
            }
        }
    }
}

// ==================== 图像融合 ====================

/**
 * @brief 简单线性混合（渐变混合）
 */
inline void blend_linear(ImageData& dst, const ImageData& src1, const ImageData& src2,
                         double alpha = 0.5) {
    if (src1.width != src2.width || src1.height != src2.height ||
        src1.channels != src2.channels) return;
    
    dst = src1;
    
    for (size_t i = 0; i < dst.data.size(); ++i) {
        dst.data[i] = static_cast<uint8_t>(
            src1.data[i] * (1 - alpha) + src2.data[i] * alpha
        );
    }
}

/**
 * @brief 渐变混合（基于距离的权重）
 */
inline void blend_gradient(ImageData& dst, const ImageData& src1, const ImageData& src2,
                           int seam_x, int blend_width = 50) {
    if (src1.width != src2.width || src1.height != src2.height ||
        src1.channels != src2.channels) return;
    
    dst = src1;
    
    int left_start = seam_x - blend_width / 2;
    int right_end = seam_x + blend_width / 2;
    
    for (uint32_t y = 0; y < dst.height; ++y) {
        for (uint32_t x = 0; x < dst.width; ++x) {
            double alpha = 0.5;
            
            if (x < left_start) {
                alpha = 0.0;  // 只使用src1
            } else if (x > right_end) {
                alpha = 1.0;  // 只使用src2
            } else {
                alpha = static_cast<double>(x - left_start) / blend_width;
            }
            
            for (int c = 0; c < dst.channels; ++c) {
                size_t idx = (y * dst.width + x) * dst.channels + c;
                dst.data[idx] = static_cast<uint8_t>(
                    src1.data[idx] * (1 - alpha) + src2.data[idx] * alpha
                );
            }
        }
    }
}

/**
 * @brief 多频段融合（简化版）
 */
inline void blend_multiband(ImageData& dst, const ImageData& src1, const ImageData& src2,
                            int levels = 3) {
    if (src1.width != src2.width || src1.height != src2.height ||
        src1.channels != src2.channels) return;
    
    // 简化实现：使用加权平均
    // 实际多频段融合需要构建拉普拉斯金字塔
    
    dst.width = src1.width;
    dst.height = src1.height;
    dst.channels = src1.channels;
    dst.format = src1.format;
    dst.data.resize(dst.width * dst.height * dst.channels);
    
    // 计算中心权重
    for (uint32_t y = 0; y < dst.height; ++y) {
        for (uint32_t x = 0; x < dst.width; ++x) {
            // 计算到中心的距离权重
            double cx = dst.width / 2.0;
            double cy = dst.height / 2.0;
            double dist1 = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
            double max_dist = std::sqrt(cx * cx + cy * cy);
            double weight1 = 1.0 - dist1 / max_dist;
            double weight2 = dist1 / max_dist;
            
            for (int c = 0; c < dst.channels; ++c) {
                size_t idx = (y * dst.width + x) * dst.channels + c;
                dst.data[idx] = static_cast<uint8_t>(
                    src1.data[idx] * weight1 + src2.data[idx] * weight2
                );
            }
        }
    }
}

/**
 * @brief 接缝融合（寻找最佳接缝线）
 */
inline void find_seam(const ImageData& src1, const ImageData& src2,
                      std::vector<int>& seam_positions) {
    if (src1.width != src2.width || src1.height != src2.height) return;
    
    seam_positions.resize(src1.height);
    
    // 计算差异图
    std::vector<double> diff(src1.width * src1.height);
    
    for (uint32_t y = 0; y < src1.height; ++y) {
        for (uint32_t x = 0; x < src1.width; ++x) {
            double d = 0;
            for (int c = 0; c < src1.channels; ++c) {
                size_t idx = (y * src1.width + x) * src1.channels + c;
                d += std::abs(static_cast<int>(src1.data[idx]) - static_cast<int>(src2.data[idx]));
            }
            diff[y * src1.width + x] = d / src1.channels;
        }
    }
    
    // 使用动态规划寻找最佳接缝线
    // 简化版：每行独立寻找最小差异点
    for (uint32_t y = 0; y < src1.height; ++y) {
        double min_diff = std::numeric_limits<double>::max();
        int best_x = src1.width / 2;
        
        for (uint32_t x = 0; x < src1.width; ++x) {
            // 检查局部差异（考虑平滑性）
            double total_diff = diff[y * src1.width + x];
            
            // 添加平滑约束（与上一行的接缝位置接近）
            if (y > 0) {
                total_diff += std::abs(static_cast<int>(x) - seam_positions[y - 1]) * 0.1;
            }
            
            if (total_diff < min_diff) {
                min_diff = total_diff;
                best_x = static_cast<int>(x);
            }
        }
        
        seam_positions[y] = best_x;
    }
}

/**
 * @brief 曝光补偿
 */
inline void compensate_exposure(ImageData& img, double gain, double bias = 0) {
    for (auto& pixel : img.data) {
        double val = pixel * gain + bias;
        pixel = static_cast<uint8_t>(std::max(0.0, std::min(255.0, val)));
    }
}

/**
 * @brief 计算图像平均亮度
 */
inline double compute_average_brightness(const ImageData& img) {
    if (img.empty()) return 0;
    
    double sum = 0;
    for (uint32_t i = 0; i < img.width * img.height; ++i) {
        if (img.channels == 1) {
            sum += img.data[i];
        } else if (img.channels >= 3) {
            // 使用灰度转换
            sum += 0.11 * img.data[i * 3] + 0.59 * img.data[i * 3 + 1] + 0.30 * img.data[i * 3 + 2];
        }
    }
    
    return sum / (img.width * img.height);
}

/**
 * @brief 多图像拼接（简单横向排列）
 */
inline void stitch_multi_horizontal(const std::vector<ImageData>& images,
                                    ImageData& result, int overlap = 100) {
    if (images.empty()) return;
    
    // 计算总宽度（考虑重叠）
    uint32_t total_width = 0;
    uint32_t max_height = 0;
    
    for (size_t i = 0; i < images.size(); ++i) {
        total_width += images[i].width;
        if (i > 0) total_width -= overlap;
        max_height = std::max(max_height, images[i].height);
    }
    
    result.width = total_width;
    result.height = max_height;
    result.channels = images[0].channels;
    result.format = images[0].format;
    result.data.resize(total_width * max_height * result.channels, 0);
    
    int offset_x = 0;
    for (size_t i = 0; i < images.size(); ++i) {
        const auto& img = images[i];
        
        for (uint32_t y = 0; y < img.height; ++y) {
            for (uint32_t x = 0; x < img.width; ++x) {
                int dst_x = offset_x + static_cast<int>(x);
                
                if (dst_x >= 0 && dst_x < static_cast<int>(total_width) &&
                    y < max_height) {
                    // 在重叠区域进行混合
                    double alpha = 1.0;
                    
                    if (i > 0 && x < overlap) {
                        alpha = static_cast<double>(x) / overlap;
                    } else if (i < images.size() - 1 && x > static_cast<int>(img.width) - overlap) {
                        alpha = static_cast<double>(img.width - x) / overlap;
                    }
                    
                    for (int c = 0; c < result.channels; ++c) {
                        size_t src_idx = (y * img.width + x) * img.channels + c;
                        size_t dst_idx = (y * total_width + dst_x) * result.channels + c;
                        
                        if (i == 0 || x >= overlap) {
                            result.data[dst_idx] = img.data[src_idx];
                        } else {
                            // 混合
                            result.data[dst_idx] = static_cast<uint8_t>(
                                result.data[dst_idx] * (1 - alpha) + img.data[src_idx] * alpha
                            );
                        }
                    }
                }
            }
        }
        
        offset_x += img.width - overlap;
    }
}

/**
 * @brief 马赛克拼接（网格排列）
 */
inline void stitch_mosaic(const std::vector<ImageData>& images,
                          ImageData& result, int cols, int rows, int spacing = 0) {
    if (images.empty() || cols <= 0 || rows <= 0) return;
    
    // 假设所有图像大小相同
    uint32_t tile_width = images[0].width;
    uint32_t tile_height = images[0].height;
    
    result.width = cols * tile_width + (cols - 1) * spacing;
    result.height = rows * tile_height + (rows - 1) * spacing;
    result.channels = images[0].channels;
    result.format = images[0].format;
    result.data.resize(result.width * result.height * result.channels, 0);
    
    for (size_t i = 0; i < images.size() && i < static_cast<size_t>(cols * rows); ++i) {
        int col = i % cols;
        int row = i / cols;
        
        int offset_x = col * (tile_width + spacing);
        int offset_y = row * (tile_height + spacing);
        
        const auto& img = images[i];
        
        for (uint32_t y = 0; y < img.height; ++y) {
            for (uint32_t x = 0; x < img.width; ++x) {
                int dst_x = offset_x + x;
                int dst_y = offset_y + y;
                
                if (dst_x >= 0 && dst_x < static_cast<int>(result.width) &&
                    dst_y >= 0 && dst_y < static_cast<int>(result.height)) {
                    for (int c = 0; c < result.channels; ++c) {
                        size_t src_idx = (y * img.width + x) * img.channels + c;
                        size_t dst_idx = (dst_y * result.width + dst_x) * result.channels + c;
                        result.data[dst_idx] = img.data[src_idx];
                    }
                }
            }
        }
    }
}

} // namespace stitch_utils

// ==================== 图像配准节点 ====================

/**
 * @brief 图像配准节点（特征点匹配）
 */
class ImageRegisterNode : public INode {
public:
    ImageRegisterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 特征点配准节点
 */
class FeatureMatchRegisterNode : public INode {
public:
    FeatureMatchRegisterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 单应性矩阵计算节点
 */
class HomographyCalcNode : public INode {
public:
    HomographyCalcNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 图像拼接节点 ====================

/**
 * @brief 图像拼接节点（全景拼接）
 */
class ImageStitchNode : public INode {
public:
    ImageStitchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 多图像拼接节点
 */
class MultiImageStitchNode : public INode {
public:
    MultiImageStitchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 全景拼接节点
 */
class PanoramaStitchNode : public INode {
public:
    PanoramaStitchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 马赛克拼接节点
 */
class MosaicStitchNode : public INode {
public:
    MosaicStitchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 图像融合节点 ====================

/**
 * @brief 图像融合节点（渐变混合）
 */
class ImageBlendNode : public INode {
public:
    ImageBlendNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 接缝融合节点
 */
class SeamBlendNode : public INode {
public:
    SeamBlendNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 曝光补偿融合节点
 */
class ExposureCompensateNode : public INode {
public:
    ExposureCompensateNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf