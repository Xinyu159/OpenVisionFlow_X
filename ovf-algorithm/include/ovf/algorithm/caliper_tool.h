/**
 * @file caliper_tool.h
 * @brief 卡尺测量工具（参考VisionPro的CogCaliperTool）
 * 
 * 包含约10个节点，提供边缘查找、宽度测量、距离测量等功能。
 * 纯C++实现，使用梯度方向搜索和亚像素定位技术。
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/measurement.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

/**
 * @brief 边缘极性
 */
enum class EdgePolarity {
    Positive = 1,   // 亮到暗（正梯度）
    Negative = -1,  // 暗到亮（负梯度）
    Both = 0         // 双向（任意梯度方向）
};

/**
 * @brief 边缘检测结果
 */
struct EdgeResult {
    float x = 0.0f;          // 边缘位置X（亚像素）
    float y = 0.0f;          // 边缘位置Y（亚像素）
    float strength = 0.0f;   // 边缘强度（梯度幅值）
    EdgePolarity polarity = EdgePolarity::Both;  // 边缘极性
    bool valid = false;      // 是否有效
    
    EdgeResult() = default;
    EdgeResult(float x_, float y_, float strength_, EdgePolarity pol, bool v = true)
        : x(x_), y(y_), strength(strength_), polarity(pol), valid(v) {}
};

/**
 * @brief 边缘对检测结果
 */
struct EdgePairResult {
    EdgeResult edge1;        // 第一个边缘
    EdgeResult edge2;        // 第二个边缘
    float distance = 0.0f;   // 两边缘距离
    float midpoint_x = 0.0f; // 中点X
    float midpoint_y = 0.0f; // 中点Y
    bool valid = false;      // 是否有效
};

/**
 * @brief 角点检测结果
 */
struct CornerResult {
    float x = 0.0f;          // 角点位置X
    float y = 0.0f;          // 角点位置Y
    float angle = 0.0f;      // 角度（度）
    float strength = 0.0f;   // 角点强度
    bool valid = false;
};

/**
 * @brief 搜索区域定义（矩形ROI）
 */
struct SearchRegion {
    float origin_x = 0.0f;    // 原点X
    float origin_y = 0.0f;    // 原点Y
    float width = 100.0f;     // 宽度
    float height = 20.0f;     // 高度
    float rotation = 0.0f;    // 旋转角度（度）
    
    // 获取四个角点
    std::vector<std::pair<float, float>> get_corners() const {
        float cos_r = std::cos(rotation * M_PI / 180.0f);
        float sin_r = std::sin(rotation * M_PI / 180.0f);
        
        float half_w = width / 2.0f;
        float half_h = height / 2.0f;
        
        std::vector<std::pair<float, float>> corners;
        corners.emplace_back(origin_x - half_w * cos_r - half_h * sin_r,
                             origin_y - half_w * sin_r + half_h * cos_r);
        corners.emplace_back(origin_x + half_w * cos_r - half_h * sin_r,
                             origin_y + half_w * sin_r + half_h * cos_r);
        corners.emplace_back(origin_x + half_w * cos_r + half_h * sin_r,
                             origin_y + half_w * sin_r - half_h * cos_r);
        corners.emplace_back(origin_x - half_w * cos_r + half_h * sin_r,
                             origin_y - half_w * sin_r - half_h * cos_r);
        return corners;
    }
};

/**
 * @brief 卡尺核心算法工具函数
 */
namespace caliper_utils {

/**
 * @brief 创建一维高斯核
 * @param sigma 标准差
 * @param kernel_size 核大小（如果为0则自动计算）
 * @return 高斯核系数
 */
inline std::vector<float> create_gaussian_kernel(float sigma, int kernel_size = 0) {
    if (kernel_size <= 0) {
        kernel_size = static_cast<int>(std::ceil(sigma * 6)) | 1; // 奇数
        if (kernel_size < 3) kernel_size = 3;
        if (kernel_size > 31) kernel_size = 31;
    }
    
    std::vector<float> kernel(kernel_size);
    int half = kernel_size / 2;
    float sum = 0.0f;
    
    for (int i = 0; i < kernel_size; ++i) {
        float x = static_cast<float>(i - half);
        float val = std::exp(-(x * x) / (2.0f * sigma * sigma));
        kernel[i] = val;
        sum += val;
    }
    
    // 归一化
    for (float& k : kernel) {
        k /= sum;
    }
    
    return kernel;
}

/**
 * @brief 平滑滤波（一维卷积）
 */
inline void smooth_profile(const std::vector<float>& input,
                           std::vector<float>& output,
                           float sigma) {
    if (input.empty()) {
        output.clear();
        return;
    }
    
    auto kernel = create_gaussian_kernel(sigma);
    int half = static_cast<int>(kernel.size()) / 2;
    int n = static_cast<int>(input.size());
    
    output.resize(n);
    
    for (int i = 0; i < n; ++i) {
        float sum = 0.0f;
        float weight_sum = 0.0f;
        
        for (int k = 0; k < static_cast<int>(kernel.size()); ++k) {
            int idx = i + k - half;
            if (idx >= 0 && idx < n) {
                sum += input[idx] * kernel[k];
                weight_sum += kernel[k];
            }
        }
        
        output[i] = (weight_sum > 0) ? sum / weight_sum : 0.0f;
    }
}

/**
 * @brief 计算梯度（一阶差分）
 */
inline void compute_gradient(const std::vector<float>& profile,
                             std::vector<float>& gradient) {
    int n = static_cast<int>(profile.size());
    gradient.resize(n);
    
    if (n < 2) {
        for (int i = 0; i < n; ++i) gradient[i] = 0.0f;
        return;
    }
    
    // 前向差分
    gradient[0] = profile[1] - profile[0];
    for (int i = 1; i < n - 1; ++i) {
        gradient[i] = (profile[i + 1] - profile[i - 1]) / 2.0f;
    }
    gradient[n - 1] = profile[n - 1] - profile[n - 2];
}

/**
 * @brief 亚像素边缘定位（二次拟合）
 * @param profile 灰度轮廓
 * @param gradient 梯度轮廓
 * @param peak_idx 梯度峰值索引
 * @return 亚像素位置（相对于数组起始）
 */
inline float subpixel_position(const std::vector<float>& profile,
                               const std::vector<float>& gradient,
                               int peak_idx) {
    int n = static_cast<int>(gradient.size());
    
    if (peak_idx <= 0 || peak_idx >= n - 1) {
        return static_cast<float>(peak_idx);
    }
    
    // 使用梯度二次拟合：g(x) = ax² + bx + c
    // 在峰值附近三点拟合，求极值点 x = -b/(2a)
    
    float g0 = gradient[peak_idx - 1];
    float g1 = gradient[peak_idx];
    float g2 = gradient[peak_idx + 1];
    
    // 三点二次拟合
    float a = (g0 + g2 - 2.0f * g1) / 2.0f;
    float b = (g2 - g0) / 2.0f;
    float c = g1;
    
    if (std::abs(a) < 1e-6f) {
        return static_cast<float>(peak_idx);
    }
    
    float subpixel_offset = -b / (2.0f * a);
    
    // 限制范围在 [-0.5, 0.5]
    if (subpixel_offset < -0.5f) subpixel_offset = -0.5f;
    if (subpixel_offset > 0.5f) subpixel_offset = 0.5f;
    
    return static_cast<float>(peak_idx) + subpixel_offset;
}

/**
 * @brief 亚像素边缘定位（泰勒展开法）
 */
inline float subpixel_position_taylor(const std::vector<float>& gradient,
                                      int peak_idx) {
    int n = static_cast<int>(gradient.size());
    
    if (peak_idx <= 0 || peak_idx >= n - 1) {
        return static_cast<float>(peak_idx);
    }
    
    float g0 = gradient[peak_idx - 1];
    float g1 = gradient[peak_idx];
    float g2 = gradient[peak_idx + 1];
    
    // 泰勒展开：在峰值点附近，一阶导数接近0
    // g'(x) ≈ (g2 - g0) / 2 + (g2 - 2g1 + g0) * (x - peak_idx)
    // 设 g'(x) = 0 求解亚像素偏移
    
    float second_deriv = g2 - 2.0f * g1 + g0;  // 二阶导数近似
    float first_deriv = (g2 - g0) / 2.0f;       // 一阶导数近似
    
    if (std::abs(second_deriv) < 1e-6f) {
        return static_cast<float>(peak_idx);
    }
    
    float offset = -first_deriv / second_deriv;
    
    // 限制范围
    if (offset < -0.5f) offset = -0.5f;
    if (offset > 0.5f) offset = 0.5f;
    
    return static_cast<float>(peak_idx) + offset;
}

/**
 * @brief 在ROI内提取投影轮廓
 * @param image 输入图像（灰度）
 * @param region 搜索区域
 * @param direction 搜索方向（角度，度）
 * @param profile 输出轮廓
 * @param positions 输出采样位置（X, Y）
 */
inline void extract_profile(const ImageData& image,
                            const SearchRegion& region,
                            float direction,
                            std::vector<float>& profile,
                            std::vector<std::pair<float, float>>& positions) {
    profile.clear();
    positions.clear();
    
    if (image.empty() || image.channels != 1) {
        return;
    }
    
    // 计算搜索方向的单位向量
    float dir_rad = direction * M_PI / 180.0f;
    float dir_x = std::cos(dir_rad);
    float dir_y = std::sin(dir_rad);
    
    // 垂直方向（用于计算投影宽度）
    float perp_x = -dir_y;
    float perp_y = dir_x;
    
    // 搜索长度 = ROI宽度
    float search_length = region.width;
    int num_samples = static_cast<int>(search_length);
    if (num_samples < 2) num_samples = 2;
    if (num_samples > 1000) num_samples = 1000;
    
    // 投影宽度 = ROI高度
    float projection_width = region.height;
    int half_width = static_cast<int>(projection_width / 2.0f);
    
    // 从ROI中心开始
    float start_x = region.origin_x - (region.width / 2.0f) * dir_x;
    float start_y = region.origin_y - (region.width / 2.0f) * dir_y;
    
    for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / (num_samples - 1);
        float cx = start_x + t * search_length * dir_x;
        float cy = start_y + t * search_length * dir_y;
        
        positions.emplace_back(cx, cy);
        
        // 在垂直方向上平均采样
        float sum = 0.0f;
        int count = 0;
        
        for (int w = -half_width; w <= half_width; ++w) {
            float px = cx + w * perp_x;
            float py = cy + w * perp_y;
            
            int ix = static_cast<int>(std::round(px));
            int iy = static_cast<int>(std::round(py));
            
            if (ix >= 0 && ix < static_cast<int>(image.width) &&
                iy >= 0 && iy < static_cast<int>(image.height)) {
                sum += image.data[iy * image.width + ix];
                ++count;
            }
        }
        
        profile.push_back((count > 0) ? sum / count : 0.0f);
    }
}

/**
 * @brief 查找单个边缘
 * @param profile 灰度轮廓
 * @param threshold 边缘阈值
 * @param polarity 边缘极性
 * @param smoothing 平滑强度
 * @return 边缘结果
 */
inline EdgeResult find_single_edge(const std::vector<float>& profile,
                                    float threshold,
                                    EdgePolarity polarity,
                                    float smoothing = 0.0f) {
    EdgeResult result;
    
    if (profile.size() < 3) {
        return result;
    }
    
    // 平滑处理
    std::vector<float> smoothed;
    if (smoothing > 0.0f) {
        smooth_profile(profile, smoothed, smoothing);
    } else {
        smoothed = profile;
    }
    
    // 计算梯度
    std::vector<float> gradient;
    compute_gradient(smoothed, gradient);
    
    // 寻找最强边缘
    float max_strength = 0.0f;
    int max_idx = -1;
    EdgePolarity detected_polarity = EdgePolarity::Both;
    
    for (int i = 1; i < static_cast<int>(gradient.size()) - 1; ++i) {
        float g = std::abs(gradient[i]);
        
        // 检查极性
        bool polarity_match = false;
        if (polarity == EdgePolarity::Both) {
            polarity_match = true;
        } else if (polarity == EdgePolarity::Positive && gradient[i] > threshold) {
            polarity_match = true;
        } else if (polarity == EdgePolarity::Negative && gradient[i] < -threshold) {
            polarity_match = true;
        }
        
        if (polarity_match && g > max_strength) {
            max_strength = g;
            max_idx = i;
            detected_polarity = (gradient[i] > 0) ? EdgePolarity::Positive : EdgePolarity::Negative;
        }
    }
    
    if (max_idx < 0 || max_strength < threshold) {
        return result;
    }
    
    // 亚像素定位
    float subpixel_pos = subpixel_position(smoothed, gradient, max_idx);
    
    result.strength = max_strength;
    result.polarity = detected_polarity;
    result.valid = true;
    
    // 将数组索引转换为相对位置（0-1范围）
    float normalized_pos = subpixel_pos / static_cast<float>(profile.size() - 1);
    result.x = normalized_pos;  // 相对位置，需要后续转换为实际坐标
    result.y = normalized_pos;
    
    return result;
}

/**
 * @brief 查找所有边缘
 * @param profile 灰度轮廓
 * @param threshold 边缘阈值
 * @param polarity 边缘极性
 * @param smoothing 平滑强度
 * @return 边缘列表（按强度排序）
 */
inline std::vector<EdgeResult> find_all_edges(const std::vector<float>& profile,
                                               float threshold,
                                               EdgePolarity polarity,
                                               float smoothing = 0.0f) {
    std::vector<EdgeResult> edges;
    
    if (profile.size() < 3) {
        return edges;
    }
    
    // 平滑处理
    std::vector<float> smoothed;
    if (smoothing > 0.0f) {
        smooth_profile(profile, smoothed, smoothing);
    } else {
        smoothed = profile;
    }
    
    // 计算梯度
    std::vector<float> gradient;
    compute_gradient(smoothed, gradient);
    
    // 寻找所有满足阈值的边缘
    for (int i = 1; i < static_cast<int>(gradient.size()) - 1; ++i) {
        float g = gradient[i];
        float abs_g = std::abs(g);
        
        // 检查极性和阈值
        bool valid_edge = false;
        EdgePolarity detected_pol = EdgePolarity::Both;
        
        if (polarity == EdgePolarity::Both && abs_g > threshold) {
            valid_edge = true;
            detected_pol = (g > 0) ? EdgePolarity::Positive : EdgePolarity::Negative;
        } else if (polarity == EdgePolarity::Positive && g > threshold) {
            valid_edge = true;
            detected_pol = EdgePolarity::Positive;
        } else if (polarity == EdgePolarity::Negative && g < -threshold) {
            valid_edge = true;
            detected_pol = EdgePolarity::Negative;
        }
        
        if (valid_edge) {
            // 检查是否是局部极值
            bool is_peak = (g > 0) ? 
                (g >= gradient[i - 1] && g >= gradient[i + 1]) :
                (g <= gradient[i - 1] && g <= gradient[i + 1]);
            
            if (is_peak) {
                float subpixel_pos = subpixel_position(smoothed, gradient, i);
                float normalized_pos = subpixel_pos / static_cast<float>(profile.size() - 1);
                
                EdgeResult edge;
                edge.strength = abs_g;
                edge.polarity = detected_pol;
                edge.valid = true;
                edge.x = normalized_pos;
                edge.y = normalized_pos;
                
                edges.push_back(edge);
            }
        }
    }
    
    // 按强度排序
    std::sort(edges.begin(), edges.end(), 
              [](const EdgeResult& a, const EdgeResult& b) {
                  return a.strength > b.strength;
              });
    
    return edges;
}

/**
 * @brief 查找边缘对（用于宽度测量）
 * @param profile 灰度轮廓
 * @param threshold 边缘阈值
 * @param polarity 边缘极性
 * @param smoothing 平滑强度
 * @return 边缘对结果
 */
inline EdgePairResult find_edge_pair(const std::vector<float>& profile,
                                      float threshold,
                                      EdgePolarity polarity,
                                      float smoothing = 0.0f) {
    EdgePairResult result;
    
    auto edges = find_all_edges(profile, threshold, polarity, smoothing);
    
    if (edges.size() < 2) {
        return result;
    }
    
    // 寻找最佳边缘对（通常是极性相反的两个最强边缘）
    EdgeResult* best_edge1 = nullptr;
    EdgeResult* best_edge2 = nullptr;
    
    if (polarity == EdgePolarity::Both) {
        // 寻找极性相反的边缘对
        for (size_t i = 0; i < edges.size(); ++i) {
            for (size_t j = i + 1; j < edges.size(); ++j) {
                if (edges[i].polarity != edges[j].polarity) {
                    best_edge1 = &edges[i];
                    best_edge2 = &edges[j];
                    break;
                }
            }
            if (best_edge1) break;
        }
        
        // 如果没有找到极性相反的边缘对，取最强的两个
        if (!best_edge1) {
            best_edge1 = &edges[0];
            best_edge2 = &edges[1];
        }
    } else {
        // 单极性模式，取最强的两个同极性边缘
        best_edge1 = &edges[0];
        best_edge2 = &edges[1];
    }
    
    if (best_edge1 && best_edge2) {
        result.edge1 = *best_edge1;
        result.edge2 = *best_edge2;
        
        // 计算距离（轮廓索引差）
        float pos1 = result.edge1.x * static_cast<float>(profile.size() - 1);
        float pos2 = result.edge2.x * static_cast<float>(profile.size() - 1);
        result.distance = std::abs(pos2 - pos1);
        
        result.midpoint_x = (result.edge1.x + result.edge2.x) / 2.0f;
        result.midpoint_y = (result.edge1.y + result.edge2.y) / 2.0f;
        result.valid = true;
    }
    
    return result;
}

/**
 * @brief 查找圆边缘（沿圆弧搜索）
 * @param image 输入图像
 * @param center_x 圆心X
 * @param center_y 圆心Y
 * @param radius 估计半径
 * @param angle_start 起始角度（度）
 * @param angle_end 结束角度（度）
 * @param threshold 边缘阈值
 * @param polarity 边缘极性
 * @return 边缘点列表
 */
inline std::vector<EdgeResult> find_circle_edges(const ImageData& image,
                                                  float center_x, float center_y,
                                                  float radius,
                                                  float angle_start, float angle_end,
                                                  float threshold,
                                                  EdgePolarity polarity) {
    std::vector<EdgeResult> edges;
    
    if (image.empty() || image.channels != 1) {
        return edges;
    }
    
    float a1 = angle_start * M_PI / 180.0f;
    float a2 = angle_end * M_PI / 180.0f;
    
    int num_angles = static_cast<int>(std::abs(a2 - a1) * radius);
    if (num_angles < 10) num_angles = 10;
    if (num_angles > 360) num_angles = 360;
    
    // 沿圆弧采样
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    
    for (int i = 0; i < num_angles; ++i) {
        float t = static_cast<float>(i) / (num_angles - 1);
        float angle = a1 + t * (a2 - a1);
        
        // 圆弧上的点
        float arc_x = center_x + radius * std::cos(angle);
        float arc_y = center_y + radius * std::sin(angle);
        
        // 沿径向方向采样（从圆心向外）
        float radial_x = std::cos(angle);
        float radial_y = std::sin(angle);
        
        float sum = 0.0f;
        int count = 0;
        
        // 径向采样范围
        int radial_range = 5;
        for (int r = -radial_range; r <= radial_range; ++r) {
            float px = arc_x + r * radial_x;
            float py = arc_y + r * radial_y;
            
            int ix = static_cast<int>(std::round(px));
            int iy = static_cast<int>(std::round(py));
            
            if (ix >= 0 && ix < static_cast<int>(image.width) &&
                iy >= 0 && iy < static_cast<int>(image.height)) {
                sum += image.data[iy * image.width + ix];
                ++count;
            }
        }
        
        profile.push_back((count > 0) ? sum / count : 0.0f);
        positions.emplace_back(arc_x, arc_y);
    }
    
    // 在轮廓中查找边缘
    auto detected_edges = find_all_edges(profile, threshold, polarity);
    
    // 将边缘转换为实际坐标
    for (const auto& edge : detected_edges) {
        int idx = static_cast<int>(edge.x * (num_angles - 1));
        if (idx >= 0 && idx < static_cast<int>(positions.size())) {
            EdgeResult actual_edge;
            actual_edge.x = positions[idx].first;
            actual_edge.y = positions[idx].second;
            actual_edge.strength = edge.strength;
            actual_edge.polarity = edge.polarity;
            actual_edge.valid = edge.valid;
            edges.push_back(actual_edge);
        }
    }
    
    return edges;
}

/**
 * @brief 查找角点（两条直线的交点）
 * @param image 输入图像
 * @param region1 第一条边的搜索区域
 * @param region2 第二条边的搜索区域
 * @param threshold 边缘阈值
 * @return 角点结果
 */
inline CornerResult find_corner(const ImageData& image,
                                const SearchRegion& region1,
                                const SearchRegion& region2,
                                float threshold) {
    CornerResult result;
    
    if (image.empty() || image.channels != 1) {
        return result;
    }
    
    // 在两个区域分别找边
    std::vector<float> profile1, profile2;
    std::vector<std::pair<float, float>> pos1, pos2;
    
    float direction1 = region1.rotation;
    float direction2 = region2.rotation;
    
    extract_profile(image, region1, direction1, profile1, pos1);
    extract_profile(image, region2, direction2, profile2, pos2);
    
    auto edge1 = find_single_edge(profile1, threshold, EdgePolarity::Both, 1.0f);
    auto edge2 = find_single_edge(profile2, threshold, EdgePolarity::Both, 1.0f);
    
    if (!edge1.valid || !edge2.valid) {
        return result;
    }
    
    // 计算边缘点坐标
    int idx1 = static_cast<int>(edge1.x * (pos1.size() - 1));
    int idx2 = static_cast<int>(edge2.x * (pos2.size() - 1));
    
    if (idx1 < 0 || idx1 >= static_cast<int>(pos1.size()) ||
        idx2 < 0 || idx2 >= static_cast<int>(pos2.size())) {
        return result;
    }
    
    // 计算两条直线的参数
    float x1 = pos1[idx1].first;
    float y1 = pos1[idx1].second;
    float dx1 = std::cos(direction1 * M_PI / 180.0f);
    float dy1 = std::sin(direction1 * M_PI / 180.0f);
    
    float x2 = pos2[idx2].first;
    float y2 = pos2[idx2].second;
    float dx2 = std::cos(direction2 * M_PI / 180.0f);
    float dy2 = std::sin(direction2 * M_PI / 180.0f);
    
    // 直线方程：ax + by + c = 0
    // 直线1: (dy1)x - (dx1)y + (dx1*y1 - dy1*x1) = 0
    float a1 = dy1;
    float b1 = -dx1;
    float c1 = dx1 * y1 - dy1 * x1;
    
    // 直线2: (dy2)x - (dx2)y + (dx2*y2 - dy2*x2) = 0
    float a2 = dy2;
    float b2 = -dx2;
    float c2 = dx2 * y2 - dy2 * x2;
    
    // 求交点
    float det = a1 * b2 - a2 * b1;
    if (std::abs(det) < 1e-6f) {
        return result; // 平行线
    }
    
    result.x = (b1 * c2 - b2 * c1) / det;
    result.y = (a2 * c1 - a1 * c2) / det;
    
    // 计算夹角
    float dot = dx1 * dx2 + dy1 * dy2;
    float cross = dx1 * dy2 - dy1 * dx2;
    result.angle = std::atan2(std::abs(cross), std::abs(dot)) * 180.0f / M_PI;
    
    result.strength = edge1.strength + edge2.strength;
    result.valid = true;
    
    return result;
}

/**
 * @brief 拟合直线（从边缘点）
 * @param edges 边缘点列表
 * @return 直线参数 (a, b, c) for ax + by + c = 0
 */
inline std::tuple<float, float, float> fit_line_from_edges(
    const std::vector<EdgeResult>& edges) {
    
    if (edges.size() < 2) {
        return std::make_tuple(0.0f, 1.0f, 0.0f);
    }
    
    std::vector<Point2Df> points;
    for (const auto& edge : edges) {
        if (edge.valid) {
            points.emplace_back(edge.x, edge.y);
        }
    }
    
    if (points.size() < 2) {
        return std::make_tuple(0.0f, 1.0f, 0.0f);
    }
    
    // 使用最小二乘法拟合
    Line2D line = measurement_utils::fit_line(points);
    
    return std::make_tuple(line.a, line.b, line.c);
}

} // namespace caliper_utils

// ============================================================================
// 节点类定义
// ============================================================================

/**
 * @brief 卡尺找边节点（单边）
 */
class CaliperFindEdgeNode : public INode {
public:
    CaliperFindEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺找边对节点
 */
class CaliperFindEdgePairNode : public INode {
public:
    CaliperFindEdgePairNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺测量宽度节点
 */
class CaliperMeasureWidthNode : public INode {
public:
    CaliperMeasureWidthNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺测量距离节点
 */
class CaliperMeasureDistanceNode : public INode {
public:
    CaliperMeasureDistanceNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺找圆边缘节点
 */
class CaliperFindCircleEdgeNode : public INode {
public:
    CaliperFindCircleEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺找角点节点
 */
class CaliperFindCornerNode : public INode {
public:
    CaliperFindCornerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺亚像素边缘节点
 */
class CaliperSubpixelEdgeNode : public INode {
public:
    CaliperSubpixelEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 多边缘卡尺节点
 */
class CaliperMultiEdgeNode : public INode {
public:
    CaliperMultiEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 投影边缘卡尺节点
 */
class CaliperProjectEdgeNode : public INode {
public:
    CaliperProjectEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 卡尺拟合直线节点
 */
class CaliperFitLineNode : public INode {
public:
    CaliperFitLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf