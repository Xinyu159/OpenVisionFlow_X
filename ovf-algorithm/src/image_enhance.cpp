/**
 * @file image_enhance.cpp
 * @brief 图像增强算子实现（纯C++实现，不依赖OpenCV）
 */

#include "ovf/algorithm/image_enhance.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <cstring>

namespace ovf {
namespace algorithm {

// ========== 辅助函数实现 ==========

namespace enhance_utils {

std::vector<double> create_gaussian_kernel(int size, double sigma) {
    std::vector<double> kernel(size * size);
    int half = size / 2;
    double sum = 0.0;
    
    for (int y = -half; y <= half; ++y) {
        for (int x = -half; x <= half; ++x) {
            double val = std::exp(-(x*x + y*y) / (2.0 * sigma * sigma));
            kernel[(y + half) * size + (x + half)] = val;
            sum += val;
        }
    }
    
    // 归一化
    for (auto& v : kernel) {
        v /= sum;
    }
    
    return kernel;
}

std::vector<int> compute_histogram(const uint8_t* data, int size) {
    std::vector<int> hist(256, 0);
    for (int i = 0; i < size; ++i) {
        hist[data[i]]++;
    }
    return hist;
}

std::vector<double> compute_cdf(const std::vector<int>& histogram, int total_pixels) {
    std::vector<double> cdf(256);
    double cumulative = 0.0;
    for (int i = 0; i < 256; ++i) {
        cumulative += histogram[i];
        cdf[i] = cumulative / total_pixels;
    }
    return cdf;
}

// ========== RGB <-> HSV 转换 ==========

void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, double& h, double& s, double& v) {
    double rn = r / 255.0;
    double gn = g / 255.0;
    double bn = b / 255.0;
    
    double max_val = std::max({rn, gn, bn});
    double min_val = std::min({rn, gn, bn});
    double delta = max_val - min_val;
    
    v = max_val;
    
    if (delta < 1e-6) {
        h = 0.0;
        s = 0.0;
        return;
    }
    
    s = delta / max_val;
    
    if (max_val == rn) {
        h = 60.0 * ((gn - bn) / delta);
    } else if (max_val == gn) {
        h = 60.0 * (2.0 + (bn - rn) / delta);
    } else {
        h = 60.0 * (4.0 + (rn - gn) / delta);
    }
    
    if (h < 0.0) h += 360.0;
}

void hsv_to_rgb(double h, double s, double v, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (s < 1e-6) {
        r = g = b = static_cast<uint8_t>(v * 255.0);
        return;
    }
    
    h = std::fmod(h, 360.0);
    if (h < 0.0) h += 360.0;
    
    int sector = static_cast<int>(h / 60.0);
    double f = (h / 60.0) - sector;
    
    double p = v * (1.0 - s);
    double q = v * (1.0 - s * f);
    double t = v * (1.0 - s * (1.0 - f));
    
    double rn, gn, bn;
    
    switch (sector) {
        case 0:  rn = v; gn = t; bn = p; break;
        case 1:  rn = q; gn = v; bn = p; break;
        case 2:  rn = p; gn = v; bn = t; break;
        case 3:  rn = p; gn = q; bn = v; break;
        case 4:  rn = t; gn = p; bn = v; break;
        default: rn = v; gn = p; bn = q; break;
    }
    
    r = static_cast<uint8_t>(std::clamp(rn * 255.0, 0.0, 255.0));
    g = static_cast<uint8_t>(std::clamp(gn * 255.0, 0.0, 255.0));
    b = static_cast<uint8_t>(std::clamp(bn * 255.0, 0.0, 255.0));
}

// ========== 滤波器实现 ==========

// 快速中值滤波 - 使用直方图优化
void median_filter(const uint8_t* src, uint8_t* dst, int width, int height, 
                   int kernel_size, int channels) {
    int half = kernel_size / 2;
    int median_pos = (kernel_size * kernel_size) / 2;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                // 使用直方图方法计算中值
                std::vector<int> hist(256, 0);
                
                for (int ky = -half; ky <= half; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -half; kx <= half; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        uint8_t val = src[(sy * width + sx) * channels + c];
                        hist[val]++;
                    }
                }
                
                // 找到中值
                int count = 0;
                uint8_t median = 0;
                for (int i = 0; i < 256; ++i) {
                    count += hist[i];
                    if (count > median_pos) {
                        median = static_cast<uint8_t>(i);
                        break;
                    }
                }
                
                dst[(y * width + x) * channels + c] = median;
            }
        }
    }
}

// 双边滤波
void bilateral_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                      int channels, int d, double sigma_color, double sigma_space) {
    int half = d / 2;
    double color_coeff = -0.5 / (sigma_color * sigma_color);
    double space_coeff = -0.5 / (sigma_space * sigma_space);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                double sum = 0.0;
                double wsum = 0.0;
                uint8_t center_val = src[(y * width + x) * channels + c];
                
                for (int ky = -half; ky <= half; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -half; kx <= half; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        uint8_t val = src[(sy * width + sx) * channels + c];
                        
                        // 空间权重
                        double spatial_dist = ky * ky + kx * kx;
                        double space_weight = std::exp(spatial_dist * space_coeff);
                        
                        // 颜色权重
                        double color_dist = static_cast<double>(val) - center_val;
                        double color_weight = std::exp(color_dist * color_dist * color_coeff);
                        
                        double weight = space_weight * color_weight;
                        sum += val * weight;
                        wsum += weight;
                    }
                }
                
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(std::clamp(sum / wsum, 0.0, 255.0));
            }
        }
    }
}

// 导向滤波（简化版）
void guided_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                   int channels, int radius, double epsilon) {
    // 导向滤波：假设引导图像就是输入图像本身
    
    // 计算均值
    std::vector<double> mean(width * height * channels, 0.0);
    int area = (2 * radius + 1) * (2 * radius + 1);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                double sum = 0.0;
                for (int ky = -radius; ky <= radius; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -radius; kx <= radius; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        sum += src[(sy * width + sx) * channels + c];
                    }
                }
                mean[(y * width + x) * channels + c] = sum / area;
            }
        }
    }
    
    // 计算方差和协方差
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                double m = mean[(y * width + x) * channels + c];
                
                // 计算方差
                double var_sum = 0.0;
                for (int ky = -radius; ky <= radius; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -radius; kx <= radius; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        double val = src[(sy * width + sx) * channels + c];
                        var_sum += (val - m) * (val - m);
                    }
                }
                double var = var_sum / area;
                
                // 计算系数
                double a = var / (var + epsilon);
                double b = (1.0 - a) * m;
                
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(
                    std::clamp(a * src[(y * width + x) * channels + c] + b, 0.0, 255.0));
            }
        }
    }
}

// 高斯滤波
void gaussian_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                     int channels, int kernel_size, double sigma) {
    auto kernel = create_gaussian_kernel(kernel_size, sigma);
    int half = kernel_size / 2;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                double sum = 0.0;
                
                for (int ky = -half; ky <= half; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -half; kx <= half; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        double val = src[(sy * width + sx) * channels + c];
                        sum += val * kernel[(ky + half) * kernel_size + (kx + half)];
                    }
                }
                
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(std::clamp(sum, 0.0, 255.0));
            }
        }
    }
}

// 方框滤波/均值滤波
void box_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                int channels, int kernel_size) {
    int half = kernel_size / 2;
    int area = kernel_size * kernel_size;
    
    // 使用积分图像加速（简化版：直接计算）
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                int sum = 0;
                
                for (int ky = -half; ky <= half; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -half; kx <= half; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        sum += src[(sy * width + sx) * channels + c];
                    }
                }
                
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(sum / area);
            }
        }
    }
}

// 拉普拉斯滤波
void laplacian_filter(const uint8_t* src, uint8_t* dst, int width, int height, int channels) {
    // 拉普拉斯算子: [0 1 0; 1 -4 1; 0 1 0]
    
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            for (int c = 0; c < channels; ++c) {
                int center = src[(y * width + x) * channels + c];
                int top = src[((y - 1) * width + x) * channels + c];
                int bottom = src[((y + 1) * width + x) * channels + c];
                int left = src[(y * width + (x - 1)) * channels + c];
                int right = src[(y * width + (x + 1)) * channels + c];
                
                int laplacian = -4 * center + top + bottom + left + right;
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(
                    std::clamp(std::abs(laplacian), 0, 255));
            }
        }
    }
    
    // 处理边界
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (y == 0 || y == height - 1 || x == 0 || x == width - 1) {
                for (int c = 0; c < channels; ++c) {
                    dst[(y * width + x) * channels + c] = src[(y * width + x) * channels + c];
                }
            }
        }
    }
}

// 锐化滤波
void sharpen_filter(const uint8_t* src, uint8_t* dst, int width, int height,
                    int channels, double strength) {
    // 锐化：原图 + strength * 拉普拉斯
    
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            for (int c = 0; c < channels; ++c) {
                int center = src[(y * width + x) * channels + c];
                int top = src[((y - 1) * width + x) * channels + c];
                int bottom = src[((y + 1) * width + x) * channels + c];
                int left = src[(y * width + (x - 1)) * channels + c];
                int right = src[(y * width + (x + 1)) * channels + c];
                
                int laplacian = -4 * center + top + bottom + left + right;
                int sharpened = center + static_cast<int>(strength * laplacian);
                
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(std::clamp(sharpened, 0, 255));
            }
        }
    }
    
    // 处理边界
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (y == 0 || y == height - 1 || x == 0 || x == width - 1) {
                for (int c = 0; c < channels; ++c) {
                    dst[(y * width + x) * channels + c] = src[(y * width + x) * channels + c];
                }
            }
        }
    }
}

// 反锐化掩蔽
void unsharp_mask(const uint8_t* src, uint8_t* dst, int width, int height,
                  int channels, int radius, double amount, double threshold) {
    // 先进行高斯模糊
    std::vector<uint8_t> blurred(width * height * channels);
    gaussian_filter(src, blurred.data(), width, height, channels, radius * 2 + 1, radius / 3.0);
    
    // 反锐化掩蔽：result = original + amount * (original - blurred)
    for (int i = 0; i < width * height * channels; ++i) {
        int diff = src[i] - blurred[i];
        
        // 只有差异大于阈值时才增强
        if (std::abs(diff) > threshold) {
            int result = src[i] + static_cast<int>(amount * diff);
            dst[i] = static_cast<uint8_t>(std::clamp(result, 0, 255));
        } else {
            dst[i] = src[i];
        }
    }
}

// ========== 去噪实现 ==========

// 非局部均值去噪（简化版）
void nlm_denoise(const uint8_t* src, uint8_t* dst, int width, int height,
                 int channels, int search_window, int template_size, double h) {
    int half_search = search_window / 2;
    int half_template = template_size / 2;
    double h2 = h * h;
    
    // 简化实现：仅处理灰度图像或逐通道处理
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                double sum = 0.0;
                double wsum = 0.0;
                
                for (int sy = -half_search; sy <= half_search; ++sy) {
                    int search_y = std::clamp(y + sy, 0, height - 1);
                    
                    for (int sx = -half_search; sx <= half_search; ++sx) {
                        int search_x = std::clamp(x + sx, 0, width - 1);
                        
                        // 计算模板距离
                        double dist = 0.0;
                        int count = 0;
                        
                        for (int ty = -half_template; ty <= half_template; ++ty) {
                            int y1 = std::clamp(y + ty, 0, height - 1);
                            int y2 = std::clamp(search_y + ty, 0, height - 1);
                            
                            for (int tx = -half_template; tx <= half_template; ++tx) {
                                int x1 = std::clamp(x + tx, 0, width - 1);
                                int x2 = std::clamp(search_x + tx, 0, width - 1);
                                
                                double diff = static_cast<double>(src[(y1 * width + x1) * channels + c]) -
                                             static_cast<double>(src[(y2 * width + x2) * channels + c]);
                                dist += diff * diff;
                                count++;
                            }
                        }
                        
                        // 计算权重
                        double weight = std::exp(-dist / (count * h2));
                        sum += weight * src[(search_y * width + search_x) * channels + c];
                        wsum += weight;
                    }
                }
                
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(std::clamp(sum / wsum, 0.0, 255.0));
            }
        }
    }
}

// 自适应去噪 - 根据局部方差调整
void adaptive_denoise(const uint8_t* src, uint8_t* dst, int width, int height,
                      int channels, int window_size, double noise_level) {
    int half = window_size / 2;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                // 计算局部均值和方差
                double sum = 0.0;
                double sum2 = 0.0;
                int count = 0;
                
                for (int ky = -half; ky <= half; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -half; kx <= half; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        double val = src[(sy * width + sx) * channels + c];
                        sum += val;
                        sum2 += val * val;
                        count++;
                    }
                }
                
                double mean = sum / count;
                double variance = sum2 / count - mean * mean;
                
                // 根据方差调整滤波强度
                // 如果局部方差小于噪声方差，说明该区域较平坦，需要更强的滤波
                double filter_strength = std::min(1.0, variance / (noise_level * noise_level));
                
                // 应用加权滤波
                double filtered_sum = 0.0;
                double weight_sum = 0.0;
                
                for (int ky = -half; ky <= half; ++ky) {
                    int sy = std::clamp(y + ky, 0, height - 1);
                    for (int kx = -half; kx <= half; ++kx) {
                        int sx = std::clamp(x + kx, 0, width - 1);
                        double val = src[(sy * width + sx) * channels + c];
                        
                        // 权重：平坦区域权重更均匀，纹理区域权重更偏向中心
                        double dist2 = ky * ky + kx * kx;
                        double weight = std::exp(-dist2 / (2.0 * (1.0 + filter_strength)));
                        
                        filtered_sum += val * weight;
                        weight_sum += weight;
                    }
                }
                
                double filtered_val = filtered_sum / weight_sum;
                
                // 混合原始值和滤波值
                double result = mean * (1.0 - filter_strength) + filtered_val * filter_strength;
                dst[(y * width + x) * channels + c] = static_cast<uint8_t>(std::clamp(result, 0.0, 255.0));
            }
        }
    }
}

// ========== 增强实现 ==========

// 对比度增强
void contrast_enhance(const uint8_t* src, uint8_t* dst, int width, int height,
                      int channels, double factor) {
    // 对比度增强：new_val = (old_val - 128) * factor + 128
    double center = 128.0;
    
    for (int i = 0; i < width * height * channels; ++i) {
        double val = (static_cast<double>(src[i]) - center) * factor + center;
        dst[i] = static_cast<uint8_t>(std::clamp(val, 0.0, 255.0));
    }
}

// 亮度调整
void brightness_adjust(const uint8_t* src, uint8_t* dst, int width, int height,
                       int channels, int delta) {
    for (int i = 0; i < width * height * channels; ++i) {
        int val = src[i] + delta;
        dst[i] = static_cast<uint8_t>(std::clamp(val, 0, 255));
    }
}

// Gamma校正
void gamma_correct(const uint8_t* src, uint8_t* dst, int width, int height,
                   int channels, double gamma) {
    // 构建查找表加速
    std::vector<uint8_t> lut(256);
    double inv_gamma = 1.0 / gamma;
    
    for (int i = 0; i < 256; ++i) {
        double val = std::pow(i / 255.0, inv_gamma) * 255.0;
        lut[i] = static_cast<uint8_t>(std::clamp(val, 0.0, 255.0));
    }
    
    for (int i = 0; i < width * height * channels; ++i) {
        dst[i] = lut[src[i]];
    }
}

// 直方图均衡化（全局）
void histogram_equalize(const uint8_t* src, uint8_t* dst, int width, int height, int channels) {
    if (channels == 1) {
        // 灰度图像
        auto hist = compute_histogram(src, width * height);
        auto cdf = compute_cdf(hist, width * height);
        
        // 构建查找表
        std::vector<uint8_t> lut(256);
        for (int i = 0; i < 256; ++i) {
            lut[i] = static_cast<uint8_t>(cdf[i] * 255.0);
        }
        
        for (int i = 0; i < width * height; ++i) {
            dst[i] = lut[src[i]];
        }
    } else {
        // RGB图像：分别对各通道进行均衡化，或转换为亮度
        for (int c = 0; c < channels; ++c) {
            std::vector<uint8_t> channel_data(width * height);
            for (int i = 0; i < width * height; ++i) {
                channel_data[i] = src[i * channels + c];
            }
            
            auto hist = compute_histogram(channel_data.data(), width * height);
            auto cdf = compute_cdf(hist, width * height);
            
            std::vector<uint8_t> lut(256);
            for (int i = 0; i < 256; ++i) {
                lut[i] = static_cast<uint8_t>(cdf[i] * 255.0);
            }
            
            for (int i = 0; i < width * height; ++i) {
                dst[i * channels + c] = lut[channel_data[i]];
            }
        }
    }
}

// 自适应直方图均衡（CLAHE简化）
void adaptive_histogram_equalize(const uint8_t* src, uint8_t* dst, int width, int height,
                                  int channels, int clip_limit, int tile_size) {
    // 将图像分成小块，对每个小块进行直方图均衡化
    int tiles_x = width / tile_size;
    int tiles_y = height / tile_size;
    
    if (tiles_x < 1) tiles_x = 1;
    if (tiles_y < 1) tiles_y = 1;
    
    // 处理每个tile
    for (int ty = 0; ty < tiles_y; ++ty) {
        for (int tx = 0; tx < tiles_x; ++tx) {
            int x_start = tx * tile_size;
            int y_start = ty * tile_size;
            int x_end = std::min(x_start + tile_size, width);
            int y_end = std::min(y_start + tile_size, height);
            
            int tile_pixels = (x_end - x_start) * (y_end - y_start);
            
            // 计算tile的直方图
            for (int c = 0; c < std::min(channels, 1); ++c) {
                std::vector<int> hist(256, 0);
                
                for (int y = y_start; y < y_end; ++y) {
                    for (int x = x_start; x < x_end; ++x) {
                        uint8_t val;
                        if (channels == 1) {
                            val = src[y * width + x];
                        } else {
                            // 使用亮度值
                            int idx = (y * width + x) * channels;
                            val = static_cast<uint8_t>(
                                (src[idx] + src[idx + 1] + src[idx + 2]) / 3);
                        }
                        hist[val]++;
                    }
                }
                
                // Clip histogram
                int excess = 0;
                for (int i = 0; i < 256; ++i) {
                    if (hist[i] > clip_limit) {
                        excess += hist[i] - clip_limit;
                        hist[i] = clip_limit;
                    }
                }
                
                // 重新分配超出部分
                int avg_excess = excess / 256;
                for (int i = 0; i < 256; ++i) {
                    hist[i] += avg_excess;
                }
                
                // 计算CDF
                auto cdf = compute_cdf(hist, tile_pixels);
                
                // 应用均衡化
                for (int y = y_start; y < y_end; ++y) {
                    for (int x = x_start; x < x_end; ++x) {
                        if (channels == 1) {
                            uint8_t val = src[y * width + x];
                            dst[y * width + x] = static_cast<uint8_t>(cdf[val] * 255.0);
                        } else {
                            // 对RGB各通道应用相同的映射
                            int idx = (y * width + x) * channels;
                            int gray = (src[idx] + src[idx + 1] + src[idx + 2]) / 3;
                            double factor = cdf[gray];
                            
                            for (int ch = 0; ch < 3; ++ch) {
                                dst[idx + ch] = static_cast<uint8_t>(
                                    std::clamp(src[idx + ch] * factor, 0.0, 255.0));
                            }
                        }
                    }
                }
            }
        }
    }
    
    // 处理边界tile（简化：直接复制）
    for (int y = tiles_y * tile_size; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                dst[(y * width + x) * channels + c] = src[(y * width + x) * channels + c];
            }
        }
    }
    
    for (int y = 0; y < tiles_y * tile_size; ++y) {
        for (int x = tiles_x * tile_size; x < width; ++x) {
            for (int c = 0; c < channels; ++c) {
                dst[(y * width + x) * channels + c] = src[(y * width + x) * channels + c];
            }
        }
    }
}

// ========== 彩调整实现 ==========

// 饱和度调整
void saturation_adjust(const uint8_t* src, uint8_t* dst, int width, int height, double factor) {
    for (int i = 0; i < width * height; ++i) {
        int idx = i * 3;
        double r = src[idx];
        double g = src[idx + 1];
        double b = src[idx + 2];
        
        // 转换到HSV
        double h, s, v;
        rgb_to_hsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), 
                   static_cast<uint8_t>(b), h, s, v);
        
        // 调整饱和度
        s = std::clamp(s * factor, 0.0, 1.0);
        
        // 转换回RGB
        uint8_t nr, ng, nb;
        hsv_to_rgb(h, s, v, nr, ng, nb);
        
        dst[idx] = nr;
        dst[idx + 1] = ng;
        dst[idx + 2] = nb;
    }
}

// 色相偏移
void hue_shift(const uint8_t* src, uint8_t* dst, int width, int height, double delta_h) {
    for (int i = 0; i < width * height; ++i) {
        int idx = i * 3;
        
        double h, s, v;
        rgb_to_hsv(src[idx], src[idx + 1], src[idx + 2], h, s, v);
        
        // 偏移色相
        h = std::fmod(h + delta_h, 360.0);
        if (h < 0.0) h += 360.0;
        
        uint8_t nr, ng, nb;
        hsv_to_rgb(h, s, v, nr, ng, nb);
        
        dst[idx] = nr;
        dst[idx + 1] = ng;
        dst[idx + 2] = nb;
    }
}

// 白平衡校正
void white_balance(const uint8_t* src, uint8_t* dst, int width, int height, const String& method) {
    if (method == "gray_world") {
        // Gray World假设：平均RGB值应该相等
        double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
        int count = width * height;
        
        for (int i = 0; i < count; ++i) {
            int idx = i * 3;
            sum_r += src[idx];
            sum_g += src[idx + 1];
            sum_b += src[idx + 2];
        }
        
        sum_r /= count;
        sum_g /= count;
        sum_b /= count;
        
        // 计算校正系数
        double avg = (sum_r + sum_g + sum_b) / 3.0;
        double kr = avg / (sum_r > 0 ? sum_r : 1.0);
        double kg = avg / (sum_g > 0 ? sum_g : 1.0);
        double kb = avg / (sum_b > 0 ? sum_b : 1.0);
        
        // 应用校正
        for (int i = 0; i < count; ++i) {
            int idx = i * 3;
            dst[idx] = static_cast<uint8_t>(std::clamp(src[idx] * kr, 0.0, 255.0));
            dst[idx + 1] = static_cast<uint8_t>(std::clamp(src[idx + 1] * kg, 0.0, 255.0));
            dst[idx + 2] = static_cast<uint8_t>(std::clamp(src[idx + 2] * kb, 0.0, 255.0));
        }
    } else if (method == "max_white") {
        // 最大值白平衡：假设最大值对应白色
        uint8_t max_r = 0, max_g = 0, max_b = 0;
        
        for (int i = 0; i < width * height; ++i) {
            int idx = i * 3;
            max_r = std::max(max_r, src[idx]);
            max_g = std::max(max_g, src[idx + 1]);
            max_b = std::max(max_b, src[idx + 2]);
        }
        
        double kr = 255.0 / (max_r > 0 ? max_r : 1.0);
        double kg = 255.0 / (max_g > 0 ? max_g : 1.0);
        double kb = 255.0 / (max_b > 0 ? max_b : 1.0);
        
        // 限制最大校正系数
        kr = std::min(kr, 3.0);
        kg = std::min(kg, 3.0);
        kb = std::min(kb, 3.0);
        
        for (int i = 0; i < width * height; ++i) {
            int idx = i * 3;
            dst[idx] = static_cast<uint8_t>(std::clamp(src[idx] * kr, 0.0, 255.0));
            dst[idx + 1] = static_cast<uint8_t>(std::clamp(src[idx + 1] * kg, 0.0, 255.0));
            dst[idx + 2] = static_cast<uint8_t>(std::clamp(src[idx + 2] * kb, 0.0, 255.0));
        }
    } else {
        // 默认：直接复制
        std::copy(src, src + width * height * 3, dst);
    }
}

// ========== 小波去噪简化版（Haar小波） ==========

void wavelet_denoise_simple(const uint8_t* src, uint8_t* dst, int width, int height,
                             int channels, double threshold) {
    // 简化版：仅做一级Haar小波分解和阈值处理
    
    for (int c = 0; c < channels; ++c) {
        // Haar小波分解（简化）
        std::vector<double> low_pass(width * height / 4);
        std::vector<double> high_pass_h(width * height / 4);
        std::vector<double> high_pass_v(width * height / 4);
        std::vector<double> high_pass_d(width * height / 4);
        
        // 简化的分解
        int half_w = width / 2;
        int half_h = height / 2;
        
        for (int y = 0; y < half_h; ++y) {
            for (int x = 0; x < half_w; ++x) {
                int idx = (y * 2 * width + x * 2) * channels + c;
                double a = src[idx];
                double b = src[idx + channels];
                double c_val = src[((y * 2 + 1) * width + x * 2) * channels + c];
                double d = src[((y * 2 + 1) * width + x * 2 + 1) * channels + c];
                
                // Haar小波系数
                low_pass[y * half_w + x] = (a + b + c_val + d) / 4.0;
                high_pass_h[y * half_w + x] = (a - b + c_val - d) / 4.0;
                high_pass_v[y * half_w + x] = (a + b - c_val - d) / 4.0;
                high_pass_d[y * half_w + x] = (a - b - c_val + d) / 4.0;
            }
        }
        
        // 阈值处理高频系数
        for (int i = 0; i < low_pass.size(); ++i) {
            if (std::abs(high_pass_h[i]) < threshold) high_pass_h[i] = 0;
            if (std::abs(high_pass_v[i]) < threshold) high_pass_v[i] = 0;
            if (std::abs(high_pass_d[i]) < threshold) high_pass_d[i] = 0;
        }
        
        // 重构
        for (int y = 0; y < half_h; ++y) {
            for (int x = 0; x < half_w; ++x) {
                double ll = low_pass[y * half_w + x];
                double hl = high_pass_h[y * half_w + x];
                double lh = high_pass_v[y * half_w + x];
                double hh = high_pass_d[y * half_w + x];
                
                int idx1 = (y * 2 * width + x * 2) * channels + c;
                int idx2 = (y * 2 * width + x * 2 + 1) * channels + c;
                int idx3 = ((y * 2 + 1) * width + x * 2) * channels + c;
                int idx4 = ((y * 2 + 1) * width + x * 2 + 1) * channels + c;
                
                dst[idx1] = static_cast<uint8_t>(std::clamp(ll + hl + lh + hh, 0.0, 255.0));
                dst[idx2] = static_cast<uint8_t>(std::clamp(ll - hl + lh - hh, 0.0, 255.0));
                dst[idx3] = static_cast<uint8_t>(std::clamp(ll + hl - lh - hh, 0.0, 255.0));
                dst[idx4] = static_cast<uint8_t>(std::clamp(ll - hl - lh + hh, 0.0, 255.0));
            }
        }
    }
}

} // namespace enhance_utils

// ========== 滤波器节点实现（8个） ==========

// 1. MedianFilterNode
MedianFilterNode::MedianFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MedianFilterNode::make_info() {
    NodeInfo info;
    info.id = "MedianFilter";
    info.name = "中值滤波";
    info.category = "图像增强/滤波";
    info.description = "使用快速中值算法去除椒盐噪声";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("kernel_size", "核大小", DataType::Number, Data(3)));
    
    return info;
}

Result<void> MedianFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int kernel_size = static_cast<int>(get_param("kernel_size", Data(3)).as_int());
    if (kernel_size % 2 == 0) kernel_size++;
    kernel_size = std::max(3, std::min(kernel_size, 15));
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::median_filter(input.data.data(), output.data.data(),
                                  input.width, input.height, kernel_size, input.channels);
    
    set_output("image", Data(output));
    OVF_INFO() << "MedianFilter executed: kernel_size=" << kernel_size;
    
    return Result<void>::success();
}

// 2. BilateralFilterNode
BilateralFilterNode::BilateralFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BilateralFilterNode::make_info() {
    NodeInfo info;
    info.id = "BilateralFilter";
    info.name = "双边滤波";
    info.category = "图像增强/滤波";
    info.description = "保边缘去噪滤波";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("d", "滤波直径", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("sigma_color", "颜色标准差", DataType::Number, Data(75.0)));
    info.params.push_back(ParamDef("sigma_space", "空间标准差", DataType::Number, Data(75.0)));
    
    return info;
}

Result<void> BilateralFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int d = static_cast<int>(get_param("d", Data(5)).as_int());
    double sigma_color = get_param("sigma_color", Data(75.0)).as_number();
    double sigma_space = get_param("sigma_space", Data(75.0)).as_number();
    
    if (d <= 0) d = 5;
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::bilateral_filter(input.data.data(), output.data.data(),
                                     input.width, input.height, input.channels,
                                     d, sigma_color, sigma_space);
    
    set_output("image", Data(output));
    OVF_INFO() << "BilateralFilter executed: d=" << d;
    
    return Result<void>::success();
}

// 3. GuidedFilterNode
GuidedFilterNode::GuidedFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GuidedFilterNode::make_info() {
    NodeInfo info;
    info.id = "GuidedFilter";
    info.name = "导向滤波";
    info.category = "图像增强/滤波";
    info.description = "导向滤波，保边缘平滑";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("radius", "滤波半径", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("epsilon", "正则化参数", DataType::Number, Data(0.01)));
    
    return info;
}

Result<void> GuidedFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int radius = static_cast<int>(get_param("radius", Data(8)).as_int());
    double epsilon = get_param("epsilon", Data(0.01)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::guided_filter(input.data.data(), output.data.data(),
                                  input.width, input.height, input.channels,
                                  radius, epsilon);
    
    set_output("image", Data(output));
    OVF_INFO() << "GuidedFilter executed: radius=" << radius;
    
    return Result<void>::success();
}

// 4. GaussianFilterNode
GaussianFilterNode::GaussianFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GaussianFilterNode::make_info() {
    NodeInfo info;
    info.id = "GaussianFilter";
    info.name = "高斯滤波";
    info.category = "图像增强/滤波";
    info.description = "高斯平滑滤波";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("kernel_size", "核大小", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("sigma", "标准差", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> GaussianFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int kernel_size = static_cast<int>(get_param("kernel_size", Data(5)).as_int());
    double sigma = get_param("sigma", Data(1.0)).as_number();
    
    if (kernel_size % 2 == 0) kernel_size++;
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::gaussian_filter(input.data.data(), output.data.data(),
                                    input.width, input.height, input.channels,
                                    kernel_size, sigma);
    
    set_output("image", Data(output));
    OVF_INFO() << "GaussianFilter executed: kernel_size=" << kernel_size << ", sigma=" << sigma;
    
    return Result<void>::success();
}

// 5. BoxFilterNode
BoxFilterNode::BoxFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BoxFilterNode::make_info() {
    NodeInfo info;
    info.id = "BoxFilter";
    info.name = "方框滤波";
    info.category = "图像增强/滤波";
    info.description = "均值滤波/方框滤波";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("kernel_size", "核大小", DataType::Number, Data(3)));
    
    return info;
}

Result<void> BoxFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int kernel_size = static_cast<int>(get_param("kernel_size", Data(3)).as_int());
    if (kernel_size % 2 == 0) kernel_size++;
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::box_filter(input.data.data(), output.data.data(),
                               input.width, input.height, input.channels, kernel_size);
    
    set_output("image", Data(output));
    OVF_INFO() << "BoxFilter executed: kernel_size=" << kernel_size;
    
    return Result<void>::success();
}

// 6. LaplacianFilterNode
LaplacianFilterNode::LaplacianFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo LaplacianFilterNode::make_info() {
    NodeInfo info;
    info.id = "LaplacianFilter";
    info.name = "拉普拉斯滤波";
    info.category = "图像增强/滤波";
    info.description = "拉普拉斯边缘增强滤波";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> LaplacianFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::laplacian_filter(input.data.data(), output.data.data(),
                                     input.width, input.height, input.channels);
    
    set_output("image", Data(output));
    OVF_INFO() << "LaplacianFilter executed";
    
    return Result<void>::success();
}

// 7. SharpenFilterNode
SharpenFilterNode::SharpenFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SharpenFilterNode::make_info() {
    NodeInfo info;
    info.id = "SharpenFilter";
    info.name = "锐化滤波";
    info.category = "图像增强/滤波";
    info.description = "图像锐化增强边缘";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("strength", "锐化强度", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> SharpenFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double strength = get_param("strength", Data(1.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::sharpen_filter(input.data.data(), output.data.data(),
                                   input.width, input.height, input.channels, strength);
    
    set_output("image", Data(output));
    OVF_INFO() << "SharpenFilter executed: strength=" << strength;
    
    return Result<void>::success();
}

// 8. UnsharpMaskNode
UnsharpMaskNode::UnsharpMaskNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo UnsharpMaskNode::make_info() {
    NodeInfo info;
    info.id = "UnsharpMask";
    info.name = "反锐化掩蔽";
    info.category = "图像增强/滤波";
    info.description = "反锐化掩蔽锐化技术";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("radius", "模糊半径", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("amount", "增强量", DataType::Number, Data(1.5)));
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(0)));
    
    return info;
}

Result<void> UnsharpMaskNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int radius = static_cast<int>(get_param("radius", Data(5)).as_int());
    double amount = get_param("amount", Data(1.5)).as_number();
    double threshold = get_param("threshold", Data(0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::unsharp_mask(input.data.data(), output.data.data(),
                                 input.width, input.height, input.channels,
                                 radius, amount, threshold);
    
    set_output("image", Data(output));
    OVF_INFO() << "UnsharpMask executed: radius=" << radius << ", amount=" << amount;
    
    return Result<void>::success();
}

// ========== 去噪节点实现（4个） ==========

// 9. DenoiseBilateralNode
DenoiseBilateralNode::DenoiseBilateralNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DenoiseBilateralNode::make_info() {
    NodeInfo info;
    info.id = "DenoiseBilateral";
    info.name = "双边去噪";
    info.category = "图像增强/去噪";
    info.description = "双边滤波去噪";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("d", "滤波直径", DataType::Number, Data(9)));
    info.params.push_back(ParamDef("sigma_color", "颜色标准差", DataType::Number, Data(75.0)));
    info.params.push_back(ParamDef("sigma_space", "空间标准差", DataType::Number, Data(75.0)));
    
    return info;
}

Result<void> DenoiseBilateralNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int d = static_cast<int>(get_param("d", Data(9)).as_int());
    double sigma_color = get_param("sigma_color", Data(75.0)).as_number();
    double sigma_space = get_param("sigma_space", Data(75.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::bilateral_filter(input.data.data(), output.data.data(),
                                     input.width, input.height, input.channels,
                                     d, sigma_color, sigma_space);
    
    set_output("image", Data(output));
    OVF_INFO() << "DenoiseBilateral executed";
    
    return Result<void>::success();
}

// 10. DenoiseNLMNode
DenoiseNLMNode::DenoiseNLMNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DenoiseNLMNode::make_info() {
    NodeInfo info;
    info.id = "DenoiseNLM";
    info.name = "非局部均值去噪";
    info.category = "图像增强/去噪";
    info.description = "非局部均值去噪算法（简化版）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("search_window", "搜索窗口", DataType::Number, Data(7)));
    info.params.push_back(ParamDef("template_size", "模板大小", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("h", "滤波参数", DataType::Number, Data(10.0)));
    
    return info;
}

Result<void> DenoiseNLMNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int search_window = static_cast<int>(get_param("search_window", Data(7)).as_int());
    int template_size = static_cast<int>(get_param("template_size", Data(3)).as_int());
    double h = get_param("h", Data(10.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::nlm_denoise(input.data.data(), output.data.data(),
                                input.width, input.height, input.channels,
                                search_window, template_size, h);
    
    set_output("image", Data(output));
    OVF_INFO() << "DenoiseNLM executed";
    
    return Result<void>::success();
}

// 11. DenoiseWaveletNode
DenoiseWaveletNode::DenoiseWaveletNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DenoiseWaveletNode::make_info() {
    NodeInfo info;
    info.id = "DenoiseWavelet";
    info.name = "小波去噪";
    info.category = "图像增强/去噪";
    info.description = "小波去噪算法（简化版Haar小波）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(20.0)));
    
    return info;
}

Result<void> DenoiseWaveletNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double threshold = get_param("threshold", Data(20.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    // 使用简化的小波去噪
    enhance_utils::wavelet_denoise_simple(input.data.data(), output.data.data(),
                                           input.width, input.height, input.channels, threshold);
    
    set_output("image", Data(output));
    OVF_INFO() << "DenoiseWavelet executed: threshold=" << threshold;
    
    return Result<void>::success();
}

// 12. DenoiseAdaptiveNode
DenoiseAdaptiveNode::DenoiseAdaptiveNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DenoiseAdaptiveNode::make_info() {
    NodeInfo info;
    info.id = "DenoiseAdaptive";
    info.name = "自适应去噪";
    info.category = "图像增强/去噪";
    info.description = "根据局部方差自适应调整去噪强度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("window_size", "窗口大小", DataType::Number, Data(7)));
    info.params.push_back(ParamDef("noise_level", "噪声水平", DataType::Number, Data(25.0)));
    
    return info;
}

Result<void> DenoiseAdaptiveNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int window_size = static_cast<int>(get_param("window_size", Data(7)).as_int());
    double noise_level = get_param("noise_level", Data(25.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::adaptive_denoise(input.data.data(), output.data.data(),
                                     input.width, input.height, input.channels,
                                     window_size, noise_level);
    
    set_output("image", Data(output));
    OVF_INFO() << "DenoiseAdaptive executed";
    
    return Result<void>::success();
}

// ========== 增强节点实现（5个） ==========

// 13. ContrastEnhanceNode
ContrastEnhanceNode::ContrastEnhanceNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContrastEnhanceNode::make_info() {
    NodeInfo info;
    info.id = "ContrastEnhance";
    info.name = "对比度增强";
    info.category = "图像增强/增强";
    info.description = "调整图像对比度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("factor", "对比度系数", DataType::Number, Data(1.5)));
    
    return info;
}

Result<void> ContrastEnhanceNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double factor = get_param("factor", Data(1.5)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::contrast_enhance(input.data.data(), output.data.data(),
                                     input.width, input.height, input.channels, factor);
    
    set_output("image", Data(output));
    OVF_INFO() << "ContrastEnhance executed: factor=" << factor;
    
    return Result<void>::success();
}

// 14. BrightnessAdjustNode
BrightnessAdjustNode::BrightnessAdjustNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BrightnessAdjustNode::make_info() {
    NodeInfo info;
    info.id = "BrightnessAdjust";
    info.name = "亮度调整";
    info.category = "图像增强/增强";
    info.description = "调整图像亮度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("delta", "亮度增量", DataType::Number, Data(50)));
    
    return info;
}

Result<void> BrightnessAdjustNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int delta = static_cast<int>(get_param("delta", Data(50)).as_int());
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::brightness_adjust(input.data.data(), output.data.data(),
                                       input.width, input.height, input.channels, delta);
    
    set_output("image", Data(output));
    OVF_INFO() << "BrightnessAdjust executed: delta=" << delta;
    
    return Result<void>::success();
}

// 15. GammaCorrectNode
GammaCorrectNode::GammaCorrectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GammaCorrectNode::make_info() {
    NodeInfo info;
    info.id = "GammaCorrect";
    info.name = "Gamma校正";
    info.category = "图像增强/增强";
    info.description = "Gamma校正调整亮度响应曲线";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("gamma", "Gamma值", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> GammaCorrectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double gamma = get_param("gamma", Data(1.0)).as_number();
    if (gamma <= 0.0) gamma = 1.0;
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::gamma_correct(input.data.data(), output.data.data(),
                                  input.width, input.height, input.channels, gamma);
    
    set_output("image", Data(output));
    OVF_INFO() << "GammaCorrect executed: gamma=" << gamma;
    
    return Result<void>::success();
}

// 16. HistogramEqualizeNode
HistogramEqualizeNode::HistogramEqualizeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo HistogramEqualizeNode::make_info() {
    NodeInfo info;
    info.id = "HistogramEqualize";
    info.name = "直方图均衡化";
    info.category = "图像增强/增强";
    info.description = "全局直方图均衡化增强对比度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> HistogramEqualizeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::histogram_equalize(input.data.data(), output.data.data(),
                                       input.width, input.height, input.channels);
    
    set_output("image", Data(output));
    OVF_INFO() << "HistogramEqualize executed";
    
    return Result<void>::success();
}

// 17. AdaptiveHistogramEqualizeNode
AdaptiveHistogramEqualizeNode::AdaptiveHistogramEqualizeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AdaptiveHistogramEqualizeNode::make_info() {
    NodeInfo info;
    info.id = "AdaptiveHistogramEqualize";
    info.name = "自适应直方图均衡";
    info.category = "图像增强/增强";
    info.description = "CLAHE自适应直方图均衡化（简化版）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("clip_limit", "截断限制", DataType::Number, Data(40)));
    info.params.push_back(ParamDef("tile_size", "块大小", DataType::Number, Data(8)));
    
    return info;
}

Result<void> AdaptiveHistogramEqualizeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int clip_limit = static_cast<int>(get_param("clip_limit", Data(40)).as_int());
    int tile_size = static_cast<int>(get_param("tile_size", Data(8)).as_int());
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::adaptive_histogram_equalize(input.data.data(), output.data.data(),
                                                input.width, input.height, input.channels,
                                                clip_limit, tile_size);
    
    set_output("image", Data(output));
    OVF_INFO() << "AdaptiveHistogramEqualize executed";
    
    return Result<void>::success();
}

// ========== 彩调整节点实现（3个） ==========

// 18. SaturationAdjustNode
SaturationAdjustNode::SaturationAdjustNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SaturationAdjustNode::make_info() {
    NodeInfo info;
    info.id = "SaturationAdjust";
    info.name = "饱和度调整";
    info.category = "图像增强/色彩";
    info.description = "调整图像饱和度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（RGB）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("factor", "饱和度系数", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> SaturationAdjustNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    if (input.channels != 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Saturation adjustment requires RGB image");
    }
    
    double factor = get_param("factor", Data(1.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = 3;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::saturation_adjust(input.data.data(), output.data.data(),
                                      input.width, input.height, factor);
    
    set_output("image", Data(output));
    OVF_INFO() << "SaturationAdjust executed: factor=" << factor;
    
    return Result<void>::success();
}

// 19. HueShiftNode
HueShiftNode::HueShiftNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo HueShiftNode::make_info() {
    NodeInfo info;
    info.id = "HueShift";
    info.name = "色相偏移";
    info.category = "图像增强/色彩";
    info.description = "偏移图像色相";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（RGB）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("delta_h", "色相偏移角度", DataType::Number, Data(0.0)));
    
    return info;
}

Result<void> HueShiftNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    if (input.channels != 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Hue shift requires RGB image");
    }
    
    double delta_h = get_param("delta_h", Data(0.0)).as_number();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = 3;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::hue_shift(input.data.data(), output.data.data(),
                              input.width, input.height, delta_h);
    
    set_output("image", Data(output));
    OVF_INFO() << "HueShift executed: delta_h=" << delta_h;
    
    return Result<void>::success();
}

// 20. WhiteBalanceNode
WhiteBalanceNode::WhiteBalanceNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WhiteBalanceNode::make_info() {
    NodeInfo info;
    info.id = "WhiteBalance";
    info.name = "白平衡校正";
    info.category = "图像增强/色彩";
    info.description = "校正图像白平衡";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（RGB）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("method", "校正方法", DataType::String, Data("gray_world")));
    
    return info;
}

Result<void> WhiteBalanceNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    if (input.channels != 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "White balance requires RGB image");
    }
    
    String method = get_param("method", Data("gray_world")).as_string();
    
    ImageData output;
    output.width = input.width;
    output.height = input.height;
    output.channels = 3;
    output.format = input.format;
    output.data.resize(input.data.size());
    
    enhance_utils::white_balance(input.data.data(), output.data.data(),
                                  input.width, input.height, method);
    
    set_output("image", Data(output));
    OVF_INFO() << "WhiteBalance executed: method=" << method;
    
    return Result<void>::success();
}

// ========== 节点注册 ==========

OVF_REGISTER_NODE(MedianFilterNode, "MedianFilter", MedianFilterNode::make_info())
OVF_REGISTER_NODE(BilateralFilterNode, "BilateralFilter", BilateralFilterNode::make_info())
OVF_REGISTER_NODE(GuidedFilterNode, "GuidedFilter", GuidedFilterNode::make_info())
OVF_REGISTER_NODE(GaussianFilterNode, "GaussianFilter", GaussianFilterNode::make_info())
OVF_REGISTER_NODE(BoxFilterNode, "BoxFilter", BoxFilterNode::make_info())
OVF_REGISTER_NODE(LaplacianFilterNode, "LaplacianFilter", LaplacianFilterNode::make_info())
OVF_REGISTER_NODE(SharpenFilterNode, "SharpenFilter", SharpenFilterNode::make_info())
OVF_REGISTER_NODE(UnsharpMaskNode, "UnsharpMask", UnsharpMaskNode::make_info())

OVF_REGISTER_NODE(DenoiseBilateralNode, "DenoiseBilateral", DenoiseBilateralNode::make_info())
OVF_REGISTER_NODE(DenoiseNLMNode, "DenoiseNLM", DenoiseNLMNode::make_info())
OVF_REGISTER_NODE(DenoiseWaveletNode, "DenoiseWavelet", DenoiseWaveletNode::make_info())
OVF_REGISTER_NODE(DenoiseAdaptiveNode, "DenoiseAdaptive", DenoiseAdaptiveNode::make_info())

OVF_REGISTER_NODE(ContrastEnhanceNode, "ContrastEnhance", ContrastEnhanceNode::make_info())
OVF_REGISTER_NODE(BrightnessAdjustNode, "BrightnessAdjust", BrightnessAdjustNode::make_info())
OVF_REGISTER_NODE(GammaCorrectNode, "GammaCorrect", GammaCorrectNode::make_info())
OVF_REGISTER_NODE(HistogramEqualizeNode, "HistogramEqualize", HistogramEqualizeNode::make_info())
OVF_REGISTER_NODE(AdaptiveHistogramEqualizeNode, "AdaptiveHistogramEqualize", AdaptiveHistogramEqualizeNode::make_info())

OVF_REGISTER_NODE(SaturationAdjustNode, "SaturationAdjust", SaturationAdjustNode::make_info())
OVF_REGISTER_NODE(HueShiftNode, "HueShift", HueShiftNode::make_info())
OVF_REGISTER_NODE(WhiteBalanceNode, "WhiteBalance", WhiteBalanceNode::make_info())

} // namespace algorithm
} // namespace ovf