/**
 * @file industrial_algo.cpp
 * @brief 工业专用算子模块实现
 */

#include "ovf/algorithm/industrial_algo.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/morphology.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <complex>

namespace ovf {
namespace algorithm {

// ========== 辅助函数 ==========

namespace {

// 获取灰度像素值
inline uint8_t get_gray_pixel(const ImageData& img, int x, int y) {
    if (x < 0 || x >= static_cast<int>(img.width) || 
        y < 0 || y >= static_cast<int>(img.height)) {
        return 0;
    }
    
    size_t idx = static_cast<size_t>(y * img.width + x);
    if (img.channels == 1) {
        return img.data[idx];
    } else {
        size_t pixel_idx = idx * img.channels;
        return static_cast<uint8_t>(
            (img.data[pixel_idx] + img.data[pixel_idx + 1] + img.data[pixel_idx + 2]) / 3);
    }
}

// 设置灰度像素值
inline void set_gray_pixel(ImageData& img, int x, int y, uint8_t value) {
    if (x < 0 || x >= static_cast<int>(img.width) || 
        y < 0 || y >= static_cast<int>(img.height)) {
        return;
    }
    
    size_t idx = static_cast<size_t>(y * img.width + x);
    if (img.channels == 1) {
        img.data[idx] = value;
    } else {
        size_t pixel_idx = idx * img.channels;
        img.data[pixel_idx] = value;
        img.data[pixel_idx + 1] = value;
        img.data[pixel_idx + 2] = value;
    }
}

// 转换为灰度图像
void to_gray_image(const ImageData& input, Vector<uint8_t>& gray, int& w, int& h) {
    w = input.width;
    h = input.height;
    gray.resize(w * h);
    
    for (int i = 0; i < w * h; ++i) {
        if (input.channels == 1) {
            gray[i] = input.data[i];
        } else {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>(
                (input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }
}

// 创建输出灰度图像
ImageData create_gray_output(int w, int h, const Vector<uint8_t>& data) {
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = data;
    return output;
}

// 计算局部均值
float compute_local_mean(const Vector<uint8_t>& gray, int w, int h, int x, int y, int radius) {
    float sum = 0.0f;
    int count = 0;
    
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                sum += gray[ny * w + nx];
                count++;
            }
        }
    }
    
    return count > 0 ? sum / count : 0.0f;
}

// 计算局部标准差
float compute_local_std(const Vector<uint8_t>& gray, int w, int h, int x, int y, int radius, float mean) {
    float sum = 0.0f;
    int count = 0;
    
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                float diff = gray[ny * w + nx] - mean;
                sum += diff * diff;
                count++;
            }
        }
    }
    
    return count > 0 ? std::sqrt(sum / count) : 0.0f;
}

// 高斯滤波
void gaussian_filter(const Vector<uint8_t>& src, Vector<uint8_t>& dst, int w, int h, int radius, float sigma) {
    dst.resize(w * h);
    
    // 创建高斯核
    Vector<float> kernel((2 * radius + 1) * (2 * radius + 1));
    float sum = 0.0f;
    
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            float val = std::exp(-(dx * dx + dy * dy) / (2 * sigma * sigma));
            kernel[(dy + radius) * (2 * radius + 1) + (dx + radius)] = val;
            sum += val;
        }
    }
    
    // 归一化
    for (size_t i = 0; i < kernel.size(); ++i) {
        kernel[i] /= sum;
    }
    
    // 应用滤波
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float acc = 0.0f;
            
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    int nx = std::clamp(x + dx, 0, w - 1);
                    int ny = std::clamp(y + dy, 0, h - 1);
                    acc += src[ny * w + nx] * kernel[(dy + radius) * (2 * radius + 1) + (dx + radius)];
                }
            }
            
            dst[y * w + x] = static_cast<uint8_t>(std::clamp(acc, 0.0f, 255.0f));
        }
    }
}

// 连通区域标记（简化版）
struct ConnectedComponent {
    int x, y, width, height;
    int area;
    int min_x, min_y, max_x, max_y;
};

void flood_fill(Vector<uint8_t>& binary, int w, int h, int x, int y, uint8_t old_val, uint8_t new_val, ConnectedComponent& comp) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    if (binary[y * w + x] != old_val) return;
    
    Vector<std::pair<int, int>> stack;
    stack.push_back({x, y});
    
    comp.min_x = x; comp.max_x = x;
    comp.min_y = y; comp.max_y = y;
    comp.area = 0;
    
    while (!stack.empty()) {
        auto [cx, cy] = stack.back();
        stack.pop_back();
        
        if (cx < 0 || cx >= w || cy < 0 || cy >= h) continue;
        if (binary[cy * w + cx] != old_val) continue;
        
        binary[cy * w + cx] = new_val;
        comp.area++;
        comp.min_x = std::min(comp.min_x, cx);
        comp.max_x = std::max(comp.max_x, cx);
        comp.min_y = std::min(comp.min_y, cy);
        comp.max_y = std::max(comp.max_y, cy);
        
        // 4邻域
        stack.push_back({cx + 1, cy});
        stack.push_back({cx - 1, cy});
        stack.push_back({cx, cy + 1});
        stack.push_back({cx, cy - 1});
    }
    
    comp.x = comp.min_x;
    comp.y = comp.min_y;
    comp.width = comp.max_x - comp.min_x + 1;
    comp.height = comp.max_y - comp.min_y + 1;
}

Vector<ConnectedComponent> find_connected_components(Vector<uint8_t>& binary, int w, int h) {
    Vector<ConnectedComponent> components;
    uint8_t label = 128; // 使用中间值标记
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (binary[y * w + x] == 255) {
                ConnectedComponent comp;
                flood_fill(binary, w, h, x, y, 255, label, comp);
                components.push_back(comp);
                label++;
                if (label >= 200) label = 128; // 防止溢出
            }
        }
    }
    
    // 恢复二值图
    for (size_t i = 0; i < binary.size(); ++i) {
        if (binary[i] >= 128) binary[i] = 255;
    }
    
    return components;
}

// Sobel边缘检测
void sobel_edge(const Vector<uint8_t>& src, Vector<uint8_t>& dst, int w, int h, int threshold) {
    dst.resize(w * h);
    std::fill(dst.begin(), dst.end(), 0);
    
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            int gx = 0, gy = 0;
            
            // Sobel算子
            gx = -src[(y-1)*w + (x-1)] + src[(y-1)*w + (x+1)]
                 -2*src[y*w + (x-1)] + 2*src[y*w + (x+1)]
                 -src[(y+1)*w + (x-1)] + src[(y+1)*w + (x+1)];
            
            gy = -src[(y-1)*w + (x-1)] - 2*src[(y-1)*w + x] - src[(y-1)*w + (x+1)]
                 +src[(y+1)*w + (x-1)] + 2*src[(y+1)*w + x] + src[(y+1)*w + (x+1)];
            
            int magnitude = static_cast<int>(std::sqrt(gx * gx + gy * gy));
            dst[y * w + x] = (magnitude > threshold) ? 255 : 0;
        }
    }
}

// 计算梯度方向
float compute_gradient_angle(const Vector<uint8_t>& src, int w, int h, int x, int y) {
    if (x < 1 || x >= w - 1 || y < 1 || y >= h - 1) return 0.0f;
    
    int gx = -src[(y-1)*w + (x-1)] + src[(y-1)*w + (x+1)]
             -2*src[y*w + (x-1)] + 2*src[y*w + (x+1)]
             -src[(y+1)*w + (x-1)] + src[(y+1)*w + (x+1)];
    
    int gy = -src[(y-1)*w + (x-1)] - 2*src[(y-1)*w + x] - src[(y-1)*w + (x+1)]
             +src[(y+1)*w + (x-1)] + 2*src[(y+1)*w + x] + src[(y+1)*w + (x+1)];
    
    return std::atan2(static_cast<float>(gy), static_cast<float>(gx)) * 180.0f / 3.14159f;
}

// 中值滤波
void median_filter(const Vector<uint8_t>& src, Vector<uint8_t>& dst, int w, int h, int radius) {
    dst.resize(w * h);
    Vector<uint8_t> window((2 * radius + 1) * (2 * radius + 1));
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int count = 0;
            
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    int nx = std::clamp(x + dx, 0, w - 1);
                    int ny = std::clamp(y + dy, 0, h - 1);
                    window[count++] = src[ny * w + nx];
                }
            }
            
            // 排序取中值
            std::sort(window.begin(), window.begin() + count);
            dst[y * w + x] = window[count / 2];
        }
    }
}

// 双边滤波简化版
void bilateral_filter_simple(const Vector<uint8_t>& src, Vector<uint8_t>& dst, int w, int h, 
                              int radius, float sigma_space, float sigma_color) {
    dst.resize(w * h);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float sum = 0.0f;
            float weight_sum = 0.0f;
            uint8_t center_val = src[y * w + x];
            
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    int nx = std::clamp(x + dx, 0, w - 1);
                    int ny = std::clamp(y + dy, 0, h - 1);
                    
                    uint8_t val = src[ny * w + nx];
                    
                    // 空间权重
                    float space_weight = std::exp(-(dx * dx + dy * dy) / (2 * sigma_space * sigma_space));
                    
                    // 颜色权重
                    float color_diff = static_cast<float>(std::abs(val - center_val));
                    float color_weight = std::exp(-(color_diff * color_diff) / (2 * sigma_color * sigma_color));
                    
                    float weight = space_weight * color_weight;
                    sum += val * weight;
                    weight_sum += weight;
                }
            }
            
            dst[y * w + x] = static_cast<uint8_t>(std::clamp(sum / weight_sum, 0.0f, 255.0f));
        }
    }
}

// 计算图像均值
float compute_image_mean(const Vector<uint8_t>& gray) {
    if (gray.empty()) return 0.0f;
    float sum = std::accumulate(gray.begin(), gray.end(), 0.0f);
    return sum / gray.size();
}

// 计算图像标准差
float compute_image_std(const Vector<uint8_t>& gray, float mean) {
    if (gray.empty()) return 0.0f;
    float sum = 0.0f;
    for (uint8_t val : gray) {
        float diff = val - mean;
        sum += diff * diff;
    }
    return std::sqrt(sum / gray.size());
}

// 直方图统计
void compute_histogram(const Vector<uint8_t>& gray, Vector<int>& hist) {
    hist.resize(256, 0);
    for (uint8_t val : gray) {
        hist[val]++;
    }
}

// 直方图均衡化
void histogram_equalize(Vector<uint8_t>& gray) {
    Vector<int> hist;
    compute_histogram(gray, hist);
    
    // 计算累积分布
    Vector<int> cdf(256);
    cdf[0] = hist[0];
    for (int i = 1; i < 256; ++i) {
        cdf[i] = cdf[i - 1] + hist[i];
    }
    
    // 找到最小非零CDF值
    int min_cdf = gray.size();
    for (int i = 0; i < 256; ++i) {
        if (cdf[i] > 0 && cdf[i] < min_cdf) {
            min_cdf = cdf[i];
        }
    }
    
    // 映射
    Vector<uint8_t> lut(256);
    int total = gray.size();
    for (int i = 0; i < 256; ++i) {
        lut[i] = static_cast<uint8_t>(std::clamp(
            static_cast<int>((cdf[i] - min_cdf) * 255 / (total - min_cdf)), 0, 255));
    }
    
    // 应用
    for (size_t i = 0; i < gray.size(); ++i) {
        gray[i] = lut[gray[i]];
    }
}

} // anonymous namespace

// ========== 缺陷检测模块实现 ==========

namespace defect_utils {

ErrorCode detect_micro_defects(const ImageData& image, Vector<DefectInfo>& defects,
                               float sensitivity, int min_size) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 多尺度检测
    Vector<int> scales = {3, 5, 7};
    float threshold_base = 50.0f * (1.0f - sensitivity);
    
    Vector<uint8_t> combined_defect(w * h, 0);
    
    for (int scale : scales) {
        // 形态学顶帽运算（检测亮缺陷）
        Vector<uint8_t> kernel(scale * scale, 255);
        Vector<uint8_t> opened(w * h);
        morph_utils::open(gray.data(), opened.data(), w, h, 
                          kernel.data(), scale, scale, 1);
        
        // 顶帽 = 原图 - 开运算
        Vector<uint8_t> top_hat(w * h);
        for (int i = 0; i < w * h; ++i) {
            int diff = gray[i] - opened[i];
            top_hat[i] = static_cast<uint8_t>(std::clamp(diff, 0, 255));
        }
        
        // 黑帽运算（检测暗缺陷）
        Vector<uint8_t> closed(w * h);
        morph_utils::close(gray.data(), closed.data(), w, h,
                           kernel.data(), scale, scale, 1);
        
        // 黑帽 = 闭运算 - 原图
        Vector<uint8_t> black_hat(w * h);
        for (int i = 0; i < w * h; ++i) {
            int diff = closed[i] - gray[i];
            black_hat[i] = static_cast<uint8_t>(std::clamp(diff, 0, 255));
        }
        
        // 合并亮暗缺陷
        float threshold = threshold_base * scale;
        for (int i = 0; i < w * h; ++i) {
            if (top_hat[i] > threshold || black_hat[i] > threshold) {
                combined_defect[i] = 255;
            }
        }
    }
    
    // 局部对比度检测补充
    int contrast_radius = 5;
    float contrast_threshold = 0.15f + sensitivity * 0.1f;
    
    for (int y = contrast_radius; y < h - contrast_radius; ++y) {
        for (int x = contrast_radius; x < w - contrast_radius; ++x) {
            float local_mean = compute_local_mean(gray, w, h, x, y, contrast_radius);
            float local_std = compute_local_std(gray, w, h, x, y, contrast_radius, local_mean);
            
            float deviation = std::abs(gray[y * w + x] - local_mean);
            if (local_std > 0 && deviation / local_std > 2.0f + (1.0f - sensitivity)) {
                combined_defect[y * w + x] = 255;
            }
        }
    }
    
    // 连通区域分析
    Vector<ConnectedComponent> components = find_connected_components(combined_defect, w, h);
    
    // 过滤并生成缺陷信息
    defects.clear();
    for (const auto& comp : components) {
        if (comp.width >= min_size && comp.height >= min_size && comp.area >= min_size * min_size) {
            DefectInfo defect;
            
            // 根据形状判断缺陷类型
            float aspect_ratio = static_cast<float>(comp.width) / comp.height;
            float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height);
            
            if (aspect_ratio > 3.0f || aspect_ratio < 0.33f) {
                defect.type = DefectType::Scratch;
            } else if (area_ratio < 0.3f) {
                defect.type = DefectType::Crack;
            } else if (comp.area < 20) {
                defect.type = DefectType::Pinhole;
            } else if (area_ratio > 0.8f && comp.area < 100) {
                defect.type = DefectType::Spot;
            } else {
                defect.type = DefectType::Contamination;
            }
            
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            defect.confidence = std::clamp(area_ratio, 0.5f, 1.0f);
            defect.description = defect_type_name(defect.type) + " @(" + 
                                std::to_string(defect.x) + "," + std::to_string(defect.y) + ")";
            
            defects.push_back(defect);
        }
    }
    
    OVF_INFO() << "Micro defect detection: found " << defects.size() << " defects";
    return ErrorCode::Success;
}

ErrorCode detect_scratches(const ImageData& image, Vector<DefectInfo>& scratches,
                          int min_length, float angle_tolerance) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 边缘检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);
    
    // 方向性分析：检测线状结构
    scratches.clear();
    
    // 使用霍夫线检测简化版
    // 检测主方向
    Vector<float> angles;
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            if (edge[y * w + x] > 0) {
                float angle = compute_gradient_angle(gray, w, h, x, y);
                angles.push_back(angle);
            }
        }
    }
    
    if (angles.empty()) {
        return ErrorCode::Success;
    }
    
    // 分组角度
    Vector<Vector<std::pair<int, int>>> angle_groups(36); // 每10度一个组
    for (size_t i = 0; i < angles.size(); ++i) {
        int group = static_cast<int>((angles[i] + 180) / 10) % 36;
        // 找到对应的边缘点
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                if (edge[y * w + x] > 0) {
                    float pt_angle = compute_gradient_angle(gray, w, h, x, y);
                    if (static_cast<int>((pt_angle + 180) / 10) % 36 == group) {
                        angle_groups[group].push_back({x, y});
                    }
                }
            }
        }
    }
    
    // 对每个角度组，检测是否有连续线段
    for (int g = 0; g < 36; ++g) {
        if (angle_groups[g].size() < min_length) continue;
        
        // 计算该组的范围
        int min_x = w, max_x = 0, min_y = h, max_y = 0;
        for (const auto& pt : angle_groups[g]) {
            min_x = std::min(min_x, pt.first);
            max_x = std::max(max_x, pt.first);
            min_y = std::min(min_y, pt.second);
            max_y = std::max(max_y, pt.second);
        }
        
        // 检查是否为线状（长宽比大）
        int length = std::max(max_x - min_x, max_y - min_y);
        int width = std::min(max_x - min_x, max_y - min_y);
        
        if (length >= min_length && static_cast<float>(length) / (width + 1) > 3.0f) {
            DefectInfo scratch;
            scratch.type = DefectType::Scratch;
            scratch.x = min_x;
            scratch.y = min_y;
            scratch.width = max_x - min_x + 1;
            scratch.height = max_y - min_y + 1;
            scratch.severity = std::clamp(static_cast<float>(length) / 100.0f, 0.0f, 1.0f);
            scratch.confidence = 0.8f;
            scratch.description = "划痕 长度=" + std::to_string(length) + "px";
            
            scratches.push_back(scratch);
        }
    }
    
    OVF_INFO() << "Scratch detection: found " << scratches.size() << " scratches";
    return ErrorCode::Success;
}

ErrorCode detect_spots(const ImageData& image, Vector<DefectInfo>& spots,
                      int min_area, float contrast_threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 局部对比度检测
    Vector<uint8_t> high_contrast(w * h, 0);
    int radius = 7;
    
    for (int y = radius; y < h - radius; ++y) {
        for (int x = radius; x < w - radius; ++x) {
            float local_mean = compute_local_mean(gray, w, h, x, y, radius);
            float local_std = compute_local_std(gray, w, h, x, y, radius, local_mean);
            
            // 与局部均值差异大的点
            float diff = std::abs(gray[y * w + x] - local_mean);
            if (diff > contrast_threshold * 255 && local_std > 5) {
                high_contrast[y * w + x] = 255;
            }
        }
    }
    
    // 形态学处理去除噪声
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> opened(w * h);
    morph_utils::open(high_contrast.data(), opened.data(), w, h, kernel.data(), 3, 3, 1);
    
    // 连通区域分析
    Vector<ConnectedComponent> components = find_connected_components(opened, w, h);
    
    spots.clear();
    for (const auto& comp : components) {
        if (comp.area >= min_area) {
            DefectInfo spot;
            spot.type = DefectType::Spot;
            spot.x = comp.x;
            spot.y = comp.y;
            spot.width = comp.width;
            spot.height = comp.height;
            
            // 计算斑点区域的平均亮度差异
            float spot_mean = compute_local_mean(gray, w, h, comp.x + comp.width/2, 
                                                  comp.y + comp.height/2, comp.width/2);
            float global_mean = compute_image_mean(gray);
            
            spot.severity = std::clamp(std::abs(spot_mean - global_mean) / 50.0f, 0.0f, 1.0f);
            spot.confidence = 0.75f;
            spot.description = "斑点 面积=" + std::to_string(comp.area) + "px";
            
            spots.push_back(spot);
        }
    }
    
    OVF_INFO() << "Spot detection: found " << spots.size() << " spots";
    return ErrorCode::Success;
}

} // namespace defect_utils

// ========== 反光处理模块实现 ==========

namespace reflection_utils {

ErrorCode detect_reflection(const ImageData& image, Vector<Point2D<int>>& reflection_points,
                            float threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    reflection_points.clear();
    
    // 高亮区域检测
    Vector<uint8_t> high_light(w * h, 0);
    for (int i = 0; i < w * h; ++i) {
        if (gray[i] >= threshold) {
            high_light[i] = 255;
        }
    }
    
    // 形态学膨胀合并相邻高亮点
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> dilated(w * h);
    morph_utils::dilate(high_light.data(), dilated.data(), w, h, kernel.data(), 3, 3, 1);
    
    // 找高光区域中心点
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            if (dilated[y * w + x] > 0) {
                // 检查是否为局部最大值区域中心
                bool is_center = true;
                int sum_x = 0, sum_y = 0, count = 0;
                
                for (int dy = -3; dy <= 3; ++dy) {
                    for (int dx = -3; dx <= 3; ++dx) {
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                            if (dilated[ny * w + nx] > 0) {
                                sum_x += nx;
                                sum_y += ny;
                                count++;
                            }
                        }
                    }
                }
                
                if (count > 0) {
                    Point2D<int> center;
                    center.x = sum_x / count;
                    center.y = sum_y / count;
                    
                    // 避免重复添加
                    bool exists = false;
                    for (const auto& pt : reflection_points) {
                        if (std::abs(pt.x - center.x) < 5 && std::abs(pt.y - center.y) < 5) {
                            exists = true;
                            break;
                        }
                    }
                    
                    if (!exists) {
                        reflection_points.push_back(center);
                    }
                }
            }
        }
    }
    
    OVF_INFO() << "Reflection detection: found " << reflection_points.size() << " reflection points";
    return ErrorCode::Success;
}

ErrorCode remove_reflection(ImageData& image, int method) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 检测反光区域
    Vector<Point2D<int>> reflection_points;
    detect_reflection(image, reflection_points, 240.0f);
    
    if (reflection_points.empty()) {
        return ErrorCode::Success;
    }
    
    Vector<uint8_t> result;
    
    switch (method) {
        case 0: // 滤波方法 - 中值滤波
            median_filter(gray, result, w, h, 3);
            break;
            
        case 1: // 插值方法 - 邻域插值修复
            result = gray;
            for (const auto& pt : reflection_points) {
                // 对每个反光点周围区域进行插值
                int radius = 10;
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        int nx = pt.x + dx;
                        int ny = pt.y + dy;
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                            if (gray[ny * w + nx] > 240) {
                                // 使用周围非高亮像素插值
                                float sum = 0;
                                int count = 0;
                                for (int sy = -3; sy <= 3; ++sy) {
                                    for (int sx = -3; sx <= 3; ++sx) {
                                        int px = nx + sx, py = ny + sy;
                                        if (px >= 0 && px < w && py >= 0 && py < h) {
                                            if (gray[py * w + px] <= 240) {
                                                sum += gray[py * w + px];
                                                count++;
                                            }
                                        }
                                    }
                                }
                                if (count > 0) {
                                    result[ny * w + nx] = static_cast<uint8_t>(sum / count);
                                }
                            }
                        }
                    }
                }
            }
            break;
            
        case 2: // 偏振模拟 - 降低高亮区域强度
            result = gray;
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    if (gray[y * w + x] > 240) {
                        // 偏振效果：压缩高亮区域动态范围
                        float val = gray[y * w + x];
                        float compressed = 240 + (val - 240) * 0.3f;
                        result[y * w + x] = static_cast<uint8_t>(compressed);
                    }
                }
            }
            break;
            
        default:
            median_filter(gray, result, w, h, 3);
            break;
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Reflection removed using method " << method;
    return ErrorCode::Success;
}

ErrorCode repair_highlights(ImageData& image, const Vector<Point2D<int>>& highlights) {
    if (image.empty() || highlights.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    Vector<uint8_t> result = gray;
    
    // 对每个高光点进行修复
    for (const auto& pt : highlights) {
        // 使用周围像素的双向插值
        int radius = 15;
        
        // 检测高光区域边界
        int min_x = pt.x, max_x = pt.x, min_y = pt.y, max_y = pt.y;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int nx = pt.x + dx, ny = pt.y + dy;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                    if (gray[ny * w + nx] > 240) {
                        min_x = std::min(min_x, nx);
                        max_x = std::max(max_x, nx);
                        min_y = std::min(min_y, ny);
                        max_y = std::max(max_y, ny);
                    }
                }
            }
        }
        
        // 使用边界像素进行内部填充
        Vector<uint8_t> boundary_pixels;
        
        // 收集边界像素
        for (int x = min_x; x <= max_x; ++x) {
            if (min_y > 0 && gray[(min_y - 1) * w + x] < 240) {
                boundary_pixels.push_back(gray[(min_y - 1) * w + x]);
            }
            if (max_y < h - 1 && gray[(max_y + 1) * w + x] < 240) {
                boundary_pixels.push_back(gray[(max_y + 1) * w + x]);
            }
        }
        for (int y = min_y; y <= max_y; ++y) {
            if (min_x > 0 && gray[y * w + (min_x - 1)] < 240) {
                boundary_pixels.push_back(gray[y * w + (min_x - 1)]);
            }
            if (max_x < w - 1 && gray[y * w + (max_x + 1)] < 240) {
                boundary_pixels.push_back(gray[y * w + (max_x + 1)]);
            }
        }
        
        if (!boundary_pixels.empty()) {
            // 计算边界均值
            float boundary_mean = std::accumulate(boundary_pixels.begin(), boundary_pixels.end(), 0.0f) 
                                  / boundary_pixels.size();
            
            // 填充高光区域
            for (int y = min_y; y <= max_y; ++y) {
                for (int x = min_x; x <= max_x; ++x) {
                    if (gray[y * w + x] > 240) {
                        // 使用距离加权插值
                        float dist_to_center = std::sqrt(static_cast<float>((x - pt.x) * (x - pt.x) + 
                                                                             (y - pt.y) * (y - pt.y)));
                        float max_dist = std::sqrt(static_cast<float>((max_x - min_x) * (max_x - min_x) / 4 + 
                                                                       (max_y - min_y) * (max_y - min_y) / 4));
                        
                        // 边缘值保持，中心使用边界均值
                        float weight = std::clamp(dist_to_center / max_dist, 0.0f, 1.0f);
                        float repaired_val = gray[y * w + x] * weight + boundary_mean * (1 - weight);
                        
                        result[y * w + x] = static_cast<uint8_t>(std::clamp(repaired_val, 0.0f, 255.0f));
                    }
                }
            }
        }
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Highlights repaired: " << highlights.size() << " points";
    return ErrorCode::Success;
}

ErrorCode correct_uneven_lighting(ImageData& image) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 计算光照背景（大核滤波）
    Vector<uint8_t> background;
    gaussian_filter(gray, background, w, h, 30, 30.0f);
    
    // 计算全局平均亮度
    float global_mean = compute_image_mean(background);
    
    // 光照校正
    Vector<uint8_t> result(w * h);
    for (int i = 0; i < w * h; ++i) {
        if (background[i] > 0) {
            float correction = global_mean / background[i];
            float corrected = gray[i] * correction;
            result[i] = static_cast<uint8_t>(std::clamp(corrected, 0.0f, 255.0f));
        } else {
            result[i] = gray[i];
        }
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Uneven lighting corrected";
    return ErrorCode::Success;
}

ErrorCode adaptive_threshold_reflection(const ImageData& image, ImageData& binary) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 自适应阈值：使用局部均值
    Vector<uint8_t> result(w * h);
    int block_size = 31;
    int offset = 10;
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float local_mean = compute_local_mean(gray, w, h, x, y, block_size / 2);
            
            // 反光区域特殊处理：降低阈值
            float threshold = local_mean - offset;
            if (gray[y * w + x] > 240) {
                threshold = local_mean - offset * 0.5f;  // 反光区域阈值更低
            }
            
            result[y * w + x] = (gray[y * w + x] > threshold) ? 255 : 0;
        }
    }
    
    binary = create_gray_output(w, h, result);
    
    OVF_INFO() << "Adaptive threshold with reflection handling completed";
    return ErrorCode::Success;
}

} // namespace reflection_utils

// ========== 频域分析模块实现 ==========

namespace frequency_utils {

ErrorCode fft2d(const ImageData& image, Vector<float>& magnitude, Vector<float>& phase) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 简化FFT实现：使用DFT（适合小图像）
    // 对于大图像，这里简化为计算简单的频域表示
    
    int n = w * h;
    magnitude.resize(n);
    phase.resize(n);
    
    // 使用简化的频域分析：计算各方向的频率强度
    // 这里不实现完整的FFT，而是使用简化方法
    
    // 中心点
    int cx = w / 2;
    int cy = h / 2;
    
    // 计算频率分布（基于图像梯度）
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // 计算局部梯度作为频率代理
            float grad_x = 0, grad_y = 0;
            
            if (x > 0 && x < w - 1) {
                grad_x = static_cast<float>(gray[y * w + x + 1]) - gray[y * w + x - 1];
            }
            if (y > 0 && y < h - 1) {
                grad_y = static_cast<float>(gray[(y + 1) * w + x]) - gray[(y - 1) * w + x];
            }
            
            // 转换到"频域"表示
            float freq_x = static_cast<float>(x - cx) / w;
            float freq_y = static_cast<float>(y - cy) / h;
            
            // 模拟FFT幅度
            magnitude[y * w + x] = std::sqrt(grad_x * grad_x + grad_y * grad_y) / 255.0f;
            phase[y * w + x] = std::atan2(grad_y, grad_x);
        }
    }
    
    OVF_INFO() << "FFT 2D (simplified) completed";
    return ErrorCode::Success;
}

ErrorCode frequency_filter(ImageData& image, int filter_type, float cutoff_freq) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    Vector<uint8_t> result(w * h);
    
    int cx = w / 2;
    int cy = h / 2;
    
    // 频域滤波简化实现：使用空间滤波器模拟
    switch (filter_type) {
        case 0: { // 低通 - 平滑滤波
            gaussian_filter(gray, result, w, h, 
                            static_cast<int>(cutoff_freq * 10), cutoff_freq * 5);
            break;
        }
        
        case 1: { // 高通 - 边缘增强
            Vector<uint8_t> lowpass;
            gaussian_filter(gray, lowpass, w, h, 
                            static_cast<int>(cutoff_freq * 10), cutoff_freq * 5);
            
            // 原图 - 低通 = 高通
            for (int i = 0; i < w * h; ++i) {
                int diff = gray[i] - lowpass[i];
                result[i] = static_cast<uint8_t>(std::clamp(diff + 128, 0, 255));
            }
            break;
        }
        
        case 2: { // 带通 - 边缘检测增强
            Vector<uint8_t> low1, low2;
            gaussian_filter(gray, low1, w, h, 
                            static_cast<int>(cutoff_freq * 5), cutoff_freq * 2.5f);
            gaussian_filter(gray, low2, w, h, 
                            static_cast<int>(cutoff_freq * 15), cutoff_freq * 7.5f);
            
            // 不同尺度差分
            for (int i = 0; i < w * h; ++i) {
                int diff = low2[i] - low1[i];
                result[i] = static_cast<uint8_t>(std::clamp(std::abs(diff), 0, 255));
            }
            break;
        }
        
        default:
            result = gray;
            break;
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Frequency filter applied: type=" << filter_type;
    return ErrorCode::Success;
}

ErrorCode remove_periodic_noise(ImageData& image) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 周期性噪声检测和去除
    // 使用中值滤波去除周期噪声
    Vector<uint8_t> filtered;
    median_filter(gray, filtered, w, h, 3);
    
    // 进一步使用双边滤波
    Vector<uint8_t> result;
    bilateral_filter_simple(filtered, result, w, h, 3, 3.0f, 30.0f);
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Periodic noise removed";
    return ErrorCode::Success;
}

ErrorCode analyze_texture(const ImageData& image, float& uniformity, float& coarseness) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 纹理均匀度：基于局部标准差的均匀性
    Vector<float> local_stds;
    int block_size = 16;
    
    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            float block_mean = compute_local_mean(gray, w, h, x + block_size/2, y + block_size/2, block_size/2);
            float block_std = compute_local_std(gray, w, h, x + block_size/2, y + block_size/2, block_size/2, block_mean);
            local_stds.push_back(block_std);
        }
    }
    
    if (local_stds.empty()) {
        uniformity = 0.0f;
        coarseness = 0.0f;
        return ErrorCode::Success;
    }
    
    // 均匀度：局部标准差的一致性
    float mean_std = std::accumulate(local_stds.begin(), local_stds.end(), 0.0f) / local_stds.size();
    float var_std = 0.0f;
    for (float std : local_stds) {
        var_std += (std - mean_std) * (std - mean_std);
    }
    var_std /= local_stds.size();
    
    uniformity = 1.0f - std::clamp(std::sqrt(var_std) / (mean_std + 1), 0.0f, 1.0f);
    
    // 粗糙度：基于梯度幅值的平均
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 20);
    
    float gradient_sum = 0.0f;
    int edge_count = 0;
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            int gx = -gray[(y-1)*w + (x-1)] + gray[(y-1)*w + (x+1)]
                     -2*gray[y*w + (x-1)] + 2*gray[y*w + (x+1)]
                     -gray[(y+1)*w + (x-1)] + gray[(y+1)*w + (x+1)];
            int gy = -gray[(y-1)*w + (x-1)] - 2*gray[(y-1)*w + x] - gray[(y-1)*w + (x+1)]
                     +gray[(y+1)*w + (x-1)] + 2*gray[(y+1)*w + x] + gray[(y+1)*w + (x+1)];
            
            float magnitude = std::sqrt(static_cast<float>(gx * gx + gy * gy));
            gradient_sum += magnitude;
            edge_count++;
        }
    }
    
    float avg_gradient = gradient_sum / edge_count;
    coarseness = 1.0f - std::clamp(avg_gradient / 100.0f, 0.0f, 1.0f);
    
    OVF_INFO() << "Texture analysis: uniformity=" << uniformity << ", coarseness=" << coarseness;
    return ErrorCode::Success;
}

ErrorCode detect_defects_frequency(const ImageData& image, Vector<defect_utils::DefectInfo>& defects) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 频域缺陷检测：检测异常频率成分
    // 使用不同尺度的滤波差分
    
    Vector<uint8_t> smooth1, smooth2, smooth3;
    gaussian_filter(gray, smooth1, w, h, 3, 1.5f);
    gaussian_filter(gray, smooth2, w, h, 7, 3.5f);
    gaussian_filter(gray, smooth3, w, h, 15, 7.5f);
    
    // 多尺度差分
    Vector<uint8_t> diff1(w * h), diff2(w * h), diff3(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff1[i] = static_cast<uint8_t>(std::clamp(std::abs(gray[i] - smooth1[i]), 0, 255));
        diff2[i] = static_cast<uint8_t>(std::clamp(std::abs(smooth1[i] - smooth2[i]), 0, 255));
        diff3[i] = static_cast<uint8_t>(std::clamp(std::abs(smooth2[i] - smooth3[i]), 0, 255));
    }
    
    // 合并异常区域
    Vector<uint8_t> anomaly(w * h, 0);
    for (int i = 0; i < w * h; ++i) {
        if (diff1[i] > 30 || diff2[i] > 20 || diff3[i] > 15) {
            anomaly[i] = 255;
        }
    }
    
    // 形态学处理
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(anomaly.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);
    
    // 连通区域分析
    Vector<ConnectedComponent> components = find_connected_components(cleaned, w, h);
    
    defects.clear();
    for (const auto& comp : components) {
        if (comp.area >= 5) {
            defect_utils::DefectInfo defect;
            defect.type = defect_utils::DefectType::Unevenness;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 50.0f, 0.0f, 1.0f);
            defect.confidence = 0.7f;
            defect.description = "频域检测异常 区域=" + std::to_string(comp.area) + "px";
            
            defects.push_back(defect);
        }
    }
    
    OVF_INFO() << "Frequency defect detection: found " << defects.size() << " defects";
    return ErrorCode::Success;
}

} // namespace frequency_utils

// ========== 图像增强模块实现 ==========

namespace enhance_utils {

ErrorCode enhance_low_contrast(ImageData& image, float strength) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 计算当前对比度
    float mean = compute_image_mean(gray);
    float std = compute_image_std(gray, mean);
    
    // 对比度增强：拉伸动态范围
    float target_std = 50.0f + strength * 30.0f; // 目标标准差
    
    Vector<uint8_t> result(w * h);
    for (int i = 0; i < w * h; ++i) {
        float val = gray[i];
        float normalized = (val - mean) / (std + 1);
        float enhanced = normalized * target_std + mean;
        
        // 防止过度增强
        float blend = strength;
        float final_val = val * (1 - blend) + enhanced * blend;
        
        result[i] = static_cast<uint8_t>(std::clamp(final_val, 0.0f, 255.0f));
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Low contrast enhanced: strength=" << strength;
    return ErrorCode::Success;
}

ErrorCode enhance_dark_image(ImageData& image, float gamma) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // Gamma校正增强暗图
    Vector<uint8_t> result(w * h);
    
    // 建立查找表提高效率
    Vector<uint8_t> lut(256);
    for (int i = 0; i < 256; ++i) {
        float normalized = i / 255.0f;
        float corrected = std::pow(normalized, gamma);
        lut[i] = static_cast<uint8_t>(std::clamp(corrected * 255, 0.0f, 255.0f));
    }
    
    for (int i = 0; i < w * h; ++i) {
        result[i] = lut[gray[i]];
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Dark image enhanced: gamma=" << gamma;
    return ErrorCode::Success;
}

ErrorCode denoise_preserve_edge(ImageData& image, int iterations) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    Vector<uint8_t> result = gray;
    
    // 双边滤波迭代
    for (int iter = 0; iter < iterations; ++iter) {
        Vector<uint8_t> temp;
        bilateral_filter_simple(result, temp, w, h, 3, 3.0f, 25.0f);
        result = temp;
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Denoise with edge preservation: iterations=" << iterations;
    return ErrorCode::Success;
}

ErrorCode sharpen(ImageData& image, float strength) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w = image.width;
    int h = image.height;
    
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 高斯模糊
    Vector<uint8_t> blurred;
    gaussian_filter(gray, blurred, w, h, 3, 1.0f);
    
    // 锐化：原图 + strength * (原图 - 模糊)
    Vector<uint8_t> result(w * h);
    for (int i = 0; i < w * h; ++i) {
        float diff = static_cast<float>(gray[i]) - blurred[i];
        float sharpened = gray[i] + strength * diff;
        result[i] = static_cast<uint8_t>(std::clamp(sharpened, 0.0f, 255.0f));
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Image sharpened: strength=" << strength;
    return ErrorCode::Success;
}

ErrorCode adaptive_enhance(ImageData& image) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);
    
    // 分析图像特征
    float mean = compute_image_mean(gray);
    float std = compute_image_std(gray, mean);
    
    Vector<int> hist;
    compute_histogram(gray, hist);
    
    // 动态范围分析
    int min_val = 0, max_val = 255;
    for (int i = 0; i < 256; ++i) {
        if (hist[i] > 0) {
            min_val = i;
            break;
        }
    }
    for (int i = 255; i >= 0; --i) {
        if (hist[i] > 0) {
            max_val = i;
            break;
        }
    }
    
    int dynamic_range = max_val - min_val;
    
    Vector<uint8_t> result = gray;
    
    // 根据图像特征选择增强策略
    if (mean < 80) {
        // 暗图：Gamma校正
        float gamma = 1.5f + (80 - mean) / 80.0f;
        enhance_dark_image(image, gamma);
        to_gray_image(image, gray, w, h);
        result = gray;
    }
    
    if (std < 30 || dynamic_range < 100) {
        // 低对比度：对比度拉伸
        float strength = 1.0f + (30 - std) / 30.0f;
        
        for (int i = 0; i < w * h; ++i) {
            float normalized = (result[i] - min_val) / (dynamic_range + 1);
            float stretched = normalized * 255;
            result[i] = static_cast<uint8_t>(std::clamp(stretched, 0.0f, 255.0f));
        }
    }
    
    // 边缘增强
    Vector<uint8_t> blurred;
    gaussian_filter(result, blurred, w, h, 2, 1.0f);
    
    float edge_strength = 0.3f;
    for (int i = 0; i < w * h; ++i) {
        float diff = result[i] - blurred[i];
        float sharpened = result[i] + edge_strength * diff;
        result[i] = static_cast<uint8_t>(std::clamp(sharpened, 0.0f, 255.0f));
    }
    
    // 更新图像
    for (int i = 0; i < w * h; ++i) {
        if (image.channels == 1) {
            image.data[i] = result[i];
        } else {
            int idx = i * image.channels;
            image.data[idx] = result[i];
            image.data[idx + 1] = result[i];
            image.data[idx + 2] = result[i];
        }
    }
    
    OVF_INFO() << "Adaptive enhancement completed";
    return ErrorCode::Success;
}

} // namespace enhance_utils

// ========== 工业算子节点实现 ==========

MicroDefectDetectNode::MicroDefectDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MicroDefectDetectNode::make_info() {
    NodeInfo info;
    info.id = "MicroDefectDetect";
    info.name = "微缺陷检测";
    info.category = "工业检测";
    info.description = "多尺度微缺陷检测节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::String));
    
    info.params.push_back(ParamDef("sensitivity", "灵敏度", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("min_size", "最小尺寸", DataType::Number, Data(3)));
    
    return info;
}

Result<void> MicroDefectDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    sensitivity_ = static_cast<float>(get_param("sensitivity", Data(0.5)).as_number());
    min_size_ = static_cast<int>(get_param("min_size", Data(3)).as_int());
    
    Vector<defect_utils::DefectInfo> defects;
    ErrorCode err = defect_utils::detect_micro_defects(input, defects, sensitivity_, min_size_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Defect detection failed");
    }
    
    // 创建标记图像
    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);
    
    // 在原图上标记缺陷
    Vector<uint8_t> marked = gray;
    for (const auto& defect : defects) {
        // 绘制矩形框
        for (int x = defect.x; x < defect.x + defect.width && x < w; ++x) {
            if (defect.y >= 0 && defect.y < h) {
                marked[defect.y * w + x] = 255; // 顶边
            }
            if (defect.y + defect.height - 1 >= 0 && defect.y + defect.height - 1 < h) {
                marked[(defect.y + defect.height - 1) * w + x] = 255; // 底边
            }
        }
        for (int y = defect.y; y < defect.y + defect.height && y < h; ++y) {
            if (defect.x >= 0 && defect.x < w) {
                marked[y * w + defect.x] = 255; // 左边
            }
            if (defect.x + defect.width - 1 >= 0 && defect.x + defect.width - 1 < w) {
                marked[y * w + (defect.x + defect.width - 1)] = 255; // 右边
            }
        }
    }
    
    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));
    
    // 输出缺陷信息
    String defect_str = "检测到 " + std::to_string(defects.size()) + " 个缺陷:\n";
    for (const auto& d : defects) {
        defect_str += d.description + "\n";
    }
    set_output("defects", Data(defect_str));
    
    OVF_INFO() << "MicroDefectDetectNode completed: " << defects.size() << " defects found";
    return Result<void>::success();
}

ReflectionRemoveNode::ReflectionRemoveNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ReflectionRemoveNode::make_info() {
    NodeInfo info;
    info.id = "ReflectionRemove";
    info.name = "反光抑制";
    info.category = "图像处理";
    info.description = "反光抑制处理节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "处理图像", DataType::Image));
    
    info.params.push_back(ParamDef("method", "处理方法", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("threshold", "亮度阈值", DataType::Number, Data(250.0)));
    
    return info;
}

Result<void> ReflectionRemoveNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    method_ = static_cast<int>(get_param("method", Data(0)).as_int());
    threshold_ = static_cast<float>(get_param("threshold", Data(250.0)).as_number());
    
    ErrorCode err = reflection_utils::remove_reflection(input, method_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Reflection removal failed");
    }
    
    set_output("image", Data(input));
    
    OVF_INFO() << "ReflectionRemoveNode completed";
    return Result<void>::success();
}

FrequencyFilterNode::FrequencyFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FrequencyFilterNode::make_info() {
    NodeInfo info;
    info.id = "FrequencyFilter";
    info.name = "频域滤波";
    info.category = "图像处理";
    info.description = "频域滤波节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "滤波图像", DataType::Image));
    
    info.params.push_back(ParamDef("filter_type", "滤波类型", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("cutoff_freq", "截止频率", DataType::Number, Data(0.5)));
    
    return info;
}

Result<void> FrequencyFilterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    filter_type_ = static_cast<int>(get_param("filter_type", Data(0)).as_int());
    cutoff_freq_ = static_cast<float>(get_param("cutoff_freq", Data(0.5)).as_number());
    
    ErrorCode err = frequency_utils::frequency_filter(input, filter_type_, cutoff_freq_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Frequency filter failed");
    }
    
    set_output("image", Data(input));
    
    OVF_INFO() << "FrequencyFilterNode completed";
    return Result<void>::success();
}

TextureAnalysisNode::TextureAnalysisNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo TextureAnalysisNode::make_info() {
    NodeInfo info;
    info.id = "TextureAnalysis";
    info.name = "纹理分析";
    info.category = "图像分析";
    info.description = "纹理分析节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("uniformity", "均匀度", DataType::Number));
    info.outputs.push_back(DataPort("coarseness", "粗糙度", DataType::Number));
    
    return info;
}

Result<void> TextureAnalysisNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    float uniformity, coarseness;
    ErrorCode err = frequency_utils::analyze_texture(input, uniformity, coarseness);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Texture analysis failed");
    }
    
    set_output("uniformity", Data(uniformity));
    set_output("coarseness", Data(coarseness));
    
    OVF_INFO() << "TextureAnalysisNode completed";
    return Result<void>::success();
}

AdaptiveEnhanceNode::AdaptiveEnhanceNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo AdaptiveEnhanceNode::make_info() {
    NodeInfo info;
    info.id = "AdaptiveEnhance";
    info.name = "自适应增强";
    info.category = "图像处理";
    info.description = "自适应图像增强节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "增强图像", DataType::Image));
    
    return info;
}

Result<void> AdaptiveEnhanceNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ErrorCode err = enhance_utils::adaptive_enhance(input);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Adaptive enhancement failed");
    }
    
    set_output("image", Data(input));
    
    OVF_INFO() << "AdaptiveEnhanceNode completed";
    return Result<void>::success();
}

// ========== 节点注册 ==========

OVF_REGISTER_NODE(MicroDefectDetectNode, "MicroDefectDetect", MicroDefectDetectNode::make_info())
OVF_REGISTER_NODE(ReflectionRemoveNode, "ReflectionRemove", ReflectionRemoveNode::make_info())
OVF_REGISTER_NODE(FrequencyFilterNode, "FrequencyFilter", FrequencyFilterNode::make_info())
OVF_REGISTER_NODE(TextureAnalysisNode, "TextureAnalysis", TextureAnalysisNode::make_info())
OVF_REGISTER_NODE(AdaptiveEnhanceNode, "AdaptiveEnhance", AdaptiveEnhanceNode::make_info())

} // namespace algorithm
} // namespace ovf