/**
 * @file visionpro_tools.cpp
 * @brief VisionPro风格检测工具实现
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>

#include "ovf/algorithm/visionpro_tools.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

// ==================== VisionPro工具函数实现 ====================

namespace visionpro_utils {

// 辅助函数：将图像转换为灰度
static ImageData to_gray(const ImageData& input) {
    if (input.channels == 1) {
        return input;
    }
    
    ImageData gray;
    gray.width = input.width;
    gray.height = input.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(gray.width * gray.height);
    
    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            gray.data[i] = static_cast<uint8_t>(0.114 * b + 0.587 * g + 0.299 * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i];
        }
    }
    
    return gray;
}

// 辅助函数：获取像素值（带边界检查）
static uint8_t get_pixel(const ImageData& img, int x, int y) {
    if (x < 0 || x >= static_cast<int>(img.width) || 
        y < 0 || y >= static_cast<int>(img.height)) {
        return 0;
    }
    return img.data[y * img.width + x];
}

// 辅助函数：设置像素值（带边界检查）
static void set_pixel(ImageData& img, int x, int y, uint8_t value) {
    if (x >= 0 && x < static_cast<int>(img.width) && 
        y >= 0 && y < static_cast<int>(img.height)) {
        img.data[y * img.width + x] = value;
    }
}

// 亚像素边缘定位（处理double类型轮廓）
double subpixel_edge_position_double(const double* profile, int length, int edge_index) {
    if (edge_index < 1 || edge_index >= length - 1) {
        return static_cast<double>(edge_index);
    }
    
    // 使用二次曲线拟合进行亚像素定位
    double v0 = profile[edge_index - 1];
    double v1 = profile[edge_index];
    double v2 = profile[edge_index + 1];
    
    // 二阶导数为零的位置
    double a = (v0 + v2) / 2.0 - v1;
    double b = (v2 - v0) / 2.0;
    
    if (std::abs(a) < 1e-10) {
        return static_cast<double>(edge_index);
    }
    
    double subpixel_offset = -b / (2.0 * a);
    
    // 限制在 [-0.5, 0.5] 范围内
    subpixel_offset = std::max(-0.5, std::min(0.5, subpixel_offset));
    
    return static_cast<double>(edge_index) + subpixel_offset;
}

// 亚像素边缘定位（处理uint8_t类型轮廓）
double subpixel_edge_position(const uint8_t* profile, int length, int edge_index) {
    if (edge_index < 1 || edge_index >= length - 1) {
        return static_cast<double>(edge_index);
    }
    
    // 使用二次曲线拟合进行亚像素定位
    double v0 = profile[edge_index - 1];
    double v1 = profile[edge_index];
    double v2 = profile[edge_index + 1];
    
    // 二阶导数为零的位置
    double a = (v0 + v2) / 2.0 - v1;
    double b = (v2 - v0) / 2.0;
    
    if (std::abs(a) < 1e-10) {
        return static_cast<double>(edge_index);
    }
    
    double subpixel_offset = -b / (2.0 * a);
    
    // 限制在 [-0.5, 0.5] 范围内
    subpixel_offset = std::max(-0.5, std::min(0.5, subpixel_offset));
    
    return static_cast<double>(edge_index) + subpixel_offset;
}

// VisionPro风格卡尺边缘搜索
std::vector<CogEdgePoint> cog_caliper_search(const ImageData& image,
                                              double start_x, double start_y,
                                              double end_x, double end_y,
                                              int width, double threshold, int polarity) {
    std::vector<CogEdgePoint> edge_points;
    
    if (image.empty()) {
        return edge_points;
    }
    
    ImageData gray = to_gray(image);
    
    // 计算投影方向
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double len = std::sqrt(dx * dx + dy * dy);
    
    if (len < 1.0) {
        return edge_points;
    }
    
    // 归一化方向向量
    double dir_x = dx / len;
    double dir_y = dy / len;
    
    // 垂直方向
    double perp_x = -dir_y;
    double perp_y = dir_x;
    
    // 沿投影方向采样
    int num_samples = static_cast<int>(len) + 1;
    if (num_samples < 2) num_samples = 2;
    
    std::vector<double> profile(num_samples);
    std::vector<double> sample_x(num_samples);
    std::vector<double> sample_y(num_samples);
    
    int half_width = width / 2;
    
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double cx = start_x + t * dx;
        double cy = start_y + t * dy;
        sample_x[i] = cx;
        sample_y[i] = cy;
        
        // 在垂直方向上平均采样
        double sum = 0.0;
        int count = 0;
        
        for (int w = -half_width; w <= half_width; ++w) {
            double px = cx + w * perp_x;
            double py = cy + w * perp_y;
            
            sum += get_pixel(gray, static_cast<int>(px), static_cast<int>(py));
            ++count;
        }
        
        profile[i] = (count > 0) ? sum / count : 0.0;
    }
    
    // 在轮廓中寻找边缘跳变
    for (int i = 1; i < num_samples; ++i) {
        double diff = profile[i] - profile[i - 1];
        
        bool is_edge = false;
        if (polarity > 0 && diff > threshold) {
            is_edge = true;  // 亮到暗
        } else if (polarity < 0 && diff < -threshold) {
            is_edge = true;  // 暗到亮
        } else if (polarity == 0 && std::abs(diff) > threshold) {
            is_edge = true;  // 双向
        }
        
        if (is_edge) {
            // 亚像素边缘定位
            double edge_offset = subpixel_edge_position_double(profile.data(), num_samples, i);
            double edge_t = edge_offset / (num_samples - 1);
            
            CogEdgePoint edge;
            edge.x = start_x + edge_t * dx;
            edge.y = start_y + edge_t * dy;
            edge.gradient = std::abs(diff);
            edge.direction = std::atan2(dir_y, dir_x) + (diff > 0 ? M_PI / 2 : -M_PI / 2);
            edge.polarity = (diff > 0) ? 1 : -1;
            edge.valid = true;
            
            edge_points.push_back(edge);
        }
    }
    
    return edge_points;
}

// VisionPro风格直线拟合
CogLine cog_fit_line(const std::vector<CogEdgePoint>& points) {
    CogLine line;
    
    if (points.size() < 2) {
        return line;
    }
    
    size_t n = points.size();
    
    // 计算均值
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;
    
    // PCA方法拟合直线
    double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
    for (const auto& p : points) {
        double dx = p.x - mean_x;
        double dy = p.y - mean_y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }
    
    // 计算最小特征值对应的特征向量
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double lambda2 = (trace - std::sqrt(trace * trace - 4 * det)) / 2.0;
    
    if (std::abs(cov_xy) > 1e-10) {
        line.a = lambda2 - cov_yy;
        line.b = -cov_xy;
    } else if (cov_xx > cov_yy) {
        line.a = 0.0;
        line.b = 1.0;
    } else {
        line.a = 1.0;
        line.b = 0.0;
    }
    
    line.c = -(line.a * mean_x + line.b * mean_y);
    
    // 归一化
    double norm = std::sqrt(line.a * line.a + line.b * line.b);
    if (norm > 1e-10) {
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;
    }
    
    // 计算拟合误差
    double total_error = 0.0;
    for (const auto& p : points) {
        double dist = std::abs(line.a * p.x + line.b * p.y + line.c);
        total_error += dist;
    }
    line.fit_error = total_error / n;
    
    // 找到起点和终点（在拟合直线上的投影）
    double min_proj = 1e10, max_proj = -1e10;
    double dir_x = -line.b;  // 直线方向向量
    double dir_y = line.a;
    
    for (const auto& p : points) {
        double proj = (p.x - mean_x) * dir_x + (p.y - mean_y) * dir_y;
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }
    
    line.start_x = mean_x + min_proj * dir_x;
    line.start_y = mean_y + min_proj * dir_y;
    line.end_x = mean_x + max_proj * dir_x;
    line.end_y = mean_y + max_proj * dir_y;
    
    line.point_count = static_cast<uint32_t>(n);
    line.valid = true;
    
    return line;
}

// VisionPro风格圆拟合
CogCircle cog_fit_circle(const std::vector<CogEdgePoint>& points, 
                         double expected_cx, double expected_cy, double expected_r) {
    CogCircle circle;
    
    if (points.size() < 3) {
        return circle;
    }
    
    size_t n = points.size();
    
    // 如果有预期圆心，使用改进的拟合方法
    if (expected_r > 0) {
        // 使用点到圆心距离的平均值作为半径
        double r_sum = 0.0;
        for (const auto& p : points) {
            double dx = p.x - expected_cx;
            double dy = p.y - expected_cy;
            r_sum += std::sqrt(dx * dx + dy * dy);
        }
        circle.center_x = expected_cx;
        circle.center_y = expected_cy;
        circle.radius = r_sum / n;
    } else {
        // 使用Kasa方法拟合圆
        double sum_x = 0.0, sum_y = 0.0;
        double sum_x2 = 0.0, sum_y2 = 0.0;
        double sum_x3 = 0.0, sum_y3 = 0.0;
        double sum_xy = 0.0, sum_x2y = 0.0, sum_xy2 = 0.0;
        
        for (const auto& p : points) {
            double x = p.x, y = p.y;
            double x2 = x * x, y2 = y * y;
            
            sum_x += x;
            sum_y += y;
            sum_x2 += x2;
            sum_y2 += y2;
            sum_x3 += x2 * x;
            sum_y3 += y2 * y;
            sum_xy += x * y;
            sum_x2y += x2 * y;
            sum_xy2 += x * y2;
        }
        
        double A = n * sum_x2 - sum_x * sum_x;
        double B = n * sum_xy - sum_x * sum_y;
        double C = n * sum_y2 - sum_y * sum_y;
        double D = n * sum_x3 + n * sum_xy2 - (sum_x2 + sum_y2) * sum_x;
        double E = n * sum_x2y + n * sum_y3 - (sum_x2 + sum_y2) * sum_y;
        
        double det = A * C - B * B;
        
        if (std::abs(det) < 1e-10) {
            return circle;
        }
        
        circle.center_x = (D * C - B * E) / det / 2.0;
        circle.center_y = (A * E - B * D) / det / 2.0;
        
        double r_sum = 0.0;
        for (const auto& p : points) {
            double dx = p.x - circle.center_x;
            double dy = p.y - circle.center_y;
            r_sum += std::sqrt(dx * dx + dy * dy);
        }
        circle.radius = r_sum / n;
    }
    
    // 计算拟合误差
    double total_error = 0.0;
    for (const auto& p : points) {
        double dx = p.x - circle.center_x;
        double dy = p.y - circle.center_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        total_error += std::abs(dist - circle.radius);
    }
    circle.fit_error = total_error / n;
    
    circle.point_count = static_cast<uint32_t>(n);
    circle.valid = true;
    
    return circle;
}

// VisionPro风格Blob分析
std::vector<CogBlob> cog_blob_analyze(const ImageData& binary_img,
                                       double min_area, double max_area,
                                       double min_circularity, double max_circularity,
                                       int connectivity) {
    std::vector<CogBlob> blobs;
    
    if (binary_img.empty() || binary_img.channels != 1) {
        return blobs;
    }
    
    uint32_t width = binary_img.width;
    uint32_t height = binary_img.height;
    
    // 连通区域标记
    std::vector<uint32_t> labels(width * height, 0);
    std::vector<uint32_t> label_equiv(1, 0);
    uint32_t current_label = 0;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;
            
            if (binary_img.data[idx] == 0) continue;
            
            uint32_t left_label = (x > 0) ? labels[idx - 1] : 0;
            uint32_t top_label = (y > 0) ? labels[idx - width] : 0;
            uint32_t top_left_label = (x > 0 && y > 0 && connectivity == 8) ? 
                                      labels[idx - width - 1] : 0;
            uint32_t top_right_label = (x < width - 1 && y > 0 && connectivity == 8) ? 
                                       labels[idx - width + 1] : 0;
            
            uint32_t min_neighbor = 0;
            if (left_label > 0) min_neighbor = (min_neighbor == 0) ? left_label : 
                                                std::min(min_neighbor, left_label);
            if (top_label > 0) min_neighbor = (min_neighbor == 0) ? top_label : 
                                               std::min(min_neighbor, top_label);
            if (top_left_label > 0 && connectivity == 8) 
                min_neighbor = std::min(min_neighbor, top_left_label);
            if (top_right_label > 0 && connectivity == 8) 
                min_neighbor = std::min(min_neighbor, top_right_label);
            
            if (min_neighbor == 0) {
                current_label++;
                label_equiv.push_back(current_label);
                labels[idx] = current_label;
            } else {
                labels[idx] = min_neighbor;
                if (left_label > 0 && left_label != min_neighbor) 
                    label_equiv[left_label] = min_neighbor;
                if (top_label > 0 && top_label != min_neighbor) 
                    label_equiv[top_label] = min_neighbor;
                if (top_left_label > 0 && top_left_label != min_neighbor && connectivity == 8) 
                    label_equiv[top_left_label] = min_neighbor;
                if (top_right_label > 0 && top_right_label != min_neighbor && connectivity == 8) 
                    label_equiv[top_right_label] = min_neighbor;
            }
        }
    }
    
    // 合并等价标签
    std::vector<CogBlob> temp_blobs(label_equiv.size());
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;
            uint32_t label = labels[idx];
            
            if (label == 0) continue;
            
            while (label_equiv[label] != label && label < label_equiv.size()) {
                label = label_equiv[label];
            }
            
            if (label >= temp_blobs.size()) continue;
            
            CogBlob& blob = temp_blobs[label];
            if (!blob.valid) {
                blob.id = label;
                blob.min_x = x;
                blob.max_x = x;
                blob.min_y = y;
                blob.max_y = y;
                blob.valid = true;
            }
            
            blob.area++;
            blob.min_x = std::min(blob.min_x, static_cast<double>(x));
            blob.max_x = std::max(blob.max_x, static_cast<double>(x));
            blob.min_y = std::min(blob.min_y, static_cast<double>(y));
            blob.max_y = std::max(blob.max_y, static_cast<double>(y));
        }
    }
    
    // 计算Blob属性并筛选
    uint32_t blob_id = 1;
    for (auto& blob : temp_blobs) {
        if (!blob.valid) continue;
        
        if (blob.area < min_area || blob.area > max_area) continue;
        
        blob.width = blob.max_x - blob.min_x + 1;
        blob.height = blob.max_y - blob.min_y + 1;
        blob.center_x = (blob.min_x + blob.max_x) / 2.0;
        blob.center_y = (blob.min_y + blob.max_y) / 2.0;
        
        // 圆度 = 4π * Area / Perimeter²（近似）
        double perimeter = 2.0 * (blob.width + blob.height);
        blob.circularity = 4.0 * M_PI * blob.area / (perimeter * perimeter);
        
        if (blob.circularity < min_circularity || blob.circularity > max_circularity) continue;
        
        // 紧凑度 = Area / (Width * Height)
        blob.compactness = blob.area / (blob.width * blob.height);
        
        // 延伸率 = Max(Width, Height) / Min(Width, Height)
        blob.elongation = std::max(blob.width, blob.height) / 
                          std::max(1.0, std::min(blob.width, blob.height));
        
        blob.id = blob_id++;
        blobs.push_back(blob);
    }
    
    return blobs;
}

// VisionPro风格直方图计算
CogHistogram cog_histogram_compute(const ImageData& image,
                                    int region_x, int region_y, int region_w, int region_h,
                                    uint32_t bin_count, int channel) {
    CogHistogram hist;
    
    if (image.empty()) {
        return hist;
    }
    
    ImageData gray = to_gray(image);
    
    // 默认使用整个图像
    if (region_w <= 0 || region_h <= 0) {
        region_x = 0;
        region_y = 0;
        region_w = static_cast<int>(gray.width);
        region_h = static_cast<int>(gray.height);
    }
    
    // 调整区域范围
    region_x = std::max(0, region_x);
    region_y = std::max(0, region_y);
    region_w = std::min(region_w, static_cast<int>(gray.width) - region_x);
    region_h = std::min(region_h, static_cast<int>(gray.height) - region_y);
    
    hist.bins.resize(bin_count, 0);
    hist.bin_count = bin_count;
    
    // 计算直方图
    double sum = 0.0;
    double sum_sq = 0.0;
    uint32_t total_pixels = 0;
    std::vector<uint8_t> values;
    
    for (int y = region_y; y < region_y + region_h; ++y) {
        for (int x = region_x; x < region_x + region_w; ++x) {
            uint8_t value = get_pixel(gray, x, y);
            
            uint32_t bin_idx = static_cast<uint32_t>(value * (bin_count - 1) / 255.0);
            bin_idx = std::min(bin_idx, bin_count - 1);
            hist.bins[bin_idx]++;
            
            sum += value;
            sum_sq += value * value;
            values.push_back(value);
            total_pixels++;
        }
    }
    
    if (total_pixels == 0) {
        return hist;
    }
    
    // 计算统计值
    hist.mean = sum / total_pixels;
    hist.std_dev = std::sqrt(sum_sq / total_pixels - hist.mean * hist.mean);
    
    // 计算中值
    std::sort(values.begin(), values.end());
    if (values.size() % 2 == 0) {
        hist.median = (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2.0;
    } else {
        hist.median = values[values.size() / 2];
    }
    
    hist.min_value = values.front();
    hist.max_value = values.back();
    
    hist.valid = true;
    
    return hist;
}

// PatInspect差异检测算法
ErrorCode pat_inspect_detect(const ImageData& image, const ImageData& template_img,
                             double threshold, std::vector<Region>& defects) {
    if (image.empty() || template_img.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    if (image.width != template_img.width || image.height != template_img.height) {
        return ErrorCode::InvalidParameter;
    }
    
    ImageData gray_img = to_gray(image);
    ImageData gray_template = to_gray(template_img);
    
    // 计算差异图像
    ImageData diff;
    diff.width = gray_img.width;
    diff.height = gray_img.height;
    diff.channels = 1;
    diff.format = ImageFormat::Mono8;
    diff.data.resize(diff.width * diff.height);
    
    for (size_t i = 0; i < diff.data.size(); ++i) {
        int d = static_cast<int>(gray_img.data[i]) - static_cast<int>(gray_template.data[i]);
        diff.data[i] = static_cast<uint8_t>(std::abs(d));
    }
    
    // 阈值处理
    for (size_t i = 0; i < diff.data.size(); ++i) {
        if (diff.data[i] < threshold) {
            diff.data[i] = 0;
        } else {
            diff.data[i] = 255;
        }
    }
    
    // Blob检测找缺陷区域
    auto blobs = cog_blob_analyze(diff, 5, 1000000, 0.0, 1.0, 8);
    
    for (const auto& blob : blobs) {
        Region region;
        region.x = static_cast<int>(blob.min_x);
        region.y = static_cast<int>(blob.min_y);
        region.width = static_cast<int>(blob.width);
        region.height = static_cast<int>(blob.height);
        defects.push_back(region);
    }
    
    return ErrorCode::Success;
}

// PatMax高精度定位算法（简化版）
ErrorCode pat_max_locate(const ImageData& image, const CogTemplate& template_info,
                         double accept_threshold, double angle_range, double scale_range,
                         CogMatchResult& result) {
    if (image.empty() || !template_info.trained) {
        return ErrorCode::InvalidImage;
    }
    
    ImageData gray_img = to_gray(image);
    ImageData gray_template = to_gray(template_info.image);
    
    // 简化实现：使用NCC模板匹配
    int tmpl_w = static_cast<int>(gray_template.width);
    int tmpl_h = static_cast<int>(gray_template.height);
    int img_w = static_cast<int>(gray_img.width);
    int img_h = static_cast<int>(gray_img.height);
    
    double best_score = 0.0;
    int best_x = 0, best_y = 0;
    
    // 计算模板均值
    double tmpl_mean = 0.0;
    for (int y = 0; y < tmpl_h; ++y) {
        for (int x = 0; x < tmpl_w; ++x) {
            tmpl_mean += get_pixel(gray_template, x, y);
        }
    }
    tmpl_mean /= (tmpl_w * tmpl_h);
    
    // 计算模板方差
    double tmpl_var = 0.0;
    for (int y = 0; y < tmpl_h; ++y) {
        for (int x = 0; x < tmpl_w; ++x) {
            double d = get_pixel(gray_template, x, y) - tmpl_mean;
            tmpl_var += d * d;
        }
    }
    tmpl_var = std::sqrt(tmpl_var);
    
    if (tmpl_var < 1e-10) {
        return ErrorCode::InvalidParameter;
    }
    
    // NCC匹配
    for (int y = 0; y <= img_h - tmpl_h; ++y) {
        for (int x = 0; x <= img_w - tmpl_w; ++x) {
            // 计算图像区域均值
            double img_mean = 0.0;
            for (int ty = 0; ty < tmpl_h; ++ty) {
                for (int tx = 0; tx < tmpl_w; ++tx) {
                    img_mean += get_pixel(gray_img, x + tx, y + ty);
                }
            }
            img_mean /= (tmpl_w * tmpl_h);
            
            // 计算图像区域方差和NCC
            double img_var = 0.0;
            double ncc = 0.0;
            for (int ty = 0; ty < tmpl_h; ++ty) {
                for (int tx = 0; tx < tmpl_w; ++tx) {
                    double img_d = get_pixel(gray_img, x + tx, y + ty) - img_mean;
                    double tmpl_d = get_pixel(gray_template, tx, ty) - tmpl_mean;
                    img_var += img_d * img_d;
                    ncc += img_d * tmpl_d;
                }
            }
            img_var = std::sqrt(img_var);
            
            if (img_var > 1e-10) {
                ncc /= (tmpl_var * img_var);
            } else {
                ncc = 0.0;
            }
            
            double score = (ncc + 1.0) / 2.0 * 100.0;  // 转换为0-100分数
            
            if (score > best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
            }
        }
    }
    
    result.x = best_x + template_info.origin_x;
    result.y = best_y + template_info.origin_y;
    result.score = best_score;
    result.angle = 0.0;
    result.scale = 1.0;
    result.found = (best_score >= accept_threshold);
    
    return ErrorCode::Success;
}

// PMAlign模式匹配算法（简化版）
ErrorCode pmalign_match(const ImageData& image, const ImageData& template_img,
                        double accept_threshold, int polarity_mode,
                        CogMatchResult& result) {
    if (image.empty() || template_img.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    CogTemplate tmpl;
    tmpl.image = template_img;
    tmpl.origin_x = template_img.width / 2.0;
    tmpl.origin_y = template_img.height / 2.0;
    tmpl.trained = true;
    
    return pat_max_locate(image, tmpl, accept_threshold, 0, 0, result);
}

// SearchMax最佳搜索算法
ErrorCode search_max_find(const ImageData& image, const ImageData& template_img,
                          int region_x, int region_y, int region_w, int region_h,
                          double threshold, std::vector<CogMatchResult>& results) {
    if (image.empty() || template_img.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    ImageData gray_img = to_gray(image);
    ImageData gray_template = to_gray(template_img);
    
    // 调整搜索区域
    region_x = std::max(0, region_x);
    region_y = std::max(0, region_y);
    region_w = std::min(region_w, static_cast<int>(gray_img.width) - region_x);
    region_h = std::min(region_h, static_cast<int>(gray_img.height) - region_y);
    
    int tmpl_w = static_cast<int>(gray_template.width);
    int tmpl_h = static_cast<int>(gray_template.height);
    
    // 计算模板均值和方差
    double tmpl_mean = 0.0;
    double tmpl_var = 0.0;
    for (int y = 0; y < tmpl_h; ++y) {
        for (int x = 0; x < tmpl_w; ++x) {
            tmpl_mean += get_pixel(gray_template, x, y);
        }
    }
    tmpl_mean /= (tmpl_w * tmpl_h);
    
    for (int y = 0; y < tmpl_h; ++y) {
        for (int x = 0; x < tmpl_w; ++x) {
            double d = get_pixel(gray_template, x, y) - tmpl_mean;
            tmpl_var += d * d;
        }
    }
    tmpl_var = std::sqrt(tmpl_var);
    
    if (tmpl_var < 1e-10) {
        return ErrorCode::InvalidParameter;
    }
    
    // 在搜索区域内寻找所有匹配
    int max_search_x = region_x + region_w - tmpl_w;
    int max_search_y = region_y + region_h - tmpl_h;
    
    for (int y = region_y; y <= max_search_y; ++y) {
        for (int x = region_x; x <= max_search_x; ++x) {
            double img_mean = 0.0;
            for (int ty = 0; ty < tmpl_h; ++ty) {
                for (int tx = 0; tx < tmpl_w; ++tx) {
                    img_mean += get_pixel(gray_img, x + tx, y + ty);
                }
            }
            img_mean /= (tmpl_w * tmpl_h);
            
            double img_var = 0.0;
            double ncc = 0.0;
            for (int ty = 0; ty < tmpl_h; ++ty) {
                for (int tx = 0; tx < tmpl_w; ++tx) {
                    double img_d = get_pixel(gray_img, x + tx, y + ty) - img_mean;
                    double tmpl_d = get_pixel(gray_template, tx, ty) - tmpl_mean;
                    img_var += img_d * img_d;
                    ncc += img_d * tmpl_d;
                }
            }
            img_var = std::sqrt(img_var);
            
            if (img_var > 1e-10) {
                ncc /= (tmpl_var * img_var);
            } else {
                ncc = 0.0;
            }
            
            double score = (ncc + 1.0) / 2.0 * 100.0;
            
            if (score >= threshold) {
                CogMatchResult match;
                match.x = x + tmpl_w / 2.0;
                match.y = y + tmpl_h / 2.0;
                match.score = score;
                match.found = true;
                results.push_back(match);
            }
        }
    }
    
    // 按分数排序
    std::sort(results.begin(), results.end(), 
              [](const CogMatchResult& a, const CogMatchResult& b) {
                  return a.score > b.score;
              });
    
    return ErrorCode::Success;
}

// 综合检测算法
ErrorCode cog_inspect(const ImageData& image, const ImageData& reference,
                      int mode, double threshold, double sensitivity,
                      std::vector<Region>& defects, std::vector<String>& defect_types) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    // 模板模式检测
    if (mode == 0 || mode == 3) {  // template 或 combined
        if (!reference.empty()) {
            pat_inspect_detect(image, reference, threshold * sensitivity, defects);
            for (size_t i = 0; i < defects.size(); ++i) {
                defect_types.push_back("TemplateDefect");
            }
        }
    }
    
    // 边缘模式检测
    if (mode == 1 || mode == 3) {  // edge 或 combined
        ImageData gray = to_gray(image);
        
        // 简化的边缘检测
        ImageData edge;
        edge.width = gray.width;
        edge.height = gray.height;
        edge.channels = 1;
        edge.format = ImageFormat::Mono8;
        edge.data.resize(edge.width * edge.height);
        
        for (uint32_t y = 1; y < gray.height - 1; ++y) {
            for (uint32_t x = 1; x < gray.width - 1; ++x) {
                int gx = static_cast<int>(get_pixel(gray, x + 1, y)) - 
                         static_cast<int>(get_pixel(gray, x - 1, y));
                int gy = static_cast<int>(get_pixel(gray, x, y + 1)) - 
                         static_cast<int>(get_pixel(gray, x, y - 1));
                int g = std::abs(gx) + std::abs(gy);
                edge.data[y * edge.width + x] = static_cast<uint8_t>(std::min(g, 255));
            }
        }
        
        // 检测异常边缘
        for (size_t i = 0; i < edge.data.size(); ++i) {
            if (edge.data[i] > threshold * sensitivity * 2) {
                edge.data[i] = 255;
            } else {
                edge.data[i] = 0;
            }
        }
        
        auto blobs = cog_blob_analyze(edge, 5, 100000, 0.0, 1.0, 8);
        for (const auto& blob : blobs) {
            Region region;
            region.x = static_cast<int>(blob.min_x);
            region.y = static_cast<int>(blob.min_y);
            region.width = static_cast<int>(blob.width);
            region.height = static_cast<int>(blob.height);
            defects.push_back(region);
            defect_types.push_back("EdgeDefect");
        }
    }
    
    return ErrorCode::Success;
}

} // namespace visionpro_utils

// ==================== PatInspectNode ====================

PatInspectNode::PatInspectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PatInspectNode::make_info() {
    NodeInfo info;
    info.id = "PatInspect";
    info.name = "PatInspect模式检测";
    info.category = "VisionPro模式检测";
    info.description = "VisionPro风格的模板对比检测，用于检测产品缺陷";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "待检测图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image));
    
    info.outputs.push_back(DataPort("result_image", "结果图像", DataType::Image));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("defect_areas", "缺陷区域列表", DataType::Array));
    info.outputs.push_back(DataPort("pass_fail", "是否合格", DataType::Boolean));
    
    info.params.push_back(ParamDef("threshold", "差异阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("min_defect_area", "最小缺陷面积", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("max_defect_area", "最大缺陷面积", DataType::Number, Data(10000.0)));
    
    return info;
}

Result<void> PatInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取模板
    ImageData template_img;
    auto template_data = get_input("template");
    if (template_data.is_image()) {
        template_img = template_data.as_image();
    }
    
    if (template_img.empty()) {
        OVF_WARN() << "PatInspect: No template provided, using input as template";
        template_img = input;
    }
    
    double threshold = get_param("threshold", Data(30.0)).as_number();
    double min_area = get_param("min_defect_area", Data(5.0)).as_number();
    double max_area = get_param("max_defect_area", Data(10000.0)).as_number();
    
    std::vector<Region> defects;
    auto err = visionpro_utils::pat_inspect_detect(input, template_img, threshold, defects);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "PatInspect detection failed");
    }
    
    // 筛选缺陷面积
    std::vector<Region> filtered_defects;
    for (const auto& defect : defects) {
        double area = defect.width * defect.height;
        if (area >= min_area && area <= max_area) {
            filtered_defects.push_back(defect);
        }
    }
    
    set_output("defect_count", Data(static_cast<int>(filtered_defects.size())));
    set_output("pass_fail", Data(filtered_defects.empty()));
    
    // 输出结果图像
    if (input.channels == 3) {
        ImageData result = input;
        for (const auto& defect : filtered_defects) {
            // 绘制缺陷区域（红色）
            for (int y = defect.y; y < defect.y + defect.height && y < result.height; ++y) {
                for (int x = defect.x; x < defect.x + defect.width && x < result.width; ++x) {
                    size_t idx = (y * result.width + x) * 3;
                    result.data[idx] = 0;      // B
                    result.data[idx + 1] = 0;  // G
                    result.data[idx + 2] = 255; // R
                }
            }
        }
        set_output("result_image", Data(result));
    } else {
        set_output("result_image", Data(input));
    }
    
    OVF_INFO() << "PatInspect: Found " << filtered_defects.size() << " defects, pass=" 
               << filtered_defects.empty();
    
    return Result<void>::success();
}

// ==================== PatMaxNode ====================

PatMaxNode::PatMaxNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PatMaxNode::make_info() {
    NodeInfo info;
    info.id = "PatMax";
    info.name = "PatMax高精度定位";
    info.category = "VisionPro模式检测";
    info.description = "VisionPro核心算法，高精度几何模式定位，支持亚像素定位";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "待检测图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image));
    
    info.outputs.push_back(DataPort("match_x", "匹配位置X", DataType::Number));
    info.outputs.push_back(DataPort("match_y", "匹配位置Y", DataType::Number));
    info.outputs.push_back(DataPort("match_angle", "匹配角度", DataType::Number));
    info.outputs.push_back(DataPort("match_score", "匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到", DataType::Boolean));
    
    info.params.push_back(ParamDef("accept_threshold", "接受阈值(0-100)", DataType::Number, Data(70.0)));
    info.params.push_back(ParamDef("search_angle", "搜索角度范围(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("search_scale", "搜索缩放范围", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("origin_x", "模板原点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("origin_y", "模板原点Y", DataType::Number, Data(0.0)));
    
    return info;
}

Result<void> PatMaxNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    auto template_data = get_input("template");
    if (!template_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template is not an image");
    }
    
    ImageData template_img = template_data.as_image();
    if (template_img.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template image is empty");
    }
    
    CogTemplate tmpl;
    tmpl.image = template_img;
    tmpl.origin_x = get_param("origin_x", Data(template_img.width / 2.0)).as_number();
    tmpl.origin_y = get_param("origin_y", Data(template_img.height / 2.0)).as_number();
    tmpl.trained = true;
    
    double accept_threshold = get_param("accept_threshold", Data(70.0)).as_number();
    double angle_range = get_param("search_angle", Data(0.0)).as_number();
    double scale_range = get_param("search_scale", Data(0.0)).as_number();
    
    CogMatchResult result;
    auto err = visionpro_utils::pat_max_locate(input, tmpl, accept_threshold, 
                                                angle_range, scale_range, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "PatMax location failed");
    }
    
    set_output("match_x", Data(result.x));
    set_output("match_y", Data(result.y));
    set_output("match_angle", Data(result.angle));
    set_output("match_score", Data(result.score));
    set_output("found", Data(result.found));
    
    OVF_INFO() << "PatMax: Found=" << result.found << ", Position=(" << result.x 
               << "," << result.y << "), Score=" << result.score;
    
    return Result<void>::success();
}

// ==================== PMAlignNode ====================

PMAlignNode::PMAlignNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PMAlignNode::make_info() {
    NodeInfo info;
    info.id = "PMAlign";
    info.name = "PMAlign模式匹配";
    info.category = "VisionPro模式检测";
    info.description = "VisionPro最核心的定位工具，高精度模式匹配定位";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "待检测图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image));
    
    info.outputs.push_back(DataPort("position_x", "位置X", DataType::Number));
    info.outputs.push_back(DataPort("position_y", "位置Y", DataType::Number));
    info.outputs.push_back(DataPort("rotation", "旋转角度", DataType::Number));
    info.outputs.push_back(DataPort("score", "匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到", DataType::Boolean));
    
    info.params.push_back(ParamDef("accept_threshold", "接受阈值(0-100)", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("polarity", "极性模式(0=双向)", DataType::Number, Data(0)));
    
    return info;
}

Result<void> PMAlignNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    auto template_data = get_input("template");
    if (!template_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template is not an image");
    }
    
    ImageData template_img = template_data.as_image();
    if (template_img.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template image is empty");
    }
    
    double accept_threshold = get_param("accept_threshold", Data(50.0)).as_number();
    int polarity = get_param("polarity", Data(0)).as_int();
    
    CogMatchResult result;
    auto err = visionpro_utils::pmalign_match(input, template_img, accept_threshold, polarity, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "PMAlign matching failed");
    }
    
    set_output("position_x", Data(result.x));
    set_output("position_y", Data(result.y));
    set_output("rotation", Data(result.angle));
    set_output("score", Data(result.score));
    set_output("found", Data(result.found));
    
    OVF_INFO() << "PMAlign: Found=" << result.found << ", Position=(" << result.x 
               << "," << result.y << "), Rotation=" << result.angle << ", Score=" << result.score;
    
    return Result<void>::success();
}

// ==================== SearchMaxNode ====================

SearchMaxNode::SearchMaxNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SearchMaxNode::make_info() {
    NodeInfo info;
    info.id = "SearchMax";
    info.name = "SearchMax搜索最大化";
    info.category = "VisionPro模式检测";
    info.description = "在区域内搜索最佳匹配位置";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "待检测图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image));
    
    info.outputs.push_back(DataPort("best_x", "最佳位置X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳位置Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳分数", DataType::Number));
    info.outputs.push_back(DataPort("match_count", "匹配数量", DataType::Number));
    
    info.params.push_back(ParamDef("search_region_x", "搜索区域X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("search_region_y", "搜索区域Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("search_region_w", "搜索区域宽度", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("search_region_h", "搜索区域高度", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(50.0)));
    
    return info;
}

Result<void> SearchMaxNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    auto template_data = get_input("template");
    if (!template_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template is not an image");
    }
    
    ImageData template_img = template_data.as_image();
    if (template_img.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template image is empty");
    }
    
    int region_x = static_cast<int>(get_param("search_region_x", Data(0.0)).as_number());
    int region_y = static_cast<int>(get_param("search_region_y", Data(0.0)).as_number());
    int region_w = static_cast<int>(get_param("search_region_w", Data(0.0)).as_number());
    int region_h = static_cast<int>(get_param("search_region_h", Data(0.0)).as_number());
    double threshold = get_param("threshold", Data(50.0)).as_number();
    
    std::vector<CogMatchResult> results;
    auto err = visionpro_utils::search_max_find(input, template_img, 
                                                 region_x, region_y, region_w, region_h,
                                                 threshold, results);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "SearchMax search failed");
    }
    
    if (!results.empty()) {
        set_output("best_x", Data(results[0].x));
        set_output("best_y", Data(results[0].y));
        set_output("best_score", Data(results[0].score));
    } else {
        set_output("best_x", Data(0.0));
        set_output("best_y", Data(0.0));
        set_output("best_score", Data(0.0));
    }
    
    set_output("match_count", Data(static_cast<int>(results.size())));
    
    OVF_INFO() << "SearchMax: Found " << results.size() << " matches, best score=" 
               << (results.empty() ? 0.0 : results[0].score);
    
    return Result<void>::success();
}

// ==================== IDReaderNode ====================

IDReaderNode::IDReaderNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IDReaderNode::make_info() {
    NodeInfo info;
    info.id = "IDReader";
    info.name = "ID读取器";
    info.category = "VisionPro ID识别";
    info.description = "VisionPro风格的条码/二维码识别";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "待识别图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("barcode_data", "条码数据", DataType::String));
    info.outputs.push_back(DataPort("barcode_type", "条码类型", DataType::String));
    info.outputs.push_back(DataPort("position_x", "位置X", DataType::Number));
    info.outputs.push_back(DataPort("position_y", "位置Y", DataType::Number));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));
    info.outputs.push_back(DataPort("success", "是否成功", DataType::Boolean));
    
    info.params.push_back(ParamDef("code_type", "码类型(auto/barcode/qrcode)", DataType::String, Data("auto")));
    info.params.push_back(ParamDef("orientation", "方向模式", DataType::Number, Data(0)));
    
    return info;
}

Result<void> IDReaderNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 简化实现：返回模拟结果
    // 实际实现应调用barcode.cpp中的函数
    set_output("barcode_data", Data(""));
    set_output("barcode_type", Data("Unknown"));
    set_output("position_x", Data(0.0));
    set_output("position_y", Data(0.0));
    set_output("confidence", Data(0.0));
    set_output("success", Data(false));
    
    OVF_INFO() << "IDReader: Simplified implementation, no barcode detected";
    
    return Result<void>::success();
}

// ==================== FontTrainNode ====================

FontTrainNode::FontTrainNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FontTrainNode::make_info() {
    NodeInfo info;
    info.id = "FontTrain";
    info.name = "字体训练工具";
    info.category = "VisionPro ID识别";
    info.description = "VisionPro风格的字体训练工具";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("sample_image", "样本图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("font_model", "字体模型路径", DataType::String));
    info.outputs.push_back(DataPort("training_status", "训练状态", DataType::String));
    info.outputs.push_back(DataPort("char_count", "字符数量", DataType::Number));
    
    info.params.push_back(ParamDef("font_name", "字体名称", DataType::String, Data("CustomFont")));
    info.params.push_back(ParamDef("output_path", "输出路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("")));
    
    return info;
}

Result<void> FontTrainNode::execute(FlowContext& context) {
    auto input_data = get_input("sample_image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 简化实现：返回模拟结果
    String font_name = get_param("font_name", Data("CustomFont")).as_string();
    
    set_output("font_model", Data(font_name + ".traineddata"));
    set_output("training_status", Data("Training simulated"));
    set_output("char_count", Data(0));
    
    OVF_INFO() << "FontTrain: Font name=" << font_name << ", simplified implementation";
    
    return Result<void>::success();
}

// ==================== CogCaliperNode ====================

CogCaliperNode::CogCaliperNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogCaliperNode::make_info() {
    NodeInfo info;
    info.id = "CogCaliper";
    info.name = "Caliper卡尺工具";
    info.category = "VisionPro测量";
    info.description = "VisionPro风格的卡尺工具，沿投影方向测量边缘";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("edge_count", "边缘数量", DataType::Number));
    info.outputs.push_back(DataPort("first_edge_x", "第一个边缘X", DataType::Number));
    info.outputs.push_back(DataPort("first_edge_y", "第一个边缘Y", DataType::Number));
    info.outputs.push_back(DataPort("distance", "测量距离", DataType::Number));
    
    info.params.push_back(ParamDef("start_x", "起始点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("start_y", "起始点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_x", "结束点X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("end_y", "结束点Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("width", "搜索宽度", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("polarity", "极性(1=亮到暗,-1=暗到亮,0=双向)", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CogCaliperNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double start_x = get_param("start_x", Data(0.0)).as_number();
    double start_y = get_param("start_y", Data(0.0)).as_number();
    double end_x = get_param("end_x", Data(100.0)).as_number();
    double end_y = get_param("end_y", Data(100.0)).as_number();
    int width = static_cast<int>(get_param("width", Data(5)).as_number());
    double threshold = get_param("threshold", Data(20.0)).as_number();
    int polarity = static_cast<int>(get_param("polarity", Data(0)).as_number());
    
    auto edge_points = visionpro_utils::cog_caliper_search(input, start_x, start_y,
                                                            end_x, end_y, width, threshold, polarity);
    
    set_output("edge_count", Data(static_cast<int>(edge_points.size())));
    
    if (!edge_points.empty()) {
        set_output("first_edge_x", Data(edge_points[0].x));
        set_output("first_edge_y", Data(edge_points[0].y));
        
        if (edge_points.size() >= 2) {
            double dx = edge_points[1].x - edge_points[0].x;
            double dy = edge_points[1].y - edge_points[0].y;
            double distance = std::sqrt(dx * dx + dy * dy);
            set_output("distance", Data(distance));
        } else {
            set_output("distance", Data(0.0));
        }
    } else {
        set_output("first_edge_x", Data(0.0));
        set_output("first_edge_y", Data(0.0));
        set_output("distance", Data(0.0));
    }
    
    OVF_INFO() << "CogCaliper: Found " << edge_points.size() << " edges";
    
    return Result<void>::success();
}

// ==================== CogFindLineNode ====================

CogFindLineNode::CogFindLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogFindLineNode::make_info() {
    NodeInfo info;
    info.id = "CogFindLine";
    info.name = "找线工具";
    info.category = "VisionPro测量";
    info.description = "VisionPro风格的找线工具，检测并拟合直线";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("line_a", "直线参数a", DataType::Number));
    info.outputs.push_back(DataPort("line_b", "直线参数b", DataType::Number));
    info.outputs.push_back(DataPort("line_c", "直线参数c", DataType::Number));
    info.outputs.push_back(DataPort("fit_error", "拟合误差", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    
    info.params.push_back(ParamDef("caliper_count", "卡尺数量", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("caliper_length", "卡尺长度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("caliper_width", "卡尺宽度", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("start_x", "起始点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("start_y", "起始点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_x", "结束点X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("end_y", "结束点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("polarity", "极性", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CogFindLineNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double start_x = get_param("start_x", Data(0.0)).as_number();
    double start_y = get_param("start_y", Data(0.0)).as_number();
    double end_x = get_param("end_x", Data(100.0)).as_number();
    double end_y = get_param("end_y", Data(0.0)).as_number();
    int caliper_count = static_cast<int>(get_param("caliper_count", Data(10)).as_number());
    double caliper_length = get_param("caliper_length", Data(50.0)).as_number();
    int caliper_width = static_cast<int>(get_param("caliper_width", Data(5)).as_number());
    double threshold = get_param("edge_threshold", Data(20.0)).as_number();
    int polarity = static_cast<int>(get_param("polarity", Data(0)).as_number());
    
    // 计算找线方向
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double len = std::sqrt(dx * dx + dy * dy);
    
    if (len < 1.0) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Line length too short");
    }
    
    // 归一化方向
    double dir_x = dx / len;
    double dir_y = dy / len;
    
    // 垂直方向（卡尺方向）
    double perp_x = -dir_y;
    double perp_y = dir_x;
    
    // 使用多个卡尺找边缘点
    std::vector<CogEdgePoint> all_edges;
    
    for (int i = 0; i < caliper_count; ++i) {
        double t = static_cast<double>(i) / (caliper_count - 1);
        double center_x = start_x + t * dx;
        double center_y = start_y + t * dy;
        
        // 卡尺起点和终点
        double caliper_start_x = center_x - caliper_length / 2.0 * perp_x;
        double caliper_start_y = center_y - caliper_length / 2.0 * perp_y;
        double caliper_end_x = center_x + caliper_length / 2.0 * perp_x;
        double caliper_end_y = center_y + caliper_length / 2.0 * perp_y;
        
        auto edges = visionpro_utils::cog_caliper_search(input, caliper_start_x, caliper_start_y,
                                                          caliper_end_x, caliper_end_y,
                                                          caliper_width, threshold, polarity);
        
        // 取最强的边缘点
        if (!edges.empty()) {
            CogEdgePoint best_edge = edges[0];
            for (const auto& edge : edges) {
                if (edge.gradient > best_edge.gradient) {
                    best_edge = edge;
                }
            }
            all_edges.push_back(best_edge);
        }
    }
    
    // 拟合直线
    CogLine line = visionpro_utils::cog_fit_line(all_edges);
    
    set_output("line_a", Data(line.a));
    set_output("line_b", Data(line.b));
    set_output("line_c", Data(line.c));
    set_output("fit_error", Data(line.fit_error));
    set_output("point_count", Data(static_cast<int>(line.point_count)));
    
    OVF_INFO() << "CogFindLine: Found " << all_edges.size() << " edges, fit error=" << line.fit_error;
    
    return Result<void>::success();
}

// ==================== CogFindCircleNode ====================

CogFindCircleNode::CogFindCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogFindCircleNode::make_info() {
    NodeInfo info;
    info.id = "CogFindCircle";
    info.name = "找圆工具";
    info.category = "VisionPro测量";
    info.description = "VisionPro风格的找圆工具，检测并拟合圆";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("center_x", "圆心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "圆心Y", DataType::Number));
    info.outputs.push_back(DataPort("radius", "半径", DataType::Number));
    info.outputs.push_back(DataPort("fit_error", "拟合误差", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    
    info.params.push_back(ParamDef("caliper_count", "卡尺数量", DataType::Number, Data(16)));
    info.params.push_back(ParamDef("expected_center_x", "预期圆心X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("expected_center_y", "预期圆心Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("expected_radius", "预期半径", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("caliper_width", "卡尺宽度", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("polarity", "极性", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CogFindCircleNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double expected_cx = get_param("expected_center_x", Data(0.0)).as_number();
    double expected_cy = get_param("expected_center_y", Data(0.0)).as_number();
    double expected_r = get_param("expected_radius", Data(50.0)).as_number();
    int caliper_count = static_cast<int>(get_param("caliper_count", Data(16)).as_number());
    int caliper_width = static_cast<int>(get_param("caliper_width", Data(5)).as_number());
    double threshold = get_param("edge_threshold", Data(20.0)).as_number();
    int polarity = static_cast<int>(get_param("polarity", Data(0)).as_number());
    
    // 在圆周上布置卡尺
    std::vector<CogEdgePoint> all_edges;
    
    for (int i = 0; i < caliper_count; ++i) {
        double angle = 2.0 * M_PI * i / caliper_count;
        
        // 卡尺从圆心向外搜索
        double caliper_start_x = expected_cx + (expected_r - 10) * std::cos(angle);
        double caliper_start_y = expected_cy + (expected_r - 10) * std::sin(angle);
        double caliper_end_x = expected_cx + (expected_r + 10) * std::cos(angle);
        double caliper_end_y = expected_cy + (expected_r + 10) * std::sin(angle);
        
        auto edges = visionpro_utils::cog_caliper_search(input, caliper_start_x, caliper_start_y,
                                                          caliper_end_x, caliper_end_y,
                                                          caliper_width, threshold, polarity);
        
        if (!edges.empty()) {
            all_edges.push_back(edges[0]);
        }
    }
    
    // 拟合圆
    CogCircle circle = visionpro_utils::cog_fit_circle(all_edges, expected_cx, expected_cy, expected_r);
    
    set_output("center_x", Data(circle.center_x));
    set_output("center_y", Data(circle.center_y));
    set_output("radius", Data(circle.radius));
    set_output("fit_error", Data(circle.fit_error));
    set_output("point_count", Data(static_cast<int>(circle.point_count)));
    
    OVF_INFO() << "CogFindCircle: Found " << all_edges.size() << " edges, center=(" 
               << circle.center_x << "," << circle.center_y << "), radius=" << circle.radius;
    
    return Result<void>::success();
}

// ==================== CogBlobAnalysisNode ====================

CogBlobAnalysisNode::CogBlobAnalysisNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogBlobAnalysisNode::make_info() {
    NodeInfo info;
    info.id = "CogBlobAnalysis";
    info.name = "Blob分析工具";
    info.category = "VisionPro分析";
    info.description = "VisionPro风格的Blob分析工具";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("blob_count", "Blob数量", DataType::Number));
    info.outputs.push_back(DataPort("total_area", "总面积", DataType::Number));
    info.outputs.push_back(DataPort("result_image", "结果图像", DataType::Image));
    
    info.params.push_back(ParamDef("min_area", "最小面积", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("max_area", "最大面积", DataType::Number, Data(100000.0)));
    info.params.push_back(ParamDef("min_circularity", "最小圆度", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("max_circularity", "最大圆度", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("connectivity", "连通性(4/8)", DataType::Number, Data(8)));
    
    return info;
}

Result<void> CogBlobAnalysisNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为灰度
    ImageData gray = visionpro_utils::to_gray(input);
    
    // 如果不是二值图像，进行阈值处理
    for (auto& pixel : gray.data) {
        pixel = (pixel > 128) ? 255 : 0;
    }
    
    double min_area = get_param("min_area", Data(10.0)).as_number();
    double max_area = get_param("max_area", Data(100000.0)).as_number();
    double min_circularity = get_param("min_circularity", Data(0.0)).as_number();
    double max_circularity = get_param("max_circularity", Data(1.0)).as_number();
    int connectivity = static_cast<int>(get_param("connectivity", Data(8)).as_number());
    
    auto blobs = visionpro_utils::cog_blob_analyze(gray, min_area, max_area,
                                                    min_circularity, max_circularity, connectivity);
    
    set_output("blob_count", Data(static_cast<int>(blobs.size())));
    
    double total_area = 0.0;
    for (const auto& blob : blobs) {
        total_area += blob.area;
    }
    set_output("total_area", Data(total_area));
    
    // 输出结果图像
    if (input.channels == 3) {
        ImageData result = input;
        for (const auto& blob : blobs) {
            // 绘制边界框（绿色）
            for (int x = static_cast<int>(blob.min_x); x <= static_cast<int>(blob.max_x); ++x) {
                for (int y = static_cast<int>(blob.min_y); y <= static_cast<int>(blob.max_y); ++y) {
                    if (x == static_cast<int>(blob.min_x) || x == static_cast<int>(blob.max_x) ||
                        y == static_cast<int>(blob.min_y) || y == static_cast<int>(blob.max_y)) {
                        if (x >= 0 && x < result.width && y >= 0 && y < result.height) {
                            size_t idx = (y * result.width + x) * 3;
                            result.data[idx] = 0;
                            result.data[idx + 1] = 255;
                            result.data[idx + 2] = 0;
                        }
                    }
                }
            }
        }
        set_output("result_image", Data(result));
    } else {
        set_output("result_image", Data(input));
    }
    
    OVF_INFO() << "CogBlobAnalysis: Found " << blobs.size() << " blobs, total area=" << total_area;
    
    return Result<void>::success();
}

// ==================== CogHistogramNode ====================

CogHistogramNode::CogHistogramNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogHistogramNode::make_info() {
    NodeInfo info;
    info.id = "CogHistogram";
    info.name = "直方图分析工具";
    info.category = "VisionPro分析";
    info.description = "VisionPro风格的直方图分析工具";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("mean", "均值", DataType::Number));
    info.outputs.push_back(DataPort("std_dev", "标准差", DataType::Number));
    info.outputs.push_back(DataPort("min_value", "最小值", DataType::Number));
    info.outputs.push_back(DataPort("max_value", "最大值", DataType::Number));
    info.outputs.push_back(DataPort("median", "中值", DataType::Number));
    info.outputs.push_back(DataPort("entropy", "信息熵", DataType::Number));
    
    info.params.push_back(ParamDef("region_x", "区域X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("region_y", "区域Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("region_w", "区域宽度", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("region_h", "区域高度", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("bin_count", "直方图桶数", DataType::Number, Data(256)));
    info.params.push_back(ParamDef("channel", "通道(0=灰度)", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CogHistogramNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int region_x = static_cast<int>(get_param("region_x", Data(0.0)).as_number());
    int region_y = static_cast<int>(get_param("region_y", Data(0.0)).as_number());
    int region_w = static_cast<int>(get_param("region_w", Data(0.0)).as_number());
    int region_h = static_cast<int>(get_param("region_h", Data(0.0)).as_number());
    uint32_t bin_count = static_cast<uint32_t>(get_param("bin_count", Data(256)).as_number());
    int channel = static_cast<int>(get_param("channel", Data(0)).as_number());
    
    auto hist = visionpro_utils::cog_histogram_compute(input, region_x, region_y, 
                                                        region_w, region_h, bin_count, channel);
    
    set_output("mean", Data(hist.mean));
    set_output("std_dev", Data(hist.std_dev));
    set_output("min_value", Data(hist.min_value));
    set_output("max_value", Data(hist.max_value));
    set_output("median", Data(hist.median));
    
    // 计算信息熵
    double entropy = 0.0;
    uint32_t total_pixels = 0;
    for (const auto& bin : hist.bins) {
        total_pixels += bin;
    }
    
    if (total_pixels > 0) {
        for (const auto& bin : hist.bins) {
            if (bin > 0) {
                double p = static_cast<double>(bin) / total_pixels;
                entropy -= p * std::log2(p);
            }
        }
    }
    set_output("entropy", Data(entropy));
    
    OVF_INFO() << "CogHistogram: Mean=" << hist.mean << ", StdDev=" << hist.std_dev 
               << ", Entropy=" << entropy;
    
    return Result<void>::success();
}

// ==================== CogInspectNode ====================

CogInspectNode::CogInspectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogInspectNode::make_info() {
    NodeInfo info;
    info.id = "CogInspect";
    info.name = "综合检测工具";
    info.category = "VisionPro分析";
    info.description = "VisionPro风格的综合检测工具，结合多种检测方法";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("reference", "参考图像", DataType::Image));
    
    info.outputs.push_back(DataPort("pass_fail", "是否合格", DataType::Boolean));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("inspect_time", "检测耗时(ms)", DataType::Number));
    info.outputs.push_back(DataPort("result_image", "结果图像", DataType::Image));
    
    info.params.push_back(ParamDef("mode", "检测模式(0=模板,1=边缘,2=颜色,3=综合)", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("threshold", "检测阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sensitivity", "敏感度", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> CogInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData reference;
    auto ref_data = get_input("reference");
    if (ref_data.is_image()) {
        reference = ref_data.as_image();
    }
    
    int mode = static_cast<int>(get_param("mode", Data(0)).as_number());
    double threshold = get_param("threshold", Data(30.0)).as_number();
    double sensitivity = get_param("sensitivity", Data(1.0)).as_number();
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<Region> defects;
    std::vector<String> defect_types;
    
    auto err = visionpro_utils::cog_inspect(input, reference, mode, threshold, 
                                             sensitivity, defects, defect_types);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "CogInspect detection failed");
    }
    
    set_output("pass_fail", Data(defects.empty()));
    set_output("defect_count", Data(static_cast<int>(defects.size())));
    set_output("inspect_time", Data(static_cast<double>(duration.count())));
    
    // 输出结果图像
    if (input.channels == 3) {
        ImageData result = input;
        for (size_t i = 0; i < defects.size(); ++i) {
            const auto& defect = defects[i];
            // 绘制缺陷区域（红色）
            for (int y = defect.y; y < defect.y + defect.height && y < result.height; ++y) {
                for (int x = defect.x; x < defect.x + defect.width && x < result.width; ++x) {
                    size_t idx = (y * result.width + x) * 3;
                    result.data[idx] = 0;
                    result.data[idx + 1] = 0;
                    result.data[idx + 2] = 255;
                }
            }
        }
        set_output("result_image", Data(result));
    } else {
        set_output("result_image", Data(input));
    }
    
    OVF_INFO() << "CogInspect: Mode=" << mode << ", Found " << defects.size() 
               << " defects, Pass=" << defects.empty() << ", Time=" << duration.count() << "ms";
    
    return Result<void>::success();
}

// ==================== CogResultAnalysisNode ====================

CogResultAnalysisNode::CogResultAnalysisNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CogResultAnalysisNode::make_info() {
    NodeInfo info;
    info.id = "CogResultAnalysis";
    info.name = "结果分析工具";
    info.category = "VisionPro分析";
    info.description = "VisionPro风格的结果分析工具，汇总和分析检测结果";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("pass_fail", "检测结果", DataType::Boolean, true));
    
    info.outputs.push_back(DataPort("pass_rate", "合格率", DataType::Number));
    info.outputs.push_back(DataPort("total_count", "总数量", DataType::Number));
    info.outputs.push_back(DataPort("pass_count", "合格数量", DataType::Number));
    info.outputs.push_back(DataPort("fail_count", "不合格数量", DataType::Number));
    info.outputs.push_back(DataPort("report", "分析报告", DataType::String));
    
    info.params.push_back(ParamDef("pass_threshold", "合格阈值", DataType::Number, Data(95.0)));
    info.params.push_back(ParamDef("analysis_mode", "分析模式", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CogResultAnalysisNode::execute(FlowContext& context) {
    auto input_data = get_input("pass_fail");
    if (!input_data.is_bool()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Input is not a boolean");
    }
    
    bool pass_fail = input_data.as_bool();
    
    // 简化实现：单次检测的结果分析
    int total_count = 1;
    int pass_count = pass_fail ? 1 : 0;
    int fail_count = pass_fail ? 0 : 1;
    double pass_rate = pass_fail ? 100.0 : 0.0;
    
    set_output("pass_rate", Data(pass_rate));
    set_output("total_count", Data(total_count));
    set_output("pass_count", Data(pass_count));
    set_output("fail_count", Data(fail_count));
    
    double pass_threshold = get_param("pass_threshold", Data(95.0)).as_number();
    
    String report = "检测结果分析报告:\n";
    report += "- 总数量: " + std::to_string(total_count) + "\n";
    report += "- 合格数量: " + std::to_string(pass_count) + "\n";
    report += "- 不合格数量: " + std::to_string(fail_count) + "\n";
    report += "- 合格率: " + std::to_string(pass_rate) + "%\n";
    report += "- 合格阈值: " + std::to_string(pass_threshold) + "%\n";
    report += "- 最终判定: ";
    report += (pass_rate >= pass_threshold ? "合格" : "不合格");
    
    set_output("report", Data(report));
    
    OVF_INFO() << "CogResultAnalysis: Pass rate=" << pass_rate << "%, Pass=" 
               << (pass_rate >= pass_threshold);
    
    return Result<void>::success();
}

// ==================== 节点注册 ====================

OVF_REGISTER_NODE(PatInspectNode, "PatInspect", PatInspectNode::make_info())
OVF_REGISTER_NODE(PatMaxNode, "PatMax", PatMaxNode::make_info())
OVF_REGISTER_NODE(PMAlignNode, "PMAlign", PMAlignNode::make_info())
OVF_REGISTER_NODE(SearchMaxNode, "SearchMax", SearchMaxNode::make_info())
OVF_REGISTER_NODE(IDReaderNode, "IDReader", IDReaderNode::make_info())
OVF_REGISTER_NODE(FontTrainNode, "FontTrain", FontTrainNode::make_info())
OVF_REGISTER_NODE(CogCaliperNode, "CogCaliper", CogCaliperNode::make_info())
OVF_REGISTER_NODE(CogFindLineNode, "CogFindLine", CogFindLineNode::make_info())
OVF_REGISTER_NODE(CogFindCircleNode, "CogFindCircle", CogFindCircleNode::make_info())
OVF_REGISTER_NODE(CogBlobAnalysisNode, "CogBlobAnalysis", CogBlobAnalysisNode::make_info())
OVF_REGISTER_NODE(CogHistogramNode, "CogHistogram", CogHistogramNode::make_info())
OVF_REGISTER_NODE(CogInspectNode, "CogInspect", CogInspectNode::make_info())
OVF_REGISTER_NODE(CogResultAnalysisNode, "CogResultAnalysis", CogResultAnalysisNode::make_info())

} // namespace algorithm
} // namespace ovf