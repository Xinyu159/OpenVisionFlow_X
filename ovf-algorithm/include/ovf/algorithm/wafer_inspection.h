/**
 * @file wafer_inspection.h
 * @brief 半导体晶圆检测算子（纯C++实现，对标VisionPro/Halcon）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <numeric>

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 晶粒(Die)结构
 */
struct WaferDie {
    uint32_t id = 0;              // 晶粒ID
    uint32_t row = 0;             // 行号
    uint32_t col = 0;             // 列号
    double center_x = 0.0;        // 中心X
    double center_y = 0.0;        // 中心Y
    uint32_t width = 0;           // 宽度
    uint32_t height = 0;          // 高度
    double rotation = 0.0;        // 旋转角度
    bool is_good = true;          // 是否良品
    double confidence = 0.0;      // 置信度
};

/**
 * @brief 缺陷结构
 */
struct WaferDefect {
    uint32_t id = 0;              // 缺陷ID
    String type;                  // 缺陷类型：scratch/particle/contamination/crack
    double center_x = 0.0;        // 中心X
    double center_y = 0.0;        // 中心Y
    uint32_t width = 0;           // 宽度
    uint32_t height = 0;          // 高度
    double area = 0.0;            // 面积
    double severity = 0.0;        // 严重程度 0-1
    double confidence = 0.0;      // 置信度
};

/**
 * @brief 对准标记结构
 */
struct AlignmentMark {
    uint32_t id = 0;              // 标记ID
    double center_x = 0.0;        // 中心X
    double center_y = 0.0;        // 中心Y
    double rotation = 0.0;        // 旋转角度
    double scale = 1.0;           // 缩放比例
    double score = 0.0;           // 匹配分数
    bool found = false;           // 是否找到
};

/**
 * @brief 切割道结构
 */
struct DicingStreet {
    uint32_t id = 0;              // 切割道ID
    double start_x = 0.0;         // 起点X
    double start_y = 0.0;         // 起点Y
    double end_x = 0.0;           // 终点X
    double end_y = 0.0;           // 终点Y
    uint32_t width = 0;           // 宽度
    double quality = 0.0;         // 质量
    bool has_defect = false;      // 是否有缺陷
};

/**
 * @brief 表面质量结构
 */
struct SurfaceQuality {
    double roughness = 0.0;       // 粗糙度
    double flatness = 0.0;        // 平坦度
    double peak_to_valley = 0.0;  // 峰谷差
    double rms_roughness = 0.0;   // RMS粗糙度
    bool is_acceptable = true;    // 是否合格
};

/**
 * @brief 厚度测量结果
 */
struct ThicknessResult {
    double center_thickness = 0.0;   // 中心厚度(μm)
    double edge_thickness = 0.0;     // 边缘厚度(μm)
    double thickness_variation = 0.0; // 厚度变化(μm)
    double bow = 0.0;                // 弯曲度(μm)
    double warp = 0.0;               // 翘曲度(μm)
    double ttv = 0.0;                // 总厚度变化(TTV)
    bool is_acceptable = true;       // 是否合格
};

/**
 * @brief 晶圆检测工具类
 */
namespace wafer_utils {

// 二维点结构
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

// 矩形区域
struct Rect {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

// 辅助函数：创建ImageData
inline ImageData create_image(uint32_t width, uint32_t height, uint32_t channels, ImageFormat format) {
    ImageData img;
    img.width = width;
    img.height = height;
    img.channels = channels;
    img.format = format;
    img.data.resize(width * height * channels);
    return img;
}

// 生成晶圆网格
inline Vector<WaferDie> generate_die_grid(uint32_t wafer_width, uint32_t wafer_height,
                                          uint32_t die_width, uint32_t die_height,
                                          uint32_t margin_x = 0, uint32_t margin_y = 0) {
    Vector<WaferDie> dies;
    
    if (wafer_width == 0 || wafer_height == 0 || die_width == 0 || die_height == 0) {
        return dies;
    }
    
    uint32_t cols = (wafer_width - 2 * margin_x) / die_width;
    uint32_t rows = (wafer_height - 2 * margin_y) / die_height;
    
    uint32_t die_id = 1;
    for (uint32_t row = 0; row < rows; ++row) {
        for (uint32_t col = 0; col < cols; ++col) {
            WaferDie die;
            die.id = die_id++;
            die.row = row;
            die.col = col;
            die.width = die_width;
            die.height = die_height;
            die.center_x = margin_x + col * die_width + die_width / 2.0;
            die.center_y = margin_y + row * die_height + die_height / 2.0;
            die.is_good = true;
            dies.push_back(die);
        }
    }
    
    return dies;
}

// 图像梯度计算（Sobel算子）
inline void sobel_gradient(const ImageData& input, ImageData& grad_x, ImageData& grad_y, ImageData& magnitude) {
    if (input.empty() || input.channels != 1) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    
    grad_x = create_image(width, height, 1, ImageFormat::Mono8);
    grad_y = create_image(width, height, 1, ImageFormat::Mono8);
    magnitude = create_image(width, height, 1, ImageFormat::Mono8);
    
    // Sobel X核: [-1, 0, 1; -2, 0, 2; -1, 0, 1]
    // Sobel Y核: [-1, -2, -1; 0, 0, 0; 1, 2, 1]
    
    for (uint32_t y = 1; y < height - 1; ++y) {
        for (uint32_t x = 1; x < width - 1; ++x) {
            int gx = 0, gy = 0;
            
            // 3x3邻域
            int p00 = input.data[(y-1)*width + (x-1)];
            int p01 = input.data[(y-1)*width + x];
            int p02 = input.data[(y-1)*width + (x+1)];
            int p10 = input.data[y*width + (x-1)];
            int p12 = input.data[y*width + (x+1)];
            int p20 = input.data[(y+1)*width + (x-1)];
            int p21 = input.data[(y+1)*width + x];
            int p22 = input.data[(y+1)*width + (x+1)];
            
            gx = -p00 + p02 - 2*p10 + 2*p12 - p20 + p22;
            gy = -p00 - 2*p01 - p02 + p20 + 2*p21 + p22;
            
            int mag = static_cast<int>(std::sqrt(gx*gx + gy*gy));
            
            grad_x.data[y*width + x] = static_cast<uint8_t>(std::min(255, std::abs(gx)));
            grad_y.data[y*width + x] = static_cast<uint8_t>(std::min(255, std::abs(gy)));
            magnitude.data[y*width + x] = static_cast<uint8_t>(std::min(255, mag));
        }
    }
}

// 自适应阈值
inline void adaptive_threshold(const ImageData& input, ImageData& output,
                               uint32_t block_size = 11, int c = 2) {
    if (input.empty()) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    
    // 确保输出是单通道
    if (input.channels != 1) {
        output = create_image(width, height, 1, ImageFormat::Mono8);
        // 简单灰度转换
        for (size_t i = 0; i < width * height; ++i) {
            if (input.channels == 3) {
                output.data[i] = static_cast<uint8_t>(
                    0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
            } else {
                output.data[i] = input.data[i];
            }
        }
    } else {
        output = input;
    }
    
    // 计算局部均值并应用阈值
    ImageData temp = output;
    int half = block_size / 2;
    
    for (uint32_t y = half; y < height - half; ++y) {
        for (uint32_t x = half; x < width - half; ++x) {
            int sum = 0;
            int count = 0;
            
            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    sum += temp.data[(y+dy)*width + (x+dx)];
                    count++;
                }
            }
            
            int threshold = sum / count - c;
            output.data[y*width + x] = (temp.data[y*width + x] > threshold) ? 255 : 0;
        }
    }
}

// 高斯模糊
inline void gaussian_blur(const ImageData& input, ImageData& output, int kernel_size = 3, double sigma = 1.0) {
    if (input.empty() || input.channels != 1) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    output = create_image(width, height, 1, ImageFormat::Mono8);
    
    int half = kernel_size / 2;
    
    // 生成高斯核
    Vector<Vector<double>> kernel(kernel_size, Vector<double>(kernel_size));
    double sum = 0.0;
    
    for (int y = -half; y <= half; ++y) {
        for (int x = -half; x <= half; ++x) {
            double val = std::exp(-(x*x + y*y) / (2 * sigma * sigma));
            kernel[y + half][x + half] = val;
            sum += val;
        }
    }
    
    // 归一化
    for (int y = 0; y < kernel_size; ++y) {
        for (int x = 0; x < kernel_size; ++x) {
            kernel[y][x] /= sum;
        }
    }
    
    // 卷积
    for (uint32_t y = half; y < height - half; ++y) {
        for (uint32_t x = half; x < width - half; ++x) {
            double val = 0.0;
            for (int ky = -half; ky <= half; ++ky) {
                for (int kx = -half; kx <= half; ++kx) {
                    val += input.data[(y+ky)*width + (x+kx)] * kernel[ky+half][kx+half];
                }
            }
            output.data[y*width + x] = static_cast<uint8_t>(std::min(255.0, val));
        }
    }
}

// 形态学腐蚀
inline void erode(const ImageData& input, ImageData& output, int kernel_size = 3) {
    if (input.empty() || input.channels != 1) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    output = create_image(width, height, 1, ImageFormat::Mono8);
    
    int half = kernel_size / 2;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint8_t min_val = 255;
            
            for (int ky = -half; ky <= half; ++ky) {
                for (int kx = -half; kx <= half; ++kx) {
                    int ny = static_cast<int>(y) + ky;
                    int nx = static_cast<int>(x) + kx;
                    
                    if (ny >= 0 && ny < static_cast<int>(height) &&
                        nx >= 0 && nx < static_cast<int>(width)) {
                        min_val = std::min(min_val, input.data[ny*width + nx]);
                    }
                }
            }
            
            output.data[y*width + x] = min_val;
        }
    }
}

// 形态学膨胀
inline void dilate(const ImageData& input, ImageData& output, int kernel_size = 3) {
    if (input.empty() || input.channels != 1) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    output = create_image(width, height, 1, ImageFormat::Mono8);
    
    int half = kernel_size / 2;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint8_t max_val = 0;
            
            for (int ky = -half; ky <= half; ++ky) {
                for (int kx = -half; kx <= half; ++kx) {
                    int ny = static_cast<int>(y) + ky;
                    int nx = static_cast<int>(x) + kx;
                    
                    if (ny >= 0 && ny < static_cast<int>(height) &&
                        nx >= 0 && nx < static_cast<int>(width)) {
                        max_val = std::max(max_val, input.data[ny*width + nx]);
                    }
                }
            }
            
            output.data[y*width + x] = max_val;
        }
    }
}

// 形态学开运算（先腐蚀后膨胀）
inline void morphological_open(const ImageData& input, ImageData& output, int kernel_size = 3) {
    ImageData temp;
    erode(input, temp, kernel_size);
    dilate(temp, output, kernel_size);
}

// 形态学闭运算（先膨胀后腐蚀）
inline void morphological_close(const ImageData& input, ImageData& output, int kernel_size = 3) {
    ImageData temp;
    dilate(input, temp, kernel_size);
    erode(temp, output, kernel_size);
}

// 霍夫直线检测简化版
inline Vector<std::pair<Point2D, Point2D>> detect_lines(const ImageData& edge_img,
                                                        int threshold = 50) {
    Vector<std::pair<Point2D, Point2D>> lines;
    
    if (edge_img.empty() || edge_img.channels != 1) return lines;
    
    uint32_t width = edge_img.width;
    uint32_t height = edge_img.height;
    
    // 简化版：使用RANSAC风格的直线检测
    // 收集边缘点
    Vector<Point2D> edge_points;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            if (edge_img.data[y * width + x] > 128) {
                edge_points.push_back({static_cast<double>(x), static_cast<double>(y)});
            }
        }
    }
    
    if (edge_points.size() < static_cast<size_t>(threshold)) return lines;
    
    // 简单的网格扫描检测直线
    // 水平线
    for (uint32_t y = 0; y < height; y += 10) {
        int count = 0;
        double sum_x = 0;
        for (uint32_t x = 0; x < width; ++x) {
            if (edge_img.data[y * width + x] > 128) {
                count++;
                sum_x += x;
            }
        }
        if (count > static_cast<int>(width / 4)) {
            lines.push_back({{0.0, static_cast<double>(y)},
                             {static_cast<double>(width), static_cast<double>(y)}});
        }
    }
    
    // 垂直线
    for (uint32_t x = 0; x < width; x += 10) {
        int count = 0;
        for (uint32_t y = 0; y < height; ++y) {
            if (edge_img.data[y * width + x] > 128) {
                count++;
            }
        }
        if (count > static_cast<int>(height / 4)) {
            lines.push_back({{static_cast<double>(x), 0.0},
                             {static_cast<double>(x), static_cast<double>(height)}});
        }
    }
    
    return lines;
}

// 圆检测简化版
inline Vector<std::pair<Point2D, double>> detect_circles(const ImageData& edge_img,
                                                         uint32_t min_radius = 10,
                                                         uint32_t max_radius = 100,
                                                         int threshold = 30) {
    Vector<std::pair<Point2D, double>> circles;
    
    if (edge_img.empty() || edge_img.channels != 1) return circles;
    
    uint32_t width = edge_img.width;
    uint32_t height = edge_img.height;
    uint32_t center_x = width / 2;
    uint32_t center_y = height / 2;
    
    // 简化版：检测晶圆边界圆
    // 从图像中心向外搜索
    for (uint32_t r = min_radius; r <= max_radius && r < std::min(width, height) / 2; ++r) {
        int edge_count = 0;
        int sample_count = 0;
        
        // 在圆周上采样
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 180) {
            uint32_t x = static_cast<uint32_t>(center_x + r * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + r * std::sin(angle));
            
            if (x < width && y < height) {
                sample_count++;
                if (edge_img.data[y * width + x] > 128) {
                    edge_count++;
                }
            }
        }
        
        // 如果边缘点比例超过阈值，记录这个圆
        if (sample_count > 0 && edge_count > sample_count * threshold / 100) {
            circles.push_back({{static_cast<double>(center_x), static_cast<double>(center_y)},
                              static_cast<double>(r)});
        }
    }
    
    return circles;
}

// 计算图像统计特性
struct ImageStats {
    double mean = 0.0;
    double std_dev = 0.0;
    double min_val = 0.0;
    double max_val = 0.0;
    double median = 0.0;
};

inline ImageStats compute_stats(const ImageData& input) {
    ImageStats stats;
    
    if (input.empty()) return stats;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    size_t total = width * height;
    
    // 单通道处理
    Vector<uint8_t> data;
    if (input.channels == 1) {
        data = input.data;
    } else {
        data.resize(total);
        for (size_t i = 0; i < total; ++i) {
            if (input.channels == 3) {
                data[i] = static_cast<uint8_t>(
                    0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
            } else {
                data[i] = input.data[i * input.channels];
            }
        }
    }
    
    // 计算均值
    double sum = 0.0;
    stats.min_val = 255.0;
    stats.max_val = 0.0;
    
    for (auto v : data) {
        sum += v;
        stats.min_val = std::min(stats.min_val, static_cast<double>(v));
        stats.max_val = std::max(stats.max_val, static_cast<double>(v));
    }
    
    stats.mean = sum / total;
    
    // 计算标准差
    double variance = 0.0;
    for (auto v : data) {
        variance += (v - stats.mean) * (v - stats.mean);
    }
    stats.std_dev = std::sqrt(variance / total);
    
    // 计算中位数
    Vector<uint8_t> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    stats.median = sorted_data[total / 2];
    
    return stats;
}

// 傅里叶变换相关函数（简化版频域分析）
inline void fft_2d_simple(const ImageData& input, Vector<double>& magnitude_spectrum) {
    if (input.empty() || input.channels != 1) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    
    magnitude_spectrum.resize(width * height, 0.0);
    
    // 简化版DFT（实际应用中应使用FFT优化）
    // 仅计算低频部分用于快速分析
    for (uint32_t v = 0; v < height / 8; ++v) {
        for (uint32_t u = 0; u < width / 8; ++u) {
            double real = 0.0;
            double imag = 0.0;
            
            for (uint32_t y = 0; y < height; ++y) {
                for (uint32_t x = 0; x < width; ++x) {
                    double val = input.data[y * width + x];
                    double angle = -2.0 * M_PI * (u * x / static_cast<double>(width) +
                                                   v * y / static_cast<double>(height));
                    real += val * std::cos(angle);
                    imag += val * std::sin(angle);
                }
            }
            
            magnitude_spectrum[v * width + u] = std::sqrt(real * real + imag * imag);
        }
    }
}

// 绘制函数
inline void draw_rect(ImageData& img, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                      uint8_t r = 255, uint8_t g = 0, uint8_t b = 0, int thickness = 1) {
    if (img.channels != 3) return;
    
    for (int t = 0; t < thickness; ++t) {
        // 上边和下边
        for (uint32_t px = x; px < x + w && px < img.width; ++px) {
            if (y + t < img.height) {
                size_t idx = ((y + t) * img.width + px) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
            if (y + h - 1 - t < img.height) {
                size_t idx = ((y + h - 1 - t) * img.width + px) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
        }
        
        // 左边和右边
        for (uint32_t py = y; py < y + h && py < img.height; ++py) {
            if (x + t < img.width) {
                size_t idx = (py * img.width + x + t) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
            if (x + w - 1 - t < img.width) {
                size_t idx = (py * img.width + x + w - 1 - t) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
        }
    }
}

inline void draw_circle(ImageData& img, uint32_t cx, uint32_t cy, uint32_t radius,
                        uint8_t r = 255, uint8_t g = 0, uint8_t b = 0) {
    if (img.channels != 3) return;
    
    for (double angle = 0; angle < 2 * M_PI; angle += 0.01) {
        int x = static_cast<int>(cx + radius * std::cos(angle));
        int y = static_cast<int>(cy + radius * std::sin(angle));
        
        if (x >= 0 && x < static_cast<int>(img.width) &&
            y >= 0 && y < static_cast<int>(img.height)) {
            size_t idx = (y * img.width + x) * 3;
            img.data[idx] = b;
            img.data[idx + 1] = g;
            img.data[idx + 2] = r;
        }
    }
}

inline void draw_cross(ImageData& img, uint32_t cx, uint32_t cy, uint32_t size,
                       uint8_t r = 255, uint8_t g = 0, uint8_t b = 0) {
    if (img.channels != 3) return;
    
    for (int dx = -static_cast<int>(size); dx <= static_cast<int>(size); ++dx) {
        int x = static_cast<int>(cx) + dx;
        if (x >= 0 && x < static_cast<int>(img.width) && cy < img.height) {
            size_t idx = (cy * img.width + x) * 3;
            img.data[idx] = b;
            img.data[idx + 1] = g;
            img.data[idx + 2] = r;
        }
    }
    
    for (int dy = -static_cast<int>(size); dy <= static_cast<int>(size); ++dy) {
        int y = static_cast<int>(cy) + dy;
        if (cx < img.width && y >= 0 && y < static_cast<int>(img.height)) {
            size_t idx = (y * img.width + cx) * 3;
            img.data[idx] = b;
            img.data[idx + 1] = g;
            img.data[idx + 2] = r;
        }
    }
}

inline void draw_line(ImageData& img, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2,
                      uint8_t r = 255, uint8_t g = 0, uint8_t b = 0) {
    if (img.channels != 3) return;
    
    // Bresenham直线算法
    int dx = std::abs(static_cast<int>(x2) - static_cast<int>(x1));
    int dy = std::abs(static_cast<int>(y2) - static_cast<int>(y1));
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    int x = x1, y = y1;
    
    while (true) {
        if (x >= 0 && x < static_cast<int>(img.width) &&
            y >= 0 && y < static_cast<int>(img.height)) {
            size_t idx = (y * img.width + x) * 3;
            img.data[idx] = b;
            img.data[idx + 1] = g;
            img.data[idx + 2] = r;
        }
        
        if (x == static_cast<int>(x2) && y == static_cast<int>(y2)) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

} // namespace wafer_utils

//==============================================================================
// 晶圆检测节点定义
//==============================================================================

/**
 * @brief 晶粒定位检测节点
 */
class WaferDieDetectionNode : public INode {
public:
    WaferDieDetectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 缺陷分类节点
 */
class WaferDefectClassificationNode : public INode {
public:
    WaferDefectClassificationNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 图案检测节点
 */
class WaferPatternInspectionNode : public INode {
public:
    WaferPatternInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 边缘检测节点
 */
class WaferEdgeInspectionNode : public INode {
public:
    WaferEdgeInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 晶圆对准节点
 */
class WaferAlignmentNode : public INode {
public:
    WaferAlignmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 切割检测节点
 */
class WaferDicingInspectionNode : public INode {
public:
    WaferDicingInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 表面检测节点
 */
class WaferSurfaceInspectionNode : public INode {
public:
    WaferSurfaceInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 污染检测节点
 */
class WaferContaminationDetectionNode : public INode {
public:
    WaferContaminationDetectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 裂纹检测节点
 */
class WaferCrackDetectionNode : public INode {
public:
    WaferCrackDetectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 厚度测量节点
 */
class WaferThicknessMeasurementNode : public INode {
public:
    WaferThicknessMeasurementNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf