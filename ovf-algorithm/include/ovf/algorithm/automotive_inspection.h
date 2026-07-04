/**
 * @file automotive_inspection.h
 * @brief 汽车零部件检测算子（纯C++实现，对标VisionPro在汽车行业20+年的积累）
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
 * @brief 表面缺陷结构
 */
struct SurfaceDefect {
    uint32_t id = 0;              // 缺陷ID
    String type;                  // 缺陷类型：scratch/dent/pit/ripple/contamination
    double center_x = 0.0;        // 中心X
    double center_y = 0.0;        // 中心Y
    uint32_t width = 0;           // 宽度
    uint32_t height = 0;          // 高度
    double area = 0.0;            // 面积
    double depth = 0.0;           // 深度（凹陷）
    double severity = 0.0;        // 严重程度 0-1
    double confidence = 0.0;      // 置信度
};

/**
 * @brief 焊缝缺陷结构
 */
struct WeldDefect {
    uint32_t id = 0;              // 缺陷ID
    String type;                  // 缺陷类型：porosity/crack/undercut/overlap/spatter
    double center_x = 0.0;        // 中心X
    double center_y = 0.0;        // 中心Y
    uint32_t width = 0;           // 宽度
    uint32_t height = 0;          // 高度
    double area = 0.0;            // 面积
    double length = 0.0;          // 长度（裂纹）
    double severity = 0.0;        // 严重程度 0-1
    double confidence = 0.0;      // 置信度
};

/**
 * @brief 焊缝质量评估结果
 */
struct WeldQualityResult {
    double weld_width = 0.0;         // 焊缝宽度
    double weld_depth = 0.0;         // 焊缝深度
    double penetration = 0.0;        // 熔深百分比
    double porosity_rate = 0.0;      // 气孔率
    int defect_count = 0;            // 缺陷数量
    double quality_score = 0.0;      // 质量分数 0-1
    bool is_acceptable = true;       // 是否合格
};

/**
 * @brief 漆面缺陷结构
 */
struct PaintDefect {
    uint32_t id = 0;              // 缺陷ID
    String type;                  // 缺陷类型：orange_peel/run/sag/scratch/pinhole/dust/color_defect
    double center_x = 0.0;        // 中心X
    double center_y = 0.0;        // 中心Y
    uint32_t width = 0;           // 宽度
    uint32_t height = 0;          // 高度
    double area = 0.0;            // 面积
    double severity = 0.0;        // 严重程度 0-1
    double confidence = 0.0;      // 置信度
};

/**
 * @brief 漆面质量评估结果
 */
struct PaintQualityResult {
    double gloss_level = 0.0;        // 光泽度 0-100
    double smoothness = 0.0;         // 平滑度 0-100
    double color_consistency = 0.0;  // 色差一致性
    int defect_count = 0;            // 缺陷数量
    double quality_score = 0.0;      // 质量分数 0-1
    bool is_acceptable = true;       // 是否合格
};

/**
 * @brief 装配验证结果
 */
struct AssemblyResult {
    uint32_t expected_parts = 0;    // 期望零件数
    uint32_t detected_parts = 0;    // 检测到零件数
    uint32_t missing_parts = 0;     // 缺失零件数
    uint32_t extra_parts = 0;       // 多余零件数
    double completeness = 0.0;      // 完整性分数 0-1
    bool is_complete = true;        // 是否完整
};

/**
 * @brief 尺寸测量结果
 */
struct DimensionResult {
    double measured_value = 0.0;    // 测量值
    double nominal_value = 0.0;     // 标称值
    double tolerance_upper = 0.0;   // 上公差
    double tolerance_lower = 0.0;   // 下公差
    double deviation = 0.0;         // 偏差值
    bool in_tolerance = true;       // 是否在公差范围内
};

/**
 * @brief 齿轮缺陷结构
 */
struct GearDefect {
    uint32_t id = 0;              // 缺陷ID
    String type;                  // 缺陷类型：wear/chipping/crack/misalignment/pitting
    uint32_t tooth_number = 0;    // 齿号
    double position_x = 0.0;      // 位置X
    double position_y = 0.0;      // 位置Y
    double severity = 0.0;        // 严重程度 0-1
    double confidence = 0.0;      // 置信度
};

/**
 * @brief 齿轮检测结果
 */
struct GearInspectionResult {
    uint32_t total_teeth = 0;        // 总齿数
    uint32_t good_teeth = 0;         // 良好齿数
    uint32_t defective_teeth = 0;    // 缺陷齿数
    double tooth_thickness = 0.0;    // 齿厚(mm)
    double tooth_pitch = 0.0;        // 齿距(mm)
    double pitch_error = 0.0;        // 齿距误差(mm)
    double profile_error = 0.0;      // 齿形误差
    double wear_level = 0.0;         // 磨损程度 0-1
    double quality_score = 0.0;      // 质量分数 0-1
    bool is_acceptable = true;       // 是否合格
};

/**
 * @brief 连接器检测结果
 */
struct ConnectorResult {
    uint32_t total_pins = 0;         // 总插针数
    uint32_t correct_pins = 0;       // 正确插针数
    uint32_t bent_pins = 0;          // 弯曲插针数
    uint32_t missing_pins = 0;       // 缺失插针数
    uint32_t misaligned_pins = 0;    // 偏位插针数
    double alignment_score = 0.0;    // 对准分数 0-1
    bool is_acceptable = true;       // 是否合格
};

/**
 * @brief 螺栓检测结果
 */
struct BoltResult {
    uint32_t expected_bolts = 0;     // 期望螺栓数
    uint32_t detected_bolts = 0;     // 检测到螺栓数
    uint32_t missing_bolts = 0;      // 缺失螺栓数
    double presence_rate = 0.0;      // 存在率 0-1
    bool is_complete = true;         // 是否完整
};

/**
 * @brief 表面粗糙度结果
 */
struct RoughnessResult {
    double ra = 0.0;                // Ra粗糙度(μm)
    double rz = 0.0;                // Rz粗糙度(μm)
    double rp = 0.0;                // Rp峰值(μm)
    double rv = 0.0;                // Rv谷值(μm)
    double rq = 0.0;                // Rq RMS粗糙度(μm)
    double peak_to_valley = 0.0;    // 峰谷差(μm)
    bool is_acceptable = true;      // 是否合格
};

/**
 * @brief 间隙测量结果
 */
struct GapResult {
    double gap_width = 0.0;         // 间隙宽度(mm)
    double nominal_gap = 0.0;       // 标称间隙(mm)
    double tolerance = 0.0;         // 公差(mm)
    double deviation = 0.0;         // 偏差(mm)
    double uniformity = 0.0;        // 均匀性分数 0-1
    bool is_acceptable = true;      // 是否合格
};

/**
 * @brief 汽车检测工具类
 */
namespace automotive_utils {

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

// 图像梯度计算（Sobel算子）
inline void sobel_gradient(const ImageData& input, ImageData& grad_x, ImageData& grad_y, ImageData& magnitude) {
    if (input.empty() || input.channels != 1) return;
    
    uint32_t width = input.width;
    uint32_t height = input.height;
    
    grad_x = create_image(width, height, 1, ImageFormat::Mono8);
    grad_y = create_image(width, height, 1, ImageFormat::Mono8);
    magnitude = create_image(width, height, 1, ImageFormat::Mono8);
    
    for (uint32_t y = 1; y < height - 1; ++y) {
        for (uint32_t x = 1; x < width - 1; ++x) {
            int gx = 0, gy = 0;
            
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
    
    if (input.channels != 1) {
        output = create_image(width, height, 1, ImageFormat::Mono8);
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
    
    Vector<Vector<double>> kernel(kernel_size, Vector<double>(kernel_size));
    double sum = 0.0;
    
    for (int y = -half; y <= half; ++y) {
        for (int x = -half; x <= half; ++x) {
            double val = std::exp(-(x*x + y*y) / (2 * sigma * sigma));
            kernel[y + half][x + half] = val;
            sum += val;
        }
    }
    
    for (int y = 0; y < kernel_size; ++y) {
        for (int x = 0; x < kernel_size; ++x) {
            kernel[y][x] /= sum;
        }
    }
    
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
    
    double sum = 0.0;
    stats.min_val = 255.0;
    stats.max_val = 0.0;
    
    for (auto v : data) {
        sum += v;
        stats.min_val = std::min(stats.min_val, static_cast<double>(v));
        stats.max_val = std::max(stats.max_val, static_cast<double>(v));
    }
    
    stats.mean = sum / total;
    
    double variance = 0.0;
    for (auto v : data) {
        variance += (v - stats.mean) * (v - stats.mean);
    }
    stats.std_dev = std::sqrt(variance / total);
    
    Vector<uint8_t> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    stats.median = sorted_data[total / 2];
    
    return stats;
}

// 绘制函数
inline void draw_rect(ImageData& img, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                      uint8_t r = 255, uint8_t g = 0, uint8_t b = 0, int thickness = 1) {
    if (img.channels != 3) return;
    
    for (int t = 0; t < thickness; ++t) {
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

// 直线检测（简化版）
inline Vector<std::pair<Point2D, Point2D>> detect_lines(const ImageData& edge_img, int threshold = 50) {
    Vector<std::pair<Point2D, Point2D>> lines;
    
    if (edge_img.empty() || edge_img.channels != 1) return lines;
    
    uint32_t width = edge_img.width;
    uint32_t height = edge_img.height;
    
    // 水平线检测
    for (uint32_t y = 0; y < height; y += 10) {
        int count = 0;
        for (uint32_t x = 0; x < width; ++x) {
            if (edge_img.data[y * width + x] > 128) count++;
        }
        if (count > static_cast<int>(width / 4)) {
            lines.push_back({{0.0, static_cast<double>(y)},
                             {static_cast<double>(width), static_cast<double>(y)}});
        }
    }
    
    // 垂直线检测
    for (uint32_t x = 0; x < width; x += 10) {
        int count = 0;
        for (uint32_t y = 0; y < height; ++y) {
            if (edge_img.data[y * width + x] > 128) count++;
        }
        if (count > static_cast<int>(height / 4)) {
            lines.push_back({{static_cast<double>(x), 0.0},
                             {static_cast<double>(x), static_cast<double>(height)}});
        }
    }
    
    return lines;
}

// 圆检测（简化版）
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
    
    for (uint32_t r = min_radius; r <= max_radius && r < std::min(width, height) / 2; ++r) {
        int edge_count = 0;
        int sample_count = 0;
        
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 180) {
            uint32_t x = static_cast<uint32_t>(center_x + r * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + r * std::sin(angle));
            
            if (x < width && y < height) {
                sample_count++;
                if (edge_img.data[y * width + x] > 128) edge_count++;
            }
        }
        
        if (sample_count > 0 && edge_count > sample_count * threshold / 100) {
            circles.push_back({{static_cast<double>(center_x), static_cast<double>(center_y)},
                              static_cast<double>(r)});
        }
    }
    
    return circles;
}

} // namespace automotive_utils

//==============================================================================
// 汽车零部件检测节点定义
//==============================================================================

/**
 * @brief 钣金检测节点 - 表面缺陷/变形检测
 */
class BodyPanelInspectionNode : public INode {
public:
    BodyPanelInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 焊缝检测节点 - 焊缝质量评估（气孔/裂纹/咬边检测）
 */
class WeldSeamInspectionNode : public INode {
public:
    WeldSeamInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 涂装检测节点 - 漆面缺陷检测（橘皮/流挂/色差检测）
 */
class PaintQualityInspectionNode : public INode {
public:
    PaintQualityInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 装配验证节点 - 零件完整性检测
 */
class AssemblyVerificationNode : public INode {
public:
    AssemblyVerificationNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 尺寸检测节点 - 公差测量
 */
class DimensionalInspectionNode : public INode {
public:
    DimensionalInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 齿轮检测节点 - 齿形/磨损检测（齿厚/齿距测量）
 */
class GearInspectionNode : public INode {
public:
    GearInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 连接器检测节点 - 插针位置检测
 */
class ConnectorInspectionNode : public INode {
public:
    ConnectorInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 螺栓检测节点 - 漏装检测（存在性验证）
 */
class BoltPresenceCheckNode : public INode {
public:
    BoltPresenceCheckNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 表面粗糙度检测节点
 */
class SurfaceRoughnessInspectionNode : public INode {
public:
    SurfaceRoughnessInspectionNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 间隙测量节点 - 面板间隙公差检测
 */
class GapMeasurementNode : public INode {
public:
    GapMeasurementNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf