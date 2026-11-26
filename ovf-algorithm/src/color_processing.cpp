/**
 * @file color_processing.cpp
 * @brief 颜色处理模块实现
 */

// Windows MSVC需要定义 _USE_MATH_DEFINES 以启用 M_PI 等数学常量
#define _USE_MATH_DEFINES
#include <cmath>

#include "ovf/algorithm/color_processing.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <sstream>
#include <map>
#include <queue>

// 如果 M_PI 仍未定义，手动定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace ovf {
namespace algorithm {

// ==================== 辅助函数 ====================

namespace {

// 限制值在范围内
template<typename T>
inline T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(max_val, value));
}

// RGB到XYZ转换的辅助函数
inline float pivot_xyz(float n) {
    return n > 0.04045f ? std::pow((n + 0.055f) / 1.055f, 2.4f) : n / 12.92f;
}

// XYZ到LAB转换的辅助函数
inline float pivot_lab(float n) {
    return n > 0.008856f ? std::pow(n, 1.0f / 3.0f) : (7.787f * n) + (16.0f / 116.0f);
}

// LAB到XYZ转换的辅助函数
inline float pivot_lab_inverse(float n) {
    const float epsilon = 0.008856f;  // (6/29)^3
    const float kappa = 903.3f;       // (29/3)^3
    
    return n > epsilon ? std::pow(n, 3.0f) : (n - 16.0f / 116.0f) / 7.787f;
}

// RGB到XYZ转换
void rgb_to_xyz(const ColorRGB& rgb, float& x, float& y, float& z) {
    // 归一化RGB值到0-1范围
    float r_normalized = rgb.r / 255.0f;
    float g_normalized = rgb.g / 255.0f;
    float b_normalized = rgb.b / 255.0f;
    
    // 应用gamma校正逆变换（sRGB到线性RGB）
    float r_linear = pivot_xyz(r_normalized);
    float g_linear = pivot_xyz(g_normalized);
    float b_linear = pivot_xyz(b_normalized);
    
    // 转换到XYZ（D65白点）
    x = r_linear * 0.4124564f + g_linear * 0.3575761f + b_linear * 0.1804375f;
    y = r_linear * 0.2126729f + g_linear * 0.7151522f + b_linear * 0.0721750f;
    z = r_linear * 0.0193339f + g_linear * 0.1191920f + b_linear * 0.9503041f;
}

// XYZ到RGB转换
void xyz_to_rgb(float x, float y, float z, ColorRGB& rgb) {
    // XYZ到线性RGB（D65白点）
    float r_linear = x * 3.2404542f + y * -1.5371385f + z * -0.4985314f;
    float g_linear = x * -0.9692660f + y * 1.8760108f + z * 0.0415560f;
    float b_linear = x * 0.0556434f + y * -0.2040259f + z * 1.0572252f;
    
    // 应用gamma校正（线性RGB到sRGB）
    auto gamma_correct = [](float n) -> float {
        return n > 0.0031308f ? 1.055f * std::pow(n, 1.0f / 2.4f) - 0.055f : 12.92f * n;
    };
    
    float r = gamma_correct(r_linear);
    float g = gamma_correct(g_linear);
    float b = gamma_correct(b_linear);
    
    // 转换到0-255范围并限制
    rgb.r = static_cast<uint8_t>(clamp(static_cast<int>(r * 255.0f), 0, 255));
    rgb.g = static_cast<uint8_t>(clamp(static_cast<int>(g * 255.0f), 0, 255));
    rgb.b = static_cast<uint8_t>(clamp(static_cast<int>(b * 255.0f), 0, 255));
}

// HSV到RGB的辅助函数
inline float hue_to_rgb(float p, float q, float t) {
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
    return p;
}

// 标准颜色名称映射
const std::map<String, ColorRGB>& get_standard_color_library() {
    static const std::map<String, ColorRGB> color_library = {
        {"Red",     ColorRGB(255, 0, 0)},
        {"Green",   ColorRGB(0, 255, 0)},
        {"Blue",    ColorRGB(0, 0, 255)},
        {"Yellow",  ColorRGB(255, 255, 0)},
        {"Cyan",    ColorRGB(0, 255, 255)},
        {"Magenta", ColorRGB(255, 0, 255)},
        {"White",   ColorRGB(255, 255, 255)},
        {"Black",   ColorRGB(0, 0, 0)},
        {"Gray",    ColorRGB(128, 128, 128)},
        {"Orange",  ColorRGB(255, 165, 0)},
        {"Purple",  ColorRGB(128, 0, 128)},
        {"Pink",    ColorRGB(255, 192, 203)},
        {"Brown",   ColorRGB(139, 69, 19)},
        {"Beige",   ColorRGB(245, 245, 220)},
        {"Navy",    ColorRGB(0, 0, 128)},
        {"Teal",    ColorRGB(0, 128, 128)},
        {"Lime",    ColorRGB(50, 205, 50)},
        {"Coral",   ColorRGB(255, 127, 80)},
        {"Salmon",  ColorRGB(250, 128, 114)},
        {"Gold",    ColorRGB(255, 215, 0)},
        {"Silver",  ColorRGB(192, 192, 192)},
        {"Maroon",  ColorRGB(128, 0, 0)},
        {"Olive",   ColorRGB(128, 128, 0)},
        {"Aqua",    ColorRGB(0, 255, 255)},
        {"Fuchsia", ColorRGB(255, 0, 255)},
        {"Indigo",  ColorRGB(75, 0, 130)},
        {"Violet",  ColorRGB(238, 130, 238)},
        {"Khaki",   ColorRGB(240, 230, 140)},
        {"Tan",     ColorRGB(210, 180, 140)},
        {"Crimson", ColorRGB(220, 20, 60)},
        {"SkyBlue", ColorRGB(135, 206, 235)}
    };
    return color_library;
}

} // anonymous namespace

// ==================== 单像素颜色空间转换实现 ====================

namespace color_utils {

void rgb_to_hsv(const ColorRGB& rgb, ColorHSV& hsv) {
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;
    
    float max_val = std::max({r, g, b});
    float min_val = std::min({r, g, b});
    float delta = max_val - min_val;
    
    // 计算明度V
    hsv.v = max_val;
    
    // 计算饱和度S
    if (max_val == 0.0f) {
        hsv.s = 0.0f;
        hsv.h = 0.0f;
        return;
    } else {
        hsv.s = delta / max_val;
    }
    
    // 计算色相H
    if (delta == 0.0f) {
        hsv.h = 0.0f;
    } else {
        if (max_val == r) {
            hsv.h = 60.0f * fmod(((g - b) / delta), 6.0f);
        } else if (max_val == g) {
            hsv.h = 60.0f * (((b - r) / delta) + 2.0f);
        } else { // max_val == b
            hsv.h = 60.0f * (((r - g) / delta) + 4.0f);
        }
        
        // 确保H在0-360范围内
        if (hsv.h < 0.0f) {
            hsv.h += 360.0f;
        }
    }
}

void hsv_to_rgb(const ColorHSV& hsv, ColorRGB& rgb) {
    float h = hsv.h;
    float s = hsv.s;
    float v = hsv.v;
    
    // 如果饱和度为0，则为灰度颜色
    if (s == 0.0f) {
        rgb.r = rgb.g = rgb.b = static_cast<uint8_t>(v * 255.0f);
        return;
    }
    
    // 将H从0-360转换为0-1
    h /= 60.0f;
    int sector = static_cast<int>(h);
    float fractional = h - sector;
    
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * fractional);
    float t = v * (1.0f - s * (1.0f - fractional));
    
    float r, g, b;
    
    switch (sector) {
        case 0:  r = v; g = t; b = p; break;
        case 1:  r = q; g = v; b = p; break;
        case 2:  r = p; g = v; b = t; break;
        case 3:  r = p; g = q; b = v; break;
        case 4:  r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    
    rgb.r = static_cast<uint8_t>(clamp(static_cast<int>(r * 255.0f), 0, 255));
    rgb.g = static_cast<uint8_t>(clamp(static_cast<int>(g * 255.0f), 0, 255));
    rgb.b = static_cast<uint8_t>(clamp(static_cast<int>(b * 255.0f), 0, 255));
}

void rgb_to_hsl(const ColorRGB& rgb, ColorHSL& hsl) {
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;
    
    float max_val = std::max({r, g, b});
    float min_val = std::min({r, g, b});
    float delta = max_val - min_val;
    
    // 计算亮度L
    hsl.l = (max_val + min_val) / 2.0f;
    
    // 计算饱和度S
    if (delta == 0.0f) {
        hsl.h = 0.0f;
        hsl.s = 0.0f;
        return;
    }
    
    hsl.s = hsl.l > 0.5f ? delta / (2.0f - max_val - min_val) : delta / (max_val + min_val);
    
    // 计算色相H
    if (max_val == r) {
        hsl.h = 60.0f * fmod(((g - b) / delta), 6.0f);
    } else if (max_val == g) {
        hsl.h = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        hsl.h = 60.0f * (((r - g) / delta) + 4.0f);
    }
    
    if (hsl.h < 0.0f) {
        hsl.h += 360.0f;
    }
}

void hsl_to_rgb(const ColorHSL& hsl, ColorRGB& rgb) {
    float h = hsl.h;
    float s = hsl.s;
    float l = hsl.l;
    
    if (s == 0.0f) {
        rgb.r = rgb.g = rgb.b = static_cast<uint8_t>(l * 255.0f);
        return;
    }
    
    float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
    float p = 2.0f * l - q;
    
    // 将H从0-360转换为0-1
    float h_normalized = h / 360.0f;
    
    float r = hue_to_rgb(p, q, h_normalized + 1.0f / 3.0f);
    float g = hue_to_rgb(p, q, h_normalized);
    float b = hue_to_rgb(p, q, h_normalized - 1.0f / 3.0f);
    
    rgb.r = static_cast<uint8_t>(clamp(static_cast<int>(r * 255.0f), 0, 255));
    rgb.g = static_cast<uint8_t>(clamp(static_cast<int>(g * 255.0f), 0, 255));
    rgb.b = static_cast<uint8_t>(clamp(static_cast<int>(b * 255.0f), 0, 255));
}

void rgb_to_lab(const ColorRGB& rgb, ColorLAB& lab) {
    // 先转换到XYZ
    float x, y, z;
    rgb_to_xyz(rgb, x, y, z);
    
    // 参考白点D65
    const float ref_x = 0.95047f;
    const float ref_y = 1.0f;
    const float ref_z = 1.08883f;
    
    // 归一化XYZ
    x /= ref_x;
    y /= ref_y;
    z /= ref_z;
    
    // 转换到LAB
    lab.l = 116.0f * pivot_lab(y) - 16.0f;
    lab.a = 500.0f * (pivot_lab(x) - pivot_lab(y));
    lab.b = 200.0f * (pivot_lab(y) - pivot_lab(z));
}

void lab_to_rgb(const ColorLAB& lab, ColorRGB& rgb) {
    // 参考白点D65
    const float ref_x = 0.95047f;
    const float ref_y = 1.0f;
    const float ref_z = 1.08883f;
    
    // LAB到XYZ
    float y = (lab.l + 16.0f) / 116.0f;
    float x = lab.a / 500.0f + y;
    float z = y - lab.b / 200.0f;
    
    x = ref_x * pivot_lab_inverse(x);
    y = ref_y * pivot_lab_inverse(y);
    z = ref_z * pivot_lab_inverse(z);
    
    // XYZ到RGB
    xyz_to_rgb(x, y, z, rgb);
}

void rgb_to_gray(const ColorRGB& rgb, uint8_t& gray) {
    // 使用标准灰度转换公式（与image_utils中一致）
    gray = static_cast<uint8_t>(0.299f * rgb.r + 0.587f * rgb.g + 0.114f * rgb.b);
}

void rgb_to_yuv(const ColorRGB& rgb, float& y, float& u, float& v) {
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;
    
    // BT.601标准（SDTV）
    y = 0.299f * r + 0.587f * g + 0.114f * b;
    u = -0.14713f * r - 0.28886f * g + 0.436f * b;
    v = 0.615f * r - 0.51499f * g - 0.10001f * b;
}

void yuv_to_rgb(float y, float u, float v, ColorRGB& rgb) {
    // BT.601标准逆变换
    float r = y + 1.13983f * v;
    float g = y - 0.39465f * u - 0.58060f * v;
    float b = y + 2.03211f * u;
    
    rgb.r = static_cast<uint8_t>(clamp(static_cast<int>(r * 255.0f), 0, 255));
    rgb.g = static_cast<uint8_t>(clamp(static_cast<int>(g * 255.0f), 0, 255));
    rgb.b = static_cast<uint8_t>(clamp(static_cast<int>(b * 255.0f), 0, 255));
}

// ==================== 图像颜色空间转换实现 ====================

void convert_rgb_to_hsv(const ImageData& rgb, ImageData& hsv) {
    if (rgb.empty() || rgb.channels < 3) {
        OVF_ERROR() << "convert_rgb_to_hsv: Invalid input image";
        return;
    }
    
    hsv.width = rgb.width;
    hsv.height = rgb.height;
    hsv.channels = 3;
    hsv.format = ImageFormat::RGB8;
    hsv.data.resize(rgb.width * rgb.height * 3);
    hsv.timestamp = rgb.timestamp;
    hsv.frame_id = rgb.frame_id;
    hsv.source_id = rgb.source_id;
    
    for (uint32_t i = 0; i < rgb.width * rgb.height; ++i) {
        ColorRGB color(rgb.data[i * 3], rgb.data[i * 3 + 1], rgb.data[i * 3 + 2]);
        ColorHSV hsv_color;
        rgb_to_hsv(color, hsv_color);
        
        // 将HSV存储为便于处理的范围
        // H: 0-180 (便于与OpenCV兼容)
        // S: 0-255
        // V: 0-255
        hsv.data[i * 3] = static_cast<uint8_t>(hsv_color.h / 2.0f);
        hsv.data[i * 3 + 1] = static_cast<uint8_t>(hsv_color.s * 255.0f);
        hsv.data[i * 3 + 2] = static_cast<uint8_t>(hsv_color.v * 255.0f);
    }
}

void convert_rgb_to_lab(const ImageData& rgb, ImageData& lab) {
    if (rgb.empty() || rgb.channels < 3) {
        OVF_ERROR() << "convert_rgb_to_lab: Invalid input image";
        return;
    }
    
    lab.width = rgb.width;
    lab.height = rgb.height;
    lab.channels = 3;
    lab.format = ImageFormat::RGB8;
    lab.data.resize(rgb.width * rgb.height * 3);
    lab.timestamp = rgb.timestamp;
    lab.frame_id = rgb.frame_id;
    lab.source_id = rgb.source_id;
    
    for (uint32_t i = 0; i < rgb.width * rgb.height; ++i) {
        ColorRGB color(rgb.data[i * 3], rgb.data[i * 3 + 1], rgb.data[i * 3 + 2]);
        ColorLAB lab_color;
        rgb_to_lab(color, lab_color);
        
        // 将LAB存储为0-255范围便于处理
        // L: 0-100 -> 0-255
        // a: -127~127 -> 0-255 (加128)
        // b: -127~127 -> 0-255 (加128)
        lab.data[i * 3] = static_cast<uint8_t>(lab_color.l * 255.0f / 100.0f);
        lab.data[i * 3 + 1] = static_cast<uint8_t>(lab_color.a + 128.0f);
        lab.data[i * 3 + 2] = static_cast<uint8_t>(lab_color.b + 128.0f);
    }
}

void convert_hsv_to_rgb(const ImageData& hsv, ImageData& rgb) {
    if (hsv.empty() || hsv.channels < 3) {
        OVF_ERROR() << "convert_hsv_to_rgb: Invalid input image";
        return;
    }
    
    rgb.width = hsv.width;
    rgb.height = hsv.height;
    rgb.channels = 3;
    rgb.format = ImageFormat::RGB8;
    rgb.data.resize(hsv.width * hsv.height * 3);
    rgb.timestamp = hsv.timestamp;
    rgb.frame_id = hsv.frame_id;
    rgb.source_id = hsv.source_id;
    
    for (uint32_t i = 0; i < hsv.width * hsv.height; ++i) {
        // 从存储格式恢复HSV值
        ColorHSV hsv_color;
        hsv_color.h = hsv.data[i * 3] * 2.0f;
        hsv_color.s = hsv.data[i * 3 + 1] / 255.0f;
        hsv_color.v = hsv.data[i * 3 + 2] / 255.0f;
        
        ColorRGB rgb_color;
        hsv_to_rgb(hsv_color, rgb_color);
        
        rgb.data[i * 3] = rgb_color.r;
        rgb.data[i * 3 + 1] = rgb_color.g;
        rgb.data[i * 3 + 2] = rgb_color.b;
    }
}

void convert_to_gray(const ImageData& src, ImageData& gray) {
    if (src.empty()) {
        OVF_ERROR() << "convert_to_gray: Invalid input image";
        return;
    }
    
    gray.width = src.width;
    gray.height = src.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(src.width * src.height);
    gray.timestamp = src.timestamp;
    gray.frame_id = src.frame_id;
    gray.source_id = src.source_id;
    
    if (src.channels == 1) {
        // 已经是灰度图，直接复制
        gray.data = src.data;
    } else if (src.channels == 3) {
        // RGB转灰度
        for (uint32_t i = 0; i < src.width * src.height; ++i) {
            ColorRGB color(src.data[i * 3], src.data[i * 3 + 1], src.data[i * 3 + 2]);
            uint8_t gray_val;
            rgb_to_gray(color, gray_val);
            gray.data[i] = gray_val;
        }
    } else if (src.channels == 4) {
        // RGBA转灰度（忽略Alpha）
        for (uint32_t i = 0; i < src.width * src.height; ++i) {
            ColorRGB color(src.data[i * 4], src.data[i * 4 + 1], src.data[i * 4 + 2]);
            uint8_t gray_val;
            rgb_to_gray(color, gray_val);
            gray.data[i] = gray_val;
        }
    }
}

void convert_color_space(const ImageData& src, ImageData& dst,
                         ColorSpace source_space, ColorSpace target_space) {
    if (src.empty()) {
        OVF_ERROR() << "convert_color_space: Invalid input image";
        return;
    }
    
    // 简化实现：只处理RGB源和常见目标
    if (source_space == ColorSpace::RGB) {
        switch (target_space) {
            case ColorSpace::HSV:
                convert_rgb_to_hsv(src, dst);
                break;
            case ColorSpace::LAB:
                convert_rgb_to_lab(src, dst);
                break;
            case ColorSpace::GRAY:
                convert_to_gray(src, dst);
                break;
            case ColorSpace::RGB:
                dst = src;
                break;
            default:
                OVF_ERROR() << "convert_color_space: Unsupported target color space";
                break;
        }
    } else if (source_space == ColorSpace::HSV && target_space == ColorSpace::RGB) {
        convert_hsv_to_rgb(src, dst);
    } else if (source_space == ColorSpace::GRAY && target_space == ColorSpace::RGB) {
        // 灰度转RGB
        dst.width = src.width;
        dst.height = src.height;
        dst.channels = 3;
        dst.format = ImageFormat::RGB8;
        dst.data.resize(src.width * src.height * 3);
        dst.timestamp = src.timestamp;
        dst.frame_id = src.frame_id;
        dst.source_id = src.source_id;
        
        for (uint32_t i = 0; i < src.width * src.height; ++i) {
            dst.data[i * 3] = src.data[i];
            dst.data[i * 3 + 1] = src.data[i];
            dst.data[i * 3 + 2] = src.data[i];
        }
    } else {
        OVF_ERROR() << "convert_color_space: Unsupported conversion combination";
    }
}

// ==================== 颜色距离计算实现 ====================

float color_distance_rgb(const ColorRGB& c1, const ColorRGB& c2) {
    float dr = static_cast<float>(c1.r) - static_cast<float>(c2.r);
    float dg = static_cast<float>(c1.g) - static_cast<float>(c2.g);
    float db = static_cast<float>(c1.b) - static_cast<float>(c2.b);
    return std::sqrt(dr * dr + dg * dg + db * db);
}

float color_distance_hsv(const ColorHSV& c1, const ColorHSV& c2) {
    // 色相距离需要考虑周期性
    float dh = std::abs(c1.h - c2.h);
    if (dh > 180.0f) {
        dh = 360.0f - dh;
    }
    dh /= 180.0f;  // 归一化到0-1
    
    float ds = std::abs(c1.s - c2.s);
    float dv = std::abs(c1.v - c2.v);
    
    // 加权距离（色相最重要）
    return std::sqrt(dh * dh * 4.0f + ds * ds + dv * dv);
}

float color_distance_lab(const ColorLAB& c1, const ColorLAB& c2) {
    // CIEDE2000简化版本
    float dl = c2.l - c1.l;
    float da = c2.a - c1.a;
    float db = c2.b - c1.b;
    
    // 简化的CIEDE2000
    float c1_ab = std::sqrt(c1.a * c1.a + c1.b * c1.b);
    float c2_ab = std::sqrt(c2.a * c2.a + c2.b * c2.b);
    float c_ab = (c1_ab + c2_ab) / 2.0f;
    
    // 色相差异
    float h1 = std::atan2(c1.b, c1.a);
    float h2 = std::atan2(c2.b, c2.a);
    float dh = std::abs(h1 - h2);
    if (dh > M_PI) {
        dh = 2.0f * M_PI - dh;
    }
    
    // 简化的加权计算
    float sl = 1.0f + 0.015f * std::abs((c1.l + c2.l) / 2.0f - 50.0f);
    float sc = 1.0f + 0.045f * c_ab;
    float sh = 1.0f + 0.015f * c_ab;
    
    return std::sqrt(dl * dl / (sl * sl) + 
                     da * da / (sc * sc) + 
                     db * db / (sc * sc) + 
                     dh * dh / (sh * sh));
}

float color_similarity(const ColorRGB& c1, const ColorRGB& c2, ColorSpace space) {
    float distance;
    
    switch (space) {
        case ColorSpace::RGB: {
            distance = color_distance_rgb(c1, c2);
            // 最大RGB距离约为441.67 (sqrt(255^2 * 3))
            return 1.0f - distance / 441.67f;
        }
        case ColorSpace::HSV: {
            ColorHSV hsv1, hsv2;
            rgb_to_hsv(c1, hsv1);
            rgb_to_hsv(c2, hsv2);
            distance = color_distance_hsv(hsv1, hsv2);
            // 最大HSV距离约为sqrt(4 + 1 + 1) = sqrt(6)
            return 1.0f - distance / std::sqrt(6.0f);
        }
        case ColorSpace::LAB: {
            ColorLAB lab1, lab2;
            rgb_to_lab(c1, lab1);
            rgb_to_lab(c2, lab2);
            distance = color_distance_lab(lab1, lab2);
            // LAB距离没有一个固定的最大值，使用经验值100
            return 1.0f - distance / 100.0f;
        }
        default:
            return 1.0f - color_distance_rgb(c1, c2) / 441.67f;
    }
}

// ==================== 颜色分割实现 ====================

bool is_color_in_range(const ColorRGB& color, const ColorRGB& target,
                       int tolerance, ColorSpace space) {
    if (space == ColorSpace::RGB) {
        int dr = std::abs(static_cast<int>(color.r) - static_cast<int>(target.r));
        int dg = std::abs(static_cast<int>(color.g) - static_cast<int>(target.g));
        int db = std::abs(static_cast<int>(color.b) - static_cast<int>(target.b));
        return dr <= tolerance && dg <= tolerance && db <= tolerance;
    } else if (space == ColorSpace::HSV) {
        ColorHSV hsv_color, hsv_target;
        rgb_to_hsv(color, hsv_color);
        rgb_to_hsv(target, hsv_target);
        
        // 色相容差需要考虑周期性
        float dh = std::abs(hsv_color.h - hsv_target.h);
        if (dh > 180.0f) dh = 360.0f - dh;
        
        float tolerance_h = static_cast<float>(tolerance) * 360.0f / 255.0f;
        float tolerance_s = static_cast<float>(tolerance) / 255.0f;
        float tolerance_v = static_cast<float>(tolerance) / 255.0f;
        
        return dh <= tolerance_h && 
               std::abs(hsv_color.s - hsv_target.s) <= tolerance_s &&
               std::abs(hsv_color.v - hsv_target.v) <= tolerance_v;
    } else if (space == ColorSpace::LAB) {
        ColorLAB lab_color, lab_target;
        rgb_to_lab(color, lab_color);
        rgb_to_lab(target, lab_target);
        
        float tolerance_l = static_cast<float>(tolerance) * 100.0f / 255.0f;
        float tolerance_a = static_cast<float>(tolerance) * 254.0f / 255.0f;
        float tolerance_b = static_cast<float>(tolerance) * 254.0f / 255.0f;
        
        return std::abs(lab_color.l - lab_target.l) <= tolerance_l &&
               std::abs(lab_color.a - lab_target.a) <= tolerance_a &&
               std::abs(lab_color.b - lab_target.b) <= tolerance_b;
    }
    
    // 默认使用RGB
    return is_color_in_range(color, target, tolerance, ColorSpace::RGB);
}

void color_segment(const ImageData& src, ImageData& mask,
                   const ColorRGB& target_color, int tolerance,
                   ColorSpace space) {
    if (src.empty()) {
        OVF_ERROR() << "color_segment: Invalid input image";
        return;
    }
    
    mask.width = src.width;
    mask.height = src.height;
    mask.channels = 1;
    mask.format = ImageFormat::Mono8;
    mask.data.resize(src.width * src.height);
    mask.timestamp = src.timestamp;
    mask.frame_id = src.frame_id;
    mask.source_id = src.source_id;
    
    for (uint32_t i = 0; i < src.width * src.height; ++i) {
        ColorRGB pixel_color;
        if (src.channels >= 3) {
            pixel_color.r = src.data[i * src.channels];
            pixel_color.g = src.data[i * src.channels + 1];
            pixel_color.b = src.data[i * src.channels + 2];
        } else {
            // 灰度图，复制到RGB
            pixel_color.r = pixel_color.g = pixel_color.b = src.data[i];
        }
        
        mask.data[i] = is_color_in_range(pixel_color, target_color, tolerance, space) ? 255 : 0;
    }
}

// ==================== 主颜色提取实现 ====================

void extract_dominant_colors(const ImageData& image, 
                              Vector<ColorRGB>& colors, 
                              int max_colors) {
    if (image.empty() || max_colors <= 0) {
        OVF_ERROR() << "extract_dominant_colors: Invalid parameters";
        return;
    }
    
    // 使用直方图峰值法提取主颜色
    // 先量化颜色（减少颜色数量以便统计）
    const int quantize_bits = 4;  // 每个通道量化到16级
    const int quantize_levels = 16;
    
    // 颜色计数
    std::map<int, int> color_counts;
    
    for (uint32_t i = 0; i < image.width * image.height; ++i) {
        uint8_t r = image.channels >= 3 ? image.data[i * image.channels] : image.data[i];
        uint8_t g = image.channels >= 3 ? image.data[i * image.channels + 1] : image.data[i];
        uint8_t b = image.channels >= 3 ? image.data[i * image.channels + 2] : image.data[i];
        
        // 量化
        r = (r >> (8 - quantize_bits)) << (8 - quantize_bits);
        g = (g >> (8 - quantize_bits)) << (8 - quantize_bits);
        b = (b >> (8 - quantize_bits)) << (8 - quantize_bits);
        
        // 生成颜色索引
        int index = (r << 16) | (g << 8) | b;
        color_counts[index]++;
    }
    
    // 按计数排序，提取前max_colors个颜色
    Vector<std::pair<int, int>> sorted_colors(color_counts.begin(), color_counts.end());
    std::sort(sorted_colors.begin(), sorted_colors.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    colors.clear();
    int count = std::min(max_colors, static_cast<int>(sorted_colors.size()));
    for (int i = 0; i < count; ++i) {
        int index = sorted_colors[i].first;
        colors.emplace_back(
            static_cast<uint8_t>((index >> 16) & 0xFF),
            static_cast<uint8_t>((index >> 8) & 0xFF),
            static_cast<uint8_t>(index & 0xFF)
        );
    }
}

void calc_color_histogram(const ImageData& image, 
                          ColorHistogram& histogram,
                          int bins,
                          const Region* roi) {
    if (image.empty() || bins <= 0) {
        OVF_ERROR() << "calc_color_histogram: Invalid parameters";
        return;
    }
    
    histogram.bins = bins;
    histogram.hist_r.resize(bins, 0);
    histogram.hist_g.resize(bins, 0);
    histogram.hist_b.resize(bins, 0);
    
    // 确定计算区域
    uint32_t start_x = 0, start_y = 0;
    uint32_t end_x = image.width, end_y = image.height;
    
    if (roi) {
        start_x = clamp(roi->x, 0, static_cast<int>(image.width));
        start_y = clamp(roi->y, 0, static_cast<int>(image.height));
        end_x = clamp(roi->x + roi->width, 0, static_cast<int>(image.width));
        end_y = clamp(roi->y + roi->height, 0, static_cast<int>(image.height));
    }
    
    float scale = static_cast<float>(bins) / 256.0f;
    
    for (uint32_t y = start_y; y < end_y; ++y) {
        for (uint32_t x = start_x; x < end_x; ++x) {
            uint32_t idx = y * image.width + x;
            
            uint8_t r = image.channels >= 3 ? image.data[idx * image.channels] : image.data[idx];
            uint8_t g = image.channels >= 3 ? image.data[idx * image.channels + 1] : image.data[idx];
            uint8_t b = image.channels >= 3 ? image.data[idx * image.channels + 2] : image.data[idx];
            
            int bin_r = static_cast<int>(r * scale);
            int bin_g = static_cast<int>(g * scale);
            int bin_b = static_cast<int>(b * scale);
            
            if (bin_r >= bins) bin_r = bins - 1;
            if (bin_g >= bins) bin_g = bins - 1;
            if (bin_b >= bins) bin_b = bins - 1;
            
            histogram.hist_r[bin_r]++;
            histogram.hist_g[bin_g]++;
            histogram.hist_b[bin_b]++;
        }
    }
}

String get_color_name(const ColorRGB& color) {
    const auto& color_library = get_standard_color_library();
    
    // 在颜色库中找最匹配的颜色
    float min_distance = std::numeric_limits<float>::max();
    String best_name = "Unknown";
    
    for (const auto& pair : color_library) {
        float distance = color_distance_rgb(color, pair.second);
        if (distance < min_distance) {
            min_distance = distance;
            best_name = pair.first;
        }
    }
    
    // 如果距离太大，返回描述性名称
    if (min_distance > 50.0f) {
        ColorHSV hsv;
        rgb_to_hsv(color, hsv);
        
        // 根据HSV值生成描述性名称
        if (hsv.s < 0.1f) {
            // 低饱和度：灰度系列
            if (hsv.v < 0.2f) return "Dark Gray";
            if (hsv.v < 0.4f) return "Gray";
            if (hsv.v < 0.6f) return "Light Gray";
            if (hsv.v < 0.8f) return "Pale Gray";
            return "White";
        }
        
        // 根据色相确定颜色类型
        float h = hsv.h;
        String hue_name;
        if (h < 15.0f || h >= 345.0f) hue_name = "Red";
        else if (h < 45.0f) hue_name = "Orange";
        else if (h < 75.0f) hue_name = "Yellow";
        else if (h < 105.0f) hue_name = "Lime";
        else if (h < 135.0f) hue_name = "Green";
        else if (h < 165.0f) hue_name = "Teal";
        else if (h < 195.0f) hue_name = "Cyan";
        else if (h < 225.0f) hue_name = "Blue";
        else if (h < 255.0f) hue_name = "Indigo";
        else if (h < 285.0f) hue_name = "Purple";
        else if (h < 315.0f) hue_name = "Magenta";
        else hue_name = "Pink";
        
        // 根据明度添加前缀
        String brightness_prefix;
        if (hsv.v < 0.3f) brightness_prefix = "Dark ";
        else if (hsv.v < 0.5f) brightness_prefix = "";
        else if (hsv.v < 0.7f) brightness_prefix = "Light ";
        else brightness_prefix = "Pale ";
        
        return brightness_prefix + hue_name;
    }
    
    return best_name;
}

ColorMatchResult match_color(const ColorRGB& color,
                              const Vector<ColorRGB>& color_library,
                              const Vector<String>& color_names,
                              ColorSpace space) {
    ColorMatchResult result;
    
    if (color_library.empty()) {
        result.similarity = 0.0f;
        result.color_name = "Unknown";
        return result;
    }
    
    float max_similarity = 0.0f;
    int best_index = 0;
    
    for (size_t i = 0; i < color_library.size(); ++i) {
        float sim = color_similarity(color, color_library[i], space);
        if (sim > max_similarity) {
            max_similarity = sim;
            best_index = static_cast<int>(i);
        }
    }
    
    result.matched_color = color_library[best_index];
    result.similarity = max_similarity;
    result.match_index = best_index;
    
    if (color_names.size() > best_index) {
        result.color_name = color_names[best_index];
    } else {
        result.color_name = "Color_" + std::to_string(best_index);
    }
    
    return result;
}

// ==================== 颜色校正实现 ====================

void white_balance(const ImageData& image, ImageData& corrected,
                   const ColorRGB* white_point) {
    if (image.empty() || image.channels < 3) {
        OVF_ERROR() << "white_balance: Invalid input image";
        return;
    }
    
    corrected = image;
    
    ColorRGB wp;
    if (white_point) {
        wp = *white_point;
    } else {
        // 自动检测白点：使用灰度世界假设
        // 计算各通道均值
        double sum_r = 0, sum_g = 0, sum_b = 0;
        size_t count = image.width * image.height;
        
        for (size_t i = 0; i < count; ++i) {
            sum_r += image.data[i * image.channels];
            sum_g += image.data[i * image.channels + 1];
            sum_b += image.data[i * image.channels + 2];
        }
        
        double avg_r = sum_r / count;
        double avg_g = sum_g / count;
        double avg_b = sum_b / count;
        
        // 将均值作为白点参考
        wp.r = static_cast<uint8_t>(avg_r);
        wp.g = static_cast<uint8_t>(avg_g);
        wp.b = static_cast<uint8_t>(avg_b);
    }
    
    // 避免除零
    if (wp.r == 0) wp.r = 1;
    if (wp.g == 0) wp.g = 1;
    if (wp.b == 0) wp.b = 1;
    
    // 计算校正系数
    float scale_r = 255.0f / wp.r;
    float scale_g = 255.0f / wp.g;
    float scale_b = 255.0f / wp.b;
    
    // 应用校正
    for (size_t i = 0; i < corrected.width * corrected.height; ++i) {
        corrected.data[i * corrected.channels] = 
            static_cast<uint8_t>(clamp(static_cast<int>(corrected.data[i * corrected.channels] * scale_r), 0, 255));
        corrected.data[i * corrected.channels + 1] = 
            static_cast<uint8_t>(clamp(static_cast<int>(corrected.data[i * corrected.channels + 1] * scale_g), 0, 255));
        corrected.data[i * corrected.channels + 2] = 
            static_cast<uint8_t>(clamp(static_cast<int>(corrected.data[i * corrected.channels + 2] * scale_b), 0, 255));
    }
}

void gamma_correction(const ImageData& image, ImageData& corrected, float gamma) {
    if (image.empty() || gamma <= 0.0f) {
        OVF_ERROR() << "gamma_correction: Invalid parameters";
        return;
    }
    
    corrected = image;
    
    // 预计算Gamma校正表（提高效率）
    uint8_t gamma_table[256];
    for (int i = 0; i < 256; ++i) {
        float normalized = i / 255.0f;
        float corrected_val = std::pow(normalized, 1.0f / gamma) * 255.0f;
        gamma_table[i] = static_cast<uint8_t>(clamp(static_cast<int>(corrected_val), 0, 255));
    }
    
    // 应用Gamma校正
    for (size_t i = 0; i < corrected.data.size(); ++i) {
        corrected.data[i] = gamma_table[corrected.data[i]];
    }
}

void auto_levels(const ImageData& image, ImageData& corrected,
                 float low_clip, float high_clip) {
    if (image.empty() || image.channels < 3) {
        OVF_ERROR() << "auto_levels: Invalid input image";
        return;
    }
    
    corrected = image;
    size_t count = image.width * image.height;
    
    // 对每个通道独立处理
    for (int c = 0; c < 3; ++c) {
        // 计算直方图
        Vector<int> hist(256, 0);
        for (size_t i = 0; i < count; ++i) {
            hist[image.data[i * image.channels + c]]++;
        }
        
        // 找到低端和高端裁剪点
        int low_threshold = static_cast<int>(count * low_clip);
        int high_threshold = static_cast<int>(count * high_clip);
        
        int low_bound = 0, high_bound = 255;
        int accumulated = 0;
        
        // 找低端边界
        for (int i = 0; i < 256; ++i) {
            accumulated += hist[i];
            if (accumulated >= low_threshold) {
                low_bound = i;
                break;
            }
        }
        
        // 找高端边界
        accumulated = 0;
        for (int i = 255; i >= 0; --i) {
            accumulated += hist[i];
            if (accumulated >= high_threshold) {
                high_bound = i;
                break;
            }
        }
        
        // 防止边界相同
        if (high_bound <= low_bound) {
            high_bound = low_bound + 1;
        }
        
        // 创建映射表
        Vector<uint8_t> map_table(256);
        for (int i = 0; i < 256; ++i) {
            if (i <= low_bound) {
                map_table[i] = 0;
            } else if (i >= high_bound) {
                map_table[i] = 255;
            } else {
                map_table[i] = static_cast<uint8_t>(
                    (i - low_bound) * 255 / (high_bound - low_bound));
            }
        }
        
        // 应用映射
        for (size_t i = 0; i < count; ++i) {
            corrected.data[i * corrected.channels + c] = map_table[corrected.data[i * corrected.channels + c]];
        }
    }
}

} // namespace color_utils

// ==================== 节点实现 ====================

// ColorConvertNode实现
ColorConvertNode::ColorConvertNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

Result<void> ColorConvertNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取输入图像
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    String source_space_str = params_.get_string("source_space", "RGB");
    String target_space_str = params_.get_string("target_space", "GRAY");
    
    ColorSpace source_space = parse_color_space(source_space_str);
    ColorSpace target_space = parse_color_space(target_space_str);
    
    // 执行转换
    ImageData output;
    color_utils::convert_color_space(input, output, source_space, target_space);
    
    if (output.empty()) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Color conversion failed");
    }
    
    // 设置输出
    set_output("converted_image", Data(output));
    
    return Result<void>::success();
}

NodeInfo ColorConvertNode::make_info() {
    NodeInfo info;
    info.id = "color_convert";
    info.name = "颜色空间转换";
    info.category = "颜色处理";
    info.description = "将图像从一个颜色空间转换到另一个颜色空间";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true)
    };
    
    info.outputs = {
        DataPort("converted_image", "转换后的图像", DataType::Image)
    };
    
    info.params = {
        ParamDef("source_space", "源颜色空间", DataType::String, Data("RGB")),
        ParamDef("target_space", "目标颜色空间", DataType::String, Data("GRAY"))
    };
    
    info.params[0].options = {"RGB", "HSV", "HSL", "LAB", "YUV", "GRAY"};
    info.params[1].options = {"RGB", "HSV", "HSL", "LAB", "YUV", "GRAY"};
    
    return info;
}

ColorSpace ColorConvertNode::parse_color_space(const String& name) {
    if (name == "RGB") return ColorSpace::RGB;
    if (name == "HSV") return ColorSpace::HSV;
    if (name == "HSL") return ColorSpace::HSL;
    if (name == "LAB") return ColorSpace::LAB;
    if (name == "YUV") return ColorSpace::YUV;
    if (name == "GRAY") return ColorSpace::GRAY;
    return ColorSpace::RGB;  // 默认
}

// ColorSegmentNode实现
ColorSegmentNode::ColorSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

Result<void> ColorSegmentNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取输入图像
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    String target_color_str = params_.get_string("target_color", "255,0,0");
    int tolerance = params_.get_number("tolerance", 30);
    String color_space_str = params_.get_string("color_space", "RGB");
    
    // 解析颜色
    ColorRGB target_color;
    if (!parse_color(target_color_str, target_color)) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Invalid color format");
    }
    
    // 解析颜色空间
    ColorSpace space = ColorSpace::RGB;
    if (color_space_str == "HSV") space = ColorSpace::HSV;
    else if (color_space_str == "LAB") space = ColorSpace::LAB;
    
    // 执行颜色分割
    ImageData mask;
    color_utils::color_segment(input, mask, target_color, tolerance, space);
    
    // 创建分割后的图像（将掩码应用到原图）
    ImageData segmented;
    segmented.width = input.width;
    segmented.height = input.height;
    segmented.channels = input.channels;
    segmented.format = input.format;
    segmented.data.resize(input.data.size());
    segmented.timestamp = input.timestamp;
    segmented.frame_id = input.frame_id;
    segmented.source_id = input.source_id;
    
    for (size_t i = 0; i < input.width * input.height; ++i) {
        if (mask.data[i] > 0) {
            for (uint32_t c = 0; c < input.channels; ++c) {
                segmented.data[i * input.channels + c] = input.data[i * input.channels + c];
            }
        } else {
            for (uint32_t c = 0; c < input.channels; ++c) {
                segmented.data[i * input.channels + c] = 0;
            }
        }
    }
    
    // 设置输出
    set_output("mask", Data(mask));
    set_output("segmented_image", Data(segmented));
    
    return Result<void>::success();
}

NodeInfo ColorSegmentNode::make_info() {
    NodeInfo info;
    info.id = "color_segment";
    info.name = "颜色分割";
    info.category = "颜色处理";
    info.description = "根据指定颜色分割图像";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true)
    };
    
    info.outputs = {
        DataPort("mask", "分割掩码", DataType::Image),
        DataPort("segmented_image", "分割后的图像", DataType::Image)
    };
    
    info.params = {
        ParamDef("target_color", "目标颜色", DataType::String, Data("255,0,0")),
        ParamDef("tolerance", "容差", DataType::Number, Data(30.0)),
        ParamDef("color_space", "颜色空间", DataType::String, Data("RGB"))
    };
    
    info.params[2].options = {"RGB", "HSV", "LAB"};
    
    return info;
}

bool ColorSegmentNode::parse_color(const String& color_str, ColorRGB& color) {
    // 支持两种格式: "r,g,b" 或 "#RRGGBB"
    
    if (color_str.empty()) return false;
    
    if (color_str[0] == '#') {
        // 十六进制格式
        if (color_str.length() != 7) return false;
        
        auto hex_to_int = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        
        int r = hex_to_int(color_str[1]) * 16 + hex_to_int(color_str[2]);
        int g = hex_to_int(color_str[3]) * 16 + hex_to_int(color_str[4]);
        int b = hex_to_int(color_str[5]) * 16 + hex_to_int(color_str[6]);
        
        if (r < 0 || g < 0 || b < 0) return false;
        
        color.r = static_cast<uint8_t>(r);
        color.g = static_cast<uint8_t>(g);
        color.b = static_cast<uint8_t>(b);
        return true;
    } else {
        // RGB逗号分隔格式
        std::istringstream iss(color_str);
        int r, g, b;
        char comma;
        
        if (iss >> r >> comma >> g >> comma >> b) {
            if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
                return false;
            }
            color.r = static_cast<uint8_t>(r);
            color.g = static_cast<uint8_t>(g);
            color.b = static_cast<uint8_t>(b);
            return true;
        }
        return false;
    }
}

// ColorRecognizeNode实现
ColorRecognizeNode::ColorRecognizeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

Result<void> ColorRecognizeNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取输入图像
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取ROI（可选）
    Region roi;
    const Region* roi_ptr = nullptr;
    if (has_input("roi")) {
        roi = get_input("roi").as_region();
        roi_ptr = &roi;
    }
    
    // 获取参数
    int max_colors = params_.get_number("max_colors", 5);
    
    // 提取主颜色
    Vector<ColorRGB> dominant_colors;
    color_utils::extract_dominant_colors(input, dominant_colors, max_colors);
    
    if (dominant_colors.empty()) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Failed to extract dominant colors");
    }
    
    // 计算颜色直方图
    ovf::algorithm::ColorHistogram histogram;
    color_utils::calc_color_histogram(input, histogram, 256, roi_ptr);
    
    // 获取主颜色名称
    String color_name = color_utils::get_color_name(dominant_colors[0]);
    
    // 构建主颜色列表的JSON格式字符串
    std::ostringstream colors_json;
    colors_json << "[";
    for (size_t i = 0; i < dominant_colors.size(); ++i) {
        if (i > 0) colors_json << ",";
        colors_json << "{\"r\":" << dominant_colors[i].r 
                   << ",\"g\":" << dominant_colors[i].g 
                   << ",\"b\":" << dominant_colors[i].b 
                   << ",\"name\":\"" << color_utils::get_color_name(dominant_colors[i]) << "\"}";
    }
    colors_json << "]";
    
    // 设置输出
    set_output("dominant_color", Data(static_cast<double>(dominant_colors[0].r * 65536 + dominant_colors[0].g * 256 + dominant_colors[0].b)));
    set_output("color_name", Data(color_name));
    set_output("dominant_colors", Data(colors_json.str()));
    
    // 输出直方图数据（简化为字符串格式）
    std::ostringstream hist_json;
    hist_json << "{\"bins\":" << histogram.bins << ",";
    hist_json << "\"r\":[";
    for (size_t i = 0; i < histogram.hist_r.size(); ++i) {
        if (i > 0) hist_json << ",";
        hist_json << histogram.hist_r[i];
    }
    hist_json << "],\"g\":[";
    for (size_t i = 0; i < histogram.hist_g.size(); ++i) {
        if (i > 0) hist_json << ",";
        hist_json << histogram.hist_g[i];
    }
    hist_json << "],\"b\":[";
    for (size_t i = 0; i < histogram.hist_b.size(); ++i) {
        if (i > 0) hist_json << ",";
        hist_json << histogram.hist_b[i];
    }
    hist_json << "]}";
    
    set_output("color_histogram", Data(hist_json.str()));
    
    return Result<void>::success();
}

NodeInfo ColorRecognizeNode::make_info() {
    NodeInfo info;
    info.id = "color_recognize";
    info.name = "颜色识别";
    info.category = "颜色处理";
    info.description = "识别图像中的主颜色";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("roi", "感兴趣区域", DataType::Region, false)
    };
    
    info.outputs = {
        DataPort("dominant_color", "主颜色", DataType::Number),
        DataPort("color_histogram", "颜色直方图", DataType::String),
        DataPort("color_name", "颜色名称", DataType::String),
        DataPort("dominant_colors", "主颜色列表", DataType::String)
    };
    
    info.params = {
        ParamDef("max_colors", "最大颜色数量", DataType::Number, Data(5.0)),
        ParamDef("method", "识别方法", DataType::String, Data("histogram"))
    };
    
    info.params[1].options = {"histogram", "kmeans"};
    
    return info;
}

// ColorMatchNode实现
ColorMatchNode::ColorMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

Result<void> ColorMatchNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取输入图像
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参考颜色（可以是输入或参数）
    ColorRGB reference_color;
    if (has_input("reference_color")) {
        int color_val = get_input("reference_color").as_int();
        reference_color.r = static_cast<uint8_t>((color_val >> 16) & 0xFF);
        reference_color.g = static_cast<uint8_t>((color_val >> 8) & 0xFF);
        reference_color.b = static_cast<uint8_t>(color_val & 0xFF);
    } else {
        String color_str = params_.get_string("reference_color", "255,0,0");
        ColorSegmentNode::parse_color(color_str, reference_color);  // 使用ColorSegmentNode的颜色解析
    }
    
    // 获取参数
    int tolerance = params_.get_number("tolerance", 30);
    int min_area = params_.get_number("min_area", 100);
    String color_space_str = params_.get_string("color_space", "RGB");
    
    // 解析颜色空间
    ColorSpace space = ColorSpace::RGB;
    if (color_space_str == "HSV") space = ColorSpace::HSV;
    else if (color_space_str == "LAB") space = ColorSpace::LAB;
    
    // 执行颜色分割
    ImageData mask;
    color_utils::color_segment(input, mask, reference_color, tolerance, space);
    
    // 计算匹配区域和相似度
    int match_count = 0;
    int total_count = input.width * input.height;
    
    int min_x = input.width, min_y = input.height;
    int max_x = 0, max_y = 0;
    
    for (uint32_t y = 0; y < input.height; ++y) {
        for (uint32_t x = 0; x < input.width; ++x) {
            if (mask.data[y * input.width + x] > 0) {
                match_count++;
                min_x = std::min(min_x, static_cast<int>(x));
                max_x = std::max(max_x, static_cast<int>(x));
                min_y = std::min(min_y, static_cast<int>(y));
                max_y = std::max(max_y, static_cast<int>(y));
            }
        }
    }
    
    float similarity = static_cast<float>(match_count) / total_count;
    
    // 构建匹配结果
    std::ostringstream match_json;
    match_json << "{\"count\":" << match_count 
               << ",\"total\":" << total_count
               << ",\"similarity\":" << similarity
               << ",\"min_area\":" << min_area << "}";
    
    // 设置输出
    set_output("match_result", Data(match_json.str()));
    set_output("similarity", Data(similarity));
    
    // 如果匹配面积足够大，输出匹配区域
    if (match_count >= min_area && min_x <= max_x && min_y <= max_y) {
        Region match_region(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
        set_output("match_region", Data(match_region));
    }
    
    return Result<void>::success();
}

NodeInfo ColorMatchNode::make_info() {
    NodeInfo info;
    info.id = "color_match";
    info.name = "颜色匹配";
    info.category = "颜色处理";
    info.description = "匹配图像中指定颜色的区域";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("reference_color", "参考颜色", DataType::Number, false)
    };
    
    info.outputs = {
        DataPort("match_result", "匹配结果", DataType::String),
        DataPort("match_region", "匹配区域", DataType::Region),
        DataPort("similarity", "相似度", DataType::Number)
    };
    
    info.params = {
        ParamDef("tolerance", "容差", DataType::Number, Data(30.0)),
        ParamDef("min_area", "最小匹配面积", DataType::Number, Data(100.0)),
        ParamDef("color_space", "颜色空间", DataType::String, Data("RGB")),
        ParamDef("reference_color", "参考颜色（参数）", DataType::String, Data("255,0,0"))
    };
    
    info.params[2].options = {"RGB", "HSV", "LAB"};
    
    return info;
}

// ColorCorrectNode实现
ColorCorrectNode::ColorCorrectNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

Result<void> ColorCorrectNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取输入图像
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty() || input.channels < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty or not RGB");
    }
    
    // 获取参数
    String method = params_.get_string("method", "white_balance");
    float gamma = params_.get_number("gamma", 1.0);
    float low_clip = params_.get_number("low_clip", 0.01);
    float high_clip = params_.get_number("high_clip", 0.01);
    
    // 执行校正
    ImageData corrected;
    
    if (method == "white_balance") {
        // 可选的白点参数
        ColorRGB white_point;
        const ColorRGB* wp_ptr = nullptr;
        
        String wp_str = params_.get_string("white_point", "");
        if (!wp_str.empty()) {
            if (ColorSegmentNode::parse_color(wp_str, white_point)) {
                wp_ptr = &white_point;
            }
        }
        
        color_utils::white_balance(input, corrected, wp_ptr);
    } else if (method == "gamma") {
        color_utils::gamma_correction(input, corrected, gamma);
    } else if (method == "auto_levels") {
        color_utils::auto_levels(input, corrected, low_clip, high_clip);
    } else {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Unknown correction method");
    }
    
    if (corrected.empty()) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Color correction failed");
    }
    
    // 设置输出
    set_output("corrected_image", Data(corrected));
    
    return Result<void>::success();
}

NodeInfo ColorCorrectNode::make_info() {
    NodeInfo info;
    info.id = "color_correct";
    info.name = "颜色校正";
    info.category = "颜色处理";
    info.description = "对图像进行颜色校正";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true)
    };
    
    info.outputs = {
        DataPort("corrected_image", "校正后的图像", DataType::Image)
    };
    
    info.params = {
        ParamDef("method", "校正方法", DataType::String, Data("white_balance")),
        ParamDef("white_point", "白点颜色", DataType::String, Data("")),
        ParamDef("gamma", "Gamma值", DataType::Number, Data(1.0)),
        ParamDef("low_clip", "低端裁剪比例", DataType::Number, Data(0.01)),
        ParamDef("high_clip", "高端裁剪比例", DataType::Number, Data(0.01))
    };
    
    info.params[0].options = {"white_balance", "gamma", "auto_levels"};
    
    return info;
}

// ColorHistogramNode实现
ColorHistogramNode::ColorHistogramNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

Result<void> ColorHistogramNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取输入图像
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取ROI（可选）
    Region roi;
    const Region* roi_ptr = nullptr;
    if (has_input("roi")) {
        roi = get_input("roi").as_region();
        roi_ptr = &roi;
    }
    
    // 获取参数
    int bins = params_.get_number("bins", 256);
    bool compute_stats = params_.get_bool("compute_stats", true);
    
    // 计算颜色直方图
    ovf::algorithm::ColorHistogram histogram;
    color_utils::calc_color_histogram(input, histogram, bins, roi_ptr);
    
    // 构建直方图输出（JSON格式）
    auto build_hist_json = [](const Vector<int>& hist) -> String {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < hist.size(); ++i) {
            if (i > 0) oss << ",";
            oss << hist[i];
        }
        oss << "]";
        return oss.str();
    };
    
    set_output("histogram_r", Data(build_hist_json(histogram.hist_r)));
    set_output("histogram_g", Data(build_hist_json(histogram.hist_g)));
    set_output("histogram_b", Data(build_hist_json(histogram.hist_b)));
    
    if (compute_stats) {
        // 计算统计信息
        double mean_r = 0, mean_g = 0, mean_b = 0;
        double std_r = 0, std_g = 0, std_b = 0;
        
        size_t count = input.width * input.height;
        if (roi_ptr) {
            count = static_cast<size_t>(roi_ptr->width * roi_ptr->height);
        }
        
        if (count > 0 && input.channels >= 3) {
            // 计算均值
            size_t actual_count = 0;
            uint32_t start_y = roi_ptr ? roi_ptr->y : 0;
            uint32_t end_y = roi_ptr ? roi_ptr->y + roi_ptr->height : input.height;
            uint32_t start_x = roi_ptr ? roi_ptr->x : 0;
            uint32_t end_x = roi_ptr ? roi_ptr->x + roi_ptr->width : input.width;
            
            for (uint32_t y = start_y; y < end_y && y < input.height; ++y) {
                for (uint32_t x = start_x; x < end_x && x < input.width; ++x) {
                    size_t idx = y * input.width + x;
                    mean_r += input.data[idx * input.channels];
                    mean_g += input.data[idx * input.channels + 1];
                    mean_b += input.data[idx * input.channels + 2];
                    actual_count++;
                }
            }
            
            if (actual_count > 0) {
                mean_r /= actual_count;
                mean_g /= actual_count;
                mean_b /= actual_count;
                
                // 计算标准差
                for (uint32_t y = start_y; y < end_y && y < input.height; ++y) {
                    for (uint32_t x = start_x; x < end_x && x < input.width; ++x) {
                        size_t idx = y * input.width + x;
                        std_r += std::pow(input.data[idx * input.channels] - mean_r, 2);
                        std_g += std::pow(input.data[idx * input.channels + 1] - mean_g, 2);
                        std_b += std::pow(input.data[idx * input.channels + 2] - mean_b, 2);
                    }
                }
                
                std_r = std::sqrt(std_r / actual_count);
                std_g = std::sqrt(std_g / actual_count);
                std_b = std::sqrt(std_b / actual_count);
            }
            
            // 构建平均颜色输出
            ColorRGB mean_color(
                static_cast<uint8_t>(mean_r),
                static_cast<uint8_t>(mean_g),
                static_cast<uint8_t>(mean_b)
            );
            
            std::ostringstream mean_json;
            mean_json << "{\"r\":" << mean_color.r 
                     << ",\"g\":" << mean_color.g 
                     << ",\"b\":" << mean_color.b 
                     << ",\"name\":\"" << color_utils::get_color_name(mean_color) << "\"}";
            
            set_output("mean_color", Data(mean_json.str()));
            
            std::ostringstream std_json;
            std_json << "{\"r\":" << std_r 
                     << ",\"g\":" << std_g 
                     << ",\"b\":" << std_b << "}";
            
            set_output("std_dev", Data(std_json.str()));
        }
    }
    
    return Result<void>::success();
}

NodeInfo ColorHistogramNode::make_info() {
    NodeInfo info;
    info.id = "color_histogram";
    info.name = "颜色直方图";
    info.category = "颜色处理";
    info.description = "计算图像的颜色直方图";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("roi", "感兴趣区域", DataType::Region, false)
    };
    
    info.outputs = {
        DataPort("histogram_r", "红色通道直方图", DataType::String),
        DataPort("histogram_g", "绿色通道直方图", DataType::String),
        DataPort("histogram_b", "蓝色通道直方图", DataType::String),
        DataPort("mean_color", "平均颜色", DataType::String),
        DataPort("std_dev", "颜色标准差", DataType::String)
    };
    
    info.params = {
        ParamDef("bins", "直方图箱子数量", DataType::Number, Data(256.0)),
        ParamDef("compute_stats", "计算统计信息", DataType::Boolean, Data(true))
    };
    
    return info;
}

// 注册节点
OVF_REGISTER_NODE(ColorConvertNode, "color_convert", ColorConvertNode::make_info())
OVF_REGISTER_NODE(ColorSegmentNode, "color_segment", ColorSegmentNode::make_info())
OVF_REGISTER_NODE(ColorRecognizeNode, "color_recognize", ColorRecognizeNode::make_info())
OVF_REGISTER_NODE(ColorMatchNode, "color_match", ColorMatchNode::make_info())
OVF_REGISTER_NODE(ColorCorrectNode, "color_correct", ColorCorrectNode::make_info())
OVF_REGISTER_NODE(ColorHistogramNode, "color_histogram", ColorHistogramNode::make_info())

} // namespace algorithm
} // namespace ovf