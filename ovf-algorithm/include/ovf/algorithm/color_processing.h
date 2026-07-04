/**
 * @file color_processing.h
 * @brief 颜色处理模块（纯C++实现，不依赖OpenCV）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 颜色空间类型
 */
enum class ColorSpace {
    RGB,
    HSV,
    HSL,
    LAB,
    YUV,
    GRAY
};

/**
 * @brief RGB颜色结构
 */
struct ColorRGB {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    
    ColorRGB() = default;
    ColorRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
    
    bool operator==(const ColorRGB& other) const {
        return r == other.r && g == other.g && b == other.b;
    }
    
    bool operator!=(const ColorRGB& other) const {
        return !(*this == other);
    }
};

/**
 * @brief HSV颜色结构
 * @note h: 0-360, s: 0-1, v: 0-1
 */
struct ColorHSV {
    float h = 0.0f;  // 色相: 0-360
    float s = 0.0f;  // 饱和度: 0-1
    float v = 0.0f;  // 明度: 0-1
    
    ColorHSV() = default;
    ColorHSV(float h_, float s_, float v_) : h(h_), s(s_), v(v_) {}
};

/**
 * @brief HSL颜色结构
 * @note h: 0-360, s: 0-1, l: 0-1
 */
struct ColorHSL {
    float h = 0.0f;  // 色相: 0-360
    float s = 0.0f;  // 饱和度: 0-1
    float l = 0.0f;  // 亮度: 0-1
    
    ColorHSL() = default;
    ColorHSL(float h_, float s_, float l_) : h(h_), s(s_), l(l_) {}
};

/**
 * @brief LAB颜色结构
 * @note l: 0-100, a: -127~127, b: -127~127
 */
struct ColorLAB {
    float l = 0.0f;   // 亮度: 0-100
    float a = 0.0f;   // a通道: -127~127
    float b = 0.0f;   // b通道: -127~127
    
    ColorLAB() = default;
    ColorLAB(float l_, float a_, float b_) : l(l_), a(a_), b(b_) {}
};

/**
 * @brief 颜色匹配结果
 */
struct ColorMatchResult {
    ColorRGB matched_color;
    float similarity = 0.0f;
    String color_name;
    int match_index = -1;
};

/**
 * @brief 颜色直方图数据
 */
struct ColorHistogram {
    Vector<int> hist_r;  // 红色通道直方图
    Vector<int> hist_g;  // 绿色通道直方图
    Vector<int> hist_b;  // 蓝色通道直方图
    int bins = 256;      // 直方图箱子数量
};

/**
 * @brief 颜色处理工具函数
 */
namespace color_utils {

// ==================== 单像素颜色空间转换 ====================

/**
 * @brief RGB转HSV
 */
void rgb_to_hsv(const ColorRGB& rgb, ColorHSV& hsv);

/**
 * @brief HSV转RGB
 */
void hsv_to_rgb(const ColorHSV& hsv, ColorRGB& rgb);

/**
 * @brief RGB转HSL
 */
void rgb_to_hsl(const ColorRGB& rgb, ColorHSL& hsl);

/**
 * @brief HSL转RGB
 */
void hsl_to_rgb(const ColorHSL& hsl, ColorRGB& rgb);

/**
 * @brief RGB转LAB
 */
void rgb_to_lab(const ColorRGB& rgb, ColorLAB& lab);

/**
 * @brief LAB转RGB
 */
void lab_to_rgb(const ColorLAB& lab, ColorRGB& rgb);

/**
 * @brief RGB转灰度
 */
void rgb_to_gray(const ColorRGB& rgb, uint8_t& gray);

/**
 * @brief RGB转YUV
 */
void rgb_to_yuv(const ColorRGB& rgb, float& y, float& u, float& v);

/**
 * @brief YUV转RGB
 */
void yuv_to_rgb(float y, float u, float v, ColorRGB& rgb);

// ==================== 图像颜色空间转换 ====================

/**
 * @brief 将RGB图像转换为HSV图像
 * @param rgb 输入RGB图像
 * @param hsv 输出HSV图像 (3通道: H[0-180], S[0-255], V[0-255] 便于存储)
 */
void convert_rgb_to_hsv(const ImageData& rgb, ImageData& hsv);

/**
 * @brief 将RGB图像转换为LAB图像
 * @param rgb 输入RGB图像
 * @param lab 输出LAB图像 (3通道: L[0-255], A[0-255], B[0-255] 便于存储)
 */
void convert_rgb_to_lab(const ImageData& rgb, ImageData& lab);

/**
 * @brief 将HSV图像转换为RGB图像
 * @param hsv 输入HSV图像
 * @param rgb 输出RGB图像
 */
void convert_hsv_to_rgb(const ImageData& hsv, ImageData& rgb);

/**
 * @brief 将RGB图像转换为灰度图像
 * @param src 输入RGB图像
 * @param gray 输出灰度图像
 */
void convert_to_gray(const ImageData& src, ImageData& gray);

/**
 * @brief 通用颜色空间转换
 * @param src 输入图像
 * @param dst 输出图像
 * @param source_space 源颜色空间
 * @param target_space 目标颜色空间
 */
void convert_color_space(const ImageData& src, ImageData& dst,
                         ColorSpace source_space, ColorSpace target_space);

// ==================== 颜色距离计算 ====================

/**
 * @brief 计算RGB颜色空间的欧氏距离
 */
float color_distance_rgb(const ColorRGB& c1, const ColorRGB& c2);

/**
 * @brief 计算HSV颜色空间的距离（考虑色相周期性）
 */
float color_distance_hsv(const ColorHSV& c1, const ColorHSV& c2);

/**
 * @brief 计算LAB颜色空间的距离（CIEDE2000简化版）
 */
float color_distance_lab(const ColorLAB& c1, const ColorLAB& c2);

/**
 * @brief 计算颜色的综合相似度（0-1之间，1表示完全相同）
 */
float color_similarity(const ColorRGB& c1, const ColorRGB& c2, ColorSpace space = ColorSpace::RGB);

// ==================== 颜色分割相关 ====================

/**
 * @brief 判断颜色是否在指定范围内
 * @param color 目标颜色
 * @param target 参考颜色
 * @param tolerance 容差
 * @param space 颜色空间
 */
bool is_color_in_range(const ColorRGB& color, const ColorRGB& target,
                       int tolerance, ColorSpace space = ColorSpace::RGB);

/**
 * @brief 颜色分割
 * @param src 输入图像
 * @param mask 输出掩码
 * @param target_color 目标颜色
 * @param tolerance 容差
 * @param space 颜色空间
 */
void color_segment(const ImageData& src, ImageData& mask,
                   const ColorRGB& target_color, int tolerance,
                   ColorSpace space = ColorSpace::RGB);

// ==================== 主颜色提取 ====================

/**
 * @brief 从图像中提取主颜色
 * @param image 输入图像
 * @param colors 输出颜色列表
 * @param max_colors 最大颜色数量
 */
void extract_dominant_colors(const ImageData& image, 
                              Vector<ColorRGB>& colors, 
                              int max_colors = 5);

/**
 * @brief 计算图像颜色直方图
 * @param image 输入图像
 * @param histogram 输出直方图
 * @param bins 直方图箱子数量
 * @param roi 感兴趣区域（可选）
 */
void calc_color_histogram(const ImageData& image, 
                          ColorHistogram& histogram,
                          int bins = 256,
                          const Region* roi = nullptr);

/**
 * @brief 获取颜色名称
 * @param color 输入颜色
 * @return 颜色名称
 */
String get_color_name(const ColorRGB& color);

// ==================== 颜色匹配 ====================

/**
 * @brief 在颜色库中匹配最相似的颜色
 * @param color 输入颜色
 * @param color_library 颜色库
 * @param color_names 颜色名称
 * @param space 颜色空间
 * @return 匹配结果
 */
ColorMatchResult match_color(const ColorRGB& color,
                              const Vector<ColorRGB>& color_library,
                              const Vector<String>& color_names,
                              ColorSpace space = ColorSpace::RGB);

// ==================== 颜色校正 ====================

/**
 * @brief 白平衡校正
 * @param image 输入图像
 * @param corrected 输出校正后的图像
 * @param white_point 白点颜色（可选，不指定则自动检测）
 */
void white_balance(const ImageData& image, ImageData& corrected,
                   const ColorRGB* white_point = nullptr);

/**
 * @brief Gamma校正
 * @param image 输入图像
 * @param corrected 输出校正后的图像
 * @param gamma Gamma值
 */
void gamma_correction(const ImageData& image, ImageData& corrected, float gamma);

/**
 * @brief 自动色阶调整
 * @param image 输入图像
 * @param corrected 输出校正后的图像
 * @param low_clip 低端裁剪比例
 * @param high_clip 高端裁剪比例
 */
void auto_levels(const ImageData& image, ImageData& corrected,
                 float low_clip = 0.01f, float high_clip = 0.01f);

} // namespace color_utils

// ==================== 颜色处理节点 ====================

/**
 * @brief 颜色空间转换节点
 * 
 * 输入:
 *   - image: 输入图像
 * 
 * 输出:
 *   - converted_image: 转换后的图像
 * 
 * 参数:
 *   - source_space: 源颜色空间 (RGB/HSV/HSL/LAB/YUV/GRAY)
 *   - target_space: 目标颜色空间
 */
class ColorConvertNode : public INode {
public:
    explicit ColorConvertNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    ColorSpace parse_color_space(const String& name);
};

/**
 * @brief 颜色分割节点
 * 
 * 输入:
 *   - image: 输入图像
 * 
 * 输出:
 *   - mask: 分割掩码
 *   - segmented_image: 分割后的图像
 * 
 * 参数:
 *   - target_color: 目标颜色 (格式: "r,g,b" 或十六进制 "#RRGGBB")
 *   - tolerance: 容差 (0-255)
 *   - color_space: 颜色空间
 */
class ColorSegmentNode : public INode {
public:
    explicit ColorSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
    // 颜色解析公共静态方法，支持 "r,g,b" 或 "#RRGGBB" 格式
    static bool parse_color(const String& color_str, ColorRGB& color);
};

/**
 * @brief 颜色识别节点
 * 
 * 输入:
 *   - image: 输入图像
 *   - roi: 感兴趣区域（可选）
 * 
 * 输出:
 *   - dominant_color: 主颜色
 *   - color_histogram: 颜色直方图
 *   - color_name: 颜色名称
 *   - dominant_colors: 主颜色列表（JSON数组格式）
 * 
 * 参数:
 *   - max_colors: 最大颜色数量
 *   - method: 识别方法 (histogram/kmeans)
 */
class ColorRecognizeNode : public INode {
public:
    explicit ColorRecognizeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 颜色匹配节点
 * 
 * 输入:
 *   - image: 输入图像
 *   - reference_color: 参考颜色
 * 
 * 输出:
 *   - match_result: 匹配结果
 *   - match_region: 匹配区域
 *   - similarity: 相似度
 * 
 * 参数:
 *   - tolerance: 容差
 *   - min_area: 最小匹配面积
 *   - color_space: 颜色空间
 */
class ColorMatchNode : public INode {
public:
    explicit ColorMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 颜色校正节点
 * 
 * 输入:
 *   - image: 输入图像
 * 
 * 输出:
 *   - corrected_image: 校正后的图像
 *   - white_point: 检测到的白点（白平衡模式）
 * 
 * 参数:
 *   - method: 校正方法 (white_balance/gamma/auto_levels)
 *   - white_point: 白点颜色（白平衡模式可选）
 *   - gamma: Gamma值（Gamma模式）
 *   - low_clip: 低端裁剪比例（自动色阶模式）
 *   - high_clip: 高端裁剪比例（自动色阶模式）
 */
class ColorCorrectNode : public INode {
public:
    explicit ColorCorrectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 颜色直方图分析节点
 * 
 * 输入:
 *   - image: 输入图像
 *   - roi: 感兴趣区域（可选）
 * 
 * 输出:
 *   - histogram_r: 红色通道直方图
 *   - histogram_g: 绿色通道直方图
 *   - histogram_b: 蓝色通道直方图
 *   - histogram_gray: 灰度直方图（灰度图像时）
 *   - mean_color: 平均颜色
 *   - std_dev: 颜色标准差
 * 
 * 参数:
 *   - bins: 直方图箱子数量
 *   - compute_stats: 是否计算统计信息
 */
class ColorHistogramNode : public INode {
public:
    explicit ColorHistogramNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf