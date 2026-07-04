/**
 * @file segmentation.h
 * @brief 图像分割算法模块（纯C++实现，不依赖OpenCV）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 阈值分割类型
 */
enum class ThresholdType {
    Binary,         // 二值化: src > thresh -> maxval, else 0
    BinaryInv,      // 反二值化: src > thresh -> 0, else maxval
    Trunc,          // 截断: src > thresh -> thresh, else src
    ToZero,         // 阈值归零: src > thresh -> src, else 0
    ToZeroInv       // 反阈值归零: src > thresh -> 0, else src
};

/**
 * @brief 自适应阈值方法
 */
enum class AdaptiveMethod {
    Mean,           // 局部均值
    Gaussian        // 局部加权平均（简化为均值+权重）
};

/**
 * @brief 边缘分割方法
 */
enum class EdgeSegmentMethod {
    Sobel,          // Sobel边缘
    Canny,          // Canny边缘（简化）
    Laplacian,      // Laplacian边缘
    Gradient        // 梯度边缘
};

/**
 * @brief 超像素方法
 */
enum class SuperpixelMethod {
    SLIC,           // SLIC超像素
    SEEDS           // SEEDS超像素（简化）
};

/**
 * @brief 分割工具函数
 */
namespace seg_utils {

/**
 * @brief 固定阈值分割
 */
void threshold_segment(const uint8_t* src, uint8_t* dst, int width, int height,
                       int thresh, int max_val, ThresholdType type);

/**
 * @brief 自适应阈值分割
 */
void adaptive_threshold(const uint8_t* src, uint8_t* dst, int width, int height,
                        int block_size, int c, AdaptiveMethod method, ThresholdType type);

/**
 * @brief 颜色分割
 */
void color_segment_rgb(const uint8_t* src, uint8_t* mask, int width, int height,
                       uint8_t r, uint8_t g, uint8_t b, int tolerance);

/**
 * @brief 边缘分割
 */
void edge_segment(const uint8_t* src, uint8_t* dst, int width, int height,
                  EdgeSegmentMethod method, int low_thresh, int high_thresh);

/**
 * @brief 分水岭分割（基于标记）
 */
void watershed_segment(const uint8_t* src, int32_t* markers, int32_t* result,
                       int width, int height);

/**
 * @brief 区域生长分割
 */
void region_growing(const uint8_t* src, uint8_t* dst, int width, int height,
                    int seed_x, int seed_y, int threshold, bool use_8connectivity);

/**
 * @brief 区域分裂合并
 */
void region_split_merge(const uint8_t* src, uint8_t* dst, int width, int height,
                        int min_size, int threshold);

/**
 * @brief K均值聚类分割（颜色聚类）
 */
void kmeans_segment(const uint8_t* src, uint8_t* dst, int width, int height, int channels,
                    int k, int max_iterations);

/**
 * @brief 图割分割（简化版）
 */
void graphcut_segment(const uint8_t* src, uint8_t* mask, int width, int height,
                      int seed_x1, int seed_y1, int seed_x2, int seed_y2);

/**
 * @brief GrabCut分割（简化版）
 */
void grabcut_segment(const uint8_t* src, uint8_t* mask, int width, int height, int channels,
                     int rect_x, int rect_y, int rect_w, int rect_h, int iterations);

/**
 * @brief MeanShift分割
 */
void meanshift_segment(const uint8_t* src, uint8_t* dst, int width, int height, int channels,
                       float spatial_radius, float color_radius, int min_density);

/**
 * @brief 超像素分割（SLIC简化版）
 */
void superpixel_segment(const uint8_t* src, int32_t* labels, int width, int height, int channels,
                        int num_superpixels, int compactness, int iterations);

/**
 * @brief 连通区域标记
 */
int connected_components(const uint8_t* binary, int32_t* labels, int width, int height, bool use_8connectivity);

/**
 * @brief 计算区域统计信息
 */
void region_statistics(const uint8_t* src, const int32_t* labels, int width, int height,
                       int label, int& area, int& min_val, int& max_val, float& mean_val);

/**
 * @brief 填充种子点区域
 */
void flood_fill(const uint8_t* src, uint8_t* dst, int width, int height,
                int seed_x, int seed_y, uint8_t fill_value, int tolerance);

/**
 * @brief 计算图像梯度
 */
void compute_gradient(const uint8_t* src, uint8_t* grad_x, uint8_t* grad_y,
                      uint8_t* magnitude, int width, int height);

/**
 * @brief 高斯滤波（简化）
 */
void gaussian_blur(const uint8_t* src, uint8_t* dst, int width, int height, int channels, int ksize);

/**
 * @brief 中值滤波
 */
void median_blur(const uint8_t* src, uint8_t* dst, int width, int height, int channels, int ksize);

} // namespace seg_utils

// ==================== 基分割节点 ====================

/**
 * @brief 阈值分割节点
 *
 * 输入:
 *   - image: 输入图像
 *
 * 输出:
 *   - binary: 二值图像
 *   - threshold_value: 使用的阈值
 *
 * 参数:
 *   - threshold_type: 阈值类型 (Binary/BinaryInv/Trunc/ToZero/ToZeroInv)
 *   - threshold_value: 阈值值 (0-255)
 *   - max_value: 最大值 (默认255)
 *   - auto_threshold: 是否自动计算阈值（Otsu方法）
 */
class ThresholdSegmentNode : public INode {
public:
    explicit ThresholdSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    int compute_otsu_threshold(const uint8_t* data, int size);
};

/**
 * @brief 自适应阈值分割节点
 *
 * 输入:
 *   - image: 输入图像（灰度）
 *
 * 输出:
 *   - binary: 二值图像
 *
 * 参数:
 *   - method: 自适应方法 (Mean/Gaussian)
 *   - block_size: 邻域块大小 (默认31)
 *   - c: 常数偏移量 (默认5)
 *   - threshold_type: 阈值类型
 */
class AdaptiveThresholdNode : public INode {
public:
    explicit AdaptiveThresholdNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 颜色分割节点
 *
 * 输入:
 *   - image: 输入RGB图像
 *
 * 输出:
 *   - mask: 分割掩码
 *   - segmented_image: 分割后的彩色图像
 *
 * 参数:
 *   - target_color: 目标颜色 (格式: "r,g,b" 或 "#RRGGBB")
 *   - tolerance: 容差 (0-255)
 *   - color_space: 颜色空间 (RGB/HSV)
 */
class ColorSegmentNode : public INode {
public:
    explicit ColorSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    bool parse_color(const String& color_str, uint8_t& r, uint8_t& g, uint8_t& b);
};

/**
 * @brief 边缘分割节点
 *
 * 输入:
 *   - image: 输入图像
 *
 * 输出:
 *   - edges: 边缘图像
 *   - edge_mask: 边缘掩码
 *
 * 参数:
 *   - method: 边缘检测方法 (Sobel/Canny/Laplacian/Gradient)
 *   - low_threshold: 低阈值 (默认50)
 *   - high_threshold: 高阈值 (默认100)
 *   - edge_threshold: 边缘阈值（用于生成掩码）
 */
class EdgeSegmentNode : public INode {
public:
    explicit EdgeSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ==================== 高级分割节点 ====================

/**
 * @brief 分水岭分割节点
 *
 * 输入:
 *   - image: 输入图像
 *   - markers: 标记图像（可选）
 *
 * 输出:
 *   - segmented: 分割结果
 *   - boundaries: 分割边界
 *   - num_regions: 区域数量
 *
 * 参数:
 *   - marker_method: 标记生成方法 (Auto/User/Gradient)
 *   - min_region_size: 最小区域大小
 */
class WatershedNode : public INode {
public:
    explicit WatershedNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    void generate_markers_from_gradient(const uint8_t* src, int32_t* markers, int w, int h);
};

/**
 * @brief 区域生长分割节点
 *
 * 输入:
 *   - image: 输入图像
 *   - seed_point: 种子点（可选）
 *
 * 输出:
 *   - region: 生长区域掩码
 *   - region_area: 区域面积
 *   - region_mean: 区域平均值
 *
 * 参数:
 *   - seed_x: 种子点X坐标
 *   - seed_y: 种子点Y坐标
 *   - threshold: 生长阈值 (默认10)
 *   - connectivity: 连通性 (4/8)
 *   - auto_seed: 自动选择种子点
 */
class RegionGrowingNode : public INode {
public:
    explicit RegionGrowingNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    void find_auto_seed(const uint8_t* src, int w, int h, int& seed_x, int& seed_y);
};

/**
 * @brief 区域分裂合并分割节点
 *
 * 输入:
 *   - image: 输入图像
 *
 * 输出:
 *   - segmented: 分割结果
 *   - region_labels: 区域标签
 *   - num_regions: 区域数量
 *
 * 参数:
 *   - min_size: 最小区域大小 (默认16)
 *   - threshold: 分裂阈值 (默认20)
 *   - merge_threshold: 合并阈值 (默认10)
 */
class RegionSplitMergeNode : public INode {
public:
    explicit RegionSplitMergeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief K均值聚类分割节点
 *
 * 输入:
 *   - image: 输入图像
 *
 * 输出:
 *   - segmented: 分割结果（量化后的图像）
 *   - labels: 像素标签
 *   - centers: 聚类中心颜色
 *
 * 参数:
 *   - k: 聚类数量 (默认4)
 *   - max_iterations: 最大迭代次数 (默认100)
 *   - convergence_threshold: 收敛阈值
 *   - color_space: 颜色空间 (RGB/HSV/LAB)
 */
class KMeansSegmentNode : public INode {
public:
    explicit KMeansSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 图割分割节点（简化版）
 *
 * 输入:
 *   - image: 输入图像
 *   - foreground_seed: 前景种子点（可选）
 *   - background_seed: 背景种子点（可选）
 *
 * 输出:
 *   - mask: 分割掩码
 *   - foreground_image: 前景图像
 *   - background_image: 背景图像
 *
 * 参数:
 *   - fg_seed_x: 前景种子X
 *   - fg_seed_y: 前景种子Y
 *   - bg_seed_x: 背景种子X
 *   - bg_seed_y: 背景种子Y
 *   - lambda: 平衡参数 (默认5)
 */
class GraphCutNode : public INode {
public:
    explicit GraphCutNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ==================== 特定分割节点 ====================

/**
 * @brief GrabCut分割节点（简化版）
 *
 * 输入:
 *   - image: 输入RGB图像
 *   - rect: 矩形区域（可选）
 *
 * 输出:
 *   - mask: 分割掩码 (0:背景, 1:前景)
 *   - foreground: 前景提取图像
 *   - background: 背景图像
 *
 * 参数:
 *   - rect_x: 矩形左上角X
 *   - rect_y: 矩形左上角Y
 *   - rect_w: 矩形宽度
 *   - rect_h: 矩形高度
 *   - iterations: 迭代次数 (默认5)
 *   - auto_rect: 是否自动确定矩形
 */
class GrabCutNode : public INode {
public:
    explicit GrabCutNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    void init_mask_from_rect(uint8_t* mask, int w, int h, int rx, int ry, int rw, int rh);
    void refine_segmentation(const uint8_t* src, uint8_t* mask, int w, int h, int channels, int iterations);
};

/**
 * @brief MeanShift分割节点
 *
 * 输入:
 *   - image: 输入图像
 *
 * 输出:
 *   - segmented: 分割结果
 *   - labels: 区域标签
 *   - num_regions: 区域数量
 *
 * 参数:
 *   - spatial_radius: 空间半径 (默认10)
 *   - color_radius: 颜色半径 (默认20)
 *   - min_density: 最小密度 (默认50)
 *   - max_iterations: 最大迭代次数
 */
class MeanShiftSegmentNode : public INode {
public:
    explicit MeanShiftSegmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 超像素分割节点（SLIC简化版）
 *
 * 输入:
 *   - image: 输入图像
 *
 * 输出:
 *   - labels: 超像素标签
 *   - boundaries: 超像素边界
 *   - num_superpixels: 超像素数量
 *   - superpixel_image: 超像素平均颜色图像
 *
 * 参数:
 *   - num_superpixels: 超像素数量 (默认100)
 *   - compactness: 紧致度 (默认10)
 *   - iterations: 迭代次数 (默认10)
 *   - method: 超像素方法 (SLIC/SEEDS)
 */
class SuperpixelNode : public INode {
public:
    explicit SuperpixelNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    void draw_superpixel_boundaries(const int32_t* labels, uint8_t* boundaries, int w, int h);
    void create_superpixel_image(const uint8_t* src, const int32_t* labels, uint8_t* dst,
                                  int w, int h, int channels);
};

} // namespace algorithm
} // namespace ovf