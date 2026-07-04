#pragma once

#include "ovf/core/types.h"
#include <vector>

namespace ovf {
namespace algorithm {

/**
 * @brief 图像处理辅助工具
 */
namespace image_utils {

/**
 * @brief 在图像上绘制矩形
 * @param image 图像数据
 * @param x 矩形起始X
 * @param y 矩形起始Y
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param r 红色分量 (0-255)
 * @param g 绿色分量 (0-255)
 * @param b 蓝色分量 (0-255)
 */
void draw_rect(ImageData& image, int x, int y, int width, int height, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 在图像上绘制圆形
 */
void draw_circle(ImageData& image, int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 在图像上绘制直线
 */
void draw_line(ImageData& image, int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 在图像上绘制点
 */
void draw_point(ImageData& image, int x, int y, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 转换灰度图像为RGB
 */
void gray_to_rgb(const uint8_t* gray, int width, int height, std::vector<uint8_t>& rgb);

/**
 * @brief 转换RGB图像为灰度
 */
void rgb_to_gray(const uint8_t* rgb, int width, int height, std::vector<uint8_t>& gray);

/**
 * @brief 图像翻转（水平/垂直）
 * @param src 源图像
 * @param dst 目标图像
 * @param horizontal 是否水平翻转
 * @param vertical 是否垂直翻转
 */
void flip(const uint8_t* src, int width, int height, int channels,
          std::vector<uint8_t>& dst, bool horizontal, bool vertical);

/**
 * @brief 图像缩放（最近邻插值）
 */
void resize_nearest(const uint8_t* src, int src_w, int src_h, int channels,
                    std::vector<uint8_t>& dst, int dst_w, int dst_h);

/**
 * @brief 图像缩放（双线性插值）
 */
void resize_bilinear(const uint8_t* src, int src_w, int src_h, int channels,
                     std::vector<uint8_t>& dst, int dst_w, int dst_h);

/**
 * @brief 创建图像金字塔
 */
void build_pyramid(const uint8_t* src, int width, int height, int channels,
                   std::vector<std::vector<uint8_t>>& pyramid, int levels);

/**
 * @brief 图像通道分离
 */
void split_channels(const uint8_t* src, int width, int height, int channels,
                    std::vector<std::vector<uint8_t>>& separated);

/**
 * @brief 图像通道合并
 */
void merge_channels(const std::vector<std::vector<uint8_t>>& channels,
                    int width, int height, std::vector<uint8_t>& merged);

/**
 * @brief 计算图像直方图
 */
void calc_histogram(const uint8_t* src, int size, std::vector<int>& histogram, int bins = 256);

/**
 * @brief 直方图均衡化
 */
void histogram_equalize(uint8_t* src, int size);

/**
 * @brief 图像归一化到指定范围
 */
void normalize(uint8_t* src, int size, uint8_t min_val, uint8_t max_val);

/**
 * @brief 计算图像均值和标准差
 */
void calc_mean_std(const uint8_t* src, int size, double& mean, double& std_dev);

/**
 * @brief 图像算术运算
 */
void add(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);
void subtract(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);
void multiply(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);
void divide(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);

/**
 * @brief 图像逻辑运算
 */
void bitwise_and(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);
void bitwise_or(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);
void bitwise_xor(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result);
void bitwise_not(const uint8_t* a, int size, std::vector<uint8_t>& result);

} // namespace image_utils

} // namespace algorithm
} // namespace ovf