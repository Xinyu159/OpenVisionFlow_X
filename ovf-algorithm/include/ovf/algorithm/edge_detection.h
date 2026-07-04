/**
 * @file edge_detection.h
 * @brief 边缘检测算子（纯C++实现，不依赖OpenCV）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 边缘检测工具
 */
namespace edge_utils {

// Sobel边缘检测
inline void sobel_edge(const ImageData& input, ImageData& output, 
                       int threshold = 50) {
    if (input.empty() || input.channels != 1) return;
    
    output.width = input.width;
    output.height = input.height;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data.resize(input.width * input.height);
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;
    
    uint32_t w = input.width;
    uint32_t h = input.height;
    
    // Sobel算子
    // Gx = [-1 0 1; -2 0 2; -1 0 1]
    // Gy = [-1 -2 -1; 0 0 0; 1 2 1]
    
    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            int gx = 0, gy = 0;
            
            // 计算3x3邻域
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    uint8_t pixel = input.data[(y + dy) * w + (x + dx)];
                    
                    // Sobel Gx权重
                    gx += pixel * dx * (dy == 0 ? 2 : 1);
                    
                    // Sobel Gy权重
                    gy += pixel * dy * (dx == 0 ? 2 : 1);
                }
            }
            
            // 计算梯度幅值
            int magnitude = static_cast<int>(std::sqrt(gx * gx + gy * gy));
            
            // 阈值化
            output.data[y * w + x] = (magnitude > threshold) ? 255 : 0;
        }
    }
}

// Laplacian边缘检测
inline void laplacian_edge(const ImageData& input, ImageData& output) {
    if (input.empty() || input.channels != 1) return;
    
    output.width = input.width;
    output.height = input.height;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data.resize(input.width * input.height);
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;
    
    uint32_t w = input.width;
    uint32_t h = input.height;
    
    // Laplacian算子: [0 1 0; 1 -4 1; 0 1 0]
    
    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            int laplacian = 0;
            
            // 中心像素 * (-4)
            laplacian -= 4 * input.data[y * w + x];
            
            // 四邻域 * 1
            laplacian += input.data[(y - 1) * w + x];     // 上
            laplacian += input.data[(y + 1) * w + x];     // 下
            laplacian += input.data[y * w + (x - 1)];     // 左
            laplacian += input.data[y * w + (x + 1)];     // 右
            
            // 取绝对值
            laplacian = std::abs(laplacian);
            
            // 限制范围
            output.data[y * w + x] = static_cast<uint8_t>(std::min(255, laplacian));
        }
    }
}

// Canny边缘检测简化版
inline void canny_edge_simple(const ImageData& input, ImageData& output,
                               int low_threshold = 50, int high_threshold = 100) {
    // 简化版Canny: Sobel + 双阈值
    
    ImageData sobel_output;
    sobel_edge(input, sobel_output, low_threshold);
    
    if (sobel_output.empty()) return;
    
    output = sobel_output;
    
    // 双阈值处理
    uint32_t w = output.width;
    uint32_t h = output.height;
    
    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            size_t idx = y * w + x;
            uint8_t val = output.data[idx];
            
            if (val >= high_threshold) {
                output.data[idx] = 255;  // 强边缘
            } else if (val >= low_threshold) {
                // 弱边缘：检查是否有强边缘邻居
                bool has_strong_neighbor = false;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (output.data[(y + dy) * w + (x + dx)] >= high_threshold) {
                            has_strong_neighbor = true;
                            break;
                        }
                    }
                    if (has_strong_neighbor) break;
                }
                
                output.data[idx] = has_strong_neighbor ? 255 : 0;
            } else {
                output.data[idx] = 0;
            }
        }
    }
}

} // namespace edge_utils

/**
 * @brief Sobel边缘检测节点
 */
class SobelEdgeNode : public INode {
public:
    SobelEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief Canny边缘检测节点（简化版）
 */
class CannyEdgeNode : public INode {
public:
    CannyEdgeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf