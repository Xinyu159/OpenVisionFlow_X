/**
 * @file blob_analysis.h
 * @brief Blob分析算子（纯C++实现，不依赖OpenCV）
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>

namespace ovf {
namespace algorithm {

/**
 * @brief Blob（连通区域）结构
 */
struct Blob {
    uint32_t id = 0;            // Blob ID
    uint32_t area = 0;          // 面积（像素数）
    uint32_t x = 0;             // 中心X坐标
    uint32_t y = 0;             // 中心Y坐标
    uint32_t min_x = 0;         // 最小X
    uint32_t max_x = 0;         // 最大X
    uint32_t min_y = 0;         // 最小Y
    uint32_t max_y = 0;         // 最大Y
    uint32_t width = 0;         // 宽度
    uint32_t height = 0;        // 高度
    double circularity = 0.0;   // 圆度
    double aspect_ratio = 0.0;  // 长宽比
    double orientation = 0.0;   // 方向角
    bool valid = false;
};

/**
 * @brief Blob分析工具
 */
namespace blob_utils {

// 连通区域标记（两遍扫描法）
inline std::vector<Blob> find_blobs(const ImageData& binary_img, 
                                     uint32_t min_area = 10, 
                                     uint32_t max_area = 1000000) {
    std::vector<Blob> blobs;
    
    if (binary_img.empty() || binary_img.channels != 1) {
        return blobs;
    }
    
    uint32_t width = binary_img.width;
    uint32_t height = binary_img.height;
    
    // 第一遍：标记
    std::vector<uint32_t> labels(width * height, 0);
    std::vector<uint32_t> label_equiv(1, 0);  // 标签等价表
    uint32_t current_label = 0;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;
            
            // 检查是否为前景像素
            if (binary_img.data[idx] == 0) continue;
            
            // 检查邻居标签
            uint32_t left_label = (x > 0) ? labels[idx - 1] : 0;
            uint32_t top_label = (y > 0) ? labels[idx - width] : 0;
            
            if (left_label == 0 && top_label == 0) {
                // 新区域
                current_label++;
                label_equiv.push_back(current_label);
                labels[idx] = current_label;
            } else if (left_label != 0 && top_label == 0) {
                // 继承左边标签
                labels[idx] = left_label;
            } else if (left_label == 0 && top_label != 0) {
                // 继承上面标签
                labels[idx] = top_label;
            } else {
                // 两边都有标签，取最小的
                uint32_t min_label = std::min(left_label, top_label);
                uint32_t max_label = std::max(left_label, top_label);
                labels[idx] = min_label;
                
                // 合并等价标签
                if (max_label < label_equiv.size()) {
                    label_equiv[max_label] = min_label;
                }
            }
        }
    }
    
    // 第二遍：合并等价标签并统计
    std::vector<Blob> temp_blobs(label_equiv.size());
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;
            uint32_t label = labels[idx];
            
            if (label == 0) continue;
            
            // 查找最终标签
            while (label_equiv[label] != label && label < label_equiv.size()) {
                label = label_equiv[label];
            }
            
            if (label >= temp_blobs.size()) continue;
            
            Blob& blob = temp_blobs[label];
            if (!blob.valid) {
                blob.id = label;
                blob.min_x = x;
                blob.max_x = x;
                blob.min_y = y;
                blob.max_y = y;
                blob.valid = true;
            }
            
            blob.area++;
            blob.min_x = std::min(blob.min_x, x);
            blob.max_x = std::max(blob.max_x, x);
            blob.min_y = std::min(blob.min_y, y);
            blob.max_y = std::max(blob.max_y, y);
        }
    }
    
    // 计算Blob属性并筛选
    uint32_t blob_id = 1;
    for (auto& blob : temp_blobs) {
        if (!blob.valid) continue;
        
        // 面积筛选
        if (blob.area < min_area || blob.area > max_area) continue;
        
        // 计算属性
        blob.width = blob.max_x - blob.min_x + 1;
        blob.height = blob.max_y - blob.min_y + 1;
        blob.x = (blob.min_x + blob.max_x) / 2;
        blob.y = (blob.min_y + blob.max_y) / 2;
        blob.aspect_ratio = static_cast<double>(blob.width) / blob.height;
        
        // 圆度近似计算（边界框面积比）
        double perimeter = 2.0 * (blob.width + blob.height);
        blob.circularity = 4.0 * M_PI * blob.area / (perimeter * perimeter);
        
        blob.id = blob_id++;
        blobs.push_back(blob);
    }
    
    return blobs;
}

// 绘制Blob边界框
inline void draw_blob_boxes(ImageData& img, const std::vector<Blob>& blobs, 
                              uint8_t r = 255, uint8_t g = 0, uint8_t b = 0) {
    if (img.channels != 3) return;
    
    for (const auto& blob : blobs) {
        // 绘制边界框
        for (uint32_t x = blob.min_x; x <= blob.max_x; ++x) {
            // 上边
            if (blob.min_y < img.height) {
                size_t idx = (blob.min_y * img.width + x) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
            // 下边
            if (blob.max_y < img.height) {
                size_t idx = (blob.max_y * img.width + x) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
        }
        
        for (uint32_t y = blob.min_y; y <= blob.max_y; ++y) {
            // 左边
            if (blob.min_x < img.width) {
                size_t idx = (y * img.width + blob.min_x) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
            // 右边
            if (blob.max_x < img.width) {
                size_t idx = (y * img.width + blob.max_x) * 3;
                img.data[idx] = b;
                img.data[idx + 1] = g;
                img.data[idx + 2] = r;
            }
        }
        
        // 绘制中心点
        if (blob.x < img.width && blob.y < img.height) {
            size_t idx = (blob.y * img.width + blob.x) * 3;
            img.data[idx] = b;
            img.data[idx + 1] = g;
            img.data[idx + 2] = r;
            
            // 绘制十字
            for (int dx = -2; dx <= 2; ++dx) {
                uint32_t px = blob.x + dx;
                if (px < img.width) {
                    size_t idx2 = (blob.y * img.width + px) * 3;
                    img.data[idx2] = b;
                    img.data[idx2 + 1] = g;
                    img.data[idx2 + 2] = r;
                }
            }
            for (int dy = -2; dy <= 2; ++dy) {
                uint32_t py = blob.y + dy;
                if (py < img.height) {
                    size_t idx2 = (py * img.width + blob.x) * 3;
                    img.data[idx2] = b;
                    img.data[idx2 + 1] = g;
                    img.data[idx2 + 2] = r;
                }
            }
        }
    }
}

} // namespace blob_utils

/**
 * @brief Blob分析节点
 */
class BlobAnalysisNode : public INode {
public:
    BlobAnalysisNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf