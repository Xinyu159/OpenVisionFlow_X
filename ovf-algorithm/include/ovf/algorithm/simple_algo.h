/**
 * @file simple_algo.h
 * @brief OpenVisionFlow 轻量级图像处理（不依赖OpenCV）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 简化的图像处理辅助函数
 */
namespace image_utils {

// 灰度转换
inline void rgb_to_gray(const ImageData& rgb, ImageData& gray) {
    if (rgb.channels != 3 || rgb.empty()) return;
    
    gray.width = rgb.width;
    gray.height = rgb.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(rgb.width * rgb.height);
    gray.timestamp = rgb.timestamp;
    gray.frame_id = rgb.frame_id;
    gray.source_id = rgb.source_id;
    
    for (size_t i = 0; i < gray.data.size(); ++i) {
        // RGB -> Gray: 0.299*R + 0.587*G + 0.114*B
        uint8_t r = rgb.data[i * 3];
        uint8_t g = rgb.data[i * 3 + 1];
        uint8_t b = rgb.data[i * 3 + 2];
        gray.data[i] = static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);
    }
}

// 图像反转
inline void invert_image(const ImageData& input, ImageData& output) {
    if (input.empty()) return;
    
    output = input;
    for (auto& pixel : output.data) {
        pixel = 255 - pixel;
    }
}

// 亮度调整
inline void adjust_brightness(const ImageData& input, ImageData& output, int delta) {
    if (input.empty()) return;
    
    output = input;
    for (auto& pixel : output.data) {
        int new_val = static_cast<int>(pixel) + delta;
        pixel = static_cast<uint8_t>(std::max(0, std::min(255, new_val)));
    }
}

// 阈值化
inline void threshold_image(const ImageData& input, ImageData& output, uint8_t thresh, uint8_t max_val = 255) {
    if (input.empty()) return;
    
    output.width = input.width;
    output.height = input.height;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data.resize(input.data.size());
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;
    
    for (size_t i = 0; i < input.data.size(); ++i) {
        output.data[i] = input.data[i] >= thresh ? max_val : 0;
    }
}

// 简化的均值滤波（3x3）
inline void blur_image(const ImageData& input, ImageData& output) {
    if (input.empty()) return;
    
    output = input;
    
    uint32_t w = input.width;
    uint32_t h = input.height;
    
    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            if (input.channels == 1) {
                int sum = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        sum += input.data[(y + dy) * w + (x + dx)];
                    }
                }
                output.data[y * w + x] = static_cast<uint8_t>(sum / 9);
            } else if (input.channels == 3) {
                for (int c = 0; c < 3; ++c) {
                    int sum = 0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            sum += input.data[(y + dy) * w * 3 + (x + dx) * 3 + c];
                        }
                    }
                    output.data[y * w * 3 + x * 3 + c] = static_cast<uint8_t>(sum / 9);
                }
            }
        }
    }
}

// 创建测试图像
inline ImageData create_test_image(uint32_t width, uint32_t height, const String& source_id = "") {
    ImageData img;
    img.width = width;
    img.height = height;
    img.channels = 3;
    img.format = ImageFormat::BGR8;
    img.data.resize(width * height * 3);
    img.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    img.frame_id = 0;
    img.source_id = source_id;
    
    // 创建渐变背景
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint8_t gray = static_cast<uint8_t>((y + x) * 255 / (height + width));
            img.data[(y * width + x) * 3] = gray;      // B
            img.data[(y * width + x) * 3 + 1] = gray;  // G
            img.data[(y * width + x) * 3 + 2] = gray;  // R
        }
    }
    
    // 添加圆形
    uint32_t cx = width / 2;
    uint32_t cy = height / 2;
    uint32_t r = std::min(width, height) / 4;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint32_t dx = x - cx;
            uint32_t dy = y - cy;
            if (dx * dx + dy * dy <= r * r) {
                img.data[(y * width + x) * 3] = 0;      // B
                img.data[(y * width + x) * 3 + 1] = 0;  // G
                img.data[(y * width + x) * 3 + 2] = 255; // R (红色圆形)
            }
        }
    }
    
    return img;
}

} // namespace image_utils

/**
 * @brief 图像源节点（轻量级实现）
 */
class SimpleImageSourceNode : public INode {
public:
    SimpleImageSourceNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 图像预处理节点（轻量级实现）
 */
class SimpleImagePreprocessNode : public INode {
public:
    SimpleImagePreprocessNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 阈值化节点（轻量级实现）
 */
class SimpleThresholdNode : public INode {
public:
    SimpleThresholdNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf