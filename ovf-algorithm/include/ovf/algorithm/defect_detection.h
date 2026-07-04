/**
 * @file defect_detection.h
 * @brief 缺陷检测算子（纯C++实现，不依赖OpenCV）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace ovf {
namespace algorithm {

/**
 * @brief 缺陷检测结果
 */
struct DefectResult {
    int id;                     // 缺陷ID
    std::string type;           // 缺陷类型
    int x, y;                   // 缺陷位置
    int width, height;          // 缺陷尺寸
    double severity;            // 严重程度 (0-1)
    double confidence;           // 置信度 (0-1)
    std::vector<uint8_t> mask;  // 缺陷掩码（可选）
};

/**
 * @brief 缺陷检测工具函数
 */
namespace defect_utils {

// 转换为灰度图像
inline void to_grayscale(const ImageData& input, ImageData& output) {
    if (input.empty()) return;

    output.width = input.width;
    output.height = input.height;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data.resize(output.width * output.height);
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    if (input.channels == 1) {
        output.data = input.data;
    } else if (input.channels >= 3) {
        for (size_t i = 0; i < output.data.size(); ++i) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            output.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r);
        }
    }
}

// 计算图像差分
inline void image_diff(const ImageData& img1, const ImageData& img2, ImageData& output) {
    if (img1.empty() || img2.empty()) return;
    if (img1.width != img2.width || img1.height != img2.height) return;

    output.width = img1.width;
    output.height = img1.height;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data.resize(output.width * output.height);

    for (size_t i = 0; i < output.data.size(); ++i) {
        int diff = std::abs(static_cast<int>(img1.data[i]) - static_cast<int>(img2.data[i]));
        output.data[i] = static_cast<uint8_t>(std::min(255, diff));
    }
}

// 计算局部对比度
inline double compute_local_contrast(const ImageData& img, int x, int y, int win_size) {
    if (img.empty()) return 0.0;

    int half = win_size / 2;
    double sum = 0, sum_sq = 0;
    int count = 0;

    for (int dy = -half; dy <= half; ++dy) {
        for (int dx = -half; dx <= half; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < static_cast<int>(img.width) &&
                ny >= 0 && ny < static_cast<int>(img.height)) {
                double val = img.data[ny * img.width + nx];
                sum += val;
                sum_sq += val * val;
                count++;
            }
        }
    }

    if (count < 2) return 0.0;
    double mean = sum / count;
    double variance = (sum_sq - sum * sum / count) / (count - 1);
    return std::sqrt(variance) / (mean + 1.0);
}

// 计算梯度幅值
inline void compute_gradient(const ImageData& input, std::vector<double>& gradient) {
    if (input.empty()) return;

    uint32_t w = input.width;
    uint32_t h = input.height;
    gradient.resize(w * h, 0.0);

    for (uint32_t y = 1; y < h - 1; ++y) {
        for (uint32_t x = 1; x < w - 1; ++x) {
            double gx = static_cast<double>(input.data[y * w + x + 1]) -
                       static_cast<double>(input.data[y * w + x - 1]);
            double gy = static_cast<double>(input.data[(y + 1) * w + x]) -
                       static_cast<double>(input.data[(y - 1) * w + x]);
            gradient[y * w + x] = std::sqrt(gx * gx + gy * gy);
        }
    }
}

// 计算局部均值
inline double compute_local_mean(const ImageData& img, int x, int y, int win_size) {
    if (img.empty()) return 0.0;

    int half = win_size / 2;
    double sum = 0;
    int count = 0;

    for (int dy = -half; dy <= half; ++dy) {
        for (int dx = -half; dx <= half; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < static_cast<int>(img.width) &&
                ny >= 0 && ny < static_cast<int>(img.height)) {
                sum += img.data[ny * img.width + nx];
                count++;
            }
        }
    }

    return count > 0 ? sum / count : 0.0;
}

// 计算局部标准差
inline double compute_local_std(const ImageData& img, int x, int y, int win_size, double mean) {
    if (img.empty()) return 0.0;

    int half = win_size / 2;
    double sum_sq = 0;
    int count = 0;

    for (int dy = -half; dy <= half; ++dy) {
        for (int dx = -half; dx <= half; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < static_cast<int>(img.width) &&
                ny >= 0 && ny < static_cast<int>(img.height)) {
                double diff = img.data[ny * img.width + nx] - mean;
                sum_sq += diff * diff;
                count++;
            }
        }
    }

    return count > 1 ? std::sqrt(sum_sq / (count - 1)) : 0.0;
}

// 连通区域标记（简化版）
struct ConnectedComponent {
    int x, y, width, height;
    int pixel_count;
    int min_x, min_y, max_x, max_y;
};

inline std::vector<ConnectedComponent> find_connected_components(
    const std::vector<uint8_t>& binary, uint32_t width, uint32_t height) {

    std::vector<ConnectedComponent> components;
    std::vector<int> labels(width * height, 0);
    int current_label = 0;

    // 简化的连通区域检测
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;
            if (binary[idx] > 0 && labels[idx] == 0) {
                // 新区域
                current_label++;
                ConnectedComponent cc;
                cc.x = x; cc.y = y;
                cc.min_x = x; cc.max_x = x;
                cc.min_y = y; cc.max_y = y;
                cc.pixel_count = 0;

                // 简单的洪水填充
                std::vector<size_t> stack;
                stack.push_back(idx);
                while (!stack.empty()) {
                    size_t cur = stack.back();
                    stack.pop_back();
                    if (labels[cur] != 0) continue;

                    labels[cur] = current_label;
                    cc.pixel_count++;

                    uint32_t cx = cur % width;
                    uint32_t cy = cur / width;

                    cc.min_x = std::min(cc.min_x, static_cast<int>(cx));
                    cc.max_x = std::max(cc.max_x, static_cast<int>(cx));
                    cc.min_y = std::min(cc.min_y, static_cast<int>(cy));
                    cc.max_y = std::max(cc.max_y, static_cast<int>(cy));

                    // 4邻域
                    if (cx > 0 && binary[cur - 1] > 0 && labels[cur - 1] == 0)
                        stack.push_back(cur - 1);
                    if (cx < width - 1 && binary[cur + 1] > 0 && labels[cur + 1] == 0)
                        stack.push_back(cur + 1);
                    if (cy > 0 && binary[cur - width] > 0 && labels[cur - width] == 0)
                        stack.push_back(cur - width);
                    if (cy < height - 1 && binary[cur + width] > 0 && labels[cur + width] == 0)
                        stack.push_back(cur + width);
                }

                cc.width = cc.max_x - cc.min_x + 1;
                cc.height = cc.max_y - cc.min_y + 1;
                cc.x = cc.min_x;
                cc.y = cc.min_y;
                components.push_back(cc);
            }
        }
    }

    return components;
}

// 计算区域的平均灰度
inline double compute_region_mean(const ImageData& img, const ConnectedComponent& cc) {
    if (img.empty()) return 0.0;

    double sum = 0;
    int count = 0;

    for (int y = cc.min_y; y <= cc.max_y; ++y) {
        for (int x = cc.min_x; x <= cc.max_x; ++x) {
            if (x >= 0 && x < static_cast<int>(img.width) &&
                y >= 0 && y < static_cast<int>(img.height)) {
                sum += img.data[y * img.width + x];
                count++;
            }
        }
    }

    return count > 0 ? sum / count : 0.0;
}

// 阈值分割
inline void threshold_binary(const ImageData& input, std::vector<uint8_t>& output,
                             int low_thresh, int high_thresh = 255) {
    if (input.empty()) return;

    output.resize(input.width * input.height);
    for (size_t i = 0; i < input.data.size(); ++i) {
        output[i] = (input.data[i] >= low_thresh && input.data[i] <= high_thresh) ? 255 : 0;
    }
}

// 形态学膨胀
inline void dilate(std::vector<uint8_t>& binary, uint32_t width, uint32_t height, int kernel_size = 3) {
    std::vector<uint8_t> temp = binary;
    int half = kernel_size / 2;

    for (uint32_t y = half; y < height - half; ++y) {
        for (uint32_t x = half; x < width - half; ++x) {
            bool has_white = false;
            for (int dy = -half; dy <= half && !has_white; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    if (temp[(y + dy) * width + (x + dx)] > 0) {
                        has_white = true;
                        break;
                    }
                }
            }
            binary[y * width + x] = has_white ? 255 : 0;
        }
    }
}

// 形态学腐蚀
inline void erode(std::vector<uint8_t>& binary, uint32_t width, uint32_t height, int kernel_size = 3) {
    std::vector<uint8_t> temp = binary;
    int half = kernel_size / 2;

    for (uint32_t y = half; y < height - half; ++y) {
        for (uint32_t x = half; x < width - half; ++x) {
            bool all_white = true;
            for (int dy = -half; dy <= half && all_white; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    if (temp[(y + dy) * width + (x + dx)] == 0) {
                        all_white = false;
                        break;
                    }
                }
            }
            binary[y * width + x] = all_white ? 255 : 0;
        }
    }
}

// 计算纹理特征（基于灰度共生矩阵简化版）
inline void compute_texture_features(const ImageData& img, int x, int y, int win_size,
                                     double& contrast, double& homogeneity, double& energy) {
    contrast = 0; homogeneity = 0; energy = 0;

    if (img.empty()) return;

    // 简化的纹理计算：使用梯度分布
    int half = win_size / 2;
    std::vector<int> grad_hist(256, 0);
    int total = 0;

    for (int dy = -half; dy <= half; ++dy) {
        for (int dx = -half; dx <= half; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 1 && nx < static_cast<int>(img.width) - 1 &&
                ny >= 1 && ny < static_cast<int>(img.height) - 1) {
                int gx = img.data[ny * img.width + nx + 1] - img.data[ny * img.width + nx - 1];
                int gy = img.data[(ny + 1) * img.width + nx] - img.data[(ny - 1) * img.width + nx];
                int grad = static_cast<int>(std::sqrt(gx * gx + gy * gy));
                grad = std::min(255, grad);
                grad_hist[grad]++;
                total++;
            }
        }
    }

    if (total == 0) return;

    for (int i = 0; i < 256; ++i) {
        double p = static_cast<double>(grad_hist[i]) / total;
        contrast += i * i * p;
        homogeneity += p / (1.0 + i);
        energy += p * p;
    }
}

// 高斯平滑（简化版）
inline void gaussian_blur(const ImageData& input, ImageData& output, int kernel_size = 3) {
    if (input.empty()) return;

    output.width = input.width;
    output.height = input.height;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data.resize(output.width * output.height);
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id;

    // 简单均值滤波代替高斯
    int half = kernel_size / 2;
    for (uint32_t y = 0; y < input.height; ++y) {
        for (uint32_t x = 0; x < input.width; ++x) {
            double sum = 0;
            int count = 0;
            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < static_cast<int>(input.width) &&
                        ny >= 0 && ny < static_cast<int>(input.height)) {
                        sum += input.data[ny * input.width + nx];
                        count++;
                    }
                }
            }
            output.data[y * input.width + x] = static_cast<uint8_t>(sum / count);
        }
    }
}

} // namespace defect_utils

// ============================================================================
// 表面缺陷检测节点
// ============================================================================

/**
 * @brief 表面缺陷检测节点（通用）
 */
class SurfaceDefectNode : public INode {
public:
    SurfaceDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 划痕检测节点
 */
class ScratchDetectNode : public INode {
public:
    ScratchDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 裂纹检测节点
 */
class CrackDetectNode : public INode {
public:
    CrackDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 污渍检测节点
 */
class StainDetectNode : public INode {
public:
    StainDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 凹坑检测节点
 */
class PitDetectNode : public INode {
public:
    PitDetectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ============================================================================
// 形状缺陷检测节点
// ============================================================================

/**
 * @brief 形状缺陷检测节点
 */
class ShapeDefectNode : public INode {
public:
    ShapeDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 尺寸缺陷检测节点
 */
class DimensionDefectNode : public INode {
public:
    DimensionDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 位置偏差检测节点
 */
class PositionDefectNode : public INode {
public:
    PositionDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 角度偏差检测节点
 */
class AngleDefectNode : public INode {
public:
    AngleDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ============================================================================
// 外观缺陷检测节点
// ============================================================================

/**
 * @brief 颜色缺陷检测节点
 */
class ColorDefectNode : public INode {
public:
    ColorDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 纹理缺陷检测节点
 */
class TextureDefectNode : public INode {
public:
    TextureDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 光泽缺陷检测节点
 */
class GlossDefectNode : public INode {
public:
    GlossDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 透明度缺陷检测节点
 */
class OpacityDefectNode : public INode {
public:
    OpacityDefectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 异物检测节点
 */
class ForeignObjectNode : public INode {
public:
    ForeignObjectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief 缺失检测节点
 */
class MissingObjectNode : public INode {
public:
    MissingObjectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf