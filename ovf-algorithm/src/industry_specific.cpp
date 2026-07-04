/**
 * @file industry_specific.cpp
 * @brief 行业专用算子模块实现
 */

#include "ovf/algorithm/industry_specific.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/morphology.h"
#include "ovf/algorithm/image_utils.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace ovf {
namespace algorithm {

// ========== 辅助函数 ==========

namespace {

// 获取灰度像素值
inline uint8_t get_gray_pixel(const ImageData& img, int x, int y) {
    if (x < 0 || x >= static_cast<int>(img.width) ||
        y < 0 || y >= static_cast<int>(img.height)) {
        return 0;
    }

    size_t idx = static_cast<size_t>(y * img.width + x);
    if (img.channels == 1) {
        return img.data[idx];
    } else {
        size_t pixel_idx = idx * img.channels;
        return static_cast<uint8_t>(
            (img.data[pixel_idx] + img.data[pixel_idx + 1] + img.data[pixel_idx + 2]) / 3);
    }
}

// 转换为灰度图像
void to_gray_image(const ImageData& input, Vector<uint8_t>& gray, int& w, int& h) {
    w = input.width;
    h = input.height;
    gray.resize(w * h);

    for (int i = 0; i < w * h; ++i) {
        if (input.channels == 1) {
            gray[i] = input.data[i];
        } else {
            int idx = i * input.channels;
            gray[i] = static_cast<uint8_t>(
                (input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }
}

// 创建输出灰度图像
ImageData create_gray_output(int w, int h, const Vector<uint8_t>& data) {
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = data;
    return output;
}

// 计算局部均值
float compute_local_mean(const Vector<uint8_t>& gray, int w, int h, int x, int y, int radius) {
    float sum = 0.0f;
    int count = 0;

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                sum += gray[ny * w + nx];
                count++;
            }
        }
    }

    return count > 0 ? sum / count : 0.0f;
}

// 计算局部标准差
float compute_local_std(const Vector<uint8_t>& gray, int w, int h, int x, int y, int radius, float mean) {
    float sum = 0.0f;
    int count = 0;

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                float diff = gray[ny * w + nx] - mean;
                sum += diff * diff;
                count++;
            }
        }
    }

    return count > 0 ? std::sqrt(sum / count) : 0.0f;
}

// 高斯滤波
void gaussian_filter(const Vector<uint8_t>& src, Vector<uint8_t>& dst, int w, int h, int radius, float sigma) {
    dst.resize(w * h);

    Vector<float> kernel((2 * radius + 1) * (2 * radius + 1));
    float sum = 0.0f;

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            float val = std::exp(-(dx * dx + dy * dy) / (2 * sigma * sigma));
            kernel[(dy + radius) * (2 * radius + 1) + (dx + radius)] = val;
            sum += val;
        }
    }

    for (size_t i = 0; i < kernel.size(); ++i) {
        kernel[i] /= sum;
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float acc = 0.0f;

            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    int nx = std::clamp(x + dx, 0, w - 1);
                    int ny = std::clamp(y + dy, 0, h - 1);
                    acc += src[ny * w + nx] * kernel[(dy + radius) * (2 * radius + 1) + (dx + radius)];
                }
            }

            dst[y * w + x] = static_cast<uint8_t>(std::clamp(acc, 0.0f, 255.0f));
        }
    }
}

// Sobel边缘检测
void sobel_edge(const Vector<uint8_t>& src, Vector<uint8_t>& dst, int w, int h, int threshold) {
    dst.resize(w * h);
    std::fill(dst.begin(), dst.end(), 0);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            int gx = -src[(y-1)*w + (x-1)] + src[(y-1)*w + (x+1)]
                     -2*src[y*w + (x-1)] + 2*src[y*w + (x+1)]
                     -src[(y+1)*w + (x-1)] + src[(y+1)*w + (x+1)];

            int gy = -src[(y-1)*w + (x-1)] - 2*src[(y-1)*w + x] - src[(y-1)*w + (x+1)]
                     +src[(y+1)*w + (x-1)] + 2*src[(y+1)*w + x] + src[(y+1)*w + (x+1)];

            int magnitude = static_cast<int>(std::sqrt(gx * gx + gy * gy));
            dst[y * w + x] = (magnitude > threshold) ? 255 : 0;
        }
    }
}

// 连通区域标记
struct ConnectedComponent {
    int x, y, width, height;
    int area;
    int min_x, min_y, max_x, max_y;
};

void flood_fill(Vector<uint8_t>& binary, int w, int h, int x, int y, uint8_t old_val, uint8_t new_val, ConnectedComponent& comp) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    if (binary[y * w + x] != old_val) return;

    Vector<std::pair<int, int>> stack;
    stack.push_back({x, y});

    comp.min_x = x; comp.max_x = x;
    comp.min_y = y; comp.max_y = y;
    comp.area = 0;

    while (!stack.empty()) {
        auto [cx, cy] = stack.back();
        stack.pop_back();

        if (cx < 0 || cx >= w || cy < 0 || cy >= h) continue;
        if (binary[cy * w + cx] != old_val) continue;

        binary[cy * w + cx] = new_val;
        comp.area++;
        comp.min_x = std::min(comp.min_x, cx);
        comp.max_x = std::max(comp.max_x, cx);
        comp.min_y = std::min(comp.min_y, cy);
        comp.max_y = std::max(comp.max_y, cy);

        stack.push_back({cx + 1, cy});
        stack.push_back({cx - 1, cy});
        stack.push_back({cx, cy + 1});
        stack.push_back({cx, cy - 1});
    }

    comp.x = comp.min_x;
    comp.y = comp.min_y;
    comp.width = comp.max_x - comp.min_x + 1;
    comp.height = comp.max_y - comp.min_y + 1;
}

Vector<ConnectedComponent> find_connected_components(Vector<uint8_t>& binary, int w, int h) {
    Vector<ConnectedComponent> components;
    uint8_t label = 128;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (binary[y * w + x] == 255) {
                ConnectedComponent comp;
                flood_fill(binary, w, h, x, y, 255, label, comp);
                components.push_back(comp);
                label++;
                if (label >= 200) label = 128;
            }
        }
    }

    for (size_t i = 0; i < binary.size(); ++i) {
        if (binary[i] >= 128) binary[i] = 255;
    }

    return components;
}

// 计算图像均值
float compute_image_mean(const Vector<uint8_t>& gray) {
    if (gray.empty()) return 0.0f;
    float sum = std::accumulate(gray.begin(), gray.end(), 0.0f);
    return sum / gray.size();
}

// 计算图像标准差
float compute_image_std(const Vector<uint8_t>& gray, float mean) {
    if (gray.empty()) return 0.0f;
    float sum = 0.0f;
    for (uint8_t val : gray) {
        float diff = val - mean;
        sum += diff * diff;
    }
    return std::sqrt(sum / gray.size());
}

// 绘制矩形标记
void draw_rect_marker(Vector<uint8_t>& marked, int w, int h, int x, int y, int width, int height) {
    for (int px = x; px < x + width && px < w; ++px) {
        if (y >= 0 && y < h) marked[y * w + px] = 255;
        if (y + height - 1 >= 0 && y + height - 1 < h) marked[(y + height - 1) * w + px] = 255;
    }
    for (int py = y; py < y + height && py < h; ++py) {
        if (x >= 0 && x < w) marked[py * w + x] = 255;
        if (x + width - 1 >= 0 && x + width - 1 < w) marked[py * w + (x + width - 1)] = 255;
    }
}

// 计算矩形填充率
float compute_fill_ratio(const Vector<uint8_t>& gray, int w, int h, int x, int y, int width, int height) {
    int count = 0;
    int total = 0;

    for (int py = y; py < y + height && py < h; ++py) {
        for (int px = x; px < x + width && px < w; ++px) {
            if (gray[py * w + px] > 128) count++;
            total++;
        }
    }

    return total > 0 ? static_cast<float>(count) / total : 0.0f;
}

// 计算区域圆度
float compute_roundness(const Vector<uint8_t>& binary, int w, int h, const ConnectedComponent& comp) {
    // 基于面积和外接圆近似计算圆度
    float area = static_cast<float>(comp.area);
    float perimeter_estimate = 2 * (comp.width + comp.height);
    float roundness = (4 * 3.14159f * area) / (perimeter_estimate * perimeter_estimate);
    return std::clamp(roundness, 0.0f, 1.0f);
}

// 检测线状结构（用于线路检测）
bool is_line_like(const ConnectedComponent& comp, float threshold = 0.3f) {
    float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
    return aspect_ratio > 3.0f || aspect_ratio < 0.33f;
}

// 检测是否为元器件形状
bool is_component_shape(const ConnectedComponent& comp) {
    float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
    float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height + 1);

    // 元器件通常是矩形，长宽比接近1，填充率高
    return aspect_ratio > 0.5f && aspect_ratio < 2.0f && area_ratio > 0.5f;
}

// 自适应阈值分割
void adaptive_threshold(const Vector<uint8_t>& gray, Vector<uint8_t>& binary, int w, int h, int block_size = 31, int offset = 10) {
    binary.resize(w * h);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float local_mean = compute_local_mean(gray, w, h, x, y, block_size / 2);
            float threshold = local_mean - offset;
            binary[y * w + x] = (gray[y * w + x] > threshold) ? 255 : 0;
        }
    }
}

} // anonymous namespace

// ========== PCB检测模块实现 ==========

namespace pcb_utils {

ErrorCode detect_components(const ImageData& image, Vector<ComponentInfo>& components,
                           float min_confidence) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 自适应阈值分割
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    // 形态学处理去除噪声
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(binary.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    // 连通区域分析
    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    components.clear();
    for (const auto& comp : comps) {
        // 过滤尺寸太小的区域
        if (comp.width < 10 || comp.height < 10 || comp.area < 50) continue;

        // 判断是否为元器件形状
        if (!is_component_shape(comp)) continue;

        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height + 1);

        // 计算置信度
        float confidence = std::clamp(area_ratio * 0.8f + (1 - std::abs(aspect_ratio - 1) * 0.2f), 0.0f, 1.0f);

        if (confidence >= min_confidence) {
            ComponentInfo component;

            // 根据形状推断元器件类型
            if (aspect_ratio > 1.5f && aspect_ratio < 3.0f) {
                component.type = ComponentType::Resistor;
            } else if (aspect_ratio < 0.7f && comp.width < 30) {
                component.type = ComponentType::Capacitor;
            } else if (comp.area > 200 && aspect_ratio > 0.8f && aspect_ratio < 1.2f) {
                component.type = ComponentType::IC;
            } else if (aspect_ratio > 2.0f) {
                component.type = ComponentType::Connector;
            } else if (comp.area < 100 && aspect_ratio > 0.8f && aspect_ratio < 1.2f) {
                component.type = ComponentType::Diode;
            } else {
                component.type = ComponentType::Unknown;
            }

            component.x = comp.x;
            component.y = comp.y;
            component.width = comp.width;
            component.height = comp.height;
            component.confidence = confidence;
            component.description = component_type_name(component.type) +
                                    " @(" + std::to_string(component.x) + "," +
                                    std::to_string(component.y) + ") " +
                                    std::to_string(component.width) + "x" +
                                    std::to_string(component.height);

            components.push_back(component);
        }
    }

    OVF_INFO() << "PCB component detection: found " << components.size() << " components";
    return ErrorCode::Success;
}

ErrorCode detect_traces(const ImageData& image, Vector<TraceInfo>& traces,
                       int min_width, float thickness_threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 边缘检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);

    // 形态学处理连接线段
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> connected(w * h);
    morph_utils::close(edge.data(), connected.data(), w, h, kernel.data(), 3, 3, 2);

    // 连通区域分析
    Vector<uint8_t> temp = connected;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    traces.clear();
    for (const auto& comp : comps) {
        // 过滤尺寸
        if (comp.width < min_width || comp.height < min_width) continue;

        // 判断是否为线路形状（长条形）
        if (!is_line_like(comp)) continue;

        TraceInfo trace;
        trace.x = comp.x;
        trace.y = comp.y;
        trace.width = comp.width;
        trace.height = comp.height;

        // 计算线路厚度（平均宽度）
        float thickness = static_cast<float>(comp.area) / (std::max(comp.width, comp.height) + 1);
        trace.thickness = thickness;

        // 检测断路（线路中间有断裂）
        // 检测短路（相邻线路连接）
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        if (thickness < thickness_threshold * (std::max(comp.width, comp.height) / 10.0f)) {
            trace.is_open = true;
            trace.description = "潜在断路 厚度=" + std::to_string(thickness);
        } else if (comp.area > (comp.width * comp.height) * 0.8f && aspect_ratio < 1.5f) {
            trace.is_short = true;
            trace.description = "潜在短路 区域=" + std::to_string(comp.area);
        } else {
            trace.description = "正常线路 厚度=" + std::to_string(thickness);
        }

        traces.push_back(trace);
    }

    OVF_INFO() << "PCB trace detection: found " << traces.size() << " traces";
    return ErrorCode::Success;
}

ErrorCode detect_pads(const ImageData& image, Vector<PadInfo>& pads,
                     float min_roundness, float min_coverage) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 高亮区域检测（焊盘通常是明亮区域）
    Vector<uint8_t> bright(w * h, 0);
    float global_mean = compute_image_mean(gray);
    float threshold = global_mean + 30;

    for (int i = 0; i < w * h; ++i) {
        if (gray[i] > threshold) {
            bright[i] = 255;
        }
    }

    // 形态学处理
    Vector<uint8_t> kernel(25, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(bright.data(), cleaned.data(), w, h, kernel.data(), 5, 5, 1);

    // 连通区域分析
    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    pads.clear();
    for (const auto& comp : comps) {
        // 过滤尺寸（焊盘通常在10-100像素范围）
        if (comp.width < 5 || comp.height < 5 || comp.width > 100 || comp.height > 100) continue;
        if (comp.area < 20) continue;

        PadInfo pad;
        pad.x = comp.x;
        pad.y = comp.y;
        pad.width = comp.width;
        pad.height = comp.height;

        // 计算圆度
        pad.roundness = compute_roundness(cleaned, w, h, comp);

        // 计算锡覆盖率（基于亮度均匀性）
        float coverage = static_cast<float>(comp.area) / (comp.width * comp.height + 1);
        pad.coverage = std::clamp(coverage, 0.0f, 1.0f);

        // 检测缺陷
        if (pad.roundness < min_roundness) {
            pad.is_defective = true;
            pad.defect_type = "形状不规则";
        } else if (pad.coverage < min_coverage) {
            pad.is_defective = true;
            pad.defect_type = "锡覆盖不足";
        }

        pad.description = "焊盘 @(" + std::to_string(pad.x) + "," +
                          std::to_string(pad.y) + ") 圆度=" +
                          std::to_string(pad.roundness) + " 覆盖率=" +
                          std::to_string(pad.coverage);

        if (pad.is_defective) {
            pad.description += " [缺陷: " + pad.defect_type + "]";
        }

        pads.push_back(pad);
    }

    OVF_INFO() << "PCB pad detection: found " << pads.size() << " pads";
    return ErrorCode::Success;
}

ErrorCode detect_pcb_defects(const ImageData& image, Vector<PCBDefectInfo>& defects,
                            float sensitivity) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    defects.clear();

    // 1. 元器件缺失检测 - 与模板对比或检测空白区域
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 20);

    // 检测可能的元器件缺失位置（应该有元器件但现在为空白）
    // 使用形态学底帽检测暗区域
    Vector<uint8_t> kernel(25, 255);
    Vector<uint8_t> closed(w * h);
    morph_utils::close(binary.data(), closed.data(), w, h, kernel.data(), 5, 5, 1);

    Vector<uint8_t> black_hat(w * h);
    for (int i = 0; i < w * h; ++i) {
        int diff = closed[i] - binary[i];
        black_hat[i] = static_cast<uint8_t>(std::clamp(diff, 0, 255));
    }

    Vector<uint8_t> temp = black_hat;
    Vector<ConnectedComponent> missing_comps = find_connected_components(temp, w, h);

    float threshold = sensitivity * 50;
    for (const auto& comp : missing_comps) {
        if (comp.area > threshold && comp.width > 10 && comp.height > 10) {
            PCBDefectInfo defect;
            defect.type = PCBDefectType::MissingComponent;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 200.0f, 0.0f, 1.0f);
            defect.confidence = 0.7f;
            defect.description = pcb_defect_type_name(defect.type) +
                                " @(" + std::to_string(defect.x) + "," +
                                std::to_string(defect.y) + ")";
            defects.push_back(defect);
        }
    }

    // 2. 焊桥检测 - 相邻焊盘之间的连接
    Vector<uint8_t> bright(w * h, 0);
    float global_mean = compute_image_mean(gray);
    for (int i = 0; i < w * h; ++i) {
        if (gray[i] > global_mean + 40) {
            bright[i] = 255;
        }
    }

    Vector<uint8_t> dilated(w * h);
    morph_utils::dilate(bright.data(), dilated.data(), w, h, kernel.data(), 5, 5, 1);

    Vector<uint8_t> temp2 = dilated;
    Vector<ConnectedComponent> bridge_comps = find_connected_components(temp2, w, h);

    for (const auto& comp : bridge_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        // 焊桥通常是连接两个焊盘的细长区域
        if (aspect_ratio > 1.5f && aspect_ratio < 5.0f && comp.area > 30) {
            PCBDefectInfo defect;
            defect.type = PCBDefectType::SolderBridge;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            defect.confidence = 0.8f;
            defect.description = pcb_defect_type_name(defect.type) +
                                " @(" + std::to_string(defect.x) + "," +
                                std::to_string(defect.y) + ")";
            defects.push_back(defect);
        }
    }

    // 3. 划痕检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 20);

    Vector<uint8_t> temp3 = edge;
    Vector<ConnectedComponent> scratch_comps = find_connected_components(temp3, w, h);

    for (const auto& comp : scratch_comps) {
        if (is_line_like(comp) && comp.area > 20) {
            PCBDefectInfo defect;
            defect.type = PCBDefectType::Scratch;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            int length = std::max(comp.width, comp.height);
            defect.severity = std::clamp(static_cast<float>(length) / 100.0f, 0.0f, 1.0f);
            defect.confidence = 0.75f;
            defect.description = pcb_defect_type_name(defect.type) +
                                " 长度=" + std::to_string(length);
            defects.push_back(defect);
        }
    }

    // 4. 污染检测 - 检测低对比度区域
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 5, 2.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    Vector<uint8_t> contamination(w * h, 0);
    float contamination_threshold = sensitivity * 20;
    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > contamination_threshold && diff[i] < 50) {
            contamination[i] = 255;
        }
    }

    Vector<uint8_t> temp4 = contamination;
    Vector<ConnectedComponent> contam_comps = find_connected_components(temp4, w, h);

    for (const auto& comp : contam_comps) {
        if (comp.area > 30 && !is_line_like(comp)) {
            PCBDefectInfo defect;
            defect.type = PCBDefectType::Contamination;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 150.0f, 0.0f, 1.0f);
            defect.confidence = 0.6f;
            defect.description = pcb_defect_type_name(defect.type) +
                                " 面积=" + std::to_string(comp.area);
            defects.push_back(defect);
        }
    }

    OVF_INFO() << "PCB defect detection: found " << defects.size() << " defects";
    return ErrorCode::Success;
}

} // namespace pcb_utils

// ========== 半导体检测模块实现 ==========

namespace wafer_utils {

ErrorCode detect_dies(const ImageData& image, Vector<DieInfo>& dies,
                     int die_width, int die_height) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    dies.clear();

    // 基于网格划分检测芯片
    int cols = w / die_width;
    int rows = h / die_height;

    float global_mean = compute_image_mean(gray);
    float global_std = compute_image_std(gray, global_mean);

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            int x = col * die_width;
            int y = row * die_height;

            // 计算该区域的统计特征
            float die_mean = compute_local_mean(gray, w, h, x + die_width/2, y + die_height/2, die_width/3);
            float die_std = compute_local_std(gray, w, h, x + die_width/2, y + die_height/2, die_width/3, die_mean);

            DieInfo die;
            die.row = row;
            die.col = col;
            die.x = x;
            die.y = y;
            die.width = die_width;
            die.height = die_height;

            // 根据特征判断状态
            float mean_diff = std::abs(die_mean - global_mean);
            float std_ratio = die_std / (global_std + 1);

            if (mean_diff < 10 && std_ratio > 0.8f && std_ratio < 1.2f) {
                die.status = DieStatus::Good;
                die.confidence = 0.9f;
            } else if (mean_diff > 30 || std_ratio < 0.3f) {
                die.status = DieStatus::Defective;
                die.confidence = 0.85f;
            } else if (die_mean < global_mean - 50) {
                die.status = DieStatus::Missing;
                die.confidence = 0.8f;
            } else if (mean_diff > 20 && mean_diff < 30) {
                die.status = DieStatus::Misaligned;
                die.confidence = 0.7f;
            } else {
                die.status = DieStatus::Unknown;
                die.confidence = 0.5f;
            }

            die.description = die_status_name(die.status) +
                             " 行=" + std::to_string(row) +
                             " 列=" + std::to_string(col);

            dies.push_back(die);
        }
    }

    OVF_INFO() << "Wafer die detection: found " << dies.size() << " dies";
    return ErrorCode::Success;
}

ErrorCode detect_wafer_defects(const ImageData& image, Vector<WaferDefectInfo>& defects,
                              float sensitivity) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    defects.clear();

    // 1. 颗粒污染检测 - 高对比度斑点
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 7, 3.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    Vector<uint8_t> particle(w * h, 0);
    float particle_threshold = sensitivity * 30;
    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > particle_threshold) {
            particle[i] = 255;
        }
    }

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(particle.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> particle_comps = find_connected_components(temp, w, h);

    for (const auto& comp : particle_comps) {
        if (comp.area >= 5 && comp.area < 200) {
            WaferDefectInfo defect;
            defect.type = WaferDefectType::Particle;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            defect.confidence = 0.85f;
            defect.description = wafer_defect_type_name(defect.type) +
                                " 面积=" + std::to_string(comp.area);
            defects.push_back(defect);
        }
    }

    // 2. 划痕检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    Vector<uint8_t> temp2 = edge;
    Vector<ConnectedComponent> scratch_comps = find_connected_components(temp2, w, h);

    for (const auto& comp : scratch_comps) {
        if (is_line_like(comp) && comp.area > 15) {
            WaferDefectInfo defect;
            defect.type = WaferDefectType::Scratch;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            int length = std::max(comp.width, comp.height);
            defect.severity = std::clamp(static_cast<float>(length) / 150.0f, 0.0f, 1.0f);
            defect.confidence = 0.8f;
            defect.description = wafer_defect_type_name(defect.type) +
                                " 长度=" + std::to_string(length);
            defects.push_back(defect);
        }
    }

    // 3. 边缘缺陷检测
    // 检测晶圆边缘区域的异常
    int edge_margin = 20;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < edge_margin || x >= w - edge_margin; ++x) {
            if (x >= w) break;

            float local_mean = compute_local_mean(gray, w, h, x, y, 5);
            float global_mean = compute_image_mean(gray);

            if (std::abs(local_mean - global_mean) > sensitivity * 40) {
                WaferDefectInfo defect;
                defect.type = WaferDefectType::EdgeDefect;
                defect.x = x;
                defect.y = y;
                defect.width = 10;
                defect.height = 10;
                defect.severity = std::clamp(std::abs(local_mean - global_mean) / 100.0f, 0.0f, 1.0f);
                defect.confidence = 0.7f;
                defect.description = wafer_defect_type_name(defect.type) +
                                    " @边缘位置";
                defects.push_back(defect);
                break; // 避免重复检测
            }
        }
    }

    // 4. 图形缺陷检测 - 检测周期性结构的异常
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 21, 10);

    Vector<uint8_t> temp3 = binary;
    Vector<ConnectedComponent> pattern_comps = find_connected_components(temp3, w, h);

    for (const auto& comp : pattern_comps) {
        float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height + 1);
        // 图形缺陷通常是不规则形状
        if (area_ratio < 0.5f && comp.area > 20 && !is_line_like(comp)) {
            WaferDefectInfo defect;
            defect.type = WaferDefectType::PatternDefect;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp((1 - area_ratio), 0.0f, 1.0f);
            defect.confidence = 0.75f;
            defect.description = wafer_defect_type_name(defect.type) +
                                " 形状异常";
            defects.push_back(defect);
        }
    }

    OVF_INFO() << "Wafer defect detection: found " << defects.size() << " defects";
    return ErrorCode::Success;
}

ErrorCode check_die_alignment(const ImageData& image, AlignmentInfo& alignment,
                             float tolerance_x, float tolerance_y) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 计算中心区域特征作为参考
    int cx = w / 2;
    int cy = h / 2;
    int ref_radius = std::min(w, h) / 4;

    float center_mean = compute_local_mean(gray, w, h, cx, cy, ref_radius);

    // 检测对准标记（通常在四角或边缘）
    // 这里使用简化方法：检测四角区域的特征差异

    // 左上角
    float corner_tl = compute_local_mean(gray, w, h, w/4, h/4, 20);
    // 右上角
    float corner_tr = compute_local_mean(gray, w, h, 3*w/4, h/4, 20);
    // 左下角
    float corner_bl = compute_local_mean(gray, w, h, w/4, 3*h/4, 20);
    // 右下角
    float corner_br = compute_local_mean(gray, w, h, 3*w/4, 3*h/4, 20);

    // 计算偏移量（基于四角对称性）
    float horizontal_diff = (corner_tr + corner_br) / 2 - (corner_tl + corner_bl) / 2;
    float vertical_diff = (corner_bl + corner_br) / 2 - (corner_tl + corner_tr) / 2;

    // 检测旋转角度（基于特征点）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);

    // 计算主方向（简化）
    float angle_sum = 0.0f;
    int angle_count = 0;

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            if (edge[y * w + x] > 0) {
                int gx = -gray[(y-1)*w + (x-1)] + gray[(y-1)*w + (x+1)]
                         -2*gray[y*w + (x-1)] + 2*gray[y*w + (x+1)]
                         -gray[(y+1)*w + (x-1)] + gray[(y+1)*w + (x+1)];

                int gy = -gray[(y-1)*w + (x-1)] - 2*gray[(y-1)*w + x] - gray[(y-1)*w + (x+1)]
                         +gray[(y+1)*w + (x-1)] + 2*gray[(y+1)*w + x] + gray[(y+1)*w + (x+1)];

                if (gx != 0 || gy != 0) {
                    float angle = std::atan2(static_cast<float>(gy), static_cast<float>(gx)) * 180.0f / 3.14159f;
                    angle_sum += angle;
                    angle_count++;
                }
            }
        }
    }

    alignment.offset_x = horizontal_diff * 0.1f; // 缩放为像素偏移
    alignment.offset_y = vertical_diff * 0.1f;
    alignment.rotation = (angle_count > 0) ? (angle_sum / angle_count - 45.0f) : 0.0f; // 参考角度45度
    alignment.scale_diff = std::abs(corner_tl - corner_br) / (center_mean + 1) * 0.01f;
    alignment.tolerance_x = tolerance_x;
    alignment.tolerance_y = tolerance_y;

    // 判断是否对准
    alignment.is_aligned = (std::abs(alignment.offset_x) < tolerance_x &&
                           std::abs(alignment.offset_y) < tolerance_y &&
                           std::abs(alignment.rotation) < alignment.tolerance_angle);

    OVF_INFO() << "Die alignment check: offset_x=" << alignment.offset_x
              << ", offset_y=" << alignment.offset_y
              << ", rotation=" << alignment.rotation
              << ", is_aligned=" << alignment.is_aligned;

    return ErrorCode::Success;
}

} // namespace wafer_utils

// ========== 包装检测模块实现 ==========

namespace package_utils {

ErrorCode detect_seal(const ImageData& image, SealInfo& seal,
                     float min_strength) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 检测密封线（通常是边缘区域的连续线条）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 20);

    // 计算边缘密度（密封强度指标）
    int edge_count = 0;
    int perimeter_estimate = 2 * (w + h);

    // 检测边缘连续性
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (edge[y * w + x] > 0) {
                edge_count++;
            }
        }
    }

    // 检测密封区域（边缘带）
    int seal_band_width = std::min(w, h) / 10;
    int seal_edge_count = 0;

    for (int y = 0; y < seal_band_width; ++y) {
        for (int x = 0; x < w; ++x) {
            if (edge[y * w + x] > 0) seal_edge_count++;
        }
    }
    for (int y = h - seal_band_width; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (edge[y * w + x] > 0) seal_edge_count++;
        }
    }
    for (int x = 0; x < seal_band_width; ++x) {
        for (int y = seal_band_width; y < h - seal_band_width; ++y) {
            if (edge[y * w + x] > 0) seal_edge_count++;
        }
    }
    for (int x = w - seal_band_width; x < w; ++x) {
        for (int y = seal_band_width; y < h - seal_band_width; ++y) {
            if (edge[y * w + x] > 0) seal_edge_count++;
        }
    }

    float seal_strength = static_cast<float>(seal_edge_count) / (2 * seal_band_width * (w + h) + 1);
    seal_strength = std::clamp(seal_strength, 0.0f, 1.0f);

    seal.x = 0;
    seal.y = 0;
    seal.width = w;
    seal.height = h;
    seal.seal_strength = seal_strength;

    // 判断密封状态
    if (seal_strength >= min_strength) {
        seal.status = SealStatus::Good;
        seal.confidence = 0.9f;
    } else if (seal_strength < min_strength * 0.3f) {
        seal.status = SealStatus::Broken;
        seal.confidence = 0.85f;
    } else if (seal_strength < min_strength * 0.7f) {
        seal.status = SealStatus::Incomplete;
        seal.confidence = 0.8f;
    } else {
        seal.status = SealStatus::Unknown;
        seal.confidence = 0.5f;
    }

    // 检测密封线是否有断裂
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> closed(w * h);
    morph_utils::close(edge.data(), closed.data(), w, h, kernel.data(), 3, 3, 2);

    int break_count = 0;
    for (int y = 0; y < seal_band_width; ++y) {
        bool has_edge = false;
        for (int x = 0; x < w; ++x) {
            if (closed[y * w + x] > 0) {
                has_edge = true;
                break;
            }
        }
        if (!has_edge) break_count++;
    }

    if (break_count > seal_band_width / 4) {
        seal.status = SealStatus::Broken;
        seal.confidence = 0.9f;
    }

    seal.description = seal_status_name(seal.status) +
                      " 强度=" + std::to_string(seal_strength);

    OVF_INFO() << "Package seal detection: status=" << seal_status_name(seal.status)
              << ", strength=" << seal_strength;

    return ErrorCode::Success;
}

ErrorCode detect_label(const ImageData& image, LabelInfo& label,
                      float min_clarity, float min_alignment) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 检测标签区域（通常是中心矩形区域）
    int label_x = w / 4;
    int label_y = h / 4;
    int label_w = w / 2;
    int label_h = h / 2;

    // 计算标签区域的对比度（清晰度指标）
    float label_mean = compute_local_mean(gray, w, h, label_x + label_w/2, label_y + label_h/2, label_w/3);
    float label_std = compute_local_std(gray, w, h, label_x + label_w/2, label_y + label_h/2, label_w/3, label_mean);

    float clarity = std::clamp(label_std / 50.0f, 0.0f, 1.0f);

    // 检测标签是否存在（基于区域特征）
    float global_mean = compute_image_mean(gray);
    float contrast_with_background = std::abs(label_mean - global_mean) / 255.0f;

    label.x = label_x;
    label.y = label_y;
    label.width = label_w;
    label.height = label_h;
    label.clarity = clarity;

    // 检测标签是否可读（基于文字特征）
    Vector<uint8_t> label_region(label_w * label_h);
    for (int y = label_y; y < label_y + label_h && y < h; ++y) {
        for (int x = label_x; x < label_x + label_w && x < w; ++x) {
            label_region[(y - label_y) * label_w + (x - label_x)] = gray[y * w + x];
        }
    }

    // 检测标签区域内的边缘（文字边缘）
    Vector<uint8_t> label_edge;
    sobel_edge(label_region, label_edge, label_w, label_h, 15);

    int label_edge_count = 0;
    for (int i = 0; i < label_w * label_h; ++i) {
        if (label_edge[i] > 0) label_edge_count++;
    }

    float text_density = static_cast<float>(label_edge_count) / (label_w * label_h + 1);

    // 判断标签状态
    label.is_present = (contrast_with_background > 0.1f && text_density > 0.05f);
    label.is_readable = (clarity >= min_clarity && text_density > 0.03f);

    // 检测对齐（标签是否居中）
    float center_offset_x = std::abs(label_x + label_w/2 - w/2) / static_cast<float>(w);
    float center_offset_y = std::abs(label_y + label_h/2 - h/2) / static_cast<float>(h);
    label.alignment_score = 1.0f - std::clamp((center_offset_x + center_offset_y) / 2, 0.0f, 1.0f);

    label.is_correct = (label.alignment_score >= min_alignment);

    label.confidence = std::clamp(contrast_with_background * 0.5f + clarity * 0.3f + label.alignment_score * 0.2f, 0.0f, 1.0f);

    label.description = "标签检测: 存在=" + String(label.is_present ? "是" : "否") +
                       ", 可读=" + String(label.is_readable ? "是" : "否") +
                       ", 对齐=" + std::to_string(label.alignment_score);

    OVF_INFO() << "Package label detection: present=" << label.is_present
              << ", readable=" << label.is_readable
              << ", clarity=" << label.clarity;

    return ErrorCode::Success;
}

ErrorCode detect_package_quality(const ImageData& image, PackageQualityInfo& quality,
                                float pass_threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    quality.x = 0;
    quality.y = 0;
    quality.width = w;
    quality.height = h;
    quality.issues.clear();

    // 计算整体质量指标
    float global_mean = compute_image_mean(gray);
    float global_std = compute_image_std(gray, global_mean);

    // 1. 检测损坏（高对比度异常）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 40);

    int damage_count = 0;
    for (int i = 0; i < w * h; ++i) {
        if (edge[i] > 0) damage_count++;
    }

    float damage_ratio = static_cast<float>(damage_count) / (w * h + 1);
    if (damage_ratio > 0.15f) {
        quality.issues.push_back(PackageIssueType::Damaged);
    }

    // 2. 检测变形（形状不规则）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 20);

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> opened(w * h);
    morph_utils::open(binary.data(), opened.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = opened;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    for (const auto& comp : comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        if (aspect_ratio > 2.5f || aspect_ratio < 0.4f) {
            if (comp.area > w * h * 0.3f) {
                quality.issues.push_back(PackageIssueType::Deformed);
                break;
            }
        }
    }

    // 3. 检测污染（低对比度斑点）
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 10, 5.0f);

    int dirty_count = 0;
    for (int i = 0; i < w * h; ++i) {
        if (gray[i] < smooth[i] - 20 || gray[i] > smooth[i] + 20) {
            dirty_count++;
        }
    }

    float dirty_ratio = static_cast<float>(dirty_count) / (w * h + 1);
    if (dirty_ratio > 0.1f) {
        quality.issues.push_back(PackageIssueType::Dirty);
    }

    // 4. 检测受潮（整体亮度降低）
    if (global_mean < 80) {
        quality.issues.push_back(PackageIssueType::Wet);
    }

    // 5. 检测颜色问题（颜色不均匀）
    if (global_std < 20) {
        quality.issues.push_back(PackageIssueType::ColorProblem);
    }

    // 计算整体评分
    float damage_score = 1.0f - std::clamp(damage_ratio * 5, 0.0f, 1.0f);
    float dirty_score = 1.0f - std::clamp(dirty_ratio * 3, 0.0f, 1.0f);
    float brightness_score = global_mean / 128.0f;
    float contrast_score = global_std / 50.0f;

    quality.overall_score = (damage_score * 0.3f + dirty_score * 0.2f +
                            std::clamp(brightness_score, 0.0f, 1.0f) * 0.2f +
                            std::clamp(contrast_score, 0.0f, 1.0f) * 0.3f);

    quality.defect_score = 1.0f - quality.overall_score;
    quality.is_pass = (quality.overall_score >= pass_threshold && quality.issues.empty());

    quality.description = "包装质量: 评分=" + std::to_string(quality.overall_score) +
                        ", 合格=" + (quality.is_pass ? "是" : "否") +
                        ", 问题数=" + std::to_string(quality.issues.size());

    OVF_INFO() << "Package quality detection: score=" << quality.overall_score
              << ", pass=" << quality.is_pass;

    return ErrorCode::Success;
}

} // namespace package_utils

// ========== 通用工业检测模块实现 ==========

namespace industry_utils {

ErrorCode check_assembly(const ImageData& image, AssemblyCheckResult& result,
                        float tolerance) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    result.items.clear();
    result.pass_count = 0;
    result.fail_count = 0;

    // 1. 元器件存在性检查
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(binary.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    for (const auto& comp : comps) {
        if (comp.width >= 20 && comp.height >= 20 && comp.area >= 100) {
            AssemblyCheckItem item;
            item.type = AssemblyItemType::ComponentPresence;
            item.x = comp.x;
            item.y = comp.y;
            item.width = comp.width;
            item.height = comp.height;
            item.is_pass = true;
            item.confidence = 0.9f;
            item.description = assembly_item_type_name(item.type) +
                              " @(" + std::to_string(item.x) + "," +
                              std::to_string(item.y) + ")";
            result.items.push_back(item);
            result.pass_count++;
        }
    }

    // 2. 元器件位置检查
    float global_mean = compute_image_mean(gray);
    float cx = w / 2;
    float cy = h / 2;

    for (const auto& comp : comps) {
        if (comp.width >= 30 && comp.height >= 30) {
            float component_cx = comp.x + comp.width / 2.0f;
            float component_cy = comp.y + comp.height / 2.0f;

            AssemblyCheckItem item;
            item.type = AssemblyItemType::ComponentPosition;
            item.x = comp.x;
            item.y = comp.y;
            item.width = comp.width;
            item.height = comp.height;
            item.deviation = std::sqrt((component_cx - cx) * (component_cx - cx) +
                                       (component_cy - cy) * (component_cy - cy));
            item.tolerance = tolerance;
            item.is_pass = (item.deviation < tolerance);
            item.confidence = 0.85f;
            item.description = assembly_item_type_name(item.type) +
                              " 偏差=" + std::to_string(item.deviation);

            result.items.push_back(item);
            if (item.is_pass) result.pass_count++;
            else result.fail_count++;
        }
    }

    // 3. 对齐检查
    // 检测主要边缘方向
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);

    int horizontal_edges = 0;
    int vertical_edges = 0;

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            if (edge[y * w + x] > 0) {
                int gx = -gray[(y-1)*w + (x-1)] + gray[(y-1)*w + (x+1)]
                         -2*gray[y*w + (x-1)] + 2*gray[y*w + (x+1)]
                         -gray[(y+1)*w + (x-1)] + gray[(y+1)*w + (x+1)];

                int gy = -gray[(y-1)*w + (x-1)] - 2*gray[(y-1)*w + x] - gray[(y-1)*w + (x+1)]
                         +gray[(y+1)*w + (x-1)] + 2*gray[(y+1)*w + x] + gray[(y+1)*w + (x+1)];

                if (std::abs(gx) > std::abs(gy)) horizontal_edges++;
                else vertical_edges++;
            }
        }
    }

    AssemblyCheckItem alignment_item;
    alignment_item.type = AssemblyItemType::Alignment;
    alignment_item.x = 0;
    alignment_item.y = 0;
    alignment_item.width = w;
    alignment_item.height = h;
    float alignment_ratio = static_cast<float>(horizontal_edges) / (vertical_edges + 1);
    alignment_item.deviation = std::abs(alignment_ratio - 1.0f);
    alignment_item.tolerance = 0.3f;
    alignment_item.is_pass = (alignment_item.deviation < alignment_item.tolerance);
    alignment_item.confidence = 0.8f;
    alignment_item.description = assembly_item_type_name(alignment_item.type) +
                                " 比率=" + std::to_string(alignment_ratio);

    result.items.push_back(alignment_item);
    if (alignment_item.is_pass) result.pass_count++;
    else result.fail_count++;

    // 4. 间隙检查
    // 检测元器件之间的距离
    for (size_t i = 0; i < comps.size(); ++i) {
        for (size_t j = i + 1; j < comps.size(); ++j) {
            int gap_x = std::abs(comps[i].x + comps[i].width - comps[j].x);
            int gap_y = std::abs(comps[i].y + comps[i].height - comps[j].y);
            int gap = std::min(gap_x, gap_y);

            if (gap > 0 && gap < 50) {
                AssemblyCheckItem gap_item;
                gap_item.type = AssemblyItemType::Gap;
                gap_item.x = (comps[i].x + comps[j].x) / 2;
                gap_item.y = (comps[i].y + comps[j].y) / 2;
                gap_item.deviation = static_cast<float>(gap);
                gap_item.tolerance = tolerance;
                gap_item.is_pass = (gap >= 5 && gap <= 30);
                gap_item.confidence = 0.75f;
                gap_item.description = assembly_item_type_name(gap_item.type) +
                                      " 间隙=" + std::to_string(gap) + "px";

                result.items.push_back(gap_item);
                if (gap_item.is_pass) result.pass_count++;
                else result.fail_count++;
            }
        }
    }

    // 计算整体评分
    int total_checks = result.pass_count + result.fail_count;
    result.overall_score = total_checks > 0 ?
                          static_cast<float>(result.pass_count) / total_checks : 0.0f;
    result.is_complete = (result.fail_count == 0);

    result.summary = "组装检查: 通过=" + std::to_string(result.pass_count) +
                    ", 失败=" + std::to_string(result.fail_count) +
                    ", 完成=" + (result.is_complete ? "是" : "否");

    OVF_INFO() << "Assembly check: pass=" << result.pass_count
              << ", fail=" << result.fail_count;

    return ErrorCode::Success;
}

ErrorCode grade_quality(const ImageData& image, QualityGradeResult& result,
                        const QualityStandard& standard) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    result.defects.clear();
    result.recommendations.clear();

    // 计算各项质量指标
    float mean = compute_image_mean(gray);
    float std = compute_image_std(gray, mean);

    // 1. 亮度评分
    float brightness_score = std::clamp(mean / 128.0f, 0.0f, 1.0f) * 20.0f;

    // 2. 对比度评分
    float contrast_score = std::clamp(std / 40.0f, 0.0f, 1.0f) * 25.0f;

    // 3. 清晰度评分
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 20);
    int edge_count = 0;
    for (int i = 0; i < w * h; ++i) {
        if (edge[i] > 0) edge_count++;
    }
    float sharpness_score = std::clamp(static_cast<float>(edge_count) / (w * h * 0.15f), 0.0f, 1.0f) * 25.0f;

    // 4. 均匀性评分
    float uniformity_score = 0.0f;
    int block_size = 32;
    Vector<float> block_means;

    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            float block_mean = compute_local_mean(gray, w, h, x + block_size/2, y + block_size/2, block_size/2);
            block_means.push_back(block_mean);
        }
    }

    if (!block_means.empty()) {
        float block_mean_avg = std::accumulate(block_means.begin(), block_means.end(), 0.0f) / block_means.size();
        float block_variance = 0.0f;
        for (float bm : block_means) {
            block_variance += (bm - block_mean_avg) * (bm - block_mean_avg);
        }
        block_variance /= block_means.size();
        uniformity_score = std::clamp(1.0f - block_variance / 1000.0f, 0.0f, 1.0f) * 30.0f;
    }

    // 计算总分
    result.score = brightness_score + contrast_score + sharpness_score + uniformity_score;

    // 根据标准进行分级
    if (result.score >= standard.a_threshold) {
        result.grade = QualityGrade::A;
    } else if (result.score >= standard.b_threshold) {
        result.grade = QualityGrade::B;
    } else if (result.score >= standard.c_threshold) {
        result.grade = QualityGrade::C;
    } else if (result.score >= standard.d_threshold) {
        result.grade = QualityGrade::D;
    } else {
        result.grade = QualityGrade::F;
    }

    // 生成缺陷描述和改进建议
    if (brightness_score < 15) {
        result.defects.push_back("亮度不足");
        result.recommendations.push_back("调整照明条件或图像曝光");
    }
    if (contrast_score < 20) {
        result.defects.push_back("对比度偏低");
        result.recommendations.push_back("增强图像对比度或调整光照角度");
    }
    if (sharpness_score < 20) {
        result.defects.push_back("清晰度不足");
        result.recommendations.push_back("检查相机聚焦或减少运动模糊");
    }
    if (uniformity_score < 25) {
        result.defects.push_back("均匀性不足");
        result.recommendations.push_back("改善照明均匀性或校正镜头畸变");
    }

    result.confidence = std::clamp(result.score / 100.0f, 0.0f, 1.0f);

    result.description = quality_grade_name(result.grade) +
                        " 评分=" + std::to_string(result.score);

    OVF_INFO() << "Quality grading: grade=" << quality_grade_name(result.grade)
              << ", score=" << result.score;

    return ErrorCode::Success;
}

} // namespace industry_utils

// ========== PCB检测节点实现 ==========

PCBComponentNode::PCBComponentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PCBComponentNode::make_info() {
    NodeInfo info;
    info.id = "PCBComponent";
    info.name = "PCB元器件检测";
    info.category = "PCB检测";
    info.description = "PCB元器件定位与类型识别";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("components", "元器件列表", DataType::String));
    info.outputs.push_back(DataPort("count", "元器件数量", DataType::Number));

    info.params.push_back(ParamDef("min_confidence", "最小置信度", DataType::Number, Data(0.5)));

    return info;
}

Result<void> PCBComponentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    min_confidence_ = static_cast<float>(get_param("min_confidence", Data(0.5)).as_number());

    Vector<pcb_utils::ComponentInfo> components;
    ErrorCode err = pcb_utils::detect_components(input, components, min_confidence_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Component detection failed");
    }

    // 创建标记图像
    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);

    Vector<uint8_t> marked = gray;
    for (const auto& comp : components) {
        draw_rect_marker(marked, w, h, comp.x, comp.y, comp.width, comp.height);
    }

    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));

    String comp_str = "检测到 " + std::to_string(components.size()) + " 个元器件:\n";
    for (const auto& c : components) {
        comp_str += c.description + "\n";
    }
    set_output("components", Data(comp_str));
    set_output("count", Data(static_cast<int>(components.size())));

    OVF_INFO() << "PCBComponentNode completed: " << components.size() << " components found";
    return Result<void>::success();
}

PCBTraceNode::PCBTraceNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PCBTraceNode::make_info() {
    NodeInfo info;
    info.id = "PCBTrace";
    info.name = "PCB线路检测";
    info.category = "PCB检测";
    info.description = "PCB线路检测，识别断路和短路";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("traces", "线路列表", DataType::String));
    info.outputs.push_back(DataPort("open_count", "断路数量", DataType::Number));
    info.outputs.push_back(DataPort("short_count", "短路数量", DataType::Number));

    info.params.push_back(ParamDef("min_width", "最小宽度", DataType::Number, Data(2)));
    info.params.push_back(ParamDef("thickness_threshold", "厚度阈值", DataType::Number, Data(0.3)));

    return info;
}

Result<void> PCBTraceNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    min_width_ = static_cast<int>(get_param("min_width", Data(2)).as_int());
    thickness_threshold_ = static_cast<float>(get_param("thickness_threshold", Data(0.3)).as_number());

    Vector<pcb_utils::TraceInfo> traces;
    ErrorCode err = pcb_utils::detect_traces(input, traces, min_width_, thickness_threshold_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Trace detection failed");
    }

    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);

    Vector<uint8_t> marked = gray;
    int open_count = 0, short_count = 0;

    for (const auto& trace : traces) {
        draw_rect_marker(marked, w, h, trace.x, trace.y, trace.width, trace.height);
        if (trace.is_open) open_count++;
        if (trace.is_short) short_count++;
    }

    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));

    String trace_str = "检测到 " + std::to_string(traces.size()) + " 条线路:\n";
    for (const auto& t : traces) {
        trace_str += t.description + "\n";
    }
    set_output("traces", Data(trace_str));
    set_output("open_count", Data(open_count));
    set_output("short_count", Data(short_count));

    OVF_INFO() << "PCBTraceNode completed: " << traces.size() << " traces, "
              << open_count << " open, " << short_count << " short";
    return Result<void>::success();
}

PCBPadNode::PCBPadNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PCBPadNode::make_info() {
    NodeInfo info;
    info.id = "PCBPad";
    info.name = "PCB焊盘检测";
    info.category = "PCB检测";
    info.description = "PCB焊盘定位与质量检测";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("pads", "焊盘列表", DataType::String));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));

    info.params.push_back(ParamDef("min_roundness", "最小圆度", DataType::Number, Data(0.7)));
    info.params.push_back(ParamDef("min_coverage", "最小覆盖率", DataType::Number, Data(0.6)));

    return info;
}

Result<void> PCBPadNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    min_roundness_ = static_cast<float>(get_param("min_roundness", Data(0.7)).as_number());
    min_coverage_ = static_cast<float>(get_param("min_coverage", Data(0.6)).as_number());

    Vector<pcb_utils::PadInfo> pads;
    ErrorCode err = pcb_utils::detect_pads(input, pads, min_roundness_, min_coverage_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Pad detection failed");
    }

    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);

    Vector<uint8_t> marked = gray;
    int defect_count = 0;

    for (const auto& pad : pads) {
        draw_rect_marker(marked, w, h, pad.x, pad.y, pad.width, pad.height);
        if (pad.is_defective) defect_count++;
    }

    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));

    String pad_str = "检测到 " + std::to_string(pads.size()) + " 个焊盘:\n";
    for (const auto& p : pads) {
        pad_str += p.description + "\n";
    }
    set_output("pads", Data(pad_str));
    set_output("defect_count", Data(defect_count));

    OVF_INFO() << "PCBPadNode completed: " << pads.size() << " pads, " << defect_count << " defective";
    return Result<void>::success();
}

PCBDefectNode::PCBDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PCBDefectNode::make_info() {
    NodeInfo info;
    info.id = "PCBDefect";
    info.name = "PCB缺陷综合检测";
    info.category = "PCB检测";
    info.description = "PCB综合缺陷检测，包括元器件缺失、焊桥、划痕、污染等";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::String));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));

    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.5)));

    return info;
}

Result<void> PCBDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    sensitivity_ = static_cast<float>(get_param("sensitivity", Data(0.5)).as_number());

    Vector<pcb_utils::PCBDefectInfo> defects;
    ErrorCode err = pcb_utils::detect_pcb_defects(input, defects, sensitivity_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "PCB defect detection failed");
    }

    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);

    Vector<uint8_t> marked = gray;
    for (const auto& defect : defects) {
        draw_rect_marker(marked, w, h, defect.x, defect.y, defect.width, defect.height);
    }

    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));

    String defect_str = "检测到 " + std::to_string(defects.size()) + " 个缺陷:\n";
    for (const auto& d : defects) {
        defect_str += d.description + "\n";
    }
    set_output("defects", Data(defect_str));
    set_output("defect_count", Data(static_cast<int>(defects.size())));

    OVF_INFO() << "PCBDefectNode completed: " << defects.size() << " defects found";
    return Result<void>::success();
}

// ========== 半导体检测节点实现 ==========

WaferDieNode::WaferDieNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo WaferDieNode::make_info() {
    NodeInfo info;
    info.id = "WaferDie";
    info.name = "晶圆芯片检测";
    info.category = "半导体检测";
    info.description = "晶圆芯片定位与状态检测";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("dies", "芯片列表", DataType::String));
    info.outputs.push_back(DataPort("good_count", "正常数量", DataType::Number));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));

    info.params.push_back(ParamDef("die_width", "芯片宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("die_height", "芯片高度", DataType::Number, Data(100)));

    return info;
}

Result<void> WaferDieNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    die_width_ = static_cast<int>(get_param("die_width", Data(100)).as_int());
    die_height_ = static_cast<int>(get_param("die_height", Data(100)).as_int());

    Vector<wafer_utils::DieInfo> dies;
    ErrorCode err = wafer_utils::detect_dies(input, dies, die_width_, die_height_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Die detection failed");
    }

    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);

    Vector<uint8_t> marked = gray;
    int good_count = 0, defect_count = 0;

    for (const auto& die : dies) {
        draw_rect_marker(marked, w, h, die.x, die.y, die.width, die.height);
        if (die.status == wafer_utils::DieStatus::Good) good_count++;
        else if (die.status == wafer_utils::DieStatus::Defective ||
                 die.status == wafer_utils::DieStatus::Cracked) defect_count++;
    }

    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));

    String die_str = "检测到 " + std::to_string(dies.size()) + " 个芯片:\n";
    for (const auto& d : dies) {
        die_str += d.description + "\n";
    }
    set_output("dies", Data(die_str));
    set_output("good_count", Data(good_count));
    set_output("defect_count", Data(defect_count));

    OVF_INFO() << "WaferDieNode completed: " << dies.size() << " dies, "
              << good_count << " good, " << defect_count << " defective";
    return Result<void>::success();
}

WaferDefectNode::WaferDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo WaferDefectNode::make_info() {
    NodeInfo info;
    info.id = "WaferDefect";
    info.name = "晶圆缺陷检测";
    info.category = "半导体检测";
    info.description = "晶圆缺陷检测，包括颗粒、划痕、边缘缺陷等";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "标记图像", DataType::Image));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::String));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));

    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.5)));

    return info;
}

Result<void> WaferDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    sensitivity_ = static_cast<float>(get_param("sensitivity", Data(0.5)).as_number());

    Vector<wafer_utils::WaferDefectInfo> defects;
    ErrorCode err = wafer_utils::detect_wafer_defects(input, defects, sensitivity_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Wafer defect detection failed");
    }

    int w = input.width;
    int h = input.height;
    Vector<uint8_t> gray;
    to_gray_image(input, gray, w, h);

    Vector<uint8_t> marked = gray;
    for (const auto& defect : defects) {
        draw_rect_marker(marked, w, h, defect.x, defect.y, defect.width, defect.height);
    }

    ImageData output = create_gray_output(w, h, marked);
    set_output("image", Data(output));

    String defect_str = "检测到 " + std::to_string(defects.size()) + " 个缺陷:\n";
    for (const auto& d : defects) {
        defect_str += d.description + "\n";
    }
    set_output("defects", Data(defect_str));
    set_output("defect_count", Data(static_cast<int>(defects.size())));

    OVF_INFO() << "WaferDefectNode completed: " << defects.size() << " defects found";
    return Result<void>::success();
}

DieAlignmentNode::DieAlignmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DieAlignmentNode::make_info() {
    NodeInfo info;
    info.id = "DieAlignment";
    info.name = "芯片对准检测";
    info.category = "半导体检测";
    info.description = "检测芯片对准偏差，包括偏移和旋转";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("offset_x", "X偏移", DataType::Number));
    info.outputs.push_back(DataPort("offset_y", "Y偏移", DataType::Number));
    info.outputs.push_back(DataPort("rotation", "旋转角度", DataType::Number));
    info.outputs.push_back(DataPort("is_aligned", "是否对准", DataType::Boolean));
    info.outputs.push_back(DataPort("report", "检测报告", DataType::String));

    info.params.push_back(ParamDef("tolerance_x", "X容差", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("tolerance_y", "Y容差", DataType::Number, Data(5.0)));

    return info;
}

Result<void> DieAlignmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    tolerance_x_ = static_cast<float>(get_param("tolerance_x", Data(5.0)).as_number());
    tolerance_y_ = static_cast<float>(get_param("tolerance_y", Data(5.0)).as_number());

    wafer_utils::AlignmentInfo alignment;
    ErrorCode err = wafer_utils::check_die_alignment(input, alignment, tolerance_x_, tolerance_y_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Die alignment check failed");
    }

    set_output("offset_x", Data(alignment.offset_x));
    set_output("offset_y", Data(alignment.offset_y));
    set_output("rotation", Data(alignment.rotation));
    set_output("is_aligned", Data(alignment.is_aligned));

    String report = "芯片对准检测结果:\n";
    report += "X偏移: " + std::to_string(alignment.offset_x) + " (容差: " + std::to_string(tolerance_x_) + ")\n";
    report += "Y偏移: " + std::to_string(alignment.offset_y) + " (容差: " + std::to_string(tolerance_y_) + ")\n";
    report += "旋转角度: " + std::to_string(alignment.rotation) + "度\n";
    report += "对准状态: " + String(alignment.is_aligned ? "合格" : "不合格");

    set_output("report", Data(report));

    OVF_INFO() << "DieAlignmentNode completed: aligned=" << alignment.is_aligned;
    return Result<void>::success();
}

// ========== 包装检测节点实现 ==========

PackageSealNode::PackageSealNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PackageSealNode::make_info() {
    NodeInfo info;
    info.id = "PackageSeal";
    info.name = "包装密封检测";
    info.category = "包装检测";
    info.description = "包装密封完整性检测";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("seal_strength", "密封强度", DataType::Number));
    info.outputs.push_back(DataPort("status", "密封状态", DataType::String));
    info.outputs.push_back(DataPort("is_good", "是否合格", DataType::Boolean));
    info.outputs.push_back(DataPort("report", "检测报告", DataType::String));

    info.params.push_back(ParamDef("min_strength", "最小强度", DataType::Number, Data(0.7)));

    return info;
}

Result<void> PackageSealNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    min_strength_ = static_cast<float>(get_param("min_strength", Data(0.7)).as_number());

    package_utils::SealInfo seal;
    ErrorCode err = package_utils::detect_seal(input, seal, min_strength_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Package seal detection failed");
    }

    set_output("seal_strength", Data(seal.seal_strength));
    set_output("status", Data(package_utils::seal_status_name(seal.status)));
    set_output("is_good", Data(seal.status == package_utils::SealStatus::Good));

    String report = "包装密封检测结果:\n";
    report += "密封强度: " + std::to_string(seal.seal_strength) + "\n";
    report += "密封状态: " + package_utils::seal_status_name(seal.status) + "\n";
    report += "合格判定: " + String(seal.status == package_utils::SealStatus::Good ? "合格" : "不合格");

    set_output("report", Data(report));

    OVF_INFO() << "PackageSealNode completed: status=" << package_utils::seal_status_name(seal.status);
    return Result<void>::success();
}

PackageLabelNode::PackageLabelNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PackageLabelNode::make_info() {
    NodeInfo info;
    info.id = "PackageLabel";
    info.name = "包装标签检测";
    info.category = "包装检测";
    info.description = "包装标签存在性、可读性和对齐检测";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("is_present", "标签存在", DataType::Boolean));
    info.outputs.push_back(DataPort("is_readable", "标签可读", DataType::Boolean));
    info.outputs.push_back(DataPort("alignment_score", "对齐评分", DataType::Number));
    info.outputs.push_back(DataPort("report", "检测报告", DataType::String));

    info.params.push_back(ParamDef("min_clarity", "最小清晰度", DataType::Number, Data(0.6)));
    info.params.push_back(ParamDef("min_alignment", "最小对齐度", DataType::Number, Data(0.7)));

    return info;
}

Result<void> PackageLabelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    min_clarity_ = static_cast<float>(get_param("min_clarity", Data(0.6)).as_number());
    min_alignment_ = static_cast<float>(get_param("min_alignment", Data(0.7)).as_number());

    package_utils::LabelInfo label;
    ErrorCode err = package_utils::detect_label(input, label, min_clarity_, min_alignment_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Package label detection failed");
    }

    set_output("is_present", Data(label.is_present));
    set_output("is_readable", Data(label.is_readable));
    set_output("alignment_score", Data(label.alignment_score));

    String report = "包装标签检测结果:\n";
    report += "标签存在: " + String(label.is_present ? "是" : "否") + "\n";
    report += "标签可读: " + String(label.is_readable ? "是" : "否") + "\n";
    report += "清晰度: " + std::to_string(label.clarity) + "\n";
    report += "对齐评分: " + std::to_string(label.alignment_score) + "\n";
    report += "合格判定: " + String(label.is_present && label.is_readable && label.is_correct ? "合格" : "不合格");

    set_output("report", Data(report));

    OVF_INFO() << "PackageLabelNode completed: present=" << label.is_present
              << ", readable=" << label.is_readable;
    return Result<void>::success();
}

PackageQualityNode::PackageQualityNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PackageQualityNode::make_info() {
    NodeInfo info;
    info.id = "PackageQuality";
    info.name = "包装质量检测";
    info.category = "包装检测";
    info.description = "包装综合质量检测，包括损坏、变形、污染等";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("overall_score", "整体评分", DataType::Number));
    info.outputs.push_back(DataPort("is_pass", "是否合格", DataType::Boolean));
    info.outputs.push_back(DataPort("issue_count", "问题数量", DataType::Number));
    info.outputs.push_back(DataPort("issues", "问题列表", DataType::String));
    info.outputs.push_back(DataPort("report", "检测报告", DataType::String));

    info.params.push_back(ParamDef("pass_threshold", "合格阈值", DataType::Number, Data(0.8)));

    return info;
}

Result<void> PackageQualityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    pass_threshold_ = static_cast<float>(get_param("pass_threshold", Data(0.8)).as_number());

    package_utils::PackageQualityInfo quality;
    ErrorCode err = package_utils::detect_package_quality(input, quality, pass_threshold_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Package quality detection failed");
    }

    set_output("overall_score", Data(quality.overall_score));
    set_output("is_pass", Data(quality.is_pass));
    set_output("issue_count", Data(static_cast<int>(quality.issues.size())));

    String issues_str = "";
    for (const auto& issue : quality.issues) {
        issues_str += package_utils::package_issue_type_name(issue) + "\n";
    }
    set_output("issues", Data(issues_str));

    String report = "包装质量检测结果:\n";
    report += "整体评分: " + std::to_string(quality.overall_score) + "\n";
    report += "合格判定: " + String(quality.is_pass ? "合格" : "不合格") + "\n";
    report += "发现问题: " + std::to_string(quality.issues.size()) + " 项\n";
    if (!quality.issues.empty()) {
        report += "问题详情:\n";
        for (const auto& issue : quality.issues) {
            report += "- " + package_utils::package_issue_type_name(issue) + "\n";
        }
    }

    set_output("report", Data(report));

    OVF_INFO() << "PackageQualityNode completed: score=" << quality.overall_score
              << ", pass=" << quality.is_pass;
    return Result<void>::success();
}

// ========== 通用工业检测节点实现 ==========

AssemblyCheckNode::AssemblyCheckNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo AssemblyCheckNode::make_info() {
    NodeInfo info;
    info.id = "AssemblyCheck";
    info.name = "组装检查";
    info.category = "通用工业";
    info.description = "产品组装完整性检查，包括元器件存在、位置、对齐等";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("overall_score", "整体评分", DataType::Number));
    info.outputs.push_back(DataPort("pass_count", "通过项数", DataType::Number));
    info.outputs.push_back(DataPort("fail_count", "失败项数", DataType::Number));
    info.outputs.push_back(DataPort("is_complete", "组装完成", DataType::Boolean));
    info.outputs.push_back(DataPort("items", "检查项列表", DataType::String));
    info.outputs.push_back(DataPort("report", "检测报告", DataType::String));

    info.params.push_back(ParamDef("tolerance", "位置容差", DataType::Number, Data(5.0)));

    return info;
}

Result<void> AssemblyCheckNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    tolerance_ = static_cast<float>(get_param("tolerance", Data(5.0)).as_number());

    industry_utils::AssemblyCheckResult result;
    ErrorCode err = industry_utils::check_assembly(input, result, tolerance_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Assembly check failed");
    }

    set_output("overall_score", Data(result.overall_score));
    set_output("pass_count", Data(result.pass_count));
    set_output("fail_count", Data(result.fail_count));
    set_output("is_complete", Data(result.is_complete));

    String items_str = "";
    for (const auto& item : result.items) {
        items_str += industry_utils::assembly_item_type_name(item.type) +
                    ": " + (item.is_pass ? "通过" : "失败") + "\n";
    }
    set_output("items", Data(items_str));

    String report = "组装检查结果:\n";
    report += "整体评分: " + std::to_string(result.overall_score) + "\n";
    report += "通过项数: " + std::to_string(result.pass_count) + "\n";
    report += "失败项数: " + std::to_string(result.fail_count) + "\n";
    report += "组装完成: " + String(result.is_complete ? "是" : "否") + "\n";
    report += result.summary;

    set_output("report", Data(report));

    OVF_INFO() << "AssemblyCheckNode completed: pass=" << result.pass_count
              << ", fail=" << result.fail_count;
    return Result<void>::success();
}

QualityGradeNode::QualityGradeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo QualityGradeNode::make_info() {
    NodeInfo info;
    info.id = "QualityGrade";
    info.name = "质量分级";
    info.category = "通用工业";
    info.description = "产品质量分级评估，支持自定义分级标准";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("grade", "质量等级", DataType::String));
    info.outputs.push_back(DataPort("score", "百分制评分", DataType::Number));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));
    info.outputs.push_back(DataPort("defects", "缺陷描述", DataType::String));
    info.outputs.push_back(DataPort("recommendations", "改进建议", DataType::String));
    info.outputs.push_back(DataPort("report", "分级报告", DataType::String));

    info.params.push_back(ParamDef("a_threshold", "A级阈值", DataType::Number, Data(95.0)));
    info.params.push_back(ParamDef("b_threshold", "B级阈值", DataType::Number, Data(85.0)));
    info.params.push_back(ParamDef("c_threshold", "C级阈值", DataType::Number, Data(75.0)));
    info.params.push_back(ParamDef("d_threshold", "D级阈值", DataType::Number, Data(60.0)));

    return info;
}

Result<void> QualityGradeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    industry_utils::QualityStandard standard;
    standard.a_threshold = static_cast<float>(get_param("a_threshold", Data(95.0)).as_number());
    standard.b_threshold = static_cast<float>(get_param("b_threshold", Data(85.0)).as_number());
    standard.c_threshold = static_cast<float>(get_param("c_threshold", Data(75.0)).as_number());
    standard.d_threshold = static_cast<float>(get_param("d_threshold", Data(60.0)).as_number());

    industry_utils::QualityGradeResult result;
    ErrorCode err = industry_utils::grade_quality(input, result, standard);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Quality grading failed");
    }

    set_output("grade", Data(industry_utils::quality_grade_name(result.grade)));
    set_output("score", Data(result.score));
    set_output("confidence", Data(result.confidence));

    String defects_str = "";
    for (const auto& defect : result.defects) {
        defects_str += defect + "\n";
    }
    set_output("defects", Data(defects_str));

    String recommendations_str = "";
    for (const auto& rec : result.recommendations) {
        recommendations_str += rec + "\n";
    }
    set_output("recommendations", Data(recommendations_str));

    String report = "质量分级报告:\n";
    report += "质量等级: " + industry_utils::quality_grade_name(result.grade) + "\n";
    report += "百分制评分: " + std::to_string(result.score) + "\n";
    report += "置信度: " + std::to_string(result.confidence) + "\n";
    if (!result.defects.empty()) {
        report += "缺陷描述:\n";
        for (const auto& defect : result.defects) {
            report += "- " + defect + "\n";
        }
    }
    if (!result.recommendations.empty()) {
        report += "改进建议:\n";
        for (const auto& rec : result.recommendations) {
            report += "- " + rec + "\n";
        }
    }

    set_output("report", Data(report));

    OVF_INFO() << "QualityGradeNode completed: grade=" << industry_utils::quality_grade_name(result.grade)
              << ", score=" << result.score;
    return Result<void>::success();
}

// ========== 节点注册 ==========

OVF_REGISTER_NODE(PCBComponentNode, "PCBComponent", PCBComponentNode::make_info())
OVF_REGISTER_NODE(PCBTraceNode, "PCBTrace", PCBTraceNode::make_info())
OVF_REGISTER_NODE(PCBPadNode, "PCBPad", PCBPadNode::make_info())
OVF_REGISTER_NODE(PCBDefectNode, "PCBDefect", PCBDefectNode::make_info())
OVF_REGISTER_NODE(WaferDieNode, "WaferDie", WaferDieNode::make_info())
OVF_REGISTER_NODE(WaferDefectNode, "WaferDefect", WaferDefectNode::make_info())
OVF_REGISTER_NODE(DieAlignmentNode, "DieAlignment", DieAlignmentNode::make_info())
OVF_REGISTER_NODE(PackageSealNode, "PackageSeal", PackageSealNode::make_info())
OVF_REGISTER_NODE(PackageLabelNode, "PackageLabel", PackageLabelNode::make_info())
OVF_REGISTER_NODE(PackageQualityNode, "PackageQuality", PackageQualityNode::make_info())
OVF_REGISTER_NODE(AssemblyCheckNode, "AssemblyCheck", AssemblyCheckNode::make_info())
OVF_REGISTER_NODE(QualityGradeNode, "QualityGrade", QualityGradeNode::make_info())

} // namespace algorithm
} // namespace ovf