/**
 * @file industry_extended.cpp
 * @brief 行业扩展算子模块实现 - 医药、食品、纺织、汽车行业专用检测节点
 */

#include "ovf/algorithm/industry_extended.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/morphology.h"
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

// 计算圆度
float compute_roundness(int area, int width, int height) {
    float perimeter = 2 * (width + height);
    float roundness = (4 * 3.14159f * area) / (perimeter * perimeter + 1);
    return std::clamp(roundness, 0.0f, 1.0f);
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

// 计算区域平均颜色
float compute_region_color(const Vector<uint8_t>& gray, int w, int h, int x, int y, int width, int height) {
    float sum = 0.0f;
    int count = 0;

    for (int py = y; py < y + height && py < h; ++py) {
        for (int px = x; px < x + width && px < w; ++px) {
            sum += gray[py * w + px];
            count++;
        }
    }

    return count > 0 ? sum / count : 0.0f;
}

// 计算形状因子
float compute_shape_factor(int area, int width, int height) {
    float aspect_ratio = static_cast<float>(width) / (height + 1);
    float fill_ratio = static_cast<float>(area) / (width * height + 1);

    // 形状因子：接近1为矩形，大于1为细长形
    return aspect_ratio * fill_ratio;
}

} // anonymous namespace

// ========== 医药行业检测模块实现 ==========

namespace pharma_utils {

ErrorCode detect_pills(const ImageData& image, Vector<PillInfo>& pills,
                      float min_roundness, int min_size) {
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

    pills.clear();
    for (const auto& comp : comps) {
        // 过滤尺寸太小的区域
        if (comp.width < min_size || comp.height < min_size || comp.area < min_size * min_size) continue;

        // 计算圆度
        float roundness = compute_roundness(comp.area, comp.width, comp.height);

        if (roundness >= min_roundness) {
            PillInfo pill;
            pill.x = comp.x;
            pill.y = comp.y;
            pill.width = comp.width;
            pill.height = comp.height;
            pill.area = static_cast<float>(comp.area);
            pill.roundness = roundness;

            // 计算颜色值
            pill.color_value = compute_region_color(gray, w, h, comp.x, comp.y, comp.width, comp.height);

            // 根据圆度判断形状
            if (roundness > 0.85f) {
                pill.shape_type = "圆形";
            } else if (roundness > 0.6f && roundness < 0.85f) {
                float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
                if (aspect_ratio > 1.3f || aspect_ratio < 0.77f) {
                    pill.shape_type = "椭圆形";
                } else {
                    pill.shape_type = "近圆形";
                }
            } else {
                pill.shape_type = "不规则";
            }

            pill.confidence = std::clamp(roundness * 0.7f + 0.2f, 0.0f, 1.0f);
            pill.description = pill.shape_type + "药片 @(" +
                              std::to_string(pill.x) + "," +
                              std::to_string(pill.y) + ") " +
                              std::to_string(pill.width) + "x" +
                              std::to_string(pill.height) +
                              " 圆度=" + std::to_string(pill.roundness);

            pills.push_back(pill);
        }
    }

    OVF_INFO() << "Pill detection: found " << pills.size() << " pills";
    return ErrorCode::Success;
}

ErrorCode detect_pill_defects(const ImageData& image, Vector<PillDefectInfo>& defects,
                             float sensitivity) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    defects.clear();

    // 1. 裂纹检测 - 检测细长线条
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 20);

    Vector<uint8_t> temp1 = edge;
    Vector<ConnectedComponent> edge_comps = find_connected_components(temp1, w, h);

    float crack_threshold = sensitivity * 30;
    for (const auto& comp : edge_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        // 裂纹通常是细长形
        if ((aspect_ratio > 3.0f || aspect_ratio < 0.33f) && comp.area > crack_threshold) {
            PillDefectInfo defect;
            defect.type = PillDefectType::Crack;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 200.0f, 0.0f, 1.0f);
            defect.confidence = 0.85f;
            defect.description = pill_defect_type_name(defect.type) +
                                " @(" + std::to_string(defect.x) + "," +
                                std::to_string(defect.y) + ") 面积=" +
                                std::to_string(comp.area);
            defects.push_back(defect);
        }
    }

    // 2. 斑点检测 - 检测局部对比度异常
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 5, 2.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    Vector<uint8_t> spot_mask(w * h, 0);
    float spot_threshold = sensitivity * 40;
    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > spot_threshold && diff[i] < 100) {
            spot_mask[i] = 255;
        }
    }

    Vector<uint8_t> temp2 = spot_mask;
    Vector<ConnectedComponent> spot_comps = find_connected_components(temp2, w, h);

    for (const auto& comp : spot_comps) {
        if (comp.area >= 5 && comp.area < 200) {
            PillDefectInfo defect;
            defect.type = PillDefectType::Spot;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            defect.confidence = 0.8f;
            defect.description = pill_defect_type_name(defect.type) +
                                " 面积=" + std::to_string(comp.area);
            defects.push_back(defect);
        }
    }

    // 3. 变形检测 - 检测不规则形状
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> kernel2(9, 255);
    Vector<uint8_t> opened(w * h);
    morph_utils::open(binary.data(), opened.data(), w, h, kernel2.data(), 3, 3, 1);

    Vector<uint8_t> temp3 = opened;
    Vector<ConnectedComponent> shape_comps = find_connected_components(temp3, w, h);

    int min_size = 20;
    for (const auto& comp : shape_comps) {
        float roundness = compute_roundness(comp.area, comp.width, comp.height);
        float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height + 1);

        // 变形：圆度低或填充率低
        if (roundness < 0.5f && area_ratio < 0.6f && comp.area >= min_size * min_size) {
            PillDefectInfo defect;
            defect.type = PillDefectType::Deformation;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(1.0f - roundness, 0.0f, 1.0f);
            defect.confidence = 0.75f;
            defect.description = pill_defect_type_name(defect.type) +
                                " 圆度=" + std::to_string(roundness);
            defects.push_back(defect);
        }
    }

    OVF_INFO() << "Pill defect detection: found " << defects.size() << " defects";
    return ErrorCode::Success;
}

ErrorCode detect_capsules(const ImageData& image, Vector<CapsuleInfo>& capsules,
                         float min_fill_ratio) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 边缘检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    // 检测胶囊形状（通常是椭圆形）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(binary.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    capsules.clear();
    for (const auto& comp : comps) {
        // 过滤尺寸
        if (comp.width < 20 || comp.height < 20) continue;

        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        // 胶囊通常是椭圆形，长宽比在1.5-3之间
        if (aspect_ratio < 1.3f || aspect_ratio > 4.0f) continue;

        CapsuleInfo capsule;
        capsule.x = comp.x;
        capsule.y = comp.y;
        capsule.width = comp.width;
        capsule.height = comp.height;

        // 计算填充度（基于区域亮度均匀性）
        float region_mean = compute_region_color(gray, w, h, comp.x, comp.y, comp.width, comp.height);
        float global_mean = compute_image_mean(gray);

        // 计算胶囊内部的标准差作为填充度指标
        float internal_std = compute_local_std(gray, w, h,
                                               comp.x + comp.width/2,
                                               comp.y + comp.height/2,
                                               comp.width/4, region_mean);

        capsule.fill_ratio = std::clamp(1.0f - internal_std / 50.0f, 0.0f, 1.0f);
        capsule.is_filled = (capsule.fill_ratio >= min_fill_ratio);

        // 计算完整性（基于边缘连续性）
        int edge_count = 0;
        for (int py = comp.y; py < comp.y + comp.height && py < h; ++py) {
            for (int px = comp.x; px < comp.x + comp.width && px < w; ++px) {
                if (edge[py * w + px] > 0) edge_count++;
            }
        }

        float edge_density = static_cast<float>(edge_count) / (comp.width * comp.height + 1);
        capsule.integrity = std::clamp(edge_density * 2, 0.0f, 1.0f);
        capsule.is_complete = (capsule.integrity > 0.5f);

        capsule.confidence = std::clamp(aspect_ratio / 2.5f * 0.5f +
                                       capsule.fill_ratio * 0.3f +
                                       capsule.integrity * 0.2f, 0.0f, 1.0f);

        capsule.description = "胶囊 @(" + std::to_string(capsule.x) + "," +
                             std::to_string(capsule.y) + ") 填充度=" +
                             std::to_string(capsule.fill_ratio) +
                             " 完整性=" + std::to_string(capsule.integrity);

        capsules.push_back(capsule);
    }

    OVF_INFO() << "Capsule detection: found " << capsules.size() << " capsules";
    return ErrorCode::Success;
}

ErrorCode detect_syringes(const ImageData& image, Vector<SyringeInfo>& syringes,
                         int min_needle_length) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 边缘检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    // 形态学处理连接线条
    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> connected(w * h);
    morph_utils::close(edge.data(), connected.data(), w, h, kernel.data(), 3, 3, 2);

    // 检测细长结构（针头）
    Vector<uint8_t> temp = connected;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    syringes.clear();
    for (const auto& comp : comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);

        // 注射器形状：细长的针头 + 较宽的针筒
        if (aspect_ratio < 0.2f || aspect_ratio > 10.0f) continue;

        // 检测是否为注射器形状
        int length = std::max(comp.width, comp.height);
        int width = std::min(comp.width, comp.height);

        // 针头部分检测
        SyringeInfo syringe;
        syringe.x = comp.x;
        syringe.y = comp.y;

        if (aspect_ratio > 3.0f) {
            // 横向注射器
            syringe.body_length = comp.width * 2 / 3;
            syringe.needle_length = comp.width / 3;
        } else if (aspect_ratio < 0.33f) {
            // 纵向注射器
            syringe.body_length = comp.height * 2 / 3;
            syringe.needle_length = comp.height / 3;
        } else {
            continue; // 不是注射器形状
        }

        syringe.needle_present = (syringe.needle_length >= min_needle_length);

        // 刻度检测（检测针筒部分的边缘密度）
        int body_x = comp.x;
        int body_y = comp.y;
        int body_w = syringe.body_length;
        int body_h = width;

        if (aspect_ratio < 0.33f) {
            body_h = syringe.body_length;
            body_w = width;
        }

        // 检测刻度线条
        int scale_lines = 0;
        for (int py = body_y; py < body_y + body_h && py < h; ++py) {
            int transitions = 0;
            for (int px = body_x; px < body_x + body_w - 1 && px < w; ++px) {
                if (gray[py * w + px] != gray[py * w + px + 1]) {
                    transitions++;
                }
            }
            if (transitions > 3) scale_lines++;
        }

        syringe.scale_count = scale_lines / 5; // 每5行算一个刻度
        syringe.scale_visible = (syringe.scale_count >= 3);

        syringe.confidence = std::clamp(static_cast<float>(length) / 200.0f * 0.5f +
                                       (syringe.needle_present ? 0.3f : 0.0f) +
                                       (syringe.scale_visible ? 0.2f : 0.0f), 0.0f, 1.0f);

        syringe.description = "注射器 @(" + std::to_string(syringe.x) + "," +
                             std::to_string(syringe.y) + ") 针头长度=" +
                             std::to_string(syringe.needle_length) +
                             " 刻度数=" + std::to_string(syringe.scale_count);

        if (syringe.needle_present || syringe.scale_visible) {
            syringes.push_back(syringe);
        }
    }

    OVF_INFO() << "Syringe detection: found " << syringes.size() << " syringes";
    return ErrorCode::Success;
}

ErrorCode detect_vials(const ImageData& image, Vector<VialInfo>& vials,
                      float min_liquid_level) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 检测瓶状容器（圆柱形）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(binary.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    vials.clear();
    for (const auto& comp : comps) {
        // 过滤尺寸
        if (comp.width < 30 || comp.height < 50) continue;

        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        // 药瓶通常是竖向圆柱形，长宽比0.3-0.7
        if (aspect_ratio < 0.2f || aspect_ratio > 1.0f) continue;

        VialInfo vial;
        vial.x = comp.x;
        vial.y = comp.y;
        vial.width = comp.width;
        vial.height = comp.height;

        // 检测液位：分析下半部分亮度
        int liquid_region_y = comp.y + comp.height / 2;
        int liquid_region_h = comp.height / 2;

        float liquid_mean = compute_region_color(gray, w, h,
                                                 comp.x, liquid_region_y,
                                                 comp.width, liquid_region_h);
        float upper_mean = compute_region_color(gray, w, h,
                                                comp.x, comp.y,
                                                comp.width, comp.height / 2);

        // 液体通常比上部空气暗或亮（取决于液体）
        float brightness_diff = std::abs(liquid_mean - upper_mean);
        vial.liquid_level = std::clamp(brightness_diff / 50.0f, 0.0f, 1.0f);

        // 检测盖子（顶部区域）
        int cap_region_y = comp.y;
        int cap_region_h = comp.height / 10;

        float cap_mean = compute_region_color(gray, w, h,
                                              comp.x + comp.width/4, cap_region_y,
                                              comp.width/2, cap_region_h);
        float body_mean = compute_region_color(gray, w, h,
                                               comp.x, comp.y + comp.height/4,
                                               comp.width, comp.height/2);

        vial.cap_present = (std::abs(cap_mean - body_mean) > 20);

        // 简化密封检测
        vial.seal_quality = vial.cap_present ? 0.8f : 0.3f;
        vial.is_sealed = (vial.seal_quality > 0.3f && vial.cap_present);

        vial.confidence = std::clamp(aspect_ratio * 0.4f +
                                   (vial.cap_present ? 0.2f : 0.0f) +
                                   (vial.is_sealed ? 0.2f : 0.0f) +
                                   vial.liquid_level * 0.2f, 0.0f, 1.0f);

        vial.description = "药瓶 @(" + std::to_string(vial.x) + "," +
                          std::to_string(vial.y) + ") 液位=" +
                          std::to_string(vial.liquid_level * 100) + "%" +
                          " 密封=" + (vial.is_sealed ? "是" : "否");

        vials.push_back(vial);
    }

    OVF_INFO() << "Vial detection: found " << vials.size() << " vials";
    return ErrorCode::Success;
}

ErrorCode detect_blister_pack(const ImageData& image, BlisterPackInfo& pack,
                             int expected_cavities) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    // 检测泡罩网格结构
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 21, 10);

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(binary.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    // 分析泡罩结构
    pack.x = 0;
    pack.y = 0;
    pack.width = w;
    pack.height = h;
    pack.total_cavities = 0;
    pack.filled_cavities = 0;
    pack.empty_cavities = 0;
    pack.defective_cavities = 0;

    // 计算预期的泡罩大小
    int expected_cell_size = static_cast<int>(std::sqrt(w * h / expected_cavities));

    // 网格检测
    for (const auto& comp : comps) {
        // 泡罩通常是圆形或方形小区域
        if (comp.width < expected_cell_size / 2 ||
            comp.height < expected_cell_size / 2) continue;

        if (comp.width > expected_cell_size * 2 ||
            comp.height > expected_cell_size * 2) continue;

        pack.total_cavities++;

        // 计算填充度
        float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height + 1);

        // 已填充：面积比例较高
        if (area_ratio > 0.7f) {
            pack.filled_cavities++;
        } else if (area_ratio < 0.3f) {
            pack.empty_cavities++;
        } else {
            pack.defective_cavities++;
        }
    }

    // 计算完整性
    pack.integrity = pack.total_cavities > 0 ?
                     static_cast<float>(pack.filled_cavities) / pack.total_cavities : 0.0f;

    pack.confidence = std::clamp(pack.integrity * 0.5f +
                                (pack.total_cavities >= expected_cavities ? 0.3f : 0.0f) +
                                (pack.defective_cavities == 0 ? 0.2f : 0.0f), 0.0f, 1.0f);

    pack.description = "泡罩包装: 总数=" + std::to_string(pack.total_cavities) +
                      " 已填充=" + std::to_string(pack.filled_cavities) +
                      " 空=" + std::to_string(pack.empty_cavities) +
                      " 缺陷=" + std::to_string(pack.defective_cavities);

    OVF_INFO() << "Blister pack detection: " << pack.filled_cavities << "/" << pack.total_cavities;
    return ErrorCode::Success;
}

} // namespace pharma_utils

// ========== 食品行业检测模块实现 ==========

namespace food_utils {

ErrorCode detect_freshness(const ImageData& image, FreshnessInfo& freshness,
                          float threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    freshness.x = 0;
    freshness.y = 0;
    freshness.width = w;
    freshness.height = h;

    // 计算颜色评分（基于亮度分布）
    float mean = compute_image_mean(gray);
    float std = compute_image_std(gray, mean);

    // 新鲜食品通常有特定的亮度范围
    float color_score = std::clamp((std / 40.0f) * 0.5f + (mean / 128.0f) * 0.5f, 0.0f, 1.0f);
    freshness.color_score = color_score;

    // 计算纹理评分（基于局部变化）
    Vector<float> local_stds;
    int block_size = 32;

    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            float block_mean = compute_local_mean(gray, w, h, x + block_size/2, y + block_size/2, block_size/2);
            float block_std = compute_local_std(gray, w, h, x + block_size/2, y + block_size/2, block_size/2, block_mean);
            local_stds.push_back(block_std);
        }
    }

    if (!local_stds.empty()) {
        float avg_std = std::accumulate(local_stds.begin(), local_stds.end(), 0.0f) / local_stds.size();
        float variance = 0.0f;
        for (float s : local_stds) {
            variance += (s - avg_std) * (s - avg_std);
        }
        variance /= local_stds.size();

        // 新鲜食品纹理均匀，方差小
        freshness.texture_score = std::clamp(1.0f - variance / 1000.0f, 0.0f, 1.0f);
    } else {
        freshness.texture_score = 0.0f;
    }

    // 计算综合新鲜度评分
    freshness.freshness_score = color_score * 0.4f + freshness.texture_score * 0.6f;

    // 判断新鲜度等级
    if (freshness.freshness_score >= threshold) {
        freshness.freshness_level = "新鲜";
    } else if (freshness.freshness_score >= threshold * 0.7f) {
        freshness.freshness_level = "一般";
    } else {
        freshness.freshness_level = "不新鲜";
    }

    freshness.confidence = std::clamp(freshness.freshness_score, 0.0f, 1.0f);
    freshness.description = "新鲜度: " + freshness.freshness_level +
                           " 评分=" + std::to_string(freshness.freshness_score) +
                           " 颜色=" + std::to_string(color_score) +
                           " 纹理=" + std::to_string(freshness.texture_score);

    OVF_INFO() << "Freshness detection: score=" << freshness.freshness_score;
    return ErrorCode::Success;
}

ErrorCode detect_contaminates(const ImageData& image, Vector<ContaminateInfo>& contaminates,
                             float sensitivity) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    contaminates.clear();

    // 异物检测：检测异常对比度区域
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 10, 5.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    // 检测高对比度异常（异物特征）
    Vector<uint8_t> anomaly(w * h, 0);
    float anomaly_threshold = sensitivity * 50;

    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > anomaly_threshold) {
            anomaly[i] = 255;
        }
    }

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(anomaly.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    for (const auto& comp : comps) {
        if (comp.area >= 5 && comp.area < 500) {
            ContaminateInfo contaminate;

            // 根据形状和大小判断异物类型
            float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
            float area_ratio = static_cast<float>(comp.area) / (comp.width * comp.height + 1);

            if (aspect_ratio > 5.0f || aspect_ratio < 0.2f) {
                // 细长形：毛发
                contaminate.type = ContaminateType::Hair;
            } else if (comp.area < 20) {
                // 小颗粒：灰尘
                contaminate.type = ContaminateType::Dust;
            } else if (area_ratio > 0.8f && comp.area < 100) {
                // 规则小物体：昆虫或塑料
                contaminate.type = ContaminateType::Insect;
            } else if (area_ratio < 0.5f) {
                // 不规则：其他异物
                contaminate.type = ContaminateType::Other;
            } else {
                // 中等大小：塑料或玻璃
                contaminate.type = ContaminateType::Plastic;
            }

            contaminate.x = comp.x;
            contaminate.y = comp.y;
            contaminate.width = comp.width;
            contaminate.height = comp.height;
            contaminate.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            contaminate.confidence = 0.75f;
            contaminate.description = contaminate_type_name(contaminate.type) +
                                     " @(" + std::to_string(contaminate.x) + "," +
                                     std::to_string(contaminate.y) + ") 面积=" +
                                     std::to_string(comp.area);

            contaminates.push_back(contaminate);
        }
    }

    OVF_INFO() << "Contaminate detection: found " << contaminates.size() << " contaminates";
    return ErrorCode::Success;
}

ErrorCode detect_fruit_quality(const ImageData& image, FruitQualityInfo& quality,
                              float maturity_threshold) {
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

    // 成熟度检测：基于颜色（亮度）
    float mean = compute_image_mean(gray);
    quality.maturity = std::clamp(mean / 255.0f, 0.0f, 1.0f);
    quality.is_mature = (quality.maturity >= maturity_threshold);

    // 损伤检测：检测异常区域
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 10, 5.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    // 统计损伤区域
    int damage_count = 0;
    int bruise_count = 0;

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            float local_mean = compute_local_mean(gray, w, h, x, y, 5);
            float local_std = compute_local_std(gray, w, h, x, y, 5, local_mean);

            if (diff[y * w + x] > 40) {
                damage_count++;
            }
            if (local_std > 30) {
                bruise_count++;
            }
        }
    }

    quality.damage_score = std::clamp(static_cast<float>(damage_count) / (w * h + 1) * 10, 0.0f, 1.0f);
    quality.has_damage = (quality.damage_score > 0.1f);

    quality.has_bruise = (static_cast<float>(bruise_count) / (w * h + 1) > 0.05f);

    // 综合质量评分
    quality.quality_score = quality.maturity * 0.4f +
                           (1.0f - quality.damage_score) * 0.3f +
                           (quality.has_bruise ? 0.0f : 1.0f) * 0.3f;

    quality.confidence = std::clamp(quality.quality_score, 0.0f, 1.0f);
    quality.description = "水果质量: 成熟度=" + std::to_string(quality.maturity) +
                         " 损伤=" + (quality.has_damage ? "有" : "无") +
                         " 碰伤=" + (quality.has_bruise ? "有" : "无") +
                         " 质量=" + std::to_string(quality.quality_score);

    OVF_INFO() << "Fruit quality: maturity=" << quality.maturity;
    return ErrorCode::Success;
}

ErrorCode inspect_bottle(const ImageData& image, BottleInspectInfo& bottle,
                        float min_liquid_level) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    bottle.x = 0;
    bottle.y = 0;
    bottle.width = w;
    bottle.height = h;

    // 液位检测
    int upper_region_h = h / 3;
    float upper_mean = compute_region_color(gray, w, h, 0, 0, w, upper_region_h);
    float lower_mean = compute_region_color(gray, w, h, 0, h - upper_region_h, w, upper_region_h);

    float brightness_diff = std::abs(lower_mean - upper_mean);
    bottle.liquid_level = std::clamp(brightness_diff / 50.0f, 0.0f, 1.0f);

    // 标签检测
    int label_x = w / 3;
    int label_y = h / 3;
    int label_w = w / 3;
    int label_h = h / 3;

    float label_mean = compute_region_color(gray, w, h, label_x, label_y, label_w, label_h);
    float background_mean = compute_image_mean(gray);

    bottle.label_present = (std::abs(label_mean - background_mean) > 15);

    // 标签对齐度
    float center_offset_x = std::abs(label_x + label_w/2 - w/2) / static_cast<float>(w);
    float center_offset_y = std::abs(label_y + label_h/2 - h/2) / static_cast<float>(h);
    bottle.label_alignment = 1.0f - std::clamp((center_offset_x + center_offset_y) / 2, 0.0f, 1.0f);

    bottle.label_correct = bottle.label_present && (bottle.label_alignment > 0.7f);

    // 密封检测
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    // 检测瓶口区域边缘
    int cap_region_h = h / 10;
    int edge_count = 0;
    for (int y = 0; y < cap_region_h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (edge[y * w + x] > 0) edge_count++;
        }
    }

    float edge_density = static_cast<float>(edge_count) / (cap_region_h * w + 1);
    bottle.seal_quality = std::clamp(edge_density, 0.0f, 1.0f);
    bottle.is_sealed = (bottle.seal_quality > 0.2f);

    bottle.confidence = std::clamp(bottle.liquid_level * 0.3f +
                                  (bottle.label_present ? 0.2f : 0.0f) +
                                  bottle.label_alignment * 0.2f +
                                  bottle.seal_quality * 0.3f, 0.0f, 1.0f);

    bottle.description = "瓶装检测: 液位=" + std::to_string(bottle.liquid_level * 100) + "%" +
                         " 标签=" + (bottle.label_present ? "有" : "无") +
                         " 密封=" + (bottle.is_sealed ? "合格" : "不合格");

    OVF_INFO() << "Bottle inspect: liquid=" << bottle.liquid_level;
    return ErrorCode::Success;
}

ErrorCode detect_meat_quality(const ImageData& image, MeatQualityInfo& quality,
                             float min_quality) {
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

    // 颜色评分（肉质颜色）
    float mean = compute_image_mean(gray);
    float std = compute_image_std(gray, mean);

    // 新鲜肉类有特定颜色范围
    quality.color_score = std::clamp(std / 40.0f, 0.0f, 1.0f);

    // 纹理评分
    Vector<float> local_stds;
    int block_size = 16;

    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            float block_mean = compute_local_mean(gray, w, h, x + block_size/2, y + block_size/2, block_size/2);
            float block_std = compute_local_std(gray, w, h, x + block_size/2, y + block_size/2, block_size/2, block_mean);
            local_stds.push_back(block_std);
        }
    }

    if (!local_stds.empty()) {
        float avg_std = std::accumulate(local_stds.begin(), local_stds.end(), 0.0f) / local_stds.size();
        quality.texture_score = std::clamp(avg_std / 30.0f, 0.0f, 1.0f);
    } else {
        quality.texture_score = 0.0f;
    }

    // 新鲜度（基于颜色和纹理）
    quality.freshness = quality.color_score * 0.5f + quality.texture_score * 0.5f;

    // 大理石纹（肥瘦分布）
    // 分析亮度变化周期性
    int transitions = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 1; x < w; ++x) {
            if (std::abs(gray[y * w + x] - gray[y * w + x - 1]) > 20) {
                transitions++;
            }
        }
    }

    quality.marbling = std::clamp(static_cast<float>(transitions) / (w * h + 1), 0.0f, 1.0f);

    // 综合质量评分
    quality.quality_grade = quality.color_score * 0.3f +
                           quality.texture_score * 0.3f +
                           quality.freshness * 0.2f +
                           quality.marbling * 0.2f;

    quality.confidence = std::clamp(quality.quality_grade, 0.0f, 1.0f);
    quality.description = "肉类质量: 颜色=" + std::to_string(quality.color_score) +
                         " 纹理=" + std::to_string(quality.texture_score) +
                         " 新鲜度=" + std::to_string(quality.freshness) +
                         " 大理石纹=" + std::to_string(quality.marbling);

    OVF_INFO() << "Meat quality: grade=" << quality.quality_grade;
    return ErrorCode::Success;
}

ErrorCode check_package_integrity(const ImageData& image, PackageIntegrityInfo& integrity,
                                 float min_integrity) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    integrity.x = 0;
    integrity.y = 0;
    integrity.width = w;
    integrity.height = h;

    // 损坏检测（边缘异常）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);

    int edge_count = 0;
    for (int i = 0; i < w * h; ++i) {
        if (edge[i] > 0) edge_count++;
    }

    float edge_ratio = static_cast<float>(edge_count) / (w * h + 1);
    integrity.has_damage = (edge_ratio > 0.15f);

    // 泄漏检测（亮度不均）
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 15, 7.0f);

    int leak_count = 0;
    for (int i = 0; i < w * h; ++i) {
        if (std::abs(gray[i] - smooth[i]) > 50) {
            leak_count++;
        }
    }

    integrity.has_leak = (static_cast<float>(leak_count) / (w * h + 1) > 0.05f);

    // 变形检测（形状不规则）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 20);

    Vector<uint8_t> temp = binary;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    for (const auto& comp : comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        if (aspect_ratio > 2.5f || aspect_ratio < 0.4f) {
            if (comp.area > w * h * 0.2f) {
                integrity.has_deformation = true;
                break;
            }
        }
    }

    // 综合完整性评分
    integrity.integrity_score = 1.0f -
                               (integrity.has_damage ? 0.3f : 0.0f) -
                               (integrity.has_leak ? 0.4f : 0.0f) -
                               (integrity.has_deformation ? 0.3f : 0.0f);

    integrity.integrity_score = std::clamp(integrity.integrity_score, 0.0f, 1.0f);
    integrity.is_complete = (integrity.integrity_score >= min_integrity);

    integrity.confidence = std::clamp(integrity.integrity_score, 0.0f, 1.0f);
    integrity.description = "包装完整性: 评分=" + std::to_string(integrity.integrity_score) +
                           " 损坏=" + (integrity.has_damage ? "有" : "无") +
                           " 泄漏=" + (integrity.has_leak ? "有" : "无") +
                           " 变形=" + (integrity.has_deformation ? "有" : "无");

    OVF_INFO() << "Package integrity: score=" << integrity.integrity_score;
    return ErrorCode::Success;
}

} // namespace food_utils

// ========== 纺织行业检测模块实现 ==========

namespace textile_utils {

ErrorCode detect_fabric_defects(const ImageData& image, Vector<FabricDefectInfo>& defects,
                               float sensitivity) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    defects.clear();

    // 1. 断纱检测 - 检测细长线条
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    Vector<uint8_t> temp1 = edge;
    Vector<ConnectedComponent> edge_comps = find_connected_components(temp1, w, h);

    float defect_threshold = sensitivity * 30;
    for (const auto& comp : edge_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);

        // 断纱：细长形线条
        if ((aspect_ratio > 3.0f || aspect_ratio < 0.33f) && comp.area > defect_threshold) {
            FabricDefectInfo defect;
            defect.type = FabricDefectType::BrokenYarn;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            defect.confidence = 0.8f;
            defect.description = fabric_defect_type_name(defect.type) +
                                " 长度=" + std::to_string(std::max(comp.width, comp.height));
            defects.push_back(defect);
        }
    }

    // 2. 污渍检测 - 检测局部对比度异常
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 8, 4.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    Vector<uint8_t> stain_mask(w * h, 0);
    float stain_threshold = sensitivity * 40;
    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > stain_threshold) {
            stain_mask[i] = 255;
        }
    }

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(stain_mask.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp2 = cleaned;
    Vector<ConnectedComponent> stain_comps = find_connected_components(temp2, w, h);

    for (const auto& comp : stain_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        // 污渍：非细长形
        if (aspect_ratio < 3.0f && aspect_ratio > 0.33f && comp.area >= 20) {
            FabricDefectInfo defect;
            defect.type = FabricDefectType::Stain;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 200.0f, 0.0f, 1.0f);
            defect.confidence = 0.75f;
            defect.description = fabric_defect_type_name(defect.type) +
                                " 面积=" + std::to_string(comp.area);
            defects.push_back(defect);
        }
    }

    // 3. 孔洞检测 - 检测低亮度小区域
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 21, 10);

    // 反转检测暗区域
    Vector<uint8_t> inverted(w * h);
    for (int i = 0; i < w * h; ++i) {
        inverted[i] = (binary[i] == 0) ? 255 : 0;
    }

    Vector<uint8_t> temp3 = inverted;
    Vector<ConnectedComponent> hole_comps = find_connected_components(temp3, w, h);

    for (const auto& comp : hole_comps) {
        if (comp.area >= 10 && comp.area < 500) {
            FabricDefectInfo defect;
            defect.type = FabricDefectType::Hole;
            defect.x = comp.x;
            defect.y = comp.y;
            defect.width = comp.width;
            defect.height = comp.height;
            defect.severity = std::clamp(static_cast<float>(comp.area) / 100.0f, 0.0f, 1.0f);
            defect.confidence = 0.85f;
            defect.description = fabric_defect_type_name(defect.type) +
                                " 尺寸=" + std::to_string(comp.width) + "x" + std::to_string(comp.height);
            defects.push_back(defect);
        }
    }

    OVF_INFO() << "Fabric defect detection: found " << defects.size() << " defects";
    return ErrorCode::Success;
}

ErrorCode detect_fabric_pattern(const ImageData& image, FabricPatternInfo& pattern,
                               float match_threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    pattern.x = 0;
    pattern.y = 0;
    pattern.width = w;
    pattern.height = h;

    // 分析图案周期性
    // 统计亮度分布
    Vector<int> row_means(h);
    Vector<int> col_means(w);

    for (int y = 0; y < h; ++y) {
        float sum = 0.0f;
        for (int x = 0; x < w; ++x) {
            sum += gray[y * w + x];
        }
        row_means[y] = static_cast<int>(sum / w);
    }

    for (int x = 0; x < w; ++x) {
        float sum = 0.0f;
        for (int y = 0; y < h; ++y) {
            sum += gray[y * w + x];
        }
        col_means[x] = static_cast<int>(sum / h);
    }

    // 计算周期性（基于重复模式）
    int row_period = 0, col_period = 0;

    // 检测行周期
    for (int p = 10; p < h / 4; ++p) {
        int matches = 0;
        for (int y = 0; y < h - p; ++y) {
            if (std::abs(row_means[y] - row_means[y + p]) < 5) {
                matches++;
            }
        }
        if (matches > h * 0.7f) {
            row_period = p;
            break;
        }
    }

    // 检测列周期
    for (int p = 10; p < w / 4; ++p) {
        int matches = 0;
        for (int x = 0; x < w - p; ++x) {
            if (std::abs(col_means[x] - col_means[x + p]) < 5) {
                matches++;
            }
        }
        if (matches > w * 0.7f) {
            col_period = p;
            break;
        }
    }

    // 判断图案匹配度
    if (row_period > 0 || col_period > 0) {
        pattern.pattern_match = std::clamp((row_period + col_period) / 100.0f, 0.0f, 1.0f);
        pattern.pattern_type = "周期性图案";
    } else {
        // 检测纹理密度
        float std = compute_image_std(gray, compute_image_mean(gray));
        pattern.pattern_match = std::clamp(std / 50.0f, 0.0f, 1.0f);
        pattern.pattern_type = "随机纹理";
    }

    pattern.is_matched = (pattern.pattern_match >= match_threshold);
    pattern.deviation = 1.0f - pattern.pattern_match;
    pattern.confidence = std::clamp(pattern.pattern_match, 0.0f, 1.0f);
    pattern.description = "织物图案: 类型=" + pattern.pattern_type +
                         " 匹配度=" + std::to_string(pattern.pattern_match);

    OVF_INFO() << "Fabric pattern: match=" << pattern.pattern_match;
    return ErrorCode::Success;
}

ErrorCode detect_fabric_color(const ImageData& image, FabricColorInfo& color,
                             float max_deviation) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    color.x = 0;
    color.y = 0;
    color.width = w;
    color.height = h;

    // 计算整体颜色均值
    float global_mean = compute_image_mean(gray);

    // 分析区域颜色差异
    int block_size = 32;
    Vector<float> block_means;

    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            float block_mean = compute_region_color(gray, w, h, x, y, block_size, block_size);
            block_means.push_back(block_mean);
        }
    }

    if (!block_means.empty()) {
        // 计算颜色偏差（最大偏差）
        float max_diff = 0.0f;
        for (float bm : block_means) {
            float diff = std::abs(bm - global_mean);
            if (diff > max_diff) max_diff = diff;
        }
        color.color_deviation = max_diff;
        color.has_color_diff = (color.color_deviation > max_deviation);

        // 计算染色均匀度（方差）
        float variance = 0.0f;
        for (float bm : block_means) {
            variance += (bm - global_mean) * (bm - global_mean);
        }
        variance /= block_means.size();

        color.uniformity = std::clamp(1.0f - variance / 500.0f, 0.0f, 1.0f);
        color.is_uniform = (color.uniformity > 0.8f);
    } else {
        color.color_deviation = 0.0f;
        color.uniformity = 1.0f;
        color.has_color_diff = false;
        color.is_uniform = true;
    }

    color.confidence = std::clamp(color.uniformity * 0.7f + (color.is_uniform ? 0.3f : 0.0f), 0.0f, 1.0f);
    color.description = "织物颜色: 色差=" + std::to_string(color.color_deviation) +
                       " 均匀度=" + std::to_string(color.uniformity);

    OVF_INFO() << "Fabric color: deviation=" << color.color_deviation;
    return ErrorCode::Success;
}

ErrorCode detect_fabric_density(const ImageData& image, FabricDensityInfo& density,
                               int expected_warp, int expected_weft) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    density.x = 0;
    density.y = 0;
    density.width = w;
    density.height = h;

    // 检测纱线密度（通过边缘检测）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 15);

    // 统计水平和垂直边缘（模拟经纬纱）
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

                if (std::abs(gx) > std::abs(gy)) {
                    horizontal_edges++; // 经纱方向
                } else {
                    vertical_edges++; // 纬纱方向
                }
            }
        }
    }

    // 估算密度（边缘数 / 单位长度）
    density.warp_density = horizontal_edges * 10 / w; // 每10像素估算
    density.weft_density = vertical_edges * 10 / h;

    // 计算密度均匀性
    float warp_ratio = static_cast<float>(density.warp_density) / (expected_warp + 1);
    float weft_ratio = static_cast<float>(density.weft_density) / (expected_weft + 1);

    density.density_uniformity = std::clamp(std::min(warp_ratio, weft_ratio), 0.0f, 1.0f);
    density.is_uniform = (density.density_uniformity > 0.8f &&
                         std::abs(density.warp_density - expected_warp) < 20 &&
                         std::abs(density.weft_density - expected_weft) < 20);

    density.confidence = std::clamp(density.density_uniformity, 0.0f, 1.0f);
    density.description = "织物密度: 经纱=" + std::to_string(density.warp_density) +
                         " 纬纱=" + std::to_string(density.weft_density) +
                         " 均匀性=" + std::to_string(density.density_uniformity);

    OVF_INFO() << "Fabric density: warp=" << density.warp_density << ", weft=" << density.weft_density;
    return ErrorCode::Success;
}

ErrorCode inspect_garment(const ImageData& image, GarmentInspectInfo& garment,
                         float min_quality) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    garment.x = 0;
    garment.y = 0;
    garment.width = w;
    garment.height = h;
    garment.defect_count = 0;

    // 缝线缺陷检测（检测线条中断）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 20);

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> connected(w * h);
    morph_utils::close(edge.data(), connected.data(), w, h, kernel.data(), 3, 3, 2);

    Vector<uint8_t> temp1 = connected;
    Vector<ConnectedComponent> line_comps = find_connected_components(temp1, w, h);

    // 检测缝线断裂
    int broken_lines = 0;
    for (const auto& comp : line_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        if (aspect_ratio > 3.0f || aspect_ratio < 0.33f) {
            // 细长线条，检测是否中断
            int length = std::max(comp.width, comp.height);
            if (comp.area < length * 3) {
                broken_lines++;
            }
        }
    }

    garment.has_stitch_defect = (broken_lines > 3);
    if (garment.has_stitch_defect) garment.defect_count++;

    // 织物缺陷检测（同fabric_defects）
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 8, 4.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    int defect_pixels = 0;
    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > 40) defect_pixels++;
    }

    garment.has_fabric_defect = (static_cast<float>(defect_pixels) / (w * h + 1) > 0.05f);
    if (garment.has_fabric_defect) garment.defect_count++;

    // 尺寸缺陷检测（形状不规则）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 20);

    Vector<uint8_t> temp2 = binary;
    Vector<ConnectedComponent> comps = find_connected_components(temp2, w, h);

    for (const auto& comp : comps) {
        if (comp.area > w * h * 0.3f) {
            float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
            if (aspect_ratio > 3.0f || aspect_ratio < 0.33f) {
                garment.has_size_defect = true;
                garment.defect_count++;
                break;
            }
        }
    }

    // 计算质量评分
    garment.quality_score = 1.0f -
                           (garment.has_stitch_defect ? 0.3f : 0.0f) -
                           (garment.has_fabric_defect ? 0.3f : 0.0f) -
                           (garment.has_size_defect ? 0.4f : 0.0f);

    garment.quality_score = std::clamp(garment.quality_score, 0.0f, 1.0f);
    garment.confidence = std::clamp(garment.quality_score, 0.0f, 1.0f);
    garment.description = "服装检测: 质量评分=" + std::to_string(garment.quality_score) +
                         " 缝线缺陷=" + (garment.has_stitch_defect ? "有" : "无") +
                         " 织物缺陷=" + (garment.has_fabric_defect ? "有" : "无");

    OVF_INFO() << "Garment inspect: quality=" << garment.quality_score;
    return ErrorCode::Success;
}

ErrorCode count_threads(const ImageData& image, ThreadCountInfo& count,
                        int expected_count) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    count.x = 0;
    count.y = 0;
    count.width = w;
    count.height = h;
    count.expected_count = expected_count;

    // 纱线计数：通过边缘检测统计
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 15);

    // 统计边缘像素作为纱线数量估算
    int edge_pixels = 0;
    for (int i = 0; i < w * h; ++i) {
        if (edge[i] > 0) edge_pixels++;
    }

    // 估算纱线数量（每10像素边缘算一条纱线）
    count.thread_count = edge_pixels / 10;

    // 计算偏差
    count.deviation = std::abs(count.thread_count - expected_count);
    count.is_correct = (count.deviation < expected_count * 0.1f);

    count.confidence = std::clamp(1.0f - count.deviation / (expected_count + 1), 0.0f, 1.0f);
    count.description = "纱线计数: 实际=" + std::to_string(count.thread_count) +
                       " 预期=" + std::to_string(expected_count) +
                       " 偏差=" + std::to_string(count.deviation);

    OVF_INFO() << "Thread count: actual=" << count.thread_count;
    return ErrorCode::Success;
}

} // namespace textile_utils

// ========== 汽车行业检测模块实现 ==========

namespace automotive_utils {

ErrorCode inspect_paint_quality(const ImageData& image, PaintQualityInfo& paint,
                               float min_score) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    paint.x = 0;
    paint.y = 0;
    paint.width = w;
    paint.height = h;
    paint.defect_count = 0;

    // 1. 划痕检测（细长线条）
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);

    Vector<uint8_t> temp1 = edge;
    Vector<ConnectedComponent> edge_comps = find_connected_components(temp1, w, h);

    for (const auto& comp : edge_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        if ((aspect_ratio > 5.0f || aspect_ratio < 0.2f) && comp.area > 50) {
            paint.has_scratch = true;
            paint.defect_count++;
            break;
        }
    }

    // 2. 气泡检测（小圆形亮点）
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 10, 5.0f);

    Vector<uint8_t> diff(w * h);
    for (int i = 0; i < w * h; ++i) {
        diff[i] = static_cast<uint8_t>(std::abs(gray[i] - smooth[i]));
    }

    Vector<uint8_t> bubble_mask(w * h, 0);
    for (int i = 0; i < w * h; ++i) {
        if (diff[i] > 50 && gray[i] > smooth[i]) {
            bubble_mask[i] = 255;
        }
    }

    Vector<uint8_t> kernel(25, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(bubble_mask.data(), cleaned.data(), w, h, kernel.data(), 5, 5, 1);

    Vector<uint8_t> temp2 = cleaned;
    Vector<ConnectedComponent> bubble_comps = find_connected_components(temp2, w, h);

    for (const auto& comp : bubble_comps) {
        float roundness = compute_roundness(comp.area, comp.width, comp.height);
        if (roundness > 0.7f && comp.area >= 10 && comp.area < 100) {
            paint.has_bubble = true;
            paint.defect_count++;
            break;
        }
    }

    // 3. 涂装脱落检测（大面积异常）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> temp3 = binary;
    Vector<ConnectedComponent> area_comps = find_connected_components(temp3, w, h);

    for (const auto& comp : area_comps) {
        if (comp.area > w * h * 0.05f && comp.area < w * h * 0.3f) {
            float mean_diff = std::abs(compute_region_color(gray, w, h, comp.x, comp.y, comp.width, comp.height) -
                                      compute_image_mean(gray));
            if (mean_diff > 40) {
                paint.has_peeling = true;
                paint.defect_count++;
                break;
            }
        }
    }

    // 4. 灰尘检测（小颗粒）
    for (const auto& comp : bubble_comps) {
        if (comp.area >= 5 && comp.area < 20) {
            paint.has_dust = true;
            paint.defect_count++;
            break;
        }
    }

    // 5. 色差检测
    float global_mean = compute_image_mean(gray);
    float global_std = compute_image_std(gray, global_mean);

    Vector<float> block_means;
    int block_size = 64;
    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            block_means.push_back(compute_region_color(gray, w, h, x, y, block_size, block_size));
        }
    }

    if (!block_means.empty()) {
        float max_diff = 0.0f;
        for (float bm : block_means) {
            float diff = std::abs(bm - global_mean);
            if (diff > max_diff) max_diff = diff;
        }
        paint.has_color_diff = (max_diff > 20);
        if (paint.has_color_diff) paint.defect_count++;
    }

    // 计算涂装质量评分
    paint.paint_score = 1.0f -
                        (paint.has_scratch ? 0.2f : 0.0f) -
                        (paint.has_bubble ? 0.15f : 0.0f) -
                        (paint.has_peeling ? 0.25f : 0.0f) -
                        (paint.has_dust ? 0.1f : 0.0f) -
                        (paint.has_color_diff ? 0.2f : 0.0f);

    paint.paint_score = std::clamp(paint.paint_score, 0.0f, 1.0f);
    paint.confidence = std::clamp(paint.paint_score, 0.0f, 1.0f);
    paint.description = "涂装质量: 评分=" + std::to_string(paint.paint_score) +
                        " 缺陷数=" + std::to_string(paint.defect_count);

    OVF_INFO() << "Paint quality: score=" << paint.paint_score;
    return ErrorCode::Success;
}

ErrorCode inspect_weld(const ImageData& image, WeldInspectInfo& weld,
                      float min_quality) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    weld.x = 0;
    weld.y = 0;
    weld.width = w;
    weld.height = h;

    // 焊缝检测：分析连续性和缺陷
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    // 检测焊点（圆形或椭圆形亮点）
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 8, 4.0f);

    Vector<uint8_t> weld_points(w * h, 0);
    for (int i = 0; i < w * h; ++i) {
        if (gray[i] > smooth[i] + 30) {
            weld_points[i] = 255;
        }
    }

    Vector<uint8_t> kernel(25, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(weld_points.data(), cleaned.data(), w, h, kernel.data(), 5, 5, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> point_comps = find_connected_components(temp, w, h);

    weld.weld_points = static_cast<int>(point_comps.size());

    // 1. 裂纹检测（焊缝中断）
    int continuous_segments = 0;
    for (const auto& comp : point_comps) {
        if (comp.area >= 20) {
            continuous_segments++;
        }
    }

    weld.is_continuous = (continuous_segments >= 3);

    // 2. 气孔检测（小暗点）
    Vector<uint8_t> inverted(w * h);
    for (int i = 0; i < w * h; ++i) {
        inverted[i] = (gray[i] < smooth[i] - 40) ? 255 : 0;
    }

    Vector<uint8_t> temp2 = inverted;
    Vector<ConnectedComponent> pore_comps = find_connected_components(temp2, w, h);

    for (const auto& comp : pore_comps) {
        if (comp.area >= 5 && comp.area < 30) {
            weld.has_porosity = true;
            break;
        }
    }

    // 3. 咬边检测（边缘凹陷）
    int edge_breaks = 0;
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            if (edge[y * w + x] > 0) {
                // 检测边缘中断
                int neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (edge[(y+dy)*w + (x+dx)] > 0) neighbors++;
                    }
                }
                if (neighbors < 3) edge_breaks++;
            }
        }
    }

    weld.has_undercut = (edge_breaks > w * h * 0.01f);

    // 4. 裂纹检测
    Vector<uint8_t> temp3 = edge;
    Vector<ConnectedComponent> crack_comps = find_connected_components(temp3, w, h);

    for (const auto& comp : crack_comps) {
        float aspect_ratio = static_cast<float>(comp.width) / (comp.height + 1);
        if ((aspect_ratio > 4.0f || aspect_ratio < 0.25f) && comp.area > 30) {
            weld.has_crack = true;
            break;
        }
    }

    // 计算焊缝质量
    weld.weld_quality = 1.0f -
                        (weld.has_crack ? 0.3f : 0.0f) -
                        (weld.has_porosity ? 0.2f : 0.0f) -
                        (weld.has_undercut ? 0.2f : 0.0f) -
                        (weld.is_continuous ? 0.0f : 0.3f);

    weld.weld_quality = std::clamp(weld.weld_quality, 0.0f, 1.0f);
    weld.confidence = std::clamp(weld.weld_quality, 0.0f, 1.0f);
    weld.description = "焊缝检测: 质量=" + std::to_string(weld.weld_quality) +
                      " 焊点数=" + std::to_string(weld.weld_points) +
                      " 连续=" + (weld.is_continuous ? "是" : "否");

    OVF_INFO() << "Weld inspect: quality=" << weld.weld_quality;
    return ErrorCode::Success;
}

ErrorCode check_panel_gap(const ImageData& image, PanelGapInfo& gap,
                         float expected_gap, float tolerance) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    gap.x = 0;
    gap.y = 0;
    gap.width = w;
    gap.height = h;

    // 钣金间隙检测：检测两条边缘之间的距离
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 30);

    // 检测间隙区域（两条边缘之间的暗区域）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 21, 10);

    // 检测间隙线条
    Vector<int> gap_positions;
    for (int y = 0; y < h; ++y) {
        int edge_count = 0;
        int first_edge = -1;
        int last_edge = -1;

        for (int x = 0; x < w; ++x) {
            if (edge[y * w + x] > 0) {
                edge_count++;
                if (first_edge < 0) first_edge = x;
                last_edge = x;
            }
        }

        if (edge_count >= 2 && first_edge >= 0 && last_edge >= 0) {
            int gap_width = last_edge - first_edge;
            if (gap_width >= 2 && gap_width <= 50) {
                gap_positions.push_back(gap_width);
            }
        }
    }

    if (!gap_positions.empty()) {
        float avg_gap = std::accumulate(gap_positions.begin(), gap_positions.end(), 0.0f) /
                       gap_positions.size();

        gap.gap_size = avg_gap;
        gap.deviation = std::abs(avg_gap - expected_gap);

        // 计算间隙均匀度
        float variance = 0.0f;
        for (int g : gap_positions) {
            variance += (g - avg_gap) * (g - avg_gap);
        }
        variance /= gap_positions.size();

        gap.gap_uniformity = std::clamp(1.0f - variance / 100.0f, 0.0f, 1.0f);
        gap.is_uniform = (gap.gap_uniformity > 0.8f && gap.deviation <= tolerance);
    } else {
        gap.gap_size = 0.0f;
        gap.gap_uniformity = 0.0f;
        gap.deviation = expected_gap;
        gap.is_uniform = false;
    }

    gap.confidence = std::clamp(gap.gap_uniformity * 0.5f +
                               (gap.is_uniform ? 0.5f : 0.0f), 0.0f, 1.0f);
    gap.description = "钣金间隙: 大小=" + std::to_string(gap.gap_size) +
                     " 均匀度=" + std::to_string(gap.gap_uniformity) +
                     " 偏差=" + std::to_string(gap.deviation);

    OVF_INFO() << "Panel gap: size=" << gap.gap_size;
    return ErrorCode::Success;
}

ErrorCode check_part_presence(const ImageData& image, PartPresenceInfo& presence,
                             int expected_parts) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    presence.x = 0;
    presence.y = 0;
    presence.width = w;
    presence.height = h;
    presence.total_parts = expected_parts;

    // 零部件检测：通过连通区域分析
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> kernel(25, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(binary.data(), cleaned.data(), w, h, kernel.data(), 5, 5, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    // 过滤有效零部件（根据尺寸）
    presence.present_parts = 0;
    int min_part_size = static_cast<int>(std::sqrt(w * h / expected_parts) / 2);
    int max_part_size = static_cast<int>(std::sqrt(w * h) * 2);

    for (const auto& comp : comps) {
        if (comp.width >= min_part_size && comp.height >= min_part_size &&
            comp.area >= min_part_size * min_part_size &&
            comp.area <= max_part_size * max_part_size) {
            presence.present_parts++;
        }
    }

    presence.missing_parts = expected_parts - presence.present_parts;
    presence.is_complete = (presence.missing_parts == 0);

    presence.confidence = std::clamp(static_cast<float>(presence.present_parts) /
                                    (expected_parts + 1), 0.0f, 1.0f);
    presence.description = "零部件检测: 存在=" + std::to_string(presence.present_parts) +
                          " 缺失=" + std::to_string(presence.missing_parts);

    OVF_INFO() << "Part presence: " << presence.present_parts << "/" << expected_parts;
    return ErrorCode::Success;
}

ErrorCode check_surface_roughness(const ImageData& image, SurfaceRoughnessInfo& roughness,
                                 float max_roughness) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    roughness.x = 0;
    roughness.y = 0;
    roughness.width = w;
    roughness.height = h;

    // 表面粗糙度检测：通过纹理分析
    // 计算局部标准差
    Vector<float> local_stds;
    int block_size = 8;

    for (int y = 0; y < h - block_size; y += block_size) {
        for (int x = 0; x < w - block_size; x += block_size) {
            float block_mean = compute_local_mean(gray, w, h, x + block_size/2, y + block_size/2, block_size/2);
            float block_std = compute_local_std(gray, w, h, x + block_size/2, y + block_size/2, block_size/2, block_mean);
            local_stds.push_back(block_std);
        }
    }

    if (!local_stds.empty()) {
        // Ra值（平均粗糙度）
        roughness.roughness_ra = std::accumulate(local_stds.begin(), local_stds.end(), 0.0f) /
                                local_stds.size();

        // Rz值（最大粗糙度）
        float max_std = 0.0f;
        for (float s : local_stds) {
            if (s > max_std) max_std = s;
        }
        roughness.roughness_rz = max_std;

        roughness.is_smooth = (roughness.roughness_ra <= max_roughness);
    } else {
        roughness.roughness_ra = 0.0f;
        roughness.roughness_rz = 0.0f;
        roughness.is_smooth = true;
    }

    roughness.confidence = std::clamp(1.0f - roughness.roughness_ra / (max_roughness + 1), 0.0f, 1.0f);
    roughness.description = "表面粗糙度: Ra=" + std::to_string(roughness.roughness_ra) +
                           " Rz=" + std::to_string(roughness.roughness_rz) +
                           " 光滑=" + (roughness.is_smooth ? "是" : "否");

    OVF_INFO() << "Surface roughness: Ra=" << roughness.roughness_ra;
    return ErrorCode::Success;
}

ErrorCode check_dimension(const ImageData& image, DimensionCheckInfo& dimension,
                         float expected_value, float tolerance) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    dimension.x = 0;
    dimension.y = 0;
    dimension.width = w;
    dimension.height = h;
    dimension.expected_value = expected_value;
    dimension.tolerance = tolerance;

    // 尺寸检测：检测孔径或间距
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    // 检测圆形结构（孔）
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 21, 10);

    Vector<uint8_t> inverted(w * h);
    for (int i = 0; i < w * h; ++i) {
        inverted[i] = (binary[i] == 0) ? 255 : 0;
    }

    Vector<uint8_t> kernel(9, 255);
    Vector<uint8_t> cleaned(w * h);
    morph_utils::open(inverted.data(), cleaned.data(), w, h, kernel.data(), 3, 3, 1);

    Vector<uint8_t> temp = cleaned;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    // 寻找圆形结构（孔）
    float measured_diameter = 0.0f;
    for (const auto& comp : comps) {
        float roundness = compute_roundness(comp.area, comp.width, comp.height);
        if (roundness > 0.7f) {
            // 计算直径
            measured_diameter = (comp.width + comp.height) / 2.0f;
            dimension.x = comp.x;
            dimension.y = comp.y;
            dimension.width = comp.width;
            dimension.height = comp.height;
            dimension.dimension_type = "孔径";
            break;
        }
    }

    // 如果没找到孔，检测间距（两条边缘之间的距离）
    if (measured_diameter == 0.0f) {
        Vector<int> distances;
        for (int y = 0; y < h; ++y) {
            int first_edge = -1;
            int last_edge = -1;

            for (int x = 0; x < w; ++x) {
                if (edge[y * w + x] > 0) {
                    if (first_edge < 0) first_edge = x;
                    last_edge = x;
                }
            }

            if (first_edge >= 0 && last_edge >= 0 && last_edge > first_edge) {
                distances.push_back(last_edge - first_edge);
            }
        }

        if (!distances.empty()) {
            measured_diameter = std::accumulate(distances.begin(), distances.end(), 0.0f) /
                               distances.size();
            dimension.dimension_type = "间距";
        }
    }

    dimension.measured_value = measured_diameter;
    dimension.deviation = std::abs(measured_diameter - expected_value);
    dimension.is_correct = (dimension.deviation <= tolerance);

    dimension.confidence = std::clamp(1.0f - dimension.deviation / (tolerance + 1), 0.0f, 1.0f);
    dimension.description = "尺寸检测: 类型=" + dimension.dimension_type +
                           " 测量值=" + std::to_string(dimension.measured_value) +
                           " 偏差=" + std::to_string(dimension.deviation);

    OVF_INFO() << "Dimension check: measured=" << dimension.measured_value;
    return ErrorCode::Success;
}

ErrorCode verify_assembly(const ImageData& image, AssemblyVerifyInfo& assembly,
                         float min_score) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    int w, h;
    Vector<uint8_t> gray;
    to_gray_image(image, gray, w, h);

    assembly.x = 0;
    assembly.y = 0;
    assembly.width = w;
    assembly.height = h;
    assembly.check_items = 5; // 默认检查5项
    assembly.pass_items = 0;
    assembly.fail_items = 0;

    // 组装验证：多项检查
    // 1. 连接完整性
    Vector<uint8_t> edge;
    sobel_edge(gray, edge, w, h, 25);

    int edge_count = 0;
    for (int i = 0; i < w * h; ++i) {
        if (edge[i] > 0) edge_count++;
    }

    bool has_good_connections = (edge_count > w * h * 0.05f);
    if (has_good_connections) assembly.pass_items++;
    else assembly.fail_items++;

    // 2. 对齐检测
    float global_mean = compute_image_mean(gray);
    Vector<float> quadrant_means(4);

    quadrant_means[0] = compute_region_color(gray, w, h, 0, 0, w/2, h/2);
    quadrant_means[1] = compute_region_color(gray, w, h, w/2, 0, w/2, h/2);
    quadrant_means[2] = compute_region_color(gray, w, h, 0, h/2, w/2, h/2);
    quadrant_means[3] = compute_region_color(gray, w, h, w/2, h/2, w/2, h/2);

    float max_diff = 0.0f;
    for (float qm : quadrant_means) {
        float diff = std::abs(qm - global_mean);
        if (diff > max_diff) max_diff = diff;
    }

    bool is_aligned = (max_diff < 20);
    if (is_aligned) assembly.pass_items++;
    else assembly.fail_items++;

    // 3. 缺失检测
    Vector<uint8_t> binary;
    adaptive_threshold(gray, binary, w, h, 31, 15);

    Vector<uint8_t> temp = binary;
    Vector<ConnectedComponent> comps = find_connected_components(temp, w, h);

    bool has_no_missing = (comps.size() >= 3);
    if (has_no_missing) assembly.pass_items++;
    else assembly.fail_items++;

    // 4. 表面质量
    float std = compute_image_std(gray, global_mean);
    bool has_good_surface = (std < 40);
    if (has_good_surface) assembly.pass_items++;
    else assembly.fail_items++;

    // 5. 组装紧密性
    Vector<uint8_t> smooth;
    gaussian_filter(gray, smooth, w, h, 10, 5.0f);

    int defect_pixels = 0;
    for (int i = 0; i < w * h; ++i) {
        if (std::abs(gray[i] - smooth[i]) > 30) defect_pixels++;
    }

    bool is_tight = (static_cast<float>(defect_pixels) / (w * h + 1) < 0.05f);
    if (is_tight) assembly.pass_items++;
    else assembly.fail_items++;

    // 计算组装评分
    assembly.assembly_score = static_cast<float>(assembly.pass_items) / assembly.check_items;
    assembly.is_complete = (assembly.fail_items == 0);
    assembly.is_correct = (assembly.assembly_score >= min_score);

    assembly.confidence = std::clamp(assembly.assembly_score, 0.0f, 1.0f);
    assembly.description = "组装验证: 评分=" + std::to_string(assembly.assembly_score) +
                          " 通过=" + std::to_string(assembly.pass_items) +
                          "/" + std::to_string(assembly.check_items);

    OVF_INFO() << "Assembly verify: score=" << assembly.assembly_score;
    return ErrorCode::Success;
}

} // namespace automotive_utils

// ========== 医药行业检测节点实现 ==========

PillDetectNode::PillDetectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> PillDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_roundness = static_cast<float>(get_param("min_roundness", Data(0.7f)).as_number());
    int min_size = get_param("min_size", Data(20)).as_int();

    Vector<pharma_utils::PillInfo> pills;
    auto result = pharma_utils::detect_pills(input, pills, min_roundness, min_size);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "药片检测失败");
    }

    set_output("pills", Data("PillInfo[" + std::to_string(pills.size()) + "]"));
    set_output("count", Data(static_cast<int>(pills.size())));

    String summary = "检测到 " + std::to_string(pills.size()) + " 个药片";
    set_output("summary", Data(summary));

    OVF_INFO() << "PillDetectNode: " << summary;
    return Result<void>::success();
}

NodeInfo PillDetectNode::make_info() {
    NodeInfo info;
    info.id = "PillDetect";
    info.name = "药片检测";
    info.category = "医药行业";
    info.description = "药片检测（数量、形状、颜色）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("pills", "药片信息", DataType::String));
    info.outputs.push_back(DataPort("count", "药片数量", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_roundness", "最小圆度", DataType::Number, Data(0.7f)));
    info.params.push_back(ParamDef("min_size", "最小尺寸", DataType::Number, Data(20)));
    return info;
}

OVF_REGISTER_NODE(PillDetectNode, "PillDetect", PillDetectNode::make_info())

PillDefectNode::PillDefectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> PillDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float sensitivity = static_cast<float>(get_param("sensitivity", Data(0.5f)).as_number());

    Vector<pharma_utils::PillDefectInfo> defects;
    auto result = pharma_utils::detect_pill_defects(input, defects, sensitivity);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "药片缺陷检测失败");
    }

    set_output("defects", Data("PillDefectInfo[" + std::to_string(defects.size()) + "]"));
    set_output("defect_count", Data(static_cast<int>(defects.size())));

    bool has_defect = !defects.empty();
    set_output("has_defect", Data(has_defect));

    String summary = has_defect ? "发现 " + std::to_string(defects.size()) + " 个缺陷" : "无缺陷";
    set_output("summary", Data(summary));

    OVF_INFO() << "PillDefectNode: " << summary;
    return Result<void>::success();
}

NodeInfo PillDefectNode::make_info() {
    NodeInfo info;
    info.id = "PillDefect";
    info.name = "药片缺陷检测";
    info.category = "医药行业";
    info.description = "药片缺陷检测（裂纹、斑点、变形）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷信息", DataType::String));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("has_defect", "是否有缺陷", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("sensitivity", "灵敏度", DataType::Number, Data(0.5f)));
    return info;
}

OVF_REGISTER_NODE(PillDefectNode, "PillDefect", PillDefectNode::make_info())

CapsuleInspectNode::CapsuleInspectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> CapsuleInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_fill_ratio = static_cast<float>(get_param("min_fill_ratio", Data(0.8f)).as_number());

    Vector<pharma_utils::CapsuleInfo> capsules;
    auto result = pharma_utils::detect_capsules(input, capsules, min_fill_ratio);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "胶囊检测失败");
    }

    set_output("capsules", Data("CapsuleInfo[" + std::to_string(capsules.size()) + "]"));
    set_output("count", Data(static_cast<int>(capsules.size())));

    int good_count = 0;
    for (const auto& cap : capsules) {
        if (cap.is_complete && cap.is_filled) good_count++;
    }
    set_output("good_count", Data(good_count));

    String summary = "检测到 " + std::to_string(capsules.size()) + " 个胶囊，合格 " + std::to_string(good_count);
    set_output("summary", Data(summary));

    OVF_INFO() << "CapsuleInspectNode: " << summary;
    return Result<void>::success();
}

NodeInfo CapsuleInspectNode::make_info() {
    NodeInfo info;
    info.id = "CapsuleInspect";
    info.name = "胶囊检测";
    info.category = "医药行业";
    info.description = "胶囊检测（完整性、填充度）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("capsules", "胶囊信息", DataType::String));
    info.outputs.push_back(DataPort("count", "胶囊数量", DataType::Number));
    info.outputs.push_back(DataPort("good_count", "合格数量", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_fill_ratio", "最小填充度", DataType::Number, Data(0.8f)));
    return info;
}

OVF_REGISTER_NODE(CapsuleInspectNode, "CapsuleInspect", CapsuleInspectNode::make_info())

SyringeCheckNode::SyringeCheckNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> SyringeCheckNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    int min_needle_length = get_param("min_needle_length", Data(20)).as_int();

    Vector<pharma_utils::SyringeInfo> syringes;
    auto result = pharma_utils::detect_syringes(input, syringes, min_needle_length);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "注射器检测失败");
    }

    set_output("syringes", Data("SyringeInfo[" + std::to_string(syringes.size()) + "]"));
    set_output("count", Data(static_cast<int>(syringes.size())));

    int qualified = 0;
    for (const auto& syr : syringes) {
        if (syr.needle_present && syr.scale_visible) qualified++;
    }
    set_output("qualified", Data(qualified));

    String summary = "检测到 " + std::to_string(syringes.size()) + " 个注射器，合格 " + std::to_string(qualified);
    set_output("summary", Data(summary));

    OVF_INFO() << "SyringeCheckNode: " << summary;
    return Result<void>::success();
}

NodeInfo SyringeCheckNode::make_info() {
    NodeInfo info;
    info.id = "SyringeCheck";
    info.name = "注射器检测";
    info.category = "医药行业";
    info.description = "注射器检测（针头、刻度）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("syringes", "注射器信息", DataType::String));
    info.outputs.push_back(DataPort("count", "注射器数量", DataType::Number));
    info.outputs.push_back(DataPort("qualified", "合格数量", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_needle_length", "最小针头长度", DataType::Number, Data(20)));
    return info;
}

OVF_REGISTER_NODE(SyringeCheckNode, "SyringeCheck", SyringeCheckNode::make_info())

VialInspectNode::VialInspectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> VialInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_liquid_level = static_cast<float>(get_param("min_liquid_level", Data(0.3f)).as_number());

    Vector<pharma_utils::VialInfo> vials;
    auto result = pharma_utils::detect_vials(input, vials, min_liquid_level);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "药瓶检测失败");
    }

    set_output("vials", Data("VialInfo[" + std::to_string(vials.size()) + "]"));
    set_output("count", Data(static_cast<int>(vials.size())));

    int qualified = 0;
    for (const auto& vial : vials) {
        if (vial.is_sealed && vial.cap_present && vial.liquid_level > 0) qualified++;
    }
    set_output("qualified", Data(qualified));

    String summary = "检测到 " + std::to_string(vials.size()) + " 个药瓶，合格 " + std::to_string(qualified);
    set_output("summary", Data(summary));

    OVF_INFO() << "VialInspectNode: " << summary;
    return Result<void>::success();
}

NodeInfo VialInspectNode::make_info() {
    NodeInfo info;
    info.id = "VialInspect";
    info.name = "药瓶检测";
    info.category = "医药行业";
    info.description = "药瓶检测（密封、液位）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("vials", "药瓶信息", DataType::String));
    info.outputs.push_back(DataPort("count", "药瓶数量", DataType::Number));
    info.outputs.push_back(DataPort("qualified", "合格数量", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_liquid_level", "最小液位", DataType::Number, Data(0.3f)));
    return info;
}

OVF_REGISTER_NODE(VialInspectNode, "VialInspect", VialInspectNode::make_info())

BlisterPackNode::BlisterPackNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> BlisterPackNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    int expected_cavities = get_param("expected_cavities", Data(10)).as_int();

    pharma_utils::BlisterPackInfo pack;
    auto result = pharma_utils::detect_blister_pack(input, pack, expected_cavities);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "泡罩包装检测失败");
    }

    set_output("pack", Data("BlisterPackInfo"));
    set_output("total", Data(pack.total_cavities));
    set_output("filled", Data(pack.filled_cavities));
    set_output("empty", Data(pack.empty_cavities));
    set_output("defective", Data(pack.defective_cavities));
    set_output("integrity", Data(pack.integrity));

    String summary = "泡罩包装: " + std::to_string(pack.filled_cavities) + "/" +
                    std::to_string(pack.total_cavities) + " 已填充";
    set_output("summary", Data(summary));

    OVF_INFO() << "BlisterPackNode: " << summary;
    return Result<void>::success();
}

NodeInfo BlisterPackNode::make_info() {
    NodeInfo info;
    info.id = "BlisterPack";
    info.name = "泡罩包装检测";
    info.category = "医药行业";
    info.description = "泡罩包装检测（完整性、位置）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("pack", "包装信息", DataType::String));
    info.outputs.push_back(DataPort("total", "总泡罩数", DataType::Number));
    info.outputs.push_back(DataPort("filled", "已填充数", DataType::Number));
    info.outputs.push_back(DataPort("empty", "空泡罩数", DataType::Number));
    info.outputs.push_back(DataPort("defective", "缺陷泡罩数", DataType::Number));
    info.outputs.push_back(DataPort("integrity", "完整性", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("expected_cavities", "预期泡罩数", DataType::Number, Data(10)));
    return info;
}

OVF_REGISTER_NODE(BlisterPackNode, "BlisterPack", BlisterPackNode::make_info())

// ========== 食品行业检测节点实现 ==========

FoodFreshnessNode::FoodFreshnessNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FoodFreshnessNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float threshold = static_cast<float>(get_param("threshold", Data(0.6f)).as_number());

    food_utils::FreshnessInfo freshness;
    auto result = food_utils::detect_freshness(input, freshness, threshold);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "新鲜度检测失败");
    }

    set_output("freshness", Data("FreshnessInfo"));
    set_output("score", Data(freshness.freshness_score));
    set_output("level", Data(freshness.freshness_level));

    String summary = "新鲜度: " + freshness.freshness_level + " (" +
                    std::to_string(freshness.freshness_score) + ")";
    set_output("summary", Data(summary));

    OVF_INFO() << "FoodFreshnessNode: " << summary;
    return Result<void>::success();
}

NodeInfo FoodFreshnessNode::make_info() {
    NodeInfo info;
    info.id = "FoodFreshness";
    info.name = "食品新鲜度检测";
    info.category = "食品行业";
    info.description = "食品新鲜度检测（颜色、纹理）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("freshness", "新鲜度信息", DataType::String));
    info.outputs.push_back(DataPort("score", "新鲜度评分", DataType::Number));
    info.outputs.push_back(DataPort("level", "新鲜度等级", DataType::String));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("threshold", "新鲜度阈值", DataType::Number, Data(0.6f)));
    return info;
}

OVF_REGISTER_NODE(FoodFreshnessNode, "FoodFreshness", FoodFreshnessNode::make_info())

FoodContaminateNode::FoodContaminateNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FoodContaminateNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float sensitivity = static_cast<float>(get_param("sensitivity", Data(0.5f)).as_number());

    Vector<food_utils::ContaminateInfo> contaminates;
    auto result = food_utils::detect_contaminates(input, contaminates, sensitivity);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "异物检测失败");
    }

    set_output("contaminates", Data("ContaminateInfo[" + std::to_string(contaminates.size()) + "]"));
    set_output("count", Data(static_cast<int>(contaminates.size())));

    bool has_contaminate = !contaminates.empty();
    set_output("has_contaminate", Data(has_contaminate));

    String summary = has_contaminate ? "发现 " + std::to_string(contaminates.size()) + " 个异物" : "无异物污染";
    set_output("summary", Data(summary));

    OVF_INFO() << "FoodContaminateNode: " << summary;
    return Result<void>::success();
}

NodeInfo FoodContaminateNode::make_info() {
    NodeInfo info;
    info.id = "FoodContaminate";
    info.name = "食品异物检测";
    info.category = "食品行业";
    info.description = "食品异物污染检测";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("contaminates", "异物信息", DataType::String));
    info.outputs.push_back(DataPort("count", "异物数量", DataType::Number));
    info.outputs.push_back(DataPort("has_contaminate", "是否有异物", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("sensitivity", "灵敏度", DataType::Number, Data(0.5f)));
    return info;
}

OVF_REGISTER_NODE(FoodContaminateNode, "FoodContaminate", FoodContaminateNode::make_info())

FruitQualityNode::FruitQualityNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FruitQualityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float maturity_threshold = static_cast<float>(get_param("maturity_threshold", Data(0.5f)).as_number());

    food_utils::FruitQualityInfo quality;
    auto result = food_utils::detect_fruit_quality(input, quality, maturity_threshold);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "水果质量检测失败");
    }

    set_output("quality", Data("FruitQualityInfo"));
    set_output("maturity", Data(quality.maturity));
    set_output("quality_score", Data(quality.quality_score));
    set_output("has_damage", Data(quality.has_damage));

    String summary = "水果质量: 成熟度 " + std::to_string(quality.maturity) +
                    " 质量 " + std::to_string(quality.quality_score);
    set_output("summary", Data(summary));

    OVF_INFO() << "FruitQualityNode: " << summary;
    return Result<void>::success();
}

NodeInfo FruitQualityNode::make_info() {
    NodeInfo info;
    info.id = "FruitQuality";
    info.name = "水果质量检测";
    info.category = "食品行业";
    info.description = "水果质量检测（成熟度、损伤）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("quality", "质量信息", DataType::String));
    info.outputs.push_back(DataPort("maturity", "成熟度", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "质量评分", DataType::Number));
    info.outputs.push_back(DataPort("has_damage", "是否有损伤", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("maturity_threshold", "成熟度阈值", DataType::Number, Data(0.5f)));
    return info;
}

OVF_REGISTER_NODE(FruitQualityNode, "FruitQuality", FruitQualityNode::make_info())

BottleInspectNode::BottleInspectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> BottleInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_liquid_level = static_cast<float>(get_param("min_liquid_level", Data(0.3f)).as_number());

    food_utils::BottleInspectInfo bottle;
    auto result = food_utils::inspect_bottle(input, bottle, min_liquid_level);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "瓶装检测失败");
    }

    set_output("bottle", Data("BottleInspectInfo"));
    set_output("liquid_level", Data(bottle.liquid_level));
    set_output("label_present", Data(bottle.label_present));
    set_output("is_sealed", Data(bottle.is_sealed));

    String summary = "瓶装检测: 液位 " + std::to_string(bottle.liquid_level * 100) + "%";
    set_output("summary", Data(summary));

    OVF_INFO() << "BottleInspectNode: " << summary;
    return Result<void>::success();
}

NodeInfo BottleInspectNode::make_info() {
    NodeInfo info;
    info.id = "BottleInspect";
    info.name = "瓶装检测";
    info.category = "食品行业";
    info.description = "瓶装检测（液位、标签、密封）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("bottle", "瓶装信息", DataType::String));
    info.outputs.push_back(DataPort("liquid_level", "液位", DataType::Number));
    info.outputs.push_back(DataPort("label_present", "标签是否存在", DataType::Boolean));
    info.outputs.push_back(DataPort("is_sealed", "是否密封", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_liquid_level", "最小液位", DataType::Number, Data(0.3f)));
    return info;
}

OVF_REGISTER_NODE(BottleInspectNode, "BottleInspect", BottleInspectNode::make_info())

MeatQualityNode::MeatQualityNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> MeatQualityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_quality = static_cast<float>(get_param("min_quality", Data(0.6f)).as_number());

    food_utils::MeatQualityInfo quality;
    auto result = food_utils::detect_meat_quality(input, quality, min_quality);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "肉类质量检测失败");
    }

    set_output("quality", Data("MeatQualityInfo"));
    set_output("color_score", Data(quality.color_score));
    set_output("texture_score", Data(quality.texture_score));
    set_output("quality_grade", Data(quality.quality_grade));

    String summary = "肉类质量: 等级 " + std::to_string(quality.quality_grade);
    set_output("summary", Data(summary));

    OVF_INFO() << "MeatQualityNode: " << summary;
    return Result<void>::success();
}

NodeInfo MeatQualityNode::make_info() {
    NodeInfo info;
    info.id = "MeatQuality";
    info.name = "肉类质量检测";
    info.category = "食品行业";
    info.description = "肉类质量检测（颜色、纹理）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("quality", "质量信息", DataType::String));
    info.outputs.push_back(DataPort("color_score", "颜色评分", DataType::Number));
    info.outputs.push_back(DataPort("texture_score", "纹理评分", DataType::Number));
    info.outputs.push_back(DataPort("quality_grade", "质量等级", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_quality", "最小质量评分", DataType::Number, Data(0.6f)));
    return info;
}

OVF_REGISTER_NODE(MeatQualityNode, "MeatQuality", MeatQualityNode::make_info())

PackageIntegrityNode::PackageIntegrityNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> PackageIntegrityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_integrity = static_cast<float>(get_param("min_integrity", Data(0.8f)).as_number());

    food_utils::PackageIntegrityInfo integrity;
    auto result = food_utils::check_package_integrity(input, integrity, min_integrity);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "包装完整性检测失败");
    }

    set_output("integrity", Data("PackageIntegrityInfo"));
    set_output("integrity_score", Data(integrity.integrity_score));
    set_output("is_complete", Data(integrity.is_complete));
    set_output("has_damage", Data(integrity.has_damage));

    String summary = "包装完整性: " + std::to_string(integrity.integrity_score);
    set_output("summary", Data(summary));

    OVF_INFO() << "PackageIntegrityNode: " << summary;
    return Result<void>::success();
}

NodeInfo PackageIntegrityNode::make_info() {
    NodeInfo info;
    info.id = "PackageIntegrity";
    info.name = "包装完整性检测";
    info.category = "食品行业";
    info.description = "包装完整性检测";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("integrity", "完整性信息", DataType::String));
    info.outputs.push_back(DataPort("integrity_score", "完整性评分", DataType::Number));
    info.outputs.push_back(DataPort("is_complete", "是否完整", DataType::Boolean));
    info.outputs.push_back(DataPort("has_damage", "是否有损坏", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_integrity", "最小完整性", DataType::Number, Data(0.8f)));
    return info;
}

OVF_REGISTER_NODE(PackageIntegrityNode, "PackageIntegrity", PackageIntegrityNode::make_info())

// ========== 纺织行业检测节点实现 ==========

FabricDefectNode::FabricDefectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FabricDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float sensitivity = static_cast<float>(get_param("sensitivity", Data(0.5f)).as_number());

    Vector<textile_utils::FabricDefectInfo> defects;
    auto result = textile_utils::detect_fabric_defects(input, defects, sensitivity);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "织物缺陷检测失败");
    }

    set_output("defects", Data("FabricDefectInfo[" + std::to_string(defects.size()) + "]"));
    set_output("defect_count", Data(static_cast<int>(defects.size())));

    bool has_defect = !defects.empty();
    set_output("has_defect", Data(has_defect));

    String summary = has_defect ? "发现 " + std::to_string(defects.size()) + " 个缺陷" : "织物完好";
    set_output("summary", Data(summary));

    OVF_INFO() << "FabricDefectNode: " << summary;
    return Result<void>::success();
}

NodeInfo FabricDefectNode::make_info() {
    NodeInfo info;
    info.id = "FabricDefect";
    info.name = "织物缺陷检测";
    info.category = "纺织行业";
    info.description = "织物缺陷检测（断纱、污渍、孔洞）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷信息", DataType::String));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("has_defect", "是否有缺陷", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("sensitivity", "灵敏度", DataType::Number, Data(0.5f)));
    return info;
}

OVF_REGISTER_NODE(FabricDefectNode, "FabricDefect", FabricDefectNode::make_info())

FabricPatternNode::FabricPatternNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FabricPatternNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float match_threshold = static_cast<float>(get_param("match_threshold", Data(0.8f)).as_number());

    textile_utils::FabricPatternInfo pattern;
    auto result = textile_utils::detect_fabric_pattern(input, pattern, match_threshold);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "织物图案检测失败");
    }

    set_output("pattern", Data("FabricPatternInfo"));
    set_output("pattern_match", Data(pattern.pattern_match));
    set_output("is_matched", Data(pattern.is_matched));
    set_output("pattern_type", Data(pattern.pattern_type));

    String summary = "织物图案: 匹配度 " + std::to_string(pattern.pattern_match);
    set_output("summary", Data(summary));

    OVF_INFO() << "FabricPatternNode: " << summary;
    return Result<void>::success();
}

NodeInfo FabricPatternNode::make_info() {
    NodeInfo info;
    info.id = "FabricPattern";
    info.name = "织物图案检测";
    info.category = "纺织行业";
    info.description = "织物图案检测（花纹匹配）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("pattern", "图案信息", DataType::String));
    info.outputs.push_back(DataPort("pattern_match", "匹配度", DataType::Number));
    info.outputs.push_back(DataPort("is_matched", "是否匹配", DataType::Boolean));
    info.outputs.push_back(DataPort("pattern_type", "图案类型", DataType::String));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("match_threshold", "匹配阈值", DataType::Number, Data(0.8f)));
    return info;
}

OVF_REGISTER_NODE(FabricPatternNode, "FabricPattern", FabricPatternNode::make_info())

FabricColorNode::FabricColorNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FabricColorNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float max_deviation = static_cast<float>(get_param("max_deviation", Data(10.0f)).as_number());

    textile_utils::FabricColorInfo color;
    auto result = textile_utils::detect_fabric_color(input, color, max_deviation);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "织物颜色检测失败");
    }

    set_output("color", Data("FabricColorInfo"));
    set_output("color_deviation", Data(color.color_deviation));
    set_output("uniformity", Data(color.uniformity));
    set_output("is_uniform", Data(color.is_uniform));

    String summary = "织物颜色: 色差 " + std::to_string(color.color_deviation);
    set_output("summary", Data(summary));

    OVF_INFO() << "FabricColorNode: " << summary;
    return Result<void>::success();
}

NodeInfo FabricColorNode::make_info() {
    NodeInfo info;
    info.id = "FabricColor";
    info.name = "织物颜色检测";
    info.category = "纺织行业";
    info.description = "织物颜色检测（色差、染色均匀）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("color", "颜色信息", DataType::String));
    info.outputs.push_back(DataPort("color_deviation", "色差值", DataType::Number));
    info.outputs.push_back(DataPort("uniformity", "染色均匀度", DataType::Number));
    info.outputs.push_back(DataPort("is_uniform", "是否均匀", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("max_deviation", "最大色差", DataType::Number, Data(10.0f)));
    return info;
}

OVF_REGISTER_NODE(FabricColorNode, "FabricColor", FabricColorNode::make_info())

FabricDensityNode::FabricDensityNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> FabricDensityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    int expected_warp = get_param("expected_warp", Data(100)).as_int();
    int expected_weft = get_param("expected_weft", Data(100)).as_int();

    textile_utils::FabricDensityInfo density;
    auto result = textile_utils::detect_fabric_density(input, density, expected_warp, expected_weft);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "织物密度检测失败");
    }

    set_output("density", Data("FabricDensityInfo"));
    set_output("warp_density", Data(density.warp_density));
    set_output("weft_density", Data(density.weft_density));
    set_output("is_uniform", Data(density.is_uniform));

    String summary = "织物密度: 经纱 " + std::to_string(density.warp_density) +
                    " 纬纱 " + std::to_string(density.weft_density);
    set_output("summary", Data(summary));

    OVF_INFO() << "FabricDensityNode: " << summary;
    return Result<void>::success();
}

NodeInfo FabricDensityNode::make_info() {
    NodeInfo info;
    info.id = "FabricDensity";
    info.name = "织物密度检测";
    info.category = "纺织行业";
    info.description = "织物密度检测（纱线密度）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("density", "密度信息", DataType::String));
    info.outputs.push_back(DataPort("warp_density", "经纱密度", DataType::Number));
    info.outputs.push_back(DataPort("weft_density", "纬纱密度", DataType::Number));
    info.outputs.push_back(DataPort("is_uniform", "是否均匀", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("expected_warp", "预期经纱密度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("expected_weft", "预期纬纱密度", DataType::Number, Data(100)));
    return info;
}

OVF_REGISTER_NODE(FabricDensityNode, "FabricDensity", FabricDensityNode::make_info())

GarmentInspectNode::GarmentInspectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> GarmentInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_quality = static_cast<float>(get_param("min_quality", Data(0.7f)).as_number());

    textile_utils::GarmentInspectInfo garment;
    auto result = textile_utils::inspect_garment(input, garment, min_quality);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "服装检测失败");
    }

    set_output("garment", Data("GarmentInspectInfo"));
    set_output("quality_score", Data(garment.quality_score));
    set_output("defect_count", Data(garment.defect_count));
    set_output("has_stitch_defect", Data(garment.has_stitch_defect));

    String summary = "服装检测: 质量 " + std::to_string(garment.quality_score) +
                    " 缺陷 " + std::to_string(garment.defect_count);
    set_output("summary", Data(summary));

    OVF_INFO() << "GarmentInspectNode: " << summary;
    return Result<void>::success();
}

NodeInfo GarmentInspectNode::make_info() {
    NodeInfo info;
    info.id = "GarmentInspect";
    info.name = "服装检测";
    info.category = "纺织行业";
    info.description = "服装检测（缝线、瑕疵）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("garment", "服装信息", DataType::String));
    info.outputs.push_back(DataPort("quality_score", "质量评分", DataType::Number));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("has_stitch_defect", "是否有缝线缺陷", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_quality", "最小质量评分", DataType::Number, Data(0.7f)));
    return info;
}

OVF_REGISTER_NODE(GarmentInspectNode, "GarmentInspect", GarmentInspectNode::make_info())

ThreadCountNode::ThreadCountNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> ThreadCountNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    int expected_count = get_param("expected_count", Data(100)).as_int();

    textile_utils::ThreadCountInfo count;
    auto result = textile_utils::count_threads(input, count, expected_count);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "纱线计数检测失败");
    }

    set_output("count_info", Data("ThreadCountInfo"));
    set_output("thread_count", Data(count.thread_count));
    set_output("expected_count", Data(count.expected_count));
    set_output("is_correct", Data(count.is_correct));
    set_output("deviation", Data(count.deviation));

    String summary = "纱线计数: " + std::to_string(count.thread_count) +
                    " 预期 " + std::to_string(count.expected_count) +
                    " 偏差 " + std::to_string(count.deviation);
    set_output("summary", Data(summary));

    OVF_INFO() << "ThreadCountNode: " << summary;
    return Result<void>::success();
}

NodeInfo ThreadCountNode::make_info() {
    NodeInfo info;
    info.id = "ThreadCount";
    info.name = "纱线计数检测";
    info.category = "纺织行业";
    info.description = "纱线计数检测";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("count_info", "计数信息", DataType::String));
    info.outputs.push_back(DataPort("thread_count", "纱线数量", DataType::Number));
    info.outputs.push_back(DataPort("expected_count", "预期数量", DataType::Number));
    info.outputs.push_back(DataPort("is_correct", "数量是否正确", DataType::Boolean));
    info.outputs.push_back(DataPort("deviation", "偏差", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("expected_count", "预期纱线数量", DataType::Number, Data(100)));
    return info;
}

OVF_REGISTER_NODE(ThreadCountNode, "ThreadCount", ThreadCountNode::make_info())

// ==================== 汽车行业检测节点 ====================

PaintQualityNode::PaintQualityNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> PaintQualityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_score = static_cast<float>(get_param("min_score", Data(0.8f)).as_number());

    automotive_utils::PaintQualityInfo paint;
    auto result = automotive_utils::inspect_paint_quality(input, paint, min_score);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "涂装质量检测失败");
    }

    set_output("paint_info", Data("PaintQualityInfo"));
    set_output("paint_score", Data(paint.paint_score));
    set_output("defect_count", Data(paint.defect_count));
    set_output("has_scratch", Data(paint.has_scratch));
    set_output("has_bubble", Data(paint.has_bubble));
    set_output("has_peeling", Data(paint.has_peeling));

    String summary = "涂装质量: " + std::to_string(paint.paint_score) +
                    " 缺陷 " + std::to_string(paint.defect_count);
    set_output("summary", Data(summary));

    OVF_INFO() << "PaintQualityNode: " << summary;
    return Result<void>::success();
}

NodeInfo PaintQualityNode::make_info() {
    NodeInfo info;
    info.id = "PaintQuality";
    info.name = "涂装质量检测";
    info.category = "汽车行业";
    info.description = "涂装质量检测（漆面缺陷）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("paint_info", "涂装信息", DataType::String));
    info.outputs.push_back(DataPort("paint_score", "涂装评分", DataType::Number));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("has_scratch", "是否有划痕", DataType::Boolean));
    info.outputs.push_back(DataPort("has_bubble", "是否有气泡", DataType::Boolean));
    info.outputs.push_back(DataPort("has_peeling", "是否有脱落", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_score", "最小质量评分", DataType::Number, Data(0.8f)));
    return info;
}

OVF_REGISTER_NODE(PaintQualityNode, "PaintQuality", PaintQualityNode::make_info())

WeldInspectNode::WeldInspectNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> WeldInspectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_quality = static_cast<float>(get_param("min_quality", Data(0.7f)).as_number());

    automotive_utils::WeldInspectInfo weld;
    auto result = automotive_utils::inspect_weld(input, weld, min_quality);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "焊缝检测失败");
    }

    set_output("weld_info", Data("WeldInspectInfo"));
    set_output("weld_quality", Data(weld.weld_quality));
    set_output("has_crack", Data(weld.has_crack));
    set_output("has_porosity", Data(weld.has_porosity));
    set_output("is_continuous", Data(weld.is_continuous));
    set_output("weld_points", Data(weld.weld_points));

    String summary = "焊缝质量: " + std::to_string(weld.weld_quality) +
                    " 焊点 " + std::to_string(weld.weld_points);
    set_output("summary", Data(summary));

    OVF_INFO() << "WeldInspectNode: " << summary;
    return Result<void>::success();
}

NodeInfo WeldInspectNode::make_info() {
    NodeInfo info;
    info.id = "WeldInspect";
    info.name = "焊缝检测";
    info.category = "汽车行业";
    info.description = "焊缝检测（焊点质量）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("weld_info", "焊缝信息", DataType::String));
    info.outputs.push_back(DataPort("weld_quality", "焊缝质量", DataType::Number));
    info.outputs.push_back(DataPort("has_crack", "是否有裂纹", DataType::Boolean));
    info.outputs.push_back(DataPort("has_porosity", "是否有气孔", DataType::Boolean));
    info.outputs.push_back(DataPort("is_continuous", "是否连续", DataType::Boolean));
    info.outputs.push_back(DataPort("weld_points", "焊点数量", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_quality", "最小质量评分", DataType::Number, Data(0.7f)));
    return info;
}

OVF_REGISTER_NODE(WeldInspectNode, "WeldInspect", WeldInspectNode::make_info())

PanelGapNode::PanelGapNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> PanelGapNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float expected_gap = static_cast<float>(get_param("expected_gap", Data(5.0f)).as_number());
    float tolerance = static_cast<float>(get_param("tolerance", Data(1.0f)).as_number());

    automotive_utils::PanelGapInfo gap;
    auto result = automotive_utils::check_panel_gap(input, gap, expected_gap, tolerance);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "钣金间隙检测失败");
    }

    set_output("gap_info", Data("PanelGapInfo"));
    set_output("gap_size", Data(gap.gap_size));
    set_output("gap_uniformity", Data(gap.gap_uniformity));
    set_output("is_uniform", Data(gap.is_uniform));
    set_output("deviation", Data(gap.deviation));

    String summary = "钣金间隙: " + std::to_string(gap.gap_size) +
                    " 均匀度 " + std::to_string(gap.gap_uniformity);
    set_output("summary", Data(summary));

    OVF_INFO() << "PanelGapNode: " << summary;
    return Result<void>::success();
}

NodeInfo PanelGapNode::make_info() {
    NodeInfo info;
    info.id = "PanelGap";
    info.name = "钣金间隙检测";
    info.category = "汽车行业";
    info.description = "钣金间隙检测（间隙均匀度）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("gap_info", "间隙信息", DataType::String));
    info.outputs.push_back(DataPort("gap_size", "间隙大小", DataType::Number));
    info.outputs.push_back(DataPort("gap_uniformity", "间隙均匀度", DataType::Number));
    info.outputs.push_back(DataPort("is_uniform", "是否均匀", DataType::Boolean));
    info.outputs.push_back(DataPort("deviation", "偏差", DataType::Number));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("expected_gap", "预期间隙", DataType::Number, Data(5.0f)));
    info.params.push_back(ParamDef("tolerance", "容差", DataType::Number, Data(1.0f)));
    return info;
}

OVF_REGISTER_NODE(PanelGapNode, "PanelGap", PanelGapNode::make_info())

PartPresenceNode::PartPresenceNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> PartPresenceNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    int expected_parts = get_param("expected_parts", Data(10)).as_int();

    automotive_utils::PartPresenceInfo presence;
    auto result = automotive_utils::check_part_presence(input, presence, expected_parts);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "零部件存在检测失败");
    }

    set_output("presence_info", Data("PartPresenceInfo"));
    set_output("total_parts", Data(presence.total_parts));
    set_output("present_parts", Data(presence.present_parts));
    set_output("missing_parts", Data(presence.missing_parts));
    set_output("is_complete", Data(presence.is_complete));

    String summary = "零部件检测: 存在 " + std::to_string(presence.present_parts) +
                    "/" + std::to_string(presence.total_parts) +
                    " 缺失 " + std::to_string(presence.missing_parts);
    set_output("summary", Data(summary));

    OVF_INFO() << "PartPresenceNode: " << summary;
    return Result<void>::success();
}

NodeInfo PartPresenceNode::make_info() {
    NodeInfo info;
    info.id = "PartPresence";
    info.name = "零部件存在检测";
    info.category = "汽车行业";
    info.description = "零部件存在检测（装配完整性）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("presence_info", "存在信息", DataType::String));
    info.outputs.push_back(DataPort("total_parts", "总零件数", DataType::Number));
    info.outputs.push_back(DataPort("present_parts", "存在零件数", DataType::Number));
    info.outputs.push_back(DataPort("missing_parts", "缺失零件数", DataType::Number));
    info.outputs.push_back(DataPort("is_complete", "是否完整", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("expected_parts", "预期零件数", DataType::Number, Data(10)));
    return info;
}

OVF_REGISTER_NODE(PartPresenceNode, "PartPresence", PartPresenceNode::make_info())

SurfaceRoughnessNode::SurfaceRoughnessNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> SurfaceRoughnessNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float max_roughness = static_cast<float>(get_param("max_roughness", Data(10.0f)).as_number());

    automotive_utils::SurfaceRoughnessInfo roughness;
    auto result = automotive_utils::check_surface_roughness(input, roughness, max_roughness);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "表面粗糙度检测失败");
    }

    set_output("roughness_info", Data("SurfaceRoughnessInfo"));
    set_output("roughness_ra", Data(roughness.roughness_ra));
    set_output("roughness_rz", Data(roughness.roughness_rz));
    set_output("is_smooth", Data(roughness.is_smooth));

    String summary = "表面粗糙度: Ra " + std::to_string(roughness.roughness_ra) +
                    " Rz " + std::to_string(roughness.roughness_rz);
    set_output("summary", Data(summary));

    OVF_INFO() << "SurfaceRoughnessNode: " << summary;
    return Result<void>::success();
}

NodeInfo SurfaceRoughnessNode::make_info() {
    NodeInfo info;
    info.id = "SurfaceRoughness";
    info.name = "表面粗糙度检测";
    info.category = "汽车行业";
    info.description = "表面粗糙度检测";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("roughness_info", "粗糙度信息", DataType::String));
    info.outputs.push_back(DataPort("roughness_ra", "Ra值", DataType::Number));
    info.outputs.push_back(DataPort("roughness_rz", "Rz值", DataType::Number));
    info.outputs.push_back(DataPort("is_smooth", "是否光滑", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("max_roughness", "最大粗糙度", DataType::Number, Data(10.0f)));
    return info;
}

OVF_REGISTER_NODE(SurfaceRoughnessNode, "SurfaceRoughness", SurfaceRoughnessNode::make_info())

DimensionCheckNode::DimensionCheckNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> DimensionCheckNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float expected_value = static_cast<float>(get_param("expected_value", Data(0.0f)).as_number());
    float tolerance = static_cast<float>(get_param("tolerance", Data(0.5f)).as_number());

    automotive_utils::DimensionCheckInfo dimension;
    auto result = automotive_utils::check_dimension(input, dimension, expected_value, tolerance);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "尺寸检测失败");
    }

    set_output("dimension_info", Data("DimensionCheckInfo"));
    set_output("measured_value", Data(dimension.measured_value));
    set_output("expected_value", Data(dimension.expected_value));
    set_output("deviation", Data(dimension.deviation));
    set_output("is_correct", Data(dimension.is_correct));
    set_output("dimension_type", Data(dimension.dimension_type));

    String summary = "尺寸检测: 测量 " + std::to_string(dimension.measured_value) +
                    " 预期 " + std::to_string(dimension.expected_value) +
                    " 偏差 " + std::to_string(dimension.deviation);
    set_output("summary", Data(summary));

    OVF_INFO() << "DimensionCheckNode: " << summary;
    return Result<void>::success();
}

NodeInfo DimensionCheckNode::make_info() {
    NodeInfo info;
    info.id = "DimensionCheck";
    info.name = "尺寸检测";
    info.category = "汽车行业";
    info.description = "尺寸检测（孔径、间距）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("dimension_info", "尺寸信息", DataType::String));
    info.outputs.push_back(DataPort("measured_value", "测量值", DataType::Number));
    info.outputs.push_back(DataPort("expected_value", "预期值", DataType::Number));
    info.outputs.push_back(DataPort("deviation", "偏差", DataType::Number));
    info.outputs.push_back(DataPort("is_correct", "是否正确", DataType::Boolean));
    info.outputs.push_back(DataPort("dimension_type", "尺寸类型", DataType::String));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("expected_value", "预期值", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("tolerance", "容差", DataType::Number, Data(0.5f)));
    return info;
}

OVF_REGISTER_NODE(DimensionCheckNode, "DimensionCheck", DimensionCheckNode::make_info())

AssemblyVerifyNode::AssemblyVerifyNode(const String& instance_id) : INode(instance_id, make_info()) {}

Result<void> AssemblyVerifyNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "需要图像输入");
    }
    ImageData input = input_data.as_image();

    float min_score = static_cast<float>(get_param("min_score", Data(0.9f)).as_number());

    automotive_utils::AssemblyVerifyInfo assembly;
    auto result = automotive_utils::verify_assembly(input, assembly, min_score);
    if (result != ErrorCode::Success) {
        return Result<void>::failure(result, "组装验证失败");
    }

    set_output("assembly_info", Data("AssemblyVerifyInfo"));
    set_output("assembly_score", Data(assembly.assembly_score));
    set_output("check_items", Data(assembly.check_items));
    set_output("pass_items", Data(assembly.pass_items));
    set_output("fail_items", Data(assembly.fail_items));
    set_output("is_complete", Data(assembly.is_complete));
    set_output("is_correct", Data(assembly.is_correct));

    String summary = "组装验证: 评分 " + std::to_string(assembly.assembly_score) +
                    " 通过 " + std::to_string(assembly.pass_items) +
                    "/" + std::to_string(assembly.check_items);
    set_output("summary", Data(summary));

    OVF_INFO() << "AssemblyVerifyNode: " << summary;
    return Result<void>::success();
}

NodeInfo AssemblyVerifyNode::make_info() {
    NodeInfo info;
    info.id = "AssemblyVerify";
    info.name = "组装验证检测";
    info.category = "汽车行业";
    info.description = "组装验证检测";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("assembly_info", "组装信息", DataType::String));
    info.outputs.push_back(DataPort("assembly_score", "组装评分", DataType::Number));
    info.outputs.push_back(DataPort("check_items", "检查项数", DataType::Number));
    info.outputs.push_back(DataPort("pass_items", "通过项数", DataType::Number));
    info.outputs.push_back(DataPort("fail_items", "失败项数", DataType::Number));
    info.outputs.push_back(DataPort("is_complete", "是否完整组装", DataType::Boolean));
    info.outputs.push_back(DataPort("is_correct", "是否正确组装", DataType::Boolean));
    info.outputs.push_back(DataPort("summary", "检测摘要", DataType::String));
    info.params.push_back(ParamDef("min_score", "最小质量评分", DataType::Number, Data(0.9f)));
    return info;
}

OVF_REGISTER_NODE(AssemblyVerifyNode, "AssemblyVerify", AssemblyVerifyNode::make_info())

} // namespace algorithm
} // namespace ovf