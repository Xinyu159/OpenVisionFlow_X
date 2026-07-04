/**
 * @file defect_detection.cpp
 * @brief 缺陷检测节点实现
 */

#include "ovf/algorithm/defect_detection.h"
#include "ovf/core/logger.h"
#include <sstream>
#include <cmath>

namespace ovf {
namespace algorithm {

// ============================================================================
// 辅助函数
// ============================================================================

namespace {

// 将缺陷结果转换为Array（使用字符串表示，因为Data不支持Array/Object类型）
Data defect_results_to_array(const std::vector<DefectResult>& results) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        if (i > 0) oss << ",";
        oss << "{id:" << r.id
            << ",type:\"" << r.type << "\""
            << ",x:" << r.x
            << ",y:" << r.y
            << ",width:" << r.width
            << ",height:" << r.height
            << ",severity:" << r.severity
            << ",confidence:" << r.confidence
            << "}";
    }
    oss << "]";
    return Data(oss.str());
}

// 转换灰度图像
ImageData ensure_grayscale(const ImageData& input) {
    ImageData gray;
    if (input.channels == 1) {
        return input;
    }
    defect_utils::to_grayscale(input, gray);
    return gray;
}

} // anonymous namespace

// ============================================================================
// SurfaceDefectNode - 表面缺陷检测（通用）
// ============================================================================

SurfaceDefectNode::SurfaceDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SurfaceDefectNode::make_info() {
    NodeInfo info;
    info.id = "SurfaceDefect";
    info.name = "表面缺陷检测";
    info.category = "缺陷检测";
    info.description = "通用表面缺陷检测，支持多种缺陷类型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "缺陷掩码图像", DataType::Image));

    info.params.push_back(ParamDef("threshold", "检测阈值", DataType::Number, Data(30)));
    info.params.push_back(ParamDef("min_area", "最小缺陷面积", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("method", "检测方法(0:差分 1:对比度 2:梯度)", DataType::Number, Data(0)));

    return info;
}

Result<void> SurfaceDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(30)).as_int();
    int min_area = get_param("min_area", Data(10)).as_int();
    int method = get_param("method", Data(0)).as_int();

    std::vector<uint8_t> defect_mask(gray.width * gray.height, 0);
    std::vector<DefectResult> defects;

    if (method == 0) {
        // 差分方法
        auto template_data = get_input("template");
        if (template_data.is_image() && !template_data.as_image().empty()) {
            ImageData templ = ensure_grayscale(template_data.as_image());
            ImageData diff;
            defect_utils::image_diff(gray, templ, diff);

            for (size_t i = 0; i < diff.data.size(); ++i) {
                if (diff.data[i] > threshold) {
                    defect_mask[i] = 255;
                }
            }
        } else {
            // 如果没有模板，使用自身平滑作为参考
            ImageData blurred;
            defect_utils::gaussian_blur(gray, blurred, 15);
            ImageData diff;
            defect_utils::image_diff(gray, blurred, diff);

            for (size_t i = 0; i < diff.data.size(); ++i) {
                if (diff.data[i] > threshold) {
                    defect_mask[i] = 255;
                }
            }
        }
    } else if (method == 1) {
        // 对比度方法
        int win_size = 15;
        for (uint32_t y = win_size / 2; y < gray.height - win_size / 2; ++y) {
            for (uint32_t x = win_size / 2; x < gray.width - win_size / 2; ++x) {
                double contrast = defect_utils::compute_local_contrast(gray, x, y, win_size);
                if (contrast * 255 > threshold) {
                    defect_mask[y * gray.width + x] = 255;
                }
            }
        }
    } else {
        // 梯度方法
        std::vector<double> gradient;
        defect_utils::compute_gradient(gray, gradient);

        for (size_t i = 0; i < gradient.size(); ++i) {
            if (gradient[i] > threshold) {
                defect_mask[i] = 255;
            }
        }
    }

    // 形态学处理
    defect_utils::dilate(defect_mask, gray.width, gray.height, 3);
    defect_utils::erode(defect_mask, gray.width, gray.height, 3);

    // 连通区域分析
    auto components = defect_utils::find_connected_components(defect_mask, gray.width, gray.height);

    int defect_id = 0;
    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "surface";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 1000.0);
            dr.confidence = 0.8;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = defect_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "SurfaceDefectNode detected " << defects.size() << " defects";

    return Result<void>::success();
}

// ============================================================================
// ScratchDetectNode - 划痕检测
// ============================================================================

ScratchDetectNode::ScratchDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ScratchDetectNode::make_info() {
    NodeInfo info;
    info.id = "ScratchDetect";
    info.name = "划痕检测";
    info.category = "缺陷检测";
    info.description = "检测表面划痕缺陷，识别细长线条状异常";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "划痕掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "梯度阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("min_length", "最小划痕长度", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("max_width", "最大划痕宽度", DataType::Number, Data(5)));

    return info;
}

Result<void> ScratchDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(50)).as_int();
    int min_length = get_param("min_length", Data(20)).as_int();
    int max_width = get_param("max_width", Data(5)).as_int();

    std::vector<uint8_t> scratch_mask(gray.width * gray.height, 0);

    // 使用Sobel检测边缘
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            int gx = 0, gy = 0;

            // Sobel算子
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    uint8_t pixel = gray.data[(y + dy) * gray.width + (x + dx)];
                    gx += pixel * dx * (dy == 0 ? 2 : 1);
                    gy += pixel * dy * (dx == 0 ? 2 : 1);
                }
            }

            int magnitude = static_cast<int>(std::sqrt(gx * gx + gy * gy));

            // 划痕特征：高梯度且方向一致
            if (magnitude > threshold) {
                scratch_mask[y * gray.width + x] = 255;
            }
        }
    }

    // 细化操作（简化：腐蚀）
    for (int i = 0; i < 2; ++i) {
        defect_utils::erode(scratch_mask, gray.width, gray.height, 3);
    }

    auto components = defect_utils::find_connected_components(scratch_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        // 划痕特征：长宽比大
        double aspect_ratio = static_cast<double>(std::max(cc.width, cc.height)) /
                             std::max(1, std::min(cc.width, cc.height));

        if (cc.pixel_count >= min_length && cc.width <= max_width && aspect_ratio > 3.0) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "scratch";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, aspect_ratio / 20.0);
            dr.confidence = 0.85;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = scratch_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "ScratchDetectNode detected " << defects.size() << " scratches";

    return Result<void>::success();
}

// ============================================================================
// CrackDetectNode - 裂纹检测
// ============================================================================

CrackDetectNode::CrackDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CrackDetectNode::make_info() {
    NodeInfo info;
    info.id = "CrackDetect";
    info.name = "裂纹检测";
    info.category = "缺陷检测";
    info.description = "检测表面裂纹缺陷，识别分叉状异常";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "裂纹掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "检测阈值", DataType::Number, Data(40)));
    info.params.push_back(ParamDef("min_length", "最小裂纹长度", DataType::Number, Data(15)));
    info.params.push_back(ParamDef("sensitivity", "敏感度", DataType::Number, Data(0.5)));

    return info;
}

Result<void> CrackDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(40)).as_int();
    int min_length = get_param("min_length", Data(15)).as_int();
    double sensitivity = get_param("sensitivity", Data(0.5)).as_number();

    std::vector<uint8_t> crack_mask(gray.width * gray.height, 0);

    // Laplacian检测裂纹（二阶导数）
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            int laplacian = 0;

            // 中心像素 * (-4)
            laplacian -= 4 * gray.data[y * gray.width + x];

            // 四邻域 * 1
            laplacian += gray.data[(y - 1) * gray.width + x];
            laplacian += gray.data[(y + 1) * gray.width + x];
            laplacian += gray.data[y * gray.width + (x - 1)];
            laplacian += gray.data[y * gray.width + (x + 1)];

            laplacian = std::abs(laplacian);

            if (laplacian > threshold) {
                crack_mask[y * gray.width + x] = 255;
            }
        }
    }

    // 形态学处理增强裂纹
    defect_utils::dilate(crack_mask, gray.width, gray.height, 2);

    auto components = defect_utils::find_connected_components(crack_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_length) {
            // 计算形状复杂度（周长平方/面积）
            double complexity = static_cast<double>(cc.pixel_count * 4) / (cc.width * cc.height + 1);

            if (complexity > sensitivity * 2) {
                DefectResult dr;
                dr.id = defect_id++;
                dr.type = "crack";
                dr.x = cc.x;
                dr.y = cc.y;
                dr.width = cc.width;
                dr.height = cc.height;
                dr.severity = std::min(1.0, complexity / 10.0);
                dr.confidence = 0.75;
                defects.push_back(dr);
            }
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = crack_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "CrackDetectNode detected " << defects.size() << " cracks";

    return Result<void>::success();
}

// ============================================================================
// StainDetectNode - 污渍检测
// ============================================================================

StainDetectNode::StainDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo StainDetectNode::make_info() {
    NodeInfo info;
    info.id = "StainDetect";
    info.name = "污渍检测";
    info.category = "缺陷检测";
    info.description = "检测表面污渍缺陷，识别斑块状异常";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "污渍掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "灰度偏差阈值", DataType::Number, Data(25)));
    info.params.push_back(ParamDef("min_area", "最小污渍面积", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("max_area", "最大污渍面积", DataType::Number, Data(5000)));

    return info;
}

Result<void> StainDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(25)).as_int();
    int min_area = get_param("min_area", Data(50)).as_int();
    int max_area = get_param("max_area", Data(5000)).as_int();

    // 计算全局均值
    double global_mean = 0;
    for (size_t i = 0; i < gray.data.size(); ++i) {
        global_mean += gray.data[i];
    }
    global_mean /= gray.data.size();

    std::vector<uint8_t> stain_mask(gray.width * gray.height, 0);

    // 局部灰度偏差检测
    int win_size = 21;
    for (uint32_t y = win_size / 2; y < gray.height - win_size / 2; ++y) {
        for (uint32_t x = win_size / 2; x < gray.width - win_size / 2; ++x) {
            double local_mean = defect_utils::compute_local_mean(gray, x, y, win_size);
            double deviation = std::abs(local_mean - global_mean);

            if (deviation > threshold) {
                stain_mask[y * gray.width + x] = 255;
            }
        }
    }

    // 形态学操作
    defect_utils::dilate(stain_mask, gray.width, gray.height, 5);
    defect_utils::erode(stain_mask, gray.width, gray.height, 5);

    auto components = defect_utils::find_connected_components(stain_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area && cc.pixel_count <= max_area) {
            // 污渍特征：面积较大，形状不规则
            double circularity = 4.0 * 3.14159 * cc.pixel_count /
                                 ((cc.width + cc.height) * (cc.width + cc.height) / 4.0 + 1);

            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "stain";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 1000.0);
            dr.confidence = circularity;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = stain_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "StainDetectNode detected " << defects.size() << " stains";

    return Result<void>::success();
}

// ============================================================================
// PitDetectNode - 凹坑检测
// ============================================================================

PitDetectNode::PitDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PitDetectNode::make_info() {
    NodeInfo info;
    info.id = "PitDetect";
    info.name = "凹坑检测";
    info.category = "缺陷检测";
    info.description = "检测表面凹坑缺陷，识别凹陷异常";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "凹坑掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "深度阈值", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("min_radius", "最小凹坑半径", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("max_radius", "最大凹坑半径", DataType::Number, Data(50)));

    return info;
}

Result<void> PitDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(20)).as_int();
    int min_radius = get_param("min_radius", Data(3)).as_int();
    int max_radius = get_param("max_radius", Data(50)).as_int();

    // 高斯平滑
    ImageData blurred;
    defect_utils::gaussian_blur(gray, blurred, 5);

    std::vector<uint8_t> pit_mask(gray.width * gray.height, 0);

    // 凹坑检测：中心暗于周围
    int check_radius = 5;
    for (uint32_t y = check_radius; y < gray.height - check_radius; ++y) {
        for (uint32_t x = check_radius; x < gray.width - check_radius; ++x) {
            double center_val = blurred.data[y * blurred.width + x];

            // 计算周围环的平均值
            double ring_sum = 0;
            int ring_count = 0;
            for (int dy = -check_radius; dy <= check_radius; ++dy) {
                for (int dx = -check_radius; dx <= check_radius; ++dx) {
                    int dist = static_cast<int>(std::sqrt(dx * dx + dy * dy));
                    if (dist >= check_radius - 1 && dist <= check_radius + 1) {
                        ring_sum += blurred.data[(y + dy) * blurred.width + (x + dx)];
                        ring_count++;
                    }
                }
            }

            double ring_mean = ring_count > 0 ? ring_sum / ring_count : center_val;
            double depth = ring_mean - center_val;

            if (depth > threshold) {
                pit_mask[y * gray.width + x] = 255;
            }
        }
    }

    // 形态学操作
    defect_utils::erode(pit_mask, gray.width, gray.height, 3);
    defect_utils::dilate(pit_mask, gray.width, gray.height, 3);

    auto components = defect_utils::find_connected_components(pit_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        int radius = (cc.width + cc.height) / 4;
        if (radius >= min_radius && radius <= max_radius) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "pit";
            dr.x = cc.x + cc.width / 2;
            dr.y = cc.y + cc.height / 2;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, radius / 30.0);
            dr.confidence = 0.8;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = pit_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "PitDetectNode detected " << defects.size() << " pits";

    return Result<void>::success();
}

// ============================================================================
// ShapeDefectNode - 形状缺陷检测
// ============================================================================

ShapeDefectNode::ShapeDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeDefectNode::make_info() {
    NodeInfo info;
    info.id = "ShapeDefect";
    info.name = "形状缺陷检测";
    info.category = "缺陷检测";
    info.description = "检测形状缺陷，识别轮廓异常";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("reference", "参考形状（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("contour", "轮廓图像", DataType::Image));

    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("circularity_min", "最小圆度", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("area_min", "最小面积", DataType::Number, Data(100)));

    return info;
}

Result<void> ShapeDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int edge_threshold = get_param("edge_threshold", Data(50)).as_int();
    double circularity_min = get_param("circularity_min", Data(0.5)).as_number();
    int area_min = get_param("area_min", Data(100)).as_int();

    // 边缘检测
    std::vector<uint8_t> edge_mask(gray.width * gray.height, 0);
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            int gx = gray.data[y * gray.width + x + 1] - gray.data[y * gray.width + x - 1];
            int gy = gray.data[(y + 1) * gray.width + x] - gray.data[(y - 1) * gray.width + x];
            int magnitude = static_cast<int>(std::sqrt(gx * gx + gy * gy));
            if (magnitude > edge_threshold) {
                edge_mask[y * gray.width + x] = 255;
            }
        }
    }

    auto components = defect_utils::find_connected_components(edge_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= area_min) {
            // 计算圆度 = 4π * 面积 / 周长²
            double perimeter = (cc.width + cc.height) * 2;
            double circularity = 4.0 * 3.14159 * cc.pixel_count / (perimeter * perimeter + 1);

            if (circularity < circularity_min) {
                DefectResult dr;
                dr.id = defect_id++;
                dr.type = "shape";
                dr.x = cc.x;
                dr.y = cc.y;
                dr.width = cc.width;
                dr.height = cc.height;
                dr.severity = 1.0 - circularity;
                dr.confidence = 0.7;
                defects.push_back(dr);
            }
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData contour_img;
    contour_img.width = gray.width;
    contour_img.height = gray.height;
    contour_img.channels = 1;
    contour_img.format = ImageFormat::Mono8;
    contour_img.data = edge_mask;
    set_output("contour", Data(contour_img));

    OVF_INFO() << "ShapeDefectNode detected " << defects.size() << " shape defects";

    return Result<void>::success();
}

// ============================================================================
// DimensionDefectNode - 尺寸缺陷检测
// ============================================================================

DimensionDefectNode::DimensionDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DimensionDefectNode::make_info() {
    NodeInfo info;
    info.id = "DimensionDefect";
    info.name = "尺寸缺陷检测";
    info.category = "缺陷检测";
    info.description = "检测尺寸偏差，与标准尺寸对比";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("measurement", "测量结果", DataType::Object));

    info.params.push_back(ParamDef("expected_width", "期望宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("expected_height", "期望高度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("tolerance", "公差(像素)", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("threshold", "二值化阈值", DataType::Number, Data(128)));

    return info;
}

Result<void> DimensionDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int expected_width = get_param("expected_width", Data(100)).as_int();
    int expected_height = get_param("expected_height", Data(100)).as_int();
    int tolerance = get_param("tolerance", Data(5)).as_int();
    int threshold = get_param("threshold", Data(128)).as_int();

    // 二值化
    std::vector<uint8_t> binary(gray.width * gray.height, 0);
    defect_utils::threshold_binary(gray, binary, threshold);

    // 查找最大区域（假设为目标物体）
    auto components = defect_utils::find_connected_components(binary, gray.width, gray.height);

    std::vector<DefectResult> defects;
    std::ostringstream measurement_oss;

    if (!components.empty()) {
        // 找最大区域
        auto max_cc = *std::max_element(components.begin(), components.end(),
            [](const defect_utils::ConnectedComponent& a, const defect_utils::ConnectedComponent& b) {
                return a.pixel_count < b.pixel_count;
            });

        int actual_width = max_cc.width;
        int actual_height = max_cc.height;

        measurement_oss << "{width:" << actual_width
                        << ",height:" << actual_height
                        << ",area:" << max_cc.pixel_count << "}";

        // 检查尺寸偏差
        int width_error = actual_width - expected_width;
        int height_error = actual_height - expected_height;

        if (std::abs(width_error) > tolerance || std::abs(height_error) > tolerance) {
            DefectResult dr;
            dr.id = 0;
            dr.type = "dimension";
            dr.x = max_cc.x;
            dr.y = max_cc.y;
            dr.width = actual_width;
            dr.height = actual_height;
            dr.severity = std::max(std::abs(width_error), std::abs(height_error)) /
                         static_cast<double>(tolerance);
            dr.confidence = 0.9;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));
    set_output("measurement", Data(measurement_oss.str()));

    OVF_INFO() << "DimensionDefectNode detected " << defects.size() << " dimension defects";

    return Result<void>::success();
}

// ============================================================================
// PositionDefectNode - 位置偏差检测
// ============================================================================

PositionDefectNode::PositionDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PositionDefectNode::make_info() {
    NodeInfo info;
    info.id = "PositionDefect";
    info.name = "位置偏差检测";
    info.category = "缺陷检测";
    info.description = "检测位置偏差，与期望位置对比";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("position", "实际位置", DataType::Object));

    info.params.push_back(ParamDef("expected_x", "期望X坐标", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("expected_y", "期望Y坐标", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("tolerance", "公差(像素)", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("threshold", "二值化阈值", DataType::Number, Data(128)));

    return info;
}

Result<void> PositionDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int expected_x = get_param("expected_x", Data(100)).as_int();
    int expected_y = get_param("expected_y", Data(100)).as_int();
    int tolerance = get_param("tolerance", Data(10)).as_int();
    int threshold = get_param("threshold", Data(128)).as_int();

    // 二值化
    std::vector<uint8_t> binary(gray.width * gray.height, 0);
    defect_utils::threshold_binary(gray, binary, threshold);

    auto components = defect_utils::find_connected_components(binary, gray.width, gray.height);

    std::vector<DefectResult> defects;
    std::ostringstream position_oss;

    if (!components.empty()) {
        // 找最大区域的中心
        auto max_cc = *std::max_element(components.begin(), components.end(),
            [](const defect_utils::ConnectedComponent& a, const defect_utils::ConnectedComponent& b) {
                return a.pixel_count < b.pixel_count;
            });

        int actual_x = max_cc.x + max_cc.width / 2;
        int actual_y = max_cc.y + max_cc.height / 2;

        position_oss << "{x:" << actual_x
                     << ",y:" << actual_y
                     << ",width:" << max_cc.width
                     << ",height:" << max_cc.height << "}";

        // 检查位置偏差
        int x_error = actual_x - expected_x;
        int y_error = actual_y - expected_y;
        double distance = std::sqrt(x_error * x_error + y_error * y_error);

        if (distance > tolerance) {
            DefectResult dr;
            dr.id = 0;
            dr.type = "position";
            dr.x = actual_x;
            dr.y = actual_y;
            dr.width = static_cast<int>(distance);
            dr.height = 0;
            dr.severity = distance / tolerance;
            dr.confidence = 0.9;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));
    set_output("position", Data(position_oss.str()));

    OVF_INFO() << "PositionDefectNode detected " << defects.size() << " position defects";

    return Result<void>::success();
}

// ============================================================================
// AngleDefectNode - 角度偏差检测
// ============================================================================

AngleDefectNode::AngleDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AngleDefectNode::make_info() {
    NodeInfo info;
    info.id = "AngleDefect";
    info.name = "角度偏差检测";
    info.category = "缺陷检测";
    info.description = "检测角度偏差，与期望角度对比";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("angle", "实际角度", DataType::Object));

    info.params.push_back(ParamDef("expected_angle", "期望角度(度)", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("tolerance", "公差(度)", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("threshold", "二值化阈值", DataType::Number, Data(128)));

    return info;
}

Result<void> AngleDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    double expected_angle = get_param("expected_angle", Data(0)).as_number();
    double tolerance = get_param("tolerance", Data(5)).as_number();
    int threshold = get_param("threshold", Data(128)).as_int();

    // 二值化
    std::vector<uint8_t> binary(gray.width * gray.height, 0);
    defect_utils::threshold_binary(gray, binary, threshold);

    // 使用边缘检测计算主方向
    std::vector<double> angles;
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            if (binary[y * gray.width + x] > 0) {
                int gx = gray.data[y * gray.width + x + 1] - gray.data[y * gray.width + x - 1];
                int gy = gray.data[(y + 1) * gray.width + x] - gray.data[(y - 1) * gray.width + x];
                if (gx != 0 || gy != 0) {
                    double angle = std::atan2(gy, gx) * 180.0 / 3.14159;
                    angles.push_back(angle);
                }
            }
        }
    }

    std::vector<DefectResult> defects;
    std::ostringstream angle_oss;

    if (!angles.empty()) {
        // 计算主方向（简化：取平均）
        double sum_sin = 0, sum_cos = 0;
        for (double a : angles) {
            double rad = a * 3.14159 / 180.0;
            sum_sin += std::sin(rad);
            sum_cos += std::cos(rad);
        }
        double actual_angle = std::atan2(sum_sin, sum_cos) * 180.0 / 3.14159;

        // 规范化角度到 [-90, 90]
        while (actual_angle > 90) actual_angle -= 180;
        while (actual_angle < -90) actual_angle += 180;

        angle_oss << "{value:" << actual_angle
                 << ",count:" << angles.size() << "}";

        // 检查角度偏差
        double angle_error = std::abs(actual_angle - expected_angle);
        if (angle_error > 90) angle_error = 180 - angle_error;

        if (angle_error > tolerance) {
            DefectResult dr;
            dr.id = 0;
            dr.type = "angle";
            dr.x = 0;
            dr.y = 0;
            dr.width = static_cast<int>(actual_angle * 10);
            dr.height = static_cast<int>(angle_error * 10);
            dr.severity = angle_error / tolerance;
            dr.confidence = 0.85;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));
    set_output("angle", Data(angle_oss.str()));

    OVF_INFO() << "AngleDefectNode detected " << defects.size() << " angle defects";

    return Result<void>::success();
}

// ============================================================================
// ColorDefectNode - 颜色缺陷检测
// ============================================================================

ColorDefectNode::ColorDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ColorDefectNode::make_info() {
    NodeInfo info;
    info.id = "ColorDefect";
    info.name = "颜色缺陷检测";
    info.category = "缺陷检测";
    info.description = "检测颜色异常缺陷";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("reference", "参考颜色（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "颜色异常掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "颜色偏差阈值", DataType::Number, Data(30)));
    info.params.push_back(ParamDef("min_area", "最小缺陷面积", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("channel", "检测通道(0:R 1:G 2:B 3:All)", DataType::Number, Data(3)));

    return info;
}

Result<void> ColorDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    if (input.channels < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Color image required");
    }

    int threshold = get_param("threshold", Data(30)).as_int();
    int min_area = get_param("min_area", Data(20)).as_int();
    int channel = get_param("channel", Data(3)).as_int();

    // 计算参考颜色（均值或从参考图像）
    std::vector<double> ref_color(3, 0);
    auto ref_data = get_input("reference");
    if (ref_data.is_image() && !ref_data.as_image().empty() && ref_data.as_image().channels >= 3) {
        ImageData ref = ref_data.as_image();
        for (size_t i = 0; i < ref.data.size() / 3; ++i) {
            ref_color[0] += ref.data[i * 3];
            ref_color[1] += ref.data[i * 3 + 1];
            ref_color[2] += ref.data[i * 3 + 2];
        }
        for (int c = 0; c < 3; ++c) ref_color[c] /= (ref.data.size() / 3);
    } else {
        // 使用输入图像均值
        for (size_t i = 0; i < input.data.size() / 3; ++i) {
            ref_color[0] += input.data[i * 3];
            ref_color[1] += input.data[i * 3 + 1];
            ref_color[2] += input.data[i * 3 + 2];
        }
        for (int c = 0; c < 3; ++c) ref_color[c] /= (input.data.size() / 3);
    }

    std::vector<uint8_t> color_mask(input.width * input.height, 0);

    // 检测颜色偏差
    for (uint32_t y = 0; y < input.height; ++y) {
        for (uint32_t x = 0; x < input.width; ++x) {
            size_t idx = (y * input.width + x) * 3;
            double color_diff = 0;

            if (channel == 3) {
                // 所有通道
                for (int c = 0; c < 3; ++c) {
                    double diff = input.data[idx + c] - ref_color[c];
                    color_diff += diff * diff;
                }
                color_diff = std::sqrt(color_diff);
            } else {
                // 单通道
                color_diff = std::abs(input.data[idx + channel] - ref_color[channel]);
            }

            if (color_diff > threshold) {
                color_mask[y * input.width + x] = 255;
            }
        }
    }

    // 形态学操作
    defect_utils::dilate(color_mask, input.width, input.height, 3);
    defect_utils::erode(color_mask, input.width, input.height, 3);

    auto components = defect_utils::find_connected_components(color_mask, input.width, input.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "color";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 500.0);
            dr.confidence = 0.8;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = input.width;
    mask_img.height = input.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = color_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "ColorDefectNode detected " << defects.size() << " color defects";

    return Result<void>::success();
}

// ============================================================================
// TextureDefectNode - 纹理缺陷检测
// ============================================================================

TextureDefectNode::TextureDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo TextureDefectNode::make_info() {
    NodeInfo info;
    info.id = "TextureDefect";
    info.name = "纹理缺陷检测";
    info.category = "缺陷检测";
    info.description = "检测纹理异常缺陷";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "纹理异常掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "纹理差异阈值", DataType::Number, Data(0.3)));
    info.params.push_back(ParamDef("window_size", "窗口大小", DataType::Number, Data(15)));
    info.params.push_back(ParamDef("min_area", "最小缺陷面积", DataType::Number, Data(30)));

    return info;
}

Result<void> TextureDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    double threshold = get_param("threshold", Data(0.3)).as_number();
    int window_size = get_param("window_size", Data(15)).as_int();
    int min_area = get_param("min_area", Data(30)).as_int();

    // 计算全局纹理特征
    double global_contrast = 0, global_homogeneity = 0, global_energy = 0;
    int sample_count = 0;

    for (uint32_t y = window_size; y < gray.height - window_size; y += window_size / 2) {
        for (uint32_t x = window_size; x < gray.width - window_size; x += window_size / 2) {
            double c, h, e;
            defect_utils::compute_texture_features(gray, x, y, window_size, c, h, e);
            global_contrast += c;
            global_homogeneity += h;
            global_energy += e;
            sample_count++;
        }
    }

    if (sample_count > 0) {
        global_contrast /= sample_count;
        global_homogeneity /= sample_count;
        global_energy /= sample_count;
    }

    std::vector<uint8_t> texture_mask(gray.width * gray.height, 0);

    // 检测局部纹理异常
    for (uint32_t y = window_size / 2; y < gray.height - window_size / 2; y += 3) {
        for (uint32_t x = window_size / 2; x < gray.width - window_size / 2; x += 3) {
            double c, h, e;
            defect_utils::compute_texture_features(gray, x, y, window_size, c, h, e);

            // 计算纹理差异
            double diff = 0;
            if (global_contrast > 0) diff += std::abs(c - global_contrast) / global_contrast;
            if (global_homogeneity > 0) diff += std::abs(h - global_homogeneity) / global_homogeneity;
            if (global_energy > 0) diff += std::abs(e - global_energy) / global_energy;

            diff /= 3;

            if (diff > threshold) {
                // 标记窗口内的像素
                int half = window_size / 4;
                for (int dy = -half; dy <= half; ++dy) {
                    for (int dx = -half; dx <= half; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height)) {
                            texture_mask[ny * gray.width + nx] = 255;
                        }
                    }
                }
            }
        }
    }

    auto components = defect_utils::find_connected_components(texture_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "texture";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 300.0);
            dr.confidence = 0.75;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = texture_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "TextureDefectNode detected " << defects.size() << " texture defects";

    return Result<void>::success();
}

// ============================================================================
// GlossDefectNode - 光泽缺陷检测
// ============================================================================

GlossDefectNode::GlossDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GlossDefectNode::make_info() {
    NodeInfo info;
    info.id = "GlossDefect";
    info.name = "光泽缺陷检测";
    info.category = "缺陷检测";
    info.description = "检测光泽异常缺陷（过亮或过暗区域）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "光泽异常掩码", DataType::Image));

    info.params.push_back(ParamDef("low_threshold", "低光泽阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("high_threshold", "高光泽阈值", DataType::Number, Data(200)));
    info.params.push_back(ParamDef("min_area", "最小缺陷面积", DataType::Number, Data(20)));

    return info;
}

Result<void> GlossDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int low_threshold = get_param("low_threshold", Data(50)).as_int();
    int high_threshold = get_param("high_threshold", Data(200)).as_int();
    int min_area = get_param("min_area", Data(20)).as_int();

    std::vector<uint8_t> gloss_mask(gray.width * gray.height, 0);

    // 检测过亮或过暗区域
    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (gray.data[i] < low_threshold || gray.data[i] > high_threshold) {
            gloss_mask[i] = 255;
        }
    }

    // 形态学操作
    defect_utils::dilate(gloss_mask, gray.width, gray.height, 3);
    defect_utils::erode(gloss_mask, gray.width, gray.height, 3);

    auto components = defect_utils::find_connected_components(gloss_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area) {
            // 计算区域平均亮度
            double mean_brightness = defect_utils::compute_region_mean(gray, cc);

            DefectResult dr;
            dr.id = defect_id++;
            dr.type = mean_brightness < 128 ? "low_gloss" : "high_gloss";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::abs(mean_brightness - 128) / 127.0;
            dr.confidence = 0.8;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = gloss_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "GlossDefectNode detected " << defects.size() << " gloss defects";

    return Result<void>::success();
}

// ============================================================================
// OpacityDefectNode - 透明度缺陷检测
// ============================================================================

OpacityDefectNode::OpacityDefectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo OpacityDefectNode::make_info() {
    NodeInfo info;
    info.id = "OpacityDefect";
    info.name = "透明度缺陷检测";
    info.category = "缺陷检测";
    info.description = "检测透明度异常缺陷（不均匀透明度）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("background", "背景图像", DataType::Image, false));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("opacity_map", "透明度图", DataType::Image));

    info.params.push_back(ParamDef("threshold", "透明度差异阈值", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("min_area", "最小缺陷面积", DataType::Number, Data(20)));

    return info;
}

Result<void> OpacityDefectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(20)).as_int();
    int min_area = get_param("min_area", Data(20)).as_int();

    // 计算透明度（简化：使用局部对比度）
    std::vector<uint8_t> opacity_map(gray.width * gray.height, 0);
    std::vector<uint8_t> defect_mask(gray.width * gray.height, 0);

    int win_size = 9;
    for (uint32_t y = win_size / 2; y < gray.height - win_size / 2; ++y) {
        for (uint32_t x = win_size / 2; x < gray.width - win_size / 2; ++x) {
            double local_mean = defect_utils::compute_local_mean(gray, x, y, win_size);
            double local_std = defect_utils::compute_local_std(gray, x, y, win_size, local_mean);

            // 透明度：高对比度区域表示更透明
            uint8_t opacity = static_cast<uint8_t>(std::min(255.0, local_std * 3));
            opacity_map[y * gray.width + x] = opacity;

            // 检测异常透明度区域
            if (local_std > threshold) {
                defect_mask[y * gray.width + x] = 255;
            }
        }
    }

    auto components = defect_utils::find_connected_components(defect_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "opacity";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 200.0);
            dr.confidence = 0.7;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData opacity_img;
    opacity_img.width = gray.width;
    opacity_img.height = gray.height;
    opacity_img.channels = 1;
    opacity_img.format = ImageFormat::Mono8;
    opacity_img.data = opacity_map;
    set_output("opacity_map", Data(opacity_img));

    OVF_INFO() << "OpacityDefectNode detected " << defects.size() << " opacity defects";

    return Result<void>::success();
}

// ============================================================================
// ForeignObjectNode - 异物检测
// ============================================================================

ForeignObjectNode::ForeignObjectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ForeignObjectNode::make_info() {
    NodeInfo info;
    info.id = "ForeignObject";
    info.name = "异物检测";
    info.category = "缺陷检测";
    info.description = "检测异物污染";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "异物掩码", DataType::Image));

    info.params.push_back(ParamDef("threshold", "差异阈值", DataType::Number, Data(25)));
    info.params.push_back(ParamDef("min_area", "最小异物面积", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("max_area", "最大异物面积", DataType::Number, Data(1000)));

    return info;
}

Result<void> ForeignObjectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = ensure_grayscale(input);

    int threshold = get_param("threshold", Data(25)).as_int();
    int min_area = get_param("min_area", Data(10)).as_int();
    int max_area = get_param("max_area", Data(1000)).as_int();

    std::vector<uint8_t> foreign_mask(gray.width * gray.height, 0);

    auto template_data = get_input("template");
    if (template_data.is_image() && !template_data.as_image().empty()) {
        // 与模板比较
        ImageData templ = ensure_grayscale(template_data.as_image());
        if (templ.width == gray.width && templ.height == gray.height) {
            for (size_t i = 0; i < gray.data.size(); ++i) {
                int diff = std::abs(static_cast<int>(gray.data[i]) - static_cast<int>(templ.data[i]));
                if (diff > threshold) {
                    foreign_mask[i] = 255;
                }
            }
        }
    } else {
        // 使用背景建模（简化：中值滤波背景）
        ImageData blurred;
        defect_utils::gaussian_blur(gray, blurred, 21);

        for (size_t i = 0; i < gray.data.size(); ++i) {
            int diff = std::abs(static_cast<int>(gray.data[i]) - static_cast<int>(blurred.data[i]));
            if (diff > threshold) {
                foreign_mask[i] = 255;
            }
        }
    }

    // 形态学操作去除噪声
    defect_utils::erode(foreign_mask, gray.width, gray.height, 2);
    defect_utils::dilate(foreign_mask, gray.width, gray.height, 2);

    auto components = defect_utils::find_connected_components(foreign_mask, gray.width, gray.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area && cc.pixel_count <= max_area) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "foreign_object";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 500.0);
            dr.confidence = 0.85;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray.width;
    mask_img.height = gray.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = foreign_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "ForeignObjectNode detected " << defects.size() << " foreign objects";

    return Result<void>::success();
}

// ============================================================================
// MissingObjectNode - 缺失检测
// ============================================================================

MissingObjectNode::MissingObjectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MissingObjectNode::make_info() {
    NodeInfo info;
    info.id = "MissingObject";
    info.name = "缺失检测";
    info.category = "缺陷检测";
    info.description = "检测缺失的物体或部件";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("mask", "缺失区域掩码", DataType::Image));
    info.outputs.push_back(DataPort("score", "匹配得分", DataType::Number));

    info.params.push_back(ParamDef("threshold", "缺失判定阈值", DataType::Number, Data(30)));
    info.params.push_back(ParamDef("min_area", "最小缺失面积", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("check_regions", "检查区域数量", DataType::Number, Data(1)));

    return info;
}

Result<void> MissingObjectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto template_data = get_input("template");

    if (!input_data.is_image() || !template_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input or template is not an image");
    }

    ImageData input = input_data.as_image();
    ImageData templ = template_data.as_image();

    if (input.empty() || templ.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input or template image is empty");
    }

    ImageData gray_input = ensure_grayscale(input);
    ImageData gray_templ = ensure_grayscale(templ);

    int threshold = get_param("threshold", Data(30)).as_int();
    int min_area = get_param("min_area", Data(50)).as_int();

    std::vector<uint8_t> missing_mask(gray_input.width * gray_input.height, 0);

    // 计算差异
    if (gray_input.width == gray_templ.width && gray_input.height == gray_templ.height) {
        double total_diff = 0;
        int diff_count = 0;

        for (size_t i = 0; i < gray_input.data.size(); ++i) {
            int diff = static_cast<int>(gray_templ.data[i]) - static_cast<int>(gray_input.data[i]);

            // 模板中有内容但输入中没有 -> 可能是缺失
            if (diff > threshold) {
                missing_mask[i] = 255;
                total_diff += diff;
                diff_count++;
            }
        }

        // 计算匹配得分
        double match_score = 1.0 - (total_diff / (gray_input.data.size() * 255.0));
        set_output("score", Data(match_score));
    } else {
        set_output("score", Data(0.0));
    }

    // 形态学操作
    defect_utils::dilate(missing_mask, gray_input.width, gray_input.height, 3);
    defect_utils::erode(missing_mask, gray_input.width, gray_input.height, 3);

    auto components = defect_utils::find_connected_components(missing_mask, gray_input.width, gray_input.height);

    std::vector<DefectResult> defects;
    int defect_id = 0;

    for (const auto& cc : components) {
        if (cc.pixel_count >= min_area) {
            DefectResult dr;
            dr.id = defect_id++;
            dr.type = "missing";
            dr.x = cc.x;
            dr.y = cc.y;
            dr.width = cc.width;
            dr.height = cc.height;
            dr.severity = std::min(1.0, cc.pixel_count / 300.0);
            dr.confidence = 0.8;
            defects.push_back(dr);
        }
    }

    set_output("defects", defect_results_to_array(defects));

    ImageData mask_img;
    mask_img.width = gray_input.width;
    mask_img.height = gray_input.height;
    mask_img.channels = 1;
    mask_img.format = ImageFormat::Mono8;
    mask_img.data = missing_mask;
    set_output("mask", Data(mask_img));

    OVF_INFO() << "MissingObjectNode detected " << defects.size() << " missing objects";

    return Result<void>::success();
}

// ============================================================================
// 注册节点
// ============================================================================

OVF_REGISTER_NODE(SurfaceDefectNode, "SurfaceDefect", SurfaceDefectNode::make_info())
OVF_REGISTER_NODE(ScratchDetectNode, "ScratchDetect", ScratchDetectNode::make_info())
OVF_REGISTER_NODE(CrackDetectNode, "CrackDetect", CrackDetectNode::make_info())
OVF_REGISTER_NODE(StainDetectNode, "StainDetect", StainDetectNode::make_info())
OVF_REGISTER_NODE(PitDetectNode, "PitDetect", PitDetectNode::make_info())
OVF_REGISTER_NODE(ShapeDefectNode, "ShapeDefect", ShapeDefectNode::make_info())
OVF_REGISTER_NODE(DimensionDefectNode, "DimensionDefect", DimensionDefectNode::make_info())
OVF_REGISTER_NODE(PositionDefectNode, "PositionDefect", PositionDefectNode::make_info())
OVF_REGISTER_NODE(AngleDefectNode, "AngleDefect", AngleDefectNode::make_info())
OVF_REGISTER_NODE(ColorDefectNode, "ColorDefect", ColorDefectNode::make_info())
OVF_REGISTER_NODE(TextureDefectNode, "TextureDefect", TextureDefectNode::make_info())
OVF_REGISTER_NODE(GlossDefectNode, "GlossDefect", GlossDefectNode::make_info())
OVF_REGISTER_NODE(OpacityDefectNode, "OpacityDefect", OpacityDefectNode::make_info())
OVF_REGISTER_NODE(ForeignObjectNode, "ForeignObject", ForeignObjectNode::make_info())
OVF_REGISTER_NODE(MissingObjectNode, "MissingObject", MissingObjectNode::make_info())

} // namespace algorithm
} // namespace ovf