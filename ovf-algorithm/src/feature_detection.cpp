/**
 * @file feature_detection.cpp
 * @brief 特征检测节点实现
 */

#include "ovf/algorithm/feature_detection.h"
#include "ovf/core/logger.h"
#include <unordered_map>

namespace ovf {
namespace algorithm {

// ==================== 辅助函数 ====================

// 转换为灰度图像
static ImageData to_gray(const ImageData& input) {
    if (input.channels == 1) return input;

    ImageData gray;
    gray.width = input.width;
    gray.height = input.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(gray.width * gray.height);
    gray.timestamp = input.timestamp;
    gray.frame_id = input.frame_id;
    gray.source_id = input.source_id;

    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            gray.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i];
        }
    }

    return gray;
}

// 转换为彩色图像（用于绘制）
static ImageData to_color(const ImageData& input) {
    if (input.channels == 3) return input;

    ImageData color;
    color.width = input.width;
    color.height = input.height;
    color.channels = 3;
    color.format = ImageFormat::RGB8;
    color.data.resize(color.width * color.height * 3);
    color.timestamp = input.timestamp;
    color.frame_id = input.frame_id;
    color.source_id = input.source_id;

    for (size_t i = 0; i < input.width * input.height; ++i) {
        uint8_t val = (input.channels == 1) ? input.data[i] : input.data[i * input.channels];
        color.data[i * 3] = val;
        color.data[i * 3 + 1] = val;
        color.data[i * 3 + 2] = val;
    }

    return color;
}

// ==================== Harris角点检测节点 ====================

HarrisCornerNode::HarrisCornerNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo HarrisCornerNode::make_info() {
    NodeInfo info;
    info.id = "HarrisCorner";
    info.name = "Harris角点检测";
    info.category = "特征检测";
    info.description = "使用Harris算法检测图像中的角点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("corner_count", "角点数量", DataType::Number));

    info.params.push_back(ParamDef("threshold", "响应阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("k", "Harris参数k", DataType::Number, Data(0.04)));
    info.params.push_back(ParamDef("block_size", "窗口大小", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("draw_corners", "绘制角点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> HarrisCornerNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 转换灰度
    ImageData gray = to_gray(input);

    // 参数
    int threshold = get_param("threshold", Data(50)).as_int();
    double k = get_param("k", Data(0.04)).as_number();
    int block_size = get_param("block_size", Data(3)).as_int();
    int max_corners = get_param("max_corners", Data(100)).as_int();
    bool draw = get_param("draw_corners", Data(true)).as_bool();

    // Harris角点检测
    std::vector<Corner> corners;
    feature_utils::harris_corner(gray, corners, k, threshold, block_size);

    // 限制数量
    if (static_cast<int>(corners.size()) > max_corners) {
        corners.resize(max_corners);
    }

    // 输出角点数量
    set_output("corner_count", Data(static_cast<int>(corners.size())));

    // 输出图像
    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_corners(output, corners, 255, 0, 0, 3);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Harris corner detection: found " << corners.size() << " corners";

    return Result<void>::success();
}

// ==================== FAST角点检测节点 ====================

FastCornerNode::FastCornerNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FastCornerNode::make_info() {
    NodeInfo info;
    info.id = "FastCorner";
    info.name = "FAST角点检测";
    info.category = "特征检测";
    info.description = "使用FAST算法检测图像中的角点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("corner_count", "角点数量", DataType::Number));

    info.params.push_back(ParamDef("threshold", "亮度阈值", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("draw_corners", "绘制角点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> FastCornerNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    int threshold = get_param("threshold", Data(20)).as_int();
    int max_corners = get_param("max_corners", Data(100)).as_int();
    bool draw = get_param("draw_corners", Data(true)).as_bool();

    std::vector<Corner> corners;
    feature_utils::fast_corner(gray, corners, threshold);

    if (static_cast<int>(corners.size()) > max_corners) {
        corners.resize(max_corners);
    }

    set_output("corner_count", Data(static_cast<int>(corners.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_corners(output, corners, 0, 255, 0, 3);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "FAST corner detection: found " << corners.size() << " corners";

    return Result<void>::success();
}

// ==================== Sobel角点检测节点 ====================

SobelCornerNode::SobelCornerNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SobelCornerNode::make_info() {
    NodeInfo info;
    info.id = "SobelCorner";
    info.name = "Sobel角点检测";
    info.category = "特征检测";
    info.description = "基于Sobel梯度方向变化检测角点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("corner_count", "角点数量", DataType::Number));

    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("draw_corners", "绘制角点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> SobelCornerNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    int threshold = get_param("threshold", Data(50)).as_int();
    int max_corners = get_param("max_corners", Data(100)).as_int();
    bool draw = get_param("draw_corners", Data(true)).as_bool();

    std::vector<Corner> corners;
    feature_utils::sobel_corner(gray, corners, threshold);

    if (static_cast<int>(corners.size()) > max_corners) {
        corners.resize(max_corners);
    }

    set_output("corner_count", Data(static_cast<int>(corners.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_corners(output, corners, 0, 0, 255, 3);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Sobel corner detection: found " << corners.size() << " corners";

    return Result<void>::success();
}

// ==================== Moravec角点检测节点 ====================

MoravecCornerNode::MoravecCornerNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MoravecCornerNode::make_info() {
    NodeInfo info;
    info.id = "MoravecCorner";
    info.name = "Moravec角点检测";
    info.category = "特征检测";
    info.description = "使用Moravec算法检测图像中的角点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("corner_count", "角点数量", DataType::Number));

    info.params.push_back(ParamDef("threshold", "响应阈值", DataType::Number, Data(10000)));
    info.params.push_back(ParamDef("window_size", "窗口大小", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("draw_corners", "绘制角点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> MoravecCornerNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    int threshold = get_param("threshold", Data(10000)).as_int();
    int window_size = get_param("window_size", Data(3)).as_int();
    int max_corners = get_param("max_corners", Data(100)).as_int();
    bool draw = get_param("draw_corners", Data(true)).as_bool();

    std::vector<Corner> corners;
    feature_utils::moravec_corner(gray, corners, threshold, window_size);

    if (static_cast<int>(corners.size()) > max_corners) {
        corners.resize(max_corners);
    }

    set_output("corner_count", Data(static_cast<int>(corners.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_corners(output, corners, 255, 255, 0, 3);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Moravec corner detection: found " << corners.size() << " corners";

    return Result<void>::success();
}

// ==================== 轮廓检测节点 ====================

ContourDetectNode::ContourDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContourDetectNode::make_info() {
    NodeInfo info;
    info.id = "ContourDetect";
    info.name = "轮廓检测";
    info.category = "特征检测";
    info.description = "检测二值图像中的轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带轮廓）", DataType::Image));
    info.outputs.push_back(DataPort("contour_count", "轮廓数量", DataType::Number));

    info.params.push_back(ParamDef("draw_contours", "绘制轮廓", DataType::Boolean, Data(true)));

    return info;
}

Result<void> ContourDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData binary = to_gray(input);
    bool draw = get_param("draw_contours", Data(true)).as_bool();

    std::vector<Contour> contours;
    feature_utils::find_contours(binary, contours, false);

    set_output("contour_count", Data(static_cast<int>(contours.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_contours(output, contours, 255, 255, 0);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Contour detection: found " << contours.size() << " contours";

    return Result<void>::success();
}

// ==================== 轮廓近似节点 ====================

ContourApproxNode::ContourApproxNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContourApproxNode::make_info() {
    NodeInfo info;
    info.id = "ContourApprox";
    info.name = "轮廓近似";
    info.category = "特征检测";
    info.description = "使用多边形拟合近似轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带近似轮廓）", DataType::Image));
    info.outputs.push_back(DataPort("contour_count", "轮廓数量", DataType::Number));

    info.params.push_back(ParamDef("epsilon", "近似精度", DataType::Number, Data(2.0)));
    info.params.push_back(ParamDef("draw_contours", "绘制轮廓", DataType::Boolean, Data(true)));

    return info;
}

Result<void> ContourApproxNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData binary = to_gray(input);
    double epsilon = get_param("epsilon", Data(2.0)).as_number();
    bool draw = get_param("draw_contours", Data(true)).as_bool();

    // 先检测轮廓
    std::vector<Contour> contours;
    feature_utils::find_contours(binary, contours, false);

    // 对每个轮廓进行近似
    std::vector<Contour> approx_contours;
    for (const auto& contour : contours) {
        Contour approx;
        feature_utils::approximate_contour(contour, approx, epsilon);
        approx_contours.push_back(approx);
    }

    set_output("contour_count", Data(static_cast<int>(approx_contours.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_contours(output, approx_contours, 0, 255, 255);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Contour approximation: processed " << approx_contours.size() << " contours";

    return Result<void>::success();
}

// ==================== 轮廓层次分析节点 ====================

ContourHierarchyNode::ContourHierarchyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContourHierarchyNode::make_info() {
    NodeInfo info;
    info.id = "ContourHierarchy";
    info.name = "轮廓层次分析";
    info.category = "特征检测";
    info.description = "分析轮廓的层次结构（外轮廓和孔洞）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带层次标注）", DataType::Image));
    info.outputs.push_back(DataPort("outer_count", "外轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("hole_count", "孔洞数量", DataType::Number));

    info.params.push_back(ParamDef("draw_hierarchy", "绘制层次", DataType::Boolean, Data(true)));

    return info;
}

Result<void> ContourHierarchyNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData binary = to_gray(input);
    bool draw = get_param("draw_hierarchy", Data(true)).as_bool();

    std::vector<Contour> contours;
    feature_utils::find_contours(binary, contours, true);

    int outer_count = 0;
    int hole_count = 0;

    std::vector<Contour> outer_contours;
    std::vector<Contour> hole_contours;

    for (const auto& c : contours) {
        if (c.is_hole) {
            hole_count++;
            hole_contours.push_back(c);
        } else {
            outer_count++;
            outer_contours.push_back(c);
        }
    }

    set_output("outer_count", Data(outer_count));
    set_output("hole_count", Data(hole_count));

    if (draw) {
        ImageData output = to_color(input);
        // 外轮廓用黄色
        feature_utils::draw_contours(output, outer_contours, 255, 255, 0);
        // 孔洞用红色
        feature_utils::draw_contours(output, hole_contours, 255, 0, 0);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Contour hierarchy: " << outer_count << " outer, " << hole_count << " holes";

    return Result<void>::success();
}

// ==================== 轮廓属性计算节点 ====================

ContourPropertyNode::ContourPropertyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContourPropertyNode::make_info() {
    NodeInfo info;
    info.id = "ContourProperty";
    info.name = "轮廓属性计算";
    info.category = "特征检测";
    info.description = "计算轮廓属性（面积、周长、凸包等）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带属性标注）", DataType::Image));
    info.outputs.push_back(DataPort("contour_count", "轮廓数量", DataType::Number));

    info.params.push_back(ParamDef("compute_hull", "计算凸包", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("draw_props", "绘制属性", DataType::Boolean, Data(true)));

    return info;
}

Result<void> ContourPropertyNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData binary = to_gray(input);
    bool compute_hull = get_param("compute_hull", Data(true)).as_bool();
    bool draw = get_param("draw_props", Data(true)).as_bool();

    std::vector<Contour> contours;
    feature_utils::find_contours(binary, contours, false);

    std::vector<ContourProperty> properties;
    for (const auto& c : contours) {
        ContourProperty prop = feature_utils::compute_contour_property(c);
        if (!compute_hull) {
            prop.convex_hull.clear();
        }
        properties.push_back(prop);

        OVF_DEBUG() << "Contour #" << prop.id << ": area=" << prop.area
                    << ", perimeter=" << prop.perimeter
                    << ", circularity=" << prop.circularity
                    << ", convexity=" << prop.convexity;
    }

    set_output("contour_count", Data(static_cast<int>(contours.size())));

    if (draw) {
        ImageData output = to_color(input);
        // 绘制轮廓
        feature_utils::draw_contours(output, contours, 255, 255, 0);

        // 绘制凸包
        if (compute_hull) {
            for (const auto& prop : properties) {
                if (prop.convex_hull.size() >= 3) {
                    Contour hull_contour;
                    hull_contour.points = prop.convex_hull;
                    feature_utils::draw_contours(output, {hull_contour}, 0, 255, 0);
                }
            }
        }
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Contour properties computed for " << contours.size() << " contours";

    return Result<void>::success();
}

// ==================== Blob检测节点 ====================

BlobDetectNode::BlobDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BlobDetectNode::make_info() {
    NodeInfo info;
    info.id = "BlobDetect";
    info.name = "Blob检测";
    info.category = "特征检测";
    info.description = "检测二值图像中的Blob（关键点）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("blob_count", "Blob数量", DataType::Number));

    info.params.push_back(ParamDef("min_area", "最小面积", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("max_area", "最大面积", DataType::Number, Data(100000)));
    info.params.push_back(ParamDef("draw_keypoints", "绘制关键点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> BlobDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData binary = to_gray(input);
    uint32_t min_area = static_cast<uint32_t>(get_param("min_area", Data(10)).as_int());
    uint32_t max_area = static_cast<uint32_t>(get_param("max_area", Data(100000)).as_int());
    bool draw = get_param("draw_keypoints", Data(true)).as_bool();

    std::vector<KeyPoint> keypoints;
    feature_utils::detect_blobs(binary, keypoints, min_area, max_area);

    set_output("blob_count", Data(static_cast<int>(keypoints.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_keypoints(output, keypoints, 0, 255, 0);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Blob detection: found " << keypoints.size() << " blobs";

    return Result<void>::success();
}

// ==================== 关键点匹配节点 ====================

KeyPointMatchNode::KeyPointMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo KeyPointMatchNode::make_info() {
    NodeInfo info;
    info.id = "KeyPointMatch";
    info.name = "关键点匹配";
    info.category = "特征检测";
    info.description = "匹配两幅图像中的关键点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("match_count", "匹配数量", DataType::Number));

    info.params.push_back(ParamDef("ratio_threshold", "比率阈值", DataType::Number, Data(0.75)));

    return info;
}

Result<void> KeyPointMatchNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");

    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();

    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);

    double ratio_threshold = get_param("ratio_threshold", Data(0.75)).as_number();

    // 检测关键点
    std::vector<KeyPoint> kp1, kp2;
    feature_utils::detect_blobs(gray1, kp1, 10, 100000);
    feature_utils::detect_blobs(gray2, kp2, 10, 100000);

    // 计算描述符
    std::vector<Descriptor> desc1, desc2;
    for (const auto& kp : kp1) {
        Descriptor d;
        feature_utils::compute_descriptor(gray1, kp, d);
        desc1.push_back(d);
    }
    for (const auto& kp : kp2) {
        Descriptor d;
        feature_utils::compute_descriptor(gray2, kp, d);
        desc2.push_back(d);
    }

    // 匹配
    std::vector<FeatureMatch> matches;
    feature_utils::match_keypoints(kp1, desc1, kp2, desc2, matches, ratio_threshold);

    set_output("match_count", Data(static_cast<int>(matches.size())));

    OVF_INFO() << "KeyPoint matching: " << kp1.size() << " vs " << kp2.size()
               << ", found " << matches.size() << " matches";

    return Result<void>::success();
}

// ==================== SIFT关键点检测节点 ====================

SIFTNode::SIFTNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SIFTNode::make_info() {
    NodeInfo info;
    info.id = "SIFT";
    info.name = "SIFT关键点检测";
    info.category = "特征检测";
    info.description = "使用SIFT算法检测关键点（简化版）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("keypoint_count", "关键点数量", DataType::Number));

    info.params.push_back(ParamDef("n_octaves", " octave数", DataType::Number, Data(4)));
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("max_keypoints", "最大关键点数", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("draw_keypoints", "绘制关键点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> SIFTNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    int n_octaves = get_param("n_octaves", Data(4)).as_int();
    int threshold = get_param("threshold", Data(10)).as_int();
    int max_keypoints = get_param("max_keypoints", Data(100)).as_int();
    bool draw = get_param("draw_keypoints", Data(true)).as_bool();

    std::vector<KeyPoint> keypoints;
    feature_utils::sift_keypoints(gray, keypoints, n_octaves, threshold);

    if (static_cast<int>(keypoints.size()) > max_keypoints) {
        keypoints.resize(max_keypoints);
    }

    set_output("keypoint_count", Data(static_cast<int>(keypoints.size())));

    if (draw) {
        ImageData output = to_color(input);
        feature_utils::draw_keypoints(output, keypoints, 255, 0, 255);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "SIFT keypoint detection: found " << keypoints.size() << " keypoints";

    return Result<void>::success();
}

// ==================== 角点匹配节点 ====================

CornerMatchNode::CornerMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CornerMatchNode::make_info() {
    NodeInfo info;
    info.id = "CornerMatch";
    info.name = "角点匹配";
    info.category = "特征检测";
    info.description = "匹配两幅图像中的角点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("match_count", "匹配数量", DataType::Number));

    info.params.push_back(ParamDef("search_radius", "搜索半径", DataType::Number, Data(50)));

    return info;
}

Result<void> CornerMatchNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");

    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();

    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);

    int search_radius = get_param("search_radius", Data(50)).as_int();

    // 检测角点
    std::vector<Corner> corners1, corners2;
    feature_utils::harris_corner(gray1, corners1, 0.04, 50, 3);
    feature_utils::harris_corner(gray2, corners2, 0.04, 50, 3);

    // 匹配
    std::vector<FeatureMatch> matches;
    feature_utils::match_corners(corners1, corners2, matches, search_radius);

    set_output("match_count", Data(static_cast<int>(matches.size())));

    OVF_INFO() << "Corner matching: " << corners1.size() << " vs " << corners2.size()
               << ", found " << matches.size() << " matches";

    return Result<void>::success();
}

// ==================== 特征匹配节点 ====================

FeatureMatchNode::FeatureMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FeatureMatchNode::make_info() {
    NodeInfo info;
    info.id = "FeatureMatch";
    info.name = "特征匹配";
    info.category = "特征检测";
    info.description = "基于描述符匹配两幅图像的特征";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("match_count", "匹配数量", DataType::Number));

    info.params.push_back(ParamDef("ratio_threshold", "比率阈值", DataType::Number, Data(0.75)));
    info.params.push_back(ParamDef("method", "检测方法", DataType::String, Data("SIFT")));

    return info;
}

Result<void> FeatureMatchNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");

    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();

    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);

    double ratio_threshold = get_param("ratio_threshold", Data(0.75)).as_number();
    String method = get_param("method", Data("SIFT")).as_string();

    // 根据方法选择检测器
    std::vector<KeyPoint> kp1, kp2;

    if (method == "SIFT") {
        feature_utils::sift_keypoints(gray1, kp1, 4, 10);
        feature_utils::sift_keypoints(gray2, kp2, 4, 10);
    } else if (method == "Blob") {
        feature_utils::detect_blobs(gray1, kp1, 10, 100000);
        feature_utils::detect_blobs(gray2, kp2, 10, 100000);
    } else {
        // 默认使用SIFT
        feature_utils::sift_keypoints(gray1, kp1, 4, 10);
        feature_utils::sift_keypoints(gray2, kp2, 4, 10);
    }

    // 计算描述符
    std::vector<Descriptor> desc1, desc2;
    for (const auto& kp : kp1) {
        Descriptor d;
        feature_utils::compute_descriptor(gray1, kp, d);
        desc1.push_back(d);
    }
    for (const auto& kp : kp2) {
        Descriptor d;
        feature_utils::compute_descriptor(gray2, kp, d);
        desc2.push_back(d);
    }

    // 匹配
    std::vector<FeatureMatch> matches;
    feature_utils::match_keypoints(kp1, desc1, kp2, desc2, matches, ratio_threshold);

    set_output("match_count", Data(static_cast<int>(matches.size())));

    OVF_INFO() << "Feature matching (" << method << "): " << kp1.size() << " vs " << kp2.size()
               << ", found " << matches.size() << " matches";

    return Result<void>::success();
}

// ==================== 特征跟踪节点 ====================

FeatureTrackNode::FeatureTrackNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FeatureTrackNode::make_info() {
    NodeInfo info;
    info.id = "FeatureTrack";
    info.name = "特征跟踪";
    info.category = "特征检测";
    info.description = "使用光流法跟踪特征点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("prev_image", "前一帧图像", DataType::Image, true));
    info.inputs.push_back(DataPort("curr_image", "当前帧图像", DataType::Image, true));
    info.outputs.push_back(DataPort("track_count", "跟踪成功数量", DataType::Number));
    info.outputs.push_back(DataPort("lost_count", "丢失数量", DataType::Number));

    info.params.push_back(ParamDef("window_size", "窗口大小", DataType::Number, Data(15)));
    info.params.push_back(ParamDef("max_iter", "最大迭代次数", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("max_features", "最大特征数", DataType::Number, Data(100)));

    return info;
}

Result<void> FeatureTrackNode::execute(FlowContext& context) {
    auto prev_data = get_input("prev_image");
    auto curr_data = get_input("curr_image");

    if (!prev_data.is_image() || !curr_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData prev_img = prev_data.as_image();
    ImageData curr_img = curr_data.as_image();

    if (prev_img.empty() || curr_img.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData prev_gray = to_gray(prev_img);
    ImageData curr_gray = to_gray(curr_img);

    int window_size = get_param("window_size", Data(15)).as_int();
    int max_iter = get_param("max_iter", Data(20)).as_int();
    int max_features = get_param("max_features", Data(100)).as_int();

    // 在前一帧检测特征点
    std::vector<Corner> corners;
    feature_utils::harris_corner(prev_gray, corners, 0.04, 50, 3);

    // 转换为关键点
    std::vector<KeyPoint> prev_kp;
    for (size_t i = 0; i < corners.size(); ++i) {
        KeyPoint kp;
        kp.id = static_cast<uint32_t>(i);
        kp.x = corners[i].x;
        kp.y = corners[i].y;
        kp.response = corners[i].response;
        kp.valid = corners[i].valid;
        prev_kp.push_back(kp);
    }

    if (static_cast<int>(prev_kp.size()) > max_features) {
        prev_kp.resize(max_features);
    }

    // 光流跟踪
    std::vector<TrackResult> tracks;
    feature_utils::track_features(prev_gray, curr_gray, prev_kp, tracks,
                                  window_size, max_iter);

    int track_count = 0;
    int lost_count = 0;
    for (const auto& t : tracks) {
        if (t.found) {
            track_count++;
        } else {
            lost_count++;
        }
    }

    set_output("track_count", Data(track_count));
    set_output("lost_count", Data(lost_count));

    OVF_INFO() << "Feature tracking: " << prev_kp.size() << " features, "
               << track_count << " tracked, " << lost_count << " lost";

    return Result<void>::success();
}

// ==================== 特征聚类节点 ====================

FeatureClusterNode::FeatureClusterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FeatureClusterNode::make_info() {
    NodeInfo info;
    info.id = "FeatureCluster";
    info.name = "特征聚类";
    info.category = "特征检测";
    info.description = "对特征点进行K-means聚类";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("cluster_count", "聚类数量", DataType::Number));
    info.outputs.push_back(DataPort("image", "输出图像（带聚类标注）", DataType::Image));

    info.params.push_back(ParamDef("k", "聚类数", DataType::Number, Data(4)));
    info.params.push_back(ParamDef("max_iter", "最大迭代次数", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("method", "检测方法", DataType::String, Data("Harris")));
    info.params.push_back(ParamDef("draw_clusters", "绘制聚类", DataType::Boolean, Data(true)));

    return info;
}

Result<void> FeatureClusterNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    int k = get_param("k", Data(4)).as_int();
    int max_iter = get_param("max_iter", Data(10)).as_int();
    String method = get_param("method", Data("Harris")).as_string();
    bool draw = get_param("draw_clusters", Data(true)).as_bool();

    // 检测特征点
    std::vector<KeyPoint> keypoints;

    // 使用角点作为特征点
    std::vector<Corner> corners;
    if (method == "Harris") {
        feature_utils::harris_corner(gray, corners, 0.04, 50, 3);
    } else if (method == "FAST") {
        feature_utils::fast_corner(gray, corners, 20);
    } else {
        feature_utils::harris_corner(gray, corners, 0.04, 50, 3);
    }

    // 转换为关键点
    for (size_t i = 0; i < corners.size(); ++i) {
        KeyPoint kp;
        kp.id = static_cast<uint32_t>(i);
        kp.x = corners[i].x;
        kp.y = corners[i].y;
        kp.response = corners[i].response;
        kp.valid = corners[i].valid;
        keypoints.push_back(kp);
    }

    // 聚类
    std::vector<std::vector<uint32_t>> clusters;
    feature_utils::cluster_features(keypoints, clusters, k, max_iter);

    set_output("cluster_count", Data(k));

    if (draw) {
        ImageData output = to_color(input);

        // 每个聚类用不同颜色
        uint8_t colors[][3] = {
            {255, 0, 0},   // 红
            {0, 255, 0},   // 绿
            {0, 0, 255},   // 蓝
            {255, 255, 0}, // 黄
            {255, 0, 255}, // 紫
            {0, 255, 255}, // 青
            {128, 0, 0},   // 深红
            {0, 128, 0}    // 深绿
        };

        for (size_t c = 0; c < clusters.size(); ++c) {
            uint8_t r = colors[c % 8][0];
            uint8_t g = colors[c % 8][1];
            uint8_t b = colors[c % 8][2];

            for (uint32_t idx : clusters[c]) {
                if (idx < keypoints.size()) {
                    int x = static_cast<int>(keypoints[idx].x);
                    int y = static_cast<int>(keypoints[idx].y);

                    // 绘制点
                    for (int dy = -2; dy <= 2; ++dy) {
                        for (int dx = -2; dx <= 2; ++dx) {
                            if (dx * dx + dy * dy <= 4) {
                                int px = x + dx;
                                int py = y + dy;

                                if (px >= 0 && px < static_cast<int>(output.width) &&
                                    py >= 0 && py < static_cast<int>(output.height)) {
                                    size_t idx = (py * output.width + px) * 3;
                                    output.data[idx] = b;
                                    output.data[idx + 1] = g;
                                    output.data[idx + 2] = r;
                                }
                            }
                        }
                    }
                }
            }
        }

        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Feature clustering: " << keypoints.size() << " features into " << k << " clusters";

    for (size_t c = 0; c < clusters.size(); ++c) {
        OVF_DEBUG() << "Cluster " << c << ": " << clusters[c].size() << " points";
    }

    return Result<void>::success();
}

// ==================== 注册节点 ====================

OVF_REGISTER_NODE(HarrisCornerNode, "HarrisCorner", HarrisCornerNode::make_info())
OVF_REGISTER_NODE(FastCornerNode, "FastCorner", FastCornerNode::make_info())
OVF_REGISTER_NODE(SobelCornerNode, "SobelCorner", SobelCornerNode::make_info())
OVF_REGISTER_NODE(MoravecCornerNode, "MoravecCorner", MoravecCornerNode::make_info())

OVF_REGISTER_NODE(ContourDetectNode, "ContourDetect", ContourDetectNode::make_info())
OVF_REGISTER_NODE(ContourApproxNode, "ContourApprox", ContourApproxNode::make_info())
OVF_REGISTER_NODE(ContourHierarchyNode, "ContourHierarchy", ContourHierarchyNode::make_info())
OVF_REGISTER_NODE(ContourPropertyNode, "ContourProperty", ContourPropertyNode::make_info())

OVF_REGISTER_NODE(BlobDetectNode, "BlobDetect", BlobDetectNode::make_info())
OVF_REGISTER_NODE(KeyPointMatchNode, "KeyPointMatch", KeyPointMatchNode::make_info())
OVF_REGISTER_NODE(SIFTNode, "SIFT", SIFTNode::make_info())

OVF_REGISTER_NODE(CornerMatchNode, "CornerMatch", CornerMatchNode::make_info())
OVF_REGISTER_NODE(FeatureMatchNode, "FeatureMatch", FeatureMatchNode::make_info())
OVF_REGISTER_NODE(FeatureTrackNode, "FeatureTrack", FeatureTrackNode::make_info())
OVF_REGISTER_NODE(FeatureClusterNode, "FeatureCluster", FeatureClusterNode::make_info())

} // namespace algorithm
} // namespace ovf