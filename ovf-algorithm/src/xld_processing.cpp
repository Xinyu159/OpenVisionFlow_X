/**
 * @file xld_processing.cpp
 * @brief XLD轮廓处理和亚像素边缘检测节点实现
 */

#include "ovf/algorithm/xld_processing.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/measurement.h"
#include <ctime>
#include <algorithm>
#include <sstream>

namespace ovf {
namespace algorithm {

//==============================================================================
// 辅助函数：将XLD轮廓转换为Data格式
//==============================================================================

namespace {

// 将轮廓数据序列化为字符串（用于Data传递）
String contours_to_string(const std::vector<XLDContour>& contours) {
    std::ostringstream oss;
    oss << "XLDContours[" << contours.size() << "]:\n";
    for (size_t i = 0; i < contours.size(); ++i) {
        const auto& c = contours[i];
        oss << "  Contour " << i << ": " << c.points.size() << " points, "
            << "length=" << c.length << ", area=" << c.area
            << ", closed=" << c.is_closed << "\n";
    }
    return oss.str();
}

// 将边缘数据序列化
String edges_to_string(const std::vector<SubPixelEdge>& edges) {
    std::ostringstream oss;
    oss << "SubPixelEdges[" << edges.size() << "]:\n";
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto& e = edges[i];
        oss << "  Edge " << i << ": pos=(" << e.position.x << "," << e.position.y
            << "), amp=" << e.amplitude << ", polarity=" << e.polarity << "\n";
    }
    return oss.str();
}

// 灰度转换
ImageData to_gray(const ImageData& input) {
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
            gray.data[i] = static_cast<uint8_t>(0.11f * b + 0.59f * g + 0.30f * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i];
        }
    }
    return gray;
}

// 轮廓排序辅助
struct ContourSortInfo {
    size_t index;
    float value;
};

}

//==============================================================================
// XLD轮廓生成节点（6个）
//==============================================================================

//------------------------------------------------------------------------------
// GenContourRegionNode
//------------------------------------------------------------------------------

GenContourRegionNode::GenContourRegionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GenContourRegionNode::make_info() {
    NodeInfo info;
    info.id = "GenContourRegion";
    info.name = "从区域生成轮廓";
    info.category = "XLD轮廓生成";
    info.description = "从二值区域生成亚像素精度轮廓（gen_contour_region_xld）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("min_length", "最小轮廓长度", DataType::Number, Data(10.0)));

    return info;
}

Result<void> GenContourRegionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 转换为灰度
    ImageData gray = to_gray(input);

    // 二值化（假设输入已经是二值图像，或者进行阈值处理）
    float min_length = get_param("min_length", Data(10.0)).as_number();

    // 从区域生成轮廓
    auto contours = xld_utils::contours_from_region(gray);

    // 按最小长度筛选
    std::vector<XLDContour> filtered;
    for (auto& c : contours) {
        if (c.length >= min_length) {
            filtered.push_back(std::move(c));
        }
    }

    set_output("contours", Data(static_cast<int>(filtered.size())));
    set_output("info", Data(contours_to_string(filtered)));

    OVF_INFO() << "GenContourRegion: generated " << filtered.size() << " contours";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// GenContoursSkeletonNode
//------------------------------------------------------------------------------

GenContoursSkeletonNode::GenContoursSkeletonNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GenContoursSkeletonNode::make_info() {
    NodeInfo info;
    info.id = "GenContoursSkeleton";
    info.name = "从骨架生成轮廓";
    info.category = "XLD轮廓生成";
    info.description = "从骨架图像生成线状轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入骨架图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    return info;
}

Result<void> GenContoursSkeletonNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    auto contours = xld_utils::contours_from_skeleton(gray);

    set_output("contours", Data(static_cast<int>(contours.size())));
    set_output("info", Data(contours_to_string(contours)));

    OVF_INFO() << "GenContoursSkeleton: generated " << contours.size() << " contours";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// GenCrossContourNode
//------------------------------------------------------------------------------

GenCrossContourNode::GenCrossContourNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GenCrossContourNode::make_info() {
    NodeInfo info;
    info.id = "GenCrossContour";
    info.name = "生成十字形轮廓";
    info.category = "XLD轮廓生成";
    info.description = "在指定位置生成十字形轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("x", "中心X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("y", "中心Y坐标", DataType::Number, true));

    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("size", "十字大小", DataType::Number, Data(20.0)));

    return info;
}

Result<void> GenCrossContourNode::execute(FlowContext& context) {
    float cx = get_input("x").as_number();
    float cy = get_input("y").as_number();
    float size = get_param("size", Data(20.0)).as_number();

    // 生成十字形轮廓（两条交叉线）
    XLDContour cross1, cross2;

    // 水平线
    cross1.is_closed = false;
    cross1.points.push_back(XLDPoint(cx - size, cy));
    cross1.points.push_back(XLDPoint(cx + size, cy));
    cross1.compute_properties();

    // 垂直线
    cross2.is_closed = false;
    cross2.points.push_back(XLDPoint(cx, cy - size));
    cross2.points.push_back(XLDPoint(cx, cy + size));
    cross2.compute_properties();

    std::vector<XLDContour> contours = {cross1, cross2};

    set_output("info", Data(contours_to_string(contours)));

    OVF_INFO() << "GenCrossContour: created cross at (" << cx << "," << cy << ") size=" << size;

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// GenPolygonContourNode
//------------------------------------------------------------------------------

GenPolygonContourNode::GenPolygonContourNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GenPolygonContourNode::make_info() {
    NodeInfo info;
    info.id = "GenPolygonContour";
    info.name = "生成多边形轮廓";
    info.category = "XLD轮廓生成";
    info.description = "根据顶点坐标生成多边形轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("vertices_x", "顶点X坐标数组（逗号分隔）", DataType::String, true));
    info.inputs.push_back(DataPort("vertices_y", "顶点Y坐标数组（逗号分隔）", DataType::String, true));

    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("closed", "是否闭合", DataType::Boolean, Data(true)));

    return info;
}

Result<void> GenPolygonContourNode::execute(FlowContext& context) {
    auto vx_str = get_input("vertices_x").as_string();
    auto vy_str = get_input("vertices_y").as_string();
    bool closed = get_param("closed", Data(true)).as_bool();

    // 解析坐标字符串
    std::vector<float> vx, vy;

    std::istringstream iss_x(vx_str);
    std::string token;
    while (std::getline(iss_x, token, ',')) {
        vx.push_back(std::stof(token));
    }

    std::istringstream iss_y(vy_str);
    while (std::getline(iss_y, token, ',')) {
        vy.push_back(std::stof(token));
    }

    if (vx.size() != vy.size() || vx.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Invalid vertices");
    }

    XLDContour polygon;
    polygon.is_closed = closed;

    for (size_t i = 0; i < vx.size(); ++i) {
        polygon.points.push_back(XLDPoint(vx[i], vy[i]));
    }

    polygon.compute_properties();

    std::vector<XLDContour> contours = {polygon};
    set_output("info", Data(contours_to_string(contours)));

    OVF_INFO() << "GenPolygonContour: created polygon with " << vx.size() << " vertices";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// GenArcContourNode
//------------------------------------------------------------------------------

GenArcContourNode::GenArcContourNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GenArcContourNode::make_info() {
    NodeInfo info;
    info.id = "GenArcContour";
    info.name = "生成弧形轮廓";
    info.category = "XLD轮廓生成";
    info.description = "生成指定参数的弧形轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("center_x", "中心X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("center_y", "中心Y坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("radius", "半径", DataType::Number, true));

    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("start_angle", "起始角度（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_angle", "结束角度（度）", DataType::Number, Data(90.0)));
    info.params.push_back(ParamDef("num_points", "采样点数", DataType::Number, Data(50)));

    return info;
}

Result<void> GenArcContourNode::execute(FlowContext& context) {
    float cx = get_input("center_x").as_number();
    float cy = get_input("center_y").as_number();
    float radius = get_input("radius").as_number();

    float start_angle = get_param("start_angle", Data(0.0)).as_number();
    float end_angle = get_param("end_angle", Data(90.0)).as_number();
    int num_points = get_param("num_points", Data(50)).as_int();

    XLDContour arc;
    arc.is_closed = false;

    float start_rad = start_angle * static_cast<float>(M_PI) / 180.0f;
    float end_rad = end_angle * static_cast<float>(M_PI) / 180.0f;

    for (int i = 0; i < num_points; ++i) {
        float t = static_cast<float>(i) / (num_points - 1);
        float angle = start_rad + t * (end_rad - start_rad);
        float x = cx + radius * std::cos(angle);
        float y = cy + radius * std::sin(angle);
        arc.points.push_back(XLDPoint(x, y));
    }

    arc.compute_properties();

    std::vector<XLDContour> contours = {arc};
    set_output("info", Data(contours_to_string(contours)));

    OVF_INFO() << "GenArcContour: created arc at (" << cx << "," << cy << ") radius=" << radius;

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// GenEllipseContourNode
//------------------------------------------------------------------------------

GenEllipseContourNode::GenEllipseContourNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GenEllipseContourNode::make_info() {
    NodeInfo info;
    info.id = "GenEllipseContour";
    info.name = "生成椭圆轮廓";
    info.category = "XLD轮廓生成";
    info.description = "生成指定参数的椭圆轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("center_x", "中心X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("center_y", "中心Y坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("semi_major", "长半轴", DataType::Number, true));
    info.inputs.push_back(DataPort("semi_minor", "短半轴", DataType::Number, true));

    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("angle", "旋转角度（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("num_points", "采样点数", DataType::Number, Data(100)));

    return info;
}

Result<void> GenEllipseContourNode::execute(FlowContext& context) {
    float cx = get_input("center_x").as_number();
    float cy = get_input("center_y").as_number();
    float semi_major = get_input("semi_major").as_number();
    float semi_minor = get_input("semi_minor").as_number();

    float angle_deg = get_param("angle", Data(0.0)).as_number();
    int num_points = get_param("num_points", Data(100)).as_int();

    XLDContour ellipse;
    ellipse.is_closed = true;

    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);

    for (int i = 0; i < num_points; ++i) {
        float t = static_cast<float>(i) / num_points * 2.0f * static_cast<float>(M_PI);

        // 椭圆参数方程
        float px = semi_major * std::cos(t);
        float py = semi_minor * std::sin(t);

        // 旋转
        float x = cx + px * cos_a - py * sin_a;
        float y = cy + px * sin_a + py * cos_a;

        ellipse.points.push_back(XLDPoint(x, y));
    }

    ellipse.compute_properties();

    std::vector<XLDContour> contours = {ellipse};
    set_output("info", Data(contours_to_string(contours)));

    OVF_INFO() << "GenEllipseContour: created ellipse at (" << cx << "," << cy << ")";

    return Result<void>::success();
}

//==============================================================================
// XLD轮廓处理节点（6个）
//==============================================================================

//------------------------------------------------------------------------------
// SelectContoursNode
//------------------------------------------------------------------------------

SelectContoursNode::SelectContoursNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SelectContoursNode::make_info() {
    NodeInfo info;
    info.id = "SelectContours";
    info.name = "轮廓选择";
    info.category = "XLD轮廓处理";
    info.description = "根据长度、方向、曲率等属性选择轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "选中轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("min_length", "最小长度", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("max_length", "最大长度", DataType::Number, Data(10000.0)));
    info.params.push_back(ParamDef("min_area", "最小面积", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("max_area", "最大面积", DataType::Number, Data(100000.0)));
    info.params.push_back(ParamDef("min_circularity", "最小圆度", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("direction", "方向范围（度）", DataType::String, Data("0,180")));

    return info;
}

Result<void> SelectContoursNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float min_length = get_param("min_length", Data(10.0)).as_number();
    float max_length = get_param("max_length", Data(10000.0)).as_number();
    float min_area = get_param("min_area", Data(0.0)).as_number();
    float max_area = get_param("max_area", Data(100000.0)).as_number();
    float min_circularity = get_param("min_circularity", Data(0.0)).as_number();

    // 解析方向范围
    auto dir_str = get_param("direction", Data("0,180")).as_string();
    float dir_min = 0.0f, dir_max = 180.0f;
    std::istringstream iss(dir_str);
    std::string token;
    if (std::getline(iss, token, ',')) dir_min = std::stof(token);
    if (std::getline(iss, token, ',')) dir_max = std::stof(token);

    dir_min = dir_min * static_cast<float>(M_PI) / 180.0f;
    dir_max = dir_max * static_cast<float>(M_PI) / 180.0f;

    // 生成轮廓
    auto contours = xld_utils::contours_from_region(gray);

    // 筛选轮廓
    std::vector<XLDContour> selected;
    for (auto& c : contours) {
        // 长度筛选
        if (c.length < min_length || c.length > max_length) continue;

        // 面积筛选
        if (c.area < min_area || c.area > max_area) continue;

        // 圆度筛选
        if (c.circularity < min_circularity) continue;

        // 方向筛选
        float angle_deg = c.angle * 180.0f / static_cast<float>(M_PI);
        if (angle_deg < dir_min * 180.0f / static_cast<float>(M_PI) ||
            angle_deg > dir_max * 180.0f / static_cast<float>(M_PI)) continue;

        selected.push_back(std::move(c));
    }

    set_output("contours", Data(static_cast<int>(selected.size())));
    set_output("info", Data(contours_to_string(selected)));

    OVF_INFO() << "SelectContours: selected " << selected.size() << " contours";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// SegmentContoursNode
//------------------------------------------------------------------------------

SegmentContoursNode::SegmentContoursNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SegmentContoursNode::make_info() {
    NodeInfo info;
    info.id = "SegmentContours";
    info.name = "轮廓分割";
    info.category = "XLD轮廓处理";
    info.description = "根据曲率变化将轮廓分割为多个子轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "分割后轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("curvature_threshold", "曲率阈值", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("min_segment_length", "最小段长度", DataType::Number, Data(5.0)));

    return info;
}

Result<void> SegmentContoursNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float curvature_threshold = get_param("curvature_threshold", Data(0.5)).as_number();
    float min_segment_length = get_param("min_segment_length", Data(5.0)).as_number();

    auto contours = xld_utils::contours_from_region(gray);

    std::vector<XLDContour> segmented;

    for (auto& c : contours) {
        if (c.points.size() < 3) {
            segmented.push_back(std::move(c));
            continue;
        }

        // 根据曲率分割轮廓
        std::vector<size_t> break_points;

        for (size_t i = 1; i < c.points.size() - 1; ++i) {
            float curv = xld_utils::curvature(c.points[i-1], c.points[i], c.points[i+1]);

            if (std::abs(curv) > curvature_threshold) {
                break_points.push_back(i);
            }
        }

        // 如果没有分割点，保留原轮廓
        if (break_points.empty()) {
            segmented.push_back(std::move(c));
            continue;
        }

        // 添加起点和终点
        break_points.insert(break_points.begin(), 0);
        break_points.push_back(c.points.size() - 1);

        // 创建子轮廓
        for (size_t i = 1; i < break_points.size(); ++i) {
            size_t start = break_points[i-1];
            size_t end = break_points[i];

            XLDContour segment;
            segment.is_closed = false;

            for (size_t j = start; j <= end; ++j) {
                segment.points.push_back(c.points[j]);
            }

            segment.compute_properties();

            if (segment.length >= min_segment_length) {
                segmented.push_back(std::move(segment));
            }
        }
    }

    set_output("contours", Data(static_cast<int>(segmented.size())));
    set_output("info", Data(contours_to_string(segmented)));

    OVF_INFO() << "SegmentContours: segmented into " << segmented.size() << " contours";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// ApproxChainNode
//------------------------------------------------------------------------------

ApproxChainNode::ApproxChainNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ApproxChainNode::make_info() {
    NodeInfo info;
    info.id = "ApproxChain";
    info.name = "轮廓近似";
    info.category = "XLD轮廓处理";
    info.description = "使用Douglas-Peucker算法简化轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));
    info.outputs.push_back(DataPort("points_reduced", "减少的点数", DataType::Number));

    info.params.push_back(ParamDef("epsilon", "近似精度", DataType::Number, Data(2.0)));

    return info;
}

Result<void> ApproxChainNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float epsilon = get_param("epsilon", Data(2.0)).as_number();

    auto contours = xld_utils::contours_from_region(gray);

    size_t total_original_points = 0;
    size_t total_approx_points = 0;

    std::vector<XLDContour> approximated;

    for (auto& c : contours) {
        total_original_points += c.points.size();

        // Douglas-Peucker简化
        auto simplified = xld_utils::simplify_contour(c.points, epsilon);

        XLDContour approx;
        approx.is_closed = c.is_closed;
        approx.points = std::move(simplified);
        approx.compute_properties();

        total_approx_points += approx.points.size();

        approximated.push_back(std::move(approx));
    }

    set_output("contours", Data(static_cast<int>(approximated.size())));
    set_output("info", Data(contours_to_string(approximated)));
    set_output("points_reduced", Data(static_cast<int>(total_original_points - total_approx_points)));

    OVF_INFO() << "ApproxChain: reduced " << total_original_points << " to " << total_approx_points << " points";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// SmoothContoursNode
//------------------------------------------------------------------------------

SmoothContoursNode::SmoothContoursNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SmoothContoursNode::make_info() {
    NodeInfo info;
    info.id = "SmoothContours";
    info.name = "轮廓平滑";
    info.category = "XLD轮廓处理";
    info.description = "使用高斯滤波平滑轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));

    return info;
}

Result<void> SmoothContoursNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float sigma = get_param("sigma", Data(1.0)).as_number();

    auto contours = xld_utils::contours_from_region(gray);

    std::vector<XLDContour> smoothed;

    for (auto& c : contours) {
        if (c.points.size() < 3) {
            smoothed.push_back(std::move(c));
            continue;
        }

        // 提取坐标
        std::vector<float> x_coords, y_coords;
        for (const auto& pt : c.points) {
            x_coords.push_back(pt.x);
            y_coords.push_back(pt.y);
        }

        // 高斯平滑
        xld_utils::gaussian_smooth(x_coords, sigma);
        xld_utils::gaussian_smooth(y_coords, sigma);

        XLDContour smooth;
        smooth.is_closed = c.is_closed;

        for (size_t i = 0; i < x_coords.size(); ++i) {
            smooth.points.push_back(XLDPoint(x_coords[i], y_coords[i]));
        }

        smooth.compute_properties();
        smoothed.push_back(std::move(smooth));
    }

    set_output("contours", Data(static_cast<int>(smoothed.size())));
    set_output("info", Data(contours_to_string(smoothed)));

    OVF_INFO() << "SmoothContours: smoothed " << smoothed.size() << " contours";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// MergeContoursNode
//------------------------------------------------------------------------------

MergeContoursNode::MergeContoursNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MergeContoursNode::make_info() {
    NodeInfo info;
    info.id = "MergeContours";
    info.name = "轮廓合并";
    info.category = "XLD轮廓处理";
    info.description = "合并相近的轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "合并后轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("max_gap", "最大间隙距离", DataType::Number, Data(5.0)));

    return info;
}

Result<void> MergeContoursNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float max_gap = get_param("max_gap", Data(5.0)).as_number();

    auto contours = xld_utils::contours_from_region(gray);

    std::vector<XLDContour> merged;
    std::vector<bool> used(contours.size(), false);

    for (size_t i = 0; i < contours.size(); ++i) {
        if (used[i]) continue;

        XLDContour current = contours[i];
        used[i] = true;

        // 查找可合并的轮廓
        bool found_merge = true;
        while (found_merge) {
            found_merge = false;

            for (size_t j = 0; j < contours.size(); ++j) {
                if (used[j]) continue;

                const auto& other = contours[j];

                // 检查端点距离
                float dist_start_start = current.points.front().distance_to(other.points.front());
                float dist_start_end = current.points.front().distance_to(other.points.back());
                float dist_end_start = current.points.back().distance_to(other.points.front());
                float dist_end_end = current.points.back().distance_to(other.points.back());

                float min_dist = std::min({dist_start_start, dist_start_end,
                                          dist_end_start, dist_end_end});

                if (min_dist < max_gap) {
                    // 合并轮廓
                    if (min_dist == dist_end_start) {
                        // current尾部连接other头部
                        current.points.insert(current.points.end(),
                                             other.points.begin(), other.points.end());
                    } else if (min_dist == dist_end_end) {
                        // current尾部连接other尾部（反转other）
                        for (auto it = other.points.rbegin(); it != other.points.rend(); ++it) {
                            current.points.push_back(*it);
                        }
                    } else if (min_dist == dist_start_start) {
                        // current头部连接other头部（反转current）
                        std::reverse(current.points.begin(), current.points.end());
                        current.points.insert(current.points.end(),
                                             other.points.begin(), other.points.end());
                    } else if (min_dist == dist_start_end) {
                        // current头部连接other尾部（反转current）
                        std::reverse(current.points.begin(), current.points.end());
                        for (auto it = other.points.rbegin(); it != other.points.rend(); ++it) {
                            current.points.push_back(*it);
                        }
                    }

                    current.compute_properties();
                    used[j] = true;
                    found_merge = true;
                    break;
                }
            }
        }

        merged.push_back(std::move(current));
    }

    set_output("contours", Data(static_cast<int>(merged.size())));
    set_output("info", Data(contours_to_string(merged)));

    OVF_INFO() << "MergeContours: merged into " << merged.size() << " contours";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// ClipContoursNode
//------------------------------------------------------------------------------

ClipContoursNode::ClipContoursNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ClipContoursNode::make_info() {
    NodeInfo info;
    info.id = "ClipContours";
    info.name = "轮廓裁剪";
    info.category = "XLD轮廓处理";
    info.description = "裁剪轮廓到指定区域";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入二值图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contours", "裁剪后轮廓数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "轮廓信息", DataType::String));

    info.params.push_back(ParamDef("min_x", "最小X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("min_y", "最小Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("max_x", "最大X", DataType::Number, Data(10000.0)));
    info.params.push_back(ParamDef("max_y", "最大Y", DataType::Number, Data(10000.0)));

    return info;
}

Result<void> ClipContoursNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float min_x = get_param("min_x", Data(0.0)).as_number();
    float min_y = get_param("min_y", Data(0.0)).as_number();
    float max_x = get_param("max_x", Data(10000.0)).as_number();
    float max_y = get_param("max_y", Data(10000.0)).as_number();

    auto contours = xld_utils::contours_from_region(gray);

    std::vector<XLDContour> clipped;

    for (auto& c : contours) {
        XLDContour clip;
        clip.is_closed = false;

        for (const auto& pt : c.points) {
            if (pt.x >= min_x && pt.x <= max_x && pt.y >= min_y && pt.y <= max_y) {
                clip.points.push_back(pt);
            }
        }

        if (clip.points.size() > 1) {
            clip.compute_properties();
            clipped.push_back(std::move(clip));
        }
    }

    set_output("contours", Data(static_cast<int>(clipped.size())));
    set_output("info", Data(contours_to_string(clipped)));

    OVF_INFO() << "ClipContours: clipped to " << clipped.size() << " contours";

    return Result<void>::success();
}

//==============================================================================
// 亚像素边缘检测节点（4个）
//==============================================================================

//------------------------------------------------------------------------------
// EdgesSubPixNode
//------------------------------------------------------------------------------

EdgesSubPixNode::EdgesSubPixNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo EdgesSubPixNode::make_info() {
    NodeInfo info;
    info.id = "EdgesSubPix";
    info.name = "亚像素边缘检测";
    info.category = "亚像素边缘检测";
    info.description = "使用梯度方向极值插值检测亚像素边缘";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("edges", "边缘数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "边缘信息", DataType::String));

    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("min_length", "最小边缘长度", DataType::Number, Data(5.0)));

    return info;
}

Result<void> EdgesSubPixNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float threshold = get_param("threshold", Data(30.0)).as_number();

    std::vector<SubPixelEdge> edges;

    // 在图像中寻找梯度极值点
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            SubPixelEdge edge;
            if (xld_utils::subpixel_edge_position(gray, static_cast<float>(x),
                                                  static_cast<float>(y),
                                                  edge.position, edge.amplitude,
                                                  edge.direction, edge.polarity)) {
                if (edge.amplitude > threshold) {
                    edge.angle = std::atan2(edge.direction.y, edge.direction.x);
                    edges.push_back(edge);
                }
            }
        }
    }

    set_output("edges", Data(static_cast<int>(edges.size())));
    set_output("info", Data(edges_to_string(edges)));

    OVF_INFO() << "EdgesSubPix: found " << edges.size() << " subpixel edges";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// EdgesSubPixSobelNode
//------------------------------------------------------------------------------

EdgesSubPixSobelNode::EdgesSubPixSobelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo EdgesSubPixSobelNode::make_info() {
    NodeInfo info;
    info.id = "EdgesSubPixSobel";
    info.name = "Sobel亚像素边缘";
    info.category = "亚像素边缘检测";
    info.description = "使用Sobel算子检测亚像素边缘";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("edges", "边缘数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "边缘信息", DataType::String));

    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));

    return info;
}

Result<void> EdgesSubPixSobelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float threshold = get_param("threshold", Data(20.0)).as_number();

    std::vector<SubPixelEdge> edges;

    // 使用Sobel计算梯度并找极值
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            // 计算梯度
            float gx = 0.0f, gy = 0.0f;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    float val = static_cast<float>(gray.data[(y + dy) * gray.width + (x + dx)]);
                    gx += val * dx * (dy == 0 ? 2 : 1);
                    gy += val * dy * (dx == 0 ? 2 : 1);
                }
            }

            float mag = std::sqrt(gx * gx + gy * gy);

            if (mag > threshold) {
                // 检查是否为局部极值
                bool is_max = true;
                float grad_len = mag;
                float nx = gx / grad_len;
                float ny = gy / grad_len;

                // 检查梯度方向邻域
                float mag_plus = 0.0f, mag_minus = 0.0f;

                int nx_int = static_cast<int>(std::round(nx));
                int ny_int = static_cast<int>(std::round(ny));

                if (x + nx_int < gray.width && y + ny_int < gray.height) {
                    float gx2 = 0.0f, gy2 = 0.0f;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            float val = static_cast<float>(gray.data[(y + ny_int + dy) * gray.width + (x + nx_int + dx)]);
                            gx2 += val * dx * (dy == 0 ? 2 : 1);
                            gy2 += val * dy * (dx == 0 ? 2 : 1);
                        }
                    }
                    mag_plus = std::sqrt(gx2 * gx2 + gy2 * gy2);
                }

                if (x - nx_int >= 0 && y - ny_int >= 0) {
                    float gx2 = 0.0f, gy2 = 0.0f;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            float val = static_cast<float>(gray.data[(y - ny_int + dy) * gray.width + (x - nx_int + dx)]);
                            gx2 += val * dx * (dy == 0 ? 2 : 1);
                            gy2 += val * dy * (dx == 0 ? 2 : 1);
                        }
                    }
                    mag_minus = std::sqrt(gx2 * gx2 + gy2 * gy2);
                }

                is_max = (mag >= mag_plus && mag >= mag_minus);

                if (is_max) {
                    SubPixelEdge edge;
                    edge.amplitude = mag;
                    edge.direction.x = -ny;
                    edge.direction.y = nx;
                    edge.angle = std::atan2(edge.direction.y, edge.direction.x);

                    // 亚像素定位
                    if (std::abs(mag_plus + mag_minus - 2 * mag) > 1e-10f) {
                        float offset = (mag_plus - mag_minus) / (2.0f * (mag_plus + mag_minus - 2 * mag));
                        offset = std::max(-0.5f, std::min(0.5f, offset));

                        edge.position.x = static_cast<float>(x) + nx * offset;
                        edge.position.y = static_cast<float>(y) + ny * offset;
                    } else {
                        edge.position.x = static_cast<float>(x);
                        edge.position.y = static_cast<float>(y);
                    }

                    // 极性
                    float v_plus = xld_utils::bilinear_interpolate(gray,
                        edge.position.x + nx, edge.position.y + ny);
                    float v_minus = xld_utils::bilinear_interpolate(gray,
                        edge.position.x - nx, edge.position.y - ny);
                    edge.polarity = (v_minus > v_plus) ? 1 : -1;

                    edges.push_back(edge);
                }
            }
        }
    }

    set_output("edges", Data(static_cast<int>(edges.size())));
    set_output("info", Data(edges_to_string(edges)));

    OVF_INFO() << "EdgesSubPixSobel: found " << edges.size() << " edges";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// EdgesSubPixCannyNode
//------------------------------------------------------------------------------

EdgesSubPixCannyNode::EdgesSubPixCannyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo EdgesSubPixCannyNode::make_info() {
    NodeInfo info;
    info.id = "EdgesSubPixCanny";
    info.name = "Canny亚像素边缘";
    info.category = "亚像素边缘检测";
    info.description = "使用Canny算法检测亚像素边缘";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("edges", "边缘数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "边缘信息", DataType::String));

    info.params.push_back(ParamDef("low_threshold", "低阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("high_threshold", "高阈值", DataType::Number, Data(50.0)));

    return info;
}

Result<void> EdgesSubPixCannyNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float low_threshold = get_param("low_threshold", Data(20.0)).as_number();
    float high_threshold = get_param("high_threshold", Data(50.0)).as_number();

    // 梯度幅值图
    std::vector<float> gradient_mag(gray.width * gray.height, 0.0f);
    std::vector<float> gradient_dir_x(gray.width * gray.height, 0.0f);
    std::vector<float> gradient_dir_y(gray.width * gray.height, 0.0f);

    // 计算梯度
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            float gx = 0.0f, gy = 0.0f;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    float val = static_cast<float>(gray.data[(y + dy) * gray.width + (x + dx)]);
                    gx += val * dx * (dy == 0 ? 2 : 1);
                    gy += val * dy * (dx == 0 ? 2 : 1);
                }
            }

            size_t idx = y * gray.width + x;
            gradient_mag[idx] = std::sqrt(gx * gx + gy * gy);

            if (gradient_mag[idx] > 1e-10f) {
                gradient_dir_x[idx] = gx / gradient_mag[idx];
                gradient_dir_y[idx] = gy / gradient_mag[idx];
            }
        }
    }

    // 非极大值抑制 + 亚像素定位
    std::vector<SubPixelEdge> edges;
    std::vector<bool> is_edge(gray.width * gray.height, false);

    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            size_t idx = y * gray.width + x;
            float mag = gradient_mag[idx];

            if (mag < low_threshold) continue;

            float nx = gradient_dir_x[idx];
            float ny = gradient_dir_y[idx];

            // 检查是否为局部极值
            float mag_plus = 0.0f, mag_minus = 0.0f;
            int x1 = static_cast<int>(x + std::round(nx));
            int y1 = static_cast<int>(y + std::round(ny));
            int x0 = static_cast<int>(x - std::round(nx));
            int y0 = static_cast<int>(y - std::round(ny));

            if (x1 >= 0 && x1 < static_cast<int>(gray.width) &&
                y1 >= 0 && y1 < static_cast<int>(gray.height)) {
                mag_plus = gradient_mag[y1 * gray.width + x1];
            }

            if (x0 >= 0 && x0 < static_cast<int>(gray.width) &&
                y0 >= 0 && y0 < static_cast<int>(gray.height)) {
                mag_minus = gradient_mag[y0 * gray.width + x0];
            }

            bool is_local_max = (mag >= mag_plus && mag >= mag_minus);

            if (is_local_max && mag >= high_threshold) {
                // 强边缘
                SubPixelEdge edge;

                // 亚像素定位
                if (std::abs(mag_plus + mag_minus - 2 * mag) > 1e-10f) {
                    float offset = (mag_plus - mag_minus) / (2.0f * (mag_plus + mag_minus - 2 * mag));
                    offset = std::max(-0.5f, std::min(0.5f, offset));
                    edge.position.x = static_cast<float>(x) + nx * offset;
                    edge.position.y = static_cast<float>(y) + ny * offset;
                } else {
                    edge.position.x = static_cast<float>(x);
                    edge.position.y = static_cast<float>(y);
                }

                edge.amplitude = mag;
                edge.direction.x = -ny;
                edge.direction.y = nx;
                edge.angle = std::atan2(edge.direction.y, edge.direction.x);

                float v_plus = xld_utils::bilinear_interpolate(gray,
                    edge.position.x + nx, edge.position.y + ny);
                float v_minus = xld_utils::bilinear_interpolate(gray,
                    edge.position.x - nx, edge.position.y - ny);
                edge.polarity = (v_minus > v_plus) ? 1 : -1;

                edges.push_back(edge);
                is_edge[idx] = true;
            }
        }
    }

    // 边缘连接（弱边缘连接到强边缘）
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            size_t idx = y * gray.width + x;
            if (is_edge[idx]) continue;

            float mag = gradient_mag[idx];
            if (mag < low_threshold || mag >= high_threshold) continue;

            // 检查是否有强边缘邻居
            bool has_strong_neighbor = false;
            for (int dy = -1; dy <= 1 && !has_strong_neighbor; ++dy) {
                for (int dx = -1; dx <= 1 && !has_strong_neighbor; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = static_cast<int>(x) + dx;
                    int ny = static_cast<int>(y) + dy;
                    if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                        ny >= 0 && ny < static_cast<int>(gray.height)) {
                        if (is_edge[ny * gray.width + nx]) {
                            has_strong_neighbor = true;
                        }
                    }
                }
            }

            if (has_strong_neighbor) {
                SubPixelEdge edge;
                edge.position.x = static_cast<float>(x);
                edge.position.y = static_cast<float>(y);
                edge.amplitude = mag;
                edge.direction.x = -gradient_dir_y[idx];
                edge.direction.y = gradient_dir_x[idx];
                edge.angle = std::atan2(edge.direction.y, edge.direction.x);
                edge.polarity = 0;

                edges.push_back(edge);
            }
        }
    }

    set_output("edges", Data(static_cast<int>(edges.size())));
    set_output("info", Data(edges_to_string(edges)));

    OVF_INFO() << "EdgesSubPixCanny: found " << edges.size() << " edges";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// LinesSubPixNode
//------------------------------------------------------------------------------

LinesSubPixNode::LinesSubPixNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo LinesSubPixNode::make_info() {
    NodeInfo info;
    info.id = "LinesSubPix";
    info.name = "亚像素直线检测";
    info.category = "亚像素边缘检测";
    info.description = "检测亚像素精度的直线边缘";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("lines", "检测到的直线数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "直线信息", DataType::String));

    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("min_length", "最小直线长度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("max_gap", "最大间隙", DataType::Number, Data(5.0)));

    return info;
}

Result<void> LinesSubPixNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float threshold = get_param("threshold", Data(30.0)).as_number();
    float min_length = get_param("min_length", Data(20.0)).as_number();
    float max_gap = get_param("max_gap", Data(5.0)).as_number();

    // 首先检测亚像素边缘
    std::vector<SubPixelEdge> all_edges;

    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            SubPixelEdge edge;
            if (xld_utils::subpixel_edge_position(gray, static_cast<float>(x),
                                                  static_cast<float>(y),
                                                  edge.position, edge.amplitude,
                                                  edge.direction, edge.polarity)) {
                if (edge.amplitude > threshold) {
                    edge.angle = std::atan2(edge.direction.y, edge.direction.x);
                    all_edges.push_back(edge);
                }
            }
        }
    }

    // 按角度分组边缘点
    std::vector<std::vector<XLDPoint>> angle_groups;

    float angle_step = 5.0f * static_cast<float>(M_PI) / 180.0f;  // 5度间隔

    for (float base_angle = 0; base_angle < static_cast<float>(M_PI); base_angle += angle_step) {
        std::vector<XLDPoint> group;

        for (const auto& edge : all_edges) {
            float angle_diff = std::abs(edge.angle - base_angle);
            // 角度可能相差180度
            if (angle_diff > static_cast<float>(M_PI) * 0.5f) {
                angle_diff = static_cast<float>(M_PI) - angle_diff;
            }

            if (angle_diff < angle_step * 0.5f) {
                group.push_back(edge.position);
            }
        }

        if (group.size() >= 2) {
            angle_groups.push_back(std::move(group));
        }
    }

    // 对每个角度组进行直线拟合
    std::vector<Line2D> lines;
    std::ostringstream info_stream;
    info_stream << "Lines[";

    int line_count = 0;
    for (auto& group : angle_groups) {
        // 使用最小二乘拟合直线
        if (group.size() < 2) continue;

        // 转换XLDPoint到Point2Df
        std::vector<Point2Df> pts;
        for (const auto& pt : group) {
            pts.push_back(Point2Df(pt.x, pt.y));
        }

        Line2D line = measurement_utils::fit_line(pts);
        if (std::abs(line.a) < 1e-10f && std::abs(line.b) < 1e-10f) continue;

        // 计算直线长度（投影范围）
        float min_proj = 1e10f, max_proj = -1e10f;
        float cx = 0, cy = 0;

        for (const auto& pt : group) {
            // 沿直线方向投影
            float proj = pt.x * (-line.b) + pt.y * line.a;  // 沿直线方向
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
            cx += pt.x;
            cy += pt.y;
        }

        float line_length = max_proj - min_proj;

        if (line_length >= min_length) {
            lines.push_back(line);
            line_count++;

            cx /= group.size();
            cy /= group.size();

            float angle_deg = std::atan2(-line.a, line.b) * 180.0f / static_cast<float>(M_PI);

            info_stream << "\n  Line " << line_count << ": angle=" << angle_deg
                       << " deg, length=" << line_length
                       << ", center=(" << cx << "," << cy << ")";
        }
    }

    info_stream << "]";

    set_output("lines", Data(line_count));
    set_output("info", Data(info_stream.str()));

    OVF_INFO() << "LinesSubPix: detected " << line_count << " lines";

    return Result<void>::success();
}

//==============================================================================
// 边缘对测量节点（4个）
//==============================================================================

//------------------------------------------------------------------------------
// MeasurePairsNode
//------------------------------------------------------------------------------

MeasurePairsNode::MeasurePairsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasurePairsNode::make_info() {
    NodeInfo info;
    info.id = "MeasurePairs";
    info.name = "边缘对测量";
    info.category = "边缘对测量";
    info.description = "使用卡尺工具测量边缘对（找边对）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("start_x", "起始X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("start_y", "起始Y坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("end_x", "结束X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("end_y", "结束Y坐标", DataType::Number, true));

    info.outputs.push_back(DataPort("pairs", "边缘对数量", DataType::Number));
    info.outputs.push_back(DataPort("distances", "边缘对间距数组", DataType::String));
    info.outputs.push_back(DataPort("centers", "边缘对中心数组", DataType::String));

    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("width", "搜索宽度", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("polarity", "极性模式", DataType::String, Data("positive")));

    return info;
}

Result<void> MeasurePairsNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float start_x = get_input("start_x").as_number();
    float start_y = get_input("start_y").as_number();
    float end_x = get_input("end_x").as_number();
    float end_y = get_input("end_y").as_number();

    float threshold = get_param("threshold", Data(20.0)).as_number();
    int width = get_param("width", Data(1.0)).as_int();

    auto polarity_str = get_param("polarity", Data("positive")).as_string();
    int polarity = 0;
    if (polarity_str == "positive") polarity = 1;
    else if (polarity_str == "negative") polarity = -1;
    else polarity = 0;

    // 沿投影线采样边缘
    auto edges = xld_utils::sample_edges_along_line(gray, start_x, start_y,
                                                    end_x, end_y, threshold, polarity);

    // 构建边缘对（相邻两个边缘为一对）
    std::vector<EdgePair> pairs;

    for (size_t i = 0; i + 1 < edges.size(); ++i) {
        EdgePair pair;
        pair.first_edge = edges[i].position;
        pair.second_edge = edges[i + 1].position;
        pair.compute_center();

        // 极性模式：正极性（亮到暗后暗到亮）或负极性
        if ((edges[i].polarity == 1 && edges[i + 1].polarity == -1) ||
            (edges[i].polarity == -1 && edges[i + 1].polarity == 1)) {
            pair.polarity = 1;  // 正负交替
            pair.valid = true;
            pairs.push_back(pair);
            ++i;  // 跳过已配对的边缘
        }
    }

    // 输出结果
    std::ostringstream dist_stream, center_stream;

    dist_stream << "[";
    center_stream << "[";

    for (size_t i = 0; i < pairs.size(); ++i) {
        if (i > 0) {
            dist_stream << ",";
            center_stream << ",";
        }
        dist_stream << pairs[i].distance;
        center_stream << "(" << pairs[i].center_x << "," << pairs[i].center_y << ")";
    }

    dist_stream << "]";
    center_stream << "]";

    set_output("pairs", Data(static_cast<int>(pairs.size())));
    set_output("distances", Data(dist_stream.str()));
    set_output("centers", Data(center_stream.str()));

    OVF_INFO() << "MeasurePairs: found " << pairs.size() << " edge pairs";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// FindLinesNode
//------------------------------------------------------------------------------

FindLinesNode::FindLinesNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FindLinesNode::make_info() {
    NodeInfo info;
    info.id = "FindLines";
    info.name = "找直线";
    info.category = "边缘对测量";
    info.description = "使用RANSAC从边缘点拟合直线";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("lines", "检测到的直线数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "直线信息", DataType::String));

    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("ransac_threshold", "RANSAC拟合阈值", DataType::Number, Data(2.0)));
    info.params.push_back(ParamDef("min_line_length", "最小直线长度", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("max_iterations", "最大迭代次数", DataType::Number, Data(100)));

    return info;
}

Result<void> FindLinesNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float threshold = get_param("threshold", Data(30.0)).as_number();
    float ransac_threshold = get_param("ransac_threshold", Data(2.0)).as_number();
    float min_line_length = get_param("min_line_length", Data(30.0)).as_number();
    int max_iterations = get_param("max_iterations", Data(100)).as_int();

    // 检测边缘点
    std::vector<XLDPoint> edge_points;

    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            SubPixelEdge edge;
            if (xld_utils::subpixel_edge_position(gray, static_cast<float>(x),
                                                  static_cast<float>(y),
                                                  edge.position, edge.amplitude,
                                                  edge.direction, edge.polarity)) {
                if (edge.amplitude > threshold) {
                    edge_points.push_back(edge.position);
                }
            }
        }
    }

    if (edge_points.size() < 2) {
        set_output("lines", Data(0));
        set_output("info", Data("No lines detected"));
        return Result<void>::success();
    }

    // 使用RANSAC检测多条直线
    std::vector<Line2D> lines;
    std::vector<XLDPoint> remaining_points = edge_points;

    std::ostringstream info_stream;
    info_stream << "Lines[\n";
    int line_count = 0;

    while (remaining_points.size() >= 10) {
        Line2D line;
        if (!xld_utils::ransac_fit_line(remaining_points, line, ransac_threshold)) {
            break;
        }

        // 计算直线长度和端点
        std::vector<XLDPoint> inliers;
        std::vector<XLDPoint> outliers;

        for (const auto& pt : remaining_points) {
            float dist = line.distance_to_point(Point2Df(pt.x, pt.y));
            if (dist < ransac_threshold) {
                inliers.push_back(pt);
            } else {
                outliers.push_back(pt);
            }
        }

        if (inliers.size() < 2) break;

        // 计算投影长度
        float min_proj = 1e10f, max_proj = -1e10f;
        float line_dir_x = -line.b;
        float line_dir_y = line.a;

        for (const auto& pt : inliers) {
            float proj = pt.x * line_dir_x + pt.y * line_dir_y;
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }

        float line_length = max_proj - min_proj;

        if (line_length >= min_line_length) {
            lines.push_back(line);
            line_count++;

            // 计算端点
            XLDPoint start_pt, end_pt;
            float cx = 0, cy = 0;
            for (const auto& pt : inliers) {
                cx += pt.x;
                cy += pt.y;
            }
            cx /= inliers.size();
            cy /= inliers.size();

            float angle_deg = std::atan2(-line.a, line.b) * 180.0f / static_cast<float>(M_PI);

            info_stream << "  Line " << line_count << ": angle=" << angle_deg
                       << " deg, length=" << line_length
                       << ", center=(" << cx << "," << cy << ")"
                       << ", inliers=" << inliers.size() << "\n";

            remaining_points = outliers;
        } else {
            break;
        }
    }

    info_stream << "]";

    set_output("lines", Data(line_count));
    set_output("info", Data(info_stream.str()));

    OVF_INFO() << "FindLines: detected " << line_count << " lines";

    return Result<void>::success();
}

//------------------------------------------------------------------------------
// FindCirclesNode
//------------------------------------------------------------------------------

FindCirclesNode::FindCirclesNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FindCirclesNode::make_info() {
    NodeInfo info;
    info.id = "FindCircles";
    info.name = "找圆";
    info.category = "边缘对测量";
    info.description = "使用RANSAC从边缘点拟合圆";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("circles", "检测到的圆数量", DataType::Number));
    info.outputs.push_back(DataPort("info", "圆信息", DataType::String));

    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("ransac_threshold", "RANSAC拟合阈值", DataType::Number, Data(2.0)));
    info.params.push_back(ParamDef("min_radius", "最小半径", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("max_radius", "最大半径", DataType::Number, Data(500.0)));
    info.params.push_back(ParamDef("max_iterations", "最大迭代次数", DataType::Number, Data(100)));

    return info;
}

Result<void> FindCirclesNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray(input);

    float threshold = get_param("threshold", Data(30.0)).as_number();
    float ransac_threshold = get_param("ransac_threshold", Data(2.0)).as_number();
    float min_radius = get_param("min_radius", Data(5.0)).as_number();
    float max_radius = get_param("max_radius", Data(500.0)).as_number();
    int max_iterations = get_param("max_iterations", Data(100)).as_int();

    // 检测边缘点
    std::vector<XLDPoint> edge_points;

    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            SubPixelEdge edge;
            if (xld_utils::subpixel_edge_position(gray, static_cast<float>(x),
                                                  static_cast<float>(y),
                                                  edge.position, edge.amplitude,
                                                  edge.direction, edge.polarity)) {
                if (edge.amplitude > threshold) {
                    edge_points.push_back(edge.position);
                }
            }
        }
    }

    if (edge_points.size() < 3) {
        set_output("circles", Data(0));
        set_output("info", Data("No circles detected"));
        return Result<void>::success();
    }

    // 使用RANSAC检测圆
    std::vector<Circle2D> circles;
    std::vector<XLDPoint> remaining_points = edge_points;

    std::ostringstream info_stream;
    info_stream << "Circles[\n";
    int circle_count = 0;

    while (remaining_points.size() >= 15) {
        Circle2D circle;
        if (!xld_utils::ransac_fit_circle(remaining_points, circle, ransac_threshold)) {
            break;
        }

        // 半径范围检查
        if (circle.radius < min_radius || circle.radius > max_radius) {
            break;
        }

        // 计算内点数
        std::vector<XLDPoint> inliers;
        std::vector<XLDPoint> outliers;

        for (const auto& pt : remaining_points) {
            float dx = pt.x - circle.center_x;
            float dy = pt.y - circle.center_y;
            float dist = std::abs(std::sqrt(dx * dx + dy * dy) - circle.radius);
            if (dist < ransac_threshold) {
                inliers.push_back(pt);
            } else {
                outliers.push_back(pt);
            }
        }

        if (inliers.size() < 10) break;

        circles.push_back(circle);
        circle_count++;

        info_stream << "  Circle " << circle_count << ": center=("
                   << circle.center_x << "," << circle.center_y << ")"
                   << ", radius=" << circle.radius
                   << ", inliers=" << inliers.size() << "\n";

        remaining_points = outliers;
    }

    info_stream << "]";

    set_output("circles", Data(circle_count));
    set_output("info", Data(info_stream.str()));

    OVF_INFO() << "FindCircles: detected " << circle_count << " circles";

    return Result<void>::success();
}

//==============================================================================
// 注册所有节点
//==============================================================================

OVF_REGISTER_NODE(GenContourRegionNode, "GenContourRegion", GenContourRegionNode::make_info())
OVF_REGISTER_NODE(GenContoursSkeletonNode, "GenContoursSkeleton", GenContoursSkeletonNode::make_info())
OVF_REGISTER_NODE(GenCrossContourNode, "GenCrossContour", GenCrossContourNode::make_info())
OVF_REGISTER_NODE(GenPolygonContourNode, "GenPolygonContour", GenPolygonContourNode::make_info())
OVF_REGISTER_NODE(GenArcContourNode, "GenArcContour", GenArcContourNode::make_info())
OVF_REGISTER_NODE(GenEllipseContourNode, "GenEllipseContour", GenEllipseContourNode::make_info())

OVF_REGISTER_NODE(SelectContoursNode, "SelectContours", SelectContoursNode::make_info())
OVF_REGISTER_NODE(SegmentContoursNode, "SegmentContours", SegmentContoursNode::make_info())
OVF_REGISTER_NODE(ApproxChainNode, "ApproxChain", ApproxChainNode::make_info())
OVF_REGISTER_NODE(SmoothContoursNode, "SmoothContours", SmoothContoursNode::make_info())
OVF_REGISTER_NODE(MergeContoursNode, "MergeContours", MergeContoursNode::make_info())
OVF_REGISTER_NODE(ClipContoursNode, "ClipContours", ClipContoursNode::make_info())

OVF_REGISTER_NODE(EdgesSubPixNode, "EdgesSubPix", EdgesSubPixNode::make_info())
OVF_REGISTER_NODE(EdgesSubPixSobelNode, "EdgesSubPixSobel", EdgesSubPixSobelNode::make_info())
OVF_REGISTER_NODE(EdgesSubPixCannyNode, "EdgesSubPixCanny", EdgesSubPixCannyNode::make_info())
OVF_REGISTER_NODE(LinesSubPixNode, "LinesSubPix", LinesSubPixNode::make_info())

OVF_REGISTER_NODE(MeasurePairsNode, "MeasurePairs", MeasurePairsNode::make_info())
OVF_REGISTER_NODE(FindLinesNode, "FindLines", FindLinesNode::make_info())
OVF_REGISTER_NODE(FindCirclesNode, "FindCircles", FindCirclesNode::make_info())

} // namespace algorithm
} // namespace ovf