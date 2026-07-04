/**
 * @file caliper_tool.cpp
 * @brief 卡尺测量工具节点实现（参考VisionPro的CogCaliperTool）
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>

#include "ovf/algorithm/caliper_tool.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

// ============================================================================
// 辅助函数
// ============================================================================

namespace {

// 将图像转换为灰度
ImageData to_gray(const ImageData& input) {
    if (input.empty()) {
        return ImageData();
    }
    
    if (input.channels == 1) {
        return input;
    }
    
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
            uint8_t b = input.data[i * input.channels];
            uint8_t g = input.data[i * input.channels + 1];
            uint8_t r = input.data[i * input.channels + 2];
            gray.data[i] = static_cast<uint8_t>(0.11f * b + 0.59f * g + 0.30f * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i * input.channels];
        }
    }
    
    return gray;
}

// 解析极性参数
EdgePolarity parse_polarity(int polarity_value) {
    if (polarity_value > 0) return EdgePolarity::Positive;
    if (polarity_value < 0) return EdgePolarity::Negative;
    return EdgePolarity::Both;
}

// 构建搜索区域
SearchRegion build_search_region(float origin_x, float origin_y,
                                  float width, float height, float rotation) {
    SearchRegion region;
    region.origin_x = origin_x;
    region.origin_y = origin_y;
    region.width = width;
    region.height = height;
    region.rotation = rotation;
    return region;
}

// 边缘点坐标转换
void convert_edge_position(const EdgeResult& edge,
                           const std::vector<std::pair<float, float>>& positions,
                           float& out_x, float& out_y) {
    if (positions.empty() || !edge.valid) {
        out_x = 0.0f;
        out_y = 0.0f;
        return;
    }
    
    float normalized_pos = edge.x;
    float actual_pos = normalized_pos * static_cast<float>(positions.size() - 1);
    
    int idx_low = static_cast<int>(actual_pos);
    int idx_high = idx_low + 1;
    
    if (idx_low < 0) idx_low = 0;
    if (idx_high >= static_cast<int>(positions.size())) idx_high = static_cast<int>(positions.size()) - 1;
    
    float t = actual_pos - idx_low;
    
    out_x = positions[idx_low].first + t * (positions[idx_high].first - positions[idx_low].first);
    out_y = positions[idx_low].second + t * (positions[idx_high].second - positions[idx_low].second);
}

} // anonymous namespace

// ============================================================================
// CaliperFindEdgeNode - 卡尺找边（单边）
// ============================================================================

CaliperFindEdgeNode::CaliperFindEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperFindEdgeNode::make_info() {
    NodeInfo info;
    info.id = "CaliperFindEdge";
    info.name = "卡尺找边";
    info.category = "卡尺测量";
    info.description = "使用卡尺工具查找单个边缘";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    // 输出端口
    info.outputs.push_back(DataPort("edge_x", "边缘位置X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("edge_y", "边缘位置Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("edge_strength", "边缘强度", DataType::Number));
    info.outputs.push_back(DataPort("edge_polarity", "边缘极性", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到边缘", DataType::Boolean));
    
    // 参数
    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("search_direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("polarity", "边缘极性（1=亮到暗，-1=暗到亮，0=双向）", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> CaliperFindEdgeNode::execute(FlowContext& context) {
    // 获取输入图像
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
    
    // 获取参数
    float roi_x = get_param("roi_x", Data(100.0)).as_number();
    float roi_y = get_param("roi_y", Data(100.0)).as_number();
    float roi_width = get_param("roi_width", Data(100.0)).as_number();
    float roi_height = get_param("roi_height", Data(20.0)).as_number();
    float search_direction = get_param("search_direction", Data(0.0)).as_number();
    float threshold = get_param("threshold", Data(20.0)).as_number();
    int polarity_value = get_param("polarity", Data(0)).as_int();
    float smoothing = get_param("smoothing", Data(1.0)).as_number();
    
    EdgePolarity polarity = parse_polarity(polarity_value);
    
    // 构建搜索区域
    SearchRegion region = build_search_region(roi_x, roi_y, roi_width, roi_height, search_direction);
    
    // 提取轮廓
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, search_direction, profile, positions);
    
    // 查找边缘
    EdgeResult edge = caliper_utils::find_single_edge(profile, threshold, polarity, smoothing);
    
    // 输出结果
    if (edge.valid && !positions.empty()) {
        float edge_x, edge_y;
        convert_edge_position(edge, positions, edge_x, edge_y);
        
        set_output("edge_x", Data(static_cast<double>(edge_x)));
        set_output("edge_y", Data(static_cast<double>(edge_y)));
        set_output("edge_strength", Data(static_cast<double>(edge.strength)));
        set_output("edge_polarity", Data(static_cast<int>(edge.polarity)));
        set_output("found", Data(true));
        
        OVF_INFO() << "CaliperFindEdge: found edge at (" << edge_x << ", " << edge_y 
                   << "), strength=" << edge.strength;
    } else {
        set_output("edge_x", Data(0.0));
        set_output("edge_y", Data(0.0));
        set_output("edge_strength", Data(0.0));
        set_output("edge_polarity", Data(0));
        set_output("found", Data(false));
        
        OVF_WARN() << "CaliperFindEdge: no edge found";
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperFindEdgePairNode - 卡尺找边对
// ============================================================================

CaliperFindEdgePairNode::CaliperFindEdgePairNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperFindEdgePairNode::make_info() {
    NodeInfo info;
    info.id = "CaliperFindEdgePair";
    info.name = "卡尺找边对";
    info.category = "卡尺测量";
    info.description = "使用卡尺工具查找边缘对（用于宽度测量）";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    // 输出端口
    info.outputs.push_back(DataPort("edge1_x", "边缘1位置X", DataType::Number));
    info.outputs.push_back(DataPort("edge1_y", "边缘1位置Y", DataType::Number));
    info.outputs.push_back(DataPort("edge2_x", "边缘2位置X", DataType::Number));
    info.outputs.push_back(DataPort("edge2_y", "边缘2位置Y", DataType::Number));
    info.outputs.push_back(DataPort("distance", "边缘间距离", DataType::Number));
    info.outputs.push_back(DataPort("midpoint_x", "中点X", DataType::Number));
    info.outputs.push_back(DataPort("midpoint_y", "中点Y", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到边缘对", DataType::Boolean));
    
    // 参数
    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("search_direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("polarity", "边缘极性", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> CaliperFindEdgePairNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float roi_x = get_param("roi_x", Data(100.0)).as_number();
    float roi_y = get_param("roi_y", Data(100.0)).as_number();
    float roi_width = get_param("roi_width", Data(100.0)).as_number();
    float roi_height = get_param("roi_height", Data(20.0)).as_number();
    float search_direction = get_param("search_direction", Data(0.0)).as_number();
    float threshold = get_param("threshold", Data(20.0)).as_number();
    int polarity_value = get_param("polarity", Data(0)).as_int();
    float smoothing = get_param("smoothing", Data(1.0)).as_number();
    
    EdgePolarity polarity = parse_polarity(polarity_value);
    SearchRegion region = build_search_region(roi_x, roi_y, roi_width, roi_height, search_direction);
    
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, search_direction, profile, positions);
    
    EdgePairResult edge_pair = caliper_utils::find_edge_pair(profile, threshold, polarity, smoothing);
    
    if (edge_pair.valid && !positions.empty()) {
        float edge1_x, edge1_y, edge2_x, edge2_y;
        convert_edge_position(edge_pair.edge1, positions, edge1_x, edge1_y);
        convert_edge_position(edge_pair.edge2, positions, edge2_x, edge2_y);
        
        // 计算实际距离
        float dx = edge2_x - edge1_x;
        float dy = edge2_y - edge1_y;
        float actual_distance = std::sqrt(dx * dx + dy * dy);
        
        float midpoint_x = (edge1_x + edge2_x) / 2.0f;
        float midpoint_y = (edge1_y + edge2_y) / 2.0f;
        
        set_output("edge1_x", Data(static_cast<double>(edge1_x)));
        set_output("edge1_y", Data(static_cast<double>(edge1_y)));
        set_output("edge2_x", Data(static_cast<double>(edge2_x)));
        set_output("edge2_y", Data(static_cast<double>(edge2_y)));
        set_output("distance", Data(static_cast<double>(actual_distance)));
        set_output("midpoint_x", Data(static_cast<double>(midpoint_x)));
        set_output("midpoint_y", Data(static_cast<double>(midpoint_y)));
        set_output("found", Data(true));
        
        OVF_INFO() << "CaliperFindEdgePair: found edge pair, distance=" << actual_distance;
    } else {
        set_output("edge1_x", Data(0.0));
        set_output("edge1_y", Data(0.0));
        set_output("edge2_x", Data(0.0));
        set_output("edge2_y", Data(0.0));
        set_output("distance", Data(0.0));
        set_output("midpoint_x", Data(0.0));
        set_output("midpoint_y", Data(0.0));
        set_output("found", Data(false));
        
        OVF_WARN() << "CaliperFindEdgePair: no edge pair found";
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperMeasureWidthNode - 卡尺测量宽度
// ============================================================================

CaliperMeasureWidthNode::CaliperMeasureWidthNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperMeasureWidthNode::make_info() {
    NodeInfo info;
    info.id = "CaliperMeasureWidth";
    info.name = "卡尺测量宽度";
    info.category = "卡尺测量";
    info.description = "使用卡尺测量物体宽度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("width", "测量宽度", DataType::Number));
    info.outputs.push_back(DataPort("center_x", "中心位置X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心位置Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("search_direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(15.0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.5)));
    
    return info;
}

Result<void> CaliperMeasureWidthNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float roi_x = get_param("roi_x", Data(100.0)).as_number();
    float roi_y = get_param("roi_y", Data(100.0)).as_number();
    float roi_width = get_param("roi_width", Data(100.0)).as_number();
    float roi_height = get_param("roi_height", Data(30.0)).as_number();
    float search_direction = get_param("search_direction", Data(0.0)).as_number();
    float threshold = get_param("threshold", Data(15.0)).as_number();
    float smoothing = get_param("smoothing", Data(1.5)).as_number();
    
    SearchRegion region = build_search_region(roi_x, roi_y, roi_width, roi_height, search_direction);
    
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, search_direction, profile, positions);
    
    // 查找边缘对（双向极性，寻找边缘对）
    EdgePairResult edge_pair = caliper_utils::find_edge_pair(profile, threshold, EdgePolarity::Both, smoothing);
    
    if (edge_pair.valid && !positions.empty()) {
        float edge1_x, edge1_y, edge2_x, edge2_y;
        convert_edge_position(edge_pair.edge1, positions, edge1_x, edge1_y);
        convert_edge_position(edge_pair.edge2, positions, edge2_x, edge2_y);
        
        float dx = edge2_x - edge1_x;
        float dy = edge2_y - edge1_y;
        float width = std::sqrt(dx * dx + dy * dy);
        
        float center_x = (edge1_x + edge2_x) / 2.0f;
        float center_y = (edge1_y + edge2_y) / 2.0f;
        
        set_output("width", Data(static_cast<double>(width)));
        set_output("center_x", Data(static_cast<double>(center_x)));
        set_output("center_y", Data(static_cast<double>(center_y)));
        set_output("valid", Data(true));
        
        OVF_INFO() << "CaliperMeasureWidth: width=" << width << " pixels";
    } else {
        set_output("width", Data(0.0));
        set_output("center_x", Data(0.0));
        set_output("center_y", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperMeasureWidth: measurement failed";
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperMeasureDistanceNode - 卡尺测量距离
// ============================================================================

CaliperMeasureDistanceNode::CaliperMeasureDistanceNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperMeasureDistanceNode::make_info() {
    NodeInfo info;
    info.id = "CaliperMeasureDistance";
    info.name = "卡尺测量距离";
    info.category = "卡尺测量";
    info.description = "测量两个边缘之间的距离";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("distance", "测量距离", DataType::Number));
    info.outputs.push_back(DataPort("edge1_x", "边缘1位置X", DataType::Number));
    info.outputs.push_back(DataPort("edge1_y", "边缘1位置Y", DataType::Number));
    info.outputs.push_back(DataPort("edge2_x", "边缘2位置X", DataType::Number));
    info.outputs.push_back(DataPort("edge2_y", "边缘2位置Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("mode", "测量模式（1=点对点，2=点到线）", DataType::Number, Data(1)));
    
    // ROI参数
    info.params.push_back(ParamDef("roi1_x", "ROI1中心X", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("roi1_y", "ROI1中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi1_width", "ROI1宽度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("roi1_height", "ROI1高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("roi1_direction", "ROI1搜索方向", DataType::Number, Data(0.0)));
    
    info.params.push_back(ParamDef("roi2_x", "ROI2中心X", DataType::Number, Data(150.0)));
    info.params.push_back(ParamDef("roi2_y", "ROI2中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi2_width", "ROI2宽度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("roi2_height", "ROI2高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("roi2_direction", "ROI2搜索方向", DataType::Number, Data(0.0)));
    
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> CaliperMeasureDistanceNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    int mode = get_param("mode", Data(1)).as_int();
    
    float roi1_x = get_param("roi1_x", Data(50.0)).as_number();
    float roi1_y = get_param("roi1_y", Data(100.0)).as_number();
    float roi1_width = get_param("roi1_width", Data(50.0)).as_number();
    float roi1_height = get_param("roi1_height", Data(20.0)).as_number();
    float roi1_direction = get_param("roi1_direction", Data(0.0)).as_number();
    
    float roi2_x = get_param("roi2_x", Data(150.0)).as_number();
    float roi2_y = get_param("roi2_y", Data(100.0)).as_number();
    float roi2_width = get_param("roi2_width", Data(50.0)).as_number();
    float roi2_height = get_param("roi2_height", Data(20.0)).as_number();
    float roi2_direction = get_param("roi2_direction", Data(0.0)).as_number();
    
    float threshold = get_param("threshold", Data(20.0)).as_number();
    float smoothing = get_param("smoothing", Data(1.0)).as_number();
    
    SearchRegion region1 = build_search_region(roi1_x, roi1_y, roi1_width, roi1_height, roi1_direction);
    SearchRegion region2 = build_search_region(roi2_x, roi2_y, roi2_width, roi2_height, roi2_direction);
    
    std::vector<float> profile1, profile2;
    std::vector<std::pair<float, float>> positions1, positions2;
    
    caliper_utils::extract_profile(gray, region1, roi1_direction, profile1, positions1);
    caliper_utils::extract_profile(gray, region2, roi2_direction, profile2, positions2);
    
    EdgeResult edge1 = caliper_utils::find_single_edge(profile1, threshold, EdgePolarity::Both, smoothing);
    EdgeResult edge2 = caliper_utils::find_single_edge(profile2, threshold, EdgePolarity::Both, smoothing);
    
    if (!edge1.valid || !edge2.valid || positions1.empty() || positions2.empty()) {
        set_output("distance", Data(0.0));
        set_output("edge1_x", Data(0.0));
        set_output("edge1_y", Data(0.0));
        set_output("edge2_x", Data(0.0));
        set_output("edge2_y", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperMeasureDistance: failed to find edges";
        return Result<void>::success();
    }
    
    float e1_x, e1_y, e2_x, e2_y;
    convert_edge_position(edge1, positions1, e1_x, e1_y);
    convert_edge_position(edge2, positions2, e2_x, e2_y);
    
    float dx = e2_x - e1_x;
    float dy = e2_y - e1_y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    set_output("distance", Data(static_cast<double>(distance)));
    set_output("edge1_x", Data(static_cast<double>(e1_x)));
    set_output("edge1_y", Data(static_cast<double>(e1_y)));
    set_output("edge2_x", Data(static_cast<double>(e2_x)));
    set_output("edge2_y", Data(static_cast<double>(e2_y)));
    set_output("valid", Data(true));
    
    OVF_INFO() << "CaliperMeasureDistance: distance=" << distance << " pixels";
    
    return Result<void>::success();
}

// ============================================================================
// CaliperFindCircleEdgeNode - 卡尺找圆边缘
// ============================================================================

CaliperFindCircleEdgeNode::CaliperFindCircleEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperFindCircleEdgeNode::make_info() {
    NodeInfo info;
    info.id = "CaliperFindCircleEdge";
    info.name = "卡尺找圆边缘";
    info.category = "卡尺测量";
    info.description = "沿圆弧方向查找边缘点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("edge_count", "边缘点数量", DataType::Number));
    info.outputs.push_back(DataPort("edge_points", "边缘点列表（JSON）", DataType::String));
    info.outputs.push_back(DataPort("fit_center_x", "拟合圆心X", DataType::Number));
    info.outputs.push_back(DataPort("fit_center_y", "拟合圆心Y", DataType::Number));
    info.outputs.push_back(DataPort("fit_radius", "拟合半径", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("center_x", "估计圆心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("center_y", "估计圆心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("radius", "估计半径", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("angle_start", "起始角度（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("angle_end", "结束角度（度）", DataType::Number, Data(360.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(15.0)));
    info.params.push_back(ParamDef("polarity", "边缘极性", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CaliperFindCircleEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float center_x = get_param("center_x", Data(100.0)).as_number();
    float center_y = get_param("center_y", Data(100.0)).as_number();
    float radius = get_param("radius", Data(50.0)).as_number();
    float angle_start = get_param("angle_start", Data(0.0)).as_number();
    float angle_end = get_param("angle_end", Data(360.0)).as_number();
    float threshold = get_param("threshold", Data(15.0)).as_number();
    int polarity_value = get_param("polarity", Data(0)).as_int();
    
    EdgePolarity polarity = parse_polarity(polarity_value);
    
    auto edges = caliper_utils::find_circle_edges(gray, center_x, center_y, radius,
                                                   angle_start, angle_end, threshold, polarity);
    
    if (edges.empty()) {
        set_output("edge_count", Data(0));
        set_output("edge_points", Data(""));
        set_output("fit_center_x", Data(0.0));
        set_output("fit_center_y", Data(0.0));
        set_output("fit_radius", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperFindCircleEdge: no edges found";
        return Result<void>::success();
    }
    
    // 构建JSON格式的边缘点列表
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < edges.size(); ++i) {
        oss << "{\"x\":" << edges[i].x << ",\"y\":" << edges[i].y 
            << ",\"strength\":" << edges[i].strength << "}";
        if (i < edges.size() - 1) oss << ",";
    }
    oss << "]";
    
    set_output("edge_count", Data(static_cast<int>(edges.size())));
    set_output("edge_points", Data(oss.str()));
    
    // 如果有足够的边缘点，拟合圆
    if (edges.size() >= 3) {
        std::vector<Point2Df> points;
        for (const auto& edge : edges) {
            points.emplace_back(edge.x, edge.y);
        }
        
        Circle2D circle = measurement_utils::fit_circle(points);
        
        if (circle.valid) {
            set_output("fit_center_x", Data(static_cast<double>(circle.center_x)));
            set_output("fit_center_y", Data(static_cast<double>(circle.center_y)));
            set_output("fit_radius", Data(static_cast<double>(circle.radius)));
            set_output("valid", Data(true));
            
            OVF_INFO() << "CaliperFindCircleEdge: found " << edges.size() 
                       << " edges, fit circle at (" << circle.center_x << "," << circle.center_y 
                       << ") radius=" << circle.radius;
        } else {
            set_output("fit_center_x", Data(0.0));
            set_output("fit_center_y", Data(0.0));
            set_output("fit_radius", Data(0.0));
            set_output("valid", Data(false));
        }
    } else {
        set_output("fit_center_x", Data(0.0));
        set_output("fit_center_y", Data(0.0));
        set_output("fit_radius", Data(0.0));
        set_output("valid", Data(false));
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperFindCornerNode - 卡尺找角点
// ============================================================================

CaliperFindCornerNode::CaliperFindCornerNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperFindCornerNode::make_info() {
    NodeInfo info;
    info.id = "CaliperFindCorner";
    info.name = "卡尺找角点";
    info.category = "卡尺测量";
    info.description = "查找两条边缘的交点（角点）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("corner_x", "角点位置X", DataType::Number));
    info.outputs.push_back(DataPort("corner_y", "角点位置Y", DataType::Number));
    info.outputs.push_back(DataPort("corner_angle", "角点角度（度）", DataType::Number));
    info.outputs.push_back(DataPort("corner_strength", "角点强度", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    // 第一条边ROI
    info.params.push_back(ParamDef("roi1_x", "ROI1中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi1_y", "ROI1中心Y", DataType::Number, Data(80.0)));
    info.params.push_back(ParamDef("roi1_width", "ROI1宽度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("roi1_height", "ROI1高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("roi1_direction", "ROI1搜索方向（度）", DataType::Number, Data(90.0)));
    
    // 第二条边ROI
    info.params.push_back(ParamDef("roi2_x", "ROI2中心X", DataType::Number, Data(80.0)));
    info.params.push_back(ParamDef("roi2_y", "ROI2中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi2_width", "ROI2宽度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("roi2_height", "ROI2高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("roi2_direction", "ROI2搜索方向（度）", DataType::Number, Data(180.0)));
    
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    
    return info;
}

Result<void> CaliperFindCornerNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float roi1_x = get_param("roi1_x", Data(100.0)).as_number();
    float roi1_y = get_param("roi1_y", Data(80.0)).as_number();
    float roi1_width = get_param("roi1_width", Data(50.0)).as_number();
    float roi1_height = get_param("roi1_height", Data(20.0)).as_number();
    float roi1_direction = get_param("roi1_direction", Data(90.0)).as_number();
    
    float roi2_x = get_param("roi2_x", Data(80.0)).as_number();
    float roi2_y = get_param("roi2_y", Data(100.0)).as_number();
    float roi2_width = get_param("roi2_width", Data(50.0)).as_number();
    float roi2_height = get_param("roi2_height", Data(20.0)).as_number();
    float roi2_direction = get_param("roi2_direction", Data(180.0)).as_number();
    
    float threshold = get_param("threshold", Data(20.0)).as_number();
    
    SearchRegion region1 = build_search_region(roi1_x, roi1_y, roi1_width, roi1_height, roi1_direction);
    SearchRegion region2 = build_search_region(roi2_x, roi2_y, roi2_width, roi2_height, roi2_direction);
    
    CornerResult corner = caliper_utils::find_corner(gray, region1, region2, threshold);
    
    if (corner.valid) {
        set_output("corner_x", Data(static_cast<double>(corner.x)));
        set_output("corner_y", Data(static_cast<double>(corner.y)));
        set_output("corner_angle", Data(static_cast<double>(corner.angle)));
        set_output("corner_strength", Data(static_cast<double>(corner.strength)));
        set_output("valid", Data(true));
        
        OVF_INFO() << "CaliperFindCorner: corner at (" << corner.x << "," << corner.y 
                   << "), angle=" << corner.angle << " deg";
    } else {
        set_output("corner_x", Data(0.0));
        set_output("corner_y", Data(0.0));
        set_output("corner_angle", Data(0.0));
        set_output("corner_strength", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperFindCorner: no corner found";
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperSubpixelEdgeNode - 卡尺亚像素边缘
// ============================================================================

CaliperSubpixelEdgeNode::CaliperSubpixelEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperSubpixelEdgeNode::make_info() {
    NodeInfo info;
    info.id = "CaliperSubpixelEdge";
    info.name = "卡尺亚像素边缘";
    info.category = "卡尺测量";
    info.description = "使用亚像素精度定位边缘";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("edge_x", "边缘位置X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("edge_y", "边缘位置Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("subpixel_accuracy", "亚像素精度", DataType::Number));
    info.outputs.push_back(DataPort("edge_strength", "边缘强度", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("search_direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("method", "定位方法（1=二次拟合，2=泰勒展开）", DataType::Number, Data(1)));
    
    return info;
}

Result<void> CaliperSubpixelEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float roi_x = get_param("roi_x", Data(100.0)).as_number();
    float roi_y = get_param("roi_y", Data(100.0)).as_number();
    float roi_width = get_param("roi_width", Data(100.0)).as_number();
    float roi_height = get_param("roi_height", Data(10.0)).as_number();
    float search_direction = get_param("search_direction", Data(0.0)).as_number();
    float threshold = get_param("threshold", Data(10.0)).as_number();
    float smoothing = get_param("smoothing", Data(0.5)).as_number();
    
    SearchRegion region = build_search_region(roi_x, roi_y, roi_width, roi_height, search_direction);
    
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, search_direction, profile, positions);
    
    // 使用更精细的亚像素定位
    EdgeResult edge = caliper_utils::find_single_edge(profile, threshold, EdgePolarity::Both, smoothing);
    
    if (edge.valid && !positions.empty()) {
        float edge_x, edge_y;
        convert_edge_position(edge, positions, edge_x, edge_y);
        
        // 亚像素精度通常可达到0.1像素级别
        float accuracy = 0.1f;
        
        set_output("edge_x", Data(static_cast<double>(edge_x)));
        set_output("edge_y", Data(static_cast<double>(edge_y)));
        set_output("subpixel_accuracy", Data(static_cast<double>(accuracy)));
        set_output("edge_strength", Data(static_cast<double>(edge.strength)));
        set_output("valid", Data(true));
        
        OVF_INFO() << "CaliperSubpixelEdge: edge at (" << edge_x << "," << edge_y 
                   << ") with subpixel accuracy ~" << accuracy << " pixels";
    } else {
        set_output("edge_x", Data(0.0));
        set_output("edge_y", Data(0.0));
        set_output("subpixel_accuracy", Data(0.0));
        set_output("edge_strength", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperSubpixelEdge: no edge found";
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperMultiEdgeNode - 多边缘卡尺
// ============================================================================

CaliperMultiEdgeNode::CaliperMultiEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperMultiEdgeNode::make_info() {
    NodeInfo info;
    info.id = "CaliperMultiEdge";
    info.name = "多边缘卡尺";
    info.category = "卡尺测量";
    info.description = "查找多个边缘并输出列表";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("edge_count", "边缘数量", DataType::Number));
    info.outputs.push_back(DataPort("edges", "边缘列表（JSON）", DataType::String));
    info.outputs.push_back(DataPort("first_edge_x", "第一个边缘X", DataType::Number));
    info.outputs.push_back(DataPort("first_edge_y", "第一个边缘Y", DataType::Number));
    info.outputs.push_back(DataPort("last_edge_x", "最后一个边缘X", DataType::Number));
    info.outputs.push_back(DataPort("last_edge_y", "最后一个边缘Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(200.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("search_direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(15.0)));
    info.params.push_back(ParamDef("polarity", "边缘极性", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("max_edges", "最大边缘数量", DataType::Number, Data(10)));
    
    return info;
}

Result<void> CaliperMultiEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float roi_x = get_param("roi_x", Data(100.0)).as_number();
    float roi_y = get_param("roi_y", Data(100.0)).as_number();
    float roi_width = get_param("roi_width", Data(200.0)).as_number();
    float roi_height = get_param("roi_height", Data(20.0)).as_number();
    float search_direction = get_param("search_direction", Data(0.0)).as_number();
    float threshold = get_param("threshold", Data(15.0)).as_number();
    int polarity_value = get_param("polarity", Data(0)).as_int();
    float smoothing = get_param("smoothing", Data(1.0)).as_number();
    int max_edges = get_param("max_edges", Data(10)).as_int();
    
    EdgePolarity polarity = parse_polarity(polarity_value);
    SearchRegion region = build_search_region(roi_x, roi_y, roi_width, roi_height, search_direction);
    
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, search_direction, profile, positions);
    
    auto edges = caliper_utils::find_all_edges(profile, threshold, polarity, smoothing);
    
    // 限制边缘数量
    if (static_cast<int>(edges.size()) > max_edges) {
        edges.resize(max_edges);
    }
    
    if (edges.empty() || positions.empty()) {
        set_output("edge_count", Data(0));
        set_output("edges", Data(""));
        set_output("first_edge_x", Data(0.0));
        set_output("first_edge_y", Data(0.0));
        set_output("last_edge_x", Data(0.0));
        set_output("last_edge_y", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperMultiEdge: no edges found";
        return Result<void>::success();
    }
    
    // 构建JSON格式输出
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < edges.size(); ++i) {
        float e_x, e_y;
        convert_edge_position(edges[i], positions, e_x, e_y);
        
        oss << "{\"x\":" << e_x << ",\"y\":" << e_y 
            << ",\"strength\":" << edges[i].strength
            << ",\"polarity\":" << static_cast<int>(edges[i].polarity) << "}";
        if (i < edges.size() - 1) oss << ",";
    }
    oss << "]";
    
    float first_x, first_y, last_x, last_y;
    convert_edge_position(edges[0], positions, first_x, first_y);
    convert_edge_position(edges.back(), positions, last_x, last_y);
    
    set_output("edge_count", Data(static_cast<int>(edges.size())));
    set_output("edges", Data(oss.str()));
    set_output("first_edge_x", Data(static_cast<double>(first_x)));
    set_output("first_edge_y", Data(static_cast<double>(first_y)));
    set_output("last_edge_x", Data(static_cast<double>(last_x)));
    set_output("last_edge_y", Data(static_cast<double>(last_y)));
    set_output("valid", Data(true));
    
    OVF_INFO() << "CaliperMultiEdge: found " << edges.size() << " edges";
    
    return Result<void>::success();
}

// ============================================================================
// CaliperProjectEdgeNode - 投影边缘卡尺
// ============================================================================

CaliperProjectEdgeNode::CaliperProjectEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperProjectEdgeNode::make_info() {
    NodeInfo info;
    info.id = "CaliperProjectEdge";
    info.name = "投影边缘卡尺";
    info.category = "卡尺测量";
    info.description = "沿指定方向投影并查找边缘";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("project_length", "投影长度", DataType::Number));
    info.outputs.push_back(DataPort("edge_x", "边缘位置X", DataType::Number));
    info.outputs.push_back(DataPort("edge_y", "边缘位置Y", DataType::Number));
    info.outputs.push_back(DataPort("projection_value", "投影值", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("start_x", "起始点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("start_y", "起始点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_x", "结束点X", DataType::Number, Data(200.0)));
    info.params.push_back(ParamDef("end_y", "结束点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("projection_width", "投影宽度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(15.0)));
    info.params.push_back(ParamDef("polarity", "边缘极性", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> CaliperProjectEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float start_x = get_param("start_x", Data(0.0)).as_number();
    float start_y = get_param("start_y", Data(0.0)).as_number();
    float end_x = get_param("end_x", Data(200.0)).as_number();
    float end_y = get_param("end_y", Data(0.0)).as_number();
    float projection_width = get_param("projection_width", Data(20.0)).as_number();
    float threshold = get_param("threshold", Data(15.0)).as_number();
    int polarity_value = get_param("polarity", Data(0)).as_int();
    float smoothing = get_param("smoothing", Data(1.0)).as_number();
    
    // 计算投影方向和长度
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float project_length = std::sqrt(dx * dx + dy * dy);
    float direction = std::atan2(dy, dx) * 180.0f / static_cast<float>(M_PI);
    
    // 构建搜索区域
    float center_x = (start_x + end_x) / 2.0f;
    float center_y = (start_y + end_y) / 2.0f;
    
    SearchRegion region = build_search_region(center_x, center_y, project_length, projection_width, direction);
    
    EdgePolarity polarity = parse_polarity(polarity_value);
    
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, direction, profile, positions);
    
    EdgeResult edge = caliper_utils::find_single_edge(profile, threshold, polarity, smoothing);
    
    if (edge.valid && !positions.empty()) {
        float edge_x, edge_y;
        convert_edge_position(edge, positions, edge_x, edge_y);
        
        // 计算投影值（边缘在投影方向上的位置）
        float proj_val = (edge_x - start_x) * dx / project_length + (edge_y - start_y) * dy / project_length;
        
        set_output("project_length", Data(static_cast<double>(project_length)));
        set_output("edge_x", Data(static_cast<double>(edge_x)));
        set_output("edge_y", Data(static_cast<double>(edge_y)));
        set_output("projection_value", Data(static_cast<double>(proj_val)));
        set_output("valid", Data(true));
        
        OVF_INFO() << "CaliperProjectEdge: edge at projection position " << proj_val;
    } else {
        set_output("project_length", Data(static_cast<double>(project_length)));
        set_output("edge_x", Data(0.0));
        set_output("edge_y", Data(0.0));
        set_output("projection_value", Data(0.0));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperProjectEdge: no edge found";
    }
    
    return Result<void>::success();
}

// ============================================================================
// CaliperFitLineNode - 卡尺拟合直线
// ============================================================================

CaliperFitLineNode::CaliperFitLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperFitLineNode::make_info() {
    NodeInfo info;
    info.id = "CaliperFitLine";
    info.name = "卡尺拟合直线";
    info.category = "卡尺测量";
    info.description = "从多个边缘点拟合直线";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("line_a", "直线参数a", DataType::Number));
    info.outputs.push_back(DataPort("line_b", "直线参数b", DataType::Number));
    info.outputs.push_back(DataPort("line_c", "直线参数c", DataType::Number));
    info.outputs.push_back(DataPort("line_angle", "直线角度（度）", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "边缘点数量", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(200.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("search_direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(15.0)));
    info.params.push_back(ParamDef("smoothing", "平滑滤波强度", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("min_points", "最小点数量", DataType::Number, Data(2)));
    
    return info;
}

Result<void> CaliperFitLineNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray = to_gray(input);
    
    float roi_x = get_param("roi_x", Data(100.0)).as_number();
    float roi_y = get_param("roi_y", Data(100.0)).as_number();
    float roi_width = get_param("roi_width", Data(200.0)).as_number();
    float roi_height = get_param("roi_height", Data(30.0)).as_number();
    float search_direction = get_param("search_direction", Data(0.0)).as_number();
    float threshold = get_param("threshold", Data(15.0)).as_number();
    float smoothing = get_param("smoothing", Data(1.0)).as_number();
    int min_points = get_param("min_points", Data(2)).as_int();
    
    SearchRegion region = build_search_region(roi_x, roi_y, roi_width, roi_height, search_direction);
    
    std::vector<float> profile;
    std::vector<std::pair<float, float>> positions;
    caliper_utils::extract_profile(gray, region, search_direction, profile, positions);
    
    auto edges = caliper_utils::find_all_edges(profile, threshold, EdgePolarity::Both, smoothing);
    
    if (static_cast<int>(edges.size()) < min_points || positions.empty()) {
        set_output("line_a", Data(0.0));
        set_output("line_b", Data(1.0));
        set_output("line_c", Data(0.0));
        set_output("line_angle", Data(0.0));
        set_output("point_count", Data(static_cast<int>(edges.size())));
        set_output("valid", Data(false));
        
        OVF_WARN() << "CaliperFitLine: not enough edge points (" << edges.size() << ")";
        return Result<void>::success();
    }
    
    // 转换边缘点为实际坐标
    std::vector<Point2Df> points;
    for (const auto& edge : edges) {
        float e_x, e_y;
        convert_edge_position(edge, positions, e_x, e_y);
        points.emplace_back(e_x, e_y);
    }
    
    // 拟合直线
    Line2D line = measurement_utils::fit_line(points);
    
    // 计算直线角度
    float line_angle = std::atan2(-line.a, line.b) * 180.0f / static_cast<float>(M_PI);
    if (line_angle < 0) line_angle += 180.0f;
    
    set_output("line_a", Data(static_cast<double>(line.a)));
    set_output("line_b", Data(static_cast<double>(line.b)));
    set_output("line_c", Data(static_cast<double>(line.c)));
    set_output("line_angle", Data(static_cast<double>(line_angle)));
    set_output("point_count", Data(static_cast<int>(points.size())));
    set_output("valid", Data(true));
    
    OVF_INFO() << "CaliperFitLine: fitted line with " << points.size() 
               << " points, angle=" << line_angle << " deg";
    
    return Result<void>::success();
}

// ============================================================================
// 注册所有节点
// ============================================================================

OVF_REGISTER_NODE(CaliperFindEdgeNode, "CaliperFindEdge", CaliperFindEdgeNode::make_info())
OVF_REGISTER_NODE(CaliperFindEdgePairNode, "CaliperFindEdgePair", CaliperFindEdgePairNode::make_info())
OVF_REGISTER_NODE(CaliperMeasureWidthNode, "CaliperMeasureWidth", CaliperMeasureWidthNode::make_info())
OVF_REGISTER_NODE(CaliperMeasureDistanceNode, "CaliperMeasureDistance", CaliperMeasureDistanceNode::make_info())
OVF_REGISTER_NODE(CaliperFindCircleEdgeNode, "CaliperFindCircleEdge", CaliperFindCircleEdgeNode::make_info())
OVF_REGISTER_NODE(CaliperFindCornerNode, "CaliperFindCorner", CaliperFindCornerNode::make_info())
OVF_REGISTER_NODE(CaliperSubpixelEdgeNode, "CaliperSubpixelEdge", CaliperSubpixelEdgeNode::make_info())
OVF_REGISTER_NODE(CaliperMultiEdgeNode, "CaliperMultiEdge", CaliperMultiEdgeNode::make_info())
OVF_REGISTER_NODE(CaliperProjectEdgeNode, "CaliperProjectEdge", CaliperProjectEdgeNode::make_info())
OVF_REGISTER_NODE(CaliperFitLineNode, "CaliperFitLine", CaliperFitLineNode::make_info())

} // namespace algorithm
} // namespace ovf