/**
 * @file position_guiding.cpp
 * @brief 定位引导流程节点实现
 */

// 禁用Windows min/max宏，避免与std::min/std::max冲突
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "ovf/algorithm/position_guiding.h"
#include "ovf/core/logger.h"
#include <sstream>
#include <iomanip>

namespace ovf {
namespace algorithm {

// ============================================================================
// CoordinateTransformNode - 坐标转换节点
// ============================================================================

CoordinateTransformNode::CoordinateTransformNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CoordinateTransformNode::make_info() {
    NodeInfo info;
    info.id = "CoordinateTransform";
    info.name = "坐标转换";
    info.category = "定位引导";
    info.description = "使用标定结果将图像坐标转换为物理坐标";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("image_x", "图像X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("image_y", "图像Y坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("calib_result", "标定结果参数", DataType::Object, true));
    
    // 输出端口
    info.outputs.push_back(DataPort("world_x", "物理X坐标", DataType::Number));
    info.outputs.push_back(DataPort("world_y", "物理Y坐标", DataType::Number));
    info.outputs.push_back(DataPort("point", "物理坐标点", DataType::Point));
    
    // 参数（也可以从输入端口获取标定参数）
    info.params.push_back(ParamDef("a", "仿射参数a", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("b", "仿射参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "仿射参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("d", "仿射参数d", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("e", "仿射参数e", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("f", "仿射参数f", DataType::Number, Data(0.0)));
    
    return info;
}

Result<void> CoordinateTransformNode::execute(FlowContext& context) {
    // 获取输入坐标
    auto x_data = get_input("image_x");
    auto y_data = get_input("image_y");
    
    if (!x_data.is_number() || !y_data.is_number()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Input coordinates must be numbers");
    }
    
    float image_x = static_cast<float>(x_data.as_number());
    float image_y = static_cast<float>(y_data.as_number());
    
    // 获取标定参数（优先从参数获取，如果参数未设置则使用默认值）
    CalibResult calib;
    calib.a = static_cast<float>(get_param("a", Data(0.0)).as_number());
    calib.b = static_cast<float>(get_param("b", Data(0.0)).as_number());
    calib.c = static_cast<float>(get_param("c", Data(0.0)).as_number());
    calib.d = static_cast<float>(get_param("d", Data(0.0)).as_number());
    calib.e = static_cast<float>(get_param("e", Data(0.0)).as_number());
    calib.f = static_cast<float>(get_param("f", Data(0.0)).as_number());
    calib.valid = true;
    
    // 执行坐标转换
    float world_x, world_y;
    calib.transform(image_x, image_y, world_x, world_y);
    
    // 设置输出
    set_output("world_x", Data(static_cast<double>(world_x)));
    set_output("world_y", Data(static_cast<double>(world_y)));
    
    // 输出点位（使用Region来表示点）
    Region point_region;
    point_region.x = static_cast<int32_t>(world_x);
    point_region.y = static_cast<int32_t>(world_y);
    point_region.width = 0;
    point_region.height = 0;
    set_output("point", Data(point_region));
    
    OVF_INFO() << "CoordinateTransform: image(" << image_x << "," << image_y 
               << ") -> world(" << world_x << "," << world_y << ")";
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(CoordinateTransformNode, "CoordinateTransform", CoordinateTransformNode::make_info())

// ============================================================================
// AlignmentCorrectionNode - 对位补正节点
// ============================================================================

AlignmentCorrectionNode::AlignmentCorrectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AlignmentCorrectionNode::make_info() {
    NodeInfo info;
    info.id = "AlignmentCorrection";
    info.name = "对位补正";
    info.category = "定位引导";
    info.description = "计算目标位置与实际位置的偏差，输出补偿值";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("target_x", "目标X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("target_y", "目标Y坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("actual_x", "实际X坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("actual_y", "实际Y坐标", DataType::Number, true));
    info.inputs.push_back(DataPort("target_angle", "目标角度", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("actual_angle", "实际角度", DataType::Number, false, Data(0.0)));
    
    // 输出端口
    info.outputs.push_back(DataPort("offset_x", "平移偏差X", DataType::Number));
    info.outputs.push_back(DataPort("offset_y", "平移偏差Y", DataType::Number));
    info.outputs.push_back(DataPort("rotation", "旋转偏差", DataType::Number));
    info.outputs.push_back(DataPort("scale_x", "缩放偏差X", DataType::Number));
    info.outputs.push_back(DataPort("scale_y", "缩放偏差Y", DataType::Number));
    info.outputs.push_back(DataPort("alignment_result", "对位结果对象", DataType::Object));
    
    return info;
}

Result<void> AlignmentCorrectionNode::execute(FlowContext& context) {
    // 获取输入数据
    auto target_x_data = get_input("target_x");
    auto target_y_data = get_input("target_y");
    auto actual_x_data = get_input("actual_x");
    auto actual_y_data = get_input("actual_y");
    
    if (!target_x_data.is_number() || !target_y_data.is_number() ||
        !actual_x_data.is_number() || !actual_y_data.is_number()) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "Target and actual coordinates must be numbers");
    }
    
    Point2D target;
    target.x = static_cast<float>(target_x_data.as_number());
    target.y = static_cast<float>(target_y_data.as_number());
    
    Point2D actual;
    actual.x = static_cast<float>(actual_x_data.as_number());
    actual.y = static_cast<float>(actual_y_data.as_number());
    
    // 获取角度（可选）
    float target_angle = static_cast<float>(get_input("target_angle").as_number(0.0));
    float actual_angle = static_cast<float>(get_input("actual_angle").as_number(0.0));
    
    // 计算对位偏差
    AlignmentResult result = alignment_utils::calculate_alignment(
        target, actual, target_angle, actual_angle);
    
    // 设置输出
    set_output("offset_x", Data(static_cast<double>(result.offset_x)));
    set_output("offset_y", Data(static_cast<double>(result.offset_y)));
    set_output("rotation", Data(static_cast<double>(result.rotation)));
    set_output("scale_x", Data(static_cast<double>(result.scale_x)));
    set_output("scale_y", Data(static_cast<double>(result.scale_y)));
    
    // 输出结果对象（以字符串形式）
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{";
    oss << "\"offset_x\":" << result.offset_x << ",";
    oss << "\"offset_y\":" << result.offset_y << ",";
    oss << "\"rotation\":" << result.rotation << ",";
    oss << "\"scale_x\":" << result.scale_x << ",";
    oss << "\"scale_y\":" << result.scale_y << ",";
    oss << "\"valid\":" << (result.valid ? "true" : "false");
    oss << "}";
    set_output("alignment_result", Data(oss.str()));
    
    OVF_INFO() << "AlignmentCorrection: offset=(" << result.offset_x << "," << result.offset_y 
               << "), rotation=" << result.rotation << " deg";
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(AlignmentCorrectionNode, "AlignmentCorrection", AlignmentCorrectionNode::make_info())

// ============================================================================
// RobotGuideNode - 机器人引导节点
// ============================================================================

RobotGuideNode::RobotGuideNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo RobotGuideNode::make_info() {
    NodeInfo info;
    info.id = "RobotGuide";
    info.name = "机器人引导";
    info.category = "定位引导";
    info.description = "根据偏差计算并输出机器人运动指令";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("offset_x", "平移偏差X", DataType::Number, true));
    info.inputs.push_back(DataPort("offset_y", "平移偏差Y", DataType::Number, true));
    info.inputs.push_back(DataPort("rotation", "旋转偏差", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("base_x", "基准位置X", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("base_y", "基准位置Y", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("base_z", "基准位置Z", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("base_rx", "基准姿态RX", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("base_ry", "基准姿态RY", DataType::Number, false, Data(0.0)));
    info.inputs.push_back(DataPort("base_rz", "基准姿态RZ", DataType::Number, false, Data(0.0)));
    
    // 输出端口
    info.outputs.push_back(DataPort("target_x", "目标位置X", DataType::Number));
    info.outputs.push_back(DataPort("target_y", "目标位置Y", DataType::Number));
    info.outputs.push_back(DataPort("target_z", "目标位置Z", DataType::Number));
    info.outputs.push_back(DataPort("target_rx", "目标姿态RX", DataType::Number));
    info.outputs.push_back(DataPort("target_ry", "目标姿态RY", DataType::Number));
    info.outputs.push_back(DataPort("target_rz", "目标姿态RZ", DataType::Number));
    info.outputs.push_back(DataPort("motion_type", "运动类型", DataType::Number));
    info.outputs.push_back(DataPort("motion_command", "运动指令对象", DataType::Object));
    
    // 参数
    info.params.push_back(ParamDef("motion_type", "运动类型(0:PTP,1:直线)", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("use_rotation", "是否使用旋转补偿", DataType::Boolean, Data(false)));
    
    return info;
}

Result<void> RobotGuideNode::execute(FlowContext& context) {
    // 获取偏差输入
    auto offset_x_data = get_input("offset_x");
    auto offset_y_data = get_input("offset_y");
    
    if (!offset_x_data.is_number() || !offset_y_data.is_number()) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "Offset values must be numbers");
    }
    
    float offset_x = static_cast<float>(offset_x_data.as_number());
    float offset_y = static_cast<float>(offset_y_data.as_number());
    float rotation = static_cast<float>(get_input("rotation").as_number(0.0));
    
    // 获取基准位置（可选）
    float base_x = static_cast<float>(get_input("base_x").as_number(0.0));
    float base_y = static_cast<float>(get_input("base_y").as_number(0.0));
    float base_z = static_cast<float>(get_input("base_z").as_number(0.0));
    float base_rx = static_cast<float>(get_input("base_rx").as_number(0.0));
    float base_ry = static_cast<float>(get_input("base_ry").as_number(0.0));
    float base_rz = static_cast<float>(get_input("base_rz").as_number(0.0));
    
    // 获取参数
    int motion_type = get_param("motion_type", Data(0)).as_int();
    bool use_rotation = get_param("use_rotation", Data(false)).as_bool();
    
    // 构建运动指令
    MotionCommand cmd;
    cmd.x = base_x + offset_x;
    cmd.y = base_y + offset_y;
    cmd.z = base_z;
    
    if (use_rotation) {
        cmd.rx = base_rx;
        cmd.ry = base_ry;
        cmd.rz = base_rz + rotation;
    } else {
        cmd.rx = base_rx;
        cmd.ry = base_ry;
        cmd.rz = base_rz;
    }
    
    cmd.motion_type = motion_type;
    cmd.valid = true;
    
    // 设置输出
    set_output("target_x", Data(static_cast<double>(cmd.x)));
    set_output("target_y", Data(static_cast<double>(cmd.y)));
    set_output("target_z", Data(static_cast<double>(cmd.z)));
    set_output("target_rx", Data(static_cast<double>(cmd.rx)));
    set_output("target_ry", Data(static_cast<double>(cmd.ry)));
    set_output("target_rz", Data(static_cast<double>(cmd.rz)));
    set_output("motion_type", Data(cmd.motion_type));
    
    // 输出运动指令对象（以字符串形式）
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{";
    oss << "\"x\":" << cmd.x << ",";
    oss << "\"y\":" << cmd.y << ",";
    oss << "\"z\":" << cmd.z << ",";
    oss << "\"rx\":" << cmd.rx << ",";
    oss << "\"ry\":" << cmd.ry << ",";
    oss << "\"rz\":" << cmd.rz << ",";
    oss << "\"motion_type\":" << cmd.motion_type << ",";
    oss << "\"valid\":" << (cmd.valid ? "true" : "false");
    oss << "}";
    set_output("motion_command", Data(oss.str()));
    
    OVF_INFO() << "RobotGuide: target=(" << cmd.x << "," << cmd.y << "," << cmd.z 
               << "), rotation=(" << cmd.rx << "," << cmd.ry << "," << cmd.rz 
               << "), motion_type=" << cmd.motion_type;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(RobotGuideNode, "RobotGuide", RobotGuideNode::make_info())

// ============================================================================
// PositionMatchNode - 位置匹配节点
// ============================================================================

PositionMatchNode::PositionMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PositionMatchNode::make_info() {
    NodeInfo info;
    info.id = "PositionMatch";
    info.name = "位置匹配";
    info.category = "定位引导";
    info.description = "模板匹配并输出匹配位置坐标";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));
    info.inputs.push_back(DataPort("search_region", "搜索区域", DataType::Region, false));
    
    // 输出端口
    info.outputs.push_back(DataPort("match_x", "匹配位置X", DataType::Number));
    info.outputs.push_back(DataPort("match_y", "匹配位置Y", DataType::Number));
    info.outputs.push_back(DataPort("match_score", "匹配置信度", DataType::Number));
    info.outputs.push_back(DataPort("match_region", "匹配区域", DataType::Region));
    info.outputs.push_back(DataPort("match_angle", "匹配角度", DataType::Number));
    
    // 参数
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7)));
    info.params.push_back(ParamDef("max_matches", "最大匹配数", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("find_angle", "是否计算角度", DataType::Boolean, Data(false)));
    
    return info;
}

Result<void> PositionMatchNode::execute(FlowContext& context) {
    // 获取输入图像
    auto image_data = get_input("image");
    auto template_data = get_input("template");
    
    if (!image_data.is_image() || !template_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage,
            "Input and template must be images");
    }
    
    ImageData image = image_data.as_image();
    ImageData template_img = template_data.as_image();
    
    if (image.empty() || template_img.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage,
            "Input image or template is empty");
    }
    
    // 获取参数
    float min_score = static_cast<float>(get_param("min_score", Data(0.7)).as_number());
    bool find_angle = get_param("find_angle", Data(false)).as_bool();
    
    // 获取搜索区域（可选）
    Region search_region;
    auto search_region_data = get_input("search_region");
    if (search_region_data.is_region()) {
        search_region = search_region_data.as_region();
    } else {
        // 默认搜索整个图像
        search_region.x = 0;
        search_region.y = 0;
        search_region.width = static_cast<int32_t>(image.width);
        search_region.height = static_cast<int32_t>(image.height);
    }
    
    // 简化版模板匹配（滑动窗口，计算归一化相关系数）
    // 注意：这是一个简化实现，实际应用中可以使用更高效的算法
    
    uint32_t template_w = template_img.width;
    uint32_t template_h = template_img.height;
    
    float best_score = 0.0f;
    int32_t best_x = 0;
    int32_t best_y = 0;
    float best_angle = 0.0f;
    
    // 只处理单通道图像
    if (image.channels != 1 || template_img.channels != 1) {
        OVF_WARN() << "PositionMatch: Only single-channel images supported in simplified version";
        // 输出默认值
        set_output("match_x", Data(0.0));
        set_output("match_y", Data(0.0));
        set_output("match_score", Data(0.0));
        set_output("match_angle", Data(0.0));
        Region default_region;
        set_output("match_region", Data(default_region));
        return Result<void>::success();
    }
    
    // 在搜索区域内滑动模板
    int32_t start_x = search_region.x;
    int32_t start_y = search_region.y;
    int32_t end_x = search_region.x + search_region.width - template_w;
    int32_t end_y = search_region.y + search_region.height - template_h;
    
    // 确保不超出图像边界
    start_x = std::max(static_cast<int32_t>(0), start_x);
    start_y = std::max(static_cast<int32_t>(0), start_y);
    end_x = (std::min)(static_cast<int32_t>(image.width - template_w), end_x);
    end_y = (std::min)(static_cast<int32_t>(image.height - template_h), end_y);
    
    // 计算模板均值和方差
    double template_mean = 0.0;
    double template_var = 0.0;
    for (size_t i = 0; i < template_img.data.size(); ++i) {
        template_mean += template_img.data[i];
    }
    template_mean /= template_img.data.size();
    
    for (size_t i = 0; i < template_img.data.size(); ++i) {
        double diff = template_img.data[i] - template_mean;
        template_var += diff * diff;
    }
    template_var = std::sqrt(template_var);
    
    if (template_var < 1e-10) {
        // 模板方差太小，无法匹配
        OVF_WARN() << "PositionMatch: Template has very low variance";
        set_output("match_x", Data(0.0));
        set_output("match_y", Data(0.0));
        set_output("match_score", Data(0.0));
        return Result<void>::success();
    }
    
    // 滑动窗口匹配（简化版，步长为1）
    // 为了效率，可以使用更大的步长
    int32_t step = 1;
    for (int32_t y = start_y; y <= end_y; y += step) {
        for (int32_t x = start_x; x <= end_x; x += step) {
            // 计算当前窗口的归一化相关系数
            double window_mean = 0.0;
            double window_var = 0.0;
            double correlation = 0.0;
            
            // 计算窗口均值
            for (uint32_t ty = 0; ty < template_h; ++ty) {
                for (uint32_t tx = 0; tx < template_w; ++tx) {
                    size_t img_idx = (y + ty) * image.width + (x + tx);
                    window_mean += image.data[img_idx];
                }
            }
            window_mean /= (template_w * template_h);
            
            // 计算窗口方差和相关系数
            for (uint32_t ty = 0; ty < template_h; ++ty) {
                for (uint32_t tx = 0; tx < template_w; ++tx) {
                    size_t img_idx = (y + ty) * image.width + (x + tx);
                    size_t tpl_idx = ty * template_w + tx;
                    
                    double img_diff = image.data[img_idx] - window_mean;
                    double tpl_diff = template_img.data[tpl_idx] - template_mean;
                    
                    window_var += img_diff * img_diff;
                    correlation += img_diff * tpl_diff;
                }
            }
            
            window_var = std::sqrt(window_var);
            
            if (window_var < 1e-10) {
                continue;  // 跳过低方差区域
            }
            
            // 计算归一化相关系数 (NCC)
            double ncc = correlation / (template_var * window_var);
            
            if (ncc > best_score) {
                best_score = static_cast<float>(ncc);
                best_x = x;
                best_y = y;
            }
        }
    }
    
    // 检查是否满足最小分数要求
    if (best_score < min_score) {
        OVF_WARN() << "PositionMatch: No match found above threshold " << min_score;
        best_x = 0;
        best_y = 0;
        best_score = 0.0f;
    }
    
    // 输出结果
    set_output("match_x", Data(static_cast<double>(best_x)));
    set_output("match_y", Data(static_cast<double>(best_y)));
    set_output("match_score", Data(static_cast<double>(best_score)));
    
    // 输出匹配区域
    Region match_region;
    match_region.x = best_x;
    match_region.y = best_y;
    match_region.width = static_cast<int32_t>(template_w);
    match_region.height = static_cast<int32_t>(template_h);
    match_region.angle = best_angle;
    set_output("match_region", Data(match_region));
    
    set_output("match_angle", Data(static_cast<double>(best_angle)));
    
    OVF_INFO() << "PositionMatch: position=(" << best_x << "," << best_y 
               << "), score=" << best_score;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(PositionMatchNode, "PositionMatch", PositionMatchNode::make_info())

// ============================================================================
// AffineTransformCalcNode - 仿射变换计算节点
// ============================================================================

AffineTransformCalcNode::AffineTransformCalcNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AffineTransformCalcNode::make_info() {
    NodeInfo info;
    info.id = "AffineTransformCalc";
    info.name = "仿射变换计算";
    info.category = "定位引导";
    info.description = "根据匹配点对计算仿射变换矩阵";
    info.version = "0.1.0";
    
    // 输入端口（支持多个点对的输入）
    info.inputs.push_back(DataPort("source_points", "源点坐标列表", DataType::String, true));
    info.inputs.push_back(DataPort("target_points", "目标点坐标列表", DataType::String, true));
    
    // 也支持单点输入
    info.inputs.push_back(DataPort("src_x1", "源点1 X", DataType::Number, false));
    info.inputs.push_back(DataPort("src_y1", "源点1 Y", DataType::Number, false));
    info.inputs.push_back(DataPort("tgt_x1", "目标点1 X", DataType::Number, false));
    info.inputs.push_back(DataPort("tgt_y1", "目标点1 Y", DataType::Number, false));
    info.inputs.push_back(DataPort("src_x2", "源点2 X", DataType::Number, false));
    info.inputs.push_back(DataPort("src_y2", "源点2 Y", DataType::Number, false));
    info.inputs.push_back(DataPort("tgt_x2", "目标点2 X", DataType::Number, false));
    info.inputs.push_back(DataPort("tgt_y2", "目标点2 Y", DataType::Number, false));
    info.inputs.push_back(DataPort("src_x3", "源点3 X", DataType::Number, false));
    info.inputs.push_back(DataPort("src_y3", "源点3 Y", DataType::Number, false));
    info.inputs.push_back(DataPort("tgt_x3", "目标点3 X", DataType::Number, false));
    info.inputs.push_back(DataPort("tgt_y3", "目标点3 Y", DataType::Number, false));
    
    // 输出端口
    info.outputs.push_back(DataPort("a", "变换参数a", DataType::Number));
    info.outputs.push_back(DataPort("b", "变换参数b", DataType::Number));
    info.outputs.push_back(DataPort("c", "变换参数c(平移X)", DataType::Number));
    info.outputs.push_back(DataPort("d", "变换参数d", DataType::Number));
    info.outputs.push_back(DataPort("e", "变换参数e", DataType::Number));
    info.outputs.push_back(DataPort("f", "变换参数f(平移Y)", DataType::Number));
    info.outputs.push_back(DataPort("translation_x", "平移量X", DataType::Number));
    info.outputs.push_back(DataPort("translation_y", "平移量Y", DataType::Number));
    info.outputs.push_back(DataPort("rotation", "旋转角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("scale_x", "缩放系数X", DataType::Number));
    info.outputs.push_back(DataPort("scale_y", "缩放系数Y", DataType::Number));
    info.outputs.push_back(DataPort("transform_matrix", "变换矩阵对象", DataType::Object));
    
    return info;
}

Result<void> AffineTransformCalcNode::execute(FlowContext& context) {
    std::vector<MatchPointPair> point_pairs;
    
    // 尝试从单点输入获取点对
    auto src_x1 = get_input("src_x1");
    auto src_y1 = get_input("src_y1");
    auto tgt_x1 = get_input("tgt_x1");
    auto tgt_y1 = get_input("tgt_y1");
    
    if (src_x1.is_number() && src_y1.is_number() && tgt_x1.is_number() && tgt_y1.is_number()) {
        MatchPointPair pair1;
        pair1.source_point.x = static_cast<float>(src_x1.as_number());
        pair1.source_point.y = static_cast<float>(src_y1.as_number());
        pair1.target_point.x = static_cast<float>(tgt_x1.as_number());
        pair1.target_point.y = static_cast<float>(tgt_y1.as_number());
        point_pairs.push_back(pair1);
    }
    
    auto src_x2 = get_input("src_x2");
    auto src_y2 = get_input("src_y2");
    auto tgt_x2 = get_input("tgt_x2");
    auto tgt_y2 = get_input("tgt_y2");
    
    if (src_x2.is_number() && src_y2.is_number() && tgt_x2.is_number() && tgt_y2.is_number()) {
        MatchPointPair pair2;
        pair2.source_point.x = static_cast<float>(src_x2.as_number());
        pair2.source_point.y = static_cast<float>(src_y2.as_number());
        pair2.target_point.x = static_cast<float>(tgt_x2.as_number());
        pair2.target_point.y = static_cast<float>(tgt_y2.as_number());
        point_pairs.push_back(pair2);
    }
    
    auto src_x3 = get_input("src_x3");
    auto src_y3 = get_input("src_y3");
    auto tgt_x3 = get_input("tgt_x3");
    auto tgt_y3 = get_input("tgt_y3");
    
    if (src_x3.is_number() && src_y3.is_number() && tgt_x3.is_number() && tgt_y3.is_number()) {
        MatchPointPair pair3;
        pair3.source_point.x = static_cast<float>(src_x3.as_number());
        pair3.source_point.y = static_cast<float>(src_y3.as_number());
        pair3.target_point.x = static_cast<float>(tgt_x3.as_number());
        pair3.target_point.y = static_cast<float>(tgt_y3.as_number());
        point_pairs.push_back(pair3);
    }
    
    // 如果单点输入不足3个，尝试从字符串解析
    if (point_pairs.size() < 3) {
        auto source_points_str = get_input("source_points");
        auto target_points_str = get_input("target_points");
        
        if (source_points_str.is_string() && target_points_str.is_string()) {
            // 解析坐标字符串（格式: "x1,y1;x2,y2;x3,y3" 或 JSON 格式）
            // 简化处理：假设格式为 "x,y;x,y;x,y"
            String src_str = source_points_str.as_string();
            String tgt_str = target_points_str.as_string();
            
            // 解析源点
            std::vector<Point2D> src_points;
            size_t pos = 0;
            while (pos < src_str.size()) {
                size_t comma_pos = src_str.find(',', pos);
                size_t semicolon_pos = src_str.find(';', pos);
                
                if (comma_pos == String::npos) break;
                
                float x = std::stof(src_str.substr(pos, comma_pos - pos));
                size_t end_pos = (semicolon_pos == String::npos) ? src_str.size() : semicolon_pos;
                float y = std::stof(src_str.substr(comma_pos + 1, end_pos - comma_pos - 1));
                
                src_points.push_back(Point2D(x, y));
                
                pos = (semicolon_pos == String::npos) ? src_str.size() : semicolon_pos + 1;
            }
            
            // 解析目标点
            std::vector<Point2D> tgt_points;
            pos = 0;
            while (pos < tgt_str.size()) {
                size_t comma_pos = tgt_str.find(',', pos);
                size_t semicolon_pos = tgt_str.find(';', pos);
                
                if (comma_pos == String::npos) break;
                
                float x = std::stof(tgt_str.substr(pos, comma_pos - pos));
                size_t end_pos = (semicolon_pos == String::npos) ? tgt_str.size() : semicolon_pos;
                float y = std::stof(tgt_str.substr(comma_pos + 1, end_pos - comma_pos - 1));
                
                tgt_points.push_back(Point2D(x, y));
                
                pos = (semicolon_pos == String::npos) ? tgt_str.size() : semicolon_pos + 1;
            }
            
            // 组合点对
            size_t min_size = std::min(src_points.size(), tgt_points.size());
            for (size_t i = 0; i < min_size; ++i) {
                MatchPointPair pair;
                pair.source_point = src_points[i];
                pair.target_point = tgt_points[i];
                point_pairs.push_back(pair);
            }
        }
    }
    
    // 检查是否有足够的点对
    if (point_pairs.size() < 3) {
        OVF_WARN() << "AffineTransformCalc: Need at least 3 point pairs, got " << point_pairs.size();
        // 输出默认值
        set_output("a", Data(1.0));
        set_output("b", Data(0.0));
        set_output("c", Data(0.0));
        set_output("d", Data(0.0));
        set_output("e", Data(1.0));
        set_output("f", Data(0.0));
        set_output("translation_x", Data(0.0));
        set_output("translation_y", Data(0.0));
        set_output("rotation", Data(0.0));
        set_output("scale_x", Data(1.0));
        set_output("scale_y", Data(1.0));
        set_output("transform_matrix", Data("{\"valid\":false}"));
        
        return Result<void>::success();
    }
    
    // 计算仿射变换
    AffineTransform transform = affine_utils::calculate_affine_transform(point_pairs);
    
    // 设置输出
    set_output("a", Data(static_cast<double>(transform.a)));
    set_output("b", Data(static_cast<double>(transform.b)));
    set_output("c", Data(static_cast<double>(transform.c)));
    set_output("d", Data(static_cast<double>(transform.d)));
    set_output("e", Data(static_cast<double>(transform.e)));
    set_output("f", Data(static_cast<double>(transform.f)));
    
    set_output("translation_x", Data(static_cast<double>(transform.get_translation_x())));
    set_output("translation_y", Data(static_cast<double>(transform.get_translation_y())));
    set_output("rotation", Data(static_cast<double>(transform.get_rotation_deg())));
    set_output("scale_x", Data(static_cast<double>(transform.get_scale_x())));
    set_output("scale_y", Data(static_cast<double>(transform.get_scale_y())));
    
    // 输出变换矩阵对象（JSON格式）
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{";
    oss << "\"a\":" << transform.a << ",";
    oss << "\"b\":" << transform.b << ",";
    oss << "\"c\":" << transform.c << ",";
    oss << "\"d\":" << transform.d << ",";
    oss << "\"e\":" << transform.e << ",";
    oss << "\"f\":" << transform.f << ",";
    oss << "\"translation_x\":" << transform.get_translation_x() << ",";
    oss << "\"translation_y\":" << transform.get_translation_y() << ",";
    oss << "\"rotation\":" << transform.get_rotation_deg() << ",";
    oss << "\"scale_x\":" << transform.get_scale_x() << ",";
    oss << "\"scale_y\":" << transform.get_scale_y() << ",";
    oss << "\"valid\":" << (transform.valid ? "true" : "false");
    oss << "}";
    set_output("transform_matrix", Data(oss.str()));
    
    OVF_INFO() << "AffineTransformCalc: translation=(" << transform.get_translation_x() 
               << "," << transform.get_translation_y() << "), rotation=" << transform.get_rotation_deg() 
               << " deg, scale=(" << transform.get_scale_x() << "," << transform.get_scale_y() << ")";
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(AffineTransformCalcNode, "AffineTransformCalc", AffineTransformCalcNode::make_info())

} // namespace algorithm
} // namespace ovf