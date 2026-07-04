/**
 * @file measurement.cpp
 * @brief 测量节点实现
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "ovf/algorithm/measurement.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

// ==================== CaliperToolNode ====================

CaliperToolNode::CaliperToolNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CaliperToolNode::make_info() {
    NodeInfo info;
    info.id = "CaliperTool";
    info.name = "卡尺工具";
    info.category = "测量";
    info.description = "沿投影方向搜索边缘跳变点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("edge_count", "边缘点数量", DataType::Number));
    info.outputs.push_back(DataPort("first_edge_x", "第一个边缘点X", DataType::Number));
    info.outputs.push_back(DataPort("first_edge_y", "第一个边缘点Y", DataType::Number));
    
    info.params.push_back(ParamDef("start_x", "起始点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("start_y", "起始点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_x", "结束点X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("end_y", "结束点Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("width", "搜索宽度", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("polarity", "极性（1=亮到暗，-1=暗到亮，0=双向）", DataType::Number, Data(0)));
    
    return info;
}

Result<void> CaliperToolNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 如果是多通道图像，转换为灰度
    ImageData gray;
    if (input.channels != 1) {
        gray.width = input.width;
        gray.height = input.height;
        gray.channels = 1;
        gray.format = ImageFormat::Mono8;
        gray.data.resize(gray.width * gray.height);
        
        for (size_t i = 0; i < gray.data.size(); ++i) {
            if (input.channels >= 3) {
                uint8_t b = input.data[i * 3];
                uint8_t g = input.data[i * 3 + 1];
                uint8_t r = input.data[i * 3 + 2];
                gray.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r);
            } else {
                gray.data[i] = input.data[i];
            }
        }
    } else {
        gray = input;
    }
    
    // 获取参数
    float start_x = get_param("start_x", Data(0.0)).as_number();
    float start_y = get_param("start_y", Data(0.0)).as_number();
    float end_x = get_param("end_x", Data(100.0)).as_number();
    float end_y = get_param("end_y", Data(100.0)).as_number();
    int width = get_param("width", Data(5)).as_int();
    float threshold = get_param("threshold", Data(20.0)).as_number();
    int polarity = get_param("polarity", Data(0)).as_int();
    
    // 执行卡尺搜索
    auto edge_points = measurement_utils::caliper_search(
        gray, start_x, start_y, end_x, end_y, width, threshold, polarity);
    
    // 输出结果
    set_output("edge_count", Data(static_cast<int>(edge_points.size())));
    
    if (!edge_points.empty()) {
        set_output("first_edge_x", Data(static_cast<double>(edge_points[0].x)));
        set_output("first_edge_y", Data(static_cast<double>(edge_points[0].y)));
    } else {
        set_output("first_edge_x", Data(0.0));
        set_output("first_edge_y", Data(0.0));
    }
    
    OVF_INFO() << "Caliper tool found " << edge_points.size() << " edge points";
    
    return Result<void>::success();
}

// ==================== LineFitNode ====================

LineFitNode::LineFitNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo LineFitNode::make_info() {
    NodeInfo info;
    info.id = "LineFit";
    info.name = "直线拟合";
    info.category = "测量";
    info.description = "使用最小二乘法拟合直线";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("line_a", "直线参数a", DataType::Number));
    info.outputs.push_back(DataPort("line_b", "直线参数b", DataType::Number));
    info.outputs.push_back(DataPort("line_c", "直线参数c", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    
    return info;
}

Result<void> LineFitNode::execute(FlowContext& context) {
    // 从参数中读取点集
    std::vector<Point2Df> points;
    
    // 从参数读取点（格式：point1_x, point1_y, point2_x, point2_y...）
    for (int i = 1; i <= 100; ++i) {  // 最多支持100个点
        std::string prefix = "point" + std::to_string(i) + "_";
        
        if (get_param(prefix + "x").is_number() && get_param(prefix + "y").is_number()) {
            float x = get_param(prefix + "x").as_number();
            float y = get_param(prefix + "y").as_number();
            points.emplace_back(x, y);
        } else {
            break;  // 没有更多点
        }
    }
    
    if (points.size() < 2) {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Need at least 2 points for line fitting");
    }
    
    // 执行直线拟合
    Line2D line = measurement_utils::fit_line(points);
    
    // 输出结果
    set_output("line_a", Data(static_cast<double>(line.a)));
    set_output("line_b", Data(static_cast<double>(line.b)));
    set_output("line_c", Data(static_cast<double>(line.c)));
    set_output("point_count", Data(static_cast<int>(points.size())));
    
    OVF_INFO() << "Line fitting completed with " << points.size() << " points"
               << ", line: " << line.a << "x + " << line.b << "y + " << line.c << " = 0";
    
    return Result<void>::success();
}

// ==================== CircleFitNode ====================

CircleFitNode::CircleFitNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CircleFitNode::make_info() {
    NodeInfo info;
    info.id = "CircleFit";
    info.name = "圆拟合";
    info.category = "测量";
    info.description = "使用最小二乘法拟合圆";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("center_x", "圆心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "圆心Y", DataType::Number));
    info.outputs.push_back(DataPort("radius", "半径", DataType::Number));
    info.outputs.push_back(DataPort("fit_error", "拟合误差", DataType::Number));
    info.outputs.push_back(DataPort("is_valid", "是否有效", DataType::Boolean));
    
    return info;
}

Result<void> CircleFitNode::execute(FlowContext& context) {
    // 从参数中读取点集
    std::vector<Point2Df> points;
    
    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        
        if (get_param(prefix + "x").is_number() && get_param(prefix + "y").is_number()) {
            float x = get_param(prefix + "x").as_number();
            float y = get_param(prefix + "y").as_number();
            points.emplace_back(x, y);
        } else {
            break;
        }
    }
    
    if (points.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Need at least 3 points for circle fitting");
    }
    
    // 执行圆拟合
    Circle2D circle = measurement_utils::fit_circle(points);
    
    // 输出结果
    set_output("center_x", Data(static_cast<double>(circle.center_x)));
    set_output("center_y", Data(static_cast<double>(circle.center_y)));
    set_output("radius", Data(static_cast<double>(circle.radius)));
    set_output("fit_error", Data(static_cast<double>(circle.error)));
    set_output("is_valid", Data(circle.valid));
    
    if (circle.valid) {
        OVF_INFO() << "Circle fitting completed with " << points.size() << " points"
                   << ", center: (" << circle.center_x << ", " << circle.center_y << ")"
                   << ", radius: " << circle.radius
                   << ", error: " << circle.error;
    } else {
        OVF_WARN() << "Circle fitting failed";
    }
    
    return Result<void>::success();
}

// ==================== DistanceMeasureNode ====================

DistanceMeasureNode::DistanceMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistanceMeasureNode::make_info() {
    NodeInfo info;
    info.id = "DistanceMeasure";
    info.name = "距离测量";
    info.category = "测量";
    info.description = "测量点距、线距";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    
    info.params.push_back(ParamDef("mode", "测量模式（1=点距，2=点到线）", DataType::Number, Data(1)));
    
    // 点距模式参数
    info.params.push_back(ParamDef("point1_x", "点1 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("point1_y", "点1 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("point2_x", "点2 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("point2_y", "点2 Y", DataType::Number, Data(0.0)));
    
    // 点到线模式参数
    info.params.push_back(ParamDef("point_x", "点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("point_y", "点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("line_a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("line_b", "直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("line_c", "直线参数c", DataType::Number, Data(0.0)));
    
    return info;
}

Result<void> DistanceMeasureNode::execute(FlowContext& context) {
    int mode = get_param("mode", Data(1)).as_int();
    
    float distance = 0.0f;
    
    if (mode == 1) {
        // 点距模式
        float p1_x = get_param("point1_x", Data(0.0)).as_number();
        float p1_y = get_param("point1_y", Data(0.0)).as_number();
        float p2_x = get_param("point2_x", Data(0.0)).as_number();
        float p2_y = get_param("point2_y", Data(0.0)).as_number();
        
        Point2Df p1(p1_x, p1_y);
        Point2Df p2(p2_x, p2_y);
        
        distance = measurement_utils::point_distance(p1, p2);
        
        OVF_INFO() << "Point-to-point distance: " << distance
                   << " between (" << p1_x << "," << p1_y << ") and (" << p2_x << "," << p2_y << ")";
    } else if (mode == 2) {
        // 点到线距离模式
        float px = get_param("point_x", Data(0.0)).as_number();
        float py = get_param("point_y", Data(0.0)).as_number();
        float la = get_param("line_a", Data(1.0)).as_number();
        float lb = get_param("line_b", Data(0.0)).as_number();
        float lc = get_param("line_c", Data(0.0)).as_number();
        
        Point2Df point(px, py);
        Line2D line;
        line.a = la;
        line.b = lb;
        line.c = lc;
        
        distance = measurement_utils::point_to_line_distance(point, line);
        
        OVF_INFO() << "Point-to-line distance: " << distance
                   << " from point (" << px << "," << py << ")"
                   << " to line " << la << "x + " << lb << "y + " << lc << " = 0";
    } else {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Invalid measurement mode. Use 1 for point distance or 2 for point-to-line distance.");
    }
    
    set_output("distance", Data(static_cast<double>(distance)));
    
    return Result<void>::success();
}

// ==================== AngleMeasureNode ====================

AngleMeasureNode::AngleMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AngleMeasureNode::make_info() {
    NodeInfo info;
    info.id = "AngleMeasure";
    info.name = "角度测量";
    info.category = "测量";
    info.description = "测量两条直线的夹角";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("angle_rad", "角度（弧度）", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度（度）", DataType::Number));
    
    info.params.push_back(ParamDef("line1_a", "直线1参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("line1_b", "直线1参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("line1_c", "直线1参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("line2_a", "直线2参数a", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("line2_b", "直线2参数b", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("line2_c", "直线2参数c", DataType::Number, Data(0.0)));
    
    return info;
}

Result<void> AngleMeasureNode::execute(FlowContext& context) {
    Line2D line1, line2;
    
    line1.a = get_param("line1_a", Data(1.0)).as_number();
    line1.b = get_param("line1_b", Data(0.0)).as_number();
    line1.c = get_param("line1_c", Data(0.0)).as_number();
    
    line2.a = get_param("line2_a", Data(0.0)).as_number();
    line2.b = get_param("line2_b", Data(1.0)).as_number();
    line2.c = get_param("line2_c", Data(0.0)).as_number();
    
    // 归一化
    line1.normalize();
    line2.normalize();
    
    // 计算夹角
    float angle_rad = measurement_utils::line_angle(line1, line2);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
    
    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    
    OVF_INFO() << "Angle measurement: " << angle_deg << " degrees (" << angle_rad << " radians)";
    
    return Result<void>::success();
}

// 注册所有节点
OVF_REGISTER_NODE(CaliperToolNode, "CaliperTool", CaliperToolNode::make_info());
OVF_REGISTER_NODE(LineFitNode, "LineFit", LineFitNode::make_info());
OVF_REGISTER_NODE(CircleFitNode, "CircleFit", CircleFitNode::make_info());
OVF_REGISTER_NODE(DistanceMeasureNode, "DistanceMeasure", DistanceMeasureNode::make_info());
OVF_REGISTER_NODE(AngleMeasureNode, "AngleMeasure", AngleMeasureNode::make_info());

} // namespace algorithm
} // namespace ovf