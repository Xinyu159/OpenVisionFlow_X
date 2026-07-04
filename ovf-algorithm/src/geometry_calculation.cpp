/**
 * @file geometry_calculation.cpp
 * @brief 几何计算工具库节点实现 - 参考VisionPro的CogDistance/CogAngle/CogIntersect工具系列
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>

#include "ovf/algorithm/geometry_calculation.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

using namespace geometry_calc_utils;

// ============================================================================
// 距离计算节点实现（10个）
// ============================================================================

// ==================== DistancePointPointNode ====================

DistancePointPointNode::DistancePointPointNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistancePointPointNode::make_info() {
    NodeInfo info;
    info.id = "DistancePointPoint";
    info.name = "点到点距离";
    info.category = "几何计算";
    info.description = "计算两个点之间的欧氏距离";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));

    info.params.push_back(ParamDef("x1", "点1 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "点1 Y坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "点2 X坐标", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "点2 Y坐标", DataType::Number, Data(0.0)));

    return info;
}

Result<void> DistancePointPointNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(0.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();

    float dist = distance_point_point(x1, y1, x2, y2);

    set_output("distance", Data(static_cast<double>(dist)));

    OVF_INFO() << "点到点距离: " << dist;

    return Result<void>::success();
}

// ==================== DistancePointLineNode ====================

DistancePointLineNode::DistancePointLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistancePointLineNode::make_info() {
    NodeInfo info;
    info.id = "DistancePointLine";
    info.name = "点到直线距离";
    info.category = "几何计算";
    info.description = "计算点到直线的距离（直线方程: ax + by + c = 0）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));

    info.params.push_back(ParamDef("px", "点 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("py", "点 Y坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "直线参数c", DataType::Number, Data(0.0)));

    return info;
}

Result<void> DistancePointLineNode::execute(FlowContext& context) {
    float px = get_param("px", Data(0.0)).as_number();
    float py = get_param("py", Data(0.0)).as_number();
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();

    float dist = distance_point_line(px, py, a, b, c);

    set_output("distance", Data(static_cast<double>(dist)));

    OVF_INFO() << "点到直线距离: " << dist;

    return Result<void>::success();
}

// ==================== DistancePointCircleNode ====================

DistancePointCircleNode::DistancePointCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistancePointCircleNode::make_info() {
    NodeInfo info;
    info.id = "DistancePointCircle";
    info.name = "点到圆距离";
    info.category = "几何计算";
    info.description = "计算点到圆边界的距离（正值=圆外，负值=圆内）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("inside", "点在圆内", DataType::Boolean));

    info.params.push_back(ParamDef("px", "点 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("py", "点 Y坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "圆心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "圆心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(50.0)));

    return info;
}

Result<void> DistancePointCircleNode::execute(FlowContext& context) {
    float px = get_param("px", Data(0.0)).as_number();
    float py = get_param("py", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float radius = get_param("radius", Data(50.0)).as_number();

    float dist = distance_point_circle(px, py, cx, cy, radius);
    bool inside = (dist < 0);

    set_output("distance", Data(static_cast<double>(std::abs(dist))));
    set_output("inside", Data(inside));

    OVF_INFO() << "点到圆距离: " << std::abs(dist) << (inside ? " (圆内)" : " (圆外)");

    return Result<void>::success();
}

// ==================== DistancePointEllipseNode ====================

DistancePointEllipseNode::DistancePointEllipseNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistancePointEllipseNode::make_info() {
    NodeInfo info;
    info.id = "DistancePointEllipse";
    info.name = "点到椭圆距离";
    info.category = "几何计算";
    info.description = "计算点到椭圆边界的距离（正值=椭圆外，负值=椭圆内）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("inside", "点在椭圆内", DataType::Boolean));

    info.params.push_back(ParamDef("px", "点 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("py", "点 Y坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "椭圆中心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "椭圆中心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("a", "长轴", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("b", "短轴", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("angle", "旋转角度(度)", DataType::Number, Data(0.0)));

    return info;
}

Result<void> DistancePointEllipseNode::execute(FlowContext& context) {
    float px = get_param("px", Data(0.0)).as_number();
    float py = get_param("py", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float a = get_param("a", Data(100.0)).as_number();
    float b = get_param("b", Data(50.0)).as_number();
    float angle_deg = get_param("angle", Data(0.0)).as_number();

    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

    float dist = distance_point_ellipse(px, py, cx, cy, a, b, angle_rad);
    bool inside = (dist < 0);

    set_output("distance", Data(static_cast<double>(std::abs(dist))));
    set_output("inside", Data(inside));

    OVF_INFO() << "点到椭圆距离: " << std::abs(dist) << (inside ? " (椭圆内)" : " (椭圆外)");

    return Result<void>::success();
}

// ==================== DistancePointSegmentNode ====================

DistancePointSegmentNode::DistancePointSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistancePointSegmentNode::make_info() {
    NodeInfo info;
    info.id = "DistancePointSegment";
    info.name = "点到线段距离";
    info.category = "几何计算";
    info.description = "计算点到线段的最短距离";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("nearest_x", "最近点 X", DataType::Number));
    info.outputs.push_back(DataPort("nearest_y", "最近点 Y", DataType::Number));

    info.params.push_back(ParamDef("px", "点 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("py", "点 Y坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x1", "线段起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "线段起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段终点 Y", DataType::Number, Data(0.0)));

    return info;
}

Result<void> DistancePointSegmentNode::execute(FlowContext& context) {
    float px = get_param("px", Data(0.0)).as_number();
    float py = get_param("py", Data(0.0)).as_number();
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();

    float dist = distance_point_segment(px, py, x1, y1, x2, y2);

    // 计算最近点
    float dx = x2 - x1;
    float dy = y2 - y1;
    float len_sq = dx * dx + dy * dy;

    float nearest_x = x1, nearest_y = y1;
    if (len_sq > 1e-10f) {
        float t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
        t = std::max(0.0f, std::min(1.0f, t));
        nearest_x = x1 + t * dx;
        nearest_y = y1 + t * dy;
    }

    set_output("distance", Data(static_cast<double>(dist)));
    set_output("nearest_x", Data(static_cast<double>(nearest_x)));
    set_output("nearest_y", Data(static_cast<double>(nearest_y)));

    OVF_INFO() << "点到线段距离: " << dist << ", 最近点(" << nearest_x << ", " << nearest_y << ")";

    return Result<void>::success();
}

// ==================== DistanceLineCircleNode ====================

DistanceLineCircleNode::DistanceLineCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistanceLineCircleNode::make_info() {
    NodeInfo info;
    info.id = "DistanceLineCircle";
    info.name = "线到圆距离";
    info.category = "几何计算";
    info.description = "计算直线到圆边界的最短距离";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("intersecting", "相交", DataType::Boolean));

    info.params.push_back(ParamDef("a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "直线参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "圆心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "圆心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(50.0)));

    return info;
}

Result<void> DistanceLineCircleNode::execute(FlowContext& context) {
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float radius = get_param("radius", Data(50.0)).as_number();

    float dist = distance_line_circle(a, b, c, cx, cy, radius);
    bool intersecting = (dist <= 0);

    set_output("distance", Data(static_cast<double>(std::abs(dist))));
    set_output("intersecting", Data(intersecting));

    OVF_INFO() << "线到圆距离: " << std::abs(dist) << (intersecting ? " (相交)" : " (不相交)");

    return Result<void>::success();
}

// ==================== DistanceLineEllipseNode ====================

DistanceLineEllipseNode::DistanceLineEllipseNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistanceLineEllipseNode::make_info() {
    NodeInfo info;
    info.id = "DistanceLineEllipse";
    info.name = "线到椭圆距离";
    info.category = "几何计算";
    info.description = "计算直线到椭圆边界的近似距离";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("intersecting", "可能相交", DataType::Boolean));

    info.params.push_back(ParamDef("a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "直线参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "椭圆中心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "椭圆中心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("ea", "长轴", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("eb", "短轴", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("angle", "旋转角度(度)", DataType::Number, Data(0.0)));

    return info;
}

Result<void> DistanceLineEllipseNode::execute(FlowContext& context) {
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float ea = get_param("ea", Data(100.0)).as_number();
    float eb = get_param("eb", Data(50.0)).as_number();
    float angle_deg = get_param("angle", Data(0.0)).as_number();

    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

    float dist = distance_line_ellipse(a, b, c, cx, cy, ea, eb, angle_rad);
    bool intersecting = (dist <= 0);

    set_output("distance", Data(static_cast<double>(std::abs(dist))));
    set_output("intersecting", Data(intersecting));

    OVF_INFO() << "线到椭圆距离: " << std::abs(dist) << " (近似值)";

    return Result<void>::success();
}

// ==================== DistanceLineLineNode ====================

DistanceLineLineNode::DistanceLineLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistanceLineLineNode::make_info() {
    NodeInfo info;
    info.id = "DistanceLineLine";
    info.name = "线到线距离";
    info.category = "几何计算";
    info.description = "计算两条直线的距离（仅平行线有效）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("parallel", "平行", DataType::Boolean));

    info.params.push_back(ParamDef("a1", "直线1参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b1", "直线1参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c1", "直线1参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("a2", "直线2参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b2", "直线2参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c2", "直线2参数c", DataType::Number, Data(50.0)));

    return info;
}

Result<void> DistanceLineLineNode::execute(FlowContext& context) {
    float a1 = get_param("a1", Data(1.0)).as_number();
    float b1 = get_param("b1", Data(0.0)).as_number();
    float c1 = get_param("c1", Data(0.0)).as_number();
    float a2 = get_param("a2", Data(1.0)).as_number();
    float b2 = get_param("b2", Data(0.0)).as_number();
    float c2 = get_param("c2", Data(50.0)).as_number();

    float dist = distance_line_line(a1, b1, c1, a2, b2, c2);

    // 检查是否平行
    float len1 = std::sqrt(a1 * a1 + b1 * b1);
    float len2 = std::sqrt(a2 * a2 + b2 * b2);
    float dot = std::abs(a1 * a2 + b1 * b2) / (len1 * len2);
    bool parallel = (dot > 0.999f);

    set_output("distance", Data(static_cast<double>(dist)));
    set_output("parallel", Data(parallel));

    if (parallel) {
        OVF_INFO() << "平行线距离: " << dist;
    } else {
        OVF_INFO() << "直线不平行，距离为0（相交）";
    }

    return Result<void>::success();
}

// ==================== DistanceSegmentCircleNode ====================

DistanceSegmentCircleNode::DistanceSegmentCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistanceSegmentCircleNode::make_info() {
    NodeInfo info;
    info.id = "DistanceSegmentCircle";
    info.name = "线段到圆距离";
    info.category = "几何计算";
    info.description = "计算线段到圆边界的最短距离";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("intersecting", "相交", DataType::Boolean));

    info.params.push_back(ParamDef("x1", "线段起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "线段起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段终点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "圆心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "圆心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(50.0)));

    return info;
}

Result<void> DistanceSegmentCircleNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float radius = get_param("radius", Data(50.0)).as_number();

    float dist = distance_segment_circle(x1, y1, x2, y2, cx, cy, radius);
    bool intersecting = (dist <= 0);

    set_output("distance", Data(static_cast<double>(std::abs(dist))));
    set_output("intersecting", Data(intersecting));

    OVF_INFO() << "线段到圆距离: " << std::abs(dist) << (intersecting ? " (相交)" : " (不相交)");

    return Result<void>::success();
}

// ==================== DistanceSegmentSegmentNode ====================

DistanceSegmentSegmentNode::DistanceSegmentSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DistanceSegmentSegmentNode::make_info() {
    NodeInfo info;
    info.id = "DistanceSegmentSegment";
    info.name = "线段到线段距离";
    info.category = "几何计算";
    info.description = "计算两条线段之间的最短距离";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("distance", "距离", DataType::Number));
    info.outputs.push_back(DataPort("intersecting", "相交", DataType::Boolean));

    info.params.push_back(ParamDef("x1", "线段1起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "线段1起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段1终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段1终点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x3", "线段2起点 X", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("y3", "线段2起点 Y", DataType::Number, Data(-50.0)));
    info.params.push_back(ParamDef("x4", "线段2终点 X", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("y4", "线段2终点 Y", DataType::Number, Data(50.0)));

    return info;
}

Result<void> DistanceSegmentSegmentNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();
    float x3 = get_param("x3", Data(50.0)).as_number();
    float y3 = get_param("y3", Data(-50.0)).as_number();
    float x4 = get_param("x4", Data(50.0)).as_number();
    float y4 = get_param("y4", Data(50.0)).as_number();

    float dist = distance_segment_segment(x1, y1, x2, y2, x3, y3, x4, y4);

    // 检查是否相交
    float ix, iy;
    bool intersecting = intersect_segment_segment(x1, y1, x2, y2, x3, y3, x4, y4, ix, iy);

    set_output("distance", Data(static_cast<double>(dist)));
    set_output("intersecting", Data(intersecting));

    OVF_INFO() << "线段到线段距离: " << dist << (intersecting ? " (相交)" : " (不相交)");

    return Result<void>::success();
}

// ============================================================================
// 角度计算节点实现（4个）
// ============================================================================

// ==================== AngleLineLineNode ====================

AngleLineLineNode::AngleLineLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AngleLineLineNode::make_info() {
    NodeInfo info;
    info.id = "AngleLineLine";
    info.name = "两直线夹角";
    info.category = "几何计算";
    info.description = "计算两条直线的夹角（返回锐角）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("angle_rad", "角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("parallel", "平行", DataType::Boolean));
    info.outputs.push_back(DataPort("perpendicular", "垂直", DataType::Boolean));

    info.params.push_back(ParamDef("a1", "直线1参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b1", "直线1参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("a2", "直线2参数a", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("b2", "直线2参数b", DataType::Number, Data(1.0)));

    return info;
}

Result<void> AngleLineLineNode::execute(FlowContext& context) {
    float a1 = get_param("a1", Data(1.0)).as_number();
    float b1 = get_param("b1", Data(0.0)).as_number();
    float a2 = get_param("a2", Data(0.0)).as_number();
    float b2 = get_param("b2", Data(1.0)).as_number();

    float angle_rad = angle_line_line(a1, b1, a2, b2);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);

    // 判断平行和垂直
    float len1 = std::sqrt(a1 * a1 + b1 * b1);
    float len2 = std::sqrt(a2 * a2 + b2 * b2);
    float dot = std::abs(a1 * a2 + b1 * b2) / (len1 * len2);
    bool parallel = (dot > 0.999f);
    bool perpendicular = (dot < 0.001f);

    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    set_output("parallel", Data(parallel));
    set_output("perpendicular", Data(perpendicular));

    OVF_INFO() << "两直线夹角: " << angle_deg << "度";

    return Result<void>::success();
}

// ==================== AnglePointPointNode ====================

AnglePointPointNode::AnglePointPointNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AnglePointPointNode::make_info() {
    NodeInfo info;
    info.id = "AnglePointPoint";
    info.name = "两点连线角度";
    info.category = "几何计算";
    info.description = "计算两点连线相对于X轴的角度";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("angle_rad", "角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度(度)", DataType::Number));

    info.params.push_back(ParamDef("x1", "点1 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "点1 Y坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "点2 X坐标", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "点2 Y坐标", DataType::Number, Data(100.0)));

    return info;
}

Result<void> AnglePointPointNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(100.0)).as_number();

    float angle_rad = angle_point_point(x1, y1, x2, y2);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);

    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));

    OVF_INFO() << "两点连线角度: " << angle_deg << "度";

    return Result<void>::success();
}

// ==================== AngleThreePointsNode ====================

AngleThreePointsNode::AngleThreePointsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AngleThreePointsNode::make_info() {
    NodeInfo info;
    info.id = "AngleThreePoints";
    info.name = "三点角度";
    info.category = "几何计算";
    info.description = "计算在点2处形成的角度（点1-点2-点3）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("angle_rad", "角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度(度)", DataType::Number));

    info.params.push_back(ParamDef("x1", "点1 X坐标", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "点1 Y坐标", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("x2", "点2 X坐标(顶点)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y2", "点2 Y坐标(顶点)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x3", "点3 X坐标", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y3", "点3 Y坐标", DataType::Number, Data(0.0)));

    return info;
}

Result<void> AngleThreePointsNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(100.0)).as_number();
    float x2 = get_param("x2", Data(0.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();
    float x3 = get_param("x3", Data(100.0)).as_number();
    float y3 = get_param("y3", Data(0.0)).as_number();

    float angle_rad = angle_three_points(x1, y1, x2, y2, x3, y3);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);

    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));

    OVF_INFO() << "三点角度: " << angle_deg << "度 (顶点在点2)";

    return Result<void>::success();
}

// ==================== AngleVectorVectorNode ====================

AngleVectorVectorNode::AngleVectorVectorNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AngleVectorVectorNode::make_info() {
    NodeInfo info;
    info.id = "AngleVectorVector";
    info.name = "两向量夹角";
    info.category = "几何计算";
    info.description = "计算两个向量之间的夹角";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("angle_rad", "角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("parallel", "平行", DataType::Boolean));
    info.outputs.push_back(DataPort("perpendicular", "垂直", DataType::Boolean));

    info.params.push_back(ParamDef("vx1", "向量1 X", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("vy1", "向量1 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("vx2", "向量2 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("vy2", "向量2 Y", DataType::Number, Data(1.0)));

    return info;
}

Result<void> AngleVectorVectorNode::execute(FlowContext& context) {
    float vx1 = get_param("vx1", Data(1.0)).as_number();
    float vy1 = get_param("vy1", Data(0.0)).as_number();
    float vx2 = get_param("vx2", Data(0.0)).as_number();
    float vy2 = get_param("vy2", Data(1.0)).as_number();

    float angle_rad = angle_vector_vector(vx1, vy1, vx2, vy2);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);

    // 判断平行和垂直
    float len1 = std::sqrt(vx1 * vx1 + vy1 * vy1);
    float len2 = std::sqrt(vx2 * vx2 + vy2 * vy2);
    float dot = std::abs(vx1 * vx2 + vy1 * vy2) / (len1 * len2);
    bool parallel = (dot > 0.999f);
    bool perpendicular = (dot < 0.001f);

    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    set_output("parallel", Data(parallel));
    set_output("perpendicular", Data(perpendicular));

    OVF_INFO() << "两向量夹角: " << angle_deg << "度";

    return Result<void>::success();
}

// ============================================================================
// 交点计算节点实现（8个）
// ============================================================================

// ==================== IntersectLineLineNode ====================

IntersectLineLineNode::IntersectLineLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectLineLineNode::make_info() {
    NodeInfo info;
    info.id = "IntersectLineLine";
    info.name = "线线交点";
    info.category = "几何计算";
    info.description = "计算两条直线的交点";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("x", "交点 X", DataType::Number));
    info.outputs.push_back(DataPort("y", "交点 Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "交点有效", DataType::Boolean));

    info.params.push_back(ParamDef("a1", "直线1参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b1", "直线1参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c1", "直线1参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("a2", "直线2参数a", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("b2", "直线2参数b", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("c2", "直线2参数c", DataType::Number, Data(-50.0)));

    return info;
}

Result<void> IntersectLineLineNode::execute(FlowContext& context) {
    float a1 = get_param("a1", Data(1.0)).as_number();
    float b1 = get_param("b1", Data(0.0)).as_number();
    float c1 = get_param("c1", Data(0.0)).as_number();
    float a2 = get_param("a2", Data(0.0)).as_number();
    float b2 = get_param("b2", Data(1.0)).as_number();
    float c2 = get_param("c2", Data(-50.0)).as_number();

    float x, y;
    bool valid = intersect_line_line(a1, b1, c1, a2, b2, c2, x, y);

    set_output("x", Data(static_cast<double>(x)));
    set_output("y", Data(static_cast<double>(y)));
    set_output("valid", Data(valid));

    if (valid) {
        OVF_INFO() << "线线交点: (" << x << ", " << y << ")";
    } else {
        OVF_INFO() << "直线平行，无交点";
    }

    return Result<void>::success();
}

// ==================== IntersectLineCircleNode ====================

IntersectLineCircleNode::IntersectLineCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectLineCircleNode::make_info() {
    NodeInfo info;
    info.id = "IntersectLineCircle";
    info.name = "线圆交点";
    info.category = "几何计算";
    info.description = "计算直线与圆的交点";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("count", "交点数量", DataType::Number));
    info.outputs.push_back(DataPort("x1", "交点1 X", DataType::Number));
    info.outputs.push_back(DataPort("y1", "交点1 Y", DataType::Number));
    info.outputs.push_back(DataPort("x2", "交点2 X", DataType::Number));
    info.outputs.push_back(DataPort("y2", "交点2 Y", DataType::Number));

    info.params.push_back(ParamDef("a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "直线参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "圆心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "圆心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(50.0)));

    return info;
}

Result<void> IntersectLineCircleNode::execute(FlowContext& context) {
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float radius = get_param("radius", Data(50.0)).as_number();

    float x1, y1, x2, y2;
    int count = intersect_line_circle(a, b, c, cx, cy, radius, x1, y1, x2, y2);

    set_output("count", Data(count));
    set_output("x1", Data(static_cast<double>(x1)));
    set_output("y1", Data(static_cast<double>(y1)));
    set_output("x2", Data(static_cast<double>(x2)));
    set_output("y2", Data(static_cast<double>(y2)));

    OVF_INFO() << "线圆交点数量: " << count;
    if (count > 0) {
        OVF_INFO() << "交点1: (" << x1 << ", " << y1 << ")";
    }
    if (count > 1) {
        OVF_INFO() << "交点2: (" << x2 << ", " << y2 << ")";
    }

    return Result<void>::success();
}

// ==================== IntersectLineEllipseNode ====================

IntersectLineEllipseNode::IntersectLineEllipseNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectLineEllipseNode::make_info() {
    NodeInfo info;
    info.id = "IntersectLineEllipse";
    info.name = "线椭圆交点";
    info.category = "几何计算";
    info.description = "计算直线与椭圆的交点（近似）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("count", "交点数量", DataType::Number));
    info.outputs.push_back(DataPort("x1", "交点1 X", DataType::Number));
    info.outputs.push_back(DataPort("y1", "交点1 Y", DataType::Number));
    info.outputs.push_back(DataPort("x2", "交点2 X", DataType::Number));
    info.outputs.push_back(DataPort("y2", "交点2 Y", DataType::Number));

    info.params.push_back(ParamDef("a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "直线参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "椭圆中心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "椭圆中心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("ea", "长轴", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("eb", "短轴", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("angle", "旋转角度(度)", DataType::Number, Data(0.0)));

    return info;
}

Result<void> IntersectLineEllipseNode::execute(FlowContext& context) {
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float ea = get_param("ea", Data(100.0)).as_number();
    float eb = get_param("eb", Data(50.0)).as_number();
    float angle_deg = get_param("angle", Data(0.0)).as_number();

    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

    float x1, y1, x2, y2;
    int count = intersect_line_ellipse(a, b, c, cx, cy, ea, eb, angle_rad, x1, y1, x2, y2);

    set_output("count", Data(count));
    set_output("x1", Data(static_cast<double>(x1)));
    set_output("y1", Data(static_cast<double>(y1)));
    set_output("x2", Data(static_cast<double>(x2)));
    set_output("y2", Data(static_cast<double>(y2)));

    OVF_INFO() << "线椭圆交点数量: " << count << " (近似值)";

    return Result<void>::success();
}

// ==================== IntersectCircleCircleNode ====================

IntersectCircleCircleNode::IntersectCircleCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectCircleCircleNode::make_info() {
    NodeInfo info;
    info.id = "IntersectCircleCircle";
    info.name = "圆圆交点";
    info.category = "几何计算";
    info.description = "计算两个圆的交点";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("count", "交点数量", DataType::Number));
    info.outputs.push_back(DataPort("x1", "交点1 X", DataType::Number));
    info.outputs.push_back(DataPort("y1", "交点1 Y", DataType::Number));
    info.outputs.push_back(DataPort("x2", "交点2 X", DataType::Number));
    info.outputs.push_back(DataPort("y2", "交点2 Y", DataType::Number));

    info.params.push_back(ParamDef("c1x", "圆1中心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c1y", "圆1中心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("r1", "圆1半径", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("c2x", "圆2中心 X", DataType::Number, Data(80.0)));
    info.params.push_back(ParamDef("c2y", "圆2中心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("r2", "圆2半径", DataType::Number, Data(50.0)));

    return info;
}

Result<void> IntersectCircleCircleNode::execute(FlowContext& context) {
    float c1x = get_param("c1x", Data(0.0)).as_number();
    float c1y = get_param("c1y", Data(0.0)).as_number();
    float r1 = get_param("r1", Data(50.0)).as_number();
    float c2x = get_param("c2x", Data(80.0)).as_number();
    float c2y = get_param("c2y", Data(0.0)).as_number();
    float r2 = get_param("r2", Data(50.0)).as_number();

    float x1, y1, x2, y2;
    int count = intersect_circle_circle(c1x, c1y, r1, c2x, c2y, r2, x1, y1, x2, y2);

    set_output("count", Data(count));
    set_output("x1", Data(static_cast<double>(x1)));
    set_output("y1", Data(static_cast<double>(y1)));
    set_output("x2", Data(static_cast<double>(x2)));
    set_output("y2", Data(static_cast<double>(y2)));

    OVF_INFO() << "圆圆交点数量: " << count;
    if (count > 0) {
        OVF_INFO() << "交点1: (" << x1 << ", " << y1 << ")";
    }
    if (count > 1) {
        OVF_INFO() << "交点2: (" << x2 << ", " << y2 << ")";
    }

    return Result<void>::success();
}

// ==================== IntersectSegmentLineNode ====================

IntersectSegmentLineNode::IntersectSegmentLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectSegmentLineNode::make_info() {
    NodeInfo info;
    info.id = "IntersectSegmentLine";
    info.name = "线段线交点";
    info.category = "几何计算";
    info.description = "计算线段与直线的交点";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("x", "交点 X", DataType::Number));
    info.outputs.push_back(DataPort("y", "交点 Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "交点有效", DataType::Boolean));

    info.params.push_back(ParamDef("x1", "线段起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "线段起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段终点 Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("a", "直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "直线参数b", DataType::Number, Data(-1.0)));
    info.params.push_back(ParamDef("c", "直线参数c", DataType::Number, Data(0.0)));

    return info;
}

Result<void> IntersectSegmentLineNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(100.0)).as_number();
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(-1.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();

    float x, y;
    bool valid = intersect_segment_line(x1, y1, x2, y2, a, b, c, x, y);

    set_output("x", Data(static_cast<double>(x)));
    set_output("y", Data(static_cast<double>(y)));
    set_output("valid", Data(valid));

    if (valid) {
        OVF_INFO() << "线段线交点: (" << x << ", " << y << ")";
    } else {
        OVF_INFO() << "无交点（平行或交点在线段外）";
    }

    return Result<void>::success();
}

// ==================== IntersectSegmentCircleNode ====================

IntersectSegmentCircleNode::IntersectSegmentCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectSegmentCircleNode::make_info() {
    NodeInfo info;
    info.id = "IntersectSegmentCircle";
    info.name = "线段圆交点";
    info.category = "几何计算";
    info.description = "计算线段与圆的交点";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("count", "交点数量", DataType::Number));
    info.outputs.push_back(DataPort("x1", "交点1 X", DataType::Number));
    info.outputs.push_back(DataPort("y1", "交点1 Y", DataType::Number));
    info.outputs.push_back(DataPort("x2", "交点2 X", DataType::Number));
    info.outputs.push_back(DataPort("y2", "交点2 Y", DataType::Number));

    info.params.push_back(ParamDef("x1", "线段起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "线段起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段终点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "圆心 X", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("cy", "圆心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(50.0)));

    return info;
}

Result<void> IntersectSegmentCircleNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();
    float cx = get_param("cx", Data(50.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float radius = get_param("radius", Data(50.0)).as_number();

    float x_out1, y_out1, x_out2, y_out2;
    int count = intersect_segment_circle(x1, y1, x2, y2, cx, cy, radius,
                                         x_out1, y_out1, x_out2, y_out2);

    set_output("count", Data(count));
    set_output("x1", Data(static_cast<double>(x_out1)));
    set_output("y1", Data(static_cast<double>(y_out1)));
    set_output("x2", Data(static_cast<double>(x_out2)));
    set_output("y2", Data(static_cast<double>(y_out2)));

    OVF_INFO() << "线段圆交点数量: " << count;

    return Result<void>::success();
}

// ==================== IntersectSegmentSegmentNode ====================

IntersectSegmentSegmentNode::IntersectSegmentSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectSegmentSegmentNode::make_info() {
    NodeInfo info;
    info.id = "IntersectSegmentSegment";
    info.name = "线段线段交点";
    info.category = "几何计算";
    info.description = "计算两条线段的交点";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("x", "交点 X", DataType::Number));
    info.outputs.push_back(DataPort("y", "交点 Y", DataType::Number));
    info.outputs.push_back(DataPort("valid", "交点有效", DataType::Boolean));

    info.params.push_back(ParamDef("x1", "线段1起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "线段1起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段1终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段1终点 Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("x3", "线段2起点 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y3", "线段2起点 Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("x4", "线段2终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y4", "线段2终点 Y", DataType::Number, Data(0.0)));

    return info;
}

Result<void> IntersectSegmentSegmentNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(100.0)).as_number();
    float x3 = get_param("x3", Data(0.0)).as_number();
    float y3 = get_param("y3", Data(100.0)).as_number();
    float x4 = get_param("x4", Data(100.0)).as_number();
    float y4 = get_param("y4", Data(0.0)).as_number();

    float x, y;
    bool valid = intersect_segment_segment(x1, y1, x2, y2, x3, y3, x4, y4, x, y);

    set_output("x", Data(static_cast<double>(x)));
    set_output("y", Data(static_cast<double>(y)));
    set_output("valid", Data(valid));

    if (valid) {
        OVF_INFO() << "线段线段交点: (" << x << ", " << y << ")";
    } else {
        OVF_INFO() << "无交点（平行或不相交）";
    }

    return Result<void>::success();
}

// ==================== IntersectSegmentEllipseNode ====================

IntersectSegmentEllipseNode::IntersectSegmentEllipseNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo IntersectSegmentEllipseNode::make_info() {
    NodeInfo info;
    info.id = "IntersectSegmentEllipse";
    info.name = "线段椭圆交点";
    info.category = "几何计算";
    info.description = "计算线段与椭圆的交点（近似）";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("count", "交点数量", DataType::Number));
    info.outputs.push_back(DataPort("x1", "交点1 X", DataType::Number));
    info.outputs.push_back(DataPort("y1", "交点1 Y", DataType::Number));
    info.outputs.push_back(DataPort("x2", "交点2 X", DataType::Number));
    info.outputs.push_back(DataPort("y2", "交点2 Y", DataType::Number));

    info.params.push_back(ParamDef("x1", "线段起点 X", DataType::Number, Data(-100.0)));
    info.params.push_back(ParamDef("y1", "线段起点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "线段终点 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "线段终点 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cx", "椭圆中心 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("cy", "椭圆中心 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("ea", "长轴", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("eb", "短轴", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("angle", "旋转角度(度)", DataType::Number, Data(0.0)));

    return info;
}

Result<void> IntersectSegmentEllipseNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(-100.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();
    float cx = get_param("cx", Data(0.0)).as_number();
    float cy = get_param("cy", Data(0.0)).as_number();
    float ea = get_param("ea", Data(100.0)).as_number();
    float eb = get_param("eb", Data(50.0)).as_number();
    float angle_deg = get_param("angle", Data(0.0)).as_number();

    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

    float x_out1, y_out1, x_out2, y_out2;
    int count = intersect_segment_ellipse(x1, y1, x2, y2, cx, cy, ea, eb, angle_rad,
                                          x_out1, y_out1, x_out2, y_out2);

    set_output("count", Data(count));
    set_output("x1", Data(static_cast<double>(x_out1)));
    set_output("y1", Data(static_cast<double>(y_out1)));
    set_output("x2", Data(static_cast<double>(x_out2)));
    set_output("y2", Data(static_cast<double>(y_out2)));

    OVF_INFO() << "线段椭圆交点数量: " << count << " (近似值)";

    return Result<void>::success();
}

// ============================================================================
// 几何创建节点实现（3个）
// ============================================================================

// ==================== CreateLineParallelNode ====================

CreateLineParallelNode::CreateLineParallelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CreateLineParallelNode::make_info() {
    NodeInfo info;
    info.id = "CreateLineParallel";
    info.name = "创建平行线";
    info.category = "几何计算";
    info.description = "创建与给定直线平行的直线";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("a", "平行线参数a", DataType::Number));
    info.outputs.push_back(DataPort("b", "平行线参数b", DataType::Number));
    info.outputs.push_back(DataPort("c", "平行线参数c", DataType::Number));

    info.params.push_back(ParamDef("a", "原直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "原直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "原直线参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("distance", "距离", DataType::Number, Data(50.0)));

    return info;
}

Result<void> CreateLineParallelNode::execute(FlowContext& context) {
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();
    float distance = get_param("distance", Data(50.0)).as_number();

    float pa, pb, pc;
    create_line_parallel(a, b, c, distance, pa, pb, pc);

    set_output("a", Data(static_cast<double>(pa)));
    set_output("b", Data(static_cast<double>(pb)));
    set_output("c", Data(static_cast<double>(pc)));

    OVF_INFO() << "创建平行线: " << pa << "x + " << pb << "y + " << pc << " = 0";

    return Result<void>::success();
}

// ==================== CreateLinePerpendicularNode ====================

CreateLinePerpendicularNode::CreateLinePerpendicularNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CreateLinePerpendicularNode::make_info() {
    NodeInfo info;
    info.id = "CreateLinePerpendicular";
    info.name = "创建垂线";
    info.category = "几何计算";
    info.description = "创建过给定点的垂直于原直线的直线";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("a", "垂线参数a", DataType::Number));
    info.outputs.push_back(DataPort("b", "垂线参数b", DataType::Number));
    info.outputs.push_back(DataPort("c", "垂线参数c", DataType::Number));

    info.params.push_back(ParamDef("a", "原直线参数a", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("b", "原直线参数b", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("c", "原直线参数c", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("px", "过点 X", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("py", "过点 Y", DataType::Number, Data(50.0)));

    return info;
}

Result<void> CreateLinePerpendicularNode::execute(FlowContext& context) {
    float a = get_param("a", Data(1.0)).as_number();
    float b = get_param("b", Data(0.0)).as_number();
    float c = get_param("c", Data(0.0)).as_number();
    float px = get_param("px", Data(50.0)).as_number();
    float py = get_param("py", Data(50.0)).as_number();

    float pa, pb, pc;
    create_line_perpendicular(a, b, c, px, py, pa, pb, pc);

    set_output("a", Data(static_cast<double>(pa)));
    set_output("b", Data(static_cast<double>(pb)));
    set_output("c", Data(static_cast<double>(pc)));

    OVF_INFO() << "创建垂线: " << pa << "x + " << pb << "y + " << pc << " = 0";

    return Result<void>::success();
}

// ==================== CreateLineBisectPointsNode ====================

CreateLineBisectPointsNode::CreateLineBisectPointsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CreateLineBisectPointsNode::make_info() {
    NodeInfo info;
    info.id = "CreateLineBisectPoints";
    info.name = "创建两点平分线";
    info.category = "几何计算";
    info.description = "创建过两点的垂直平分线";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("a", "平分线参数a", DataType::Number));
    info.outputs.push_back(DataPort("b", "平分线参数b", DataType::Number));
    info.outputs.push_back(DataPort("c", "平分线参数c", DataType::Number));
    info.outputs.push_back(DataPort("mid_x", "中点 X", DataType::Number));
    info.outputs.push_back(DataPort("mid_y", "中点 Y", DataType::Number));

    info.params.push_back(ParamDef("x1", "点1 X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y1", "点1 Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("x2", "点2 X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("y2", "点2 Y", DataType::Number, Data(0.0)));

    return info;
}

Result<void> CreateLineBisectPointsNode::execute(FlowContext& context) {
    float x1 = get_param("x1", Data(0.0)).as_number();
    float y1 = get_param("y1", Data(0.0)).as_number();
    float x2 = get_param("x2", Data(100.0)).as_number();
    float y2 = get_param("y2", Data(0.0)).as_number();

    float a, b, c;
    create_line_bisect_points(x1, y1, x2, y2, a, b, c);

    float mid_x = (x1 + x2) / 2.0f;
    float mid_y = (y1 + y2) / 2.0f;

    set_output("a", Data(static_cast<double>(a)));
    set_output("b", Data(static_cast<double>(b)));
    set_output("c", Data(static_cast<double>(c)));
    set_output("mid_x", Data(static_cast<double>(mid_x)));
    set_output("mid_y", Data(static_cast<double>(mid_y)));

    OVF_INFO() << "创建平分线: " << a << "x + " << b << "y + " << c << " = 0, 中点(" << mid_x << ", " << mid_y << ")";

    return Result<void>::success();
}

// ============================================================================
// 注册所有节点
// ============================================================================

// 距离计算节点（10个）
OVF_REGISTER_NODE(DistancePointPointNode, "DistancePointPoint", DistancePointPointNode::make_info())
OVF_REGISTER_NODE(DistancePointLineNode, "DistancePointLine", DistancePointLineNode::make_info())
OVF_REGISTER_NODE(DistancePointCircleNode, "DistancePointCircle", DistancePointCircleNode::make_info())
OVF_REGISTER_NODE(DistancePointEllipseNode, "DistancePointEllipse", DistancePointEllipseNode::make_info())
OVF_REGISTER_NODE(DistancePointSegmentNode, "DistancePointSegment", DistancePointSegmentNode::make_info())
OVF_REGISTER_NODE(DistanceLineCircleNode, "DistanceLineCircle", DistanceLineCircleNode::make_info())
OVF_REGISTER_NODE(DistanceLineEllipseNode, "DistanceLineEllipse", DistanceLineEllipseNode::make_info())
OVF_REGISTER_NODE(DistanceLineLineNode, "DistanceLineLine", DistanceLineLineNode::make_info())
OVF_REGISTER_NODE(DistanceSegmentCircleNode, "DistanceSegmentCircle", DistanceSegmentCircleNode::make_info())
OVF_REGISTER_NODE(DistanceSegmentSegmentNode, "DistanceSegmentSegment", DistanceSegmentSegmentNode::make_info())

// 角度计算节点（4个）
OVF_REGISTER_NODE(AngleLineLineNode, "AngleLineLine", AngleLineLineNode::make_info())
OVF_REGISTER_NODE(AnglePointPointNode, "AnglePointPoint", AnglePointPointNode::make_info())
OVF_REGISTER_NODE(AngleThreePointsNode, "AngleThreePoints", AngleThreePointsNode::make_info())
OVF_REGISTER_NODE(AngleVectorVectorNode, "AngleVectorVector", AngleVectorVectorNode::make_info())

// 交点计算节点（8个）
OVF_REGISTER_NODE(IntersectLineLineNode, "IntersectLineLine", IntersectLineLineNode::make_info())
OVF_REGISTER_NODE(IntersectLineCircleNode, "IntersectLineCircle", IntersectLineCircleNode::make_info())
OVF_REGISTER_NODE(IntersectLineEllipseNode, "IntersectLineEllipse", IntersectLineEllipseNode::make_info())
OVF_REGISTER_NODE(IntersectCircleCircleNode, "IntersectCircleCircle", IntersectCircleCircleNode::make_info())
OVF_REGISTER_NODE(IntersectSegmentLineNode, "IntersectSegmentLine", IntersectSegmentLineNode::make_info())
OVF_REGISTER_NODE(IntersectSegmentCircleNode, "IntersectSegmentCircle", IntersectSegmentCircleNode::make_info())
OVF_REGISTER_NODE(IntersectSegmentSegmentNode, "IntersectSegmentSegment", IntersectSegmentSegmentNode::make_info())
OVF_REGISTER_NODE(IntersectSegmentEllipseNode, "IntersectSegmentEllipse", IntersectSegmentEllipseNode::make_info())

// 几何创建节点（3个）
OVF_REGISTER_NODE(CreateLineParallelNode, "CreateLineParallel", CreateLineParallelNode::make_info())
OVF_REGISTER_NODE(CreateLinePerpendicularNode, "CreateLinePerpendicular", CreateLinePerpendicularNode::make_info())
OVF_REGISTER_NODE(CreateLineBisectPointsNode, "CreateLineBisectPoints", CreateLineBisectPointsNode::make_info())

} // namespace algorithm
} // namespace ovf