/**
 * @file geometry_measure.cpp
 * @brief 几何测量节点实现
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include "ovf/algorithm/geometry_measure.h"
#include "ovf/algorithm/measurement.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

// ==================== AreaMeasureNode ====================

AreaMeasureNode::AreaMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AreaMeasureNode::make_info() {
    NodeInfo info;
    info.id = "AreaMeasure";
    info.name = "面积测量";
    info.category = "几何测量";
    info.description = "测量多边形或轮廓的面积";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    
    // 从参数中读取点集
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> AreaMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    // 从参数读取点
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少3个点来计算面积");
    }
    
    // 计算面积（Shoelace公式）
    float area = geometry_measure_utils::polygon_area(x_coords, y_coords);
    
    set_output("area", Data(static_cast<double>(area)));
    set_output("point_count", Data(static_cast<int>(x_coords.size())));
    
    OVF_INFO() << "面积测量完成: " << area << " 平方像素, 点数: " << x_coords.size();
    
    return Result<void>::success();
}

// ==================== PerimeterMeasureNode ====================

PerimeterMeasureNode::PerimeterMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PerimeterMeasureNode::make_info() {
    NodeInfo info;
    info.id = "PerimeterMeasure";
    info.name = "周长测量";
    info.category = "几何测量";
    info.description = "测量多边形或轮廓的周长";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> PerimeterMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 2) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少2个点来计算周长");
    }
    
    // 计算周长
    float perimeter = geometry_measure_utils::polygon_perimeter(x_coords, y_coords);
    
    set_output("perimeter", Data(static_cast<double>(perimeter)));
    set_output("point_count", Data(static_cast<int>(x_coords.size())));
    
    OVF_INFO() << "周长测量完成: " << perimeter << " 像素, 点数: " << x_coords.size();
    
    return Result<void>::success();
}

// ==================== CenterMeasureNode ====================

CenterMeasureNode::CenterMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CenterMeasureNode::make_info() {
    NodeInfo info;
    info.id = "CenterMeasure";
    info.name = "中心点测量";
    info.category = "几何测量";
    info.description = "测量点集的中心点（质心）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("center_x", "中心点X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心点Y", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> CenterMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少1个点来计算中心");
    }
    
    // 计算中心点
    float center_x = 0.0f, center_y = 0.0f;
    geometry_measure_utils::polygon_center(x_coords, y_coords, center_x, center_y);
    
    set_output("center_x", Data(static_cast<double>(center_x)));
    set_output("center_y", Data(static_cast<double>(center_y)));
    set_output("point_count", Data(static_cast<int>(x_coords.size())));
    
    OVF_INFO() << "中心点测量完成: (" << center_x << ", " << center_y << "), 点数: " << x_coords.size();
    
    return Result<void>::success();
}

// ==================== CircleMeasureNode ====================

CircleMeasureNode::CircleMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CircleMeasureNode::make_info() {
    NodeInfo info;
    info.id = "CircleMeasure";
    info.name = "圆测量";
    info.category = "几何测量";
    info.description = "测量轮廓的圆度、等效直径等特征";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("circularity", "圆度", DataType::Number));
    info.outputs.push_back(DataPort("equivalent_diameter", "等效直径", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("bounding_radius", "外接圆半径", DataType::Number));
    
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> CircleMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少3个点来测量圆度");
    }
    
    // 计算面积和周长
    float area = geometry_measure_utils::polygon_area(x_coords, y_coords);
    float perimeter = geometry_measure_utils::polygon_perimeter(x_coords, y_coords);
    
    // 计算圆度
    float circularity = geometry_measure_utils::circularity(area, perimeter);
    
    // 等效直径（面积等效圆的直径）
    float equivalent_diameter = 2.0f * std::sqrt(area / static_cast<float>(M_PI));
    
    // 计算中心
    float center_x = 0.0f, center_y = 0.0f;
    geometry_measure_utils::polygon_center(x_coords, y_coords, center_x, center_y);
    
    // 计算外接圆半径
    float bounding_radius = 0.0f;
    geometry_measure_utils::min_bounding_circle(x_coords, y_coords, center_x, center_y, bounding_radius);
    
    set_output("circularity", Data(static_cast<double>(circularity)));
    set_output("equivalent_diameter", Data(static_cast<double>(equivalent_diameter)));
    set_output("area", Data(static_cast<double>(area)));
    set_output("perimeter", Data(static_cast<double>(perimeter)));
    set_output("center_x", Data(static_cast<double>(center_x)));
    set_output("center_y", Data(static_cast<double>(center_y)));
    set_output("bounding_radius", Data(static_cast<double>(bounding_radius)));
    
    OVF_INFO() << "圆测量完成: 圆度=" << circularity << ", 等效直径=" << equivalent_diameter;
    
    return Result<void>::success();
}

// ==================== LineMeasureNode ====================

LineMeasureNode::LineMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo LineMeasureNode::make_info() {
    NodeInfo info;
    info.id = "LineMeasure";
    info.name = "直线测量";
    info.category = "几何测量";
    info.description = "测量直线的长度和角度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("length", "长度", DataType::Number));
    info.outputs.push_back(DataPort("angle_rad", "角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("start_x", "起点X", DataType::Number));
    info.outputs.push_back(DataPort("start_y", "起点Y", DataType::Number));
    info.outputs.push_back(DataPort("end_x", "终点X", DataType::Number));
    info.outputs.push_back(DataPort("end_y", "终点Y", DataType::Number));
    
    info.params.push_back(ParamDef("start_x", "起点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("start_y", "起点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_x", "终点X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("end_y", "终点Y", DataType::Number, Data(0.0)));
    
    return info;
}

Result<void> LineMeasureNode::execute(FlowContext& context) {
    float start_x = get_param("start_x", Data(0.0)).as_number();
    float start_y = get_param("start_y", Data(0.0)).as_number();
    float end_x = get_param("end_x", Data(100.0)).as_number();
    float end_y = get_param("end_y", Data(0.0)).as_number();
    
    // 计算长度
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float length = std::sqrt(dx * dx + dy * dy);
    
    // 计算角度（相对于X轴）
    float angle_rad = std::atan2(dy, dx);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
    
    set_output("length", Data(static_cast<double>(length)));
    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    set_output("start_x", Data(static_cast<double>(start_x)));
    set_output("start_y", Data(static_cast<double>(start_y)));
    set_output("end_x", Data(static_cast<double>(end_x)));
    set_output("end_y", Data(static_cast<double>(end_y)));
    
    OVF_INFO() << "直线测量完成: 长度=" << length << ", 角度=" << angle_deg << "度";
    
    return Result<void>::success();
}

// ==================== RectangleMeasureNode ====================

RectangleMeasureNode::RectangleMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo RectangleMeasureNode::make_info() {
    NodeInfo info;
    info.id = "RectangleMeasure";
    info.name = "矩形测量";
    info.category = "几何测量";
    info.description = "测量矩形的长宽、面积、矩形度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("width", "宽度", DataType::Number));
    info.outputs.push_back(DataPort("height", "高度", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("rectangularity", "矩形度", DataType::Number));
    info.outputs.push_back(DataPort("aspect_ratio", "宽高比", DataType::Number));
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> RectangleMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少3个点来测量矩形度");
    }
    
    // 计算实际面积
    float area = geometry_measure_utils::polygon_area(x_coords, y_coords);
    
    // 计算边界矩形
    Rectangle2D bounding_rect = geometry_measure_utils::min_bounding_rect(x_coords, y_coords);
    float bounding_area = bounding_rect.width * bounding_rect.height;
    
    // 计算矩形度
    float rectangularity = geometry_measure_utils::rectangularity(area, bounding_area);
    
    // 宽高比
    float aspect_ratio = (bounding_rect.height > 1e-6f) ?
        bounding_rect.width / bounding_rect.height : 0.0f;
    
    // 计算中心
    float center_x = bounding_rect.x + bounding_rect.width / 2.0f;
    float center_y = bounding_rect.y + bounding_rect.height / 2.0f;
    
    set_output("width", Data(static_cast<double>(bounding_rect.width)));
    set_output("height", Data(static_cast<double>(bounding_rect.height)));
    set_output("area", Data(static_cast<double>(area)));
    set_output("rectangularity", Data(static_cast<double>(rectangularity)));
    set_output("aspect_ratio", Data(static_cast<double>(aspect_ratio)));
    set_output("center_x", Data(static_cast<double>(center_x)));
    set_output("center_y", Data(static_cast<double>(center_y)));
    
    OVF_INFO() << "矩形测量完成: 宽=" << bounding_rect.width << ", 高=" << bounding_rect.height
               << ", 矩形度=" << rectangularity;
    
    return Result<void>::success();
}

// ==================== EllipseMeasureNode ====================

EllipseMeasureNode::EllipseMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo EllipseMeasureNode::make_info() {
    NodeInfo info;
    info.id = "EllipseMeasure";
    info.name = "椭圆测量";
    info.category = "几何测量";
    info.description = "测量椭圆的长短轴、面积、椭圆度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("major_axis", "长轴", DataType::Number));
    info.outputs.push_back(DataPort("minor_axis", "短轴", DataType::Number));
    info.outputs.push_back(DataPort("angle_rad", "旋转角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "旋转角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("ellipticity", "椭圆度", DataType::Number));
    info.outputs.push_back(DataPort("is_valid", "是否有效", DataType::Boolean));
    
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> EllipseMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 5) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少5个点来拟合椭圆");
    }
    
    // 椭圆拟合
    Ellipse2D ellipse = geometry_measure_utils::fit_ellipse(x_coords, y_coords);
    
    // 计算椭圆面积
    float area = static_cast<float>(M_PI) * ellipse.major_axis * ellipse.minor_axis / 4.0f;
    
    // 椭圆度（长短轴比）
    float ellipticity = (ellipse.major_axis > 1e-6f) ?
        ellipse.minor_axis / ellipse.major_axis : 0.0f;
    
    // 旋转角度
    float angle_deg = ellipse.angle * 180.0f / static_cast<float>(M_PI);
    
    set_output("center_x", Data(static_cast<double>(ellipse.center_x)));
    set_output("center_y", Data(static_cast<double>(ellipse.center_y)));
    set_output("major_axis", Data(static_cast<double>(ellipse.major_axis)));
    set_output("minor_axis", Data(static_cast<double>(ellipse.minor_axis)));
    set_output("angle_rad", Data(static_cast<double>(ellipse.angle)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    set_output("area", Data(static_cast<double>(area)));
    set_output("ellipticity", Data(static_cast<double>(ellipticity)));
    set_output("is_valid", Data(ellipse.valid));
    
    if (ellipse.valid) {
        OVF_INFO() << "椭圆测量完成: 长轴=" << ellipse.major_axis << ", 短轴=" << ellipse.minor_axis
                   << ", 椭圆度=" << ellipticity;
    } else {
        OVF_WARN() << "椭圆测量失败";
    }
    
    return Result<void>::success();
}

// ==================== PolygonMeasureNode ====================

PolygonMeasureNode::PolygonMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PolygonMeasureNode::make_info() {
    NodeInfo info;
    info.id = "PolygonMeasure";
    info.name = "多边形测量";
    info.category = "几何测量";
    info.description = "测量多边形的基本几何特征";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("vertex_count", "顶点数", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("bounding_width", "边界宽度", DataType::Number));
    info.outputs.push_back(DataPort("bounding_height", "边界高度", DataType::Number));
    info.outputs.push_back(DataPort("circularity", "圆度", DataType::Number));
    info.outputs.push_back(DataPort("rectangularity", "矩形度", DataType::Number));
    
    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> PolygonMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 50; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少3个点来测量多边形");
    }
    
    // 基本测量
    float area = geometry_measure_utils::polygon_area(x_coords, y_coords);
    float perimeter = geometry_measure_utils::polygon_perimeter(x_coords, y_coords);
    float center_x = 0.0f, center_y = 0.0f;
    geometry_measure_utils::polygon_center(x_coords, y_coords, center_x, center_y);
    
    // 边界矩形
    Rectangle2D bounding_rect = geometry_measure_utils::min_bounding_rect(x_coords, y_coords);
    float bounding_area = bounding_rect.width * bounding_rect.height;
    
    // 形状特征
    float circularity = geometry_measure_utils::circularity(area, perimeter);
    float rectangularity = geometry_measure_utils::rectangularity(area, bounding_area);
    
    set_output("vertex_count", Data(static_cast<int>(x_coords.size())));
    set_output("area", Data(static_cast<double>(area)));
    set_output("perimeter", Data(static_cast<double>(perimeter)));
    set_output("center_x", Data(static_cast<double>(center_x)));
    set_output("center_y", Data(static_cast<double>(center_y)));
    set_output("bounding_width", Data(static_cast<double>(bounding_rect.width)));
    set_output("bounding_height", Data(static_cast<double>(bounding_rect.height)));
    set_output("circularity", Data(static_cast<double>(circularity)));
    set_output("rectangularity", Data(static_cast<double>(rectangularity)));
    
    OVF_INFO() << "多边形测量完成: 顶点数=" << x_coords.size() << ", 面积=" << area
               << ", 周长=" << perimeter << ", 圆度=" << circularity;
    
    return Result<void>::success();
}

// ==================== CircleFitMeasureNode ====================

CircleFitMeasureNode::CircleFitMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CircleFitMeasureNode::make_info() {
    NodeInfo info;
    info.id = "CircleFitMeasure";
    info.name = "圆拟合测量";
    info.category = "拟合测量";
    info.description = "拟合圆并测量圆的特征参数";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("center_x", "圆心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "圆心Y", DataType::Number));
    info.outputs.push_back(DataPort("radius", "半径", DataType::Number));
    info.outputs.push_back(DataPort("diameter", "直径", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("fit_error", "拟合误差", DataType::Number));
    info.outputs.push_back(DataPort("is_valid", "是否有效", DataType::Boolean));
    info.outputs.push_back(DataPort("circularity", "圆度", DataType::Number));
    
    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> CircleFitMeasureNode::execute(FlowContext& context) {
    std::vector<Point2Df> points;

    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        points.emplace_back(x, y);
    }

    if (points.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少3个点来拟合圆");
    }
    
    // 使用measurement_utils中的圆拟合
    Circle2D circle = measurement_utils::fit_circle(points);
    
    // 计算衍生参数
    float diameter = circle.radius * 2.0f;
    float area = static_cast<float>(M_PI) * circle.radius * circle.radius;
    float perimeter = 2.0f * static_cast<float>(M_PI) * circle.radius;
    
    // 计算实际轮廓的圆度
    std::vector<float> x_coords, y_coords;
    for (const auto& p : points) {
        x_coords.push_back(p.x);
        y_coords.push_back(p.y);
    }
    float actual_area = geometry_measure_utils::polygon_area(x_coords, y_coords);
    float actual_perimeter = geometry_measure_utils::polygon_perimeter(x_coords, y_coords);
    float circularity = geometry_measure_utils::circularity(actual_area, actual_perimeter);
    
    set_output("center_x", Data(static_cast<double>(circle.center_x)));
    set_output("center_y", Data(static_cast<double>(circle.center_y)));
    set_output("radius", Data(static_cast<double>(circle.radius)));
    set_output("diameter", Data(static_cast<double>(diameter)));
    set_output("area", Data(static_cast<double>(area)));
    set_output("perimeter", Data(static_cast<double>(perimeter)));
    set_output("fit_error", Data(static_cast<double>(circle.error)));
    set_output("is_valid", Data(circle.valid));
    set_output("circularity", Data(static_cast<double>(circularity)));
    
    if (circle.valid) {
        OVF_INFO() << "圆拟合测量完成: 圆心=(" << circle.center_x << "," << circle.center_y
                   << "), 半径=" << circle.radius << ", 拟合误差=" << circle.error;
    } else {
        OVF_WARN() << "圆拟合失败";
    }
    
    return Result<void>::success();
}

// ==================== LineFitMeasureNode ====================

LineFitMeasureNode::LineFitMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo LineFitMeasureNode::make_info() {
    NodeInfo info;
    info.id = "LineFitMeasure";
    info.name = "直线拟合测量";
    info.category = "拟合测量";
    info.description = "拟合直线并测量直线特征";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("line_a", "直线参数a", DataType::Number));
    info.outputs.push_back(DataPort("line_b", "直线参数b", DataType::Number));
    info.outputs.push_back(DataPort("line_c", "直线参数c", DataType::Number));
    info.outputs.push_back(DataPort("angle_rad", "角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("length", "长度", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    info.outputs.push_back(DataPort("mean_error", "平均拟合误差", DataType::Number));
    
    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> LineFitMeasureNode::execute(FlowContext& context) {
    std::vector<Point2Df> points;

    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        points.emplace_back(x, y);
    }

    if (points.size() < 2) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少2个点来拟合直线");
    }
    
    // 使用measurement_utils中的直线拟合
    Line2D line = measurement_utils::fit_line(points);
    
    // 计算角度（相对于X轴）
    // 对于直线 ax + by + c = 0，方向向量是 (-b, a)
    float angle_rad = std::atan2(-line.b, line.a);
    float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
    
    // 计算长度（端点距离）
    float min_dist = std::numeric_limits<float>::max();
    float max_dist = 0.0f;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            float dist = points[i].distance_to(points[j]);
            min_dist = std::min(min_dist, dist);
            max_dist = std::max(max_dist, dist);
        }
    }
    
    // 计算平均拟合误差
    double total_error = 0.0;
    for (const auto& p : points) {
        total_error += line.distance_to_point(p);
    }
    float mean_error = static_cast<float>(total_error / points.size());
    
    set_output("line_a", Data(static_cast<double>(line.a)));
    set_output("line_b", Data(static_cast<double>(line.b)));
    set_output("line_c", Data(static_cast<double>(line.c)));
    set_output("angle_rad", Data(static_cast<double>(angle_rad)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    set_output("length", Data(static_cast<double>(max_dist)));
    set_output("point_count", Data(static_cast<int>(points.size())));
    set_output("mean_error", Data(static_cast<double>(mean_error)));
    
    OVF_INFO() << "直线拟合测量完成: 直线=" << line.a << "x + " << line.b << "y + "
               << line.c << " = 0, 角度=" << angle_deg << "度, 平均误差=" << mean_error;
    
    return Result<void>::success();
}

// ==================== EllipseFitMeasureNode ====================

EllipseFitMeasureNode::EllipseFitMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo EllipseFitMeasureNode::make_info() {
    NodeInfo info;
    info.id = "EllipseFitMeasure";
    info.name = "椭圆拟合测量";
    info.category = "拟合测量";
    info.description = "拟合椭圆并测量椭圆特征";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("major_axis", "长轴", DataType::Number));
    info.outputs.push_back(DataPort("minor_axis", "短轴", DataType::Number));
    info.outputs.push_back(DataPort("angle_rad", "旋转角度(弧度)", DataType::Number));
    info.outputs.push_back(DataPort("angle_deg", "旋转角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("ellipticity", "椭圆度", DataType::Number));
    info.outputs.push_back(DataPort("fit_error", "拟合误差", DataType::Number));
    info.outputs.push_back(DataPort("is_valid", "是否有效", DataType::Boolean));
    
    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> EllipseFitMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (x_coords.size() < 5) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少5个点来拟合椭圆");
    }
    
    // 椭圆拟合
    Ellipse2D ellipse = geometry_measure_utils::fit_ellipse(x_coords, y_coords);
    
    // 计算椭圆面积
    float area = static_cast<float>(M_PI) * ellipse.major_axis * ellipse.minor_axis / 4.0f;
    
    // 椭圆周长近似（Ramanujan公式）
    float h = std::pow((ellipse.major_axis - ellipse.minor_axis) /
                       (ellipse.major_axis + ellipse.minor_axis), 2);
    float perimeter = static_cast<float>(M_PI) * (ellipse.major_axis + ellipse.minor_axis) / 2.0f *
                      (1.0f + 3.0f * h / (10.0f + std::sqrt(4.0f - 3.0f * h)));
    
    // 椭圆度
    float ellipticity = (ellipse.major_axis > 1e-6f) ?
        ellipse.minor_axis / ellipse.major_axis : 0.0f;
    
    // 计算拟合误差
    double total_error = 0.0;
    for (size_t i = 0; i < x_coords.size(); ++i) {
        float dist = geometry_measure_utils::point_to_ellipse_distance(
            x_coords[i], y_coords[i],
            ellipse.center_x, ellipse.center_y,
            ellipse.major_axis / 2.0f, ellipse.minor_axis / 2.0f,
            ellipse.angle);
        total_error += dist;
    }
    ellipse.error = static_cast<float>(total_error / x_coords.size());
    
    float angle_deg = ellipse.angle * 180.0f / static_cast<float>(M_PI);
    
    set_output("center_x", Data(static_cast<double>(ellipse.center_x)));
    set_output("center_y", Data(static_cast<double>(ellipse.center_y)));
    set_output("major_axis", Data(static_cast<double>(ellipse.major_axis)));
    set_output("minor_axis", Data(static_cast<double>(ellipse.minor_axis)));
    set_output("angle_rad", Data(static_cast<double>(ellipse.angle)));
    set_output("angle_deg", Data(static_cast<double>(angle_deg)));
    set_output("area", Data(static_cast<double>(area)));
    set_output("perimeter", Data(static_cast<double>(perimeter)));
    set_output("ellipticity", Data(static_cast<double>(ellipticity)));
    set_output("fit_error", Data(static_cast<double>(ellipse.error)));
    set_output("is_valid", Data(ellipse.valid));
    
    if (ellipse.valid) {
        OVF_INFO() << "椭圆拟合测量完成: 长轴=" << ellipse.major_axis << ", 短轴=" << ellipse.minor_axis
                   << ", 椭圆度=" << ellipticity << ", 拟合误差=" << ellipse.error;
    } else {
        OVF_WARN() << "椭圆拟合失败";
    }
    
    return Result<void>::success();
}

// ==================== ContourFitMeasureNode ====================

ContourFitMeasureNode::ContourFitMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContourFitMeasureNode::make_info() {
    NodeInfo info;
    info.id = "ContourFitMeasure";
    info.name = "轮廓拟合测量";
    info.category = "拟合测量";
    info.description = "拟合轮廓并测量轮廓特征（支持多种形状类型）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点集", DataType::Array, true));
    
    info.outputs.push_back(DataPort("fit_type", "拟合类型", DataType::Number));
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("circularity", "圆度", DataType::Number));
    info.outputs.push_back(DataPort("rectangularity", "矩形度", DataType::Number));
    info.outputs.push_back(DataPort("fit_error", "拟合误差", DataType::Number));
    info.outputs.push_back(DataPort("is_valid", "是否有效", DataType::Boolean));
    
    info.params.push_back(ParamDef("fit_mode", "拟合模式(1=圆,2=椭圆,3=矩形,4=多边形)", DataType::Number, Data(1)));
    
    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> ContourFitMeasureNode::execute(FlowContext& context) {
    std::vector<Point2Df> points;
    std::vector<float> x_coords, y_coords;

    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        points.emplace_back(x, y);
        x_coords.push_back(x);
        y_coords.push_back(y);
    }

    if (points.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少3个点来拟合轮廓");
    }
    
    int fit_mode = get_param("fit_mode", Data(1)).as_int();
    
    float area = 0.0f, perimeter = 0.0f, center_x = 0.0f, center_y = 0.0f;
    float circularity = 0.0f, rectangularity = 0.0f, fit_error = 0.0f;
    bool is_valid = true;
    
    // 基本测量
    area = geometry_measure_utils::polygon_area(x_coords, y_coords);
    perimeter = geometry_measure_utils::polygon_perimeter(x_coords, y_coords);
    geometry_measure_utils::polygon_center(x_coords, y_coords, center_x, center_y);
    
    Rectangle2D bounding_rect = geometry_measure_utils::min_bounding_rect(x_coords, y_coords);
    float bounding_area = bounding_rect.width * bounding_rect.height;
    
    circularity = geometry_measure_utils::circularity(area, perimeter);
    rectangularity = geometry_measure_utils::rectangularity(area, bounding_area);
    
    // 根据拟合模式计算拟合误差
    switch (fit_mode) {
        case 1: { // 圆拟合
            Circle2D circle = measurement_utils::fit_circle(points);
            fit_error = circle.error;
            is_valid = circle.valid;
            break;
        }
        case 2: { // 椭圆拟合
            if (points.size() >= 5) {
                Ellipse2D ellipse = geometry_measure_utils::fit_ellipse(x_coords, y_coords);
                fit_error = ellipse.error;
                is_valid = ellipse.valid;
            } else {
                is_valid = false;
                fit_error = 0.0f;
            }
            break;
        }
        case 3: { // 矩形拟合
            // 计算点到边界矩形的平均距离
            double total_error = 0.0;
            for (size_t i = 0; i < x_coords.size(); ++i) {
                // 计算点到矩形边界的最小距离
                float dx = std::min(std::abs(x_coords[i] - bounding_rect.x),
                                   std::abs(x_coords[i] - (bounding_rect.x + bounding_rect.width)));
                float dy = std::min(std::abs(y_coords[i] - bounding_rect.y),
                                   std::abs(y_coords[i] - (bounding_rect.y + bounding_rect.height)));
                total_error += std::min(dx, dy);
            }
            fit_error = static_cast<float>(total_error / x_coords.size());
            is_valid = true;
            break;
        }
        case 4: { // 多边形拟合（直接使用轮廓）
            fit_error = 0.0f; // 多边形拟合无误差（就是轮廓本身）
            is_valid = true;
            break;
        }
        default:
            return Result<void>::failure(ErrorCode::InvalidParameter,
                "无效的拟合模式，请使用1-4");
    }
    
    set_output("fit_type", Data(fit_mode));
    set_output("center_x", Data(static_cast<double>(center_x)));
    set_output("center_y", Data(static_cast<double>(center_y)));
    set_output("area", Data(static_cast<double>(area)));
    set_output("perimeter", Data(static_cast<double>(perimeter)));
    set_output("circularity", Data(static_cast<double>(circularity)));
    set_output("rectangularity", Data(static_cast<double>(rectangularity)));
    set_output("fit_error", Data(static_cast<double>(fit_error)));
    set_output("is_valid", Data(is_valid));
    
    OVF_INFO() << "轮廓拟合测量完成: 模式=" << fit_mode << ", 面积=" << area
               << ", 周长=" << perimeter << ", 拟合误差=" << fit_error;
    
    return Result<void>::success();
}

// ==================== PointCloudFitMeasureNode ====================

PointCloudFitMeasureNode::PointCloudFitMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PointCloudFitMeasureNode::make_info() {
    NodeInfo info;
    info.id = "PointCloudFitMeasure";
    info.name = "点云拟合测量";
    info.category = "拟合测量";
    info.description = "拟合点云数据并测量特征";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("points", "输入点云", DataType::Array, true));
    
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("center_z", "中心Z", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    info.outputs.push_back(DataPort("spread_x", "X方向分散度", DataType::Number));
    info.outputs.push_back(DataPort("spread_y", "Y方向分散度", DataType::Number));
    info.outputs.push_back(DataPort("spread_z", "Z方向分散度", DataType::Number));
    info.outputs.push_back(DataPort("total_spread", "总体分散度", DataType::Number));
    info.outputs.push_back(DataPort("density", "密度", DataType::Number));
    info.outputs.push_back(DataPort("bounding_width", "边界宽度", DataType::Number));
    info.outputs.push_back(DataPort("bounding_height", "边界高度", DataType::Number));
    info.outputs.push_back(DataPort("bounding_depth", "边界深度", DataType::Number));
    
    info.params.push_back(ParamDef("has_z", "是否包含Z坐标", DataType::Boolean, Data(false)));
    
    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        info.params.push_back(ParamDef(prefix + "x", "点" + std::to_string(i) + " X", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "y", "点" + std::to_string(i) + " Y", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef(prefix + "z", "点" + std::to_string(i) + " Z", DataType::Number, Data(0.0)));
    }
    
    return info;
}

Result<void> PointCloudFitMeasureNode::execute(FlowContext& context) {
    std::vector<float> x_coords, y_coords, z_coords;
    bool has_z = get_param("has_z", Data(false)).as_bool();

    for (int i = 1; i <= 100; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";

        float x = get_param(prefix + "x", Data(0.0)).as_number();
        float y = get_param(prefix + "y", Data(0.0)).as_number();

        // 如果点的坐标都为0（默认值），且不是第一个点，则认为后续点未设置
        if (i > 1 && x == 0.0f && y == 0.0f) {
            break;
        }

        x_coords.push_back(x);
        y_coords.push_back(y);

        if (has_z) {
            float z = get_param(prefix + "z", Data(0.0)).as_number();
            z_coords.push_back(z);
        }
    }

    if (x_coords.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter,
            "需要至少1个点来测量点云");
    }
    
    // 计算中心点
    float center_x = 0.0f, center_y = 0.0f, center_z = 0.0f;
    for (size_t i = 0; i < x_coords.size(); ++i) {
        center_x += x_coords[i];
        center_y += y_coords[i];
    }
    center_x /= x_coords.size();
    center_y /= y_coords.size();
    
    if (has_z && !z_coords.empty()) {
        for (float z : z_coords) {
            center_z += z;
        }
        center_z /= z_coords.size();
    }
    
    // 计算分散度（标准差）
    float spread_x = 0.0f, spread_y = 0.0f, spread_z = 0.0f;
    for (size_t i = 0; i < x_coords.size(); ++i) {
        spread_x += std::pow(x_coords[i] - center_x, 2);
        spread_y += std::pow(y_coords[i] - center_y, 2);
    }
    spread_x = std::sqrt(spread_x / x_coords.size());
    spread_y = std::sqrt(spread_y / y_coords.size());
    
    if (has_z && !z_coords.empty()) {
        for (size_t i = 0; i < z_coords.size(); ++i) {
            spread_z += std::pow(z_coords[i] - center_z, 2);
        }
        spread_z = std::sqrt(spread_z / z_coords.size());
    }
    
    // 总体分散度
    float total_spread = std::sqrt(spread_x * spread_x + spread_y * spread_y + spread_z * spread_z);
    
    // 边界范围
    float min_x = *std::min_element(x_coords.begin(), x_coords.end());
    float max_x = *std::max_element(x_coords.begin(), x_coords.end());
    float min_y = *std::min_element(y_coords.begin(), y_coords.end());
    float max_y = *std::max_element(y_coords.begin(), y_coords.end());
    
    float bounding_width = max_x - min_x;
    float bounding_height = max_y - min_y;
    float bounding_depth = 0.0f;
    
    if (has_z && !z_coords.empty()) {
        float min_z = *std::min_element(z_coords.begin(), z_coords.end());
        float max_z = *std::max_element(z_coords.begin(), z_coords.end());
        bounding_depth = max_z - min_z;
    }
    
    // 计算密度（点数/体积）
    float volume = bounding_width * bounding_height;
    if (has_z) {
        volume *= bounding_depth;
    }
    float density = (volume > 1e-6f) ? static_cast<float>(x_coords.size()) / volume : 0.0f;
    
    set_output("center_x", Data(static_cast<double>(center_x)));
    set_output("center_y", Data(static_cast<double>(center_y)));
    set_output("center_z", Data(static_cast<double>(center_z)));
    set_output("point_count", Data(static_cast<int>(x_coords.size())));
    set_output("spread_x", Data(static_cast<double>(spread_x)));
    set_output("spread_y", Data(static_cast<double>(spread_y)));
    set_output("spread_z", Data(static_cast<double>(spread_z)));
    set_output("total_spread", Data(static_cast<double>(total_spread)));
    set_output("density", Data(static_cast<double>(density)));
    set_output("bounding_width", Data(static_cast<double>(bounding_width)));
    set_output("bounding_height", Data(static_cast<double>(bounding_height)));
    set_output("bounding_depth", Data(static_cast<double>(bounding_depth)));
    
    OVF_INFO() << "点云拟合测量完成: 点数=" << x_coords.size() << ", 中心=("
               << center_x << "," << center_y << ")";
    if (has_z) {
        OVF_INFO() << "," << center_z;
    }
    OVF_INFO() << ", 总分散度=" << total_spread;
    
    return Result<void>::success();
}

// 注册所有节点
OVF_REGISTER_NODE(AreaMeasureNode, "AreaMeasure", AreaMeasureNode::make_info());
OVF_REGISTER_NODE(PerimeterMeasureNode, "PerimeterMeasure", PerimeterMeasureNode::make_info());
OVF_REGISTER_NODE(CenterMeasureNode, "CenterMeasure", CenterMeasureNode::make_info());
OVF_REGISTER_NODE(CircleMeasureNode, "CircleMeasure", CircleMeasureNode::make_info());
OVF_REGISTER_NODE(LineMeasureNode, "LineMeasure", LineMeasureNode::make_info());
OVF_REGISTER_NODE(RectangleMeasureNode, "RectangleMeasure", RectangleMeasureNode::make_info());
OVF_REGISTER_NODE(EllipseMeasureNode, "EllipseMeasure", EllipseMeasureNode::make_info());
OVF_REGISTER_NODE(PolygonMeasureNode, "PolygonMeasure", PolygonMeasureNode::make_info());
OVF_REGISTER_NODE(CircleFitMeasureNode, "CircleFitMeasure", CircleFitMeasureNode::make_info());
OVF_REGISTER_NODE(LineFitMeasureNode, "LineFitMeasure", LineFitMeasureNode::make_info());
OVF_REGISTER_NODE(EllipseFitMeasureNode, "EllipseFitMeasure", EllipseFitMeasureNode::make_info());
OVF_REGISTER_NODE(ContourFitMeasureNode, "ContourFitMeasure", ContourFitMeasureNode::make_info());
OVF_REGISTER_NODE(PointCloudFitMeasureNode, "PointCloudFitMeasure", PointCloudFitMeasureNode::make_info());

} // namespace algorithm
} // namespace ovf