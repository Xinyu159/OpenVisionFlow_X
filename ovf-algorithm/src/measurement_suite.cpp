/**
 * @file measurement_suite.cpp
 * @brief 测量套件实现
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "ovf/algorithm/measurement_suite.h"
#include "ovf/core/logger.h"
#include "ovf/core/error.h"
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace ovf {
namespace algorithm {

// ==================== 辅助函数实现 ====================

Vector<double> EdgeDetector::gaussian_smooth(const Vector<double>& data, double sigma) {
    if (data.empty() || sigma <= 0.0) return data;

    int size = static_cast<int>(data.size());
    int kernel_size = static_cast<int>(std::ceil(sigma * 6)) | 1; // 确保奇数
    int half = kernel_size / 2;

    Vector<double> kernel(kernel_size);
    double sum = 0.0;

    // 生成高斯核
    for (int i = 0; i < kernel_size; ++i) {
        double x = i - half;
        kernel[i] = std::exp(-(x * x) / (2 * sigma * sigma));
        sum += kernel[i];
    }

    // 归一化
    for (int i = 0; i < kernel_size; ++i) {
        kernel[i] /= sum;
    }

    // 卷积
    Vector<double> result(size, 0.0);
    for (int i = 0; i < size; ++i) {
        double val = 0.0;
        for (int j = 0; j < kernel_size; ++j) {
            int idx = i + j - half;
            if (idx >= 0 && idx < size) {
                val += data[idx] * kernel[j];
            }
        }
        result[i] = val;
    }

    return result;
}

Vector<double> EdgeDetector::compute_gradient(const Vector<double>& data) {
    if (data.size() < 2) return Vector<double>(data.size(), 0.0);

    int size = static_cast<int>(data.size());
    Vector<double> gradient(size);

    // Sobel算子梯度
    for (int i = 0; i < size; ++i) {
        if (i == 0) {
            gradient[i] = data[1] - data[0];
        } else if (i == size - 1) {
            gradient[i] = data[size - 1] - data[size - 2];
        } else {
            gradient[i] = (data[i + 1] - data[i - 1]) / 2.0;
        }
    }

    return gradient;
}

double EdgeDetector::subpixel_position(const Vector<double>& gradient, int index) {
    if (index <= 0 || index >= static_cast<int>(gradient.size()) - 1) {
        return static_cast<double>(index);
    }

    // 抛物线拟合亚像素定位
    double g0 = gradient[index - 1];
    double g1 = gradient[index];
    double g2 = gradient[index + 1];

    // 极值点位置
    double denom = 2.0 * (g0 - 2.0 * g1 + g2);
    if (std::abs(denom) < 1e-10) {
        return static_cast<double>(index);
    }

    double offset = (g0 - g2) / denom;
    return static_cast<double>(index) + offset;
}

Vector<EdgePoint> EdgeDetector::detect_edges_1d(
    const uint8_t* profile,
    int length,
    double threshold,
    double sigma,
    TransitionType transition) {

    Vector<EdgePoint> edges;

    if (length < 3) return edges;

    // 转换为double
    Vector<double> data(length);
    for (int i = 0; i < length; ++i) {
        data[i] = static_cast<double>(profile[i]);
    }

    // 高斯平滑
    Vector<double> smoothed = gaussian_smooth(data, sigma);

    // 计算梯度
    Vector<double> gradient = compute_gradient(smoothed);

    // 检测边缘
    for (int i = 1; i < length - 1; ++i) {
        double grad = gradient[i];
        bool is_edge = false;

        // 检查是否为局部极值
        if (std::abs(grad) > threshold) {
            if ((grad > 0 && grad > gradient[i - 1] && grad > gradient[i + 1]) ||
                (grad < 0 && grad < gradient[i - 1] && grad < gradient[i + 1])) {
                is_edge = true;
            }
        }

        if (is_edge) {
            // 检查边缘类型
            int trans_type = (grad > 0) ? 1 : -1;

            bool match_transition = false;
            switch (transition) {
                case TransitionType::Positive:
                    match_transition = (trans_type == 1);
                    break;
                case TransitionType::Negative:
                    match_transition = (trans_type == -1);
                    break;
                case TransitionType::All:
                case TransitionType::Strongest:
                    match_transition = true;
                    break;
            }

            if (match_transition) {
                // 亚像素定位
                double pos = subpixel_position(gradient, i);

                EdgePoint edge;
                edge.row = pos; // 在1D情况下，使用row表示位置
                edge.col = 0;
                edge.amplitude = std::abs(grad);
                edge.distance = pos;
                edge.transition = trans_type;

                edges.push_back(edge);
            }
        }
    }

    // 如果只需要最强边缘
    if (transition == TransitionType::Strongest && !edges.empty()) {
        EdgePoint strongest = edges[0];
        for (const auto& e : edges) {
            if (e.amplitude > strongest.amplitude) {
                strongest = e;
            }
        }
        edges.clear();
        edges.push_back(strongest);
    }

    return edges;
}

// ==================== 几何拟合实现 ====================

double GeometryFitter::distance(double r1, double c1, double r2, double c2) {
    double dr = r2 - r1;
    double dc = c2 - c1;
    return std::sqrt(dr * dr + dc * dc);
}

double GeometryFitter::angle_between_points(double r1, double c1, double r2, double c2) {
    return std::atan2(r2 - r1, c2 - c1);
}

double GeometryFitter::point_to_line_distance(
    double row, double col,
    double line_row, double line_col, double line_angle) {

    // 直线方向向量
    double dx = std::cos(line_angle);
    double dy = std::sin(line_angle);

    // 点到直线的向量
    double px = col - line_col;
    double py = row - line_row;

    // 叉积的绝对值除以直线方向向量的模
    double cross = std::abs(px * dy - py * dx);
    return cross;
}

bool GeometryFitter::fit_line(
    const Vector<EdgePoint>& points,
    double& center_row,
    double& center_col,
    double& angle,
    double& error) {

    if (points.size() < 2) {
        error = 1e10;
        return false;
    }

    // 计算质心
    double sum_row = 0.0, sum_col = 0.0;
    for (const auto& p : points) {
        sum_row += p.row;
        sum_col += p.col;
    }
    center_row = sum_row / points.size();
    center_col = sum_col / points.size();

    // PCA拟合直线方向
    double c00 = 0.0, c01 = 0.0, c11 = 0.0;
    for (const auto& p : points) {
        double dr = p.row - center_row;
        double dc = p.col - center_col;
        c00 += dr * dr;
        c01 += dr * dc;
        c11 += dc * dc;
    }

    // 计算特征值和特征向量
    double trace = c00 + c11;
    double det = c00 * c11 - c01 * c01;
    double delta = std::sqrt(trace * trace / 4.0 - det);

    double lambda1 = trace / 2.0 + delta;
    double lambda2 = trace / 2.0 - delta;

    angle = std::atan2(c01, lambda1 - c11);

    // 计算拟合误差
    error = 0.0;
    for (const auto& p : points) {
        double dist = point_to_line_distance(p.row, p.col, center_row, center_col, angle);
        error += dist * dist;
    }
    error = std::sqrt(error / points.size());

    return true;
}

bool GeometryFitter::fit_circle(
    const Vector<EdgePoint>& points,
    double& center_row,
    double& center_col,
    double& radius,
    double& error) {

    if (points.size() < 3) {
        error = 1e10;
        return false;
    }

    // 使用最小二乘法拟合圆
    // (r - rc)^2 + (c - cc)^2 = R^2
    // 展开: r^2 + c^2 - 2*r*rc - 2*c*cc + rc^2 + cc^2 - R^2 = 0
    // 令 A = -2*rc, B = -2*cc, C = rc^2 + cc^2 - R^2
    // 则: r^2 + c^2 + A*r + B*c + C = 0
    // 构建线性方程组求解 A, B, C

    double sum_r = 0.0, sum_c = 0.0;
    double sum_r2 = 0.0, sum_c2 = 0.0;
    double sum_r3 = 0.0, sum_c3 = 0.0;
    double sum_rc = 0.0, sum_r2c = 0.0, sum_rc2 = 0.0;

    for (const auto& p : points) {
        double r = p.row;
        double c = p.col;
        sum_r += r;
        sum_c += c;
        sum_r2 += r * r;
        sum_c2 += c * c;
        sum_r3 += r * r * r;
        sum_c3 += c * c * c;
        sum_rc += r * c;
        sum_r2c += r * r * c;
        sum_rc2 += r * c * c;
    }

    int n = static_cast<int>(points.size());
    double A = n * sum_r2 - sum_r * sum_r;
    double B = n * sum_rc - sum_r * sum_c;
    double C = n * sum_c2 - sum_c * sum_c;

    double D = n * sum_r2c - sum_r * sum_rc;
    double E = n * sum_rc2 - sum_c * sum_rc;

    double denom = A * C - B * B;
    if (std::abs(denom) < 1e-10) {
        error = 1e10;
        return false;
    }

    double a = (D * C - B * E) / denom;
    double b = (A * E - B * D) / denom;

    center_row = a / 2.0;
    center_col = b / 2.0;

    // 计算半径
    double sum_dist = 0.0;
    for (const auto& p : points) {
        double d = distance(p.row, p.col, center_row, center_col);
        sum_dist += d;
    }
    radius = sum_dist / points.size();

    // 计算拟合误差
    error = 0.0;
    for (const auto& p : points) {
        double d = distance(p.row, p.col, center_row, center_col);
        error += (d - radius) * (d - radius);
    }
    error = std::sqrt(error / points.size());

    return true;
}

bool GeometryFitter::fit_arc(
    const Vector<EdgePoint>& points,
    double& center_row,
    double& center_col,
    double& radius,
    double& start_angle,
    double& end_angle,
    double& error) {

    // 先拟合圆
    if (!fit_circle(points, center_row, center_col, radius, error)) {
        return false;
    }

    // 计算起始和终止角度
    if (points.empty()) {
        return false;
    }

    start_angle = angle_between_points(center_row, center_col, points[0].row, points[0].col);
    end_angle = angle_between_points(center_row, center_col, points.back().row, points.back().col);

    return true;
}

// ==================== 测量模型创建节点 ====================

MeasureModelCreateNode::MeasureModelCreateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureModelCreateNode::make_info() {
    NodeInfo info;
    info.id = "MeasureModelCreate";
    info.name = "创建测量模型";
    info.category = "测量";
    info.description = "创建空的测量模型";
    info.version = "0.1.0";

    info.outputs.push_back(DataPort("model", "测量模型", DataType::Object, false));

    info.params.push_back(ParamDef("model_id", "模型ID", DataType::String, Data("measure_model")));
    info.params.push_back(ParamDef("model_name", "模型名称", DataType::String, Data("测量模型")));
    info.params.push_back(ParamDef("pixel_size", "像素尺寸", DataType::Number, Data(1.0)));

    return info;
}

Result<void> MeasureModelCreateNode::execute(FlowContext& context) {
    String model_id = get_param("model_id", Data("measure_model")).as_string();
    String model_name = get_param("model_name", Data("测量模型")).as_string();
    double pixel_size = get_param("pixel_size", Data(1.0)).as_number();

    MeasureModel model;
    model.id = model_id;
    model.name = model_name;
    model.pixel_size = pixel_size;
    model.calibrated = (pixel_size != 1.0);

    set_output("model", Data("MeasureModel[...]"));

    OVF_INFO() << "Measure model created: " << model_id;

    return Result<void>::success();
}

MeasureModelAddLineNode::MeasureModelAddLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureModelAddLineNode::make_info() {
    NodeInfo info;
    info.id = "MeasureModelAddLine";
    info.name = "添加直线测量";
    info.category = "测量";
    info.description = "向测量模型添加直线测量对象";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "测量模型", DataType::Object, true));
    info.outputs.push_back(DataPort("model", "测量模型", DataType::Object, false));

    info.params.push_back(ParamDef("object_id", "对象ID", DataType::String, Data("line1")));
    info.params.push_back(ParamDef("center_row", "中心行", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("center_col", "中心列", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("length", "长度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("angle", "角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("transition", "边缘类型", DataType::String, Data("all")));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("measure_type", "测量类型", DataType::String, Data("position")));
    info.params.push_back(ParamDef("nominal", "标称值", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("tolerance_min", "最小公差", DataType::Number, Data(-1.0)));
    info.params.push_back(ParamDef("tolerance_max", "最大公差", DataType::Number, Data(1.0)));

    return info;
}

Result<void> MeasureModelAddLineNode::execute(FlowContext& context) {
    auto input = get_input("model");

    // 从输入或创建新模型
    MeasureModel model;
    // 注意：这里简化处理，实际应该从Data中解析

    MeasureObject obj;
    obj.id = get_param("object_id", Data("line1")).as_string();
    obj.type = MeasureObjectType::Line;
    obj.center_row = get_param("center_row", Data(100.0)).as_number();
    obj.center_col = get_param("center_col", Data(100.0)).as_number();
    obj.length1 = get_param("length", Data(50.0)).as_number();
    obj.angle = get_param("angle", Data(0.0)).as_number() * M_PI / 180.0;

    String trans_str = get_param("transition", Data("all")).as_string();
    if (trans_str == "positive") obj.transition = TransitionType::Positive;
    else if (trans_str == "negative") obj.transition = TransitionType::Negative;
    else if (trans_str == "all") obj.transition = TransitionType::All;
    else obj.transition = TransitionType::Strongest;

    obj.threshold = get_param("threshold", Data(30.0)).as_number();
    obj.sigma = get_param("sigma", Data(1.0)).as_number();

    String measure_str = get_param("measure_type", Data("position")).as_string();
    if (measure_str == "position") obj.measure_type = MeasureType::Position;
    else if (measure_str == "distance") obj.measure_type = MeasureType::Distance;
    else if (measure_str == "angle") obj.measure_type = MeasureType::Angle;

    obj.nominal_value = get_param("nominal", Data(0.0)).as_number();
    obj.tolerance_min = get_param("tolerance_min", Data(-1.0)).as_number();
    obj.tolerance_max = get_param("tolerance_max", Data(1.0)).as_number();

    model.add_object(obj);
    set_output("model", Data("MeasureModel[...]"));

    OVF_INFO() << "Line measure object added: " << obj.id;

    return Result<void>::success();
}

MeasureModelAddCircleNode::MeasureModelAddCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureModelAddCircleNode::make_info() {
    NodeInfo info;
    info.id = "MeasureModelAddCircle";
    info.name = "添加圆测量";
    info.category = "测量";
    info.description = "向测量模型添加圆测量对象";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "测量模型", DataType::Object, true));
    info.outputs.push_back(DataPort("model", "测量模型", DataType::Object, false));

    info.params.push_back(ParamDef("object_id", "对象ID", DataType::String, Data("circle1")));
    info.params.push_back(ParamDef("center_row", "中心行", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("center_col", "中心列", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("transition", "边缘类型", DataType::String, Data("all")));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("num_points", "测量点数", DataType::Number, Data(36)));
    info.params.push_back(ParamDef("measure_type", "测量类型", DataType::String, Data("radius")));
    info.params.push_back(ParamDef("nominal", "标称值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("tolerance_min", "最小公差", DataType::Number, Data(-0.5)));
    info.params.push_back(ParamDef("tolerance_max", "最大公差", DataType::Number, Data(0.5)));

    return info;
}

Result<void> MeasureModelAddCircleNode::execute(FlowContext& context) {
    MeasureModel model;

    MeasureObject obj;
    obj.id = get_param("object_id", Data("circle1")).as_string();
    obj.type = MeasureObjectType::Circle;
    obj.center_row = get_param("center_row", Data(100.0)).as_number();
    obj.center_col = get_param("center_col", Data(100.0)).as_number();
    obj.radius = get_param("radius", Data(30.0)).as_number();

    String trans_str = get_param("transition", Data("all")).as_string();
    if (trans_str == "positive") obj.transition = TransitionType::Positive;
    else if (trans_str == "negative") obj.transition = TransitionType::Negative;
    else if (trans_str == "all") obj.transition = TransitionType::All;
    else obj.transition = TransitionType::Strongest;

    obj.threshold = get_param("threshold", Data(30.0)).as_number();
    obj.sigma = get_param("sigma", Data(1.0)).as_number();
    obj.num_points = get_param("num_points", Data(36)).as_int();

    String measure_str = get_param("measure_type", Data("radius")).as_string();
    if (measure_str == "radius") obj.measure_type = MeasureType::Radius;
    else if (measure_str == "diameter") obj.measure_type = MeasureType::Diameter;
    else if (measure_str == "position") obj.measure_type = MeasureType::Position;

    obj.nominal_value = get_param("nominal", Data(30.0)).as_number();
    obj.tolerance_min = get_param("tolerance_min", Data(-0.5)).as_number();
    obj.tolerance_max = get_param("tolerance_max", Data(0.5)).as_number();

    model.add_object(obj);
    set_output("model", Data("MeasureModel[...]"));

    OVF_INFO() << "Circle measure object added: " << obj.id;

    return Result<void>::success();
}

MeasureModelAddRectangleNode::MeasureModelAddRectangleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureModelAddRectangleNode::make_info() {
    NodeInfo info;
    info.id = "MeasureModelAddRectangle";
    info.name = "添加矩形测量";
    info.category = "测量";
    info.description = "向测量模型添加矩形测量对象";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "测量模型", DataType::Object, true));
    info.outputs.push_back(DataPort("model", "测量模型", DataType::Object, false));

    info.params.push_back(ParamDef("object_id", "对象ID", DataType::String, Data("rect1")));
    info.params.push_back(ParamDef("center_row", "中心行", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("center_col", "中心列", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("length1", "长度1", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("length2", "长度2", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("angle", "角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("transition", "边缘类型", DataType::String, Data("all")));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("measure_type", "测量类型", DataType::String, Data("distance")));
    info.params.push_back(ParamDef("nominal", "标称值", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("tolerance_min", "最小公差", DataType::Number, Data(-1.0)));
    info.params.push_back(ParamDef("tolerance_max", "最大公差", DataType::Number, Data(1.0)));

    return info;
}

Result<void> MeasureModelAddRectangleNode::execute(FlowContext& context) {
    MeasureModel model;

    MeasureObject obj;
    obj.id = get_param("object_id", Data("rect1")).as_string();
    obj.type = MeasureObjectType::Rectangle;
    obj.center_row = get_param("center_row", Data(100.0)).as_number();
    obj.center_col = get_param("center_col", Data(100.0)).as_number();
    obj.length1 = get_param("length1", Data(50.0)).as_number();
    obj.length2 = get_param("length2", Data(30.0)).as_number();
    obj.angle = get_param("angle", Data(0.0)).as_number() * M_PI / 180.0;

    String trans_str = get_param("transition", Data("all")).as_string();
    if (trans_str == "positive") obj.transition = TransitionType::Positive;
    else if (trans_str == "negative") obj.transition = TransitionType::Negative;
    else if (trans_str == "all") obj.transition = TransitionType::All;
    else obj.transition = TransitionType::Strongest;

    obj.threshold = get_param("threshold", Data(30.0)).as_number();

    String measure_str = get_param("measure_type", Data("distance")).as_string();
    if (measure_str == "distance") obj.measure_type = MeasureType::Distance;
    else if (measure_str == "position") obj.measure_type = MeasureType::Position;

    obj.nominal_value = get_param("nominal", Data(50.0)).as_number();
    obj.tolerance_min = get_param("tolerance_min", Data(-1.0)).as_number();
    obj.tolerance_max = get_param("tolerance_max", Data(1.0)).as_number();

    model.add_object(obj);
    set_output("model", Data("MeasureModel[...]"));

    OVF_INFO() << "Rectangle measure object added: " << obj.id;

    return Result<void>::success();
}

MeasureModelAddArcNode::MeasureModelAddArcNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureModelAddArcNode::make_info() {
    NodeInfo info;
    info.id = "MeasureModelAddArc";
    info.name = "添加弧测量";
    info.category = "测量";
    info.description = "向测量模型添加弧测量对象";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "测量模型", DataType::Object, true));
    info.outputs.push_back(DataPort("model", "测量模型", DataType::Object, false));

    info.params.push_back(ParamDef("object_id", "对象ID", DataType::String, Data("arc1")));
    info.params.push_back(ParamDef("center_row", "中心行", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("center_col", "中心列", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("radius", "半径", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("start_angle", "起始角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_angle", "终止角度(度)", DataType::Number, Data(90.0)));
    info.params.push_back(ParamDef("transition", "边缘类型", DataType::String, Data("all")));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("measure_type", "测量类型", DataType::String, Data("radius")));
    info.params.push_back(ParamDef("nominal", "标称值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("tolerance_min", "最小公差", DataType::Number, Data(-0.5)));
    info.params.push_back(ParamDef("tolerance_max", "最大公差", DataType::Number, Data(0.5)));

    return info;
}

Result<void> MeasureModelAddArcNode::execute(FlowContext& context) {
    MeasureModel model;

    MeasureObject obj;
    obj.id = get_param("object_id", Data("arc1")).as_string();
    obj.type = MeasureObjectType::Arc;
    obj.center_row = get_param("center_row", Data(100.0)).as_number();
    obj.center_col = get_param("center_col", Data(100.0)).as_number();
    obj.radius = get_param("radius", Data(30.0)).as_number();
    obj.start_angle = get_param("start_angle", Data(0.0)).as_number() * M_PI / 180.0;
    obj.end_angle = get_param("end_angle", Data(90.0)).as_number() * M_PI / 180.0;

    String trans_str = get_param("transition", Data("all")).as_string();
    if (trans_str == "positive") obj.transition = TransitionType::Positive;
    else if (trans_str == "negative") obj.transition = TransitionType::Negative;
    else if (trans_str == "all") obj.transition = TransitionType::All;
    else obj.transition = TransitionType::Strongest;

    obj.threshold = get_param("threshold", Data(30.0)).as_number();
    obj.sigma = get_param("sigma", Data(1.0)).as_number();

    String measure_str = get_param("measure_type", Data("radius")).as_string();
    if (measure_str == "radius") obj.measure_type = MeasureType::Radius;
    else if (measure_str == "angle") obj.measure_type = MeasureType::Angle;

    obj.nominal_value = get_param("nominal", Data(30.0)).as_number();
    obj.tolerance_min = get_param("tolerance_min", Data(-0.5)).as_number();
    obj.tolerance_max = get_param("tolerance_max", Data(0.5)).as_number();

    model.add_object(obj);
    set_output("model", Data("MeasureModel[...]"));

    OVF_INFO() << "Arc measure object added: " << obj.id;

    return Result<void>::success();
}

// ==================== 测量执行节点 ====================

MeasureModelApplyNode::MeasureModelApplyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureModelApplyNode::make_info() {
    NodeInfo info;
    info.id = "MeasureModelApply";
    info.name = "应用测量模型";
    info.category = "测量";
    info.description = "在图像上应用测量模型，执行所有测量对象";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("model", "测量模型", DataType::Object, true));
    info.outputs.push_back(DataPort("results", "测量结果", DataType::Array, false));

    info.params.push_back(ParamDef("pixel_size", "像素尺寸", DataType::Number, Data(1.0)));

    return info;
}

Vector<double> MeasureModelApplyNode::extract_profile(
    const ImageData& image,
    const MeasureObject& obj) {

    Vector<double> profile;

    // 简化实现：沿测量线提取灰度值
    // 实际应该根据测量对象类型提取不同方向的轮廓

    if (image.empty()) return profile;

    int length = static_cast<int>(obj.length1);
    profile.resize(length);

    double angle = obj.angle;
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    for (int i = 0; i < length; ++i) {
        int row = static_cast<int>(obj.center_row + dy * i);
        int col = static_cast<int>(obj.center_col + dx * i);

        if (row >= 0 && row < static_cast<int>(image.height) &&
            col >= 0 && col < static_cast<int>(image.width)) {
            size_t idx = row * image.width * image.channels + col * image.channels;
            if (image.channels == 1) {
                profile[i] = static_cast<double>(image.data[idx]);
            } else {
                // 转换为灰度
                double gray = 0.299 * image.data[idx + 2] +
                              0.587 * image.data[idx + 1] +
                              0.114 * image.data[idx];
                profile[i] = gray;
            }
        } else {
            profile[i] = 0.0;
        }
    }

    return profile;
}

Vector<EdgePoint> MeasureModelApplyNode::detect_edges_in_profile(
    const Vector<double>& profile,
    const MeasureObject& obj) {

    Vector<EdgePoint> edges;

    if (profile.empty()) return edges;

    Vector<uint8_t> profile_u8(profile.size());
    for (size_t i = 0; i < profile.size(); ++i) {
        profile_u8[i] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, profile[i])));
    }

    edges = EdgeDetector::detect_edges_1d(
        profile_u8.data(),
        static_cast<int>(profile_u8.size()),
        obj.threshold,
        obj.sigma,
        obj.transition
    );

    return edges;
}

Result<void> MeasureModelApplyNode::measure_object(
    const ImageData& image,
    const MeasureObject& obj,
    MeasureResult& result) {

    result.object_id = obj.id;
    result.status = MeasureStatus::OK;

    // 提取轮廓
    Vector<double> profile = extract_profile(image, obj);

    // 检测边缘
    Vector<EdgePoint> edges = detect_edges_in_profile(profile, obj);
    result.edge_points = edges;
    result.num_edges = static_cast<int>(edges.size());

    if (edges.empty()) {
        result.status = MeasureStatus::Error;
        return Result<void>::success();
    }

    // 根据选择策略选择边缘
    EdgePoint selected_edge;
    switch (obj.select) {
        case SelectStrategy::First:
            selected_edge = edges.front();
            break;
        case SelectStrategy::Last:
            selected_edge = edges.back();
            break;
        case SelectStrategy::Strongest:
            selected_edge = edges[0];
            for (const auto& e : edges) {
                if (e.amplitude > selected_edge.amplitude) {
                    selected_edge = e;
                }
            }
            break;
        default:
            selected_edge = edges.front();
            break;
    }

    // 根据测量类型计算测量值
    switch (obj.measure_type) {
        case MeasureType::Position:
            result.value = selected_edge.distance;
            result.first_edge_pos = selected_edge.row;
            break;

        case MeasureType::Distance:
            if (edges.size() >= 2) {
                result.edge_distance = std::abs(edges.back().distance - edges.front().distance);
                result.value = result.edge_distance;
            }
            break;

        case MeasureType::Radius:
        case MeasureType::Diameter:
            if (edges.size() >= 3) {
                double center_row, center_col, radius, error;
                if (GeometryFitter::fit_circle(edges, center_row, center_col, radius, error)) {
                    result.fit_row = center_row;
                    result.fit_col = center_col;
                    result.fit_radius = radius;
                    result.fit_error = error;
                    result.value = (obj.measure_type == MeasureType::Diameter) ? radius * 2 : radius;
                }
            }
            break;

        default:
            break;
    }

    // 计算误差
    result.error = result.value - obj.nominal_value;

    // 判断是否合格
    if (obj.tolerance_min != 0.0 || obj.tolerance_max != 0.0) {
        if (result.value < obj.nominal_value + obj.tolerance_min ||
            result.value > obj.nominal_value + obj.tolerance_max) {
            result.status = MeasureStatus::NG;
        }
    }

    return Result<void>::success();
}

Result<void> MeasureModelApplyNode::execute(FlowContext& context) {
    auto image_data = get_input("image");
    auto model_data = get_input("model");

    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData image = image_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取测量模型
    MeasureModel model;
    // 注意：实际应从Data中解析

    // 执行测量
    Vector<MeasureResult> results;
    for (const auto& obj : model.objects) {
        MeasureResult result;
        measure_object(image, obj, result);
        results.push_back(result);
    }

    set_output("results", Data("MeasureResult[...]"));

    OVF_INFO() << "Measurement applied, " << results.size() << " objects measured";

    return Result<void>::success();
}

MeasurePosNode::MeasurePosNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasurePosNode::make_info() {
    NodeInfo info;
    info.id = "MeasurePos";
    info.name = "测量位置";
    info.category = "测量";
    info.description = "在指定位置进行边缘测量";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("positions", "边缘位置", DataType::Array, false));
    info.outputs.push_back(DataPort("amplitudes", "边缘幅度", DataType::Array, false));

    info.params.push_back(ParamDef("row", "行位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("col", "列位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("angle", "角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("length", "测量长度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("transition", "边缘类型", DataType::String, Data("all")));
    info.params.push_back(ParamDef("select", "选择策略", DataType::String, Data("all")));

    return info;
}

Result<void> MeasurePosNode::execute(FlowContext& context) {
    auto image_data = get_input("image");

    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData image = image_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    double row = get_param("row", Data(100.0)).as_number();
    double col = get_param("col", Data(100.0)).as_number();
    double angle = get_param("angle", Data(0.0)).as_number() * M_PI / 180.0;
    int length = get_param("length", Data(50)).as_int();
    double threshold = get_param("threshold", Data(30.0)).as_number();
    double sigma = get_param("sigma", Data(1.0)).as_number();

    String trans_str = get_param("transition", Data("all")).as_string();
    TransitionType transition = TransitionType::All;
    if (trans_str == "positive") transition = TransitionType::Positive;
    else if (trans_str == "negative") transition = TransitionType::Negative;
    else if (trans_str == "strongest") transition = TransitionType::Strongest;

    // 提取轮廓
    Vector<double> profile(length);
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    for (int i = 0; i < length; ++i) {
        int r = static_cast<int>(row + dy * (i - length / 2));
        int c = static_cast<int>(col + dx * (i - length / 2));

        if (r >= 0 && r < static_cast<int>(image.height) &&
            c >= 0 && c < static_cast<int>(image.width)) {
            size_t idx = r * image.width * image.channels + c * image.channels;
            profile[i] = (image.channels == 1) ?
                static_cast<double>(image.data[idx]) :
                0.299 * image.data[idx + 2] + 0.587 * image.data[idx + 1] + 0.114 * image.data[idx];
        }
    }

    // 检测边缘
    Vector<uint8_t> profile_u8(length);
    for (int i = 0; i < length; ++i) {
        profile_u8[i] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, profile[i])));
    }

    Vector<EdgePoint> edges = EdgeDetector::detect_edges_1d(
        profile_u8.data(), length, threshold, sigma, transition);

    // 输出结果
    Vector<double> positions;
    Vector<double> amplitudes;

    for (const auto& e : edges) {
        positions.push_back(e.row);
        amplitudes.push_back(e.amplitude);
    }

    set_output("positions", Data("positions[...]"));
    set_output("amplitudes", Data("amplitudes[...]"));

    OVF_INFO() << "Edge positions measured: " << edges.size() << " edges found";

    return Result<void>::success();
}

MeasureThresholdNode::MeasureThresholdNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureThresholdNode::make_info() {
    NodeInfo info;
    info.id = "MeasureThreshold";
    info.name = "测量阈值";
    info.category = "测量";
    info.description = "使用阈值方法测量边缘位置";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("positions", "边缘位置", DataType::Array, false));

    info.params.push_back(ParamDef("row", "行位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("col", "列位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("angle", "角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("length", "测量长度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("threshold", "灰度阈值", DataType::Number, Data(128.0)));

    return info;
}

Result<void> MeasureThresholdNode::execute(FlowContext& context) {
    auto image_data = get_input("image");

    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData image = image_data.as_image();

    double row = get_param("row", Data(100.0)).as_number();
    double col = get_param("col", Data(100.0)).as_number();
    double angle = get_param("angle", Data(0.0)).as_number() * M_PI / 180.0;
    int length = get_param("length", Data(50)).as_int();
    double threshold = get_param("threshold", Data(128.0)).as_number();

    // 提取轮廓
    Vector<double> profile(length);
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    for (int i = 0; i < length; ++i) {
        int r = static_cast<int>(row + dy * (i - length / 2));
        int c = static_cast<int>(col + dx * (i - length / 2));

        if (r >= 0 && r < static_cast<int>(image.height) &&
            c >= 0 && c < static_cast<int>(image.width)) {
            size_t idx = r * image.width * image.channels + c * image.channels;
            profile[i] = (image.channels == 1) ?
                static_cast<double>(image.data[idx]) :
                0.299 * image.data[idx + 2] + 0.587 * image.data[idx + 1] + 0.114 * image.data[idx];
        }
    }

    // 使用阈值找边缘
    Vector<double> positions;
    for (int i = 1; i < length; ++i) {
        if ((profile[i - 1] < threshold && profile[i] >= threshold) ||
            (profile[i - 1] >= threshold && profile[i] < threshold)) {
            // 线性插值亚像素定位
            double pos = i - (profile[i] - threshold) / (profile[i] - profile[i - 1]);
            positions.push_back(pos);
        }
    }

    set_output("positions", Data("positions[...]"));

    OVF_INFO() << "Threshold measurement: " << positions.size() << " edges found";

    return Result<void>::success();
}

MeasureProfileNode::MeasureProfileNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureProfileNode::make_info() {
    NodeInfo info;
    info.id = "MeasureProfile";
    info.name = "测量轮廓";
    info.category = "测量";
    info.description = "提取图像的灰度轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("profile", "灰度轮廓", DataType::Array, false));

    info.params.push_back(ParamDef("row", "行位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("col", "列位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("angle", "角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("length", "测量长度", DataType::Number, Data(50.0)));

    return info;
}

Result<void> MeasureProfileNode::execute(FlowContext& context) {
    auto image_data = get_input("image");

    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData image = image_data.as_image();

    double row = get_param("row", Data(100.0)).as_number();
    double col = get_param("col", Data(100.0)).as_number();
    double angle = get_param("angle", Data(0.0)).as_number() * M_PI / 180.0;
    int length = get_param("length", Data(50)).as_int();

    // 提取轮廓
    Vector<double> profile(length);
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    for (int i = 0; i < length; ++i) {
        int r = static_cast<int>(row + dy * (i - length / 2));
        int c = static_cast<int>(col + dx * (i - length / 2));

        if (r >= 0 && r < static_cast<int>(image.height) &&
            c >= 0 && c < static_cast<int>(image.width)) {
            size_t idx = r * image.width * image.channels + c * image.channels;
            profile[i] = (image.channels == 1) ?
                static_cast<double>(image.data[idx]) :
                0.299 * image.data[idx + 2] + 0.587 * image.data[idx + 1] + 0.114 * image.data[idx];
        }
    }

    set_output("profile", Data("profile[...]"));

    OVF_INFO() << "Profile extracted: " << length << " points";

    return Result<void>::success();
}

MeasureFuzzyNode::MeasureFuzzyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureFuzzyNode::make_info() {
    NodeInfo info;
    info.id = "MeasureFuzzy";
    info.name = "模糊测量";
    info.category = "测量";
    info.description = "使用模糊逻辑进行边缘测量";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("positions", "边缘位置", DataType::Array, false));
    info.outputs.push_back(DataPort("scores", "模糊得分", DataType::Array, false));

    info.params.push_back(ParamDef("row", "行位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("col", "列位置", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("angle", "角度(度)", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("length", "测量长度", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("fuzzy_threshold", "模糊阈值", DataType::Number, Data(0.5)));

    return info;
}

Result<void> MeasureFuzzyNode::execute(FlowContext& context) {
    auto image_data = get_input("image");

    if (!image_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData image = image_data.as_image();

    double row = get_param("row", Data(100.0)).as_number();
    double col = get_param("col", Data(100.0)).as_number();
    double angle = get_param("angle", Data(0.0)).as_number() * M_PI / 180.0;
    int length = get_param("length", Data(50)).as_int();
    double fuzzy_threshold = get_param("fuzzy_threshold", Data(0.5)).as_number();

    // 提取轮廓
    Vector<double> profile(length);
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    for (int i = 0; i < length; ++i) {
        int r = static_cast<int>(row + dy * (i - length / 2));
        int c = static_cast<int>(col + dx * (i - length / 2));

        if (r >= 0 && r < static_cast<int>(image.height) &&
            c >= 0 && c < static_cast<int>(image.width)) {
            size_t idx = r * image.width * image.channels + c * image.channels;
            profile[i] = (image.channels == 1) ?
                static_cast<double>(image.data[idx]) :
                0.299 * image.data[idx + 2] + 0.587 * image.data[idx + 1] + 0.114 * image.data[idx];
        }
    }

    // 计算梯度作为模糊隶属度
    Vector<double> gradient(length - 1);
    for (int i = 0; i < length - 1; ++i) {
        gradient[i] = std::abs(profile[i + 1] - profile[i]);
    }

    // 归一化到[0,1]
    double max_grad = 0.0;
    for (double g : gradient) {
        max_grad = std::max(max_grad, g);
    }

    Vector<double> positions;
    Vector<double> scores;

    if (max_grad > 0) {
        for (int i = 0; i < length - 1; ++i) {
            double score = gradient[i] / max_grad;
            if (score >= fuzzy_threshold) {
                positions.push_back(static_cast<double>(i));
                scores.push_back(score);
            }
        }
    }

    set_output("positions", Data("positions[...]"));
    set_output("scores", Data("scores[...]"));

    OVF_INFO() << "Fuzzy measurement: " << positions.size() << " edges found";

    return Result<void>::success();
}

// ==================== 测量结果处理节点 ====================

MeasureResultsGetNode::MeasureResultsGetNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureResultsGetNode::make_info() {
    NodeInfo info;
    info.id = "MeasureResultsGet";
    info.name = "获取测量结果";
    info.category = "测量";
    info.description = "从测量结果中提取指定数据";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("results", "测量结果", DataType::Array, true));
    info.outputs.push_back(DataPort("values", "测量值", DataType::Array, false));
    info.outputs.push_back(DataPort("statuses", "状态", DataType::Array, false));
    info.outputs.push_back(DataPort("errors", "误差", DataType::Array, false));

    info.params.push_back(ParamDef("object_id", "对象ID", DataType::String, Data("")));

    return info;
}

Result<void> MeasureResultsGetNode::execute(FlowContext& context) {
    auto results_data = get_input("results");

    // 注意：实际应从Data中解析MeasureResult数组
    Vector<MeasureResult> results;

    String object_id = get_param("object_id", Data("")).as_string();

    Vector<double> values;
    Vector<int> statuses;
    Vector<double> errors;

    for (const auto& result : results) {
        if (object_id.empty() || result.object_id == object_id) {
            values.push_back(result.value);
            statuses.push_back(static_cast<int>(result.status));
            errors.push_back(result.error);
        }
    }

    set_output("values", Data("values[...]"));
    set_output("statuses", Data("statuses[...]"));
    set_output("errors", Data("errors[...]"));

    OVF_INFO() << "Measure results retrieved: " << values.size() << " results";

    return Result<void>::success();
}

MeasureResultsToRegionsNode::MeasureResultsToRegionsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureResultsToRegionsNode::make_info() {
    NodeInfo info;
    info.id = "MeasureResultsToRegions";
    info.name = "结果转区域";
    info.category = "测量";
    info.description = "将测量结果转换为可视化的区域";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("results", "测量结果", DataType::Array, true));
    info.outputs.push_back(DataPort("regions", "区域列表", DataType::Array, false));

    info.params.push_back(ParamDef("region_size", "区域大小", DataType::Number, Data(5.0)));

    return info;
}

Result<void> MeasureResultsToRegionsNode::execute(FlowContext& context) {
    auto results_data = get_input("results");

    // 注意：实际应从Data中解析MeasureResult数组
    Vector<MeasureResult> results;

    double region_size = get_param("region_size", Data(5.0)).as_number();

    Vector<Region> regions;

    for (const auto& result : results) {
        for (const auto& edge : result.edge_points) {
            Region r;
            r.x = static_cast<int32_t>(edge.col - region_size / 2);
            r.y = static_cast<int32_t>(edge.row - region_size / 2);
            r.width = static_cast<int32_t>(region_size);
            r.height = static_cast<int32_t>(region_size);
            regions.push_back(r);
        }
    }

    set_output("regions", Data("Region[...]"));

    OVF_INFO() << "Converted to regions: " << regions.size() << " regions";

    return Result<void>::success();
}

MeasureStatisticsNode::MeasureStatisticsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureStatisticsNode::make_info() {
    NodeInfo info;
    info.id = "MeasureStatistics";
    info.name = "测量统计";
    info.category = "测量";
    info.description = "计算测量结果的统计信息";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("results", "测量结果", DataType::Array, true));
    info.inputs.push_back(DataPort("history", "历史数据", DataType::Array, false));
    info.outputs.push_back(DataPort("statistics", "统计结果", DataType::Object, false));

    info.params.push_back(ParamDef("object_id", "对象ID", DataType::String, Data("")));
    info.params.push_back(ParamDef("lsl", "下规格限", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("usl", "上规格限", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("nominal", "标称值", DataType::Number, Data(0.0)));

    return info;
}

Result<void> MeasureStatisticsNode::execute(FlowContext& context) {
    auto results_data = get_input("results");

    // 注意：实际应从Data中解析MeasureResult数组
    Vector<MeasureResult> results;

    String object_id = get_param("object_id", Data("")).as_string();
    double lsl = get_param("lsl", Data(0.0)).as_number();
    double usl = get_param("usl", Data(0.0)).as_number();
    double nominal = get_param("nominal", Data(0.0)).as_number();

    MeasureStatistics stats;
    stats.object_id = object_id;
    stats.lsl = lsl;
    stats.usl = usl;
    stats.nominal = nominal;

    // 提取测量值
    Vector<double> values;
    for (const auto& result : results) {
        if (object_id.empty() || result.object_id == object_id) {
            values.push_back(result.value);
        }
    }

    if (values.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No measurement data found");
    }

    stats.sample_count = static_cast<int>(values.size());

    // 计算统计值
    double sum = 0.0;
    stats.min_value = values[0];
    stats.max_value = values[0];

    for (double v : values) {
        sum += v;
        stats.min_value = std::min(stats.min_value, v);
        stats.max_value = std::max(stats.max_value, v);
    }

    stats.mean = sum / values.size();
    stats.range = stats.max_value - stats.min_value;

    // 计算标准差
    double sum_sq = 0.0;
    for (double v : values) {
        double diff = v - stats.mean;
        sum_sq += diff * diff;
    }
    stats.std_dev = std::sqrt(sum_sq / values.size());

    // 计算过程能力指数
    if (stats.std_dev > 0.0 && (usl - lsl) > 0.0) {
        stats.cp = (usl - lsl) / (6.0 * stats.std_dev);

        double cpu = (usl - stats.mean) / (3.0 * stats.std_dev);
        double cpl = (stats.mean - lsl) / (3.0 * stats.std_dev);

        stats.cpu = cpu;
        stats.cpl = cpl;
        stats.cpk = std::min(cpu, cpl);
    }

    set_output("statistics", Data("MeasureStatistics[...]"));

    OVF_INFO() << "Statistics calculated: mean=" << stats.mean
               << ", std=" << stats.std_dev
               << ", cp=" << stats.cp
               << ", cpk=" << stats.cpk;

    return Result<void>::success();
}

MeasureCompareNode::MeasureCompareNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureCompareNode::make_info() {
    NodeInfo info;
    info.id = "MeasureCompare";
    info.name = "测量比较";
    info.category = "测量";
    info.description = "将测量结果与标准值比较";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("results", "测量结果", DataType::Array, true));
    info.inputs.push_back(DataPort("reference", "参考值", DataType::Number, false));
    info.outputs.push_back(DataPort("comparison", "比较结果", DataType::Object, false));
    info.outputs.push_back(DataPort("pass", "是否合格", DataType::Boolean, false));

    info.params.push_back(ParamDef("tolerance", "公差", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("tolerance_type", "公差类型", DataType::String, Data("absolute")));

    return info;
}

Result<void> MeasureCompareNode::execute(FlowContext& context) {
    auto results_data = get_input("results");
    auto reference_data = get_input("reference");

    // 注意：实际应从Data中解析
    Vector<MeasureResult> results;
    double reference = reference_data.as_number();

    double tolerance = get_param("tolerance", Data(1.0)).as_number();
    String tolerance_type = get_param("tolerance_type", Data("absolute")).as_string();

    bool overall_pass = true;
    Vector<double> deviations;

    for (const auto& result : results) {
        double deviation = result.value - reference;

        if (tolerance_type == "relative") {
            deviation = deviation / reference * 100.0; // 百分比
        }

        deviations.push_back(deviation);

        bool pass = std::abs(deviation) <= tolerance;
        if (!pass) {
            overall_pass = false;
        }
    }

    set_output("comparison", Data("deviations[...]"));
    set_output("pass", Data(overall_pass));

    OVF_INFO() << "Comparison result: " << (overall_pass ? "PASS" : "FAIL");

    return Result<void>::success();
}

MeasureReportNode::MeasureReportNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MeasureReportNode::make_info() {
    NodeInfo info;
    info.id = "MeasureReport";
    info.name = "测量报告";
    info.category = "测量";
    info.description = "生成测量报告";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("results", "测量结果", DataType::Array, true));
    info.inputs.push_back(DataPort("statistics", "统计信息", DataType::Object, false));
    info.outputs.push_back(DataPort("report", "测量报告", DataType::Object, false));

    info.params.push_back(ParamDef("report_id", "报告ID", DataType::String, Data("report_001")));
    info.params.push_back(ParamDef("inspection_id", "检测ID", DataType::String, Data("insp_001")));
    info.params.push_back(ParamDef("model_name", "模型名称", DataType::String, Data("测量模型")));

    return info;
}

Result<void> MeasureReportNode::execute(FlowContext& context) {
    auto results_data = get_input("results");

    // 注意：实际应从Data中解析
    Vector<MeasureResult> results;

    String report_id = get_param("report_id", Data("report_001")).as_string();
    String inspection_id = get_param("inspection_id", Data("insp_001")).as_string();
    String model_name = get_param("model_name", Data("测量模型")).as_string();

    MeasureReport report;
    report.report_id = report_id;
    report.inspection_id = inspection_id;
    report.model_name = model_name;
    report.results = results;

    // 统计
    for (const auto& result : results) {
        switch (result.status) {
            case MeasureStatus::OK:
                report.pass_count++;
                break;
            case MeasureStatus::NG:
                report.fail_count++;
                break;
            case MeasureStatus::Warning:
                report.warning_count++;
                break;
            default:
                break;
        }
    }

    // 总体状态
    if (report.fail_count > 0) {
        report.overall_status = MeasureStatus::NG;
    } else if (report.warning_count > 0) {
        report.overall_status = MeasureStatus::Warning;
    } else {
        report.overall_status = MeasureStatus::OK;
    }

    set_output("report", Data("MeasureReport[...]"));

    OVF_INFO() << "Report generated: " << report_id
               << ", Pass=" << report.pass_count
               << ", Fail=" << report.fail_count
               << ", Warning=" << report.warning_count;

    return Result<void>::success();
}

// ==================== 注册节点 ====================

OVF_REGISTER_NODE(MeasureModelCreateNode, "MeasureModelCreate", MeasureModelCreateNode::make_info());
OVF_REGISTER_NODE(MeasureModelAddLineNode, "MeasureModelAddLine", MeasureModelAddLineNode::make_info());
OVF_REGISTER_NODE(MeasureModelAddCircleNode, "MeasureModelAddCircle", MeasureModelAddCircleNode::make_info());
OVF_REGISTER_NODE(MeasureModelAddRectangleNode, "MeasureModelAddRectangle", MeasureModelAddRectangleNode::make_info());
OVF_REGISTER_NODE(MeasureModelAddArcNode, "MeasureModelAddArc", MeasureModelAddArcNode::make_info());

OVF_REGISTER_NODE(MeasureModelApplyNode, "MeasureModelApply", MeasureModelApplyNode::make_info());
OVF_REGISTER_NODE(MeasurePosNode, "MeasurePos", MeasurePosNode::make_info());
OVF_REGISTER_NODE(MeasureThresholdNode, "MeasureThreshold", MeasureThresholdNode::make_info());
OVF_REGISTER_NODE(MeasureProfileNode, "MeasureProfile", MeasureProfileNode::make_info());
OVF_REGISTER_NODE(MeasureFuzzyNode, "MeasureFuzzy", MeasureFuzzyNode::make_info());

OVF_REGISTER_NODE(MeasureResultsGetNode, "MeasureResultsGet", MeasureResultsGetNode::make_info());
OVF_REGISTER_NODE(MeasureResultsToRegionsNode, "MeasureResultsToRegions", MeasureResultsToRegionsNode::make_info());
OVF_REGISTER_NODE(MeasureStatisticsNode, "MeasureStatistics", MeasureStatisticsNode::make_info());
OVF_REGISTER_NODE(MeasureCompareNode, "MeasureCompare", MeasureCompareNode::make_info());
OVF_REGISTER_NODE(MeasureReportNode, "MeasureReport", MeasureReportNode::make_info());

} // namespace algorithm
} // namespace ovf