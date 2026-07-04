/**
 * @file calibration_pattern.cpp
 * @brief 标定板检测节点实现
 */

#include "ovf/algorithm/calibration_pattern.h"
#include "ovf/core/logger.h"
#include <unordered_map>

namespace ovf {
namespace algorithm {

// ==================== 辅助函数 ====================

// 转换为灰度图像
static ImageData to_gray_image(const ImageData& input) {
    ImageData gray;
    calibration_pattern_utils::to_gray(input, gray);
    return gray;
}

// 转换为彩色图像（用于绘制）
static ImageData to_color_image(const ImageData& input) {
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

// ==================== 棋盘格检测节点 ====================

CheckerboardDetectNode::CheckerboardDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CheckerboardDetectNode::make_info() {
    NodeInfo info;
    info.id = "CheckerboardDetect";
    info.name = "棋盘格检测";
    info.category = "标定板检测";
    info.description = "检测图像中的棋盘格标定板并提取角点";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("pattern_found", "是否检测成功", DataType::Boolean));
    info.outputs.push_back(DataPort("corner_count", "角点数量", DataType::Number));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));

    info.params.push_back(ParamDef("pattern_rows", "标定板行数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("pattern_cols", "标定板列数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("square_size", "方格尺寸（毫米）", DataType::Number, Data(25.0)));
    info.params.push_back(ParamDef("threshold", "响应阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("min_corners", "最小角点数", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("draw_corners", "绘制角点", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("draw_numbers", "绘制编号", DataType::Boolean, Data(false)));

    return info;
}

Result<void> CheckerboardDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 转换灰度
    ImageData gray = to_gray_image(input);

    // 参数
    int pattern_rows = get_param("pattern_rows", Data(8)).as_int();
    int pattern_cols = get_param("pattern_cols", Data(8)).as_int();
    double square_size = get_param("square_size", Data(25.0)).as_number();
    int threshold = get_param("threshold", Data(50)).as_int();
    int min_corners = get_param("min_corners", Data(100)).as_int();
    bool draw_corners = get_param("draw_corners", Data(true)).as_bool();
    bool draw_numbers = get_param("draw_numbers", Data(false)).as_bool();

    // 检测棋盘格
    PatternResult result;
    bool found = calibration_pattern_utils::detect_checkerboard(
        gray, result, pattern_rows, pattern_cols, threshold, min_corners);

    // 输出
    set_output("pattern_found", Data(found));
    set_output("corner_count", Data(static_cast<int>(result.corners.size())));
    set_output("confidence", Data(result.confidence));

    // 输出图像
    if (draw_corners && found) {
        ImageData output = to_color_image(input);
        calibration_pattern_utils::draw_calibration_corners(output, result.corners, 255, 0, 0);

        if (draw_numbers) {
            calibration_pattern_utils::draw_point_numbers(output, result.corners);
        }

        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    if (found) {
        OVF_INFO() << "Checkerboard detected: " << result.rows << "x" << result.cols
                   << ", " << result.corners.size() << " corners, confidence=" << result.confidence;
    } else {
        OVF_INFO() << "Checkerboard not detected";
    }

    return Result<void>::success();
}

// ==================== 圆点阵列检测节点 ====================

CircleGridDetectNode::CircleGridDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CircleGridDetectNode::make_info() {
    NodeInfo info;
    info.id = "CircleGridDetect";
    info.name = "圆点阵列检测";
    info.category = "标定板检测";
    info.description = "检测图像中的圆点阵列标定板";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("pattern_found", "是否检测成功", DataType::Boolean));
    info.outputs.push_back(DataPort("circle_count", "圆点数量", DataType::Number));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));

    info.params.push_back(ParamDef("pattern_rows", "标定板行数", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("pattern_cols", "标定板列数", DataType::Number, Data(7)));
    info.params.push_back(ParamDef("circle_spacing", "圆点间距（毫米）", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("min_radius", "最小半径", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_radius", "最大半径", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("threshold", "边缘阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("draw_circles", "绘制圆点", DataType::Boolean, Data(true)));

    return info;
}

Result<void> CircleGridDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = to_gray_image(input);

    // 参数
    int pattern_rows = get_param("pattern_rows", Data(5)).as_int();
    int pattern_cols = get_param("pattern_cols", Data(7)).as_int();
    double circle_spacing = get_param("circle_spacing", Data(20.0)).as_number();
    float min_radius = static_cast<float>(get_param("min_radius", Data(5)).as_number());
    float max_radius = static_cast<float>(get_param("max_radius", Data(50)).as_number());
    int threshold = get_param("threshold", Data(50)).as_int();
    bool draw_circles = get_param("draw_circles", Data(true)).as_bool();

    // 检测圆点阵列
    PatternResult result;
    bool found = calibration_pattern_utils::detect_circle_grid(
        gray, result, pattern_rows, pattern_cols, min_radius, max_radius, threshold);

    // 输出
    set_output("pattern_found", Data(found));
    set_output("circle_count", Data(static_cast<int>(result.circles.size())));
    set_output("confidence", Data(result.confidence));

    // 输出图像
    if (draw_circles && found) {
        ImageData output = to_color_image(input);
        calibration_pattern_utils::draw_calibration_circles(output, result.circles, 0, 255, 0);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    if (found) {
        OVF_INFO() << "Circle grid detected: " << result.rows << "x" << result.cols
                   << ", " << result.circles.size() << " circles, confidence=" << result.confidence;
    } else {
        OVF_INFO() << "Circle grid not detected";
    }

    return Result<void>::success();
}

// ==================== 标定点提取节点 ====================

CalibrationPointsExtractNode::CalibrationPointsExtractNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CalibrationPointsExtractNode::make_info() {
    NodeInfo info;
    info.id = "CalibrationPointsExtract";
    info.name = "标定点提取";
    info.category = "标定板检测";
    info.description = "从标定板图像中提取标定点的精确坐标";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("pattern_type", "标定板类型", DataType::String, false, Data("Checkerboard")));
    info.outputs.push_back(DataPort("calibration_points", "标定点列表", DataType::PointCloud));
    info.outputs.push_back(DataPort("point_count", "点数量", DataType::Number));
    info.outputs.push_back(DataPort("extract_success", "提取成功", DataType::Boolean));
    info.outputs.push_back(DataPort("avg_spacing", "平均间距", DataType::Number));

    info.params.push_back(ParamDef("pattern_rows", "行数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("pattern_cols", "列数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("unit_size", "单位尺寸（毫米）", DataType::Number, Data(25.0)));
    info.params.push_back(ParamDef("refine_window", "优化窗口大小", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("threshold", "检测阈值", DataType::Number, Data(50)));

    return info;
}

Result<void> CalibrationPointsExtractNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    auto pattern_type_data = get_input("pattern_type");
    String pattern_type = pattern_type_data.is_string() ? pattern_type_data.as_string() : "Checkerboard";

    ImageData gray = to_gray_image(input);

    // 参数
    int pattern_rows = get_param("pattern_rows", Data(8)).as_int();
    int pattern_cols = get_param("pattern_cols", Data(8)).as_int();
    double unit_size = get_param("unit_size", Data(25.0)).as_number();
    int refine_window = get_param("refine_window", Data(5)).as_int();
    int threshold = get_param("threshold", Data(50)).as_int();

    PatternResult pattern_result;

    bool success = false;
    if (pattern_type == "Checkerboard") {
        success = calibration_pattern_utils::detect_checkerboard(
            gray, pattern_result, pattern_rows, pattern_cols, threshold);
    } else if (pattern_type == "CircleGrid") {
        success = calibration_pattern_utils::detect_circle_grid(
            gray, pattern_result, pattern_rows, pattern_cols, 5, 50, threshold);
    } else {
        // 默认使用棋盘格
        success = calibration_pattern_utils::detect_checkerboard(
            gray, pattern_result, pattern_rows, pattern_cols, threshold);
    }

    set_output("extract_success", Data(success));

    if (!success) {
        set_output("point_count", Data(0));
        set_output("avg_spacing", Data(0.0));
        OVF_INFO() << "Calibration points extraction failed";
        return Result<void>::success();
    }

    // 创建标定点云（包含图像坐标和物理坐标）
    PointCloudData calib_points;

    if (!pattern_result.corners.empty()) {
        calib_points.points.resize(pattern_result.corners.size());
        for (size_t i = 0; i < pattern_result.corners.size(); ++i) {
            const auto& corner = pattern_result.corners[i];
            calib_points.points[i].x = corner.x;                          // 图像X
            calib_points.points[i].y = corner.y;                          // 图像Y
            calib_points.points[i].z = corner.col * unit_size;            // 物理X（毫米）
        }
    } else if (!pattern_result.circles.empty()) {
        calib_points.points.resize(pattern_result.circles.size());
        for (size_t i = 0; i < pattern_result.circles.size(); ++i) {
            const auto& circle = pattern_result.circles[i];
            calib_points.points[i].x = circle.center_x;
            calib_points.points[i].y = circle.center_y;
            calib_points.points[i].z = circle.col * unit_size;
        }
    }

    // 计算平均间距
    float avg_spacing = 0;
    int spacing_count = 0;

    if (!pattern_result.corners.empty()) {
        for (int r = 0; r < pattern_rows; ++r) {
            for (int c = 0; c < pattern_cols - 1; ++c) {
                float dx = pattern_result.corners[r * pattern_cols + c + 1].x -
                          pattern_result.corners[r * pattern_cols + c].x;
                float dy = pattern_result.corners[r * pattern_cols + c + 1].y -
                          pattern_result.corners[r * pattern_cols + c].y;
                avg_spacing += std::sqrt(dx * dx + dy * dy);
                spacing_count++;
            }
        }
    }

    if (spacing_count > 0) {
        avg_spacing /= spacing_count;
    }

    set_output("calibration_points", Data(calib_points));
    set_output("point_count", Data(static_cast<int>(calib_points.size())));
    set_output("avg_spacing", Data(avg_spacing));

    OVF_INFO() << "Calibration points extracted: " << calib_points.size()
               << " points, avg_spacing=" << avg_spacing << " pixels";

    return Result<void>::success();
}

// ==================== 标定板匹配节点 ====================

CalibrationPatternMatchNode::CalibrationPatternMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CalibrationPatternMatchNode::make_info() {
    NodeInfo info;
    info.id = "CalibrationPatternMatch";
    info.name = "标定板匹配";
    info.category = "标定板检测";
    info.description = "匹配标定板模板与图像，定位标定板位置";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, false));
    info.outputs.push_back(DataPort("match_found", "匹配成功", DataType::Boolean));
    info.outputs.push_back(DataPort("match_score", "匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("position_x", "位置X", DataType::Number));
    info.outputs.push_back(DataPort("position_y", "位置Y", DataType::Number));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));

    info.params.push_back(ParamDef("pattern_rows", "标定板行数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("pattern_cols", "标定板列数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("search_scale", "搜索尺度范围", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("match_threshold", "匹配阈值", DataType::Number, Data(0.7)));
    info.params.push_back(ParamDef("draw_match", "绘制匹配结果", DataType::Boolean, Data(true)));

    return info;
}

Result<void> CalibrationPatternMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 参数
    int pattern_rows = get_param("pattern_rows", Data(8)).as_int();
    int pattern_cols = get_param("pattern_cols", Data(8)).as_int();
    double match_threshold = get_param("match_threshold", Data(0.7)).as_number();
    bool draw_match = get_param("draw_match", Data(true)).as_bool();

    ImageData gray = to_gray_image(input);

    // 尝试检测标定板作为匹配
    PatternResult result;
    bool found = calibration_pattern_utils::detect_checkerboard(
        gray, result, pattern_rows, pattern_cols, 50, 100);

    double match_score = found ? result.confidence : 0.0;
    bool match_found = found && (match_score >= match_threshold);

    set_output("match_found", Data(match_found));
    set_output("match_score", Data(match_score));

    // 计算中心位置
    if (found && !result.corners.empty()) {
        float center_x = 0, center_y = 0;
        for (const auto& c : result.corners) {
            center_x += c.x;
            center_y += c.y;
        }
        center_x /= result.corners.size();
        center_y /= result.corners.size();

        set_output("position_x", Data(center_x));
        set_output("position_y", Data(center_y));
    } else {
        set_output("position_x", Data(0));
        set_output("position_y", Data(0));
    }

    // 输出图像
    if (draw_match && match_found) {
        ImageData output = to_color_image(input);
        calibration_pattern_utils::draw_calibration_corners(output, result.corners, 0, 255, 0);
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    if (match_found) {
        OVF_INFO() << "Pattern matched: score=" << match_score;
    } else {
        OVF_INFO() << "Pattern not matched";
    }

    return Result<void>::success();
}

// ==================== 标定质量检查节点 ====================

CalibrationQualityCheckNode::CalibrationQualityCheckNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CalibrationQualityCheckNode::make_info() {
    NodeInfo info;
    info.id = "CalibrationQualityCheck";
    info.name = "标定质量检查";
    info.category = "标定板检测";
    info.description = "检查标定板检测的质量和完整性";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("pattern_result", "检测结果", DataType::PointCloud, false));
    info.outputs.push_back(DataPort("quality_valid", "质量有效", DataType::Boolean));
    info.outputs.push_back(DataPort("quality_level", "质量等级", DataType::String));
    info.outputs.push_back(DataPort("coverage", "覆盖率", DataType::Number));
    info.outputs.push_back(DataPort("uniformity", "均匀性", DataType::Number));
    info.outputs.push_back(DataPort("valid_points", "有效点数", DataType::Number));
    info.outputs.push_back(DataPort("image", "输出图像（带质量标注）", DataType::Image));

    info.params.push_back(ParamDef("pattern_rows", "预期行数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("pattern_cols", "预期列数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("pattern_type", "标定板类型", DataType::String, Data("Checkerboard")));
    info.params.push_back(ParamDef("threshold", "检测阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("draw_quality", "绘制质量标注", DataType::Boolean, Data(true)));

    return info;
}

Result<void> CalibrationQualityCheckNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 参数
    int expected_rows = get_param("pattern_rows", Data(8)).as_int();
    int expected_cols = get_param("pattern_cols", Data(8)).as_int();
    String pattern_type = get_param("pattern_type", Data("Checkerboard")).as_string();
    int threshold = get_param("threshold", Data(50)).as_int();
    bool draw_quality = get_param("draw_quality", Data(true)).as_bool();

    ImageData gray = to_gray_image(input);

    // 检测标定板
    PatternResult pattern_result;

    bool found = false;
    if (pattern_type == "Checkerboard") {
        found = calibration_pattern_utils::detect_checkerboard(
            gray, pattern_result, expected_rows, expected_cols, threshold);
    } else if (pattern_type == "CircleGrid") {
        found = calibration_pattern_utils::detect_circle_grid(
            gray, pattern_result, expected_rows, expected_cols, 5, 50, threshold);
    }

    // 计算质量
    QualityResult quality = calibration_pattern_utils::compute_quality(
        pattern_result, expected_rows, expected_cols);

    // 输出
    set_output("quality_valid", Data(quality.valid));
    set_output("quality_level", Data(quality.quality_level));
    set_output("coverage", Data(quality.coverage));
    set_output("uniformity", Data(quality.uniformity));
    set_output("valid_points", Data(quality.valid_points));

    // 输出图像
    if (draw_quality && found) {
        ImageData output = to_color_image(input);

        // 根据质量选择颜色
        uint8_t r = 0, g = 0, b = 0;
        if (quality.quality_level == "Excellent") {
            g = 255;  // 绿色
        } else if (quality.quality_level == "Good") {
            r = 255; g = 255;  // 黄色
        } else if (quality.quality_level == "Average") {
            r = 255; g = 165; b = 0;  // 橙色
        } else {
            r = 255;  // 红色
        }

        if (!pattern_result.corners.empty()) {
            calibration_pattern_utils::draw_calibration_corners(output, pattern_result.corners, r, g, b);
        } else if (!pattern_result.circles.empty()) {
            calibration_pattern_utils::draw_calibration_circles(output, pattern_result.circles, r, g, b);
        }

        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }

    OVF_INFO() << "Quality check: level=" << quality.quality_level
               << ", coverage=" << quality.coverage
               << ", uniformity=" << quality.uniformity
               << ", valid_points=" << quality.valid_points;

    return Result<void>::success();
}

// ==================== 注册节点 ====================

OVF_REGISTER_NODE(CheckerboardDetectNode, "CheckerboardDetect", CheckerboardDetectNode::make_info())
OVF_REGISTER_NODE(CircleGridDetectNode, "CircleGridDetect", CircleGridDetectNode::make_info())
OVF_REGISTER_NODE(CalibrationPointsExtractNode, "CalibrationPointsExtract", CalibrationPointsExtractNode::make_info())
OVF_REGISTER_NODE(CalibrationPatternMatchNode, "CalibrationPatternMatch", CalibrationPatternMatchNode::make_info())
OVF_REGISTER_NODE(CalibrationQualityCheckNode, "CalibrationQualityCheck", CalibrationQualityCheckNode::make_info())

} // namespace algorithm
} // namespace ovf