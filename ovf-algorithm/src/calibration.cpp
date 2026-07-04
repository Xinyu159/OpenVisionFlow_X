/**
 * @file calibration.cpp
 * @brief 标定节点实现
 */

#include "ovf/algorithm/calibration.h"
#include "ovf/core/logger.h"

namespace ovf {
namespace algorithm {

NinePointCalibrationNode::NinePointCalibrationNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo NinePointCalibrationNode::make_info() {
    NodeInfo info;
    info.id = "NinePointCalibration";
    info.name = "九点标定";
    info.category = "标定";
    info.description = "九点标定算法，将图像坐标转换为物理坐标";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_points", "图像坐标点列表", DataType::Array, true));
    info.inputs.push_back(DataPort("world_points", "物理坐标点列表", DataType::Array, true));
    
    info.outputs.push_back(DataPort("calibration_result", "标定结果", DataType::Object));
    info.outputs.push_back(DataPort("transform_matrix", "变换矩阵参数", DataType::Array));
    info.outputs.push_back(DataPort("error", "标定误差", DataType::Number));
    
    info.params.push_back(ParamDef("min_points", "最少标定点数", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("max_error_threshold", "最大允许误差", DataType::Number, Data(1.0)));
    
    return info;
}

Result<void> NinePointCalibrationNode::execute(FlowContext& context) {
    auto image_points_data = get_input("image_points");
    auto world_points_data = get_input("world_points");
    
    if (image_points_data.is_none() || world_points_data.is_none()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "Image points and world points are required");
    }
    
    // 从输入数据中解析标定点
    // 输入格式: 数组形式，每个元素包含 x, y 坐标
    // 这里需要从 Data 中提取数据，由于 Data 类目前不直接支持数组类型的详细访问
    // 我们需要通过参数来设置标定点
    
    std::vector<CalibrationPoint> points;
    
    // 从参数中读取标定点
    // 参数格式: "points" 参数为数组，每个元素包含 image_x, image_y, world_x, world_y
    // 由于 ParamSet 的限制，我们使用字符串参数来传递标定点数据
    
    // 尝试从输入获取点数据（假设通过字符串格式传递）
    // 格式: "ix1,iy1,wx1,wy1;ix2,iy2,wx2,wy2;..."
    
    // 暂时使用参数方式获取标定点
    // 后续可以扩展 Data 类支持更复杂的数组类型
    
    // 简化实现：从9个独立的点参数读取
    for (int i = 1; i <= 9; ++i) {
        std::string prefix = "point" + std::to_string(i) + "_";
        
        bool has_image_x = get_param(prefix + "image_x").is_number();
        bool has_image_y = get_param(prefix + "image_y").is_number();
        bool has_world_x = get_param(prefix + "world_x").is_number();
        bool has_world_y = get_param(prefix + "world_y").is_number();
        
        if (has_image_x && has_image_y && has_world_x && has_world_y) {
            CalibrationPoint pt;
            pt.image_x = get_param(prefix + "image_x").as_number();
            pt.image_y = get_param(prefix + "image_y").as_number();
            pt.world_x = get_param(prefix + "world_x").as_number();
            pt.world_y = get_param(prefix + "world_y").as_number();
            points.push_back(pt);
        }
    }
    
    int min_points = get_param("min_points", Data(3)).as_int();
    
    if (static_cast<int>(points.size()) < min_points) {
        return Result<void>::failure(ErrorCode::CalibrationFailed, 
            "Not enough calibration points. Need at least " + std::to_string(min_points));
    }
    
    // 执行标定
    result_ = calibration_utils::nine_point_calibration(points);
    
    if (!result_.valid) {
        return Result<void>::failure(ErrorCode::CalibrationFailed, 
            "Calibration algorithm failed. Points may be collinear.");
    }
    
    double max_error_threshold = get_param("max_error_threshold", Data(1.0)).as_number();
    
    if (result_.max_error > max_error_threshold) {
        OVF_WARN() << "Calibration max error " << result_.max_error 
                   << " exceeds threshold " << max_error_threshold;
    }
    
    // 输出标定结果
    // 变换矩阵参数 [a, b, c, d, e, f]
    // 输出为多个独立数值
    set_output("error", Data(static_cast<double>(result_.error)));
    
    // 由于 Data 类限制，输出变换参数通过多个输出端口
    set_output("a", Data(static_cast<double>(result_.a)));
    set_output("b", Data(static_cast<double>(result_.b)));
    set_output("c", Data(static_cast<double>(result_.c)));
    set_output("d", Data(static_cast<double>(result_.d)));
    set_output("e", Data(static_cast<double>(result_.e)));
    set_output("f", Data(static_cast<double>(result_.f)));
    
    // 额外输出信息
    set_output("avg_error", Data(static_cast<double>(result_.error)));
    set_output("max_error", Data(static_cast<double>(result_.max_error)));
    set_output("is_valid", Data(result_.valid));
    
    OVF_INFO() << "Nine-point calibration completed. "
               << "Average error: " << result_.error 
               << ", Max error: " << result_.max_error
               << ", Points used: " << points.size();
    
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(NinePointCalibrationNode, "NinePointCalibration", NinePointCalibrationNode::make_info());

} // namespace algorithm
} // namespace ovf