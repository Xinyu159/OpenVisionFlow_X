/**
 * @file edge_detection.cpp
 * @brief 边缘检测节点实现
 */

#include "ovf/algorithm/edge_detection.h"
#include "ovf/core/logger.h"

namespace ovf {
namespace algorithm {

SobelEdgeNode::SobelEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SobelEdgeNode::make_info() {
    NodeInfo info;
    info.id = "SobelEdge";
    info.name = "Sobel边缘检测";
    info.category = "图像处理";
    info.description = "使用Sobel算子检测图像边缘";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（边缘）", DataType::Image));
    
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(50)));
    
    return info;
}

Result<void> SobelEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为灰度图像
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
    
    // Sobel边缘检测
    int threshold = get_param("threshold", Data(50)).as_int();
    
    ImageData output;
    edge_utils::sobel_edge(gray, output, threshold);
    
    set_output("image", Data(output));
    
    OVF_INFO() << "Sobel edge detection completed with threshold " << threshold;
    
    return Result<void>::success();
}

CannyEdgeNode::CannyEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CannyEdgeNode::make_info() {
    NodeInfo info;
    info.id = "CannyEdge";
    info.name = "Canny边缘检测";
    info.category = "图像处理";
    info.description = "使用Canny算法检测图像边缘（简化版）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（边缘）", DataType::Image));
    
    info.params.push_back(ParamDef("low_threshold", "低阈值", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("high_threshold", "高阈值", DataType::Number, Data(100)));
    
    return info;
}

Result<void> CannyEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为灰度图像
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
    
    // Canny边缘检测（简化版）
    int low_threshold = get_param("low_threshold", Data(50)).as_int();
    int high_threshold = get_param("high_threshold", Data(100)).as_int();
    
    ImageData output;
    edge_utils::canny_edge_simple(gray, output, low_threshold, high_threshold);
    
    set_output("image", Data(output));
    
    OVF_INFO() << "Canny edge detection completed: low=" << low_threshold 
               << ", high=" << high_threshold;
    
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(SobelEdgeNode, "SobelEdge", SobelEdgeNode::make_info())
OVF_REGISTER_NODE(CannyEdgeNode, "CannyEdge", CannyEdgeNode::make_info())

} // namespace algorithm
} // namespace ovf