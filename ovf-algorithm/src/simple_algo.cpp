/**
 * @file simple_algo.cpp
 * @brief 轻量级图像处理节点实现
 */

#include "ovf/algorithm/simple_algo.h"

namespace ovf {
namespace algorithm {

// ============== SimpleImageSourceNode ==============

SimpleImageSourceNode::SimpleImageSourceNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SimpleImageSourceNode::make_info() {
    NodeInfo info;
    info.id = "SimpleImageSource";
    info.name = "图像源";
    info.category = "图像采集";
    info.description = "生成测试图像";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("width", "宽度", DataType::Number, Data(512)));
    info.params.push_back(ParamDef("height", "高度", DataType::Number, Data(512)));
    
    return info;
}

Result<void> SimpleImageSourceNode::init() {
    return Result<void>::success();
}

Result<void> SimpleImageSourceNode::execute(FlowContext& context) {
    uint32_t width = static_cast<uint32_t>(get_param("width", Data(512)).as_int());
    uint32_t height = static_cast<uint32_t>(get_param("height", Data(512)).as_int());
    
    ImageData img = image_utils::create_test_image(width, height, instance_id());
    set_output("image", Data(img));
    
    return Result<void>::success();
}

// ============== SimpleImagePreprocessNode ==============

SimpleImagePreprocessNode::SimpleImagePreprocessNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SimpleImagePreprocessNode::make_info() {
    NodeInfo info;
    info.id = "SimpleImagePreprocess";
    info.name = "图像预处理";
    info.category = "图像处理";
    info.description = "图像模糊处理";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> SimpleImagePreprocessNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData output;
    image_utils::blur_image(input, output);
    
    set_output("image", Data(output));
    return Result<void>::success();
}

// ============== SimpleThresholdNode ==============

SimpleThresholdNode::SimpleThresholdNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SimpleThresholdNode::make_info() {
    NodeInfo info;
    info.id = "SimpleThreshold";
    info.name = "阈值化";
    info.category = "图像处理";
    info.description = "图像阈值化处理";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(128)));
    info.params.push_back(ParamDef("max_value", "最大值", DataType::Number, Data(255)));
    
    return info;
}

Result<void> SimpleThresholdNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint8_t thresh = static_cast<uint8_t>(get_param("threshold", Data(128)).as_int());
    uint8_t max_val = static_cast<uint8_t>(get_param("max_value", Data(255)).as_int());
    
    ImageData output;
    image_utils::threshold_image(input, output, thresh, max_val);
    
    set_output("image", Data(output));
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(SimpleImageSourceNode, "SimpleImageSource", SimpleImageSourceNode::make_info());
OVF_REGISTER_NODE(SimpleImagePreprocessNode, "SimpleImagePreprocess", SimpleImagePreprocessNode::make_info());
OVF_REGISTER_NODE(SimpleThresholdNode, "SimpleThreshold", SimpleThresholdNode::make_info());

} // namespace algorithm
} // namespace ovf