/**
 * @file blob_analysis.cpp
 * @brief Blob分析节点实现
 */

#include "ovf/algorithm/blob_analysis.h"
#include "ovf/core/logger.h"

namespace ovf {
namespace algorithm {

BlobAnalysisNode::BlobAnalysisNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BlobAnalysisNode::make_info() {
    NodeInfo info;
    info.id = "BlobAnalysis";
    info.name = "Blob分析";
    info.category = "图像分析";
    info.description = "检测二值图像中的连通区域（Blob）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（带标注）", DataType::Image));
    info.outputs.push_back(DataPort("blob_count", "Blob数量", DataType::Number));
    info.outputs.push_back(DataPort("blob_list", "Blob列表", DataType::Array));
    
    info.params.push_back(ParamDef("min_area", "最小面积", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("max_area", "最大面积", DataType::Number, Data(100000)));
    info.params.push_back(ParamDef("draw_boxes", "绘制边界框", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> BlobAnalysisNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 如果是多通道图像，需要先转换
    ImageData binary;
    if (input.channels != 1) {
        // 简化处理：取第一个通道
        binary.width = input.width;
        binary.height = input.height;
        binary.channels = 1;
        binary.format = ImageFormat::Mono8;
        binary.data.resize(binary.width * binary.height);
        
        for (size_t i = 0; i < binary.data.size(); ++i) {
            // 对BGR图像，简单取灰度值
            if (input.channels >= 3) {
                uint8_t b = input.data[i * 3];
                uint8_t g = input.data[i * 3 + 1];
                uint8_t r = input.data[i * 3 + 2];
                binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r);
            } else {
                binary.data[i] = input.data[i];
            }
        }
    } else {
        binary = input;
    }
    
    // Blob分析
    uint32_t min_area = static_cast<uint32_t>(get_param("min_area", Data(10)).as_int());
    uint32_t max_area = static_cast<uint32_t>(get_param("max_area", Data(100000)).as_int());
    
    auto blobs = blob_utils::find_blobs(binary, min_area, max_area);
    
    // 输出Blob数量
    set_output("blob_count", Data(static_cast<int>(blobs.size())));
    
    // 输出图像
    bool draw_boxes = get_param("draw_boxes", Data(true)).as_bool();
    if (draw_boxes && input.channels == 3) {
        // 绘制边界框（需要在彩色图像上）
        ImageData output = input;
        blob_utils::draw_blob_boxes(output, blobs, 0, 255, 0);  // 绿色边界框
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input));
    }
    
    // 日志输出
    OVF_INFO() << "Blob analysis: found " << blobs.size() << " blobs";
    for (const auto& blob : blobs) {
        OVF_DEBUG() << "Blob #" << blob.id << ": area=" << blob.area 
                    << ", center=(" << blob.x << "," << blob.y << ")"
                    << ", size=" << blob.width << "x" << blob.height;
    }
    
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(BlobAnalysisNode, "BlobAnalysis", BlobAnalysisNode::make_info())

} // namespace algorithm
} // namespace ovf