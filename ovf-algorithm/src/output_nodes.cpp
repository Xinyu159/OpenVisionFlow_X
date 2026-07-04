/**
 * @file output_nodes.cpp
 * @brief 输出节点实现
 */

#include "ovf/algorithm/output_nodes.h"
#include "ovf/core/logger.h"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace ovf {
namespace algorithm {

ImageSaveNode::ImageSaveNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageSaveNode::make_info() {
    NodeInfo info;
    info.id = "ImageSave";
    info.name = "图像保存";
    info.category = "输出";
    info.description = "保存图像到文件";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.params.push_back(ParamDef("filepath", "文件路径", DataType::String, Data("output.raw")));
    info.params.push_back(ParamDef("format", "格式", DataType::String, Data("raw")));
    
    return info;
}

Result<void> ImageSaveNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    String filepath = get_param("filepath", Data("output.raw")).as_string();
    String format = get_param("format", Data("raw")).as_string();
    
    // 保存图像
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        OVF_ERROR() << "Failed to open file: " << filepath;
        return Result<void>::failure(ErrorCode::FileOpenFailed, 
            "Failed to open file: " + filepath);
    }
    
    if (format == "raw") {
        // RAW格式：直接保存像素数据
        // 写入头部信息
        uint32_t width = input.width;
        uint32_t height = input.height;
        uint32_t channels = input.channels;
        
        file.write(reinterpret_cast<char*>(&width), 4);
        file.write(reinterpret_cast<char*>(&height), 4);
        file.write(reinterpret_cast<char*>(&channels), 4);
        
        // 写入像素数据
        file.write(reinterpret_cast<char*>(input.data.data()), input.data.size());
    } else if (format == "ppm") {
        // PPM格式（仅支持RGB）
        file << "P6\n" << input.width << " " << input.height << "\n255\n";
        
        if (input.channels == 3) {
            // BGR转RGB
            for (size_t i = 0; i < input.data.size(); i += 3) {
                file.put(input.data[i + 2]);  // R
                file.put(input.data[i + 1]);  // G
                file.put(input.data[i]);      // B
            }
        } else if (input.channels == 1) {
            // 灰度转RGB
            for (uint8_t pixel : input.data) {
                file.put(pixel);
                file.put(pixel);
                file.put(pixel);
            }
        }
    } else {
        // 默认RAW格式
        file.write(reinterpret_cast<char*>(input.data.data()), input.data.size());
    }
    
    file.close();
    
    OVF_INFO() << "Image saved: " << filepath << " (" << input.width << "x" 
               << input.height << ", " << input.channels << " channels)";
    
    return Result<void>::success();
}

ResultOutputNode::ResultOutputNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ResultOutputNode::make_info() {
    NodeInfo info;
    info.id = "ResultOutput";
    info.name = "结果输出";
    info.category = "输出";
    info.description = "输出检测结果";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("data", "输入数据", DataType::Any, true));
    
    info.params.push_back(ParamDef("output_format", "输出格式", DataType::String, Data("text")));
    info.params.push_back(ParamDef("label", "标签", DataType::String, Data("Result")));
    
    return info;
}

Result<void> ResultOutputNode::execute(FlowContext& context) {
    auto input_data = get_input("data");
    String label = get_param("label", Data("Result")).as_string();
    String output_format = get_param("output_format", Data("text")).as_string();
    
    // 输出结果
    std::ostringstream oss;
    
    if (output_format == "json") {
        oss << "{\"label\":\"" << label << "\",\"value\":";
        if (input_data.is_number()) {
            oss << input_data.as_number();
        } else if (input_data.is_string()) {
            oss << "\"" << input_data.as_string() << "\"";
        } else if (input_data.is_bool()) {
            oss << (input_data.as_bool() ? "true" : "false");
        } else {
            oss << "\"" << input_data.to_string() << "\"";
        }
        oss << "}";
    } else {
        oss << label << ": " << input_data.to_string();
    }
    
    OVF_INFO() << oss.str();
    
    return Result<void>::success();
}

ImageDisplayNode::ImageDisplayNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageDisplayNode::make_info() {
    NodeInfo info;
    info.id = "ImageDisplay";
    info.name = "图像信息";
    info.category = "输出";
    info.description = "显示图像基本信息";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    return info;
}

Result<void> ImageDisplayNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    
    // 计算统计信息
    uint64_t sum = 0;
    uint8_t min_val = 255;
    uint8_t max_val = 0;
    
    for (uint8_t pixel : input.data) {
        sum += pixel;
        min_val = std::min(min_val, pixel);
        max_val = std::max(max_val, pixel);
    }
    
    double avg = static_cast<double>(sum) / input.data.size();
    
    OVF_INFO() << "Image Info:";
    OVF_INFO() << "  Size: " << input.width << "x" << input.height 
               << " (" << input.channels << " channels)";
    OVF_INFO() << "  Format: " << static_cast<int>(input.format);
    OVF_INFO() << "  Min: " << min_val << ", Max: " << max_val 
               << ", Avg: " << std::fixed << std::setprecision(2) << avg;
    OVF_INFO() << "  Data size: " << input.data.size() << " bytes";
    
    return Result<void>::success();
}

ROICropNode::ROICropNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ROICropNode::make_info() {
    NodeInfo info;
    info.id = "ROICrop";
    info.name = "ROI裁剪";
    info.category = "图像处理";
    info.description = "裁剪图像的指定区域";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("x", "起始X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("y", "起始Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("width", "宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("height", "高度", DataType::Number, Data(100)));
    
    return info;
}

Result<void> ROICropNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint32_t x = static_cast<uint32_t>(get_param("x", Data(0)).as_int());
    uint32_t y = static_cast<uint32_t>(get_param("y", Data(0)).as_int());
    uint32_t roi_w = static_cast<uint32_t>(get_param("width", Data(100)).as_int());
    uint32_t roi_h = static_cast<uint32_t>(get_param("height", Data(100)).as_int());
    
    // 边界检查
    if (x >= input.width || y >= input.height) {
        return Result<void>::failure(ErrorCode::OutOfRange, "ROI start position out of image bounds");
    }
    
    roi_w = std::min(roi_w, input.width - x);
    roi_h = std::min(roi_h, input.height - y);
    
    // 裁剪
    ImageData output;
    output.width = roi_w;
    output.height = roi_h;
    output.channels = input.channels;
    output.format = input.format;
    output.data.resize(roi_w * roi_h * input.channels);
    output.timestamp = input.timestamp;
    output.frame_id = input.frame_id;
    output.source_id = input.source_id + "_roi";
    
    for (uint32_t row = 0; row < roi_h; ++row) {
        size_t src_offset = ((y + row) * input.width + x) * input.channels;
        size_t dst_offset = row * roi_w * input.channels;
        
        std::memcpy(&output.data[dst_offset], &input.data[src_offset], 
                    roi_w * input.channels);
    }
    
    set_output("image", Data(output));
    
    OVF_INFO() << "ROI cropped: (" << x << "," << y << ") " 
               << roi_w << "x" << roi_h;
    
    return Result<void>::success();
}

ImageConcatNode::ImageConcatNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageConcatNode::make_info() {
    NodeInfo info;
    info.id = "ImageConcat";
    info.name = "图像拼接";
    info.category = "图像处理";
    info.description = "水平或垂直拼接两张图像";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("direction", "拼接方向", DataType::String, Data("horizontal")));
    
    return info;
}

Result<void> ImageConcatNode::execute(FlowContext& context) {
    auto data1 = get_input("image1");
    auto data2 = get_input("image2");
    
    if (!data1.is_image() || !data2.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Both inputs must be images");
    }
    
    ImageData img1 = data1.as_image();
    ImageData img2 = data2.as_image();
    
    if (img1.empty() || img2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    String direction = get_param("direction", Data("horizontal")).as_string();
    
    ImageData output;
    
    if (direction == "horizontal" || direction == "h") {
        // 水平拼接：高度必须相同
        if (img1.height != img2.height) {
            return Result<void>::failure(ErrorCode::InvalidParameter, 
                "Images must have same height for horizontal concatenation");
        }
        
        output.width = img1.width + img2.width;
        output.height = img1.height;
        output.channels = std::max(img1.channels, img2.channels);
        output.format = img1.format;
        output.data.resize(output.width * output.height * output.channels);
        
        // 复制第一张图像
        for (uint32_t y = 0; y < output.height; ++y) {
            for (uint32_t x = 0; x < img1.width; ++x) {
                for (uint32_t c = 0; c < img1.channels; ++c) {
                    output.data[(y * output.width + x) * output.channels + c] =
                        img1.data[(y * img1.width + x) * img1.channels + c];
                }
            }
        }
        
        // 复制第二张图像
        for (uint32_t y = 0; y < output.height; ++y) {
            for (uint32_t x = 0; x < img2.width; ++x) {
                for (uint32_t c = 0; c < img2.channels; ++c) {
                    output.data[(y * output.width + img1.width + x) * output.channels + c] =
                        img2.data[(y * img2.width + x) * img2.channels + c];
                }
            }
        }
    } else {
        // 垂直拼接：宽度必须相同
        if (img1.width != img2.width) {
            return Result<void>::failure(ErrorCode::InvalidParameter, 
                "Images must have same width for vertical concatenation");
        }
        
        output.width = img1.width;
        output.height = img1.height + img2.height;
        output.channels = std::max(img1.channels, img2.channels);
        output.format = img1.format;
        output.data.resize(output.width * output.height * output.channels);
        
        // 复制第一张图像
        for (uint32_t y = 0; y < img1.height; ++y) {
            for (uint32_t x = 0; x < output.width; ++x) {
                for (uint32_t c = 0; c < img1.channels; ++c) {
                    output.data[(y * output.width + x) * output.channels + c] =
                        img1.data[(y * img1.width + x) * img1.channels + c];
                }
            }
        }
        
        // 复制第二张图像
        for (uint32_t y = 0; y < img2.height; ++y) {
            for (uint32_t x = 0; x < output.width; ++x) {
                for (uint32_t c = 0; c < img2.channels; ++c) {
                    output.data[((img1.height + y) * output.width + x) * output.channels + c] =
                        img2.data[(y * img2.width + x) * img2.channels + c];
                }
            }
        }
    }
    
    output.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    set_output("image", Data(output));
    
    OVF_INFO() << "Images concatenated: " << output.width << "x" << output.height;
    
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(ImageSaveNode, "ImageSave", ImageSaveNode::make_info());
OVF_REGISTER_NODE(ResultOutputNode, "ResultOutput", ResultOutputNode::make_info());
OVF_REGISTER_NODE(ImageDisplayNode, "ImageDisplay", ImageDisplayNode::make_info());
OVF_REGISTER_NODE(ROICropNode, "ROICrop", ROICropNode::make_info());
OVF_REGISTER_NODE(ImageConcatNode, "ImageConcat", ImageConcatNode::make_info());

} // namespace algorithm
} // namespace ovf