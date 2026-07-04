/**
 * @file image_arithmetic.cpp
 * @brief 图像运算算子实现（纯C++实现，参考VisionPro CogIPTwoImage系列）
 */

#include "ovf/algorithm/image_arithmetic.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace ovf {
namespace algorithm {

// ========== 工具函数实现 ==========

namespace arithmetic_utils {

bool validate_images(const ImageData& a, const ImageData& b) {
    if (a.empty() || b.empty()) {
        return false;
    }
    if (a.width != b.width || a.height != b.height) {
        return false;
    }
    if (a.channels != b.channels) {
        return false;
    }
    return true;
}

// 处理浮点图像的辅助函数
bool is_float_image(const ImageData& img) {
    return img.format == ImageFormat::Float32;
}

// 获取像素值（考虑不同格式）
inline double get_pixel_value(const uint8_t* data, size_t idx, ImageFormat format) {
    if (format == ImageFormat::Float32) {
        const float* fdata = reinterpret_cast<const float*>(data);
        return static_cast<double>(fdata[idx]);
    }
    return static_cast<double>(data[idx]);
}

// 设置像素值（考虑不同格式和饱和截断）
inline void set_pixel_value(uint8_t* data, size_t idx, ImageFormat format, 
                            double value, bool saturate) {
    if (format == ImageFormat::Float32) {
        // 浮点图像保持原始值
        float* fdata = reinterpret_cast<float*>(data);
        fdata[idx] = static_cast<float>(value);
    } else {
        // 8位图像根据saturate参数处理
        if (saturate) {
            data[idx] = saturate_cast(value);
        } else {
            // 不截断，直接转换（可能溢出）
            data[idx] = static_cast<uint8_t>(std::clamp(value, 0.0, 255.0));
        }
    }
}

void image_add(const ImageData& a, const ImageData& b, ImageData& result,
               double scale_factor, double offset, bool saturate) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image add: images not compatible";
        return;
    }
    
    // 准备输出图像
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        // 加法运算：result = (a + b) * scale_factor + offset
        double result_val = (val_a + val_b) * scale_factor + offset;
        
        set_pixel_value(result.data.data(), i, result.format, result_val, saturate);
    }
}

void image_subtract(const ImageData& a, const ImageData& b, ImageData& result,
                    double scale_factor, double offset, bool saturate) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image subtract: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        // 减法运算：result = (a - b) * scale_factor + offset
        double result_val = (val_a - val_b) * scale_factor + offset;
        
        set_pixel_value(result.data.data(), i, result.format, result_val, saturate);
    }
}

void image_multiply(const ImageData& a, const ImageData& b, ImageData& result,
                    double scale_factor, double offset, bool saturate) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image multiply: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        // 乘法运算：result = (a * b) * scale_factor + offset
        // 对于8位图像，需要归一化（假设输入是0-255）
        if (a.format != ImageFormat::Float32) {
            val_a /= 255.0;
            val_b /= 255.0;
        }
        
        double result_val = (val_a * val_b) * scale_factor + offset;
        
        // 如果输入是8位图像，输出也要归一化回0-255范围
        if (a.format != ImageFormat::Float32 && saturate) {
            result_val *= 255.0;
        }
        
        set_pixel_value(result.data.data(), i, result.format, result_val, saturate);
    }
}

void image_divide(const ImageData& a, const ImageData& b, ImageData& result,
                  double scale_factor, double offset, bool saturate) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image divide: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        // 除法运算：result = (a / b) * scale_factor + offset
        // 避免除以零
        if (std::abs(val_b) < 1e-10) {
            val_b = 1e-10;
        }
        
        double result_val = (val_a / val_b) * scale_factor + offset;
        
        set_pixel_value(result.data.data(), i, result.format, result_val, saturate);
    }
}

void image_abs_diff(const ImageData& a, const ImageData& b, ImageData& result,
                    double scale_factor, double offset, bool saturate) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image abs_diff: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        // 绝对差值：result = |a - b| * scale_factor + offset
        double result_val = std::abs(val_a - val_b) * scale_factor + offset;
        
        set_pixel_value(result.data.data(), i, result.format, result_val, saturate);
    }
}

void image_min(const ImageData& a, const ImageData& b, ImageData& result) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image min: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        double result_val = std::min(val_a, val_b);
        
        set_pixel_value(result.data.data(), i, result.format, result_val, true);
    }
}

void image_max(const ImageData& a, const ImageData& b, ImageData& result) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image max: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        double result_val = std::max(val_a, val_b);
        
        set_pixel_value(result.data.data(), i, result.format, result_val, true);
    }
}

void image_and(const ImageData& a, const ImageData& b, ImageData& result) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image and: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    // 位运算只对8位图像有意义
    if (a.format == ImageFormat::Float32) {
        // 对于浮点图像，转换为逻辑AND（大于阈值视为1）
        for (size_t i = 0; i < total_pixels; ++i) {
            double val_a = get_pixel_value(a.data.data(), i, a.format);
            double val_b = get_pixel_value(b.data.data(), i, b.format);
            
            float result_val = (val_a > 0.5 && val_b > 0.5) ? 1.0f : 0.0f;
            float* fdata = reinterpret_cast<float*>(result.data.data());
            fdata[i] = result_val;
        }
    } else {
        // 8位图像：直接位运算
        for (size_t i = 0; i < total_pixels; ++i) {
            result.data[i] = a.data[i] & b.data[i];
        }
    }
}

void image_or(const ImageData& a, const ImageData& b, ImageData& result) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image or: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    if (a.format == ImageFormat::Float32) {
        for (size_t i = 0; i < total_pixels; ++i) {
            double val_a = get_pixel_value(a.data.data(), i, a.format);
            double val_b = get_pixel_value(b.data.data(), i, b.format);
            
            float result_val = (val_a > 0.5 || val_b > 0.5) ? 1.0f : 0.0f;
            float* fdata = reinterpret_cast<float*>(result.data.data());
            fdata[i] = result_val;
        }
    } else {
        for (size_t i = 0; i < total_pixels; ++i) {
            result.data[i] = a.data[i] | b.data[i];
        }
    }
}

void image_xor(const ImageData& a, const ImageData& b, ImageData& result) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image xor: images not compatible";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    if (a.format == ImageFormat::Float32) {
        for (size_t i = 0; i < total_pixels; ++i) {
            double val_a = get_pixel_value(a.data.data(), i, a.format);
            double val_b = get_pixel_value(b.data.data(), i, b.format);
            
            float result_val = ((val_a > 0.5) != (val_b > 0.5)) ? 1.0f : 0.0f;
            float* fdata = reinterpret_cast<float*>(result.data.data());
            fdata[i] = result_val;
        }
    } else {
        for (size_t i = 0; i < total_pixels; ++i) {
            result.data[i] = a.data[i] ^ b.data[i];
        }
    }
}

void image_not(const ImageData& a, ImageData& result) {
    if (a.empty()) {
        OVF_ERROR() << "Image not: input image is empty";
        return;
    }
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    if (a.format == ImageFormat::Float32) {
        for (size_t i = 0; i < total_pixels; ++i) {
            double val_a = get_pixel_value(a.data.data(), i, a.format);
            
            float result_val = static_cast<float>(1.0 - val_a);
            float* fdata = reinterpret_cast<float*>(result.data.data());
            fdata[i] = result_val;
        }
    } else {
        for (size_t i = 0; i < total_pixels; ++i) {
            result.data[i] = ~a.data[i];
        }
    }
}

void image_blend(const ImageData& a, const ImageData& b, ImageData& result,
                 double blend_weight, bool saturate) {
    if (!validate_images(a, b)) {
        OVF_ERROR() << "Image blend: images not compatible";
        return;
    }
    
    // 限制权重范围
    blend_weight = std::clamp(blend_weight, 0.0, 1.0);
    
    result.width = a.width;
    result.height = a.height;
    result.channels = a.channels;
    result.format = a.format;
    result.data.resize(a.data.size());
    
    size_t total_pixels = static_cast<size_t>(a.width) * a.height * a.channels;
    
    for (size_t i = 0; i < total_pixels; ++i) {
        double val_a = get_pixel_value(a.data.data(), i, a.format);
        double val_b = get_pixel_value(b.data.data(), i, b.format);
        
        // 混合：result = a * weight + b * (1 - weight)
        double result_val = val_a * blend_weight + val_b * (1.0 - blend_weight);
        
        set_pixel_value(result.data.data(), i, result.format, result_val, saturate);
    }
}

} // namespace arithmetic_utils

// ========== 算术运算节点实现 ==========

// 1. ImageAddNode
ImageAddNode::ImageAddNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageAddNode::make_info() {
    NodeInfo info;
    info.id = "ImageAdd";
    info.name = "图像加法";
    info.category = "图像运算/算术运算";
    info.description = "对两幅图像进行加法运算：Output = (ImageA + ImageB) * ScaleFactor + Offset";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("scale_factor", "缩放因子", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("offset", "偏移值", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("saturate", "饱和截断", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> ImageAddNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    double scale_factor = get_param("scale_factor", Data(0.5)).as_number(0.5);
    double offset = get_param("offset", Data(0.0)).as_number(0.0);
    bool saturate = get_param("saturate", Data(true)).as_bool(true);
    
    ImageData output;
    arithmetic_utils::image_add(img_a, img_b, output, scale_factor, offset, saturate);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageAdd executed: scale=" << scale_factor << ", offset=" << offset;
    
    return Result<void>::success();
}

// 2. ImageSubtractNode
ImageSubtractNode::ImageSubtractNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageSubtractNode::make_info() {
    NodeInfo info;
    info.id = "ImageSubtract";
    info.name = "图像减法";
    info.category = "图像运算/算术运算";
    info.description = "对两幅图像进行减法运算：Output = (ImageA - ImageB) * ScaleFactor + Offset";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("scale_factor", "缩放因子", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("offset", "偏移值", DataType::Number, Data(128.0)));
    info.params.push_back(ParamDef("saturate", "饱和截断", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> ImageSubtractNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    double scale_factor = get_param("scale_factor", Data(1.0)).as_number(1.0);
    double offset = get_param("offset", Data(128.0)).as_number(128.0);
    bool saturate = get_param("saturate", Data(true)).as_bool(true);
    
    ImageData output;
    arithmetic_utils::image_subtract(img_a, img_b, output, scale_factor, offset, saturate);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageSubtract executed: scale=" << scale_factor << ", offset=" << offset;
    
    return Result<void>::success();
}

// 3. ImageMultiplyNode
ImageMultiplyNode::ImageMultiplyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageMultiplyNode::make_info() {
    NodeInfo info;
    info.id = "ImageMultiply";
    info.name = "图像乘法";
    info.category = "图像运算/算术运算";
    info.description = "对两幅图像进行乘法运算：Output = (ImageA * ImageB) * ScaleFactor + Offset";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("scale_factor", "缩放因子", DataType::Number, Data(255.0)));
    info.params.push_back(ParamDef("offset", "偏移值", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("saturate", "饱和截断", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> ImageMultiplyNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    double scale_factor = get_param("scale_factor", Data(255.0)).as_number(255.0);
    double offset = get_param("offset", Data(0.0)).as_number(0.0);
    bool saturate = get_param("saturate", Data(true)).as_bool(true);
    
    ImageData output;
    arithmetic_utils::image_multiply(img_a, img_b, output, scale_factor, offset, saturate);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageMultiply executed: scale=" << scale_factor << ", offset=" << offset;
    
    return Result<void>::success();
}

// 4. ImageDivideNode
ImageDivideNode::ImageDivideNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageDivideNode::make_info() {
    NodeInfo info;
    info.id = "ImageDivide";
    info.name = "图像除法";
    info.category = "图像运算/算术运算";
    info.description = "对两幅图像进行除法运算：Output = (ImageA / ImageB) * ScaleFactor + Offset";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("scale_factor", "缩放因子", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("offset", "偏移值", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("saturate", "饱和截断", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> ImageDivideNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    double scale_factor = get_param("scale_factor", Data(1.0)).as_number(1.0);
    double offset = get_param("offset", Data(0.0)).as_number(0.0);
    bool saturate = get_param("saturate", Data(true)).as_bool(true);
    
    ImageData output;
    arithmetic_utils::image_divide(img_a, img_b, output, scale_factor, offset, saturate);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageDivide executed: scale=" << scale_factor << ", offset=" << offset;
    
    return Result<void>::success();
}

// 5. ImageAbsDiffNode
ImageAbsDiffNode::ImageAbsDiffNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageAbsDiffNode::make_info() {
    NodeInfo info;
    info.id = "ImageAbsDiff";
    info.name = "绝对差值";
    info.category = "图像运算/算术运算";
    info.description = "计算两幅图像的绝对差值：Output = |ImageA - ImageB| * ScaleFactor + Offset";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("scale_factor", "缩放因子", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("offset", "偏移值", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("saturate", "饱和截断", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> ImageAbsDiffNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    double scale_factor = get_param("scale_factor", Data(1.0)).as_number(1.0);
    double offset = get_param("offset", Data(0.0)).as_number(0.0);
    bool saturate = get_param("saturate", Data(true)).as_bool(true);
    
    ImageData output;
    arithmetic_utils::image_abs_diff(img_a, img_b, output, scale_factor, offset, saturate);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageAbsDiff executed: scale=" << scale_factor << ", offset=" << offset;
    
    return Result<void>::success();
}

// 6. ImageMinNode
ImageMinNode::ImageMinNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageMinNode::make_info() {
    NodeInfo info;
    info.id = "ImageMin";
    info.name = "图像最小值";
    info.category = "图像运算/比较运算";
    info.description = "计算两幅图像对应像素的最小值：Output = min(ImageA, ImageB)";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> ImageMinNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    ImageData output;
    arithmetic_utils::image_min(img_a, img_b, output);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageMin executed";
    
    return Result<void>::success();
}

// 7. ImageMaxNode
ImageMaxNode::ImageMaxNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageMaxNode::make_info() {
    NodeInfo info;
    info.id = "ImageMax";
    info.name = "图像最大值";
    info.category = "图像运算/比较运算";
    info.description = "计算两幅图像对应像素的最大值：Output = max(ImageA, ImageB)";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> ImageMaxNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    ImageData output;
    arithmetic_utils::image_max(img_a, img_b, output);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageMax executed";
    
    return Result<void>::success();
}

// ========== 位运算节点实现 ==========

// 8. ImageAndNode
ImageAndNode::ImageAndNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageAndNode::make_info() {
    NodeInfo info;
    info.id = "ImageAnd";
    info.name = "位与运算";
    info.category = "图像运算/位运算";
    info.description = "对两幅图像进行按位与运算：Output = ImageA & ImageB";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> ImageAndNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    ImageData output;
    arithmetic_utils::image_and(img_a, img_b, output);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageAnd executed";
    
    return Result<void>::success();
}

// 9. ImageOrNode
ImageOrNode::ImageOrNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageOrNode::make_info() {
    NodeInfo info;
    info.id = "ImageOr";
    info.name = "位或运算";
    info.category = "图像运算/位运算";
    info.description = "对两幅图像进行按位或运算：Output = ImageA | ImageB";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> ImageOrNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    ImageData output;
    arithmetic_utils::image_or(img_a, img_b, output);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageOr executed";
    
    return Result<void>::success();
}

// 10. ImageXorNode
ImageXorNode::ImageXorNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageXorNode::make_info() {
    NodeInfo info;
    info.id = "ImageXor";
    info.name = "位异或运算";
    info.category = "图像运算/位运算";
    info.description = "对两幅图像进行按位异或运算：Output = ImageA ^ ImageB";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> ImageXorNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    ImageData output;
    arithmetic_utils::image_xor(img_a, img_b, output);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageXor executed";
    
    return Result<void>::success();
}

// 11. ImageNotNode
ImageNotNode::ImageNotNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageNotNode::make_info() {
    NodeInfo info;
    info.id = "ImageNot";
    info.name = "位非运算";
    info.category = "图像运算/位运算";
    info.description = "对图像进行按位非运算：Output = ~Image（对于浮点图像：Output = 1 - Image）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    return info;
}

Result<void> ImageNotNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData output;
    arithmetic_utils::image_not(input, output);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageNot executed";
    
    return Result<void>::success();
}

// ========== 混合运算节点实现 ==========

// 12. ImageBlendNode
ImageBlendNode::ImageBlendNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageBlendNode::make_info() {
    NodeInfo info;
    info.id = "ImageBlend";
    info.name = "图像混合";
    info.category = "图像运算/混合运算";
    info.description = "按权重混合两幅图像：Output = ImageA * Weight + ImageB * (1 - Weight)";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image_a", "图像A", DataType::Image, true));
    info.inputs.push_back(DataPort("image_b", "图像B", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("blend_weight", "混合权重", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("saturate", "饱和截断", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> ImageBlendNode::execute(FlowContext& context) {
    auto input_a = get_input("image_a");
    auto input_b = get_input("image_b");
    
    if (!input_a.is_image() || !input_b.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Inputs are not images");
    }
    
    ImageData img_a = input_a.as_image();
    ImageData img_b = input_b.as_image();
    
    if (img_a.empty() || img_b.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    double blend_weight = get_param("blend_weight", Data(0.5)).as_number(0.5);
    bool saturate = get_param("saturate", Data(true)).as_bool(true);
    
    ImageData output;
    arithmetic_utils::image_blend(img_a, img_b, output, blend_weight, saturate);
    
    set_output("image", Data(output));
    OVF_INFO() << "ImageBlend executed: weight=" << blend_weight;
    
    return Result<void>::success();
}

// ========== 节点注册 ==========

OVF_REGISTER_NODE(ImageAddNode, "ImageAdd", ImageAddNode::make_info())
OVF_REGISTER_NODE(ImageSubtractNode, "ImageSubtract", ImageSubtractNode::make_info())
OVF_REGISTER_NODE(ImageMultiplyNode, "ImageMultiply", ImageMultiplyNode::make_info())
OVF_REGISTER_NODE(ImageDivideNode, "ImageDivide", ImageDivideNode::make_info())
OVF_REGISTER_NODE(ImageAbsDiffNode, "ImageAbsDiff", ImageAbsDiffNode::make_info())
OVF_REGISTER_NODE(ImageMinNode, "ImageMin", ImageMinNode::make_info())
OVF_REGISTER_NODE(ImageMaxNode, "ImageMax", ImageMaxNode::make_info())

OVF_REGISTER_NODE(ImageAndNode, "ImageAnd", ImageAndNode::make_info())
OVF_REGISTER_NODE(ImageOrNode, "ImageOr", ImageOrNode::make_info())
OVF_REGISTER_NODE(ImageXorNode, "ImageXor", ImageXorNode::make_info())
OVF_REGISTER_NODE(ImageNotNode, "ImageNot", ImageNotNode::make_info())

OVF_REGISTER_NODE(ImageBlendNode, "ImageBlend", ImageBlendNode::make_info())

} // namespace algorithm
} // namespace ovf