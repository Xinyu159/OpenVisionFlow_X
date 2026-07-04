#include "ovf/algorithm/geometry.h"
#include "ovf/algorithm/image_utils.h"
#include "ovf/core/logger.h"
#include <cmath>
#include <algorithm>

namespace ovf {
namespace algorithm {

// ========== RotateNode Implementation ==========

RotateNode::RotateNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo RotateNode::make_info() {
    NodeInfo info;
    info.id = "Rotate";
    info.name = "图像旋转";
    info.category = "几何变换";
    info.description = "图像旋转节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "旋转后的图像", DataType::Image));
    info.outputs.push_back(DataPort("new_width", "新图像宽度", DataType::Number));
    info.outputs.push_back(DataPort("new_height", "新图像高度", DataType::Number));
    
    info.params.push_back(ParamDef("angle", "旋转角度", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("center_x", "旋转中心X", DataType::Number, Data(-1.0f)));
    info.params.push_back(ParamDef("center_y", "旋转中心Y", DataType::Number, Data(-1.0f)));
    info.params.push_back(ParamDef("expand_canvas", "扩展画布", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> RotateNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    angle_ = get_param("angle", Data(0.0f)).as_number();
    center_x_ = get_param("center_x", Data(-1.0f)).as_number();
    center_y_ = get_param("center_y", Data(-1.0f)).as_number();
    expand_canvas_ = get_param("expand_canvas", Data(true)).as_bool();

    int src_w = input.width;
    int src_h = input.height;
    int channels = input.channels;

    float rad = angle_ * 3.14159265f / 180.0f;
    float cos_a = std::cos(rad);
    float sin_a = std::sin(rad);

    // 计算旋转中心
    float cx = center_x_ < 0 ? src_w / 2.0f : center_x_;
    float cy = center_y_ < 0 ? src_h / 2.0f : center_y_;

    // 计算输出图像大小
    int dst_w, dst_h;
    if (expand_canvas_) {
        // 计算旋转后四个角的位置
        float corners[4][2] = {
            {0 - cx, 0 - cy},
            {src_w - 1 - cx, 0 - cy},
            {0 - cx, src_h - 1 - cy},
            {src_w - 1 - cx, src_h - 1 - cy}
        };

        float min_x = corners[0][0] * cos_a - corners[0][1] * sin_a;
        float max_x = min_x;
        float min_y = corners[0][0] * sin_a + corners[0][1] * cos_a;
        float max_y = min_y;

        for (int i = 1; i < 4; ++i) {
            float rx = corners[i][0] * cos_a - corners[i][1] * sin_a;
            float ry = corners[i][0] * sin_a + corners[i][1] * cos_a;
            min_x = std::min(min_x, rx);
            max_x = std::max(max_x, rx);
            min_y = std::min(min_y, ry);
            max_y = std::max(max_y, ry);
        }

        dst_w = static_cast<int>(std::ceil(max_x - min_x));
        dst_h = static_cast<int>(std::ceil(max_y - min_y));

        // 新的旋转中心
        cx = (src_w - 1) / 2.0f;
        cy = (src_h - 1) / 2.0f;
        float new_cx = dst_w / 2.0f;
        float new_cy = dst_h / 2.0f;
        cx = new_cx;
        cy = new_cy;
    } else {
        dst_w = src_w;
        dst_h = src_h;
        cx = center_x_ < 0 ? src_w / 2.0f : center_x_;
        cy = center_y_ < 0 ? src_h / 2.0f : center_y_;
    }

    // 创建输出图像
    ImageData output;
    output.width = dst_w;
    output.height = dst_h;
    output.channels = channels;
    output.format = input.format;
    output.data.resize(dst_w * dst_h * channels, 0);

    // 旋转变换
    for (int dy = 0; dy < dst_h; ++dy) {
        for (int dx = 0; dx < dst_w; ++dx) {
            // 反向映射
            float sx = (dx - cx) * cos_a + (dy - cy) * sin_a + src_w / 2.0f;
            float sy = -(dx - cx) * sin_a + (dy - cy) * cos_a + src_h / 2.0f;

            if (sx >= 0 && sx < src_w - 1 && sy >= 0 && sy < src_h - 1) {
                // 双线性插值
                int x0 = static_cast<int>(sx);
                int y0 = static_cast<int>(sy);
                int x1 = x0 + 1;
                int y1 = y0 + 1;

                float fx = sx - x0;
                float fy = sy - y0;

                int dst_idx = (dy * dst_w + dx) * channels;

                for (int c = 0; c < channels; ++c) {
                    float v00 = input.data[(y0 * src_w + x0) * channels + c];
                    float v01 = input.data[(y0 * src_w + x1) * channels + c];
                    float v10 = input.data[(y1 * src_w + x0) * channels + c];
                    float v11 = input.data[(y1 * src_w + x1) * channels + c];

                    float v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                              (1 - fx) * fy * v10 + fx * fy * v11;
                    output.data[dst_idx + c] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
                }
            }
        }
    }

    set_output("image", Data(output));
    set_output("new_width", Data(static_cast<int32_t>(output.width)));
    set_output("new_height", Data(static_cast<int32_t>(output.height)));
    
    OVF_INFO() << "Rotate completed: angle=" << angle_ << " degrees";

    return Result<void>::success();
}

// ========== ResizeNode Implementation ==========

ResizeNode::ResizeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ResizeNode::make_info() {
    NodeInfo info;
    info.id = "Resize";
    info.name = "图像缩放";
    info.category = "几何变换";
    info.description = "图像缩放节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "缩放后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("width", "目标宽度", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("height", "目标高度", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("scale_x", "水平缩放比例", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("scale_y", "垂直缩放比例", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("interpolation", "插值方法", DataType::String, Data("bilinear")));
    
    return info;
}

Result<void> ResizeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    new_width_ = static_cast<int>(get_param("width", Data(0)).as_int());
    new_height_ = static_cast<int>(get_param("height", Data(0)).as_int());
    scale_x_ = get_param("scale_x", Data(1.0f)).as_number();
    scale_y_ = get_param("scale_y", Data(1.0f)).as_number();
    interpolation_ = get_param("interpolation", Data("bilinear")).as_string();

    use_scale_ = (new_width_ <= 0 || new_height_ <= 0);

    int src_w = input.width;
    int src_h = input.height;
    int channels = input.channels;

    int dst_w, dst_h;
    if (use_scale_) {
        dst_w = static_cast<int>(src_w * scale_x_);
        dst_h = static_cast<int>(src_h * scale_y_);
    } else {
        dst_w = new_width_;
        dst_h = new_height_;
    }

    if (dst_w < 1) dst_w = 1;
    if (dst_h < 1) dst_h = 1;

    // 创建输出图像
    ImageData output;
    output.width = dst_w;
    output.height = dst_h;
    output.channels = channels;
    output.format = input.format;
    output.data.resize(dst_w * dst_h * channels);

    if (interpolation_ == "nearest") {
        image_utils::resize_nearest(input.data.data(), src_w, src_h, channels,
                                     output.data, dst_w, dst_h);
    } else {
        image_utils::resize_bilinear(input.data.data(), src_w, src_h, channels,
                                      output.data, dst_w, dst_h);
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Resize completed: " << src_w << "x" << src_h << " -> " << dst_w << "x" << dst_h;

    return Result<void>::success();
}

// ========== TranslateNode Implementation ==========

TranslateNode::TranslateNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo TranslateNode::make_info() {
    NodeInfo info;
    info.id = "Translate";
    info.name = "图像平移";
    info.category = "几何变换";
    info.description = "图像平移节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "平移后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("offset_x", "水平偏移", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("offset_y", "垂直偏移", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("fill_value", "填充值", DataType::Number, Data(0)));
    
    return info;
}

Result<void> TranslateNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    offset_x_ = static_cast<int>(get_param("offset_x", Data(0)).as_int());
    offset_y_ = static_cast<int>(get_param("offset_y", Data(0)).as_int());
    fill_value_ = static_cast<uint8_t>(get_param("fill_value", Data(0)).as_int());

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 创建输出图像
    ImageData output = input;
    output.data.assign(w * h * ch, fill_value_);

    // 平移
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int src_x = x - offset_x_;
            int src_y = y - offset_y_;

            if (src_x >= 0 && src_x < w && src_y >= 0 && src_y < h) {
                int dst_idx = (y * w + x) * ch;
                int src_idx = (src_y * w + src_x) * ch;
                for (int c = 0; c < ch; ++c) {
                    output.data[dst_idx + c] = input.data[src_idx + c];
                }
            }
        }
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Translate completed: offset=(" << offset_x_ << "," << offset_y_ << ")";

    return Result<void>::success();
}

// ========== AffineTransformNode Implementation ==========

AffineTransformNode::AffineTransformNode(const String& instance_id)
    : INode(instance_id, make_info()) {
    transform_matrix_ = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}; // 单位矩阵
}

NodeInfo AffineTransformNode::make_info() {
    NodeInfo info;
    info.id = "AffineTransform";
    info.name = "仿射变换";
    info.category = "几何变换";
    info.description = "仿射变换节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "变换后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("matrix", "变换矩阵2x3", DataType::String, Data("[1,0,0,0,1,0]")));
    
    return info;
}

void AffineTransformNode::set_matrix(const std::vector<float>& matrix) {
    if (matrix.size() == 6) {
        transform_matrix_ = matrix;
    }
}

Result<void> AffineTransformNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int src_w = input.width;
    int src_h = input.height;
    int ch = input.channels;

    // 计算变换后的图像范围
    float a = transform_matrix_[0];
    float b = transform_matrix_[1];
    float c = transform_matrix_[2];
    float d = transform_matrix_[3];
    float e = transform_matrix_[4];
    float f = transform_matrix_[5];

    // 计算四个角点
    std::vector<std::pair<float, float>> corners = {
        {c, f},                           // (0,0)
        {a * (src_w - 1) + c, e * (src_w - 1) + f},  // (w-1,0)
        {b * (src_h - 1) + c, d * (src_h - 1) + f},  // (0,h-1)
        {a * (src_w - 1) + b * (src_h - 1) + c, e * (src_w - 1) + d * (src_h - 1) + f}  // (w-1,h-1)
    };

    float min_x = corners[0].first, max_x = corners[0].first;
    float min_y = corners[0].second, max_y = corners[0].second;

    for (const auto& corner : corners) {
        min_x = std::min(min_x, corner.first);
        max_x = std::max(max_x, corner.first);
        min_y = std::min(min_y, corner.second);
        max_y = std::max(max_y, corner.second);
    }

    int dst_w = static_cast<int>(std::ceil(max_x - min_x));
    int dst_h = static_cast<int>(std::ceil(max_y - min_y));

    // 创建输出图像
    ImageData output;
    output.width = dst_w;
    output.height = dst_h;
    output.channels = ch;
    output.format = input.format;
    output.data.resize(dst_w * dst_h * ch, 0);

    // 计算逆矩阵（3x3但只用前两行）
    float det = a * d - b * e;
    if (std::abs(det) < 1e-10) {
        // 变换不可逆，返回原图
        output = input;
        set_output("image", Data(output));
        return Result<void>::success();
    }

    float inv_a = d / det;
    float inv_b = -b / det;
    float inv_c = -e / det;
    float inv_d = a / det;
    float inv_tx = -(d * c - b * f) / det;
    float inv_ty = -(-e * c + a * f) / det;

    // 应用变换
    for (int dy = 0; dy < dst_h; ++dy) {
        for (int dx = 0; dx < dst_w; ++dx) {
            // 反向映射
            float px = dx + min_x;
            float py = dy + min_y;

            float sx = inv_a * px + inv_b * py + inv_tx;
            float sy = inv_c * px + inv_d * py + inv_ty;

            if (sx >= 0 && sx < src_w - 1 && sy >= 0 && sy < src_h - 1) {
                int x0 = static_cast<int>(sx);
                int y0 = static_cast<int>(sy);
                int x1 = x0 + 1;
                int y1 = y0 + 1;

                float fx = sx - x0;
                float fy = sy - y0;

                int dst_idx = (dy * dst_w + dx) * ch;

                for (int c = 0; c < ch; ++c) {
                    float v00 = input.data[(y0 * src_w + x0) * ch + c];
                    float v01 = input.data[(y0 * src_w + x1) * ch + c];
                    float v10 = input.data[(y1 * src_w + x0) * ch + c];
                    float v11 = input.data[(y1 * src_w + x1) * ch + c];

                    float v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                              (1 - fx) * fy * v10 + fx * fy * v11;
                    output.data[dst_idx + c] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
                }
            }
        }
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Affine transform completed";

    return Result<void>::success();
}

// ========== PerspectiveTransformNode Implementation ==========

PerspectiveTransformNode::PerspectiveTransformNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PerspectiveTransformNode::make_info() {
    NodeInfo info;
    info.id = "PerspectiveTransform";
    info.name = "透视变换";
    info.category = "几何变换";
    info.description = "透视变换节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "变换后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("src_points", "源点(4点)", DataType::String, Data("[0,0,w-1,0,0,h-1,w-1,h-1]")));
    info.params.push_back(ParamDef("dst_points", "目标点(4点)", DataType::String, Data("[0,0,w,0,0,h,w,h]")));
    
    return info;
}

Result<void> PerspectiveTransformNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 需要计算透视变换矩阵
    // 简化实现：假设目标矩形
    int dst_w = static_cast<int>(dst_points_[6] - dst_points_[0]);
    int dst_h = static_cast<int>(dst_points_[5] - dst_points_[1]);

    if (dst_w < 1) dst_w = 1;
    if (dst_h < 1) dst_h = 1;

    int src_w = input.width;
    int src_h = input.height;
    int ch = input.channels;

    // 创建输出图像
    ImageData output;
    output.width = dst_w;
    output.height = dst_h;
    output.channels = ch;
    output.format = input.format;
    output.data.resize(dst_w * dst_h * ch, 0);

    // 简化的透视变换：假设src_points是四边形角点
    float x0 = src_points_[0], y0 = src_points_[1];
    float x1 = src_points_[2], y1 = src_points_[3];
    float x2 = src_points_[4], y2 = src_points_[5];
    float x3 = src_points_[6], y3 = src_points_[7];

    for (int dy = 0; dy < dst_h; ++dy) {
        for (int dx = 0; dx < dst_w; ++dx) {
            // 计算在源四边形中的对应点
            float u = static_cast<float>(dx) / dst_w;
            float v = static_cast<float>(dy) / dst_h;

            // 双线性插值四边形
            float sx = (1 - u) * (1 - v) * x0 + u * (1 - v) * x1 +
                       (1 - u) * v * x2 + u * v * x3;
            float sy = (1 - u) * (1 - v) * y0 + u * (1 - v) * y1 +
                       (1 - u) * v * y2 + u * v * y3;

            if (sx >= 0 && sx < src_w - 1 && sy >= 0 && sy < src_h - 1) {
                int x0_src = static_cast<int>(sx);
                int y0_src = static_cast<int>(sy);
                int x1_src = x0_src + 1;
                int y1_src = y0_src + 1;

                float fx = sx - x0_src;
                float fy = sy - y0_src;

                int dst_idx = (dy * dst_w + dx) * ch;

                for (int c = 0; c < ch; ++c) {
                    float v00 = input.data[(y0_src * src_w + x0_src) * ch + c];
                    float v01 = input.data[(y0_src * src_w + x1_src) * ch + c];
                    float v10 = input.data[(y1_src * src_w + x0_src) * ch + c];
                    float v11 = input.data[(y1_src * src_w + x1_src) * ch + c];

                    float v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                              (1 - fx) * fy * v10 + fx * fy * v11;
                    output.data[dst_idx + c] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
                }
            }
        }
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Perspective transform completed";

    return Result<void>::success();
}

// ========== FlipNode Implementation ==========

FlipNode::FlipNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FlipNode::make_info() {
    NodeInfo info;
    info.id = "Flip";
    info.name = "图像翻转";
    info.category = "几何变换";
    info.description = "图像翻转节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "翻转后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("horizontal", "水平翻转", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("vertical", "垂直翻转", DataType::Boolean, Data(false)));
    
    return info;
}

Result<void> FlipNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    horizontal_ = get_param("horizontal", Data(true)).as_bool();
    vertical_ = get_param("vertical", Data(false)).as_bool();

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 创建输出图像
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = ch;
    output.format = input.format;
    output.data.resize(w * h * ch);

    image_utils::flip(input.data.data(), w, h, ch, output.data, horizontal_, vertical_);

    set_output("image", Data(output));
    
    OVF_INFO() << "Flip completed: horizontal=" << horizontal_ << ", vertical=" << vertical_;

    return Result<void>::success();
}

// ========== CropNode Implementation ==========

CropNode::CropNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo CropNode::make_info() {
    NodeInfo info;
    info.id = "Crop";
    info.name = "图像裁剪";
    info.category = "几何变换";
    info.description = "图像裁剪节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "裁剪后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("roi_x", "ROI起始X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("roi_y", "ROI起始Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(100)));
    
    return info;
}

Result<void> CropNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    roi_x_ = static_cast<int>(get_param("roi_x", Data(0)).as_int());
    roi_y_ = static_cast<int>(get_param("roi_y", Data(0)).as_int());
    roi_width_ = static_cast<int>(get_param("roi_width", Data(100)).as_int());
    roi_height_ = static_cast<int>(get_param("roi_height", Data(100)).as_int());

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 边界检查
    if (roi_x_ < 0) roi_x_ = 0;
    if (roi_y_ < 0) roi_y_ = 0;
    if (roi_x_ + roi_width_ > w) roi_width_ = w - roi_x_;
    if (roi_y_ + roi_height_ > h) roi_height_ = h - roi_y_;

    if (roi_width_ <= 0 || roi_height_ <= 0) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Invalid ROI dimensions");
    }

    // 创建输出图像
    ImageData output;
    output.width = roi_width_;
    output.height = roi_height_;
    output.channels = ch;
    output.format = input.format;
    output.data.resize(roi_width_ * roi_height_ * ch);

    for (int y = 0; y < roi_height_; ++y) {
        for (int x = 0; x < roi_width_; ++x) {
            int src_idx = ((roi_y_ + y) * w + (roi_x_ + x)) * ch;
            int dst_idx = (y * roi_width_ + x) * ch;
            for (int c = 0; c < ch; ++c) {
                output.data[dst_idx + c] = input.data[src_idx + c];
            }
        }
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Crop completed: roi=(" << roi_x_ << "," << roi_y_ << ") size=" << roi_width_ << "x" << roi_height_;

    return Result<void>::success();
}

// ========== RemapNode Implementation ==========

RemapNode::RemapNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo RemapNode::make_info() {
    NodeInfo info;
    info.id = "Remap";
    info.name = "坐标映射";
    info.category = "几何变换";
    info.description = "坐标映射节点（自定义变换）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("map_x", "X坐标映射", DataType::Image, false));
    info.inputs.push_back(DataPort("map_y", "Y坐标映射", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "映射后的图像", DataType::Image));
    
    return info;
}

Result<void> RemapNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int src_w = input.width;
    int src_h = input.height;
    int ch = input.channels;

    int dst_w = map_width_ > 0 ? map_width_ : src_w;
    int dst_h = map_height_ > 0 ? map_height_ : src_h;

    // 如果没有映射数据，返回原图
    if (map_x_.empty()) {
        set_output("image", Data(input));
        return Result<void>::success();
    }

    // 创建输出图像
    ImageData output;
    output.width = dst_w;
    output.height = dst_h;
    output.channels = ch;
    output.format = input.format;
    output.data.resize(dst_w * dst_h * ch, 0);

    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            float sx = map_x_[y * dst_w + x];
            float sy = map_y_[y * dst_w + x];

            if (sx >= 0 && sx < src_w - 1 && sy >= 0 && sy < src_h - 1) {
                int x0 = static_cast<int>(sx);
                int y0 = static_cast<int>(sy);
                int x1 = x0 + 1;
                int y1 = y0 + 1;

                float fx = sx - x0;
                float fy = sy - y0;

                int dst_idx = (y * dst_w + x) * ch;

                for (int c = 0; c < ch; ++c) {
                    float v00 = input.data[(y0 * src_w + x0) * ch + c];
                    float v01 = input.data[(y0 * src_w + x1) * ch + c];
                    float v10 = input.data[(y1 * src_w + x0) * ch + c];
                    float v11 = input.data[(y1 * src_w + x1) * ch + c];

                    float v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                              (1 - fx) * fy * v10 + fx * fy * v11;
                    output.data[dst_idx + c] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
                }
            }
        }
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Remap completed";

    return Result<void>::success();
}

// ========== GeometricCalibrationNode Implementation ==========

GeometricCalibrationNode::GeometricCalibrationNode(const String& instance_id)
    : INode(instance_id, make_info()) {
    calibration_matrix_ = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
}

NodeInfo GeometricCalibrationNode::make_info() {
    NodeInfo info;
    info.id = "GeometricCalibration";
    info.name = "几何校准";
    info.category = "几何变换";
    info.description = "几何校准节点（标定）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "校准后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("pixel_size_x", "像素尺寸X", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("pixel_size_y", "像素尺寸Y", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("distortion_k1", "畸变系数k1", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("distortion_k2", "畸变系数k2", DataType::Number, Data(0.0f)));
    
    return info;
}

Result<void> GeometricCalibrationNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    pixel_size_x_ = get_param("pixel_size_x", Data(1.0f)).as_number();
    pixel_size_y_ = get_param("pixel_size_y", Data(1.0f)).as_number();
    distortion_k1_ = get_param("distortion_k1", Data(0.0f)).as_number();
    distortion_k2_ = get_param("distortion_k2", Data(0.0f)).as_number();

    int w = input.width;
    int h = input.height;
    int ch = input.channels;

    // 创建输出图像（应用畸变校正）
    ImageData output = input;

    float cx = w / 2.0f;
    float cy = h / 2.0f;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // 计算归一化坐标
            float xn = (x - cx) / cx;
            float yn = (y - cy) / cy;

            // 计算畸变
            float r2 = xn * xn + yn * yn;
            float r4 = r2 * r2;
            float distortion = 1.0f + distortion_k1_ * r2 + distortion_k2_ * r4;

            // 畸变校正后的坐标
            float xn_corrected = xn * distortion;
            float yn_corrected = yn * distortion;

            // 转换回像素坐标
            float src_x = xn_corrected * cx + cx;
            float src_y = yn_corrected * cy + cy;

            if (src_x >= 0 && src_x < w - 1 && src_y >= 0 && src_y < h - 1) {
                int x0 = static_cast<int>(src_x);
                int y0 = static_cast<int>(src_y);
                int x1 = x0 + 1;
                int y1 = y0 + 1;

                float fx = src_x - x0;
                float fy = src_y - y0;

                int dst_idx = (y * w + x) * ch;

                for (int c = 0; c < ch; ++c) {
                    float v00 = input.data[(y0 * w + x0) * ch + c];
                    float v01 = input.data[(y0 * w + x1) * ch + c];
                    float v10 = input.data[(y1 * w + x0) * ch + c];
                    float v11 = input.data[(y1 * w + x1) * ch + c];

                    float v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                              (1 - fx) * fy * v10 + fx * fy * v11;
                    output.data[dst_idx + c] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
                }
            }
        }
    }

    set_output("image", Data(output));
    
    OVF_INFO() << "Geometric calibration completed";

    return Result<void>::success();
}

// ========== Node Registration ==========

OVF_REGISTER_NODE(RotateNode, "Rotate", RotateNode::make_info())
OVF_REGISTER_NODE(ResizeNode, "Resize", ResizeNode::make_info())
OVF_REGISTER_NODE(TranslateNode, "Translate", TranslateNode::make_info())
OVF_REGISTER_NODE(AffineTransformNode, "AffineTransform", AffineTransformNode::make_info())
OVF_REGISTER_NODE(PerspectiveTransformNode, "PerspectiveTransform", PerspectiveTransformNode::make_info())
OVF_REGISTER_NODE(FlipNode, "Flip", FlipNode::make_info())
OVF_REGISTER_NODE(CropNode, "Crop", CropNode::make_info())
OVF_REGISTER_NODE(RemapNode, "Remap", RemapNode::make_info())
OVF_REGISTER_NODE(GeometricCalibrationNode, "GeometricCalibration", GeometricCalibrationNode::make_info())

} // namespace algorithm
} // namespace ovf