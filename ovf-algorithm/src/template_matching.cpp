/**
 * @file template_matching.cpp
 * @brief 模板匹配节点实现
 */

#include "ovf/algorithm/template_matching.h"
#include "ovf/algorithm/image_utils.h"
#include "ovf/core/logger.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace ovf {
namespace algorithm {

// ========== Template Utils Implementation ==========

namespace template_utils {

ErrorCode match_sad(const uint8_t* src, int src_width, int src_height,
                    const uint8_t* tmpl, int tmpl_width, int tmpl_height,
                    MatchResult& result) {
    if (!src || !tmpl) {
        return ErrorCode::InvalidParameter;
    }
    if (tmpl_width > src_width || tmpl_height > src_height) {
        return ErrorCode::InvalidParameter;
    }

    int search_width = src_width - tmpl_width + 1;
    int search_height = src_height - tmpl_height + 1;

    uint32_t min_sad = std::numeric_limits<uint32_t>::max();
    int best_x = 0, best_y = 0;

    for (int y = 0; y < search_height; ++y) {
        for (int x = 0; x < search_width; ++x) {
            uint32_t sad = 0;
            for (int ty = 0; ty < tmpl_height; ++ty) {
                for (int tx = 0; tx < tmpl_width; ++tx) {
                    int s_idx = (y + ty) * src_width + (x + tx);
                    int t_idx = ty * tmpl_width + tx;
                    int diff = static_cast<int>(src[s_idx]) - static_cast<int>(tmpl[t_idx]);
                    sad += static_cast<uint32_t>(std::abs(diff));
                }
            }

            if (sad < min_sad) {
                min_sad = sad;
                best_x = x;
                best_y = y;
            }
        }
    }

    result.x = best_x;
    result.y = best_y;
    result.score = static_cast<float>(min_sad);
    result.angle = 0.0f;
    result.scale = 1.0f;

    return ErrorCode::Success;
}

ErrorCode match_ncc(const uint8_t* src, int src_width, int src_height,
                    const uint8_t* tmpl, int tmpl_width, int tmpl_height,
                    MatchResult& result) {
    if (!src || !tmpl) {
        return ErrorCode::InvalidParameter;
    }
    if (tmpl_width > src_width || tmpl_height > src_height) {
        return ErrorCode::InvalidParameter;
    }

    int search_width = src_width - tmpl_width + 1;
    int search_height = src_height - tmpl_height + 1;
    int tmpl_size = tmpl_width * tmpl_height;

    double tmpl_sum = 0.0;
    for (int i = 0; i < tmpl_size; ++i) {
        tmpl_sum += tmpl[i];
    }
    double tmpl_mean = tmpl_sum / tmpl_size;

    double tmpl_var = 0.0;
    for (int i = 0; i < tmpl_size; ++i) {
        double diff = tmpl[i] - tmpl_mean;
        tmpl_var += diff * diff;
    }
    double tmpl_std = std::sqrt(tmpl_var);

    float max_ncc = -1.0f;
    int best_x = 0, best_y = 0;

    for (int y = 0; y < search_height; ++y) {
        for (int x = 0; x < search_width; ++x) {
            double src_sum = 0.0;
            for (int ty = 0; ty < tmpl_height; ++ty) {
                for (int tx = 0; tx < tmpl_width; ++tx) {
                    int s_idx = (y + ty) * src_width + (x + tx);
                    src_sum += src[s_idx];
                }
            }
            double src_mean = src_sum / tmpl_size;

            double numerator = 0.0;
            double src_var = 0.0;

            for (int ty = 0; ty < tmpl_height; ++ty) {
                for (int tx = 0; tx < tmpl_width; ++tx) {
                    int s_idx = (y + ty) * src_width + (x + tx);
                    int t_idx = ty * tmpl_width + tx;

                    double src_diff = src[s_idx] - src_mean;
                    double tmpl_diff = tmpl[t_idx] - tmpl_mean;

                    numerator += src_diff * tmpl_diff;
                    src_var += src_diff * src_diff;
                }
            }

            double src_std = std::sqrt(src_var);
            double denominator = src_std * tmpl_std;

            float ncc = -1.0f;
            if (denominator > 1e-10) {
                ncc = static_cast<float>(numerator / denominator);
            }

            if (ncc > max_ncc) {
                max_ncc = ncc;
                best_x = x;
                best_y = y;
            }
        }
    }

    result.x = best_x;
    result.y = best_y;
    result.score = max_ncc;
    result.angle = 0.0f;
    result.scale = 1.0f;

    return ErrorCode::Success;
}

} // namespace template_utils

// ========== TemplateMatchNode Implementation ==========

TemplateMatchNode::TemplateMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {
    result_.x = 0;
    result_.y = 0;
    result_.score = 0.0f;
    result_.angle = 0.0f;
    result_.scale = 1.0f;
}

NodeInfo TemplateMatchNode::make_info() {
    NodeInfo info;
    info.id = "TemplateMatch";
    info.name = "模板匹配";
    info.category = "图像分析";
    info.description = "模板匹配节点，支持SAD/SSD/NCC方法";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, false));
    
    info.outputs.push_back(DataPort("result_x", "匹配位置X", DataType::Number));
    info.outputs.push_back(DataPort("result_y", "匹配位置Y", DataType::Number));
    info.outputs.push_back(DataPort("score", "匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("method", "匹配方法", DataType::String, Data("NCC")));
    info.params.push_back(ParamDef("threshold", "匹配阈值", DataType::Number, Data(0.7f)));
    
    return info;
}

Result<void> TemplateMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    auto tmpl_data = get_input("template");
    if (!tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template is not an image");
    }
    
    ImageData tmpl = tmpl_data.as_image();
    if (tmpl.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Template image is empty");
    }

    // 转换为灰度
    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;

    if (input.channels == 1) {
        src_gray = input.data;
        src_w = input.width;
        src_h = input.height;
    } else {
        src_w = input.width;
        src_h = input.height;
        src_gray.resize(src_w * src_h);
        for (size_t i = 0; i < src_gray.size(); ++i) {
            size_t idx = i * input.channels;
            src_gray[i] = static_cast<uint8_t>(
                (input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    if (tmpl.channels == 1) {
        tmpl_gray = tmpl.data;
        tmpl_w = tmpl.width;
        tmpl_h = tmpl.height;
    } else {
        tmpl_w = tmpl.width;
        tmpl_h = tmpl.height;
        tmpl_gray.resize(tmpl_w * tmpl_h);
        for (size_t i = 0; i < tmpl_gray.size(); ++i) {
            size_t idx = i * tmpl.channels;
            tmpl_gray[i] = static_cast<uint8_t>(
                (tmpl.data[idx] + tmpl.data[idx + 1] + tmpl.data[idx + 2]) / 3);
        }
    }

    // 获取参数
    String method = get_param("method", Data("NCC")).as_string();
    MatchMethod match_method = MatchMethod::NCC;
    if (method == "SAD") match_method = MatchMethod::SAD;
    else if (method == "SSD") match_method = MatchMethod::SSD;

    ErrorCode err;
    switch (match_method) {
        case MatchMethod::SAD:
            err = template_utils::match_sad(src_gray.data(), src_w, src_h,
                                            tmpl_gray.data(), tmpl_w, tmpl_h, result_);
            break;
        case MatchMethod::NCC:
        default:
            err = template_utils::match_ncc(src_gray.data(), src_w, src_h,
                                            tmpl_gray.data(), tmpl_w, tmpl_h, result_);
            break;
    }

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Template matching failed");
    }

    // 创建输出图像（在匹配位置绘制矩形）
    ImageData output = input;
    image_utils::draw_rect(output, result_.x, result_.y, tmpl_w, tmpl_h, 0, 255, 0);

    // 设置输出
    set_output("result_x", Data(result_.x));
    set_output("result_y", Data(result_.y));
    set_output("score", Data(result_.score));
    set_output("image", Data(output));

    OVF_INFO() << "Template match: position=(" << result_.x << "," << result_.y 
              << "), score=" << result_.score;

    return Result<void>::success();
}

// ========== TemplateTrainNode Implementation ==========

TemplateTrainNode::TemplateTrainNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo TemplateTrainNode::make_info() {
    NodeInfo info;
    info.id = "TemplateTrain";
    info.name = "模板训练";
    info.category = "图像分析";
    info.description = "模板训练节点，从图像中提取模板";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("template", "模板图像", DataType::Image));
    info.outputs.push_back(DataPort("width", "模板宽度", DataType::Number));
    info.outputs.push_back(DataPort("height", "模板高度", DataType::Number));
    
    info.params.push_back(ParamDef("roi_x", "ROI起始X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("roi_y", "ROI起始Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(100)));
    
    return info;
}

Result<void> TemplateTrainNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int roi_x = static_cast<int>(get_param("roi_x", Data(0)).as_int());
    int roi_y = static_cast<int>(get_param("roi_y", Data(0)).as_int());
    int roi_width = static_cast<int>(get_param("roi_width", Data(100)).as_int());
    int roi_height = static_cast<int>(get_param("roi_height", Data(100)).as_int());

    // 边界检查
    if (roi_x < 0 || roi_y < 0 ||
        roi_x + roi_width > input.width ||
        roi_y + roi_height > input.height) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "ROI超出图像边界");
    }

    // 提取模板
    template_image_.width = roi_width;
    template_image_.height = roi_height;
    template_image_.channels = input.channels;
    template_image_.format = input.format;
    template_image_.data.resize(roi_width * roi_height * input.channels);

    for (int y = 0; y < roi_height; ++y) {
        for (int x = 0; x < roi_width; ++x) {
            size_t src_idx = static_cast<size_t>((roi_y + y) * input.width + (roi_x + x)) * input.channels;
            size_t dst_idx = static_cast<size_t>(y * roi_width + x) * input.channels;
            for (int c = 0; c < input.channels; ++c) {
                template_image_.data[dst_idx + c] = input.data[src_idx + c];
            }
        }
    }

    set_output("template", Data(template_image_));
    set_output("width", Data(static_cast<int>(template_image_.width)));
    set_output("height", Data(static_cast<int>(template_image_.height)));

    OVF_INFO() << "Template extracted: size=" << template_image_.width << "x" << template_image_.height;

    return Result<void>::success();
}

// ========== MultiTemplateMatchNode Implementation ==========

MultiTemplateMatchNode::MultiTemplateMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MultiTemplateMatchNode::make_info() {
    NodeInfo info;
    info.id = "MultiTemplateMatch";
    info.name = "多模板匹配";
    info.category = "图像分析";
    info.description = "多模板匹配节点，同时匹配多个模板";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("best_match_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_match_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_match_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("method", "匹配方法", DataType::String, Data("NCC")));
    info.params.push_back(ParamDef("threshold", "匹配阈值", DataType::Number, Data(0.7f)));
    
    return info;
}

Result<void> MultiTemplateMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    if (templates_.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No templates configured");
    }

    // 转换为灰度
    std::vector<uint8_t> src_gray;
    int src_w, src_h;

    if (input.channels == 1) {
        src_gray = input.data;
        src_w = input.width;
        src_h = input.height;
    } else {
        src_w = input.width;
        src_h = input.height;
        src_gray.resize(src_w * src_h);
        for (size_t i = 0; i < src_gray.size(); ++i) {
            size_t idx = i * input.channels;
            src_gray[i] = static_cast<uint8_t>(
                (input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    String method = get_param("method", Data("NCC")).as_string();
    MatchMethod match_method = MatchMethod::NCC;
    if (method == "SAD") match_method = MatchMethod::SAD;

    MatchResult best_result;
    best_result.score = -1.0f;
    size_t best_idx = 0;

    for (size_t i = 0; i < templates_.size(); ++i) {
        std::vector<uint8_t> tmpl_gray;
        int tmpl_w, tmpl_h;

        if (templates_[i].image.channels == 1) {
            tmpl_gray = templates_[i].image.data;
            tmpl_w = templates_[i].image.width;
            tmpl_h = templates_[i].image.height;
        } else {
            tmpl_w = templates_[i].image.width;
            tmpl_h = templates_[i].image.height;
            tmpl_gray.resize(tmpl_w * tmpl_h);
            for (size_t j = 0; j < tmpl_gray.size(); ++j) {
                size_t idx = j * templates_[i].image.channels;
                tmpl_gray[j] = static_cast<uint8_t>(
                    (templates_[i].image.data[idx] + templates_[i].image.data[idx + 1] +
                     templates_[i].image.data[idx + 2]) / 3);
            }
        }

        MatchResult result;
        ErrorCode err = template_utils::match_ncc(src_gray.data(), src_w, src_h,
                                                  tmpl_gray.data(), tmpl_w, tmpl_h, result);

        if (err == ErrorCode::Success && result.score > best_result.score) {
            best_result = result;
            best_idx = i;
        }
    }

    // 创建输出
    ImageData output = input;
    if (!templates_.empty() && best_result.score > 0) {
        image_utils::draw_rect(output, best_result.x, best_result.y,
                               templates_[best_idx].image.width,
                               templates_[best_idx].image.height, 0, 255, 0);
    }

    set_output("best_match_x", Data(best_result.x));
    set_output("best_match_y", Data(best_result.y));
    set_output("best_match_score", Data(best_result.score));
    set_output("image", Data(output));

    return Result<void>::success();
}

void MultiTemplateMatchNode::add_template(const ImageData& tmpl, const String& name) {
    TemplateInfo info;
    info.image = tmpl;
    info.name = name;
    templates_.push_back(info);
}

void MultiTemplateMatchNode::clear_templates() {
    templates_.clear();
}

// ========== Node Registration ==========

OVF_REGISTER_NODE(TemplateMatchNode, "TemplateMatch", TemplateMatchNode::make_info())
OVF_REGISTER_NODE(TemplateTrainNode, "TemplateTrain", TemplateTrainNode::make_info())
OVF_REGISTER_NODE(MultiTemplateMatchNode, "MultiTemplateMatch", MultiTemplateMatchNode::make_info())

} // namespace algorithm
} // namespace ovf