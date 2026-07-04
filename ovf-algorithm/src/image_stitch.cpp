/**
 * @file image_stitch.cpp
 * @brief 图像拼接和融合节点实现
 */

#include "ovf/algorithm/image_stitch.h"
#include "ovf/core/logger.h"
#include <unordered_map>

namespace ovf {
namespace algorithm {

// ==================== 辅助函数 ====================

// 转换为灰度图像
static ImageData to_gray(const ImageData& input) {
    if (input.channels == 1) return input;
    
    ImageData gray;
    gray.width = input.width;
    gray.height = input.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(gray.width * gray.height);
    gray.timestamp = input.timestamp;
    gray.frame_id = input.frame_id;
    gray.source_id = input.source_id;
    
    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            gray.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i];
        }
    }
    
    return gray;
}

// 转换为彩色图像（用于绘制）
static ImageData to_color(const ImageData& input) {
    if (input.channels == 3) return input;
    
    ImageData color;
    color.width = input.width;
    color.height = input.height;
    color.channels = 3;
    color.format = ImageFormat::BGR8;
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

// ==================== 图像配准节点 ====================

ImageRegisterNode::ImageRegisterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageRegisterNode::make_info() {
    NodeInfo info;
    info.id = "ImageRegister";
    info.name = "图像配准";
    info.category = "图像拼接";
    info.description = "使用特征点匹配进行图像配准";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("match_count", "匹配点数量", DataType::Number));
    info.outputs.push_back(DataPort("success", "配准成功", DataType::Boolean));
    
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(500)));
    info.params.push_back(ParamDef("ratio_threshold", "比率阈值", DataType::Number, Data(0.75)));
    info.params.push_back(ParamDef("min_matches", "最小匹配数", DataType::Number, Data(10)));
    
    return info;
}

Result<void> ImageRegisterNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");
    
    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();
    
    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为灰度
    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);
    
    // 参数
    int max_corners = get_param("max_corners", Data(500)).as_int();
    double ratio_threshold = get_param("ratio_threshold", Data(0.75)).as_number();
    int min_matches = get_param("min_matches", Data(10)).as_int();
    
    // 检测角点
    std::vector<std::pair<double, double>> corners1, corners2;
    stitch_utils::detect_harris_corners(gray1, corners1, 0.04, 50, 3, max_corners);
    stitch_utils::detect_harris_corners(gray2, corners2, 0.04, 50, 3, max_corners);
    
    // 计算描述符
    std::vector<std::vector<float>> desc1, desc2;
    for (const auto& c : corners1) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray1, c.first, c.second, d);
        desc1.push_back(d);
    }
    for (const auto& c : corners2) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray2, c.first, c.second, d);
        desc2.push_back(d);
    }
    
    // 匹配特征点
    std::vector<MatchPair> matches;
    stitch_utils::match_features(corners1, desc1, corners2, desc2, matches, ratio_threshold);
    
    // 输出结果
    int match_count = static_cast<int>(matches.size());
    bool success = match_count >= min_matches;
    
    set_output("match_count", Data(match_count));
    set_output("success", Data(success));
    
    OVF_INFO() << "Image registration: " << corners1.size() << " vs " << corners2.size()
               << " corners, " << match_count << " matches, success: " << success;
    
    return Result<void>::success();
}

// ==================== 特征点配准节点 ====================

FeatureMatchRegisterNode::FeatureMatchRegisterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FeatureMatchRegisterNode::make_info() {
    NodeInfo info;
    info.id = "FeatureMatchRegister";
    info.name = "特征点配准";
    info.category = "图像拼接";
    info.description = "基于特征点匹配进行精确配准";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("match_count", "匹配点数量", DataType::Number));
    info.outputs.push_back(DataPort("inlier_count", "内点数量", DataType::Number));
    info.outputs.push_back(DataPort("image", "输出图像（带匹配标注）", DataType::Image));
    
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(500)));
    info.params.push_back(ParamDef("ransac_threshold", "RANSAC阈值", DataType::Number, Data(3.0)));
    info.params.push_back(ParamDef("draw_matches", "绘制匹配", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> FeatureMatchRegisterNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");
    
    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();
    
    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);
    
    int max_corners = get_param("max_corners", Data(500)).as_int();
    double ransac_threshold = get_param("ransac_threshold", Data(3.0)).as_number();
    bool draw = get_param("draw_matches", Data(true)).as_bool();
    
    // 检测角点
    std::vector<std::pair<double, double>> corners1, corners2;
    stitch_utils::detect_harris_corners(gray1, corners1, 0.04, 50, 3, max_corners);
    stitch_utils::detect_harris_corners(gray2, corners2, 0.04, 50, 3, max_corners);
    
    // 计算描述符
    std::vector<std::vector<float>> desc1, desc2;
    for (const auto& c : corners1) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray1, c.first, c.second, d);
        desc1.push_back(d);
    }
    for (const auto& c : corners2) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray2, c.first, c.second, d);
        desc2.push_back(d);
    }
    
    // 匹配特征点
    std::vector<MatchPair> matches;
    stitch_utils::match_features(corners1, desc1, corners2, desc2, matches, 0.75);
    
    // RANSAC估计单应性矩阵
    Homography H = stitch_utils::ransac_homography(matches, 1000, ransac_threshold, 0.99);
    
    // 计算内点数
    int inlier_count = 0;
    for (const auto& m : matches) {
        double err = stitch_utils::compute_homography_error(H, m);
        if (err < ransac_threshold) {
            inlier_count++;
        }
    }
    
    set_output("match_count", Data(static_cast<int>(matches.size())));
    set_output("inlier_count", Data(inlier_count));
    
    // 绘制匹配结果（简化版：并排显示）
    if (draw) {
        uint32_t total_width = input1.width + input2.width;
        uint32_t max_height = std::max(input1.height, input2.height);
        
        ImageData output;
        output.width = total_width;
        output.height = max_height;
        output.channels = 3;
        output.format = ImageFormat::BGR8;
        output.data.resize(total_width * max_height * 3, 0);
        
        // 复制图像1
        ImageData color1 = to_color(input1);
        for (uint32_t y = 0; y < input1.height; ++y) {
            for (uint32_t x = 0; x < input1.width; ++x) {
                for (int c = 0; c < 3; ++c) {
                    output.data[(y * total_width + x) * 3 + c] = color1.data[(y * input1.width + x) * 3 + c];
                }
            }
        }
        
        // 复制图像2
        ImageData color2 = to_color(input2);
        for (uint32_t y = 0; y < input2.height; ++y) {
            for (uint32_t x = 0; x < input2.width; ++x) {
                for (int c = 0; c < 3; ++c) {
                    output.data[(y * total_width + input1.width + x) * 3 + c] = color2.data[(y * input2.width + x) * 3 + c];
                }
            }
        }
        
        // 绘制匹配线（简化版：只绘制内点）
        int offset_x = input1.width;
        int line_count = 0;
        for (const auto& m : matches) {
            double err = stitch_utils::compute_homography_error(H, m);
            if (err < ransac_threshold && line_count < 50) {
                // 绘制连接线
                int x1 = static_cast<int>(m.x1);
                int y1 = static_cast<int>(m.y1);
                int x2 = static_cast<int>(m.x2) + offset_x;
                int y2 = static_cast<int>(m.y2);
                
                // Bresenham直线算法（绿色）
                int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
                int sx = (x1 < x2) ? 1 : -1, sy = (y1 < y2) ? 1 : -1;
                int err_line = dx - dy;
                
                while (true) {
                    if (x1 >= 0 && x1 < static_cast<int>(total_width) &&
                        y1 >= 0 && y1 < static_cast<int>(max_height)) {
                        size_t idx = (y1 * total_width + x1) * 3;
                        output.data[idx] = 0;
                        output.data[idx + 1] = 255;
                        output.data[idx + 2] = 0;
                    }
                    
                    if (x1 == x2 && y1 == y2) break;
                    
                    int e2 = 2 * err_line;
                    if (e2 > -dy) { err_line -= dy; x1 += sx; }
                    if (e2 < dx) { err_line += dx; y1 += sy; }
                }
                
                line_count++;
            }
        }
        
        set_output("image", Data(output));
    } else {
        set_output("image", Data(input1));
    }
    
    OVF_INFO() << "Feature match registration: " << matches.size() << " matches, "
               << inlier_count << " inliers";
    
    return Result<void>::success();
}

// ==================== 单应性矩阵计算节点 ====================

HomographyCalcNode::HomographyCalcNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo HomographyCalcNode::make_info() {
    NodeInfo info;
    info.id = "HomographyCalc";
    info.name = "单应性矩阵计算";
    info.category = "图像拼接";
    info.description = "从匹配点计算单应性变换矩阵";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("valid", "矩阵有效", DataType::Boolean));
    info.outputs.push_back(DataPort("image", "变换后的图像", DataType::Image));
    
    info.params.push_back(ParamDef("ransac_iter", "RANSAC迭代次数", DataType::Number, Data(1000)));
    info.params.push_back(ParamDef("ransac_threshold", "RANSAC阈值", DataType::Number, Data(3.0)));
    
    return info;
}

Result<void> HomographyCalcNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");
    
    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();
    
    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);
    
    int ransac_iter = get_param("ransac_iter", Data(1000)).as_int();
    double ransac_threshold = get_param("ransac_threshold", Data(3.0)).as_number();
    
    // 检测角点
    std::vector<std::pair<double, double>> corners1, corners2;
    stitch_utils::detect_harris_corners(gray1, corners1, 0.04, 50, 3, 500);
    stitch_utils::detect_harris_corners(gray2, corners2, 0.04, 50, 3, 500);
    
    // 计算描述符
    std::vector<std::vector<float>> desc1, desc2;
    for (const auto& c : corners1) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray1, c.first, c.second, d);
        desc1.push_back(d);
    }
    for (const auto& c : corners2) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray2, c.first, c.second, d);
        desc2.push_back(d);
    }
    
    // 匹配特征点
    std::vector<MatchPair> matches;
    stitch_utils::match_features(corners1, desc1, corners2, desc2, matches, 0.75);
    
    bool valid = false;
    ImageData output;
    
    if (matches.size() >= 4) {
        // RANSAC估计单应性矩阵
        Homography H = stitch_utils::ransac_homography(matches, ransac_iter, ransac_threshold, 0.99);
        
        // 计算变换后的边界
        double min_x, max_x, min_y, max_y;
        stitch_utils::compute_transformed_bounds(input1, H, min_x, max_x, min_y, max_y);
        
        int offset_x = static_cast<int>(-min_x);
        int offset_y = static_cast<int>(-min_y);
        int dst_width = static_cast<int>(max_x - min_x) + 1;
        int dst_height = static_cast<int>(max_y - min_y) + 1;
        
        // 限制输出图像大小
        dst_width = std::min(dst_width, 4096);
        dst_height = std::min(dst_height, 4096);
        
        // 变换图像
        stitch_utils::warp_image(input1, output, H, dst_width, dst_height, offset_x, offset_y);
        
        valid = true;
        
        OVF_INFO() << "Homography computed: output size " << dst_width << "x" << dst_height;
    } else {
        OVF_WARN() << "Not enough matches for homography: " << matches.size();
        output = input1;
    }
    
    set_output("valid", Data(valid));
    set_output("image", Data(output));
    
    return Result<void>::success();
}

// ==================== 图像拼接节点 ====================

ImageStitchNode::ImageStitchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ImageStitchNode::make_info() {
    NodeInfo info;
    info.id = "ImageStitch";
    info.name = "图像拼接";
    info.category = "图像拼接";
    info.description = "将两幅图像拼接成全景图";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "拼接结果", DataType::Image));
    info.outputs.push_back(DataPort("success", "拼接成功", DataType::Boolean));
    
    info.params.push_back(ParamDef("blend_width", "混合宽度", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("blend_method", "混合方法", DataType::String, Data("linear")));
    
    return info;
}

Result<void> ImageStitchNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");
    
    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();
    
    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    ImageData gray1 = to_gray(input1);
    ImageData gray2 = to_gray(input2);
    
    int blend_width = get_param("blend_width", Data(50)).as_int();
    String blend_method = get_param("blend_method", Data("linear")).as_string();
    
    // 检测角点
    std::vector<std::pair<double, double>> corners1, corners2;
    stitch_utils::detect_harris_corners(gray1, corners1, 0.04, 50, 3, 500);
    stitch_utils::detect_harris_corners(gray2, corners2, 0.04, 50, 3, 500);
    
    // 计算描述符并匹配
    std::vector<std::vector<float>> desc1, desc2;
    for (const auto& c : corners1) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray1, c.first, c.second, d);
        desc1.push_back(d);
    }
    for (const auto& c : corners2) {
        std::vector<float> d;
        stitch_utils::compute_sift_descriptor(gray2, c.first, c.second, d);
        desc2.push_back(d);
    }
    
    std::vector<MatchPair> matches;
    stitch_utils::match_features(corners1, desc1, corners2, desc2, matches, 0.75);
    
    bool success = false;
    ImageData output;
    
    if (matches.size() >= 4) {
        // 计算单应性矩阵
        Homography H = stitch_utils::ransac_homography(matches, 1000, 3.0, 0.99);
        
        // 计算边界
        double min_x1, max_x1, min_y1, max_y1;
        stitch_utils::compute_transformed_bounds(input1, H, min_x1, max_x1, min_y1, max_y1);
        
        double min_x2 = 0, max_x2 = input2.width;
        double min_y2 = 0, max_y2 = input2.height;
        
        double min_x = std::min(min_x1, min_x2);
        double max_x = std::max(max_x1, max_x2);
        double min_y = std::min(min_y1, min_y2);
        double max_y = std::max(max_y1, max_y2);
        
        int offset_x = static_cast<int>(-min_x);
        int offset_y = static_cast<int>(-min_y);
        int dst_width = static_cast<int>(max_x - min_x) + 1;
        int dst_height = static_cast<int>(max_y - min_y) + 1;
        
        // 限制大小
        dst_width = std::min(dst_width, 4096);
        dst_height = std::min(dst_height, 4096);
        
        // 变换图像1
        ImageData warped1;
        stitch_utils::warp_image(input1, warped1, H, dst_width, dst_height, offset_x, offset_y);
        
        // 将图像2放置到同一坐标系
        ImageData warped2;
        warped2.width = dst_width;
        warped2.height = dst_height;
        warped2.channels = input2.channels;
        warped2.format = input2.format;
        warped2.data.resize(dst_width * dst_height * warped2.channels, 0);
        
        for (uint32_t y = 0; y < input2.height; ++y) {
            for (uint32_t x = 0; x < input2.width; ++x) {
                int dst_x = static_cast<int>(x) + offset_x;
                int dst_y = static_cast<int>(y) + offset_y;
                
                if (dst_x >= 0 && dst_x < dst_width && dst_y >= 0 && dst_y < dst_height) {
                    for (int c = 0; c < warped2.channels; ++c) {
                        size_t src_idx = (y * input2.width + x) * input2.channels + c;
                        size_t dst_idx = (dst_y * dst_width + dst_x) * warped2.channels + c;
                        warped2.data[dst_idx] = input2.data[src_idx];
                    }
                }
            }
        }
        
        // 混合图像
        if (blend_method == "linear") {
            stitch_utils::blend_linear(output, warped1, warped2, 0.5);
        } else if (blend_method == "gradient") {
            int seam_x = dst_width / 2;
            stitch_utils::blend_gradient(output, warped1, warped2, seam_x, blend_width);
        } else if (blend_method == "multiband") {
            stitch_utils::blend_multiband(output, warped1, warped2, 3);
        } else {
            stitch_utils::blend_linear(output, warped1, warped2, 0.5);
        }
        
        success = true;
        
        OVF_INFO() << "Image stitching completed: " << dst_width << "x" << dst_height
                   << " with " << matches.size() << " matches";
    } else {
        OVF_WARN() << "Not enough matches for stitching: " << matches.size();
        output = input1;
    }
    
    set_output("image", Data(output));
    set_output("success", Data(success));
    
    return Result<void>::success();
}

// ==================== 多图像拼接节点 ====================

MultiImageStitchNode::MultiImageStitchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MultiImageStitchNode::make_info() {
    NodeInfo info;
    info.id = "MultiImageStitch";
    info.name = "多图像拼接";
    info.category = "图像拼接";
    info.description = "将多幅图像拼接成一幅大图";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.inputs.push_back(DataPort("image3", "图像3", DataType::Image, false));
    info.inputs.push_back(DataPort("image4", "图像4", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "拼接结果", DataType::Image));
    
    info.params.push_back(ParamDef("overlap", "重叠宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("direction", "拼接方向", DataType::String, Data("horizontal")));
    
    return info;
}

Result<void> MultiImageStitchNode::execute(FlowContext& context) {
    std::vector<ImageData> images;
    
    // 获取所有输入图像
    if (has_input("image1")) {
        auto img1 = get_input("image1");
        if (img1.is_image() && !img1.as_image().empty()) {
            images.push_back(img1.as_image());
        }
    }
    if (has_input("image2")) {
        auto img2 = get_input("image2");
        if (img2.is_image() && !img2.as_image().empty()) {
            images.push_back(img2.as_image());
        }
    }
    if (has_input("image3")) {
        auto img3 = get_input("image3");
        if (img3.is_image() && !img3.as_image().empty()) {
            images.push_back(img3.as_image());
        }
    }
    if (has_input("image4")) {
        auto img4 = get_input("image4");
        if (img4.is_image() && !img4.as_image().empty()) {
            images.push_back(img4.as_image());
        }
    }
    
    if (images.size() < 2) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Need at least 2 images");
    }
    
    int overlap = get_param("overlap", Data(100)).as_int();
    String direction = get_param("direction", Data("horizontal")).as_string();
    
    ImageData output;
    
    if (direction == "horizontal") {
        stitch_utils::stitch_multi_horizontal(images, output, overlap);
    } else {
        // 垂直拼接（简化实现）
        // 转换为横向拼接的变种
        uint32_t max_width = 0;
        uint32_t total_height = 0;
        
        for (size_t i = 0; i < images.size(); ++i) {
            max_width = std::max(max_width, images[i].width);
            total_height += images[i].height;
            if (i > 0) total_height -= overlap;
        }
        
        output.width = max_width;
        output.height = total_height;
        output.channels = images[0].channels;
        output.format = images[0].format;
        output.data.resize(max_width * total_height * output.channels, 0);
        
        int offset_y = 0;
        for (size_t i = 0; i < images.size(); ++i) {
            const auto& img = images[i];
            
            for (uint32_t y = 0; y < img.height; ++y) {
                for (uint32_t x = 0; x < img.width; ++x) {
                    int dst_y = offset_y + static_cast<int>(y);
                    
                    if (dst_y >= 0 && dst_y < static_cast<int>(total_height) &&
                        x < max_width) {
                        for (int c = 0; c < output.channels; ++c) {
                            size_t src_idx = (y * img.width + x) * img.channels + c;
                            size_t dst_idx = (dst_y * max_width + x) * output.channels + c;
                            output.data[dst_idx] = img.data[src_idx];
                        }
                    }
                }
            }
            
            offset_y += img.height - overlap;
        }
    }
    
    set_output("image", Data(output));
    
    OVF_INFO() << "Multi-image stitching: " << images.size() << " images, "
               << output.width << "x" << output.height;
    
    return Result<void>::success();
}

// ==================== 全景拼接节点 ====================

PanoramaStitchNode::PanoramaStitchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PanoramaStitchNode::make_info() {
    NodeInfo info;
    info.id = "PanoramaStitch";
    info.name = "全景拼接";
    info.category = "图像拼接";
    info.description = "将多幅图像拼接成全景图（带投影变换）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.inputs.push_back(DataPort("image3", "图像3", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "全景结果", DataType::Image));
    info.outputs.push_back(DataPort("success", "拼接成功", DataType::Boolean));
    
    info.params.push_back(ParamDef("blend_width", "混合宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("projection", "投影方式", DataType::String, Data("flat")));
    
    return info;
}

Result<void> PanoramaStitchNode::execute(FlowContext& context) {
    std::vector<ImageData> images;
    
    if (has_input("image1")) {
        auto img1 = get_input("image1");
        if (img1.is_image() && !img1.as_image().empty()) {
            images.push_back(img1.as_image());
        }
    }
    if (has_input("image2")) {
        auto img2 = get_input("image2");
        if (img2.is_image() && !img2.as_image().empty()) {
            images.push_back(img2.as_image());
        }
    }
    if (has_input("image3")) {
        auto img3 = get_input("image3");
        if (img3.is_image() && !img3.as_image().empty()) {
            images.push_back(img3.as_image());
        }
    }
    
    if (images.size() < 2) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Need at least 2 images");
    }
    
    int blend_width = get_param("blend_width", Data(100)).as_int();
    String projection = get_param("projection", Data("flat")).as_string();
    
    bool success = false;
    ImageData output;
    
    // 简化实现：使用顺序拼接
    if (images.size() == 2) {
        ImageData gray1 = to_gray(images[0]);
        ImageData gray2 = to_gray(images[1]);
        
        std::vector<std::pair<double, double>> corners1, corners2;
        stitch_utils::detect_harris_corners(gray1, corners1, 0.04, 50, 3, 500);
        stitch_utils::detect_harris_corners(gray2, corners2, 0.04, 50, 3, 500);
        
        std::vector<std::vector<float>> desc1, desc2;
        for (const auto& c : corners1) {
            std::vector<float> d;
            stitch_utils::compute_sift_descriptor(gray1, c.first, c.second, d);
            desc1.push_back(d);
        }
        for (const auto& c : corners2) {
            std::vector<float> d;
            stitch_utils::compute_sift_descriptor(gray2, c.first, c.second, d);
            desc2.push_back(d);
        }
        
        std::vector<MatchPair> matches;
        stitch_utils::match_features(corners1, desc1, corners2, desc2, matches, 0.75);
        
        if (matches.size() >= 4) {
            Homography H = stitch_utils::ransac_homography(matches, 1000, 3.0, 0.99);
            
            double min_x1, max_x1, min_y1, max_y1;
            stitch_utils::compute_transformed_bounds(images[0], H, min_x1, max_x1, min_y1, max_y1);
            
            double min_x = std::min(min_x1, 0.0);
            double max_x = std::max(max_x1, static_cast<double>(images[1].width));
            double min_y = std::min(min_y1, 0.0);
            double max_y = std::max(max_y1, static_cast<double>(images[1].height));
            
            int offset_x = static_cast<int>(-min_x);
            int offset_y = static_cast<int>(-min_y);
            int dst_width = std::min(static_cast<int>(max_x - min_x) + 1, 4096);
            int dst_height = std::min(static_cast<int>(max_y - min_y) + 1, 4096);
            
            ImageData warped1, warped2;
            stitch_utils::warp_image(images[0], warped1, H, dst_width, dst_height, offset_x, offset_y);
            
            warped2.width = dst_width;
            warped2.height = dst_height;
            warped2.channels = images[1].channels;
            warped2.format = images[1].format;
            warped2.data.resize(dst_width * dst_height * warped2.channels, 0);
            
            for (uint32_t y = 0; y < images[1].height; ++y) {
                for (uint32_t x = 0; x < images[1].width; ++x) {
                    int dst_x = static_cast<int>(x) + offset_x;
                    int dst_y = static_cast<int>(y) + offset_y;
                    
                    if (dst_x >= 0 && dst_x < dst_width && dst_y >= 0 && dst_y < dst_height) {
                        for (int c = 0; c < warped2.channels; ++c) {
                            size_t src_idx = (y * images[1].width + x) * images[1].channels + c;
                            size_t dst_idx = (dst_y * dst_width + dst_x) * warped2.channels + c;
                            warped2.data[dst_idx] = images[1].data[src_idx];
                        }
                    }
                }
            }
            
            stitch_utils::blend_gradient(output, warped1, warped2, dst_width / 2, blend_width);
            success = true;
        }
    } else {
        // 多图像：使用简单横向拼接
        stitch_utils::stitch_multi_horizontal(images, output, blend_width);
        success = true;
    }
    
    if (!success) {
        output = images[0];
    }
    
    set_output("image", Data(output));
    set_output("success", Data(success));
    
    OVF_INFO() << "Panorama stitching: " << images.size() << " images, success: " << success;
    
    return Result<void>::success();
}

// ==================== 马赛克拼接节点 ====================

MosaicStitchNode::MosaicStitchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MosaicStitchNode::make_info() {
    NodeInfo info;
    info.id = "MosaicStitch";
    info.name = "马赛克拼接";
    info.category = "图像拼接";
    info.description = "将多幅图像按网格排列拼接";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.inputs.push_back(DataPort("image3", "图像3", DataType::Image, false));
    info.inputs.push_back(DataPort("image4", "图像4", DataType::Image, false));
    info.inputs.push_back(DataPort("image5", "图像5", DataType::Image, false));
    info.inputs.push_back(DataPort("image6", "图像6", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "马赛克结果", DataType::Image));
    
    info.params.push_back(ParamDef("cols", "列数", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("rows", "行数", DataType::Number, Data(2)));
    info.params.push_back(ParamDef("spacing", "间距", DataType::Number, Data(0)));
    
    return info;
}

Result<void> MosaicStitchNode::execute(FlowContext& context) {
    std::vector<ImageData> images;
    
    // 获取所有输入图像
    for (int i = 1; i <= 6; ++i) {
        String port_name = "image" + std::to_string(i);
        if (has_input(port_name)) {
            auto img = get_input(port_name);
            if (img.is_image() && !img.as_image().empty()) {
                images.push_back(img.as_image());
            }
        }
    }
    
    if (images.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "No input images");
    }
    
    int cols = get_param("cols", Data(3)).as_int();
    int rows = get_param("rows", Data(2)).as_int();
    int spacing = get_param("spacing", Data(0)).as_int();
    
    if (cols <= 0 || rows <= 0) {
        cols = static_cast<int>(std::sqrt(images.size()));
        rows = static_cast<int>(images.size() / cols) + (images.size() % cols > 0 ? 1 : 0);
    }
    
    ImageData output;
    stitch_utils::stitch_mosaic(images, output, cols, rows, spacing);
    
    set_output("image", Data(output));
    
    OVF_INFO() << "Mosaic stitching: " << images.size() << " images in "
               << cols << "x" << rows << " grid";
    
    return Result<void>::success();
}

// ==================== 接缝融合节点 ====================

SeamBlendNode::SeamBlendNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SeamBlendNode::make_info() {
    NodeInfo info;
    info.id = "SeamBlend";
    info.name = "接缝融合";
    info.category = "图像融合";
    info.description = "寻找最佳接缝线进行融合";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "融合结果", DataType::Image));
    info.outputs.push_back(DataPort("seam_count", "接缝点数", DataType::Number));
    
    info.params.push_back(ParamDef("blend_width", "混合宽度", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("draw_seam", "绘制接缝", DataType::Boolean, Data(false)));
    
    return info;
}

Result<void> SeamBlendNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");
    
    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();
    
    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    if (input1.width != input2.width || input1.height != input2.height) {
        OVF_WARN() << "Image sizes don't match for seam blending";
        set_output("image", Data(input1));
        set_output("seam_count", Data(0));
        return Result<void>::success();
    }
    
    int blend_width = get_param("blend_width", Data(20)).as_int();
    bool draw_seam = get_param("draw_seam", Data(false)).as_bool();
    
    // 寻找接缝线
    std::vector<int> seam_positions;
    stitch_utils::find_seam(input1, input2, seam_positions);
    
    // 按接缝线融合
    ImageData output;
    output.width = input1.width;
    output.height = input1.height;
    output.channels = input1.channels;
    output.format = input1.format;
    output.data.resize(output.width * output.height * output.channels);
    
    for (uint32_t y = 0; y < output.height; ++y) {
        int seam_x = seam_positions[y];
        int left_start = seam_x - blend_width / 2;
        int right_end = seam_x + blend_width / 2;
        
        for (uint32_t x = 0; x < output.width; ++x) {
            double alpha = 0.5;
            
            if (x < left_start) {
                alpha = 0.0;
            } else if (x > right_end) {
                alpha = 1.0;
            } else {
                alpha = static_cast<double>(x - left_start) / blend_width;
            }
            
            for (int c = 0; c < output.channels; ++c) {
                size_t idx = (y * output.width + x) * output.channels + c;
                output.data[idx] = static_cast<uint8_t>(
                    input1.data[idx] * (1 - alpha) + input2.data[idx] * alpha
                );
            }
        }
    }
    
    // 绘制接缝线
    if (draw_seam) {
        ImageData color_output = to_color(output);
        
        for (uint32_t y = 0; y < output.height; ++y) {
            int x = seam_positions[y];
            if (x >= 0 && x < static_cast<int>(output.width)) {
                // 绘制红色接缝线
                for (int dx = -1; dx <= 1; ++dx) {
                    int px = x + dx;
                    if (px >= 0 && px < static_cast<int>(output.width)) {
                        size_t idx = (y * output.width + px) * 3;
                        color_output.data[idx] = 0;
                        color_output.data[idx + 1] = 0;
                        color_output.data[idx + 2] = 255;
                    }
                }
            }
        }
        
        output = color_output;
    }
    
    set_output("image", Data(output));
    set_output("seam_count", Data(static_cast<int>(seam_positions.size())));
    
    OVF_INFO() << "Seam blending: " << seam_positions.size() << " seam points";
    
    return Result<void>::success();
}

// ==================== 曝光补偿融合节点 ====================

ExposureCompensateNode::ExposureCompensateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ExposureCompensateNode::make_info() {
    NodeInfo info;
    info.id = "ExposureCompensate";
    info.name = "曝光补偿融合";
    info.category = "图像融合";
    info.description = "调整曝光差异并进行融合";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image1", "图像1", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "图像2", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "融合结果", DataType::Image));
    info.outputs.push_back(DataPort("brightness1", "图像1亮度", DataType::Number));
    info.outputs.push_back(DataPort("brightness2", "图像2亮度", DataType::Number));
    
    info.params.push_back(ParamDef("compensate", "补偿方式", DataType::String, Data("match")));
    info.params.push_back(ParamDef("target_brightness", "目标亮度", DataType::Number, Data(128)));
    
    return info;
}

Result<void> ExposureCompensateNode::execute(FlowContext& context) {
    auto input1_data = get_input("image1");
    auto input2_data = get_input("image2");
    
    if (!input1_data.is_image() || !input2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input1 = input1_data.as_image();
    ImageData input2 = input2_data.as_image();
    
    if (input1.empty() || input2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    String compensate = get_param("compensate", Data("match")).as_string();
    double target_brightness = get_param("target_brightness", Data(128)).as_number();
    
    // 计算平均亮度
    double brightness1 = stitch_utils::compute_average_brightness(input1);
    double brightness2 = stitch_utils::compute_average_brightness(input2);
    
    ImageData adjusted1 = input1;
    ImageData adjusted2 = input2;
    
    if (compensate == "match") {
        // 将图像2调整为图像1的亮度
        if (brightness1 > 0 && brightness2 > 0) {
            double gain = brightness1 / brightness2;
            stitch_utils::compensate_exposure(adjusted2, gain, 0);
        }
    } else if (compensate == "target") {
        // 将两幅图像调整为目标亮度
        if (brightness1 > 0) {
            double gain1 = target_brightness / brightness1;
            stitch_utils::compensate_exposure(adjusted1, gain1, 0);
        }
        if (brightness2 > 0) {
            double gain2 = target_brightness / brightness2;
            stitch_utils::compensate_exposure(adjusted2, gain2, 0);
        }
    } else if (compensate == "average") {
        // 将两幅图像调整为平均亮度
        double avg_brightness = (brightness1 + brightness2) / 2;
        if (brightness1 > 0) {
            double gain1 = avg_brightness / brightness1;
            stitch_utils::compensate_exposure(adjusted1, gain1, 0);
        }
        if (brightness2 > 0) {
            double gain2 = avg_brightness / brightness2;
            stitch_utils::compensate_exposure(adjusted2, gain2, 0);
        }
    }
    
    // 融合图像
    ImageData output;
    
    if (input1.width == input2.width && input1.height == input2.height) {
        stitch_utils::blend_linear(output, adjusted1, adjusted2, 0.5);
    } else {
        // 如果大小不同，只输出第一幅
        output = adjusted1;
    }
    
    set_output("image", Data(output));
    set_output("brightness1", Data(brightness1));
    set_output("brightness2", Data(brightness2));
    
    OVF_INFO() << "Exposure compensation: b1=" << brightness1 << ", b2=" << brightness2
               << ", method=" << compensate;
    
    return Result<void>::success();
}

// ==================== 注册节点 ====================

OVF_REGISTER_NODE(ImageRegisterNode, "ImageRegister", ImageRegisterNode::make_info())
OVF_REGISTER_NODE(FeatureMatchRegisterNode, "FeatureMatchRegister", FeatureMatchRegisterNode::make_info())
OVF_REGISTER_NODE(HomographyCalcNode, "HomographyCalc", HomographyCalcNode::make_info())

OVF_REGISTER_NODE(ImageStitchNode, "ImageStitch", ImageStitchNode::make_info())
OVF_REGISTER_NODE(MultiImageStitchNode, "MultiImageStitch", MultiImageStitchNode::make_info())
OVF_REGISTER_NODE(PanoramaStitchNode, "PanoramaStitch", PanoramaStitchNode::make_info())
OVF_REGISTER_NODE(MosaicStitchNode, "MosaicStitch", MosaicStitchNode::make_info())

OVF_REGISTER_NODE(SeamBlendNode, "SeamBlend", SeamBlendNode::make_info())
OVF_REGISTER_NODE(ExposureCompensateNode, "ExposureCompensate", ExposureCompensateNode::make_info())

} // namespace algorithm
} // namespace ovf