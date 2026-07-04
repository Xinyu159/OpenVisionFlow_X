/**
 * @file automotive_inspection.cpp
 * @brief 汽车零部件检测节点实现
 */

#include "ovf/algorithm/automotive_inspection.h"
#include "ovf/core/logger.h"
#include <sstream>
#include <cmath>

namespace ovf {
namespace algorithm {

// 辅助函数：创建ImageData对象
static ImageData create_image(int width, int height, int channels = 1, ImageFormat format = ImageFormat::Mono8) {
    ImageData img;
    img.width = width;
    img.height = height;
    img.channels = channels;
    img.format = format;
    img.data.resize(width * height * channels, 0);
    return img;
}

//==============================================================================
// BodyPanelInspectionNode - 钣金检测节点（表面缺陷/变形检测）
//==============================================================================

BodyPanelInspectionNode::BodyPanelInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BodyPanelInspectionNode::make_info() {
    NodeInfo info;
    info.id = "BodyPanelInspection";
    info.name = "钣金检测";
    info.category = "汽车零部件检测";
    info.description = "检测钣金表面缺陷：划痕、凹陷、凸起、变形等";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("scratch_count", "划痕数量", DataType::Number));
    info.outputs.push_back(DataPort("dent_count", "凹陷数量", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "质量分数", DataType::Number));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    
    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.6)));
    info.params.push_back(ParamDef("min_defect_size", "最小缺陷尺寸", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("max_defect_size", "最大缺陷尺寸", DataType::Number, Data(500)));
    info.params.push_back(ParamDef("detect_dents", "检测凹陷", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("detect_deformation", "检测变形", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(25.0)));
    
    return info;
}

Result<void> BodyPanelInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double sensitivity = get_param("sensitivity", Data(0.6)).as_number();
    uint32_t min_size = static_cast<uint32_t>(get_param("min_defect_size", Data(10)).as_int());
    uint32_t max_size = static_cast<uint32_t>(get_param("max_defect_size", Data(500)).as_int());
    bool detect_dents = get_param("detect_dents", Data(true)).as_bool();
    double edge_threshold = get_param("edge_threshold", Data(25.0)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 高斯模糊降噪
    ImageData blurred;
    automotive_utils::gaussian_blur(gray, blurred, 5, 1.5);
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(blurred, grad_x, grad_y, edge_mag);
    
    // 自适应阈值分割
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 15, static_cast<int>(30 * (1.0 - sensitivity)));
    
    // 形态学处理
    ImageData morph_open;
    automotive_utils::morphological_open(binary, morph_open, 3);
    
    // 连通区域分析检测缺陷
    std::vector<SurfaceDefect> defects;
    uint32_t defect_id = 1;
    int scratch_count = 0, dent_count = 0;
    
    std::vector<bool> visited(gray.width * gray.height, false);
    
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            if (morph_open.data[y * gray.width + x] > 128 && !visited[y * gray.width + x]) {
                // BFS找连通区域
                std::vector<std::pair<uint32_t, uint32_t>> region;
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                
                uint32_t min_x = x, max_x = x, min_y = y, max_y = y;
                double intensity_sum = 0.0;
                
                int dx[] = {-1, 0, 1, 0, -1, -1, 1, 1};
                int dy[] = {0, -1, 0, 1, -1, 1, -1, 1};
                
                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    region.push_back({cx, cy});
                    
                    min_x = std::min(min_x, cx);
                    max_x = std::max(max_x, cx);
                    min_y = std::min(min_y, cy);
                    max_y = std::max(max_y, cy);
                    intensity_sum += gray.data[cy * gray.width + cx];
                    
                    for (int i = 0; i < 8; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height) &&
                            !visited[ny * gray.width + nx] &&
                            morph_open.data[ny * gray.width + nx] > 128) {
                            visited[ny * gray.width + nx] = true;
                            queue.push_back({nx, ny});
                        }
                    }
                }
                
                uint32_t width = max_x - min_x + 1;
                uint32_t height = max_y - min_y + 1;
                uint32_t area = static_cast<uint32_t>(region.size());
                
                if (area >= min_size && area <= max_size) {
                    SurfaceDefect defect;
                    defect.id = defect_id++;
                    defect.center_x = (min_x + max_x) / 2.0;
                    defect.center_y = (min_y + max_y) / 2.0;
                    defect.width = width;
                    defect.height = height;
                    defect.area = area;
                    
                    // 缺陷分类
                    double aspect_ratio = static_cast<double>(width) / height;
                    double avg_intensity = intensity_sum / area;
                    double circularity = 4.0 * M_PI * area / (2.0 * (width + height) * (width + height));
                    
                    // 划痕：长条形
                    if (aspect_ratio > 3.0 && area < 1000) {
                        defect.type = "scratch";
                        defect.severity = std::min(1.0, aspect_ratio / 8.0);
                        scratch_count++;
                    } 
                    // 凹陷：圆形或椭圆形，灰度较低
                    else if (detect_dents && avg_intensity < 100 && circularity > 0.4) {
                        defect.type = "dent";
                        defect.depth = (128.0 - avg_intensity) / 128.0 * 2.0;
                        defect.severity = std::min(1.0, area / 200.0);
                        dent_count++;
                    }
                    // 变形/凸起
                    else if (avg_intensity > 150) {
                        defect.type = "bulge";
                        defect.severity = std::min(1.0, area / 300.0);
                    }
                    // 其他缺陷
                    else {
                        defect.type = "pit";
                        defect.severity = std::min(1.0, std::sqrt(area / 50.0));
                    }
                    
                    defect.confidence = 0.7 + 0.3 * sensitivity;
                    defects.push_back(defect);
                }
            }
        }
    }
    
    // 计算质量分数
    double quality_score = 1.0;
    for (const auto& d : defects) {
        quality_score -= d.severity * 0.15;
    }
    quality_score = std::max(0.0, std::min(1.0, quality_score));
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& defect : defects) {
            uint8_t r = 0, g = 0, b = 0;
            
            if (defect.type == "scratch") { r = 255; g = 165; b = 0; }      // 橙色
            else if (defect.type == "dent") { r = 255; g = 0; b = 0; }      // 红色
            else if (defect.type == "bulge") { r = 0; g = 255; b = 255; }   // 青色
            else { r = 255; g = 0; b = 255; }                                // 紫色
            
            automotive_utils::draw_rect(output,
                static_cast<uint32_t>(defect.center_x - defect.width/2),
                static_cast<uint32_t>(defect.center_y - defect.height/2),
                defect.width, defect.height, r, g, b, 2);
            automotive_utils::draw_cross(output,
                static_cast<uint32_t>(defect.center_x),
                static_cast<uint32_t>(defect.center_y), 5, r, g, b);
        }
    }
    
    set_output("image", Data(output));
    set_output("defect_count", Data(static_cast<int>(defects.size())));
    set_output("scratch_count", Data(scratch_count));
    set_output("dent_count", Data(dent_count));
    set_output("quality_score", Data(quality_score));
    
    OVF_INFO() << "BodyPanelInspection: " << defects.size() << " defects"
               << " (scratch:" << scratch_count << " dent:" << dent_count << ")"
               << " quality=" << quality_score;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(BodyPanelInspectionNode, "BodyPanelInspection", BodyPanelInspectionNode::make_info())

//==============================================================================
// WeldSeamInspectionNode - 焊缝检测节点（气孔/裂纹/咬边检测）
//==============================================================================

WeldSeamInspectionNode::WeldSeamInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WeldSeamInspectionNode::make_info() {
    NodeInfo info;
    info.id = "WeldSeamInspection";
    info.name = "焊缝检测";
    info.category = "汽车零部件检测";
    info.description = "焊缝质量评估：气孔、裂纹、咬边、焊瘤检测";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("weld_width", "焊缝宽度", DataType::Number));
    info.outputs.push_back(DataPort("porosity_rate", "气孔率", DataType::Number));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "质量分数", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    
    info.params.push_back(ParamDef("expected_width", "期望焊缝宽度", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("width_tolerance", "宽度容差", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("max_porosity", "最大气孔率", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("detect_cracks", "检测裂纹", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("detect_undercut", "检测咬边", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.7)));
    
    return info;
}

Result<void> WeldSeamInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double expected_width = get_param("expected_width", Data(5.0)).as_number();
    double width_tolerance = get_param("width_tolerance", Data(1.0)).as_number();
    double max_porosity = get_param("max_porosity", Data(5.0)).as_number();
    bool detect_cracks = get_param("detect_cracks", Data(true)).as_bool();
    bool detect_undercut = get_param("detect_undercut", Data(true)).as_bool();
    double sensitivity = get_param("sensitivity", Data(0.7)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 高斯模糊
    ImageData blurred;
    automotive_utils::gaussian_blur(gray, blurred, 5, 1.5);
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(blurred, grad_x, grad_y, edge_mag);
    
    // 自适应阈值
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 15, static_cast<int>(20 * (1.0 - sensitivity)));
    
    // 检测焊缝区域（假设焊缝在图像中间）
    uint32_t weld_start_y = gray.height * 0.3;
    uint32_t weld_end_y = gray.height * 0.7;
    
    // 计算焊缝宽度
    std::vector<uint32_t> weld_widths;
    for (uint32_t y = weld_start_y; y < weld_end_y; ++y) {
        uint32_t left_edge = 0, right_edge = gray.width;
        
        for (uint32_t x = 0; x < gray.width / 2; ++x) {
            if (binary.data[y * gray.width + x] > 128) {
                left_edge = x;
                break;
            }
        }
        
        for (uint32_t x = gray.width - 1; x > gray.width / 2; --x) {
            if (binary.data[y * gray.width + x] > 128) {
                right_edge = x;
                break;
            }
        }
        
        if (right_edge > left_edge) {
            weld_widths.push_back(right_edge - left_edge);
        }
    }
    
    double weld_width = 0.0;
    if (!weld_widths.empty()) {
        weld_width = std::accumulate(weld_widths.begin(), weld_widths.end(), 0.0) / weld_widths.size();
        // 转换像素到mm（假设比例）
        weld_width = weld_width * 0.1; // 像素到mm转换因子
    }
    
    // 检测气孔（圆形暗点）
    std::vector<WeldDefect> defects;
    uint32_t defect_id = 1;
    int porosity_count = 0, crack_count = 0, undercut_count = 0;
    
    // 在焊缝区域内搜索气孔
    std::vector<bool> visited(gray.width * gray.height, false);
    
    for (uint32_t y = weld_start_y; y < weld_end_y; ++y) {
        for (uint32_t x = gray.width * 0.2; x < gray.width * 0.8; ++x) {
            if (gray.data[y * gray.width + x] < 50 && !visited[y * gray.width + x]) {
                // BFS找连通区域（气孔特征）
                std::vector<std::pair<uint32_t, uint32_t>> region;
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                
                uint32_t min_x = x, max_x = x, min_y = y, max_y = y;
                
                int dx[] = {-1, 0, 1, 0};
                int dy[] = {0, -1, 0, 1};
                
                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    region.push_back({cx, cy});
                    
                    min_x = std::min(min_x, cx);
                    max_x = std::max(max_x, cx);
                    min_y = std::min(min_y, cy);
                    max_y = std::max(max_y, cy);
                    
                    for (int i = 0; i < 4; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height) &&
                            !visited[ny * gray.width + nx] &&
                            gray.data[ny * gray.width + nx] < 80) {
                            visited[ny * gray.width + nx] = true;
                            queue.push_back({nx, ny});
                        }
                    }
                }
                
                uint32_t area = static_cast<uint32_t>(region.size());
                
                if (area >= 5 && area <= 100) {
                    WeldDefect defect;
                    defect.id = defect_id++;
                    defect.type = "porosity";
                    defect.center_x = (min_x + max_x) / 2.0;
                    defect.center_y = (min_y + max_y) / 2.0;
                    defect.width = max_x - min_x + 1;
                    defect.height = max_y - min_y + 1;
                    defect.area = area;
                    defect.severity = std::min(1.0, area / 30.0);
                    defect.confidence = 0.8;
                    
                    defects.push_back(defect);
                    porosity_count++;
                }
            }
        }
    }
    
    // 检测裂纹（长条形暗线）
    if (detect_cracks) {
        auto lines = automotive_utils::detect_lines(binary, 20);
        for (const auto& line : lines) {
            double length = std::sqrt(
                (line.second.x - line.first.x) * (line.second.x - line.first.x) +
                (line.second.y - line.first.y) * (line.second.y - line.first.y));
            
            if (length > 30) {
                WeldDefect defect;
                defect.id = defect_id++;
                defect.type = "crack";
                defect.center_x = (line.first.x + line.second.x) / 2.0;
                defect.center_y = (line.first.y + line.second.y) / 2.0;
                defect.length = length;
                defect.severity = std::min(1.0, length / 50.0);
                defect.confidence = 0.75;
                
                defects.push_back(defect);
                crack_count++;
            }
        }
    }
    
    // 检测咬边（焊缝边缘的不规则凹陷）
    if (detect_undercut) {
        for (uint32_t y = weld_start_y; y < weld_end_y; y += 5) {
            // 检查边缘区域
            for (uint32_t x = gray.width * 0.15; x < gray.width * 0.25; ++x) {
                if (gray.data[y * gray.width + x] < 70 && 
                    gray.data[y * gray.width + x + 5] > 150) {
                    WeldDefect defect;
                    defect.id = defect_id++;
                    defect.type = "undercut";
                    defect.center_x = x + 2;
                    defect.center_y = y;
                    defect.severity = 0.6;
                    defect.confidence = 0.65;
                    
                    defects.push_back(defect);
                    undercut_count++;
                }
            }
        }
    }
    
    // 计算气孔率
    uint32_t weld_area = gray.width * 0.6 * (weld_end_y - weld_start_y);
    double porosity_rate = 0.0;
    for (const auto& d : defects) {
        if (d.type == "porosity") {
            porosity_rate += d.area;
        }
    }
    porosity_rate = porosity_rate / weld_area * 100.0;
    
    // 评估质量
    WeldQualityResult result;
    result.weld_width = weld_width;
    result.porosity_rate = porosity_rate;
    result.defect_count = static_cast<int>(defects.size());
    
    double quality_score = 1.0;
    
    // 焊缝宽度评估
    if (std::abs(weld_width - expected_width) > width_tolerance) {
        quality_score -= 0.2;
    }
    
    // 气孔率评估
    if (porosity_rate > max_porosity) {
        quality_score -= porosity_rate / max_porosity * 0.3;
    }
    
    // 缺陷惩罚
    quality_score -= crack_count * 0.2;
    quality_score -= undercut_count * 0.1;
    quality_score -= porosity_count * 0.05;
    
    result.quality_score = std::max(0.0, std::min(1.0, quality_score));
    result.is_acceptable = (result.quality_score > 0.6) && 
                           (porosity_rate <= max_porosity) &&
                           (crack_count == 0);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制焊缝边界
        automotive_utils::draw_line(output, 0, weld_start_y, gray.width, weld_start_y, 0, 255, 0);
        automotive_utils::draw_line(output, 0, weld_end_y, gray.width, weld_end_y, 0, 255, 0);
        
        for (const auto& defect : defects) {
            uint8_t r = 0, g = 0, b = 0;
            
            if (defect.type == "porosity") { r = 255; g = 255; b = 0; }    // 青色
            else if (defect.type == "crack") { r = 255; g = 0; b = 0; }   // 红色
            else if (defect.type == "undercut") { r = 255; g = 165; b = 0; } // 橙色
            
            automotive_utils::draw_circle(output,
                static_cast<uint32_t>(defect.center_x),
                static_cast<uint32_t>(defect.center_y),
                8, r, g, b);
            automotive_utils::draw_cross(output,
                static_cast<uint32_t>(defect.center_x),
                static_cast<uint32_t>(defect.center_y), 5, r, g, b);
        }
    }
    
    set_output("image", Data(output));
    set_output("weld_width", Data(weld_width));
    set_output("porosity_rate", Data(porosity_rate));
    set_output("defect_count", Data(static_cast<int>(defects.size())));
    set_output("quality_score", Data(result.quality_score));
    set_output("is_acceptable", Data(result.is_acceptable));
    
    OVF_INFO() << "WeldSeamInspection: width=" << weld_width << "mm"
               << ", porosity=" << porosity_rate << "%"
               << ", defects=" << defects.size()
               << " (porosity:" << porosity_count << " crack:" << crack_count << " undercut:" << undercut_count << ")"
               << ", quality=" << result.quality_score
               << ", acceptable=" << (result.is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WeldSeamInspectionNode, "WeldSeamInspection", WeldSeamInspectionNode::make_info())

//==============================================================================
// PaintQualityInspectionNode - 涂装检测节点（橘皮/流挂/色差检测）
//==============================================================================

PaintQualityInspectionNode::PaintQualityInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PaintQualityInspectionNode::make_info() {
    NodeInfo info;
    info.id = "PaintQualityInspection";
    info.name = "涂装检测";
    info.category = "汽车零部件检测";
    info.description = "漆面质量检测：橘皮、流挂、色差、划痕、灰尘等缺陷";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("gloss_level", "光泽度", DataType::Number));
    info.outputs.push_back(DataPort("smoothness", "平滑度", DataType::Number));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "质量分数", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    
    info.params.push_back(ParamDef("min_gloss", "最小光泽度", DataType::Number, Data(70.0)));
    info.params.push_back(ParamDef("min_smoothness", "最小平滑度", DataType::Number, Data(80.0)));
    info.params.push_back(ParamDef("detect_orange_peel", "检测橘皮", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("detect_runs", "检测流挂", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.5)));
    
    return info;
}

Result<void> PaintQualityInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double min_gloss = get_param("min_gloss", Data(70.0)).as_number();
    double min_smoothness = get_param("min_smoothness", Data(80.0)).as_number();
    bool detect_orange_peel = get_param("detect_orange_peel", Data(true)).as_bool();
    bool detect_runs = get_param("detect_runs", Data(true)).as_bool();
    double sensitivity = get_param("sensitivity", Data(0.5)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 计算光泽度（基于图像对比度和反射特征）
    auto stats = automotive_utils::compute_stats(gray);
    double gloss_level = 0.0;
    
    // 光泽度计算：高对比度和高亮度表示高光泽
    double contrast = stats.max_val - stats.min_val;
    gloss_level = (stats.mean / 255.0 * 50.0) + (contrast / 255.0 * 50.0);
    
    // 计算平滑度（基于纹理分析）
    double smoothness = 100.0;
    
    // 分块分析纹理
    uint32_t block_size = 32;
    std::vector<double> block_stds;
    
    for (uint32_t by = 0; by < gray.height; by += block_size) {
        for (uint32_t bx = 0; bx < gray.width; bx += block_size) {
            double sum = 0.0, sum_sq = 0.0;
            uint32_t count = 0;
            
            for (uint32_t y = by; y < by + block_size && y < gray.height; ++y) {
                for (uint32_t x = bx; x < bx + block_size && x < gray.width; ++x) {
                    double val = gray.data[y * gray.width + x];
                    sum += val;
                    sum_sq += val * val;
                    count++;
                }
            }
            
            if (count > 0) {
                double mean = sum / count;
                double variance = (sum_sq / count) - (mean * mean);
                double std = std::sqrt(std::max(0.0, variance));
                block_stds.push_back(std);
            }
        }
    }
    
    // 计算平均纹理强度
    double avg_texture = 0.0;
    for (double std : block_stds) avg_texture += std;
    avg_texture /= block_stds.size();
    
    // 平滑度：纹理强度越低，平滑度越高
    smoothness = 100.0 - std::min(50.0, avg_texture);
    
    // 检测漆面缺陷
    std::vector<PaintDefect> defects;
    uint32_t defect_id = 1;
    int orange_peel_count = 0, run_count = 0, scratch_count = 0, dust_count = 0;
    
    // 橘皮检测：不规则的波浪纹理
    if (detect_orange_peel && avg_texture > 15) {
        // 检测橘皮区域
        for (size_t i = 0; i < block_stds.size(); ++i) {
            if (block_stds[i] > avg_texture * 1.3) {
                PaintDefect defect;
                defect.id = defect_id++;
                defect.type = "orange_peel";
                defect.severity = (block_stds[i] - avg_texture) / avg_texture;
                defect.confidence = 0.75;
                
                // 计算块位置
                uint32_t block_idx = static_cast<uint32_t>(i);
                uint32_t bx = (block_idx % (gray.width / block_size)) * block_size;
                uint32_t by = (block_idx / (gray.width / block_size)) * block_size;
                defect.center_x = bx + block_size / 2.0;
                defect.center_y = by + block_size / 2.0;
                defect.width = block_size;
                defect.height = block_size;
                
                defects.push_back(defect);
                orange_peel_count++;
            }
        }
    }
    
    // 流挂检测：垂直方向的灰度渐变条纹
    if (detect_runs) {
        ImageData blurred;
        automotive_utils::gaussian_blur(gray, blurred, 7, 2.0);
        
        for (uint32_t x = 0; x < gray.width; x += 10) {
            // 检查垂直方向的灰度渐变
            double gradient_sum = 0.0;
            for (uint32_t y = 10; y < gray.height - 10; y += 5) {
                gradient_sum += std::abs(
                    static_cast<int>(gray.data[(y+5) * gray.width + x]) -
                    static_cast<int>(gray.data[(y-5) * gray.width + x]));
            }
            
            if (gradient_sum > gray.height * 1.5) {
                PaintDefect defect;
                defect.id = defect_id++;
                defect.type = "run";
                defect.center_x = x;
                defect.center_y = gray.height / 2.0;
                defect.width = 10;
                defect.height = gray.height;
                defect.severity = std::min(1.0, gradient_sum / gray.height / 3.0);
                defect.confidence = 0.7;
                
                defects.push_back(defect);
                run_count++;
            }
        }
    }
    
    // 检测灰尘/颗粒（小的亮点或暗点）
    ImageData binary;
    automotive_utils::adaptive_threshold(gray, binary, 11, 5);
    
    std::vector<bool> visited(gray.width * gray.height, false);
    for (uint32_t y = 2; y < gray.height - 2; ++y) {
        for (uint32_t x = 2; x < gray.width - 2; ++x) {
            if (binary.data[y * gray.width + x] > 128 && !visited[y * gray.width + x]) {
                // 小区域检测
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                uint32_t area = 1;
                
                int dx[] = {-1, 0, 1, 0};
                int dy[] = {0, -1, 0, 1};
                
                while (!queue.empty() && area < 50) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    
                    for (int i = 0; i < 4; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height) &&
                            !visited[ny * gray.width + nx] &&
                            binary.data[ny * gray.width + nx] > 128) {
                            visited[ny * gray.width + nx] = true;
                            queue.push_back({nx, ny});
                            area++;
                        }
                    }
                }
                
                if (area >= 3 && area <= 20) {
                    PaintDefect defect;
                    defect.id = defect_id++;
                    defect.type = "dust";
                    defect.center_x = x;
                    defect.center_y = y;
                    defect.area = area;
                    defect.severity = 0.4;
                    defect.confidence = 0.65;
                    
                    defects.push_back(defect);
                    dust_count++;
                }
            }
        }
    }
    
    // 计算质量分数
    double quality_score = 1.0;
    
    // 光泽度影响
    if (gloss_level < min_gloss) {
        quality_score -= (min_gloss - gloss_level) / min_gloss * 0.3;
    }
    
    // 平滑度影响
    if (smoothness < min_smoothness) {
        quality_score -= (min_smoothness - smoothness) / min_smoothness * 0.3;
    }
    
    // 缺陷影响
    quality_score -= orange_peel_count * 0.15;
    quality_score -= run_count * 0.25;
    quality_score -= scratch_count * 0.1;
    quality_score -= dust_count * 0.02;
    
    quality_score = std::max(0.0, std::min(1.0, quality_score));
    
    bool is_acceptable = (quality_score > 0.7) && 
                         (gloss_level >= min_gloss) &&
                         (smoothness >= min_smoothness) &&
                         (run_count == 0);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& defect : defects) {
            uint8_t r = 0, g = 0, b = 0;
            
            if (defect.type == "orange_peel") { r = 255; g = 200; b = 0; }  // 橙色
            else if (defect.type == "run") { r = 255; g = 0; b = 0; }       // 红色
            else if (defect.type == "scratch") { r = 0; g = 255; b = 255; } // 青色
            else if (defect.type == "dust") { r = 255; g = 255; b = 0; }    // 黄色
            
            automotive_utils::draw_circle(output,
                static_cast<uint32_t>(defect.center_x),
                static_cast<uint32_t>(defect.center_y),
                10, r, g, b);
            automotive_utils::draw_cross(output,
                static_cast<uint32_t>(defect.center_x),
                static_cast<uint32_t>(defect.center_y), 5, r, g, b);
        }
    }
    
    set_output("image", Data(output));
    set_output("gloss_level", Data(gloss_level));
    set_output("smoothness", Data(smoothness));
    set_output("defect_count", Data(static_cast<int>(defects.size())));
    set_output("quality_score", Data(quality_score));
    set_output("is_acceptable", Data(is_acceptable));
    
    OVF_INFO() << "PaintQualityInspection: gloss=" << gloss_level
               << ", smoothness=" << smoothness
               << ", defects=" << defects.size()
               << " (orange_peel:" << orange_peel_count << " run:" << run_count << " dust:" << dust_count << ")"
               << ", quality=" << quality_score
               << ", acceptable=" << (is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(PaintQualityInspectionNode, "PaintQualityInspection", PaintQualityInspectionNode::make_info())

//==============================================================================
// AssemblyVerificationNode - 装配验证节点（零件完整性检测）
//==============================================================================

AssemblyVerificationNode::AssemblyVerificationNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AssemblyVerificationNode::make_info() {
    NodeInfo info;
    info.id = "AssemblyVerification";
    info.name = "装配验证";
    info.category = "汽车零部件检测";
    info.description = "验证零件装配完整性，检测缺失或多余零件";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("detected_parts", "检测到零件数", DataType::Number));
    info.outputs.push_back(DataPort("missing_parts", "缺失零件数", DataType::Number));
    info.outputs.push_back(DataPort("completeness", "完整性分数", DataType::Number));
    info.outputs.push_back(DataPort("is_complete", "是否完整", DataType::Boolean));
    
    info.params.push_back(ParamDef("expected_parts", "期望零件数", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("min_part_size", "最小零件尺寸", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("max_part_size", "最大零件尺寸", DataType::Number, Data(500)));
    info.params.push_back(ParamDef("match_threshold", "匹配阈值", DataType::Number, Data(0.7)));
    info.params.push_back(ParamDef("allow_extra", "允许多余零件", DataType::Boolean, Data(false)));
    
    return info;
}

Result<void> AssemblyVerificationNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint32_t expected_parts = static_cast<uint32_t>(get_param("expected_parts", Data(10)).as_int());
    uint32_t min_size = static_cast<uint32_t>(get_param("min_part_size", Data(20)).as_int());
    uint32_t max_size = static_cast<uint32_t>(get_param("max_part_size", Data(500)).as_int());
    double match_threshold = get_param("match_threshold", Data(0.7)).as_number();
    bool allow_extra = get_param("allow_extra", Data(false)).as_bool();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 自适应阈值
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 15, 10);
    
    // 形态学处理
    ImageData morph_close;
    automotive_utils::morphological_close(binary, morph_close, 5);
    
    // 检测零件（连通区域）
    std::vector<automotive_utils::Rect> detected_parts;
    std::vector<bool> visited(gray.width * gray.height, false);
    
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            if (morph_close.data[y * gray.width + x] > 128 && !visited[y * gray.width + x]) {
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                
                uint32_t min_x = x, max_x = x, min_y = y, max_y = y;
                uint32_t area = 1;
                
                int dx[] = {-1, 0, 1, 0, -1, -1, 1, 1};
                int dy[] = {0, -1, 0, 1, -1, 1, -1, 1};
                
                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    
                    min_x = std::min(min_x, cx);
                    max_x = std::max(max_x, cx);
                    min_y = std::min(min_y, cy);
                    max_y = std::max(max_y, cy);
                    
                    for (int i = 0; i < 8; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height) &&
                            !visited[ny * gray.width + nx] &&
                            morph_close.data[ny * gray.width + nx] > 128) {
                            visited[ny * gray.width + nx] = true;
                            queue.push_back({nx, ny});
                            area++;
                        }
                    }
                }
                
                uint32_t width = max_x - min_x + 1;
                uint32_t height = max_y - min_y + 1;
                
                if (area >= min_size && area <= max_size) {
                    automotive_utils::Rect part;
                    part.x = min_x;
                    part.y = min_y;
                    part.width = width;
                    part.height = height;
                    detected_parts.push_back(part);
                }
            }
        }
    }
    
    // 计算装配结果
    AssemblyResult result;
    result.expected_parts = expected_parts;
    result.detected_parts = static_cast<uint32_t>(detected_parts.size());
    result.missing_parts = (expected_parts > detected_parts.size()) ?
                           (expected_parts - detected_parts.size()) : 0;
    result.extra_parts = (detected_parts.size() > expected_parts) ?
                         (detected_parts.size() - expected_parts) : 0;
    
    double completeness = static_cast<double>(detected_parts.size()) / expected_parts;
    if (completeness > 1.0) completeness = allow_extra ? 1.0 : 1.0 - (completeness - 1.0);
    
    result.completeness = completeness;
    result.is_complete = (detected_parts.size() == expected_parts) ||
                         (allow_extra && detected_parts.size() >= expected_parts);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        for (size_t i = 0; i < detected_parts.size(); ++i) {
            uint8_t r = (i < expected_parts) ? 0 : 255;
            uint8_t g = (i < expected_parts) ? 255 : 0;
            uint8_t b = 0;
            
            automotive_utils::draw_rect(output,
                detected_parts[i].x, detected_parts[i].y,
                detected_parts[i].width, detected_parts[i].height,
                r, g, b, 2);
            
            // 标注零件编号
            automotive_utils::draw_cross(output,
                detected_parts[i].x + detected_parts[i].width / 2,
                detected_parts[i].y + detected_parts[i].height / 2,
                5, r, g, b);
        }
    }
    
    set_output("image", Data(output));
    set_output("detected_parts", Data(static_cast<int>(detected_parts.size())));
    set_output("missing_parts", Data(static_cast<int>(result.missing_parts)));
    set_output("completeness", Data(result.completeness));
    set_output("is_complete", Data(result.is_complete));
    
    OVF_INFO() << "AssemblyVerification: detected=" << detected_parts.size()
               << ", expected=" << expected_parts
               << ", missing=" << result.missing_parts
               << ", extra=" << result.extra_parts
               << ", completeness=" << result.completeness
               << ", complete=" << (result.is_complete ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(AssemblyVerificationNode, "AssemblyVerification", AssemblyVerificationNode::make_info())

//==============================================================================
// DimensionalInspectionNode - 尺寸检测节点（公差测量）
//==============================================================================

DimensionalInspectionNode::DimensionalInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DimensionalInspectionNode::make_info() {
    NodeInfo info;
    info.id = "DimensionalInspection";
    info.name = "尺寸检测";
    info.category = "汽车零部件检测";
    info.description = "零件尺寸测量，公差范围验证";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("measured_value", "测量值", DataType::Number));
    info.outputs.push_back(DataPort("deviation", "偏差", DataType::Number));
    info.outputs.push_back(DataPort("in_tolerance", "是否在公差内", DataType::Boolean));
    info.outputs.push_back(DataPort("measurements", "测量结果列表", DataType::Array));
    
    info.params.push_back(ParamDef("nominal_value", "标称值", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("tolerance_upper", "上公差", DataType::Number, Data(2.0)));
    info.params.push_back(ParamDef("tolerance_lower", "下公差", DataType::Number, Data(2.0)));
    info.params.push_back(ParamDef("measure_type", "测量类型", DataType::String, Data("width")));
    info.params.push_back(ParamDef("pixel_scale", "像素比例(mm/pixel)", DataType::Number, Data(0.1)));
    info.params.push_back(ParamDef("measure_points", "测量点数", DataType::Number, Data(5)));
    
    return info;
}

Result<void> DimensionalInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double nominal_value = get_param("nominal_value", Data(100.0)).as_number();
    double tolerance_upper = get_param("tolerance_upper", Data(2.0)).as_number();
    double tolerance_lower = get_param("tolerance_lower", Data(2.0)).as_number();
    std::string measure_type = get_param("measure_type", Data("width")).as_string();
    double pixel_scale = get_param("pixel_scale", Data(0.1)).as_number();
    int measure_points = get_param("measure_points", Data(5)).as_int();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 阈值分割
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 15, 15);
    
    // 尺寸测量
    std::vector<double> measurements;
    double measured_value = 0.0;
    
    if (measure_type == "width") {
        // 测量宽度（水平方向）
        for (int p = 0; p < measure_points; ++p) {
            uint32_t y = gray.height * (p + 1) / (measure_points + 1);
            
            uint32_t left_edge = 0, right_edge = gray.width;
            for (uint32_t x = 0; x < gray.width; ++x) {
                if (binary.data[y * gray.width + x] > 128) {
                    left_edge = x;
                    break;
                }
            }
            for (uint32_t x = gray.width - 1; x > 0; --x) {
                if (binary.data[y * gray.width + x] > 128) {
                    right_edge = x;
                    break;
                }
            }
            
            double width_pixels = right_edge - left_edge;
            double width_mm = width_pixels * pixel_scale;
            measurements.push_back(width_mm);
        }
        
        // 取平均值
        measured_value = std::accumulate(measurements.begin(), measurements.end(), 0.0) / measurements.size();
    } 
    else if (measure_type == "height") {
        // 测量高度（垂直方向）
        for (int p = 0; p < measure_points; ++p) {
            uint32_t x = gray.width * (p + 1) / (measure_points + 1);
            
            uint32_t top_edge = 0, bottom_edge = gray.height;
            for (uint32_t y = 0; y < gray.height; ++y) {
                if (binary.data[y * gray.width + x] > 128) {
                    top_edge = y;
                    break;
                }
            }
            for (uint32_t y = gray.height - 1; y > 0; --y) {
                if (binary.data[y * gray.width + x] > 128) {
                    bottom_edge = y;
                    break;
                }
            }
            
            double height_pixels = bottom_edge - top_edge;
            double height_mm = height_pixels * pixel_scale;
            measurements.push_back(height_mm);
        }
        
        measured_value = std::accumulate(measurements.begin(), measurements.end(), 0.0) / measurements.size();
    }
    else if (measure_type == "diameter") {
        // 测量直径（圆形零件）
        // 检测圆
        auto circles = automotive_utils::detect_circles(binary, 10, gray.width/2, 20);
        if (!circles.empty()) {
            measured_value = circles[0].second * 2 * pixel_scale;
            measurements.push_back(measured_value);
        }
    }
    
    // 计算偏差
    double deviation = measured_value - nominal_value;
    bool in_tolerance = (deviation >= -tolerance_lower) && (deviation <= tolerance_upper);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制测量线
        for (int p = 0; p < measure_points && p < static_cast<int>(measurements.size()); ++p) {
            uint32_t pos = (measure_type == "width") ?
                          gray.height * (p + 1) / (measure_points + 1) :
                          gray.width * (p + 1) / (measure_points + 1);
            
            uint8_t r = in_tolerance ? 0 : 255;
            uint8_t g = in_tolerance ? 255 : 0;
            
            if (measure_type == "width") {
                automotive_utils::draw_line(output, 0, pos, gray.width, pos, r, g, 0);
            } else {
                automotive_utils::draw_line(output, pos, 0, pos, gray.height, r, g, 0);
            }
        }
        
        // 绘制测量值标注
        automotive_utils::draw_rect(output, 10, 10, 150, 40,
            in_tolerance ? 0 : 255, in_tolerance ? 255 : 0, 0, 2);
    }
    
    set_output("image", Data(output));
    set_output("measured_value", Data(measured_value));
    set_output("deviation", Data(deviation));
    set_output("in_tolerance", Data(in_tolerance));
    
    OVF_INFO() << "DimensionalInspection: measured=" << measured_value << "mm"
               << ", nominal=" << nominal_value << "mm"
               << ", deviation=" << deviation << "mm"
               << ", tolerance=[" << (-tolerance_lower) << ", " << tolerance_upper << "]"
               << ", in_tolerance=" << (in_tolerance ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(DimensionalInspectionNode, "DimensionalInspection", DimensionalInspectionNode::make_info())

//==============================================================================
// GearInspectionNode - 齿轮检测节点（齿形/磨损检测）
//==============================================================================

GearInspectionNode::GearInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GearInspectionNode::make_info() {
    NodeInfo info;
    info.id = "GearInspection";
    info.name = "齿轮检测";
    info.category = "汽车零部件检测";
    info.description = "齿轮齿形检测、齿厚齿距测量、磨损检测";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("total_teeth", "总齿数", DataType::Number));
    info.outputs.push_back(DataPort("good_teeth", "良好齿数", DataType::Number));
    info.outputs.push_back(DataPort("tooth_thickness", "齿厚(mm)", DataType::Number));
    info.outputs.push_back(DataPort("tooth_pitch", "齿距(mm)", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "质量分数", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    
    info.params.push_back(ParamDef("expected_teeth", "期望齿数", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("min_thickness", "最小齿厚(mm)", DataType::Number, Data(2.0)));
    info.params.push_back(ParamDef("max_pitch_error", "最大齿距误差(mm)", DataType::Number, Data(0.1)));
    info.params.push_back(ParamDef("pixel_scale", "像素比例(mm/pixel)", DataType::Number, Data(0.05)));
    info.params.push_back(ParamDef("detect_wear", "检测磨损", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> GearInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint32_t expected_teeth = static_cast<uint32_t>(get_param("expected_teeth", Data(20)).as_int());
    double min_thickness = get_param("min_thickness", Data(2.0)).as_number();
    double max_pitch_error = get_param("max_pitch_error", Data(0.1)).as_number();
    double pixel_scale = get_param("pixel_scale", Data(0.05)).as_number();
    bool detect_wear = get_param("detect_wear", Data(true)).as_bool();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 阈值分割
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 11, 10);
    
    // 齿轮中心检测（假设齿轮在图像中心）
    uint32_t center_x = gray.width / 2;
    uint32_t center_y = gray.height / 2;
    
    // 检测齿轮外圆半径
    uint32_t outer_radius = 0;
    for (uint32_t r = gray.width / 4; r < gray.width / 2; ++r) {
        int edge_count = 0;
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 180) {
            uint32_t x = static_cast<uint32_t>(center_x + r * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + r * std::sin(angle));
            
            if (x < gray.width && y < gray.height && 
                binary.data[y * gray.width + x] > 128) {
                edge_count++;
            }
        }
        
        if (edge_count > 300) {
            outer_radius = r;
            break;
        }
    }
    
    // 齿数检测：通过角度扫描检测齿峰和齿谷
    std::vector<double> radial_profile;
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 360) {
        double avg_intensity = 0.0;
        int count = 0;
        
        for (uint32_t r = outer_radius - 20; r <= outer_radius; r += 5) {
            uint32_t x = static_cast<uint32_t>(center_x + r * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + r * std::sin(angle));
            
            if (x < gray.width && y < gray.height) {
                avg_intensity += gray.data[y * gray.width + x];
                count++;
            }
        }
        
        if (count > 0) radial_profile.push_back(avg_intensity / count);
    }
    
    // 分析齿峰和齿谷
    uint32_t total_teeth = 0;
    std::vector<double> tooth_positions;
    double last_val = radial_profile[0];
    bool rising = false;
    
    for (size_t i = 1; i < radial_profile.size(); ++i) {
        if (radial_profile[i] > last_val && !rising) {
            rising = true;
        } else if (radial_profile[i] < last_val && rising) {
            rising = false;
            total_teeth++;
            tooth_positions.push_back(i * M_PI / 360);
        }
        last_val = radial_profile[i];
    }
    
    // 齿厚测量（简化版）
    double tooth_thickness = 0.0;
    if (total_teeth > 0) {
        // 齿厚 = 齿峰宽度的一半
        tooth_thickness = outer_radius * M_PI / total_teeth * 0.5 * pixel_scale;
    }
    
    // 齿距测量
    double tooth_pitch = 0.0;
    if (total_teeth > 1) {
        // 齿距 = 齿轮周长 / 齿数
        tooth_pitch = 2 * M_PI * outer_radius / total_teeth * pixel_scale;
    }
    
    // 齿距误差计算
    double pitch_error = 0.0;
    if (tooth_positions.size() > 1) {
        std::vector<double> pitches;
        for (size_t i = 1; i < tooth_positions.size(); ++i) {
            pitches.push_back(tooth_positions[i] - tooth_positions[i-1]);
        }
        
        // 计算标准偏差作为齿距误差
        double mean_pitch = 2 * M_PI / total_teeth;
        for (double p : pitches) {
            pitch_error += std::abs(p - mean_pitch);
        }
        pitch_error = pitch_error / pitches.size() * outer_radius * pixel_scale;
    }
    
    // 磨损检测（齿形偏差）
    uint32_t defective_teeth = 0;
    double wear_level = 0.0;
    
    if (detect_wear) {
        for (size_t i = 0; i < radial_profile.size(); ++i) {
            // 检查齿峰是否低于期望值
            double expected_peak = 255.0;
            if (radial_profile[i] > 200) {  // 齿峰
                if (radial_profile[i] < expected_peak * 0.9) {
                    defective_teeth++;
                    wear_level += (expected_peak - radial_profile[i]) / expected_peak;
                }
            }
        }
        
        if (defective_teeth > 0) wear_level /= defective_teeth;
    }
    
    uint32_t good_teeth = total_teeth - defective_teeth;
    
    // 质量评估
    double quality_score = 1.0;
    
    if (total_teeth != expected_teeth) {
        quality_score -= 0.3;
    }
    
    if (tooth_thickness < min_thickness) {
        quality_score -= 0.25;
    }
    
    if (pitch_error > max_pitch_error) {
        quality_score -= 0.2;
    }
    
    quality_score -= defective_teeth * 0.1;
    quality_score -= wear_level * 0.2;
    
    quality_score = std::max(0.0, std::min(1.0, quality_score));
    
    bool is_acceptable = (total_teeth == expected_teeth) &&
                         (tooth_thickness >= min_thickness) &&
                         (pitch_error <= max_pitch_error) &&
                         (defective_teeth < total_teeth * 0.1);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制齿轮中心
        automotive_utils::draw_cross(output, center_x, center_y, 10, 255, 0, 0);
        
        // 绘制外圆
        automotive_utils::draw_circle(output, center_x, center_y, outer_radius, 0, 255, 0);
        
        // 标注齿位
        for (size_t i = 0; i < tooth_positions.size(); ++i) {
            uint32_t x = static_cast<uint32_t>(center_x + outer_radius * std::cos(tooth_positions[i]));
            uint32_t y = static_cast<uint32_t>(center_y + outer_radius * std::sin(tooth_positions[i]));
            
            uint8_t r = (i < defective_teeth) ? 255 : 0;
            uint8_t g = (i < defective_teeth) ? 0 : 255;
            
            automotive_utils::draw_cross(output, x, y, 5, r, g, 0);
        }
    }
    
    set_output("image", Data(output));
    set_output("total_teeth", Data(static_cast<int>(total_teeth)));
    set_output("good_teeth", Data(static_cast<int>(good_teeth)));
    set_output("tooth_thickness", Data(tooth_thickness));
    set_output("tooth_pitch", Data(tooth_pitch));
    set_output("quality_score", Data(quality_score));
    set_output("is_acceptable", Data(is_acceptable));
    
    OVF_INFO() << "GearInspection: teeth=" << total_teeth << "/" << expected_teeth
               << ", good=" << good_teeth
               << ", defective=" << defective_teeth
               << ", thickness=" << tooth_thickness << "mm"
               << ", pitch=" << tooth_pitch << "mm"
               << ", pitch_error=" << pitch_error << "mm"
               << ", wear=" << wear_level
               << ", quality=" << quality_score
               << ", acceptable=" << (is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(GearInspectionNode, "GearInspection", GearInspectionNode::make_info())

//==============================================================================
// ConnectorInspectionNode - 连接器检测节点（插针位置检测）
//==============================================================================

ConnectorInspectionNode::ConnectorInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ConnectorInspectionNode::make_info() {
    NodeInfo info;
    info.id = "ConnectorInspection";
    info.name = "连接器检测";
    info.category = "汽车零部件检测";
    info.description = "检测连接器插针位置、弯曲、缺失等缺陷";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("total_pins", "总插针数", DataType::Number));
    info.outputs.push_back(DataPort("correct_pins", "正确插针数", DataType::Number));
    info.outputs.push_back(DataPort("bent_pins", "弯曲插针数", DataType::Number));
    info.outputs.push_back(DataPort("missing_pins", "缺失插针数", DataType::Number));
    info.outputs.push_back(DataPort("alignment_score", "对准分数", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    
    info.params.push_back(ParamDef("expected_pins", "期望插针数", DataType::Number, Data(16)));
    info.params.push_back(ParamDef("pin_spacing", "插针间距(pixels)", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("max_bend_angle", "最大弯曲角度", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("alignment_tolerance", "对准容差(pixels)", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("row_count", "行数", DataType::Number, Data(2)));
    info.params.push_back(ParamDef("col_count", "列数", DataType::Number, Data(8)));
    
    return info;
}

Result<void> ConnectorInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint32_t expected_pins = static_cast<uint32_t>(get_param("expected_pins", Data(16)).as_int());
    uint32_t pin_spacing = static_cast<uint32_t>(get_param("pin_spacing", Data(20)).as_int());
    double max_bend_angle = get_param("max_bend_angle", Data(5.0)).as_number();
    uint32_t alignment_tolerance = static_cast<uint32_t>(get_param("alignment_tolerance", Data(3)).as_int());
    uint32_t row_count = static_cast<uint32_t>(get_param("row_count", Data(2)).as_int());
    uint32_t col_count = static_cast<uint32_t>(get_param("col_count", Data(8)).as_int());
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 阈值分割
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 11, 10);
    
    // 检测插针（圆形或矩形特征）
    std::vector<std::pair<uint32_t, uint32_t>> detected_pins;
    std::vector<bool> visited(gray.width * gray.height, false);
    
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            if (binary.data[y * gray.width + x] > 128 && !visited[y * gray.width + x]) {
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                
                uint32_t min_x = x, max_x = x, min_y = y, max_y = y;
                uint32_t area = 1;
                
                int dx[] = {-1, 0, 1, 0};
                int dy[] = {0, -1, 0, 1};
                
                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    
                    min_x = std::min(min_x, cx);
                    max_x = std::max(max_x, cx);
                    min_y = std::min(min_y, cy);
                    max_y = std::max(max_y, cy);
                    
                    for (int i = 0; i < 4; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height) &&
                            !visited[ny * gray.width + nx] &&
                            binary.data[ny * gray.width + nx] > 128) {
                            visited[ny * gray.width + nx] = true;
                            queue.push_back({nx, ny});
                            area++;
                        }
                    }
                }
                
                // 检测插针特征
                uint32_t width = max_x - min_x + 1;
                uint32_t height = max_y - min_y + 1;
                
                if (area >= 15 && area <= 200 && 
                    std::abs(static_cast<int>(width) - static_cast<int>(height)) < 10) {
                    detected_pins.push_back({(min_x + max_x) / 2, (min_y + max_y) / 2});
                }
            }
        }
    }
    
    // 分析插针状态
    uint32_t total_pins = static_cast<uint32_t>(detected_pins.size());
    uint32_t correct_pins = 0, bent_pins = 0, missing_pins = 0, misaligned_pins = 0;
    
    // 确定连接器基准位置
    uint32_t start_x = gray.width / 2 - col_count * pin_spacing / 2;
    uint32_t start_y = gray.height / 2 - row_count * pin_spacing / 2;
    
    // 检查每个期望位置的插针
    for (uint32_t row = 0; row < row_count; ++row) {
        for (uint32_t col = 0; col < col_count; ++col) {
            uint32_t expected_x = start_x + col * pin_spacing;
            uint32_t expected_y = start_y + row * pin_spacing;
            
            bool pin_found = false;
            uint32_t closest_pin_idx = 0;
            uint32_t min_dist = UINT32_MAX;
            
            // 寻找最近的插针
            for (size_t i = 0; i < detected_pins.size(); ++i) {
                uint32_t dist = std::abs(static_cast<int>(detected_pins[i].first) - static_cast<int>(expected_x)) +
                               std::abs(static_cast<int>(detected_pins[i].second) - static_cast<int>(expected_y));
                
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_pin_idx = i;
                }
            }
            
            if (min_dist <= pin_spacing) {
                pin_found = true;
                detected_pins[closest_pin_idx] = {UINT32_MAX, UINT32_MAX};  // 标记已使用
                
                // 检查对准偏差
                if (min_dist <= alignment_tolerance) {
                    correct_pins++;
                } else {
                    misaligned_pins++;
                }
                
                // 检查弯曲（基于插针形状）
                uint32_t dx = std::abs(static_cast<int>(detected_pins[closest_pin_idx].first) - static_cast<int>(expected_x));
                uint32_t dy = std::abs(static_cast<int>(detected_pins[closest_pin_idx].second) - static_cast<int>(expected_y));
                
                double bend_angle = std::atan2(dy, dx) * 180.0 / M_PI;
                if (bend_angle > max_bend_angle) {
                    bent_pins++;
                }
            } else {
                missing_pins++;
            }
        }
    }
    
    // 计算对准分数
    double alignment_score = static_cast<double>(correct_pins) / expected_pins;
    
    // 质量评估
    bool is_acceptable = (missing_pins == 0) &&
                         (bent_pins <= expected_pins * 0.05) &&
                         (misaligned_pins <= expected_pins * 0.1) &&
                         (alignment_score >= 0.9);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制期望插针位置网格
        for (uint32_t row = 0; row < row_count; ++row) {
            for (uint32_t col = 0; col < col_count; ++col) {
                uint32_t x = start_x + col * pin_spacing;
                uint32_t y = start_y + row * pin_spacing;
                
                // 检查是否有插针
                bool has_pin = false;
                for (const auto& pin : detected_pins) {
                    uint32_t dist = std::abs(static_cast<int>(pin.first) - static_cast<int>(x)) +
                                   std::abs(static_cast<int>(pin.second) - static_cast<int>(y));
                    if (dist <= pin_spacing) has_pin = true;
                }
                
                uint8_t r = has_pin ? 0 : 255;
                uint8_t g = has_pin ? 255 : 0;
                
                automotive_utils::draw_circle(output, x, y, 5, r, g, 0);
                automotive_utils::draw_cross(output, x, y, 3, r, g, 0);
            }
        }
    }
    
    set_output("image", Data(output));
    set_output("total_pins", Data(static_cast<int>(total_pins)));
    set_output("correct_pins", Data(static_cast<int>(correct_pins)));
    set_output("bent_pins", Data(static_cast<int>(bent_pins)));
    set_output("missing_pins", Data(static_cast<int>(missing_pins)));
    set_output("alignment_score", Data(alignment_score));
    set_output("is_acceptable", Data(is_acceptable));
    
    OVF_INFO() << "ConnectorInspection: total=" << total_pins
               << ", correct=" << correct_pins
               << ", bent=" << bent_pins
               << ", missing=" << missing_pins
               << ", misaligned=" << misaligned_pins
               << ", alignment=" << alignment_score
               << ", acceptable=" << (is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(ConnectorInspectionNode, "ConnectorInspection", ConnectorInspectionNode::make_info())

//==============================================================================
// BoltPresenceCheckNode - 螺栓检测节点（漏装检测）
//==============================================================================

BoltPresenceCheckNode::BoltPresenceCheckNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BoltPresenceCheckNode::make_info() {
    NodeInfo info;
    info.id = "BoltPresenceCheck";
    info.name = "螺栓检测";
    info.category = "汽车零部件检测";
    info.description = "螺栓存在性验证，检测漏装螺栓";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("detected_bolts", "检测到螺栓数", DataType::Number));
    info.outputs.push_back(DataPort("missing_bolts", "缺失螺栓数", DataType::Number));
    info.outputs.push_back(DataPort("presence_rate", "存在率", DataType::Number));
    info.outputs.push_back(DataPort("is_complete", "是否完整", DataType::Boolean));
    
    info.params.push_back(ParamDef("expected_bolts", "期望螺栓数", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("bolt_diameter", "螺栓直径(pixels)", DataType::Number, Data(30)));
    info.params.push_back(ParamDef("positions", "螺栓位置列表", DataType::String, Data("")));
    info.params.push_back(ParamDef("tolerance", "位置容差(pixels)", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("confidence_threshold", "置信阈值", DataType::Number, Data(0.7)));
    
    return info;
}

Result<void> BoltPresenceCheckNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint32_t expected_bolts = static_cast<uint32_t>(get_param("expected_bolts", Data(8)).as_int());
    uint32_t bolt_diameter = static_cast<uint32_t>(get_param("bolt_diameter", Data(30)).as_int());
    uint32_t tolerance = static_cast<uint32_t>(get_param("tolerance", Data(10)).as_int());
    double confidence_threshold = get_param("confidence_threshold", Data(0.7)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 阈值分割
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 11, 8);
    
    // 形态学处理
    ImageData morph_close;
    automotive_utils::morphological_close(binary, morph_close, 5);
    
    // 检测圆形螺栓
    auto circles = automotive_utils::detect_circles(morph_close, 
                                                     bolt_diameter / 2, 
                                                     bolt_diameter * 2,
                                                     40);
    
    uint32_t detected_bolts = static_cast<uint32_t>(circles.size());
    uint32_t missing_bolts = (expected_bolts > detected_bolts) ?
                            (expected_bolts - detected_bolts) : 0;
    
    double presence_rate = static_cast<double>(detected_bolts) / expected_bolts;
    if (presence_rate > 1.0) presence_rate = 1.0;
    
    bool is_complete = (detected_bolts >= expected_bolts);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制检测到的螺栓
        for (const auto& circle : circles) {
            uint32_t cx = static_cast<uint32_t>(circle.first.x);
            uint32_t cy = static_cast<uint32_t>(circle.first.y);
            uint32_t radius = static_cast<uint32_t>(circle.second);
            
            // 绿色表示检测到
            automotive_utils::draw_circle(output, cx, cy, radius, 0, 255, 0);
            automotive_utils::draw_cross(output, cx, cy, 10, 0, 255, 0);
        }
        
        // 绘制期望位置（如果螺栓缺失）
        // 简化：假设期望位置均匀分布
        for (uint32_t i = 0; i < expected_bolts && i < detected_bolts + missing_bolts; ++i) {
            if (i >= detected_bolts) {
                // 缺失螺栓位置
                uint32_t x = gray.width * (i + 1) / (expected_bolts + 1);
                uint32_t y = gray.height / 2;
                
                // 红色表示缺失
                automotive_utils::draw_circle(output, x, y, bolt_diameter / 2, 255, 0, 0);
                automotive_utils::draw_cross(output, x, y, 10, 255, 0, 0);
            }
        }
    }
    
    set_output("image", Data(output));
    set_output("detected_bolts", Data(static_cast<int>(detected_bolts)));
    set_output("missing_bolts", Data(static_cast<int>(missing_bolts)));
    set_output("presence_rate", Data(presence_rate));
    set_output("is_complete", Data(is_complete));
    
    OVF_INFO() << "BoltPresenceCheck: detected=" << detected_bolts
               << ", expected=" << expected_bolts
               << ", missing=" << missing_bolts
               << ", presence=" << presence_rate
               << ", complete=" << (is_complete ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(BoltPresenceCheckNode, "BoltPresenceCheck", BoltPresenceCheckNode::make_info())

//==============================================================================
// SurfaceRoughnessInspectionNode - 表面粗糙度检测节点
//==============================================================================

SurfaceRoughnessInspectionNode::SurfaceRoughnessInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SurfaceRoughnessInspectionNode::make_info() {
    NodeInfo info;
    info.id = "SurfaceRoughnessInspection";
    info.name = "表面粗糙度检测";
    info.category = "汽车零部件检测";
    info.description = "表面粗糙度测量：Ra、Rz、Rp、Rv等参数";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("depth_map", "深度图", DataType::DepthImage, false));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("ra", "Ra粗糙度(μm)", DataType::Number));
    info.outputs.push_back(DataPort("rz", "Rz粗糙度(μm)", DataType::Number));
    info.outputs.push_back(DataPort("peak_to_valley", "峰谷差(μm)", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    info.outputs.push_back(DataPort("roughness_result", "粗糙度结果", DataType::Object));
    
    info.params.push_back(ParamDef("max_ra", "最大Ra值(μm)", DataType::Number, Data(1.6)));
    info.params.push_back(ParamDef("max_rz", "最大Rz值(μm)", DataType::Number, Data(6.3)));
    info.params.push_back(ParamDef("pixel_scale", "像素比例(μm/pixel)", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("measure_length", "测量长度(pixels)", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("sampling_interval", "采样间隔", DataType::Number, Data(5)));
    
    return info;
}

Result<void> SurfaceRoughnessInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double max_ra = get_param("max_ra", Data(1.6)).as_number();
    double max_rz = get_param("max_rz", Data(6.3)).as_number();
    double pixel_scale = get_param("pixel_scale", Data(0.5)).as_number();
    uint32_t measure_length = static_cast<uint32_t>(get_param("measure_length", Data(100)).as_int());
    uint32_t sampling_interval = static_cast<uint32_t>(get_param("sampling_interval", Data(5)).as_int());
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 采样表面轮廓（沿中心线）
    std::vector<double> profile;
    uint32_t center_y = gray.height / 2;
    uint32_t start_x = gray.width / 2 - measure_length / 2;
    
    for (uint32_t x = start_x; x < start_x + measure_length && x < gray.width; x += sampling_interval) {
        profile.push_back(gray.data[center_y * gray.width + x]);
    }
    
    // 计算粗糙度参数
    // Ra (平均粗糙度)
    double mean = std::accumulate(profile.begin(), profile.end(), 0.0) / profile.size();
    double ra = 0.0;
    for (double val : profile) {
        ra += std::abs(val - mean);
    }
    ra = ra / profile.size() * pixel_scale;
    
    // Rz (十点高度)
    std::vector<double> sorted_profile = profile;
    std::sort(sorted_profile.begin(), sorted_profile.end());
    
    double rp = 0.0, rv = 0.0;
    size_t n = sorted_profile.size();
    
    // 取5个最高峰和5个最低谷的平均值
    for (size_t i = 0; i < 5 && i < n; ++i) {
        rp += sorted_profile[n - 1 - i];  // 高峰
        rv += sorted_profile[i];           // 低谷
    }
    rp /= 5;
    rv /= 5;
    
    double rz = (rp - rv) * pixel_scale;
    double peak_to_valley = (sorted_profile[n-1] - sorted_profile[0]) * pixel_scale;
    
    // Rq (RMS粗糙度)
    double rq = 0.0;
    for (double val : profile) {
        rq += (val - mean) * (val - mean);
    }
    rq = std::sqrt(rq / profile.size()) * pixel_scale;
    
    // 判断是否合格
    bool is_acceptable = (ra <= max_ra) && (rz <= max_rz);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制测量线
        automotive_utils::draw_line(output, start_x, center_y, 
                                    start_x + measure_length, center_y, 
                                    0, 255, 0);
        
        // 绘制采样点
        for (uint32_t x = start_x; x < start_x + measure_length && x < gray.width; 
             x += sampling_interval) {
            automotive_utils::draw_cross(output, x, center_y, 3, 255, 255, 0);
        }
        
        // 绘制粗糙度可视化
        uint32_t graph_y = gray.height - 50;
        automotive_utils::draw_line(output, start_x, graph_y, 
                                    start_x + measure_length, graph_y, 
                                    128, 128, 128);
        
        for (size_t i = 0; i < profile.size() - 1; ++i) {
            uint32_t x1 = start_x + i * sampling_interval;
            uint32_t x2 = start_x + (i + 1) * sampling_interval;
            uint32_t y1 = graph_y - static_cast<uint32_t>((profile[i] - mean) * 0.5);
            uint32_t y2 = graph_y - static_cast<uint32_t>((profile[i+1] - mean) * 0.5);
            
            automotive_utils::draw_line(output, x1, y1, x2, y2, 
                                        is_acceptable ? 0 : 255,
                                        is_acceptable ? 255 : 0, 0);
        }
    }
    
    set_output("image", Data(output));
    set_output("ra", Data(ra));
    set_output("rz", Data(rz));
    set_output("peak_to_valley", Data(peak_to_valley));
    set_output("is_acceptable", Data(is_acceptable));
    
    OVF_INFO() << "SurfaceRoughnessInspection: Ra=" << ra << "μm"
               << ", Rz=" << rz << "μm"
               << ", Rp=" << rp * pixel_scale << "μm"
               << ", Rv=" << rv * pixel_scale << "μm"
               << ", Rq=" << rq << "μm"
               << ", peak_to_valley=" << peak_to_valley << "μm"
               << ", acceptable=" << (is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(SurfaceRoughnessInspectionNode, "SurfaceRoughnessInspection", SurfaceRoughnessInspectionNode::make_info())

//==============================================================================
// GapMeasurementNode - 间隙测量节点（面板间隙公差检测）
//==============================================================================

GapMeasurementNode::GapMeasurementNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GapMeasurementNode::make_info() {
    NodeInfo info;
    info.id = "GapMeasurement";
    info.name = "间隙测量";
    info.category = "汽车零部件检测";
    info.description = "面板间隙测量，公差验证";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("gap_width", "间隙宽度(mm)", DataType::Number));
    info.outputs.push_back(DataPort("deviation", "偏差(mm)", DataType::Number));
    info.outputs.push_back(DataPort("uniformity", "均匀性分数", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    info.outputs.push_back(DataPort("measurements", "测量点列表", DataType::Array));
    
    info.params.push_back(ParamDef("nominal_gap", "标称间隙(mm)", DataType::Number, Data(3.0)));
    info.params.push_back(ParamDef("tolerance", "公差(mm)", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("pixel_scale", "像素比例(mm/pixel)", DataType::Number, Data(0.1)));
    info.params.push_back(ParamDef("measure_points", "测量点数", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("gap_direction", "间隙方向", DataType::String, Data("horizontal")));
    
    return info;
}

Result<void> GapMeasurementNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double nominal_gap = get_param("nominal_gap", Data(3.0)).as_number();
    double tolerance = get_param("tolerance", Data(0.5)).as_number();
    double pixel_scale = get_param("pixel_scale", Data(0.1)).as_number();
    int measure_points = get_param("measure_points", Data(10)).as_int();
    std::string gap_direction = get_param("gap_direction", Data("horizontal")).as_string();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray = create_image(input.width, input.height, 1, ImageFormat::Mono8);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    automotive_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 阈值分割
    ImageData binary;
    automotive_utils::adaptive_threshold(edge_mag, binary, 11, 10);
    
    // 间隙测量
    std::vector<double> gap_measurements;
    
    if (gap_direction == "horizontal") {
        // 水平间隙测量
        for (int p = 0; p < measure_points; ++p) {
            uint32_t y = gray.height * (p + 1) / (measure_points + 1);
            
            // 搜索间隙（暗区域）
            uint32_t gap_start = 0, gap_end = gray.width;
            bool in_gap = false;
            
            for (uint32_t x = gray.width / 4; x < gray.width * 3 / 4; ++x) {
                if (gray.data[y * gray.width + x] < 50 && !in_gap) {
                    gap_start = x;
                    in_gap = true;
                } else if (gray.data[y * gray.width + x] > 100 && in_gap) {
                    gap_end = x;
                    break;
                }
            }
            
            if (gap_end > gap_start) {
                double gap_pixels = gap_end - gap_start;
                double gap_mm = gap_pixels * pixel_scale;
                gap_measurements.push_back(gap_mm);
            }
        }
    } 
    else if (gap_direction == "vertical") {
        // 垂直间隙测量
        for (int p = 0; p < measure_points; ++p) {
            uint32_t x = gray.width * (p + 1) / (measure_points + 1);
            
            uint32_t gap_start = 0, gap_end = gray.height;
            bool in_gap = false;
            
            for (uint32_t y = gray.height / 4; y < gray.height * 3 / 4; ++y) {
                if (gray.data[y * gray.width + x] < 50 && !in_gap) {
                    gap_start = y;
                    in_gap = true;
                } else if (gray.data[y * gray.width + x] > 100 && in_gap) {
                    gap_end = y;
                    break;
                }
            }
            
            if (gap_end > gap_start) {
                double gap_pixels = gap_end - gap_start;
                double gap_mm = gap_pixels * pixel_scale;
                gap_measurements.push_back(gap_mm);
            }
        }
    }
    
    // 计算统计数据
    double gap_width = 0.0;
    double uniformity = 0.0;
    
    if (!gap_measurements.empty()) {
        gap_width = std::accumulate(gap_measurements.begin(), gap_measurements.end(), 0.0) / 
                   gap_measurements.size();
        
        // 计算均匀性（基于标准差）
        double variance = 0.0;
        for (double g : gap_measurements) {
            variance += (g - gap_width) * (g - gap_width);
        }
        double std_dev = std::sqrt(variance / gap_measurements.size());
        
        uniformity = 1.0 - std::min(1.0, std_dev / tolerance);
    }
    
    // 计算偏差
    double deviation = gap_width - nominal_gap;
    
    // 判断是否合格
    bool is_acceptable = (std::abs(deviation) <= tolerance) && (uniformity >= 0.8);
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制测量点
        for (int p = 0; p < measure_points && p < static_cast<int>(gap_measurements.size()); ++p) {
            uint32_t pos = (gap_direction == "horizontal") ?
                          gray.height * (p + 1) / (measure_points + 1) :
                          gray.width * (p + 1) / (measure_points + 1);
            
            uint8_t r = std::abs(gap_measurements[p] - nominal_gap) > tolerance ? 255 : 0;
            uint8_t g = std::abs(gap_measurements[p] - nominal_gap) > tolerance ? 0 : 255;
            
            if (gap_direction == "horizontal") {
                automotive_utils::draw_line(output, 
                    gray.width * 0.3, pos, gray.width * 0.7, pos, r, g, 0);
                automotive_utils::draw_cross(output, gray.width / 2, pos, 5, r, g, 0);
            } else {
                automotive_utils::draw_line(output, 
                    pos, gray.height * 0.3, pos, gray.height * 0.7, r, g, 0);
                automotive_utils::draw_cross(output, pos, gray.height / 2, 5, r, g, 0);
            }
        }
        
        // 绘制间隙区域标记
        uint32_t center = (gap_direction == "horizontal") ? gray.height / 2 : gray.width / 2;
        automotive_utils::draw_rect(output, 
            (gap_direction == "horizontal") ? gray.width * 0.4 : center - 10,
            (gap_direction == "horizontal") ? center - 10 : gray.height * 0.4,
            (gap_direction == "horizontal") ? gray.width * 0.2 : 20,
            (gap_direction == "horizontal") ? 20 : gray.height * 0.2,
            is_acceptable ? 0 : 255, is_acceptable ? 255 : 0, 0, 2);
    }
    
    set_output("image", Data(output));
    set_output("gap_width", Data(gap_width));
    set_output("deviation", Data(deviation));
    set_output("uniformity", Data(uniformity));
    set_output("is_acceptable", Data(is_acceptable));
    
    OVF_INFO() << "GapMeasurement: width=" << gap_width << "mm"
               << ", nominal=" << nominal_gap << "mm"
               << ", deviation=" << deviation << "mm"
               << ", tolerance=" << tolerance << "mm"
               << ", uniformity=" << uniformity
               << ", measurements=" << gap_measurements.size()
               << ", acceptable=" << (is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(GapMeasurementNode, "GapMeasurement", GapMeasurementNode::make_info())

} // namespace algorithm
} // namespace ovf