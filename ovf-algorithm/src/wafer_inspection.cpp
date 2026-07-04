/**
 * @file wafer_inspection.cpp
 * @brief 晶圆检测节点实现
 */

#include "ovf/algorithm/wafer_inspection.h"
#include "ovf/core/logger.h"
#include <sstream>
#include <cmath>

namespace ovf {
namespace algorithm {

//==============================================================================
// WaferDieDetectionNode - 晶粒定位检测节点
//==============================================================================

WaferDieDetectionNode::WaferDieDetectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferDieDetectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferDieDetection";
    info.name = "晶粒定位检测";
    info.category = "晶圆检测";
    info.description = "网格化晶粒定位，检测晶圆上每个晶粒的位置和状态";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("die_count", "晶粒数量", DataType::Number));
    info.outputs.push_back(DataPort("good_count", "良品数量", DataType::Number));
    info.outputs.push_back(DataPort("defect_count", "缺陷数量", DataType::Number));
    info.outputs.push_back(DataPort("die_list", "晶粒列表", DataType::Array));
    
    info.params.push_back(ParamDef("die_width", "晶粒宽度(像素)", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("die_height", "晶粒高度(像素)", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("margin_x", "X边距(像素)", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("margin_y", "Y边距(像素)", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("defect_threshold", "缺陷阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("draw_grid", "绘制网格", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("mark_defects", "标记缺陷", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> WaferDieDetectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    uint32_t die_width = static_cast<uint32_t>(get_param("die_width", Data(50)).as_int());
    uint32_t die_height = static_cast<uint32_t>(get_param("die_height", Data(50)).as_int());
    uint32_t margin_x = static_cast<uint32_t>(get_param("margin_x", Data(100)).as_int());
    uint32_t margin_y = static_cast<uint32_t>(get_param("margin_y", Data(100)).as_int());
    double defect_threshold = get_param("defect_threshold", Data(30.0)).as_number();
    bool draw_grid = get_param("draw_grid", Data(true)).as_bool();
    bool mark_defects = get_param("mark_defects", Data(true)).as_bool();
    
    // 生成晶粒网格
    auto dies = wafer_utils::generate_die_grid(input.width, input.height, die_width, die_height, margin_x, margin_y);
    
    // 转换为灰度图像进行缺陷检测
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else if (input.channels == 1) {
        gray = input;
    } else {
        return Result<void>::failure(ErrorCode::InvalidImage, "Unsupported image format");
    }
    
    // 检测每个晶粒的缺陷
    uint32_t good_count = 0;
    uint32_t defect_count = 0;
    
    for (auto& die : dies) {
        // 计算晶粒区域统计信息
        uint32_t x_start = static_cast<uint32_t>(die.center_x - die_width / 2);
        uint32_t y_start = static_cast<uint32_t>(die.center_y - die_height / 2);
        
        double sum = 0.0;
        double sum_sq = 0.0;
        uint32_t count = 0;
        double min_val = 255.0;
        double max_val = 0.0;
        
        for (uint32_t y = y_start; y < y_start + die_height && y < gray.height; ++y) {
            for (uint32_t x = x_start; x < x_start + die_width && x < gray.width; ++x) {
                double val = gray.data[y * gray.width + x];
                sum += val;
                sum_sq += val * val;
                min_val = std::min(min_val, val);
                max_val = std::max(max_val, val);
                count++;
            }
        }
        
        if (count > 0) {
            double mean = sum / count;
            double variance = (sum_sq / count) - (mean * mean);
            double std_dev = std::sqrt(std::max(0.0, variance));
            
            // 基于灰度变化判断是否为缺陷
            // 灰度变化过大或有异常灰度值认为是缺陷
            die.is_good = (std_dev < defect_threshold) && (max_val - min_val < defect_threshold * 2);
            die.confidence = 1.0 - std_dev / 128.0;
        }
        
        if (die.is_good) {
            good_count++;
        } else {
            defect_count++;
        }
    }
    
    // 输出图像
    ImageData output = input;
    
    if (draw_grid && output.channels == 3) {
        for (const auto& die : dies) {
            uint32_t x = static_cast<uint32_t>(die.center_x - die_width / 2);
            uint32_t y = static_cast<uint32_t>(die.center_y - die_height / 2);
            
            // 根据状态选择颜色：绿色-良品，红色-缺陷
            uint8_t r = die.is_good ? 0 : 255;
            uint8_t g = die.is_good ? 255 : 0;
            uint8_t b = 0;
            
            wafer_utils::draw_rect(output, x, y, die_width, die_height, r, g, b, 1);
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("die_count", Data(static_cast<int>(dies.size())));
    set_output("good_count", Data(static_cast<int>(good_count)));
    set_output("defect_count", Data(static_cast<int>(defect_count)));
    
    OVF_INFO() << "WaferDieDetection: " << dies.size() << " dies, " 
               << good_count << " good, " << defect_count << " defects";
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferDieDetectionNode, "WaferDieDetection", WaferDieDetectionNode::make_info())

//==============================================================================
// WaferDefectClassificationNode - 缺陷分类节点
//==============================================================================

WaferDefectClassificationNode::WaferDefectClassificationNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferDefectClassificationNode::make_info() {
    NodeInfo info;
    info.id = "WaferDefectClassification";
    info.name = "缺陷分类";
    info.category = "晶圆检测";
    info.description = "对缺陷进行分类：划痕/颗粒/污染/裂纹";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("defects", "缺陷列表", DataType::Array));
    info.outputs.push_back(DataPort("scratch_count", "划痕数量", DataType::Number));
    info.outputs.push_back(DataPort("particle_count", "颗粒数量", DataType::Number));
    info.outputs.push_back(DataPort("contamination_count", "污染数量", DataType::Number));
    info.outputs.push_back(DataPort("crack_count", "裂纹数量", DataType::Number));
    
    info.params.push_back(ParamDef("min_defect_size", "最小缺陷尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_defect_size", "最大缺陷尺寸", DataType::Number, Data(500)));
    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("classify_method", "分类方法", DataType::String, Data("auto")));
    
    return info;
}

Result<void> WaferDefectClassificationNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    uint32_t min_size = static_cast<uint32_t>(get_param("min_defect_size", Data(5)).as_int());
    uint32_t max_size = static_cast<uint32_t>(get_param("max_defect_size", Data(500)).as_int());
    double sensitivity = get_param("sensitivity", Data(0.5)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 图像预处理：高斯模糊
    ImageData blurred;
    wafer_utils::gaussian_blur(gray, blurred, 3, 1.0);
    
    // 边缘检测
    ImageData grad_x, grad_y, magnitude;
    wafer_utils::sobel_gradient(blurred, grad_x, grad_y, magnitude);
    
    // 自适应阈值分割
    ImageData binary;
    wafer_utils::adaptive_threshold(magnitude, binary, 15, static_cast<int>(30 * sensitivity));
    
    // 形态学处理
    ImageData morph_open;
    wafer_utils::morphological_open(binary, morph_open, 3);
    
    // 连通区域分析（简化版）
    std::vector<WaferDefect> defects;
    uint32_t defect_id = 1;
    
    // 使用简化的连通区域检测
    std::vector<bool> visited(gray.width * gray.height, false);
    
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            if (morph_open.data[y * gray.width + x] > 128 && !visited[y * gray.width + x]) {
                // BFS找到连通区域
                std::vector<std::pair<uint32_t, uint32_t>> region;
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                
                uint32_t min_x = x, max_x = x, min_y = y, max_y = y;
                
                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    region.push_back({cx, cy});
                    
                    min_x = std::min(min_x, cx);
                    max_x = std::max(max_x, cx);
                    min_y = std::min(min_y, cy);
                    max_y = std::max(max_y, cy);
                    
                    // 4邻域
                    int dx[] = {-1, 1, 0, 0};
                    int dy[] = {0, 0, -1, 1};
                    
                    for (int i = 0; i < 4; ++i) {
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
                
                // 尺寸过滤
                if (area >= min_size && area <= max_size) {
                    WaferDefect defect;
                    defect.id = defect_id++;
                    defect.center_x = (min_x + max_x) / 2.0;
                    defect.center_y = (min_y + max_y) / 2.0;
                    defect.width = width;
                    defect.height = height;
                    defect.area = area;
                    
                    // 计算形状特征
                    double aspect_ratio = static_cast<double>(width) / height;
                    double circularity = 4.0 * M_PI * area / (2.0 * (width + height) * (width + height));
                    double fill_ratio = static_cast<double>(area) / (width * height);
                    
                    // 计算边缘强度特征
                    double edge_strength = 0.0;
                    for (auto& [px, py] : region) {
                        edge_strength += magnitude.data[py * gray.width + px];
                    }
                    edge_strength /= area;
                    
                    // 分类缺陷类型
                    // 划痕：长条形，长宽比大
                    // 颗粒：圆形，面积小
                    // 污染：不规则，面积大
                    // 裂纹：分支状，边缘强度高
                    
                    if (aspect_ratio > 3.0 && fill_ratio > 0.3) {
                        defect.type = "scratch";  // 划痕
                        defect.severity = std::min(1.0, aspect_ratio / 10.0);
                    } else if (circularity > 0.7 && area < 500) {
                        defect.type = "particle";  // 颗粒
                        defect.severity = std::min(1.0, area / 200.0);
                    } else if (edge_strength > 150 && fill_ratio < 0.5) {
                        defect.type = "crack";  // 裂纹
                        defect.severity = std::min(1.0, edge_strength / 255.0);
                    } else {
                        defect.type = "contamination";  // 污染
                        defect.severity = std::min(1.0, std::sqrt(area / 1000.0));
                    }
                    
                    defect.confidence = 0.5 + 0.5 * sensitivity;
                    defects.push_back(defect);
                }
            }
        }
    }
    
    // 统计各类型缺陷数量
    int scratch_count = 0, particle_count = 0, contamination_count = 0, crack_count = 0;
    for (const auto& d : defects) {
        if (d.type == "scratch") scratch_count++;
        else if (d.type == "particle") particle_count++;
        else if (d.type == "contamination") contamination_count++;
        else if (d.type == "crack") crack_count++;
    }
    
    // 绘制标注
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& defect : defects) {
            uint8_t r = 0, g = 0, b = 0;
            
            if (defect.type == "scratch") { r = 255; g = 165; b = 0; }      // 橙色 - 划痕
            else if (defect.type == "particle") { r = 0; g = 255; b = 255; } // 青色 - 颗粒
            else if (defect.type == "contamination") { r = 255; g = 0; b = 255; } // 紫色 - 污染
            else if (defect.type == "crack") { r = 255; g = 0; b = 0; }     // 红色 - 裂纹
            
            uint32_t x = static_cast<uint32_t>(defect.center_x - defect.width / 2);
            uint32_t y = static_cast<uint32_t>(defect.center_y - defect.height / 2);
            
            wafer_utils::draw_rect(output, x, y, defect.width, defect.height, r, g, b, 2);
            wafer_utils::draw_cross(output, static_cast<uint32_t>(defect.center_x),
                                   static_cast<uint32_t>(defect.center_y), 5, r, g, b);
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("scratch_count", Data(scratch_count));
    set_output("particle_count", Data(particle_count));
    set_output("contamination_count", Data(contamination_count));
    set_output("crack_count", Data(crack_count));
    
    OVF_INFO() << "WaferDefectClassification: " << defects.size() << " defects found"
               << " (scratch:" << scratch_count << " particle:" << particle_count
               << " contamination:" << contamination_count << " crack:" << crack_count << ")";
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferDefectClassificationNode, "WaferDefectClassification", WaferDefectClassificationNode::make_info())

//==============================================================================
// WaferPatternInspectionNode - 图案检测节点
//==============================================================================

WaferPatternInspectionNode::WaferPatternInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferPatternInspectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferPatternInspection";
    info.name = "图案检测";
    info.category = "晶圆检测";
    info.description = "光刻图案对比检测，检测图案偏差和缺陷";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("match_score", "匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("pattern_count", "图案数量", DataType::Number));
    info.outputs.push_back(DataPort("defect_regions", "缺陷区域", DataType::Array));
    
    info.params.push_back(ParamDef("threshold", "匹配阈值", DataType::Number, Data(0.85)));
    info.params.push_back(ParamDef("min_contrast", "最小对比度", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("pattern_type", "图案类型", DataType::String, Data("auto")));
    info.params.push_back(ParamDef("inspect_edges", "检测边缘", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> WaferPatternInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double threshold = get_param("threshold", Data(0.85)).as_number();
    double min_contrast = get_param("min_contrast", Data(10.0)).as_number();
    bool inspect_edges = get_param("inspect_edges", Data(true)).as_bool();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 计算图像统计信息
    auto stats = wafer_utils::compute_stats(gray);
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    wafer_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 检测直线图案（晶圆常见的光刻图案）
    auto lines = wafer_utils::detect_lines(edge_mag, 30);
    
    // 检测圆形图案
    auto circles = wafer_utils::detect_circles(edge_mag, 10, 200, 20);
    
    // 计算图案一致性分数
    double total_edge_energy = 0.0;
    for (size_t i = 0; i < edge_mag.data.size(); ++i) {
        total_edge_energy += edge_mag.data[i];
    }
    double avg_edge_energy = total_edge_energy / edge_mag.data.size();
    
    // 计算对比度
    double contrast = stats.max_val - stats.min_val;
    
    // 检测图案缺陷区域（基于局部对比度异常）
    std::vector<wafer_utils::Rect> defect_regions;
    uint32_t block_size = 32;
    
    for (uint32_t y = 0; y < gray.height; y += block_size) {
        for (uint32_t x = 0; x < gray.width; x += block_size) {
            // 计算局部统计
            double local_sum = 0.0;
            double local_sum_sq = 0.0;
            uint32_t count = 0;
            
            for (uint32_t dy = 0; dy < block_size && y + dy < gray.height; ++dy) {
                for (uint32_t dx = 0; dx < block_size && x + dx < gray.width; ++dx) {
                    double val = gray.data[(y + dy) * gray.width + (x + dx)];
                    local_sum += val;
                    local_sum_sq += val * val;
                    count++;
                }
            }
            
            if (count > 0) {
                double local_mean = local_sum / count;
                double local_var = (local_sum_sq / count) - (local_mean * local_mean);
                double local_std = std::sqrt(std::max(0.0, local_var));
                
                // 如果局部标准差异常，可能是图案缺陷
                if (local_std > stats.std_dev * 1.5 || local_std < stats.std_dev * 0.3) {
                    wafer_utils::Rect rect;
                    rect.x = x;
                    rect.y = y;
                    rect.width = block_size;
                    rect.height = block_size;
                    defect_regions.push_back(rect);
                }
            }
        }
    }
    
    // 计算匹配分数
    double pattern_score = 0.0;
    if (contrast >= min_contrast) {
        pattern_score = (lines.size() > 0 || circles.size() > 0) ? 
                       std::min(1.0, avg_edge_energy / 50.0) : 0.3;
    }
    
    // 边缘完整性评估
    if (inspect_edges) {
        // 检查边缘连续性
        double edge_continuity = 0.0;
        uint32_t edge_pixels = 0;
        for (size_t i = 0; i < edge_mag.data.size(); ++i) {
            if (edge_mag.data[i] > 30) edge_pixels++;
        }
        edge_continuity = static_cast<double>(edge_pixels) / edge_mag.data.size();
        pattern_score = pattern_score * 0.7 + edge_continuity * 0.3;
    }
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制检测到的直线
        for (const auto& line : lines) {
            wafer_utils::draw_line(output, 
                                   static_cast<uint32_t>(line.first.x),
                                   static_cast<uint32_t>(line.first.y),
                                   static_cast<uint32_t>(line.second.x),
                                   static_cast<uint32_t>(line.second.y),
                                   0, 255, 0);  // 绿色
        }
        
        // 绘制检测到的圆
        for (const auto& circle : circles) {
            wafer_utils::draw_circle(output,
                                     static_cast<uint32_t>(circle.first.x),
                                     static_cast<uint32_t>(circle.first.y),
                                     static_cast<uint32_t>(circle.second),
                                     255, 255, 0);  // 青色
        }
        
        // 绘制缺陷区域
        for (const auto& rect : defect_regions) {
            wafer_utils::draw_rect(output, rect.x, rect.y, rect.width, rect.height, 255, 0, 0, 2);  // 红色
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("match_score", Data(pattern_score));
    set_output("pattern_count", Data(static_cast<int>(lines.size() + circles.size())));
    
    OVF_INFO() << "WaferPatternInspection: score=" << pattern_score 
               << ", patterns=" << (lines.size() + circles.size())
               << ", defects=" << defect_regions.size();
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferPatternInspectionNode, "WaferPatternInspection", WaferPatternInspectionNode::make_info())

//==============================================================================
// WaferEdgeInspectionNode - 边缘检测节点
//==============================================================================

WaferEdgeInspectionNode::WaferEdgeInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferEdgeInspectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferEdgeInspection";
    info.name = "边缘检测";
    info.category = "晶圆检测";
    info.description = "检测晶圆边缘缺陷：崩边、缺损等";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("edge_quality", "边缘质量分数", DataType::Number));
    info.outputs.push_back(DataPort("chipping_count", "崩边数量", DataType::Number));
    info.outputs.push_back(DataPort("notch_detected", "缺口检测", DataType::Boolean));
    info.outputs.push_back(DataPort("edge_contour", "边缘轮廓", DataType::Array));
    
    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(30)));
    info.params.push_back(ParamDef("min_defect_size", "最小缺陷尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("detect_notch", "检测缺口", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("inspect_zone", "检测区域宽度", DataType::Number, Data(50)));
    
    return info;
}

Result<void> WaferEdgeInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int edge_threshold = static_cast<int>(get_param("edge_threshold", Data(30)).as_int());
    uint32_t min_defect_size = static_cast<uint32_t>(get_param("min_defect_size", Data(5)).as_int());
    bool detect_notch = get_param("detect_notch", Data(true)).as_bool();
    uint32_t inspect_zone = static_cast<uint32_t>(get_param("inspect_zone", Data(50)).as_int());
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    wafer_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 检测晶圆边界（假设晶圆在图像中心）
    uint32_t center_x = gray.width / 2;
    uint32_t center_y = gray.height / 2;
    
    // 寻找晶圆边界
    std::vector<double> edge_contour;
    std::vector<int> edge_radii;
    
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 180) {
        // 从中心向外搜索边缘
        int last_edge_r = -1;
        for (int r = 50; r < static_cast<int>(std::min(gray.width, gray.height)) / 2; ++r) {
            uint32_t x = static_cast<uint32_t>(center_x + r * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + r * std::sin(angle));
            
            if (x < gray.width && y < gray.height) {
                if (edge_mag.data[y * gray.width + x] > edge_threshold) {
                    last_edge_r = r;
                }
            }
        }
        edge_radii.push_back(last_edge_r);
    }
    
    // 计算理想的圆形边界（平均值）
    double avg_radius = 0.0;
    int valid_count = 0;
    for (int r : edge_radii) {
        if (r > 0) {
            avg_radius += r;
            valid_count++;
        }
    }
    if (valid_count > 0) avg_radius /= valid_count;
    
    // 检测崩边和缺损
    int chipping_count = 0;
    std::vector<std::pair<double, double>> chipping_locations;
    
    for (size_t i = 0; i < edge_radii.size(); ++i) {
        if (edge_radii[i] > 0) {
            double deviation = edge_radii[i] - avg_radius;
            // 如果向内偏移超过阈值，认为是崩边
            if (deviation < -static_cast<double>(min_defect_size)) {
                chipping_count++;
                double angle = i * M_PI / 180;
                chipping_locations.push_back({center_x + avg_radius * std::cos(angle),
                                              center_y + avg_radius * std::sin(angle)});
            }
        }
    }
    
    // 检测缺口（Notch）
    bool notch_detected = false;
    double notch_position = -1.0;
    
    if (detect_notch) {
        // 缺口通常在晶圆底部边缘
        // 检测角度范围：-30°到+30°（底部）
        int notch_start = static_cast<int>(edge_radii.size() * 0.75);  // 270°
        int notch_end = static_cast<int>(edge_radii.size() * 0.92);    // 330°
        
        double min_radius = avg_radius;
        int min_idx = -1;
        
        for (int i = notch_start; i < notch_end; ++i) {
            size_t idx = i % edge_radii.size();
            if (edge_radii[idx] > 0 && edge_radii[idx] < min_radius) {
                min_radius = edge_radii[idx];
                min_idx = static_cast<int>(idx);
            }
        }
        
        if (min_radius < avg_radius * 0.95) {
            notch_detected = true;
            notch_position = min_idx * M_PI / 180;
        }
    }
    
    // 计算边缘质量分数
    double edge_quality = 1.0;
    if (chipping_count > 0) {
        edge_quality -= std::min(0.5, chipping_count * 0.05);
    }
    edge_quality = std::max(0.0, edge_quality);
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制理想边界
        wafer_utils::draw_circle(output, center_x, center_y, 
                                 static_cast<uint32_t>(avg_radius), 0, 255, 0);  // 绿色
        
        // 绘制检测到的崩边位置
        for (const auto& loc : chipping_locations) {
            wafer_utils::draw_cross(output, static_cast<uint32_t>(loc.first),
                                   static_cast<uint32_t>(loc.second), 10, 255, 0, 0);  // 红色
        }
        
        // 如果检测到缺口，标记
        if (notch_detected) {
            uint32_t nx = static_cast<uint32_t>(center_x + avg_radius * std::cos(notch_position));
            uint32_t ny = static_cast<uint32_t>(center_y + avg_radius * std::sin(notch_position));
            wafer_utils::draw_cross(output, nx, ny, 15, 255, 255, 0);  // 青色
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("edge_quality", Data(edge_quality));
    set_output("chipping_count", Data(chipping_count));
    set_output("notch_detected", Data(notch_detected));
    
    OVF_INFO() << "WaferEdgeInspection: quality=" << edge_quality
               << ", chipping=" << chipping_count
               << ", notch=" << (notch_detected ? "detected" : "not detected");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferEdgeInspectionNode, "WaferEdgeInspection", WaferEdgeInspectionNode::make_info())

//==============================================================================
// WaferAlignmentNode - 晶圆对准节点
//==============================================================================

WaferAlignmentNode::WaferAlignmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferAlignmentNode::make_info() {
    NodeInfo info;
    info.id = "WaferAlignment";
    info.name = "晶圆对准";
    info.category = "晶圆检测";
    info.description = "检测晶圆对准标记，计算对准偏移和旋转";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("reference", "参考标记", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("offset_x", "X偏移(像素)", DataType::Number));
    info.outputs.push_back(DataPort("offset_y", "Y偏移(像素)", DataType::Number));
    info.outputs.push_back(DataPort("rotation", "旋转角度(度)", DataType::Number));
    info.outputs.push_back(DataPort("marks_found", "检测到的标记数", DataType::Number));
    info.outputs.push_back(DataPort("alignment_score", "对准分数", DataType::Number));
    
    info.params.push_back(ParamDef("mark_type", "标记类型", DataType::String, Data("cross")));
    info.params.push_back(ParamDef("expected_marks", "期望标记数", DataType::Number, Data(2)));
    info.params.push_back(ParamDef("search_region", "搜索区域比例", DataType::Number, Data(0.2)));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7)));
    
    return info;
}

Result<void> WaferAlignmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    std::string mark_type = get_param("mark_type", Data("cross")).as_string();
    int expected_marks = get_param("expected_marks", Data(2)).as_int();
    double search_region = get_param("search_region", Data(0.2)).as_number();
    double min_score = get_param("min_score", Data(0.7)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    wafer_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 定义搜索区域（假设对准标记在晶圆四角或边缘）
    uint32_t search_width = static_cast<uint32_t>(input.width * search_region);
    uint32_t search_height = static_cast<uint32_t>(input.height * search_region);
    
    std::vector<AlignmentMark> marks;
    
    // 定义4个搜索区域（左上、右上、左下、右下）
    std::vector<std::pair<uint32_t, uint32_t>> search_centers = {
        {search_width / 2, search_height / 2},                                      // 左上
        {input.width - search_width / 2, search_height / 2},                         // 右上
        {search_width / 2, input.height - search_height / 2},                       // 左下
        {input.width - search_width / 2, input.height - search_height / 2}          // 右下
    };
    
    uint32_t mark_id = 1;
    for (size_t i = 0; i < search_centers.size() && marks.size() < static_cast<size_t>(expected_marks); ++i) {
        auto [cx, cy] = search_centers[i];
        
        // 在搜索区域内寻找十字标记
        // 十字标记特征：中心暗，四个方向有直线
        
        uint32_t x_start = cx - search_width / 4;
        uint32_t y_start = cy - search_height / 4;
        uint32_t x_end = cx + search_width / 4;
        uint32_t y_end = cy + search_height / 4;
        
        // 确保边界有效
        x_start = std::max(0u, x_start);
        y_start = std::max(0u, y_start);
        x_end = std::min(input.width - 1, x_end);
        y_end = std::min(input.height - 1, y_end);
        
        // 寻找十字中心（边缘梯度最大值的位置）
        double max_grad = 0.0;
        uint32_t mark_x = cx, mark_y = cy;
        
        for (uint32_t y = y_start; y < y_end; y += 3) {
            for (uint32_t x = x_start; x < x_end; x += 3) {
                double grad_sum = 0.0;
                int count = 0;
                
                // 检查水平和垂直方向的边缘强度
                for (int dx = -10; dx <= 10; dx += 2) {
                    int px = static_cast<int>(x) + dx;
                    if (px >= 0 && px < static_cast<int>(gray.width)) {
                        grad_sum += edge_mag.data[y * gray.width + px];
                        count++;
                    }
                }
                for (int dy = -10; dy <= 10; dy += 2) {
                    int py = static_cast<int>(y) + dy;
                    if (py >= 0 && py < static_cast<int>(gray.height)) {
                        grad_sum += edge_mag.data[py * gray.width + x];
                        count++;
                    }
                }
                
                if (count > 0) {
                    double avg_grad = grad_sum / count;
                    if (avg_grad > max_grad) {
                        max_grad = avg_grad;
                        mark_x = x;
                        mark_y = y;
                    }
                }
            }
        }
        
        // 如果找到了有效标记
        if (max_grad > 30) {  // 边缘强度阈值
            AlignmentMark mark;
            mark.id = mark_id++;
            mark.center_x = mark_x;
            mark.center_y = mark_y;
            mark.score = max_grad / 255.0;
            mark.found = true;
            marks.push_back(mark);
        }
    }
    
    // 计算对准偏移和旋转
    double offset_x = 0.0, offset_y = 0.0, rotation = 0.0;
    double alignment_score = 0.0;
    
    if (marks.size() >= 2) {
        // 计算期望的标记位置（晶圆中心对称）
        double expected_cx = input.width / 2.0;
        double expected_cy = input.height / 2.0;
        
        // 计算实际标记位置的平均偏移
        double total_offset_x = 0.0, total_offset_y = 0.0;
        
        for (const auto& mark : marks) {
            total_offset_x += mark.center_x - (mark.id <= 2 ? expected_cx - 200 : expected_cx + 200);
            total_offset_y += mark.center_y - (mark.id % 2 == 1 ? expected_cy - 200 : expected_cy + 200);
        }
        
        offset_x = total_offset_x / marks.size();
        offset_y = total_offset_y / marks.size();
        
        // 计算旋转角度（如果有多个标记）
        if (marks.size() >= 2) {
            // 使用第一个和最后一个标记计算角度
            double dx = marks.back().center_x - marks.front().center_x;
            double dy = marks.back().center_y - marks.front().center_y;
            rotation = std::atan2(dy, dx) * 180.0 / M_PI;
        }
        
        // 计算对准分数
        alignment_score = 1.0 - std::sqrt(offset_x * offset_x + offset_y * offset_y) / 100.0;
        alignment_score = std::max(0.0, std::min(1.0, alignment_score));
    }
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& mark : marks) {
            if (mark.found) {
                // 绘制十字标记
                wafer_utils::draw_cross(output, 
                                        static_cast<uint32_t>(mark.center_x),
                                        static_cast<uint32_t>(mark.center_y),
                                        15, 0, 255, 0);  // 绿色
                
                // 绘制标记框
                wafer_utils::draw_rect(output,
                                       static_cast<uint32_t>(mark.center_x - 20),
                                       static_cast<uint32_t>(mark.center_y - 20),
                                       40, 40, 255, 255, 0, 2);  // 青色
            }
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("offset_x", Data(offset_x));
    set_output("offset_y", Data(offset_y));
    set_output("rotation", Data(rotation));
    set_output("marks_found", Data(static_cast<int>(marks.size())));
    set_output("alignment_score", Data(alignment_score));
    
    OVF_INFO() << "WaferAlignment: marks=" << marks.size()
               << ", offset=(" << offset_x << "," << offset_y << ")"
               << ", rotation=" << rotation << "°"
               << ", score=" << alignment_score;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferAlignmentNode, "WaferAlignment", WaferAlignmentNode::make_info())

//==============================================================================
// WaferDicingInspectionNode - 切割检测节点
//==============================================================================

WaferDicingInspectionNode::WaferDicingInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferDicingInspectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferDicingInspection";
    info.name = "切割检测";
    info.category = "晶圆检测";
    info.description = "检测晶圆切割道质量，识别切割缺陷";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("street_count", "切割道数量", DataType::Number));
    info.outputs.push_back(DataPort("defect_streets", "缺陷切割道", DataType::Array));
    info.outputs.push_back(DataPort("quality_score", "质量分数", DataType::Number));
    info.outputs.push_back(DataPort("avg_width", "平均宽度(像素)", DataType::Number));
    
    info.params.push_back(ParamDef("expected_width", "期望宽度(像素)", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("width_tolerance", "宽度容差(像素)", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("detect_direction", "检测方向", DataType::String, Data("both")));
    info.params.push_back(ParamDef("min_contrast", "最小对比度", DataType::Number, Data(20.0)));
    
    return info;
}

Result<void> WaferDicingInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    uint32_t expected_width = static_cast<uint32_t>(get_param("expected_width", Data(50)).as_int());
    uint32_t width_tolerance = static_cast<uint32_t>(get_param("width_tolerance", Data(5)).as_int());
    std::string detect_direction = get_param("detect_direction", Data("both")).as_string();
    double min_contrast = get_param("min_contrast", Data(20.0)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    wafer_utils::sobel_gradient(gray, grad_x, grad_y, edge_mag);
    
    // 检测水平切割道（水平线，低灰度值的区域）
    std::vector<DicingStreet> h_streets;
    std::vector<DicingStreet> v_streets;
    
    // 水平切割道检测
    if (detect_direction == "both" || detect_direction == "horizontal") {
        for (uint32_t y = expected_width; y < gray.height - expected_width; y += expected_width) {
            // 检测该行附近是否有切割道特征
            double street_contrast = 0.0;
            uint32_t street_start = y;
            uint32_t street_end = y;
            
            // 计算水平方向的平均灰度变化
            for (uint32_t dy = 0; dy < expected_width / 2; ++dy) {
                if (y + dy >= gray.height) break;
                
                double line_avg = 0.0;
                for (uint32_t x = 0; x < gray.width; ++x) {
                    line_avg += gray.data[(y + dy) * gray.width + x];
                }
                line_avg /= gray.width;
                
                // 切割道通常比周围区域暗
                double above_avg = 0.0, below_avg = 0.0;
                for (uint32_t x = 0; x < gray.width; ++x) {
                    if (y + dy >= 2) {
                        above_avg += gray.data[(y + dy - 2) * gray.width + x];
                    }
                    if (y + dy + 2 < gray.height) {
                        below_avg += gray.data[(y + dy + 2) * gray.width + x];
                    }
                }
                above_avg /= gray.width;
                below_avg /= gray.width;
                
                if (line_avg < above_avg - min_contrast && line_avg < below_avg - min_contrast) {
                    street_contrast = above_avg - line_avg;
                    street_end = y + dy;
                }
            }
            
            if (street_contrast > min_contrast) {
                DicingStreet street;
                street.id = static_cast<uint32_t>(h_streets.size() + 1);
                street.start_x = 0;
                street.start_y = street_start;
                street.end_x = gray.width;
                street.end_y = street_end;
                street.width = street_end - street_start;
                street.quality = std::min(1.0, street_contrast / 50.0);
                street.has_defect = street.width < expected_width - width_tolerance ||
                                    street.width > expected_width + width_tolerance;
                h_streets.push_back(street);
            }
        }
    }
    
    // 垂直切割道检测
    if (detect_direction == "both" || detect_direction == "vertical") {
        for (uint32_t x = expected_width; x < gray.width - expected_width; x += expected_width) {
            double street_contrast = 0.0;
            uint32_t street_start = x;
            uint32_t street_end = x;
            
            for (uint32_t dx = 0; dx < expected_width / 2; ++dx) {
                if (x + dx >= gray.width) break;
                
                double col_avg = 0.0;
                for (uint32_t y = 0; y < gray.height; ++y) {
                    col_avg += gray.data[y * gray.width + (x + dx)];
                }
                col_avg /= gray.height;
                
                double left_avg = 0.0, right_avg = 0.0;
                for (uint32_t y = 0; y < gray.height; ++y) {
                    if (x + dx >= 2) {
                        left_avg += gray.data[y * gray.width + (x + dx - 2)];
                    }
                    if (x + dx + 2 < gray.width) {
                        right_avg += gray.data[y * gray.width + (x + dx + 2)];
                    }
                }
                left_avg /= gray.height;
                right_avg /= gray.height;
                
                if (col_avg < left_avg - min_contrast && col_avg < right_avg - min_contrast) {
                    street_contrast = left_avg - col_avg;
                    street_end = x + dx;
                }
            }
            
            if (street_contrast > min_contrast) {
                DicingStreet street;
                street.id = static_cast<uint32_t>(v_streets.size() + h_streets.size() + 1);
                street.start_x = street_start;
                street.start_y = 0;
                street.end_x = street_end;
                street.end_y = gray.height;
                street.width = street_end - street_start;
                street.quality = std::min(1.0, street_contrast / 50.0);
                street.has_defect = street.width < expected_width - width_tolerance ||
                                    street.width > expected_width + width_tolerance;
                v_streets.push_back(street);
            }
        }
    }
    
    // 合并结果
    std::vector<DicingStreet> all_streets;
    all_streets.insert(all_streets.end(), h_streets.begin(), h_streets.end());
    all_streets.insert(all_streets.end(), v_streets.begin(), v_streets.end());
    
    // 计算统计数据
    int defect_count = 0;
    double total_width = 0.0;
    double quality_sum = 0.0;
    
    for (const auto& street : all_streets) {
        if (street.has_defect) defect_count++;
        total_width += street.width;
        quality_sum += street.quality;
    }
    
    double avg_width = all_streets.empty() ? 0.0 : total_width / all_streets.size();
    double quality_score = all_streets.empty() ? 0.0 : quality_sum / all_streets.size();
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& street : all_streets) {
            uint8_t r = street.has_defect ? 255 : 0;
            uint8_t g = street.has_defect ? 0 : 255;
            uint8_t b = 0;
            
            if (street.start_y == street.end_y) {
                // 水平切割道
                wafer_utils::draw_line(output, 
                                       static_cast<uint32_t>(street.start_x),
                                       static_cast<uint32_t>(street.start_y),
                                       static_cast<uint32_t>(street.end_x),
                                       static_cast<uint32_t>(street.end_y),
                                       r, g, b);
            } else {
                // 垂直切割道
                wafer_utils::draw_line(output,
                                       static_cast<uint32_t>(street.start_x),
                                       static_cast<uint32_t>(street.start_y),
                                       static_cast<uint32_t>(street.end_x),
                                       static_cast<uint32_t>(street.end_y),
                                       r, g, b);
            }
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("street_count", Data(static_cast<int>(all_streets.size())));
    set_output("quality_score", Data(quality_score));
    set_output("avg_width", Data(avg_width));
    
    OVF_INFO() << "WaferDicingInspection: streets=" << all_streets.size()
               << ", defects=" << defect_count
               << ", avg_width=" << avg_width
               << ", quality=" << quality_score;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferDicingInspectionNode, "WaferDicingInspection", WaferDicingInspectionNode::make_info())

//==============================================================================
// WaferSurfaceInspectionNode - 表面检测节点
//==============================================================================

WaferSurfaceInspectionNode::WaferSurfaceInspectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferSurfaceInspectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferSurfaceInspection";
    info.name = "表面检测";
    info.category = "晶圆检测";
    info.description = "检测晶圆表面质量：平坦度、粗糙度等";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("roughness", "粗糙度", DataType::Number));
    info.outputs.push_back(DataPort("flatness", "平坦度", DataType::Number));
    info.outputs.push_back(DataPort("peak_to_valley", "峰谷差", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    info.outputs.push_back(DataPort("surface_quality", "表面质量", DataType::Object));
    
    info.params.push_back(ParamDef("roughness_threshold", "粗糙度阈值", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("flatness_threshold", "平坦度阈值", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("block_size", "分析块大小", DataType::Number, Data(32)));
    info.params.push_back(ParamDef("use_frequency_analysis", "频域分析", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> WaferSurfaceInspectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double roughness_threshold = get_param("roughness_threshold", Data(10.0)).as_number();
    double flatness_threshold = get_param("flatness_threshold", Data(5.0)).as_number();
    uint32_t block_size = static_cast<uint32_t>(get_param("block_size", Data(32)).as_int());
    bool use_frequency = get_param("use_frequency_analysis", Data(true)).as_bool();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 计算全局统计信息
    auto global_stats = wafer_utils::compute_stats(gray);
    
    // 分块分析表面质量
    std::vector<double> block_means;
    std::vector<double> block_stds;
    std::vector<std::pair<uint32_t, uint32_t>> defective_blocks;
    
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
                double std_dev = std::sqrt(std::max(0.0, variance));
                
                block_means.push_back(mean);
                block_stds.push_back(std_dev);
                
                // 检测异常块
                if (std_dev > global_stats.std_dev * 1.5) {
                    defective_blocks.push_back({bx, by});
                }
            }
        }
    }
    
    // 计算粗糙度（基于局部标准差的均值）
    double roughness = 0.0;
    for (double std : block_stds) {
        roughness += std;
    }
    if (!block_stds.empty()) roughness /= block_stds.size();
    
    // 计算平坦度（基于块均值的变化）
    double flatness = 0.0;
    if (!block_means.empty()) {
        double mean_mean = 0.0;
        for (double m : block_means) mean_mean += m;
        mean_mean /= block_means.size();
        
        for (double m : block_means) {
            flatness += std::abs(m - mean_mean);
        }
        flatness /= block_means.size();
    }
    
    // 计算峰谷差
    double peak_to_valley = global_stats.max_val - global_stats.min_val;
    
    // 频域分析（可选）
    double rms_roughness = roughness;
    if (use_frequency) {
        // 简化的频域分析：计算高频能量比例
        ImageData blurred;
        wafer_utils::gaussian_blur(gray, blurred, 5, 2.0);
        
        double high_freq_energy = 0.0;
        double total_energy = 0.0;
        
        for (size_t i = 0; i < gray.data.size(); ++i) {
            double diff = std::abs(static_cast<int>(gray.data[i]) - static_cast<int>(blurred.data[i]));
            high_freq_energy += diff * diff;
            total_energy += gray.data[i] * gray.data[i];
        }
        
        if (total_energy > 0) {
            rms_roughness = std::sqrt(high_freq_energy / gray.data.size());
        }
    }
    
    // 判断是否合格
    bool is_acceptable = (roughness <= roughness_threshold) && (flatness <= flatness_threshold);
    
    // 构建表面质量对象
    SurfaceQuality surface_quality;
    surface_quality.roughness = roughness;
    surface_quality.flatness = flatness;
    surface_quality.peak_to_valley = peak_to_valley;
    surface_quality.rms_roughness = rms_roughness;
    surface_quality.is_acceptable = is_acceptable;
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        // 标记缺陷块
        for (const auto& [bx, by] : defective_blocks) {
            wafer_utils::draw_rect(output, bx, by, block_size, block_size, 255, 165, 0, 2);  // 橙色
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("roughness", Data(roughness));
    set_output("flatness", Data(flatness));
    set_output("peak_to_valley", Data(peak_to_valley));
    set_output("is_acceptable", Data(is_acceptable));
    
    OVF_INFO() << "WaferSurfaceInspection: roughness=" << roughness
               << ", flatness=" << flatness
               << ", p2v=" << peak_to_valley
               << ", acceptable=" << (is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferSurfaceInspectionNode, "WaferSurfaceInspection", WaferSurfaceInspectionNode::make_info())

//==============================================================================
// WaferContaminationDetectionNode - 污染检测节点
//==============================================================================

WaferContaminationDetectionNode::WaferContaminationDetectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferContaminationDetectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferContaminationDetection";
    info.name = "污染检测";
    info.category = "晶圆检测";
    info.description = "检测晶圆表面异物和污染";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("contamination_count", "污染数量", DataType::Number));
    info.outputs.push_back(DataPort("contamination_area", "污染总面积", DataType::Number));
    info.outputs.push_back(DataPort("contaminations", "污染列表", DataType::Array));
    info.outputs.push_back(DataPort("cleanliness", "清洁度", DataType::Number));
    
    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("min_area", "最小面积", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("max_area", "最大面积", DataType::Number, Data(10000)));
    info.params.push_back(ParamDef("background_model", "背景模型", DataType::String, Data("global")));
    
    return info;
}

Result<void> WaferContaminationDetectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double sensitivity = get_param("sensitivity", Data(0.5)).as_number();
    uint32_t min_area = static_cast<uint32_t>(get_param("min_area", Data(10)).as_int());
    uint32_t max_area = static_cast<uint32_t>(get_param("max_area", Data(10000)).as_int());
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 计算全局统计
    auto stats = wafer_utils::compute_stats(gray);
    
    // 基于背景模型的阈值分割
    double threshold_offset = (255.0 - stats.std_dev) * (1.0 - sensitivity);
    double contamination_threshold_high = stats.mean + threshold_offset;
    double contamination_threshold_low = stats.mean - threshold_offset;
    
    // 创建污染掩码
    ImageData mask;
    mask.width = gray.width; mask.height = gray.height; mask.channels = 1; mask.format = ImageFormat::Mono8;
    mask.data.resize(gray.width * gray.height);
    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (gray.data[i] > contamination_threshold_high || 
            gray.data[i] < contamination_threshold_low) {
            mask.data[i] = 255;
        } else {
            mask.data[i] = 0;
        }
    }
    
    // 形态学处理去噪
    ImageData cleaned;
    wafer_utils::morphological_open(mask, cleaned, 3);
    wafer_utils::morphological_close(cleaned, mask, 3);
    
    // 连通区域分析
    std::vector<WaferDefect> contaminations;
    uint32_t contamination_id = 1;
    uint32_t total_area = 0;
    
    std::vector<bool> visited(gray.width * gray.height, false);
    
    for (uint32_t y = 1; y < gray.height - 1; ++y) {
        for (uint32_t x = 1; x < gray.width - 1; ++x) {
            if (mask.data[y * gray.width + x] > 128 && !visited[y * gray.width + x]) {
                // BFS找到连通区域
                std::vector<std::pair<uint32_t, uint32_t>> region;
                std::vector<std::pair<uint32_t, uint32_t>> queue;
                queue.push_back({x, y});
                visited[y * gray.width + x] = true;
                
                uint32_t min_x = x, max_x = x, min_y = y, max_y = y;
                double sum_intensity = 0.0;
                
                while (!queue.empty()) {
                    auto [cx, cy] = queue.back();
                    queue.pop_back();
                    region.push_back({cx, cy});
                    
                    min_x = std::min(min_x, cx);
                    max_x = std::max(max_x, cx);
                    min_y = std::min(min_y, cy);
                    max_y = std::max(max_y, cy);
                    sum_intensity += gray.data[cy * gray.width + cx];
                    
                    // 8邻域
                    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
                    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
                    
                    for (int i = 0; i < 8; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        
                        if (nx >= 0 && nx < static_cast<int>(gray.width) &&
                            ny >= 0 && ny < static_cast<int>(gray.height) &&
                            !visited[ny * gray.width + nx] &&
                            mask.data[ny * gray.width + nx] > 128) {
                            visited[ny * gray.width + nx] = true;
                            queue.push_back({nx, ny});
                        }
                    }
                }
                
                uint32_t area = static_cast<uint32_t>(region.size());
                
                // 面积过滤
                if (area >= min_area && area <= max_area) {
                    WaferDefect contamination;
                    contamination.id = contamination_id++;
                    contamination.type = "contamination";
                    contamination.center_x = (min_x + max_x) / 2.0;
                    contamination.center_y = (min_y + max_y) / 2.0;
                    contamination.width = max_x - min_x + 1;
                    contamination.height = max_y - min_y + 1;
                    contamination.area = area;
                    contamination.confidence = 0.8;
                    contamination.severity = (sum_intensity / area > stats.mean) ? 0.7 : 0.5;
                    
                    contaminations.push_back(contamination);
                    total_area += area;
                }
            }
        }
    }
    
    // 计算清洁度
    double cleanliness = 1.0 - static_cast<double>(total_area) / (gray.width * gray.height);
    cleanliness = std::max(0.0, std::min(1.0, cleanliness));
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& cont : contaminations) {
            uint32_t cx = static_cast<uint32_t>(cont.center_x);
            uint32_t cy = static_cast<uint32_t>(cont.center_y);
            
            // 绘制污染区域
            wafer_utils::draw_rect(output,
                                   static_cast<uint32_t>(cont.center_x - cont.width / 2),
                                   static_cast<uint32_t>(cont.center_y - cont.height / 2),
                                   cont.width, cont.height,
                                   255, 255, 0, 2);  // 青色
            
            // 标记中心
            wafer_utils::draw_cross(output, cx, cy, 5, 255, 0, 255);  // 紫色
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("contamination_count", Data(static_cast<int>(contaminations.size())));
    set_output("contamination_area", Data(static_cast<int>(total_area)));
    set_output("cleanliness", Data(cleanliness));
    
    OVF_INFO() << "WaferContaminationDetection: count=" << contaminations.size()
               << ", area=" << total_area
               << ", cleanliness=" << cleanliness;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferContaminationDetectionNode, "WaferContaminationDetection", WaferContaminationDetectionNode::make_info())

//==============================================================================
// WaferCrackDetectionNode - 裂纹检测节点
//==============================================================================

WaferCrackDetectionNode::WaferCrackDetectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferCrackDetectionNode::make_info() {
    NodeInfo info;
    info.id = "WaferCrackDetection";
    info.name = "裂纹检测";
    info.category = "晶圆检测";
    info.description = "检测晶圆微裂纹";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("crack_count", "裂纹数量", DataType::Number));
    info.outputs.push_back(DataPort("crack_list", "裂纹列表", DataType::Array));
    info.outputs.push_back(DataPort("max_crack_length", "最大裂纹长度", DataType::Number));
    info.outputs.push_back(DataPort("quality_score", "质量分数", DataType::Number));
    
    info.params.push_back(ParamDef("sensitivity", "检测灵敏度", DataType::Number, Data(0.7)));
    info.params.push_back(ParamDef("min_length", "最小裂纹长度", DataType::Number, Data(10)));
    info.params.push_back(ParamDef("edge_threshold", "边缘阈值", DataType::Number, Data(30)));
    info.params.push_back(ParamDef("use_morphology", "使用形态学", DataType::Boolean, Data(true)));
    
    return info;
}

Result<void> WaferCrackDetectionNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double sensitivity = get_param("sensitivity", Data(0.7)).as_number();
    uint32_t min_length = static_cast<uint32_t>(get_param("min_length", Data(10)).as_int());
    int edge_threshold = static_cast<int>(get_param("edge_threshold", Data(30)).as_int());
    bool use_morphology = get_param("use_morphology", Data(true)).as_bool();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 高斯模糊减少噪声
    ImageData blurred;
    wafer_utils::gaussian_blur(gray, blurred, 3, 1.0);
    
    // 边缘检测
    ImageData grad_x, grad_y, edge_mag;
    wafer_utils::sobel_gradient(blurred, grad_x, grad_y, edge_mag);
    
    // 自适应阈值分割边缘
    int threshold = static_cast<int>(edge_threshold * sensitivity);
    ImageData edge_binary;
    edge_binary.width = gray.width; edge_binary.height = gray.height; edge_binary.channels = 1; edge_binary.format = ImageFormat::Mono8;
    edge_binary.data.resize(gray.width * gray.height);
    for (size_t i = 0; i < edge_mag.data.size(); ++i) {
        edge_binary.data[i] = (edge_mag.data[i] > threshold) ? 255 : 0;
    }
    
    // 形态学处理
    if (use_morphology) {
        ImageData temp;
        wafer_utils::morphological_close(edge_binary, temp, 2);
        wafer_utils::morphological_open(temp, edge_binary, 2);
    }
    
    // 检测直线（裂纹特征）
    auto lines = wafer_utils::detect_lines(edge_binary, static_cast<int>(min_length / 2));
    
    // 分析裂纹特征
    struct CrackInfo {
        double start_x, start_y;
        double end_x, end_y;
        double length;
        double angle;
        double severity;
    };
    
    std::vector<CrackInfo> cracks;
    double max_crack_length = 0.0;
    
    for (const auto& line : lines) {
        double dx = line.second.x - line.first.x;
        double dy = line.second.y - line.first.y;
        double length = std::sqrt(dx * dx + dy * dy);
        
        // 过滤短裂纹
        if (length >= min_length) {
            CrackInfo crack;
            crack.start_x = line.first.x;
            crack.start_y = line.first.y;
            crack.end_x = line.second.x;
            crack.end_y = line.second.y;
            crack.length = length;
            crack.angle = std::atan2(dy, dx) * 180.0 / M_PI;
            crack.severity = std::min(1.0, length / 100.0);
            
            cracks.push_back(crack);
            max_crack_length = std::max(max_crack_length, length);
        }
    }
    
    // 计算质量分数
    double quality_score = 1.0;
    if (!cracks.empty()) {
        // 每个裂纹降低质量分数
        quality_score -= cracks.size() * 0.1;
        // 长裂纹额外惩罚
        if (max_crack_length > 50) {
            quality_score -= (max_crack_length - 50) / 100.0;
        }
        quality_score = std::max(0.0, quality_score);
    }
    
    // 绘制结果
    ImageData output = input;
    if (output.channels == 3) {
        for (const auto& crack : cracks) {
            // 根据严重程度选择颜色
            uint8_t r = 255;
            uint8_t g = static_cast<uint8_t>(255 * (1.0 - crack.severity));
            uint8_t b = 0;
            
            wafer_utils::draw_line(output,
                                   static_cast<uint32_t>(crack.start_x),
                                   static_cast<uint32_t>(crack.start_y),
                                   static_cast<uint32_t>(crack.end_x),
                                   static_cast<uint32_t>(crack.end_y),
                                   r, g, b);
            
            // 标记裂纹端点
            wafer_utils::draw_cross(output,
                                   static_cast<uint32_t>(crack.start_x),
                                   static_cast<uint32_t>(crack.start_y),
                                   5, r, g, b);
            wafer_utils::draw_cross(output,
                                   static_cast<uint32_t>(crack.end_x),
                                   static_cast<uint32_t>(crack.end_y),
                                   5, r, g, b);
        }
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("crack_count", Data(static_cast<int>(cracks.size())));
    set_output("max_crack_length", Data(max_crack_length));
    set_output("quality_score", Data(quality_score));
    
    OVF_INFO() << "WaferCrackDetection: cracks=" << cracks.size()
               << ", max_length=" << max_crack_length
               << ", quality=" << quality_score;
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferCrackDetectionNode, "WaferCrackDetection", WaferCrackDetectionNode::make_info())

//==============================================================================
// WaferThicknessMeasurementNode - 厚度测量节点
//==============================================================================

WaferThicknessMeasurementNode::WaferThicknessMeasurementNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WaferThicknessMeasurementNode::make_info() {
    NodeInfo info;
    info.id = "WaferThicknessMeasurement";
    info.name = "厚度测量";
    info.category = "晶圆检测";
    info.description = "模拟干涉测量法测量晶圆厚度、弯曲度和翘曲度";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（干涉图）", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像（标注）", DataType::Image));
    info.outputs.push_back(DataPort("center_thickness", "中心厚度(μm)", DataType::Number));
    info.outputs.push_back(DataPort("edge_thickness", "边缘厚度(μm)", DataType::Number));
    info.outputs.push_back(DataPort("thickness_variation", "厚度变化(μm)", DataType::Number));
    info.outputs.push_back(DataPort("bow", "弯曲度(μm)", DataType::Number));
    info.outputs.push_back(DataPort("warp", "翘曲度(μm)", DataType::Number));
    info.outputs.push_back(DataPort("ttv", "TTV(μm)", DataType::Number));
    info.outputs.push_back(DataPort("is_acceptable", "是否合格", DataType::Boolean));
    
    info.params.push_back(ParamDef("nominal_thickness", "标称厚度(μm)", DataType::Number, Data(775.0)));
    info.params.push_back(ParamDef("thickness_tolerance", "厚度容差(μm)", DataType::Number, Data(5.0)));
    info.params.push_back(ParamDef("bow_tolerance", "弯曲度容差(μm)", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("warp_tolerance", "翘曲度容差(μm)", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("wavelength", "光源波长(nm)", DataType::Number, Data(632.8)));
    
    return info;
}

Result<void> WaferThicknessMeasurementNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    double nominal_thickness = get_param("nominal_thickness", Data(775.0)).as_number();
    double thickness_tolerance = get_param("thickness_tolerance", Data(5.0)).as_number();
    double bow_tolerance = get_param("bow_tolerance", Data(30.0)).as_number();
    double warp_tolerance = get_param("warp_tolerance", Data(50.0)).as_number();
    double wavelength = get_param("wavelength", Data(632.8)).as_number();
    
    // 转换为灰度图像
    ImageData gray;
    if (input.channels == 3) {
        gray.width = input.width; gray.height = input.height; gray.channels = 1; gray.format = ImageFormat::Mono8; gray.data.resize(input.width * input.height);
        for (size_t i = 0; i < input.width * input.height; ++i) {
            gray.data[i] = static_cast<uint8_t>(
                0.114 * input.data[i*3] + 0.587 * input.data[i*3+1] + 0.299 * input.data[i*3+2]);
        }
    } else {
        gray = input;
    }
    
    // 模拟干涉测量
    // 通过分析图像强度分布来模拟厚度测量
    // 实际应用中，这里会使用相位解包裹算法
    
    // 计算图像统计
    auto stats = wafer_utils::compute_stats(gray);
    
    // 模拟厚度分布计算
    // 基于干涉条纹的强度变化来估算厚度变化
    // 这里使用简化的模型
    
    uint32_t center_x = gray.width / 2;
    uint32_t center_y = gray.height / 2;
    double max_radius = std::min(center_x, center_y) * 0.9;
    
    // 计算不同半径处的平均强度（模拟不同位置的厚度）
    std::vector<double> radial_intensity;
    std::vector<double> radii;
    
    for (uint32_t r = 0; r <= static_cast<uint32_t>(max_radius); r += 10) {
        double sum = 0.0;
        int count = 0;
        
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 36) {
            uint32_t x = static_cast<uint32_t>(center_x + r * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + r * std::sin(angle));
            
            if (x < gray.width && y < gray.height) {
                sum += gray.data[y * gray.width + x];
                count++;
            }
        }
        
        if (count > 0) {
            radial_intensity.push_back(sum / count);
            radii.push_back(r);
        }
    }
    
    // 基于强度变化模拟厚度计算
    // 干涉条纹强度：I = I0 * (1 + cos(4*pi*t/lambda))
    // 简化：假设强度变化对应厚度变化
    
    ThicknessResult result;
    
    // 中心厚度（基于中心区域平均强度）
    double center_intensity = 0.0;
    int center_count = 0;
    for (uint32_t y = center_y - 20; y < center_y + 20 && y < gray.height; ++y) {
        for (uint32_t x = center_x - 20; x < center_x + 20 && x < gray.width; ++x) {
            center_intensity += gray.data[y * gray.width + x];
            center_count++;
        }
    }
    center_intensity /= center_count;
    
    // 边缘厚度（基于边缘区域平均强度）
    double edge_intensity = 0.0;
    int edge_count = 0;
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 36) {
        uint32_t x = static_cast<uint32_t>(center_x + max_radius * 0.9 * std::cos(angle));
        uint32_t y = static_cast<uint32_t>(center_y + max_radius * 0.9 * std::sin(angle));
        
        if (x < gray.width && y < gray.height) {
            for (int dx = -5; dx <= 5; ++dx) {
                for (int dy = -5; dy <= 5; ++dy) {
                    uint32_t px = x + dx;
                    uint32_t py = y + dy;
                    if (px < gray.width && py < gray.height) {
                        edge_intensity += gray.data[py * gray.width + px];
                        edge_count++;
                    }
                }
            }
        }
    }
    edge_intensity /= edge_count;
    
    // 模拟厚度计算（实际需要相位解包裹）
    // 使用简化模型：强度差异对应厚度差异
    double intensity_to_thickness = wavelength / (4 * M_PI * 255) * 1000;  // 转换为μm
    
    result.center_thickness = nominal_thickness + 
        (center_intensity - stats.mean) * intensity_to_thickness;
    result.edge_thickness = nominal_thickness + 
        (edge_intensity - stats.mean) * intensity_to_thickness;
    
    // 计算厚度变化
    double min_thickness = result.center_thickness;
    double max_thickness = result.center_thickness;
    
    for (size_t i = 0; i < radial_intensity.size(); ++i) {
        double thickness = nominal_thickness + 
            (radial_intensity[i] - stats.mean) * intensity_to_thickness;
        min_thickness = std::min(min_thickness, thickness);
        max_thickness = std::max(max_thickness, thickness);
    }
    
    result.thickness_variation = max_thickness - min_thickness;
    
    // 弯曲度(Bow): 中心与参考面的偏差
    result.bow = result.center_thickness - (min_thickness + max_thickness) / 2;
    
    // 翘曲度(Warp): 最大正偏差与最大负偏差之差
    result.warp = max_thickness - min_thickness;
    
    // TTV (Total Thickness Variation)
    result.ttv = result.thickness_variation;
    
    // 判断是否合格
    result.is_acceptable = 
        (std::abs(result.center_thickness - nominal_thickness) <= thickness_tolerance) &&
        (std::abs(result.edge_thickness - nominal_thickness) <= thickness_tolerance) &&
        (std::abs(result.bow) <= bow_tolerance) &&
        (result.warp <= warp_tolerance);
    
    // 绘制厚度分布可视化
    ImageData output = input;
    if (output.channels == 3) {
        // 绘制测量位置
        wafer_utils::draw_circle(output, center_x, center_y, 
                                static_cast<uint32_t>(max_radius), 0, 255, 0);  // 晶圆边界
        
        // 绘制中心测量点
        wafer_utils::draw_cross(output, center_x, center_y, 10, 255, 0, 0);  // 红色
        
        // 绘制边缘测量点
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 6) {
            uint32_t x = static_cast<uint32_t>(center_x + max_radius * 0.9 * std::cos(angle));
            uint32_t y = static_cast<uint32_t>(center_y + max_radius * 0.9 * std::sin(angle));
            wafer_utils::draw_cross(output, x, y, 5, 255, 255, 0);  // 青色
        }
        
        // 根据结果绘制状态
        uint8_t status_color_r = result.is_acceptable ? 0 : 255;
        uint8_t status_color_g = result.is_acceptable ? 255 : 0;
        wafer_utils::draw_rect(output, 10, 10, 100, 30, status_color_r, status_color_g, 0, 2);
    }
    
    // 设置输出
    set_output("image", Data(output));
    set_output("center_thickness", Data(result.center_thickness));
    set_output("edge_thickness", Data(result.edge_thickness));
    set_output("thickness_variation", Data(result.thickness_variation));
    set_output("bow", Data(result.bow));
    set_output("warp", Data(result.warp));
    set_output("ttv", Data(result.ttv));
    set_output("is_acceptable", Data(result.is_acceptable));
    
    OVF_INFO() << "WaferThicknessMeasurement: center=" << result.center_thickness << "μm"
               << ", edge=" << result.edge_thickness << "μm"
               << ", bow=" << result.bow << "μm"
               << ", warp=" << result.warp << "μm"
               << ", acceptable=" << (result.is_acceptable ? "yes" : "no");
    
    return Result<void>::success();
}

OVF_REGISTER_NODE(WaferThicknessMeasurementNode, "WaferThicknessMeasurement", WaferThicknessMeasurementNode::make_info())

} // namespace algorithm
} // namespace ovf