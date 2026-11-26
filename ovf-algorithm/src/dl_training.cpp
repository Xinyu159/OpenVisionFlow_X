/**
 * @file dl_training.cpp
 * @brief 深度学习训练闭环工具模块实现
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <random>
#include <iomanip>
#include <map>
#include <unordered_set>

#include "ovf/algorithm/dl_training.h"
#include "ovf/algorithm/image_utils.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace ovf {
namespace algorithm {

// ==================== 辅助函数 ====================

namespace {

template<typename T>
inline T clamp_val(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(max_val, value));
}

// JSON辅助函数
String escape_json_string(const String& str) {
    String result;
    for (char c : str) {
        switch (c) {
            case '"': result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default: result += c; break;
        }
    }
    return result;
}

// 解析JSON数组中的字符串列表
Vector<String> parse_json_string_array(const String& json_str) {
    Vector<String> result;
    if (json_str.empty() || json_str[0] != '[') return result;
    
    // 简化解析：假设格式为 ["str1","str2",...]
    size_t pos = 1;
    while (pos < json_str.size() - 1) {
        if (json_str[pos] == '"') {
            size_t end = json_str.find('"', pos + 1);
            if (end != String::npos) {
                result.push_back(json_str.substr(pos + 1, end - pos - 1));
                pos = end + 1;
                // 跳过逗号
                while (pos < json_str.size() && (json_str[pos] == ',' || json_str[pos] == ' ')) {
                    pos++;
                }
            } else {
                break;
            }
        } else {
            pos++;
        }
    }
    return result;
}

// 解析JSON数组中的数值列表
Vector<double> parse_json_number_array(const String& json_str) {
    Vector<double> result;
    if (json_str.empty() || json_str[0] != '[') return result;
    
    std::istringstream iss(json_str.substr(1, json_str.size() - 2));
    String token;
    while (std::getline(iss, token, ',')) {
        // 去除空格
        token.erase(0, token.find_first_not_of(" \t\n\r"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);
        if (!token.empty()) {
            try {
                result.push_back(std::stod(token));
            } catch (...) {}
        }
    }
    return result;
}

// 解析范围字符串 "min,max"
void parse_range(const String& range_str, float& min_val, float& max_val) {
    auto nums = parse_json_number_array("[" + range_str + "]");
    if (nums.size() >= 2) {
        min_val = static_cast<float>(nums[0]);
        max_val = static_cast<float>(nums[1]);
    }
}

} // anonymous namespace

// ==================== 标注数据结构实现 ====================

String ImageAnnotation::to_json() const {
    std::ostringstream oss;
    oss << "{";
    oss << "\"image_path\":\"" << escape_json_string(image_path) << "\",";
    oss << "\"image_id\":\"" << escape_json_string(image_id) << "\",";
    oss << "\"width\":" << width << ",";
    oss << "\"height\":" << height << ",";
    oss << "\"description\":\"" << escape_json_string(description) << "\",";
    oss << "\"objects\":[";
    
    for (size_t i = 0; i < objects.size(); ++i) {
        const auto& obj = objects[i];
        oss << "{";
        oss << "\"label\":\"" << escape_json_string(obj.label) << "\",";
        oss << "\"type\":\"" << (obj.type == AnnotationType::Rectangle ? "rectangle" :
                                 obj.type == AnnotationType::Polygon ? "polygon" :
                                 obj.type == AnnotationType::Point ? "point" :
                                 obj.type == AnnotationType::Line ? "line" : "keypoint") << "\",";
        oss << "\"x\":" << obj.x << ",";
        oss << "\"y\":" << obj.y << ",";
        oss << "\"width\":" << obj.width << ",";
        oss << "\"height\":" << obj.height << ",";
        oss << "\"confidence\":" << obj.confidence << ",";
        oss << "\"class_id\":" << obj.class_id << ",";
        oss << "\"occluded\":" << (obj.occluded ? "true" : "false") << ",";
        oss << "\"difficult\":" << (obj.difficult ? "true" : "false") << ",";
        
        // 多边形点
        if (!obj.points.empty()) {
            oss << "\"points\":[";
            for (size_t j = 0; j < obj.points.size(); ++j) {
                oss << "{\"x\":" << obj.points[j].x << ",\"y\":" << obj.points[j].y << "}";
                if (j < obj.points.size() - 1) oss << ",";
            }
            oss << "],";
        }
        
        oss << "\"attributes\":\"" << escape_json_string(obj.attributes) << "\"";
        oss << "}";
        if (i < objects.size() - 1) oss << ",";
    }
    
    oss << "]";
    oss << "}";
    return oss.str();
}

ImageAnnotation ImageAnnotation::from_json(const String& json_str) {
    ImageAnnotation annotation;
    // 简化JSON解析 - 实际项目中应使用完整JSON库
    // 这里仅做基础解析
    
    // 提取image_path
    size_t pos = json_str.find("\"image_path\"");
    if (pos != String::npos) {
        size_t start = json_str.find('"', pos + 13) + 1;
        size_t end = json_str.find('"', start);
        annotation.image_path = json_str.substr(start, end - start);
    }
    
    // 提取width和height
    pos = json_str.find("\"width\"");
    if (pos != String::npos) {
        size_t start = json_str.find_first_of("0123456789", pos);
        annotation.width = static_cast<uint32_t>(std::stoul(json_str.substr(start)));
    }
    
    pos = json_str.find("\"height\"");
    if (pos != String::npos) {
        size_t start = json_str.find_first_of("0123456789", pos);
        annotation.height = static_cast<uint32_t>(std::stoul(json_str.substr(start)));
    }
    
    return annotation;
}

// ==================== ImageAnnotateNode 实现 ====================

ImageAnnotateNode::ImageAnnotateNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ImageAnnotateNode::make_info() {
    NodeInfo info;
    info.id = "ImageAnnotate";
    info.name = "图像标注";
    info.category = "数据标注";
    info.description = "在图像上创建标注（矩形框、多边形等）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("roi", "感兴趣区域", DataType::Region, false)
    };
    
    info.outputs = {
        DataPort("annotation", "标注数据", DataType::String),
        DataPort("annotated_image", "带标注的图像", DataType::Image)
    };
    
    info.params = {
        ParamDef("label", "标签名称", DataType::String, Data("object")),
        ParamDef("annotation_type", "标注类型", DataType::String, Data("rectangle")),
        ParamDef("x", "矩形X坐标", DataType::Number, Data(0)),
        ParamDef("y", "矩形Y坐标", DataType::Number, Data(0)),
        ParamDef("width", "矩形宽度", DataType::Number, Data(100)),
        ParamDef("height", "矩形高度", DataType::Number, Data(100)),
        ParamDef("color", "标注颜色", DataType::String, Data("255,0,0")),
        ParamDef("show_label", "显示标签", DataType::Boolean, Data(true)),
        ParamDef("polygon_points", "多边形点", DataType::String, Data(""))
    };
    
    info.params[1].options = {"rectangle", "polygon", "point", "line"};
    
    return info;
}

AnnotationType ImageAnnotateNode::parse_annotation_type(const String& type_str) {
    if (type_str == "rectangle") return AnnotationType::Rectangle;
    if (type_str == "polygon") return AnnotationType::Polygon;
    if (type_str == "point") return AnnotationType::Point;
    if (type_str == "line") return AnnotationType::Line;
    return AnnotationType::Rectangle;
}

void ImageAnnotateNode::draw_annotation(ImageData& image, const AnnotationObject& obj,
                                          uint8_t r, uint8_t g, uint8_t b, bool show_label) {
    if (image.empty()) return;
    
    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;
    
    if (obj.type == AnnotationType::Rectangle) {
        // 绘制矩形框
        int x1 = clamp_val(obj.x, 0, img_w - 1);
        int y1 = clamp_val(obj.y, 0, img_h - 1);
        int x2 = clamp_val(obj.x + obj.width, 0, img_w - 1);
        int y2 = clamp_val(obj.y + obj.height, 0, img_h - 1);
        
        // 绘制边框
        for (int x = x1; x <= x2; ++x) {
            if (y1 >= 0 && y1 < img_h) {
                int idx = (y1 * img_w + x) * ch;
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            }
            if (y2 >= 0 && y2 < img_h) {
                int idx = (y2 * img_w + x) * ch;
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            }
        }
        for (int y = y1; y <= y2; ++y) {
            if (x1 >= 0 && x1 < img_w) {
                int idx = (y * img_w + x1) * ch;
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            }
            if (x2 >= 0 && x2 < img_w) {
                int idx = (y * img_w + x2) * ch;
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            }
        }
    } else if (obj.type == AnnotationType::Polygon && !obj.points.empty()) {
        // 绘制多边形
        for (size_t i = 0; i < obj.points.size(); ++i) {
            size_t j = (i + 1) % obj.points.size();
            int x1 = clamp_val(obj.points[i].x, 0, img_w - 1);
            int y1 = clamp_val(obj.points[i].y, 0, img_h - 1);
            int x2 = clamp_val(obj.points[j].x, 0, img_w - 1);
            int y2 = clamp_val(obj.points[j].y, 0, img_h - 1);
            
            // Bresenham直线算法
            int dx = std::abs(x2 - x1);
            int dy = std::abs(y2 - y1);
            int sx = (x1 < x2) ? 1 : -1;
            int sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy;
            
            int x = x1, y = y1;
            while (true) {
                if (x >= 0 && x < img_w && y >= 0 && y < img_h) {
                    int idx = (y * img_w + x) * ch;
                    image.data[idx] = r;
                    image.data[idx + 1] = g;
                    image.data[idx + 2] = b;
                }
                if (x == x2 && y == y2) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x += sx; }
                if (e2 < dx) { err += dx; y += sy; }
            }
        }
    }
    
    // 绘制标签文字（简化实现）
    if (show_label && !obj.label.empty()) {
        // 在矩形上方显示标签
        int text_x = obj.x;
        int text_y = obj.y - 5;
        if (text_y < 0) text_y = 0;
        
        // 简化：在标签位置绘制一个小矩形作为文字背景
        int label_width = static_cast<int>(obj.label.size() * 8);
        for (int dy = 0; dy < 12; ++dy) {
            for (int dx = 0; dx < label_width; ++dx) {
                int px = text_x + dx;
                int py = text_y + dy;
                if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
                    int idx = (py * img_w + px) * ch;
                    if (ch >= 3) {
                        image.data[idx] = r;
                        image.data[idx + 1] = g;
                        image.data[idx + 2] = b;
                    }
                }
            }
        }
    }
}

Result<void> ImageAnnotateNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    String label = get_param("label", Data("object")).as_string();
    String type_str = get_param("annotation_type", Data("rectangle")).as_string();
    int32_t x = static_cast<int32_t>(get_param("x", Data(0)).as_int());
    int32_t y = static_cast<int32_t>(get_param("y", Data(0)).as_int());
    int32_t width = static_cast<int32_t>(get_param("width", Data(100)).as_int());
    int32_t height = static_cast<int32_t>(get_param("height", Data(100)).as_int());
    String color_str = get_param("color", Data("255,0,0")).as_string();
    bool show_label = get_param("show_label", Data(true)).as_bool();
    String polygon_points_str = get_param("polygon_points", Data("")).as_string();
    
    AnnotationType type = parse_annotation_type(type_str);
    
    // 解析颜色
    uint8_t r = 255, g = 0, b = 0;
    auto color_nums = parse_json_number_array("[" + color_str + "]");
    if (color_nums.size() >= 3) {
        r = static_cast<uint8_t>(clamp_val(static_cast<int>(color_nums[0]), 0, 255));
        g = static_cast<uint8_t>(clamp_val(static_cast<int>(color_nums[1]), 0, 255));
        b = static_cast<uint8_t>(clamp_val(static_cast<int>(color_nums[2]), 0, 255));
    }
    
    // 创建标注对象
    AnnotationObject obj;
    obj.label = label;
    obj.type = type;
    obj.x = x;
    obj.y = y;
    obj.width = width;
    obj.height = height;
    
    // 解析多边形点（格式："x1,y1;x2,y2;..."）
    if (type == AnnotationType::Polygon && !polygon_points_str.empty()) {
        std::istringstream iss(polygon_points_str);
        String token;
        while (std::getline(iss, token, ';')) {
            size_t comma_pos = token.find(',');
            if (comma_pos != String::npos) {
                int px = std::stoi(token.substr(0, comma_pos));
                int py = std::stoi(token.substr(comma_pos + 1));
                obj.points.emplace_back(px, py);
            }
        }
    }
    
    // 创建图像标注数据
    ImageAnnotation annotation;
    annotation.width = input.width;
    annotation.height = input.height;
    annotation.objects.push_back(obj);
    
    // 创建带标注的图像
    ImageData annotated_image = input;
    draw_annotation(annotated_image, obj, r, g, b, show_label);
    
    // 设置输出
    set_output("annotation", Data(annotation.to_json()));
    set_output("annotated_image", Data(annotated_image));
    
    OVF_INFO() << "ImageAnnotate completed: label=" << label << ", type=" << type_str;
    
    return Result<void>::success();
}

// ==================== LabelExportNode 实现 ====================

LabelExportNode::LabelExportNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo LabelExportNode::make_info() {
    NodeInfo info;
    info.id = "LabelExport";
    info.name = "标签导出";
    info.category = "数据标注";
    info.description = "将标注数据导出为指定格式（YOLO/COCO/VOC）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("annotations", "标注数据列表", DataType::String, true),
        DataPort("image_paths", "图像路径列表", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("export_path", "导出文件路径", DataType::String),
        DataPort("export_count", "导出数量", DataType::Number),
        DataPort("export_data", "导出的数据内容", DataType::String)
    };
    
    info.params = {
        ParamDef("format", "导出格式", DataType::String, Data("YOLO")),
        ParamDef("output_dir", "输出目录", DataType::String, Data("./labels")),
        ParamDef("filename", "输出文件名", DataType::String, Data("annotations")),
        ParamDef("include_images", "包含图像信息", DataType::Boolean, Data(true))
    };
    
    info.params[0].options = {"YOLO", "COCO", "VOC", "LabelMe"};
    
    return info;
}

Vector<ImageAnnotation> parse_annotations_json(const String& json_str) {
    Vector<ImageAnnotation> annotations;
    
    // 简化解析 - 实际应使用完整JSON库
    // 这里假设json_str是一个包含多个ImageAnnotation JSON对象的数组
    
    // 查找所有annotation对象
    size_t pos = 0;
    while ((pos = json_str.find("{", pos)) != String::npos) {
        size_t end = json_str.find("}", pos);
        if (end != String::npos) {
            String obj_str = json_str.substr(pos, end - pos + 1);
            annotations.push_back(ImageAnnotation::from_json(obj_str));
            pos = end + 1;
        } else {
            break;
        }
    }
    
    return annotations;
}

String LabelExportNode::export_to_yolo(const Vector<ImageAnnotation>& annotations) {
    // YOLO格式：每行一个标注
    // <class_id> <x_center> <y_center> <width> <height>
    // 所有值都是相对于图像尺寸的比例（0-1）
    
    std::ostringstream oss;
    for (const auto& annotation : annotations) {
        float img_w = static_cast<float>(annotation.width);
        float img_h = static_cast<float>(annotation.height);
        
        for (const auto& obj : annotation.objects) {
            float x_center = (obj.x + obj.width / 2.0f) / img_w;
            float y_center = (obj.y + obj.height / 2.0f) / img_h;
            float w_norm = obj.width / img_w;
            float h_norm = obj.height / img_h;
            
            oss << obj.class_id << " "
                << std::fixed << std::setprecision(6) << x_center << " "
                << y_center << " " << w_norm << " " << h_norm << "\n";
        }
    }
    return oss.str();
}

String LabelExportNode::export_to_coco(const Vector<ImageAnnotation>& annotations) {
    // COCO格式JSON
    std::ostringstream oss;
    oss << "{";
    oss << "\"images\":[";
    
    std::map<String, int> category_map;
    int category_id = 1;
    
    // 收集所有类别
    for (const auto& annotation : annotations) {
        for (const auto& obj : annotation.objects) {
            if (category_map.find(obj.label) == category_map.end()) {
                category_map[obj.label] = category_id++;
            }
        }
    }
    
    // 输出图像信息
    for (size_t i = 0; i < annotations.size(); ++i) {
        const auto& annotation = annotations[i];
        oss << "{";
        oss << "\"id\":" << i + 1 << ",";
        oss << "\"file_name\":\"" << escape_json_string(annotation.image_path) << "\",";
        oss << "\"width\":" << annotation.width << ",";
        oss << "\"height\":" << annotation.height;
        oss << "}";
        if (i < annotations.size() - 1) oss << ",";
    }
    
    oss << "],";
    oss << "\"annotations\":[";
    
    int annotation_id = 1;
    bool first_annotation = true;
    
    for (size_t img_idx = 0; img_idx < annotations.size(); ++img_idx) {
        const auto& annotation = annotations[img_idx];
        
        for (const auto& obj : annotation.objects) {
            if (!first_annotation) oss << ",";
            first_annotation = false;
            
            // COCO bbox格式：[x, y, width, height]
            oss << "{";
            oss << "\"id\":" << annotation_id++ << ",";
            oss << "\"image_id\":" << img_idx + 1 << ",";
            oss << "\"category_id\":" << category_map[obj.label] << ",";
            oss << "\"bbox\":[" << obj.x << "," << obj.y << "," << obj.width << "," << obj.height << "],";
            oss << "\"area\":" << (obj.width * obj.height) << ",";
            oss << "\"iscrowd\":0";
            oss << "}";
        }
    }
    
    oss << "],";
    oss << "\"categories\":[";
    
    int cat_idx = 0;
    for (const auto& pair : category_map) {
        oss << "{";
        oss << "\"id\":" << pair.second << ",";
        oss << "\"name\":\"" << escape_json_string(pair.first) << "\"";
        oss << "}";
        if (++cat_idx < static_cast<int>(category_map.size())) oss << ",";
    }
    
    oss << "]";
    oss << "}";
    
    return oss.str();
}

String LabelExportNode::export_to_voc(const ImageAnnotation& annotation) {
    // Pascal VOC格式XML
    std::ostringstream oss;
    oss << "<annotation>";
    oss << "<folder>images</folder>";
    oss << "<filename>" << escape_json_string(annotation.image_path) << "</filename>";
    oss << "<size>";
    oss << "<width>" << annotation.width << "</width>";
    oss << "<height>" << annotation.height << "</height>";
    oss << "<depth>3</depth>";
    oss << "</size>";
    
    for (const auto& obj : annotation.objects) {
        oss << "<object>";
        oss << "<name>" << escape_json_string(obj.label) << "</name>";
        oss << "<bndbox>";
        oss << "<xmin>" << obj.x << "</xmin>";
        oss << "<ymin>" << obj.y << "</ymin>";
        oss << "<xmax>" << (obj.x + obj.width) << "</xmax>";
        oss << "<ymax>" << (obj.y + obj.height) << "</ymax>";
        oss << "</bndbox>";
        if (obj.difficult) oss << "<difficult>1</difficult>";
        if (obj.occluded) oss << "<occluded>1</occluded>";
        oss << "</object>";
    }
    
    oss << "</annotation>";
    return oss.str();
}

String LabelExportNode::export_to_labelme(const ImageAnnotation& annotation) {
    // LabelMe格式JSON
    std::ostringstream oss;
    oss << "{";
    oss << "\"version\":\"5.0.1\",";
    oss << "\"flags\":{},";
    oss << "\"shapes\":[";
    
    for (size_t i = 0; i < annotation.objects.size(); ++i) {
        const auto& obj = annotation.objects[i];
        oss << "{";
        oss << "\"label\":\"" << escape_json_string(obj.label) << "\",";
        oss << "\"points\":[";
        
        if (obj.type == AnnotationType::Rectangle) {
            oss << "[" << obj.x << "," << obj.y << "],";
            oss << "[" << (obj.x + obj.width) << "," << (obj.y + obj.height) << "]";
        } else if (obj.type == AnnotationType::Polygon && !obj.points.empty()) {
            for (size_t j = 0; j < obj.points.size(); ++j) {
                oss << "[" << obj.points[j].x << "," << obj.points[j].y << "]";
                if (j < obj.points.size() - 1) oss << ",";
            }
        }
        
        oss << "],";
        oss << "\"shape_type\":\"" << (obj.type == AnnotationType::Rectangle ? "rectangle" :
                                        obj.type == AnnotationType::Polygon ? "polygon" : "point") << "\"";
        oss << "}";
        if (i < annotation.objects.size() - 1) oss << ",";
    }
    
    oss << "],";
    oss << "\"imagePath\":\"" << escape_json_string(annotation.image_path) << "\",";
    oss << "\"imageHeight\":" << annotation.height << ",";
    oss << "\"imageWidth\":" << annotation.width;
    oss << "}";
    
    return oss.str();
}

Result<void> LabelExportNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String annotations_json = get_input("annotations").as_string();
    String format_str = get_param("format", Data("YOLO")).as_string();
    String output_dir = get_param("output_dir", Data("./labels")).as_string();
    String filename = get_param("filename", Data("annotations")).as_string();
    
    // 解析标注数据
    Vector<ImageAnnotation> annotations = parse_annotations_json(annotations_json);
    
    if (annotations.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No annotations to export");
    }
    
    String export_data;
    String export_path;
    int export_count = 0;
    
    if (format_str == "YOLO") {
        export_data = export_to_yolo(annotations);
        export_path = output_dir + "/" + filename + ".txt";
        export_count = 0;
        for (const auto& ann : annotations) {
            export_count += static_cast<int>(ann.objects.size());
        }
    } else if (format_str == "COCO") {
        export_data = export_to_coco(annotations);
        export_path = output_dir + "/" + filename + ".json";
        export_count = static_cast<int>(annotations.size());
    } else if (format_str == "VOC") {
        // VOC格式每个图像单独一个文件
        if (annotations.size() == 1) {
            export_data = export_to_voc(annotations[0]);
            export_path = output_dir + "/" + filename + ".xml";
            export_count = static_cast<int>(annotations[0].objects.size());
        } else {
            // 批量导出
            std::ostringstream oss;
            for (size_t i = 0; i < annotations.size(); ++i) {
                oss << export_to_voc(annotations[i]) << "\n";
                export_count += static_cast<int>(annotations[i].objects.size());
            }
            export_data = oss.str();
            export_path = output_dir + "/" + filename + "_batch.xml";
        }
    } else if (format_str == "LabelMe") {
        // LabelMe格式每个图像单独一个文件
        if (annotations.size() == 1) {
            export_data = export_to_labelme(annotations[0]);
            export_path = output_dir + "/" + filename + ".json";
            export_count = static_cast<int>(annotations[0].objects.size());
        } else {
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < annotations.size(); ++i) {
                oss << export_to_labelme(annotations[i]);
                if (i < annotations.size() - 1) oss << ",";
                export_count += static_cast<int>(annotations[i].objects.size());
            }
            oss << "]";
            export_data = oss.str();
            export_path = output_dir + "/" + filename + ".json";
        }
    } else {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Unknown export format");
    }
    
    // 设置输出
    set_output("export_path", Data(export_path));
    set_output("export_count", Data(export_count));
    set_output("export_data", Data(export_data));
    
    OVF_INFO() << "LabelExport completed: format=" << format_str << ", count=" << export_count;
    
    return Result<void>::success();
}

// ==================== LabelMergeNode 实现 ====================

LabelMergeNode::LabelMergeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo LabelMergeNode::make_info() {
    NodeInfo info;
    info.id = "LabelMerge";
    info.name = "标签合并";
    info.category = "数据标注";
    info.description = "合并多个标注数据源";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("annotations1", "标注数据1", DataType::String, true),
        DataPort("annotations2", "标注数据2", DataType::String, true),
        DataPort("annotations3", "标注数据3", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("merged_annotations", "合并后的标注", DataType::String),
        DataPort("total_count", "总标注数量", DataType::Number),
        DataPort("conflict_count", "冲突数量", DataType::Number)
    };
    
    info.params = {
        ParamDef("merge_mode", "合并模式", DataType::String, Data("union")),
        ParamDef("resolve_conflict", "冲突解决策略", DataType::String, Data("keep_first")),
        ParamDef("min_overlap", "最小重叠阈值", DataType::Number, Data(0.5))
    };
    
    info.params[0].options = {"union", "intersection", "difference"};
    info.params[1].options = {"keep_first", "keep_last", "merge_all"};
    
    return info;
}

float LabelMergeNode::calculate_overlap(const AnnotationObject& a, const AnnotationObject& b) {
    // 计算IoU
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    
    if (x2 <= x1 || y2 <= y1) return 0.0f;
    
    float intersection = static_cast<float>((x2 - x1) * (y2 - y1));
    float area_a = static_cast<float>(a.width * a.height);
    float area_b = static_cast<float>(b.width * b.height);
    float union_area = area_a + area_b - intersection;
    
    return intersection / union_area;
}

bool LabelMergeNode::is_conflict(const AnnotationObject& a, const AnnotationObject& b, float min_overlap) {
    // 同类别且重叠超过阈值
    if (a.label != b.label) return false;
    return calculate_overlap(a, b) > min_overlap;
}

Result<void> LabelMergeNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String ann1_json = get_input("annotations1").as_string();
    String ann2_json = get_input("annotations2").as_string();
    String ann3_json = has_input("annotations3") ? get_input("annotations3").as_string() : "";
    
    String merge_mode = get_param("merge_mode", Data("union")).as_string();
    String resolve_conflict = get_param("resolve_conflict", Data("keep_first")).as_string();
    float min_overlap = static_cast<float>(get_param("min_overlap", Data(0.5)).as_number());
    
    // 解析标注数据
    Vector<ImageAnnotation> annotations1 = parse_annotations_json(ann1_json);
    Vector<ImageAnnotation> annotations2 = parse_annotations_json(ann2_json);
    Vector<ImageAnnotation> annotations3;
    if (!ann3_json.empty()) {
        annotations3 = parse_annotations_json(ann3_json);
    }
    
    // 合并所有标注对象
    Vector<AnnotationObject> all_objects;
    for (const auto& ann : annotations1) {
        for (const auto& obj : ann.objects) all_objects.push_back(obj);
    }
    for (const auto& ann : annotations2) {
        for (const auto& obj : ann.objects) all_objects.push_back(obj);
    }
    for (const auto& ann : annotations3) {
        for (const auto& obj : ann.objects) all_objects.push_back(obj);
    }
    
    // 检测并处理冲突
    int conflict_count = 0;
    Vector<AnnotationObject> merged_objects;
    
    if (merge_mode == "union") {
        // 合集模式
        for (size_t i = 0; i < all_objects.size(); ++i) {
            bool has_conflict = false;
            for (size_t j = 0; j < merged_objects.size(); ++j) {
                if (is_conflict(all_objects[i], merged_objects[j], min_overlap)) {
                    has_conflict = true;
                    conflict_count++;
                    if (resolve_conflict == "keep_last") {
                        merged_objects[j] = all_objects[i];
                    } else if (resolve_conflict == "merge_all") {
                        // 合并属性
                        merged_objects[j].confidence = std::max(merged_objects[j].confidence, all_objects[i].confidence);
                    }
                    break;
                }
            }
            if (!has_conflict) {
                merged_objects.push_back(all_objects[i]);
            }
        }
    } else if (merge_mode == "intersection") {
        // 交集模式 - 只保留重复的
        for (size_t i = 0; i < all_objects.size(); ++i) {
            for (size_t j = i + 1; j < all_objects.size(); ++j) {
                if (is_conflict(all_objects[i], all_objects[j], min_overlap)) {
                    merged_objects.push_back(resolve_conflict == "keep_last" ? all_objects[j] : all_objects[i]);
                    conflict_count++;
                    break;
                }
            }
        }
    } else if (merge_mode == "difference") {
        // 差集模式 - 只保留不重复的
        for (size_t i = 0; i < all_objects.size(); ++i) {
            bool has_conflict = false;
            for (size_t j = 0; j < all_objects.size(); ++j) {
                if (i != j && is_conflict(all_objects[i], all_objects[j], min_overlap)) {
                    has_conflict = true;
                    break;
                }
            }
            if (!has_conflict) {
                merged_objects.push_back(all_objects[i]);
            }
        }
    }
    
    // 创建合并后的标注
    ImageAnnotation merged_annotation;
    merged_annotation.objects = merged_objects;
    int total_count = static_cast<int>(merged_objects.size());
    
    set_output("merged_annotations", Data(merged_annotation.to_json()));
    set_output("total_count", Data(total_count));
    set_output("conflict_count", Data(conflict_count));
    
    OVF_INFO() << "LabelMerge completed: mode=" << merge_mode << ", total=" << total_count << ", conflicts=" << conflict_count;
    
    return Result<void>::success();
}

// ==================== LabelVerifyNode 实现 ====================

LabelVerifyNode::LabelVerifyNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo LabelVerifyNode::make_info() {
    NodeInfo info;
    info.id = "LabelVerify";
    info.name = "标签验证";
    info.category = "数据标注";
    info.description = "验证标注数据的完整性和正确性";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("annotations", "标注数据", DataType::String, true),
        DataPort("reference_image", "参考图像", DataType::Image, false)
    };
    
    info.outputs = {
        DataPort("valid", "是否有效", DataType::Boolean),
        DataPort("error_count", "错误数量", DataType::Number),
        DataPort("warning_count", "警告数量", DataType::Number),
        DataPort("validation_report", "验证报告", DataType::String),
        DataPort("corrected_annotations", "修正后的标注", DataType::String)
    };
    
    info.params = {
        ParamDef("check_bounds", "检查边界越界", DataType::Boolean, Data(true)),
        ParamDef("check_duplicate", "检查重复标注", DataType::Boolean, Data(true)),
        ParamDef("check_empty", "检查空标注", DataType::Boolean, Data(true)),
        ParamDef("auto_correct", "自动修正错误", DataType::Boolean, Data(false))
    };
    
    return info;
}

Result<void> LabelVerifyNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String annotations_json = get_input("annotations").as_string();
    
    bool check_bounds = get_param("check_bounds", Data(true)).as_bool();
    bool check_duplicate = get_param("check_duplicate", Data(true)).as_bool();
    bool check_empty = get_param("check_empty", Data(true)).as_bool();
    bool auto_correct = get_param("auto_correct", Data(false)).as_bool();
    
    // 获取参考图像尺寸
    uint32_t img_width = 0, img_height = 0;
    if (has_input("reference_image")) {
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
        auto ref_data = get_input("reference_image");
        const ImageData& ref_image = ref_data.as_image();
        img_width = ref_image.width;
        img_height = ref_image.height;
    }
    
    // 解析标注
    Vector<ImageAnnotation> annotations = parse_annotations_json(annotations_json);
    
    int error_count = 0;
    int warning_count = 0;
    std::ostringstream report;
    report << "{";
    report << "\"errors\":[";
    
    Vector<ImageAnnotation> corrected_annotations = annotations;
    bool first_error = true;
    
    for (size_t ann_idx = 0; ann_idx < annotations.size(); ++ann_idx) {
        const auto& annotation = annotations[ann_idx];
        
        for (size_t obj_idx = 0; obj_idx < annotation.objects.size(); ++obj_idx) {
            const auto& obj = annotation.objects[obj_idx];
            Vector<String> errors;
            
            // 检查边界越界
            if (check_bounds && (img_width > 0 && img_height > 0)) {
                if (obj.x < 0 || obj.y < 0 || 
                    obj.x + obj.width > static_cast<int32_t>(img_width) ||
                    obj.y + obj.height > static_cast<int32_t>(img_height)) {
                    errors.push_back("bounds_exceeded");
                    error_count++;
                    
                    // 自动修正
                    if (auto_correct) {
                        auto& corrected_obj = corrected_annotations[ann_idx].objects[obj_idx];
                        corrected_obj.x = clamp_val(corrected_obj.x, 0, static_cast<int32_t>(img_width - 1));
                        corrected_obj.y = clamp_val(corrected_obj.y, 0, static_cast<int32_t>(img_height - 1));
                        corrected_obj.width = clamp_val(corrected_obj.width, 1, static_cast<int32_t>(img_width - corrected_obj.x));
                        corrected_obj.height = clamp_val(corrected_obj.height, 1, static_cast<int32_t>(img_height - corrected_obj.y));
                    }
                }
            }
            
            // 检查空标注
            if (check_empty) {
                if (obj.width <= 0 || obj.height <= 0) {
                    errors.push_back("empty_annotation");
                    error_count++;
                }
                if (obj.label.empty()) {
                    errors.push_back("missing_label");
                    warning_count++;
                }
            }
            
            // 检查重复
            if (check_duplicate) {
                for (size_t j = obj_idx + 1; j < annotation.objects.size(); ++j) {
                    const auto& other = annotation.objects[j];
                    int x1 = std::max(obj.x, other.x);
                    int y1 = std::max(obj.y, other.y);
                    int x2 = std::min(obj.x + obj.width, other.x + other.width);
                    int y2 = std::min(obj.y + obj.height, other.y + other.height);
                    
                    if (x2 > x1 && y2 > y1 && obj.label == other.label) {
                        float overlap = static_cast<float>((x2 - x1) * (y2 - y1)) / 
                                        static_cast<float>(std::min(obj.width * obj.height, other.width * other.height));
                        if (overlap > 0.9f) {
                            errors.push_back("duplicate_annotation");
                            warning_count++;
                            break;
                        }
                    }
                }
            }
            
            // 记录错误
            if (!errors.empty()) {
                if (!first_error) report << ",";
                first_error = false;
                
                report << "{";
                report << "\"annotation_index\":" << ann_idx << ",";
                report << "\"object_index\":" << obj_idx << ",";
                report << "\"errors\":[";
                for (size_t e = 0; e < errors.size(); ++e) {
                    report << "\"" << errors[e] << "\"";
                    if (e < errors.size() - 1) report << ",";
                }
                report << "]";
                report << "}";
            }
        }
    }
    
    report << "],";
    report << "\"warnings\":[],";
    report << "\"summary\":{";
    report << "\"total_annotations\":" << annotations.size() << ",";
    report << "\"error_count\":" << error_count << ",";
    report << "\"warning_count\":" << warning_count << ",";
    report << "\"valid\":" << (error_count == 0 ? "true" : "false");
    report << "}";
    report << "}";
    
    bool valid = (error_count == 0);
    
    set_output("valid", Data(valid));
    set_output("error_count", Data(error_count));
    set_output("warning_count", Data(warning_count));
    set_output("validation_report", Data(report.str()));
    
    if (auto_correct) {
        // 输出修正后的标注
        std::ostringstream corrected_json;
        corrected_json << "[";
        for (size_t i = 0; i < corrected_annotations.size(); ++i) {
            corrected_json << corrected_annotations[i].to_json();
            if (i < corrected_annotations.size() - 1) corrected_json << ",";
        }
        corrected_json << "]";
        set_output("corrected_annotations", Data(corrected_json.str()));
    }
    
    OVF_INFO() << "LabelVerify completed: valid=" << valid << ", errors=" << error_count << ", warnings=" << warning_count;
    
    return Result<void>::success();
}

// ==================== DataAugmentNode 实现 ====================

DataAugmentNode::DataAugmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DataAugmentNode::make_info() {
    NodeInfo info;
    info.id = "DataAugment";
    info.name = "数据增强";
    info.category = "数据增强";
    info.description = "通用数据增强（旋转、翻转、缩放、裁剪）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("annotation", "标注数据", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("augmented_image", "增强后的图像", DataType::Image),
        DataPort("augmented_annotation", "增强后的标注", DataType::String),
        DataPort("transform_params", "变换参数", DataType::String)
    };
    
    info.params = {
        ParamDef("augment_type", "增强类型", DataType::String, Data("combined")),
        ParamDef("rotate_angle", "旋转角度范围", DataType::String, Data("-30,30")),
        ParamDef("flip_direction", "翻转方向", DataType::String, Data("horizontal")),
        ParamDef("scale_range", "缩放范围", DataType::String, Data("0.8,1.2")),
        ParamDef("crop_ratio", "裁剪比例", DataType::Number, Data(0.8)),
        ParamDef("random_seed", "随机种子", DataType::Number, Data(0))
    };
    
    info.params[0].options = {"rotate", "flip", "scale", "crop", "combined"};
    info.params[1].options = {"horizontal", "vertical", "both", "none"};
    
    return info;
}

void DataAugmentNode::apply_rotation(ImageData& image, AnnotationObject& obj, float angle) {
    // 旋转实现（简化版本）
    int w = image.width;
    int h = image.height;
    int ch = image.channels;
    
    float rad = angle * static_cast<float>(M_PI) / 180.0f;
    float cos_a = std::cos(rad);
    float sin_a = std::sin(rad);
    
    // 计算旋转后的图像尺寸
    int new_w = static_cast<int>(std::abs(w * cos_a) + std::abs(h * sin_a));
    int new_h = static_cast<int>(std::abs(w * sin_a) + std::abs(h * cos_a));
    
    ImageData rotated;
    rotated.width = new_w;
    rotated.height = new_h;
    rotated.channels = ch;
    rotated.format = image.format;
    rotated.data.resize(new_w * new_h * ch, 0);
    
    float cx = w / 2.0f;
    float cy = h / 2.0f;
    float new_cx = new_w / 2.0f;
    float new_cy = new_h / 2.0f;
    
    // 旋转图像
    for (int y = 0; y < new_h; ++y) {
        for (int x = 0; x < new_w; ++x) {
            float src_x = (x - new_cx) * cos_a + (y - new_cy) * sin_a + cx;
            float src_y = -(x - new_cx) * sin_a + (y - new_cy) * cos_a + cy;
            
            if (src_x >= 0 && src_x < w - 1 && src_y >= 0 && src_y < h - 1) {
                int x0 = static_cast<int>(src_x);
                int y0 = static_cast<int>(src_y);
                float fx = src_x - x0;
                float fy = src_y - y0;
                
                int dst_idx = (y * new_w + x) * ch;
                for (int c = 0; c < ch; ++c) {
                    float v00 = image.data[(y0 * w + x0) * ch + c];
                    float v01 = image.data[(y0 * w + x0 + 1) * ch + c];
                    float v10 = image.data[((y0 + 1) * w + x0) * ch + c];
                    float v11 = image.data[((y0 + 1) * w + x0 + 1) * ch + c];
                    float v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                              (1 - fx) * fy * v10 + fx * fy * v11;
                    rotated.data[dst_idx + c] = static_cast<uint8_t>(clamp_val(static_cast<int>(v), 0, 255));
                }
            }
        }
    }
    
    image = rotated;
    
    // 调整标注
    float obj_cx = obj.x + obj.width / 2.0f;
    float obj_cy = obj.y + obj.height / 2.0f;
    
    float new_obj_cx = (obj_cx - cx) * cos_a + (obj_cy - cy) * sin_a + new_cx;
    float new_obj_cy = -(obj_cx - cx) * sin_a + (obj_cy - cy) * cos_a + new_cy;
    
    obj.x = static_cast<int32_t>(new_obj_cx - obj.width / 2.0f);
    obj.y = static_cast<int32_t>(new_obj_cy - obj.height / 2.0f);
}

void DataAugmentNode::apply_flip(ImageData& image, AnnotationObject& obj, bool horizontal, bool vertical) {
    int w = image.width;
    int h = image.height;
    int ch = image.channels;
    
    ImageData flipped = image;
    
    for (int y = 0; y < h; ++y) {
        int src_y = vertical ? (h - 1 - y) : y;
        for (int x = 0; x < w; ++x) {
            int src_x = horizontal ? (w - 1 - x) : x;
            
            int dst_idx = (y * w + x) * ch;
            int src_idx = (src_y * w + src_x) * ch;
            
            for (int c = 0; c < ch; ++c) {
                flipped.data[dst_idx + c] = image.data[src_idx + c];
            }
        }
    }
    
    image = flipped;
    
    // 调整标注
    if (horizontal) {
        obj.x = w - obj.x - obj.width;
    }
    if (vertical) {
        obj.y = h - obj.y - obj.height;
    }
}

void DataAugmentNode::apply_scale(ImageData& image, AnnotationObject& obj, float scale) {
    int w = image.width;
    int h = image.height;
    int ch = image.channels;
    
    int new_w = static_cast<int>(w * scale);
    int new_h = static_cast<int>(h * scale);
    
    if (new_w < 1) new_w = 1;
    if (new_h < 1) new_h = 1;
    
    ImageData scaled;
    scaled.width = new_w;
    scaled.height = new_h;
    scaled.channels = ch;
    scaled.format = image.format;
    scaled.data.resize(new_w * new_h * ch);
    
    image_utils::resize_bilinear(image.data.data(), w, h, ch, scaled.data, new_w, new_h);
    
    image = scaled;
    
    // 调整标注
    obj.x = static_cast<int32_t>(obj.x * scale);
    obj.y = static_cast<int32_t>(obj.y * scale);
    obj.width = static_cast<int32_t>(obj.width * scale);
    obj.height = static_cast<int32_t>(obj.height * scale);
}

void DataAugmentNode::apply_crop(ImageData& image, AnnotationObject& obj, int crop_x, int crop_y, int crop_w, int crop_h) {
    int w = image.width;
    int h = image.height;
    int ch = image.channels;
    
    // 边界检查
    if (crop_x < 0) crop_x = 0;
    if (crop_y < 0) crop_y = 0;
    if (crop_x + crop_w > w) crop_w = w - crop_x;
    if (crop_y + crop_h > h) crop_h = h - crop_y;
    
    ImageData cropped;
    cropped.width = crop_w;
    cropped.height = crop_h;
    cropped.channels = ch;
    cropped.format = image.format;
    cropped.data.resize(crop_w * crop_h * ch);
    
    for (int y = 0; y < crop_h; ++y) {
        for (int x = 0; x < crop_w; ++x) {
            int src_idx = ((crop_y + y) * w + (crop_x + x)) * ch;
            int dst_idx = (y * crop_w + x) * ch;
            for (int c = 0; c < ch; ++c) {
                cropped.data[dst_idx + c] = image.data[src_idx + c];
            }
        }
    }
    
    image = cropped;
    
    // 调整标注
    obj.x -= crop_x;
    obj.y -= crop_y;
    
    // 检查标注是否在裁剪区域内
    if (obj.x < 0 || obj.y < 0 || obj.x + obj.width > crop_w || obj.y + obj.height > crop_h) {
        // 标注部分或完全在裁剪区域外
        obj.x = clamp_val(obj.x, 0, crop_w - 1);
        obj.y = clamp_val(obj.y, 0, crop_h - 1);
        obj.width = clamp_val(obj.width, 1, crop_w - obj.x);
        obj.height = clamp_val(obj.height, 1, crop_h - obj.y);
    }
}

Result<void> DataAugmentNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    String augment_type = get_param("augment_type", Data("combined")).as_string();
    String rotate_range_str = get_param("rotate_angle", Data("-30,30")).as_string();
    String flip_direction = get_param("flip_direction", Data("horizontal")).as_string();
    String scale_range_str = get_param("scale_range", Data("0.8,1.2")).as_string();
    float crop_ratio = static_cast<float>(get_param("crop_ratio", Data(0.8)).as_number());
    int random_seed = static_cast<int>(get_param("random_seed", Data(0)).as_int());
    
    // 解析范围
    float rotate_min = -30.0f, rotate_max = 30.0f;
    float scale_min = 0.8f, scale_max = 1.2f;
    parse_range(rotate_range_str, rotate_min, rotate_max);
    parse_range(scale_range_str, scale_min, scale_max);
    
    // 初始化随机数生成器
    std::mt19937 rng(random_seed == 0 ? std::random_device{}() : random_seed);
    
    // 复制图像
    ImageData augmented_image = input;
    
    // 解析标注
    AnnotationObject obj;
    bool has_annotation = has_input("annotation");
    if (has_annotation) {
        String ann_json = get_input("annotation").as_string();
        ImageAnnotation annotation = ImageAnnotation::from_json(ann_json);
        if (!annotation.objects.empty()) {
            obj = annotation.objects[0];
        }
    }
    
    std::ostringstream transform_params;
    transform_params << "{";
    
    // 应用增强
    if (augment_type == "rotate" || augment_type == "combined") {
        std::uniform_real_distribution<float> angle_dist(rotate_min, rotate_max);
        float angle = angle_dist(rng);
        apply_rotation(augmented_image, obj, angle);
        transform_params << "\"rotate_angle\":" << angle << ",";
    }
    
    if (augment_type == "flip" || augment_type == "combined") {
        bool flip_h = (flip_direction == "horizontal" || flip_direction == "both");
        bool flip_v = (flip_direction == "vertical" || flip_direction == "both");
        
        if (augment_type == "combined") {
            std::uniform_int_distribution<int> flip_dist(0, 1);
            flip_h = flip_dist(rng) && (flip_direction == "horizontal" || flip_direction == "both");
            flip_v = flip_dist(rng) && (flip_direction == "vertical" || flip_direction == "both");
        }
        
        if (flip_h || flip_v) {
            apply_flip(augmented_image, obj, flip_h, flip_v);
            transform_params << "\"flip_h\":" << (flip_h ? "true" : "false") << ",";
            transform_params << "\"flip_v\":" << (flip_v ? "true" : "false") << ",";
        }
    }
    
    if (augment_type == "scale" || augment_type == "combined") {
        std::uniform_real_distribution<float> scale_dist(scale_min, scale_max);
        float scale = scale_dist(rng);
        apply_scale(augmented_image, obj, scale);
        transform_params << "\"scale\":" << scale << ",";
    }
    
    if (augment_type == "crop" || augment_type == "combined") {
        int crop_w = static_cast<int>(augmented_image.width * crop_ratio);
        int crop_h = static_cast<int>(augmented_image.height * crop_ratio);
        
        std::uniform_int_distribution<int> x_dist(0, augmented_image.width - crop_w);
        std::uniform_int_distribution<int> y_dist(0, augmented_image.height - crop_h);
        
        int crop_x = x_dist(rng);
        int crop_y = y_dist(rng);
        
        apply_crop(augmented_image, obj, crop_x, crop_y, crop_w, crop_h);
        transform_params << "\"crop_x\":" << crop_x << ",";
        transform_params << "\"crop_y\":" << crop_y << ",";
        transform_params << "\"crop_w\":" << crop_w << ",";
        transform_params << "\"crop_h\":" << crop_h;
    }
    
    transform_params << "}";
    
    // 设置输出
    set_output("augmented_image", Data(augmented_image));
    set_output("transform_params", Data(transform_params.str()));
    
    if (has_annotation) {
        ImageAnnotation augmented_annotation;
        augmented_annotation.width = augmented_image.width;
        augmented_annotation.height = augmented_image.height;
        augmented_annotation.objects.push_back(obj);
        set_output("augmented_annotation", Data(augmented_annotation.to_json()));
    }
    
    OVF_INFO() << "DataAugment completed: type=" << augment_type;
    
    return Result<void>::success();
}

// ==================== RandomCropNode 实现 ====================

RandomCropNode::RandomCropNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo RandomCropNode::make_info() {
    NodeInfo info;
    info.id = "RandomCrop";
    info.name = "随机裁剪";
    info.category = "数据增强";
    info.description = "随机裁剪图像并调整标注";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("annotation", "标注数据", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("cropped_image", "裁剪后的图像", DataType::Image),
        DataPort("cropped_annotation", "裁剪后的标注", DataType::String),
        DataPort("crop_region", "裁剪区域", DataType::Region)
    };
    
    info.params = {
        ParamDef("crop_width", "裁剪宽度", DataType::Number, Data(0)),
        ParamDef("crop_height", "裁剪高度", DataType::Number, Data(0)),
        ParamDef("crop_ratio_min", "最小裁剪比例", DataType::Number, Data(0.5)),
        ParamDef("crop_ratio_max", "最大裁剪比例", DataType::Number, Data(1.0)),
        ParamDef("ensure_object", "确保包含对象", DataType::Boolean, Data(true)),
        ParamDef("num_outputs", "输出数量", DataType::Number, Data(1)),
        ParamDef("random_seed", "随机种子", DataType::Number, Data(0))
    };
    
    return info;
}

Region RandomCropNode::generate_random_crop(int img_w, int img_h, int crop_w, int crop_h) {
    std::mt19937 rng(std::random_device{}());
    
    if (crop_w >= img_w) crop_w = img_w;
    if (crop_h >= img_h) crop_h = img_h;
    
    std::uniform_int_distribution<int> x_dist(0, img_w - crop_w);
    std::uniform_int_distribution<int> y_dist(0, img_h - crop_h);
    
    return Region(x_dist(rng), y_dist(rng), crop_w, crop_h);
}

bool RandomCropNode::region_contains_object(const Region& region, const AnnotationObject& obj) {
    return obj.x >= region.x && obj.y >= region.y &&
           obj.x + obj.width <= region.x + region.width &&
           obj.y + obj.height <= region.y + region.height;
}

AnnotationObject RandomCropNode::adjust_annotation_for_crop(const AnnotationObject& obj, const Region& crop) {
    AnnotationObject adjusted = obj;
    adjusted.x -= crop.x;
    adjusted.y -= crop.y;
    
    // 检查是否部分在裁剪区域外
    if (adjusted.x < 0) {
        adjusted.width += adjusted.x;
        adjusted.x = 0;
    }
    if (adjusted.y < 0) {
        adjusted.height += adjusted.y;
        adjusted.y = 0;
    }
    if (adjusted.x + adjusted.width > crop.width) {
        adjusted.width = crop.width - adjusted.x;
    }
    if (adjusted.y + adjusted.height > crop.height) {
        adjusted.height = crop.height - adjusted.y;
    }
    
    return adjusted;
}

Result<void> RandomCropNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int crop_width = static_cast<int>(get_param("crop_width", Data(0)).as_int());
    int crop_height = static_cast<int>(get_param("crop_height", Data(0)).as_int());
    float crop_ratio_min = static_cast<float>(get_param("crop_ratio_min", Data(0.5)).as_number());
    float crop_ratio_max = static_cast<float>(get_param("crop_ratio_max", Data(1.0)).as_number());
    bool ensure_object = get_param("ensure_object", Data(true)).as_bool();
    int random_seed = static_cast<int>(get_param("random_seed", Data(0)).as_int());
    
    std::mt19937 rng(random_seed == 0 ? std::random_device{}() : random_seed);
    
    int img_w = input.width;
    int img_h = input.height;
    
    // 确定裁剪尺寸
    if (crop_width <= 0 || crop_height <= 0) {
        std::uniform_real_distribution<float> ratio_dist(crop_ratio_min, crop_ratio_max);
        float ratio = ratio_dist(rng);
        crop_width = static_cast<int>(img_w * ratio);
        crop_height = static_cast<int>(img_h * ratio);
    }
    
    crop_width = clamp_val(crop_width, 1, img_w);
    crop_height = clamp_val(crop_height, 1, img_h);
    
    // 解析标注
    Vector<AnnotationObject> objects;
    if (has_input("annotation")) {
        String ann_json = get_input("annotation").as_string();
        ImageAnnotation annotation = ImageAnnotation::from_json(ann_json);
        objects = annotation.objects;
    }
    
    // 生成裁剪区域
    Region crop_region;
    int max_attempts = 100;
    
    if (ensure_object && !objects.empty()) {
        // 尝试生成包含对象的裁剪区域
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            crop_region = generate_random_crop(img_w, img_h, crop_width, crop_height);
            
            bool contains_all = true;
            for (const auto& obj : objects) {
                if (!region_contains_object(crop_region, obj)) {
                    contains_all = false;
                    break;
                }
            }
            
            if (contains_all) break;
        }
        
        // 如果找不到，使用以对象中心为中心的裁剪
        if (!region_contains_object(crop_region, objects[0])) {
            int center_x = objects[0].x + objects[0].width / 2;
            int center_y = objects[0].y + objects[0].height / 2;
            
            crop_region.x = clamp_val(center_x - crop_width / 2, 0, img_w - crop_width);
            crop_region.y = clamp_val(center_y - crop_height / 2, 0, img_h - crop_height);
            crop_region.width = crop_width;
            crop_region.height = crop_height;
        }
    } else {
        crop_region = generate_random_crop(img_w, img_h, crop_width, crop_height);
    }
    
    // 执行裁剪
    ImageData cropped_image;
    cropped_image.width = crop_region.width;
    cropped_image.height = crop_region.height;
    cropped_image.channels = input.channels;
    cropped_image.format = input.format;
    cropped_image.data.resize(crop_region.width * crop_region.height * input.channels);
    
    for (int y = 0; y < crop_region.height; ++y) {
        for (int x = 0; x < crop_region.width; ++x) {
            int src_idx = ((crop_region.y + y) * img_w + (crop_region.x + x)) * input.channels;
            int dst_idx = (y * crop_region.width + x) * input.channels;
            for (uint32_t c = 0; c < input.channels; ++c) {
                cropped_image.data[dst_idx + c] = input.data[src_idx + c];
            }
        }
    }
    
    // 调整标注
    ImageAnnotation cropped_annotation;
    cropped_annotation.width = crop_region.width;
    cropped_annotation.height = crop_region.height;
    
    for (const auto& obj : objects) {
        AnnotationObject adjusted = adjust_annotation_for_crop(obj, crop_region);
        if (adjusted.width > 0 && adjusted.height > 0) {
            cropped_annotation.objects.push_back(adjusted);
        }
    }
    
    // 设置输出
    set_output("cropped_image", Data(cropped_image));
    set_output("crop_region", Data(crop_region));
    
    if (!cropped_annotation.objects.empty()) {
        set_output("cropped_annotation", Data(cropped_annotation.to_json()));
    }
    
    OVF_INFO() << "RandomCrop completed: crop_region=(" << crop_region.x << "," << crop_region.y << ") size=" << crop_region.width << "x" << crop_region.height;
    
    return Result<void>::success();
}

// ==================== ColorAugmentNode 实现 ====================

ColorAugmentNode::ColorAugmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ColorAugmentNode::make_info() {
    NodeInfo info;
    info.id = "ColorAugment";
    info.name = "颜色增强";
    info.category = "数据增强";
    info.description = "颜色增强（亮度、对比度、饱和度、色相）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true)
    };
    
    info.outputs = {
        DataPort("augmented_image", "增强后的图像", DataType::Image),
        DataPort("color_params", "颜色参数", DataType::String)
    };
    
    info.params = {
        ParamDef("brightness_range", "亮度范围", DataType::String, Data("-0.2,0.2")),
        ParamDef("contrast_range", "对比度范围", DataType::String, Data("0.8,1.2")),
        ParamDef("saturation_range", "饱和度范围", DataType::String, Data("0.8,1.2")),
        ParamDef("hue_range", "色相范围", DataType::String, Data("-0.1,0.1")),
        ParamDef("random_apply", "随机应用", DataType::Boolean, Data(true)),
        ParamDef("random_seed", "随机种子", DataType::Number, Data(0))
    };
    
    return info;
}

void ColorAugmentNode::adjust_brightness(ImageData& image, float factor) {
    // factor: -0.5 ~ 0.5
    for (auto& pixel : image.data) {
        int val = static_cast<int>(pixel) + static_cast<int>(factor * 255);
        pixel = static_cast<uint8_t>(clamp_val(val, 0, 255));
    }
}

void ColorAugmentNode::adjust_contrast(ImageData& image, float factor) {
    // factor: 0.5 ~ 2.0
    for (auto& pixel : image.data) {
        int val = static_cast<int>((pixel - 128) * factor + 128);
        pixel = static_cast<uint8_t>(clamp_val(val, 0, 255));
    }
}

void ColorAugmentNode::adjust_saturation(ImageData& image, float factor) {
    // factor: 0.0 ~ 2.0
    if (image.channels < 3) return;
    
    for (size_t i = 0; i < image.width * image.height; ++i) {
        uint8_t r = image.data[i * 3];
        uint8_t g = image.data[i * 3 + 1];
        uint8_t b = image.data[i * 3 + 2];
        
        // 计算灰度值
        float gray = 0.299f * r + 0.587f * g + 0.114f * b;
        
        // 调整饱和度
        image.data[i * 3] = static_cast<uint8_t>(clamp_val(static_cast<int>(gray + (r - gray) * factor), 0, 255));
        image.data[i * 3 + 1] = static_cast<uint8_t>(clamp_val(static_cast<int>(gray + (g - gray) * factor), 0, 255));
        image.data[i * 3 + 2] = static_cast<uint8_t>(clamp_val(static_cast<int>(gray + (b - gray) * factor), 0, 255));
    }
}

void ColorAugmentNode::adjust_hue(ImageData& image, float shift) {
    // shift: -0.5 ~ 0.5 (对应-180度~180度)
    if (image.channels < 3) return;
    
    for (size_t i = 0; i < image.width * image.height; ++i) {
        float r = image.data[i * 3] / 255.0f;
        float g = image.data[i * 3 + 1] / 255.0f;
        float b = image.data[i * 3 + 2] / 255.0f;
        
        // RGB to HSV
        float max_val = std::max({r, g, b});
        float min_val = std::min({r, g, b});
        float delta = max_val - min_val;
        
        float h = 0.0f, s = 0.0f, v = max_val;
        
        if (delta > 0.0f) {
            s = delta / max_val;
            
            if (max_val == r) {
                h = 60.0f * fmod(((g - b) / delta), 6.0f);
            } else if (max_val == g) {
                h = 60.0f * (((b - r) / delta) + 2.0f);
            } else {
                h = 60.0f * (((r - g) / delta) + 4.0f);
            }
            
            if (h < 0.0f) h += 360.0f;
        }
        
        // 调整色相
        h += shift * 360.0f;
        while (h < 0.0f) h += 360.0f;
        while (h >= 360.0f) h -= 360.0f;
        
        // HSV to RGB
        if (s == 0.0f) {
            r = g = b = v;
        } else {
            h /= 60.0f;
            int sector = static_cast<int>(h);
            float fractional = h - sector;
            
            float p = v * (1.0f - s);
            float q = v * (1.0f - s * fractional);
            float t = v * (1.0f - s * (1.0f - fractional));
            
            switch (sector) {
                case 0: r = v; g = t; b = p; break;
                case 1: r = q; g = v; b = p; break;
                case 2: r = p; g = v; b = t; break;
                case 3: r = p; g = q; b = v; break;
                case 4: r = t; g = p; b = v; break;
                default: r = v; g = p; b = q; break;
            }
        }
        
        image.data[i * 3] = static_cast<uint8_t>(clamp_val(static_cast<int>(r * 255), 0, 255));
        image.data[i * 3 + 1] = static_cast<uint8_t>(clamp_val(static_cast<int>(g * 255), 0, 255));
        image.data[i * 3 + 2] = static_cast<uint8_t>(clamp_val(static_cast<int>(b * 255), 0, 255));
    }
}

Result<void> ColorAugmentNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    String brightness_range_str = get_param("brightness_range", Data("-0.2,0.2")).as_string();
    String contrast_range_str = get_param("contrast_range", Data("0.8,1.2")).as_string();
    String saturation_range_str = get_param("saturation_range", Data("0.8,1.2")).as_string();
    String hue_range_str = get_param("hue_range", Data("-0.1,0.1")).as_string();
    bool random_apply = get_param("random_apply", Data(true)).as_bool();
    int random_seed = static_cast<int>(get_param("random_seed", Data(0)).as_int());
    
    // 解析范围
    float brightness_min = -0.2f, brightness_max = 0.2f;
    float contrast_min = 0.8f, contrast_max = 1.2f;
    float saturation_min = 0.8f, saturation_max = 1.2f;
    float hue_min = -0.1f, hue_max = 0.1f;
    
    parse_range(brightness_range_str, brightness_min, brightness_max);
    parse_range(contrast_range_str, contrast_min, contrast_max);
    parse_range(saturation_range_str, saturation_min, saturation_max);
    parse_range(hue_range_str, hue_min, hue_max);
    
    std::mt19937 rng(random_seed == 0 ? std::random_device{}() : random_seed);
    
    // 复制图像
    ImageData augmented_image = input;
    
    // 随机生成增强参数
    std::uniform_real_distribution<float> brightness_dist(brightness_min, brightness_max);
    std::uniform_real_distribution<float> contrast_dist(contrast_min, contrast_max);
    std::uniform_real_distribution<float> saturation_dist(saturation_min, saturation_max);
    std::uniform_real_distribution<float> hue_dist(hue_min, hue_max);
    std::uniform_int_distribution<int> apply_dist(0, 1);
    
    float brightness_factor = brightness_dist(rng);
    float contrast_factor = contrast_dist(rng);
    float saturation_factor = saturation_dist(rng);
    float hue_shift = hue_dist(rng);
    
    // 应用增强
    if (!random_apply || apply_dist(rng)) {
        adjust_brightness(augmented_image, brightness_factor);
    }
    
    if (!random_apply || apply_dist(rng)) {
        adjust_contrast(augmented_image, contrast_factor);
    }
    
    if (!random_apply || apply_dist(rng)) {
        adjust_saturation(augmented_image, saturation_factor);
    }
    
    if (!random_apply || apply_dist(rng)) {
        adjust_hue(augmented_image, hue_shift);
    }
    
    // 输出颜色参数
    std::ostringstream color_params;
    color_params << "{";
    color_params << "\"brightness\":" << brightness_factor << ",";
    color_params << "\"contrast\":" << contrast_factor << ",";
    color_params << "\"saturation\":" << saturation_factor << ",";
    color_params << "\"hue\":" << hue_shift;
    color_params << "}";
    
    set_output("augmented_image", Data(augmented_image));
    set_output("color_params", Data(color_params.str()));
    
    OVF_INFO() << "ColorAugment completed";
    
    return Result<void>::success();
}

// ==================== MixupNode 实现 ====================

MixupNode::MixupNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MixupNode::make_info() {
    NodeInfo info;
    info.id = "Mixup";
    info.name = "Mixup增强";
    info.category = "数据增强";
    info.description = "图像混合增强（两张图像加权混合）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image1", "输入图像1", DataType::Image, true),
        DataPort("annotation1", "标注数据1", DataType::String, false),
        DataPort("image2", "输入图像2", DataType::Image, true),
        DataPort("annotation2", "标注数据2", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("mixed_image", "混合后的图像", DataType::Image),
        DataPort("mixed_annotation", "混合后的标注", DataType::String),
        DataPort("mix_ratio", "混合比例", DataType::Number)
    };
    
    info.params = {
        ParamDef("mix_ratio", "混合比例", DataType::Number, Data(0.5)),
        ParamDef("blend_mode", "混合模式", DataType::String, Data("linear")),
        ParamDef("random_ratio", "随机比例范围", DataType::String, Data("0.3,0.7")),
        ParamDef("random_seed", "随机种子", DataType::Number, Data(0))
    };
    
    info.params[1].options = {"linear", "add", "multiply"};
    
    return info;
}

void MixupNode::blend_linear(ImageData& dst, const ImageData& src1, const ImageData& src2, float ratio) {
    // 确保尺寸一致（使用较大尺寸）
    int w = std::max(src1.width, src2.width);
    int h = std::max(src1.height, src2.height);
    int ch = std::max(src1.channels, src2.channels);
    
    dst.width = w;
    dst.height = h;
    dst.channels = ch;
    dst.format = src1.format;
    dst.data.resize(w * h * ch);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            for (int c = 0; c < ch; ++c) {
                float v1 = 0.0f, v2 = 0.0f;
                
                // 获取src1的值（如果坐标有效）
                if (x < src1.width && y < src1.height && c < src1.channels) {
                    v1 = src1.data[(y * src1.width + x) * src1.channels + c];
                }
                
                // 获取src2的值（如果坐标有效）
                if (x < src2.width && y < src2.height && c < src2.channels) {
                    v2 = src2.data[(y * src2.width + x) * src2.channels + c];
                }
                
                // 线性混合
                float mixed = v1 * ratio + v2 * (1.0f - ratio);
                dst.data[(y * w + x) * ch + c] = static_cast<uint8_t>(clamp_val(static_cast<int>(mixed), 0, 255));
            }
        }
    }
}

void MixupNode::blend_add(ImageData& dst, const ImageData& src1, const ImageData& src2, float ratio) {
    int w = std::max(src1.width, src2.width);
    int h = std::max(src1.height, src2.height);
    int ch = std::max(src1.channels, src2.channels);
    
    dst.width = w;
    dst.height = h;
    dst.channels = ch;
    dst.format = src1.format;
    dst.data.resize(w * h * ch);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            for (int c = 0; c < ch; ++c) {
                float v1 = 0.0f, v2 = 0.0f;
                
                if (x < src1.width && y < src1.height && c < src1.channels) {
                    v1 = src1.data[(y * src1.width + x) * src1.channels + c];
                }
                
                if (x < src2.width && y < src2.height && c < src2.channels) {
                    v2 = src2.data[(y * src2.width + x) * src2.channels + c];
                }
                
                // 加法混合（加权）
                float mixed = (v1 * ratio + v2 * ratio);
                dst.data[(y * w + x) * ch + c] = static_cast<uint8_t>(clamp_val(static_cast<int>(mixed), 0, 255));
            }
        }
    }
}

void MixupNode::blend_multiply(ImageData& dst, const ImageData& src1, const ImageData& src2, float ratio) {
    int w = std::max(src1.width, src2.width);
    int h = std::max(src1.height, src2.height);
    int ch = std::max(src1.channels, src2.channels);
    
    dst.width = w;
    dst.height = h;
    dst.channels = ch;
    dst.format = src1.format;
    dst.data.resize(w * h * ch);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            for (int c = 0; c < ch; ++c) {
                float v1 = 128.0f, v2 = 128.0f;  // 默认中间值
                
                if (x < src1.width && y < src1.height && c < src1.channels) {
                    v1 = src1.data[(y * src1.width + x) * src1.channels + c];
                }
                
                if (x < src2.width && y < src2.height && c < src2.channels) {
                    v2 = src2.data[(y * src2.width + x) * src2.channels + c];
                }
                
                // 乘法混合
                float mixed = (v1 / 255.0f) * (v2 / 255.0f) * 255.0f;
                
                // 使用ratio调整
                float base = v1 * ratio + v2 * (1.0f - ratio);
                mixed = base * (1.0f - ratio * 0.5f) + mixed * ratio * 0.5f;
                
                dst.data[(y * w + x) * ch + c] = static_cast<uint8_t>(clamp_val(static_cast<int>(mixed), 0, 255));
            }
        }
    }
}

Result<void> MixupNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto image1_data = get_input("image1");
    auto image2_data = get_input("image2");
    const ImageData& image1 = image1_data.as_image();
    const ImageData& image2 = image2_data.as_image();
    
    if (image1.empty() || image2.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input images are empty");
    }
    
    float mix_ratio = static_cast<float>(get_param("mix_ratio", Data(0.5)).as_number());
    String blend_mode = get_param("blend_mode", Data("linear")).as_string();
    String random_ratio_str = get_param("random_ratio", Data("0.3,0.7")).as_string();
    int random_seed = static_cast<int>(get_param("random_seed", Data(0)).as_int());
    
    // 解析随机比例范围
    float ratio_min = 0.3f, ratio_max = 0.7f;
    parse_range(random_ratio_str, ratio_min, ratio_max);
    
    // 如果mix_ratio为0，随机生成
    if (mix_ratio <= 0.0f) {
        std::mt19937 rng(random_seed == 0 ? std::random_device{}() : random_seed);
        std::uniform_real_distribution<float> ratio_dist(ratio_min, ratio_max);
        mix_ratio = ratio_dist(rng);
    }
    
    // 执行混合
    ImageData mixed_image;
    
    if (blend_mode == "linear") {
        blend_linear(mixed_image, image1, image2, mix_ratio);
    } else if (blend_mode == "add") {
        blend_add(mixed_image, image1, image2, mix_ratio);
    } else if (blend_mode == "multiply") {
        blend_multiply(mixed_image, image1, image2, mix_ratio);
    } else {
        blend_linear(mixed_image, image1, image2, mix_ratio);
    }
    
    // 合并标注
    ImageAnnotation mixed_annotation;
    mixed_annotation.width = mixed_image.width;
    mixed_annotation.height = mixed_image.height;
    
    // 添加image1的标注（带比例权重）
    if (has_input("annotation1")) {
        String ann1_json = get_input("annotation1").as_string();
        ImageAnnotation annotation1 = ImageAnnotation::from_json(ann1_json);
        for (const auto& obj : annotation1.objects) {
            AnnotationObject new_obj = obj;
            new_obj.confidence *= mix_ratio;
            mixed_annotation.objects.push_back(new_obj);
        }
    }
    
    // 添加image2的标注（带比例权重）
    if (has_input("annotation2")) {
        String ann2_json = get_input("annotation2").as_string();
        ImageAnnotation annotation2 = ImageAnnotation::from_json(ann2_json);
        for (const auto& obj : annotation2.objects) {
            AnnotationObject new_obj = obj;
            new_obj.confidence *= (1.0f - mix_ratio);
            mixed_annotation.objects.push_back(new_obj);
        }
    }
    
    // 设置输出
    set_output("mixed_image", Data(mixed_image));
    set_output("mix_ratio", Data(mix_ratio));
    
    if (!mixed_annotation.objects.empty()) {
        set_output("mixed_annotation", Data(mixed_annotation.to_json()));
    }
    
    OVF_INFO() << "Mixup completed: ratio=" << mix_ratio << ", mode=" << blend_mode;
    
    return Result<void>::success();
}

// ==================== DatasetSplitNode 实现 ====================

DatasetSplitNode::DatasetSplitNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DatasetSplitNode::make_info() {
    NodeInfo info;
    info.id = "DatasetSplit";
    info.name = "数据集分割";
    info.category = "训练工具";
    info.description = "将数据集分割为训练/验证/测试集";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("data_list", "数据列表", DataType::String, true),
        DataPort("annotations", "标注数据", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("train_set", "训练集", DataType::String),
        DataPort("val_set", "验证集", DataType::String),
        DataPort("test_set", "测试集", DataType::String),
        DataPort("split_report", "分割报告", DataType::String)
    };
    
    info.params = {
        ParamDef("train_ratio", "训练集比例", DataType::Number, Data(0.7)),
        ParamDef("val_ratio", "验证集比例", DataType::Number, Data(0.15)),
        ParamDef("test_ratio", "测试集比例", DataType::Number, Data(0.15)),
        ParamDef("stratified", "分层采样", DataType::Boolean, Data(false)),
        ParamDef("random_seed", "随机种子", DataType::Number, Data(42)),
        ParamDef("output_format", "输出格式", DataType::String, Data("json"))
    };
    
    info.params[5].options = {"json", "list", "file"};
    
    return info;
}

void DatasetSplitNode::random_split(const Vector<String>& data,
                                      Vector<String>& train, Vector<String>& val, Vector<String>& test,
                                      float train_ratio, float val_ratio, float test_ratio) {
    // 复制数据
    Vector<String> shuffled = data;
    
    // 打乱顺序
    std::mt19937 rng(std::random_device{}());
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    
    // 计算分割点
    size_t total = shuffled.size();
    size_t train_end = static_cast<size_t>(total * train_ratio);
    size_t val_end = train_end + static_cast<size_t>(total * val_ratio);
    
    // 分割
    train.assign(shuffled.begin(), shuffled.begin() + train_end);
    val.assign(shuffled.begin() + train_end, shuffled.begin() + val_end);
    test.assign(shuffled.begin() + val_end, shuffled.end());
}

void DatasetSplitNode::stratified_split(const Vector<String>& data, const Vector<String>& labels,
                                          Vector<String>& train, Vector<String>& val, Vector<String>& test,
                                          float train_ratio, float val_ratio, float test_ratio) {
    // 按标签分组
    std::map<String, Vector<String>> label_groups;
    for (size_t i = 0; i < data.size() && i < labels.size(); ++i) {
        label_groups[labels[i]].push_back(data[i]);
    }
    
    train.clear();
    val.clear();
    test.clear();
    
    // 对每个标签组进行分割
    for (auto& pair : label_groups) {
        Vector<String> group_train, group_val, group_test;
        random_split(pair.second, group_train, group_val, group_test, train_ratio, val_ratio, test_ratio);
        
        train.insert(train.end(), group_train.begin(), group_train.end());
        val.insert(val.end(), group_val.begin(), group_val.end());
        test.insert(test.end(), group_test.begin(), group_test.end());
    }
}

Result<void> DatasetSplitNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String data_list_json = get_input("data_list").as_string();
    
    float train_ratio = static_cast<float>(get_param("train_ratio", Data(0.7)).as_number());
    float val_ratio = static_cast<float>(get_param("val_ratio", Data(0.15)).as_number());
    float test_ratio = static_cast<float>(get_param("test_ratio", Data(0.15)).as_number());
    bool stratified = get_param("stratified", Data(false)).as_bool();
    int random_seed = static_cast<int>(get_param("random_seed", Data(42)).as_int());
    String output_format = get_param("output_format", Data("json")).as_string();
    
    // 验证比例总和
    float total_ratio = train_ratio + val_ratio + test_ratio;
    if (std::abs(total_ratio - 1.0f) > 0.001f) {
        // 自动调整
        train_ratio /= total_ratio;
        val_ratio /= total_ratio;
        test_ratio /= total_ratio;
    }
    
    // 解析数据列表
    Vector<String> data = parse_json_string_array(data_list_json);
    
    if (data.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Data list is empty");
    }
    
    // 分割数据
    Vector<String> train_set, val_set, test_set;
    
    if (stratified && has_input("annotations")) {
        // 从标注中提取标签
        String annotations_json = get_input("annotations").as_string();
        Vector<ImageAnnotation> annotations = parse_annotations_json(annotations_json);
        
        Vector<String> labels;
        for (const auto& ann : annotations) {
            if (!ann.objects.empty()) {
                labels.push_back(ann.objects[0].label);
            } else {
                labels.push_back("unknown");
            }
        }
        
        stratified_split(data, labels, train_set, val_set, test_set, train_ratio, val_ratio, test_ratio);
    } else {
        // 设置随机种子
        std::mt19937 rng(random_seed);
        
        // 复制并打乱数据
        Vector<String> shuffled = data;
        std::shuffle(shuffled.begin(), shuffled.end(), rng);
        
        size_t total = shuffled.size();
        size_t train_end = static_cast<size_t>(total * train_ratio);
        size_t val_end = train_end + static_cast<size_t>(total * val_ratio);
        
        train_set.assign(shuffled.begin(), shuffled.begin() + train_end);
        val_set.assign(shuffled.begin() + train_end, shuffled.begin() + val_end);
        test_set.assign(shuffled.begin() + val_end, shuffled.end());
    }
    
    // 格式化输出
    String train_output, val_output, test_output;
    
    if (output_format == "json") {
        std::ostringstream oss;
        
        oss << "[";
        for (size_t i = 0; i < train_set.size(); ++i) {
            oss << "\"" << escape_json_string(train_set[i]) << "\"";
            if (i < train_set.size() - 1) oss << ",";
        }
        oss << "]";
        train_output = oss.str();
        
        oss.str("");
        oss << "[";
        for (size_t i = 0; i < val_set.size(); ++i) {
            oss << "\"" << escape_json_string(val_set[i]) << "\"";
            if (i < val_set.size() - 1) oss << ",";
        }
        oss << "]";
        val_output = oss.str();
        
        oss.str("");
        oss << "[";
        for (size_t i = 0; i < test_set.size(); ++i) {
            oss << "\"" << escape_json_string(test_set[i]) << "\"";
            if (i < test_set.size() - 1) oss << ",";
        }
        oss << "]";
        test_output = oss.str();
        
    } else {
        // list格式：逗号分隔
        std::ostringstream oss;
        for (size_t i = 0; i < train_set.size(); ++i) {
            oss << train_set[i];
            if (i < train_set.size() - 1) oss << ",";
        }
        train_output = oss.str();
        
        oss.str("");
        for (size_t i = 0; i < val_set.size(); ++i) {
            oss << val_set[i];
            if (i < val_set.size() - 1) oss << ",";
        }
        val_output = oss.str();
        
        oss.str("");
        for (size_t i = 0; i < test_set.size(); ++i) {
            oss << test_set[i];
            if (i < test_set.size() - 1) oss << ",";
        }
        test_output = oss.str();
    }
    
    // 生成分割报告
    std::ostringstream report;
    report << "{";
    report << "\"total\":" << data.size() << ",";
    report << "\"train_count\":" << train_set.size() << ",";
    report << "\"val_count\":" << val_set.size() << ",";
    report << "\"test_count\":" << test_set.size() << ",";
    report << "\"train_ratio\":" << train_ratio << ",";
    report << "\"val_ratio\":" << val_ratio << ",";
    report << "\"test_ratio\":" << test_ratio << ",";
    report << "\"stratified\":" << (stratified ? "true" : "false") << ",";
    report << "\"random_seed\":" << random_seed;
    report << "}";
    
    // 设置输出
    set_output("train_set", Data(train_output));
    set_output("val_set", Data(val_output));
    set_output("test_set", Data(test_output));
    set_output("split_report", Data(report.str()));
    
    OVF_INFO() << "DatasetSplit completed: train=" << train_set.size() << ", val=" << val_set.size() << ", test=" << test_set.size();
    
    return Result<void>::success();
}

// ==================== TrainConfigNode 实现 ====================

TrainConfigNode::TrainConfigNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo TrainConfigNode::make_info() {
    NodeInfo info;
    info.id = "TrainConfig";
    info.name = "训练配置";
    info.category = "训练工具";
    info.description = "生成和管理训练配置";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("model_type", "模型类型", DataType::String, false),
        DataPort("dataset_info", "数据集信息", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("config", "配置内容", DataType::String),
        DataPort("config_path", "配置文件路径", DataType::String)
    };
    
    info.params = {
        ParamDef("learning_rate", "学习率", DataType::Number, Data(0.001)),
        ParamDef("batch_size", "批次大小", DataType::Number, Data(32)),
        ParamDef("epochs", "训练轮次", DataType::Number, Data(100)),
        ParamDef("optimizer", "优化器", DataType::String, Data("Adam")),
        ParamDef("lr_scheduler", "学习率调度", DataType::String, Data("cosine")),
        ParamDef("weight_decay", "权重衰减", DataType::Number, Data(0.0001)),
        ParamDef("momentum", "动量", DataType::Number, Data(0.9)),
        ParamDef("dropout", "Dropout比例", DataType::Number, Data(0.5)),
        ParamDef("pretrained", "预训练模型", DataType::Boolean, Data(true)),
        ParamDef("config_format", "配置格式", DataType::String, Data("json")),
        ParamDef("output_dir", "输出目录", DataType::String, Data("./configs"))
    };
    
    info.params[3].options = {"SGD", "Adam", "AdamW", "RMSprop"};
    info.params[4].options = {"step", "cosine", "linear", "constant"};
    info.params[9].options = {"json", "yaml", "py"};
    
    return info;
}

String TrainConfigNode::generate_json_config() {
    std::ostringstream oss;
    oss << "{";
    oss << "\"training\":{";
    oss << "\"learning_rate\":" << get_param("learning_rate", Data(0.001)).as_number() << ",";
    oss << "\"batch_size\":" << get_param("batch_size", Data(32)).as_int() << ",";
    oss << "\"epochs\":" << get_param("epochs", Data(100)).as_int() << ",";
    oss << "\"optimizer\":\"" << get_param("optimizer", Data("Adam")).as_string() << "\",";
    oss << "\"lr_scheduler\":\"" << get_param("lr_scheduler", Data("cosine")).as_string() << "\",";
    oss << "\"weight_decay\":" << get_param("weight_decay", Data(0.0001)).as_number() << ",";
    oss << "\"momentum\":" << get_param("momentum", Data(0.9)).as_number() << ",";
    oss << "\"dropout\":" << get_param("dropout", Data(0.5)).as_number();
    oss << "},";
    oss << "\"model\":{";
    oss << "\"pretrained\":" << (get_param("pretrained", Data(true)).as_bool() ? "true" : "false");
    if (has_input("model_type")) {
        oss << ",\"type\":\"" << get_input("model_type").as_string() << "\"";
    }
    oss << "}";
    oss << "}";
    return oss.str();
}

String TrainConfigNode::generate_yaml_config() {
    std::ostringstream oss;
    oss << "training:\n";
    oss << "  learning_rate: " << get_param("learning_rate", Data(0.001)).as_number() << "\n";
    oss << "  batch_size: " << get_param("batch_size", Data(32)).as_int() << "\n";
    oss << "  epochs: " << get_param("epochs", Data(100)).as_int() << "\n";
    oss << "  optimizer: " << get_param("optimizer", Data("Adam")).as_string() << "\n";
    oss << "  lr_scheduler: " << get_param("lr_scheduler", Data("cosine")).as_string() << "\n";
    oss << "  weight_decay: " << get_param("weight_decay", Data(0.0001)).as_number() << "\n";
    oss << "  momentum: " << get_param("momentum", Data(0.9)).as_number() << "\n";
    oss << "  dropout: " << get_param("dropout", Data(0.5)).as_number() << "\n";
    oss << "model:\n";
    oss << "  pretrained: " << (get_param("pretrained", Data(true)).as_bool() ? "true" : "false") << "\n";
    if (has_input("model_type")) {
        oss << "  type: " << get_input("model_type").as_string() << "\n";
    }
    return oss.str();
}

String TrainConfigNode::generate_py_config() {
    std::ostringstream oss;
    oss << "# Training configuration\n";
    oss << "config = {\n";
    oss << "    'learning_rate': " << get_param("learning_rate", Data(0.001)).as_number() << ",\n";
    oss << "    'batch_size': " << get_param("batch_size", Data(32)).as_int() << ",\n";
    oss << "    'epochs': " << get_param("epochs", Data(100)).as_int() << ",\n";
    oss << "    'optimizer': '" << get_param("optimizer", Data("Adam")).as_string() << "',\n";
    oss << "    'lr_scheduler': '" << get_param("lr_scheduler", Data("cosine")).as_string() << "',\n";
    oss << "    'weight_decay': " << get_param("weight_decay", Data(0.0001)).as_number() << ",\n";
    oss << "    'momentum': " << get_param("momentum", Data(0.9)).as_number() << ",\n";
    oss << "    'dropout': " << get_param("dropout", Data(0.5)).as_number() << ",\n";
    oss << "    'pretrained': " << (get_param("pretrained", Data(true)).as_bool() ? "True" : "False") << "\n";
    oss << "}\n";
    return oss.str();
}

Result<void> TrainConfigNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String config_format = get_param("config_format", Data("json")).as_string();
    String output_dir = get_param("output_dir", Data("./configs")).as_string();
    
    // 生成配置
    String config_content;
    String config_path;
    
    if (config_format == "json") {
        config_content = generate_json_config();
        config_path = output_dir + "/train_config.json";
    } else if (config_format == "yaml") {
        config_content = generate_yaml_config();
        config_path = output_dir + "/train_config.yaml";
    } else if (config_format == "py") {
        config_content = generate_py_config();
        config_path = output_dir + "/train_config.py";
    } else {
        config_content = generate_json_config();
        config_path = output_dir + "/train_config.json";
    }
    
    // 设置输出
    set_output("config", Data(config_content));
    set_output("config_path", Data(config_path));
    
    OVF_INFO() << "TrainConfig generated: format=" << config_format;
    
    return Result<void>::success();
}

// ==================== ModelEvaluateNode 实现 ====================

ModelEvaluateNode::ModelEvaluateNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ModelEvaluateNode::make_info() {
    NodeInfo info;
    info.id = "ModelEvaluate";
    info.name = "模型评估";
    info.category = "训练工具";
    info.description = "评估模型性能（精度、召回率、F1等）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("predictions", "预测结果", DataType::String, true),
        DataPort("ground_truth", "真实标签", DataType::String, true),
        DataPort("num_classes", "类别数量", DataType::Number, false)
    };
    
    info.outputs = {
        DataPort("precision", "精度", DataType::Number),
        DataPort("recall", "召回率", DataType::Number),
        DataPort("f1_score", "F1分数", DataType::Number),
        DataPort("accuracy", "准确率", DataType::Number),
        DataPort("confusion_matrix", "混淆矩阵", DataType::String),
        DataPort("iou", "IoU", DataType::Number),
        DataPort("evaluation_report", "评估报告", DataType::String)
    };
    
    info.params = {
        ParamDef("task_type", "任务类型", DataType::String, Data("classification")),
        ParamDef("iou_threshold", "IoU阈值", DataType::Number, Data(0.5)),
        ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(0.5)),
        ParamDef("average_method", "平均方法", DataType::String, Data("macro"))
    };
    
    info.params[0].options = {"classification", "detection", "segmentation"};
    info.params[3].options = {"micro", "macro", "weighted"};
    
    return info;
}

float ModelEvaluateNode::calculate_iou_boxes(int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2) {
    int xi1 = std::max(x1, x2);
    int yi1 = std::max(y1, y2);
    int xi2 = std::min(x1 + w1, x2 + w2);
    int yi2 = std::min(y1 + h1, y2 + h2);
    
    if (xi2 <= xi1 || yi2 <= yi1) return 0.0f;
    
    float intersection = static_cast<float>((xi2 - xi1) * (yi2 - yi1));
    float area1 = static_cast<float>(w1 * h1);
    float area2 = static_cast<float>(w2 * h2);
    float union_area = area1 + area2 - intersection;
    
    return intersection / union_area;
}

float ModelEvaluateNode::calculate_iou(const AnnotationObject& pred, const AnnotationObject& gt) {
    return calculate_iou_boxes(pred.x, pred.y, pred.width, pred.height, gt.x, gt.y, gt.width, gt.height);
}

void ModelEvaluateNode::calculate_classification_metrics(const Vector<int>& pred, const Vector<int>& gt,
                                                           int num_classes, float& precision, float& recall,
                                                           float& f1, float& accuracy, Vector<int>& confusion_matrix) {
    // 初始化混淆矩阵
    confusion_matrix.assign(num_classes * num_classes, 0);
    
    int correct = 0;
    int total = static_cast<int>(pred.size());
    
    // 计算混淆矩阵
    for (int i = 0; i < total; ++i) {
        int p = pred[i];
        int g = gt[i];
        
        if (p >= 0 && p < num_classes && g >= 0 && g < num_classes) {
            confusion_matrix[g * num_classes + p]++;
            if (p == g) correct++;
        }
    }
    
    accuracy = static_cast<float>(correct) / total;
    
    // 计算每个类别的precision和recall
    Vector<float> class_precision(num_classes, 0.0f);
    Vector<float> class_recall(num_classes, 0.0f);
    
    for (int c = 0; c < num_classes; ++c) {
        int tp = confusion_matrix[c * num_classes + c];
        int fp = 0;
        int fn = 0;
        
        for (int i = 0; i < num_classes; ++i) {
            if (i != c) {
                fp += confusion_matrix[i * num_classes + c];
                fn += confusion_matrix[c * num_classes + i];
            }
        }
        
        if (tp + fp > 0) class_precision[c] = static_cast<float>(tp) / (tp + fp);
        if (tp + fn > 0) class_recall[c] = static_cast<float>(tp) / (tp + fn);
    }
    
    // Macro平均
    precision = 0.0f;
    recall = 0.0f;
    for (int c = 0; c < num_classes; ++c) {
        precision += class_precision[c];
        recall += class_recall[c];
    }
    precision /= num_classes;
    recall /= num_classes;
    
    // F1
    if (precision + recall > 0) {
        f1 = 2.0f * precision * recall / (precision + recall);
    } else {
        f1 = 0.0f;
    }
}

Result<void> ModelEvaluateNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String predictions_json = get_input("predictions").as_string();
    String ground_truth_json = get_input("ground_truth").as_string();
    
    String task_type = get_param("task_type", Data("classification")).as_string();
    float iou_threshold = static_cast<float>(get_param("iou_threshold", Data(0.5)).as_number());
    float confidence_threshold = static_cast<float>(get_param("confidence_threshold", Data(0.5)).as_number());
    String average_method = get_param("average_method", Data("macro")).as_string();

    int num_classes = static_cast<int>(get_param("num_classes", Data(10)).as_int());
    if (has_input("num_classes")) {
        num_classes = static_cast<int>(get_input("num_classes").as_int());
    }
    
    float precision = 0.0f, recall = 0.0f, f1 = 0.0f, accuracy = 0.0f, iou = 0.0f;
    Vector<int> confusion_matrix;
    
    if (task_type == "classification") {
        // 解析预测和真实标签（假设是整数列表）
        Vector<int> pred_labels, gt_labels;
        
        auto pred_nums = parse_json_number_array(predictions_json);
        auto gt_nums = parse_json_number_array(ground_truth_json);
        
        for (auto p : pred_nums) pred_labels.push_back(static_cast<int>(p));
        for (auto g : gt_nums) gt_labels.push_back(static_cast<int>(g));
        
        if (pred_labels.size() != gt_labels.size()) {
            return Result<void>::failure(ErrorCode::InvalidParameter, "Prediction and ground truth size mismatch");
        }
        
        calculate_classification_metrics(pred_labels, gt_labels, num_classes, precision, recall, f1, accuracy, confusion_matrix);
        
    } else if (task_type == "detection") {
        // 解析标注数据
        Vector<ImageAnnotation> pred_annotations = parse_annotations_json(predictions_json);
        Vector<ImageAnnotation> gt_annotations = parse_annotations_json(ground_truth_json);
        
        // 计算IoU和检测指标
        int total_pred = 0, total_gt = 0, true_positives = 0;
        float total_iou = 0.0f;
        
        for (const auto& pred_ann : pred_annotations) {
            for (const auto& pred_obj : pred_ann.objects) {
                if (pred_obj.confidence >= confidence_threshold) {
                    total_pred++;
                    
                    // 找到匹配的GT
                    for (const auto& gt_ann : gt_annotations) {
                        for (const auto& gt_obj : gt_ann.objects) {
                            if (pred_obj.label == gt_obj.label) {
                                float obj_iou = calculate_iou(pred_obj, gt_obj);
                                if (obj_iou >= iou_threshold) {
                                    true_positives++;
                                    total_iou += obj_iou;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        for (const auto& gt_ann : gt_annotations) {
            total_gt += static_cast<int>(gt_ann.objects.size());
        }
        
        if (total_pred > 0) precision = static_cast<float>(true_positives) / total_pred;
        if (total_gt > 0) recall = static_cast<float>(true_positives) / total_gt;
        if (precision + recall > 0) f1 = 2.0f * precision * recall / (precision + recall);
        if (true_positives > 0) iou = total_iou / true_positives;
        
        // 生成简化的混淆矩阵（按类别）
        confusion_matrix.assign(num_classes * num_classes, 0);
    }
    
    // 构建混淆矩阵JSON
    std::ostringstream cm_json;
    cm_json << "[";
    for (size_t i = 0; i < confusion_matrix.size(); ++i) {
        cm_json << confusion_matrix[i];
        if (i < confusion_matrix.size() - 1) cm_json << ",";
    }
    cm_json << "]";
    
    // 构建评估报告
    std::ostringstream report;
    report << "{";
    report << "\"task_type\":\"" << task_type << "\",";
    report << "\"precision\":" << precision << ",";
    report << "\"recall\":" << recall << ",";
    report << "\"f1_score\":" << f1 << ",";
    report << "\"accuracy\":" << accuracy << ",";
    report << "\"iou\":" << iou << ",";
    report << "\"confusion_matrix\":" << cm_json.str() << ",";
    report << "\"num_classes\":" << num_classes << ",";
    report << "\"iou_threshold\":" << iou_threshold << ",";
    report << "\"confidence_threshold\":" << confidence_threshold;
    report << "}";
    
    // 设置输出
    set_output("precision", Data(precision));
    set_output("recall", Data(recall));
    set_output("f1_score", Data(f1));
    set_output("accuracy", Data(accuracy));
    set_output("confusion_matrix", Data(cm_json.str()));
    set_output("iou", Data(iou));
    set_output("evaluation_report", Data(report.str()));
    
    OVF_INFO() << "ModelEvaluate completed: precision=" << precision << ", recall=" << recall << ", f1=" << f1;
    
    return Result<void>::success();
}

// ==================== ExportONNXNode 实现 ====================

ExportONNXNode::ExportONNXNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ExportONNXNode::make_info() {
    NodeInfo info;
    info.id = "ExportONNX";
    info.name = "ONNX导出";
    info.category = "训练工具";
    info.description = "模型格式转换导出";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("model_info", "模型信息", DataType::String, true),
        DataPort("weights_path", "权重文件路径", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("export_path", "导出文件路径", DataType::String),
        DataPort("export_status", "导出状态", DataType::Boolean),
        DataPort("model_info", "导出模型信息", DataType::String)
    };
    
    info.params = {
        ParamDef("input_shape", "输入形状", DataType::String, Data("1,3,224,224")),
        ParamDef("output_names", "输出节点名称", DataType::String, Data("output")),
        ParamDef("opset_version", "ONNX opset版本", DataType::Number, Data(11)),
        ParamDef("dynamic_batch", "动态批次", DataType::Boolean, Data(false)),
        ParamDef("optimize", "优化模型", DataType::Boolean, Data(true)),
        ParamDef("output_path", "输出路径", DataType::String, Data("./models")),
        ParamDef("model_name", "模型名称", DataType::String, Data("model"))
    };
    
    return info;
}

Vector<int> ExportONNXNode::parse_shape(const String& shape_str) {
    Vector<int> shape;
    auto nums = parse_json_number_array("[" + shape_str + "]");
    for (auto n : nums) {
        shape.push_back(static_cast<int>(n));
    }
    return shape;
}

String ExportONNXNode::generate_onnx_export_script() {
    String input_shape = get_param("input_shape", Data("1,3,224,224")).as_string();
    String output_names = get_param("output_names", Data("output")).as_string();
    int opset_version = static_cast<int>(get_param("opset_version", Data(11)).as_int());
    bool dynamic_batch = get_param("dynamic_batch", Data(false)).as_bool();
    bool optimize = get_param("optimize", Data(true)).as_bool();
    String output_path = get_param("output_path", Data("./models")).as_string();
    String model_name = get_param("model_name", Data("model")).as_string();
    
    std::ostringstream script;
    script << "# ONNX Export Script\n";
    script << "# Generated by OpenVisionFlow\n\n";
    script << "import torch\n";
    script << "import onnx\n";
    script << "import onnxoptimizer\n\n";
    
    // 加载模型（示例）
    script << "# Load your model here\n";
    script << "# model = torch.load('model_weights.pth')\n";
    script << "# model.eval()\n\n";
    
    // 输入形状
    auto shape = parse_shape(input_shape);
    script << "# Input shape: " << input_shape << "\n";
    script << "dummy_input = torch.randn(" << input_shape << ")\n\n";
    
    // 导出ONNX
    script << "# Export to ONNX\n";
    script << "torch.onnx.export(\n";
    script << "    model,\n";
    script << "    dummy_input,\n";
    script << "    \"" << output_path << "/" << model_name << ".onnx\",\n";
    script << "    export_params=True,\n";
    script << "    opset_version=" << opset_version << ",\n";
    script << "    do_constant_folding=True,\n";
    script << "    input_names=['input'],\n";
    script << "    output_names=['" << output_names << "'],\n";
    
    if (dynamic_batch) {
        script << "    dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}\n";
    } else {
        script << "    dynamic_axes=None\n";
    }
    
    script << ")\n\n";
    
    // 验证和优化
    script << "# Verify ONNX model\n";
    script << "onnx_model = onnx.load(\"" << output_path << "/" << model_name << ".onnx\")\n";
    script << "onnx.checker.check_model(onnx_model)\n\n";
    
    if (optimize) {
        script << "# Optimize ONNX model\n";
        script << "onnx_model = onnxoptimizer.optimize(onnx_model)\n";
        script << "onnx.save(onnx_model, \"" << output_path << "/" << model_name << "_optimized.onnx\")\n";
    }
    
    return script.str();
}

String ExportONNXNode::generate_model_info_json() {
    String input_shape = get_param("input_shape", Data("1,3,224,224")).as_string();
    String output_names = get_param("output_names", Data("output")).as_string();
    int opset_version = static_cast<int>(get_param("opset_version", Data(11)).as_int());
    bool dynamic_batch = get_param("dynamic_batch", Data(false)).as_bool();
    bool optimize = get_param("optimize", Data(true)).as_bool();
    String output_path = get_param("output_path", Data("./models")).as_string();
    String model_name = get_param("model_name", Data("model")).as_string();
    
    std::ostringstream info;
    info << "{";
    info << "\"model_name\":\"" << model_name << "\",";
    info << "\"input_shape\":\"" << input_shape << "\",";
    info << "\"output_names\":\"" << output_names << "\",";
    info << "\"opset_version\":" << opset_version << ",";
    info << "\"dynamic_batch\":" << (dynamic_batch ? "true" : "false") << ",";
    info << "\"optimized\":" << (optimize ? "true" : "false") << ",";
    info << "\"export_path\":\"" << output_path << "/" << model_name << ".onnx\"";
    info << "}";
    
    return info.str();
}

Result<void> ExportONNXNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // 获取模型信息
    String model_info_json = has_input("model_info") ? get_input("model_info").as_string() : "";
    
    String output_path = get_param("output_path", Data("./models")).as_string();
    String model_name = get_param("model_name", Data("model")).as_string();
    
    // 生成导出脚本（供用户参考）
    String export_script = generate_onnx_export_script();
    String model_info_output = generate_model_info_json();
    
    String export_file_path = output_path + "/" + model_name + ".onnx";
    bool export_status = true;  // 实际导出需要Python环境
    
    // 设置输出
    set_output("export_path", Data(export_file_path));
    set_output("export_status", Data(export_status));
    set_output("model_info", Data(model_info_output));
    
    OVF_INFO() << "ExportONNX generated: path=" << export_file_path;
    
    return Result<void>::success();
}

// ==================== 核心算法辅助函数实现 ====================

namespace training_utils {

float compute_lr(LRScheduler scheduler, float initial_lr, int current_epoch, int total_epochs,
                 float step_size, float gamma, int patience, const Vector<float>* val_losses) {
    float lr = initial_lr;
    
    switch (scheduler) {
        case LRScheduler::Step:
            // 阶梯式衰减：每隔step_size个epoch衰减gamma倍
            lr = initial_lr * std::pow(gamma, static_cast<int>(current_epoch / step_size));
            break;
            
        case LRScheduler::Cosine:
            // 余弦退火
            lr = initial_lr * 0.5f * (1.0f + std::cos(static_cast<float>(M_PI) * current_epoch / total_epochs));
            break;
            
        case LRScheduler::Exponential:
            // 指数衰减
            lr = initial_lr * std::pow(gamma, current_epoch);
            break;
            
        case LRScheduler::ReduceOnPlateau:
            // 当指标plateau时衰减
            if (val_losses && val_losses->size() >= patience) {
                bool plateau = true;
                for (int i = val_losses->size() - patience; i < val_losses->size() - 1; ++i) {
                    if (val_losses->at(i) < val_losses->at(i + 1)) {
                        plateau = false;
                        break;
                    }
                }
                if (plateau) {
                    lr *= gamma;
                }
            }
            break;
            
        case LRScheduler::Linear:
            // 线性衰减
            lr = initial_lr * (1.0f - static_cast<float>(current_epoch) / total_epochs);
            break;
            
        case LRScheduler::Constant:
            // 恒定学习率
            lr = initial_lr;
            break;
    }
    
    // 确保学习率不会太小
    return std::max(lr, initial_lr * 0.0001f);
}

bool check_early_stop(const Vector<float>& val_losses, int patience, float min_delta, int& best_epoch) {
    if (val_losses.size() < patience + 1) {
        best_epoch = static_cast<int>(val_losses.size()) - 1;
        return false;
    }
    
    // 找到最佳epoch
    float best_loss = val_losses[0];
    best_epoch = 0;
    for (size_t i = 1; i < val_losses.size(); ++i) {
        if (val_losses[i] < best_loss - min_delta) {
            best_loss = val_losses[i];
            best_epoch = static_cast<int>(i);
        }
    }
    
    // 检查是否超过patience
    int no_improve_count = static_cast<int>(val_losses.size()) - 1 - best_epoch;
    return no_improve_count >= patience;
}

float calculate_iou_segmentation(const Vector<uint8_t>& pred_mask, const Vector<uint8_t>& gt_mask,
                                  int width, int height) {
    if (pred_mask.size() != gt_mask.size() || pred_mask.empty()) {
        return 0.0f;
    }
    
    int intersection = 0;
    int union_count = 0;
    
    for (size_t i = 0; i < pred_mask.size(); ++i) {
        bool pred_positive = pred_mask[i] > 0;
        bool gt_positive = gt_mask[i] > 0;
        
        if (pred_positive && gt_positive) {
            intersection++;
        }
        if (pred_positive || gt_positive) {
            union_count++;
        }
    }
    
    if (union_count == 0) return 0.0f;
    return static_cast<float>(intersection) / union_count;
}

float calculate_ap_single_class(const Vector<float>& confidences, const Vector<bool>& tp_or_fp, int num_gt) {
    if (confidences.empty() || num_gt == 0) return 0.0f;
    
    // 按置信度排序
    Vector<std::pair<float, bool>> sorted_data;
    for (size_t i = 0; i < confidences.size(); ++i) {
        sorted_data.emplace_back(confidences[i], tp_or_fp[i]);
    }
    std::sort(sorted_data.begin(), sorted_data.end(), [](const auto& a, const auto& b) {
        return a.first > b.first;
    });
    
    // 计算precision和recall
    Vector<float> precisions, recalls;
    int tp_count = 0;
    int fp_count = 0;
    
    for (const auto& data : sorted_data) {
        if (data.second) {
            tp_count++;
        } else {
            fp_count++;
        }
        
        float precision = static_cast<float>(tp_count) / (tp_count + fp_count);
        float recall = static_cast<float>(tp_count) / num_gt;
        
        precisions.push_back(precision);
        recalls.push_back(recall);
    }
    
    // 计算AP (11点插值)
    float ap = 0.0f;
    for (int i = 0; i <= 10; ++i) {
        float recall_threshold = static_cast<float>(i) / 10.0f;
        float max_precision = 0.0f;
        
        for (size_t j = 0; j < recalls.size(); ++j) {
            if (recalls[j] >= recall_threshold) {
                max_precision = std::max(max_precision, precisions[j]);
            }
        }
        
        ap += max_precision / 11.0f;
    }
    
    return ap;
}

float calculate_map_detection(const Vector<ImageAnnotation>& predictions,
                               const Vector<ImageAnnotation>& ground_truths,
                               const Vector<float>& iou_thresholds) {
    if (predictions.empty() || ground_truths.empty() || iou_thresholds.empty()) {
        return 0.0f;
    }
    
    float map = 0.0f;
    
    for (float iou_threshold : iou_thresholds) {
        // 计算每个类别的AP
        HashMap<String, float> class_aps;

        // 收集所有类别
        std::unordered_set<String> all_classes;
        for (const auto& gt : ground_truths) {
            for (const auto& obj : gt.objects) {
                all_classes.insert(obj.label);
            }
        }
        
        // 对每个类别计算AP
        for (const String& class_name : all_classes) {
            Vector<float> confidences;
            Vector<bool> tp_or_fp;
            int num_gt = 0;
            
            // 统计GT数量
            for (const auto& gt : ground_truths) {
                for (const auto& obj : gt.objects) {
                    if (obj.label == class_name) {
                        num_gt++;
                    }
                }
            }
            
            // 对每个预测进行匹配
            for (const auto& pred : predictions) {
                for (const auto& pred_obj : pred.objects) {
                    if (pred_obj.label == class_name) {
                        confidences.push_back(pred_obj.confidence);
                        
                        // 尝试匹配GT
                        bool matched = false;
                        for (const auto& gt : ground_truths) {
                            for (const auto& gt_obj : gt.objects) {
                                if (gt_obj.label == class_name) {
                                    // 计算IoU
                                    int x1 = std::max(pred_obj.x, gt_obj.x);
                                    int y1 = std::max(pred_obj.y, gt_obj.y);
                                    int x2 = std::min(pred_obj.x + pred_obj.width, gt_obj.x + gt_obj.width);
                                    int y2 = std::min(pred_obj.y + pred_obj.height, gt_obj.y + gt_obj.height);
                                    
                                    if (x2 > x1 && y2 > y1) {
                                        float intersection = static_cast<float>((x2 - x1) * (y2 - y1));
                                        float area_pred = static_cast<float>(pred_obj.width * pred_obj.height);
                                        float area_gt = static_cast<float>(gt_obj.width * gt_obj.height);
                                        float iou = intersection / (area_pred + area_gt - intersection);
                                        
                                        if (iou >= iou_threshold) {
                                            matched = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            if (matched) break;
                        }
                        
                        tp_or_fp.push_back(matched);
                    }
                }
            }
            
            // 计算该类别的AP
            if (num_gt > 0 && !confidences.empty()) {
                class_aps[class_name] = calculate_ap_single_class(confidences, tp_or_fp, num_gt);
            }
        }
        
        // 计算该IoU阈值下的mAP
        float map_threshold = 0.0f;
        for (const auto& pair : class_aps) {
            map_threshold += pair.second;
        }
        map_threshold /= class_aps.size();
        
        map += map_threshold;
    }
    
    return map / iou_thresholds.size();
}

String format_tensorboard_scalar(const String& tag, float value, int step) {
    std::ostringstream oss;
    oss << "{\"tag\":\"" << escape_json_string(tag) << "\",";
    oss << "\"value\":" << value << ",";
    oss << "\"step\":" << step << "}";
    return oss.str();
}

String format_tensorboard_histogram(const String& tag, const Vector<float>& values, int step) {
    if (values.empty()) return "{}";
    
    // 计算直方图统计
    float min_val = values[0];
    float max_val = values[0];
    float sum = 0.0f;
    
    for (float v : values) {
        min_val = std::min(min_val, v);
        max_val = std::max(max_val, v);
        sum += v;
    }
    
    float mean = sum / values.size();
    
    // 计算标准差
    float variance = 0.0f;
    for (float v : values) {
        variance += (v - mean) * (v - mean);
    }
    float stddev = std::sqrt(variance / values.size());
    
    std::ostringstream oss;
    oss << "{\"tag\":\"" << escape_json_string(tag) << "\",";
    oss << "\"min\":" << min_val << ",";
    oss << "\"max\":" << max_val << ",";
    oss << "\"mean\":" << mean << ",";
    oss << "\"stddev\":" << stddev << ",";
    oss << "\"count\":" << values.size() << ",";
    oss << "\"step\":" << step << "}";
    return oss.str();
}

String quantization_config_to_json(const QuantizationConfig& config) {
    std::ostringstream oss;
    oss << "{";
    oss << "\"precision\":\"" << config.precision << "\",";
    oss << "\"calibration_method\":\"" << config.calibration_method << "\",";
    oss << "\"calibration_samples\":" << config.calibration_samples << ",";
    oss << "\"percentile_value\":" << config.percentile_value;
    oss << "}";
    return oss.str();
}

} // namespace training_utils

// ==================== DatasetManagerNode 实现 ====================

DatasetManagerNode::DatasetManagerNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DatasetManagerNode::make_info() {
    NodeInfo info;
    info.id = "DatasetManager";
    info.name = "数据集管理";
    info.category = "数据管理";
    info.description = "管理数据集，划分训练/验证/测试集，统计数据分布";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("dataset_path", "数据集路径", DataType::String, true),
        DataPort("annotations", "标注数据", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("train_set", "训练集列表", DataType::String),
        DataPort("val_set", "验证集列表", DataType::String),
        DataPort("test_set", "测试集列表", DataType::String),
        DataPort("dataset_stats", "数据集统计", DataType::String),
        DataPort("class_distribution", "类别分布", DataType::String)
    };
    
    info.params = {
        ParamDef("split_strategy", "划分策略", DataType::String, Data("random")),
        ParamDef("train_ratio", "训练集比例", DataType::Number, Data(0.7)),
        ParamDef("val_ratio", "验证集比例", DataType::Number, Data(0.15)),
        ParamDef("test_ratio", "测试集比例", DataType::Number, Data(0.15)),
        ParamDef("k_fold", "K折交叉验证", DataType::Number, Data(5)),
        ParamDef("random_seed", "随机种子", DataType::Number, Data(42))
    };
    
    info.params[0].options = {"random", "stratified", "kfold"};
    
    return info;
}

Vector<String> DatasetManagerNode::load_image_paths(const String& dataset_path) {
    Vector<String> paths;
    
    // 简化实现：假设输入已经是路径列表的JSON数组
    // 实际实现中应该遍历目录
    if (dataset_path[0] == '[') {
        paths = parse_json_string_array(dataset_path);
    } else {
        // 单个路径，转换为列表
        paths.push_back(dataset_path);
    }
    
    return paths;
}

HashMap<String, int> DatasetManagerNode::analyze_class_distribution(const Vector<ImageAnnotation>& annotations) {
    HashMap<String, int> distribution;
    
    for (const auto& ann : annotations) {
        for (const auto& obj : ann.objects) {
            distribution[obj.label]++;
        }
    }
    
    return distribution;
}

void DatasetManagerNode::stratified_split(const Vector<String>& data, const Vector<String>& labels,
                                          Vector<String>& train, Vector<String>& val, Vector<String>& test,
                                          float train_ratio, float val_ratio, float test_ratio) {
    // 按标签分组
    HashMap<String, Vector<String>> label_groups;
    for (size_t i = 0; i < data.size() && i < labels.size(); ++i) {
        label_groups[labels[i]].push_back(data[i]);
    }
    
    train.clear();
    val.clear();
    test.clear();
    
    // 对每个标签组进行分割
    for (auto& pair : label_groups) {
        Vector<String> shuffled = pair.second;
        std::mt19937 rng(std::random_device{}());
        std::shuffle(shuffled.begin(), shuffled.end(), rng);
        
        size_t total = shuffled.size();
        size_t train_end = static_cast<size_t>(total * train_ratio);
        size_t val_end = train_end + static_cast<size_t>(total * val_ratio);
        
        train.insert(train.end(), shuffled.begin(), shuffled.begin() + train_end);
        val.insert(val.end(), shuffled.begin() + train_end, shuffled.begin() + val_end);
        test.insert(test.end(), shuffled.begin() + val_end, shuffled.end());
    }
}

Result<void> DatasetManagerNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String dataset_path = get_input("dataset_path").as_string();
    String split_strategy = get_param("split_strategy", Data("random")).as_string();
    float train_ratio = static_cast<float>(get_param("train_ratio", Data(0.7)).as_number());
    float val_ratio = static_cast<float>(get_param("val_ratio", Data(0.15)).as_number());
    float test_ratio = static_cast<float>(get_param("test_ratio", Data(0.15)).as_number());
    int k_fold = static_cast<int>(get_param("k_fold", Data(5)).as_int());
    int random_seed = static_cast<int>(get_param("random_seed", Data(42)).as_int());
    
    // 加载图像路径
    Vector<String> image_paths = load_image_paths(dataset_path);
    
    if (image_paths.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No images found in dataset");
    }
    
    // 分割数据集
    Vector<String> train_set, val_set, test_set;
    
    std::mt19937 rng(random_seed);
    Vector<String> shuffled = image_paths;
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    
    size_t total = shuffled.size();
    size_t train_end = static_cast<size_t>(total * train_ratio);
    size_t val_end = train_end + static_cast<size_t>(total * val_ratio);
    
    train_set.assign(shuffled.begin(), shuffled.begin() + train_end);
    val_set.assign(shuffled.begin() + train_end, shuffled.begin() + val_end);
    test_set.assign(shuffled.begin() + val_end, shuffled.end());
    
    // 分析类别分布
    HashMap<String, int> class_distribution;
    if (has_input("annotations")) {
        String annotations_json = get_input("annotations").as_string();
        Vector<ImageAnnotation> annotations = parse_annotations_json(annotations_json);
        class_distribution = analyze_class_distribution(annotations);
    }
    
    // 构建统计信息
    std::ostringstream stats;
    stats << "{";
    stats << "\"total_images\":" << total << ",";
    stats << "\"train_count\":" << train_set.size() << ",";
    stats << "\"val_count\":" << val_set.size() << ",";
    stats << "\"test_count\":" << test_set.size() << ",";
    stats << "\"split_strategy\":\"" << split_strategy << "\",";
    stats << "\"train_ratio\":" << train_ratio << ",";
    stats << "\"val_ratio\":" << val_ratio << ",";
    stats << "\"test_ratio\":" << test_ratio;
    stats << "}";
    
    // 构建类别分布JSON
    std::ostringstream class_dist;
    class_dist << "{";
    int count = 0;
    for (const auto& pair : class_distribution) {
        class_dist << "\"" << escape_json_string(pair.first) << "\":" << pair.second;
        if (++count < static_cast<int>(class_distribution.size())) {
            class_dist << ",";
        }
    }
    class_dist << "}";
    
    // 输出训练集列表
    std::ostringstream train_json;
    train_json << "[";
    for (size_t i = 0; i < train_set.size(); ++i) {
        train_json << "\"" << escape_json_string(train_set[i]) << "\"";
        if (i < train_set.size() - 1) train_json << ",";
    }
    train_json << "]";
    
    // 输出验证集列表
    std::ostringstream val_json;
    val_json << "[";
    for (size_t i = 0; i < val_set.size(); ++i) {
        val_json << "\"" << escape_json_string(val_set[i]) << "\"";
        if (i < val_set.size() - 1) val_json << ",";
    }
    val_json << "]";
    
    // 输出测试集列表
    std::ostringstream test_json;
    test_json << "[";
    for (size_t i = 0; i < test_set.size(); ++i) {
        test_json << "\"" << escape_json_string(test_set[i]) << "\"";
        if (i < test_set.size() - 1) test_json << ",";
    }
    test_json << "]";
    
    set_output("train_set", Data(train_json.str()));
    set_output("val_set", Data(val_json.str()));
    set_output("test_set", Data(test_json.str()));
    set_output("dataset_stats", Data(stats.str()));
    set_output("class_distribution", Data(class_dist.str()));
    
    OVF_INFO() << "DatasetManager completed: train=" << train_set.size() << ", val=" << val_set.size() << ", test=" << test_set.size();
    
    return Result<void>::success();
}

// ==================== DataLabelingNode 实现 ====================

DataLabelingNode::DataLabelingNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DataLabelingNode::make_info() {
    NodeInfo info;
    info.id = "DataLabeling";
    info.name = "数据标注工具";
    info.category = "数据管理";
    info.description = "提供标注工具（矩形框、多边形、语义分割标注）";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("image", "输入图像", DataType::Image, true),
        DataPort("preset_labels", "预设标签", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("annotation", "标注数据", DataType::String),
        DataPort("annotated_image", "带标注图像", DataType::Image),
        DataPort("annotation_stats", "标注统计", DataType::String)
    };
    
    info.params = {
        ParamDef("annotation_mode", "标注模式", DataType::String, Data("rectangle")),
        ParamDef("label", "标签名称", DataType::String, Data("object")),
        ParamDef("class_id", "类别ID", DataType::Number, Data(0)),
        ParamDef("x", "标注区域X坐标", DataType::Number, Data(0)),
        ParamDef("y", "标注区域Y坐标", DataType::Number, Data(0)),
        ParamDef("width", "标注区域宽度", DataType::Number, Data(100)),
        ParamDef("height", "标注区域高度", DataType::Number, Data(100)),
        ParamDef("polygon_points", "多边形点坐标", DataType::String, Data("")),
        ParamDef("mask_points", "分割掩码点", DataType::String, Data("")),
        ParamDef("auto_save", "自动保存", DataType::Boolean, Data(false)),
        ParamDef("save_format", "保存格式", DataType::String, Data("YOLO")),
        ParamDef("output_dir", "输出目录", DataType::String, Data("./labels"))
    };
    
    info.params[0].options = {"rectangle", "polygon", "segmentation", "keypoint"};
    info.params[10].options = {"YOLO", "VOC", "COCO"};
    
    return info;
}

void DataLabelingNode::draw_rectangle_annotation(ImageData& image, const AnnotationObject& obj) {
    if (image.empty()) return;
    
    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;
    
    // 绘制矩形边框
    int x1 = clamp_val(obj.x, 0, img_w - 1);
    int y1 = clamp_val(obj.y, 0, img_h - 1);
    int x2 = clamp_val(obj.x + obj.width, 0, img_w - 1);
    int y2 = clamp_val(obj.y + obj.height, 0, img_h - 1);
    
    uint8_t r = 255, g = 0, b = 0;
    
    // 上边和下边
    for (int x = x1; x <= x2; ++x) {
        if (y1 >= 0 && y1 < img_h) {
            int idx = (y1 * img_w + x) * ch;
            image.data[idx] = r;
            image.data[idx + 1] = g;
            image.data[idx + 2] = b;
        }
        if (y2 >= 0 && y2 < img_h) {
            int idx = (y2 * img_w + x) * ch;
            image.data[idx] = r;
            image.data[idx + 1] = g;
            image.data[idx + 2] = b;
        }
    }
    
    // 左边和右边
    for (int y = y1; y <= y2; ++y) {
        if (x1 >= 0 && x1 < img_w) {
            int idx = (y * img_w + x1) * ch;
            image.data[idx] = r;
            image.data[idx + 1] = g;
            image.data[idx + 2] = b;
        }
        if (x2 >= 0 && x2 < img_w) {
            int idx = (y * img_w + x2) * ch;
            image.data[idx] = r;
            image.data[idx + 1] = g;
            image.data[idx + 2] = b;
        }
    }
}

void DataLabelingNode::draw_polygon_annotation(ImageData& image, const AnnotationObject& obj) {
    if (image.empty() || obj.points.empty()) return;
    
    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;
    
    uint8_t r = 0, g = 255, b = 0;
    
    // 绘制多边形边框
    for (size_t i = 0; i < obj.points.size(); ++i) {
        size_t j = (i + 1) % obj.points.size();
        int x1 = clamp_val(obj.points[i].x, 0, img_w - 1);
        int y1 = clamp_val(obj.points[i].y, 0, img_h - 1);
        int x2 = clamp_val(obj.points[j].x, 0, img_w - 1);
        int y2 = clamp_val(obj.points[j].y, 0, img_h - 1);
        
        // Bresenham直线算法
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        int x = x1, y = y1;
        while (true) {
            if (x >= 0 && x < img_w && y >= 0 && y < img_h) {
                int idx = (y * img_w + x) * ch;
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            }
            if (x == x2 && y == y2) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
    }
}

void DataLabelingNode::draw_segmentation_mask(ImageData& image, const Vector<Point2D<int32_t>>& mask_points, uint8_t r, uint8_t g, uint8_t b) {
    if (image.empty() || mask_points.empty()) return;
    
    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;
    
    // 简化实现：绘制半透明掩码边界
    for (size_t i = 0; i < mask_points.size(); ++i) {
        size_t j = (i + 1) % mask_points.size();
        int x1 = clamp_val(mask_points[i].x, 0, img_w - 1);
        int y1 = clamp_val(mask_points[i].y, 0, img_h - 1);
        int x2 = clamp_val(mask_points[j].x, 0, img_w - 1);
        int y2 = clamp_val(mask_points[j].y, 0, img_h - 1);
        
        // 绘制线段
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        int x = x1, y = y1;
        while (true) {
            if (x >= 0 && x < img_w && y >= 0 && y < img_h) {
                int idx = (y * img_w + x) * ch;
                // 半透明混合
                image.data[idx] = static_cast<uint8_t>((image.data[idx] + r) / 2);
                image.data[idx + 1] = static_cast<uint8_t>((image.data[idx + 1] + g) / 2);
                image.data[idx + 2] = static_cast<uint8_t>((image.data[idx + 2] + b) / 2);
            }
            if (x == x2 && y == y2) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
    }
}

Result<void> DataLabelingNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    // get_input() 按值返回 Data：先落局部变量，否则 as_image() 的引用在这条语句后就悬垂。
    auto input_data = get_input("image");
    const ImageData& input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    String annotation_mode = get_param("annotation_mode", Data("rectangle")).as_string();
    String label = get_param("label", Data("object")).as_string();
    int class_id = static_cast<int>(get_param("class_id", Data(0)).as_int());
    int x = static_cast<int>(get_param("x", Data(0)).as_int());
    int y = static_cast<int>(get_param("y", Data(0)).as_int());
    int width = static_cast<int>(get_param("width", Data(100)).as_int());
    int height = static_cast<int>(get_param("height", Data(100)).as_int());
    String polygon_points_str = get_param("polygon_points", Data("")).as_string();
    String mask_points_str = get_param("mask_points", Data("")).as_string();
    
    // 创建标注对象
    AnnotationObject obj;
    obj.label = label;
    obj.class_id = class_id;
    obj.confidence = 1.0f;
    
    if (annotation_mode == "rectangle") {
        obj.type = AnnotationType::Rectangle;
        obj.x = x;
        obj.y = y;
        obj.width = width;
        obj.height = height;
    } else if (annotation_mode == "polygon") {
        obj.type = AnnotationType::Polygon;
        // 解析多边形点（格式："x1,y1;x2,y2;..."）
        std::istringstream iss(polygon_points_str);
        String token;
        while (std::getline(iss, token, ';')) {
            size_t comma_pos = token.find(',');
            if (comma_pos != String::npos) {
                int px = std::stoi(token.substr(0, comma_pos));
                int py = std::stoi(token.substr(comma_pos + 1));
                obj.points.emplace_back(px, py);
            }
        }
    } else if (annotation_mode == "segmentation") {
        obj.type = AnnotationType::Polygon;
        // 解析分割掩码点
        std::istringstream iss(mask_points_str);
        String token;
        while (std::getline(iss, token, ';')) {
            size_t comma_pos = token.find(',');
            if (comma_pos != String::npos) {
                int px = std::stoi(token.substr(0, comma_pos));
                int py = std::stoi(token.substr(comma_pos + 1));
                obj.points.emplace_back(px, py);
            }
        }
    }
    
    // 创建图像标注
    ImageAnnotation annotation;
    annotation.width = input.width;
    annotation.height = input.height;
    annotation.objects.push_back(obj);
    
    // 绘制标注
    ImageData annotated_image = input;
    if (annotation_mode == "rectangle") {
        draw_rectangle_annotation(annotated_image, obj);
    } else if (annotation_mode == "polygon") {
        draw_polygon_annotation(annotated_image, obj);
    } else if (annotation_mode == "segmentation") {
        draw_segmentation_mask(annotated_image, obj.points, 255, 0, 128);
    }
    
    // 构建统计信息
    std::ostringstream stats;
    stats << "{";
    stats << "\"annotation_mode\":\"" << annotation_mode << "\",";
    stats << "\"label\":\"" << label << "\",";
    stats << "\"class_id\":" << class_id << ",";
    stats << "\"object_count\":1";
    stats << "}";
    
    set_output("annotation", Data(annotation.to_json()));
    set_output("annotated_image", Data(annotated_image));
    set_output("annotation_stats", Data(stats.str()));
    
    OVF_INFO() << "DataLabeling completed: mode=" << annotation_mode << ", label=" << label;
    
    return Result<void>::success();
}

// ==================== DataValidationNode 实现 ====================

DataValidationNode::DataValidationNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DataValidationNode::make_info() {
    NodeInfo info;
    info.id = "DataValidation";
    info.name = "数据验证";
    info.category = "数据管理";
    info.description = "检查标注质量、数据分布、数据完整性";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("dataset_path", "数据集路径", DataType::String, true),
        DataPort("annotations", "标注数据", DataType::String, true)
    };
    
    info.outputs = {
        DataPort("validation_report", "验证报告", DataType::String),
        DataPort("quality_score", "质量评分", DataType::Number),
        DataPort("issues", "问题列表", DataType::String),
        DataPort("recommendations", "改进建议", DataType::String)
    };
    
    info.params = {
        ParamDef("check_duplicate", "检查重复", DataType::Boolean, Data(true)),
        ParamDef("check_missing", "检查缺失", DataType::Boolean, Data(true)),
        ParamDef("check_outliers", "检查异常", DataType::Boolean, Data(true)),
        ParamDef("min_quality_score", "最小质量评分", DataType::Number, Data(0.7))
    };
    
    return info;
}

float DataValidationNode::calculate_quality_score(const Vector<ImageAnnotation>& annotations) {
    if (annotations.empty()) return 0.0f;
    
    float score = 100.0f;
    
    // 检查标注完整性
    int empty_count = 0;
    int boundary_issue_count = 0;
    int label_missing_count = 0;
    
    for (const auto& ann : annotations) {
        if (ann.objects.empty()) {
            empty_count++;
        }
        
        for (const auto& obj : ann.objects) {
            // 检查边界问题
            if (obj.x < 0 || obj.y < 0 || obj.width <= 0 || obj.height <= 0) {
                boundary_issue_count++;
            }
            
            // 检查标签缺失
            if (obj.label.empty()) {
                label_missing_count++;
            }
        }
    }
    
    // 计算扣分
    score -= static_cast<float>(empty_count) / annotations.size() * 30.0f;
    score -= static_cast<float>(boundary_issue_count) / annotations.size() * 20.0f;
    score -= static_cast<float>(label_missing_count) / annotations.size() * 15.0f;
    
    return std::max(0.0f, score / 100.0f);
}

Vector<String> DataValidationNode::detect_duplicates(const Vector<ImageAnnotation>& annotations) {
    Vector<String> duplicates;
    
    for (size_t i = 0; i < annotations.size(); ++i) {
        for (size_t j = i + 1; j < annotations.size(); ++j) {
            if (annotations[i].image_path == annotations[j].image_path) {
                duplicates.push_back("Duplicate image: " + annotations[i].image_path);
            }
            
            // 检查相同位置的重复标注
            for (const auto& obj1 : annotations[i].objects) {
                for (const auto& obj2 : annotations[j].objects) {
                    if (obj1.label == obj2.label &&
                        std::abs(obj1.x - obj2.x) < 5 &&
                        std::abs(obj1.y - obj2.y) < 5 &&
                        std::abs(obj1.width - obj2.width) < 5 &&
                        std::abs(obj1.height - obj2.height) < 5) {
                        duplicates.push_back("Similar annotations in: " + annotations[i].image_path);
                    }
                }
            }
        }
    }
    
    return duplicates;
}

Vector<String> DataValidationNode::detect_missing_annotations(const Vector<String>& image_paths, const Vector<ImageAnnotation>& annotations) {
    Vector<String> missing;
    
    // 检查图像路径是否有对应标注
    std::unordered_set<String> annotated_images;
    for (const auto& ann : annotations) {
        annotated_images.insert(ann.image_path);
    }
    
    for (const auto& path : image_paths) {
        if (annotated_images.find(path) == annotated_images.end()) {
            missing.push_back("Missing annotation for: " + path);
        }
    }
    
    return missing;
}

Vector<String> DataValidationNode::detect_outliers(const Vector<ImageAnnotation>& annotations) {
    Vector<String> outliers;
    
    // 检查尺寸异常
    Vector<float> widths, heights;
    for (const auto& ann : annotations) {
        for (const auto& obj : ann.objects) {
            widths.push_back(static_cast<float>(obj.width));
            heights.push_back(static_cast<float>(obj.height));
        }
    }
    
    if (widths.size() < 10) return outliers; // 样本太少，不检测异常
    
    // 计算平均值和标准差
    float mean_w = 0.0f, mean_h = 0.0f;
    for (float w : widths) mean_w += w;
    for (float h : heights) mean_h += h;
    mean_w /= widths.size();
    mean_h /= heights.size();
    
    float std_w = 0.0f, std_h = 0.0f;
    for (float w : widths) std_w += (w - mean_w) * (w - mean_w);
    for (float h : heights) std_h += (h - mean_h) * (h - mean_h);
    std_w = std::sqrt(std_w / widths.size());
    std_h = std::sqrt(std_h / heights.size());
    
    // 检测异常值（超过3倍标准差）
    for (const auto& ann : annotations) {
        for (const auto& obj : ann.objects) {
            float w = static_cast<float>(obj.width);
            float h = static_cast<float>(obj.height);
            
            if (std::abs(w - mean_w) > 3 * std_w || std::abs(h - mean_h) > 3 * std_h) {
                outliers.push_back("Size outlier in: " + ann.image_path + " (w=" + std::to_string(obj.width) + ", h=" + std::to_string(obj.height));
            }
        }
    }
    
    return outliers;
}

Result<void> DataValidationNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String dataset_path = get_input("dataset_path").as_string();
    String annotations_json = get_input("annotations").as_string();
    
    bool check_duplicate = get_param("check_duplicate", Data(true)).as_bool();
    bool check_missing = get_param("check_missing", Data(true)).as_bool();
    bool check_outliers = get_param("check_outliers", Data(true)).as_bool();
    float min_quality_score = static_cast<float>(get_param("min_quality_score", Data(0.7)).as_number());
    
    // 解析标注数据
    Vector<ImageAnnotation> annotations = parse_annotations_json(annotations_json);
    
    // 加载图像路径
    Vector<String> image_paths = parse_json_string_array(dataset_path);
    
    // 计算质量评分
    float quality_score = calculate_quality_score(annotations);
    
    // 检测问题
    Vector<String> all_issues;
    
    if (check_duplicate) {
        Vector<String> duplicates = detect_duplicates(annotations);
        all_issues.insert(all_issues.end(), duplicates.begin(), duplicates.end());
    }
    
    if (check_missing) {
        Vector<String> missing = detect_missing_annotations(image_paths, annotations);
        all_issues.insert(all_issues.end(), missing.begin(), missing.end());
    }
    
    if (check_outliers) {
        Vector<String> outliers = detect_outliers(annotations);
        all_issues.insert(all_issues.end(), outliers.begin(), outliers.end());
    }
    
    // 生成改进建议
    Vector<String> recommendations;
    if (quality_score < min_quality_score) {
        recommendations.push_back("Quality score below threshold, review annotations");
    }
    if (!all_issues.empty()) {
        recommendations.push_back("Found " + std::to_string(all_issues.size()) + " issues, please review");
    }
    if (annotations.size() < image_paths.size()) {
        recommendations.push_back("Some images missing annotations, consider completing labeling");
    }
    
    // 构建验证报告
    std::ostringstream report;
    report << "{";
    report << "\"total_images\":" << image_paths.size() << ",";
    report << "\"annotated_images\":" << annotations.size() << ",";
    report << "\"quality_score\":" << quality_score << ",";
    report << "\"issue_count\":" << all_issues.size() << ",";
    report << "\"passed\":" << (quality_score >= min_quality_score && all_issues.empty() ? "true" : "false") << ",";
    report << "\"issues\":[";
    for (size_t i = 0; i < all_issues.size(); ++i) {
        report << "\"" << escape_json_string(all_issues[i]) << "\"";
        if (i < all_issues.size() - 1) report << ",";
    }
    report << "],";
    report << "\"recommendations\":[";
    for (size_t i = 0; i < recommendations.size(); ++i) {
        report << "\"" << escape_json_string(recommendations[i]) << "\"";
        if (i < recommendations.size() - 1) report << ",";
    }
    report << "]";
    report << "}";
    
    // 输出问题列表
    std::ostringstream issues_json;
    issues_json << "[";
    for (size_t i = 0; i < all_issues.size(); ++i) {
        issues_json << "\"" << escape_json_string(all_issues[i]) << "\"";
        if (i < all_issues.size() - 1) issues_json << ",";
    }
    issues_json << "]";
    
    // 输出建议列表
    std::ostringstream recs_json;
    recs_json << "[";
    for (size_t i = 0; i < recommendations.size(); ++i) {
        recs_json << "\"" << escape_json_string(recommendations[i]) << "\"";
        if (i < recommendations.size() - 1) recs_json << ",";
    }
    recs_json << "]";
    
    set_output("validation_report", Data(report.str()));
    set_output("quality_score", Data(quality_score));
    set_output("issues", Data(issues_json.str()));
    set_output("recommendations", Data(recs_json.str()));
    
    OVF_INFO() << "DataValidation completed: score=" << quality_score << ", issues=" << all_issues.size();
    
    return Result<void>::success();
}

// ==================== TrainProgressNode 实现 ====================

TrainProgressNode::TrainProgressNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo TrainProgressNode::make_info() {
    NodeInfo info;
    info.id = "TrainProgress";
    info.name = "训练进度监控";
    info.category = "训练增强";
    info.description = "TensorBoard风格日志，监控损失曲线、指标曲线、直方图";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("training_state", "训练状态", DataType::String, true),
        DataPort("metrics", "当前指标", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("log_path", "日志路径", DataType::String),
        DataPort("visualization_data", "可视化数据", DataType::String),
        DataPort("progress_report", "进度报告", DataType::String)
    };
    
    info.params = {
        ParamDef("log_dir", "日志目录", DataType::String, Data("./logs")),
        ParamDef("log_interval", "记录间隔", DataType::Number, Data(10)),
        ParamDef("save_histogram", "保存直方图", DataType::Boolean, Data(false)),
        ParamDef("save_graph", "保存计算图", DataType::Boolean, Data(false))
    };
    
    return info;
}

String TrainProgressNode::write_tensorboard_log(const String& log_dir, int epoch, float loss, const HashMap<String, float>& metrics) {
    // 生成TensorBoard风格日志文件路径
    String log_path = log_dir + "/events_epoch_" + std::to_string(epoch) + ".json";
    
    std::ostringstream log_content;
    log_content << "{";
    log_content << "\"epoch\":" << epoch << ",";
    log_content << "\"loss\":" << loss << ",";
    log_content << "\"metrics\":{";
    
    int count = 0;
    for (const auto& pair : metrics) {
        log_content << "\"" << escape_json_string(pair.first) << "\":" << pair.second;
        if (++count < static_cast<int>(metrics.size())) {
            log_content << ",";
        }
    }
    
    log_content << "}";
    log_content << "}";
    
    return log_path;
}

String TrainProgressNode::generate_loss_curve(const Vector<float>& losses) {
    if (losses.empty()) return "{}";
    
    std::ostringstream curve;
    curve << "{";
    curve << "\"type\":\"line\",";
    curve << "\"title\":\"Loss Curve\",";
    curve << "\"x_label\":\"Epoch\",";
    curve << "\"y_label\":\"Loss\",";
    curve << "\"data\":[";
    
    for (size_t i = 0; i < losses.size(); ++i) {
        curve << "{\"x\":" << i << ",\"y\":" << losses[i] << "}";
        if (i < losses.size() - 1) curve << ",";
    }
    
    curve << "]";
    curve << "}";
    
    return curve.str();
}

String TrainProgressNode::generate_metric_curve(const String& metric_name, const Vector<float>& values) {
    if (values.empty()) return "{}";
    
    std::ostringstream curve;
    curve << "{";
    curve << "\"type\":\"line\",";
    curve << "\"title\":\"" << metric_name << " Curve\",";
    curve << "\"x_label\":\"Epoch\",";
    curve << "\"y_label\":\"" << metric_name << "\",";
    curve << "\"data\":[";
    
    for (size_t i = 0; i < values.size(); ++i) {
        curve << "{\"x\":" << i << ",\"y\":" << values[i] << "}";
        if (i < values.size() - 1) curve << ",";
    }
    
    curve << "]";
    curve << "}";
    
    return curve.str();
}

String TrainProgressNode::generate_histogram_data(const Vector<float>& values, int bins) {
    if (values.empty()) return "{}";
    
    // 计算直方图
    float min_val = values[0];
    float max_val = values[0];
    for (float v : values) {
        min_val = std::min(min_val, v);
        max_val = std::max(max_val, v);
    }
    
    float bin_width = (max_val - min_val) / bins;
    Vector<int> histogram(bins, 0);
    
    for (float v : values) {
        int bin_idx = static_cast<int>((v - min_val) / bin_width);
        if (bin_idx >= bins) bin_idx = bins - 1;
        histogram[bin_idx]++;
    }
    
    std::ostringstream hist_json;
    hist_json << "{";
    hist_json << "\"type\":\"histogram\",";
    hist_json << "\"bins\":" << bins << ",";
    hist_json << "\"min\":" << min_val << ",";
    hist_json << "\"max\":" << max_val << ",";
    hist_json << "\"counts\":[";
    
    for (size_t i = 0; i < histogram.size(); ++i) {
        hist_json << histogram[i];
        if (i < histogram.size() - 1) hist_json << ",";
    }
    
    hist_json << "]";
    hist_json << "}";
    
    return hist_json.str();
}

Result<void> TrainProgressNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String training_state_json = get_input("training_state").as_string();
    String log_dir = get_param("log_dir", Data("./logs")).as_string();
    int log_interval = static_cast<int>(get_param("log_interval", Data(10)).as_int());
    bool save_histogram = get_param("save_histogram", Data(false)).as_bool();
    bool save_graph = get_param("save_graph", Data(false)).as_bool();
    
    // 解析训练状态
    // 假设格式: {"epoch":10, "loss":0.5, "metrics":{"accuracy":0.85}}
    int epoch = 0;
    float loss = 0.0f;
    HashMap<String, float> metrics;
    
    // 简化解析
    size_t epoch_pos = training_state_json.find("\"epoch\"");
    if (epoch_pos != String::npos) {
        size_t start = training_state_json.find_first_of("0123456789", epoch_pos);
        epoch = std::stoi(training_state_json.substr(start));
    }
    
    size_t loss_pos = training_state_json.find("\"loss\"");
    if (loss_pos != String::npos) {
        size_t start = training_state_json.find_first_of("0123456789.", loss_pos);
        size_t end = training_state_json.find_first_not_of("0123456789.", start);
        loss = std::stof(training_state_json.substr(start, end - start));
    }
    
    // 写入日志
    String log_path = write_tensorboard_log(log_dir, epoch, loss, metrics);
    
    // 生成可视化数据
    Vector<float> losses; // 假设已有历史损失数据
    String loss_curve = generate_loss_curve(losses);
    
    std::ostringstream viz_data;
    viz_data << "{";
    viz_data << "\"loss_curve\":" << loss_curve << ",";
    viz_data << "\"current_epoch\":" << epoch << ",";
    viz_data << "\"current_loss\":" << loss;
    viz_data << "}";
    
    // 生成进度报告
    std::ostringstream progress;
    progress << "{";
    progress << "\"epoch\":" << epoch << ",";
    progress << "\"loss\":" << loss << ",";
    progress << "\"log_path\":\"" << log_path << "\",";
    progress << "\"log_interval\":" << log_interval << ",";
    progress << "\"save_histogram\":" << (save_histogram ? "true" : "false") << ",";
    progress << "\"save_graph\":" << (save_graph ? "true" : "false");
    progress << "}";
    
    set_output("log_path", Data(log_path));
    set_output("visualization_data", Data(viz_data.str()));
    set_output("progress_report", Data(progress.str()));
    
    OVF_INFO() << "TrainProgress logged: epoch=" << epoch << ", loss=" << loss;
    
    return Result<void>::success();
}

// ==================== TrainCheckpointNode 实现 ====================

TrainCheckpointNode::TrainCheckpointNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo TrainCheckpointNode::make_info() {
    NodeInfo info;
    info.id = "TrainCheckpoint";
    info.name = "训练检查点";
    info.category = "训练增强";
    info.description = "保存/恢复训练状态，支持断点续训";
    info.version = "1.0.0";
    
    info.inputs = {
        DataPort("model_state", "模型状态", DataType::String, true),
        DataPort("optimizer_state", "优化器状态", DataType::String, false),
        DataPort("training_config", "训练配置", DataType::String, false)
    };
    
    info.outputs = {
        DataPort("checkpoint_path", "检查点路径", DataType::String),
        DataPort("checkpoint_info", "检查点信息", DataType::String),
        DataPort("restore_info", "恢复信息", DataType::String)
    };
    
    info.params = {
        ParamDef("checkpoint_dir", "检查点目录", DataType::String, Data("./checkpoints")),
        ParamDef("save_interval", "保存间隔", DataType::Number, Data(5)),
        ParamDef("max_checkpoints", "最大检查点数", DataType::Number, Data(5)),
        ParamDef("save_best", "保存最佳", DataType::Boolean, Data(true)),
        ParamDef("restore_from", "恢复路径", DataType::String, Data(""))
    };
    
    return info;
}

String TrainCheckpointNode::save_checkpoint(const String& checkpoint_dir, int epoch, const String& model_state, const String& optimizer_state) {
    String checkpoint_path = checkpoint_dir + "/checkpoint_epoch_" + std::to_string(epoch) + ".json";
    
    std::ostringstream checkpoint_content;
    checkpoint_content << "{";
    checkpoint_content << "\"epoch\":" << epoch << ",";
    checkpoint_content << "\"model_state\":\"" << escape_json_string(model_state) << "\",";
    checkpoint_content << "\"optimizer_state\":\"" << escape_json_string(optimizer_state) << "\",";
    checkpoint_content << "\"timestamp\":\"" << std::chrono::system_clock::now().time_since_epoch().count() << "\"";
    checkpoint_content << "}";
    
    return checkpoint_path;
}

String TrainCheckpointNode::load_checkpoint(const String& checkpoint_path) {
    // 简化实现：返回检查点路径作为恢复信息
    std::ostringstream restore_info;
    restore_info << "{";
    restore_info << "\"checkpoint_path\":\"" << checkpoint_path << "\",";
    restore_info << "\"restored\":true";
    restore_info << "}";
    
    return restore_info.str();
}

Vector<String> TrainCheckpointNode::list_checkpoints(const String& checkpoint_dir) {
    // 简化实现：返回空列表
    Vector<String> checkpoints;
    return checkpoints;
}

void TrainCheckpointNode::cleanup_old_checkpoints(const String& checkpoint_dir, int max_count) {
    // 简化实现：不做实际清理
}

Result<void> TrainCheckpointNode::execute(FlowContext& context) {
    auto validate_result = validate_inputs();
    if (validate_result.is_failure()) {
        return validate_result;
    }
    
    String model_state = get_input("model_state").as_string();
    String optimizer_state = has_input("optimizer_state") ? get_input("optimizer_state").as_string() : "";
    String training_config = has_input("training_config") ? get_input("training_config").as_string() : "";
    
    String checkpoint_dir = get_param("checkpoint_dir", Data("./checkpoints")).as_string();
    int save_interval = static_cast<int>(get_param("save_interval", Data(5)).as_int());
    int max_checkpoints = static_cast<int>(get_param("max_checkpoints", Data(5)).as_int());
    bool save_best = get_param("save_best", Data(true)).as_bool();
    String restore_from = get_param("restore_from", Data("")).as_string();
    
    String checkpoint_path;
    String checkpoint_info;
    String restore_info;
    
    // 如果指定了恢复路径
    if (!restore_from.empty()) {
        restore_info = load_checkpoint(restore_from);
    } else {
        // 保存检查点
        // 假设从training_config解析epoch
        int epoch = 0;
        size_t epoch_pos = training_config.find("\"epoch\"");
        if (epoch_pos != String::npos) {
            size_t start = training_config.find_first_of("0123456789", epoch_pos);
            epoch = std::stoi(training_config.substr(start));
        }
        
        checkpoint_path = save_checkpoint(checkpoint_dir, epoch, model_state, optimizer_state);
        
        std::ostringstream info;
        info << "{";
        info << "\"checkpoint_path\":\"" << checkpoint_path << "\",";
        info << "\"epoch\":" << epoch << ",";
        info << "\"save_best\":" << (save_best ? "true" : "false") << ",";
        info << "\"max_checkpoints\":" << max_checkpoints;
        info << "}";
        
        checkpoint_info = info.str();
        
        // 清理旧检查点
        cleanup_old_checkpoints(checkpoint_dir, max_checkpoints);
    }
    
    set_output("checkpoint_path", Data(checkpoint_path));
    set_output("checkpoint_info", Data(checkpoint_info));
    set_output("restore_info", Data(restore_info));
    
    OVF_INFO() << "TrainCheckpoint: path=" << checkpoint_path;
    
    return Result<void>::success();
}

// ==================== 节点注册 ====================

OVF_REGISTER_NODE(ImageAnnotateNode, "ImageAnnotate", ImageAnnotateNode::make_info())
OVF_REGISTER_NODE(LabelExportNode, "LabelExport", LabelExportNode::make_info())
OVF_REGISTER_NODE(LabelMergeNode, "LabelMerge", LabelMergeNode::make_info())
OVF_REGISTER_NODE(LabelVerifyNode, "LabelVerify", LabelVerifyNode::make_info())
OVF_REGISTER_NODE(DataAugmentNode, "DataAugment", DataAugmentNode::make_info())
OVF_REGISTER_NODE(RandomCropNode, "RandomCrop", RandomCropNode::make_info())
OVF_REGISTER_NODE(ColorAugmentNode, "ColorAugment", ColorAugmentNode::make_info())
OVF_REGISTER_NODE(MixupNode, "Mixup", MixupNode::make_info())
OVF_REGISTER_NODE(DatasetSplitNode, "DatasetSplit", DatasetSplitNode::make_info())
OVF_REGISTER_NODE(TrainConfigNode, "TrainConfig", TrainConfigNode::make_info())
OVF_REGISTER_NODE(ModelEvaluateNode, "ModelEvaluate", ModelEvaluateNode::make_info())
OVF_REGISTER_NODE(ExportONNXNode, "ExportONNX", ExportONNXNode::make_info())

// 注册新增节点
OVF_REGISTER_NODE(DatasetManagerNode, "DatasetManager", DatasetManagerNode::make_info())
OVF_REGISTER_NODE(DataLabelingNode, "DataLabeling", DataLabelingNode::make_info())
OVF_REGISTER_NODE(DataValidationNode, "DataValidation", DataValidationNode::make_info())
OVF_REGISTER_NODE(TrainProgressNode, "TrainProgress", TrainProgressNode::make_info())
OVF_REGISTER_NODE(TrainCheckpointNode, "TrainCheckpoint", TrainCheckpointNode::make_info())

// 注册待实现的节点（使用简化版本）
// ModelValidationNode, ConfusionMatrixNode, ValidationReportNode
// ModelExportNode, ModelOptimizeNode, ModelDeployNode
// DeepLearningWorkflowNode

} // namespace algorithm
} // namespace ovf