/**
 * @file onnx_inference.cpp
 * @brief ONNX推理模块实现
 */

#include "ovf/algorithm/onnx_inference.h"
#include "ovf/core/logger.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <random>

namespace ovf {
namespace algorithm {
namespace onnx_utils {

// ONNXEnvironment 实现

ONNXEnvironment::ONNXEnvironment()
    : execution_provider_("CPU")
    , inference_threads_(4)
    , model_loaded_(false) {
}

ONNXEnvironment::~ONNXEnvironment() {
    model_loaded_ = false;
}

ErrorCode ONNXEnvironment::load_model(const String& model_path) {
    // 检查文件是否存在
    std::ifstream file(model_path, std::ios::binary);
    if (!file.is_open()) {
        OVF_ERROR() << "Model file not found: " << model_path;
        return ErrorCode::FileNotFound;
    }
    
    // 获取文件大小
    file.seekg(0, std::ios::end);
    model_info_.model_size = file.tellg();
    file.close();
    
    model_path_ = model_path;
    
    // 提取模型名称
    size_t last_sep = model_path.find_last_of("/\\");
    if (last_sep != String::npos) {
        model_info_.model_name = model_path.substr(last_sep + 1);
    } else {
        model_info_.model_name = model_path;
    }
    
    // 设置默认输入输出（模拟）
    model_info_.input_names.clear();
    model_info_.output_names.clear();
    model_info_.input_shapes.clear();
    model_info_.output_shapes.clear();
    
    // 添加默认输入输出
    model_info_.input_names.push_back("input");
    model_info_.input_shapes.push_back({1, 3, 224, 224});  // NCHW格式
    
    model_info_.output_names.push_back("output");
    model_info_.output_shapes.push_back({1, 1000});  // 分类输出
    
    model_loaded_ = true;
    
    OVF_INFO() << "ONNX model loaded (simulated): " << model_path 
               << " (" << model_info_.model_size << " bytes)";
    
    return ErrorCode::Success;
}

ErrorCode ONNXEnvironment::get_model_info(ModelInfo& info) {
    if (!model_loaded_) {
        return ErrorCode::InvalidModel;
    }
    
    info = model_info_;
    return ErrorCode::Success;
}

ErrorCode ONNXEnvironment::run_inference(const Vector<float>& input_data, InferenceResult& result) {
    if (!model_loaded_) {
        OVF_ERROR() << "Model not loaded";
        return ErrorCode::InvalidModel;
    }
    
    // 模拟推理过程
    simulate_inference(input_data, result);
    
    return ErrorCode::Success;
}

ErrorCode ONNXEnvironment::run_inference_multi(const Vector<Vector<float>>& inputs, 
                                                Vector<InferenceResult>& results) {
    if (!model_loaded_) {
        return ErrorCode::InvalidModel;
    }
    
    results.clear();
    results.reserve(inputs.size());
    
    for (const auto& input : inputs) {
        InferenceResult result;
        ErrorCode err = run_inference(input, result);
        if (err != ErrorCode::Success) {
            return err;
        }
        results.push_back(result);
    }
    
    return ErrorCode::Success;
}

void ONNXEnvironment::set_execution_provider(const String& provider) {
    execution_provider_ = provider;
    OVF_INFO() << "Execution provider set to: " << provider;
}

void ONNXEnvironment::set_inference_threads(int threads) {
    inference_threads_ = std::max(1, threads);
    OVF_INFO() << "Inference threads set to: " << inference_threads_;
}

void ONNXEnvironment::simulate_inference(const Vector<float>& input, InferenceResult& output) {
    // 模拟推理时间（基于输入大小）
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 模拟计算延迟
    size_t input_size = input.size();
    int simulated_delay_ms = static_cast<int>(input_size / 1000) + 5;  // 基础延迟5ms
    
    // 生成模拟输出
    output.output_name = model_info_.output_names.empty() ? "output" : model_info_.output_names[0];
    
    // 设置输出形状
    if (!model_info_.output_shapes.empty()) {
        output.output_shape = model_info_.output_shapes[0];
    } else {
        output.output_shape = {1, static_cast<int>(input_size)};
    }
    
    // 计算输出大小
    size_t output_size = 1;
    for (int dim : output.output_shape) {
        output_size *= dim;
    }
    
    // 生成模拟输出数据（伪随机，基于输入）
    output.output_data.resize(output_size);
    
    std::mt19937 rng(static_cast<unsigned>(input.size() > 0 ? 
        static_cast<unsigned>(input[0] * 1000) : 42));
    std::normal_distribution<float> dist(0.5f, 0.3f);
    
    for (size_t i = 0; i < output_size; ++i) {
        output.output_data[i] = dist(rng);
    }
    
    // 归一化输出（模拟softmax）
    float sum = 0.0f;
    for (float val : output.output_data) {
        sum += std::exp(val);
    }
    for (float& val : output.output_data) {
        val = std::exp(val) / sum;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    output.inference_time_ms = static_cast<float>(duration.count()) + simulated_delay_ms;
}

} // namespace onnx_utils

// ONNXInferenceNode 实现

ONNXInferenceNode::ONNXInferenceNode(const String& instance_id)
    : INode(instance_id, make_info())
    , initialized_(false) {
}

NodeInfo ONNXInferenceNode::make_info() {
    NodeInfo info;
    info.id = "ONNXInference";
    info.name = "ONNX推理";
    info.category = "深度学习";
    info.description = "通用ONNX模型推理节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, false));
    info.inputs.push_back(DataPort("input_data", "输入数据", DataType::Array, false));
    
    info.outputs.push_back(DataPort("output_data", "输出数据", DataType::Array));
    info.outputs.push_back(DataPort("output_shape", "输出形状", DataType::Array));
    info.outputs.push_back(DataPort("inference_time", "推理时间", DataType::Number));
    
    info.params.push_back(ParamDef("model_path", "模型路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("input_name", "输入名称", DataType::String, Data("input")));
    info.params.push_back(ParamDef("output_name", "输出名称", DataType::String, Data("output")));
    info.params.push_back(ParamDef("execution_provider", "执行提供器", DataType::String, Data("CPU")));
    
    return info;
}

Result<void> ONNXInferenceNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    String provider = get_param("execution_provider", Data("CPU")).as_string();
    
    if (!model_path.empty()) {
        env_.set_execution_provider(provider);
        ErrorCode err = env_.load_model(model_path);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "Failed to load ONNX model: " + model_path);
        }
        initialized_ = true;
        OVF_INFO() << "ONNXInferenceNode initialized with model: " << model_path;
    }
    
    return Result<void>::success();
}

Result<void> ONNXInferenceNode::execute(FlowContext& context) {
    // 获取输入
    Vector<float> input_data;
    
    if (has_input("image")) {
        auto img_data = get_input("image");
        if (img_data.is_image()) {
            ImageData img = img_data.as_image();
            // 将图像转换为float数据（简单转换）
            input_data.resize(img.data.size());
            for (size_t i = 0; i < img.data.size(); ++i) {
                input_data[i] = static_cast<float>(img.data[i]) / 255.0f;
            }
        }
    } else if (has_input("input_data")) {
        // 从Array类型获取数据（这里简化处理）
        auto data = get_input("input_data");
        // 如果是字符串表示的数组，需要解析（简化：使用默认数据）
        input_data.resize(224 * 224 * 3);
        std::fill(input_data.begin(), input_data.end(), 0.5f);
    } else {
        // 使用默认输入
        input_data.resize(224 * 224 * 3);
        std::fill(input_data.begin(), input_data.end(), 0.5f);
    }
    
    // 执行推理
    onnx_utils::InferenceResult result;
    ErrorCode err = env_.run_inference(input_data, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Inference failed");
    }
    
    // 设置输出（将float数组转换为字符串表示）
    // 输出数据 - 使用Object类型存储
    String output_str;
    output_str = "[" + std::to_string(result.output_data.size()) + " floats]";
    set_output("output_data", Data(output_str));
    
    // 输出形状
    String shape_str = "[";
    for (size_t i = 0; i < result.output_shape.size(); ++i) {
        shape_str += std::to_string(result.output_shape[i]);
        if (i < result.output_shape.size() - 1) shape_str += ",";
    }
    shape_str += "]";
    set_output("output_shape", Data(shape_str));
    
    // 推理时间
    set_output("inference_time", Data(result.inference_time_ms));
    
    OVF_INFO() << "ONNX inference completed in " << result.inference_time_ms << "ms";
    
    return Result<void>::success();
}

// DetectionNode 实现

DetectionNode::DetectionNode(const String& instance_id)
    : INode(instance_id, make_info())
    , initialized_(false) {
}

NodeInfo DetectionNode::make_info() {
    NodeInfo info;
    info.id = "Detection";
    info.name = "目标检测";
    info.category = "深度学习";
    info.description = "目标检测推理节点（YOLO/SSD等）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("boxes", "检测框", DataType::Array));
    info.outputs.push_back(DataPort("scores", "置信度", DataType::Array));
    info.outputs.push_back(DataPort("classes", "类别", DataType::Array));
    info.outputs.push_back(DataPort("detection_count", "检测数量", DataType::Number));
    
    info.params.push_back(ParamDef("model_path", "模型路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(0.5)));
    info.params.push_back(ParamDef("nms_threshold", "NMS阈值", DataType::Number, Data(0.45)));
    info.params.push_back(ParamDef("execution_provider", "执行提供器", DataType::String, Data("CPU")));
    
    return info;
}

Result<void> DetectionNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    String provider = get_param("execution_provider", Data("CPU")).as_string();
    
    if (!model_path.empty()) {
        env_.set_execution_provider(provider);
        ErrorCode err = env_.load_model(model_path);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "Failed to load detection model");
        }
        initialized_ = true;
    }
    
    return Result<void>::success();
}

Result<void> DetectionNode::execute(FlowContext& context) {
    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData img = img_data.as_image();
    
    float conf_threshold = get_param("confidence_threshold", Data(0.5)).as_number();
    float nms_threshold = get_param("nms_threshold", Data(0.45)).as_number();
    
    // 准备输入数据
    Vector<float> input_data;
    input_data.resize(img.data.size());
    for (size_t i = 0; i < img.data.size(); ++i) {
        input_data[i] = static_cast<float>(img.data[i]) / 255.0f;
    }
    
    // 执行推理
    onnx_utils::InferenceResult result;
    ErrorCode err = env_.run_inference(input_data, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Detection inference failed");
    }
    
    // 模拟检测结果（实际应用中需要根据模型输出格式解析）
    Vector<Region> boxes;
    Vector<float> scores;
    Vector<int> classes;
    
    // 生成模拟检测结果
    std::mt19937 rng(img.frame_id);
    std::uniform_int_distribution<int> pos_dist(0, std::min(img.width, img.height) / 2);
    std::uniform_int_distribution<int> size_dist(50, 200);
    std::uniform_real_distribution<float> score_dist(0.3f, 0.95f);
    std::uniform_int_distribution<int> class_dist(0, 9);
    
    int num_detections = 3 + (rng() % 5);  // 3-7个检测结果
    
    for (int i = 0; i < num_detections; ++i) {
        int x = pos_dist(rng);
        int y = pos_dist(rng);
        int w = size_dist(rng);
        int h = size_dist(rng);
        
        boxes.emplace_back(x, y, w, h);
        scores.push_back(score_dist(rng));
        classes.push_back(class_dist(rng));
    }
    
    // 应用NMS
    apply_nms(boxes, scores, classes, conf_threshold, nms_threshold);
    
    // 设置输出
    String boxes_str = "[" + std::to_string(boxes.size()) + " boxes]";
    set_output("boxes", Data(boxes_str));
    
    String scores_str = "[";
    for (size_t i = 0; i < scores.size(); ++i) {
        scores_str += std::to_string(scores[i]);
        if (i < scores.size() - 1) scores_str += ",";
    }
    scores_str += "]";
    set_output("scores", Data(scores_str));
    
    String classes_str = "[";
    for (size_t i = 0; i < classes.size(); ++i) {
        classes_str += std::to_string(classes[i]);
        if (i < classes.size() - 1) classes_str += ",";
    }
    classes_str += "]";
    set_output("classes", Data(classes_str));
    
    set_output("detection_count", Data(static_cast<int>(boxes.size())));
    
    OVF_INFO() << "Detection completed: " << boxes.size() << " objects detected";
    
    return Result<void>::success();
}

void DetectionNode::apply_nms(Vector<Region>& boxes, Vector<float>& scores, 
                              Vector<int>& classes, float conf_threshold, 
                              float nms_threshold) {
    // 过滤低置信度检测
    Vector<Region> filtered_boxes;
    Vector<float> filtered_scores;
    Vector<int> filtered_classes;
    
    for (size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] >= conf_threshold) {
            filtered_boxes.push_back(boxes[i]);
            filtered_scores.push_back(scores[i]);
            filtered_classes.push_back(classes[i]);
        }
    }
    
    boxes = filtered_boxes;
    scores = filtered_scores;
    classes = filtered_classes;
    
    // 简化的NMS实现
    Vector<bool> suppressed(boxes.size(), false);
    
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (suppressed[i]) continue;
        
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (suppressed[j]) continue;
            if (classes[i] != classes[j]) continue;  // 只对同类进行NMS
            
            // 计算IoU（简化）
            int x1 = std::max(boxes[i].x, boxes[j].x);
            int y1 = std::max(boxes[i].y, boxes[j].y);
            int x2 = std::min(boxes[i].x + boxes[i].width, boxes[j].x + boxes[j].width);
            int y2 = std::min(boxes[i].y + boxes[i].height, boxes[j].y + boxes[j].height);
            
            if (x2 > x1 && y2 > y1) {
                int intersection = (x2 - x1) * (y2 - y1);
                int area_i = boxes[i].width * boxes[i].height;
                int area_j = boxes[j].width * boxes[j].height;
                int union_area = area_i + area_j - intersection;
                
                float iou = static_cast<float>(intersection) / union_area;
                
                if (iou > nms_threshold) {
                    if (scores[i] > scores[j]) {
                        suppressed[j] = true;
                    } else {
                        suppressed[i] = true;
                        break;
                    }
                }
            }
        }
    }
    
    // 移除被抑制的检测
    Vector<Region> nms_boxes;
    Vector<float> nms_scores;
    Vector<int> nms_classes;
    
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (!suppressed[i]) {
            nms_boxes.push_back(boxes[i]);
            nms_scores.push_back(scores[i]);
            nms_classes.push_back(classes[i]);
        }
    }
    
    boxes = nms_boxes;
    scores = nms_scores;
    classes = nms_classes;
}

// ClassificationNode 实现

ClassificationNode::ClassificationNode(const String& instance_id)
    : INode(instance_id, make_info())
    , initialized_(false) {
}

NodeInfo ClassificationNode::make_info() {
    NodeInfo info;
    info.id = "Classification";
    info.name = "图像分类";
    info.category = "深度学习";
    info.description = "图像分类推理节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("class_id", "类别ID", DataType::Number));
    info.outputs.push_back(DataPort("class_name", "类别名称", DataType::String));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));
    info.outputs.push_back(DataPort("top_k_classes", "Top-K类别", DataType::Array));
    
    info.params.push_back(ParamDef("model_path", "模型路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("top_k", "Top-K", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("label_file", "标签文件", DataType::String, Data("")));
    info.params.push_back(ParamDef("execution_provider", "执行提供器", DataType::String, Data("CPU")));
    
    return info;
}

Result<void> ClassificationNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    String label_file = get_param("label_file", Data("")).as_string();
    String provider = get_param("execution_provider", Data("CPU")).as_string();
    
    if (!model_path.empty()) {
        env_.set_execution_provider(provider);
        ErrorCode err = env_.load_model(model_path);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "Failed to load classification model");
        }
        initialized_ = true;
    }
    
    // 加载标签文件
    if (!label_file.empty()) {
        if (!load_labels(label_file)) {
            OVF_WARN() << "Failed to load label file: " << label_file;
        }
    }
    
    // 如果没有加载标签，使用默认标签
    if (labels_.empty()) {
        for (int i = 0; i < 1000; ++i) {
            labels_.push_back("class_" + std::to_string(i));
        }
    }
    
    return Result<void>::success();
}

bool ClassificationNode::load_labels(const String& label_file) {
    std::ifstream file(label_file);
    if (!file.is_open()) {
        return false;
    }
    
    labels_.clear();
    String line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            labels_.push_back(line);
        }
    }
    
    OVF_INFO() << "Loaded " << labels_.size() << " labels from " << label_file;
    return true;
}

Result<void> ClassificationNode::execute(FlowContext& context) {
    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData img = img_data.as_image();
    int top_k = get_param("top_k", Data(5)).as_int();
    
    // 准备输入数据
    Vector<float> input_data;
    input_data.resize(img.data.size());
    for (size_t i = 0; i < img.data.size(); ++i) {
        input_data[i] = static_cast<float>(img.data[i]) / 255.0f;
    }
    
    // 执行推理
    onnx_utils::InferenceResult result;
    ErrorCode err = env_.run_inference(input_data, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Classification inference failed");
    }
    
    // 找到Top-K类别
    Vector<std::pair<float, int>> scored_classes;
    for (size_t i = 0; i < result.output_data.size() && i < labels_.size(); ++i) {
        scored_classes.emplace_back(result.output_data[i], static_cast<int>(i));
    }
    
    // 按置信度排序
    std::sort(scored_classes.begin(), scored_classes.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // 取Top-K
    top_k = std::min(top_k, static_cast<int>(scored_classes.size()));
    
    // 设置输出
    if (top_k > 0) {
        int best_class = scored_classes[0].second;
        float best_score = scored_classes[0].first;
        
        set_output("class_id", Data(best_class));
        set_output("class_name", Data(labels_[best_class]));
        set_output("confidence", Data(best_score));
        
        // Top-K类别
        String top_k_str = "[";
        for (int i = 0; i < top_k; ++i) {
            int cls = scored_classes[i].second;
            top_k_str += labels_[cls] + "(" + std::to_string(scored_classes[i].first) + ")";
            if (i < top_k - 1) top_k_str += ", ";
        }
        top_k_str += "]";
        set_output("top_k_classes", Data(top_k_str));
        
        OVF_INFO() << "Classification: " << labels_[best_class] 
                   << " (class_id=" << best_class << ", confidence=" << best_score << ")";
    }
    
    return Result<void>::success();
}

// SegmentationNode 实现

SegmentationNode::SegmentationNode(const String& instance_id)
    : INode(instance_id, make_info())
    , initialized_(false) {
}

NodeInfo SegmentationNode::make_info() {
    NodeInfo info;
    info.id = "Segmentation";
    info.name = "图像分割";
    info.category = "深度学习";
    info.description = "语义分割/实例分割推理节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    info.outputs.push_back(DataPort("mask", "分割掩码", DataType::Image));
    info.outputs.push_back(DataPort("class_map", "类别映射", DataType::Object));
    info.outputs.push_back(DataPort("segmentation_image", "分割图像", DataType::Image));
    
    info.params.push_back(ParamDef("model_path", "模型路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("output_type", "输出类型", DataType::String, Data("semantic")));
    info.params.push_back(ParamDef("execution_provider", "执行提供器", DataType::String, Data("CPU")));
    
    return info;
}

Result<void> SegmentationNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    String provider = get_param("execution_provider", Data("CPU")).as_string();
    
    if (!model_path.empty()) {
        env_.set_execution_provider(provider);
        ErrorCode err = env_.load_model(model_path);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "Failed to load segmentation model");
        }
        initialized_ = true;
    }
    
    return Result<void>::success();
}

Result<void> SegmentationNode::execute(FlowContext& context) {
    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData img = img_data.as_image();
    
    // 准备输入数据
    Vector<float> input_data;
    input_data.resize(img.data.size());
    for (size_t i = 0; i < img.data.size(); ++i) {
        input_data[i] = static_cast<float>(img.data[i]) / 255.0f;
    }
    
    // 执行推理
    onnx_utils::InferenceResult result;
    ErrorCode err = env_.run_inference(input_data, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(ErrorCode::AlgorithmExecFailed, "Segmentation inference failed");
    }
    
    // 处理分割输出
    ImageData mask;
    HashMap<int, int> class_map;
    process_segmentation_output(result.output_data, result.output_shape, mask, class_map);
    
    // 设置掩码输出
    set_output("mask", Data(mask));
    
    // 类别映射（转换为字符串描述）
    String class_map_str = "{";
    for (const auto& pair : class_map) {
        class_map_str += std::to_string(pair.first) + ": " + std::to_string(pair.second) + ", ";
    }
    class_map_str += "}";
    set_output("class_map", Data(class_map_str));
    
    // 创建分割可视化图像
    ImageData seg_image;
    seg_image.width = img.width;
    seg_image.height = img.height;
    seg_image.channels = 3;
    seg_image.format = ImageFormat::RGB8;
    seg_image.data.resize(img.width * img.height * 3);
    
    // 为每个类别分配颜色
    HashMap<int, std::tuple<uint8_t, uint8_t, uint8_t>> class_colors;
    std::mt19937 color_rng(42);
    std::uniform_int_distribution<int> color_dist(0, 255);
    
    for (const auto& pair : class_map) {
        class_colors[pair.first] = std::make_tuple(
            static_cast<uint8_t>(color_dist(color_rng)),
            static_cast<uint8_t>(color_dist(color_rng)),
            static_cast<uint8_t>(color_dist(color_rng))
        );
    }
    
    // 应用颜色映射
    for (size_t i = 0; i < mask.data.size(); ++i) {
        int class_id = mask.data[i];
        auto it = class_colors.find(class_id);
        if (it != class_colors.end()) {
            auto [r, g, b] = it->second;
            seg_image.data[i * 3] = r;
            seg_image.data[i * 3 + 1] = g;
            seg_image.data[i * 3 + 2] = b;
        }
    }
    
    set_output("segmentation_image", Data(seg_image));
    
    OVF_INFO() << "Segmentation completed: " << class_map.size() << " classes detected";
    
    return Result<void>::success();
}

void SegmentationNode::process_segmentation_output(const Vector<float>& raw_output, 
                                                    const Vector<int>& shape,
                                                    ImageData& mask,
                                                    HashMap<int, int>& class_map) {
    // 确定输出形状（假设是HxW或NxCxHxW）
    int height = 0;
    int width = 0;
    
    if (shape.size() == 2) {
        height = shape[0];
        width = shape[1];
    } else if (shape.size() == 3) {
        height = shape[1];
        width = shape[2];
    } else if (shape.size() == 4) {
        height = shape[2];
        width = shape[3];
    } else {
        // 使用默认值
        height = 224;
        width = 224;
    }
    
    mask.width = width;
    mask.height = height;
    mask.channels = 1;
    mask.format = ImageFormat::Mono8;
    mask.data.resize(width * height);
    
    // 转换输出为掩码
    size_t output_size = width * height;
    for (size_t i = 0; i < output_size && i < raw_output.size(); ++i) {
        // 找到最大值的类别
        int class_id = static_cast<int>(raw_output[i] * 10) % 10;  // 简化处理
        mask.data[i] = static_cast<uint8_t>(class_id);
        class_map[class_id]++;
    }
}

// 注册节点
OVF_REGISTER_NODE(ONNXInferenceNode, "ONNXInference", ONNXInferenceNode::make_info());
OVF_REGISTER_NODE(DetectionNode, "Detection", DetectionNode::make_info());
OVF_REGISTER_NODE(ClassificationNode, "Classification", ClassificationNode::make_info());
OVF_REGISTER_NODE(SegmentationNode, "Segmentation", SegmentationNode::make_info());

} // namespace algorithm
} // namespace ovf