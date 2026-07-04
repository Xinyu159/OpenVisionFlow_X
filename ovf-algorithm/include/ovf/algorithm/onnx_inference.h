/**
 * @file onnx_inference.h
 * @brief ONNX推理模块 - 提供深度学习模型推理功能
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/types.h"
#include <chrono>

namespace ovf {
namespace algorithm {
namespace onnx_utils {

/**
 * @brief 推理结果结构
 */
struct InferenceResult {
    Vector<float> output_data;      // 输出数据
    Vector<int> output_shape;       // 输出形状
    String output_name;             // 输出名称
    float inference_time_ms;        // 推理时间（毫秒）
};

/**
 * @brief 模型信息结构
 */
struct ModelInfo {
    Vector<String> input_names;     // 输入名称列表
    Vector<String> output_names;    // 输出名称列表
    Vector<Vector<int>> input_shapes;  // 输入形状
    Vector<Vector<int>> output_shapes; // 输出形状
    String model_name;
    int64_t model_size;
};

/**
 * @brief ONNX推理环境 - 管理模型加载和推理
 */
class ONNXEnvironment {
public:
    ONNXEnvironment();
    ~ONNXEnvironment();
    
    // 加载模型
    ErrorCode load_model(const String& model_path);
    
    // 获取模型信息
    ErrorCode get_model_info(ModelInfo& info);
    
    // 推理
    ErrorCode run_inference(const Vector<float>& input_data, InferenceResult& result);
    ErrorCode run_inference_multi(const Vector<Vector<float>>& inputs, Vector<InferenceResult>& results);
    
    // 参数设置
    void set_execution_provider(const String& provider); // "CPU", "CUDA", "OpenCL"
    void set_inference_threads(int threads);
    
    // 状态查询
    bool is_model_loaded() const { return model_loaded_; }
    const String& model_path() const { return model_path_; }
    
private:
    String model_path_;
    String execution_provider_;
    int inference_threads_;
    bool model_loaded_;
    ModelInfo model_info_;
    
    // 模拟推理（实际使用时动态加载ONNX Runtime DLL）
    void simulate_inference(const Vector<float>& input, InferenceResult& output);
};

} // namespace onnx_utils

/**
 * @brief ONNX推理节点 - 通用深度学习推理节点
 * 
 * 输入: image 或 input_data
 * 输出: output_data, output_shape, inference_time
 * 参数: model_path, input_name, output_name, execution_provider
 */
class ONNXInferenceNode : public INode {
public:
    ONNXInferenceNode(const String& instance_id);
    ~ONNXInferenceNode() override = default;
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    onnx_utils::ONNXEnvironment env_;
    bool initialized_;
};

/**
 * @brief 目标检测节点 - 用于目标检测模型推理
 * 
 * 输入: image
 * 输出: boxes, scores, classes, detection_count
 * 参数: model_path, confidence_threshold, nms_threshold
 */
class DetectionNode : public INode {
public:
    DetectionNode(const String& instance_id);
    ~DetectionNode() override = default;
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    onnx_utils::ONNXEnvironment env_;
    bool initialized_;
    
    // NMS后处理
    void apply_nms(Vector<Region>& boxes, Vector<float>& scores, 
                   Vector<int>& classes, float conf_threshold, float nms_threshold);
};

/**
 * @brief 分类节点 - 用于图像分类模型推理
 * 
 * 输入: image
 * 输出: class_id, class_name, confidence, top_k_classes
 * 参数: model_path, top_k, label_file
 */
class ClassificationNode : public INode {
public:
    ClassificationNode(const String& instance_id);
    ~ClassificationNode() override = default;
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    onnx_utils::ONNXEnvironment env_;
    bool initialized_;
    Vector<String> labels_;  // 类别标签
    
    // 加载标签文件
    bool load_labels(const String& label_file);
};

/**
 * @brief 分割节点 - 用于语义分割/实例分割模型推理
 * 
 * 输入: image
 * 输出: mask, class_map, segmentation_image
 * 参数: model_path, output_type
 */
class SegmentationNode : public INode {
public:
    SegmentationNode(const String& instance_id);
    ~SegmentationNode() override = default;
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    onnx_utils::ONNXEnvironment env_;
    bool initialized_;
    
    // 后处理分割结果
    void process_segmentation_output(const Vector<float>& raw_output, 
                                     const Vector<int>& shape,
                                     ImageData& mask,
                                     HashMap<int, int>& class_map);
};

} // namespace algorithm
} // namespace ovf