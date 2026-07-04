/**
 * @file dl_training.h
 * @brief 深度学习训练闭环工具模块（完整版）
 * 
 * 包含25个节点:
 * - 数据标注：ImageAnnotateNode, LabelExportNode, LabelMergeNode, LabelVerifyNode
 * - 数据增强：DataAugmentNode, RandomCropNode, ColorAugmentNode, MixupNode
 * - 数据管理增强：DatasetManagerNode, DataLabelingNode, DataValidationNode
 * - 训练工具：DatasetSplitNode, TrainConfigNode
 * - 训练增强：TrainProgressNode, TrainCheckpointNode
 * - 验证增强：ModelValidationNode, ConfusionMatrixNode, ValidationReportNode
 * - 部署增强：ModelExportNode, ModelOptimizeNode, ModelDeployNode
 * - 完整流程：DeepLearningWorkflowNode
 * - 原有评估：ModelEvaluateNode, ExportONNXNode
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <random>

namespace ovf {
namespace algorithm {

// ==================== 标注数据结构 ====================

/**
 * @brief 标注类型
 */
enum class AnnotationType {
    Rectangle,      // 矩形框
    Polygon,        // 多边形
    Point,          // 点标注
    Line,           // 线标注
    Keypoint        // 关键点
};

/**
 * @brief 单个标注对象
 */
struct AnnotationObject {
    String label;                   // 标签名称
    AnnotationType type;            // 标注类型
    
    // 矩形标注数据
    int32_t x = 0;                  // 矩形左上角X
    int32_t y = 0;                  // 矩形左上角Y
    int32_t width = 0;              // 矩形宽度
    int32_t height = 0;             // 矩形高度
    
    // 多边形标注数据
    Vector<Point2D<int32_t>> points; // 多边形顶点
    
    // 属性
    float confidence = 1.0f;        // 置信度
    int32_t class_id = 0;           // 类别ID
    bool occluded = false;          // 是否遮挡
    bool difficult = false;         // 是否困难样本
    
    // 元数据
    String attributes;              // 自定义属性（JSON格式）
};

/**
 * @brief 图像标注数据
 */
struct ImageAnnotation {
    String image_path;              // 图像路径
    String image_id;                // 图像ID
    uint32_t width = 0;             // 图像宽度
    uint32_t height = 0;            // 图像高度
    Vector<AnnotationObject> objects; // 标注对象列表
    String description;             // 描述信息
    
    // 转换为JSON字符串
    String to_json() const;
    
    // 从JSON字符串解析
    static ImageAnnotation from_json(const String& json_str);
};

/**
 * @brief 标注文件格式
 */
enum class LabelFormat {
    YOLO,           // YOLO格式（txt）
    COCO,           // COCO格式（JSON）
    VOC,            // Pascal VOC格式（XML）
    LabelMe,        // LabelMe格式（JSON）
    Custom          // 自定义格式
};

// 自由函数：解析标注JSON字符串
Vector<ImageAnnotation> parse_annotations_json(const String& json_str);

// ==================== 数据标注节点 ====================

/**
 * @brief 图像标注节点
 * 
 * 功能：在图像上创建标注（矩形框、多边形等）
 * 
 * 输入：
 *   - image: 输入图像
 *   - roi: 感兴趣区域（可选）
 * 
 * 输出：
 *   - annotation: 标注数据
 *   - annotated_image: 带标注显示的图像
 * 
 * 参数：
 *   - label: 标签名称
 *   - annotation_type: 标注类型 (rectangle/polygon/point/line)
 *   - color: 标注颜色
 *   - show_label: 是否显示标签名称
 */
class ImageAnnotateNode : public INode {
public:
    explicit ImageAnnotateNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    AnnotationType parse_annotation_type(const String& type_str);
    void draw_annotation(ImageData& image, const AnnotationObject& obj, 
                         uint8_t r, uint8_t g, uint8_t b, bool show_label);
};

/**
 * @brief 标签导出节点
 * 
 * 功能：将标注数据导出为指定格式
 * 
 * 输入：
 *   - annotations: 标注数据列表（JSON数组格式）
 *   - image_paths: 图像路径列表（可选）
 * 
 * 输出：
 *   - export_path: 导出文件路径
 *   - export_count: 导出数量
 *   - export_data: 导出的数据内容
 * 
 * 参数：
 *   - format: 导出格式 (YOLO/COCO/VOC/LabelMe)
 *   - output_dir: 输出目录
 *   - filename: 输出文件名
 *   - include_images: 是否包含图像信息
 */
class LabelExportNode : public INode {
public:
    explicit LabelExportNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String export_to_yolo(const Vector<ImageAnnotation>& annotations);
    String export_to_coco(const Vector<ImageAnnotation>& annotations);
    String export_to_voc(const ImageAnnotation& annotation);
    String export_to_labelme(const ImageAnnotation& annotation);
};

/**
 * @brief 标签合并节点
 * 
 * 功能：合并多个标注数据源
 * 
 * 输入：
 *   - annotations1: 标注数据1
 *   - annotations2: 标注数据2
 *   - annotations3: 标注数据3（可选）
 * 
 * 输出：
 *   - merged_annotations: 合后的标注数据
 *   - total_count: 总标注数量
 *   - conflict_count: 冲突数量
 * 
 * 参数：
 *   - merge_mode: 合并模式 (union/intersection/difference)
 *   - resolve_conflict: 冲突解决策略 (keep_first/keep_last/merge_all)
 *   - min_overlap: 最小重叠阈值（用于检测冲突）
 */
class LabelMergeNode : public INode {
public:
    explicit LabelMergeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    float calculate_overlap(const AnnotationObject& a, const AnnotationObject& b);
    bool is_conflict(const AnnotationObject& a, const AnnotationObject& b, float min_overlap);
};

/**
 * @brief 标签验证节点
 * 
 * 功能：验证标注数据的完整性和正确性
 * 
 * 输入：
 *   - annotations: 标注数据
 *   - reference_image: 参考图像（可选）
 * 
 * 输出：
 *   - valid: 是否有效
 *   - error_count: 错误数量
 *   - warning_count: 警告数量
 *   - validation_report: 验证报告
 *   - corrected_annotations: 修正后的标注（可选）
 * 
 * 参数：
 *   - check_bounds: 检查边界越界
 *   - check_duplicate: 检查重复标注
 *   - check_empty: 检查空标注
 *   - auto_correct: 自动修正错误
 */
class LabelVerifyNode : public INode {
public:
    explicit LabelVerifyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

// ==================== 数据增强节点 ====================

/**
 * @brief 数据增强节点
 * 
 * 功能：通用数据增强（旋转、翻转、缩放、裁剪）
 * 
 * 输入：
 *   - image: 输入图像
 *   - annotation: 标注数据（可选）
 * 
 * 输出：
 *   - augmented_image: 增强后的图像
 *   - augmented_annotation: 增强后的标注
 *   - transform_params: 变换参数
 * 
 * 参数：
 *   - augment_type: 增强类型 (rotate/flip/scale/crop/combined)
 *   - rotate_angle: 旋转角度范围（min,max）
 *   - flip_direction: 翻转方向 (horizontal/vertical/both)
 *   - scale_range: 缩放范围（min,max）
 *   - crop_ratio: 裁剪比例
 *   - random_seed: 随机种子
 */
class DataAugmentNode : public INode {
public:
    explicit DataAugmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    void apply_rotation(ImageData& image, AnnotationObject& obj, float angle);
    void apply_flip(ImageData& image, AnnotationObject& obj, bool horizontal, bool vertical);
    void apply_scale(ImageData& image, AnnotationObject& obj, float scale);
    void apply_crop(ImageData& image, AnnotationObject& obj, int crop_x, int crop_y, int crop_w, int crop_h);
};

/**
 * @brief 随机裁剪增强节点
 * 
 * 功能：随机裁剪图像并调整标注
 * 
 * 输入：
 *   - image: 输入图像
 *   - annotation: 标注数据（可选）
 * 
 * 输出：
 *   - cropped_image: 裁剪后的图像
 *   - cropped_annotation: 裁剪后的标注
 *   - crop_region: 裁剪区域
 * 
 * 参数：
 *   - crop_width: 裁剪宽度（0表示随机）
 *   - crop_height: 裁剪高度（0表示随机）
 *   - crop_ratio_min: 最小裁剪比例
 *   - crop_ratio_max: 最大裁剪比例
 *   - ensure_object: 确保裁剪区域包含标注对象
 *   - num_outputs: 输出数量（多裁剪）
 */
class RandomCropNode : public INode {
public:
    explicit RandomCropNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    Region generate_random_crop(int img_w, int img_h, int crop_w, int crop_h);
    bool region_contains_object(const Region& region, const AnnotationObject& obj);
    AnnotationObject adjust_annotation_for_crop(const AnnotationObject& obj, const Region& crop);
};

/**
 * @brief 颜色增强节点
 * 
 * 功能：颜色增强（亮度、对比度、饱和度）
 * 
 * 输入：
 *   - image: 输入图像
 * 
 * 输出：
 *   - augmented_image: 增强后的图像
 *   - color_params: 颜色参数
 * 
 * 参数：
 *   - brightness_range: 亮度调整范围（-0.5~0.5）
 *   - contrast_range: 对比度调整范围（0.5~2.0）
 *   - saturation_range: 饱和度调整范围（0.5~2.0）
 *   - hue_range: 色相调整范围（-0.5~0.5）
 *   - random_apply: 随机应用
 */
class ColorAugmentNode : public INode {
public:
    explicit ColorAugmentNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    void adjust_brightness(ImageData& image, float factor);
    void adjust_contrast(ImageData& image, float factor);
    void adjust_saturation(ImageData& image, float factor);
    void adjust_hue(ImageData& image, float shift);
};

/**
 * @brief Mixup增强节点
 * 
 * 功能：图像混合增强（两张图像加权混合）
 * 
 * 输入：
 *   - image1: 输入图像1
 *   - annotation1: 标注数据1（可选）
 *   - image2: 输入图像2
 *   - annotation2: 标注数据2（可选）
 * 
 * 输出：
 *   - mixed_image: 混合后的图像
 *   - mixed_annotation: 混合后的标注
 *   - mix_ratio: 混合比例
 * 
 * 参数：
 *   - mix_ratio: 混合比例（0.0~1.0，0表示随机）
 *   - blend_mode: 混合模式 (linear/add/multiply)
 *   - random_ratio: 随机比例范围（min,max）
 */
class MixupNode : public INode {
public:
    explicit MixupNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    void blend_linear(ImageData& dst, const ImageData& src1, const ImageData& src2, float ratio);
    void blend_add(ImageData& dst, const ImageData& src1, const ImageData& src2, float ratio);
    void blend_multiply(ImageData& dst, const ImageData& src1, const ImageData& src2, float ratio);
};

// ==================== 训练工具节点 ====================

/**
 * @brief 数据集分割节点
 * 
 * 功能：将数据集分割为训练/验证/测试集
 * 
 * 输入：
 *   - data_list: 数据列表（图像路径或ID）
 *   - annotations: 标注数据（可选）
 * 
 * 输出：
 *   - train_set: 训练集
 *   - val_set: 验证集
 *   - test_set: 测试集
 *   - split_report: 分割报告
 * 
 * 参数：
 *   - train_ratio: 训练集比例
 *   - val_ratio: 验证集比例
 *   - test_ratio: 测试集比例
 *   - stratified: 是否分层采样
 *   - random_seed: 随机种子
 *   - output_format: 输出格式 (list/json/file)
 */
class DatasetSplitNode : public INode {
public:
    explicit DatasetSplitNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    void stratified_split(const Vector<String>& data, const Vector<String>& labels,
                          Vector<String>& train, Vector<String>& val, Vector<String>& test,
                          float train_ratio, float val_ratio, float test_ratio);
    void random_split(const Vector<String>& data,
                      Vector<String>& train, Vector<String>& val, Vector<String>& test,
                      float train_ratio, float val_ratio, float test_ratio);
};

/**
 * @brief 训练配置节点
 * 
 * 功能：生成和管理训练配置
 * 
 * 输入：
 *   - model_type: 模型类型（可选）
 *   - dataset_info: 数据集信息（可选）
 * 
 * 输出：
 *   - config: 配置内容
 *   - config_path: 配置文件路径
 * 
 * 参数：
 *   - learning_rate: 学习率
 *   - batch_size: 批次大小
 *   - epochs: 训练轮次
 *   - optimizer: 优化器 (SGD/Adam/AdamW/RMSprop)
 *   - lr_scheduler: 学习率调度器 (step/cosine/linear/constant)
 *   - weight_decay: 权重衰减
 *   - momentum: 动量（SGD）
 *   - dropout: Dropout比例
 *   - pretrained: 是否使用预训练模型
 *   - config_format: 配置格式 (json/yaml/py)
 */
class TrainConfigNode : public INode {
public:
    explicit TrainConfigNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String generate_json_config();
    String generate_yaml_config();
    String generate_py_config();
};

/**
 * @brief 模型评估节点
 * 
 * 功能：评估模型性能（精度、召回率、F1等）
 * 
 * 输入：
 *   - predictions: 预测结果
 *   - ground_truth: 真实标签
 *   - num_classes: 类别数量
 * 
 * 输出：
 *   - precision: 精度
 *   - recall: 召回率
 *   - f1_score: F1分数
 *   - accuracy: 准确率
 *   - confusion_matrix: 混淆矩阵
 *   - iou: IoU分数（检测任务）
 *   - evaluation_report: 详细评估报告
 * 
 * 参数：
 *   - task_type: 任务类型 (classification/detection/segmentation)
 *   - iou_threshold: IoU阈值（检测任务）
 *   - confidence_threshold: 置信度阈值
 *   - average_method: 平均方法 (micro/macro/weighted)
 */
class ModelEvaluateNode : public INode {
public:
    explicit ModelEvaluateNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    void calculate_classification_metrics(const Vector<int>& pred, const Vector<int>& gt,
                                          int num_classes, float& precision, float& recall,
                                          float& f1, float& accuracy, Vector<int>& confusion_matrix);
    float calculate_iou(const AnnotationObject& pred, const AnnotationObject& gt);
    float calculate_iou_boxes(int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2);
};

/**
 * @brief ONNX导出节点
 * 
 * 功能：模型格式转换导出
 * 
 * 输入：
 *   - model_info: 模型信息（JSON格式）
 *   - weights_path: 权重文件路径（可选）
 * 
 * 输出：
 *   - export_path: 导出文件路径
 *   - export_status: 导出状态
 *   - model_info: 导出模型信息
 * 
 * 参数：
 *   - input_shape: 输入形状（如"1,3,224,224"）
 *   - output_names: 输出节点名称
 *   - opset_version: ONNX opset版本
 *   - dynamic_batch: 是否动态批次
 *   - optimize: 是否优化模型
 *   - output_path: 输出路径
 *   - model_name: 模型名称
 */
class ExportONNXNode : public INode {
public:
    explicit ExportONNXNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String generate_onnx_export_script();
    String generate_model_info_json();
    Vector<int> parse_shape(const String& shape_str);
};

// ==================== 数据管理增强节点 ====================

/**
 * @brief 数据集管理节点
 * 
 * 功能：管理数据集（划分训练/验证/测试集，统计数据分布）
 * 
 * 输入：
 *   - dataset_path: 数据集路径
 *   - annotations: 标注数据（可选）
 * 
 * 输出：
 *   - train_set: 训练集列表
 *   - val_set: 验证集列表
 *   - test_set: 测试集列表
 *   - dataset_stats: 数据集统计信息
 *   - class_distribution: 类别分布
 * 
 * 参数：
 *   - split_strategy: 划分策略（random/stratified/kfold）
 *   - train_ratio: 训练集比例
 *   - val_ratio: 验证集比例
 *   - test_ratio: 测试集比例
 *   - k_fold: K折交叉验证的K值
 *   - random_seed: 随机种子
 */
class DatasetManagerNode : public INode {
public:
    explicit DatasetManagerNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    Vector<String> load_image_paths(const String& dataset_path);
    HashMap<String, int> analyze_class_distribution(const Vector<ImageAnnotation>& annotations);
    void stratified_split(const Vector<String>& data, const Vector<String>& labels,
                          Vector<String>& train, Vector<String>& val, Vector<String>& test,
                          float train_ratio, float val_ratio, float test_ratio);
};

/**
 * @brief 数据标注工具节点
 * 
 * 功能：提供标注工具（矩形框、多边形、语义分割标注）
 * 
 * 输入：
 *   - image: 输入图像
 *   - preset_labels: 预设标签列表（可选）
 * 
 * 输出：
 *   - annotation: 标注数据
 *   - annotated_image: 带标注的图像
 *   - annotation_stats: 标注统计
 * 
 * 参数：
 *   - annotation_mode: 标注模式（rectangle/polygon/segmentation/keypoint）
 *   - auto_save: 自动保存
 *   - save_format: 保存格式（YOLO/VOC/COCO）
 *   - output_dir: 输出目录
 */
class DataLabelingNode : public INode {
public:
    explicit DataLabelingNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    void draw_rectangle_annotation(ImageData& image, const AnnotationObject& obj);
    void draw_polygon_annotation(ImageData& image, const AnnotationObject& obj);
    void draw_segmentation_mask(ImageData& image, const Vector<Point2D<int32_t>>& mask_points, uint8_t r, uint8_t g, uint8_t b);
};

/**
 * @brief 数据验证节点
 * 
 * 功能：检查标注质量、数据分布、数据完整性
 * 
 * 输入：
 *   - dataset_path: 数据集路径
 *   - annotations: 标注数据
 * 
 * 输出：
 *   - validation_report: 验证报告
 *   - quality_score: 质量评分
 *   - issues: 问题列表
 *   - recommendations: 改进建议
 * 
 * 参数：
 *   - check_duplicate: 检查重复标注
 *   - check_missing: 检查缺失标注
 *   - check_outliers: 检查异常数据
 *   - min_quality_score: 最小质量评分阈值
 */
class DataValidationNode : public INode {
public:
    explicit DataValidationNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    float calculate_quality_score(const Vector<ImageAnnotation>& annotations);
    Vector<String> detect_duplicates(const Vector<ImageAnnotation>& annotations);
    Vector<String> detect_missing_annotations(const Vector<String>& image_paths, const Vector<ImageAnnotation>& annotations);
    Vector<String> detect_outliers(const Vector<ImageAnnotation>& annotations);
};

// ==================== 训练增强节点 ====================

/**
 * @brief 训练进度监控节点
 * 
 * 功能：TensorBoard风格日志，监控损失曲线、指标曲线、直方图
 * 
 * 输入：
 *   - training_state: 训练状态（JSON）
 *   - metrics: 当前指标
 * 
 * 输出：
 *   - log_path: 日志文件路径
 *   - visualization_data: 可视化数据
 *   - progress_report: 进度报告
 * 
 * 参数：
 *   - log_dir: 日志目录
 *   - log_interval: 记录间隔（步数）
 *   - save_histogram: 是否保存参数直方图
 *   - save_graph: 是否保存计算图
 */
class TrainProgressNode : public INode {
public:
    explicit TrainProgressNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String write_tensorboard_log(const String& log_dir, int epoch, float loss, const HashMap<String, float>& metrics);
    String generate_loss_curve(const Vector<float>& losses);
    String generate_metric_curve(const String& metric_name, const Vector<float>& values);
    String generate_histogram_data(const Vector<float>& values, int bins);
};

/**
 * @brief 训练检查点节点
 * 
 * 功能：保存/恢复训练状态，支持断点续训
 * 
 * 输入：
 *   - model_state: 模型状态
 *   - optimizer_state: 优化器状态
 *   - training_config: 训练配置
 * 
 * 输出：
 *   - checkpoint_path: 检查点路径
 *   - checkpoint_info: 检查点信息
 *   - restore_info: 恢复信息（如果恢复）
 * 
 * 参数：
 *   - checkpoint_dir: 检查点目录
 *   - save_interval: 保存间隔（轮次）
 *   - max_checkpoints: 最大检查点数量
 *   - save_best: 是否保存最佳模型
 *   - restore_from: 从指定检查点恢复
 */
class TrainCheckpointNode : public INode {
public:
    explicit TrainCheckpointNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String save_checkpoint(const String& checkpoint_dir, int epoch, const String& model_state, const String& optimizer_state);
    String load_checkpoint(const String& checkpoint_path);
    Vector<String> list_checkpoints(const String& checkpoint_dir);
    void cleanup_old_checkpoints(const String& checkpoint_dir, int max_count);
};

// ==================== 验证增强节点 ====================

/**
 * @brief 模型验证节点（增强版）
 * 
 * 功能：计算mAP、IoU、准确率等详细指标
 * 
 * 输入：
 *   - predictions: 预测结果
 *   - ground_truth: 真实标签
 *   - num_classes: 类别数量
 * 
 * 输出：
 *   - mAP: 平均精度（目标检测）
 *   - mAP_50: IoU=0.5时的mAP
 *   - mAP_75: IoU=0.75时的mAP
 *   - per_class_ap: 每个类别的AP
 *   - iou_scores: IoU分数列表
 *   - precision_recall_curve: PR曲线数据
 * 
 * 参数：
 *   - task_type: 任务类型（detection/segmentation/classification）
 *   - iou_thresholds: IoU阈值列表
 *   - confidence_threshold: 置信度阈值
 */
class ModelValidationNode : public INode {
public:
    explicit ModelValidationNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    float calculate_ap(const Vector<float>& precisions, const Vector<float>& recalls);
    float calculate_map(const Vector<ImageAnnotation>& predictions, const Vector<ImageAnnotation>& ground_truths, float iou_threshold);
    Vector<float> calculate_precision_recall_curve(const Vector<ImageAnnotation>& predictions, const Vector<ImageAnnotation>& ground_truths, float iou_threshold);
    HashMap<String, float> calculate_per_class_ap(const Vector<ImageAnnotation>& predictions, const Vector<ImageAnnotation>& ground_truths, float iou_threshold);
};

/**
 * @brief 混淆矩阵生成节点
 * 
 * 功能：生成并可视化混淆矩阵
 * 
 * 输入：
 *   - predictions: 预测结果
 *   - ground_truth: 真实标签
 *   - class_names: 类别名称列表
 * 
 * 输出：
 *   - confusion_matrix: 混淆矩阵数据
 *   - normalized_matrix: 归一化混淆矩阵
 *   - visualization: 可视化图像（CSV/图像）
 *   - error_analysis: 错误分析报告
 * 
 * 参数：
 *   - normalize: 是否归一化
 *   - output_format: 输出格式（json/csv/image）
 *   - color_scheme: 颜色方案
 */
class ConfusionMatrixNode : public INode {
public:
    explicit ConfusionMatrixNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    Vector<int> build_confusion_matrix(const Vector<int>& predictions, const Vector<int>& ground_truth, int num_classes);
    Vector<float> normalize_confusion_matrix(const Vector<int>& matrix, int num_classes);
    String generate_matrix_csv(const Vector<int>& matrix, const Vector<String>& class_names);
    String generate_error_analysis(const Vector<int>& matrix, const Vector<String>& class_names);
};

/**
 * @brief 验证报告生成节点
 * 
 * 功能：生成详细的验证报告（PDF/HTML/JSON）
 * 
 * 输入：
 *   - validation_results: 验证结果
 *   - confusion_matrix: 混淆矩阵
 *   - model_info: 模型信息
 * 
 * 输出：
 *   - report_path: 报告路径
 *   - report_content: 报告内容
 *   - summary: 摘要信息
 * 
 * 参数：
 *   - report_format: 报告格式（html/json/pdf）
 *   - include_visualizations: 包含可视化
 *   - include_recommendations: 包含改进建议
 *   - output_dir: 输出目录
 */
class ValidationReportNode : public INode {
public:
    explicit ValidationReportNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String generate_html_report(const HashMap<String, Data>& validation_results, const String& model_info);
    String generate_json_report(const HashMap<String, Data>& validation_results, const String& model_info);
    String generate_summary(const HashMap<String, Data>& validation_results);
};

// ==================== 部署增强节点 ====================

/**
 * @brief 模型导出节点（多格式）
 * 
 * 功能：导出ONNX/TensorRT/OpenVINO格式
 * 
 * 输入：
 *   - model_path: 模型路径
 *   - model_config: 模型配置
 * 
 * 输出：
 *   - onnx_path: ONNX模型路径
 *   - tensorrt_path: TensorRT模型路径（可选）
 *   - openvino_path: OpenVINO模型路径（可选）
 *   - export_log: 导出日志
 * 
 * 参数：
 *   - export_formats: 导出格式列表（onnx/tensorrt/openvino）
 *   - input_shape: 输入形状
 *   - opset_version: ONNX opset版本
 *   - precision: 精度（fp32/fp16/int8）
 *   - optimize: 是否优化
 */
class ModelExportNode : public INode {
public:
    explicit ModelExportNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String export_to_onnx(const String& model_path, const String& output_dir, const Vector<int>& input_shape, int opset_version);
    String export_to_tensorrt(const String& onnx_path, const String& output_dir, const String& precision);
    String export_to_openvino(const String& onnx_path, const String& output_dir, const String& precision);
    String generate_export_script(const String& format, const HashMap<String, String>& config);
};

/**
 * @brief 模型优化节点
 * 
 * 功能：模型量化、剪枝、蒸馏
 * 
 * 输入：
 *   - model_path: 模型路径
 *   - calibration_data: 校准数据（量化用）
 * 
 * 输出：
 *   - optimized_model_path: 优化后的模型路径
 *   - optimization_report: 优化报告
 *   - performance_gain: 性能提升信息
 * 
 * 参数：
 *   - optimization_type: 优化类型（quantization/pruning/distillation）
 *   - precision: 目标精度（int8/fp16）
 *   - pruning_ratio: 剪枝比例
 *   - calibration_method: 校准方法（minmax/entropy percentile）
 */
class ModelOptimizeNode : public INode {
public:
    explicit ModelOptimizeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String quantize_model_int8(const String& model_path, const String& output_dir, const String& calibration_data);
    String prune_model(const String& model_path, const String& output_dir, float pruning_ratio);
    String generate_optimization_report(const String& original_path, const String& optimized_path);
};

/**
 * @brief 模型部署节点
 * 
 * 功能：生成部署包（推理引擎、示例代码、文档）
 * 
 * 输入：
 *   - model_path: 模型路径
 *   - model_config: 模型配置
 *   - deployment_config: 部署配置
 * 
 * 输出：
 *   - deployment_package: 部署包路径
 *   - inference_code: 推理代码
 *   - deployment_guide: 部署指南
 *   - test_script: 测试脚本
 * 
 * 参数：
 *   - deployment_target: 部署目标（edge/server/cloud/mobile）
 *   - runtime: 运行时环境（onnxruntime/tensorrt/openvino）
 *   - include_examples: 包含示例代码
 *   - include_documentation: 包含文档
 */
class ModelDeployNode : public INode {
public:
    explicit ModelDeployNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    String generate_inference_code(const String& model_path, const String& runtime, const String& target);
    String generate_deployment_guide(const String& model_info, const String& target);
    String generate_test_script(const String& model_path, const String& runtime);
    String package_deployment_files(const String& output_dir, const Vector<String>& files);
};

// ==================== 完整训练流程节点 ====================

/**
 * @brief 深度学习训练工作流节点
 * 
 * 功能：完整的训练工作流（数据准备→训练→验证→部署）
 * 
 * 输入：
 *   - dataset_path: 数据集路径
 *   - model_config: 模型配置
 *   - training_config: 训练配置
 * 
 * 输出：
 *   - workflow_status: 工作流状态
 *   - final_model_path: 最终模型路径
 *   - deployment_package: 部署包路径
 *   - workflow_report: 工作流报告
 * 
 * 参数：
 *   - workflow_steps: 工作流步骤（data_prep/train/validate/deploy）
 *   - auto_deploy: 训练完成后自动部署
 *   - early_stopping: 是否启用早停
 *   - patience: 早停耐心值
 *   - min_delta: 最小改进阈值
 */
class DeepLearningWorkflowNode : public INode {
public:
    explicit DeepLearningWorkflowNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    // 工作流步骤
    Result<void> step_data_preparation(FlowContext& context, const String& dataset_path);
    Result<void> step_training(FlowContext& context, const String& train_config);
    Result<void> step_validation(FlowContext& context, const String& model_path);
    Result<void> step_deployment(FlowContext& context, const String& model_path, const String& deploy_config);
    
    // 早停机制
    bool check_early_stopping(const Vector<float>& val_losses, int patience, float min_delta);
    
    // 学习率调度
    float calculate_learning_rate(const String& scheduler, float initial_lr, int epoch, int total_epochs, const Vector<float>& val_losses);
    
    // 生成工作流报告
    String generate_workflow_report(const HashMap<String, String>& step_results);
};

// ==================== 核心算法辅助函数 ====================

namespace training_utils {

/**
 * @brief 学习率调度器
 */
enum class LRScheduler {
    Step,           // 阶梯式衰减
    Cosine,         // 余弦退火
    Exponential,    // 指数衰减
    ReduceOnPlateau,// 指标 plateau 时衰减
    Linear,         // 线性衰减
    Constant        // 恒定
};

/**
 * @brief 计算学习率
 */
float compute_lr(LRScheduler scheduler, float initial_lr, int current_epoch, int total_epochs, 
                 float step_size = 0.1f, float gamma = 0.1f, int patience = 10, 
                 const Vector<float>* val_losses = nullptr);

/**
 * @brief 早停检查
 */
bool check_early_stop(const Vector<float>& val_losses, int patience, float min_delta, int& best_epoch);

/**
 * @brief 计算mAP（目标检测）
 */
float calculate_map_detection(const Vector<ImageAnnotation>& predictions, 
                               const Vector<ImageAnnotation>& ground_truths,
                               const Vector<float>& iou_thresholds);

/**
 * @brief 计算IoU（分割）
 */
float calculate_iou_segmentation(const Vector<uint8_t>& pred_mask, 
                                  const Vector<uint8_t>& gt_mask,
                                  int width, int height);

/**
 * @brief 计算AP（单类别）
 */
float calculate_ap_single_class(const Vector<float>& confidences, 
                                 const Vector<bool>& tp_or_fp,
                                 int num_gt);

/**
 * @brief 生成TensorBoard日志格式
 */
String format_tensorboard_scalar(const String& tag, float value, int step);
String format_tensorboard_histogram(const String& tag, const Vector<float>& values, int step);

/**
 * @brief 模型量化辅助
 */
struct QuantizationConfig {
    String precision;           // int8/fp16
    String calibration_method;  // minmax/entropy/percentile
    int calibration_samples;    // 校准样本数
    float percentile_value;     // percentile方法的百分位值
};

String quantization_config_to_json(const QuantizationConfig& config);

} // namespace training_utils

} // namespace algorithm
} // namespace ovf