/**
 * @file advanced_matching.h
 * @brief Halcon风格的高级模板匹配算子（纯C++实现）
 *
 * 支持基于形状、组件、灰度和描述符的模板匹配
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <memory>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 形状模型数据结构
 */
struct ShapeModel {
    uint32_t model_id = 0;                  // 模型ID
    uint32_t width = 0;                     // 模板宽度
    uint32_t height = 0;                    // 模板高度
    int num_levels = 4;                     // 金字塔层数
    float angle_start = 0.0f;               // 起始角度（度）
    float angle_extent = 360.0f;            // 角度范围（度）
    float angle_step = 1.0f;                // 角度步长（度）
    float scale_min = 1.0f;                 // 最小缩放
    float scale_max = 1.0f;                 // 最大缩放
    float scale_step = 0.01f;               // 缩放步长
    float min_contrast = 30.0f;             // 最小对比度
    float min_score = 0.7f;                 // 最小匹配分数
    int num_matches = 1;                    // 最大匹配数量
    float max_overlap = 0.5f;               // 最大重叠率

    // 边缘点数据
    struct EdgePoint {
        int x;
        int y;
        float direction;  // 梯度方向（弧度）
        float magnitude;  // 梯度幅值
    };
    std::vector<EdgePoint> edge_points;

    // 金字塔各层的边缘点
    std::vector<std::vector<EdgePoint>> pyramid_edges;

    // 模板图像数据（灰度）
    std::vector<uint8_t> template_data;
};

/**
 * @brief 组件模型数据结构
 */
struct ComponentModel {
    uint32_t model_id = 0;
    String model_name;
    int num_components = 0;

    struct Component {
        int component_id;
        int x, y;                    // 相对于模型中心的位置
        int width, height;
        std::vector<ShapeModel::EdgePoint> edge_points;
        std::vector<uint8_t> template_data;
    };
    std::vector<Component> components;

    // 组件间关系
    struct ComponentRelation {
        int comp1_id;
        int comp2_id;
        float distance;
        float angle;
    };
    std::vector<ComponentRelation> relations;

    // 训练参数
    float min_contrast = 30.0f;
    float min_score = 0.7f;
    int num_levels = 4;
};

/**
 * @brief 匹配结果（扩展）
 */
struct AdvancedMatchResult {
    float x;                  // 匹配位置X
    float y;                  // 匹配位置Y
    float score;              // 匹配分数
    float angle;              // 旋转角度（度）
    float scale;              // 缩放比例
    int model_id;             // 模型ID
    String model_name;        // 模型名称

    // 组件匹配结果
    struct ComponentResult {
        int component_id;
        float x, y;
        float score;
    };
    std::vector<ComponentResult> component_results;
};

/**
 * @brief 特征点结构
 */
struct FeaturePoint {
    float x, y;               // 特征点位置
    float scale;              // 尺度
    float orientation;        // 方向
    float response;           // 响应值
    std::vector<float> descriptor;  // 描述符（简化版SIFT）
};

// ========== 高级匹配工具函数 ==========

namespace advanced_match_utils {

/**
 * @brief 计算图像梯度
 */
void compute_gradient(const uint8_t* src, int width, int height,
                      std::vector<float>& grad_x, std::vector<float>& grad_y);

/**
 * @brief 边缘检测（Canny简化版）
 */
void detect_edges(const uint8_t* src, int width, int height,
                  std::vector<ShapeModel::EdgePoint>& edges,
                  float min_contrast = 30.0f);

/**
 * @brief 创建图像金字塔
 */
void build_image_pyramid(const uint8_t* src, int width, int height,
                         std::vector<std::vector<uint8_t>>& pyramid, int levels);

/**
 * @brief 图像旋转
 */
void rotate_image(const uint8_t* src, int width, int height,
                  std::vector<uint8_t>& dst, float angle_deg);

/**
 * @brief 图像缩放
 */
void scale_image(const uint8_t* src, int width, int height,
                 std::vector<uint8_t>& dst, float scale);

/**
 * @brief 计算NCC（归一化互相关）
 */
float compute_ncc(const uint8_t* src, int src_w, int src_h,
                  const uint8_t* tmpl, int tmpl_w, int tmpl_h,
                  int x, int y);

/**
 * @brief 计算SAD（绝对差和）
 */
float compute_sad(const uint8_t* src, int src_w, int src_h,
                  const uint8_t* tmpl, int tmpl_w, int tmpl_h,
                  int x, int y);

/**
 * @brief 金字塔搜索
 */
void pyramid_search(const uint8_t* src, int width, int height,
                    const ShapeModel& model,
                    std::vector<AdvancedMatchResult>& results,
                    float min_score, int max_matches);

/**
 * @brief 检测特征点（简化SIFT）
 */
void detect_feature_points(const uint8_t* src, int width, int height,
                           std::vector<FeaturePoint>& features,
                           int max_features = 500);

/**
 * @brief 计算特征描述符
 */
void compute_descriptors(const uint8_t* src, int width, int height,
                         std::vector<FeaturePoint>& features);

/**
 * @brief 匹配特征点
 */
void match_features(const std::vector<FeaturePoint>& features1,
                    const std::vector<FeaturePoint>& features2,
                    std::vector<std::pair<int, int>>& matches,
                    float threshold = 0.8f);

/**
 * @brief 非极大值抑制
 */
void non_max_suppression(std::vector<AdvancedMatchResult>& results,
                         float max_overlap, int width, int height);

/**
 * @brief 细化边缘位置（亚像素精度）
 */
void refine_position(const uint8_t* src, int src_w, int src_h,
                     const uint8_t* tmpl, int tmpl_w, int tmpl_h,
                     float& x, float& y);

} // namespace advanced_match_utils

// ========== 基于形状的匹配节点 ==========

/**
 * @brief 带缩放的形状匹配节点
 */
class ShapeMatchScaleNode : public INode {
public:
    ShapeMatchScaleNode(const String& instance_id);
    ~ShapeMatchScaleNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 带旋转的形状匹配节点
 */
class ShapeMatchRotateNode : public INode {
public:
    ShapeMatchRotateNode(const String& instance_id);
    ~ShapeMatchRotateNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 带缩放+旋转的形状匹配节点
 */
class ShapeMatchScaleRotateNode : public INode {
public:
    ShapeMatchScaleRotateNode(const String& instance_id);
    ~ShapeMatchScaleRotateNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 创建形状模板节点
 */
class CreateShapeModelNode : public INode {
public:
    CreateShapeModelNode(const String& instance_id);
    ~CreateShapeModelNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    const ShapeModel& get_model() const { return model_; }

private:
    ShapeModel model_;
};

/**
 * @brief 查找形状模板节点
 */
class FindShapeModelNode : public INode {
public:
    FindShapeModelNode(const String& instance_id);
    ~FindShapeModelNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void set_model(const ShapeModel& model) { model_ = model; }
    const std::vector<AdvancedMatchResult>& get_results() const { return results_; }

private:
    ShapeModel model_;
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 保存形状模板节点
 */
class WriteShapeModelNode : public INode {
public:
    WriteShapeModelNode(const String& instance_id);
    ~WriteShapeModelNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    // 内部序列化函数
    bool serialize_model(const ShapeModel& model, const String& filepath);
};

// ========== 基于组件的匹配节点 ==========

/**
 * @brief 基于组件的匹配节点
 */
class ComponentMatchNode : public INode {
public:
    ComponentMatchNode(const String& instance_id);
    ~ComponentMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    ComponentModel model_;
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 训练组件模型节点
 */
class TrainComponentModelNode : public INode {
public:
    TrainComponentModelNode(const String& instance_id);
    ~TrainComponentModelNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    const ComponentModel& get_model() const { return model_; }

private:
    ComponentModel model_;

    // 将模板分解为组件
    void decompose_into_components(const uint8_t* src, int width, int height,
                                   int num_components);
};

/**
 * @brief 查找组件模型节点
 */
class FindComponentModelNode : public INode {
public:
    FindComponentModelNode(const String& instance_id);
    ~FindComponentModelNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void set_model(const ComponentModel& model) { model_ = model; }
    const std::vector<AdvancedMatchResult>& get_results() const { return results_; }

private:
    ComponentModel model_;
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 组件检测节点
 */
class InspectComponentNode : public INode {
public:
    InspectComponentNode(const String& instance_id);
    ~InspectComponentNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    struct InspectionResult {
        int component_id;
        bool found;
        float score;
        float x, y;
        String status;
    };
    std::vector<InspectionResult> inspection_results_;
};

// ========== 基于灰度的匹配节点 ==========

/**
 * @brief 基于灰度的匹配节点（NCC/SAD）
 */
class GrayMatchNode : public INode {
public:
    GrayMatchNode(const String& instance_id);
    ~GrayMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<AdvancedMatchResult> results_;
};

/**
 * @brief 最佳匹配节点（单目标）
 */
class BestMatchNode : public INode {
public:
    BestMatchNode(const String& instance_id);
    ~BestMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    AdvancedMatchResult best_result_;
};

/**
 * @brief 所有匹配节点（多目标）
 */
class AllMatchNode : public INode {
public:
    AllMatchNode(const String& instance_id);
    ~AllMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<AdvancedMatchResult> all_results_;
};

// ========== 基于描述符的匹配节点 ==========

/**
 * @brief 基于描述符的匹配节点（SIFT简化版）
 */
class DescriptorMatchNode : public INode {
public:
    DescriptorMatchNode(const String& instance_id);
    ~DescriptorMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<FeaturePoint> template_features_;
    std::vector<FeaturePoint> image_features_;
    std::vector<std::pair<int, int>> matches_;
    AdvancedMatchResult result_;
};

/**
 * @brief 特征点匹配节点
 */
class FeaturePointMatchNode : public INode {
public:
    FeaturePointMatchNode(const String& instance_id);
    ~FeaturePointMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    std::vector<FeaturePoint> features_;
    std::vector<std::pair<int, int>> matches_;

    // 计算变换矩阵
    bool compute_transform(const std::vector<FeaturePoint>& src_points,
                          const std::vector<FeaturePoint>& dst_points,
                          const std::vector<std::pair<int, int>>& matches,
                          float& tx, float& ty, float& angle, float& scale);
};

} // namespace algorithm
} // namespace ovf