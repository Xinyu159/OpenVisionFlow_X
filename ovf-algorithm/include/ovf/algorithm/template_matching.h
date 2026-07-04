#pragma once

#include "ovf/core/node.h"
#include <vector>

namespace ovf {
namespace algorithm {

/**
 * @brief 模板匹配方法
 */
enum class MatchMethod {
    SAD,    // 平方差和 (Sum of Absolute Differences)
    SSD,    // 平方和差 (Sum of Squared Differences)
    NCC,    // 归一化互相关 (Normalized Cross-Correlation)
    NCC_SAD // NCC预筛选 + SAD精确定位
};

/**
 * @brief 匹配结果
 */
struct MatchResult {
    int x;              // 匹配位置X
    int y;              // 匹配位置Y
    float score;        // 匹配分数 (NCC: -1~1, SAD/SSD: 越小越好)
    float angle;        // 旋转角度（如果支持）
    float scale;        // 缩放比例（如果支持）
};

/**
 * @brief 模板匹配工具函数
 */
namespace template_utils {

/**
 * @brief 使用SAD方法进行模板匹配
 */
ErrorCode match_sad(const uint8_t* src, int src_width, int src_height,
                    const uint8_t* tmpl, int tmpl_width, int tmpl_height,
                    MatchResult& result);

/**
 * @brief 使用SSD方法进行模板匹配
 */
ErrorCode match_ssd(const uint8_t* src, int src_width, int src_height,
                    const uint8_t* tmpl, int tmpl_width, int tmpl_height,
                    MatchResult& result);

/**
 * @brief 使用NCC方法进行模板匹配
 */
ErrorCode match_ncc(const uint8_t* src, int src_width, int src_height,
                    const uint8_t* tmpl, int tmpl_width, int tmpl_height,
                    MatchResult& result);

} // namespace template_utils

/**
 * @brief 模板匹配节点
 */
class TemplateMatchNode : public INode {
public:
    TemplateMatchNode(const String& instance_id);
    ~TemplateMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void set_method(MatchMethod method) { method_ = method; }
    void set_threshold(float threshold) { threshold_ = threshold; }
    void set_template(const ImageData& tmpl) { template_image_ = tmpl; }
    MatchResult get_result() const { return result_; }

private:
    MatchMethod method_ = MatchMethod::NCC;
    float threshold_ = 0.7f;
    ImageData template_image_;
    MatchResult result_;
};

/**
 * @brief 模板训练节点
 */
class TemplateTrainNode : public INode {
public:
    TemplateTrainNode(const String& instance_id);
    ~TemplateTrainNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void set_roi(int x, int y, int width, int height);
    ImageData get_template() const { return template_image_; }

private:
    int roi_x_ = 0;
    int roi_y_ = 0;
    int roi_width_ = 100;
    int roi_height_ = 100;
    ImageData template_image_;
};

/**
 * @brief 多模板匹配节点
 */
class MultiTemplateMatchNode : public INode {
public:
    MultiTemplateMatchNode(const String& instance_id);
    ~MultiTemplateMatchNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

    void add_template(const ImageData& tmpl, const String& name);
    void clear_templates();

private:
    struct TemplateInfo {
        ImageData image;
        String name;
    };
    std::vector<TemplateInfo> templates_;
    MatchMethod method_ = MatchMethod::NCC;
    float threshold_ = 0.7f;
};

} // namespace algorithm
} // namespace ovf