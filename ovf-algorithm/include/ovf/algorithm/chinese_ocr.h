/**
 * @file chinese_ocr.h
 * @brief OpenVisionFlow 印刷体汉字 OCR 识别模块
 * @author OpenVisionFlow Team
 * @version 0.1.0
 *
 * 纯 C++ 实现的印刷体汉字识别能力，不依赖 Tesseract / OpenCV。
 * 核心算法包含：MSER 文本检测、字符分割（投影法 + 连通域）、
 * 模板匹配汉字识别（网格 + 方向 + 投影特征，余弦相似度）。
 *
 * 节点列表（约 10 个）：
 *   字符检测：CharDetectNode / CharSegmentNode / CharBoxNode
 *   字符识别：ChineseOCRNode / DigitOCRNode / EnglishOCRNode / MixedOCRNode
 *   OCR 训练：OCRTrainNode / OCRModelNode
 *   验证工具：OCRVerifyNode
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/types.h"
#include <memory>

namespace ovf {
namespace algorithm {

/**
 * @brief 字符集枚举
 */
enum class CharSet : uint8_t {
    Chinese = 0,   // 汉字（GB2312 一级常用汉字 3755 个）
    Digit   = 1,   // 数字 0-9
    English = 2,   // 英文字母 A-Z
    Mixed   = 3    // 混合字符（汉字 + 数字 + 英文）
};

/**
 * @brief CharSet枚举输出运算符
 */
inline std::ostream& operator<<(std::ostream& os, CharSet charset) {
    switch (charset) {
        case CharSet::Chinese: return os << "Chinese";
        case CharSet::Digit:   return os << "Digit";
        case CharSet::English: return os << "English";
        case CharSet::Mixed:   return os << "Mixed";
        default:               return os << "Unknown";
    }
}

/**
 * @brief 字符框（位置、尺寸）
 */
struct CharBox {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t height = 0;
    float confidence = 0.0f;     // 检测/识别置信度

    CharBox() = default;
    CharBox(int32_t x_, int32_t y_, int32_t w, int32_t h)
        : x(x_), y(y_), width(w), height(h) {}

    bool valid() const { return width > 0 && height > 0; }
    int32_t area() const { return width * height; }
    float aspect_ratio() const {
        return height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 0.0f;
    }

    // 转换为 Region
    Region to_region() const {
        return Region(x, y, width, height);
    }
};

/**
 * @brief 字符识别结果（字符、置信度）
 */
struct CharResult {
    String character;            // 识别出的字符
    float confidence = 0.0f;      // 置信度 (0-100)
    CharBox box;                  // 字符框
    CharSet char_set = CharSet::Chinese;

    CharResult() = default;
    CharResult(const String& ch, float conf, const CharBox& b, CharSet cs = CharSet::Chinese)
        : character(ch), confidence(conf), box(b), char_set(cs) {}

    bool is_valid() const { return !character.empty(); }
};

/**
 * @brief 特征向量类型
 */
using FeatureVector = Vector<float>;

/**
 * @brief OCR 模板条目（字符 + 特征向量）
 */
struct OCRTemplate {
    String label;                 // 字符标签
    CharSet char_set = CharSet::Chinese;
    FeatureVector feature;         // 特征向量
    int sample_count = 0;         // 训练样本数

    OCRTemplate() = default;
    OCRTemplate(const String& lbl, CharSet cs, const FeatureVector& feat)
        : label(lbl), char_set(cs), feature(feat), sample_count(1) {}
};

/**
 * @brief OCR 模型（模板库、特征向量）
 */
struct OCRModel {
    String name;                              // 模型名称
    CharSet char_set = CharSet::Chinese;       // 字符集
    Vector<OCRTemplate> templates;             // 模板库
    int feature_dim = 128;                     // 特征维度（64 + 32 + 32）
    String version;                           // 模型版本
    String created_time;                       // 创建时间

    // 基本方法
    void clear() { templates.clear(); }
    size_t size() const { return templates.size(); }
    bool empty() const { return templates.empty(); }

    // 查找模板
    OCRTemplate* find(const String& label);
    const OCRTemplate* find(const String& label) const;

    // 添加或更新模板（增量训练）
    void add_template(const String& label, CharSet cs, const FeatureVector& feat);
};

/**
 * @brief OCR 训练数据（图像、标签）
 */
struct OCRTrainData {
    ImageData image;              // 字符样本图像
    String label;                 // 字符标签
    CharSet char_set = CharSet::Chinese;
    int sample_id = 0;            // 样本 ID
};

/**
 * @brief 汉字 OCR 完整识别结果
 */
struct ChineseOCRResult {
    String text;                          // 识别的文本字符串
    Vector<CharResult> chars;             // 字符结果数组（位置 + 字符 + 置信度）
    float confidence = 0.0f;               // 整体置信度
    CharSet char_set = CharSet::Chinese;

    ChineseOCRResult() = default;
};

/**
 * @brief 汉字 OCR 工具函数命名空间
 *
 * 提供核心算法实现：
 *  - MSER 文本检测
 *  - 字符分割（垂直投影 + 连通域）
 *  - 特征提取（网格 + 方向 + 投影）
 *  - 模板匹配（余弦相似度）
 *  - 模型 I/O（JSON 格式）
 */
namespace chinese_ocr_utils {

// ----------------------------------------------------------------------
// 图像预处理
// ----------------------------------------------------------------------

/**
 * @brief 转灰度图
 */
ErrorCode to_gray(const ImageData& src, ImageData& gray);

/**
 * @brief Otsu 二值化
 */
ErrorCode threshold_otsu(const ImageData& gray, ImageData& binary);

/**
 * @brief 自适应阈值二值化（均值法）
 */
ErrorCode threshold_adaptive(const ImageData& gray, ImageData& binary, int block_size = 15);

/**
 * @brief 调整图像尺寸（双线性插值）
 */
ErrorCode resize_image(const ImageData& src, ImageData& dst, int new_w, int new_h);

/**
 * @brief 字符图像预处理：归一化到固定大小（居中、保持比例）
 */
ErrorCode preprocess_char(const ImageData& src, ImageData& dst, int target_size = 32);

/**
 * @brief 从图像中提取 ROI
 */
ErrorCode crop_image(const ImageData& src, ImageData& dst, int x, int y, int w, int h);

// ----------------------------------------------------------------------
// MSER 文本检测
// ----------------------------------------------------------------------

/**
 * @brief MSER 区域
 */
struct MSERRegion {
    int32_t x = 0, y = 0, width = 0, height = 0;
    int area = 0;                 // 区域像素数
    float stability = 0.0f;       // 区域稳定性
    Vector<Point2D<int>> pixels; // 区域像素坐标
};

/**
 * @brief MSER 文本检测（Maximally Stable Extremal Regions）
 *
 * 多阈值二值化 → 计算区域稳定性 → 筛选文本候选区域
 *
 * @param gray 灰度图
 * @param regions 输出 MSER 区域列表
 * @param delta 阈值步长（默认 5）
 * @param min_area 最小区域面积
 * @param max_area 最大区域面积
 */
ErrorCode mser_detect(const ImageData& gray, Vector<MSERRegion>& regions,
                      int delta = 5, int min_area = 60, int max_area = 14400);

/**
 * @brief 非极大值抑制（NMS）
 * @param boxes 输入字符框（会被原地修改）
 * @param scores 对应置信度
 * @param threshold IoU 阈值
 */
ErrorCode nms(Vector<CharBox>& boxes, Vector<float>& scores, float threshold = 0.3f);

/**
 * @brief 计算 IoU（交并比）
 */
float compute_iou(const CharBox& a, const CharBox& b);

// ----------------------------------------------------------------------
// 字符分割
// ----------------------------------------------------------------------

/**
 * @brief 垂直投影法字符分割
 *
 * 统计每列像素和 → 寻找投影谷点 → 切分字符
 *
 * @param binary 二值图
 * @param chars 输出字符框列表
 * @param min_char_width 最小字符宽度
 * @param max_char_width 最大字符宽度
 */
ErrorCode segment_by_projection(const ImageData& binary, Vector<CharBox>& chars,
                                int min_char_width = 5, int max_char_width = 100);

/**
 * @brief 连通域法字符分割
 *
 * 两遍扫描连通域标记 → 按尺寸筛选字符
 *
 * @param binary 二值图
 * @param chars 输出字符框列表
 * @param min_char_size 最小字符尺寸
 * @param max_char_size 最大字符尺寸
 */
ErrorCode segment_by_connected(const ImageData& binary, Vector<CharBox>& chars,
                                int min_char_size = 5, int max_char_size = 100);

/**
 * @brief 综合字符分割（投影法 + 连通域）
 *
 * 先用投影法分出字符行，对每个候选区域再用连通域微调
 */
ErrorCode segment_chars(const ImageData& binary, Vector<CharBox>& chars,
                        int min_char_size = 5, int max_char_size = 100);

// ----------------------------------------------------------------------
// 特征提取
// ----------------------------------------------------------------------

/**
 * @brief 网格特征提取（8x8 网格像素密度，64 维）
 */
ErrorCode extract_grid_feature(const ImageData& char_img, FeatureVector& feature, int grid = 8);

/**
 * @brief 方向特征提取（4 方向梯度直方图，32 维）
 *
 * 梯度方向分 4 个区间（水平 / 垂直 / 45° / 135°），
 * 每区间 8 个 bin，共 32 维。
 */
ErrorCode extract_direction_feature(const ImageData& char_img, FeatureVector& feature, int bins = 32);

/**
 * @brief 投影特征提取（水平 + 垂直投影，32 维）
 *
 * 水平投影 16 维 + 垂直投影 16 维
 */
ErrorCode extract_projection_feature(const ImageData& char_img, FeatureVector& feature, int bins = 32);

/**
 * @brief 提取完整特征向量（网格 64 + 方向 32 + 投影 32 = 128 维）
 */
ErrorCode extract_features(const ImageData& char_img, FeatureVector& feature);

/**
 * @brief 获取特征维度
 */
int get_feature_dim();

// ----------------------------------------------------------------------
// 模板匹配
// ----------------------------------------------------------------------

/**
 * @brief 余弦相似度
 */
float cosine_similarity(const FeatureVector& a, const FeatureVector& b);

/**
 * @brief 模板匹配识别
 *
 * 与训练库比较，余弦相似度，返回最佳匹配
 *
 * @param feature 待识别特征向量
 * @param model OCR 模型
 * @param result 输出识别结果
 * @param threshold 置信度阈值
 */
ErrorCode match_template(const FeatureVector& feature, const OCRModel& model,
                        CharResult& result, float threshold = 0.0f);

// ----------------------------------------------------------------------
// 模型 I/O（JSON 格式）
// ----------------------------------------------------------------------

/**
 * @brief 保存 OCR 模型到 JSON 文件
 */
ErrorCode save_model(const OCRModel& model, const String& path);

/**
 * @brief 从 JSON 文件加载 OCR 模型
 */
ErrorCode load_model(OCRModel& model, const String& path);

/**
 * @brief 内置默认模型（GB2312 一级常用汉字 + 数字 + 英文字母）
 *
 * 注意：实际部署应通过 OCRTrainNode 训练生成完整模型。
 * 此处仅返回一个空模板模型框架，用于初始化。
 */
OCRModel create_default_model(CharSet char_set);

// ----------------------------------------------------------------------
// 字符集工具
// ----------------------------------------------------------------------

/**
 * @brief 获取字符集名称
 */
String char_set_name(CharSet char_set);

/**
 * @brief 解析字符集名称
 */
CharSet parse_char_set(const String& name);

/**
 * @brief 获取字符集支持的字符列表
 *
 * - Chinese: GB2312 一级常用汉字 3755 个（返回常用子集以避免过大）
 * - Digit:   0-9
 * - English: A-Z
 * - Mixed:   汉字 + 数字 + 英文
 */
Vector<String> get_char_set(CharSet char_set);

/**
 * @brief 获取当前时间字符串（用于模型时间戳）
 */
String current_time_string();

} // namespace chinese_ocr_utils

// ======================================================================
// 节点类声明
// ======================================================================

/**
 * @brief 字符区域检测节点（MSER + NMS）
 *
 * 输入: image - 输入图像
 * 输出: char_region   - 第一个检测到的字符区域（Region）
 *       region_count  - 检测到的字符区域数量
 *       text          - 检测概要信息
 * 参数: min_char_size  - 最小字符尺寸
 *       max_char_size  - 最大字符尺寸
 *       delta          - MSER 阈值步长
 */
class CharDetectNode : public INode {
public:
    CharDetectNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 字符分割节点（投影法 + 连通域）
 *
 * 输入: image - 输入图像（或二值图）
 * 输出: char_count       - 分割出的字符数量
 *       first_char_region- 第一个字符框（Region）
 *       text             - 分割概要
 * 参数: min_char_size - 最小字符尺寸
 *       max_char_size - 最大字符尺寸
 *       method        - 分割方法（projection / connected / auto）
 */
class CharSegmentNode : public INode {
public:
    CharSegmentNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 字符框定位节点
 *
 * 输入: image - 输入图像
 * 输出: char_box   - 主字符框（Region）
 *       box_count  - 字符框数量
 *       text       - 定位概要
 * 参数: min_char_size - 最小字符尺寸
 *       max_char_size - 最大字符尺寸
 *       aspect_ratio  - 字符宽高比约束
 */
class CharBoxNode : public INode {
public:
    CharBoxNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 汉字 OCR 识别节点（模板匹配法）
 *
 * 输入: image - 待识别图像
 * 输出: text        - 识别的文本字符串
 *       confidence  - 整体置信度
 *       chars       - 字符结果数组（输出第一个字符的 Region + 文本）
 * 参数: char_set             - 字符集（chinese）
 *       min_char_size        - 最小字符尺寸
 *       max_char_size        - 最大字符尺寸
 *       confidence_threshold - 置信度阈值
 *       model_path           - 模型库路径
 */
class ChineseOCRNode : public INode {
public:
    ChineseOCRNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    OCRModel model_;
    bool model_loaded_ = false;
};

/**
 * @brief 数字 OCR 识别节点
 *
 * 输入: image - 待识别图像
 * 输出: text, confidence, chars
 * 参数: char_set (固定为 digit), confidence_threshold, model_path
 */
class DigitOCRNode : public INode {
public:
    DigitOCRNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    OCRModel model_;
    bool model_loaded_ = false;
};

/**
 * @brief 英文字母识别节点
 *
 * 输入: image - 待识别图像
 * 输出: text, confidence, chars
 * 参数: char_set (固定为 english), confidence_threshold, model_path
 */
class EnglishOCRNode : public INode {
public:
    EnglishOCRNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    OCRModel model_;
    bool model_loaded_ = false;
};

/**
 * @brief 混合字符识别节点（汉字 + 数字 + 英文）
 *
 * 输入: image - 待识别图像
 * 输出: text, confidence, chars
 * 参数: char_set (固定为 mixed), confidence_threshold, model_path
 */
class MixedOCRNode : public INode {
public:
    MixedOCRNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    OCRModel model_;
    bool model_loaded_ = false;
};

/**
 * @brief OCR 字符训练节点
 *
 * 收集字符样本 → 提取特征向量 → 存储到模板库（JSON 格式）→ 支持增量训练
 *
 * 输入: image - 字符样本图像
 *       label - 字符标签
 * 输出: success       - 是否成功
 *       model_path    - 模型库路径
 *       sample_count  - 当前模板总数
 * 参数: char_set             - 字符集
 *       model_path           - 模型库路径
 *       confidence_threshold - 置信度阈值（用于去重）
 */
class OCRTrainNode : public INode {
public:
    OCRTrainNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief OCR 模型管理节点
 *
 * 支持加载 / 保存 / 清空 / 查询模型信息
 *
 * 输入: image - 可选，用于追加样本
 * 输出: model_size - 模板数量
 *       model_path - 模型路径
 *       model_name - 模型名称
 * 参数: action     - 操作（load/save/clear/info）
 *       model_path - 模型库路径
 *       char_set   - 字符集
 *       model_name - 模型名称
 */
class OCRModelNode : public INode {
public:
    OCRModelNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief OCR 识别验证节点
 *
 * 输入: image        - 待识别图像
 *       expected_text- 期望文本
 * 输出: matched   - 是否匹配
 *       accuracy  - 准确率
 *       actual_text - 实际识别文本
 *       confidence - 识别置信度
 * 参数: char_set             - 字符集
 *       confidence_threshold - 置信度阈值
 *       model_path           - 模型库路径
 *       case_sensitive       - 是否区分大小写
 */
class OCRVerifyNode : public INode {
public:
    OCRVerifyNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    OCRModel model_;
    bool model_loaded_ = false;
};

} // namespace algorithm
} // namespace ovf
