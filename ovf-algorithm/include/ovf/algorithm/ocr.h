/**
 * @file ocr.h
 * @brief OpenVisionFlow OCR 文字识别模块
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/types.h"
#include <memory>

namespace ovf {
namespace algorithm {

/**
 * @brief OCR 识别结果结构
 */
struct OCRResult {
    String text;                    // 识别文本
    float confidence = 0.0f;        // 置信度 (0-100)
    Vector<Point2D<int>> text_box;  // 文本区域边界（四个角点）
    String language;                // 语言代码
    
    // 扩展信息
    Vector<String> lines;           // 行级别文本
    Vector<float> line_confidences; // 行置信度
    Vector<Region> text_regions;    // 文本区域列表
    
    // 默认构造
    OCRResult() = default;
    
    // 带参数构造
    OCRResult(const String& t, float conf, const String& lang = "eng")
        : text(t), confidence(conf), language(lang) {}
};

/**
 * @brief OCR 工具函数命名空间
 */
namespace ocr_utils {

/**
 * @brief OCR 识别（调用 Tesseract）
 * @param image 输入图像
 * @param result 识别结果
 * @param lang 语言代码 (eng, chi_sim, chi_tra 等)
 * @return 错误码
 */
ErrorCode recognize(const ImageData& image, OCRResult& result, const String& lang = "eng");

/**
 * @brief 带区域的 OCR 识别
 * @param image 输入图像
 * @param x ROI 左上角 X
 * @param y ROI 左上角 Y
 * @param w ROI 宽度
 * @param h ROI 高度
 * @param result 识别结果
 * @param lang 语言代码
 * @return 错误码
 */
ErrorCode recognize_roi(const ImageData& image, int x, int y, int w, int h, 
                        OCRResult& result, const String& lang = "eng");

/**
 * @brief 设置 OCR 参数
 * @param model_path 语言模型路径（tessdata 目录）
 * @param lang 语言代码
 */
void set_ocr_params(const String& model_path, const String& lang);

/**
 * @brief 设置字符白名单（只识别这些字符）
 * @param chars 白名单字符
 */
void set_char_whitelist(const String& chars);

/**
 * @brief 设置字符黑名单（不识别这些字符）
 * @param chars 黑名单字符
 */
void set_char_blacklist(const String& chars);

/**
 * @brief 获取当前 OCR 状态
 * @return 是否已初始化
 */
bool is_ocr_initialized();

/**
 * @brief 初始化 OCR 引擎
 * @param model_path 语言模型路径
 * @param lang 语言代码
 * @return 错误码
 */
ErrorCode initialize_ocr(const String& model_path, const String& lang = "eng");

/**
 * @brief 关闭 OCR 引擎
 */
void shutdown_ocr();

/**
 * @brief 获取支持的语言列表
 * @return 语言代码列表
 */
Vector<String> get_supported_languages();

} // namespace ocr_utils

/**
 * @brief OCR 识别节点
 *
 * 输入: image - 待识别图像
 * 输出: text - 识别文本
 *       confidence - 置信度
 *       text_regions - 文本区域列表
 * 参数: language - 语言
 *       whitelist - 白名单字符
 *       blacklist - 黑名单字符
 */
class OCRNode : public INode {
public:
    OCRNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief OCR 训练节点（简化版）
 *
 * 用于字体训练和自定义字典创建
 * 这是一个简化实现，实际训练需要外部工具
 *
 * 输入: image - 样本图像
 *       text - 对应文本
 * 输出: model_path - 生成的模型路径
 * 参数: font_name - 字体名称
 *       output_dir - 输出目录
 */
class OCRTrainNode : public INode {
public:
    OCRTrainNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 文本定位节点
 *
 * 检测图像中的文本区域，不进行识别
 *
 * 输入: image - 输入图像
 * 输出: text_regions - 文本区域列表
 *       region_count - 区域数量
 * 参数: min_area - 最小区域面积
 *       max_area - 最大区域面积
 *       sensitivity - 检测敏感度
 */
class TextLocateNode : public INode {
public:
    TextLocateNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief OCR 结果数据类型扩展
 *
 * 用于 Data 容器存储 OCRResult
 */
struct OCRData {
    OCRResult result;
    
    // 转换为字符串
    String to_string() const {
        return "OCR[" + result.text + ", conf=" + 
               std::to_string(result.confidence) + "]";
    }
};

} // namespace algorithm
} // namespace ovf