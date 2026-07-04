/**
 * @file tesseract_stub.h
 * @brief Tesseract OCR 简化头文件 - 提供接口定义，不实际依赖库
 * @author OpenVisionFlow Team
 * @version 0.1.0
 *
 * 说明：
 * - 这是一个简化版的 Tesseract 接口定义
 * - 实际 OCR 功能通过运行时动态加载 Tesseract DLL 实现
 * - 支持多语言识别（中文/英文等）
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace ovf {
namespace thirdparty {
namespace tesseract {

/**
 * @brief Tesseract OCR 页面分割模式
 */
enum class PageSegMode {
    PSM_OSD_ONLY = 0,           // 仅方向和脚本检测
    PSM_AUTO_OSD = 1,           // 自动方向和脚本检测
    PSM_AUTO_ONLY = 2,          // 自动分割，无OSD
    PSM_AUTO = 3,               // 全自动页面分割
    PSM_SINGLE_COLUMN = 4,      // 单列变长文本
    PSM_SINGLE_BLOCK_VERT = 5,  // 单列垂直对齐文本块
    PSM_SINGLE_BLOCK = 6,       // 单个均匀文本块
    PSM_SINGLE_LINE = 7,        // 单行文本
    PSM_SINGLE_WORD = 8,        // 单个词
    PSM_CIRCLE_WORD = 9,        // 圆圈中的单个词
    PSM_SINGLE_CHAR = 10,       // 单个字符
    PSM_SPARSE_TEXT = 11,       // 稀疏文本，无特定顺序
    PSM_SPARSE_TEXT_OSD = 12,   // 稀疏文本带OSD
    PSM_RAW_LINE = 13           // 原始行，绕过hack
};

/**
 * @brief OCR 引擎模式
 */
enum class OcrEngineMode {
    OEM_TESSERACT_ONLY = 0,     // 仅Tesseract
    OEM_LSTM_ONLY = 1,          // 仅LSTM
    OEM_TESSERACT_LSTM_COMBINED = 2, // Tesseract + LSTM
    OEM_DEFAULT = 3             // 默认
};

/**
 * @brief 文本识别级别
 */
enum class TextlineOrder {
    LEFT_TO_RIGHT = 0,
    RIGHT_TO_LEFT = 1,
    TOP_TO_BOTTOM = 2
};

/**
 * @brief 文本边界框
 */
struct BoundingBox {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t height = 0;
};

/**
 * @brief 单个识别结果
 */
struct RecognitionResult {
    std::string text;           // 识别的文本
    float confidence = 0.0f;    // 置信度 (0-1)
    BoundingBox bbox;           // 边界框
};

/**
 * @brief 页面识别结果
 */
struct PageResult {
    std::string full_text;      // 完整文本
    std::vector<RecognitionResult> words; // 词级别结果
    std::vector<RecognitionResult> lines; // 行级别结果
    int32_t width = 0;          // 页面宽度
    int32_t height = 0;         // 页面高度
    int32_t orientation = 0;    // 方向角度
    float mean_confidence = 0.0f; // 平均置信度
};

/**
 * @brief Tesseract 初始化参数
 */
struct InitParams {
    std::string data_path;      // 语言数据路径
    std::string language;       // 语言代码 (eng, chi_sim, chi_tra等)
    OcrEngineMode engine_mode = OcrEngineMode::OEM_DEFAULT;
    PageSegMode page_seg_mode = PageSegMode::PSM_AUTO;
    bool enable_debug = false;  // 启用调试输出
};

/**
 * @brief Tesseract OCR 引擎接口（简化版）
 *
 * 这是一个抽象接口，实际实现通过动态加载 DLL
 */
class ITesseractEngine {
public:
    virtual ~ITesseractEngine() = default;

    // 初始化
    virtual bool initialize(const InitParams& params) = 0;
    virtual void shutdown() = 0;
    virtual bool is_initialized() const = 0;

    // 识别
    virtual bool recognize(const uint8_t* image_data,
                          int32_t width, int32_t height,
                          int32_t channels,
                          PageResult& result) = 0;

    // 区域识别
    virtual bool recognize_roi(const uint8_t* image_data,
                              int32_t width, int32_t height,
                              int32_t channels,
                              int32_t x, int32_t y, int32_t w, int32_t h,
                              PageResult& result) = 0;

    // 设置参数
    virtual void set_variable(const std::string& key, const std::string& value) = 0;

    // 设置白名单/黑名单
    virtual void set_char_whitelist(const std::string& chars) = 0;
    virtual void set_char_blacklist(const std::string& chars) = 0;

    // 获取支持的语言列表
    virtual std::vector<std::string> get_available_languages() const = 0;
};

// 支持的语言代码
constexpr const char* LANG_ENGLISH = "eng";
constexpr const char* LANG_CHINESE_SIMPLIFIED = "chi_sim";
constexpr const char* LANG_CHINESE_TRADITIONAL = "chi_tra";
constexpr const char* LANG_JAPANESE = "jpn";
constexpr const char* LANG_KOREAN = "kor";

/**
 * @brief 创建 Tesseract 引擎实例
 * @return 引擎指针，失败返回 nullptr
 */
ITesseractEngine* create_tesseract_engine();

/**
 * @brief 销毁 Tesseract 引擎实例
 */
void destroy_tesseract_engine(ITesseractEngine* engine);

} // namespace tesseract
} // namespace thirdparty
} // namespace ovf