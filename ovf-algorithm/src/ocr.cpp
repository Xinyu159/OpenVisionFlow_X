/**
 * @file ocr.cpp
 * @brief OCR 文字识别模块实现
 */

#include "ovf/algorithm/ocr.h"
#include "ovf/core/logger.h"
#include <mutex>
#include <sstream>

namespace ovf {
namespace algorithm {

// ============================================================================
// OCR 工具函数实现（简化版，不实际依赖 Tesseract）
// ============================================================================

namespace {

// 全局 OCR 状态
struct OCRState {
    bool initialized = false;
    String model_path;
    String language = "eng";
    String whitelist;
    String blacklist;
    std::mutex mutex;
};

OCRState g_ocr_state;

// Mock OCR 引擎 - 简化实现
class MockOCREngine {
public:
    bool recognize_mock(const ImageData& image, OCRResult& result) {
        // 模拟识别 - 返回示例文本
        result.text = "[OCR Mock] Sample Text Recognition";
        result.confidence = 85.5f;
        result.language = g_ocr_state.language;
        
        // 添加示例文本区域
        result.lines.push_back("Sample Text");
        result.line_confidences.push_back(90.0f);
        
        // 添加示例文本区域边界框
        Region region;
        region.x = 10;
        region.y = 10;
        region.width = static_cast<int32_t>(image.width) - 20;
        region.height = static_cast<int32_t>(image.height) - 20;
        result.text_regions.push_back(region);
        
        // 设置边界框角点
        result.text_box.push_back(Point2D<int>(10, 10));
        result.text_box.push_back(Point2D<int>(static_cast<int>(image.width) - 10, 10));
        result.text_box.push_back(Point2D<int>(static_cast<int>(image.width) - 10, static_cast<int>(image.height) - 10));
        result.text_box.push_back(Point2D<int>(10, static_cast<int>(image.height) - 10));
        
        return true;
    }
};

MockOCREngine g_mock_engine;

} // anonymous namespace

namespace ocr_utils {

ErrorCode recognize(const ImageData& image, OCRResult& result, const String& lang) {
    std::lock_guard<std::mutex> lock(g_ocr_state.mutex);
    
    // 检查图像有效性
    if (image.empty()) {
        OVF_ERROR() << "OCR: Input image is empty";
        return ErrorCode::InvalidImage;
    }
    
    // 简化实现：使用 Mock 引擎
    // 实际实现应动态加载 Tesseract DLL
    g_ocr_state.language = lang;
    
    if (!g_ocr_state.initialized) {
        // 自动初始化
        OVF_INFO() << "OCR: Auto-initializing with language: " << lang;
        g_ocr_state.initialized = true;
    }
    
    // 执行识别
    bool success = g_mock_engine.recognize_mock(image, result);
    result.language = lang;
    
    if (!success) {
        OVF_ERROR() << "OCR: Recognition failed";
        return ErrorCode::AlgorithmExecFailed;
    }
    
    // 应用白名单/黑名单过滤（简化）
    if (!g_ocr_state.whitelist.empty() || !g_ocr_state.blacklist.empty()) {
        // 实际实现应在识别前设置 Tesseract 参数
        OVF_DEBUG() << "OCR: Using whitelist/blacklist filters";
    }
    
    OVF_INFO() << "OCR: Recognition completed, text length: " << result.text.length()
               << ", confidence: " << result.confidence;
    
    return ErrorCode::Success;
}

ErrorCode recognize_roi(const ImageData& image, int x, int y, int w, int h, 
                        OCRResult& result, const String& lang) {
    // 参数验证
    if (x < 0 || y < 0 || w <= 0 || h <= 0) {
        OVF_ERROR() << "OCR ROI: Invalid ROI parameters";
        return ErrorCode::InvalidParameter;
    }
    
    if (image.empty()) {
        OVF_ERROR() << "OCR ROI: Input image is empty";
        return ErrorCode::InvalidImage;
    }
    
    // 边界检查
    if (x >= static_cast<int>(image.width) || y >= static_cast<int>(image.height)) {
        OVF_ERROR() << "OCR ROI: ROI out of image bounds";
        return ErrorCode::OutOfRange;
    }
    
    // 创建 ROI 图像（简化实现）
    ImageData roi_image;
    roi_image.width = w;
    roi_image.height = h;
    roi_image.channels = image.channels;
    roi_image.format = image.format;
    
    // 计算实际可用的 ROI 区域
    int actual_w = std::min(w, static_cast<int>(image.width) - x);
    int actual_h = std::min(h, static_cast<int>(image.height) - y);
    
    roi_image.data.resize(actual_w * actual_h * roi_image.channels);
    
    // 复制 ROI 数据（简化）
    for (int dy = 0; dy < actual_h; ++dy) {
        for (int dx = 0; dx < actual_w; ++dx) {
            for (uint32_t c = 0; c < roi_image.channels; ++c) {
                size_t src_idx = ((y + dy) * image.width + (x + dx)) * image.channels + c;
                size_t dst_idx = (dy * actual_w + dx) * roi_image.channels + c;
                if (src_idx < image.data.size() && dst_idx < roi_image.data.size()) {
                    roi_image.data[dst_idx] = image.data[src_idx];
                }
            }
        }
    }
    
    // 调用标准识别
    ErrorCode code = recognize(roi_image, result, lang);
    
    // 调整边界框坐标到原图坐标系
    for (auto& pt : result.text_box) {
        pt.x += x;
        pt.y += y;
    }
    
    for (auto& region : result.text_regions) {
        region.x += x;
        region.y += y;
    }
    
    return code;
}

void set_ocr_params(const String& model_path, const String& lang) {
    std::lock_guard<std::mutex> lock(g_ocr_state.mutex);
    g_ocr_state.model_path = model_path;
    g_ocr_state.language = lang;
    OVF_INFO() << "OCR: Set params - model_path: " << model_path 
               << ", language: " << lang;
}

void set_char_whitelist(const String& chars) {
    std::lock_guard<std::mutex> lock(g_ocr_state.mutex);
    g_ocr_state.whitelist = chars;
    OVF_INFO() << "OCR: Set whitelist: " << chars;
}

void set_char_blacklist(const String& chars) {
    std::lock_guard<std::mutex> lock(g_ocr_state.mutex);
    g_ocr_state.blacklist = chars;
    OVF_INFO() << "OCR: Set blacklist: " << chars;
}

bool is_ocr_initialized() {
    return g_ocr_state.initialized;
}

ErrorCode initialize_ocr(const String& model_path, const String& lang) {
    std::lock_guard<std::mutex> lock(g_ocr_state.mutex);
    
    OVF_INFO() << "OCR: Initializing - model_path: " << model_path 
               << ", language: " << lang;
    
    g_ocr_state.model_path = model_path;
    g_ocr_state.language = lang;
    g_ocr_state.initialized = true;
    
    // 实际实现应检查 tessdata 文件是否存在
    // 并动态加载 Tesseract DLL
    
    return ErrorCode::Success;
}

void shutdown_ocr() {
    std::lock_guard<std::mutex> lock(g_ocr_state.mutex);
    
    OVF_INFO() << "OCR: Shutting down";
    g_ocr_state.initialized = false;
    g_ocr_state.model_path.clear();
    g_ocr_state.whitelist.clear();
    g_ocr_state.blacklist.clear();
}

Vector<String> get_supported_languages() {
    Vector<String> languages;
    languages.push_back("eng");        // 英语
    languages.push_back("chi_sim");    // 简体中文
    languages.push_back("chi_tra");    // 繁体中文
    languages.push_back("jpn");        // 日语
    languages.push_back("kor");        // 韩语
    languages.push_back("fra");        // 法语
    languages.push_back("deu");        // 德语
    languages.push_back("spa");        // 西班牙语
    return languages;
}

} // namespace ocr_utils

// ============================================================================
// OCRNode 实现
// ============================================================================

OCRNode::OCRNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo OCRNode::make_info() {
    NodeInfo info;
    info.id = "OCR";
    info.name = "OCR文字识别";
    info.category = "文字识别";
    info.description = "使用 OCR 识别图像中的文字";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";
    
    // 输入端口
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    // 输出端口
    info.outputs.push_back(DataPort("text", "识别文本", DataType::String));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));
    info.outputs.push_back(DataPort("text_regions", "文本区域", DataType::Array));
    info.outputs.push_back(DataPort("ocr_result", "完整结果", DataType::Object));
    
    // 参数
    info.params.push_back(ParamDef("language", "语言", DataType::String, Data("eng")));
    info.params.push_back(ParamDef("whitelist", "白名单字符", DataType::String, Data("")));
    info.params.push_back(ParamDef("blacklist", "黑名单字符", DataType::String, Data("")));
    info.params.push_back(ParamDef("model_path", "模型路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("min_confidence", "最小置信度", DataType::Number, Data(0.0)));
    
    // 构建语言选项列表
    Vector<String> lang_options;
    lang_options.push_back("eng");
    lang_options.push_back("chi_sim");
    lang_options.push_back("chi_tra");
    lang_options.push_back("jpn");
    lang_options.push_back("kor");
    info.params.back().options = lang_options;
    
    return info;
}

Result<void> OCRNode::init() {
    // 获取模型路径参数
    String model_path = get_param("model_path", Data("")).as_string();
    String language = get_param("language", Data("eng")).as_string();
    
    if (!model_path.empty()) {
        ErrorCode code = ocr_utils::initialize_ocr(model_path, language);
        if (code != ErrorCode::Success) {
            return Result<void>::failure(code, "OCR initialization failed");
        }
    }
    
    // 设置白名单/黑名单
    String whitelist = get_param("whitelist", Data("")).as_string();
    String blacklist = get_param("blacklist", Data("")).as_string();
    
    if (!whitelist.empty()) {
        ocr_utils::set_char_whitelist(whitelist);
    }
    if (!blacklist.empty()) {
        ocr_utils::set_char_blacklist(blacklist);
    }
    
    return Result<void>::success();
}

Result<void> OCRNode::execute(FlowContext& context) {
    // 获取输入图像
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    String language = get_param("language", Data("eng")).as_string();
    double min_confidence = get_param("min_confidence", Data(0.0)).as_number();
    
    // 执行 OCR 识别
    OCRResult ocr_result;
    ErrorCode code = ocr_utils::recognize(image, ocr_result, language);
    
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "OCR recognition failed");
    }
    
    // 检查置信度阈值
    if (ocr_result.confidence < min_confidence) {
        OVF_WARN() << "OCR: Confidence below threshold: " << ocr_result.confidence
                   << " < " << min_confidence;
    }
    
    // 设置输出
    set_output("text", Data(ocr_result.text));
    set_output("confidence", Data(ocr_result.confidence));
    
    // 输出文本区域（转换为 Region 数组）
    // 由于 Data 类不直接支持数组，使用简化方案
    // 输出第一个区域
    if (!ocr_result.text_regions.empty()) {
        set_output("text_regions", Data(ocr_result.text_regions[0]));
    } else {
        Region empty_region;
        set_output("text_regions", Data(empty_region));
    }
    
    // 输出完整结果（简化：使用文本）
    set_output("ocr_result", Data(ocr_result.text));
    
    return Result<void>::success();
}

// ============================================================================
// TextLocateNode 实现
// ============================================================================

TextLocateNode::TextLocateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo TextLocateNode::make_info() {
    NodeInfo info;
    info.id = "TextLocate";
    info.name = "文本定位";
    info.category = "文字识别";
    info.description = "检测图像中的文本区域（不进行识别）";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";
    
    // 输入端口
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    
    // 输出端口
    info.outputs.push_back(DataPort("text_regions", "文本区域", DataType::Region));
    info.outputs.push_back(DataPort("region_count", "区域数量", DataType::Number));
    
    // 参数
    info.params.push_back(ParamDef("min_area", "最小面积", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("max_area", "最大面积", DataType::Number, Data(10000)));
    info.params.push_back(ParamDef("sensitivity", "敏感度", DataType::Number, Data(0.5)));
    
    return info;
}

Result<void> TextLocateNode::init() {
    return Result<void>::success();
}

Result<void> TextLocateNode::execute(FlowContext& context) {
    // 获取输入
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 获取参数
    int min_area = get_param("min_area", Data(100)).as_int();
    int max_area = get_param("max_area", Data(10000)).as_int();
    double sensitivity = get_param("sensitivity", Data(0.5)).as_number();
    
    OVF_DEBUG() << "TextLocate: Detecting text regions"
                << ", min_area: " << min_area
                << ", sensitivity: " << sensitivity;
    
    // 简化实现：模拟文本检测
    // 实际实现可使用：
    // 1. EAST (Efficient and Accurate Scene Text Detector)
    // 2. CTPN (Connectionist Text Proposal Network)
    // 3. 传统形态学方法
    
    // 模拟检测结果
    Vector<Region> detected_regions;
    
    // 模拟检测几个区域
    Region region1;
    region1.x = static_cast<int32_t>(image.width * 0.1);
    region1.y = static_cast<int32_t>(image.height * 0.1);
    region1.width = static_cast<int32_t>(image.width * 0.3);
    region1.height = static_cast<int32_t>(image.height * 0.1);
    
    Region region2;
    region2.x = static_cast<int32_t>(image.width * 0.5);
    region2.y = static_cast<int32_t>(image.height * 0.3);
    region2.width = static_cast<int32_t>(image.width * 0.4);
    region2.height = static_cast<int32_t>(image.height * 0.15);
    
    detected_regions.push_back(region1);
    detected_regions.push_back(region2);
    
    // 应用面积过滤
    Vector<Region> filtered_regions;
    for (const auto& r : detected_regions) {
        int area = r.width * r.height;
        if (area >= min_area && area <= max_area) {
            filtered_regions.push_back(r);
        }
    }
    
    // 设置输出
    if (!filtered_regions.empty()) {
        // 输出第一个区域（简化）
        set_output("text_regions", Data(filtered_regions[0]));
    } else {
        Region empty;
        set_output("text_regions", Data(empty));
    }
    
    set_output("region_count", Data(static_cast<int32_t>(filtered_regions.size())));
    
    OVF_INFO() << "TextLocate: Detected " << filtered_regions.size() << " text regions";
    
    return Result<void>::success();
}

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(OCRNode, "OCR", OCRNode::make_info());
OVF_REGISTER_NODE(TextLocateNode, "TextLocate", TextLocateNode::make_info());

} // namespace algorithm
} // namespace ovf