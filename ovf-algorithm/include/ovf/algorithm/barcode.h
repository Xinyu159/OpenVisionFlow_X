/**
 * @file barcode.h
 * @brief OpenVisionFlow 条码/二维码识别模块
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 条码类型枚举
 */
enum class BarcodeType : uint8_t {
    // 一维码
    CODE128 = 0,      // CODE128码
    CODE39 = 1,       // CODE39码
    EAN13 = 2,        // EAN-13码
    EAN8 = 3,         // EAN-8码
    UPC_A = 4,        // UPC-A码
    UPC_E = 5,        // UPC-E码
    INTERLEAVED_25 = 6, // 交叉25码
    // 二维码
    QR_CODE = 10,     // QR码
    DATA_MATRIX = 11, // DataMatrix码
    PDF417 = 12,      // PDF417码
    AZTEC = 13,       // Aztec码
    // 自动检测
    AUTO = 255        // 自动检测条码类型
};

/**
 * @brief 条码识别结果
 */
struct BarcodeResult {
    String data;                    // 解码内容
    BarcodeType type = BarcodeType::AUTO;  // 条码类型
    String type_name;               // 类型名称
    Vector<Point2D<float>> corners; // 四角坐标
    float quality = 0.0f;          // 质量/置信度 (0.0-1.0)
    int orientation = 0;            // 方向角度 (0, 90, 180, 270)
    
    BarcodeResult() = default;
    
    bool is_valid() const { return !data.empty(); }
};

/**
 * @brief 条码工具函数命名空间
 */
namespace barcode_utils {

/**
 * @brief 将条码类型转换为名称
 * @param type 条码类型
 * @return 类型名称字符串
 */
String type_to_name(BarcodeType type);

/**
 * @brief 将名称转换为条码类型
 * @param name 类型名称
 * @return 条码类型
 */
BarcodeType name_to_type(const String& name);

/**
 * @brief 解码条码（简化实现，提供接口）
 * @param image 输入图像
 * @param result 输出结果
 * @param type 条码类型（默认AUTO自动检测）
 * @return 操作结果
 */
Result<BarcodeResult> decode(const ImageData& image, BarcodeType type = BarcodeType::AUTO);

/**
 * @brief 解码图像中的所有条码
 * @param image 输入图像
 * @param results 输出结果列表
 * @return 操作结果
 */
Result<Vector<BarcodeResult>> decode_all(const ImageData& image);

/**
 * @brief 生成QR码图像
 * @param data 要编码的数据
 * @param output 输出图像
 * @param size 图像尺寸（像素）
 * @return 操作结果
 */
Result<void> generate_qr(const String& data, ImageData& output, int size = 200);

/**
 * @brief 检查图像中是否包含条码
 * @param image 输入图像
 * @return 是否包含条码
 */
bool has_barcode(const ImageData& image);

} // namespace barcode_utils

/**
 * @brief 条码解码节点
 * 支持一维码和二维码的解码
 */
class BarcodeDecodeNode : public INode {
public:
    BarcodeDecodeNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
    
private:
    BarcodeType parse_barcode_type(const String& type_str);
};

/**
 * @brief QR码专用解码节点
 */
class QRCodeNode : public INode {
public:
    QRCodeNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief DataMatrix码专用解码节点
 */
class DataMatrixNode : public INode {
public:
    DataMatrixNode(const String& instance_id);
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief QR码生成节点
 */
class QRGenerateNode : public INode {
public:
    QRGenerateNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief QR码解码节点（增强版）
 * 支持多角度检测、版本识别、Reed-Solomon纠错
 */
class QRCodeDecodeNode : public INode {
public:
    QRCodeDecodeNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief DataMatrix解码节点
 * 支持DataMatrix码的定位和解码
 */
class DataMatrixDecodeNode : public INode {
public:
    DataMatrixDecodeNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief PDF417解码节点
 * 支持PDF417码的解码
 */
class PDF417DecodeNode : public INode {
public:
    PDF417DecodeNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief Aztec码解码节点
 * 支持Aztec码的定位和解码
 */
class AztecCodeDecodeNode : public INode {
public:
    AztecCodeDecodeNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief MaxiCode解码节点
 * 支持MaxiCode码的解码
 */
class MaxiCodeDecodeNode : public INode {
public:
    MaxiCodeDecodeNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 条码训练节点
 * 用于自定义条码模板的训练和生成
 */
class BarcodeTrainNode : public INode {
public:
    BarcodeTrainNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 条码验证节点
 * 用于条码质量检测和验证
 */
class BarcodeVerifyNode : public INode {
public:
    BarcodeVerifyNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 条码评级节点
 * 按照ISO/IEC标准对条码进行质量评级
 */
class BarcodeGradeNode : public INode {
public:
    BarcodeGradeNode(const String& instance_id);

    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 条码质量评级结果
 */
struct BarcodeGradeResult {
    float overall_grade = 0.0f;       // 总体评级 (0.0-4.0, 对应A-F)
    float decode_grade = 0.0f;        // 解码等级
    float symbol_contrast = 0.0f;     // 符号对比度
    float modulation = 0.0f;          // 调制
    float defects = 0.0f;             // 缺陷
    float decodability = 0.0f;        // 可解码性
    String grade_letter;               // 等级字母 (A/B/C/D/F)
    String standard;                   // 参照标准
    String details;                    // 详细信息
};

} // namespace algorithm
} // namespace ovf