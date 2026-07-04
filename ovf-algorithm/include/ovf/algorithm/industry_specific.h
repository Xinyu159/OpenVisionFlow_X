/**
 * @file industry_specific.h
 * @brief 行业专用算子模块 - PCB检测、半导体检测、包装检测、通用工业检测
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/types.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

// ========== PCB检测模块 ==========

namespace pcb_utils {

/**
 * @brief PCB元器件类型
 */
enum class ComponentType {
    Resistor,       // 电阻
    Capacitor,      // 电容
    IC,             // 集成电路
    Connector,      // 连接器
    Diode,          // 二极管
    Transistor,     // 晶体管
    Inductor,       // 电感
    Unknown         // 未知
};

/**
 * @brief PCB元器件信息
 */
struct ComponentInfo {
    ComponentType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float confidence = 0.0f;
    String description;
    bool is_defective = false;
};

/**
 * @brief PCB线路信息
 */
struct TraceInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float thickness = 0.0f;
    bool is_open = false;       // 断路
    bool is_short = false;      // 短路
    String description;
};

/**
 * @brief PCB焊盘信息
 */
struct PadInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float roundness = 0.0f;     // 圆度
    float coverage = 0.0f;      // 锡覆盖率
    bool is_defective = false;
    String defect_type;
    String description;         // 描述信息
};

/**
 * @brief PCB缺陷类型
 */
enum class PCBDefectType {
    MissingComponent,   // 元器件缺失
    MisalignedComponent, // 元器件偏移
    SolderBridge,       // 焊桥
    SolderInsufficient, // 焊锡不足
    SolderExcess,       // 焊锡过多
    OpenCircuit,        // 断路
    ShortCircuit,       // 短路
    PadDefect,          // 焊盘缺陷
    Scratch,            // 划痕
    Contamination       // 污染
};

/**
 * @brief PCB缺陷信息
 */
struct PCBDefectInfo {
    PCBDefectType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float severity = 0.0f;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 元器件检测
 */
ErrorCode detect_components(const ImageData& image, Vector<ComponentInfo>& components,
                           float min_confidence = 0.5f);

/**
 * @brief 线路检测（断路/短路）
 */
ErrorCode detect_traces(const ImageData& image, Vector<TraceInfo>& traces,
                       int min_width = 2, float thickness_threshold = 0.3f);

/**
 * @brief 焊盘检测
 */
ErrorCode detect_pads(const ImageData& image, Vector<PadInfo>& pads,
                     float min_roundness = 0.7f, float min_coverage = 0.6f);

/**
 * @brief PCB缺陷综合检测
 */
ErrorCode detect_pcb_defects(const ImageData& image, Vector<PCBDefectInfo>& defects,
                            float sensitivity = 0.5f);

/**
 * @brief 获取元器件类型名称
 */
inline String component_type_name(ComponentType type) {
    switch (type) {
        case ComponentType::Resistor: return "电阻";
        case ComponentType::Capacitor: return "电容";
        case ComponentType::IC: return "集成电路";
        case ComponentType::Connector: return "连接器";
        case ComponentType::Diode: return "二极管";
        case ComponentType::Transistor: return "晶体管";
        case ComponentType::Inductor: return "电感";
        default: return "未知";
    }
}

/**
 * @brief 获取PCB缺陷类型名称
 */
inline String pcb_defect_type_name(PCBDefectType type) {
    switch (type) {
        case PCBDefectType::MissingComponent: return "元器件缺失";
        case PCBDefectType::MisalignedComponent: return "元器件偏移";
        case PCBDefectType::SolderBridge: return "焊桥";
        case PCBDefectType::SolderInsufficient: return "焊锡不足";
        case PCBDefectType::SolderExcess: return "焊锡过多";
        case PCBDefectType::OpenCircuit: return "断路";
        case PCBDefectType::ShortCircuit: return "短路";
        case PCBDefectType::PadDefect: return "焊盘缺陷";
        case PCBDefectType::Scratch: return "划痕";
        case PCBDefectType::Contamination: return "污染";
        default: return "未知";
    }
}

} // namespace pcb_utils

// ========== 半导体检测模块 ==========

namespace wafer_utils {

/**
 * @brief 晶圆芯片状态
 */
enum class DieStatus {
    Good,           // 正常
    Defective,      // 缺陷
    Missing,        // 缺失
    Misaligned,     // 偏移
    Cracked,        // 破裂
    Unknown         // 未知
};

/**
 * @brief 晶圆芯片信息
 */
struct DieInfo {
    int row = 0;        // 行号
    int col = 0;        // 列号
    int x = 0;          // 像素坐标X
    int y = 0;          // 坏素坐标Y
    int width = 0;
    int height = 0;
    DieStatus status = DieStatus::Unknown;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 晶圆缺陷类型
 */
enum class WaferDefectType {
    Particle,       // 颗粒污染
    Scratch,        // 划痕
    CrystalDefect,  // 晶体缺陷
    PatternDefect,  // 图形缺陷
    EdgeDefect,     // 边缘缺陷
    OxideDefect,    // 氧化层缺陷
    MetalDefect,    // 金属层缺陷
    Unknown         // 未知
};

/**
 * @brief 晶圆缺陷信息
 */
struct WaferDefectInfo {
    WaferDefectType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float severity = 0.0f;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 对准偏差信息
 */
struct AlignmentInfo {
    float offset_x = 0.0f;      // X方向偏移
    float offset_y = 0.0f;      // Y方向偏移
    float rotation = 0.0f;      // 旋转角度
    float scale_diff = 0.0f;    // 缩放差异
    bool is_aligned = true;     // 是否对准
    float tolerance_x = 5.0f;   // X容差
    float tolerance_y = 5.0f;   // Y容差
    float tolerance_angle = 2.0f; // 角度容差
};

/**
 * @brief 晶圆芯片检测
 */
ErrorCode detect_dies(const ImageData& image, Vector<DieInfo>& dies,
                     int die_width = 100, int die_height = 100);

/**
 * @brief 晶圆缺陷检测
 */
ErrorCode detect_wafer_defects(const ImageData& image, Vector<WaferDefectInfo>& defects,
                              float sensitivity = 0.5f);

/**
 * @brief 芯片对准检测
 */
ErrorCode check_die_alignment(const ImageData& image, AlignmentInfo& alignment,
                             float tolerance_x = 5.0f, float tolerance_y = 5.0f);

/**
 * @brief 获取芯片状态名称
 */
inline String die_status_name(DieStatus status) {
    switch (status) {
        case DieStatus::Good: return "正常";
        case DieStatus::Defective: return "缺陷";
        case DieStatus::Missing: return "缺失";
        case DieStatus::Misaligned: return "偏移";
        case DieStatus::Cracked: return "破裂";
        default: return "未知";
    }
}

/**
 * @brief 获取晶圆缺陷类型名称
 */
inline String wafer_defect_type_name(WaferDefectType type) {
    switch (type) {
        case WaferDefectType::Particle: return "颗粒污染";
        case WaferDefectType::Scratch: return "划痕";
        case WaferDefectType::CrystalDefect: return "晶体缺陷";
        case WaferDefectType::PatternDefect: return "图形缺陷";
        case WaferDefectType::EdgeDefect: return "边缘缺陷";
        case WaferDefectType::OxideDefect: return "氧化层缺陷";
        case WaferDefectType::MetalDefect: return "金属层缺陷";
        default: return "未知";
    }
}

} // namespace wafer_utils

// ========== 包装检测模块 ==========

namespace package_utils {

/**
 * @brief 包装密封状态
 */
enum class SealStatus {
    Good,           // 正常密封
    Broken,         // 密封破损
    Incomplete,     // 密封不完整
    Oversealed,     // 过度密封
    Misaligned,     // 密封偏移
    Unknown         // 未知
};

/**
 * @brief 密封检测结果
 */
struct SealInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    SealStatus status = SealStatus::Unknown;
    float seal_strength = 0.0f;    // 密封强度
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 标签检测结果
 */
struct LabelInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    bool is_present = true;        // 标签是否存在
    bool is_readable = true;       // 标签是否可读
    bool is_correct = true;        // 标签内容是否正确
    float clarity = 0.0f;          // 清晰度
    float alignment_score = 0.0f;  // 对齐评分
    float confidence = 0.0f;       // 置信度
    String content;                // 标签内容
    String description;
};

/**
 * @brief 包装质量问题类型
 */
enum class PackageIssueType {
    Damaged,       // 包装损坏
    Deformed,      // 变形
    Dirty,         // 污染
    Wet,           // 受潮
    IncorrectLabel, // 标签错误
    MissingLabel,  // 标签缺失
    SealProblem,   // 密封问题
    SizeProblem,   // 尺寸问题
    ColorProblem,  // 颜色问题
    Unknown        // 未知
};

/**
 * @brief 包装质量检测结果
 */
struct PackageQualityInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float overall_score = 0.0f;    // 整体评分
    float defect_score = 0.0f;     // 缺陷评分
    bool is_pass = true;           // 是否合格
    Vector<PackageIssueType> issues;
    String description;
};

/**
 * @brief 包装密封检测
 */
ErrorCode detect_seal(const ImageData& image, SealInfo& seal,
                     float min_strength = 0.7f);

/**
 * @brief 包装标签检测
 */
ErrorCode detect_label(const ImageData& image, LabelInfo& label,
                      float min_clarity = 0.6f, float min_alignment = 0.7f);

/**
 * @brief 包装质量检测
 */
ErrorCode detect_package_quality(const ImageData& image, PackageQualityInfo& quality,
                                float pass_threshold = 0.8f);

/**
 * @brief 获取密封状态名称
 */
inline String seal_status_name(SealStatus status) {
    switch (status) {
        case SealStatus::Good: return "正常密封";
        case SealStatus::Broken: return "密封破损";
        case SealStatus::Incomplete: return "密封不完整";
        case SealStatus::Oversealed: return "过度密封";
        case SealStatus::Misaligned: return "密封偏移";
        default: return "未知";
    }
}

/**
 * @brief 获取包装问题类型名称
 */
inline String package_issue_type_name(PackageIssueType type) {
    switch (type) {
        case PackageIssueType::Damaged: return "包装损坏";
        case PackageIssueType::Deformed: return "变形";
        case PackageIssueType::Dirty: return "污染";
        case PackageIssueType::Wet: return "受潮";
        case PackageIssueType::IncorrectLabel: return "标签错误";
        case PackageIssueType::MissingLabel: return "标签缺失";
        case PackageIssueType::SealProblem: return "密封问题";
        case PackageIssueType::SizeProblem: return "尺寸问题";
        case PackageIssueType::ColorProblem: return "颜色问题";
        default: return "未知";
    }
}

} // namespace package_utils

// ========== 通用工业检测模块 ==========

namespace industry_utils {

/**
 * @brief 组装检查项类型
 */
enum class AssemblyItemType {
    ComponentPresence,     // 元器件存在性
    ComponentPosition,     // 元器件位置
    ComponentOrientation,  // 元器件方向
    FastenerPresence,      // 紧固件存在性
    FastenerTorque,        // 紧固件扭矩（视觉推断）
    Alignment,             // 对齐检查
    Gap,                   // 间隙检查
    Overlap,               // 重叠检查
    Connection,            // 连接检查
    Unknown                // 未知
};

/**
 * @brief 组装检查项结果
 */
struct AssemblyCheckItem {
    AssemblyItemType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    bool is_pass = true;
    float deviation = 0.0f;      // 偏差量
    float tolerance = 0.0f;      // 容差
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 组装检查结果
 */
struct AssemblyCheckResult {
    float overall_score = 0.0f;      // 整体评分
    int pass_count = 0;              // 通过项数
    int fail_count = 0;              // 失败项数
    bool is_complete = true;         // 组装是否完成
    Vector<AssemblyCheckItem> items;
    String summary;
};

/**
 * @brief 质量等级
 */
enum class QualityGrade {
    A,      // 优秀
    B,      // 良好
    C,      // 一般
    D,      // 较差
    F,      // 不合格
    Unknown // 未知
};

/**
 * @brief 质量分级标准
 */
struct QualityStandard {
    float a_threshold = 95.0f;   // A级阈值
    float b_threshold = 85.0f;   // B级阈值
    float c_threshold = 75.0f;   // C级阈值
    float d_threshold = 60.0f;   // D级阈值
    // F级: < 60
};

/**
 * @brief 质量分级结果
 */
struct QualityGradeResult {
    QualityGrade grade = QualityGrade::Unknown;
    float score = 0.0f;          // 百分制评分
    float confidence = 0.0f;     // 置信度
    Vector<String> defects;      // 缺陷描述列表
    Vector<String> recommendations; // 改进建议
    String description;
};

/**
 * @brief 组装检查
 */
ErrorCode check_assembly(const ImageData& image, AssemblyCheckResult& result,
                        float tolerance = 5.0f);

/**
 * @brief 质量分级
 */
ErrorCode grade_quality(const ImageData& image, QualityGradeResult& result,
                        const QualityStandard& standard = QualityStandard());

/**
 * @brief 获取组装检查项名称
 */
inline String assembly_item_type_name(AssemblyItemType type) {
    switch (type) {
        case AssemblyItemType::ComponentPresence: return "元器件存在性";
        case AssemblyItemType::ComponentPosition: return "元器件位置";
        case AssemblyItemType::ComponentOrientation: return "元器件方向";
        case AssemblyItemType::FastenerPresence: return "紧固件存在性";
        case AssemblyItemType::FastenerTorque: return "紧固件扭矩";
        case AssemblyItemType::Alignment: return "对齐检查";
        case AssemblyItemType::Gap: return "间隙检查";
        case AssemblyItemType::Overlap: return "重叠检查";
        case AssemblyItemType::Connection: return "连接检查";
        default: return "未知";
    }
}

/**
 * @brief 获取质量等级名称
 */
inline String quality_grade_name(QualityGrade grade) {
    switch (grade) {
        case QualityGrade::A: return "优秀(A)";
        case QualityGrade::B: return "良好(B)";
        case QualityGrade::C: return "一般(C)";
        case QualityGrade::D: return "较差(D)";
        case QualityGrade::F: return "不合格(F)";
        default: return "未知";
    }
}

} // namespace industry_utils

// ========== PCB检测节点 ==========

/**
 * @brief PCB元器件检测节点
 */
class PCBComponentNode : public INode {
public:
    PCBComponentNode(const String& instance_id);
    ~PCBComponentNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_confidence_ = 0.5f;
};

/**
 * @brief PCB线路检测节点
 */
class PCBTraceNode : public INode {
public:
    PCBTraceNode(const String& instance_id);
    ~PCBTraceNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int min_width_ = 2;
    float thickness_threshold_ = 0.3f;
};

/**
 * @brief PCB焊盘检测节点
 */
class PCBPadNode : public INode {
public:
    PCBPadNode(const String& instance_id);
    ~PCBPadNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_roundness_ = 0.7f;
    float min_coverage_ = 0.6f;
};

/**
 * @brief PCB缺陷综合检测节点
 */
class PCBDefectNode : public INode {
public:
    PCBDefectNode(const String& instance_id);
    ~PCBDefectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float sensitivity_ = 0.5f;
};

// ========== 半导体检测节点 ==========

/**
 * @brief 晶圆芯片检测节点
 */
class WaferDieNode : public INode {
public:
    WaferDieNode(const String& instance_id);
    ~WaferDieNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int die_width_ = 100;
    int die_height_ = 100;
};

/**
 * @brief 晶圆缺陷检测节点
 */
class WaferDefectNode : public INode {
public:
    WaferDefectNode(const String& instance_id);
    ~WaferDefectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float sensitivity_ = 0.5f;
};

/**
 * @brief 芯片对准检测节点
 */
class DieAlignmentNode : public INode {
public:
    DieAlignmentNode(const String& instance_id);
    ~DieAlignmentNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float tolerance_x_ = 5.0f;
    float tolerance_y_ = 5.0f;
};

// ========== 包装检测节点 ==========

/**
 * @brief 包装密封检测节点
 */
class PackageSealNode : public INode {
public:
    PackageSealNode(const String& instance_id);
    ~PackageSealNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_strength_ = 0.7f;
};

/**
 * @brief 包装标签检测节点
 */
class PackageLabelNode : public INode {
public:
    PackageLabelNode(const String& instance_id);
    ~PackageLabelNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_clarity_ = 0.6f;
    float min_alignment_ = 0.7f;
};

/**
 * @brief 包装质量检测节点
 */
class PackageQualityNode : public INode {
public:
    PackageQualityNode(const String& instance_id);
    ~PackageQualityNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float pass_threshold_ = 0.8f;
};

// ========== 通用工业检测节点 ==========

/**
 * @brief 组装检查节点
 */
class AssemblyCheckNode : public INode {
public:
    AssemblyCheckNode(const String& instance_id);
    ~AssemblyCheckNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float tolerance_ = 5.0f;
};

/**
 * @brief 质量分级节点
 */
class QualityGradeNode : public INode {
public:
    QualityGradeNode(const String& instance_id);
    ~QualityGradeNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float a_threshold_ = 95.0f;
    float b_threshold_ = 85.0f;
    float c_threshold_ = 75.0f;
    float d_threshold_ = 60.0f;
};

} // namespace algorithm
} // namespace ovf