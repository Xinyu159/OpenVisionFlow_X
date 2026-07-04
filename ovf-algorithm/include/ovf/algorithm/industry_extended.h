/**
 * @file industry_extended.h
 * @brief 行业扩展算子模块 - 医药、食品、纺织、汽车行业专用检测节点
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

// ========== 医药行业检测模块 ==========

namespace pharma_utils {

/**
 * @brief 药片信息
 */
struct PillInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float area = 0.0f;
    float roundness = 0.0f;        // 圆度
    float color_value = 0.0f;      // 颜色值
    String shape_type;              // 形状类型：圆形、椭圆形等
    bool is_defective = false;
    String defect_type;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 药片缺陷类型
 */
enum class PillDefectType {
    Crack,          // 裂纹
    Spot,           // 斑点
    Deformation,    // 变形
    ColorAnomaly,   // 颜色异常
    SizeAnomaly,    // 尺寸异常
    Missing,        // 缺失
    Unknown
};

/**
 * @brief 药片缺陷信息
 */
struct PillDefectInfo {
    PillDefectType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float severity = 0.0f;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 胶囊信息
 */
struct CapsuleInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float fill_ratio = 0.0f;       // 填充度
    float integrity = 0.0f;        // 完整性
    bool is_complete = true;       // 是否完整
    bool is_filled = true;         // 是否填充
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 注射器信息
 */
struct SyringeInfo {
    int x = 0;
    int y = 0;
    int needle_length = 0;         // 针头长度
    int body_length = 0;           // 针筒长度
    bool needle_present = true;    // 针头是否存在
    bool scale_visible = true;     // 刻度是否可见
    int scale_count = 0;           // 刻度数量
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 药瓶信息
 */
struct VialInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float liquid_level = 0.0f;     // 液位百分比
    bool is_sealed = true;         // 是否密封
    bool cap_present = true;       // 盖子是否存在
    float seal_quality = 0.0f;     // 密封质量
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 泡罩包装信息
 */
struct BlisterPackInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int total_cavities = 0;        // 总泡罩数
    int filled_cavities = 0;       // 已填充数
    int empty_cavities = 0;        // 空泡罩数
    int defective_cavities = 0;    // 缺陷泡罩数
    float integrity = 0.0f;        // 整体完整性
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 药片检测
 */
ErrorCode detect_pills(const ImageData& image, Vector<PillInfo>& pills,
                      float min_roundness = 0.7f, int min_size = 20);

/**
 * @brief 药片缺陷检测
 */
ErrorCode detect_pill_defects(const ImageData& image, Vector<PillDefectInfo>& defects,
                             float sensitivity = 0.5f);

/**
 * @brief 胶囊检测
 */
ErrorCode detect_capsules(const ImageData& image, Vector<CapsuleInfo>& capsules,
                         float min_fill_ratio = 0.8f);

/**
 * @brief 注射器检测
 */
ErrorCode detect_syringes(const ImageData& image, Vector<SyringeInfo>& syringes,
                         int min_needle_length = 20);

/**
 * @brief 药瓶检测
 */
ErrorCode detect_vials(const ImageData& image, Vector<VialInfo>& vials,
                      float min_liquid_level = 0.3f);

/**
 * @brief 泡罩包装检测
 */
ErrorCode detect_blister_pack(const ImageData& image, BlisterPackInfo& pack,
                             int expected_cavities = 10);

/**
 * @brief 获取药片缺陷类型名称
 */
inline String pill_defect_type_name(PillDefectType type) {
    switch (type) {
        case PillDefectType::Crack: return "裂纹";
        case PillDefectType::Spot: return "斑点";
        case PillDefectType::Deformation: return "变形";
        case PillDefectType::ColorAnomaly: return "颜色异常";
        case PillDefectType::SizeAnomaly: return "尺寸异常";
        case PillDefectType::Missing: return "缺失";
        default: return "未知";
    }
}

} // namespace pharma_utils

// ========== 食品行业检测模块 ==========

namespace food_utils {

/**
 * @brief 食品新鲜度信息
 */
struct FreshnessInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float freshness_score = 0.0f;  // 新鲜度评分 0-1
    float color_score = 0.0f;      // 颜色评分
    float texture_score = 0.0f;    // 纹理评分
    String freshness_level;        // 新鲜、一般、不新鲜
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 食品异物类型
 */
enum class ContaminateType {
    Hair,           // 毛发
    Plastic,        // 塑料
    Metal,          // 金属
    Glass,          // 玻璃
    Insect,         // 昆虫
    Dust,           // 灰尘
    Other,          // 其他异物
    Unknown
};

/**
 * @brief 异物污染信息
 */
struct ContaminateInfo {
    ContaminateType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float severity = 0.0f;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 水果质量信息
 */
struct FruitQualityInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float maturity = 0.0f;         // 成熟度 0-1
    float damage_score = 0.0f;     // 损伤评分
    bool has_damage = false;       // 是否有损伤
    bool has_bruise = false;       // 是否有碰伤
    bool is_mature = true;         // 是否成熟
    float quality_score = 0.0f;    // 综合质量评分
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 瓶装检测结果
 */
struct BottleInspectInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float liquid_level = 0.0f;     // 液位百分比
    bool label_present = true;     // 标签是否存在
    bool label_correct = true;     // 标签是否正确
    float label_alignment = 0.0f;  // 标签对齐度
    bool is_sealed = true;         // 是否密封
    float seal_quality = 0.0f;     // 密封质量
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 肉类质量信息
 */
struct MeatQualityInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float color_score = 0.0f;      // 颜色评分（肉质颜色）
    float texture_score = 0.0f;    // 纹理评分
    float freshness = 0.0f;        // 新鲜度
    float marbling = 0.0f;         // 肥瘦分布（大理石纹）
    float quality_grade = 0.0f;    // 质量等级
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 包装完整性信息
 */
struct PackageIntegrityInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float integrity_score = 0.0f;  // 完整性评分
    bool has_damage = false;       // 是否有损坏
    bool has_leak = false;         // 是否有泄漏
    bool has_deformation = false;  // 是否变形
    bool is_complete = true;       // 是否完整
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 食品新鲜度检测
 */
ErrorCode detect_freshness(const ImageData& image, FreshnessInfo& freshness,
                          float threshold = 0.6f);

/**
 * @brief 食品异物检测
 */
ErrorCode detect_contaminates(const ImageData& image, Vector<ContaminateInfo>& contaminates,
                             float sensitivity = 0.5f);

/**
 * @brief 水果质量检测
 */
ErrorCode detect_fruit_quality(const ImageData& image, FruitQualityInfo& quality,
                              float maturity_threshold = 0.5f);

/**
 * @brief 瓶装检测
 */
ErrorCode inspect_bottle(const ImageData& image, BottleInspectInfo& bottle,
                        float min_liquid_level = 0.3f);

/**
 * @brief 肉类质量检测
 */
ErrorCode detect_meat_quality(const ImageData& image, MeatQualityInfo& quality,
                             float min_quality = 0.6f);

/**
 * @brief 包装完整性检测
 */
ErrorCode check_package_integrity(const ImageData& image, PackageIntegrityInfo& integrity,
                                 float min_integrity = 0.8f);

/**
 * @brief 获取异物类型名称
 */
inline String contaminate_type_name(ContaminateType type) {
    switch (type) {
        case ContaminateType::Hair: return "毛发";
        case ContaminateType::Plastic: return "塑料";
        case ContaminateType::Metal: return "金属";
        case ContaminateType::Glass: return "玻璃";
        case ContaminateType::Insect: return "昆虫";
        case ContaminateType::Dust: return "灰尘";
        case ContaminateType::Other: return "其他异物";
        default: return "未知";
    }
}

} // namespace food_utils

// ========== 纺织行业检测模块 ==========

namespace textile_utils {

/**
 * @brief 织物缺陷类型
 */
enum class FabricDefectType {
    BrokenYarn,     // 断纱
    Stain,          // 污渍
    Hole,           // 孔洞
    Knot,           // 结头
    UnevenWeave,    // 织造不均
    ColorPatch,     // 色斑
    LooseThread,    // 松线
    Other,          // 其他缺陷
    Unknown
};

/**
 * @brief 织物缺陷信息
 */
struct FabricDefectInfo {
    FabricDefectType type;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float severity = 0.0f;
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 织物图案信息
 */
struct FabricPatternInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float pattern_match = 0.0f;    // 图案匹配度
    bool is_matched = true;        // 是否匹配
    float deviation = 0.0f;        // 偏差量
    float confidence = 0.0f;
    String pattern_type;           // 图案类型
    String description;
};

/**
 * @brief 织物颜色信息
 */
struct FabricColorInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float color_deviation = 0.0f;  // 色差值
    float uniformity = 0.0f;       // 染色均匀度
    bool has_color_diff = false;   // 是否有色差
    bool is_uniform = true;        // 是否均匀
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 织物密度信息
 */
struct FabricDensityInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int warp_density = 0;          // 经纱密度
    int weft_density = 0;          // 纬纱密度
    float density_uniformity = 0.0f; // 密度均匀性
    bool is_uniform = true;        // 是否均匀
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 服装检测信息
 */
struct GarmentInspectInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int defect_count = 0;          // 缺陷数量
    bool has_stitch_defect = false; // 是否有缝线缺陷
    bool has_fabric_defect = false; // 是否有织物缺陷
    bool has_size_defect = false;  // 是否有尺寸缺陷
    float quality_score = 0.0f;    // 质量评分
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 纱线计数信息
 */
struct ThreadCountInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int thread_count = 0;          // 纱线数量
    int expected_count = 0;        // 预期数量
    bool is_correct = true;        // 数量是否正确
    float deviation = 0.0f;        // 偏差
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 织物缺陷检测
 */
ErrorCode detect_fabric_defects(const ImageData& image, Vector<FabricDefectInfo>& defects,
                               float sensitivity = 0.5f);

/**
 * @brief 织物图案检测
 */
ErrorCode detect_fabric_pattern(const ImageData& image, FabricPatternInfo& pattern,
                               float match_threshold = 0.8f);

/**
 * @brief 织物颜色检测
 */
ErrorCode detect_fabric_color(const ImageData& image, FabricColorInfo& color,
                             float max_deviation = 10.0f);

/**
 * @brief 织物密度检测
 */
ErrorCode detect_fabric_density(const ImageData& image, FabricDensityInfo& density,
                               int expected_warp = 100, int expected_weft = 100);

/**
 * @brief 服装检测
 */
ErrorCode inspect_garment(const ImageData& image, GarmentInspectInfo& garment,
                         float min_quality = 0.7f);

/**
 * @brief 纱线计数检测
 */
ErrorCode count_threads(const ImageData& image, ThreadCountInfo& count,
                        int expected_count = 100);

/**
 * @brief 获取织物缺陷类型名称
 */
inline String fabric_defect_type_name(FabricDefectType type) {
    switch (type) {
        case FabricDefectType::BrokenYarn: return "断纱";
        case FabricDefectType::Stain: return "污渍";
        case FabricDefectType::Hole: return "孔洞";
        case FabricDefectType::Knot: return "结头";
        case FabricDefectType::UnevenWeave: return "织造不均";
        case FabricDefectType::ColorPatch: return "色斑";
        case FabricDefectType::LooseThread: return "松线";
        case FabricDefectType::Other: return "其他缺陷";
        default: return "未知";
    }
}

} // namespace textile_utils

// ========== 汽车行业检测模块 ==========

namespace automotive_utils {

/**
 * @brief 涂装质量信息
 */
struct PaintQualityInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float paint_score = 0.0f;      // 涂装评分
    int defect_count = 0;          // 缺陷数量
    bool has_scratch = false;      // 是否有划痕
    bool has_bubble = false;       // 是否有气泡
    bool has_peeling = false;      // 是否有脱落
    bool has_dust = false;         // 是否有灰尘
    bool has_color_diff = false;   // 是否有色差
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 焊缝信息
 */
struct WeldInspectInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float weld_quality = 0.0f;     // 焊缝质量
    bool has_crack = false;        // 是否有裂纹
    bool has_porosity = false;     // 是否有气孔
    bool has_undercut = false;     // 是否有咬边
    bool is_continuous = true;     // 是否连续
    int weld_points = 0;           // 焊点数量
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 钣金间隙信息
 */
struct PanelGapInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float gap_size = 0.0f;         // 间隙大小
    float gap_uniformity = 0.0f;   // 间隙均匀度
    bool is_uniform = true;        // 是否均匀
    float deviation = 0.0f;        // 偏差
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 零部件存在检测信息
 */
struct PartPresenceInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int total_parts = 0;           // 总零件数
    int present_parts = 0;         // 存在的零件数
    int missing_parts = 0;         // 缺失的零件数
    bool is_complete = true;       // 是否完整
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 表面粗糙度信息
 */
struct SurfaceRoughnessInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float roughness_ra = 0.0f;     // Ra值（平均粗糙度）
    float roughness_rz = 0.0f;     // Rz值（最大粗糙度）
    bool is_smooth = true;         // 是否光滑
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 尺寸检测信息
 */
struct DimensionCheckInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float measured_value = 0.0f;   // 测量值
    float expected_value = 0.0f;   // 预期值
    float tolerance = 0.0f;        // 容差
    float deviation = 0.0f;        // 偏差
    bool is_correct = true;        // 是否正确
    String dimension_type;         // 尺寸类型：孔径、间距等
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 组装验证信息
 */
struct AssemblyVerifyInfo {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    float assembly_score = 0.0f;   // 组装评分
    int check_items = 0;           // 检查项数
    int pass_items = 0;            // 通过项数
    int fail_items = 0;            // 失败项数
    bool is_complete = true;       // 是否完整组装
    bool is_correct = true;        // 是否正确组装
    float confidence = 0.0f;
    String description;
};

/**
 * @brief 涂装质量检测
 */
ErrorCode inspect_paint_quality(const ImageData& image, PaintQualityInfo& paint,
                               float min_score = 0.8f);

/**
 * @brief 焊缝检测
 */
ErrorCode inspect_weld(const ImageData& image, WeldInspectInfo& weld,
                      float min_quality = 0.7f);

/**
 * @brief 钣金间隙检测
 */
ErrorCode check_panel_gap(const ImageData& image, PanelGapInfo& gap,
                         float expected_gap = 5.0f, float tolerance = 1.0f);

/**
 * @brief 零部件存在检测
 */
ErrorCode check_part_presence(const ImageData& image, PartPresenceInfo& presence,
                             int expected_parts = 10);

/**
 * @brief 表面粗糙度检测
 */
ErrorCode check_surface_roughness(const ImageData& image, SurfaceRoughnessInfo& roughness,
                                 float max_roughness = 10.0f);

/**
 * @brief 尺寸检测
 */
ErrorCode check_dimension(const ImageData& image, DimensionCheckInfo& dimension,
                         float expected_value, float tolerance = 0.5f);

/**
 * @brief 组装验证检测
 */
ErrorCode verify_assembly(const ImageData& image, AssemblyVerifyInfo& assembly,
                         float min_score = 0.9f);

} // namespace automotive_utils

// ========== 医药行业检测节点 ==========

/**
 * @brief 药片检测节点（数量、形状、颜色）
 */
class PillDetectNode : public INode {
public:
    PillDetectNode(const String& instance_id);
    ~PillDetectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_roundness_ = 0.7f;
    int min_size_ = 20;
};

/**
 * @brief 药片缺陷检测节点（裂纹、斑点、变形）
 */
class PillDefectNode : public INode {
public:
    PillDefectNode(const String& instance_id);
    ~PillDefectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float sensitivity_ = 0.5f;
};

/**
 * @brief 胶囊检测节点（完整性、填充度）
 */
class CapsuleInspectNode : public INode {
public:
    CapsuleInspectNode(const String& instance_id);
    ~CapsuleInspectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_fill_ratio_ = 0.8f;
};

/**
 * @brief 注射器检测节点（针头、刻度）
 */
class SyringeCheckNode : public INode {
public:
    SyringeCheckNode(const String& instance_id);
    ~SyringeCheckNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int min_needle_length_ = 20;
};

/**
 * @brief 药瓶检测节点（密封、液位）
 */
class VialInspectNode : public INode {
public:
    VialInspectNode(const String& instance_id);
    ~VialInspectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_liquid_level_ = 0.3f;
};

/**
 * @brief 泡罩包装检测节点（完整性、位置）
 */
class BlisterPackNode : public INode {
public:
    BlisterPackNode(const String& instance_id);
    ~BlisterPackNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int expected_cavities_ = 10;
};

// ========== 食品行业检测节点 ==========

/**
 * @brief 食品新鲜度检测节点（颜色、纹理）
 */
class FoodFreshnessNode : public INode {
public:
    FoodFreshnessNode(const String& instance_id);
    ~FoodFreshnessNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float threshold_ = 0.6f;
};

/**
 * @brief 食品异物污染检测节点
 */
class FoodContaminateNode : public INode {
public:
    FoodContaminateNode(const String& instance_id);
    ~FoodContaminateNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float sensitivity_ = 0.5f;
};

/**
 * @brief 水果质量检测节点（成熟度、损伤）
 */
class FruitQualityNode : public INode {
public:
    FruitQualityNode(const String& instance_id);
    ~FruitQualityNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float maturity_threshold_ = 0.5f;
};

/**
 * @brief 瓶装检测节点（液位、标签、密封）
 */
class BottleInspectNode : public INode {
public:
    BottleInspectNode(const String& instance_id);
    ~BottleInspectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_liquid_level_ = 0.3f;
};

/**
 * @brief 肉类质量检测节点（颜色、纹理）
 */
class MeatQualityNode : public INode {
public:
    MeatQualityNode(const String& instance_id);
    ~MeatQualityNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_quality_ = 0.6f;
};

/**
 * @brief 包装完整性检测节点
 */
class PackageIntegrityNode : public INode {
public:
    PackageIntegrityNode(const String& instance_id);
    ~PackageIntegrityNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_integrity_ = 0.8f;
};

// ========== 纺织行业检测节点 ==========

/**
 * @brief 织物缺陷检测节点（断纱、污渍、孔洞）
 */
class FabricDefectNode : public INode {
public:
    FabricDefectNode(const String& instance_id);
    ~FabricDefectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float sensitivity_ = 0.5f;
};

/**
 * @brief 织物图案检测节点（花纹匹配）
 */
class FabricPatternNode : public INode {
public:
    FabricPatternNode(const String& instance_id);
    ~FabricPatternNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float match_threshold_ = 0.8f;
};

/**
 * @brief 织物颜色检测节点（色差、染色均匀）
 */
class FabricColorNode : public INode {
public:
    FabricColorNode(const String& instance_id);
    ~FabricColorNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float max_deviation_ = 10.0f;
};

/**
 * @brief 织物密度检测节点（纱线密度）
 */
class FabricDensityNode : public INode {
public:
    FabricDensityNode(const String& instance_id);
    ~FabricDensityNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int expected_warp_ = 100;
    int expected_weft_ = 100;
};

/**
 * @brief 服装检测节点（缝线、瑕疵）
 */
class GarmentInspectNode : public INode {
public:
    GarmentInspectNode(const String& instance_id);
    ~GarmentInspectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_quality_ = 0.7f;
};

/**
 * @brief 纱线计数检测节点
 */
class ThreadCountNode : public INode {
public:
    ThreadCountNode(const String& instance_id);
    ~ThreadCountNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int expected_count_ = 100;
};

// ========== 汽车行业检测节点 ==========

/**
 * @brief 涂装质量检测节点（漆面缺陷）
 */
class PaintQualityNode : public INode {
public:
    PaintQualityNode(const String& instance_id);
    ~PaintQualityNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_score_ = 0.8f;
};

/**
 * @brief 焊缝检测节点（焊点质量）
 */
class WeldInspectNode : public INode {
public:
    WeldInspectNode(const String& instance_id);
    ~WeldInspectNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_quality_ = 0.7f;
};

/**
 * @brief 钣金间隙检测节点（间隙均匀度）
 */
class PanelGapNode : public INode {
public:
    PanelGapNode(const String& instance_id);
    ~PanelGapNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float expected_gap_ = 5.0f;
    float tolerance_ = 1.0f;
};

/**
 * @brief 零部件存在检测节点（装配完整性）
 */
class PartPresenceNode : public INode {
public:
    PartPresenceNode(const String& instance_id);
    ~PartPresenceNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    int expected_parts_ = 10;
};

/**
 * @brief 表面粗糙度检测节点
 */
class SurfaceRoughnessNode : public INode {
public:
    SurfaceRoughnessNode(const String& instance_id);
    ~SurfaceRoughnessNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float max_roughness_ = 10.0f;
};

/**
 * @brief 尺寸检测节点（孔径、间距）
 */
class DimensionCheckNode : public INode {
public:
    DimensionCheckNode(const String& instance_id);
    ~DimensionCheckNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float expected_value_ = 0.0f;
    float tolerance_ = 0.5f;
};

/**
 * @brief 组装验证检测节点
 */
class AssemblyVerifyNode : public INode {
public:
    AssemblyVerifyNode(const String& instance_id);
    ~AssemblyVerifyNode() override = default;

    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();

private:
    float min_score_ = 0.9f;
};

} // namespace algorithm
} // namespace ovf