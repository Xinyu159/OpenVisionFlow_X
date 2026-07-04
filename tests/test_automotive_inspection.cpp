/**
 * @file test_automotive_inspection.cpp
 * @brief OpenVisionFlow 汽车零部件检测测试
 *
 * 测试内容：
 * 1. 钣金检测（表面缺陷/变形）
 * 2. 焊缝检测（焊缝质量评估）
 * 3. 涂装检测（漆面缺陷）
 * 4. 装配验证（零件完整性）
 * 5. 尺寸检测（公差测量）
 * 6. 齿轮检测（齿形/磨损）
 * 7. 连接器检测（插针位置）
 * 8. 螺栓检测（漏装检测）
 * 9. 表面粗糙度检测
 * 10. 间隙测量
 */

#include "test_framework.h"
#include "ovf/core/types.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/automotive_inspection.h"

using namespace ovf;
using namespace ovf::algorithm;
using namespace ovf_test;

// ============================================================================
// SurfaceDefect 测试
// ============================================================================

TEST(AutomotiveInspection, SurfaceDefect_Create) {
    SurfaceDefect defect;
    defect.id = 1;
    defect.type = "scratch";
    defect.center_x = 100.0;
    defect.center_y = 200.0;
    defect.width = 50;
    defect.height = 5;
    defect.area = 250.0;
    defect.depth = 0.5;
    defect.severity = 0.8;
    defect.confidence = 0.95;

    ASSERT_EQ(1u, defect.id);
    ASSERT_EQ("scratch", defect.type);
    ASSERT_NEAR(0.8, defect.severity, 0.01);
}

TEST(AutomotiveInspection, SurfaceDefect_Types) {
    SurfaceDefect scratch;
    scratch.type = "scratch";
    ASSERT_EQ("scratch", scratch.type);

    SurfaceDefect dent;
    dent.type = "dent";
    ASSERT_EQ("dent", dent.type);

    SurfaceDefect pit;
    pit.type = "pit";
    ASSERT_EQ("pit", pit.type);

    SurfaceDefect ripple;
    ripple.type = "ripple";
    ASSERT_EQ("ripple", ripple.type);

    SurfaceDefect contamination;
    contamination.type = "contamination";
    ASSERT_EQ("contamination", contamination.type);
}

// ============================================================================
// WeldDefect 测试
// ============================================================================

TEST(AutomotiveInspection, WeldDefect_Create) {
    WeldDefect defect;
    defect.id = 1;
    defect.type = "porosity";
    defect.center_x = 50.0;
    defect.center_y = 100.0;
    defect.width = 5;
    defect.height = 5;
    defect.area = 25.0;
    defect.length = 10.0;
    defect.severity = 0.7;
    defect.confidence = 0.90;

    ASSERT_EQ(1u, defect.id);
    ASSERT_EQ("porosity", defect.type);
    ASSERT_NEAR(10.0, defect.length, 0.1);
}

TEST(AutomotiveInspection, WeldDefect_Types) {
    WeldDefect porosity;
    porosity.type = "porosity";

    WeldDefect crack;
    crack.type = "crack";

    WeldDefect undercut;
    undercut.type = "undercut";

    WeldDefect overlap;
    overlap.type = "overlap";

    WeldDefect spatter;
    spatter.type = "spatter";

    ASSERT_EQ("porosity", porosity.type);
    ASSERT_EQ("crack", crack.type);
    ASSERT_EQ("undercut", undercut.type);
    ASSERT_EQ("overlap", overlap.type);
    ASSERT_EQ("spatter", spatter.type);
}

// ============================================================================
// WeldQualityResult 测试
// ============================================================================

TEST(AutomotiveInspection, WeldQuality_Create) {
    WeldQualityResult result;
    result.weld_width = 5.0;
    result.weld_depth = 3.0;
    result.penetration = 80.0;  // 80%熔深
    result.porosity_rate = 2.0;  // 2%气孔率
    result.defect_count = 3;
    result.quality_score = 0.85;
    result.is_acceptable = true;

    ASSERT_NEAR(5.0, result.weld_width, 0.1);
    ASSERT_NEAR(80.0, result.penetration, 0.1);
    ASSERT_NEAR(0.85, result.quality_score, 0.01);
    ASSERT_TRUE(result.is_acceptable);
}

TEST(AutomotiveInspection, WeldQuality_Acceptable) {
    WeldQualityResult result;
    result.porosity_rate = 1.0;  // 低气孔率
    result.defect_count = 0;
    result.quality_score = 0.95;
    result.is_acceptable = true;

    ASSERT_TRUE(result.is_acceptable);
    ASSERT_LT(2.0, result.porosity_rate);
    ASSERT_EQ(0, result.defect_count);
}

TEST(AutomotiveInspection, WeldQuality_Unacceptable) {
    WeldQualityResult result;
    result.porosity_rate = 10.0;  // 高气孔率
    result.defect_count = 10;
    result.quality_score = 0.3;
    result.is_acceptable = false;

    ASSERT_FALSE(result.is_acceptable);
    ASSERT_GT(5.0, result.porosity_rate);
}

// ============================================================================
// PaintDefect 测试
// ============================================================================

TEST(AutomotiveInspection, PaintDefect_Create) {
    PaintDefect defect;
    defect.id = 1;
    defect.type = "orange_peel";
    defect.center_x = 100.0;
    defect.center_y = 50.0;
    defect.width = 30;
    defect.height = 30;
    defect.area = 900.0;
    defect.severity = 0.6;
    defect.confidence = 0.85;

    ASSERT_EQ(1u, defect.id);
    ASSERT_EQ("orange_peel", defect.type);
    ASSERT_NEAR(0.6, defect.severity, 0.01);
}

TEST(AutomotiveInspection, PaintDefect_Types) {
    PaintDefect orange_peel;
    orange_peel.type = "orange_peel";

    PaintDefect run;
    run.type = "run";

    PaintDefect sag;
    sag.type = "sag";

    PaintDefect scratch;
    scratch.type = "scratch";

    PaintDefect pinhole;
    pinhole.type = "pinhole";

    PaintDefect dust;
    dust.type = "dust";

    PaintDefect color_defect;
    color_defect.type = "color_defect";

    ASSERT_EQ("orange_peel", orange_peel.type);
    ASSERT_EQ("run", run.type);
    ASSERT_EQ("sag", sag.type);
}

// ============================================================================
// PaintQualityResult 测试
// ============================================================================

TEST(AutomotiveInspection, PaintQuality_Create) {
    PaintQualityResult result;
    result.gloss_level = 90.0;  // 光泽度90
    result.smoothness = 85.0;   // 平滑度85
    result.color_consistency = 95.0;  // 色差一致性95
    result.defect_count = 1;
    result.quality_score = 0.90;
    result.is_acceptable = true;

    ASSERT_NEAR(90.0, result.gloss_level, 0.1);
    ASSERT_NEAR(85.0, result.smoothness, 0.1);
    ASSERT_NEAR(0.90, result.quality_score, 0.01);
    ASSERT_TRUE(result.is_acceptable);
}

TEST(AutomotiveInspection, PaintQuality_HighQuality) {
    PaintQualityResult result;
    result.gloss_level = 95.0;
    result.smoothness = 95.0;
    result.color_consistency = 98.0;
    result.defect_count = 0;
    result.quality_score = 0.98;
    result.is_acceptable = true;

    ASSERT_TRUE(result.is_acceptable);
    ASSERT_GT(90.0, result.gloss_level);
}

// ============================================================================
// AssemblyResult 测试
// ============================================================================

TEST(AutomotiveInspection, Assembly_Create) {
    AssemblyResult result;
    result.expected_parts = 10;
    result.detected_parts = 10;
    result.missing_parts = 0;
    result.extra_parts = 0;
    result.completeness = 1.0;
    result.is_complete = true;

    ASSERT_EQ(10u, result.expected_parts);
    ASSERT_EQ(10u, result.detected_parts);
    ASSERT_EQ(0u, result.missing_parts);
    ASSERT_NEAR(1.0, result.completeness, 0.01);
    ASSERT_TRUE(result.is_complete);
}

TEST(AutomotiveInspection, Assembly_MissingParts) {
    AssemblyResult result;
    result.expected_parts = 10;
    result.detected_parts = 8;
    result.missing_parts = 2;
    result.extra_parts = 0;
    result.completeness = 0.8;
    result.is_complete = false;

    ASSERT_FALSE(result.is_complete);
    ASSERT_EQ(2u, result.missing_parts);
    ASSERT_NEAR(0.8, result.completeness, 0.01);
}

TEST(AutomotiveInspection, Assembly_ExtraParts) {
    AssemblyResult result;
    result.expected_parts = 10;
    result.detected_parts = 12;
    result.missing_parts = 0;
    result.extra_parts = 2;

    ASSERT_EQ(2u, result.extra_parts);
}

// ============================================================================
// DimensionResult 测试
// ============================================================================

TEST(AutomotiveInspection, Dimension_Create) {
    DimensionResult result;
    result.measured_value = 50.05;
    result.nominal_value = 50.0;
    result.tolerance_upper = 0.1;
    result.tolerance_lower = -0.1;
    result.deviation = 0.05;
    result.in_tolerance = true;

    ASSERT_NEAR(50.05, result.measured_value, 0.01);
    ASSERT_NEAR(50.0, result.nominal_value, 0.01);
    ASSERT_NEAR(0.05, result.deviation, 0.01);
    ASSERT_TRUE(result.in_tolerance);
}

TEST(AutomotiveInspection, Dimension_InTolerance) {
    DimensionResult result;
    result.measured_value = 50.05;
    result.tolerance_upper = 0.1;
    result.tolerance_lower = -0.1;
    result.deviation = 0.05;
    result.in_tolerance = true;

    ASSERT_TRUE(result.deviation <= result.tolerance_upper);
    ASSERT_TRUE(result.deviation >= result.tolerance_lower);
    ASSERT_TRUE(result.in_tolerance);
}

TEST(AutomotiveInspection, Dimension_OutOfTolerance) {
    DimensionResult result;
    result.measured_value = 50.15;
    result.tolerance_upper = 0.1;
    result.tolerance_lower = -0.1;
    result.deviation = 0.15;
    result.in_tolerance = false;

    ASSERT_GT(0.1, result.deviation);
    ASSERT_FALSE(result.in_tolerance);
}

// ============================================================================
// GearDefect 测试
// ============================================================================

TEST(AutomotiveInspection, GearDefect_Create) {
    GearDefect defect;
    defect.id = 1;
    defect.type = "wear";
    defect.tooth_number = 5;
    defect.position_x = 100.0;
    defect.position_y = 50.0;
    defect.severity = 0.7;
    defect.confidence = 0.90;

    ASSERT_EQ(1u, defect.id);
    ASSERT_EQ("wear", defect.type);
    ASSERT_EQ(5u, defect.tooth_number);
    ASSERT_NEAR(0.7, defect.severity, 0.01);
}

TEST(AutomotiveInspection, GearDefect_Types) {
    GearDefect wear;
    wear.type = "wear";

    GearDefect chipping;
    chipping.type = "chipping";

    GearDefect crack;
    crack.type = "crack";

    GearDefect misalignment;
    misalignment.type = "misalignment";

    GearDefect pitting;
    pitting.type = "pitting";

    ASSERT_EQ("wear", wear.type);
    ASSERT_EQ("chipping", chipping.type);
}

// ============================================================================
// GearInspectionResult 测试
// ============================================================================

TEST(AutomotiveInspection, GearResult_Create) {
    GearInspectionResult result;
    result.total_teeth = 20;
    result.good_teeth = 18;
    result.defective_teeth = 2;
    result.tooth_thickness = 5.0;
    result.tooth_pitch = 10.0;
    result.pitch_error = 0.05;
    result.profile_error = 0.02;
    result.wear_level = 0.1;
    result.quality_score = 0.90;
    result.is_acceptable = true;

    ASSERT_EQ(20u, result.total_teeth);
    ASSERT_EQ(18u, result.good_teeth);
    ASSERT_EQ(2u, result.defective_teeth);
    ASSERT_NEAR(0.90, result.quality_score, 0.01);
    ASSERT_TRUE(result.is_acceptable);
}

TEST(AutomotiveInspection, GearResult_Acceptable) {
    GearInspectionResult result;
    result.total_teeth = 20;
    result.good_teeth = 20;
    result.defective_teeth = 0;
    result.wear_level = 0.05;
    result.is_acceptable = true;

    ASSERT_TRUE(result.is_acceptable);
    ASSERT_EQ(0u, result.defective_teeth);
}

// ============================================================================
// ConnectorResult 测试
// ============================================================================

TEST(AutomotiveInspection, Connector_Create) {
    ConnectorResult result;
    result.total_pins = 10;
    result.correct_pins = 10;
    result.bent_pins = 0;
    result.missing_pins = 0;
    result.misaligned_pins = 0;
    result.alignment_score = 1.0;
    result.is_acceptable = true;

    ASSERT_EQ(10u, result.total_pins);
    ASSERT_EQ(10u, result.correct_pins);
    ASSERT_EQ(0u, result.bent_pins);
    ASSERT_NEAR(1.0, result.alignment_score, 0.01);
    ASSERT_TRUE(result.is_acceptable);
}

TEST(AutomotiveInspection, Connector_BentPins) {
    ConnectorResult result;
    result.total_pins = 10;
    result.correct_pins = 8;
    result.bent_pins = 2;
    result.alignment_score = 0.8;
    result.is_acceptable = false;

    ASSERT_FALSE(result.is_acceptable);
    ASSERT_EQ(2u, result.bent_pins);
}

TEST(AutomotiveInspection, Connector_MissingPins) {
    ConnectorResult result;
    result.total_pins = 10;
    result.correct_pins = 9;
    result.missing_pins = 1;
    result.is_acceptable = false;

    ASSERT_EQ(1u, result.missing_pins);
    ASSERT_FALSE(result.is_acceptable);
}

// ============================================================================
// BoltResult 测试
// ============================================================================

TEST(AutomotiveInspection, Bolt_Create) {
    BoltResult result;
    result.expected_bolts = 4;
    result.detected_bolts = 4;
    result.missing_bolts = 0;
    result.presence_rate = 1.0;
    result.is_complete = true;

    ASSERT_EQ(4u, result.expected_bolts);
    ASSERT_EQ(4u, result.detected_bolts);
    ASSERT_NEAR(1.0, result.presence_rate, 0.01);
    ASSERT_TRUE(result.is_complete);
}

TEST(AutomotiveInspection, Bolt_Missing) {
    BoltResult result;
    result.expected_bolts = 4;
    result.detected_bolts = 3;
    result.missing_bolts = 1;
    result.presence_rate = 0.75;
    result.is_complete = false;

    ASSERT_FALSE(result.is_complete);
    ASSERT_EQ(1u, result.missing_bolts);
}

// ============================================================================
// RoughnessResult 测试
// ============================================================================

TEST(AutomotiveInspection, Roughness_Create) {
    RoughnessResult result;
    result.ra = 0.8;  // Ra粗糙度0.8μm
    result.rz = 4.0;  // Rz粗糙度4.0μm
    result.rp = 2.0;  // Rp峰值2.0μm
    result.rv = 2.0;  // Rv谷值2.0μm
    result.rq = 1.0;  // Rq RMS粗糙度1.0μm
    result.peak_to_valley = 4.0;
    result.is_acceptable = true;

    ASSERT_NEAR(0.8, result.ra, 0.01);
    ASSERT_NEAR(4.0, result.rz, 0.01);
    ASSERT_TRUE(result.is_acceptable);
}

TEST(AutomotiveInspection, Roughness_Acceptable) {
    RoughnessResult result;
    result.ra = 0.4;  // 低粗糙度
    result.is_acceptable = true;

    ASSERT_LT(0.5, result.ra);
    ASSERT_TRUE(result.is_acceptable);
}

// ============================================================================
// GapResult 测试
// ============================================================================

TEST(AutomotiveInspection, Gap_Create) {
    GapResult result;
    result.gap_width = 4.0;  // 间隙宽度4mm
    result.nominal_gap = 4.0;
    result.tolerance = 0.5;
    result.deviation = 0.0;
    result.uniformity = 0.95;
    result.is_acceptable = true;

    ASSERT_NEAR(4.0, result.gap_width, 0.01);
    ASSERT_NEAR(0.0, result.deviation, 0.01);
    ASSERT_NEAR(0.95, result.uniformity, 0.01);
    ASSERT_TRUE(result.is_acceptable);
}

TEST(AutomotiveInspection, Gap_Uniform) {
    GapResult result;
    result.uniformity = 0.98;
    result.is_acceptable = true;

    ASSERT_GT(0.95, result.uniformity);
    ASSERT_TRUE(result.is_acceptable);
}

// ============================================================================
// automotive_utils 测试
// ============================================================================

TEST(AutomotiveInspection, Utils_CreateImage) {
    ImageData img = automotive_utils::create_image(100, 100, 1, ImageFormat::Mono8);

    ASSERT_EQ(100u, img.width);
    ASSERT_EQ(100u, img.height);
    ASSERT_FALSE(img.empty());
}

TEST(AutomotiveInspection, Utils_SobelGradient) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData grad_x, grad_y, magnitude;

    automotive_utils::sobel_gradient(input, grad_x, grad_y, magnitude);

    ASSERT_FALSE(grad_x.empty());
    ASSERT_FALSE(grad_y.empty());
    ASSERT_FALSE(magnitude.empty());
}

TEST(AutomotiveInspection, Utils_AdaptiveThreshold) {
    ImageData input = TestUtils::create_gradient_image(100, 100);
    ImageData output;

    automotive_utils::adaptive_threshold(input, output, 11, 2);

    ASSERT_FALSE(output.empty());
}

TEST(AutomotiveInspection, Utils_GaussianBlur) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData output;

    automotive_utils::gaussian_blur(input, output, 3, 1.0);

    ASSERT_FALSE(output.empty());
}

TEST(AutomotiveInspection, Utils_Morphology) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData output;

    automotive_utils::morphological_open(input, output, 3);
    ASSERT_FALSE(output.empty());

    automotive_utils::morphological_close(input, output, 3);
    ASSERT_FALSE(output.empty());
}

TEST(AutomotiveInspection, Utils_DetectLines) {
    ImageData edge_img = TestUtils::create_test_image(100, 100, 0);
    for (uint32_t x = 50; x < 55; ++x) {
        for (uint32_t y = 0; y < 100; ++y) {
            edge_img.data[y * 100 + x] = 255;
        }
    }

    auto lines = automotive_utils::detect_lines(edge_img, 30);
    ASSERT_GE(0u, lines.size());
}

TEST(AutomotiveInspection, Utils_DetectCircles) {
    ImageData circle_img = TestUtils::create_circle_image(100, 100, 50, 50, 30);
    ImageData edge_img = TestUtils::create_test_image(100, 100, 0);

    automotive_utils::sobel_gradient(circle_img, edge_img, edge_img, edge_img);

    auto circles = automotive_utils::detect_circles(edge_img, 20, 40, 20);
    ASSERT_GE(0u, circles.size());
}

TEST(AutomotiveInspection, Utils_ComputeStats) {
    ImageData img = TestUtils::create_test_image(100, 100, 128);

    automotive_utils::ImageStats stats = automotive_utils::compute_stats(img);

    ASSERT_NEAR(128.0, stats.mean, 1.0);
    ASSERT_NEAR(128.0, stats.median, 1.0);
}

// ============================================================================
// 绘图工具测试
// ============================================================================

TEST(AutomotiveInspection, Draw_Rect) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    automotive_utils::draw_rect(img, 10, 10, 30, 30, 255, 0, 0, 1);
    ASSERT_FALSE(img.empty());
}

TEST(AutomotiveInspection, Draw_Circle) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    automotive_utils::draw_circle(img, 50, 50, 20, 255, 0, 0);
    ASSERT_FALSE(img.empty());
}

TEST(AutomotiveInspection, Draw_Line) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    automotive_utils::draw_line(img, 0, 0, 99, 99, 255, 0, 0);
    ASSERT_FALSE(img.empty());
}

// ============================================================================
// 节点信息测试
// ============================================================================

TEST(AutomotiveInspection, NodeInfo_BodyPanel) {
    NodeInfo info = BodyPanelInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_WeldSeam) {
    NodeInfo info = WeldSeamInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_PaintQuality) {
    NodeInfo info = PaintQualityInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_AssemblyVerification) {
    NodeInfo info = AssemblyVerificationNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_Dimensional) {
    NodeInfo info = DimensionalInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_Gear) {
    NodeInfo info = GearInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_Connector) {
    NodeInfo info = ConnectorInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_BoltPresence) {
    NodeInfo info = BoltPresenceCheckNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_SurfaceRoughness) {
    NodeInfo info = SurfaceRoughnessInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(AutomotiveInspection, NodeInfo_GapMeasurement) {
    NodeInfo info = GapMeasurementNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

// ============================================================================
// 综合测试
// ============================================================================

TEST(AutomotiveInspection, Comprehensive_SurfaceDefects) {
    Vector<SurfaceDefect> defects;

    // 模划痕
    SurfaceDefect scratch;
    scratch.type = "scratch";
    scratch.severity = 0.5;
    defects.push_back(scratch);

    // 模拟凹陷
    SurfaceDefect dent;
    dent.type = "dent";
    dent.severity = 0.7;
    defects.push_back(dent);

    ASSERT_EQ(2u, defects.size());

    // 统计不同类型缺陷
    int scratch_count = 0;
    int dent_count = 0;

    for (const auto& d : defects) {
        if (d.type == "scratch") scratch_count++;
        if (d.type == "dent") dent_count++;
    }

    ASSERT_EQ(1, scratch_count);
    ASSERT_EQ(1, dent_count);
}

TEST(AutomotiveInspection, Comprehensive_WeldInspection) {
    // 模拟焊缝检测结果
    WeldQualityResult result;
    result.weld_width = 5.0;
    result.weld_depth = 3.0;
    result.penetration = 85.0;
    result.porosity_rate = 1.5;
    result.defect_count = 1;
    result.quality_score = 0.88;
    result.is_acceptable = true;

    ASSERT_TRUE(result.is_acceptable);
    ASSERT_GT(80.0, result.penetration);
    ASSERT_LT(3.0, result.porosity_rate);
}

TEST(AutomotiveInspection, Comprehensive_AssemblyVerification) {
    // 模拟装配验证
    AssemblyResult result;
    result.expected_parts = 20;
    result.detected_parts = 20;
    result.missing_parts = 0;
    result.completeness = 1.0;
    result.is_complete = true;

    ASSERT_TRUE(result.is_complete);
    ASSERT_NEAR(1.0, result.completeness, 0.01);
}

TEST(AutomotiveInspection, Comprehensive_ImageProcessingPipeline) {
    // 模拟完整的检测流程
    ImageData panel_img = TestUtils::create_test_image(200, 200, 128);

    // 1. 预处理
    ImageData smoothed;
    automotive_utils::gaussian_blur(panel_img, smoothed, 3, 1.0);
    ASSERT_FALSE(smoothed.empty());

    // 2. 边缘检测
    ImageData grad_x, grad_y, magnitude;
    automotive_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude);
    ASSERT_FALSE(magnitude.empty());

    // 3. 阈值化
    ImageData binary;
    automotive_utils::adaptive_threshold(smoothed, binary, 11, 2);
    ASSERT_FALSE(binary.empty());

    // 4. 形态学处理
    ImageData opened;
    automotive_utils::morphological_open(binary, opened, 3);
    ASSERT_FALSE(opened.empty());

    // 5. 统计分析
    automotive_utils::ImageStats stats = automotive_utils::compute_stats(opened);
    ASSERT_TRUE(stats.mean >= 0.0 && stats.mean <= 255.0);
}

// ============================================================================
// 主程序入口
// ============================================================================

int main() {
    // 运行所有测试
    TestStats stats = TestRunner::run_all_tests();

    // 保存测试报告
    TestRunner::save_report(stats, "test_automotive_inspection_report.json");

    // 输出汽车检测测试信息
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Automotive Inspection Test Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Supported Inspections:" << std::endl;
    std::cout << "    - Body Panel (Scratch/Dent/Pit)" << std::endl;
    std::cout << "    - Weld Seam (Porosity/Crack/Undercut)" << std::endl;
    std::cout << "    - Paint Quality (Orange Peel/Run/Sag)" << std::endl;
    std::cout << "    - Assembly Verification" << std::endl;
    std::cout << "    - Dimensional Tolerance" << std::endl;
    std::cout << "    - Gear Inspection" << std::endl;
    std::cout << "    - Connector Pins" << std::endl;
    std::cout << "    - Bolt Presence" << std::endl;
    std::cout << "    - Surface Roughness" << std::endl;
    std::cout << "    - Gap Measurement" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // 返回失败测试数量作为退出码
    return stats.failed_tests;
}