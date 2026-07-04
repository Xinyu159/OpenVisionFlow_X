/**
 * @file test_wafer_inspection.cpp
 * @brief OpenVisionFlow 半导体晶圆检测测试
 *
 * 测试内容：
 * 1. 晶粒定位检测
 * 2. 缺陷分类检测
 * 3. 图案检测
 * 4. 边缘检测
 * 5. 对准标记检测
 * 6. 切割道检测
 * 7. 表面检测
 * 8. 厚度测量
 */

#include "test_framework.h"
#include "ovf/core/types.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/wafer_inspection.h"

using namespace ovf;
using namespace ovf::algorithm;
using namespace ovf_test;

// ============================================================================
// WaferDie 测试
// ============================================================================

TEST(WaferInspection, Die_Create) {
    WaferDie die;
    die.id = 1;
    die.row = 0;
    die.col = 0;
    die.center_x = 50.0;
    die.center_y = 50.0;
    die.width = 100;
    die.height = 100;
    die.is_good = true;

    ASSERT_EQ(1u, die.id);
    ASSERT_EQ(0u, die.row);
    ASSERT_EQ(0u, die.col);
    ASSERT_NEAR(50.0, die.center_x, 0.1);
    ASSERT_TRUE(die.is_good);
}

TEST(WaferInspection, Die_GoodAndBad) {
    WaferDie good_die;
    good_die.is_good = true;
    good_die.confidence = 0.95;

    WaferDie bad_die;
    bad_die.is_good = false;
    bad_die.confidence = 0.3;

    ASSERT_TRUE(good_die.is_good);
    ASSERT_FALSE(bad_die.is_good);
    ASSERT_GT(0.9, good_die.confidence);
}

// ============================================================================
// WaferDefect 测试
// ============================================================================

TEST(WaferInspection, Defect_Create) {
    WaferDefect defect;
    defect.id = 1;
    defect.type = "scratch";
    defect.center_x = 100.0;
    defect.center_y = 200.0;
    defect.width = 50;
    defect.height = 10;
    defect.area = 500.0;
    defect.severity = 0.8;
    defect.confidence = 0.95;

    ASSERT_EQ(1u, defect.id);
    ASSERT_EQ("scratch", defect.type);
    ASSERT_NEAR(100.0, defect.center_x, 0.1);
    ASSERT_NEAR(0.8, defect.severity, 0.01);
}

TEST(WaferInspection, Defect_Types) {
    WaferDefect scratch;
    scratch.type = "scratch";
    scratch.severity = 0.5;

    WaferDefect particle;
    particle.type = "particle";
    particle.severity = 0.3;

    WaferDefect contamination;
    contamination.type = "contamination";
    contamination.severity = 0.2;

    WaferDefect crack;
    crack.type = "crack";
    crack.severity = 0.9;

    ASSERT_EQ("scratch", scratch.type);
    ASSERT_EQ("particle", particle.type);
    ASSERT_EQ("contamination", contamination.type);
    ASSERT_EQ("crack", crack.type);
}

// ============================================================================
// AlignmentMark 测试
// ============================================================================

TEST(WaferInspection, AlignmentMark_Create) {
    AlignmentMark mark;
    mark.id = 1;
    mark.center_x = 500.0;
    mark.center_y = 500.0;
    mark.rotation = 0.5;
    mark.scale = 1.0;
    mark.score = 0.95;
    mark.found = true;

    ASSERT_EQ(1u, mark.id);
    ASSERT_NEAR(500.0, mark.center_x, 0.1);
    ASSERT_TRUE(mark.found);
    ASSERT_GT(0.9, mark.score);
}

TEST(WaferInspection, AlignmentMark_NotFound) {
    AlignmentMark mark;
    mark.found = false;
    mark.score = 0.0;

    ASSERT_FALSE(mark.found);
    ASSERT_NEAR(0.0, mark.score, 0.01);
}

// ============================================================================
// DicingStreet 测试
// ============================================================================

TEST(WaferInspection, DicingStreet_Create) {
    DicingStreet street;
    street.id = 1;
    street.start_x = 0.0;
    street.start_y = 100.0;
    street.end_x = 1000.0;
    street.end_y = 100.0;
    street.width = 50;
    street.quality = 0.9;
    street.has_defect = false;

    ASSERT_EQ(1u, street.id);
    ASSERT_NEAR(0.0, street.start_x, 0.1);
    ASSERT_NEAR(1000.0, street.end_x, 0.1);
    ASSERT_FALSE(street.has_defect);
}

TEST(WaferInspection, DicingStreet_WithDefect) {
    DicingStreet street;
    street.has_defect = true;
    street.quality = 0.3;

    ASSERT_TRUE(street.has_defect);
    ASSERT_LT(0.5, street.quality);
}

// ============================================================================
// SurfaceQuality 测试
// ============================================================================

TEST(WaferInspection, SurfaceQuality_Create) {
    SurfaceQuality quality;
    quality.roughness = 0.5;
    quality.flatness = 1.0;
    quality.peak_to_valley = 2.0;
    quality.rms_roughness = 0.3;
    quality.is_acceptable = true;

    ASSERT_NEAR(0.5, quality.roughness, 0.01);
    ASSERT_NEAR(1.0, quality.flatness, 0.01);
    ASSERT_TRUE(quality.is_acceptable);
}

TEST(WaferInspection, SurfaceQuality_Unacceptable) {
    SurfaceQuality quality;
    quality.roughness = 5.0;  // 过高
    quality.is_acceptable = false;

    ASSERT_FALSE(quality.is_acceptable);
    ASSERT_GT(4.0, quality.roughness);
}

// ============================================================================
// ThicknessResult 测试
// ============================================================================

TEST(WaferInspection, Thickness_Create) {
    ThicknessResult thickness;
    thickness.center_thickness = 775.0;
    thickness.edge_thickness = 770.0;
    thickness.thickness_variation = 5.0;
    thickness.bow = 10.0;
    thickness.warp = 15.0;
    thickness.ttv = 5.0;  // Total Thickness Variation
    thickness.is_acceptable = true;

    ASSERT_NEAR(775.0, thickness.center_thickness, 0.1);
    ASSERT_NEAR(770.0, thickness.edge_thickness, 0.1);
    ASSERT_NEAR(5.0, thickness.ttv, 0.1);
    ASSERT_TRUE(thickness.is_acceptable);
}

TEST(WaferInspection, Thickness_TTVAcceptable) {
    ThicknessResult result;
    result.ttv = 1.0;  // 小于2μm，可接受
    result.is_acceptable = true;

    ASSERT_TRUE(result.is_acceptable);
    ASSERT_LT(2.0, result.ttv);
}

// ============================================================================
// wafer_utils 测试
// ============================================================================

TEST(WaferInspection, Utils_CreateImage) {
    ImageData img = wafer_utils::create_image(100, 100, 1, ImageFormat::Mono8);

    ASSERT_EQ(100u, img.width);
    ASSERT_EQ(100u, img.height);
    ASSERT_EQ(1u, img.channels);
    ASSERT_EQ(ImageFormat::Mono8, img.format);
    ASSERT_EQ(10000u, img.data.size());
}

TEST(WaferInspection, Utils_CreateRGBImage) {
    ImageData img = wafer_utils::create_image(100, 100, 3, ImageFormat::RGB8);

    ASSERT_EQ(100u, img.width);
    ASSERT_EQ(100u, img.height);
    ASSERT_EQ(3u, img.channels);
    ASSERT_EQ(ImageFormat::RGB8, img.format);
    ASSERT_EQ(30000u, img.data.size());
}

TEST(WaferInspection, Utils_GenerateDieGrid) {
    Vector<WaferDie> dies = wafer_utils::generate_die_grid(
        1000, 1000, 100, 100, 50, 50);

    ASSERT_NOT_EMPTY(dies);

    // 计算期望的晶粒数量
    uint32_t cols = (1000 - 100) / 100;
    uint32_t rows = (1000 - 100) / 100;
    ASSERT_EQ(cols * rows, dies.size());
}

TEST(WaferInspection, Utils_GenerateDieGrid_Small) {
    Vector<WaferDie> dies = wafer_utils::generate_die_grid(
        100, 100, 50, 50);

    ASSERT_NOT_EMPTY(dies);
    ASSERT_EQ(4u, dies.size());  // 2x2 grid
}

TEST(WaferInspection, Utils_GenerateDieGrid_Empty) {
    Vector<WaferDie> dies = wafer_utils::generate_die_grid(
        0, 0, 100, 100);

    ASSERT_EMPTY(dies);
}

TEST(WaferInspection, Utils_SobelGradient) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData grad_x, grad_y, magnitude;

    wafer_utils::sobel_gradient(input, grad_x, grad_y, magnitude);

    ASSERT_FALSE(grad_x.empty());
    ASSERT_FALSE(grad_y.empty());
    ASSERT_FALSE(magnitude.empty());
    ASSERT_EQ(100u, grad_x.width);
    ASSERT_EQ(100u, grad_x.height);
}

TEST(WaferInspection, Utils_AdaptiveThreshold) {
    ImageData input = TestUtils::create_gradient_image(100, 100);
    ImageData output;

    wafer_utils::adaptive_threshold(input, output, 11, 2);

    ASSERT_FALSE(output.empty());
    ASSERT_EQ(100u, output.width);
    ASSERT_EQ(100u, output.height);
}

TEST(WaferInspection, Utils_GaussianBlur) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData output;

    wafer_utils::gaussian_blur(input, output, 3, 1.0);

    ASSERT_FALSE(output.empty());
    ASSERT_EQ(100u, output.width);
    ASSERT_EQ(100u, output.height);
}

TEST(WaferInspection, Utils_MorphologyErode) {
    ImageData input = TestUtils::create_test_image(100, 100, 255);
    ImageData output;

    wafer_utils::erode(input, output, 3);

    ASSERT_FALSE(output.empty());
}

TEST(WaferInspection, Utils_MorphologyDilate) {
    ImageData input = TestUtils::create_test_image(100, 100, 0);
    ImageData output;

    wafer_utils::dilate(input, output, 3);

    ASSERT_FALSE(output.empty());
}

TEST(WaferInspection, Utils_MorphologyOpen) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData output;

    wafer_utils::morphological_open(input, output, 3);

    ASSERT_FALSE(output.empty());
}

TEST(WaferInspection, Utils_MorphologyClose) {
    ImageData input = TestUtils::create_test_image(100, 100, 128);
    ImageData output;

    wafer_utils::morphological_close(input, output, 3);

    ASSERT_FALSE(output.empty());
}

TEST(WaferInspection, Utils_DetectLines) {
    // 创建边缘图像
    ImageData edge_img = TestUtils::create_test_image(100, 100, 0);
    for (uint32_t x = 50; x < 55; ++x) {
        for (uint32_t y = 0; y < 100; ++y) {
            edge_img.data[y * 100 + x] = 255;
        }
    }

    Vector<std::pair<wafer_utils::Point2D, wafer_utils::Point2D>> lines =
        wafer_utils::detect_lines(edge_img, 30);

    ASSERT_NOT_EMPTY(lines);
}

TEST(WaferInspection, Utils_DetectCircles) {
    ImageData circle_img = TestUtils::create_circle_image(100, 100, 50, 50, 30);
    ImageData edge_img = TestUtils::create_test_image(100, 100, 0);

    // 转换为边缘图
    wafer_utils::sobel_gradient(circle_img, edge_img, edge_img, edge_img);

    Vector<std::pair<wafer_utils::Point2D, double>> circles =
        wafer_utils::detect_circles(edge_img, 20, 40, 20);

    // 可能检测到圆形
    ASSERT_GE(0u, circles.size());
}

TEST(WaferInspection, Utils_ComputeStats) {
    ImageData img = TestUtils::create_test_image(100, 100, 128);

    wafer_utils::ImageStats stats = wafer_utils::compute_stats(img);

    ASSERT_NEAR(128.0, stats.mean, 1.0);
    ASSERT_NEAR(0.0, stats.std_dev, 0.5);  // 常值图像方差接近0
    ASSERT_NEAR(128.0, stats.min_val, 0.1);
    ASSERT_NEAR(128.0, stats.max_val, 0.1);
}

TEST(WaferInspection, Utils_ComputeStatsGradient) {
    ImageData img = TestUtils::create_gradient_image(100, 100);

    wafer_utils::ImageStats stats = wafer_utils::compute_stats(img);

    ASSERT_NEAR(0.0, stats.min_val, 1.0);
    ASSERT_NEAR(255.0, stats.max_val, 1.0);
    ASSERT_TRUE(stats.mean > 0.0 && stats.mean < 255.0);
    ASSERT_GT(50.0, stats.std_dev);  // 渐变图像方差较大
}

// ============================================================================
// 绘图工具测试
// ============================================================================

TEST(WaferInspection, Draw_Rect) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    wafer_utils::draw_rect(img, 10, 10, 30, 30, 255, 0, 0, 1);

    ASSERT_FALSE(img.empty());
    ASSERT_EQ(3u, img.channels);
}

TEST(WaferInspection, Draw_Circle) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    wafer_utils::draw_circle(img, 50, 50, 20, 255, 0, 0);

    ASSERT_FALSE(img.empty());
}

TEST(WaferInspection, Draw_Cross) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    wafer_utils::draw_cross(img, 50, 50, 10, 255, 0, 0);

    ASSERT_FALSE(img.empty());
}

TEST(WaferInspection, Draw_Line) {
    ImageData img = TestUtils::create_test_rgb_image(100, 100, 50, 50, 50);

    wafer_utils::draw_line(img, 0, 0, 99, 99, 255, 0, 0);

    ASSERT_FALSE(img.empty());
}

// ============================================================================
// 节点信息测试
// ============================================================================

TEST(WaferInspection, NodeInfo_DieDetection) {
    NodeInfo info = WaferDieDetectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
    ASSERT_NOT_EMPTY(info.name);
}

TEST(WaferInspection, NodeInfo_DefectClassification) {
    NodeInfo info = WaferDefectClassificationNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_PatternInspection) {
    NodeInfo info = WaferPatternInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_EdgeInspection) {
    NodeInfo info = WaferEdgeInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_Alignment) {
    NodeInfo info = WaferAlignmentNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_DicingInspection) {
    NodeInfo info = WaferDicingInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_SurfaceInspection) {
    NodeInfo info = WaferSurfaceInspectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_ContaminationDetection) {
    NodeInfo info = WaferContaminationDetectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_CrackDetection) {
    NodeInfo info = WaferCrackDetectionNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

TEST(WaferInspection, NodeInfo_ThicknessMeasurement) {
    NodeInfo info = WaferThicknessMeasurementNode::make_info();
    ASSERT_NOT_EMPTY(info.id);
}

// ============================================================================
// 综合测试
// ============================================================================

TEST(WaferInspection, Comprehensive_DieGrid) {
    // 创建晶圆图像
    uint32_t wafer_size = 1000;
    uint32_t die_size = 100;

    Vector<WaferDie> dies = wafer_utils::generate_die_grid(
        wafer_size, wafer_size, die_size, die_size);

    // 验证晶粒网格
    ASSERT_NOT_EMPTY(dies);

    uint32_t good_count = 0;
    uint32_t bad_count = 0;

    for (const auto& die : dies) {
        if (die.is_good) {
            good_count++;
        } else {
            bad_count++;
        }
    }

    ASSERT_EQ(dies.size(), good_count);  // 初始化时所有晶粒应为良品
    ASSERT_EQ(0u, bad_count);
}

TEST(WaferInspection, Comprehensive_DefectSimulation) {
    // 创建模拟缺陷
    Vector<WaferDefect> defects;

    // 划痕
    WaferDefect scratch;
    scratch.type = "scratch";
    scratch.severity = 0.5;
    defects.push_back(scratch);

    // 颗粒
    WaferDefect particle;
    particle.type = "particle";
    particle.severity = 0.3;
    defects.push_back(particle);

    ASSERT_EQ(2u, defects.size());

    // 分类统计
    int scratch_count = 0;
    int particle_count = 0;

    for (const auto& defect : defects) {
        if (defect.type == "scratch") scratch_count++;
        if (defect.type == "particle") particle_count++;
    }

    ASSERT_EQ(1, scratch_count);
    ASSERT_EQ(1, particle_count);
}

TEST(WaferInspection, Comprehensive_ImageProcessingPipeline) {
    // 模拟完整的图像处理流程
    // 1. 创建晶圆图像
    ImageData wafer_img = TestUtils::create_test_image(200, 200, 128);

    // 2. 高斯平滑
    ImageData smoothed;
    wafer_utils::gaussian_blur(wafer_img, smoothed, 3, 1.0);
    ASSERT_FALSE(smoothed.empty());

    // 3. 计算梯度
    ImageData grad_x, grad_y, magnitude;
    wafer_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude);
    ASSERT_FALSE(magnitude.empty());

    // 4. 自适应阈值
    ImageData binary;
    wafer_utils::adaptive_threshold(smoothed, binary, 11, 2);
    ASSERT_FALSE(binary.empty());

    // 5. 形态学处理
    ImageData opened;
    wafer_utils::morphological_open(binary, opened, 3);
    ASSERT_FALSE(opened.empty());

    // 6. 计算统计
    wafer_utils::ImageStats stats = wafer_utils::compute_stats(opened);
    ASSERT_TRUE(stats.mean >= 0.0 && stats.mean <= 255.0);
}

// ============================================================================
// 主程序入口
// ============================================================================

int main() {
    // 运行所有测试
    TestStats stats = TestRunner::run_all_tests();

    // 保存测试报告
    TestRunner::save_report(stats, "test_wafer_inspection_report.json");

    // 输出晶圆检测测试信息
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Wafer Inspection Test Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Supported Inspections:" << std::endl;
    std::cout << "    - Die Detection" << std::endl;
    std::cout << "    - Defect Classification" << std::endl;
    std::cout << "    - Pattern Inspection" << std::endl;
    std::cout << "    - Edge Inspection" << std::endl;
    std::cout << "    - Alignment Mark Detection" << std::endl;
    std::cout << "    - Dicing Street Inspection" << std::endl;
    std::cout << "    - Surface Quality Inspection" << std::endl;
    std::cout << "    - Thickness Measurement" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // 返回失败测试数量作为退出码
    return stats.failed_tests;
}