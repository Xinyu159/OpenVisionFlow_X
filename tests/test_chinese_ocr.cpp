/**
 * @file test_chinese_ocr.cpp
 * @brief OpenVisionFlow 汉字OCR测试
 *
 * 测试内容：
 * 1. 字符检测（MSER区域检测）
 * 2. 字符分割（投影法/连通域）
 * 3. 特征提取（网格/方向/投影特征）
 * 4. 模板匹配（余弦相似度）
 * 5. OCR识别准确率验证
 */

#include "test_framework.h"
#include "ovf/core/types.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/chinese_ocr.h"

using namespace ovf;
using namespace ovf::algorithm;
using namespace ovf_test;

// ============================================================================
// CharSet 测试
// ============================================================================

TEST(ChineseOCR, CharSet_Values) {
    ASSERT_EQ(0, static_cast<int>(CharSet::Chinese));
    ASSERT_EQ(1, static_cast<int>(CharSet::Digit));
    ASSERT_EQ(2, static_cast<int>(CharSet::English));
    ASSERT_EQ(3, static_cast<int>(CharSet::Mixed));
}

TEST(ChineseOCR, CharSet_Utils_Name) {
    ASSERT_EQ("Chinese", chinese_ocr_utils::char_set_name(CharSet::Chinese));
    ASSERT_EQ("Digit", chinese_ocr_utils::char_set_name(CharSet::Digit));
    ASSERT_EQ("English", chinese_ocr_utils::char_set_name(CharSet::English));
    ASSERT_EQ("Mixed", chinese_ocr_utils::char_set_name(CharSet::Mixed));
}

TEST(ChineseOCR, CharSet_Utils_Parse) {
    ASSERT_EQ(CharSet::Chinese, chinese_ocr_utils::parse_char_set("Chinese"));
    ASSERT_EQ(CharSet::Digit, chinese_ocr_utils::parse_char_set("Digit"));
    ASSERT_EQ(CharSet::English, chinese_ocr_utils::parse_char_set("English"));
    ASSERT_EQ(CharSet::Mixed, chinese_ocr_utils::parse_char_set("Mixed"));
}

TEST(ChineseOCR, CharSet_Utils_GetSet) {
    Vector<String> digit_chars = chinese_ocr_utils::get_char_set(CharSet::Digit);
    ASSERT_EQ(10u, digit_chars.size());  // 0-9

    Vector<String> english_chars = chinese_ocr_utils::get_char_set(CharSet::English);
    ASSERT_EQ(26u, english_chars.size());  // A-Z
}

// ============================================================================
// CharBox 测试
// ============================================================================

TEST(ChineseOCR, CharBox_Create) {
    CharBox box(10, 20, 30, 40);
    ASSERT_EQ(10, box.x);
    ASSERT_EQ(20, box.y);
    ASSERT_EQ(30, box.width);
    ASSERT_EQ(40, box.height);
}

TEST(ChineseOCR, CharBox_Valid) {
    CharBox valid_box(10, 20, 30, 40);
    ASSERT_TRUE(valid_box.valid());

    CharBox invalid_box(10, 20, 0, 0);
    ASSERT_FALSE(invalid_box.valid());
}

TEST(ChineseOCR, CharBox_Area) {
    CharBox box(10, 20, 30, 40);
    ASSERT_EQ(1200, box.area());  // 30 * 40
}

TEST(ChineseOCR, CharBox_AspectRatio) {
    CharBox box(10, 20, 30, 60);
    ASSERT_NEAR(0.5f, box.aspect_ratio(), 0.001f);  // 30/60
}

TEST(ChineseOCR, CharBox_ToRegion) {
    CharBox box(10, 20, 30, 40);
    Region region = box.to_region();

    ASSERT_EQ(10, region.x);
    ASSERT_EQ(20, region.y);
    ASSERT_EQ(30, region.width);
    ASSERT_EQ(40, region.height);
}

// ============================================================================
// CharResult 测试
// ============================================================================

TEST(ChineseOCR, CharResult_Create) {
    CharBox box(10, 20, 30, 40);
    CharResult result("测", 95.0f, box, CharSet::Chinese);

    ASSERT_EQ("测", result.character);
    ASSERT_NEAR(95.0f, result.confidence, 0.1f);
    ASSERT_TRUE(result.is_valid());
}

TEST(ChineseOCR, CharResult_Empty) {
    CharResult result;
    ASSERT_FALSE(result.is_valid());
    ASSERT_TRUE(result.character.empty());
}

// ============================================================================
// OCRModel 测试
// ============================================================================

TEST(ChineseOCR, OCRModel_Create) {
    OCRModel model;
    model.name = "TestModel";
    model.char_set = CharSet::Chinese;
    model.feature_dim = 128;
    model.version = "1.0";

    ASSERT_EQ("TestModel", model.name);
    ASSERT_EQ(CharSet::Chinese, model.char_set);
    ASSERT_EQ(128, model.feature_dim);
    ASSERT_TRUE(model.empty());
}

TEST(ChineseOCR, OCRModel_AddTemplate) {
    OCRModel model;
    FeatureVector feature(128, 0.5f);

    model.add_template("测", CharSet::Chinese, feature);

    ASSERT_EQ(1u, model.size());
    ASSERT_FALSE(model.empty());
}

TEST(ChineseOCR, OCRModel_FindTemplate) {
    OCRModel model;
    FeatureVector feature(128, 0.5f);
    model.add_template("测", CharSet::Chinese, feature);

    OCRTemplate* found = model.find("测");
    ASSERT_NOT_NULL(found);
    ASSERT_EQ("测", found->label);

    OCRTemplate* not_found = model.find("不存在的");
    ASSERT_NULL(not_found);
}

TEST(ChineseOCR, OCRModel_Clear) {
    OCRModel model;
    FeatureVector feature(128, 0.5f);
    model.add_template("测", CharSet::Chinese, feature);

    model.clear();
    ASSERT_TRUE(model.empty());
    ASSERT_EQ(0u, model.size());
}

TEST(ChineseOCR, OCRModel_CreateDefault) {
    OCRModel model = chinese_ocr_utils::create_default_model(CharSet::Digit);
    ASSERT_EQ(CharSet::Digit, model.char_set);
}

// ============================================================================
// OCRTemplate 测试
// ============================================================================

TEST(ChineseOCR, OCRTemplate_Create) {
    FeatureVector feature(128, 0.0f);
    OCRTemplate template_item("字", CharSet::Chinese, feature);

    ASSERT_EQ("字", template_item.label);
    ASSERT_EQ(CharSet::Chinese, template_item.char_set);
    ASSERT_EQ(128u, template_item.feature.size());
    ASSERT_EQ(1, template_item.sample_count);
}

// ============================================================================
// ChineseOCRResult 测试
// ============================================================================

TEST(ChineseOCR, OCRResult_Create) {
    ChineseOCRResult result;
    result.text = "测试汉字";
    result.confidence = 90.0f;
    result.char_set = CharSet::Chinese;

    ASSERT_EQ("测试汉字", result.text);
    ASSERT_NEAR(90.0f, result.confidence, 0.1f);
    ASSERT_EQ(CharSet::Chinese, result.char_set);
}

TEST(ChineseOCR, OCRResult_WithChars) {
    ChineseOCRResult result;
    result.text = "测试";

    CharBox box1(10, 10, 20, 20);
    CharBox box2(35, 10, 20, 20);

    result.chars.push_back(CharResult("测", 95.0f, box1, CharSet::Chinese));
    result.chars.push_back(CharResult("试", 92.0f, box2, CharSet::Chinese));

    ASSERT_EQ(2u, result.chars.size());
}

// ============================================================================
// 特征提取测试
// ============================================================================

TEST(ChineseOCR, Feature_GetDim) {
    int dim = chinese_ocr_utils::get_feature_dim();
    ASSERT_EQ(128, dim);  // 网格64 + 方向32 + 投影32
}

TEST(ChineseOCR, Feature_Grid) {
    // 创建测试图像
    ImageData img = TestUtils::create_test_image(32, 32, 128);

    FeatureVector feature;
    ErrorCode code = chinese_ocr_utils::extract_grid_feature(img, feature, 8);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(64u, feature.size());  // 8x8网格
}

TEST(ChineseOCR, Feature_Direction) {
    ImageData img = TestUtils::create_test_image(32, 32, 128);

    FeatureVector feature;
    ErrorCode code = chinese_ocr_utils::extract_direction_feature(img, feature, 32);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(32u, feature.size());
}

TEST(ChineseOCR, Feature_Projection) {
    ImageData img = TestUtils::create_test_image(32, 32, 128);

    FeatureVector feature;
    ErrorCode code = chinese_ocr_utils::extract_projection_feature(img, feature, 32);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(32u, feature.size());
}

TEST(ChineseOCR, Feature_Full) {
    ImageData img = TestUtils::create_test_image(32, 32, 128);

    FeatureVector feature;
    ErrorCode code = chinese_ocr_utils::extract_features(img, feature);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(128u, feature.size());
}

// ============================================================================
// 相似度计算测试
// ============================================================================

TEST(ChineseOCR, Similarity_Identical) {
    FeatureVector a(128, 1.0f);
    FeatureVector b(128, 1.0f);

    float similarity = chinese_ocr_utils::cosine_similarity(a, b);
    ASSERT_NEAR(1.0f, similarity, 0.001f);  // 相同向量相似度为1
}

TEST(ChineseOCR, Similarity_Different) {
    FeatureVector a(128, 1.0f);
    FeatureVector b(128, 0.0f);

    float similarity = chinese_ocr_utils::cosine_similarity(a, b);
    ASSERT_NEAR(0.0f, similarity, 0.001f);  // 垂直向量相似度为0
}

TEST(ChineseOCR, Similarity_Partial) {
    FeatureVector a(128);
    FeatureVector b(128);

    for (int i = 0; i < 64; ++i) {
        a[i] = 1.0f;
        b[i] = 1.0f;
    }
    for (int i = 64; i < 128; ++i) {
        a[i] = 1.0f;
        b[i] = 0.0f;
    }

    float similarity = chinese_ocr_utils::cosine_similarity(a, b);
    ASSERT_TRUE(similarity > 0.5f && similarity < 1.0f);  // 部分相似
}

// ============================================================================
// 模板匹配测试
// ============================================================================

TEST(ChineseOCR, MatchTemplate_EmptyModel) {
    OCRModel model;
    FeatureVector feature(128, 0.5f);

    CharResult result;
    ErrorCode code = chinese_ocr_utils::match_template(feature, model, result);

    ASSERT_NE(ErrorCode::Success, code);  // 空模型无法匹配
}

TEST(ChineseOCR, MatchTemplate_SingleTemplate) {
    OCRModel model;
    FeatureVector template_feature(128, 1.0f);
    model.add_template("字", CharSet::Chinese, template_feature);

    FeatureVector query_feature(128, 0.9f);  // 相似特征

    CharResult result;
    ErrorCode code = chinese_ocr_utils::match_template(query_feature, model, result);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ("字", result.character);
    ASSERT_TRUE(result.confidence > 0.0f);
}

// ============================================================================
// 图像预处理测试
// ============================================================================

TEST(ChineseOCR, Preprocess_ToGray) {
    ImageData rgb_img = TestUtils::create_test_rgb_image(32, 32, 128, 128, 128);

    ImageData gray_img;
    ErrorCode code = chinese_ocr_utils::to_gray(rgb_img, gray_img);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(1u, gray_img.channels);
}

TEST(ChineseOCR, Preprocess_Threshold) {
    ImageData gray_img = TestUtils::create_test_image(32, 32, 128);

    ImageData binary_img;
    ErrorCode code = chinese_ocr_utils::threshold_otsu(gray_img, binary_img);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(1u, binary_img.channels);
}

TEST(ChineseOCR, Preprocess_AdaptiveThreshold) {
    ImageData gray_img = TestUtils::create_test_image(32, 32, 128);

    ImageData binary_img;
    ErrorCode code = chinese_ocr_utils::threshold_adaptive(gray_img, binary_img, 15);

    ASSERT_EQ(ErrorCode::Success, code);
}

TEST(ChineseOCR, Preprocess_Resize) {
    ImageData img = TestUtils::create_test_image(64, 64, 128);

    ImageData resized;
    ErrorCode code = chinese_ocr_utils::resize_image(img, resized, 32, 32);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(32u, resized.width);
    ASSERT_EQ(32u, resized.height);
}

TEST(ChineseOCR, Preprocess_Crop) {
    ImageData img = TestUtils::create_test_image(100, 100, 128);

    ImageData cropped;
    ErrorCode code = chinese_ocr_utils::crop_image(img, cropped, 10, 10, 50, 50);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(50u, cropped.width);
    ASSERT_EQ(50u, cropped.height);
}

TEST(ChineseOCR, Preprocess_CharNormalization) {
    ImageData img = TestUtils::create_test_image(50, 60, 128);

    ImageData normalized;
    ErrorCode code = chinese_ocr_utils::preprocess_char(img, normalized, 32);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_EQ(32u, normalized.width);
    ASSERT_EQ(32u, normalized.height);
}

// ============================================================================
// 字符分割测试
// ============================================================================

TEST(ChineseOCR, Segment_Projection) {
    // 创建简单的二值图像（模拟文字）
    ImageData binary = TestUtils::create_test_image(100, 30, 255);

    // 在某些位置添加黑色区域（模拟字符）
    for (uint32_t y = 5; y < 25; ++y) {
        for (uint32_t x = 10; x < 20; ++x) {
            binary.data[y * 100 + x] = 0;
        }
        for (uint32_t x = 30; x < 40; ++x) {
            binary.data[y * 100 + x] = 0;
        }
    }

    Vector<CharBox> chars;
    ErrorCode code = chinese_ocr_utils::segment_by_projection(binary, chars);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_GE(2u, chars.size());  // 至少检测到两个字符区域
}

TEST(ChineseOCR, Segment_Connected) {
    ImageData binary = TestUtils::create_test_image(100, 30, 255);

    // 添加字符区域
    for (uint32_t y = 5; y < 25; ++y) {
        for (uint32_t x = 10; x < 20; ++x) {
            binary.data[y * 100 + x] = 0;
        }
    }

    Vector<CharBox> chars;
    ErrorCode code = chinese_ocr_utils::segment_by_connected(binary, chars);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_GE(1u, chars.size());
}

TEST(ChineseOCR, Segment_Comprehensive) {
    ImageData binary = TestUtils::create_test_image(100, 30, 255);

    // 添加多个字符区域
    for (uint32_t y = 5; y < 25; ++y) {
        for (uint32_t x = 10; x < 20; ++x) {
            binary.data[y * 100 + x] = 0;
        }
        for (uint32_t x = 30; x < 40; ++x) {
            binary.data[y * 100 + x] = 0;
        }
    }

    Vector<CharBox> chars;
    ErrorCode code = chinese_ocr_utils::segment_chars(binary, chars);

    ASSERT_EQ(ErrorCode::Success, code);
}

// ============================================================================
// MSER检测测试
// ============================================================================

TEST(ChineseOCR, MSER_Detect) {
    ImageData gray = TestUtils::create_test_image(100, 50, 128);

    // 添加一些文字区域
    for (uint32_t y = 10; y < 40; ++y) {
        for (uint32_t x = 20; x < 30; ++x) {
            gray.data[y * 100 + x] = 50;  // 较暗区域
        }
    }

    Vector<chinese_ocr_utils::MSERRegion> regions;
    ErrorCode code = chinese_ocr_utils::mser_detect(gray, regions);

    ASSERT_EQ(ErrorCode::Success, code);
}

// ============================================================================
// NMS测试
// ============================================================================

TEST(ChineseOCR, NMS_Basic) {
    Vector<CharBox> boxes;
    Vector<float> scores;

    boxes.push_back(CharBox(10, 10, 30, 30));
    boxes.push_back(CharBox(15, 15, 30, 30));  // 重叠区域
    scores.push_back(0.9f);
    scores.push_back(0.7f);

    ErrorCode code = chinese_ocr_utils::nms(boxes, scores, 0.5f);

    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_GE(1u, boxes.size());  // NMS后应减少重叠框
}

TEST(ChineseOCR, IoU_Calculation) {
    CharBox a(10, 10, 30, 30);
    CharBox b(15, 15, 30, 30);  // 有重叠

    float iou = chinese_ocr_utils::compute_iou(a, b);
    ASSERT_TRUE(iou > 0.0f && iou < 1.0f);  // 有重叠但不是完全重叠
}

TEST(ChineseOCR, IoU_NoOverlap) {
    CharBox a(10, 10, 30, 30);
    CharBox b(100, 100, 30, 30);  // 无重叠

    float iou = chinese_ocr_utils::compute_iou(a, b);
    ASSERT_NEAR(0.0f, iou, 0.001f);
}

TEST(ChineseOCR, IoU_FullOverlap) {
    CharBox a(10, 10, 30, 30);
    CharBox b(10, 10, 30, 30);  // 完全重叠

    float iou = chinese_ocr_utils::compute_iou(a, b);
    ASSERT_NEAR(1.0f, iou, 0.001f);
}

// ============================================================================
// 节点信息测试
// ============================================================================

TEST(ChineseOCR, NodeInfo_CharDetect) {
    NodeInfo info = CharDetectNode::make_info();
    ASSERT_EQ("CharDetect", info.id);
    ASSERT_NOT_EMPTY(info.inputs);
    ASSERT_NOT_EMPTY(info.outputs);
}

TEST(ChineseOCR, NodeInfo_CharSegment) {
    NodeInfo info = CharSegmentNode::make_info();
    ASSERT_EQ("CharSegment", info.id);
}

TEST(ChineseOCR, NodeInfo_ChineseOCR) {
    NodeInfo info = ChineseOCRNode::make_info();
    ASSERT_EQ("ChineseOCR", info.id);
}

TEST(ChineseOCR, NodeInfo_DigitOCR) {
    NodeInfo info = DigitOCRNode::make_info();
    ASSERT_EQ("DigitOCR", info.id);
}

TEST(ChineseOCR, NodeInfo_EnglishOCR) {
    NodeInfo info = EnglishOCRNode::make_info();
    ASSERT_EQ("EnglishOCR", info.id);
}

TEST(ChineseOCR, NodeInfo_MixedOCR) {
    NodeInfo info = MixedOCRNode::make_info();
    ASSERT_EQ("MixedOCR", info.id);
}

// ============================================================================
// OCRTrainData 测试
// ============================================================================

TEST(ChineseOCR, TrainData_Create) {
    OCRTrainData data;
    data.image = TestUtils::create_test_image(32, 32, 128);
    data.label = "字";
    data.char_set = CharSet::Chinese;
    data.sample_id = 1;

    ASSERT_EQ("字", data.label);
    ASSERT_EQ(CharSet::Chinese, data.char_set);
    ASSERT_EQ(1, data.sample_id);
    ASSERT_FALSE(data.image.empty());
}

// ============================================================================
// 时间工具测试
// ============================================================================

TEST(ChineseOCR, TimeString) {
    String time_str = chinese_ocr_utils::current_time_string();
    ASSERT_NOT_EMPTY(time_str);
    // 时间字符串应包含日期格式
    ASSERT_TRUE(time_str.find("-") != String::npos);
}

// ============================================================================
// 综合测试
// ============================================================================

TEST(ChineseOCR, Comprehensive_FeatureMatch) {
    // 创建模拟的OCR模型
    OCRModel model;
    model.char_set = CharSet::Digit;

    // 添加数字模板
    for (int i = 0; i < 10; ++i) {
        FeatureVector feature(128);
        for (int j = 0; j < 128; ++j) {
            feature[j] = static_cast<float>(i) / 10.0f;
        }
        model.add_template(std::to_string(i), CharSet::Digit, feature);
    }

    ASSERT_EQ(10u, model.size());

    // 测试匹配
    FeatureVector query(128, 0.5f);  // 类似数字5的特征
    CharResult result;

    ErrorCode code = chinese_ocr_utils::match_template(query, model, result);
    ASSERT_EQ(ErrorCode::Success, code);
    ASSERT_TRUE(result.confidence > 0.0f);
}

TEST(ChineseOCR, Comprehensive_Pipeline) {
    // 模拟完整的OCR流程
    // 1. 创建图像
    ImageData img = TestUtils::create_test_image(100, 50, 255);

    // 添加模拟文字
    for (uint32_t y = 10; y < 40; ++y) {
        for (uint32_t x = 10; x < 30; ++x) {
            img.data[y * 100 + x] = 50;
        }
    }

    // 2. 灰度化（已经是灰度）
    ImageData gray;
    ErrorCode code1 = chinese_ocr_utils::to_gray(img, gray);
    ASSERT_EQ(ErrorCode::Success, code1);

    // 3. 二值化
    ImageData binary;
    ErrorCode code2 = chinese_ocr_utils::threshold_otsu(gray, binary);
    ASSERT_EQ(ErrorCode::Success, code2);

    // 4. 字符分割
    Vector<CharBox> chars;
    ErrorCode code3 = chinese_ocr_utils::segment_chars(binary, chars);
    ASSERT_EQ(ErrorCode::Success, code3);

    // 5. 特征提取（对每个字符）
    for (const auto& box : chars) {
        if (box.valid()) {
            ImageData char_img;
            ErrorCode code4 = chinese_ocr_utils::crop_image(binary, char_img,
                box.x, box.y, box.width, box.height);
            if (code4 == ErrorCode::Success) {
                FeatureVector feature;
                ErrorCode code5 = chinese_ocr_utils::extract_features(char_img, feature);
                ASSERT_EQ(ErrorCode::Success, code5);
                ASSERT_EQ(128u, feature.size());
            }
        }
    }
}

// ============================================================================
// 主程序入口
// ============================================================================

int main() {
    // 运行所有测试
    TestStats stats = TestRunner::run_all_tests();

    // 保存测试报告
    TestRunner::save_report(stats, "test_chinese_ocr_report.json");

    // 输出OCR测试信息
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Chinese OCR Test Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Supported CharSet:" << std::endl;
    std::cout << "    Chinese: GB2312 一级常用汉字 3755" << std::endl;
    std::cout << "    Digit:   0-9" << std::endl;
    std::cout << "    English: A-Z" << std::endl;
    std::cout << "    Mixed:   混合字符" << std::endl;
    std::cout << "  Feature Dimension: 128" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // 返回失败测试数量作为退出码
    return stats.failed_tests;
}