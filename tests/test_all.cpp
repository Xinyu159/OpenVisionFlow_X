/**
 * @file test_all.cpp
 * @brief OpenVisionFlow 综合测试入口
 * @author OpenVisionFlow Team
 * @version 0.1.0
 *
 * 运行所有测试模块，生成综合测试报告。
 */

#include "test_framework.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdlib>

using namespace ovf_test;

// ============================================================================
// 测试模块声明
// ============================================================================

// 外部测试模块（声明其main函数）
extern int test_core_types_main();
extern int test_flow_engine_main();
extern int test_subpixel_precision_main();
extern int test_chinese_ocr_main();
extern int test_wafer_inspection_main();
extern int test_automotive_inspection_main();
extern int test_cuda_accelerator_main();
extern int test_debugger_main();

// ============================================================================
// 综合测试配置
// ============================================================================

struct TestModule {
    std::string name;
    std::function<int()> runner;
    int result = 0;
    double duration_ms = 0.0;
};

// ============================================================================
// 综合测试套件
// ============================================================================

TEST(AllTests, CoreTypesModule) {
    std::cout << "\n>>> Running Core Types Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_core_types_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Core Types Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, FlowEngineModule) {
    std::cout << "\n>>> Running Flow Engine Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_flow_engine_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Flow Engine Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, SubpixelPrecisionModule) {
    std::cout << "\n>>> Running Subpixel Precision Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_subpixel_precision_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Subpixel Precision Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, ChineseOCRModule) {
    std::cout << "\n>>> Running Chinese OCR Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_chinese_ocr_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Chinese OCR Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, WaferInspectionModule) {
    std::cout << "\n>>> Running Wafer Inspection Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_wafer_inspection_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Wafer Inspection Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, AutomotiveInspectionModule) {
    std::cout << "\n>>> Running Automotive Inspection Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_automotive_inspection_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Automotive Inspection Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, CudaAcceleratorModule) {
    std::cout << "\n>>> Running CUDA Accelerator Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_cuda_accelerator_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> CUDA Accelerator Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

TEST(AllTests, DebuggerModule) {
    std::cout << "\n>>> Running Debugger Test Module..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int result = test_debugger_main();
    auto end = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << ">>> Debugger Module completed in " << duration << " ms" << std::endl;
    
    ASSERT_EQ(0, result);
}

// ============================================================================
// 测试框架完整性验证
// ============================================================================

TEST(TestFramework, AssertionMacros) {
    // 验证所有断言宏工作正常
    
    ASSERT_TRUE(true);
    ASSERT_FALSE(false);
    ASSERT_EQ(5, 5);
    ASSERT_NE(3, 4);
    ASSERT_NEAR(1.0, 1.0001, 0.001);
    ASSERT_GT(0, 1);
    ASSERT_GE(5, 5);
    ASSERT_LT(10, 5);
    ASSERT_LE(5, 5);
    
    std::vector<int> vec = {1, 2, 3};
    ASSERT_NOT_EMPTY(vec);
    
    std::vector<int> empty_vec;
    ASSERT_EMPTY(empty_vec);
    
    int* ptr = new int(42);
    ASSERT_NOT_NULL(ptr);
    delete ptr;
    
    int* null_ptr = nullptr;
    ASSERT_NULL(null_ptr);
    
    ASSERT_NO_THROW(1 + 1);
}

TEST(TestFramework, TestUtilsFunctions) {
    // 验证测试辅助工具工作正常
    
    auto gray_img = TestUtils::create_test_image(100, 100, 128);
    ASSERT_EQ(100, gray_img.width);
    ASSERT_EQ(100, gray_img.height);
    ASSERT_EQ(1, gray_img.channels);
    
    auto rgb_img = TestUtils::create_test_rgb_image(50, 50, 100, 150, 200);
    ASSERT_EQ(50, rgb_img.width);
    ASSERT_EQ(3, rgb_img.channels);
    
    auto gradient_img = TestUtils::create_gradient_image(256, 256);
    ASSERT_EQ(256, gradient_img.width);
    
    auto edge_img = TestUtils::create_edge_image(100, 100, 50);
    ASSERT_EQ(100, edge_img.width);
    
    auto circle_img = TestUtils::create_circle_image(200, 200, 100, 100, 50);
    ASSERT_EQ(200, circle_img.width);
}

TEST(TestFramework, TestRunnerFunctions) {
    // 验证TestRunner的基本功能
    
    auto& registry = ovf_test::TestRegistry::instance();
    
    // 清空之前的测试
    registry.clear();
    
    // 运行空测试
    ovf_test::TestStats stats = ovf_test::TestRunner::run_all_tests();
    
    ASSERT_EQ(0, stats.total_tests);
    ASSERT_EQ(0, stats.passed_tests);
    ASSERT_EQ(0, stats.failed_tests);
    
    // 生成报告
    std::string report = ovf_test::TestRunner::generate_report(stats);
    ASSERT_TRUE(!report.empty());
}

// ============================================================================
// 性能基准汇总测试
// ============================================================================

TEST(BenchmarkSummary, MeasureTestFrameworkPerformance) {
    // 测试框架性能基准
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // 运行100次简单断言
    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(true);
        ASSERT_EQ(i, i);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
    
    std::cout << "    100 assertions took " << duration_us << " us" << std::endl;
    std::cout << "    Average: " << duration_us / 100.0 << " us per assertion" << std::endl;
    
    // 验证性能在合理范围内（每次断言 < 100us）
    ASSERT_LT(10000.0, duration_us);
}

// ============================================================================
// 项目完整性检查
// ============================================================================

TEST(ProjectIntegrity, CheckModuleNames) {
    // 验证测试模块名称列表
    
    std::vector<std::string> modules = {
        "CoreTypes",
        "FlowEngine",
        "SubpixelPrecision",
        "ChineseOCR",
        "WaferInspection",
        "AutomotiveInspection",
        "CudaAccelerator",
        "Debugger"
    };
    
    ASSERT_EQ(8, modules.size());
    
    for (const auto& name : modules) {
        ASSERT_TRUE(!name.empty());
    }
}

// ============================================================================
// 测试报告生成器
// ============================================================================

class ComprehensiveTestReporter {
public:
    static void generate_final_report(const std::vector<TestModule>& modules) {
        std::ofstream file("test_all_comprehensive_report.json");
        
        if (!file.is_open()) {
            std::cout << "Failed to open report file" << std::endl;
            return;
        }
        
        int total_passed = 0;
        int total_failed = 0;
        double total_duration = 0.0;
        
        file << "{\n";
        file << "  \"report_type\": \"comprehensive\",\n";
        file << "  \"modules\": [\n";
        
        for (size_t i = 0; i < modules.size(); ++i) {
            const auto& m = modules[i];
            
            file << "    {\n";
            file << "      \"name\": \"" << m.name << "\",\n";
            file << "      \"passed\": " << (m.result == 0 ? "true" : "false") << ",\n";
            file << "      \"result_code\": " << m.result << ",\n";
            file << "      \"duration_ms\": " << m.duration_ms << "\n";
            file << "    }";
            
            if (i < modules.size() - 1) file << ",";
            file << "\n";
            
            if (m.result == 0) total_passed++;
            else total_failed++;
            total_duration += m.duration_ms;
        }
        
        file << "  ],\n";
        file << "  \"summary\": {\n";
        file << "    \"total_modules\": " << modules.size() << ",\n";
        file << "    \"passed_modules\": " << total_passed << ",\n";
        file << "    \"failed_modules\": " << total_failed << ",\n";
        file << "    \"pass_rate\": " << std::fixed << std::setprecision(2)
             << (modules.size() > 0 ? (total_passed * 100.0 / modules.size()) : 0.0) << ",\n";
        file << "    \"total_duration_ms\": " << total_duration << "\n";
        file << "  }\n";
        file << "}\n";
        
        file.close();
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "  Comprehensive Test Report" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "  Modules Tested: " << modules.size() << std::endl;
        std::cout << "  Passed Modules: " << total_passed << std::endl;
        std::cout << "  Failed Modules: " << total_failed << std::endl;
        std::cout << "  Total Duration: " << total_duration << " ms" << std::endl;
        std::cout << "  Report saved to: test_all_comprehensive_report.json" << std::endl;
        std::cout << "========================================\n" << std::endl;
    }
};

// ============================================================================
// 主函数 - 综合测试入口
// ============================================================================

int main() {
    std::cout << "\n" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "  OpenVisionFlow Comprehensive Test Suite" << std::endl;
    std::cout << "  Version 0.1.0" << std::endl;
    std::cout << "  Pure C++ Implementation (No External Dependencies)" << std::endl;
    std::cout << "============================================\n" << std::endl;
    
    // 设置随机种子
    srand(42);
    
    // 定义测试模块列表
    std::vector<TestModule> modules = {
        {"CoreTypes", test_core_types_main},
        {"FlowEngine", test_flow_engine_main},
        {"SubpixelPrecision", test_subpixel_precision_main},
        {"ChineseOCR", test_chinese_ocr_main},
        {"WaferInspection", test_wafer_inspection_main},
        {"AutomotiveInspection", test_automotive_inspection_main},
        {"CudaAccelerator", test_cuda_accelerator_main},
        {"Debugger", test_debugger_main}
    };
    
    // 运行每个测试模块
    int total_result = 0;
    
    std::cout << "\n>>> Running " << modules.size() << " test modules...\n" << std::endl;
    
    for (auto& module : modules) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "  Module: " << module.name << std::endl;
        std::cout << "========================================\n" << std::endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        try {
            module.result = module.runner();
        } catch (const std::exception& e) {
            std::cout << "    Module exception: " << e.what() << std::endl;
            module.result = 1;
        } catch (...) {
            std::cout << "    Module unknown exception" << std::endl;
            module.result = 1;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        module.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
        
        if (module.result == 0) {
            std::cout << "\n  [SUCCESS] " << module.name << " passed in "
                      << module.duration_ms << " ms" << std::endl;
        } else {
            std::cout << "\n  [FAILURE] " << module.name << " failed with code "
                      << module.result << std::endl;
            total_result = 1;
        }
    }
    
    // 生成综合报告
    ComprehensiveTestReporter::generate_final_report(modules);
    
    // 同时运行框架自身的测试
    std::cout << "\n>>> Running Test Framework Validation Tests...\n" << std::endl;
    auto framework_stats = ovf_test::TestRunner::run_all_tests();
    ovf_test::TestRunner::save_report(framework_stats, "test_framework_validation_report.json");
    
    if (framework_stats.failed_tests > 0) {
        total_result = 1;
    }
    
    // 最终汇总
    std::cout << "\n============================================" << std::endl;
    std::cout << "  Final Test Summary" << std::endl;
    std::cout << "============================================" << std::endl;
    
    int passed_modules = 0;
    for (const auto& m : modules) {
        if (m.result == 0) passed_modules++;
    }
    
    std::cout << "  Modules Passed: " << passed_modules << "/" << modules.size() << std::endl;
    std::cout << "  Framework Tests: " << framework_stats.passed_tests
              << "/" << framework_stats.total_tests << std::endl;
    
    if (total_result == 0) {
        std::cout << "\n  *** ALL TESTS PASSED! ***" << std::endl;
    } else {
        std::cout << "\n  *** SOME TESTS FAILED! ***" << std::endl;
    }
    
    std::cout << "\n  Reports Generated:" << std::endl;
    std::cout << "    - test_all_comprehensive_report.json" << std::endl;
    std::cout << "    - test_framework_validation_report.json" << std::endl;
    std::cout << "============================================\n" << std::endl;
    
    return total_result;
}

// ============================================================================
// 外部main函数别名（用于链接）
// ============================================================================

int test_core_types_main() {
    return test_core_types_main();
}

int test_flow_engine_main() {
    return test_flow_engine_main();
}

int test_subpixel_precision_main() {
    return test_subpixel_precision_main();
}

int test_chinese_ocr_main() {
    return test_chinese_ocr_main();
}

int test_wafer_inspection_main() {
    return test_wafer_inspection_main();
}

int test_automotive_inspection_main() {
    return test_automotive_inspection_main();
}

int test_cuda_accelerator_main() {
    return test_cuda_accelerator_main();
}

int test_debugger_main() {
    return test_debugger_main();
}