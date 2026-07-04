/**
 * @file test_framework.h
 * @brief OpenVisionFlow 纯C++测试框架（不依赖外部测试库）
 * @author OpenVisionFlow Team
 * @version 0.1.0
 *
 * 提供测试宏、TestRunner、测试报告生成等核心功能。
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/data.h"
#include "ovf/core/node.h"
#include "ovf/core/flow.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>

namespace ovf_test {

// ============================================================================
// 测试结果结构
// ============================================================================

/**
 * @brief 单个测试结果
 */
struct TestResult {
    std::string test_name;           // 测试名称
    bool passed = false;             // 是否通过
    std::string message;             // 结果消息
    std::string file;                // 文件名
    int line = 0;                    // 行号
    double duration_ms = 0.0;        // 执行时间（毫秒）
};

/**
 * @brief 测试套件结果
 */
struct TestSuiteResult {
    std::string suite_name;          // 套件名称
    std::vector<TestResult> results; // 测试结果列表
    int passed_count = 0;            // 通过数
    int failed_count = 0;            // 失败数
    double total_duration_ms = 0.0;  // 总时间
};

/**
 * @brief 全局测试统计
 */
struct TestStats {
    int total_tests = 0;
    int passed_tests = 0;
    int failed_tests = 0;
    int skipped_tests = 0;
    double total_duration_ms = 0.0;
    std::vector<TestSuiteResult> suite_results;
};

// ============================================================================
// 测试异常（用于断言失败）
// ============================================================================

/**
 * @brief 测试断言失败异常
 */
class TestFailureException : public std::exception {
public:
    explicit TestFailureException(const std::string& msg, const std::string& file, int line)
        : message_(msg), file_(file), line_(line) {
        full_message_ = "[" + file + ":" + std::to_string(line) + "] " + msg;
    }

    const char* what() const noexcept override { return full_message_.c_str(); }
    const std::string& message() const { return message_; }
    const std::string& file() const { return file_; }
    int line() const { return line_; }

private:
    std::string message_;
    std::string file_;
    int line_;
    std::string full_message_;
};

// ============================================================================
// 断言宏定义
// ============================================================================

/**
 * @brief 断言条件为真
 */
#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_TRUE failed: " #condition " is false", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言条件为假
 */
#define ASSERT_FALSE(condition) \
    do { \
        if (condition) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_FALSE failed: " #condition " is true", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言相等
 */
#define ASSERT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            std::ostringstream oss; \
            oss << "ASSERT_EQ failed: expected " << (expected) << " but got " << (actual); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言不相等
 */
#define ASSERT_NE(expected, actual) \
    do { \
        if ((expected) == (actual)) { \
            std::ostringstream oss; \
            oss << "ASSERT_NE failed: values are equal (" << (expected) << ")"; \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言浮点数近似相等（用于浮点数比较）
 */
#define ASSERT_NEAR(expected, actual, epsilon) \
    do { \
        double _ovf_assert_diff_ = std::abs(static_cast<double>(expected) - static_cast<double>(actual)); \
        if (_ovf_assert_diff_ > static_cast<double>(epsilon)) { \
            std::ostringstream oss; \
            oss << "ASSERT_NEAR failed: |" << (expected) << " - " << (actual) \
                << "| = " << _ovf_assert_diff_ << " > " << (epsilon); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言大于
 */
#define ASSERT_GT(expected, actual) \
    do { \
        if ((actual) <= (expected)) { \
            std::ostringstream oss; \
            oss << "ASSERT_GT failed: " << (actual) << " <= " << (expected); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言大于等于
 */
#define ASSERT_GE(expected, actual) \
    do { \
        if ((actual) < (expected)) { \
            std::ostringstream oss; \
            oss << "ASSERT_GE failed: " << (actual) << " < " << (expected); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言小于
 */
#define ASSERT_LT(expected, actual) \
    do { \
        if ((actual) >= (expected)) { \
            std::ostringstream oss; \
            oss << "ASSERT_LT failed: " << (actual) << " >= " << (expected); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言小于等于
 */
#define ASSERT_LE(expected, actual) \
    do { \
        if ((actual) > (expected)) { \
            std::ostringstream oss; \
            oss << "ASSERT_LE failed: " << (actual) << " > " << (expected); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言非空
 */
#define ASSERT_NOT_EMPTY(container) \
    do { \
        if ((container).empty()) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_NOT_EMPTY failed: container is empty", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言为空
 */
#define ASSERT_EMPTY(container) \
    do { \
        if (!(container).empty()) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_EMPTY failed: container is not empty", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言指针非空
 */
#define ASSERT_NOT_NULL(ptr) \
    do { \
        if ((ptr) == nullptr) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_NOT_NULL failed: pointer is null", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言指针为空
 */
#define ASSERT_NULL(ptr) \
    do { \
        if ((ptr) != nullptr) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_NULL failed: pointer is not null", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言抛出异常
 */
#define ASSERT_THROW(expression, exception_type) \
    do { \
        bool caught = false; \
        try { \
            (expression); \
        } catch (const exception_type&) { \
            caught = true; \
        } catch (...) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_THROW failed: unexpected exception type", __FILE__, __LINE__); \
        } \
        if (!caught) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_THROW failed: no exception thrown", __FILE__, __LINE__); \
        } \
    } while (0)

/**
 * @brief 断言不抛出异常
 */
#define ASSERT_NO_THROW(expression) \
    do { \
        try { \
            (expression); \
        } catch (const std::exception& e) { \
            std::ostringstream oss; \
            oss << "ASSERT_NO_THROW failed: exception thrown: " << e.what(); \
            throw ovf_test::TestFailureException(oss.str(), __FILE__, __LINE__); \
        } catch (...) { \
            throw ovf_test::TestFailureException( \
                "ASSERT_NO_THROW failed: unknown exception thrown", __FILE__, __LINE__); \
        } \
    } while (0)

// ============================================================================
// 测试函数注册
// ============================================================================

/**
 * @brief 测试函数类型
 */
using TestFunction = std::function<void()>;

/**
 * @brief 测试注册器（自动注册测试函数）
 */
class TestRegistry {
public:
    /**
     * @brief 测试信息结构（必须在类开头定义）
     */
    struct TestInfo {
        std::string name;
        TestFunction func;
        std::string file;
    };

    static TestRegistry& instance() {
        static TestRegistry registry;
        return registry;
    }

    /**
     * @brief 注册测试
     */
    void register_test(const std::string& suite_name, const std::string& test_name,
                      TestFunction test_func, const std::string& file) {
        tests_[suite_name].push_back({test_name, test_func, file});
    }

    /**
     * @brief 获取所有测试
     */
    const std::map<std::string, std::vector<TestInfo>>& get_tests() const {
        return tests_;
    }

    /**
     * @brief 清空所有测试
     */
    void clear() {
        tests_.clear();
    }

    /**
     * @brief 获取测试总数
     */
    size_t total_tests() const {
        size_t count = 0;
        for (const auto& suite : tests_) {
            count += suite.second.size();
        }
        return count;
    }

private:
    TestRegistry() = default;
    std::map<std::string, std::vector<TestInfo>> tests_;
};

/**
 * @brief 自动注册测试的辅助类
 */
class TestRegistrar {
public:
    TestRegistrar(const std::string& suite_name, const std::string& test_name,
                 TestFunction test_func, const std::string& file) {
        TestRegistry::instance().register_test(suite_name, test_name, test_func, file);
    }
};

/**
 * @brief 定义测试套件
 */
#define TEST_SUITE(suite_name) \
    namespace ovf_test_suite_##suite_name { \
        static const std::string SUITE_NAME = #suite_name; \
    }

/**
 * @brief 定义测试
 */
#define TEST(suite_name, test_name) \
    void test_##suite_name##_##test_name(); \
    namespace { \
        ovf_test::TestRegistrar registrar_##suite_name##_##test_name( \
            #suite_name, #test_name, test_##suite_name##_##test_name, __FILE__); \
    } \
    void test_##suite_name##_##test_name()

// ============================================================================
// TestRunner - 测试运行器
// ============================================================================

/**
 * @brief 测试运行器
 */
class TestRunner {
public:
    /**
     * @brief 运行所有测试
     */
    static TestStats run_all_tests() {
        TestStats stats;
        auto& registry = TestRegistry::instance();
        const auto& tests = registry.get_tests();

        std::cout << "\n========================================" << std::endl;
        std::cout << "  OpenVisionFlow Test Framework" << std::endl;
        std::cout << "========================================\n" << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();

        for (const auto& suite : tests) {
            TestSuiteResult suite_result;
            suite_result.suite_name = suite.first;

            std::cout << "\n[Suite: " << suite.first << "]" << std::endl;

            for (const auto& test_info : suite.second) {
                TestResult result;
                result.test_name = test_info.name;
                result.file = test_info.file;

                auto test_start = std::chrono::high_resolution_clock::now();

                try {
                    test_info.func();
                    result.passed = true;
                    result.message = "PASSED";
                    suite_result.passed_count++;
                    stats.passed_tests++;

                    std::cout << "  [PASS] " << test_info.name << std::endl;
                } catch (const TestFailureException& e) {
                    result.passed = false;
                    result.message = e.what();
                    result.file = e.file();
                    result.line = e.line();
                    suite_result.failed_count++;
                    stats.failed_tests++;

                    std::cout << "  [FAIL] " << test_info.name << std::endl;
                    std::cout << "         " << e.what() << std::endl;
                } catch (const std::exception& e) {
                    result.passed = false;
                    result.message = "Unexpected exception: " + std::string(e.what());
                    suite_result.failed_count++;
                    stats.failed_tests++;

                    std::cout << "  [FAIL] " << test_info.name << std::endl;
                    std::cout << "         Unexpected exception: " << e.what() << std::endl;
                } catch (...) {
                    result.passed = false;
                    result.message = "Unknown exception";
                    suite_result.failed_count++;
                    stats.failed_tests++;

                    std::cout << "  [FAIL] " << test_info.name << std::endl;
                    std::cout << "         Unknown exception" << std::endl;
                }

                auto test_end = std::chrono::high_resolution_clock::now();
                result.duration_ms = std::chrono::duration<double, std::milli>(test_end - test_start).count();
                suite_result.total_duration_ms += result.duration_ms;

                suite_result.results.push_back(result);
                stats.total_tests++;
            }

            stats.suite_results.push_back(suite_result);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        stats.total_duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // 输出汇总报告
        print_summary(stats);

        return stats;
    }

    /**
     * @brief 运行指定套件的测试
     */
    static TestSuiteResult run_suite(const std::string& suite_name) {
        auto& registry = TestRegistry::instance();
        const auto& tests = registry.get_tests();

        TestSuiteResult result;
        result.suite_name = suite_name;

        auto it = tests.find(suite_name);
        if (it == tests.end()) {
            std::cout << "Suite not found: " << suite_name << std::endl;
            return result;
        }

        std::cout << "\n[Suite: " << suite_name << "]" << std::endl;

        for (const auto& test_info : it->second) {
            TestResult test_result;
            test_result.test_name = test_info.name;
            test_result.file = test_info.file;

            auto test_start = std::chrono::high_resolution_clock::now();

            try {
                test_info.func();
                test_result.passed = true;
                test_result.message = "PASSED";
                result.passed_count++;

                std::cout << "  [PASS] " << test_info.name << std::endl;
            } catch (const TestFailureException& e) {
                test_result.passed = false;
                test_result.message = e.what();
                test_result.file = e.file();
                test_result.line = e.line();
                result.failed_count++;

                std::cout << "  [FAIL] " << test_info.name << std::endl;
                std::cout << "         " << e.what() << std::endl;
            } catch (...) {
                test_result.passed = false;
                test_result.message = "Unknown exception";
                result.failed_count++;

                std::cout << "  [FAIL] " << test_info.name << std::endl;
                std::cout << "         Unknown exception" << std::endl;
            }

            auto test_end = std::chrono::high_resolution_clock::now();
            test_result.duration_ms = std::chrono::duration<double, std::milli>(test_end - test_start).count();
            result.total_duration_ms += test_result.duration_ms;

            result.results.push_back(test_result);
        }

        return result;
    }

    /**
     * @brief 生成测试报告（JSON格式）
     */
    static std::string generate_report(const TestStats& stats) {
        std::ostringstream oss;

        oss << "{\n";
        oss << "  \"summary\": {\n";
        oss << "    \"total_tests\": " << stats.total_tests << ",\n";
        oss << "    \"passed\": " << stats.passed_tests << ",\n";
        oss << "    \"failed\": " << stats.failed_tests << ",\n";
        oss << "    \"skipped\": " << stats.skipped_tests << ",\n";
        oss << "    \"pass_rate\": " << std::fixed << std::setprecision(2)
            << (stats.total_tests > 0 ? (stats.passed_tests * 100.0 / stats.total_tests) : 0.0)
            << ",\n";
        oss << "    \"total_duration_ms\": " << stats.total_duration_ms << "\n";
        oss << "  },\n";
        oss << "  \"suites\": [\n";

        for (size_t i = 0; i < stats.suite_results.size(); ++i) {
            const auto& suite = stats.suite_results[i];
            oss << "    {\n";
            oss << "      \"name\": \"" << suite.suite_name << "\",\n";
            oss << "      \"passed\": " << suite.passed_count << ",\n";
            oss << "      \"failed\": " << suite.failed_count << ",\n";
            oss << "      \"duration_ms\": " << suite.total_duration_ms << ",\n";
            oss << "      \"tests\": [\n";

            for (size_t j = 0; j < suite.results.size(); ++j) {
                const auto& test = suite.results[j];
                oss << "        {\n";
                oss << "          \"name\": \"" << test.test_name << "\",\n";
                oss << "          \"passed\": " << (test.passed ? "true" : "false") << ",\n";
                oss << "          \"message\": \"" << escape_json(test.message) << "\",\n";
                oss << "          \"file\": \"" << test.file << "\",\n";
                oss << "          \"line\": " << test.line << ",\n";
                oss << "          \"duration_ms\": " << test.duration_ms << "\n";
                oss << "        }";
                if (j < suite.results.size() - 1) oss << ",";
                oss << "\n";
            }

            oss << "      ]\n";
            oss << "    }";
            if (i < stats.suite_results.size() - 1) oss << ",";
            oss << "\n";
        }

        oss << "  ]\n";
        oss << "}\n";

        return oss.str();
    }

    /**
     * @brief 保存测试报告到文件
     */
    static void save_report(const TestStats& stats, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << generate_report(stats);
            file.close();
            std::cout << "\nReport saved to: " << filename << std::endl;
        } else {
            std::cout << "\nFailed to save report to: " << filename << std::endl;
        }
    }

private:
    /**
     * @brief 输出汇总报告
     */
    static void print_summary(const TestStats& stats) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "  Test Summary" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "  Total Tests:  " << stats.total_tests << std::endl;
        std::cout << "  Passed:       " << stats.passed_tests << " ("
                  << std::fixed << std::setprecision(1)
                  << (stats.total_tests > 0 ? (stats.passed_tests * 100.0 / stats.total_tests) : 0.0)
                  << "%)" << std::endl;
        std::cout << "  Failed:       " << stats.failed_tests << std::endl;
        std::cout << "  Skipped:      " << stats.skipped_tests << std::endl;
        std::cout << "  Duration:     " << std::fixed << std::setprecision(2)
                  << stats.total_duration_ms << " ms" << std::endl;
        std::cout << "========================================" << std::endl;

        if (stats.failed_tests == 0) {
            std::cout << "\n  *** ALL TESTS PASSED! ***\n" << std::endl;
        } else {
            std::cout << "\n  *** SOME TESTS FAILED! ***\n" << std::endl;

            // 输出失败的测试列表
            std::cout << "\nFailed Tests:" << std::endl;
            for (const auto& suite : stats.suite_results) {
                for (const auto& result : suite.results) {
                    if (!result.passed) {
                        std::cout << "  - " << suite.suite_name << "::" << result.test_name << std::endl;
                    }
                }
            }
        }
    }

    /**
     * @brief 转义JSON字符串
     */
    static std::string escape_json(const std::string& str) {
        std::string result;
        for (char c : str) {
            switch (c) {
                case '"': result += "\\\""; break;
                case '\\': result += "\\\\"; break;
                case '\n': result += "\\n"; break;
                case '\r': result += "\\r"; break;
                case '\t': result += "\\t"; break;
                default: result += c; break;
            }
        }
        return result;
    }
};

// ============================================================================
// 辅助工具
// ============================================================================

/**
 * @brief 测试辅助工具类
 */
class TestUtils {
public:
    /**
     * @brief 创建测试图像（灰度）
     */
    static ovf::ImageData create_test_image(uint32_t width, uint32_t height, uint8_t value = 128) {
        ovf::ImageData img;
        img.width = width;
        img.height = height;
        img.channels = 1;
        img.format = ovf::ImageFormat::Mono8;
        img.data.resize(width * height, value);
        return img;
    }

    /**
     * @brief 创建测试图像（RGB）
     */
    static ovf::ImageData create_test_rgb_image(uint32_t width, uint32_t height,
                                                uint8_t r = 128, uint8_t g = 128, uint8_t b = 128) {
        ovf::ImageData img;
        img.width = width;
        img.height = height;
        img.channels = 3;
        img.format = ovf::ImageFormat::RGB8;
        img.data.resize(width * height * 3);
        for (size_t i = 0; i < width * height; ++i) {
            img.data[i * 3] = r;
            img.data[i * 3 + 1] = g;
            img.data[i * 3 + 2] = b;
        }
        return img;
    }

    /**
     * @brief 创建渐变测试图像
     */
    static ovf::ImageData create_gradient_image(uint32_t width, uint32_t height) {
        ovf::ImageData img;
        img.width = width;
        img.height = height;
        img.channels = 1;
        img.format = ovf::ImageFormat::Mono8;
        img.data.resize(width * height);

        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                img.data[y * width + x] = static_cast<uint8_t>(
                    (x * 255) / (width > 1 ? width - 1 : 1));
            }
        }
        return img;
    }

    /**
     * @brief 创建边缘测试图像（左黑右白）
     */
    static ovf::ImageData create_edge_image(uint32_t width, uint32_t height, uint32_t edge_x) {
        ovf::ImageData img;
        img.width = width;
        img.height = height;
        img.channels = 1;
        img.format = ovf::ImageFormat::Mono8;
        img.data.resize(width * height);

        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                img.data[y * width + x] = (x < edge_x) ? 50 : 200;
            }
        }
        return img;
    }

    /**
     * @brief 创建圆形测试图像
     */
    static ovf::ImageData create_circle_image(uint32_t width, uint32_t height,
                                              uint32_t cx, uint32_t cy, uint32_t radius,
                                              uint8_t fg = 200, uint8_t bg = 50) {
        ovf::ImageData img;
        img.width = width;
        img.height = height;
        img.channels = 1;
        img.format = ovf::ImageFormat::Mono8;
        img.data.resize(width * height, bg);

        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                double dx = static_cast<double>(x) - cx;
                double dy = static_cast<double>(y) - cy;
                if (dx * dx + dy * dy <= radius * radius) {
                    img.data[y * width + x] = fg;
                }
            }
        }
        return img;
    }

    /**
     * @brief 计算两个图像的差异
     */
    static double calculate_image_difference(const ovf::ImageData& a, const ovf::ImageData& b) {
        if (a.width != b.width || a.height != b.height || a.channels != b.channels) {
            return -1.0; // 图像尺寸不匹配
        }

        double sum = 0.0;
        size_t total = a.data.size();
        for (size_t i = 0; i < total; ++i) {
            double diff = static_cast<double>(a.data[i]) - static_cast<double>(b.data[i]);
            sum += diff * diff;
        }

        return std::sqrt(sum / total);
    }

    /**
     * @brief 生成随机数据
     */
    static std::vector<uint8_t> generate_random_data(size_t size, uint8_t min_val = 0, uint8_t max_val = 255) {
        std::vector<uint8_t> data(size);
        for (size_t i = 0; i < size; ++i) {
            data[i] = static_cast<uint8_t>(min_val + (rand() % (max_val - min_val + 1)));
        }
        return data;
    }
};

} // namespace ovf_test

// 引入ovf命名空间以便测试使用
namespace ovf_test {
    // 引入ovf命名空间的类型（实际定义在ovf/core/types.h中）
    using ovf::ImageData;
    using ovf::ImageFormat;
    using ovf::Data;
    using ovf::DataType;
    using ovf::ErrorCode;
    using ovf::Result;
    using ovf::Region;
    using ovf::Pose;
    using ovf::Point2D;
    using ovf::Point3Df;
    using ovf::PointCloudData;
    using ovf::DepthImageData;
    using ovf::String;
    using ovf::ByteArray;
    using ovf::Vector;
    using ovf::HashMap;
    using ovf::Ptr;
}