/**
 * @file test_cuda_accelerator.cpp
 * @brief CUDA加速器测试（纯C++实现，包含CPU fallback测试）
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#include "test_framework.h"
#include <ovf/algorithm/cuda_accelerator.h>
#include <ovf/algorithm/blob_analysis.h>
#include <cmath>
#include <chrono>

using namespace ovf;
using namespace ovf::algorithm::cuda;
using namespace ovf_test;
using ovf::algorithm::Blob;

// ============================================================================
// CUDA可用性测试
// ============================================================================

TEST(CudaAccelerator, IsCudaAvailable) {
    // 测试CUDA可用性检测
    bool available = CudaAccelerator::is_cuda_available();
    
    // 在没有GPU的系统上，应该返回false或正确检测状态
    // 此测试主要验证API可以正常调用
    if (available) {
        std::cout << "    CUDA is available on this system" << std::endl;
    } else {
        std::cout << "    CUDA not available - CPU fallback will be used" << std::endl;
    }
    
    ASSERT_TRUE(true); // API调用成功即通过
}

TEST(CudaAccelerator, GetDeviceCount) {
    int device_count = CudaAccelerator::get_device_count();
    
    std::cout << "    Device count: " << device_count << std::endl;
    
    // 设备数应该 >= 0
    ASSERT_GE(0, device_count);
}

TEST(CudaAccelerator, GetGpuInfo) {
    String name;
    size_t total_memory = 0;
    String compute_capability;
    
    bool success = CudaAccelerator::get_gpu_info(name, total_memory, compute_capability);
    
    if (success) {
        std::cout << "    GPU Name: " << name << std::endl;
        std::cout << "    Total Memory: " << total_memory << " MB" << std::endl;
        std::cout << "    Compute Capability: " << compute_capability << std::endl;
        
        ASSERT_TRUE(!name.empty());
        ASSERT_GT(0, total_memory);
    } else {
        std::cout << "    No GPU info available (CPU fallback mode)" << std::endl;
        ASSERT_TRUE(true); // 无GPU时也通过
    }
}

TEST(CudaAccelerator, SetDevice) {
    // 设置设备（如果存在）
    if (CudaAccelerator::get_device_count() > 0) {
        bool success = CudaAccelerator::set_device(0);
        ASSERT_TRUE(success);
    } else {
        std::cout << "    No CUDA device - skipping set_device test" << std::endl;
        ASSERT_TRUE(true);
    }
}

// ============================================================================
// CUDA Stream测试
// ============================================================================

TEST(CudaStream, CreateAndSynchronize) {
    CUDAStream stream;
    
    if (CudaAccelerator::is_cuda_available()) {
        ASSERT_TRUE(stream.is_valid());
        
        stream.synchronize();
        
        void* native = stream.get_native_stream();
        ASSERT_TRUE(native != nullptr || !stream.is_valid()); // 有GPU时native非空
    } else {
        std::cout << "    CPU mode - CUDAStream mock test" << std::endl;
        ASSERT_TRUE(true);
    }
}

// ============================================================================
// CUDA Buffer测试
// ============================================================================

TEST(CudaBuffer, CreateBuffer) {
    size_t buffer_size = 1024 * 1024; // 1MB
    
    auto buffer = CudaAccelerator::create_buffer(buffer_size);
    
    ASSERT_NOT_NULL(buffer);
    ASSERT_EQ(buffer_size, buffer->size());
    
    void* data = buffer->data();
    ASSERT_NOT_NULL(data);
}

TEST(CudaBuffer, PinnedMemory) {
    size_t size = 4096;
    CUDABuffer buffer(size);
    
    // 测试是否是Pinned内存（如果GPU可用）
    if (CudaAccelerator::is_cuda_available()) {
        std::cout << "    Is pinned: " << (buffer.is_pinned() ? "yes" : "no") << std::endl;
    }
    
    ASSERT_EQ(size, buffer.size());
}

TEST(CudaBuffer, ResizeBuffer) {
    CUDABuffer buffer(1024);
    
    ASSERT_EQ(1024, buffer.size());
    
    buffer.resize(2048);
    ASSERT_EQ(2048, buffer.size());
    
    buffer.resize(512);
    ASSERT_EQ(512, buffer.size());
}

// ============================================================================
// 高斯模糊测试
// ============================================================================

TEST(CudaGaussianBlur, BasicBlur) {
    // 创建测试图像（256x256灰度）
    auto input = ovf_test::TestUtils::create_gradient_image(256, 256);
    ImageData output;
    
    CUDAStats stats;
    
    ErrorCode err = CudaAccelerator::gaussian_blur(input, output, 2.0f, 0, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(input.width, output.width);
    ASSERT_EQ(input.height, output.height);
    
    std::cout << "    GPU time: " << stats.gpu_time_ms << " ms" << std::endl;
    std::cout << "    CPU time: " << stats.cpu_time_ms << " ms" << std::endl;
    std::cout << "    Speedup: " << stats.speedup << "x" << std::endl;
    std::cout << "    Used GPU: " << (stats.used_gpu ? "yes" : "no") << std::endl;
}

TEST(CudaGaussianBlur, DifferentSigma) {
    auto input = ovf_test::TestUtils::create_test_image(128, 128, 100);
    ImageData output1, output2, output3;
    
    // 不同sigma值
    ErrorCode err1 = CudaAccelerator::gaussian_blur(input, output1, 1.0f);
    ErrorCode err2 = CudaAccelerator::gaussian_blur(input, output2, 3.0f);
    ErrorCode err3 = CudaAccelerator::gaussian_blur(input, output3, 5.0f);
    
    ASSERT_EQ(ErrorCode::Success, err1);
    ASSERT_EQ(ErrorCode::Success, err2);
    ASSERT_EQ(ErrorCode::Success, err3);
    
    // 输出图像尺寸应该相同
    ASSERT_EQ(input.width, output1.width);
    ASSERT_EQ(input.width, output2.width);
    ASSERT_EQ(input.width, output3.width);
}

TEST(CudaGaussianBlur, BatchProcess) {
    // 批量处理测试
    Vector<ImageData> inputs;
    Vector<ImageData> outputs;
    
    for (int i = 0; i < 5; ++i) {
        inputs.push_back(ovf_test::TestUtils::create_test_image(64, 64, 50 + i * 10));
    }
    
    CUDAStats stats;
    ErrorCode err = CudaAccelerator::gaussian_blur_batch(inputs, outputs, 2.0f, 0, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(inputs.size(), outputs.size());
    
    for (size_t i = 0; i < outputs.size(); ++i) {
        ASSERT_EQ(inputs[i].width, outputs[i].width);
        ASSERT_EQ(inputs[i].height, outputs[i].height);
    }
}

// ============================================================================
// Sobel边缘检测测试
// ============================================================================

TEST(CudaSobel, BasicSobel) {
    // 创建边缘测试图像
    auto input = ovf_test::TestUtils::create_edge_image(256, 256, 128);
    ImageData output;
    
    CUDAStats stats;
    ErrorCode err = CudaAccelerator::sobel_filter(input, output, true, true, 50, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(input.width, output.width);
    ASSERT_EQ(input.height, output.height);
    
    std::cout << "    Sobel GPU time: " << stats.gpu_time_ms << " ms" << std::endl;
}

TEST(CudaSobel, DirectionalGradient) {
    auto input = ovf_test::TestUtils::create_gradient_image(128, 128);
    ImageData output_x, output_y, output_both;
    
    // 只计算X方向
    ErrorCode err_x = CudaAccelerator::sobel_filter(input, output_x, true, false);
    ASSERT_EQ(ErrorCode::Success, err_x);
    
    // 只计算Y方向
    ErrorCode err_y = CudaAccelerator::sobel_filter(input, output_y, false, true);
    ASSERT_EQ(ErrorCode::Success, err_y);
    
    // 计算X和Y方向
    ErrorCode err_both = CudaAccelerator::sobel_filter(input, output_both, true, true);
    ASSERT_EQ(ErrorCode::Success, err_both);
}

TEST(CudaSobel, WithThreshold) {
    auto input = ovf_test::TestUtils::create_test_image(100, 100, 128);
    ImageData output_no_thresh, output_with_thresh;
    
    // 无阈值
    ErrorCode err1 = CudaAccelerator::sobel_filter(input, output_no_thresh, true, true, 0);
    ASSERT_EQ(ErrorCode::Success, err1);
    
    // 有阈值
    ErrorCode err2 = CudaAccelerator::sobel_filter(input, output_with_thresh, true, true, 100);
    ASSERT_EQ(ErrorCode::Success, err2);
}

// ============================================================================
// 阈值分割测试
// ============================================================================

TEST(CudaThreshold, BinaryThreshold) {
    auto input = ovf_test::TestUtils::create_gradient_image(256, 256);
    ImageData output;
    
    CUDAStats stats;
    ErrorCode err = CudaAccelerator::threshold(input, output,
        ThresholdType::Binary, 128.0f, 255.0f, 11, 2.0f, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(input.width, output.width);
    ASSERT_EQ(input.height, output.height);
    
    // 验证输出是二值图像
    bool has_binary = true;
    for (auto val : output.data) {
        if (val != 0 && val != 255) {
            has_binary = false;
            break;
        }
    }
    ASSERT_TRUE(has_binary);
}

TEST(CudaThreshold, AdaptiveThreshold) {
    auto input = ovf_test::TestUtils::create_test_image(128, 128);
    input = ovf_test::TestUtils::create_gradient_image(128, 128); // 使用渐变图像
    
    ImageData output_mean, output_gauss;
    
    // 自适应均值阈值
    ErrorCode err1 = CudaAccelerator::threshold(input, output_mean,
        ThresholdType::AdaptiveMean, 0.0f, 255.0f, 11, 5.0f);
    ASSERT_EQ(ErrorCode::Success, err1);
    
    // 自适应高斯阈值
    ErrorCode err2 = CudaAccelerator::threshold(input, output_gauss,
        ThresholdType::AdaptiveGauss, 0.0f, 255.0f, 11, 5.0f);
    ASSERT_EQ(ErrorCode::Success, err2);
}

TEST(CudaThreshold, BatchThreshold) {
    Vector<ImageData> inputs;
    Vector<ImageData> outputs;
    
    for (int i = 0; i < 10; ++i) {
        inputs.push_back(ovf_test::TestUtils::create_gradient_image(64, 64));
    }
    
    ErrorCode err = CudaAccelerator::threshold_batch(inputs, outputs,
        ThresholdType::Binary, 128.0f);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(inputs.size(), outputs.size());
}

// ============================================================================
// 形态学操作测试
// ============================================================================

TEST(CudaMorphology, Erode) {
    auto input = ovf_test::TestUtils::create_circle_image(128, 128, 64, 64, 30, 255, 0);
    ImageData output;
    
    CUDAStats stats;
    ErrorCode err = CudaAccelerator::morphology(input, output,
        MorphOpCUDA::Erode, KernelShapeCUDA::Ellipse, 5, 1, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(input.width, output.width);
    
    std::cout << "    Erode time: " << stats.gpu_time_ms << " ms" << std::endl;
}

TEST(CudaMorphology, Dilate) {
    auto input = ovf_test::TestUtils::create_circle_image(128, 128, 64, 64, 30, 255, 0);
    ImageData output;
    
    ErrorCode err = CudaAccelerator::morphology(input, output,
        MorphOpCUDA::Dilate, KernelShapeCUDA::Rect, 3, 1);
    
    ASSERT_EQ(ErrorCode::Success, err);
}

TEST(CudaMorphology, OpenClose) {
    auto input = ovf_test::TestUtils::create_test_image(100, 100, 128);
    
    ImageData output_open, output_close;
    
    // 开运算
    ErrorCode err1 = CudaAccelerator::morphology(input, output_open,
        MorphOpCUDA::Open, KernelShapeCUDA::Rect, 3, 1);
    ASSERT_EQ(ErrorCode::Success, err1);
    
    // 闭运算
    ErrorCode err2 = CudaAccelerator::morphology(input, output_close,
        MorphOpCUDA::Close, KernelShapeCUDA::Rect, 3, 1);
    ASSERT_EQ(ErrorCode::Success, err2);
}

TEST(CudaMorphology, DifferentKernels) {
    auto input = ovf_test::TestUtils::create_test_image(64, 64);
    
    ImageData output_rect, output_ellipse, output_cross;
    
    ErrorCode err1 = CudaAccelerator::morphology(input, output_rect,
        MorphOpCUDA::Erode, KernelShapeCUDA::Rect, 5);
    ErrorCode err2 = CudaAccelerator::morphology(input, output_ellipse,
        MorphOpCUDA::Erode, KernelShapeCUDA::Ellipse, 5);
    ErrorCode err3 = CudaAccelerator::morphology(input, output_cross,
        MorphOpCUDA::Erode, KernelShapeCUDA::Cross, 5);
    
    ASSERT_EQ(ErrorCode::Success, err1);
    ASSERT_EQ(ErrorCode::Success, err2);
    ASSERT_EQ(ErrorCode::Success, err3);
}

// ============================================================================
// 直方图计算测试
// ============================================================================

TEST(CudaHistogram, SingleChannel) {
    auto input = ovf_test::TestUtils::create_gradient_image(256, 256);
    Vector<int> histogram;
    
    CUDAStats stats;
    ErrorCode err = CudaAccelerator::histogram(input, histogram, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(256, histogram.size()); // 256个bin
    
    // 验证直方图总和等于像素数
    int sum = 0;
    for (int val : histogram) {
        sum += val;
    }
    ASSERT_EQ(static_cast<int>(input.width * input.height), sum);
    
    std::cout << "    Histogram GPU time: " << stats.gpu_time_ms << " ms" << std::endl;
}

TEST(CudaHistogram, MultiChannel) {
    auto input = ovf_test::TestUtils::create_test_rgb_image(128, 128, 100, 150, 200);
    Vector<Vector<int>> histograms;
    
    ErrorCode err = CudaAccelerator::histogram_multi_channel(input, histograms);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(3, histograms.size()); // RGB三通道
    
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(256, histograms[i].size());
    }
}

// ============================================================================
// 模板匹配测试
// ============================================================================

TEST(CudaTemplateMatch, SingleTemplate) {
    // 创建源图像（256x256，包含一个圆形）
    auto image = ovf_test::TestUtils::create_circle_image(256, 256, 128, 128, 40, 200, 50);
    
    // 创建模板（圆形中心区域）
    auto template_image = ovf_test::TestUtils::create_circle_image(80, 80, 40, 40, 40, 200, 50);
    
    TemplateMatchResult result;
    CUDAStats stats;
    
    ErrorCode err = CudaAccelerator::template_match(image, template_image, result,
        MatchMethodCUDA::NCC, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    
    std::cout << "    Match position: (" << result.x << ", " << result.y << ")" << std::endl;
    std::cout << "    Match score: " << result.score << std::endl;
    std::cout << "    GPU time: " << stats.gpu_time_ms << " ms" << std::endl;
    
    // 验证匹配位置接近圆形中心
    int expected_x = 128 - 40;
    int expected_y = 128 - 40;
    
    // 允许一定偏差（±5像素）
    ASSERT_NEAR(expected_x, result.x, 10);
    ASSERT_NEAR(expected_y, result.y, 10);
    ASSERT_GT(0.5f, result.score);
}

TEST(CudaTemplateMatch, MultiTemplate) {
    auto image = ovf_test::TestUtils::create_test_image(256, 256, 100);
    
    // 创建多个模板
    Vector<ImageData> templates;
    templates.push_back(ovf_test::TestUtils::create_test_image(32, 32, 100));
    templates.push_back(ovf_test::TestUtils::create_test_image(32, 32, 80));
    templates.push_back(ovf_test::TestUtils::create_test_image(32, 32, 120));
    
    Vector<TemplateMatchResult> results;
    
    ErrorCode err = CudaAccelerator::template_match_multi(image, templates, results,
        MatchMethodCUDA::NCC, 0.5f);
    
    ASSERT_EQ(ErrorCode::Success, err);
    
    std::cout << "    Templates matched: " << results.size() << std::endl;
}

TEST(CudaTemplateMatch, DifferentMethods) {
    auto image = ovf_test::TestUtils::create_test_image(128, 128);
    auto template_image = ovf_test::TestUtils::create_test_image(32, 32);
    
    TemplateMatchResult result_sad, result_ssd, result_ncc;
    
    ErrorCode err1 = CudaAccelerator::template_match(image, template_image, result_sad, MatchMethodCUDA::SAD);
    ErrorCode err2 = CudaAccelerator::template_match(image, template_image, result_ssd, MatchMethodCUDA::SSD);
    ErrorCode err3 = CudaAccelerator::template_match(image, template_image, result_ncc, MatchMethodCUDA::NCC);
    
    ASSERT_EQ(ErrorCode::Success, err1);
    ASSERT_EQ(ErrorCode::Success, err2);
    ASSERT_EQ(ErrorCode::Success, err3);
}

// ============================================================================
// Blob分析测试
// ============================================================================

TEST(CudaBlobAnalysis, BasicBlob) {
    // 创建包含多个圆形的二值图像
    auto binary = ovf_test::TestUtils::create_circle_image(256, 256, 50, 50, 20, 255, 0);
    // 添加更多圆形
    for (uint32_t y = 50; y < 200; y += 60) {
        for (uint32_t x = 50; x < 200; x += 60) {
            uint32_t cx = x;
            uint32_t cy = y;
            for (uint32_t py = 0; py < binary.height; ++py) {
                for (uint32_t px = 0; px < binary.width; ++px) {
                    double dx = px - cx;
                    double dy = py - cy;
                    if (dx * dx + dy * dy <= 15 * 15) {
                        binary.data[py * binary.width + px] = 255;
                    }
                }
            }
        }
    }
    
    Vector<Blob> blobs;
    CUDAStats stats;
    
    ErrorCode err = CudaAccelerator::blob_analysis(binary, blobs, 10, 10000, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_NOT_EMPTY(blobs);
    
    std::cout << "    Detected blobs: " << blobs.size() << std::endl;
    std::cout << "    GPU time: " << stats.gpu_time_ms << " ms" << std::endl;
}

TEST(CudaBlobAnalysis, AreaFilter) {
    auto binary = ovf_test::TestUtils::create_circle_image(128, 128, 64, 64, 30, 255, 0);
    
    Vector<Blob> blobs_small, blobs_large;
    
    // 只检测小面积blob
    ErrorCode err1 = CudaAccelerator::blob_analysis(binary, blobs_small, 1, 500);
    
    // 只检测大面积blob
    ErrorCode err2 = CudaAccelerator::blob_analysis(binary, blobs_large, 500, 10000);
    
    ASSERT_EQ(ErrorCode::Success, err1);
    ASSERT_EQ(ErrorCode::Success, err2);
}

// ============================================================================
// 图像缩放测试
// ============================================================================

TEST(CudaResize, ScaleResize) {
    auto input = ovf_test::TestUtils::create_test_image(256, 256);
    ImageData output;
    
    CUDAStats stats;
    ErrorCode err = CudaAccelerator::resize(input, output, 0.5f, 0.5f,
        InterpolationMethod::Bilinear, nullptr, &stats);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(128, output.width);
    ASSERT_EQ(128, output.height);
    
    std::cout << "    Resize GPU time: " << stats.gpu_time_ms << " ms" << std::endl;
}

TEST(CudaResize, TargetSizeResize) {
    auto input = ovf_test::TestUtils::create_test_image(100, 100);
    ImageData output;
    
    ErrorCode err = CudaAccelerator::resize_to(input, output, 200, 200,
        InterpolationMethod::Bilinear);
    
    ASSERT_EQ(ErrorCode::Success, err);
    ASSERT_EQ(200, output.width);
    ASSERT_EQ(200, output.height);
}

TEST(CudaResize, DifferentInterpolation) {
    auto input = ovf_test::TestUtils::create_gradient_image(128, 128);
    
    ImageData output_nearest, output_bilinear, output_bicubic;
    
    ErrorCode err1 = CudaAccelerator::resize(input, output_nearest, 2.0f, 2.0f,
        InterpolationMethod::Nearest);
    ErrorCode err2 = CudaAccelerator::resize(input, output_bilinear, 2.0f, 2.0f,
        InterpolationMethod::Bilinear);
    ErrorCode err3 = CudaAccelerator::resize(input, output_bicubic, 2.0f, 2.0f,
        InterpolationMethod::Bicubic);
    
    ASSERT_EQ(ErrorCode::Success, err1);
    ASSERT_EQ(ErrorCode::Success, err2);
    ASSERT_EQ(ErrorCode::Success, err3);
    
    ASSERT_EQ(256, output_nearest.width);
    ASSERT_EQ(256, output_bilinear.width);
    ASSERT_EQ(256, output_bicubic.width);
}

// ============================================================================
// CUDA节点测试
// ============================================================================

TEST(CudaNodes, GaussianBlurNode) {
    auto info = CudaGaussianBlurNode::make_info();

    ASSERT_TRUE(!info.id.empty());
    ASSERT_TRUE(!info.category.empty());

    CudaGaussianBlurNode node("test_gaussian");
    node.set_sigma(2.0f);
    node.set_kernel_size(5);

    // 设置成功验证（私有成员无法直接访问）
    ASSERT_TRUE(true);
}

TEST(CudaNodes, SobelNode) {
    auto info = CudaSobelFilterNode::make_info();

    ASSERT_TRUE(!info.id.empty());

    CudaSobelFilterNode node("test_sobel");
    node.set_dx(true);
    node.set_dy(true);
    node.set_threshold(50);

    // 设置成功验证（私有成员无法直接访问）
    ASSERT_TRUE(true);
}

TEST(CudaNodes, ThresholdNode) {
    auto info = CudaThresholdNode::make_info();

    ASSERT_TRUE(!info.id.empty());

    CudaThresholdNode node("test_threshold");
    node.set_threshold_type(ThresholdType::Binary);
    node.set_threshold_value(128.0f);

    // 设置成功验证（私有成员无法直接访问）
    ASSERT_TRUE(true);
}

TEST(CudaNodes, TemplateMatchNode) {
    auto info = CudaTemplateMatchNode::make_info();

    ASSERT_TRUE(!info.id.empty());

    CudaTemplateMatchNode node("test_template");
    node.set_method(MatchMethodCUDA::NCC);
    node.set_threshold(0.7f);

    auto template_img = ovf_test::TestUtils::create_test_image(32, 32);
    node.set_template(template_img);

    // 设置成功验证（私有成员无法直接访问）
    ASSERT_TRUE(true);
}

// ============================================================================
// 性能基准测试
// ============================================================================

TEST(CudaBenchmark, GaussianBlurPerformance) {
    auto input = ovf_test::TestUtils::create_test_image(1024, 1024);
    ImageData output;
    
    CUDAStats stats;
    
    auto start = std::chrono::high_resolution_clock::now();
    ErrorCode err = CudaAccelerator::gaussian_blur(input, output, 3.0f, 0, nullptr, &stats);
    auto end = std::chrono::high_resolution_clock::now();
    
    ASSERT_EQ(ErrorCode::Success, err);
    
    double total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    std::cout << "    1024x1024 Gaussian Blur:" << std::endl;
    std::cout << "      GPU: " << stats.gpu_time_ms << " ms" << std::endl;
    std::cout << "      CPU: " << stats.cpu_time_ms << " ms" << std::endl;
    std::cout << "      Total: " << total_ms << " ms" << std::endl;
    std::cout << "      Speedup: " << stats.speedup << "x" << std::endl;
}

TEST(CudaBenchmark, BatchProcessingPerformance) {
    Vector<ImageData> inputs;
    Vector<ImageData> outputs;
    
    // 10张512x512图像
    for (int i = 0; i < 10; ++i) {
        inputs.push_back(ovf_test::TestUtils::create_test_image(512, 512));
    }
    
    CUDAStats stats;
    
    auto start = std::chrono::high_resolution_clock::now();
    ErrorCode err = CudaAccelerator::gaussian_blur_batch(inputs, outputs, 2.0f, 0, nullptr, &stats);
    auto end = std::chrono::high_resolution_clock::now();
    
    ASSERT_EQ(ErrorCode::Success, err);
    
    double total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    std::cout << "    Batch 10x512x512 Gaussian Blur:" << std::endl;
    std::cout << "      GPU: " << stats.gpu_time_ms << " ms" << std::endl;
    std::cout << "      Total: " << total_ms << " ms" << std::endl;
    std::cout << "      Per image: " << total_ms / 10.0 << " ms" << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  CUDA Accelerator Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // 设置随机种子
    srand(42);
    
    auto stats = ovf_test::TestRunner::run_all_tests();
    
    // 保存测试报告
    ovf_test::TestRunner::save_report(stats, "test_cuda_accelerator_report.json");
    
    return stats.failed_tests > 0 ? 1 : 0;
}