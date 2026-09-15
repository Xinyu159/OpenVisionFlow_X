# OpenVisionFlow 测试模块构建配置
cmake_minimum_required(VERSION 3.16)

project(ovf-tests
    VERSION 0.1.0
    DESCRIPTION "OpenVisionFlow Test Suite"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(MSVC)
    add_compile_options(/W4 /utf-8)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/ovf-core/include
    ${CMAKE_SOURCE_DIR}/ovf-algorithm/include
)

# 链接库目录
link_directories(
    ${CMAKE_BINARY_DIR}/lib/Release
    ${CMAKE_BINARY_DIR}/lib/Debug
)

# ============================================================================
# 测试可执行程序
# ============================================================================

# 1. 核心类型测试
add_executable(test_core_types
    test_core_types.cpp
)
target_link_libraries(test_core_types
    ovf-core
    ovf-algorithm
)

# 2. 流程引擎测试
add_executable(test_flow_engine
    test_flow_engine.cpp
)
target_link_libraries(test_flow_engine
    ovf-core
    ovf-algorithm
)

# 3. 亚像素精度测试
add_executable(test_subpixel_precision
    test_subpixel_precision.cpp
)
target_link_libraries(test_subpixel_precision
    ovf-core
    ovf-algorithm
)

# 4. 汉字OCR测试
add_executable(test_chinese_ocr
    test_chinese_ocr.cpp
)
target_link_libraries(test_chinese_ocr
    ovf-core
    ovf-algorithm
)

# 5. 晶圆检测测试
add_executable(test_wafer_inspection
    test_wafer_inspection.cpp
)
target_link_libraries(test_wafer_inspection
    ovf-core
    ovf-algorithm
)

# 6. 汽车零部件检测测试
add_executable(test_automotive_inspection
    test_automotive_inspection.cpp
)
target_link_libraries(test_automotive_inspection
    ovf-core
    ovf-algorithm
)

# 7. CUDA加速测试
add_executable(test_cuda_accelerator
    test_cuda_accelerator.cpp
)
target_link_libraries(test_cuda_accelerator
    ovf-core
    ovf-algorithm
)

# 8. 调试工具测试
add_executable(test_debugger
    test_debugger.cpp
)
target_link_libraries(test_debugger
    ovf-core
    ovf-algorithm
)

# 9. 综合测试入口（运行所有测试）
add_executable(test_all
    test_all.cpp
)
target_link_libraries(test_all
    ovf-core
    ovf-algorithm
)

# 10. 写算子的安全带 API（阶段 3：in_image / p_num / step_ok / OVF_TRY_IN）
add_executable(test_node_helpers
    test_node_helpers.cpp
)
target_link_libraries(test_node_helpers
    ovf-core
    ovf-algorithm
)

# ============================================================================
# 输出目录配置
# ============================================================================
if(WIN32)
    set_target_properties(test_core_types test_flow_engine test_subpixel_precision
        test_chinese_ocr test_wafer_inspection test_automotive_inspection
        test_cuda_accelerator test_debugger test_all test_node_helpers
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
    )
else()
    set_target_properties(test_core_types test_flow_engine test_subpixel_precision
        test_chinese_ocr test_wafer_inspection test_automotive_inspection
        test_cuda_accelerator test_debugger test_all test_node_helpers
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endif()

# ============================================================================
# CTest集成
# ============================================================================
enable_testing()

# 添加测试到CTest
add_test(NAME CoreTypesTest COMMAND test_core_types)
add_test(NAME FlowEngineTest COMMAND test_flow_engine)
add_test(NAME SubpixelPrecisionTest COMMAND test_subpixel_precision)
add_test(NAME ChineseOCRTest COMMAND test_chinese_ocr)
add_test(NAME WaferInspectionTest COMMAND test_wafer_inspection)
add_test(NAME AutomotiveInspectionTest COMMAND test_automotive_inspection)
add_test(NAME CUDAAcceleratorTest COMMAND test_cuda_accelerator)
add_test(NAME DebuggerTest COMMAND test_debugger)
add_test(NAME AllTests COMMAND test_all)
add_test(NAME NodeHelpersTest COMMAND test_node_helpers)

# ============================================================================
# 安装配置
# ============================================================================
install(TARGETS
    test_core_types
    test_flow_engine
    test_subpixel_precision
    test_chinese_ocr
    test_wafer_inspection
    test_automotive_inspection
    test_cuda_accelerator
    test_debugger
    test_all
    test_node_helpers
    RUNTIME DESTINATION bin
)

# 安装测试框架头文件
install(FILES test_framework.h DESTINATION include/ovf/tests)

# ============================================================================
# 自定义测试命令
# ============================================================================

# 运行所有测试的自定义命令
add_custom_target(run_all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --verbose
    DEPENDS test_core_types test_flow_engine test_subpixel_precision
            test_chinese_ocr test_wafer_inspection test_automotive_inspection
            test_cuda_accelerator test_debugger test_all test_node_helpers
    COMMENT "Running all OpenVisionFlow tests..."
)

# 生成测试报告的自定义命令
add_custom_target(generate_test_report
    COMMAND test_all --report=${CMAKE_BINARY_DIR}/test_report.json
    DEPENDS test_all
    COMMENT "Generating test report..."
)