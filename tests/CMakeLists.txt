# OpenVisionFlow 测试模块构建配置（windows-rebuild：只测引擎与自建算子）
cmake_minimum_required(VERSION 3.16)

project(ovf-tests
    VERSION 0.2.0
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
    ${CMAKE_SOURCE_DIR}/ovf-nodes/include
)

# ============================================================================
# 测试可执行程序
# 注：原 test_all.cpp 聚合入口已移除 —— 它 extern 的 test_*_main() 在各测试
# 文件里并不存在（各文件只有 main()），此前靠文件尾的自递归“别名”函数链接
# 通过、运行即栈溢出。聚合测试改用 ctest（下方 run_all_tests）。
# ============================================================================

# 1. 核心类型测试
add_executable(test_core_types test_core_types.cpp)
target_link_libraries(test_core_types ovf-core)

# 2. 流程引擎测试
add_executable(test_flow_engine test_flow_engine.cpp)
target_link_libraries(test_flow_engine ovf-core)

# 3. 调试工具测试
add_executable(test_debugger test_debugger.cpp)
target_link_libraries(test_debugger ovf-core)

# 4. 写算子的安全带 API（in_image / p_num / step_ok / OVF_TRY_IN）
add_executable(test_node_helpers test_node_helpers.cpp)
target_link_libraries(test_node_helpers ovf-core ovf-nodes)

# ============================================================================
# 输出目录配置
# ============================================================================
set(_all_tests test_core_types test_flow_engine test_debugger test_node_helpers)

if(WIN32)
    foreach(_t ${_all_tests})
        set_target_properties(${_t} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
            RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
        )
    endforeach()
else()
    foreach(_t ${_all_tests})
        set_target_properties(${_t} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        )
    endforeach()
endif()

# ============================================================================
# CTest 集成
# ============================================================================
enable_testing()

add_test(NAME CoreTypesTest COMMAND test_core_types)
add_test(NAME FlowEngineTest COMMAND test_flow_engine)
add_test(NAME DebuggerTest COMMAND test_debugger)
add_test(NAME NodeHelpersTest COMMAND test_node_helpers)

# ============================================================================
# 安装配置
# ============================================================================
install(TARGETS ${_all_tests} RUNTIME DESTINATION bin)
install(FILES test_framework.h DESTINATION include/ovf/tests)

# 运行所有测试的自定义命令
add_custom_target(run_all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS ${_all_tests}
    COMMENT "Running all OpenVisionFlow tests..."
)
