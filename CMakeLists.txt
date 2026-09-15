# OpenVisionFlow - 开源工业机器视觉平台
# CMakeLists.txt - 顶层构建配置

cmake_minimum_required(VERSION 3.16)

project(OpenVisionFlow
    VERSION 0.2.0
    DESCRIPTION "Open Source Industrial Machine Vision Platform"
    LANGUAGES CXX C
)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
if(MSVC)
    add_compile_options(/W4 /utf-8)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    add_compile_definitions(NOMINMAX)  # 禁用Windows min/max宏
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Windows下需要配置Release/Debug子目录
if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/Release)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/Debug)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib/Release)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib/Debug)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib/Release)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib/Debug)
endif()

# 查找依赖 (可选)
find_package(OpenCV QUIET)
find_package(Qt6 COMPONENTS Core Widgets Gui Network QUIET)

# 依赖状态报告
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
else()
    message(STATUS "OpenCV not found - algorithm module will be disabled")
endif()

if(Qt6_FOUND)
    message(STATUS "Qt6 found - editor will be built")
else()
    message(STATUS "Qt6 not found - editor will be disabled")
endif()

# CUDA默认禁用（需要完整工具集）
option(ENABLE_CUDA "Enable CUDA acceleration" OFF)
message(STATUS "CUDA acceleration: ${ENABLE_CUDA}")

# 用户算子模块（你自己写的算子住在 ovf-nodes/）。
# -DBUILD_OVF_NODES=OFF 会把整块摘掉，精确还原"没有这个模块"的构建。
option(BUILD_OVF_NODES "Build the user operator module (ovf-nodes)" ON)
message(STATUS "User operator module (ovf-nodes): ${BUILD_OVF_NODES}")

# 测试模块（纯C++测试框架，无外部依赖）
# 从下面挪到这儿：ovf-nodes/ 也要看它来决定建不建自己的测试。
option(BUILD_TESTS "Build test suite" ON)

# 全局定义
add_compile_definitions(OVF_VERSION="${PROJECT_VERSION}")
add_compile_definitions(OVF_PROJECT_NAME="${PROJECT_NAME}")

# 子模块
add_subdirectory(ovf-core)
add_subdirectory(ovf-algorithm)

# 用户算子模块。位置在 ovf-web-server 之前，因为后者要链 ovf-nodes-link。
# 它只依赖 ovf-core —— 501 个老算子是历史资产，不是新算子的依赖。
if(BUILD_OVF_NODES)
    add_subdirectory(ovf-nodes)
endif()

add_subdirectory(ovf-sdk)
add_subdirectory(ovf-sdk-python)
add_subdirectory(ovf-web-server)

if(Qt6_FOUND)
    add_subdirectory(ovf-editor)
endif()

# 插件模块（简化版，不依赖OpenCV）
add_subdirectory(ovf-plugins)

# 相机驱动插件（基于GenICam标准）
add_subdirectory(ovf-plugin-camera)

# 测试模块（纯C++测试框架，无外部依赖）
if(BUILD_TESTS)
    add_subdirectory(tests)
    message(STATUS "Test suite enabled - pure C++ implementation (no external test libraries)")
endif()

# 安装配置
install(FILES README.md LICENSE DESTINATION .)
install(DIRECTORY docs/ DESTINATION docs)