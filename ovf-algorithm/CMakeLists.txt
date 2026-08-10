# ovf-algorithm - 算法插件层（轻量级实现，不依赖OpenCV）
cmake_minimum_required(VERSION 3.16)

# 检测CUDA是否可用
option(ENABLE_CUDA "Enable CUDA acceleration" ON)
set(CUDA_AVAILABLE OFF)

if(ENABLE_CUDA)
    # 尝试查找CUDA
    find_package(CUDAToolkit QUIET)
    
    if(CUDAToolkit_FOUND)
        # 检查CUDA版本
        if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL 10.0)
            set(CUDA_AVAILABLE ON)
            message(STATUS "CUDA found: ${CUDAToolkit_VERSION} at ${CUDAToolkit_ROOT_DIR}")
            message(STATUS "CUDA acceleration enabled - GPU algorithms will be available")
        else()
            message(WARNING "CUDA version too old: ${CUDAToolkit_VERSION}, requires >= 10.0")
        endif()
    else()
        message(STATUS "CUDA not found - using CPU fallback only")
    endif()
else()
    message(STATUS "CUDA disabled by user - using CPU fallback only")
endif()

# 根据CUDA可用性设置项目语言
if(CUDA_AVAILABLE)
    project(ovf-algorithm VERSION 0.2.0 LANGUAGES CXX CUDA)
else()
    project(ovf-algorithm VERSION 0.2.0 LANGUAGES CXX)
endif()

# 源文件（C++）
file(GLOB_RECURSE ALGO_SOURCES "src/*.cpp")
file(GLOB_RECURSE ALGO_HEADERS "include/*.h")

# 确保algorithm_init.cpp被包含
if(NOT ALGO_SOURCES MATCHES "algorithm_init.cpp")
    list(APPEND ALGO_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm_init.cpp")
endif()

# CUDA源文件（如果可用）
if(CUDA_AVAILABLE)
    file(GLOB_RECURSE CUDA_SOURCES "src/*.cu")
    message(STATUS "CUDA sources found: ${CUDA_SOURCES}")
else()
    set(CUDA_SOURCES "")
endif()

# 创建动态库
add_library(ovf-algorithm SHARED ${ALGO_SOURCES} ${ALGO_HEADERS} ${CUDA_SOURCES})

target_include_directories(ovf-algorithm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/ovf-thirdparty/tesseract
        ${CMAKE_SOURCE_DIR}/ovf-thirdparty/onnxruntime
)

# CUDA特定设置
if(CUDA_AVAILABLE)
    # 设置CUDA标准
    set_target_properties(ovf-algorithm PROPERTIES
        CUDA_STANDARD 11
        CUDA_STANDARD_REQUIRED ON
        CUDA_SEPARABLE_COMPILATION ON
    )
    
    # CUDA编译选项
    target_compile_options(ovf-algorithm PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_60>  # 支持Pascal及以上GPU
        $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
    )
    
    # 链接CUDA库
    target_link_libraries(ovf-algorithm PRIVATE CUDA::cudart)
    
    message(STATUS "CUDA acceleration configured successfully")
endif()

target_link_libraries(ovf-algorithm
    PUBLIC
        ovf-core
)

# Windows下生成导入库
if(WIN32)
    set_target_properties(ovf-algorithm PROPERTIES 
        WINDOWS_EXPORT_ALL_SYMBOLS ON
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
    )
endif()

# 定义CUDA可用性宏
if(CUDA_AVAILABLE)
    target_compile_definitions(ovf-algorithm PRIVATE OVF_CUDA_AVAILABLE=1)
else()
    target_compile_definitions(ovf-algorithm PRIVATE OVF_CUDA_AVAILABLE=0)
endif()

# 安装
install(TARGETS ovf-algorithm
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)