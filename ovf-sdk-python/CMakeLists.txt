# ovf-sdk-python - Python SDK绑定模块
cmake_minimum_required(VERSION 3.16)

project(ovf-sdk-python VERSION 0.1.0 LANGUAGES CXX)

# 查找Python
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# 检查Python版本
if(Python3_VERSION VERSION_LESS "3.6")
    message(FATAL_ERROR "Python 3.6 or higher is required")
endif()

message(STATUS "Python3 found: ${Python3_VERSION}")
message(STATUS "Python3 executable: ${Python3_EXECUTABLE}")
message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 library dirs: ${Python3_LIBRARY_DIRS}")

# 源文件
set(PYTHON_SDK_SOURCES
    src/ovf_python.cpp
    src/python_bindings.cpp
    ../ovf-sdk/cpp/src/api.cpp
)

set(PYTHON_SDK_HEADERS
    include/ovf_python.h
)

# 创建Python模块（共享库）
# Python模块在Windows上需要特殊的输出名称
add_library(ovf-python MODULE ${PYTHON_SDK_SOURCES} ${PYTHON_SDK_HEADERS})

# 设置目标属性
set_target_properties(ovf-python PROPERTIES
    # Python模块不需要前缀"lib"
    PREFIX ""
    # Windows下输出为.pyd扩展名
    OUTPUT_NAME "ovf"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Windows特定配置
if(WIN32)
    # Windows下Python模块使用.pyd扩展名
    set_target_properties(ovf-python PROPERTIES
        SUFFIX ".pyd"
        # 确保导出所有符号
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
    
    # 设置输出目录
    set_target_properties(ovf-python PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
    )
else()
    # Linux/macOS下使用.so扩展名
    set_target_properties(ovf-python PROPERTIES
        SUFFIX ".so"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    )
    
    # Linux下需要隐藏其他符号，只导出模块初始化函数
    set_target_properties(ovf-python PROPERTIES
        C_VISIBILITY_PRESET hidden
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
endif()

# 包含目录
target_include_directories(ovf-python
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${Python3_INCLUDE_DIRS}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ovf-core/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ovf-sdk/cpp/include>
)

# 链接库
target_link_libraries(ovf-python
    PRIVATE
        ovf-core
        Python3::Python
)

# Windows下需要链接python3x.lib
if(WIN32)
    # Python3::Python已包含必要的库
endif()

# 定义导出宏
target_compile_definitions(ovf-python PRIVATE
    OVF_PYTHON_EXPORTS
)

# 安装配置
install(TARGETS ovf-python
    LIBRARY DESTINATION lib/python
    RUNTIME DESTINATION bin
)

# 安装Python模块到Python包目录（可选）
# install(FILES "${CMAKE_BINARY_DIR}/lib/ovf.pyd" 
#         DESTINATION "${Python3_SITEARCH}/ovf")

# 打印构建信息
message(STATUS "Python SDK module will be built as: ovf.pyd (Windows) / ovf.so (Linux)")
message(STATUS "Python SDK sources: ${PYTHON_SDK_SOURCES}")