# ovf-web-server - Web图形化编辑器后端API服务
cmake_minimum_required(VERSION 3.16)

project(ovf-web-server
    VERSION 0.1.0
    DESCRIPTION "OpenVisionFlow Web Server"
    LANGUAGES CXX
)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(OpenSSL QUIET)

# 源文件
set(WEB_SERVER_SOURCES
    src/web_server.cpp
    src/api_handlers.cpp
    src/execution_log_visualizer.cpp
    src/performance_analyzer.cpp
    src/flow_execution_monitor.cpp
    src/main.cpp
)

# 头文件
set(WEB_SERVER_HEADERS
    include/ovf/web_server.h
    include/ovf/execution_log_visualizer.h
    include/ovf/performance_analyzer.h
    include/ovf/flow_execution_monitor.h
)

# 创建可执行文件
add_executable(ovf-web-server
    ${WEB_SERVER_SOURCES}
    ${WEB_SERVER_HEADERS}
)

# 包含目录
target_include_directories(ovf-web-server
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../ovf-core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../ovf-core/thirdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../ovf-algorithm/include
)

# 链接库（ws2_32 是 Windows 专属，只在下面 if(WIN32) 块里链，不能放这里）
target_link_libraries(ovf-web-server
    PRIVATE
        ovf-core
        ovf-algorithm
)

# Windows特定设置
if(WIN32)
    target_compile_definitions(ovf-web-server PRIVATE
        _WIN32_WINNT=0x0601
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )
    
    # 链接Windows网络库
    target_link_libraries(ovf-web-server PRIVATE
        ws2_32
        mswsock
    )
endif()

# 安装
install(TARGETS ovf-web-server
    RUNTIME DESTINATION bin
)