# ovf-core - 微内核核心模块
cmake_minimum_required(VERSION 3.16)

project(ovf-core VERSION 0.2.0 LANGUAGES CXX)

# 源文件
file(GLOB_RECURSE CORE_SOURCES
    "src/*.cpp"
)

file(GLOB_RECURSE CORE_HEADERS
    "include/*.h"
)

# 库类型：Windows 下用静态库。
# 原因：WINDOWS_EXPORT_ALL_SYMBOLS 对 MinGW 无法导出虚函数 vtable/内联成员
# （ovf-nodes 链接时报 undefined reference to vtable for ovf::INode），
# 静态库直接绕开整套 dllimport/dllexport 泥潭，产物也更好分发。
# Linux 保持 SHARED（静态注册 + --as-needed 的老问题按 ovf-nodes-link 处理）。
if(WIN32)
    add_library(ovf-core STATIC ${CORE_SOURCES} ${CORE_HEADERS})
else()
    add_library(ovf-core SHARED ${CORE_SOURCES} ${CORE_HEADERS})
endif()

target_include_directories(ovf-core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Windows Socket库
if(WIN32)
    target_link_libraries(ovf-core PUBLIC ws2_32)
endif()

# 安装
install(TARGETS ovf-core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY thirdparty/ DESTINATION include)