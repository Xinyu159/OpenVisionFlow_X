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

# 创建动态库
add_library(ovf-core SHARED ${CORE_SOURCES} ${CORE_HEADERS})

# Windows下生成导入库
if(WIN32)
    set_target_properties(ovf-core PROPERTIES 
        WINDOWS_EXPORT_ALL_SYMBOLS ON
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
    )
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