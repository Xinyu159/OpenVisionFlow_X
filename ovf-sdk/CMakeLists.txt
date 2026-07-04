# ovf-sdk - SDK模块
cmake_minimum_required(VERSION 3.16)

project(ovf-sdk VERSION 0.1.0 LANGUAGES CXX)

# C++ SDK
file(GLOB_RECURSE SDK_CPP_SOURCES "cpp/src/*.cpp")
file(GLOB_RECURSE SDK_CPP_HEADERS "cpp/include/*.h")

add_library(ovf-sdk-cpp SHARED ${SDK_CPP_SOURCES} ${SDK_CPP_HEADERS})

target_include_directories(ovf-sdk-cpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ovf-sdk-cpp PUBLIC ovf-core)

# 安装
install(TARGETS ovf-sdk-cpp
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY cpp/include/ DESTINATION include)