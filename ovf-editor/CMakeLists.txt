# ovf-editor - 图形化编辑器
cmake_minimum_required(VERSION 3.16)

project(ovf-editor VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 源文件
file(GLOB_RECURSE EDITOR_SOURCES "src/*.cpp")
file(GLOB_RECURSE EDITOR_HEADERS "src/*.h")
file(GLOB_RECURSE EDITOR_UI "ui/*.ui")
file(GLOB_RECURSE EDITOR_QRC "resources/*.qrc")

# 可执行文件
add_executable(ovf-editor
    ${EDITOR_SOURCES}
    ${EDITOR_HEADERS}
    ${EDITOR_UI}
    ${EDITOR_QRC}
)

target_include_directories(ovf-editor
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ovf-editor
    PRIVATE
        ovf-core
        ovf-algorithm
        Qt6::Core
        Qt6::Widgets
        Qt6::Gui
        Qt6::Network
)

# 安装
install(TARGETS ovf-editor RUNTIME DESTINATION bin)