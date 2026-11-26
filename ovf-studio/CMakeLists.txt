# ovf-studio — 桌面端工作台（HDevelop 式，不再依赖浏览器）
# Dear ImGui (docking) + Win32 + D3D11，单 exe，无外部运行时依赖。
cmake_minimum_required(VERSION 3.16)

project(ovf-studio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui)

add_executable(ovf-studio
    src/studio_main.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_win32.cpp
    ${IMGUI_DIR}/backends/imgui_impl_dx11.cpp
)

target_include_directories(ovf-studio PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/../ovf-core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../ovf-nodes/include
)

# 算子来自 ovf-nodes（whole-archive 由 ovf-nodes-link 负责，静态注册必须拉全）
target_link_libraries(ovf-studio PRIVATE
    ovf-core
    ovf-nodes-link
    d3d11
    dxgi
    d3dcompiler
    dwmapi
)

if(MSVC)
    target_compile_options(ovf-studio PRIVATE /utf-8)
    target_compile_definitions(ovf-studio PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
    # -municode 必须同时进编译和链接（入口选 wWinMain 的 unicode crt）
    target_compile_options(ovf-studio PRIVATE -municode)
    target_link_options(ovf-studio PRIVATE -municode)
endif()

install(TARGETS ovf-studio RUNTIME DESTINATION bin)
