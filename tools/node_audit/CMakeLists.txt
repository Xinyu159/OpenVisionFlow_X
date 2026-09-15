# ovf-node-audit —— 501 个上游算子的体检工具
#
# ★ 它是**可执行文件**，不是库。这一点很关键：父进程自己必须先是一个
#   正常的、单线程的、已经注册完 501 个算子的进程，然后才能 fork。
#   （算子的注册是静态初始化期完成的，所以 main() 跑起来的时候注册表已经满了。）
#
# ★ 链 ovf-algorithm 时**没有** ovf-algorithm-link 这种 INTERFACE target
#   （计划书要求 ovf-algorithm/CMakeLists.txt 对上游零 diff，不能给它加包装）。
#   所以这里靠第二层防御：main.cpp 里那句 `initialize_algorithm_module()`
#   是对非弱符号的 ODR-use，链接器不敢把库当成"没人要"丢掉。
#   实测（阶段 4 的组合 D）：单独第二层就足够救活库。

cmake_minimum_required(VERSION 3.16)

project(ovf-node-audit
    VERSION 0.1.0
    DESCRIPTION "OpenVisionFlow node health audit tool"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ovf-node-audit
    main.cpp
    audit_child.cpp
    audit_worker.cpp
    audit_report.cpp
)

target_include_directories(ovf-node-audit
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../ovf-core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../ovf-core/thirdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../../ovf-algorithm/include
)

target_link_libraries(ovf-node-audit
    PRIVATE
        ovf-core
        ovf-algorithm
)

# 把仓库根路径烧进二进制：
#   - 可中断性统计要扫 ovf-algorithm/src
#   - 标注表默认写到 <仓库根>/health/node_health.json
# 这样从任何 cwd 启动都找得到（构建目录被删掉的话这两个功能才失效，
# 那时候命令行 --source-dir / --health-out 依然能覆盖）。
target_compile_definitions(ovf-node-audit PRIVATE
    OVF_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

# 顺手也体检自己的算子。注意链的是 ovf-nodes-link（--no-as-needed 钉住），
# 直接链 ovf-nodes 会被链接器整库丢掉，你的算子一个都不会出现在报告里。
if(BUILD_OVF_NODES)
    target_link_libraries(ovf-node-audit PRIVATE ovf-nodes-link)
endif()

# 体检报告不写进 build/bin，统一落在这里
set_target_properties(ovf-node-audit PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

install(TARGETS ovf-node-audit RUNTIME DESTINATION bin)
