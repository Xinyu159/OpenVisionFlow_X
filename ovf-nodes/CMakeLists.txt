# ovf-nodes - 用户算子模块
#
# 你自己的算子住在这里。一个算子一个 .cpp，扔进 src/ 就行 —— 这棵树用了
# CONFIGURE_DEPENDS 的 glob，**不需要手动重跑 cmake**。
cmake_minimum_required(VERSION 3.16)

project(ovf-nodes
    VERSION 0.1.0
    DESCRIPTION "OpenVisionFlow user operator module"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 源文件
# ============================================================================

# CONFIGURE_DEPENDS：构建时如果这个目录下多了/少了 .cpp，CMake 会自己重跑
# 配置把新文件拾取进来。这是"写完新算子直接 cmake --build 就跑"能成立的前提。
# （`ovf-algorithm/` 那边没有这个，那是上游的写法，不动它。）
file(GLOB_RECURSE NODES_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE NODES_HEADERS CONFIGURE_DEPENDS "include/*.h")

add_library(ovf-nodes SHARED ${NODES_SOURCES} ${NODES_HEADERS})

target_include_directories(ovf-nodes
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 只依赖 ovf-core。不依赖 ovf-algorithm —— 501 个老算子是你的历史资产，
# 不是新算子的依赖。这是有意的分层，见 upstream_headers.h 里的说明。
target_link_libraries(ovf-nodes PUBLIC ovf-core)

# ★ 给本模块**所有**翻译单元打上注册出处标签。
#
#   OVF_REGISTER_NODE 在静态初始化期读的是编译期常量 OVF_NODE_ORIGIN
#   （见 ovf-core/include/ovf/core/node.h:397），所以它必须是**编译期**定义 ——
#   main() 里再调 set_origin() 对那些早就注册完的算子已经太晚了。
#
#   PRIVATE：只影响本模块的翻译单元，不污染消费方。
target_compile_definitions(ovf-nodes PRIVATE "OVF_NODE_ORIGIN=\"ovf-nodes\"")

# ============================================================================
# ★ 第 1 层防御：把库钉在链接命令行上（--as-needed）
# ============================================================================
#
# 背景：算子是**静态初始化期自注册**的。一个共享库如果消费者没引用它任何
# 非弱符号，Ubuntu 默认的 `--as-needed` 会认为"没人要"，在链接期整库丢掉 ——
# 零警告。库一丢，静态初始化不跑，算子全部消失，运行时才报
# "Node type 'X' not found"。
#
# 为什么不用 $<LINK_LIBRARY:WHOLE_ARCHIVE>：那是 CMake ≥ 3.24 的生成器表达式
# （本机是 3.22.1），而且它本身是**静态库**的概念 —— 强制把 .a 的所有成员拉进来。
# 对 SHARED 库，正解就是 --no-as-needed：不丢弃，DT_NEEDED 照常记。
#
# 用途：**消费方只链 ovf-nodes-link，永远不要直接链 ovf-nodes。**
#
#     target_link_libraries(你的目标 PRIVATE ovf-nodes-link)
#
add_library(ovf-nodes-link INTERFACE)

if(WIN32)
    # MSVC 没有 --as-needed；它不会丢弃"没被引用的"导入库。
    # Windows 上这个 target 只是个语义别名，保持两种平台调用姿势一致。
    target_link_libraries(ovf-nodes-link INTERFACE ovf-nodes)
else()
    # --no-as-needed 走 LINK_OPTIONS：它是个**开关**，作用于其后出现的库，
    # 而 CMake 把 LINK_OPTIONS 放在链接命令行靠前的位置，正好合适。
    target_link_options(ovf-nodes-link INTERFACE "-Wl,--no-as-needed")
    # 库本体 + 把开关恢复成默认。这两项按原顺序留在 LINK_LIBRARIES 里，
    # 于是 ld 看到的是 `... -Wl,--no-as-needed libovf-nodes.so -Wl,--as-needed ...`
    # —— 只对本库生效，不会把后面所有库都变成"不丢弃"。
    target_link_libraries(ovf-nodes-link INTERFACE ovf-nodes "-Wl,--as-needed")
endif()

# ============================================================================
# 测试
# ============================================================================
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# ============================================================================
# 安装
# ============================================================================
if(WIN32)
    set_target_properties(ovf-nodes PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

install(TARGETS ovf-nodes
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
