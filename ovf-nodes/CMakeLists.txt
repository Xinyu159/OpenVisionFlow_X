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

# Windows 下用静态库：ovf-core 已是静态（理由见其 CMakeLists），DLL 方案会把
# core 符号再自动导出一遍，配合 whole-archive 就是 multiple definition。
# 静态 + whole-archive（ovf-nodes-link）= 注册对象必然进镜像。
if(WIN32)
    add_library(ovf-nodes STATIC ${NODES_SOURCES} ${NODES_HEADERS})
else()
    add_library(ovf-nodes SHARED ${NODES_SOURCES} ${NODES_HEADERS})
endif()

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
    # Windows 的对偶问题：ovf-core 改静态库后，链接器对静态库只抽取"被引用过"
    # 的目标文件 —— 带 OVF_REGISTER_NODE 静态注册对象的目标文件没有任何显式
    # 引用，整块不被抽取，算子全部消失（症状与 Linux 的 --as-needed 一模一样：
    # 注册表为空、链接期零警告）。
    # 修法：whole-archive（MinGW ld 展开为 -Wl,--whole-archive，MSVC 为
    # /WHOLEARCHIVE），把库里的注册对象无条件拉进最终镜像。
    # CMake >= 3.24 的 LINK_LIBRARY:WHOLE_ARCHIVE 生成器表达式同时覆盖两家。
    target_link_libraries(ovf-nodes-link INTERFACE
        "$<LINK_LIBRARY:WHOLE_ARCHIVE,ovf-nodes>"
    )
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
