#pragma once
/**
 * @file nodes.h
 * @brief 你自己的算子住在这儿 —— 模块入口 + `--as-needed` 的三层防御
 *
 * ## 一分钟上手
 *
 * ```cpp
 * // ovf-nodes/src/MyBlur.cpp
 * #include "ovf/nodes/nodes.h"
 * namespace ovf { namespace nodes {
 *
 * class MyBlur : public INode {
 * public:
 *     MyBlur(const String& id) : INode(id, make_info()) {}
 *     static NodeInfo make_info();
 *     Result<void> execute(FlowContext& ctx) override;
 * };
 *
 * NodeInfo MyBlur::make_info() { ... }
 * Result<void> MyBlur::execute(FlowContext& ctx) { ... }
 *
 * OVF_REGISTER_USER_NODE(MyBlur, "nodes.MyBlur", MyBlur::make_info())
 * }}
 * ```
 *
 * 然后 `cmake --build build -j16` —— 不需要重新跑 cmake，
 * `ovf-nodes/CMakeLists.txt` 用的是 `CONFIGURE_DEPENDS` 的 glob。
 *
 * ---------------------------------------------------------------------------
 *
 * ## 为什么这里有一整套"防御"
 *
 * 算子是**静态初始化期自注册**的：每个算子文件里那个 `OVF_REGISTER_NODE`
 * 展开出一个匿名 namespace 里的全局对象，它的构造函数在 `main()` 之前
 * 就往 `NodeFactory` 里登记。这套机制很优雅，但有一个致命的失败模式：
 *
 * > **链接器可以把整个 `libovf-nodes.so` 丢掉。**
 *
 * Ubuntu 的 gcc/ld 默认开 `--as-needed`：一个共享库如果**没有任何被引用的
 * 非弱符号**，链接器认为"没人要"，直接在链接期扔掉它。库一被扔，静态初始化
 * 就不跑，你的算子**一个都不剩** —— 而且**链接期零警告、零报错**，
 * 一直到你画完流程点运行，才看见 `Node type 'X' not found`。
 *
 * 更糟的是这跟"没写"长得一模一样：注册表里就是没有，没有痕迹可查。
 *
 * 所以这里上三层纵深防御，每一层挡的是不同的东西：
 *
 * | 层 | 机制 | 挡住什么 | 什么时候报 |
 * |---|---|---|---|
 * | 1 | `ovf-nodes-link` 这个 INTERFACE target（自带 `-Wl,--no-as-needed`） | 库被链接器丢掉 | 构建期（根本不发生） |
 * | 2 | `initialize_user_nodes()`（消费方 ODR-use 它） | 同上，给"忘了写防御"的消费方兜底 | 链接期（符号找不到） |
 * | 3 | `require_user_node_module_loaded()` + `require_user_nodes()` | 库在/不在、算子被顶掉、注册进了另一个 NodeFactory 副本 | **运行期，启动时**，消息可操作 |
 *
 * **消费方的唯一正确姿势：只链 `ovf-nodes-link`，永远不要直接链 `ovf-nodes`。**
 *
 * ```cmake
 * target_link_libraries(你的目标 PRIVATE ovf-nodes-link)
 * ```
 */

#include <ovf/core/node.h>
#include <ovf/core/error.h>
#include <ovf/core/logger.h>

#include "ovf/nodes/upstream_headers.h"

// 导出宏：与 OVF_CORE_API 同一模式。DLL/EXE 边界上必须只有一个实例，
// 否则静态局部变量会各自独立，节点注册就丢了。
#if defined(_WIN32) || defined(__CYGWIN__)
    #define OVF_NODES_API __declspec(dllexport)
#else
    #define OVF_NODES_API __attribute__((visibility("default")))
#endif

namespace ovf {
namespace nodes {

/// 本模块的标识。注册出处（`RegistrationSite::origin`）与清单都用它。
OVF_NODES_API const char* module_name();

// ===========================================================================
// 第 3 层 · 之一：库到底在不在（弱符号探测）
// ===========================================================================

// `ovf_nodes_module_anchor()` 定义在 `src/nodes_init.cpp` 里，就是
// libovf-nodes.so 的"门牌号"。消费方这边声明成 **weak**（弱引用），于是：
//
//   * 库被 `--as-needed` 丢掉 → 链接**照样成功**（弱引用不参与"要不要保留这个库"
//     的判定），运行时这个符号解析成 nullptr → 我们能**当场**把话说清楚；
//   * 库在 → 弱引用照常解析到定义，非空。
//
// 为什么要故意用弱引用把"链接期报错"换成"运行期断言"：链接期报错只有
// `undefined reference to ...` 一行，看得出符号名、看不出原因；而这里能
// 直接告诉你 `--as-needed` 是怎么回事、该改哪一行 CMake。
//
// ⚠️ 这里必须是个**函数**，不能是 `extern const char* ovf_nodes_module_anchor`。
// 实测（GCC 13 / ld 2.42）：弱**变量**符号解析不到时地址是 0，而读它的值
// 就是**空指针解引用** —— 直接段错误。弱**函数**符号比较的是函数地址本身，
// 才有"没有就是 nullptr"这个语义。两种写法的对照实验见
// docs/hardening/stage4_nodes.md 的"踩到的坑"一节。
//
// Windows 上没有 `__attribute__((weak))`（MSVC 的 `/alternatename` 是另一回事），
// 那边的防线是链接期本身就过不去 —— 报错更早，也够响，所以退化成恒真。
#if defined(OVF_NODES_DEFINING_ANCHOR)
    // 本模块自己的翻译单元：这里就是定义处，不能声明成 weak，
    // 否则导出出去的也是个弱符号。
    extern "C" OVF_NODES_API const char* ovf_nodes_module_anchor();
    #define OVF_NODES_ANCHOR_WEAK 0
#elif defined(__GNUC__) || defined(__clang__)
    extern "C" const char* ovf_nodes_module_anchor() __attribute__((weak));
    #define OVF_NODES_ANCHOR_WEAK 1
#else
    extern "C" OVF_NODES_API const char* ovf_nodes_module_anchor();
    #define OVF_NODES_ANCHOR_WEAK 0
#endif

/// libovf-nodes.so 是否真的进了这个进程
inline bool user_node_module_loaded() {
#if OVF_NODES_ANCHOR_WEAK
    return ovf_nodes_module_anchor != nullptr;
#else
    return true;
#endif
}

/**
 * @brief 断言 libovf-nodes.so 在这个进程里；不在就抛，消息带可操作的修法
 *
 * @param who 谁在问（写进消息，方便定位是哪个消费方需要算子）
 */
inline void require_user_node_module_loaded(const char* who) {
    if (user_node_module_loaded()) return;

    throw Exception(ErrorCode::NotFound, String(who) +
        ": user node module (libovf-nodes.so) is NOT loaded in this process.\n"
        "\n"
        "  这个断言几乎总是意味着同一件事：链接器把整个 libovf-nodes.so 丢掉了。\n"
        "  Ubuntu 的 gcc/ld 默认开 --as-needed —— 一个共享库只要没有被引用的\n"
        "  非弱符号，链接期就被当成\"没人要\"扔掉，**零警告**。库一被扔，算子\n"
        "  的静态初始化就不跑，你的算子全部消失，运行时才报 Node type 'X' not found。\n"
        "\n"
        "  修法：把链接目标从 ovf-nodes 换成 ovf-nodes-link（它自带\n"
        "        -Wl,--no-as-needed 把库钉住）：\n"
        "\n"
        "            target_link_libraries(你的目标 PRIVATE ovf-nodes-link)\n"
        "\n"
        "  顺带确认构建时开了用户算子模块：-DBUILD_OVF_NODES=ON（默认就是 ON）。");
}

// ===========================================================================
// 第 3 层 · 之二：库在，但算子是不是都注册上了
// ===========================================================================

/**
 * @brief 用户算子模块的自检结论
 *
 * 三类失败分得很开，因为**修法完全不同**（这是这套自检存在的全部理由）：
 *   - `missing`  —— 注册表里**根本没有**这个 type_id。库没进来 / 该 .cpp 没参与构建 /
 *                   `OVF_REGISTER_USER_NODE` 被删了。
 *   - `hijacked` —— type_id 在注册表里，但**不是我们注册的**。撞车了：
 *                   上游或另一个文件先注册了同名 type_id，FirstWins 把我们的丢掉了。
 */
struct UserNodeReport {
    Vector<String> declared;     ///< 本模块声明要注册的 type_id（字典序）
    Vector<String> registered;   ///< 真的注册上了、且出处是本模块
    Vector<String> missing;      ///< 注册表里根本没有
    Vector<String> hijacked;     ///< 有，但被别人先占了

    bool ok() const { return missing.empty() && hijacked.empty(); }
};

/// 把清单记进本模块的 declared 表（由 OVF_REGISTER_USER_NODE 在静态初始化期调用）
OVF_NODES_API void declare_user_node(const String& type_id);

/// 本模块声明的全部 type_id（字典序）
OVF_NODES_API Vector<String> declared_type_ids();

/// 当前 NodeFactory 里出处标着本模块的 type_id（字典序）—— 诊断用
OVF_NODES_API Vector<String> tagged_type_ids();

/// 拿 `declared` 去比对当前注册表。这个重载是为了**测试**：能注入假清单。
OVF_NODES_API UserNodeReport check_user_nodes_against(const Vector<String>& declared);

/// `check_user_nodes_against(declared_type_ids())`
OVF_NODES_API UserNodeReport check_user_nodes();

/// 把结论格式化成一段**可操作**的说明（不是"出错了"，是"该改哪一行"）
OVF_NODES_API String format_user_node_report(const UserNodeReport& report);

/// 自检不过就抛。**绝不允许静默少一个算子。**
OVF_NODES_API void require_user_nodes();

// ===========================================================================
// 第 2 层：启动入口
// ===========================================================================

/**
 * @brief 启动入口 —— 消费方在 `main()` 里调一次
 *
 * ⚠️ 说清楚它**不是**什么：它**不是**"注册的触发点"。注册发生在静态初始化期，
 * 比 `main()` 早得多；等这个函数跑起来，注册早就结束了。这个函数做到的是两件事：
 *
 *   1. **被消费方 ODR-use** —— 只要有这一句调用，链接器就必须保留整个
 *      libovf-nodes.so，静态初始化才有机会跑；（这就是第 2 层）
 *   2. 把自检放在**启动路径**上 —— 算子少了就在启动时炸，
 *      而不是等你画完流程点运行才发现。
 *
 * 位置参照仓库已有的 `initialize_algorithm_module()`，两者是同一个惯例。
 */
OVF_NODES_API void initialize_user_nodes();

} // namespace nodes
} // namespace ovf

// ===========================================================================
// 注册宏
// ===========================================================================

/**
 * @brief 注册一个**自己的**算子（区别于上游那 501 个的 `OVF_REGISTER_NODE`）
 *
 * 与 `OVF_REGISTER_NODE` 有两处不同，都是有意的：
 *
 *   1. **严格模式**（等价于 `OVF_REGISTER_NODE_STRICT`）—— 元数据问题
 *      （比如 `info.id` 和注册用的 type_id 对不上、端口重名）会在
 *      `initialize_user_nodes()` 里被升级为致命，一次列全。你自己的算子
 *      从第一天起就是干净的，不会攒成 501 份历史债。
 *   2. **记进模块清单** —— `require_user_nodes()` 靠这份清单判断
 *      "有没有算子被静默丢掉"。没有清单就没法知道"应该有几个"。
 *
 * 用法（推荐带 `模块.` 前缀，撞车一眼可见）：
 *
 * ```cpp
 * OVF_REGISTER_USER_NODE(MyBlur, "nodes.MyBlur", MyBlur::make_info())
 * ```
 *
 * 注意类名可以写限定名（`ovf::nodes::MyBlur`），注册器对象的名字是按
 * `__LINE__` 取的，不是按类名粘贴的 —— 所以限定名不会坏。
 */
#define OVF_REGISTER_USER_NODE(NodeClass, type_id, info)                       \
    OVF_REGISTER_NODE_IMPL(NodeClass, type_id, info,                           \
        ovf::RegistrationPolicy::FirstWins, /*strict=*/true)                   \
    namespace {                                                                \
        struct OVF_CONCAT(OvfUserNodeDecl_, __LINE__) {                        \
            OVF_CONCAT(OvfUserNodeDecl_, __LINE__)() {                         \
                ovf::nodes::declare_user_node(type_id);                        \
            }                                                                  \
        } OVF_CONCAT(ovf_user_node_decl_, __LINE__);                           \
    }
