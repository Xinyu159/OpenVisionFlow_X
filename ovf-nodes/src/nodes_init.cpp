/**
 * @file nodes_init.cpp
 * @brief 用户算子模块的锚点与自检
 *
 * 这个文件里没有算子，只有三样东西：
 *   1. **锚点** `ovf_nodes_module_anchor` —— libovf-nodes.so 的"门牌号"，
 *      消费方用弱引用探它，就能知道库到底进没进这个进程；
 *   2. **清单** —— 每个算子文件通过 `OVF_REGISTER_USER_NODE` 在静态初始化期
 *      往这里登记自己的 type_id，于是"应该有几个算子"是可查的；
 *   3. **自检** —— 拿清单比对 `NodeFactory` 的真实内容，把"静默少了一个算子"
 *      变成启动时的一条可操作报错。
 *
 * 参照 `ovf-algorithm/src/algorithm_init.cpp` 的写法，但那个文件**不能改**
 * （`ovf-algorithm/` 对上游保持零 diff），所以同类的收尾工作落在新建的
 * `ovf-nodes/` 里。
 */

// 必须在 include nodes.h 之前：本文件是锚点的**定义**处，
// 不能让头文件里的 weak 声明先生效（那会把定义也变成弱符号）。
#define OVF_NODES_DEFINING_ANCHOR 1

#include "ovf/nodes/nodes.h"

#include <algorithm>
#include <mutex>

namespace ovf {
namespace nodes {

// ===========================================================================
// 锚点
// ===========================================================================

// 定义在这里、导出、**非弱**。消费方那边是 weak 声明，所以：
//   库进了进程   → 弱引用解析到这个函数，非空；
//   库被丢掉了   → 弱引用是 nullptr，require_user_node_module_loaded() 当场报错。
//
// 必须是函数而不是变量：弱**变量**解析不到时地址是 0，读它的值就是空指针
// 解引用（实测直接段错误）。见 nodes.h 里那段注释。
OVF_NODES_API const char* ovf_nodes_module_anchor() { return "ovf-nodes"; }

const char* module_name() { return ovf_nodes_module_anchor(); }

// ===========================================================================
// 清单
// ===========================================================================

namespace {

// Meyers 单例，不是 namespace 级的全局 Vector。
//
// 原因很实在：登记发生在**静态初始化期**，而那正是"静态初始化顺序问题"
// 的战场 —— 一个 namespace 级的 Vector 可能还没构造就被别的 TU 的注册器
// 碰了。函数内的 static 保证"第一次用到时才构造"，顺序问题不存在。
Vector<String>& declared_registry() {
    static Vector<String> ids;
    return ids;
}

// 静态初始化是单线程的（main 之前没有别的线程），本来是够的；
// 加锁是因为节点清单也会被 `new_node.sh` 之外的路径（比如测试）在运行期写，
// 而 `NodeFactory::register_node` 本身就上了锁 —— 这里跟着惯例走。
std::mutex& declared_mutex() {
    static std::mutex m;
    return m;
}

} // namespace

void declare_user_node(const String& type_id) {
    std::lock_guard<std::mutex> lock(declared_mutex());
    auto& ids = declared_registry();
    if (std::find(ids.begin(), ids.end(), type_id) == ids.end()) {
        ids.push_back(type_id);
    }
}

Vector<String> declared_type_ids() {
    std::lock_guard<std::mutex> lock(declared_mutex());
    Vector<String> ids = declared_registry();
    // 排序：阶段 1 已经给 get_all_types() 做过同样的事。顺序不固定的话
    // 每次启动的日志、每份体检报告都没法 diff。
    std::sort(ids.begin(), ids.end());
    return ids;
}

Vector<String> tagged_type_ids() {
    auto& factory = NodeFactory::instance();
    Vector<String> out;
    for (const auto& id : factory.get_all_types()) {
        if (factory.origin_of(id).origin == module_name()) {
            out.push_back(id);
        }
    }
    return out;   // get_all_types() 已经是有序的
}

// ===========================================================================
// 自检
// ===========================================================================

UserNodeReport check_user_nodes_against(const Vector<String>& declared) {
    auto& factory = NodeFactory::instance();

    UserNodeReport report;
    report.declared = declared;
    std::sort(report.declared.begin(), report.declared.end());

    for (const auto& id : report.declared) {
        if (!factory.has_type(id)) {
            report.missing.push_back(id);
        } else if (factory.origin_of(id).origin != module_name()) {
            // 注册表里有这个 type_id，但不是我们注册的 —— 撞车了。
            // FirstWins 策略下先注册的先赢，我们的那份被**静默丢弃**。
            // 这跟"根本没有"是两回事：一个要改链接，一个要改 type_id。
            report.hijacked.push_back(id);
        } else {
            report.registered.push_back(id);
        }
    }
    return report;
}

UserNodeReport check_user_nodes() {
    return check_user_nodes_against(declared_type_ids());
}

namespace {

String join(const Vector<String>& ids) {
    if (ids.empty()) return "（无）";
    String s;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) s += ", ";
        s += ids[i];
    }
    return s;
}

} // namespace

String format_user_node_report(const UserNodeReport& report) {
    if (report.ok()) {
        return "user node module '" + String(module_name()) + "': " +
               std::to_string(report.registered.size()) + " node type(s), all registered";
    }

    String msg = "用户算子模块自检失败：声明 " + std::to_string(report.declared.size()) +
                 " 个，实际注册上 " + std::to_string(report.registered.size()) + " 个。\n";

    if (!report.missing.empty()) {
        msg += "\n  根本没注册上（注册表里查不到这个 type_id）：\n    " + join(report.missing) + "\n";
    }
    if (!report.hijacked.empty()) {
        msg += "\n  type_id 被别的注册点抢先占了（FirstWins 把我们的那份丢了）：\n    " +
               join(report.hijacked) + "\n";
    }

    msg += "\n  当前注册表里出处标着 \"" + String(module_name()) + "\" 的类型共 " +
           std::to_string(tagged_type_ids().size()) + " 个：\n    " + join(tagged_type_ids()) + "\n";

    msg +=
        "\n  这三种原因，修法完全不同：\n"
        "\n"
        "  [1] libovf-nodes.so 被链接器整个丢掉了\n"
        "      Ubuntu 的 gcc/ld 默认开 --as-needed：一个共享库只要没有被引用的\n"
        "      非弱符号，链接期就被当成\"没人要\"扔掉，**零警告**。丢掉了，静态\n"
        "      初始化就不跑，你的算子全部变成运行时的 Node type 'X' not found。\n"
        "      修法：消费方把链接目标从 ovf-nodes 换成 ovf-nodes-link：\n"
        "            target_link_libraries(你的目标 PRIVATE ovf-nodes-link)\n"
        "      它自带 -Wl,--no-as-needed，把库钉死。\n"
        "\n"
        "  [2] 进程里存在两个 NodeFactory 实例\n"
        "      同一个 ovf-core 既以静态库、又以动态库进了同一个二进制时，\n"
        "      静态库那份会有一份自己的单例。算子注册进了另一个副本，\n"
        "      本进程看不见。修法：统一走 libovf-core.so，别混用。\n"
        "\n"
        "  [3] type_id 撞车\n"
        "      上面的 \"被抢先占了\" 那一栏就是这种。修法：换个 type_id\n"
        "      （推荐带 '模块.' 前缀，比如 nodes.MyBlur），或显式改用\n"
        "      RegistrationPolicy::Replace —— 那是你有意要覆盖老算子时才做的。\n"
        "\n"
        "  诊断完成之后重跑：cmake --build build -j16";

    return msg;
}

void require_user_nodes() {
    const UserNodeReport report = check_user_nodes();
    if (!report.ok()) {
        // 抛而不是 abort：调用方能加上"是谁在等这些算子"的上下文，
        // 而且栈能保住，调试器里看得见是谁触发的。
        throw Exception(ErrorCode::NodeNotFound, format_user_node_report(report));
    }
}

// ===========================================================================
// 启动入口
// ===========================================================================

void initialize_user_nodes() {
    auto& factory = NodeFactory::instance();

    // 定位：注册本身早在 main() 之前就完成了（静态初始化期），这个函数
    // **不能**让注册发生，它只做收尾和自检。它存在的第一个理由是 ODR-use：
    // 只要有这一句调用，链接器就不敢把 libovf-nodes.so 当成"没人要"丢掉。
    factory.set_origin(module_name());

    // 严格注册（OVF_REGISTER_USER_NODE）的元数据问题在这里升级为致命。
    // 攒到这一步再抛，是为了能一次把所有问题连同 文件:行 全列出来 ——
    // 在静态初始化期当场抛只会得到一句 "terminate called after throwing..."
    // 和 SIGABRT，看不到是哪个算子、违反了哪条。
    factory.assert_no_strict_violations();

    const UserNodeReport report = check_user_nodes();
    if (report.ok()) {
        OVF_INFO() << "User node module '" << module_name() << "': "
                   << report.registered.size() << " node type(s) registered";
    } else {
        // 不在这里抛：消息交给 initialize_user_nodes 的调用方决定怎么呈现。
        // 但**一定要抛**出去的路径是 require_user_nodes()，那是硬断言。
        require_user_nodes();
    }
}

} // namespace nodes
} // namespace ovf
