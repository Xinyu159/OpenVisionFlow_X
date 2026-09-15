/**
 * @file node.h
 * @brief OpenVisionFlow 流程节点基类
 */

#pragma once

#include "types.h"
#include "error.h"
#include "data.h"
#include "logger.h"
#include <memory>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <functional>

namespace ovf {

// 前向声明
class FlowContext;

/**
 * @brief 节点信息
 */
struct NodeInfo {
    String id;              // 节点类型ID
    String name;            // 节点显示名称
    String category;        // 分类
    String description;     // 描述
    String version;         // 版本
    String author;          // 作者
    
    Vector<DataPort> inputs;   // 输入端口
    Vector<DataPort> outputs;  // 输出端口
    Vector<ParamDef> params;   // 参数定义
};

/**
 * @brief 节点基类 - 所有算子节点的基类
 */
class INode : public std::enable_shared_from_this<INode> {
public:
    using Ptr = std::shared_ptr<INode>;
    using WeakPtr = std::weak_ptr<INode>;
    
    // FlowEngine需要访问protected成员
    friend class FlowEngine;
    
    INode(const String& instance_id, const NodeInfo& info);
    virtual ~INode() = default;
    
    // 基本信息
    const String& instance_id() const { return instance_id_; }
    const NodeInfo& info() const { return info_; }
    NodeState state() const { return state_.load(); }
    const String& error_message() const { return error_message_; }
    
    // 参数管理
    void set_param(const String& key, const Data& value);
    Data get_param(const String& key, const Data& default_val = Data{}) const;
    const ParamSet& params() const { return params_; }
    
    // 输入数据管理
    void set_input(const String& port_id, const Data& data);
    Data get_input(const String& port_id) const;
    bool has_input(const String& port_id) const;
    
    // 输出数据管理
    void set_output(const String& port_id, const Data& data);
    Data get_output(const String& port_id) const;
    
    // 端口连接
    void connect_output(const String& port_id, Ptr target, const String& target_port);
    void disconnect_output(const String& port_id, Ptr target = nullptr);
    void disconnect_all();
    
    // 执行
    virtual Result<void> init() { return Result<void>::success(); }
    virtual Result<void> execute(FlowContext& context) = 0;
    virtual Result<void> reset();
    
    // 启用/禁用
    void set_enabled(bool enabled) { enabled_ = enabled; }
    bool is_enabled() const { return enabled_; }
    
    // 执行时间
    uint64_t last_execute_time() const { return last_execute_time_; }
    
protected:
    // 验证输入
    Result<void> validate_inputs() const;
    
    // 设置错误
    void set_error(const String& message);
    void clear_error();
    
    // 记录执行时间
    void record_execute_time(uint64_t microseconds);

protected:
    String instance_id_;
    NodeInfo info_;
    ParamSet params_;
    
    HashMap<String, Data> inputs_;
    HashMap<String, Data> outputs_;
    
    struct Connection {
        WeakPtr target;
        String target_port;
    };
    HashMap<String, Vector<Connection>> connections_;
    
    std::atomic<NodeState> state_{NodeState::Idle};
    String error_message_;
    bool enabled_ = true;
    uint64_t last_execute_time_ = 0;
};

// 导出宏：确保DLL/EXE边界只有一个实例，静态局部变量会导致各自独立实例、节点注册丢失。
// Windows 用 __declspec，Linux/GCC 用 visibility 属性 —— 与 ovf/api.h 的 OVF_API 同一模式。
#if defined(_WIN32) || defined(__CYGWIN__)
    #define OVF_CORE_API __declspec(dllexport)
#else
    #define OVF_CORE_API __attribute__((visibility("default")))
#endif

// ============================================================================
// 注册元数据 —— 让"注册了什么、谁覆盖了谁、元数据对不对"可查、可报告
// ============================================================================

/**
 * @brief 同一个 type_id 被注册两次时的处理策略
 */
enum class RegistrationPolicy {
    FirstWins,  //!< 默认。首次写入者获胜；同 id 的后续注册被忽略，但**记为冲突**
    Replace     //!< 显式覆盖。有意替换已有算子时才用
};

/**
 * @brief 一个注册点的出处，用来把冲突和警告指到具体的 文件:行
 */
struct RegistrationSite {
    String file;        //!< 通常是 __FILE__
    int    line = 0;    //!< 通常是 __LINE__
    String origin;      //!< 来源模块标签，默认 "unknown"；可由 set_origin() 设置

    bool empty() const { return file.empty() && line == 0; }
    String str() const;   //!< "file:line [origin]"
};

/**
 * @brief 一次 type_id 冲突
 *
 * 只在 FirstWins 策略下产生：winner 是实际生效的那个注册点，
 * loser 是被忽略的那个。两者都保留下来 —— 冲突本身就是要报告的信息。
 */
struct RegistrationConflict {
    String           type_id;
    RegistrationSite winner;
    RegistrationSite loser;
};

/**
 * @brief 一条元数据校验发现
 */
struct MetadataIssue {
    /**
     * @brief 严重度
     *
     * Error = 功能性缺陷（会让编辑器画错、让运行时取错数据），默认打 WARN。
     * Note  = 风格/完整性问题（如 author 未填），默认打 DEBUG 以免刷屏。
     */
    enum class Severity { Note, Error };

    Severity severity = Severity::Note;
    String   type_id;
    String   rule;      //!< 规则号，如 "V2"
    String   message;
};

/**
 * @brief 校验 NodeInfo 的元数据自洽性
 *
 * **只报告、不修改、绝不拒绝注册。** 返回发现的问题列表（可能为空）。
 * 之所以不拒绝：501 个上游算子里有多少条违规在体检之前是未知数，
 * 一旦设成致命就会直接违反"老算子继续能跑"的硬约束。
 *
 * 规则（详见 docs/node_authoring.md）：
 *   V1  info.id 非空                    V2  info.id == 注册用的 type_id
 *   V3  info.name 非空                  V4  输入端口 id 非空
 *   V5  输出端口 id 非空                V6  参数 id 非空
 *   V7  输入端口 id 不重复              V8  输出端口 id 不重复
 *   V9  参数 id 不重复                  V10 ParamDef 默认值类型与声明类型一致
 *   V11 min_value/max_value 为数值      V12 min <= max
 *   V13 options 里的选项非空            V14 options 仅在 String 类型上使用
 *   V15 目录/版本/作者等描述字段齐全（Note 级）
 * V15 单列成一条 Note，因为上游 501 个里绝大多数没填 author/version。
 */
OVF_CORE_API Vector<MetadataIssue> validate_node_info(const String& type_id, const NodeInfo& info);

/**
 * @brief 把校验结果压成一句话，供启动摘要使用
 */
OVF_CORE_API String summarize_metadata_issues(const Vector<MetadataIssue>& issues);

/**
 * @brief 节点工厂
 */
class OVF_CORE_API NodeFactory {
public:
    using Creator = std::function<INode::Ptr(const String&)>;

    static NodeFactory& instance();

    /**
     * @brief 注册一个节点类型
     *
     * @param site   注册点出处（OVF_REGISTER_NODE 会自动填 __FILE__/__LINE__）
     * @param policy FirstWins（默认）时同 id 重复注册**不覆盖**已有注册，
     *               而是记入 conflicts()；Replace 时才覆盖。
     * @param strict 严格模式。为 true 时该类型的元数据问题会在
     *               assert_no_strict_violations() 里被升级为致命。
     *
     * 后三个参数都有默认值，所以既有的 3 参数调用点（相机插件 2 处、
     * 流程引擎测试 4 处）一行都不用改。
     */
    void register_node(const String& type_id, Creator creator, const NodeInfo& info,
                       const RegistrationSite& site = {},
                       RegistrationPolicy policy = RegistrationPolicy::FirstWins,
                       bool strict = false);

    INode::Ptr create(const String& type_id, const String& instance_id);

    /**
     * @brief 取节点元数据
     * @warning 返回的是注册表内部的指针。注册阶段（静态初始化期）结束之后
     *          注册表不再变化，指针才是稳定的；不要在多线程注册的过程中持有它。
     */
    const NodeInfo* get_info(const String& type_id) const;

    /// 已注册的全部 type_id，**按字典序排序**
    /// （不排序的话遍历 unordered_map 每次顺序都不同，体检报告无法 diff）
    Vector<String> get_all_types() const;

    bool has_type(const String& type_id) const;

    // ------------------------------------------------------------------
    // 以下为注册加固新增（纯增量，不改变任何既有行为）
    // ------------------------------------------------------------------

    /// 已注册类型数
    size_t size() const;

    /// 本进程内发生过的 type_id 冲突，按发生顺序
    Vector<RegistrationConflict> conflicts() const;

    bool has_conflicts() const;

    /// 该 type_id 的注册出处；未注册时返回空 site
    RegistrationSite origin_of(const String& type_id) const;

    /// 全部元数据问题（含 Note 级）
    Vector<MetadataIssue> metadata_issues() const;

    /// 严格注册（OVF_REGISTER_NODE_STRICT）的类型集合
    Vector<String> strict_types() const;

    /**
     * @brief 设置后续注册的默认来源标签
     *
     * 供 initialize_algorithm_module() / initialize_user_nodes() 之类的
     * 启动入口调用，这样注册出处里能区分 upstream 和用户模块。
     */
    void set_origin(const String& origin);

    /// 当前默认来源标签
    String current_origin() const;

    /**
     * @brief 严格注册若有元数据问题则抛出
     *
     * 由 initialize_user_nodes() 之类的启动入口调用。
     *
     * 之所以不在静态初始化期当场抛：那样只会得到一句
     * "terminate called after throwing an instance of ..." 和 SIGABRT，
     * 看不到是哪个算子、违反了哪条规则、该去哪儿改。攒到启动检查再抛，
     * 就能一次把所有问题连同 文件:行 全列出来。
     */
    void assert_no_strict_violations() const;

    /// 把注册摘要（数量/冲突/元数据问题）写进日志
    void log_summary() const;

private:
    NodeFactory() = default;
    NodeFactory(const NodeFactory&) = delete;
    NodeFactory& operator=(const NodeFactory&) = delete;

    mutable std::mutex mutex_;
    HashMap<String, Creator>          creators_;
    HashMap<String, NodeInfo>         infos_;
    HashMap<String, RegistrationSite> sites_;
    HashMap<String, bool>             strict_;
    Vector<RegistrationConflict>      conflicts_;
    Vector<MetadataIssue>             issues_;
    String                            current_origin_ = "unknown";
};

// ============================================================================
// 节点注册宏
// ============================================================================

// 匿名 namespace 里的注册器名字按 __LINE__ 取，而**不是**按类名粘贴。
//
// 原因：`struct NodeClass##Registrar` 在 NodeClass 是限定名时会粘贴出
// `struct ovf::nodes::FooRegistrar`，而 C++ 不允许用限定名做类定义
// （gcc: "qualified name does not name a class before '{' token"）。
// 老算子因为类和自己同在一个 namespace 里、传的都是非限定名才没撞上。
// 改成按行号命名后，限定名和非限定名都能用 —— 这对 new_node.sh 生成的
// 新算子很重要（它们不在 ovf::algorithm 里）。
// 已核实：全仓库没有任何一行出现两个以上 OVF_REGISTER_NODE，不会撞名。
#define OVF_CONCAT_IMPL(a, b) a##b
#define OVF_CONCAT(a, b) OVF_CONCAT_IMPL(a, b)

// 来源标签优先取编译期常量 OVF_NODE_ORIGIN（若该翻译单元定义了它），
// 否则取运行时的 current_origin()。
// 为什么要编译期那一条：注册发生在静态初始化期，比 main() 早得多，
// 所以 main() 里再调 set_origin() 对已经注册完的算子已经太晚。
// 在文件头 `#define OVF_NODE_ORIGIN "ovf-nodes"` 就能给自己模块打上标签。
#ifdef OVF_NODE_ORIGIN
    #define OVF_REGISTRATION_ORIGIN() ovf::String(OVF_NODE_ORIGIN)
#else
    #define OVF_REGISTRATION_ORIGIN() ovf::NodeFactory::instance().current_origin()
#endif

#define OVF_REGISTER_NODE_IMPL(NodeClass, type_id, info, policy, strict) \
    namespace { \
        struct OVF_CONCAT(OvfNodeRegistrar_, __LINE__) { \
            OVF_CONCAT(OvfNodeRegistrar_, __LINE__)() { \
                ovf::NodeFactory::instance().register_node(type_id, \
                    [](const ovf::String& id) -> ovf::INode::Ptr { \
                        return std::make_shared<NodeClass>(id); \
                    }, \
                    info, \
                    ovf::RegistrationSite{__FILE__, __LINE__, OVF_REGISTRATION_ORIGIN()}, \
                    policy, strict); \
            } \
        } OVF_CONCAT(ovf_node_registrar_, __LINE__); \
    }

/**
 * @brief 注册一个节点类型（老算子用这个）
 *
 * 用法： OVF_REGISTER_NODE(RotateNode, "RotateNode", RotateNode::make_info())
 * 也接受限定名： OVF_REGISTER_NODE(ovf::nodes::WaferEdgeFind, "WaferEdgeFind", ...)
 *
 * 元数据问题只记录、只告警，不影响注册。
 */
#define OVF_REGISTER_NODE(NodeClass, type_id, info) \
    OVF_REGISTER_NODE_IMPL(NodeClass, type_id, info, \
        ovf::RegistrationPolicy::FirstWins, false)

/**
 * @brief 注册一个节点类型（严格模式，新算子用这个）
 *
 * 与 OVF_REGISTER_NODE 的区别只有一点：元数据问题会在启动自检
 * （NodeFactory::assert_no_strict_violations()）里被升级为致命错误。
 * 用于自己写的算子 —— 从第一天起元数据就是干净的。
 */
#define OVF_REGISTER_NODE_STRICT(NodeClass, type_id, info) \
    OVF_REGISTER_NODE_IMPL(NodeClass, type_id, info, \
        ovf::RegistrationPolicy::FirstWins, true)

} // namespace ovf