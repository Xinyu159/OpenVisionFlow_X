/**
 * @file node.cpp
 * @brief 节点基类实现
 */

#include "ovf/core/node.h"
#include "ovf/core/flow.h"
#include <chrono>
#include <algorithm>

namespace ovf {

// ============================================================================
// 注册元数据
// ============================================================================

String RegistrationSite::str() const {
    if (empty()) {
        return origin.empty() ? String("(unknown registration site)")
                              : ("(" + origin + ", unknown location)");
    }
    String s = file + ":" + std::to_string(line);
    if (!origin.empty()) s += " [" + origin + "]";
    return s;
}

Vector<MetadataIssue> validate_node_info(const String& type_id, const NodeInfo& info) {
    Vector<MetadataIssue> issues;

    auto add = [&](MetadataIssue::Severity sev, const char* rule, const String& msg) {
        MetadataIssue i;
        i.severity = sev;
        i.type_id  = type_id;
        i.rule     = rule;
        i.message  = msg;
        issues.push_back(i);
    };

    const auto Error = MetadataIssue::Severity::Error;
    const auto Note  = MetadataIssue::Severity::Note;

    // ---- V1 / V2：type_id 与 NodeInfo.id 的一致性 ----
    if (info.id.empty()) {
        add(Error, "V1", "NodeInfo.id is empty (registered as '" + type_id + "')");
    } else if (info.id != type_id) {
        add(Error, "V2", "NodeInfo.id ('" + info.id + "') != registered type_id ('" + type_id +
                         "'); node->info().id will not match what was registered");
    }

    // ---- V3：显示名 ----
    if (info.name.empty()) {
        add(Error, "V3", "NodeInfo.name is empty; the editor will show a blank entry");
    }

    // ---- V4/V7：输入端口 id 非空且不重复 ----
    {
        HashMap<String, int> seen;
        for (const auto& p : info.inputs) {
            if (p.id.empty()) {
                add(Error, "V4", "input port has an empty id");
                continue;
            }
            if (++seen[p.id] == 2) {
                add(Error, "V7", "duplicate input port id '" + p.id +
                                 "'; set_input() keys on the id, so the two would overwrite each other");
            }
        }
    }

    // ---- V5/V8：输出端口 id 非空且不重复 ----
    {
        HashMap<String, int> seen;
        for (const auto& p : info.outputs) {
            if (p.id.empty()) {
                add(Error, "V5", "output port has an empty id");
                continue;
            }
            if (++seen[p.id] == 2) {
                add(Error, "V8", "duplicate output port id '" + p.id + "'");
            }
        }
    }

    // ---- V6/V9/V10/V11/V12/V13/V14：参数 ----
    {
        HashMap<String, int> seen;
        for (const auto& p : info.params) {
            if (p.id.empty()) {
                add(Error, "V6", "parameter has an empty id");
                continue;
            }
            if (++seen[p.id] == 2) {
                add(Error, "V9", "duplicate parameter id '" + p.id +
                                 "'; params_ is a map, so one default silently wins");
            }

            // V10：默认值类型必须和声明的类型一致
            if (!p.default_value.is_none() &&
                p.type != DataType::Any &&
                p.default_value.type() != p.type) {
                add(Error, "V10", "param '" + p.id + "' declares type " +
                                  data_type_name(p.type) + " but its default value is " +
                                  data_type_name(p.default_value.type()));
            }

            // V11 / V12：min/max 必须是数值，且 min <= max
            const bool has_min = !p.min_value.is_none();
            const bool has_max = !p.max_value.is_none();
            if (has_min && !p.min_value.is_number()) {
                add(Error, "V11", "param '" + p.id + "' has a non-numeric min_value (" +
                                  data_type_name(p.min_value.type()) + ")");
            }
            if (has_max && !p.max_value.is_number()) {
                add(Error, "V11", "param '" + p.id + "' has a non-numeric max_value (" +
                                  data_type_name(p.max_value.type()) + ")");
            }
            if (has_min && has_max && p.min_value.is_number() && p.max_value.is_number() &&
                p.min_value.as_number() > p.max_value.as_number()) {
                add(Error, "V12", "param '" + p.id + "' has min_value (" +
                                  std::to_string(p.min_value.as_number()) + ") > max_value (" +
                                  std::to_string(p.max_value.as_number()) + ")");
            }

            // V13：选项不能有空串
            for (size_t i = 0; i < p.options.size(); ++i) {
                if (p.options[i].empty()) {
                    add(Error, "V13", "param '" + p.id + "' has an empty option at index " +
                                      std::to_string(i));
                    break;
                }
            }

            // V14：type 与 options 自相矛盾。
            // 上游有若干点云/OCR 节点写成 type=Number + options=[中文标签]，
            // 前端不会把 Number+options 渲染成下拉框（转了会把存的整数下标
            // 悄悄换成字符串），所以这些参数在编辑器里没有像样的控件。
            if (!p.options.empty() && p.type != DataType::String && p.type != DataType::Any) {
                add(Error, "V14", "param '" + p.id + "' declares type " + data_type_name(p.type) +
                                  " but also carries " + std::to_string(p.options.size()) +
                                  " string options; the editor cannot render that as a control");
            }
        }
    }

    // ---- V15：描述字段完整度（Note 级，上游绝大多数没填，别当缺陷）----
    {
        Vector<String> missing;
        if (info.category.empty())    missing.push_back("category");
        if (info.description.empty()) missing.push_back("description");
        if (info.version.empty())     missing.push_back("version");
        if (info.author.empty())      missing.push_back("author");
        if (!missing.empty()) {
            String joined;
            for (size_t i = 0; i < missing.size(); ++i) {
                if (i) joined += ", ";
                joined += missing[i];
            }
            add(Note, "V15", "NodeInfo is missing documentation fields: " + joined);
        }
    }

    return issues;
}

String summarize_metadata_issues(const Vector<MetadataIssue>& issues) {
    size_t errors = 0, notes = 0;
    for (const auto& i : issues) {
        if (i.severity == MetadataIssue::Severity::Error) ++errors; else ++notes;
    }
    return std::to_string(issues.size()) + " metadata issue(s): " +
           std::to_string(errors) + " error, " + std::to_string(notes) + " note";
}

// ============================================================================
// NodeFactory implementation
// ============================================================================

NodeFactory& NodeFactory::instance() {
    static NodeFactory factory;
    return factory;
}

void NodeFactory::register_node(const String& type_id, Creator creator, const NodeInfo& info,
                                const RegistrationSite& site, RegistrationPolicy policy,
                                bool strict) {
    RegistrationSite s = site;
    if (s.origin.empty()) s.origin = current_origin();

    if (type_id.empty()) {
        OVF_ERROR() << "NodeFactory::register_node: refusing to register an empty type_id"
                    << " (from " << s.str() << ")";
        return;
    }
    if (!creator) {
        OVF_ERROR() << "NodeFactory::register_node: refusing to register '" << type_id
                    << "' with an empty Creator (from " << s.str() << ")";
        return;
    }

    // 元数据校验：只报告，绝不拒绝注册。
    // Error 级打 WARN，Note 级打 DEBUG（否则上游 501 个算子的 author/version
    // 缺失会刷满整个启动日志）。
    Vector<MetadataIssue> issues = validate_node_info(type_id, info);
    for (const auto& iss : issues) {
        if (iss.severity == MetadataIssue::Severity::Error) {
            OVF_WARN() << "[" << iss.rule << "] " << iss.message << " (from " << s.str() << ")";
        } else {
            OVF_DEBUG() << "[" << iss.rule << "] " << iss.message << " (from " << s.str() << ")";
        }
    }

    bool      duplicate = false;
    bool      replaced  = false;
    RegistrationSite previous;
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = creators_.find(type_id);
        if (it != creators_.end()) {
            previous = sites_.count(type_id) ? sites_.at(type_id) : RegistrationSite{};

            if (policy != RegistrationPolicy::Replace) {
                // 首次写入者获胜。静态初始化顺序未定义，"后来者覆盖"等于把
                // 结果交给运气；不覆盖至少是确定的，而且冲突被保留下来可查。
                RegistrationConflict c;
                c.type_id = type_id;
                c.winner  = previous;
                c.loser   = s;
                conflicts_.push_back(c);
                duplicate = true;
            } else {
                replaced = true;
            }
        }

        if (!duplicate) {
            creators_[type_id] = std::move(creator);
            infos_[type_id]    = info;
            sites_[type_id]    = s;
            strict_[type_id]   = strict;
            issues_.insert(issues_.end(), issues.begin(), issues.end());
        }
    }
    // 锁已释放再打日志：Logger 持自己的锁调 sink，别把两把锁叠在一起

    if (duplicate) {
        OVF_ERROR() << "Duplicate node type '" << type_id << "'. "
                    << "Kept: " << previous.str() << ". "
                    << "Ignored: " << s.str() << ". "
                    << "First registration wins; this one has NO effect. "
                    << "If the override is intentional, use RegistrationPolicy::Replace.";
        return;
    }
    if (replaced) {
        OVF_WARN() << "Node type '" << type_id << "' explicitly replaced (RegistrationPolicy::Replace). "
                   << "Was: " << previous.str() << ". Now: " << s.str();
    }
}

INode::Ptr NodeFactory::create(const String& type_id, const String& instance_id) {
    Creator creator;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = creators_.find(type_id);
        if (it == creators_.end()) {
            // 落到锁外再报，避免持锁进 Logger
        } else {
            creator = it->second;
        }
    }

    if (!creator) {
        OVF_ERROR() << "Node type not found: " << type_id;
        return nullptr;
    }

    // Creator 是我们自己注册的 lambda，正常不会抛；但如果某个节点类的构造
    // 函数抛了异常，这里不该让整个进程 std::terminate。交给上层的 try/catch
    // （flow.cpp 的 execute_node）去处理更好，所以这里只做兜底并给出可读信息。
    try {
        return creator(instance_id);
    } catch (const std::exception& e) {
        OVF_ERROR() << "Constructing node '" << type_id << "' (instance '" << instance_id
                    << "') threw: " << e.what();
        return nullptr;
    } catch (...) {
        OVF_ERROR() << "Constructing node '" << type_id << "' (instance '" << instance_id
                    << "') threw an unknown exception";
        return nullptr;
    }
}

const NodeInfo* NodeFactory::get_info(const String& type_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = infos_.find(type_id);
    return it != infos_.end() ? &it->second : nullptr;
}

Vector<String> NodeFactory::get_all_types() const {
    Vector<String> types;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        types.reserve(creators_.size());
        for (const auto& pair : creators_) {
            types.push_back(pair.first);
        }
    }
    // 排序：unordered_map 的遍历顺序每次都可能不同，不排序的话
    // /api/nodes 每次刷新顺序都变、体检报告无法做 diff。
    std::sort(types.begin(), types.end());
    return types;
}

bool NodeFactory::has_type(const String& type_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return creators_.find(type_id) != creators_.end();
}

// ----------------------------------------------------------------------------
// 注册加固新增的查询接口
// ----------------------------------------------------------------------------

size_t NodeFactory::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return creators_.size();
}

Vector<RegistrationConflict> NodeFactory::conflicts() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return conflicts_;
}

bool NodeFactory::has_conflicts() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !conflicts_.empty();
}

RegistrationSite NodeFactory::origin_of(const String& type_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sites_.find(type_id);
    return it != sites_.end() ? it->second : RegistrationSite{};
}

Vector<MetadataIssue> NodeFactory::metadata_issues() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return issues_;
}

Vector<String> NodeFactory::strict_types() const {
    Vector<String> out;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& pair : strict_) {
        if (pair.second) out.push_back(pair.first);
    }
    std::sort(out.begin(), out.end());
    return out;
}

void NodeFactory::set_origin(const String& origin) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_origin_ = origin;
}

String NodeFactory::current_origin() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_origin_;
}

void NodeFactory::assert_no_strict_violations() const {
    Vector<String>    strict;
    Vector<MetadataIssue> bad;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& pair : strict_) {
            if (pair.second) strict.push_back(pair.first);
        }
        for (const auto& iss : issues_) {
            if (iss.severity != MetadataIssue::Severity::Error) continue;
            for (const auto& s : strict) {
                if (s == iss.type_id) { bad.push_back(iss); break; }
            }
        }
    }
    if (bad.empty()) return;

    String msg = "Strict node registration (OVF_REGISTER_NODE_STRICT) found " +
                 std::to_string(bad.size()) + " metadata problem(s):";
    for (const auto& iss : bad) {
        msg += "\n  [" + iss.rule + "] " + iss.type_id + ": " + iss.message;
    }
    // 抛而不是 abort：这样调用方能加自己的上下文，用户也能看到完整清单
    throw Exception(ErrorCode::InvalidParameter, msg);
}

void NodeFactory::log_summary() const {
    size_t n = 0, conflicts = 0, errors = 0, notes = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        n         = creators_.size();
        conflicts = conflicts_.size();
        for (const auto& i : issues_) {
            if (i.severity == MetadataIssue::Severity::Error) ++errors; else ++notes;
        }
    }

    OVF_INFO() << "NodeFactory: " << n << " node type(s) registered, "
               << conflicts << " duplicate conflict(s), "
               << errors << " metadata error(s), " << notes << " metadata note(s)";

    if (conflicts) {
        OVF_WARN() << "NodeFactory has duplicate registrations - one node type silently lost per conflict."
                   << " Run ovf-node-audit --metadata-only for the full list.";
    }
}

// ============================================================================
// INode implementation
// ============================================================================

INode::INode(const String& instance_id, const NodeInfo& info)
    : instance_id_(instance_id)
    , info_(info) {
    // 初始化默认参数值
    for (const auto& param : info.params) {
        params_.set(param.id, param.default_value);
    }
}

void INode::set_param(const String& key, const Data& value) {
    params_.set(key, value);
}

Data INode::get_param(const String& key, const Data& default_val) const {
    return params_.get(key, default_val);
}

void INode::set_input(const String& port_id, const Data& data) {
    inputs_[port_id] = data;
}

Data INode::get_input(const String& port_id) const {
    auto it = inputs_.find(port_id);
    if (it != inputs_.end()) {
        return it->second;
    }
    
    // 尝试从端口定义获取默认值
    for (const auto& port : info_.inputs) {
        if (port.id == port_id) {
            return port.default_value;
        }
    }
    
    return Data{};
}

bool INode::has_input(const String& port_id) const {
    return inputs_.find(port_id) != inputs_.end();
}

void INode::set_output(const String& port_id, const Data& data) {
    outputs_[port_id] = data;
    
    // 传播数据到连接的下游节点
    auto it = connections_.find(port_id);
    if (it != connections_.end()) {
        for (const auto& conn : it->second) {
            auto target = conn.target.lock();
            if (target) {
                target->set_input(conn.target_port, data);
            }
        }
    }
}

Data INode::get_output(const String& port_id) const {
    auto it = outputs_.find(port_id);
    return it != outputs_.end() ? it->second : Data{};
}

void INode::connect_output(const String& port_id, Ptr target, const String& target_port) {
    Connection conn;
    conn.target = target;
    conn.target_port = target_port;
    connections_[port_id].push_back(conn);
}

void INode::disconnect_output(const String& port_id, Ptr target) {
    auto it = connections_.find(port_id);
    if (it == connections_.end()) return;
    
    if (target) {
        // 移除特定连接
        auto& conns = it->second;
        conns.erase(std::remove_if(conns.begin(), conns.end(),
            [&target](const Connection& c) {
                return c.target.lock() == target;
            }), conns.end());
    } else {
        // 移除所有连接
        connections_.erase(port_id);
    }
}

void INode::disconnect_all() {
    connections_.clear();
}

Result<void> INode::reset() {
    inputs_.clear();
    outputs_.clear();
    clear_error();
    state_ = NodeState::Idle;
    return Result<void>::success();
}

Result<void> INode::validate_inputs() const {
    for (const auto& port : info_.inputs) {
        if (port.required) {
            if (!has_input(port.id) && get_input(port.id).is_none()) {
                return Result<void>::failure(
                    ErrorCode::InvalidParameter,
                    "Required input port '" + port.name + "' is missing"
                );
            }
        }
    }
    return Result<void>::success();
}

void INode::set_error(const String& message) {
    error_message_ = message;
    state_ = NodeState::Failed;
}

void INode::clear_error() {
    error_message_.clear();
}

void INode::record_execute_time(uint64_t microseconds) {
    last_execute_time_ = microseconds;
}

} // namespace ovf