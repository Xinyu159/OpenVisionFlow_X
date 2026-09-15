/**
 * @file node.cpp
 * @brief 节点基类实现
 */

#include "ovf/core/node.h"
#include "ovf/core/flow.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>

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

// ============================================================================
// 写算子的安全带
//
// 全部是新增 API。501 个老算子继续用 get_input/get_param，一行没动。
// ============================================================================

namespace {

/// 在节点声明里按 id 找输入端口；找不到返回 nullptr（= 端口名拼错了）
const DataPort* find_input_port(const NodeInfo& info, const String& id) {
    for (const auto& p : info.inputs) {
        if (p.id == id) return &p;
    }
    return nullptr;
}

/// 在节点声明里按 id 找参数定义；找不到返回 nullptr
const ParamDef* find_param_def(const NodeInfo& info, const String& id) {
    for (const auto& p : info.params) {
        if (p.id == id) return &p;
    }
    return nullptr;
}

/// 把 id 列表拼成 "a, b, c" —— 报错时告诉对方**到底有哪些**，省一轮翻代码
String join_ids(const Vector<String>& ids) {
    if (ids.empty()) return "(none)";
    String s;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) s += ", ";
        s += ids[i];
    }
    return s;
}

String join_input_ids(const Vector<DataPort>& ports) {
    Vector<String> ids;
    ids.reserve(ports.size());
    for (const auto& p : ports) ids.push_back(p.id);
    return join_ids(ids);
}

String join_param_ids(const Vector<ParamDef>& params) {
    Vector<String> ids;
    ids.reserve(params.size());
    for (const auto& p : params) ids.push_back(p.id);
    return join_ids(ids);
}

/// "Node 'blur_1'" —— 每条错误消息都必须带它，否则同名节点一多就不知道是谁
String who_of(const String& instance_id) {
    return "Node '" + instance_id + "'";
}

/// 数字的可读形式：走 stream 默认格式，不会打出 "0.000000" 这种噪声
String num_str(double v) {
    std::ostringstream oss;
    oss << v;
    return oss.str();
}

/// 按给定边界夹取（has_lo/has_hi 为 false 表示那一侧不限）
double clamp_value(double v, bool has_lo, double lo, bool has_hi, double hi) {
    if (has_lo && v < lo) return lo;
    if (has_hi && v > hi) return hi;
    return v;
}

/**
 * @brief 取值 + 类型检查，类型不符就报「期望 X，实际是 Y」
 *
 * get 故意返回**可变引用**：这样上面能用 move 把它搬进 Result，
 * 而不是让整幅图像在 Result 构造里再拷一次。
 */
template <typename T, typename Check, typename Get>
Result<T> extract_typed(Data& data, const String& who, const String& port,
                        const char* expected, Check check, Get get) {
    if (!check(data)) {
        return Result<T>::failure(
            ErrorCode::InvalidData,
            who + ": input '" + port + "' expects " + expected +
            ", got " + data_type_name(data.type()));
    }
    return Result<T>::success(std::move(get(data)));
}

} // namespace

Result<Data> INode::in_data(const String& port) {
    const String who = who_of(instance_id_);

    auto it = inputs_.find(port);
    if (it != inputs_.end() && !it->second.is_none()) {
        return Result<Data>::success(it->second);
    }

    // 拿不到数据的三种原因，修法完全不同，所以分开报：
    const DataPort* def = find_input_port(info_, port);
    if (!def) {
        // ① 端口压根没声明过 —— 几乎总是算子代码里把端口名写错了
        return Result<Data>::failure(
            ErrorCode::NotFound,
            who + ": no input port named '" + port + "'. Declared inputs: " +
            join_input_ids(info_.inputs));
    }
    if (!def->default_value.is_none()) {
        // ② 没连线，但端口声明了默认值 —— 与 get_input() 的既有语义保持一致
        return Result<Data>::success(def->default_value);
    }
    // ③ 声明了、没默认值，但拿不到数据
    //   （没连线，或者连了而上游这一轮什么都没产出 —— 两种都可能）
    return Result<Data>::failure(
        ErrorCode::InvalidData,
        who + ": input port '" + port + "' has no data"
        " (not connected, or the upstream node produced nothing)");
}

Result<ImageData> INode::in_image(const String& port) {
    auto data = in_data(port);
    if (!data) return Result<ImageData>::failure(data.code(), data.message());
    return extract_typed<ImageData>(*data, who_of(instance_id_), port, "Image",
        [](const Data& d) { return d.is_image(); },
        [](Data& d) -> ImageData& { return d.as_image(); });
}

Result<Region> INode::in_region(const String& port) {
    auto data = in_data(port);
    if (!data) return Result<Region>::failure(data.code(), data.message());
    return extract_typed<Region>(*data, who_of(instance_id_), port, "Region",
        [](const Data& d) { return d.is_region(); },
        [](Data& d) -> Region& { return const_cast<Region&>(d.as_region()); });
}

Result<PointCloudData> INode::in_pointcloud(const String& port) {
    auto data = in_data(port);
    if (!data) return Result<PointCloudData>::failure(data.code(), data.message());
    return extract_typed<PointCloudData>(*data, who_of(instance_id_), port, "PointCloud",
        [](const Data& d) { return d.is_pointcloud(); },
        [](Data& d) -> PointCloudData& { return const_cast<PointCloudData&>(d.as_pointcloud()); });
}

Result<DepthImageData> INode::in_depth_image(const String& port) {
    auto data = in_data(port);
    if (!data) return Result<DepthImageData>::failure(data.code(), data.message());
    return extract_typed<DepthImageData>(*data, who_of(instance_id_), port, "DepthImage",
        [](const Data& d) { return d.is_depth_image(); },
        [](Data& d) -> DepthImageData& { return const_cast<DepthImageData&>(d.as_depth_image()); });
}

Result<double> INode::in_number(const String& port) {
    auto data = in_data(port);
    if (!data) return Result<double>::failure(data.code(), data.message());
    return extract_typed<double>(*data, who_of(instance_id_), port, "Number",
        [](const Data& d) { return d.is_number(); },
        [](Data& d) { return d.as_number(); });
}

// 说明：Region / PointCloudData / DepthImageData 只有 const 版 as_xxx()，
// 上面用 const_cast 把引用取出来，纯粹是为了让 extract_typed 能 move 进 Result。
// 安全前提有两条，缺一不可：
//   1. extract_typed **先做类型检查、通过了才调 get()**，所以永远不会碰到
//      那些 as_xxx() 在类型不符时返回的 static 空对象（否则会把它 move 空）；
//   2. 那个 Data 是 in_data() 刚返回的**局部副本**，改它不动上游任何数据。

Result<Data> INode::read_param(const String& key, DataType expected) {
    const String who = who_of(instance_id_);
    const ParamDef* def = find_param_def(info_, key);

    if (!def && !params_.has(key)) {
        // 参数名拼错时，老 API 会静默返回 Data{}，等于拿 0 去算 —— 这是最难查的一类
        return Result<Data>::failure(
            ErrorCode::NotFound,
            who + ": no parameter named '" + key + "'. Declared params: " +
            join_param_ids(info_.params));
    }

    // def 存在时 params_ 在构造函数里就被默认值填过了，所以这里拿到的
    // 要么是外面设的值，要么是声明的默认值。
    const Data raw = params_.has(key) ? params_.get(key) : def->default_value;

    if (raw.is_none()) {
        return Result<Data>::failure(
            ErrorCode::InvalidParameter,
            who + ": parameter '" + key + "' is not set and has no default value");
    }
    if (expected != DataType::Any) {
        if (def && def->type != DataType::Any && def->type != expected) {
            return Result<Data>::failure(
                ErrorCode::InvalidParameter,
                who + ": parameter '" + key + "' is declared as " + data_type_name(def->type) +
                " but read as " + data_type_name(expected));
        }
        if (raw.type() != expected) {
            return Result<Data>::failure(
                ErrorCode::InvalidParameter,
                who + ": parameter '" + key + "' expects " + data_type_name(expected) +
                ", got " + data_type_name(raw.type()));
        }
    }
    return Result<Data>::success(raw);
}

Result<double> INode::p_num(const String& key) {
    const String who = who_of(instance_id_);
    auto raw = read_param(key, DataType::Number);
    if (!raw) return Result<double>::failure(raw.code(), raw.message());

    const double v = raw->as_number();
    if (std::isnan(v)) {
        return Result<double>::failure(
            ErrorCode::InvalidParameter, who + ": parameter '" + key + "' is NaN");
    }

    const ParamDef* def = find_param_def(info_, key);
    const bool has_lo = def && def->min_value.is_number();
    const bool has_hi = def && def->max_value.is_number();
    const double lo = has_lo ? def->min_value.as_number() : 0.0;
    const double hi = has_hi ? def->max_value.as_number() : 0.0;

    const double c = clamp_value(v, has_lo, lo, has_hi, hi);
    if (c != v) {
        warn_once("param:" + key,
            "parameter '" + key + "' = " + num_str(v) + " is outside the declared range [" +
            (has_lo ? num_str(lo) : String("-inf")) + ", " +
            (has_hi ? num_str(hi) : String("+inf")) + "]; clamped to " + num_str(c));
    }
    return Result<double>::success(c);
}

Result<int32_t> INode::p_int(const String& key) {
    const String who = who_of(instance_id_);
    auto raw = read_param(key, DataType::Number);
    if (!raw) return Result<int32_t>::failure(raw.code(), raw.message());

    const double v = raw->as_number();
    if (std::isnan(v)) {
        return Result<int32_t>::failure(
            ErrorCode::InvalidParameter, who + ": parameter '" + key + "' is NaN");
    }

    const ParamDef* def = find_param_def(info_, key);
    const bool has_lo = def && def->min_value.is_number();
    const bool has_hi = def && def->max_value.is_number();
    const double lo = has_lo ? def->min_value.as_number() : 0.0;
    const double hi = has_hi ? def->max_value.as_number() : 0.0;

    // 先按范围夹、再取整。反过来的话 2.7 会先被截成 2 再判范围，边界上差一格。
    const double c = clamp_value(v, has_lo, lo, has_hi, hi);
    if (c != v) {
        warn_once("param:" + key,
            "parameter '" + key + "' = " + num_str(v) + " is outside the declared range [" +
            (has_lo ? num_str(lo) : String("-inf")) + ", " +
            (has_hi ? num_str(hi) : String("+inf")) + "]; clamped to " + num_str(c));
    }
    return Result<int32_t>::success(static_cast<int32_t>(c));
}

Result<bool> INode::p_bool(const String& key) {
    auto raw = read_param(key, DataType::Boolean);
    if (!raw) return Result<bool>::failure(raw.code(), raw.message());
    return Result<bool>::success(raw->as_bool());
}

Result<String> INode::p_str(const String& key) {
    const String who = who_of(instance_id_);
    auto raw = read_param(key, DataType::String);
    if (!raw) return Result<String>::failure(raw.code(), raw.message());

    const String v = raw->as_string();

    // 声明了 options 就是枚举。取值不在里面**不能**凑合跑 ——
    // 悄悄退回第一个选项会让流程走进完全不同的分支，比直接失败危险得多。
    const ParamDef* def = find_param_def(info_, key);
    if (def && !def->options.empty() &&
        std::find(def->options.begin(), def->options.end(), v) == def->options.end()) {
        return Result<String>::failure(
            ErrorCode::InvalidParameter,
            who + ": parameter '" + key + "' = '" + v + "' is not one of: " +
            join_ids(def->options));
    }
    return Result<String>::success(v);
}

Result<double> INode::p_num_in(const String& key, double lo, double hi) {
    if (!(lo <= hi)) {
        // 边界写反是**算子代码**的 bug，不是配置问题 —— 直接报出来
        return Result<double>::failure(
            ErrorCode::InvalidParameter,
            who_of(instance_id_) + ": p_num_in('" + key + "') called with an inverted range [" +
            num_str(lo) + ", " + num_str(hi) + "]");
    }
    auto raw = read_param(key, DataType::Number);
    if (!raw) return Result<double>::failure(raw.code(), raw.message());

    const double v = raw->as_number();
    const double c = clamp_value(v, true, lo, true, hi);
    if (c != v) {
        warn_once("param:" + key,
            "parameter '" + key + "' = " + num_str(v) + " is outside the allowed range [" +
            num_str(lo) + ", " + num_str(hi) + "]; clamped to " + num_str(c));
    }
    return Result<double>::success(c);
}

Result<int32_t> INode::p_int_in(const String& key, int32_t lo, int32_t hi) {
    if (!(lo <= hi)) {
        return Result<int32_t>::failure(
            ErrorCode::InvalidParameter,
            who_of(instance_id_) + ": p_int_in('" + key + "') called with an inverted range [" +
            std::to_string(lo) + ", " + std::to_string(hi) + "]");
    }
    auto raw = read_param(key, DataType::Number);
    if (!raw) return Result<int32_t>::failure(raw.code(), raw.message());

    const double v = raw->as_number();
    const double c = clamp_value(v, true, static_cast<double>(lo), true, static_cast<double>(hi));
    if (c != v) {
        warn_once("param:" + key,
            "parameter '" + key + "' = " + num_str(v) + " is outside the allowed range [" +
            std::to_string(lo) + ", " + std::to_string(hi) + "]; clamped to " + num_str(c));
    }
    return Result<int32_t>::success(static_cast<int32_t>(c));
}

bool INode::step_ok(FlowContext& context, uint64_t iteration, uint64_t max_iterations) {
    if (context.is_stopped()) {
        // 停止请求是正常操作，不是异常 —— 不打警告，免得刷屏
        return false;
    }
    if (max_iterations > 0 && iteration >= max_iterations) {
        warn_once("loop-budget:" + std::to_string(max_iterations),
            "loop hit its budget of " + std::to_string(max_iterations) +
            " iteration(s) and bailed out early; the result may be incomplete");
        return false;
    }
    return true;
}

void INode::warn_once(const String& key, const String& message) {
    if (!warned_params_.insert(key).second) {
        return;   // 这个 key 已经警告过一次了
    }
    OVF_WARN() << "[node " << instance_id_ << " (" << info_.id << ")] " << message;
}

} // namespace ovf