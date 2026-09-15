/**
 * @file flow.cpp
 * @brief 流程引擎实现
 */

#include "ovf/core/flow.h"
#include <chrono>
#include <algorithm>
#include <exception>
#include <fstream>
#include <set>
#include <sstream>

// nlohmann_json 头文件 (内置简化版本)
#include "nlohmann/json.hpp"

namespace ovf {

using json = nlohmann::json;

// ============== FlowContext ==============

FlowContext::FlowContext() {}

FlowContext::~FlowContext() {}

void FlowContext::pause() {
    paused_ = true;
}

void FlowContext::resume() {
    paused_ = false;
}

void FlowContext::stop() {
    stopped_ = true;
}
void FlowContext::reset() {
    paused_ = false;
    stopped_ = false;
}

void FlowContext::set_variable(const String& key, const Data& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    variables_[key] = value;
}

Data FlowContext::get_variable(const String& key, const Data& default_val) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = variables_.find(key);
    return it != variables_.end() ? it->second : default_val;
}

// ============== FlowEngine ==============

FlowEngine::FlowEngine() {}

FlowEngine::~FlowEngine() {}

Result<void> FlowEngine::load_flow(const FlowDef& flow_def) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    clear();
    flow_def_ = flow_def;
    
    // 创建所有节点
    for (const auto& node_inst : flow_def.nodes) {
        auto node = NodeFactory::instance().create(node_inst.type_id, node_inst.id);
        if (!node) {
            return Result<void>::failure(
                ErrorCode::NodeNotFound,
                "Node type '" + node_inst.type_id + "' not found"
            );
        }
        
        node->set_enabled(node_inst.enabled);

        // 设置参数 - 将节点实例的参数传递给节点对象
        const auto& all_params = node_inst.params.get_all();
        for (const auto& param_pair : all_params) {
            node->set_param(param_pair.first, param_pair.second);
        }

        nodes_[node_inst.id] = node;
    }
    
    // 建立连接关系
    for (const auto& node_inst : flow_def.nodes) {
        auto node = nodes_[node_inst.id];
        if (!node) continue;
        
        for (const auto& conn_pair : node_inst.input_connections) {
            const String& input_port = conn_pair.first;
            const auto& conn = conn_pair.second;
            
            auto source_node = nodes_[conn.source_node_id];
            if (!source_node) {
                return Result<void>::failure(
                    ErrorCode::NodeNotFound,
                    "Source node '" + conn.source_node_id + "' not found"
                );
            }
            
            source_node->connect_output(conn.source_port, node, input_port);

            // 构建邻接表（依赖关系）。
            // 必须去重：一个节点的**每个输入端**都会走到这里，所以 image 和 mask
            // 都接同一个上游时，这里会push 两条相同的边 → 拓扑排序按 size() 算
            // 入度会得 2，而递减时最多只减 1 → 入度永远减不到 0 →
            // 一个完全合法的 DAG 被判成 "Flow has cyclic dependencies"。
            auto& deps = adjacency_list_[node_inst.id];
            if (std::find(deps.begin(), deps.end(), conn.source_node_id) == deps.end()) {
                deps.push_back(conn.source_node_id);
            }
        }
    }
    
    // 计算执行顺序
    auto order_result = topological_sort();
    if (order_result.is_failure()) {
        return Result<void>::failure(order_result.code(), order_result.message());
    }
    
    execution_order_ = order_result.value();
    
    return Result<void>::success();
}

// ── JSON ⇄ FlowDef 编解码辅助 ──────────────────────────────────────
// 注意：本仓库的 thirdparty/nlohmann/json.hpp 是自带的"简化版"实现，
// 对象的迭代器是 object_begin()/object_end()；begin()/end() 只对数组有效，
// 对对象调用会 check_type 抛异常。原先"简化版json不支持迭代器"的注释即由此而来。
namespace {

// JSON 标量 → Data。只处理 bool/number/string，其它（null/数组/对象）返回 false。
bool json_value_to_data(const json& v, Data& out) {
    if (v.is_boolean()) { out = Data(v.get_bool());    return true; }
    if (v.is_number())  { out = Data(v.get_double());  return true; }
    if (v.is_string())  { out = Data(v.get_string());  return true; }
    return false;
}

// 节点的 params 对象 → ParamSet。前端与后端两种写法都是 {参数id: 值} 扁平对象。
void parse_node_params(const json& params_j, ParamSet& out) {
    if (!params_j.is_object()) return;
    for (auto it = params_j.object_begin(); it != params_j.object_end(); ++it) {
        Data d;
        if (json_value_to_data(it->second, d)) {
            out.set(it->first, d);
        }
    }
}

// Data → JSON 标量。只写 bool/number/string；图像/区域等大对象不落盘（与 parse 对称）。
bool data_to_json_value(const Data& d, json& out) {
    if (d.is_bool())   { out = json(d.as_bool());    return true; }
    if (d.is_number()) { out = json(d.as_number());  return true; }
    if (d.is_string()) { out = json(d.as_string());  return true; }
    return false;
}

// 把端口"下标"解析成端口 id（前端的 connections[] 用的是下标，不是 id）。
// 下标越界或节点类型未注册时返回空串，由调用方决定是忽略还是报错。
String resolve_port_id(const String& type_id, bool is_input, int index) {
    const NodeInfo* info = NodeFactory::instance().get_info(type_id);
    if (!info) return "";
    const Vector<DataPort>& ports = is_input ? info->inputs : info->outputs;
    if (index < 0 || static_cast<size_t>(index) >= ports.size()) return "";
    return ports[index].id;
}

} // namespace

Result<void> FlowEngine::load_from_file(const String& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return Result<void>::failure(
            ErrorCode::FileOpenFailed,
            "Failed to open file: " + filepath
        );
    }
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    return load_from_json(content);
}

Result<void> FlowEngine::load_from_json(const String& json_text) {
    try {
        json j = json::parse(json_text);

        FlowDef flow_def;
        // id/name 原先用 get_string()，字段缺失会抛异常变成"JSON parse error"，
        // 对前端导出的文件太苛刻（前端的 top-level id 是自动生成的、可能没有 name）。改为可选。
        flow_def.id = j.value("id", std::string(""));
        flow_def.name = j.value("name", std::string(""));
        flow_def.description = j.value("description", std::string(""));
        // 版本号：前端放在 metadata.version 里，后端放在顶层 version
        flow_def.version = j.value("version", std::string("1.0"));
        if (flow_def.version == "1.0" && j.contains("metadata") && j["metadata"].is_object()) {
            flow_def.version = j["metadata"].value("version", std::string("1.0"));
        }

        // 解析节点
        if (j.contains("nodes") && j["nodes"].is_array()) {
            const json& nodes_j = j["nodes"];
            for (size_t i = 0; i < nodes_j.size(); ++i) {
                const json& node_j = nodes_j[i];
                FlowDef::NodeInstance node_inst;
                node_inst.id = node_j.value("id", std::string(""));
                // type_id 是后端字段名，type 是前端字段名，两种都认
                node_inst.type_id = node_j.value("type_id", std::string(""));
                if (node_inst.type_id.empty()) {
                    node_inst.type_id = node_j.value("type", std::string(""));
                }
                node_inst.name = node_j.value("name", std::string(""));
                node_inst.x = node_j.value("x", 0);
                node_inst.y = node_j.value("y", 0);
                // enabled 是后端字段，disabled 是前端字段（语义相反）
                if (node_j.contains("disabled")) {
                    node_inst.enabled = !node_j.value("disabled", false);
                } else {
                    node_inst.enabled = node_j.value("enabled", true);
                }

                // 解析参数：{参数id: 值} 扁平对象 → ParamSet
                if (node_j.contains("params")) {
                    parse_node_params(node_j["params"], node_inst.params);
                }

                // 解析连接（后端原生格式）：inputs 是 {输入端口id: {source_node, source_port}}
                if (node_j.contains("inputs") && node_j["inputs"].is_object()) {
                    const json& inputs_j = node_j["inputs"];
                    for (auto it = inputs_j.object_begin(); it != inputs_j.object_end(); ++it) {
                        const json& conn_j = it->second;
                        if (!conn_j.is_object()) continue;
                        FlowDef::NodeInstance::InputConnection conn;
                        conn.source_node_id = conn_j.value("source_node", std::string(""));
                        conn.source_port    = conn_j.value("source_port", std::string(""));
                        if (!conn.source_node_id.empty()) {
                            node_inst.input_connections[it->first] = conn;
                        }
                    }
                }

                flow_def.nodes.push_back(node_inst);
            }
        }

        // 解析连接（前端格式）：顶层 connections[] 数组，端口用"下标"而非 id。
        //   {"fromNode": <节点id>, "fromPort": <输出端口下标>, "toNode": ..., "toPort": ...}
        // 需要借 NodeFactory 里该类型的 NodeInfo 把下标翻回端口 id。
        if (j.contains("connections") && j["connections"].is_array()) {
            // 建 id → 节点下标 的索引，便于按 id 找节点
            HashMap<String, size_t> id_to_index;
            for (size_t i = 0; i < flow_def.nodes.size(); ++i) {
                id_to_index[flow_def.nodes[i].id] = i;
            }

            const json& conns_j = j["connections"];
            for (size_t i = 0; i < conns_j.size(); ++i) {
                const json& c = conns_j[i];
                if (!c.is_object()) continue;

                const String from_node = c.value("fromNode", std::string(""));
                const String to_node   = c.value("toNode",   std::string(""));
                auto to_it = id_to_index.find(to_node);
                if (to_it == id_to_index.end()) continue;   // 目标节点不存在，忽略这条连线

                FlowDef::NodeInstance& target = flow_def.nodes[to_it->second];

                // fromPort / toPort：优先当下标解析，解析不了再当端口 id 用
                String from_port, to_port;
                auto from_idx_it = id_to_index.find(from_node);
                const String from_type = (from_idx_it != id_to_index.end())
                                       ? flow_def.nodes[from_idx_it->second].type_id : "";

                if (c.contains("fromPort") && c["fromPort"].is_number()) {
                    from_port = resolve_port_id(from_type, false, c["fromPort"].get_int());
                } else if (c.contains("fromPort") && c["fromPort"].is_string()) {
                    from_port = c["fromPort"].get_string();
                }
                if (c.contains("toPort") && c["toPort"].is_number()) {
                    to_port = resolve_port_id(target.type_id, true, c["toPort"].get_int());
                } else if (c.contains("toPort") && c["toPort"].is_string()) {
                    to_port = c["toPort"].get_string();
                }

                if (from_port.empty() || to_port.empty()) continue;   // 端口解析不出来，忽略

                FlowDef::NodeInstance::InputConnection conn;
                conn.source_node_id = from_node;
                conn.source_port    = from_port;
                target.input_connections[to_port] = conn;
            }
        }

        flow_def.created_time = j.value("created_time", std::string(""));
        flow_def.modified_time = j.value("modified_time", std::string(""));
        flow_def.author = j.value("author", std::string(""));
        
        return load_flow(flow_def);
    } catch (const std::exception& e) {
        return Result<void>::failure(
            ErrorCode::FileParseFailed,
            "JSON parse error: " + String(e.what())
        );
    }
}

Result<void> FlowEngine::save_to_file(const String& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return Result<void>::failure(
            ErrorCode::FileWriteFailed,
            "Failed to open file for writing: " + filepath
        );
    }
    
    json j;
    j["id"] = flow_def_.id;
    j["name"] = flow_def_.name;
    j["description"] = flow_def_.description;
    j["version"] = flow_def_.version;
    
    // 保存节点
    json nodes_j = json::array();
    for (const auto& node_inst : flow_def_.nodes) {
        json node_j;
        node_j["id"] = node_inst.id;
        node_j["type_id"] = node_inst.type_id;
        node_j["name"] = node_inst.name;
        node_j["x"] = node_inst.x;
        node_j["y"] = node_inst.y;
        node_j["enabled"] = node_inst.enabled;

        // 保存参数（原先硬编码成空对象，参数全丢）
        json params_j = json::object();
        for (auto it = node_inst.params.begin(); it != node_inst.params.end(); ++it) {
            json v;
            if (data_to_json_value(it->second, v)) {
                params_j[it->first] = v;
            }
        }
        node_j["params"] = params_j;

        // 保存连接
        json inputs_j = json::object();
        for (const auto& conn_pair : node_inst.input_connections) {
            json conn_j;
            conn_j["source_node"] = conn_pair.second.source_node_id;
            conn_j["source_port"] = conn_pair.second.source_port;
            inputs_j[conn_pair.first] = conn_j;
        }
        node_j["inputs"] = inputs_j;
        
        nodes_j.push_back(node_j);
    }
    j["nodes"] = nodes_j;
    
    j[std::string("created_time")] = flow_def_.created_time;
    j[std::string("modified_time")] = flow_def_.modified_time;
    j[std::string("author")] = flow_def_.author;
    
    file << j.dump(4);
    return Result<void>::success();
}

void FlowEngine::clear() {
    nodes_.clear();
    adjacency_list_.clear();
    execution_order_.clear();
    flow_def_ = FlowDef{};
}

INode::Ptr FlowEngine::get_node(const String& instance_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodes_.find(instance_id);
    return it != nodes_.end() ? it->second : nullptr;
}

Vector<INode::Ptr> FlowEngine::get_all_nodes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<INode::Ptr> result;
    for (const auto& pair : nodes_) {
        result.push_back(pair.second);
    }
    return result;
}

FlowResult FlowEngine::run(FlowContext& context) {
    // 第 2 层 try/catch：编排本身（快照、派发、暂停等待、状态回调）抛出的异常
    // 也要兜住，不能让它顺着 FlowRunner::run_thread 冒到线程入口去。
    try {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 先在本锁内拷一份快照，**出锁再执行**。
        // 不能全程持锁：算子的回调和 node_state_callback_ 会重入引擎
        // （get_node / step_next / set_param 都上锁）→ 持着 mutex_ 跑流程会自死锁。
        // shared_ptr 保证节点在快照之外仍然活着。
        ExecutionMode mode;
        Vector<String> order;
        HashMap<String, INode::Ptr> nodes;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            mode = execution_mode_;
            order = execution_order_;
            nodes = nodes_;
        }

        if (mode == ExecutionMode::Parallel) {
            return execute_parallel(context);
        }
        if (mode == ExecutionMode::DataDriven) {
            return execute_data_driven(context);
        }

        bool stopped_by_request = false;
        HashMap<String, uint64_t> node_times;

        for (const auto& node_id : order) {
            // 每轮开头先查停：停止也可能发生在上一轮的暂停等待里，
            // 检查放这里才能保证"叫停之后不会再多跑一个节点"。
            if (context.is_stopped()) {
                stopped_by_request = true;
                break;
            }

            // find 而不是 operator[]：后者查不到会**插入一个空条目**污染 nodes_，
            // 而 nodes_ 正是拓扑排序算入度的依据。
            auto it = nodes.find(node_id);
            if (it == nodes.end() || !it->second) {
                // 执行顺序是从 nodes_ 推出来的，这里查不到说明两者对不上。
                // 报出来，比静默少跑一个节点强。
                FlowResult broken = FlowResult::fail("Node not found: " + node_id, node_id);
                broken.node_times = node_times;
                return broken;
            }
            auto node = it->second;
            if (!node->is_enabled()) continue;

            FlowResult result;
            try {
                result = execute_node(node, context);
            } catch (const std::exception& e) {
                result = FlowResult::fail(
                    String("Node '") + node_id + "' threw an exception: " + e.what(), node_id);
                node->set_error(result.error_message);
            } catch (...) {
                result = FlowResult::fail(
                    "Node '" + node_id + "' threw an unknown exception", node_id);
                node->set_error(result.error_message);
            }

            // 把这一轮的耗时并进总表（成功路径原先整个丢掉）。
            for (const auto& t : result.node_times) {
                node_times[t.first] = t.second;
            }

            if (!result.success) {
                // 节点报错的同时又被叫停 —— 算取消，不算失败
                result.stopped = context.is_stopped();
                result.node_times = node_times;
                return result;
            }

            // 等待暂停恢复
            while (context.is_paused() && !context.is_stopped()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        // 循环自然走完（在最后一个节点之后才叫停）也要认出来
        if (context.is_stopped()) {
            stopped_by_request = true;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        FlowResult result = FlowResult::ok();
        result.total_time_us = total_time.count();
        result.node_times = node_times;

        if (stopped_by_request) {
            // 中途被叫停 = 没跑完。这里报成功，前端就会把整条流程画成全绿。
            result.success = false;
            result.stopped = true;
            result.error_message = "Flow execution stopped by request";
        }
        return result;
    } catch (const std::exception& e) {
        return FlowResult::fail(String("Flow execution failed: ") + e.what());
    } catch (...) {
        return FlowResult::fail("Flow execution failed: unknown exception");
    }
}

FlowResult FlowEngine::run_node(const String& node_id, FlowContext& context) {
    INode::Ptr node;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodes_.find(node_id);
        if (it == nodes_.end()) {
            return FlowResult::fail("Node not found: " + node_id, node_id);
        }
        node = it->second;
    }
    // 出锁再执行：算子的回调会重入引擎，持着 mutex_ 调 execute 会自死锁。
    // shared_ptr 保证节点在这里仍然活着。
    if (!node) {
        return FlowResult::fail("Node is null: " + node_id, node_id);
    }
    return execute_node(node, context);
}

void FlowEngine::step_begin() {
    std::lock_guard<std::mutex> lock(mutex_);
    stepping_ = true;
    current_step_ = 0;
}

Result<INode::Ptr> FlowEngine::step_next() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!stepping_ || current_step_ >= execution_order_.size()) {
        return Result<INode::Ptr>::failure(ErrorCode::OutOfRange, "No more steps");
    }
    
    auto node_id = execution_order_[current_step_];
    // find 而不是 operator[]：后者查不到会插入空条目污染 nodes_
    auto it = nodes_.find(node_id);
    if (it == nodes_.end() || !it->second) {
        // 失败时**不推进** current_step_，让调用方可以重试同一步
        return Result<INode::Ptr>::failure(ErrorCode::NodeNotFound, "Node not found: " + node_id);
    }
    ++current_step_;
    return Result<INode::Ptr>::success(it->second);
}

bool FlowEngine::step_has_more() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stepping_ && current_step_ < execution_order_.size();
}

void FlowEngine::step_end() {
    std::lock_guard<std::mutex> lock(mutex_);
    stepping_ = false;
}

Result<Vector<String>> FlowEngine::get_execution_order() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Result<Vector<String>>::success(execution_order_);
}

Result<Vector<String>> FlowEngine::get_dependencies(const String& node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = adjacency_list_.find(node_id);
    if (it == adjacency_list_.end()) {
        return Result<Vector<String>>::success({});
    }
    return Result<Vector<String>>::success(it->second);
}

Result<Vector<String>> FlowEngine::get_dependents(const String& node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<String> dependents;
    for (const auto& pair : adjacency_list_) {
        for (const auto& dep : pair.second) {
            if (dep == node_id) {
                dependents.push_back(pair.first);
            }
        }
    }
    return Result<Vector<String>>::success(dependents);
}

Result<Vector<String>> FlowEngine::topological_sort() const {
    // Kahn 算法拓扑排序。
    //
    // 入度 = 上游节点**个数**，前提是 adjacency_list_ 里没有重复条目。
    // load_flow 建表时已经去重，这里再用 std::count 计数递减（而不是
    // std::find 命中一次就减 1）作为第二道保险：只要还剩一条重复边，
    // 入度就永远减不到 0，一个合法的 DAG 会被误判成有环。
    HashMap<String, int> in_degree;

    for (const auto& pair : nodes_) {
        in_degree[pair.first] = 0;
    }

    // adjacency_list_ 存的是每个节点的上游节点列表
    for (const auto& pair : adjacency_list_) {
        in_degree[pair.first] = static_cast<int>(pair.second.size());
    }

    // 入度为 0 的先入队。
    // nodes_/in_degree 是 unordered_map，迭代顺序每次都可能不同 ——
    // 用有序集合当 frontier，保证同一个流程每次跑出来的节点顺序完全一致
    // （体检报告要能 diff，编辑器里刷新一次也不该换个顺序）。
    std::set<String> frontier;
    for (const auto& pair : in_degree) {
        if (pair.second == 0) {
            frontier.insert(pair.first);
        }
    }

    Vector<String> result;
    while (!frontier.empty()) {
        String node_id = *frontier.begin();   // 字典序最小的先出，顺序确定
        frontier.erase(frontier.begin());
        result.push_back(node_id);

        // 更新下游节点的入度
        for (const auto& pair : adjacency_list_) {
            auto hits = std::count(pair.second.begin(), pair.second.end(), node_id);
            if (hits > 0) {
                in_degree[pair.first] -= static_cast<int>(hits);
                // <=0 而不是 ==0：万一还有重复边漏网把入度减成负数，
                // 也得让这个节点出得来，不能卡死整条流程。
                if (in_degree[pair.first] <= 0) {
                    frontier.insert(pair.first);
                }
            }
        }
    }

    // 检查是否有环
    if (result.size() != nodes_.size()) {
        return Result<Vector<String>>::failure(
            ErrorCode::CyclicDependency,
            "Flow has cyclic dependencies"
        );
    }

    return Result<Vector<String>>::success(result);
}

FlowResult FlowEngine::execute_node(INode::Ptr node, FlowContext& context) {
    if (!node) {
        return FlowResult::fail("Node is null");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 重置节点状态（但不清空输入，因为上游节点已经传递了数据）
    node->clear_error();
    node->state_ = NodeState::Idle;
    
    // 验证输入（此时上游节点已执行，输入数据已就绪）
    auto validate_result = node->validate_inputs();
    if (validate_result.is_failure()) {
        node->set_error(validate_result.message());
        return FlowResult::fail(validate_result.message(), node->instance_id());
    }
    
    // 设置状态
    node->state_ = NodeState::Running;
    if (node_state_callback_) {
        node_state_callback_(node->instance_id(), NodeState::Running);
    }
    context.notify_node_state(node->instance_id(), NodeState::Running);
    
    // 执行。
    // 第 1 层 try/catch —— 这是整条执行路径上最关键的一处：写算子是手写指针、
    // 手写循环的重活，抛异常是常态。原先这里没有任何兜底，算子一抛就顺着
    // run() → run_thread() 冒到线程入口变成 std::terminate，整个 Web 服务当场没。
    // 现在兜成一条带 failed_node_id 的体面失败。
    auto exec_result = [&]() -> Result<void> {
        try {
            return node->execute(context);
        } catch (const std::exception& e) {
            return Result<void>::failure(
                ErrorCode::ExecutionFailed,
                String("Node '") + node->instance_id() + "' threw an exception: " + e.what());
        } catch (...) {
            return Result<void>::failure(
                ErrorCode::ExecutionFailed,
                "Node '" + node->instance_id() + "' threw a non-standard exception");
        }
    }();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto exec_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    node->record_execute_time(exec_time.count());
    
    FlowResult result;
    if (exec_result.is_success()) {
        node->state_ = NodeState::Success;
        result.success = true;
    } else {
        node->set_error(exec_result.message());
        result.success = false;
        result.error_message = exec_result.message();
        result.failed_node_id = node->instance_id();
    }
    
    if (node_state_callback_) {
        node_state_callback_(node->instance_id(), node->state_);
    }
    context.notify_node_state(node->instance_id(), node->state_);
    
    result.node_times[node->instance_id()] = exec_time.count();
    return result;
}

FlowResult FlowEngine::execute_parallel(FlowContext& context) {
    (void)context;
    // 原先这里是 `return run(context);` —— 而 run() 又按 execution_mode_ 派发回
    // 这里，两边互相递归，一调用就是栈溢出（进程直接没，连栈回溯都难拿）。
    //
    // 不做"假并行"：真正的并行需要 INode 线程安全，而全代码库有 600+ 处
    // 写路径、501 个算子一个都没做同步，那是另一个大得多的工程。
    // 这里明确报"不支持"，比装成能跑然后偶发数据损坏强得多。
    return FlowResult::fail(
        "ExecutionMode::Parallel is not supported: INode is not thread-safe. "
        "Use ExecutionMode::Sequential.");
}

FlowResult FlowEngine::execute_data_driven(FlowContext& context) {
    (void)context;
    // 同上：原先与 run() 互相递归。
    // 真正的数据驱动调度需要给每个节点维护"输入是否就绪"的状态机，
    // 而引擎现在只有拓扑序，没有这套状态。明确报不支持。
    return FlowResult::fail(
        "ExecutionMode::DataDriven is not supported. "
        "Use ExecutionMode::Sequential.");
}

// ============== FlowRunner ==============

FlowRunner::FlowRunner() {}

FlowRunner::~FlowRunner() {
    stop();
}

void FlowRunner::set_engine(FlowEngine::Ptr engine) {
    std::lock_guard<std::mutex> lock(mutex_);
    engine_ = engine;
}

Result<void> FlowRunner::start(RunMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (running_) {
        return Result<void>::failure(ErrorCode::DeviceBusy, "Runner is already running");
    }
    
    if (!engine_) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Engine not set");
    }
    
    run_mode_ = mode;
    running_ = true;
    thread_ = std::thread(&FlowRunner::run_thread, this);
    
    return Result<void>::success();
}

void FlowRunner::stop() {
    running_ = false;
    trigger_ = true;
    cv_.notify_all();
    
    if (thread_.joinable()) {
        thread_.join();
    }
}

void FlowRunner::trigger() {
    trigger_ = true;
    cv_.notify_all();
}

double FlowRunner::average_time_ms() const {
    if (total_runs_ == 0) return 0.0;
    return static_cast<double>(total_time_us_) / total_runs_ / 1000.0;
}

void FlowRunner::run_thread() {
    while (running_) {
        FlowResult result;
        
        if (run_mode_ == RunMode::Triggered) {
            // 等待触发
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return trigger_ || !running_; });
            if (!running_) break;
            trigger_ = false;
        }
        
        // 执行流程。
        // 第 3 层 try/catch —— run_thread 是**线程入口**，这是全代码库最重要的
        // 一处 catch：异常从线程函数里逃出去就是 std::terminate，整个进程直接没，
        // 而且连是哪个算子弄死的都看不到。
        try {
            auto engine = engine_;   // 快照，避免与 set_engine 竞争
            if (!engine) {
                result = FlowResult::fail("FlowRunner: engine not set");
            } else {
                result = engine->run(context_);
            }
        } catch (const std::exception& e) {
            result = FlowResult::fail(String("Flow run threw an exception: ") + e.what());
        } catch (...) {
            result = FlowResult::fail("Flow run threw a non-standard exception");
        }

        // 更新统计
        total_runs_++;
        if (result.success) {
            success_runs_++;
        } else {
            failed_runs_++;
        }
        total_time_us_ += result.total_time_us;

        // 回调。result_callback_ 是外部代码，同样可能抛 ——
        // 抛在这也一样能带走整个进程，单独兜一层。
        if (result_callback_) {
            try {
                result_callback_(result);
            } catch (const std::exception& e) {
                OVF_ERROR() << "FlowRunner: result callback threw: " << e.what();
            } catch (...) {
                OVF_ERROR() << "FlowRunner: result callback threw a non-standard exception";
            }
        }

        if (run_mode_ == RunMode::Once) {
            running_ = false;
            break;
        }
        
        if (run_mode_ == RunMode::Continuous) {
            // 立即继续下一轮
        }
    }
}

} // namespace ovf