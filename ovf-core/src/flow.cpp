/**
 * @file flow.cpp
 * @brief 流程引擎实现
 */

#include "ovf/core/flow.h"
#include <chrono>
#include <algorithm>
#include <fstream>
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

        // 设置参数 - 直接使用配置（简化）
        // 参数由节点自行处理

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
            
            // 构建邻接表（依赖关系）
            adjacency_list_[node_inst.id].push_back(conn.source_node_id);
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

Result<void> FlowEngine::load_from_file(const String& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return Result<void>::failure(
            ErrorCode::FileOpenFailed,
            "Failed to open file: " + filepath
        );
    }
    
    try {
        // 读取文件内容
        std::string content((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
        json j = json::parse(content);
        
        FlowDef flow_def;
        flow_def.id = j["id"].get_string();
        flow_def.name = j["name"].get_string();
        flow_def.description = j.value("description", std::string(""));
        flow_def.version = j.value("version", std::string("1.0"));
        
        // 解析节点
        auto nodes_j = j["nodes"];
        if (nodes_j.is_array()) {
            for (size_t i = 0; i < nodes_j.size(); ++i) {
                auto node_j = nodes_j[i];
                FlowDef::NodeInstance node_inst;
                node_inst.id = node_j["id"].get_string();
                node_inst.type_id = node_j["type_id"].get_string();
                node_inst.name = node_j.value("name", std::string(""));
                node_inst.x = node_j.value("x", 0);
                node_inst.y = node_j.value("y", 0);
                node_inst.enabled = node_j.value("enabled", true);

                // 解析参数（简化：直接存储json字符串）
                if (node_j.contains("params") && node_j["params"].is_object()) {
                    // 存储原始json以便节点初始化时使用
                    // 后续节点加载时会处理这些参数
                }

                // 解析输入连接
                if (node_j.contains("inputs") && node_j["inputs"].is_object()) {
                    // 简化版json不支持迭代器，需要显式处理
                    // 这里暂时跳过，后续完善
                }

                flow_def.nodes.push_back(node_inst);
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

        // 保存参数（简化）
        node_j["params"] = json::object();

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
    auto start_time = std::chrono::high_resolution_clock::now();
    
    switch (execution_mode_) {
        case ExecutionMode::Sequential:
            for (const auto& node_id : execution_order_) {
                auto node = nodes_[node_id];
                if (!node || !node->is_enabled()) continue;
                
                auto result = execute_node(node, context);
                if (!result.success) {
                    return result;
                }
                
                if (context.is_stopped()) {
                    break;
                }
                
                // 等待暂停恢复
                while (context.is_paused() && !context.is_stopped()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            break;
            
        case ExecutionMode::Parallel:
            return execute_parallel(context);
            
        case ExecutionMode::DataDriven:
            return execute_data_driven(context);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    FlowResult result = FlowResult::ok();
    result.total_time_us = total_time.count();
    return result;
}

FlowResult FlowEngine::run_node(const String& node_id, FlowContext& context) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto node = nodes_.find(node_id);
    if (node == nodes_.end()) {
        return FlowResult::fail("Node not found: " + node_id, node_id);
    }
    
    return execute_node(node->second, context);
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
    
    auto node_id = execution_order_[current_step_++];
    auto node = nodes_[node_id];
    return Result<INode::Ptr>::success(node);
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
    // Kahn算法拓扑排序
    HashMap<String, int> in_degree;
    
    // 初始化入度
    for (const auto& pair : nodes_) {
        in_degree[pair.first] = 0;
    }
    
    // 计算入度
    for (const auto& pair : adjacency_list_) {
        for (const auto& dep : pair.second) {
            // dep -> pair.first 的边
            // pair.first 的入度增加
        }
    }
    
    // 重新计算入度（根据实际连接）
    for (const auto& pair : adjacency_list_) {
        for (const auto& dep : pair.second) {
            // dep 是 pair.first 的依赖（上游节点）
            // 所以 pair.first 的入度应该是其上游节点数量
        }
    }
    
    // 实际上 adjacency_list_ 存储的是每个节点的上游节点列表
    for (const auto& pair : adjacency_list_) {
        in_degree[pair.first] = static_cast<int>(pair.second.size());
    }
    
    // 找到所有入度为0的节点
    Vector<String> queue;
    for (const auto& pair : in_degree) {
        if (pair.second == 0) {
            queue.push_back(pair.first);
        }
    }
    
    Vector<String> result;
    while (!queue.empty()) {
        String node_id = queue.back();
        queue.pop_back();
        result.push_back(node_id);
        
        // 更新下游节点的入度
        for (const auto& pair : adjacency_list_) {
            if (std::find(pair.second.begin(), pair.second.end(), node_id) != pair.second.end()) {
                in_degree[pair.first]--;
                if (in_degree[pair.first] == 0) {
                    queue.push_back(pair.first);
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
    
    node->reset();
    
    // 验证输入
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
    
    // 执行
    auto exec_result = node->execute(context);
    
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
    // 并行执行 - 基于层级并行
    // 暂时简化实现，后续完善
    return run(context);
}

FlowResult FlowEngine::execute_data_driven(FlowContext& context) {
    // 数据驱动执行 - 当数据就绪时执行节点
    // 暂时简化实现，后续完善
    return run(context);
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
        
        // 执行流程
        result = engine_->run(context_);
        
        // 更新统计
        total_runs_++;
        if (result.success) {
            success_runs_++;
        } else {
            failed_runs_++;
        }
        total_time_us_ += result.total_time_us;
        
        // 回调
        if (result_callback_) {
            result_callback_(result);
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