/**
 * @file node.cpp
 * @brief 节点基类实现
 */

#include "ovf/core/node.h"
#include "ovf/core/flow.h"
#include <chrono>

namespace ovf {

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