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

/**
 * @brief 节点工厂
 */
class NodeFactory {
public:
    using Creator = std::function<INode::Ptr(const String&)>;
    
    static NodeFactory& instance() {
        static NodeFactory factory;
        return factory;
    }
    
    void register_node(const String& type_id, Creator creator, const NodeInfo& info) {
        creators_[type_id] = creator;
        infos_[type_id] = info;
    }
    
    INode::Ptr create(const String& type_id, const String& instance_id) {
        auto it = creators_.find(type_id);
        if (it == creators_.end()) {
            OVF_ERROR() << "Node type not found: " << type_id;
            return nullptr;
        }
        return it->second(instance_id);
    }
    
    const NodeInfo* get_info(const String& type_id) const {
        auto it = infos_.find(type_id);
        return it != infos_.end() ? &it->second : nullptr;
    }
    
    Vector<String> get_all_types() const {
        Vector<String> types;
        for (const auto& pair : creators_) {
            types.push_back(pair.first);
        }
        return types;
    }
    
    bool has_type(const String& type_id) const {
        return creators_.find(type_id) != creators_.end();
    }

private:
    NodeFactory() = default;
    
    HashMap<String, Creator> creators_;
    HashMap<String, NodeInfo> infos_;
};

// 节点注册宏
#define OVF_REGISTER_NODE(NodeClass, type_id, info) \
    namespace { \
        struct NodeClass##Registrar { \
            NodeClass##Registrar() { \
                ovf::NodeFactory::instance().register_node(type_id, \
                    [](const ovf::String& id) -> ovf::INode::Ptr { \
                        return std::make_shared<NodeClass>(id); \
                    }, info); \
            } \
        } registrar_##NodeClass; \
    }

} // namespace ovf