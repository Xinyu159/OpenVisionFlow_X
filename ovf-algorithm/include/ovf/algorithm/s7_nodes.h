/**
 * @file s7_nodes.h
 * @brief S7协议节点（西门子PLC读写和控制）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/comm/communication.h"

namespace ovf {
namespace algorithm {

/**
 * @brief S7数据读取节点
 */
class S7ReadNode : public INode {
public:
    S7ReadNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief S7数据写入节点
 */
class S7WriteNode : public INode {
public:
    S7WriteNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief S7 PLC控制节点
 */
class S7ControlNode : public INode {
public:
    S7ControlNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf