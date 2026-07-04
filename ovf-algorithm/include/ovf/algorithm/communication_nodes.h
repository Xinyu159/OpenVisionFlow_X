/**
 * @file communication_nodes.h
 * @brief 通信节点（TCP发送、Modbus读写）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/comm/communication.h"

namespace ovf {
namespace algorithm {

/**
 * @brief TCP数据发送节点
 */
class TcpSendNode : public INode {
public:
    TcpSendNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief TCP数据接收节点
 */
class TcpReceiveNode : public INode {
public:
    TcpReceiveNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief Modbus寄存器读取节点
 */
class ModbusReadNode : public INode {
public:
    ModbusReadNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief Modbus寄存器写入节点
 */
class ModbusWriteNode : public INode {
public:
    ModbusWriteNode(const String& instance_id);
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 数据格式化节点（将数据转换为字符串）
 */
class DataFormatNode : public INode {
public:
    DataFormatNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf