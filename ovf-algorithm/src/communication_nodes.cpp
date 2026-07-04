/**
 * @file communication_nodes.cpp
 * @brief 通信节点实现
 */

#include "ovf/algorithm/communication_nodes.h"
#include "ovf/core/logger.h"
#include <sstream>

namespace ovf {
namespace algorithm {

TcpSendNode::TcpSendNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo TcpSendNode::make_info() {
    NodeInfo info;
    info.id = "TcpSend";
    info.name = "TCP发送";
    info.category = "通信";
    info.description = "通过TCP发送数据";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("data", "发送数据", DataType::String, true));
    
    info.params.push_back(ParamDef("address", "服务器地址", DataType::String, Data("127.0.0.1")));
    info.params.push_back(ParamDef("port", "端口", DataType::Number, Data(8888)));
    
    return info;
}

Result<void> TcpSendNode::init() {
    return Result<void>::success();
}

Result<void> TcpSendNode::execute(FlowContext& context) {
    String address = get_param("address", Data("127.0.0.1")).as_string();
    uint16_t port = static_cast<uint16_t>(get_param("port", Data(8888)).as_int());
    
    auto input_data = get_input("data");
    String data_str;
    
    if (input_data.is_string()) {
        data_str = input_data.as_string();
    } else if (input_data.is_number()) {
        data_str = std::to_string(input_data.as_number());
    } else if (input_data.is_image()) {
        std::ostringstream oss;
        oss << "Image[" << input_data.as_image().width << "x" 
            << input_data.as_image().height << "]";
        data_str = oss.str();
    } else {
        data_str = input_data.to_string();
    }
    
    // 创建TCP客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = address;
    device_info.port = port;
    device_info.protocol = "TCP";
    
    comm::TcpClient client(device_info);
    
    // 连接服务器
    auto connect_result = client.connect();
    if (connect_result.is_failure()) {
        OVF_ERROR() << "TCP connect failed: " << connect_result.message();
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "Failed to connect to " + address + ":" + std::to_string(port));
    }
    
    // 发送数据
    ByteArray bytes(data_str.size());
    std::memcpy(bytes.data(), data_str.data(), data_str.size());
    
    auto send_result = client.send(bytes);
    if (send_result.is_failure()) {
        client.disconnect();
        OVF_ERROR() << "TCP send failed: " << send_result.message();
        return Result<void>::failure(ErrorCode::SendFailed, "Failed to send data");
    }
    
    client.disconnect();
    
    OVF_INFO() << "TCP sent " << data_str.size() << " bytes to " 
               << address << ":" << port;
    
    return Result<void>::success();
}

TcpReceiveNode::TcpReceiveNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo TcpReceiveNode::make_info() {
    NodeInfo info;
    info.id = "TcpReceive";
    info.name = "TCP接收";
    info.category = "通信";
    info.description = "通过TCP接收数据";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("data", "接收数据", DataType::String));
    
    info.params.push_back(ParamDef("address", "服务器地址", DataType::String, Data("127.0.0.1")));
    info.params.push_back(ParamDef("port", "端口", DataType::Number, Data(8888)));
    info.params.push_back(ParamDef("timeout", "超时(ms)", DataType::Number, Data(1000)));
    
    return info;
}

Result<void> TcpReceiveNode::init() {
    return Result<void>::success();
}

Result<void> TcpReceiveNode::execute(FlowContext& context) {
    String address = get_param("address", Data("127.0.0.1")).as_string();
    uint16_t port = static_cast<uint16_t>(get_param("port", Data(8888)).as_int());
    uint32_t timeout = static_cast<uint32_t>(get_param("timeout", Data(1000)).as_int());
    
    // 创建TCP客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = address;
    device_info.port = port;
    device_info.protocol = "TCP";
    
    comm::TcpClient client(device_info);
    
    // 连接服务器
    auto connect_result = client.connect();
    if (connect_result.is_failure()) {
        OVF_ERROR() << "TCP connect failed: " << connect_result.message();
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "Failed to connect to " + address + ":" + std::to_string(port));
    }
    
    // 接收数据
    auto receive_result = client.receive_string(timeout);
    
    client.disconnect();
    
    if (receive_result.is_failure()) {
        OVF_WARN() << "TCP receive timeout or failed";
        set_output("data", Data(""));
        return Result<void>::success();
    }
    
    set_output("data", Data(receive_result.value()));
    
    OVF_INFO() << "TCP received " << receive_result.value().size() << " bytes";
    
    return Result<void>::success();
}

ModbusReadNode::ModbusReadNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ModbusReadNode::make_info() {
    NodeInfo info;
    info.id = "ModbusRead";
    info.name = "Modbus读取";
    info.category = "通信";
    info.description = "读取Modbus寄存器";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("value", "读取值", DataType::Number));
    
    info.params.push_back(ParamDef("address", "服务器地址", DataType::String, Data("127.0.0.1")));
    info.params.push_back(ParamDef("port", "端口", DataType::Number, Data(502)));
    info.params.push_back(ParamDef("unit_id", "单元ID", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("reg_address", "寄存器地址", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("reg_type", "寄存器类型", DataType::String, Data("holding")));
    
    return info;
}

Result<void> ModbusReadNode::init() {
    return Result<void>::success();
}

Result<void> ModbusReadNode::execute(FlowContext& context) {
    String address = get_param("address", Data("127.0.0.1")).as_string();
    uint16_t port = static_cast<uint16_t>(get_param("port", Data(502)).as_int());
    uint8_t unit_id = static_cast<uint8_t>(get_param("unit_id", Data(1)).as_int());
    uint16_t reg_address = static_cast<uint16_t>(get_param("reg_address", Data(0)).as_int());
    String reg_type = get_param("reg_type", Data("holding")).as_string();
    
    // 创建Modbus客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = address;
    device_info.port = port;
    device_info.protocol = "MODBUS_TCP";
    
    comm::ModbusClient client(device_info, comm::ModbusClient::ModbusType::TCP);
    
    // 连接
    auto connect_result = client.connect();
    if (connect_result.is_failure()) {
        OVF_ERROR() << "Modbus connect failed: " << connect_result.message();
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "Failed to connect to Modbus server");
    }
    
    // 读取寄存器
    Result<uint16_t> read_result = Result<uint16_t>::failure(ErrorCode::Unknown, "Not initialized");
    
    if (reg_type == "holding" || reg_type == "HoldingRegister") {
        read_result = client.read_holding_register(unit_id, reg_address);
    } else if (reg_type == "input" || reg_type == "InputRegister") {
        read_result = client.read_input_register(unit_id, reg_address);
    } else if (reg_type == "coil") {
        auto coil_result = client.read_coil(unit_id, reg_address);
        if (coil_result.is_success()) {
            read_result = Result<uint16_t>::success(coil_result.value());
        } else {
            read_result = Result<uint16_t>::failure(coil_result.code(), coil_result.message());
        }
    } else {
        read_result = Result<uint16_t>::failure(ErrorCode::InvalidParameter, 
            "Unknown register type: " + reg_type);
    }
    
    client.disconnect();
    
    if (read_result.is_failure()) {
        OVF_ERROR() << "Modbus read failed: " << read_result.message();
        return Result<void>::failure(read_result.code(), read_result.message());
    }
    
    set_output("value", Data(static_cast<int>(read_result.value())));
    
    OVF_INFO() << "Modbus read: address=" << reg_address << ", value=" << read_result.value();
    
    return Result<void>::success();
}

ModbusWriteNode::ModbusWriteNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ModbusWriteNode::make_info() {
    NodeInfo info;
    info.id = "ModbusWrite";
    info.name = "Modbus写入";
    info.category = "通信";
    info.description = "写入Modbus寄存器";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("value", "写入值", DataType::Number, true));
    
    info.params.push_back(ParamDef("address", "服务器地址", DataType::String, Data("127.0.0.1")));
    info.params.push_back(ParamDef("port", "端口", DataType::Number, Data(502)));
    info.params.push_back(ParamDef("unit_id", "单元ID", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("reg_address", "寄存器地址", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("reg_type", "寄存器类型", DataType::String, Data("holding")));
    
    return info;
}

Result<void> ModbusWriteNode::init() {
    return Result<void>::success();
}

Result<void> ModbusWriteNode::execute(FlowContext& context) {
    String address = get_param("address", Data("127.0.0.1")).as_string();
    uint16_t port = static_cast<uint16_t>(get_param("port", Data(502)).as_int());
    uint8_t unit_id = static_cast<uint8_t>(get_param("unit_id", Data(1)).as_int());
    uint16_t reg_address = static_cast<uint16_t>(get_param("reg_address", Data(0)).as_int());
    String reg_type = get_param("reg_type", Data("holding")).as_string();
    
    auto value_data = get_input("value");
    if (!value_data.is_number()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Input value must be a number");
    }
    
    uint16_t value = static_cast<uint16_t>(value_data.as_int());
    
    // 创建Modbus客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = address;
    device_info.port = port;
    device_info.protocol = "MODBUS_TCP";
    
    comm::ModbusClient client(device_info, comm::ModbusClient::ModbusType::TCP);
    
    // 连接
    auto connect_result = client.connect();
    if (connect_result.is_failure()) {
        OVF_ERROR() << "Modbus connect failed: " << connect_result.message();
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "Failed to connect to Modbus server");
    }
    
    // 写入寄存器
    Result<void> write_result = Result<void>::failure(ErrorCode::Unknown, "Not initialized");
    
    if (reg_type == "holding" || reg_type == "HoldingRegister") {
        write_result = client.write_single_register(unit_id, reg_address, value);
    } else if (reg_type == "coil") {
        write_result = client.write_single_coil(unit_id, reg_address, value != 0);
    } else {
        write_result = Result<void>::failure(ErrorCode::InvalidParameter, 
            "Unknown register type: " + reg_type);
    }
    
    client.disconnect();
    
    if (write_result.is_failure()) {
        OVF_ERROR() << "Modbus write failed: " << write_result.message();
        return write_result;
    }
    
    OVF_INFO() << "Modbus write: address=" << reg_address << ", value=" << value;
    
    return Result<void>::success();
}

DataFormatNode::DataFormatNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DataFormatNode::make_info() {
    NodeInfo info;
    info.id = "DataFormat";
    info.name = "数据格式化";
    info.category = "数据处理";
    info.description = "将输入数据格式化为字符串";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("data", "输入数据", DataType::Any, true));
    info.outputs.push_back(DataPort("output", "格式化输出", DataType::String));
    
    info.params.push_back(ParamDef("format", "格式模板", DataType::String, Data("{}")));
    info.params.push_back(ParamDef("prefix", "前缀", DataType::String, Data("")));
    info.params.push_back(ParamDef("suffix", "后缀", DataType::String, Data("")));
    
    return info;
}

Result<void> DataFormatNode::execute(FlowContext& context) {
    auto input_data = get_input("data");
    String format = get_param("format", Data("{}")).as_string();
    String prefix = get_param("prefix", Data("")).as_string();
    String suffix = get_param("suffix", Data("")).as_string();
    
    // 格式化数据
    String data_str;
    if (input_data.is_number()) {
        data_str = std::to_string(input_data.as_number());
    } else if (input_data.is_string()) {
        data_str = input_data.as_string();
    } else if (input_data.is_bool()) {
        data_str = input_data.as_bool() ? "true" : "false";
    } else if (input_data.is_image()) {
        std::ostringstream oss;
        oss << "Image[" << input_data.as_image().width << "x" 
            << input_data.as_image().height << "]";
        data_str = oss.str();
    } else {
        data_str = input_data.to_string();
    }
    
    // 替换格式模板中的{}
    String output = format;
    size_t pos = output.find("{}");
    if (pos != String::npos) {
        output.replace(pos, 2, data_str);
    } else {
        output = data_str;
    }
    
    output = prefix + output + suffix;
    
    set_output("output", Data(output));
    
    OVF_INFO() << "Data formatted: " << output;
    
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(TcpSendNode, "TcpSend", TcpSendNode::make_info());
OVF_REGISTER_NODE(TcpReceiveNode, "TcpReceive", TcpReceiveNode::make_info());
OVF_REGISTER_NODE(ModbusReadNode, "ModbusRead", ModbusReadNode::make_info());
OVF_REGISTER_NODE(ModbusWriteNode, "ModbusWrite", ModbusWriteNode::make_info());
OVF_REGISTER_NODE(DataFormatNode, "DataFormat", DataFormatNode::make_info());

} // namespace algorithm
} // namespace ovf