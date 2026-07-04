/**
 * @file s7_nodes.cpp
 * @brief S7协议节点实现
 */

#include "ovf/algorithm/s7_nodes.h"
#include "ovf/core/logger.h"

namespace ovf {
namespace algorithm {

// ============== S7ReadNode ==============

S7ReadNode::S7ReadNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo S7ReadNode::make_info() {
    NodeInfo info;
    info.id = "S7Read";
    info.name = "S7读取";
    info.category = "通信";
    info.description = "读取西门子PLC数据";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("value", "读取值", DataType::Number));
    
    info.params.push_back(ParamDef("ip", "PLC IP地址", DataType::String, Data("192.168.0.1")));
    info.params.push_back(ParamDef("rack", "Rack编号", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("slot", "Slot编号", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("area", "数据区域", DataType::String, Data("DB")));
    info.params.push_back(ParamDef("db_number", "DB块编号", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("address", "字节地址", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("data_type", "数据类型", DataType::String, Data("int")));
    info.params.push_back(ParamDef("bit_offset", "位偏移", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("size", "读取长度", DataType::Number, Data(4)));
    
    return info;
}

Result<void> S7ReadNode::init() {
    return Result<void>::success();
}

Result<void> S7ReadNode::execute(FlowContext& context) {
    String ip = get_param("ip", Data("192.168.0.1")).as_string();
    int rack = get_param("rack", Data(0)).as_int();
    int slot = get_param("slot", Data(1)).as_int();
    String area_str = get_param("area", Data("DB")).as_string();
    int db_number = get_param("db_number", Data(1)).as_int();
    int address = get_param("address", Data(0)).as_int();
    String data_type = get_param("data_type", Data("int")).as_string();
    int size = get_param("size", Data(4)).as_int();
    
    // 解析数据区域
    comm::S7Area area = comm::S7Area::DB;
    if (area_str == "DB" || area_str == "db") {
        area = comm::S7Area::DB;
    } else if (area_str == "PE" || area_str == "Input" || area_str == "I") {
        area = comm::S7Area::PE;
    } else if (area_str == "PA" || area_str == "Output" || area_str == "Q") {
        area = comm::S7Area::PA;
    } else if (area_str == "MK" || area_str == "Memory" || area_str == "M") {
        area = comm::S7Area::MK;
    } else if (area_str == "CT" || area_str == "Counter" || area_str == "C") {
        area = comm::S7Area::CT;
    } else if (area_str == "TM" || area_str == "Timer" || area_str == "T") {
        area = comm::S7Area::TM;
    }
    
    // 创建S7客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = ip;
    device_info.port = 102;
    device_info.protocol = "S7";
    
    comm::S7Client client(device_info);
    
    // 连接PLC
    auto connect_result = client.connect_plc(ip, rack, slot);
    if (connect_result.is_failure()) {
        OVF_ERROR() << "S7连接失败: " << connect_result.message();
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "无法连接PLC: " + ip);
    }
    
    // 读取数据
    if (data_type == "int" || data_type == "INT" || data_type == "DINT") {
        auto read_result = client.read_int(area, db_number, address);
        client.disconnect();
        
        if (read_result.is_failure()) {
            OVF_ERROR() << "S7读取失败: " << read_result.message();
            return Result<void>::failure(read_result.code(), read_result.message());
        }
        
        set_output("value", Data(static_cast<int>(read_result.value())));
        OVF_INFO() << "S7读取INT: DB" << db_number << "." << address << " = " << read_result.value();
    } 
    else if (data_type == "float" || data_type == "FLOAT" || data_type == "REAL") {
        auto read_result = client.read_float(area, db_number, address);
        client.disconnect();
        
        if (read_result.is_failure()) {
            OVF_ERROR() << "S7读取失败: " << read_result.message();
            return Result<void>::failure(read_result.code(), read_result.message());
        }
        
        set_output("value", Data(static_cast<double>(read_result.value())));
        OVF_INFO() << "S7读取FLOAT: DB" << db_number << "." << address << " = " << read_result.value();
    } 
    else if (data_type == "bool" || data_type == "BOOL" || data_type == "bit" || data_type == "BIT") {
        int bit_offset = get_param("bit_offset", Data(0)).as_int();
        auto read_result = client.read_bit(area, db_number, address, bit_offset);
        client.disconnect();
        
        if (read_result.is_failure()) {
            OVF_ERROR() << "S7读取失败: " << read_result.message();
            return Result<void>::failure(read_result.code(), read_result.message());
        }
        
        set_output("value", Data(read_result.value()));
        OVF_INFO() << "S7读取BOOL: DB" << db_number << "." << address << "." << bit_offset 
                   << " = " << (read_result.value() ? "true" : "false");
    } 
    else if (data_type == "bytes" || data_type == "BYTES" || data_type == "raw" || data_type == "RAW") {
        auto read_result = client.read_bytes(area, db_number, address, size);
        client.disconnect();
        
        if (read_result.is_failure()) {
            OVF_ERROR() << "S7读取失败: " << read_result.message();
            return Result<void>::failure(read_result.code(), read_result.message());
        }
        
        // 将字节数组转为数值（第一个字节）
        if (!read_result.value().empty()) {
            int value = read_result.value()[0];
            set_output("value", Data(value));
        } else {
            set_output("value", Data(0));
        }
        OVF_INFO() << "S7读取BYTES: DB" << db_number << "." << address 
                   << " [" << size << " bytes]";
    } 
    else {
        client.disconnect();
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "不支持的数据类型: " + data_type);
    }
    
    return Result<void>::success();
}

// ============== S7WriteNode ==============

S7WriteNode::S7WriteNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo S7WriteNode::make_info() {
    NodeInfo info;
    info.id = "S7Write";
    info.name = "S7写入";
    info.category = "通信";
    info.description = "写入西门子PLC数据";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("value", "写入值", DataType::Number, true));
    
    info.params.push_back(ParamDef("ip", "PLC IP地址", DataType::String, Data("192.168.0.1")));
    info.params.push_back(ParamDef("rack", "Rack编号", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("slot", "Slot编号", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("area", "数据区域", DataType::String, Data("DB")));
    info.params.push_back(ParamDef("db_number", "DB块编号", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("address", "字节地址", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("data_type", "数据类型", DataType::String, Data("int")));
    info.params.push_back(ParamDef("bit_offset", "位偏移", DataType::Number, Data(0)));
    
    return info;
}

Result<void> S7WriteNode::init() {
    return Result<void>::success();
}

Result<void> S7WriteNode::execute(FlowContext& context) {
    String ip = get_param("ip", Data("192.168.0.1")).as_string();
    int rack = get_param("rack", Data(0)).as_int();
    int slot = get_param("slot", Data(1)).as_int();
    String area_str = get_param("area", Data("DB")).as_string();
    int db_number = get_param("db_number", Data(1)).as_int();
    int address = get_param("address", Data(0)).as_int();
    String data_type = get_param("data_type", Data("int")).as_string();
    int bit_offset = get_param("bit_offset", Data(0)).as_int();
    
    // 获取输入值
    auto value_data = get_input("value");
    if (!value_data.is_number() && !value_data.is_bool()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "输入值必须是数值或布尔值");
    }
    
    // 解析数据区域
    comm::S7Area area = comm::S7Area::DB;
    if (area_str == "DB" || area_str == "db") {
        area = comm::S7Area::DB;
    } else if (area_str == "PE" || area_str == "Input" || area_str == "I") {
        area = comm::S7Area::PE;
    } else if (area_str == "PA" || area_str == "Output" || area_str == "Q") {
        area = comm::S7Area::PA;
    } else if (area_str == "MK" || area_str == "Memory" || area_str == "M") {
        area = comm::S7Area::MK;
    }
    
    // 创建S7客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = ip;
    device_info.port = 102;
    device_info.protocol = "S7";
    
    comm::S7Client client(device_info);
    
    // 连接PLC
    auto connect_result = client.connect_plc(ip, rack, slot);
    if (connect_result.is_failure()) {
        OVF_ERROR() << "S7连接失败: " << connect_result.message();
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "无法连接PLC: " + ip);
    }
    
    // 写入数据
    Result<void> write_result = Result<void>::success();
    
    if (data_type == "int" || data_type == "INT" || data_type == "DINT") {
        int32_t value = static_cast<int32_t>(value_data.as_int());
        write_result = client.write_int(area, db_number, address, value);
        
        if (write_result.is_success()) {
            OVF_INFO() << "S7写入INT: DB" << db_number << "." << address << " = " << value;
        }
    } 
    else if (data_type == "float" || data_type == "FLOAT" || data_type == "REAL") {
        float value = static_cast<float>(value_data.as_number());
        write_result = client.write_float(area, db_number, address, value);
        
        if (write_result.is_success()) {
            OVF_INFO() << "S7写入FLOAT: DB" << db_number << "." << address << " = " << value;
        }
    } 
    else if (data_type == "bool" || data_type == "BOOL" || data_type == "bit" || data_type == "BIT") {
        bool value = value_data.is_bool() ? value_data.as_bool() : (value_data.as_int() != 0);
        write_result = client.write_bit(area, db_number, address, bit_offset, value);
        
        if (write_result.is_success()) {
            OVF_INFO() << "S7写入BOOL: DB" << db_number << "." << address 
                       << "." << bit_offset << " = " << (value ? "true" : "false");
        }
    } 
    else {
        client.disconnect();
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "不支持的数据类型: " + data_type);
    }
    
    client.disconnect();
    
    if (write_result.is_failure()) {
        OVF_ERROR() << "S7写入失败: " << write_result.message();
        return write_result;
    }
    
    return Result<void>::success();
}

// ============== S7ControlNode ==============

S7ControlNode::S7ControlNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo S7ControlNode::make_info() {
    NodeInfo info;
    info.id = "S7Control";
    info.name = "S7控制";
    info.category = "通信";
    info.description = "控制西门子PLC运行状态";
    info.version = "0.1.0";
    
    info.outputs.push_back(DataPort("status", "状态信息", DataType::String));
    
    info.params.push_back(ParamDef("ip", "PLC IP地址", DataType::String, Data("192.168.0.1")));
    info.params.push_back(ParamDef("rack", "Rack编号", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("slot", "Slot编号", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("operation", "操作", DataType::String, Data("info")));
    
    Vector<String> options;
    options.push_back("start");
    options.push_back("stop");
    options.push_back("hot_start");
    options.push_back("info");
    info.params.back().options = options;
    
    return info;
}

Result<void> S7ControlNode::init() {
    return Result<void>::success();
}

Result<void> S7ControlNode::execute(FlowContext& context) {
    String ip = get_param("ip", Data("192.168.0.1")).as_string();
    int rack = get_param("rack", Data(0)).as_int();
    int slot = get_param("slot", Data(1)).as_int();
    String operation = get_param("operation", Data("info")).as_string();
    
    // 创建S7客户端
    comm::CommDeviceInfo device_info;
    device_info.id = instance_id();
    device_info.address = ip;
    device_info.port = 102;
    device_info.protocol = "S7";
    
    comm::S7Client client(device_info);
    
    // 连接PLC
    auto connect_result = client.connect_plc(ip, rack, slot);
    if (connect_result.is_failure()) {
        OVF_ERROR() << "S7连接失败: " << connect_result.message();
        set_output("status", Data("连接失败: " + connect_result.message()));
        return Result<void>::failure(ErrorCode::ConnectionFailed, 
            "无法连接PLC: " + ip);
    }
    
    Result<void> control_result = Result<void>::success();
    String status_msg;
    
    if (operation == "start") {
        control_result = client.plc_start();
        status_msg = control_result.is_success() ? "PLC启动成功" : "PLC启动失败";
    } 
    else if (operation == "stop") {
        control_result = client.plc_stop();
        status_msg = control_result.is_success() ? "PLC停止成功" : "PLC停止失败";
    } 
    else if (operation == "hot_start") {
        control_result = client.plc_hot_start();
        status_msg = control_result.is_success() ? "PLC热启动成功" : "PLC热启动失败";
    } 
    else if (operation == "info") {
        auto info_result = client.get_plc_info();
        if (info_result.is_success()) {
            status_msg = info_result.value();
        } else {
            status_msg = "获取PLC信息失败";
            control_result = Result<void>::failure(info_result.code(), info_result.message());
        }
    } 
    else {
        client.disconnect();
        return Result<void>::failure(ErrorCode::InvalidParameter, 
            "不支持的操作: " + operation);
    }
    
    client.disconnect();
    
    set_output("status", Data(status_msg));
    
    if (control_result.is_failure()) {
        OVF_ERROR() << "S7控制失败: " << control_result.message();
        return control_result;
    }
    
    OVF_INFO() << "S7控制操作完成: " << operation;
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(S7ReadNode, "S7Read", S7ReadNode::make_info());
OVF_REGISTER_NODE(S7WriteNode, "S7Write", S7WriteNode::make_info());
OVF_REGISTER_NODE(S7ControlNode, "S7Control", S7ControlNode::make_info());

} // namespace algorithm
} // namespace ovf