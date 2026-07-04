/**
 * @file s7_protocol.cpp
 * @brief S7协议(西门子PLC)通信实现
 */

#include "ovf/comm/communication.h"
#include "ovf/core/logger.h"

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
#endif

#include <cstring>
#include <chrono>

namespace ovf {
namespace comm {

// S7协议常量
constexpr int S7_PORT = 102;  // S7默认端口
constexpr uint8_t PDU_TYPE_DATA = 0x02;
constexpr uint8_t PDU_TYPE_CONNECT = 0x0E;
constexpr uint8_t S7_READ = 0x04;
constexpr uint8_t S7_WRITE = 0x05;
constexpr uint8_t S7_CONTROL = 0x28;

// COTP连接请求模板
static const uint8_t COTP_CONNECT_REQUEST[] = {
    // TPKT Header (4 bytes)
    0x03, 0x00, 0x00, 0x16,  // Version 3, Length 22
    
    // COTP Header (3 bytes)
    0x11,                    // Length 17 (PDU Type Connect)
    0xE0,                    // PDU Type: Connect Request
    0x00,                    // Destination Reference (0 = unknown)
    
    // COTP Parameters
    0x00, 0x01, 0x00,       // Source Reference
    0xC1,                    // Parameter: Called TSAP
    0x02,                    // Parameter Length
    0x01, 0x00,              // Called TSAP: PG
    0xC2,                    // Parameter: Calling TSAP
    0x02,                    // Parameter Length
    0x01, 0x02,              // Calling TSAP: Rack=1, Slot=2 (will be modified)
    0xC0,                    // Parameter: TPDU Size
    0x01,                    // Parameter Length
    0x0A                     // TPDU Size: 1024 (0x0A = 10, 2^10 = 1024)
};

// S7通信设置请求模板
static const uint8_t S7_COMM_SETUP[] = {
    // TPKT Header (4 bytes)
    0x03, 0x00, 0x00, 0x19,  // Version 3, Length 25
    
    // COTP Header (3 bytes)
    0x02,                    // Length 2 (Data)
    0xF0,                    // PDU Type: Data
    0x80,                    // EOT = 1, TPDU Number = 0
    
    // S7 Header (12 bytes)
    0x32,                    // S7 Protocol ID
    0x01,                    // Message Type: Job Request
    0x00, 0x00,              // Redundancy ID
    0x00, 0x05,              // PDU Reference (will be modified)
    0x00, 0x08,              // Parameter Length: 8
    0x00, 0x00,              // Data Length: 0
    
    // S7 Parameters
    0xF0,                    // Function: Setup Communication
    0x00,                    // Reserved
    0x00, 0x01,              // Max AMQ Caller: 1
    0x00, 0x01,              // Max AMQ Callee: 1
    0x00, 0xF0,              // PDU Size: 240 (will be negotiated)
};

// ============== S7Client ==============

S7Client::S7Client(const CommDeviceInfo& info) : info_(info) {
    // Windows Socket初始化
#ifdef _WIN32
    static bool winsock_initialized = false;
    if (!winsock_initialized) {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        winsock_initialized = true;
    }
#endif
}

S7Client::~S7Client() {
    disconnect();
}

Result<void> S7Client::connect() {
    // 默认使用info中的rack和slot
    return connect_plc(info_.address, 0, 1);
}

Result<void> S7Client::connect_plc(const String& ip, int rack, int slot) {
    if (is_connected()) {
        return Result<void>::success();
    }
    
    state_ = ConnectionState::Connecting;
    ip_ = ip;
    rack_ = rack;
    slot_ = slot;
    
    // 创建TCP Socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == INVALID_SOCKET) {
        state_ = ConnectionState::Error;
        last_error_ = "Failed to create socket";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    // 设置服务器地址
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(S7_PORT);
    
#ifdef _WIN32
    inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);
#else
    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());
#endif
    
    // 连接PLC
    if (::connect(socket_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
        state_ = ConnectionState::Error;
        last_error_ = "Failed to connect to PLC at " + ip;
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    // Step 1: 发送COTP连接请求
    ByteArray cotp_request = build_cotp_connection_request(rack, slot);
    auto send_result = send(cotp_request);
    if (send_result.is_failure()) {
        disconnect();
        return send_result;
    }
    
    // 接收COTP连接响应
    auto cotp_response = receive(3000);
    if (cotp_response.is_failure()) {
        disconnect();
        return Result<void>::failure(ErrorCode::ConnectionFailed, "COTP connection timeout");
    }
    
    // 解析COTP响应
    auto parse_result = parse_cotp_connection_response(cotp_response.value());
    if (parse_result.is_failure()) {
        disconnect();
        return parse_result;
    }
    
    // Step 2: 发送S7通信设置请求
    ByteArray s7_setup = build_s7_communication_request();
    send_result = send(s7_setup);
    if (send_result.is_failure()) {
        disconnect();
        return send_result;
    }
    
    // 接收S7通信设置响应
    auto s7_response = receive(3000);
    if (s7_response.is_failure()) {
        disconnect();
        return Result<void>::failure(ErrorCode::ConnectionFailed, "S7 setup timeout");
    }
    
    // 解析S7响应
    parse_result = parse_s7_communication_response(s7_response.value());
    if (parse_result.is_failure()) {
        disconnect();
        return parse_result;
    }
    
    state_ = ConnectionState::Connected;
    
    OVF_INFO() << "S7Client connected to PLC: " << ip << " (Rack=" << rack << ", Slot=" << slot << ")";
    return Result<void>::success();
}

Result<void> S7Client::disconnect() {
    if (socket_fd_ != -1) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
    }
    
    state_ = ConnectionState::Disconnected;
    message_number_ = 0;
    
    OVF_INFO() << "S7Client disconnected";
    return Result<void>::success();
}

Result<void> S7Client::send(const ByteArray& data) {
    if (socket_fd_ == -1) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    int sent = ::send(socket_fd_, (const char*)data.data(), (int)data.size(), 0);
    if (sent == SOCKET_ERROR) {
        last_error_ = "Send failed";
        return Result<void>::failure(ErrorCode::SendFailed, last_error_);
    }
    
    return Result<void>::success();
}

Result<ByteArray> S7Client::receive(uint32_t timeout_ms) {
    if (socket_fd_ == -1) {
        return Result<ByteArray>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    // 设置接收超时
#ifdef _WIN32
    DWORD timeout = timeout_ms;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
    
    // 先读取TPKT头部获取完整消息长度
    uint8_t tpkt_header[4];
    int received = ::recv(socket_fd_, (char*)tpkt_header, 4, 0);
    
    if (received != 4) {
        return Result<ByteArray>::failure(ErrorCode::ReceiveFailed, "Failed to read TPKT header");
    }
    
    // TPKT格式: Version(1) + Reserved(1) + Length(2)
    if (tpkt_header[0] != 3) {
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, "Invalid TPKT version");
    }
    
    uint16_t total_length = (tpkt_header[2] << 8) | tpkt_header[3];
    if (total_length < 4) {
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, "Invalid TPKT length");
    }
    
    // 读取剩余数据
    uint16_t data_length = total_length - 4;
    ByteArray data(data_length);
    
    int total_received = 0;
    while (total_received < data_length) {
        received = ::recv(socket_fd_, (char*)data.data() + total_received, 
                         data_length - total_received, 0);
        if (received <= 0) {
            return Result<ByteArray>::failure(ErrorCode::ReceiveFailed, "Receive failed");
        }
        total_received += received;
    }
    
    // 组合完整响应（包含TPKT头）
    ByteArray full_response(total_length);
    std::memcpy(full_response.data(), tpkt_header, 4);
    std::memcpy(full_response.data() + 4, data.data(), data_length);
    
    return Result<ByteArray>::success(full_response);
}

ByteArray S7Client::build_cotp_connection_request(int rack, int slot) {
    ByteArray request(sizeof(COTP_CONNECT_REQUEST));
    std::memcpy(request.data(), COTP_CONNECT_REQUEST, sizeof(COTP_CONNECT_REQUEST));
    
    // 设置Rack和Slot (Called TSAP)
    // Called TSAP: 设为PG连接类型 (0x01 0x00)
    request[10] = 0x01;
    request[11] = 0x00;
    
    // Calling TSAP: Rack和Slot编码
    // Rack = 0-7, Slot = 0-31 (编码为 0x0100 + (rack << 5) + slot)
    request[14] = 0x01;
    request[15] = static_cast<uint8_t>((rack << 5) | slot);
    
    return request;
}

ByteArray S7Client::build_s7_communication_request() {
    ByteArray request(sizeof(S7_COMM_SETUP));
    std::memcpy(request.data(), S7_COMM_SETUP, sizeof(S7_COMM_SETUP));
    
    // 设置消息序号
    message_number_++;
    request[11] = static_cast<uint8_t>(message_number_ >> 8);
    request[12] = static_cast<uint8_t>(message_number_ & 0xFF);
    
    return request;
}

ByteArray S7Client::build_read_request(S7Area area, int db_number, int start, int size) {
    // S7读取请求结构
    // TPKT(4) + COTP(3) + S7 Header(12) + S7 Parameter(4) + S7 Data(4)
    
    ByteArray request;
    request.resize(31);
    
    // TPKT Header
    request[0] = 0x03;  // Version
    request[1] = 0x00;  // Reserved
    uint16_t length = 31;
    request[2] = static_cast<uint8_t>(length >> 8);
    request[3] = static_cast<uint8_t>(length & 0xFF);
    
    // COTP Header
    request[4] = 0x02;  // Length
    request[5] = 0xF0;  // PDU Type: Data
    request[6] = 0x80;  // EOT=1, TPDU Number=0
    
    // S7 Header
    request[7] = 0x32;  // S7 Protocol ID
    request[8] = 0x01;  // Message Type: Job Request
    request[9] = 0x00;  // Redundancy ID
    request[10] = 0x00;
    message_number_++;
    request[11] = static_cast<uint8_t>(message_number_ >> 8);
    request[12] = static_cast<uint8_t>(message_number_ & 0xFF);
    request[13] = 0x00;  // Parameter Length (4 bytes)
    request[14] = 0x04;
    request[15] = 0x00;  // Data Length (4 bytes)
    request[16] = 0x04;
    
    // S7 Parameter
    request[17] = S7_READ;  // Function: Read
    request[18] = 0x01;     // Item Count: 1
    
    // S7 Item
    request[19] = 0x12;     // Variable Specification: Address
    request[20] = 0x0A;     // Length of following: 10
    request[21] = 0x10;     // Syntax ID: S7ANY
    
    // Area Code
    uint16_t area_code = get_area_code(area, db_number);
    request[22] = static_cast<uint8_t>(area_code >> 8);
    request[23] = static_cast<uint8_t>(area_code & 0xFF);
    
    // DB Number (for DB area)
    request[24] = static_cast<uint8_t>(db_number >> 8);
    request[25] = static_cast<uint8_t>(db_number & 0xFF);
    
    // Start Address (3 bytes, byte offset * 8 + bit offset)
    uint32_t address = start * 8;  // Byte address * 8
    request[26] = static_cast<uint8_t>((address >> 16) & 0xFF);
    request[27] = static_cast<uint8_t>((address >> 8) & 0xFF);
    request[28] = static_cast<uint8_t>(address & 0xFF);
    
    // Length/Count
    request[29] = static_cast<uint8_t>(size >> 8);
    request[30] = static_cast<uint8_t>(size & 0xFF);
    
    return request;
}

ByteArray S7Client::build_write_request(S7Area area, int db_number, int start, const ByteArray& data) {
    // S7写入请求结构
    int data_len = static_cast<int>(data.size());
    int header_len = 35;  // 固定头部长度
    int total_len = header_len + 4 + data_len;  // +4字节数据项头部
    
    ByteArray request;
    request.resize(total_len);
    
    // TPKT Header
    request[0] = 0x03;
    request[1] = 0x00;
    request[2] = static_cast<uint8_t>(total_len >> 8);
    request[3] = static_cast<uint8_t>(total_len & 0xFF);
    
    // COTP Header
    request[4] = 0x02;
    request[5] = 0xF0;
    request[6] = 0x80;
    
    // S7 Header
    request[7] = 0x32;
    request[8] = 0x01;
    request[9] = 0x00;
    request[10] = 0x00;
    message_number_++;
    request[11] = static_cast<uint8_t>(message_number_ >> 8);
    request[12] = static_cast<uint8_t>(message_number_ & 0xFF);
    request[13] = 0x00;  // Parameter Length: 4
    request[14] = 0x04;
    int data_field_len = 4 + 4 + data_len;
    request[15] = static_cast<uint8_t>(data_field_len >> 8);
    request[16] = static_cast<uint8_t>(data_field_len & 0xFF);
    
    // S7 Parameter
    request[17] = S7_WRITE;
    request[18] = 0x01;
    
    // S7 Item (Address)
    request[19] = 0x12;
    request[20] = 0x0A;
    request[21] = 0x10;
    
    uint16_t area_code = get_area_code(area, db_number);
    request[22] = static_cast<uint8_t>(area_code >> 8);
    request[23] = static_cast<uint8_t>(area_code & 0xFF);
    request[24] = static_cast<uint8_t>(db_number >> 8);
    request[25] = static_cast<uint8_t>(db_number & 0xFF);
    
    uint32_t address = start * 8;
    request[26] = static_cast<uint8_t>((address >> 16) & 0xFF);
    request[27] = static_cast<uint8_t>((address >> 8) & 0xFF);
    request[28] = static_cast<uint8_t>(address & 0xFF);
    
    request[29] = static_cast<uint8_t>(data_len >> 8);
    request[30] = static_cast<uint8_t>(data_len & 0xFF);
    
    // S7 Data Item
    request[31] = 0x00;  // Return Code
    request[32] = 0x00;  // Transport Size: byte/word
    request[33] = static_cast<uint8_t>((data_len + 1) >> 8);  // Data Length (rounded)
    request[34] = static_cast<uint8_t>((data_len + 1) & 0xFF);
    
    // 填充字节
    request[35] = 0x00;
    
    // 实际数据
    if (!data.empty()) {
        std::memcpy(&request[36], data.data(), data_len);
    }
    
    return request;
}

ByteArray S7Client::build_control_request(uint8_t function) {
    ByteArray request(19);
    
    // TPKT Header
    request[0] = 0x03;
    request[1] = 0x00;
    request[2] = 0x00;
    request[3] = 0x13;  // Length: 19
    
    // COTP Header
    request[4] = 0x02;
    request[5] = 0xF0;
    request[6] = 0x80;
    
    // S7 Header
    request[7] = 0x32;
    request[8] = 0x01;
    request[9] = 0x00;
    request[10] = 0x00;
    message_number_++;
    request[11] = static_cast<uint8_t>(message_number_ >> 8);
    request[12] = static_cast<uint8_t>(message_number_ & 0xFF);
    request[13] = 0x00;  // Parameter Length: 2
    request[14] = 0x02;
    request[15] = 0x00;  // Data Length: 0
    request[16] = 0x00;
    
    // S7 Parameter
    request[17] = function;  // Control Function (PLC Start/Stop etc)
    request[18] = 0x00;      // Reserved
    
    return request;
}

uint16_t S7Client::get_area_code(S7Area area, int db_number) {
    switch (area) {
        case S7Area::PE: return 0x81;   // Process Image Input
        case S7Area::PA: return 0x82;   // Process Image Output
        case S7Area::MK: return 0x83;   // Markers (Memory)
        case S7Area::DB: return static_cast<uint16_t>(0x84 | (db_number & 0xFF));  // Data Block
        case S7Area::CT: return 0x1C;   // Counters
        case S7Area::TM: return 0x1D;   // Timers
        default: return 0x84;           // Default: DB
    }
}

Result<void> S7Client::parse_cotp_connection_response(const ByteArray& response) {
    if (response.size() < 8) {
        return Result<void>::failure(ErrorCode::ProtocolError, "Invalid COTP response length");
    }
    
    // TPKT Header检查
    if (response[0] != 3) {
        return Result<void>::failure(ErrorCode::ProtocolError, "Invalid TPKT version");
    }
    
    // COTP PDU Type检查 (Connect Confirm = 0xD0)
    uint8_t cotp_length = response[4];
    uint8_t pdu_type = response[5];
    
    if (pdu_type != 0xD0) {  // Connect Confirm
        return Result<void>::failure(ErrorCode::ProtocolError, "COTP connection refused");
    }
    
    return Result<void>::success();
}

Result<void> S7Client::parse_s7_communication_response(const ByteArray& response) {
    if (response.size() < 20) {
        return Result<void>::failure(ErrorCode::ProtocolError, "Invalid S7 setup response length");
    }
    
    // S7 Header检查
    uint8_t s7_protocol_id = response[7];
    uint8_t message_type = response[8];
    
    if (s7_protocol_id != 0x32) {
        return Result<void>::failure(ErrorCode::ProtocolError, "Invalid S7 protocol ID");
    }
    
    // Message Type: Ack (0x03) or Ack Data (0x07)
    if (message_type != 0x03 && message_type != 0x07) {
        return Result<void>::failure(ErrorCode::ProtocolError, "S7 setup failed");
    }
    
    // 检查错误码
    uint8_t error_class = response[17];
    uint8_t error_code = response[18];
    
    if (error_class != 0 || error_code != 0) {
        return Result<void>::failure(ErrorCode::ProtocolError, 
            "S7 error: class=" + std::to_string(error_class) + ", code=" + std::to_string(error_code));
    }
    
    // 获取协商的PDU大小
    if (response.size() >= 30) {
        pdu_size_ = (response[29] << 8) | response[30];
    }
    
    return Result<void>::success();
}

Result<ByteArray> S7Client::parse_read_response(const ByteArray& response) {
    if (response.size() < 25) {
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, "Read response too short");
    }
    
    // 检查S7 Header
    uint8_t message_type = response[8];
    uint8_t error_class = response[10];
    uint8_t error_code = response[11];
    
    if (message_type != 0x03 && message_type != 0x07) {
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, "Invalid read response type");
    }
    
    if (error_class != 0 || error_code != 0) {
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, 
            "Read error: class=" + std::to_string(error_class) + ", code=" + std::to_string(error_code));
    }
    
    // 检查返回码
    uint8_t return_code = response[21];
    if (return_code != 0xFF) {  // Success = 0xFF
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, 
            "Read failed: return code=" + std::to_string(return_code));
    }
    
    // 获取数据长度
    uint16_t data_length = (response[23] << 8) | response[24];
    
    // 提取数据
    if (response.size() < 25 + data_length) {
        return Result<ByteArray>::failure(ErrorCode::ProtocolError, "Read response data truncated");
    }
    
    ByteArray data(data_length - 1);  // 减去填充字节
    std::memcpy(data.data(), &response[26], data_length - 1);
    
    return Result<ByteArray>::success(data);
}

Result<void> S7Client::parse_write_response(const ByteArray& response) {
    if (response.size() < 22) {
        return Result<void>::failure(ErrorCode::ProtocolError, "Write response too short");
    }
    
    // 检查S7 Header
    uint8_t message_type = response[8];
    
    if (message_type != 0x03) {
        return Result<void>::failure(ErrorCode::ProtocolError, "Invalid write response type");
    }
    
    // 检查返回码
    uint8_t return_code = response[21];
    if (return_code != 0xFF) {
        return Result<void>::failure(ErrorCode::ProtocolError, 
            "Write failed: return code=" + std::to_string(return_code));
    }
    
    return Result<void>::success();
}

Result<ByteArray> S7Client::send_s7_request(const ByteArray& request) {
    auto send_result = send(request);
    if (send_result.is_failure()) {
        return Result<ByteArray>::failure(send_result.code(), send_result.message());
    }
    
    return receive(2000);
}

// ============== 读取操作 ==============

Result<ByteArray> S7Client::read_bytes(S7Area area, int db_number, int start, int size) {
    if (!is_connected()) {
        return Result<ByteArray>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    // 分块读取（PDU限制）
    ByteArray result;
    int remaining = size;
    int offset = start;
    
    while (remaining > 0) {
        int chunk_size = (std::min)(remaining, static_cast<int>(pdu_size_) - 20);
        
        ByteArray request = build_read_request(area, db_number, offset, chunk_size);
        auto response = send_s7_request(request);
        
        if (response.is_failure()) {
            return Result<ByteArray>::failure(response.code(), response.message());
        }
        
        auto data = parse_read_response(response.value());
        if (data.is_failure()) {
            return Result<ByteArray>::failure(data.code(), data.message());
        }
        
        // 合并数据
        result.insert(result.end(), data.value().begin(), data.value().end());
        
        offset += chunk_size;
        remaining -= chunk_size;
    }
    
    return Result<ByteArray>::success(result);
}

Result<int32_t> S7Client::read_int(S7Area area, int db_number, int start) {
    auto data = read_bytes(area, db_number, start, 4);
    if (data.is_failure()) {
        return Result<int32_t>::failure(data.code(), data.message());
    }
    
    const ByteArray& bytes = data.value();
    if (bytes.size() < 4) {
        return Result<int32_t>::failure(ErrorCode::ProtocolError, "Insufficient data for int");
    }
    
    // S7使用大端序
    int32_t value = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    return Result<int32_t>::success(value);
}

Result<float> S7Client::read_float(S7Area area, int db_number, int start) {
    auto data = read_bytes(area, db_number, start, 4);
    if (data.is_failure()) {
        return Result<float>::failure(data.code(), data.message());
    }
    
    const ByteArray& bytes = data.value();
    if (bytes.size() < 4) {
        return Result<float>::failure(ErrorCode::ProtocolError, "Insufficient data for float");
    }
    
    // IEEE 754 大端序
    uint32_t raw = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    float value;
    std::memcpy(&value, &raw, sizeof(float));
    
    return Result<float>::success(value);
}

Result<bool> S7Client::read_bit(S7Area area, int db_number, int byte_offset, int bit_offset) {
    auto data = read_bytes(area, db_number, byte_offset, 1);
    if (data.is_failure()) {
        return Result<bool>::failure(data.code(), data.message());
    }
    
    const ByteArray& bytes = data.value();
    if (bytes.empty()) {
        return Result<bool>::failure(ErrorCode::ProtocolError, "Insufficient data for bit");
    }
    
    bool value = (bytes[0] & (1 << bit_offset)) != 0;
    return Result<bool>::success(value);
}

// ============== 写入操作 ==============

Result<void> S7Client::write_bytes(S7Area area, int db_number, int start, const ByteArray& data) {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    // 分块写入（PDU限制）
    int remaining = static_cast<int>(data.size());
    int offset = start;
    int data_offset = 0;
    
    while (remaining > 0) {
        int chunk_size = (std::min)(remaining, static_cast<int>(pdu_size_) - 40);
        
        ByteArray chunk(data.begin() + data_offset, data.begin() + data_offset + chunk_size);
        ByteArray request = build_write_request(area, db_number, offset, chunk);
        
        auto response = send_s7_request(request);
        if (response.is_failure()) {
            return Result<void>::failure(response.code(), response.message());
        }
        
        auto parse_result = parse_write_response(response.value());
        if (parse_result.is_failure()) {
            return parse_result;
        }
        
        offset += chunk_size;
        data_offset += chunk_size;
        remaining -= chunk_size;
    }
    
    return Result<void>::success();
}

Result<void> S7Client::write_int(S7Area area, int db_number, int start, int32_t value) {
    ByteArray data(4);
    // S7使用大端序
    data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(value & 0xFF);
    
    return write_bytes(area, db_number, start, data);
}

Result<void> S7Client::write_float(S7Area area, int db_number, int start, float value) {
    ByteArray data(4);
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(float));
    
    // IEEE 754 大端序
    data[0] = static_cast<uint8_t>((raw >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(raw & 0xFF);
    
    return write_bytes(area, db_number, start, data);
}

Result<void> S7Client::write_bit(S7Area area, int db_number, int byte_offset, int bit_offset, bool value) {
    // 先读取当前字节
    auto read_result = read_bytes(area, db_number, byte_offset, 1);
    if (read_result.is_failure()) {
        return Result<void>::failure(read_result.code(), read_result.message());
    }
    
    uint8_t current = read_result.value()[0];
    
    // 修改位
    if (value) {
        current |= (1 << bit_offset);
    } else {
        current &= ~(1 << bit_offset);
    }
    
    ByteArray data(1);
    data[0] = current;
    
    return write_bytes(area, db_number, byte_offset, data);
}

// ============== PLC控制 ==============

Result<void> S7Client::plc_start() {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    ByteArray request = build_control_request(S7_CONTROL);
    request[17] = 0x28;  // PLC Start Function
    
    auto response = send_s7_request(request);
    if (response.is_failure()) {
        return Result<void>::failure(response.code(), response.message());
    }
    
    // 检查响应
    if (response.value().size() < 19) {
        return Result<void>::failure(ErrorCode::ProtocolError, "PLC start response invalid");
    }
    
    uint8_t message_type = response.value()[8];
    if (message_type != 0x03) {
        return Result<void>::failure(ErrorCode::ProtocolError, "PLC start failed");
    }
    
    OVF_INFO() << "PLC Start command sent";
    return Result<void>::success();
}

Result<void> S7Client::plc_stop() {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    ByteArray request = build_control_request(S7_CONTROL);
    request[17] = 0x29;  // PLC Stop Function
    
    auto response = send_s7_request(request);
    if (response.is_failure()) {
        return Result<void>::failure(response.code(), response.message());
    }
    
    if (response.value().size() < 19) {
        return Result<void>::failure(ErrorCode::ProtocolError, "PLC stop response invalid");
    }
    
    uint8_t message_type = response.value()[8];
    if (message_type != 0x03) {
        return Result<void>::failure(ErrorCode::ProtocolError, "PLC stop failed");
    }
    
    OVF_INFO() << "PLC Stop command sent";
    return Result<void>::success();
}

Result<void> S7Client::plc_hot_start() {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    ByteArray request = build_control_request(S7_CONTROL);
    request[17] = 0x2A;  // PLC Hot Start Function
    
    auto response = send_s7_request(request);
    if (response.is_failure()) {
        return Result<void>::failure(response.code(), response.message());
    }
    
    if (response.value().size() < 19) {
        return Result<void>::failure(ErrorCode::ProtocolError, "PLC hot start response invalid");
    }
    
    uint8_t message_type = response.value()[8];
    if (message_type != 0x03) {
        return Result<void>::failure(ErrorCode::ProtocolError, "PLC hot start failed");
    }
    
    OVF_INFO() << "PLC Hot Start command sent";
    return Result<void>::success();
}

Result<String> S7Client::get_plc_info() {
    if (!is_connected()) {
        return Result<String>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    // 简化的PLC信息获取
    String info = "PLC Info:\n";
    info += "  IP: " + ip_ + "\n";
    info += "  Rack: " + std::to_string(rack_) + "\n";
    info += "  Slot: " + std::to_string(slot_) + "\n";
    
    String type_str;
    switch (plc_type_) {
        case S7PLCType::S7_200: type_str = "S7-200"; break;
        case S7PLCType::S7_300: type_str = "S7-300"; break;
        case S7PLCType::S7_400: type_str = "S7-400"; break;
        case S7PLCType::S7_1200: type_str = "S7-1200"; break;
        case S7PLCType::S7_1500: type_str = "S7-1500"; break;
        default: type_str = "Unknown"; break;
    }
    info += "  Type: " + type_str + "\n";
    info += "  PDU Size: " + std::to_string(pdu_size_);
    
    return Result<String>::success(info);
}

} // namespace comm
} // namespace ovf