/**
 * @file communication.cpp
 * @brief 通信服务层实现
 */

#include "ovf/comm/communication.h"

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using ovf_socklen_t = int;  // Winsock 的 accept/recvfrom 长度参数是 int*
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>    // open/O_RDWR/O_NOCTTY —— 串口分支需要
    #include <termios.h>  // termios/tcgetattr/tcsetattr/TCSANOW/CLOCAL/CREAD
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
    using ovf_socklen_t = socklen_t;
#endif

#include <cstring>
#include <chrono>

namespace ovf {
namespace comm {

// Windows Socket初始化
static bool winsock_initialized = false;

static void init_winsock() {
#ifdef _WIN32
    if (!winsock_initialized) {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        winsock_initialized = true;
    }
#endif
}

static void cleanup_winsock() {
#ifdef _WIN32
    if (winsock_initialized) {
        WSACleanup();
        winsock_initialized = false;
    }
#endif
}

// ============== TcpClient ==============

TcpClient::TcpClient(const CommDeviceInfo& info) : info_(info) {
    init_winsock();
}

TcpClient::~TcpClient() {
    disconnect();
}

Result<void> TcpClient::connect() {
    if (is_connected()) {
        return Result<void>::success();
    }
    
    state_ = ConnectionState::Connecting;
    
    // 创建Socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == INVALID_SOCKET) {
        state_ = ConnectionState::Error;
        last_error_ = "Failed to create socket";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    // 设置服务器地址
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(info_.port);
    
#ifdef _WIN32
    inet_pton(AF_INET, info_.address.c_str(), &server_addr.sin_addr);
#else
    server_addr.sin_addr.s_addr = inet_addr(info_.address.c_str());
#endif
    
    // 连接
    if (::connect(socket_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
        state_ = ConnectionState::Error;
        last_error_ = "Failed to connect to server";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    state_ = ConnectionState::Connected;
    
    // 启动接收线程
    running_ = true;
    receive_thread_ = std::thread(&TcpClient::receive_thread, this);
    
    OVF_INFO() << "TcpClient connected to " << info_.address << ":" << info_.port;
    return Result<void>::success();
}

Result<void> TcpClient::disconnect() {
    if (!is_connected() && socket_fd_ == -1) {
        return Result<void>::success();
    }
    
    running_ = false;
    
    if (socket_fd_ != -1) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
    }
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    state_ = ConnectionState::Disconnected;
    OVF_INFO() << "TcpClient disconnected";
    return Result<void>::success();
}

Result<void> TcpClient::send(const ByteArray& data) {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    int sent = ::send(socket_fd_, (const char*)data.data(), (int)data.size(), 0);
    if (sent == SOCKET_ERROR) {
        last_error_ = "Send failed";
        return Result<void>::failure(ErrorCode::SendFailed, last_error_);
    }
    
    return Result<void>::success();
}

Result<void> TcpClient::send_string(const String& data) {
    ByteArray bytes(data.size());
    std::memcpy(bytes.data(), data.data(), data.size());
    return send(bytes);
}

Result<ByteArray> TcpClient::receive(uint32_t timeout_ms) {
    if (!is_connected()) {
        return Result<ByteArray>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (receive_queue_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    }
    
    if (receive_queue_.empty()) {
        return Result<ByteArray>::failure(ErrorCode::Timeout, "Receive timeout");
    }
    
    ByteArray data = receive_queue_.front();
    receive_queue_.pop();
    return Result<ByteArray>::success(data);
}

Result<String> TcpClient::receive_string(uint32_t timeout_ms) {
    auto result = receive(timeout_ms);
    if (result.is_failure()) {
        return Result<String>::failure(result.code(), result.message());
    }
    
    String str((char*)result.value().data(), result.value().size());
    return Result<String>::success(str);
}

void TcpClient::set_data_callback(DataCallback callback) {
    data_callback_ = callback;
}

void TcpClient::receive_thread() {
    constexpr int BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    
    while (running_ && socket_fd_ != -1) {
        int received = ::recv(socket_fd_, buffer, BUFFER_SIZE, 0);
        
        if (received > 0) {
            ByteArray data(received);
            std::memcpy(data.data(), buffer, received);
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                receive_queue_.push(data);
            }
            cv_.notify_one();
            
            if (data_callback_) {
                data_callback_(data);
            }
        }
        else if (received == 0) {
            // 连接关闭
            state_ = ConnectionState::Disconnected;
            running_ = false;
            break;
        }
        else {
            // 错误
            state_ = ConnectionState::Error;
            running_ = false;
            break;
        }
    }
}

// ============== TcpServer ==============

TcpServer::TcpServer(const CommDeviceInfo& info) : info_(info) {
    init_winsock();
}

TcpServer::~TcpServer() {
    disconnect();
}

Result<void> TcpServer::connect() {
    if (is_connected()) {
        return Result<void>::success();
    }
    
    state_ = ConnectionState::Connecting;
    
    // 创建Socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ == INVALID_SOCKET) {
        state_ = ConnectionState::Error;
        last_error_ = "Failed to create socket";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    // 绑定地址
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(info_.port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(server_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        closesocket(server_fd_);
        server_fd_ = -1;
        state_ = ConnectionState::Error;
        last_error_ = "Failed to bind socket";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    // 监听
    if (listen(server_fd_, 5) == SOCKET_ERROR) {
        closesocket(server_fd_);
        server_fd_ = -1;
        state_ = ConnectionState::Error;
        last_error_ = "Failed to listen";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    state_ = ConnectionState::Connected;
    
    // 启动服务器线程
    running_ = true;
    server_thread_ = std::thread(&TcpServer::server_thread, this);
    
    OVF_INFO() << "TcpServer listening on port " << info_.port;
    return Result<void>::success();
}

Result<void> TcpServer::disconnect() {
    running_ = false;
    
    // 关闭所有客户端
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int fd : client_fds_) {
            closesocket(fd);
        }
        client_fds_.clear();
    }
    
    if (server_fd_ != -1) {
        closesocket(server_fd_);
        server_fd_ = -1;
    }
    
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    
    state_ = ConnectionState::Disconnected;
    return Result<void>::success();
}

Result<void> TcpServer::send(const ByteArray& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int fd : client_fds_) {
        ::send(fd, (const char*)data.data(), (int)data.size(), 0);
    }
    return Result<void>::success();
}

Result<void> TcpServer::send_string(const String& data) {
    ByteArray bytes(data.size());
    std::memcpy(bytes.data(), data.data(), data.size());
    return send(bytes);
}

Result<ByteArray> TcpServer::receive(uint32_t timeout_ms) {
    return Result<ByteArray>::failure(ErrorCode::NotSupported, "Server receive requires client-specific handling");
}

Result<String> TcpServer::receive_string(uint32_t timeout_ms) {
    return Result<String>::failure(ErrorCode::NotSupported, "Server receive requires client-specific handling");
}

void TcpServer::set_data_callback(DataCallback callback) {
    data_callback_ = callback;
}

void TcpServer::set_client_callback(ClientCallback callback) {
    client_callback_ = callback;
}

Vector<int> TcpServer::get_clients() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return client_fds_;
}

Result<void> TcpServer::send_to_client(int client_fd, const ByteArray& data) {
    int sent = ::send(client_fd, (const char*)data.data(), (int)data.size(), 0);
    if (sent == SOCKET_ERROR) {
        return Result<void>::failure(ErrorCode::SendFailed, "Send to client failed");
    }
    return Result<void>::success();
}

Result<void> TcpServer::close_client(int client_fd) {
    std::lock_guard<std::mutex> lock(mutex_);
    closesocket(client_fd);
    client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), client_fd), client_fds_.end());
    return Result<void>::success();
}

void TcpServer::server_thread() {
    while (running_ && server_fd_ != -1) {
        sockaddr_in client_addr;
        ovf_socklen_t addr_len = sizeof(client_addr);
        
        SOCKET client_fd = accept(server_fd_, (sockaddr*)&client_addr, &addr_len);
        if (client_fd == INVALID_SOCKET) {
            continue;
        }
        
        // 获取客户端地址
        String client_addr_str;
#ifdef _WIN32
        char addr_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, addr_buf, INET_ADDRSTRLEN);
        client_addr_str = String(addr_buf);
#else
        client_addr_str = inet_ntoa(client_addr.sin_addr);
#endif
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            client_fds_.push_back(client_fd);
        }
        
        if (client_callback_) {
            client_callback_(client_fd, client_addr_str);
        }
        
        // 启动客户端接收线程
        std::thread(&TcpServer::client_thread, this, client_fd, client_addr_str).detach();
        
        OVF_INFO() << "New client connected: " << client_addr_str;
    }
}

void TcpServer::client_thread(int client_fd, String client_addr) {
    constexpr int BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    
    while (running_) {
        int received = ::recv(client_fd, buffer, BUFFER_SIZE, 0);
        
        if (received > 0) {
            ByteArray data(received);
            std::memcpy(data.data(), buffer, received);
            
            if (data_callback_) {
                data_callback_(data);
            }
        }
        else {
            // 客户端断开
            {
                std::lock_guard<std::mutex> lock(mutex_);
                client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), client_fd), client_fds_.end());
            }
            closesocket(client_fd);
            OVF_INFO() << "Client disconnected: " << client_addr;
            break;
        }
    }
}

// ============== UdpSocket ==============

UdpSocket::UdpSocket(const CommDeviceInfo& info) : info_(info) {
    init_winsock();
}

UdpSocket::~UdpSocket() {
    disconnect();
}

Result<void> UdpSocket::connect() {
    if (is_connected()) {
        return Result<void>::success();
    }
    
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ == INVALID_SOCKET) {
        state_ = ConnectionState::Error;
        last_error_ = "Failed to create socket";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(info_.port);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(socket_fd_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
        state_ = ConnectionState::Error;
        last_error_ = "Failed to bind socket";
        return Result<void>::failure(ErrorCode::ConnectionFailed, last_error_);
    }
    
    state_ = ConnectionState::Connected;
    running_ = true;
    receive_thread_ = std::thread(&UdpSocket::receive_thread, this);
    
    return Result<void>::success();
}

Result<void> UdpSocket::disconnect() {
    running_ = false;
    
    if (socket_fd_ != -1) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
    }
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    state_ = ConnectionState::Disconnected;
    return Result<void>::success();
}

Result<void> UdpSocket::send(const ByteArray& data) {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(info_.port);
    
#ifdef _WIN32
    inet_pton(AF_INET, info_.address.c_str(), &dest_addr.sin_addr);
#else
    dest_addr.sin_addr.s_addr = inet_addr(info_.address.c_str());
#endif
    
    int sent = sendto(socket_fd_, (const char*)data.data(), (int)data.size(), 0,
                      (sockaddr*)&dest_addr, sizeof(dest_addr));
    
    if (sent == SOCKET_ERROR) {
        return Result<void>::failure(ErrorCode::SendFailed, "Sendto failed");
    }
    
    return Result<void>::success();
}

Result<void> UdpSocket::send_string(const String& data) {
    ByteArray bytes(data.size());
    std::memcpy(bytes.data(), data.data(), data.size());
    return send(bytes);
}

Result<ByteArray> UdpSocket::receive(uint32_t timeout_ms) {
    if (!is_connected()) {
        return Result<ByteArray>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (receive_queue_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    }
    
    if (receive_queue_.empty()) {
        return Result<ByteArray>::failure(ErrorCode::Timeout, "Receive timeout");
    }
    
    ByteArray data = receive_queue_.front();
    receive_queue_.pop();
    return Result<ByteArray>::success(data);
}

Result<String> UdpSocket::receive_string(uint32_t timeout_ms) {
    auto result = receive(timeout_ms);
    if (result.is_failure()) {
        return Result<String>::failure(result.code(), result.message());
    }
    
    String str((char*)result.value().data(), result.value().size());
    return Result<String>::success(str);
}

void UdpSocket::set_data_callback(DataCallback callback) {
    data_callback_ = callback;
}

void UdpSocket::receive_thread() {
    constexpr int BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    sockaddr_in from_addr;
    ovf_socklen_t from_len = sizeof(from_addr);
    
    while (running_ && socket_fd_ != -1) {
        int received = recvfrom(socket_fd_, buffer, BUFFER_SIZE, 0,
                                (sockaddr*)&from_addr, &from_len);
        
        if (received > 0) {
            ByteArray data(received);
            std::memcpy(data.data(), buffer, received);
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                receive_queue_.push(data);
            }
            cv_.notify_one();
            
            if (data_callback_) {
                data_callback_(data);
            }
        }
    }
}

// ============== SerialPort ==============

SerialPort::SerialPort(const CommDeviceInfo& info)
    : SerialPort(info, SerialConfig{}) {}

SerialPort::SerialPort(const CommDeviceInfo& info, const SerialConfig& config)
    : info_(info), config_(config) {}

SerialPort::~SerialPort() {
    disconnect();
}

Result<void> SerialPort::connect() {
#ifdef _WIN32
    String port_path = "\\\\.\\COM" + info_.address;
    handle_ = CreateFileA(port_path.c_str(),
                          GENERIC_READ | GENERIC_WRITE,
                          0, nullptr, OPEN_EXISTING,
                          FILE_ATTRIBUTE_NORMAL, nullptr);
    
    if (handle_ == INVALID_HANDLE_VALUE) {
        state_ = ConnectionState::Error;
        last_error_ = "Failed to open serial port";
        return Result<void>::failure(ErrorCode::DeviceOpenFailed, last_error_);
    }
    
    // 配置串口参数
    DCB dcb;
    GetCommState(handle_, &dcb);
    dcb.BaudRate = config_.baud_rate;
    dcb.ByteSize = config_.data_bits;
    dcb.StopBits = config_.stop_bits == 1 ? ONESTOPBIT : TWOSTOPBITS;
    dcb.Parity = config_.parity == 0 ? NOPARITY : (config_.parity == 1 ? ODDPARITY : EVENPARITY);
    SetCommState(handle_, &dcb);
    
    // 设置超时
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts(handle_, &timeouts);
    
    state_ = ConnectionState::Connected;
    running_ = true;
    receive_thread_ = std::thread(&SerialPort::receive_thread, this);
    
    OVF_INFO() << "SerialPort opened: COM" << info_.address;
    return Result<void>::success();
#else
    // Linux实现
    String port_path = "/dev/tty" + info_.address;
    fd_ = open(port_path.c_str(), O_RDWR | O_NOCTTY);
    
    if (fd_ < 0) {
        state_ = ConnectionState::Error;
        last_error_ = "Failed to open serial port";
        return Result<void>::failure(ErrorCode::DeviceOpenFailed, last_error_);
    }
    
    struct termios options;
    tcgetattr(fd_, &options);
    cfsetispeed(&options, config_.baud_rate);
    cfsetospeed(&options, config_.baud_rate);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &options);
    
    state_ = ConnectionState::Connected;
    running_ = true;
    receive_thread_ = std::thread(&SerialPort::receive_thread, this);
    
    return Result<void>::success();
#endif
}

Result<void> SerialPort::disconnect() {
    running_ = false;
    
#ifdef _WIN32
    if (handle_ != nullptr && handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(handle_);
        handle_ = nullptr;
    }
#else
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
#endif
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    state_ = ConnectionState::Disconnected;
    return Result<void>::success();
}

Result<void> SerialPort::send(const ByteArray& data) {
    if (!is_connected()) {
        return Result<void>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
#ifdef _WIN32
    DWORD written;
    if (!WriteFile(handle_, data.data(), (DWORD)data.size(), &written, nullptr)) {
        return Result<void>::failure(ErrorCode::SendFailed, "Write failed");
    }
#else
    if (write(fd_, data.data(), data.size()) < 0) {
        return Result<void>::failure(ErrorCode::SendFailed, "Write failed");
    }
#endif
    
    return Result<void>::success();
}

Result<void> SerialPort::send_string(const String& data) {
    ByteArray bytes(data.size());
    std::memcpy(bytes.data(), data.data(), data.size());
    return send(bytes);
}

Result<ByteArray> SerialPort::receive(uint32_t timeout_ms) {
    if (!is_connected()) {
        return Result<ByteArray>::failure(ErrorCode::ConnectionLost, "Not connected");
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (receive_queue_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    }
    
    if (receive_queue_.empty()) {
        return Result<ByteArray>::failure(ErrorCode::Timeout, "Receive timeout");
    }
    
    ByteArray data = receive_queue_.front();
    receive_queue_.pop();
    return Result<ByteArray>::success(data);
}

Result<String> SerialPort::receive_string(uint32_t timeout_ms) {
    auto result = receive(timeout_ms);
    if (result.is_failure()) {
        return Result<String>::failure(result.code(), result.message());
    }
    
    String str((char*)result.value().data(), result.value().size());
    return Result<String>::success(str);
}

void SerialPort::set_data_callback(DataCallback callback) {
    data_callback_ = callback;
}

Result<void> SerialPort::set_config(const SerialConfig& config) {
    config_ = config;
    if (is_connected()) {
        // 重新配置串口
#ifdef _WIN32
        DCB dcb;
        GetCommState(handle_, &dcb);
        dcb.BaudRate = config_.baud_rate;
        dcb.ByteSize = config_.data_bits;
        dcb.StopBits = config_.stop_bits == 1 ? ONESTOPBIT : TWOSTOPBITS;
        dcb.Parity = config_.parity == 0 ? NOPARITY : (config_.parity == 1 ? ODDPARITY : EVENPARITY);
        SetCommState(handle_, &dcb);
#endif
    }
    return Result<void>::success();
}

void SerialPort::receive_thread() {
    constexpr int BUFFER_SIZE = 4096;
    uint8_t buffer[BUFFER_SIZE];
    
    while (running_) {
#ifdef _WIN32
        DWORD read_count;
        if (ReadFile(handle_, buffer, BUFFER_SIZE, &read_count, nullptr) && read_count > 0) {
#else
        int read_count = read(fd_, buffer, BUFFER_SIZE);
        if (read_count > 0) {
#endif
            ByteArray data(read_count);
            std::memcpy(data.data(), buffer, read_count);
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                receive_queue_.push(data);
            }
            cv_.notify_one();
            
            if (data_callback_) {
                data_callback_(data);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ============== ModbusClient ==============

ModbusClient::ModbusClient(const CommDeviceInfo& info, ModbusType type)
    : info_(info), type_(type) {
    if (type == ModbusType::TCP) {
        tcp_client_ = std::make_shared<TcpClient>(info);
    }
}

ModbusClient::~ModbusClient() {
    disconnect();
}

Result<void> ModbusClient::connect() {
    if (type_ == ModbusType::TCP) {
        auto result = tcp_client_->connect();
        if (result.is_success()) {
            state_ = ConnectionState::Connected;
        }
        return result;
    }
    else {
        // RTU通过串口
        CommDeviceInfo serial_info;
        serial_info.address = info_.address;
        serial_port_ = std::make_shared<SerialPort>(serial_info);
        auto result = serial_port_->connect();
        if (result.is_success()) {
            state_ = ConnectionState::Connected;
        }
        return result;
    }
}

Result<void> ModbusClient::disconnect() {
    if (type_ == ModbusType::TCP && tcp_client_) {
        tcp_client_->disconnect();
    }
    if (serial_port_) {
        serial_port_->disconnect();
    }
    state_ = ConnectionState::Disconnected;
    return Result<void>::success();
}

Result<void> ModbusClient::send(const ByteArray& data) {
    if (type_ == ModbusType::TCP) {
        return tcp_client_->send(data);
    }
    return serial_port_->send(data);
}

Result<ByteArray> ModbusClient::receive(uint32_t timeout_ms) {
    if (type_ == ModbusType::TCP) {
        return tcp_client_->receive(timeout_ms);
    }
    return serial_port_->receive(timeout_ms);
}

ByteArray ModbusClient::build_request(uint8_t unit_id, uint8_t function, 
                                       uint16_t address, uint16_t count, 
                                       const ByteArray& data) {
    ByteArray request;
    
    if (type_ == ModbusType::TCP) {
        // Modbus TCP ADU
        request.resize(6 + 2 + data.size());
        
        // Transaction ID
        request[0] = (transaction_id_ >> 8) & 0xFF;
        request[1] = transaction_id_ & 0xFF;
        transaction_id_++;
        
        // Protocol ID (0 for Modbus)
        request[2] = 0;
        request[3] = 0;
        
        // Length
        uint16_t length = 2 + data.size();
        request[4] = (length >> 8) & 0xFF;
        request[5] = length & 0xFF;
        
        // Unit ID
        request[6] = unit_id;
        
        // Function code
        request[7] = function;
        
        // Data (address, count, values)
        if (!data.empty()) {
            std::memcpy(&request[8], data.data(), data.size());
        }
    }
    else {
        // Modbus RTU (需要CRC)
        request.resize(1 + 1 + data.size() + 2);
        request[0] = unit_id;
        request[1] = function;
        if (!data.empty()) {
            std::memcpy(&request[2], data.data(), data.size());
        }
        
        // CRC计算（简化）
        uint16_t crc = 0;
        for (size_t i = 0; i < request.size() - 2; i++) {
            crc += request[i];
        }
        request[request.size() - 2] = crc & 0xFF;
        request[request.size() - 1] = (crc >> 8) & 0xFF;
    }
    
    return request;
}

Result<ByteArray> ModbusClient::send_request(uint8_t unit_id, const ByteArray& request) {
    auto send_result = send(request);
    if (send_result.is_failure()) {
        return Result<ByteArray>::failure(send_result.code(), send_result.message());
    }
    
    return receive(1000);
}

Result<uint16_t> ModbusClient::read_holding_register(uint8_t unit_id, uint16_t address) {
    ByteArray data(4);
    data[0] = (address >> 8) & 0xFF;
    data[1] = address & 0xFF;
    data[2] = 0;
    data[3] = 1;  // Count = 1
    
    ByteArray request = build_request(unit_id, 0x03, address, 1, data);
    auto response = send_request(unit_id, request);
    
    if (response.is_failure()) {
        return Result<uint16_t>::failure(response.code(), response.message());
    }
    
    // 解析响应
    const auto& resp = response.value();
    if (resp.size() < 9) {
        return Result<uint16_t>::failure(ErrorCode::DataParseError, "Invalid response length");
    }
    
    uint16_t value = (resp[8] << 8) | resp[9];
    return Result<uint16_t>::success(value);
}

Result<Vector<uint16_t>> ModbusClient::read_holding_registers(uint8_t unit_id, 
                                                               uint16_t address, 
                                                               uint16_t count) {
    ByteArray data(4);
    data[0] = (address >> 8) & 0xFF;
    data[1] = address & 0xFF;
    data[2] = (count >> 8) & 0xFF;
    data[3] = count & 0xFF;
    
    ByteArray request = build_request(unit_id, 0x03, address, count, data);
    auto response = send_request(unit_id, request);
    
    if (response.is_failure()) {
        return Result<Vector<uint16_t>>::failure(response.code(), response.message());
    }
    
    const auto& resp = response.value();
    uint8_t byte_count = resp[8];
    
    Vector<uint16_t> values;
    for (uint8_t i = 0; i < byte_count / 2; i++) {
        uint16_t value = (resp[9 + i * 2] << 8) | resp[10 + i * 2];
        values.push_back(value);
    }
    
    return Result<Vector<uint16_t>>::success(values);
}

Result<void> ModbusClient::write_single_register(uint8_t unit_id, uint16_t address, uint16_t value) {
    ByteArray data(4);
    data[0] = (address >> 8) & 0xFF;
    data[1] = address & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;
    
    ByteArray request = build_request(unit_id, 0x06, address, 1, data);
    auto response = send_request(unit_id, request);
    
    if (response.is_failure()) {
        return Result<void>::failure(response.code(), response.message());
    }
    
    return Result<void>::success();
}

Result<uint16_t> ModbusClient::read_coil(uint8_t unit_id, uint16_t address) {
    ByteArray data(4);
    data[0] = (address >> 8) & 0xFF;
    data[1] = address & 0xFF;
    data[2] = 0;
    data[3] = 1;
    
    ByteArray request = build_request(unit_id, 0x01, address, 1, data);
    auto response = send_request(unit_id, request);
    
    if (response.is_failure()) {
        return Result<uint16_t>::failure(response.code(), response.message());
    }
    
    return Result<uint16_t>::success(response.value()[9]);
}

Result<void> ModbusClient::write_single_coil(uint8_t unit_id, uint16_t address, bool value) {
    ByteArray data(4);
    data[0] = (address >> 8) & 0xFF;
    data[1] = address & 0xFF;
    data[2] = value ? 0xFF : 0;
    data[3] = 0;
    
    ByteArray request = build_request(unit_id, 0x05, address, 1, data);
    auto response = send_request(unit_id, request);
    
    if (response.is_failure()) {
        return Result<void>::failure(response.code(), response.message());
    }
    
    return Result<void>::success();
}

// 其他Modbus方法简化实现
Result<uint16_t> ModbusClient::read_discrete_input(uint8_t unit_id, uint16_t address) {
    return Result<uint16_t>::failure(ErrorCode::NotSupported, "Not implemented");
}

Result<Vector<uint16_t>> ModbusClient::read_coils(uint8_t unit_id, uint16_t address, uint16_t count) {
    return Result<Vector<uint16_t>>::failure(ErrorCode::NotSupported, "Not implemented");
}

Result<Vector<uint16_t>> ModbusClient::read_discrete_inputs(uint8_t unit_id, uint16_t address, uint16_t count) {
    return Result<Vector<uint16_t>>::failure(ErrorCode::NotSupported, "Not implemented");
}

Result<uint16_t> ModbusClient::read_input_register(uint8_t unit_id, uint16_t address) {
    return Result<uint16_t>::failure(ErrorCode::NotSupported, "Not implemented");
}

Result<Vector<uint16_t>> ModbusClient::read_input_registers(uint8_t unit_id, uint16_t address, uint16_t count) {
    return Result<Vector<uint16_t>>::failure(ErrorCode::NotSupported, "Not implemented");
}

Result<void> ModbusClient::write_multiple_coils(uint8_t unit_id, uint16_t address, const Vector<bool>& values) {
    return Result<void>::failure(ErrorCode::NotSupported, "Not implemented");
}

Result<void> ModbusClient::write_multiple_registers(uint8_t unit_id, uint16_t address, const Vector<uint16_t>& values) {
    return Result<void>::failure(ErrorCode::NotSupported, "Not implemented");
}

// ============== CommunicationManager ==============

ICommDevice::Ptr CommunicationManager::create_tcp_client(const String& id, 
                                                          const String& address, 
                                                          uint16_t port) {
    CommDeviceInfo info;
    info.id = id;
    info.address = address;
    info.port = port;
    info.protocol = "TCP";
    
    auto device = std::make_shared<TcpClient>(info);
    add_device(device);
    return device;
}

ICommDevice::Ptr CommunicationManager::create_tcp_server(const String& id, uint16_t port) {
    CommDeviceInfo info;
    info.id = id;
    info.port = port;
    info.protocol = "TCP_SERVER";
    
    auto device = std::make_shared<TcpServer>(info);
    add_device(device);
    return device;
}

ICommDevice::Ptr CommunicationManager::create_udp_socket(const String& id, 
                                                          const String& address, 
                                                          uint16_t port) {
    CommDeviceInfo info;
    info.id = id;
    info.address = address;
    info.port = port;
    info.protocol = "UDP";
    
    auto device = std::make_shared<UdpSocket>(info);
    add_device(device);
    return device;
}

ICommDevice::Ptr CommunicationManager::create_serial_port(const String& id, 
                                                           const String& port_name, 
                                                           uint32_t baud_rate) {
    CommDeviceInfo info;
    info.id = id;
    info.address = port_name;
    info.protocol = "SERIAL";
    
    SerialPort::SerialConfig config;
    config.baud_rate = baud_rate;
    
    auto device = std::make_shared<SerialPort>(info, config);
    add_device(device);
    return device;
}

ICommDevice::Ptr CommunicationManager::create_modbus_client(const String& id, 
                                                             const String& address, 
                                                             uint16_t port) {
    CommDeviceInfo info;
    info.id = id;
    info.address = address;
    info.port = port;
    info.protocol = "MODBUS_TCP";
    
    auto device = std::make_shared<ModbusClient>(info);
    add_device(device);
    return device;
}

ICommDevice::Ptr CommunicationManager::create_s7_client(const String& id, 
                                                         const String& ip, 
                                                         int rack, 
                                                         int slot) {
    CommDeviceInfo info;
    info.id = id;
    info.address = ip;
    info.port = 102;  // S7默认端口
    info.protocol = "S7";
    
    auto device = std::make_shared<S7Client>(info);
    // 直接连接PLC（使用rack和slot参数）
    auto result = device->connect_plc(ip, rack, slot);
    if (result.is_failure()) {
        OVF_WARN() << "S7 client creation: " << result.message();
    }
    
    add_device(device);
    return device;
}

void CommunicationManager::add_device(ICommDevice::Ptr device) {
    std::lock_guard<std::mutex> lock(mutex_);
    devices_[device->info().id] = device;
}

void CommunicationManager::remove_device(const String& device_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    devices_.erase(device_id);
}

ICommDevice::Ptr CommunicationManager::get_device(const String& device_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = devices_.find(device_id);
    return it != devices_.end() ? it->second : nullptr;
}

Vector<ICommDevice::Ptr> CommunicationManager::get_all_devices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<ICommDevice::Ptr> result;
    for (const auto& pair : devices_) {
        result.push_back(pair.second);
    }
    return result;
}

Vector<ICommDevice::Ptr> CommunicationManager::get_devices_by_protocol(const String& protocol) const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<ICommDevice::Ptr> result;
    for (const auto& pair : devices_) {
        if (pair.second->info().protocol == protocol) {
            result.push_back(pair.second);
        }
    }
    return result;
}

} // namespace comm
} // namespace ovf