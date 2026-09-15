/**
 * @file communication.h
 * @brief OpenVisionFlow 通信服务层基础框架
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/error.h"
#include "ovf/core/data.h"
#include "ovf/core/logger.h"
#include <memory>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>

namespace ovf {
namespace comm {

/**
 * @brief 连接状态
 */
enum class ConnectionState : uint8_t {
    Disconnected = 0,   // 断开
    Connecting = 1,     // 连接中
    Connected = 2,      // 已连接
    Error = 3           // 错误
};

/**
 * @brief 通信设备信息
 */
struct CommDeviceInfo {
    String id;              // 设备ID
    String name;            // 设备名称
    String protocol;        // 协议类型
    String address;         // 地址
    uint16_t port = 0;      // 端口
    String description;     // 描述
};

/**
 * @brief 通信接口基类
 */
class ICommDevice {
public:
    using Ptr = std::shared_ptr<ICommDevice>;
    
    virtual ~ICommDevice() = default;
    
    // 基本信息
    virtual const CommDeviceInfo& info() const = 0;
    virtual ConnectionState state() const = 0;
    
    // 连接管理
    virtual Result<void> connect() = 0;
    virtual Result<void> disconnect() = 0;
    virtual bool is_connected() const = 0;
    
    // 数据发送
    virtual Result<void> send(const ByteArray& data) = 0;
    virtual Result<void> send_string(const String& data) = 0;
    
    // 数据接收
    virtual Result<ByteArray> receive(uint32_t timeout_ms = 1000) = 0;
    virtual Result<String> receive_string(uint32_t timeout_ms = 1000) = 0;
    
    // 回调接收
    using DataCallback = std::function<void(const ByteArray&)>;
    virtual void set_data_callback(DataCallback callback) = 0;
    
    // 错误信息
    virtual String last_error() const = 0;
};

/**
 * @brief TCP客户端
 */
class TcpClient : public ICommDevice {
public:
    explicit TcpClient(const CommDeviceInfo& info);
    ~TcpClient();
    
    const CommDeviceInfo& info() const override { return info_; }
    ConnectionState state() const override { return state_; }
    
    Result<void> connect() override;
    Result<void> disconnect() override;
    bool is_connected() const override { return state_ == ConnectionState::Connected; }
    
    Result<void> send(const ByteArray& data) override;
    Result<void> send_string(const String& data) override;
    
    Result<ByteArray> receive(uint32_t timeout_ms = 1000) override;
    Result<String> receive_string(uint32_t timeout_ms = 1000) override;
    
    void set_data_callback(DataCallback callback) override;
    
    String last_error() const override { return last_error_; }

private:
    void receive_thread();
    
private:
    CommDeviceInfo info_;
    ConnectionState state_ = ConnectionState::Disconnected;
    String last_error_;
    
    // Socket相关（跨平台封装）
    int socket_fd_ = -1;
    
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<ByteArray> receive_queue_;
    DataCallback data_callback_;
};

/**
 * @brief TCP服务器
 */
class TcpServer : public ICommDevice {
public:
    using ClientCallback = std::function<void(int client_fd, const String& client_addr)>;
    
    explicit TcpServer(const CommDeviceInfo& info);
    ~TcpServer();
    
    const CommDeviceInfo& info() const override { return info_; }
    ConnectionState state() const override { return state_; }
    
    Result<void> connect() override;  // 启动监听
    Result<void> disconnect() override;
    bool is_connected() const override { return state_ == ConnectionState::Connected; }
    
    // 发送到所有客户端
    Result<void> send(const ByteArray& data) override;
    Result<void> send_string(const String& data) override;
    
    // 接收（从任意客户端）
    Result<ByteArray> receive(uint32_t timeout_ms = 1000) override;
    Result<String> receive_string(uint32_t timeout_ms = 1000) override;
    
    void set_data_callback(DataCallback callback) override;
    void set_client_callback(ClientCallback callback);
    
    // 客户端管理
    Vector<int> get_clients() const;
    Result<void> send_to_client(int client_fd, const ByteArray& data);
    Result<void> close_client(int client_fd);
    
    String last_error() const override { return last_error_; }

private:
    void server_thread();
    void client_thread(int client_fd, String client_addr);
    
private:
    CommDeviceInfo info_;
    ConnectionState state_ = ConnectionState::Disconnected;
    String last_error_;
    
    int server_fd_ = -1;
    Vector<int> client_fds_;
    
    std::atomic<bool> running_{false};
    std::thread server_thread_;
    mutable std::mutex mutex_;
    
    DataCallback data_callback_;
    ClientCallback client_callback_;
};

/**
 * @brief UDP通信
 */
class UdpSocket : public ICommDevice {
public:
    explicit UdpSocket(const CommDeviceInfo& info);
    ~UdpSocket();
    
    const CommDeviceInfo& info() const override { return info_; }
    ConnectionState state() const override { return state_; }
    
    Result<void> connect() override;
    Result<void> disconnect() override;
    bool is_connected() const override { return state_ == ConnectionState::Connected; }
    
    Result<void> send(const ByteArray& data) override;
    Result<void> send_string(const String& data) override;
    
    Result<ByteArray> receive(uint32_t timeout_ms = 1000) override;
    Result<String> receive_string(uint32_t timeout_ms = 1000) override;
    
    void set_data_callback(DataCallback callback) override;
    
    String last_error() const override { return last_error_; }

private:
    void receive_thread();
    
private:
    CommDeviceInfo info_;
    ConnectionState state_ = ConnectionState::Disconnected;
    String last_error_;
    
    int socket_fd_ = -1;
    
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<ByteArray> receive_queue_;
    DataCallback data_callback_;
};

/**
 * @brief 串口通信
 */
class SerialPort : public ICommDevice {
public:
    struct SerialConfig {
        uint32_t baud_rate = 9600;
        uint8_t data_bits = 8;
        uint8_t stop_bits = 1;
        uint8_t parity = 0;  // 0=无, 1=奇, 2=偶
        uint8_t flow_control = 0;  // 0=无, 1=硬件, 2=软件
    };
    
    // 注：不能写成 config = {} —— SerialConfig 是带NSDMI的嵌套类，
    // 在外层类作用域内不算完整类型，GCC 依标准拒绝（MSVC 宽松故原代码可编）。用重载替代。
    explicit SerialPort(const CommDeviceInfo& info);
    explicit SerialPort(const CommDeviceInfo& info, const SerialConfig& config);
    ~SerialPort();
    
    const CommDeviceInfo& info() const override { return info_; }
    ConnectionState state() const override { return state_; }
    
    Result<void> connect() override;
    Result<void> disconnect() override;
    bool is_connected() const override { return state_ == ConnectionState::Connected; }
    
    Result<void> send(const ByteArray& data) override;
    Result<void> send_string(const String& data) override;
    
    Result<ByteArray> receive(uint32_t timeout_ms = 1000) override;
    Result<String> receive_string(uint32_t timeout_ms = 1000) override;
    
    void set_data_callback(DataCallback callback) override;
    
    // 配置
    Result<void> set_config(const SerialConfig& config);
    SerialConfig get_config() const { return config_; }
    
    String last_error() const override { return last_error_; }

private:
    void receive_thread();
    
private:
    CommDeviceInfo info_;
    SerialConfig config_;
    ConnectionState state_ = ConnectionState::Disconnected;
    String last_error_;
    
    // 平台相关句柄
#ifdef _WIN32
    void* handle_ = nullptr;  // HANDLE
#else
    int fd_ = -1;
#endif
    
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<ByteArray> receive_queue_;
    DataCallback data_callback_;
};

/**
 * @brief Modbus客户端
 */
class ModbusClient : public ICommDevice {
public:
    enum class ModbusType : uint8_t {
        TCP = 0,
        RTU = 1
    };
    
    explicit ModbusClient(const CommDeviceInfo& info, ModbusType type = ModbusType::TCP);
    ~ModbusClient();
    
    const CommDeviceInfo& info() const override { return info_; }
    ConnectionState state() const override { return state_; }
    
    Result<void> connect() override;
    Result<void> disconnect() override;
    bool is_connected() const override { return state_ == ConnectionState::Connected; }
    
    // 基础发送/接收
    Result<void> send(const ByteArray& data) override;
    Result<ByteArray> receive(uint32_t timeout_ms = 1000) override;
    Result<void> send_string(const String& data) override { return Result<void>::failure(ErrorCode::NotSupported, "Modbus does not support string"); }
    Result<String> receive_string(uint32_t timeout_ms = 1000) override { return Result<String>::failure(ErrorCode::NotSupported, "Modbus does not support string"); }
    
    void set_data_callback(DataCallback callback) override {}
    
    // Modbus操作
    Result<uint16_t> read_coil(uint8_t unit_id, uint16_t address);
    Result<Vector<uint16_t>> read_coils(uint8_t unit_id, uint16_t address, uint16_t count);
    
    Result<uint16_t> read_discrete_input(uint8_t unit_id, uint16_t address);
    Result<Vector<uint16_t>> read_discrete_inputs(uint8_t unit_id, uint16_t address, uint16_t count);
    
    Result<uint16_t> read_holding_register(uint8_t unit_id, uint16_t address);
    Result<Vector<uint16_t>> read_holding_registers(uint8_t unit_id, uint16_t address, uint16_t count);
    
    Result<uint16_t> read_input_register(uint8_t unit_id, uint16_t address);
    Result<Vector<uint16_t>> read_input_registers(uint8_t unit_id, uint16_t address, uint16_t count);
    
    Result<void> write_single_coil(uint8_t unit_id, uint16_t address, bool value);
    Result<void> write_multiple_coils(uint8_t unit_id, uint16_t address, const Vector<bool>& values);
    
    Result<void> write_single_register(uint8_t unit_id, uint16_t address, uint16_t value);
    Result<void> write_multiple_registers(uint8_t unit_id, uint16_t address, const Vector<uint16_t>& values);
    
    String last_error() const override { return last_error_; }

private:
    ByteArray build_request(uint8_t unit_id, uint8_t function, uint16_t address, uint16_t count, const ByteArray& data = {});
    Result<ByteArray> send_request(uint8_t unit_id, const ByteArray& request);
    Result<void> parse_response(const ByteArray& response, uint8_t expected_function);
    
private:
    CommDeviceInfo info_;
    ModbusType type_;
    ConnectionState state_ = ConnectionState::Disconnected;
    String last_error_;
    
    // TCP或RTU底层设备
    std::shared_ptr<TcpClient> tcp_client_;
    std::shared_ptr<SerialPort> serial_port_;
    
    uint8_t transaction_id_ = 0;
};

/**
 * @brief S7连接类型
 */
enum class S7ConnectionType : uint8_t {
    PG = 0,      // 编程器连接
    OP = 1,      // 操作面板连接
    Basic = 2    // 基本连接(S7-300/400)
};

/**
 * @brief S7 PLC类型
 */
enum class S7PLCType : uint8_t {
    S7_200 = 0,
    S7_300 = 1,
    S7_400 = 2,
    S7_1200 = 3,
    S7_1500 = 4
};

/**
 * @brief S7数据区域
 */
enum class S7Area : uint8_t {
    PE = 0,  // 输入(process image input)
    PA = 1,  // 输出(process image output)
    MK = 2,  // 位存储(markers)
    DB = 3,  // 数据块(data block)
    CT = 4,  // 计数器
    TM = 5   // 定时器
};

/**
 * @brief S7客户端 - 西门子PLC通信
 */
class S7Client : public ICommDevice {
public:
    explicit S7Client(const CommDeviceInfo& info);
    ~S7Client();
    
    const CommDeviceInfo& info() const override { return info_; }
    ConnectionState state() const override { return state_; }
    
    Result<void> connect() override;
    Result<void> disconnect() override;
    bool is_connected() const override { return state_ == ConnectionState::Connected; }
    
    // 基础发送/接收（S7协议特有，不直接使用）
    Result<void> send(const ByteArray& data) override;
    Result<ByteArray> receive(uint32_t timeout_ms = 1000) override;
    Result<void> send_string(const String& data) override { return Result<void>::failure(ErrorCode::NotSupported, "S7 does not support raw string"); }
    Result<String> receive_string(uint32_t timeout_ms = 1000) override { return Result<String>::failure(ErrorCode::NotSupported, "S7 does not support raw string"); }
    
    void set_data_callback(DataCallback callback) override {}
    
    // S7专用连接方法
    Result<void> connect_plc(const String& ip, int rack, int slot);
    
    // 读取数据
    Result<ByteArray> read_bytes(S7Area area, int db_number, int start, int size);
    Result<int32_t> read_int(S7Area area, int db_number, int start);
    Result<float> read_float(S7Area area, int db_number, int start);
    Result<bool> read_bit(S7Area area, int db_number, int byte_offset, int bit_offset);
    
    // 写入数据
    Result<void> write_bytes(S7Area area, int db_number, int start, const ByteArray& data);
    Result<void> write_int(S7Area area, int db_number, int start, int32_t value);
    Result<void> write_float(S7Area area, int db_number, int start, float value);
    Result<void> write_bit(S7Area area, int db_number, int byte_offset, int bit_offset, bool value);
    
    // PLC控制
    Result<void> plc_start();
    Result<void> plc_stop();
    Result<void> plc_hot_start();
    Result<String> get_plc_info();
    
    // 配置
    void set_plc_type(S7PLCType type) { plc_type_ = type; }
    S7PLCType get_plc_type() const { return plc_type_; }
    
    String last_error() const override { return last_error_; }

private:
    // S7协议内部方法
    ByteArray build_cotp_connection_request(int rack, int slot);
    ByteArray build_s7_communication_request();
    ByteArray build_read_request(S7Area area, int db_number, int start, int size);
    ByteArray build_write_request(S7Area area, int db_number, int start, const ByteArray& data);
    ByteArray build_control_request(uint8_t function);
    
    Result<ByteArray> send_s7_request(const ByteArray& request);
    Result<void> parse_cotp_connection_response(const ByteArray& response);
    Result<void> parse_s7_communication_response(const ByteArray& response);
    Result<ByteArray> parse_read_response(const ByteArray& response);
    Result<void> parse_write_response(const ByteArray& response);
    
    uint16_t get_area_code(S7Area area, int db_number);

private:
    CommDeviceInfo info_;
    ConnectionState state_ = ConnectionState::Disconnected;
    String last_error_;
    
    String ip_;
    int rack_ = 0;
    int slot_ = 1;
    S7PLCType plc_type_ = S7PLCType::S7_1200;
    
    // Socket
    int socket_fd_ = -1;
    
    // S7协议参数
    uint16_t pdu_size_ = 240;  // PDU大小
    uint16_t message_number_ = 0;  // 消息序号
};

/**
 * @brief 通信管理器
 */
class CommunicationManager {
public:
    static CommunicationManager& instance() {
        static CommunicationManager manager;
        return manager;
    }
    
    // 设备创建
    ICommDevice::Ptr create_tcp_client(const String& id, const String& address, uint16_t port);
    ICommDevice::Ptr create_tcp_server(const String& id, uint16_t port);
    ICommDevice::Ptr create_udp_socket(const String& id, const String& address, uint16_t port);
    ICommDevice::Ptr create_serial_port(const String& id, const String& port_name, uint32_t baud_rate = 9600);
    ICommDevice::Ptr create_modbus_client(const String& id, const String& address, uint16_t port);
    ICommDevice::Ptr create_s7_client(const String& id, const String& ip, int rack = 0, int slot = 1);
    
    // 设备管理
    void add_device(ICommDevice::Ptr device);
    void remove_device(const String& device_id);
    ICommDevice::Ptr get_device(const String& device_id) const;
    Vector<ICommDevice::Ptr> get_all_devices() const;
    
    // 设备查询
    Vector<ICommDevice::Ptr> get_devices_by_protocol(const String& protocol) const;

private:
    CommunicationManager() = default;
    
    HashMap<String, ICommDevice::Ptr> devices_;
    mutable std::mutex mutex_;
};

} // namespace comm
} // namespace ovf