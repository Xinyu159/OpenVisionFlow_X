/**
 * @file types.h
 * @brief OpenVisionFlow 基础类型定义
 * @author OpenVisionFlow Team
 * @version 0.2.0
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <cmath>  // std::sqrt/cos/sin -- MSVC 传递包含，GCC 需显式引入
#include <chrono>
#include <functional>
#include <unordered_map>
#include <any>
#include <iostream>  // 用于枚举类型输出运算符

namespace ovf {

// 版本信息
constexpr const char* VERSION = "0.2.0";
constexpr int VERSION_MAJOR = 0;
constexpr int VERSION_MINOR = 2;
constexpr int VERSION_PATCH = 0;

// 类型别名
using String = std::string;
using ByteArray = std::vector<uint8_t>;

template<typename T>
using Vector = std::vector<T>;

template<typename T>
using Ptr = std::shared_ptr<T>;

template<typename T>
using WeakPtr = std::weak_ptr<T>;

template<typename K, typename V>
using HashMap = std::unordered_map<K, V>;

// 错误码定义
enum class ErrorCode : int32_t {
    Success = 0,
    
    // 通用错误 (1-99)
    Unknown = 1,
    InvalidParameter = 2,
    NullPointer = 3,
    OutOfRange = 4,
    NotSupported = 5,
    Timeout = 6,
    InvalidData = 7,
    NotFound = 8,
    Unsupported = 9,
    AlreadyExists = 10,
    AccessDenied = 11,
    IOError = 12,
    FormatError = 13,
    
    // 流程引擎错误 (100-199)
    FlowNotFound = 100,
    NodeNotFound = 101,
    InvalidFlow = 102,
    InvalidNode = 103,
    ConnectionFailed = 104,
    ExecutionFailed = 105,
    CyclicDependency = 106,
    
    // 硬件错误 (200-299)
    DeviceNotFound = 200,
    DeviceOpenFailed = 201,
    DeviceBusy = 202,
    DeviceError = 203,
    CameraCaptureFailed = 204,
    TriggerTimeout = 205,
    
    // 算法错误 (300-399)
    AlgorithmInitFailed = 300,
    AlgorithmExecFailed = 301,
    InvalidModel = 302,
    InvalidImage = 303,
    CalibrationFailed = 304,
    
    // 通信错误 (400-499)
    ConnectionLost = 400,
    ProtocolError = 401,
    DataParseError = 402,
    SendFailed = 403,
    ReceiveFailed = 404,
    
    // 文件错误 (500-599)
    FileNotFound = 500,
    FileOpenFailed = 501,
    FileParseFailed = 502,
    FileWriteFailed = 503,
    
    // 插件错误 (600-699)
    PluginLoadFailed = 600,
    PluginNotFound = 601,
    PluginVersionMismatch = 602,
    PluginInitFailed = 603,

    // 时钟同步错误 (700-799)
    ClockSyncFailed = 700,
    ClockTimeout = 701,
    FrameSyncError = 702,
    InvalidTimestamp = 703,
    CameraSyncError = 704,
    SyncBufferOverflow = 705,
    SyncNotReady = 706
};

// 图像格式
enum class ImageFormat : uint8_t {
    Unknown = 0,
    Mono8 = 1,
    Mono16 = 2,
    RGB8 = 3,
    RGBA8 = 4,
    BGR8 = 5,
    BGRA8 = 6,
    Float32 = 7
};

// 触发模式
enum class TriggerMode : uint8_t {
    Continuous = 0,
    Software = 1,
    Hardware = 2,
    External = 3
};

// 节点状态
enum class NodeState : uint8_t {
    Idle = 0,
    Running = 1,
    Success = 2,
    Failed = 3,
    Disabled = 4
};

// 数据类型
enum class DataType : uint8_t {
    None = 0,
    Image = 1,
    Number = 2,
    String = 3,
    Boolean = 4,
    Array = 5,
    Object = 6,
    Point = 7,
    Region = 8,
    Pose = 9,
    PointCloud = 10,     // 点云数据
    DepthImage = 11,     // 深度图数据
    Any = 255
};

/**
 * @brief DataType 的可读名，用于错误/警告消息
 *
 * 报"期望 Image，实际是 Number"比报"期望 1，实际是 2"有用得多。
 */
inline const char* data_type_name(DataType type) {
    switch (type) {
        case DataType::None:       return "None";
        case DataType::Image:      return "Image";
        case DataType::Number:     return "Number";
        case DataType::String:     return "String";
        case DataType::Boolean:    return "Boolean";
        case DataType::Array:      return "Array";
        case DataType::Object:     return "Object";
        case DataType::Point:      return "Point";
        case DataType::Region:     return "Region";
        case DataType::Pose:       return "Pose";
        case DataType::PointCloud: return "PointCloud";
        case DataType::DepthImage: return "DepthImage";
        case DataType::Any:        return "Any";
    }
    return "Unknown";
}

// 图像数据
struct ImageData {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 1;
    ImageFormat format = ImageFormat::Unknown;
    ByteArray data;
    uint64_t timestamp = 0;
    uint32_t frame_id = 0;
    String source_id;
    
    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }
};

// 点结构
template<typename T>
struct Point2D {
    T x = 0;
    T y = 0;
    
    Point2D() = default;
    Point2D(T x_, T y_) : x(x_), y(y_) {}
};

template<typename T>
struct Point3D {
    T x = 0;
    T y = 0;
    T z = 0;
    
    Point3D() = default;
    Point3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
    
    // 计算到另一点的距离
    T distance_to(const Point3D<T>& other) const {
        T dx = x - other.x;
        T dy = y - other.y;
        T dz = z - other.z;
        return static_cast<T>(std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    
    // 向量运算
    Point3D<T> operator+(const Point3D<T>& other) const {
        return Point3D<T>(x + other.x, y + other.y, z + other.z);
    }
    
    Point3D<T> operator-(const Point3D<T>& other) const {
        return Point3D<T>(x - other.x, y - other.y, z - other.z);
    }
    
    Point3D<T> operator*(T scalar) const {
        return Point3D<T>(x * scalar, y * scalar, z * scalar);
    }
    
    // 点积
    T dot(const Point3D<T>& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    // 叉积
    Point3D<T> cross(const Point3D<T>& other) const {
        return Point3D<T>(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    // 归一化
    Point3D<T> normalized() const {
        T len = static_cast<T>(std::sqrt(x * x + y * y + z * z));
        if (len > static_cast<T>(1e-10)) {
            return Point3D<T>(x / len, y / len, z / len);
        }
        return *this;
    }
    
    // 长度
    T length() const {
        return static_cast<T>(std::sqrt(x * x + y * y + z * z));
    }
};

// 常用类型别名
using Point3Df = Point3D<float>;
using Point3Dd = Point3D<double>;

// RGB颜色结构
struct ColorRGB {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    
    ColorRGB() = default;
    ColorRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
};

// 区域结构
struct Region {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t height = 0;
    float angle = 0.0f;
    
    Region() = default;
    Region(int32_t x_, int32_t y_, int32_t w, int32_t h, float a = 0.0f)
        : x(x_), y(y_), width(w), height(h), angle(a) {}
};

// 位姿结构
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;  // 绕X轴旋转
    double ry = 0.0;  // 绕Y轴旋转
    double rz = 0.0;  // 绕Z轴旋转
};

// 点云数据结构
struct PointCloudData {
    Vector<Point3Df> points;          // 点数据
    Vector<uint8_t> intensities;      // 强度（可选）
    Vector<ColorRGB> colors;          // 颜色（可选）
    int width = 0;                    // 如果是 organized 点云
    int height = 0;                   // 如果是 organized 点云
    bool is_organized = false;        // 是否为有序点云
    
    // 基本方法
    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }
    void clear() {
        points.clear();
        intensities.clear();
        colors.clear();
        width = height = 0;
        is_organized = false;
    }
    
    // 添加点
    void add_point(const Point3Df& pt) {
        points.push_back(pt);
    }
    
    void add_point(float x, float y, float z) {
        points.emplace_back(x, y, z);
    }
    
    // 是否有强度数据
    bool has_intensities() const { return !intensities.empty() && intensities.size() == points.size(); }
    
    // 是否有颜色数据
    bool has_colors() const { return !colors.empty() && colors.size() == points.size(); }
};

// 深度图像数据（深度图+彩色图）
struct DepthImageData {
    ImageData depth;                  // 深度图
    ImageData color;                  // 彩色图（可选）
    float depth_scale = 1.0f;         // 深度缩放因子
    float depth_offset = 0.0f;        // 深度偏移
    float focal_length_x = 0.0f;      // 焦距X
    float focal_length_y = 0.0f;      // 焦距Y
    float center_x = 0.0f;            // 光心坐标X
    float center_y = 0.0f;            // 光心坐标Y
    
    bool empty() const { return depth.empty(); }
    bool has_color() const { return !color.empty(); }
    
    // 获取深度值（考虑缩放因子和偏移）
    float get_depth(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(depth.width) || 
            y < 0 || y >= static_cast<int>(depth.height)) {
            return 0.0f;
        }
        
        float raw_depth = 0.0f;
        if (depth.format == ImageFormat::Mono16) {
            // 16位深度
            const uint16_t* ptr = reinterpret_cast<const uint16_t*>(depth.data.data());
            raw_depth = static_cast<float>(ptr[y * depth.width + x]);
        } else if (depth.format == ImageFormat::Float32) {
            // 32位浮点深度
            const float* ptr = reinterpret_cast<const float*>(depth.data.data());
            raw_depth = ptr[y * depth.width + x];
        } else {
            // 8位深度
            raw_depth = static_cast<float>(depth.data[y * depth.width + x]);
        }
        
        return raw_depth * depth_scale + depth_offset;
    }
};

// 3D平面结构
struct Plane3D {
    Point3Df center;      // 平面中心点
    Point3Df normal;      // 平面法向量（归一化）
    float d = 0.0f;       // 平面方程 d 值 (ax + by + cz + d = 0)
    float fit_error = 0.0f; // 拟合误差
    bool valid = false;   // 是否有效
};

// 3D包围盒
struct BoundingBox3D {
    Point3Df min_pt;       // 最小点
    Point3Df max_pt;       // 最大点
    Point3Df center;      // 中心点
    Point3Df size;         // 尺寸
    
    void update() {
        center = Point3Df(
            (min_pt.x + max_pt.x) * 0.5f,
            (min_pt.y + max_pt.y) * 0.5f,
            (min_pt.z + max_pt.z) * 0.5f
        );
        size = Point3Df(
            max_pt.x - min_pt.x,
            max_pt.y - min_pt.y,
            max_pt.z - min_pt.z
        );
    }
};

// 3D变换矩阵 (4x4)
struct Transform3D {
    float m[4][4];
    
    Transform3D() {
        // 初始化为单位矩阵
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }
    
    // 设置平移
    void set_translation(float tx, float ty, float tz) {
        m[0][3] = tx;
        m[1][3] = ty;
        m[2][3] = tz;
    }
    
    // 设置绕X轴旋转
    void set_rotation_x(float angle_rad) {
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        m[1][1] = c;  m[1][2] = -s;
        m[2][1] = s;  m[2][2] = c;
    }
    
    // 设置绕Y轴旋转
    void set_rotation_y(float angle_rad) {
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        m[0][0] = c;   m[0][2] = s;
        m[2][0] = -s;  m[2][2] = c;
    }
    
    // 设置绕Z轴旋转
    void set_rotation_z(float angle_rad) {
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        m[0][0] = c;  m[0][1] = -s;
        m[1][0] = s;  m[1][1] = c;
    }
    
    // 应用到点
    Point3Df transform(const Point3Df& pt) const {
        float x = m[0][0] * pt.x + m[0][1] * pt.y + m[0][2] * pt.z + m[0][3];
        float y = m[1][0] * pt.x + m[1][1] * pt.y + m[1][2] * pt.z + m[1][3];
        float z = m[2][0] * pt.x + m[2][1] * pt.y + m[2][2] * pt.z + m[2][3];
        return Point3Df(x, y, z);
    }
};

// 时间戳类型
using Timestamp = std::chrono::steady_clock::time_point;

// 获取当前时间戳
inline Timestamp now() {
    return std::chrono::steady_clock::now();
}

// 回调函数类型
template<typename T>
using Callback = std::function<void(const T&)>;

using VoidCallback = std::function<void()>;
using ErrorCallback = std::function<void(ErrorCode, const String&)>;

// 时钟源类型
enum class ClockSource : uint8_t {
    System = 0,      // 系统时钟
    Hardware = 1,    // 硬件时钟（外部触发）
    PTP = 2,         // 精确时间协议（IEEE 1588）
    NTP = 3          // 网络时间协议
};

// 同步模式
enum class SyncMode : uint8_t {
    Software = 0,    // 软同步
    Hardware = 1,    // 硬同步（共用触发线）
    MasterSlave = 2  // 主从模式
};

// ============================================================================
// 枚举类型输出运算符（用于调试和测试输出）
// ============================================================================

// ErrorCode 输出运算符
inline std::ostream& operator<<(std::ostream& os, ErrorCode code) {
    switch (code) {
        case ErrorCode::Success: os << "Success(0)"; break;
        case ErrorCode::Unknown: os << "Unknown(1)"; break;
        case ErrorCode::InvalidParameter: os << "InvalidParameter(2)"; break;
        case ErrorCode::NullPointer: os << "NullPointer(3)"; break;
        case ErrorCode::OutOfRange: os << "OutOfRange(4)"; break;
        case ErrorCode::NotSupported: os << "NotSupported(5)"; break;
        case ErrorCode::Timeout: os << "Timeout(6)"; break;
        case ErrorCode::InvalidData: os << "InvalidData(7)"; break;
        case ErrorCode::NotFound: os << "NotFound(8)"; break;
        case ErrorCode::Unsupported: os << "Unsupported(9)"; break;
        case ErrorCode::AlreadyExists: os << "AlreadyExists(10)"; break;
        case ErrorCode::AccessDenied: os << "AccessDenied(11)"; break;
        case ErrorCode::IOError: os << "IOError(12)"; break;
        case ErrorCode::FormatError: os << "FormatError(13)"; break;
        case ErrorCode::FlowNotFound: os << "FlowNotFound(100)"; break;
        case ErrorCode::NodeNotFound: os << "NodeNotFound(101)"; break;
        case ErrorCode::InvalidFlow: os << "InvalidFlow(102)"; break;
        case ErrorCode::InvalidNode: os << "InvalidNode(103)"; break;
        case ErrorCode::ConnectionFailed: os << "ConnectionFailed(104)"; break;
        case ErrorCode::ExecutionFailed: os << "ExecutionFailed(105)"; break;
        case ErrorCode::CyclicDependency: os << "CyclicDependency(106)"; break;
        case ErrorCode::DeviceNotFound: os << "DeviceNotFound(200)"; break;
        case ErrorCode::DeviceOpenFailed: os << "DeviceOpenFailed(201)"; break;
        case ErrorCode::DeviceBusy: os << "DeviceBusy(202)"; break;
        case ErrorCode::DeviceError: os << "DeviceError(203)"; break;
        case ErrorCode::CameraCaptureFailed: os << "CameraCaptureFailed(204)"; break;
        case ErrorCode::TriggerTimeout: os << "TriggerTimeout(205)"; break;
        case ErrorCode::AlgorithmInitFailed: os << "AlgorithmInitFailed(300)"; break;
        case ErrorCode::AlgorithmExecFailed: os << "AlgorithmExecFailed(301)"; break;
        case ErrorCode::InvalidModel: os << "InvalidModel(302)"; break;
        case ErrorCode::InvalidImage: os << "InvalidImage(303)"; break;
        case ErrorCode::CalibrationFailed: os << "CalibrationFailed(304)"; break;
        case ErrorCode::ConnectionLost: os << "ConnectionLost(400)"; break;
        case ErrorCode::ProtocolError: os << "ProtocolError(401)"; break;
        case ErrorCode::DataParseError: os << "DataParseError(402)"; break;
        case ErrorCode::SendFailed: os << "SendFailed(403)"; break;
        case ErrorCode::ReceiveFailed: os << "ReceiveFailed(404)"; break;
        case ErrorCode::FileNotFound: os << "FileNotFound(500)"; break;
        case ErrorCode::FileOpenFailed: os << "FileOpenFailed(501)"; break;
        case ErrorCode::FileParseFailed: os << "FileParseFailed(502)"; break;
        case ErrorCode::FileWriteFailed: os << "FileWriteFailed(503)"; break;
        case ErrorCode::PluginLoadFailed: os << "PluginLoadFailed(600)"; break;
        case ErrorCode::PluginNotFound: os << "PluginNotFound(601)"; break;
        case ErrorCode::PluginVersionMismatch: os << "PluginVersionMismatch(602)"; break;
        case ErrorCode::PluginInitFailed: os << "PluginInitFailed(603)"; break;
        case ErrorCode::ClockSyncFailed: os << "ClockSyncFailed(700)"; break;
        case ErrorCode::ClockTimeout: os << "ClockTimeout(701)"; break;
        case ErrorCode::FrameSyncError: os << "FrameSyncError(702)"; break;
        case ErrorCode::InvalidTimestamp: os << "InvalidTimestamp(703)"; break;
        case ErrorCode::CameraSyncError: os << "CameraSyncError(704)"; break;
        case ErrorCode::SyncBufferOverflow: os << "SyncBufferOverflow(705)"; break;
        case ErrorCode::SyncNotReady: os << "SyncNotReady(706)"; break;
        default: os << "ErrorCode(" << static_cast<int>(code) << ")"; break;
    }
    return os;
}

// ImageFormat 输出运算符
inline std::ostream& operator<<(std::ostream& os, ImageFormat format) {
    switch (format) {
        case ImageFormat::Unknown: os << "Unknown(0)"; break;
        case ImageFormat::Mono8: os << "Mono8(1)"; break;
        case ImageFormat::Mono16: os << "Mono16(2)"; break;
        case ImageFormat::RGB8: os << "RGB8(3)"; break;
        case ImageFormat::RGBA8: os << "RGBA8(4)"; break;
        case ImageFormat::BGR8: os << "BGR8(5)"; break;
        case ImageFormat::BGRA8: os << "BGRA8(6)"; break;
        case ImageFormat::Float32: os << "Float32(7)"; break;
        default: os << "ImageFormat(" << static_cast<int>(format) << ")"; break;
    }
    return os;
}

// DataType 输出运算符
inline std::ostream& operator<<(std::ostream& os, DataType type) {
    switch (type) {
        case DataType::None: os << "None(0)"; break;
        case DataType::Image: os << "Image(1)"; break;
        case DataType::Number: os << "Number(2)"; break;
        case DataType::String: os << "String(3)"; break;
        case DataType::Boolean: os << "Boolean(4)"; break;
        case DataType::Array: os << "Array(5)"; break;
        case DataType::Object: os << "Object(6)"; break;
        case DataType::Point: os << "Point(7)"; break;
        case DataType::Region: os << "Region(8)"; break;
        case DataType::Pose: os << "Pose(9)"; break;
        case DataType::PointCloud: os << "PointCloud(10)"; break;
        case DataType::DepthImage: os << "DepthImage(11)"; break;
        case DataType::Any: os << "Any(255)"; break;
        default: os << "DataType(" << static_cast<int>(type) << ")"; break;
    }
    return os;
}

// NodeState 输出运算符
inline std::ostream& operator<<(std::ostream& os, NodeState state) {
    switch (state) {
        case NodeState::Idle: os << "Idle(0)"; break;
        case NodeState::Running: os << "Running(1)"; break;
        case NodeState::Success: os << "Success(2)"; break;
        case NodeState::Failed: os << "Failed(3)"; break;
        case NodeState::Disabled: os << "Disabled(4)"; break;
        default: os << "NodeState(" << static_cast<int>(state) << ")"; break;
    }
    return os;
}

// TriggerMode 输出运算符
inline std::ostream& operator<<(std::ostream& os, TriggerMode mode) {
    switch (mode) {
        case TriggerMode::Continuous: os << "Continuous(0)"; break;
        case TriggerMode::Software: os << "Software(1)"; break;
        case TriggerMode::Hardware: os << "Hardware(2)"; break;
        case TriggerMode::External: os << "External(3)"; break;
        default: os << "TriggerMode(" << static_cast<int>(mode) << ")"; break;
    }
    return os;
}

} // namespace ovf