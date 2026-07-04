# OpenVisionFlow 架构设计文档

**版本**: 0.1.0  
**作者**: OpenVisionFlow Team  
**目标读者**: 开发者、架构师

---

## 目录

1. [系统架构](#1-系统架构)
2. [核心设计](#2-核心设计)
3. [数据流设计](#3-数据流设计)
4. [插件架构](#4-插件架构)
5. [扩展机制](#5-扩展机制)
6. [性能优化](#6-性能优化)

---

## 1. 系统架构

### 1.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          OpenVisionFlow Platform                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         Application Layer                            │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │  ovf-web-server     │    ovf-sdk-python    │    ovf-sdk-cpp         │   │
│  │  (Web服务/API)      │    (Python绑定)      │    (C++ SDK)           │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                          Flow Engine Layer                           │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                    FlowEngine (流程引擎核心)                          │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │   │
│  │  │ FlowContext  │  │ FlowRunner   │  │ FlowDef      │               │   │
│  │  │ (执行上下文) │  │ (运行控制)   │  │ (流程定义)   │               │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                          Node Layer (节点层)                         │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │  ┌──────────────────────────────────────────────────────────────┐   │   │
│  │  │              NodeFactory (节点工厂 - 注册/创建)                │   │   │
│  │  └──────────────────────────────────────────────────────────────┘   │   │
│  │                                   │                                  │   │
│  │           ┌───────────────────────┼───────────────────────┐         │   │
│  │           ▼                       ▼                       ▼         │   │
│  │  ┌────────────┐  ┌────────────────┐  ┌────────────────┐              │   │
│  │  │ ovf-core   │  │ ovf-algorithm  │  │ ovf-plugin-*   │              │   │
│  │  │ (核心节点) │  │ (算法节点)     │  │ (插件节点)     │              │   │
│  │  └────────────┘  └────────────────┘  └────────────────┘              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        Data Layer (数据层)                           │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │    Data (数据容器)  │  DataType (类型系统)  │  Result (结果处理)     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      Hardware Layer (硬件层)                         │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                      HALManager (硬件抽象层管理)                      │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐     │   │
│  │  │ ICamera    │  │ ILightSrc  │  │ IMotion    │  │ IIODevice  │     │   │
│  │  │ (相机接口) │  │ (光源接口) │  │ (运动控制) │  │ (IO设备)   │     │   │
│  │  └────────────┘  └────────────┘  └────────────┘  └────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 核心模块 (ovf-core)

**位置**: `ovf-core/`

**职责**:
- 提供系统基础类型定义 (`types.h`)
- 定义节点基类 `INode` (`node.h`)
- 实现流程引擎 `FlowEngine` (`flow.h`)
- 提供数据容器 `Data` (`data.h`)
- 实现错误处理机制 `Result` (`error.h`)
- 硬件抽象层 HAL (`hal.h`)
- 日志系统 (`logger.h`)
- 调试器 (`debugger.h`)
- 时钟同步 (`clock_sync.h`)
- 通信模块 (`communication.h`)

**关键类**:
```
ovf-core/
├── types.h          # 基础类型、枚举、数据结构
├── node.h           # INode基类、NodeInfo、NodeFactory
├── flow.h           # FlowEngine、FlowContext、FlowDef、FlowRunner
├── data.h           # Data容器、DataPort、ParamDef、ParamSet
├── error.h          # Result<T>、Exception、ErrorInfo
├── hal.h            # IDevice、ICamera、HALManager
└── logger.h         # 日志系统
```

### 1.3 算法模块 (ovf-algorithm)

**位置**: `ovf-algorithm/`

**职责**: 提供工业机器视觉算法节点实现

**主要算法分类**:

| 分类 | 算法节点 |
|------|----------|
| **图像处理** | 高斯模糊、Sobel边缘、Canny边缘、形态学、直方图均衡化 |
| **图像增强** | 对比度增强、去噪、锐化、白平衡校正 |
| **特征检测** | Blob分析、角点检测、轮廓提取、形状匹配 |
| **测量分析** | 几何测量、卡尺工具、亚像素精度测量 |
| **模板匹配** | NCC模板匹配、形状模板匹配、多模板匹配 |
| **分割算法** | 阈值分割、区域生长、分水岭、边缘分割 |
| **标定** | 相机标定、手眼标定、自动标定 |
| **3D视觉** | 点云处理、立体视觉、光度立体、3D重建 |
| **深度学习** | ONNX推理、模型训练、缺陷检测 |
| **工业应用** | 晶圆检测、汽车零部件检测、OCR识别、条码识别 |
| **CUDA加速** | GPU加速算子（高斯、Sobel、阈值、形态学等） |

**文件组织**:
```
ovf-algorithm/
├── include/ovf/algorithm/
│   ├── edge_detection.h      # 边缘检测
│   ├── blob_analysis.h       # Blob分析
│   ├── measurement.h         # 测量分析
│   ├── template_matching.h   # 模板匹配
│   ├── calibration.h         # 标定算法
│   ├── 3d_advanced.h         # 3D视觉高级
│   ├── cuda_accelerator.h    # CUDA加速
│   ├── onnx_inference.h      # 深度学习推理
│   ├── industrial_algo.h     # 工业应用算法
│   └── ...
└── src/
    ├── edge_detection.cpp
    ├── blob_analysis.cpp
    └── ...
```

### 1.4 插件系统 (ovf-plugin-*)

**插件模块**:

| 插件 | 位置 | 功能 |
|------|------|------|
| **ovf-plugin-algorithm-ext** | `ovf-plugins/algorithm/` | 扩展算法插件 |
| **ovf-plugin-camera** | `ovf-plugin-camera/` | 相机驱动插件（GenICam标准） |
| **ovf-plugin-communication** | `ovf-plugins/communication/` | 通信协议插件（PLC、串口、TCP/IP） |
| **ovf-plugin-hardware** | `ovf-plugins/hardware/` | 硬件设备插件（光源、运动控制） |
| **ovf-plugin-custom** | `ovf-plugins/custom/` | 用户自定义插件 |

**插件加载流程**:
```
┌──────────────┐
│ 加载插件DLL  │
└──────────────┘
       │
       ▼
┌──────────────┐
│ 调用init函数 │
└──────────────┘
       │
       ▼
┌──────────────────────────┐
│ 注册节点到NodeFactory    │
│ 注册驱动到HALManager     │
└──────────────────────────┘
       │
       ▼
┌──────────────┐
│ 插件可用     │
└──────────────┘
```

### 1.5 SDK模块

#### C++ SDK (ovf-sdk-cpp)

**位置**: `ovf-sdk/cpp/`

**API接口** (`api.h`):
```cpp
namespace ovf::api {
    // SDK初始化
    Result<void> initialize();
    Result<void> shutdown();
    
    // 流程引擎创建
    FlowEngine::Ptr create_flow_engine();
    FlowRunner::Ptr create_flow_runner();
    
    // 流程文件操作
    Result<FlowDef> load_flow_file(const String& filepath);
    Result<void> save_flow_file(const String& filepath, const FlowDef& flow);
    
    // 节点管理
    Vector<String> get_registered_node_types();
    const NodeInfo* get_node_info(const String& type_id);
    INode::Ptr create_node(const String& type_id, const String& instance_id);
    
    // 硬件管理
    hal::HALManager& get_hal_manager();
    Vector<hal::DeviceInfo> enumerate_cameras();
    hal::ICamera::Ptr create_camera(const String& driver_type, const hal::DeviceInfo& info);
}
```

#### Python SDK (ovf-sdk-python)

**位置**: `ovf-sdk-python/`

**绑定实现** (`python_bindings.cpp`):
- 绑定核心类型：`Data`, `Result`, `ImageData`, `Region`, `Pose`
- 绑定流程引擎：`FlowEngine`, `FlowRunner`, `FlowContext`
- 绑定节点系统：`INode`, `NodeFactory`
- 绑定硬件接口：`HALManager`, `ICamera`

### 1.6 Web服务 (ovf-web-server)

**位置**: `ovf-web-server/`

**功能**:
- RESTful API接口
- 流程执行监控 (`flow_execution_monitor.h`)
- 性能分析 (`performance_analyzer.h`)
- 执行日志可视化 (`execution_log_visualizer.h`)
- Web Socket实时通信

**API端点设计**:
```
/api/v1/
├── flow/
│   ├── load        # 加载流程
│   ├── run         # 执行流程
│   ├── stop        # 停止流程
│   └── status      # 获取状态
├── nodes/
│   ├── list        # 获取节点列表
│   ├── info        # 获取节点信息
│   └── create      # 创建节点
├── hardware/
│   ├── cameras     # 枚举相机
│   └── devices     # 枚举设备
└── monitor/
    ├── performance # 性能数据
    └── logs        # 执行日志
```

---

## 2. 核心设计

### 2.1 INode基类设计

**定义位置**: `ovf-core/include/ovf/core/node.h`

#### 类结构

```cpp
class INode : public std::enable_shared_from_this<INode> {
public:
    using Ptr = std::shared_ptr<INode>;
    using WeakPtr = std::weak_ptr<INode>;
    
    // 构造与基本信息
    INode(const String& instance_id, const NodeInfo& info);
    const String& instance_id() const;
    const NodeInfo& info() const;
    NodeState state() const;
    
    // 参数管理
    void set_param(const String& key, const Data& value);
    Data get_param(const String& key, const Data& default_val) const;
    
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
    
    // 执行接口（纯虚函数，必须实现）
    virtual Result<void> init();
    virtual Result<void> execute(FlowContext& context) = 0;
    virtual Result<void> reset();
    
    // 启用/禁用
    void set_enabled(bool enabled);
    bool is_enabled() const;
    
protected:
    // 验证输入
    Result<void> validate_inputs() const;
    
    // 错误处理
    void set_error(const String& message);
    void clear_error();
    
    // 执行时间记录
    void record_execute_time(uint64_t microseconds);
};
```

#### 节点信息结构

```cpp
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
```

#### 内部数据成员

```cpp
protected:
    String instance_id_;              // 实例唯一ID
    NodeInfo info_;                   // 节点信息
    ParamSet params_;                 // 参数集合
    
    HashMap<String, Data> inputs_;    // 输入数据映射
    HashMap<String, Data> outputs_;   // 输出数据映射
    
    // 连接关系
    struct Connection {
        WeakPtr target;
        String target_port;
    };
    HashMap<String, Vector<Connection>> connections_;
    
    std::atomic<NodeState> state_;    // 节点状态
    String error_message_;            // 错误信息
    bool enabled_;                    // 是否启用
    uint64_t last_execute_time_;      // 上次执行时间
```

#### 生命周期

```
┌──────────────┐
│   创建节点   │ (NodeFactory::create)
└──────────────┘
       │
       ▼
┌──────────────┐
│   初始化     │ (init())
└──────────────┘
       │
       ▼
┌──────────────┐
│   设置参数   │ (set_param)
│   建立连接   │ (connect_output)
└──────────────┘
       │
       ▼
┌──────────────────────┐
│   执行循环           │
│   ┌────────────────┐ │
│   │ set_input      │ │ ← 上游节点输出
│   └────────────────┘ │
│          │           │
│          ▼           │
│   ┌────────────────┐ │
│   │ execute()      │ │ ← 核心执行逻辑
│   └────────────────┘ │
│          │           │
│          ▼           │
│   ┌────────────────┐ │
│   │ set_output     │ │ → 传播到下游节点
│   └────────────────┘ │
└──────────────────────┘
       │
       ▼
┌──────────────┐
│   重置       │ (reset())
└──────────────┘
       │
       ▼
┌──────────────┐
│   销毁       │
└──────────────┘
```

### 2.2 Data数据系统

**定义位置**: `ovf-core/include/ovf/core/data.h`

#### 数据容器设计

```cpp
class Data {
public:
    Data() : type_(DataType::None) {}
    
    // 多类型构造函数
    explicit Data(int32_t value);
    explicit Data(double value);
    explicit Data(bool value);
    explicit Data(const String& value);
    explicit Data(const ImageData& value);
    explicit Data(const Region& value);
    explicit Data(const Pose& value);
    explicit Data(const Point3Df& value);
    explicit Data(const PointCloudData& value);
    explicit Data(const DepthImageData& value);
    
    // 类型检查
    DataType type() const;
    bool is_valid() const;
    bool is_number() const;
    bool is_string() const;
    bool is_image() const;
    bool is_region() const;
    bool is_pose() const;
    bool is_pointcloud() const;
    
    // 类型转换
    double as_number(double default_val = 0.0) const;
    int32_t as_int(int32_t default_val = 0) const;
    const String& as_string(const String& default_val = "") const;
    const ImageData& as_image() const;
    const Region& as_region() const;
    const Pose& as_pose() const;
    
    // 字符串表示
    String to_string() const;
    
    // 克隆
    Data clone() const;

private:
    DataType type_;
    std::variant<
        std::monostate,
        double,
        bool,
        String,
        ImageData,
        Region,
        Pose,
        Point3Df,
        PointCloudData,
        DepthImageData
    > value_;
};
```

#### 数据类型枚举

```cpp
enum class DataType : uint8_t {
    None = 0,
    Image = 1,        // 图像数据
    Number = 2,       // 数值
    String = 3,       // 字符串
    Boolean = 4,      // 布尔值
    Array = 5,        // 数组
    Object = 6,       // 对象
    Point = 7,        // 点
    Region = 8,       // 区域
    Pose = 9,         // 位姿
    PointCloud = 10,  // 点云
    DepthImage = 11,  // 深度图
    Any = 255         // 任意类型
};
```

#### 图像数据结构

```cpp
struct ImageData {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 1;
    ImageFormat format = ImageFormat::Unknown;
    ByteArray data;          // 像素数据
    uint64_t timestamp = 0;  // 时间戳
    uint32_t frame_id = 0;   // 帧ID
    String source_id;        // 来源ID
    
    bool empty() const;
    size_t size() const;
};
```

#### 参数集合

```cpp
class ParamSet {
public:
    void set(const String& key, const Data& value);
    Data get(const String& key, const Data& default_val = Data{}) const;
    bool has(const String& key) const;
    
    // 便捷方法
    double get_number(const String& key, double default_val = 0.0) const;
    int32_t get_int(const String& key, int32_t default_val = 0) const;
    String get_string(const String& key, const String& default_val = "") const;
    bool get_bool(const String& key, bool default_val = false) const;
    
private:
    HashMap<String, Data> params_;
};
```

### 2.3 Result结果系统

**定义位置**: `ovf-core/include/ovf/core/error.h`

#### Result模板类

```cpp
template<typename T = void>
class Result {
public:
    // 成功构造
    static Result<T> success(const T& value = T{}) {
        return Result<T>(value, ErrorCode::Success, "");
    }
    
    // 失败构造
    static Result<T> failure(ErrorCode code, const String& message = "") {
        return Result<T>(T{}, code, message);
    }
    
    // 状态检查
    bool is_success() const;
    bool is_failure() const;
    
    // 获取值（失败时抛出异常）
    const T& value() const;
    T& value();
    
    // 错误信息
    ErrorCode code() const;
    const String& message() const;
    
    // 操作符重载
    explicit operator bool() const;
    const T& operator*() const;
    const T* operator->() const;
};
```

#### void特化

```cpp
template<>
class Result<void> {
public:
    static Result<void> success();
    static Result<void> failure(ErrorCode code, const String& message = "");
    
    bool is_success() const;
    bool is_failure() const;
    ErrorCode code() const;
    const String& message() const;
    explicit operator bool() const;
};
```

#### 错误码分类

```cpp
enum class ErrorCode : int32_t {
    // 通用错误 (1-99)
    Success = 0,
    Unknown = 1,
    InvalidParameter = 2,
    Timeout = 6,
    InvalidData = 7,
    NotFound = 8,
    
    // 流程引擎错误 (100-199)
    FlowNotFound = 100,
    NodeNotFound = 101,
    InvalidFlow = 102,
    ExecutionFailed = 105,
    CyclicDependency = 106,
    
    // 硬件错误 (200-299)
    DeviceNotFound = 200,
    DeviceOpenFailed = 201,
    CameraCaptureFailed = 204,
    
    // 算法错误 (300-399)
    AlgorithmInitFailed = 300,
    AlgorithmExecFailed = 301,
    InvalidModel = 302,
    
    // 通信错误 (400-499)
    ConnectionLost = 400,
    ProtocolError = 401,
    
    // 文件错误 (500-599)
    FileNotFound = 500,
    FileParseFailed = 502,
    
    // 插件错误 (600-699)
    PluginLoadFailed = 600,
    PluginNotFound = 601,
};
```

#### 使用示例

```cpp
// 返回成功
Result<void> result = Result<void>::success();

// 返回失败
Result<void> result = Result<void>::failure(ErrorCode::InvalidParameter, "Invalid input");

// 检查结果
if (result.is_failure()) {
    OVF_ERROR() << "Error: " << result.message();
    return result;
}

// 链式错误传播
OVF_RETURN_IF_ERROR(some_operation());
```

### 2.4 FlowEngine流程引擎

**定义位置**: `ovf-core/include/ovf/core/flow.h`

#### FlowEngine类

```cpp
class FlowEngine {
public:
    using Ptr = std::shared_ptr<FlowEngine>;
    
    FlowEngine();
    ~FlowEngine();
    
    // 流程管理
    Result<void> load_flow(const FlowDef& flow_def);
    Result<void> load_from_file(const String& filepath);
    Result<void> save_to_file(const String& filepath);
    void clear();
    
    // 节点访问
    INode::Ptr get_node(const String& instance_id) const;
    Vector<INode::Ptr> get_all_nodes() const;
    
    // 执行
    FlowResult run(FlowContext& context);
    FlowResult run_node(const String& node_id, FlowContext& context);
    
    // 单步执行
    void step_begin();
    Result<INode::Ptr> step_next();
    bool step_has_more() const;
    void step_end();
    
    // 执行模式
    enum class ExecutionMode {
        Sequential,    // 顺序执行
        Parallel,      // 并行执行
        DataDriven     // 数据驱动
    };
    
    void set_execution_mode(ExecutionMode mode);
    
    // 拓扑分析
    Result<Vector<String>> get_execution_order() const;
    Result<Vector<String>> get_dependencies(const String& node_id) const;
    Result<Vector<String>> get_dependents(const String& node_id) const;
};
```

#### 执行流程

```
┌──────────────────────────────────────────────────────────────────┐
│                      FlowEngine执行流程                          │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐                                               │
│  │ load_flow    │ 加载流程定义                                   │
│  └──────────────┘                                               │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────────────────────────────────────┐               │
│  │              创建节点实例                     │               │
│  │  for each NodeInstance in FlowDef.nodes:    │               │
│  │      node = NodeFactory::create(type_id)    │               │
│  │      node->set_param(params)                │               │
│  └──────────────────────────────────────────────┘               │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────────────────────────────────────┐               │
│  │              建立连接关系                     │               │
│  │  for each connection:                       │               │
│  │      source->connect_output(port, target)   │               │
│  └──────────────────────────────────────────────┘               │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────────────────────────────────────┐               │
│  │              拓扑排序                         │               │
│  │  execution_order = topological_sort()       │               │
│  └──────────────────────────────────────────────┘               │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────────────────────────────────────┐               │
│  │              执行节点                         │               │
│  │  for node_id in execution_order:            │               │
│  │      node->init()                           │               │
│  │      node->execute(context)                 │               │
│  │      if failed: break                       │               │
│  └──────────────────────────────────────────────┘               │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────┐                                               │
│  │ return result│                                               │
│  └──────────────┘                                               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

#### FlowContext

```cpp
class FlowContext {
public:
    FlowContext();
    
    // 状态控制
    void pause();
    void resume();
    void stop();
    bool is_paused() const;
    bool is_stopped() const;
    
    // 全局变量
    void set_variable(const String& key, const Data& value);
    Data get_variable(const String& key, const Data& default_val = Data{}) const;
    
    // 流程信息
    uint64_t current_frame() const;
    void increment_frame();
    
    // 回调
    using NodeCallback = std::function<void(const String& node_id, NodeState state)>;
    void set_node_callback(NodeCallback callback);
};
```

#### FlowRunner

```cpp
class FlowRunner {
public:
    using Ptr = std::shared_ptr<FlowRunner>;
    
    enum class RunMode {
        Once,        // 单次执行
        Continuous,  // 连续执行
        Triggered    // 触发执行
    };
    
    FlowRunner();
    
    // 设置引擎
    void set_engine(FlowEngine::Ptr engine);
    
    // 运行控制
    Result<void> start(RunMode mode);
    void stop();
    void trigger();  // 触发执行
    
    // 状态
    bool is_running() const;
    RunMode run_mode() const;
    
    // 统计
    uint64_t total_runs() const;
    uint64_t success_runs() const;
    uint64_t failed_runs() const;
    double average_time_ms() const;
};
```

### 2.5 NodeFactory节点工厂

**定义位置**: `ovf-core/include/ovf/core/node.h`

#### 工厂设计

```cpp
class NodeFactory {
public:
    using Creator = std::function<INode::Ptr(const String&)>;
    
    static NodeFactory& instance();  // 单例模式
    
    // 注册节点
    void register_node(const String& type_id, Creator creator, const NodeInfo& info);
    
    // 创建节点
    INode::Ptr create(const String& type_id, const String& instance_id);
    
    // 获取信息
    const NodeInfo* get_info(const String& type_id) const;
    Vector<String> get_all_types() const;
    bool has_type(const String& type_id) const;

private:
    HashMap<String, Creator> creators_;
    HashMap<String, NodeInfo> infos_;
};
```

#### 注册宏

```cpp
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
```

#### 使用示例

```cpp
// 定义节点
class MyThresholdNode : public INode {
public:
    MyThresholdNode(const String& instance_id)
        : INode(instance_id, NodeInfo{
            "threshold", "Threshold", "Image Processing",
            "Binary threshold segmentation",
            "0.1.0", "OVF Team",
            {DataPort{"image", "Image", DataType::Image, true}},
            {DataPort{"binary", "Binary", DataType::Image, true}},
            {ParamDef{"threshold_value", "Threshold", DataType::Number, Data(128.0)}
        }} {}
    
    Result<void> execute(FlowContext& context) override {
        // 实现阈值分割
        ...
    }
};

// 注册节点（静态初始化）
OVF_REGISTER_NODE(MyThresholdNode, "threshold", MyThresholdNode::make_info())
```

---

## 3. 数据流设计

### 3.1 输入输出端口

#### DataPort结构

```cpp
struct DataPort {
    String id;              // 端口ID
    String name;            // 端口名称
    DataType data_type;     // 数据类型
    bool required;          // 是否必须
    Data default_value;     // 默认值
    String description;     // 描述
    
    DataPort(const String& id, const String& name, DataType type, 
             bool required = false, const Data& default_val = Data{});
};
```

#### 端口定义示例

```cpp
// 输入端口
Vector<DataPort> inputs = {
    DataPort{"image", "Input Image", DataType::Image, true},
    DataPort{"threshold", "Threshold", DataType::Number, false, Data(128.0)},
    DataPort{"invert", "Invert", DataType::Boolean, false, Data(false)}
};

// 输出端口
Vector<DataPort> outputs = {
    DataPort{"binary", "Binary Image", DataType::Image, true},
    DataPort{"count", "Object Count", DataType::Number, false, Data(0)}
};
```

### 3.2 数据类型系统

#### 基础类型

| 类型 | DataType值 | 说明 |
|------|------------|------|
| None | 0 | 空数据 |
| Image | 1 | 图像数据 (ImageData) |
| Number | 2 | 数值 (double) |
| String | 3 | 字符串 (std::string) |
| Boolean | 4 | 布尔值 (bool) |
| Point | 7 | 点坐标 (Point3Df) |
| Region | 8 | 区域 (Region) |
| Pose | 9 | 位姿 (Pose) |
| PointCloud | 10 | 点云 (PointCloudData) |
| DepthImage | 11 | 深度图 (DepthImageData) |

#### 图像格式

```cpp
enum class ImageFormat : uint8_t {
    Unknown = 0,
    Mono8 = 1,      // 8位单通道灰度
    Mono16 = 2,     // 16位单通道灰度
    RGB8 = 3,       // 8位RGB
    RGBA8 = 4,      // 8位RGBA
    BGR8 = 5,       // 8位BGR
    BGRA8 = 6,      // 8位BGRA
    Float32 = 7     // 32位浮点
};
```

### 3.3 数据传递机制

#### 推送模式

```
┌──────────────┐
│  Source Node │
│              │
│  set_output  │
│  ("image")   │
└──────────────┘
       │
       │ 数据传播
       ▼
┌──────────────────────────────────────────┐
│         connections_["image"]             │
│  ┌────────────────┐  ┌────────────────┐  │
│  │ Connection 1   │  │ Connection 2   │  │
│  │ target=node1   │  │ target=node2   │  │
│  │ port="input"   │  │ port="image"   │  │
│  └────────────────┘  └────────────────┘  │
└──────────────────────────────────────────┘
       │                    │
       ▼                    ▼
┌──────────────┐    ┌──────────────┐
│   Node 1     │    │   Node 2     │
│  set_input   │    │  set_input   │
│  ("input")   │    │  ("image")   │
└──────────────┘    └──────────────┘
```

#### 实现逻辑 (node.cpp)

```cpp
void INode::set_output(const String& port_id, const Data& data) {
    outputs_[port_id] = data;
    
    // 自动传播数据到下游节点
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
```

---

## 4. 插件架构

### 4.1 插件接口设计

#### HAL设备接口

```cpp
// 设备基类
class IDevice {
public:
    using Ptr = std::shared_ptr<IDevice>;
    
    virtual ~IDevice() = default;
    
    // 基本信息
    virtual const DeviceInfo& info() const = 0;
    virtual DeviceState state() const = 0;
    
    // 连接管理
    virtual Result<void> open() = 0;
    virtual Result<void> close() = 0;
    virtual bool is_open() const = 0;
    
    // 参数设置
    virtual Result<void> set_param(const String& key, const Data& value) = 0;
    virtual Result<Data> get_param(const String& key) const = 0;
    
    // 错误信息
    virtual String last_error() const = 0;
};
```

#### 相机接口 (GenICam标准)

```cpp
class ICamera : public IDevice {
public:
    using Ptr = std::shared_ptr<ICamera>;
    
    // GenICam特性
    virtual Vector<GenICamFeatureInfo> get_genicam_features() const = 0;
    virtual Result<void> set_feature_value(const String& name, const Data& value) = 0;
    virtual Result<Data> get_feature_value(const String& name) const = 0;
    
    // 采集控制
    virtual Result<void> start_capture() = 0;
    virtual Result<void> stop_capture() = 0;
    virtual Result<ImageData> capture_frame(uint32_t timeout_ms = 1000) = 0;
    
    // 触发控制
    virtual Result<void> set_trigger_mode(TriggerMode mode) = 0;
    virtual Result<void> send_soft_trigger() = 0;
    virtual TriggerMode get_trigger_mode() const = 0;
    
    // 便捷接口
    virtual Result<void> set_exposure(double us) = 0;
    virtual Result<void> set_gain(double gain) = 0;
    virtual Result<void> set_white_balance(double r, double g, double b) = 0;
};
```

### 4.2 插件加载机制

#### HALManager

```cpp
class HALManager {
public:
    static HALManager& instance();
    
    // 设备注册
    template<typename T>
    void register_driver(const String& driver_type, 
                         std::function<typename T::Ptr(const DeviceInfo&)> creator);
    
    // 设备创建
    ICamera::Ptr create_camera(const String& driver_type, const DeviceInfo& info);
    
    // 设备枚举
    Vector<DeviceInfo> enumerate_cameras(const String& driver_type = "");
    
    // 设备管理
    void add_device(IDevice::Ptr device);
    void remove_device(const String& device_id);
    IDevice::Ptr get_device(const String& device_id) const;

private:
    HashMap<String, std::function<IDevice::Ptr(const DeviceInfo&)>> creators_;
    HashMap<String, IDevice::Ptr> devices_;
};
```

#### 驱动注册宏

```cpp
#define OVF_REGISTER_CAMERA_DRIVER(DriverClass, driver_type) \
    namespace { \
        struct DriverClass##Registrar { \
            DriverClass##Registrar() { \
                ovf::hal::HALManager::instance().register_driver<ovf::hal::ICamera>( \
                    driver_type, [](const ovf::hal::DeviceInfo& info) -> ovf::hal::ICamera::Ptr { \
                        return std::make_shared<DriverClass>(info); \
                    }); \
            } \
        } registrar_##DriverClass; \
    }
```

### 4.3 插件生命周期

```
┌──────────────────────────────────────────────────────────────────┐
│                      插件生命周期                                │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Phase 1: 编译阶段                                               │
│  ┌──────────────────────────────────────────────┐               │
│  │  OVF_REGISTER_NODE / OVF_REGISTER_DRIVER     │               │
│  │  → 静态初始化对象                             │               │
│  │  → 编译到DLL/共享库                           │               │
│  └──────────────────────────────────────────────┘               │
│                                                                  │
│  Phase 2: 加载阶段                                               │
│  ┌──────────────────────────────────────────────┐               │
│  │  加载DLL                                      │               │
│  │  → 静态对象构造                               │               │
│  │  → 注册到NodeFactory/HALManager              │               │
│  └──────────────────────────────────────────────┘               │
│                                                                  │
│  Phase 3: 使用阶段                                               │
│  ┌──────────────────────────────────────────────┐               │
│  │  NodeFactory::create(type_id)                │               │
│  │  HALManager::create_camera(driver_type)      │               │
│  │  → 创建节点/设备实例                          │               │
│  └──────────────────────────────────────────────┘               │
│                                                                  │
│  Phase 4: 卸载阶段                                               │
│  ┌──────────────────────────────────────────────┐               │
│  │  卸载DLL                                      │               │
│  │  → 静态对象析构                               │               │
│  │  → 从工厂移除注册                             │               │
│  └──────────────────────────────────────────────┘               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 5. 扩展机制

### 5.1 自定义节点开发

#### 开发步骤

1. **定义节点类**
```cpp
class CustomNode : public INode {
public:
    CustomNode(const String& instance_id)
        : INode(instance_id, make_info()) {}
    
    Result<void> execute(FlowContext& context) override {
        // 1. 获取输入
        Data input = get_input("input");
        
        // 2. 获取参数
        int param = get_param("param", Data(0)).as_int();
        
        // 3. 执行算法
        Data output = process(input, param);
        
        // 4. 设置输出
        set_output("output", output);
        
        return Result<void>::success();
    }
    
    static NodeInfo make_info() {
        return NodeInfo{
            "custom_node",
            "Custom Node",
            "Custom",
            "Custom processing node",
            "1.0.0",
            "Developer",
            {DataPort{"input", "Input", DataType::Any, true}},
            {DataPort{"output", "Output", DataType::Any, true}},
            {ParamDef{"param", "Parameter", DataType::Number, Data(0)}
        };
    }
};
```

2. **注册节点**
```cpp
OVF_REGISTER_NODE(CustomNode, "custom_node", CustomNode::make_info())
```

3. **编译为插件DLL**

### 5.2 自定义插件开发

#### 插件目录结构

```
ovf-plugin-custom/
├── include/
│   └── ovf/
│       └── plugins/
│           └── custom_plugin.h
├── src/
│   └── custom_plugin.cpp  # 包含注册宏
└── CMakeLists.txt
```

#### 插件导出函数

```cpp
// 插件初始化函数
extern "C" OVF_PLUGIN_API void init_plugin() {
    // 执行必要的初始化
    OVF_INFO() << "Custom plugin initialized";
}

// 插件信息函数
extern "C" OVF_PLUGIN_API PluginInfo get_plugin_info() {
    return PluginInfo{
        "custom_plugin",
        "Custom Plugin",
        "1.0.0",
        "Custom functionality"
    };
}
```

### 5.3 自定义数据类型

#### 扩展DataType

```cpp
// 1. 在types.h添加新类型
enum class DataType : uint8_t {
    ...
    CustomType = 20,  // 自定义类型
};

// 2. 定义数据结构
struct CustomData {
    Vector<float> values;
    String metadata;
    
    bool empty() const { return values.empty(); }
};

// 3. 扩展Data类
class Data {
public:
    explicit Data(const CustomData& value) 
        : type_(DataType::CustomType), value_(value) {}
    
    bool is_custom() const { return type_ == DataType::CustomType; }
    const CustomData& as_custom() const {
        static CustomData empty;
        return is_custom() ? std::get<CustomData>(value_) : empty;
    }
};
```

---

## 6. 性能优化

### 6.1 CUDA加速架构

**位置**: `ovf-algorithm/include/ovf/algorithm/cuda_accelerator.h`

#### CudaAccelerator类

```cpp
class CudaAccelerator {
public:
    // 可用性检测
    static bool is_cuda_available();
    static bool get_gpu_info(String& name, size_t& memory, String& capability);
    
    // 核心算子（5-20x加速）
    static ErrorCode gaussian_blur(const ImageData& input, ImageData& output, 
                                   float sigma, int kernel_size = 0);
    static ErrorCode sobel_filter(const ImageData& input, ImageData& output);
    static ErrorCode threshold(const ImageData& input, ImageData& output, 
                               ThresholdType type, float value);
    static ErrorCode morphology(const ImageData& input, ImageData& output, 
                                MorphOpCUDA op);
    static ErrorCode histogram(const ImageData& input, Vector<int>& hist);
    static ErrorCode template_match(const ImageData& image, 
                                    const ImageData& template_image,
                                    TemplateMatchResult& result);
    static ErrorCode blob_analysis(const ImageData& binary, Vector<Blob>& blobs);
    static ErrorCode resize(const ImageData& input, ImageData& output, 
                            float scale_x, float scale_y);
    
    // 批量处理
    static ErrorCode gaussian_blur_batch(const Vector<ImageData>& inputs,
                                         Vector<ImageData>& outputs);
    
    // 内存管理
    static Ptr<CUDABuffer> create_buffer(size_t size);
    static void synchronize_stream(CUDAStream* stream);
};
```

#### 加速比统计

| 算子 | CPU时间 | GPU时间 | 加速比 |
|------|---------|---------|--------|
| 高斯模糊 (5x5) | 15ms | 1.5ms | 10x |
| Sobel边缘 | 12ms | 1.5ms | 8x |
| 自适应阈值 | 25ms | 2.5ms | 10x |
| 形态学 | 18ms | 3ms | 6x |
| 直方图 | 8ms | 0.5ms | 15x |
| 模板匹配 | 100ms | 5ms | 20x |
| Blob分析 | 30ms | 2.5ms | 12x |
| 图像缩放 | 20ms | 2ms | 10x |

#### CUDA节点基类

```cpp
class CudaAcceleratorNode : public INode {
public:
    CudaAcceleratorNode(const String& instance_id, const NodeInfo& info);
    
    void set_force_cpu(bool force_cpu);  // 强制CPU模式
    bool used_gpu() const;               // 是否使用GPU
    const CUDAStats& get_stats() const;  // 执行统计

protected:
    bool force_cpu_ = false;
    bool used_gpu_ = false;
    CUDAStats stats_;
    Ptr<CUDAStream> stream_;
};
```

### 6.2 并行执行设计

#### 执行模式

```cpp
enum class ExecutionMode {
    Sequential,    // 顺序执行（默认）
    Parallel,      // 并行执行（独立节点）
    DataDriven     // 数据驱动执行
};
```

#### 并行执行策略

```
┌──────────────────────────────────────────────────────────────────┐
│                      并行执行拓扑                                │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│       [Camera]                                                   │
│          │                                                       │
│          ▼                                                       │
│    ┌─────────────────┐                                          │
│    │                 │                                          │
│    ▼                 ▼                                          │
│ [Gaussian]      [Sobel]     ← 并行执行                          │
│    │                 │                                          │
│    ▼                 ▼                                          │
│ [Threshold]    [Canny]       ← 并行执行                          │
│    │                 │                                          │
│    └─────────┬───────┘                                          │
│              │                                                   │
│              ▼                                                   │
│         [Blob分析]     ← 需等待两个输入                          │
│              │                                                   │
│              ▼                                                   │
│         [输出]                                                   │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

#### 实现要点

```cpp
FlowResult FlowEngine::execute_parallel(FlowContext& context) {
    // 1. 分析依赖图，识别并行层
    auto layers = analyze_parallel_layers();
    
    for (const auto& layer : layers) {
        // 2. 并行执行当前层的节点
        std::vector<std::future<FlowResult>> futures;
        for (const auto& node_id : layer) {
            futures.push_back(std::async(std::launch::async, 
                [this, &node_id, &context]() {
                    return execute_node(nodes_[node_id], context);
                }));
        }
        
        // 3. 等待所有节点完成
        for (auto& f : futures) {
            auto result = f.get();
            if (!result.success) {
                return result;
            }
        }
    }
    
    return FlowResult::ok();
}
```

### 6.3 内存管理策略

#### 环形缓冲区设计

```cpp
struct BufferConfig {
    uint32_t buffer_count = 5;     // 缓冲区数量
    uint32_t max_queue_size = 10;  // 最大队列大小
    bool use_ring_buffer = true;   // 使用环形缓冲区
};
```

#### 图像数据生命周期

```
┌──────────────┐
│   捕获图像   │
└──────────────┘
       │
       ▼
┌──────────────────────────┐
│ 环形缓冲区 (Ring Buffer) │
│  [Frame 0] [Frame 1] ... │
└──────────────────────────┘
       │
       │ 共享指针传递
       ▼
┌──────────────┐
│   节点处理   │ (引用计数)
└──────────────┘
       │
       ▼
┌──────────────┐
│   输出传递   │ (智能指针)
└──────────────┘
       │
       ▼
┌──────────────┐
│   自动释放   │ (引用计数归零)
└──────────────┘
```

#### CUDA Pinned Memory

```cpp
class CUDABuffer {
public:
    CUDABuffer(size_t size);
    
    void* data();
    bool is_pinned() const;  // 是否使用固定内存
    
    // 优势：
    // 1. CPU-GPU传输更快（DMA直接访问）
    // 2. 异步传输可行
    // 3. 避免额外拷贝
};
```

---

## 附录

### A. 模块依赖关系

```
ovf-web-server
    └── ovf-sdk-cpp
        └── ovf-algorithm
        │       └── ovf-core
        └── ovf-core

ovf-sdk-python
    └── ovf-core

ovf-plugin-camera
    └── ovf-core (hal)

ovf-algorithm
    └── ovf-core

ovf-core (无外部依赖)
```

### B. 编译配置

```cmake
# CMakeLists.txt 核心配置
cmake_minimum_required(VERSION 3.16)
project(OpenVisionFlow VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

# 模块
add_subdirectory(ovf-core)
add_subdirectory(ovf-algorithm)
add_subdirectory(ovf-sdk)
add_subdirectory(ovf-sdk-python)
add_subdirectory(ovf-web-server)
add_subdirectory(ovf-plugins)
add_subdirectory(ovf-plugin-camera)
```

### C. 目录结构总览

```
OpenVisionFlow/
├── ovf-core/              # 核心模块
│   ├── include/ovf/core/  # 核心头文件
│   ├── include/ovf/hal/   # 硬件抽象层
│   └── src/               # 实现文件
├── ovf-algorithm/         # 算法模块
│   ├── include/ovf/algorithm/
│   └── src/
├── ovf-plugins/           # 插件系统
│   ├── algorithm/
│   ├── communication/
│   ├── custom/
│   └── hardware/
├── ovf-plugin-camera/     # 相机插件
├── ovf-sdk/               # C++ SDK
│   └ cpp/
├── ovf-sdk-python/        # Python SDK
├── ovf-web-server/        # Web服务
├── tests/                 # 测试模块
├── docs/                  # 文档
├── examples/              # 示例
└── build/                 # 构建输出
```

---

**文档版本**: 0.1.0  
**最后更新**: 2025-07-05