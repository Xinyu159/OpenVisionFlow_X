# OpenVisionFlow API 参考文档

版本：0.1.0

本文档为开发者提供完整的 OpenVisionFlow API 参考。

---

## 目录

1. [核心API (ovf-core)](#1-核心api-ovf-core)
   - [INode 基类](#inode-基类)
   - [FlowContext 流程上下文](#flowcontext-流程上下文)
   - [Data 数据类型](#data-数据类型)
   - [Result 结果类型](#result-结果类型)
   - [NodeInfo 节点信息](#nodeinfo-节点信息)
   - [NodeFactory 节点工厂](#nodefactory-节点工厂)
   - [FlowEngine 流程引擎](#flowengine-流程引擎)

2. [算法API (ovf-algorithm)](#2-算法api-ovf-algorithm)
   - [图像处理节点](#图像处理节点)
   - [亚像素精度节点](#亚像素精度节点)
   - [Blob分析节点](#blob分析节点)
   - [测量节点](#测量节点)
   - [OCR节点](#ocr节点)
   - [半导体晶圆检测节点](#半导体晶圆检测节点)
   - [汽车零部件检测节点](#汽车零部件检测节点)

3. [Python SDK API (ovf-sdk-python)](#3-python-sdk-api-ovf-sdk-python)
   - [ovf.Image 图像类](#ovfimage-图像类)
   - [ovf.Flow 流程类](#ovfflow-流程类)
   - [ovf.Node 节点类](#ovfnode-节点类)
   - [ovf.Camera 相机类](#ovfcamera-相机类)
   - [ovf.Runner 运行器类](#ovfrunner-运行器类)

4. [Web API (ovf-web-server)](#4-web-api-ovf-web-server)
   - [REST API](#rest-api)
   - [WebSocket API](#websocket-api)

---

## 1. 核心API (ovf-core)

### INode 基类

所有算法节点的基类，定义了节点的通用接口和行为。

#### 类定义

```cpp
namespace ovf {

class INode : public std::enable_shared_from_this<INode> {
public:
    using Ptr = std::shared_ptr<INode>;
    using WeakPtr = std::weak_ptr<INode>;
    
    INode(const String& instance_id, const NodeInfo& info);
    virtual ~INode() = default;
    
    // 基本信息
    const String& instance_id() const;
    const NodeInfo& info() const;
    NodeState state() const;
    const String& error_message() const;
    
    // 参数管理
    void set_param(const String& key, const Data& value);
    Data get_param(const String& key, const Data& default_val = Data{}) const;
    const ParamSet& params() const;
    
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
    virtual Result<void> init();
    virtual Result<void> execute(FlowContext& context) = 0;
    virtual Result<void> reset();
    
    // 启用/禁用
    void set_enabled(bool enabled);
    bool is_enabled() const;
    
    // 执行时间
    uint64_t last_execute_time() const;
    
protected:
    Result<void> validate_inputs() const;
    void set_error(const String& message);
    void clear_error();
    void record_execute_time(uint64_t microseconds);
};

} // namespace ovf
```

#### 方法说明

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `instance_id()` | 无 | `const String&` | 获取节点实例ID |
| `info()` | 无 | `const NodeInfo&` | 获取节点类型信息 |
| `state()` | 无 | `NodeState` | 获取节点状态 (Idle/Running/Success/Failed/Disabled) |
| `error_message()` | 无 | `const String&` | 获取错误消息 |
| `set_param()` | `key`, `value` | `void` | 设置节点参数 |
| `get_param()` | `key`, `default_val` | `Data` | 获取节点参数 |
| `set_input()` | `port_id`, `data` | `void` | 设置输入端口数据 |
| `get_input()` | `port_id` | `Data` | 获取输入端口数据 |
| `has_input()` | `port_id` | `bool` | 检查输入端口是否有数据 |
| `set_output()` | `port_id`, `data` | `void` | 设置输出端口数据 |
| `get_output()` | `port_id` | `Data` | 获取输出端口数据 |
| `connect_output()` | `port_id`, `target`, `target_port` | `void` | 连接输出端口到目标节点 |
| `disconnect_output()` | `port_id`, `target` | `void` | 断开输出端口连接 |
| `init()` | 无 | `Result<void>` | 初始化节点 |
| `execute()` | `context` | `Result<void>` | 执行节点（必须实现） |
| `reset()` | 无 | `Result<void>` | 重置节点状态 |
| `set_enabled()` | `enabled` | `void` | 启用/禁用节点 |
| `is_enabled()` | 无 | `bool` | 检查节点是否启用 |
| `last_execute_time()` | 无 | `uint64_t` | 获取上次执行时间（微秒） |

#### 使用示例

```cpp
// 创建自定义节点
class MyNode : public ovf::INode {
public:
    MyNode(const String& id) : INode(id, make_info()) {}
    
    Result<void> execute(FlowContext& context) override {
        // 获取输入图像
        auto input = get_input("image");
        if (input.is_none()) {
            return Result<void>::failure(ErrorCode::InvalidData, "No input image");
        }
        
        // 执行处理逻辑...
        ImageData output = process_image(input.as_image());
        
        // 设置输出
        set_output("output", Data(output));
        
        return Result<void>::success();
    }
    
    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "MyNode";
        info.name = "自定义节点";
        info.category = "Custom";
        info.description = "示例自定义节点";
        info.inputs = { DataPort("image", "输入图像", DataType::Image, true) };
        info.outputs = { DataPort("output", "输出图像", DataType::Image) };
        return info;
    }
};

// 注册节点
OVF_REGISTER_NODE(MyNode, "MyNode", MyNode::make_info())
```

---

### FlowContext 流程上下文

流程执行上下文，管理执行过程中的状态和全局变量。

#### 类定义

```cpp
namespace ovf {

class FlowContext {
public:
    FlowContext();
    ~FlowContext();
    
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
    void notify_node_state(const String& node_id, NodeState state);
};

} // namespace ovf
```

#### 方法说明

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `pause()` | 无 | `void` | 暂停流程执行 |
| `resume()` | 无 | `void` | 恢复流程执行 |
| `stop()` | 无 | `void` | 停止流程执行 |
| `is_paused()` | 无 | `bool` | 检查是否暂停 |
| `is_stopped()` | 无 | `bool` | 检查是否停止 |
| `set_variable()` | `key`, `value` | `void` | 设置全局变量 |
| `get_variable()` | `key`, `default_val` | `Data` | 获取全局变量 |
| `current_frame()` | 无 | `uint64_t` | 获取当前帧号 |
| `increment_frame()` | 无 | `void` | 增加帧号计数 |
| `set_node_callback()` | `callback` | `void` | 设置节点状态回调 |

#### 使用示例

```cpp
// 创建流程上下文
ovf::FlowContext context;

// 设置全局变量
context.set_variable("threshold", ovf::Data(50.0));
context.set_variable("model_path", ovf::Data("/path/to/model"));

// 设置节点状态回调
context.set_node_callback([](const String& node_id, ovf::NodeState state) {
    std::cout << "Node " << node_id << " state: " << state << std::endl;
});

// 执行流程
auto engine = ovf::api::create_flow_engine();
engine->load_from_file("flow.json");
auto result = engine->run(context);

// 获取执行结果
if (result.success) {
    std::cout << "Execution time: " << result.total_time_us << " us" << std::endl;
}
```

---

### Data 数据类型

数据容器类，用于节点间数据传递。

#### 类定义

```cpp
namespace ovf {

class Data {
public:
    Data();
    
    // 从各种类型构造
    explicit Data(int32_t value);
    explicit Data(int64_t value);
    explicit Data(float value);
    explicit Data(double value);
    explicit Data(bool value);
    explicit Data(const char* value);
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
    bool is_none() const;
    bool is_number() const;
    bool is_string() const;
    bool is_bool() const;
    bool is_image() const;
    bool is_region() const;
    bool is_pose() const;
    bool is_point() const;
    bool is_pointcloud() const;
    bool is_depth_image() const;
    
    // 获取值
    double as_number(double default_val = 0.0) const;
    int32_t as_int(int32_t default_val = 0) const;
    const String& as_string(const String& default_val = "") const;
    bool as_bool(bool default_val = false) const;
    const ImageData& as_image() const;
    ImageData& as_image();
    const Region& as_region() const;
    const Pose& as_pose() const;
    const Point3Df& as_point() const;
    const PointCloudData& as_pointcloud() const;
    const DepthImageData& as_depth_image() const;
    
    // 转换为字符串
    String to_string() const;
    
    // 复制
    Data clone() const;
};

} // namespace ovf
```

#### 数据类型枚举

| 类型 | 值 | 说明 |
|------|-----|------|
| `DataType::None` | 0 | 无数据 |
| `DataType::Image` | 1 | 图像数据 |
| `DataType::Number` | 2 | 数值 |
| `DataType::String` | 3 | 字符串 |
| `DataType::Boolean` | 4 | 布尔值 |
| `DataType::Array` | 5 | 数组 |
| `DataType::Object` | 6 | 对象 |
| `DataType::Point` | 7 | 点 |
| `DataType::Region` | 8 | 区域 |
| `DataType::Pose` | 9 | 位姿 |
| `DataType::PointCloud` | 10 | 点云 |
| `DataType::DepthImage` | 11 | 深度图 |
| `DataType::Any` | 255 | 任意类型 |

#### 使用示例

```cpp
// 创建不同类型的数据
ovf::Data num_data(42.5);           // 数值
ovf::Data str_data("hello");        // 字符串
ovf::Data bool_data(true);          // 布尔值

// 创建图像数据
ovf::ImageData img;
img.width = 640;
img.height = 480;
img.channels = 1;
img.format = ovf::ImageFormat::Mono8;
img.data.resize(640 * 480);
ovf::Data img_data(img);

// 类型检查和获取值
if (num_data.is_number()) {
    double value = num_data.as_number();
    std::cout << "Value: " << value << std::endl;
}

if (img_data.is_image()) {
    auto& image = img_data.as_image();
    std::cout << "Image size: " << image.width << "x" << image.height << std::endl;
}
```

---

### Result 结果类型

操作结果封装类，用于返回成功/失败状态。

#### 类定义

```cpp
namespace ovf {

template<typename T = void>
class Result {
public:
    // 成功构造
    static Result<T> success(const T& value = T{});
    
    // 失败构造
    static Result<T> failure(ErrorCode code, const String& message = "");
    
    // 状态检查
    bool is_success() const;
    bool is_failure() const;
    
    // 获取值
    const T& value() const;
    T& value();
    
    // 错误信息
    ErrorCode code() const;
    const String& message() const;
    
    // 操作符
    explicit operator bool() const;
    const T& operator*() const;
    T& operator*();
};

// void 特化
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

} // namespace ovf
```

#### 错误码枚举

| 错误码 | 值 | 说明 |
|--------|-----|------|
| `Success` | 0 | 成功 |
| `Unknown` | 1 | 未知错误 |
| `InvalidParameter` | 2 | 参数无效 |
| `NullPointer` | 3 | 空指针 |
| `OutOfRange` | 4 | 超出范围 |
| `Timeout` | 6 | 超时 |
| `InvalidData` | 7 | 数据无效 |
| `FlowNotFound` | 100 | 流程未找到 |
| `NodeNotFound` | 101 | 节点未找到 |
| `InvalidFlow` | 102 | 流程无效 |
| `ExecutionFailed` | 105 | 执行失败 |
| `DeviceNotFound` | 200 | 设备未找到 |
| `AlgorithmInitFailed` | 300 | 算法初始化失败 |
| `AlgorithmExecFailed` | 301 | 算法执行失败 |
| `FileNotFound` | 500 | 文件未找到 |

#### 使用示例

```cpp
// 返回成功结果
ovf::Result<void> result = ovf::Result<void>::success();

// 返回失败结果
ovf::Result<void> error = ovf::Result<void>::failure(
    ovf::ErrorCode::InvalidParameter, 
    "Parameter 'threshold' must be positive"
);

// 检查结果
if (result.is_success()) {
    std::cout << "Success!" << std::endl;
} else {
    std::cout << "Error: " << result.message() << std::endl;
}

// 使用宏简化错误检查
OVF_RETURN_IF_ERROR(result);  // 如果失败则返回
```

---

### NodeInfo 节点信息

节点类型信息结构。

#### 结构定义

```cpp
namespace ovf {

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

struct DataPort {
    String id;              // 端口ID
    String name;            // 端口名称
    DataType data_type;     // 数据类型
    bool required;          // 是否必须
    Data default_value;     // 默认值
    String description;     // 描述
};

struct ParamDef {
    String id;              // 参数ID
    String name;            // 参数名称
    DataType type;          // 参数类型
    Data default_value;     // 默认值
    Data min_value;         // 最小值
    Data max_value;         // 最大值
    Vector<String> options; // 选项列表
    String description;     // 描述
};

} // namespace ovf
```

#### 使用示例

```cpp
// 定义节点信息
ovf::NodeInfo info;
info.id = "ThresholdNode";
info.name = "阈值分割";
info.category = "ImageProcessing";
info.description = "将图像进行阈值分割";

// 定义输入端口
info.inputs.push_back({
    "image", "输入图像", ovf::DataType::Image, true
});

// 定义输出端口
info.outputs.push_back({
    "binary", "二值图像", ovf::DataType::Image
});

// 定义参数
info.params.push_back({
    "threshold", "阈值", ovf::DataType::Number, ovf::Data(128.0)
});
```

---

### NodeFactory 节点工厂

节点注册和创建工厂。

#### 类定义

```cpp
namespace ovf {

class NodeFactory {
public:
    static NodeFactory& instance();
    
    // 注册节点
    void register_node(const String& type_id, Creator creator, const NodeInfo& info);
    
    // 创建节点
    INode::Ptr create(const String& type_id, const String& instance_id);
    
    // 获取节点信息
    const NodeInfo* get_info(const String& type_id) const;
    
    // 获取所有类型
    Vector<String> get_all_types() const;
    
    // 检查类型是否存在
    bool has_type(const String& type_id) const;
    
private:
    NodeFactory() = default;
    using Creator = std::function<INode::Ptr(const String&)>;
};

} // namespace ovf
```

#### 节点注册宏

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
// 注册节点
ovf::NodeFactory::instance().register_node(
    "ThresholdNode",
    [](const String& id) { return std::make_shared<ThresholdNode>(id); },
    ThresholdNode::make_info()
);

// 创建节点实例
auto node = ovf::NodeFactory::instance().create("ThresholdNode", "threshold_1");

// 获取所有注册的节点类型
auto types = ovf::NodeFactory::instance().get_all_types();
for (const auto& type : types) {
    std::cout << "Type: " << type << std::endl;
}

// 使用宏注册（推荐方式）
OVF_REGISTER_NODE(ThresholdNode, "ThresholdNode", ThresholdNode::make_info())
```

---

### FlowEngine 流程引擎

流程执行引擎，负责加载、管理和执行流程。

#### 类定义

```cpp
namespace ovf {

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
        Sequential,     // 顺序执行
        Parallel,       // 并行执行
        DataDriven      // 数据驱动
    };
    
    void set_execution_mode(ExecutionMode mode);
    ExecutionMode execution_mode() const;
    
    // 回调
    using NodeStateCallback = std::function<void(const String& node_id, NodeState state)>;
    void set_node_state_callback(NodeStateCallback callback);
    
    // 拓扑分析
    Result<Vector<String>> get_execution_order() const;
    Result<Vector<String>> get_dependencies(const String& node_id) const;
    Result<Vector<String>> get_dependents(const String& node_id) const;
};

struct FlowResult {
    bool success = false;
    String error_message;
    String failed_node_id;
    uint64_t total_time_us = 0;
    HashMap<String, uint64_t> node_times;
    
    static FlowResult ok();
    static FlowResult fail(const String& msg, const String& node_id = "");
};

struct FlowDef {
    String id;
    String name;
    String description;
    String version;
    Vector<NodeInstance> nodes;
    
    struct NodeInstance {
        String id;
        String type_id;
        String name;
        int32_t x, y;
        bool enabled;
        ParamSet params;
        HashMap<String, InputConnection> input_connections;
        
        struct InputConnection {
            String source_node_id;
            String source_port;
        };
    };
};

} // namespace ovf
```

#### 使用示例

```cpp
// 创建流程引擎
auto engine = ovf::FlowEngine::Ptr(new ovf::FlowEngine());

// 加载流程文件
auto result = engine->load_from_file("flow.json");
if (!result.is_success()) {
    std::cerr << "Failed to load flow: " << result.message() << std::endl;
    return;
}

// 设置执行模式
engine->set_execution_mode(ovf::FlowEngine::ExecutionMode::Sequential);

// 设置节点状态回调
engine->set_node_state_callback([](const String& id, ovf::NodeState state) {
    std::cout << "Node " << id << " -> " << state << std::endl;
});

// 创建执行上下文
ovf::FlowContext context;

// 执行流程
auto exec_result = engine->run(context);
if (exec_result.success) {
    std::cout << "Total time: " << exec_result.total_time_us << " us" << std::endl;
    
    // 输出各节点执行时间
    for (const auto& [node_id, time] : exec_result.node_times) {
        std::cout << node_id << ": " << time << " us" << std::endl;
    }
} else {
    std::cerr << "Execution failed at node: " << exec_result.failed_node_id << std::endl;
    std::cerr << "Error: " << exec_result.error_message << std::endl;
}

// 单步执行模式
engine->step_begin();
while (engine->step_has_more()) {
    auto node = engine->step_next();
    if (node) {
        std::cout << "Executing: " << node->instance_id() << std::endl;
    }
}
engine->step_end();
```

---

## 2. 算法API (ovf-algorithm)

### 图像处理节点

#### SobelEdgeNode - Sobel边缘检测节点

**节点ID**: `SobelEdgeNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `output` | 边缘图像 | Image | 二值边缘图像 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `threshold` | 阈值 | Number | 50 | 边缘强度阈值 |

**使用示例**:
```cpp
auto node = ovf::NodeFactory::instance().create("SobelEdgeNode", "sobel_1");
node->set_param("threshold", ovf::Data(100));
node->set_input("image", input_image_data);
node->execute(context);
auto output = node->get_output("output").as_image();
```

---

#### CannyEdgeNode - Canny边缘检测节点

**节点ID**: `CannyEdgeNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `output` | 边缘图像 | Image | 二值边缘图像 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `low_threshold` | 低阈值 | Number | 50 | 弱边缘阈值 |
| `high_threshold` | 高阈值 | Number | 100 | 强边缘阈值 |

---

#### TemplateMatchNode - 模板匹配节点

**节点ID**: `TemplateMatchNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 源图像 | Image | 是 | 待搜索图像 |
| `template` | 模板图像 | Image | 是 | 模板图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `x` | 位置X | Number | 匹配位置X坐标 |
| `y` | 位置Y | Number | 匹配位置Y坐标 |
| `score` | 匹配分数 | Number | 匹配分数 (NCC: -1~1) |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `method` | 匹配方法 | String | "NCC" | SAD/SSD/NCC |
| `threshold` | 阈值 | Number | 0.7 | 匹配阈值 |

**匹配方法说明**:
| 方法 | 说明 | 分数范围 |
|------|------|----------|
| SAD | 绝对差之和 | 越小越好 |
| SSD | 平方差之和 | 越小越好 |
| NCC | 归一化互相关 | -1~1，越大越好 |

---

### 亚像素精度节点

#### SubpixelEdgeNode - 亚像素边缘定位节点

**节点ID**: `SubpixelEdgeNode`

对标 Halcon `edges_sub_pix`，精度可达 ±0.01像素。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `edges` | 边缘点列表 | Array | SubpixelEdgePoint数组 |
| `count` | 边缘点数 | Number | 检测到的边缘点数量 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `method` | 方法 | String | "taylor" | taylor/parabola/gaussian/zernike |
| `threshold` | 阈值 | Number | 30 | 边缘强度阈值 |
| `sigma` | 平滑系数 | Number | 1.0 | 高斯平滑系数 |

**亚像素定位方法**:
| 方法 | 精度 | 说明 |
|------|------|------|
| Taylor | ±0.01像素 | 二阶泰勒展开（推荐） |
| Parabola | ±0.02像素 | 三点抛物线拟合 |
| Gaussian | ±0.01像素 | 高斯函数拟合 |
| Zernike | ±0.005像素 | Zernike矩法（最高精度） |

---

#### SubpixelCornerNode - 亚像素角点定位节点

**节点ID**: `SubpixelCornerNode`

精度可达 ±0.03像素。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `corners` | 角点列表 | Array | SubpixelCornerPoint数组 |
| `count` | 角点数 | Number | 检测到的角点数量 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `k` | Harris系数 | Number | 0.04 | Harris角点响应系数 |
| `threshold` | 阈值 | Number | 1000 | 角点响应阈值 |
| `window_size` | 窗口大小 | Number | 3 | 邻域窗口大小 |

---

#### SubpixelLineNode - 亚像素直线定位节点

**节点ID**: `SubpixelLineNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `line` | 直线参数 | Object | Line2D结构 |
| `rms_error` | RMS误差 | Number | 拟合误差(像素) |
| `point_count` | 点数 | Number | 参与拟合的点数 |

---

#### SubpixelCircleNode - 亚像素圆定位节点

**节点ID**: `SubpixelCircleNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `circle` | 圆参数 | Object | Circle2D结构 |
| `rms_error` | RMS误差 | Number | 拟合误差(像素) |
| `point_count` | 点数 | Number | 参与拟合的点数 |

---

#### PrecisionTestNode - 精度测试节点

**节点ID**: `PrecisionTestNode`

生成已知参数的合成测试图，用于精度验证。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `test_image` | 测试图像 | Image | 合成测试图像 |
| `ground_truth` | 理论值 | Object | 已知的理论参数 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `pattern_type` | 图案类型 | String | "Circle" | VerticalEdge/HorizontalEdge/Circle/Line/Corner |
| `width` | 图像宽度 | Number | 512 | 生成图像宽度 |
| `height` | 图像高度 | Number | 512 | 生成图像高度 |
| `param1` | 参数1 | Number | 256 | 圆心X/边缘位置等 |
| `param2` | 参数2 | Number | 100 | 圆心Y/半径等 |

---

### Blob分析节点

#### BlobAnalysisNode - Blob分析节点

**节点ID**: `BlobAnalysisNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `binary_image` | 二值图像 | Image | 是 | 二值图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `blobs` | Blob列表 | Array | Blob结构数组 |
| `count` | Blob数量 | Number | 检测到的Blob数 |
| `annotated_image` | 标注图像 | Image | 绘制了边界框的图像 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `min_area` | 最小面积 | Number | 10 | 最小Blob面积(像素) |
| `max_area` | 最大面积 | Number | 1000000 | 最大Blob面积(像素) |

**Blob结构**:
```cpp
struct Blob {
    uint32_t id;            // Blob ID
    uint32_t area;          // 面积
    uint32_t x, y;          // 中心坐标
    uint32_t min_x, max_x;  // X范围
    uint32_t min_y, max_y;  // Y范围
    uint32_t width, height; // 尺寸
    double circularity;     // 圆度
    double aspect_ratio;    // 长宽比
    double orientation;     // 方向角(弧度)
};
```

---

### 测量节点

#### CaliperToolNode - 卡尺测量节点

**节点ID**: `CaliperToolNode`

沿投影方向搜索边缘跳变点。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 灰度图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `edge_points` | 边缘点 | Array | Point2Df数组 |
| `count` | 点数 | Number | 边缘点数量 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `start_x` | 起点X | Number | 0 | 搜索起点X |
| `start_y` | 起点Y | Number | 0 | 搜索起点Y |
| `end_x` | 终点X | Number | 640 | 搜索终点X |
| `end_y` | 终点Y | Number | 480 | 搜索终点Y |
| `width` | 搜索宽度 | Number | 5 | 垂直于投影方向的宽度 |
| `threshold` | 阈值 | Number | 20 | 边缘跳变阈值 |
| `polarity` | 极性 | Number | 0 | 1=亮到暗,-1=暗到亮,0=双向 |

---

#### LineFitNode - 直线拟合节点

**节点ID**: `LineFitNode`

使用最小二乘法拟合直线。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `points` | 点集 | Array | 是 | Point2Df数组 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `line` | 直线 | Object | Line2D结构(ax+by+c=0) |
| `rms_error` | RMS误差 | Number | 拟合误差 |

---

#### CircleFitNode - 圆拟合节点

**节点ID**: `CircleFitNode`

使用最小二乘法拟合圆。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `points` | 点集 | Array | 是 | Point2Df数组 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `circle` | 圆 | Object | Circle2D结构 |
| `rms_error` | RMS误差 | Number | 拟合误差 |

---

#### DistanceMeasureNode - 距离测量节点

**节点ID**: `DistanceMeasureNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `point1` | 点1 | Point | 是 | 第一个点 |
| `point2` | 点2 | Point | 是 | 第二个点 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `distance` | 距离 | Number | 两点距离 |

---

#### AngleMeasureNode - 角度测量节点

**节点ID**: `AngleMeasureNode`

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `line1` | 直线1 | Object | 是 | Line2D结构 |
| `line2` | 直线2 | Object | 是 | Line2D结构 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `angle` | 角度 | Number | 两直线夹角(弧度) |
| `angle_deg` | 角度(度) | Number | 两直线夹角(度) |

---

### OCR节点

#### OCRNode - OCR识别节点

**节点ID**: `OCRNode`

基于 Tesseract 的文字识别。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 待识别图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `text` | 识别文本 | String | 识别结果 |
| `confidence` | 置信度 | Number | 识别置信度(0-100) |
| `text_regions` | 文本区域 | Array | 文本区域列表 |

**参数**:
| 参数ID | 名称 | 类型 | 默认值 | 说明 |
|--------|------|------|--------|------|
| `language` | 语言 | String | "eng" | 语言代码 |
| `whitelist` | 白名单 | String | "" | 只识别这些字符 |
| `blacklist` | 黑名单 | String | "" | 不识别这些字符 |
| `model_path` | 模型路径 | String | "" | tessdata目录路径 |

**支持语言**:
| 代码 | 语言 |
|------|------|
| eng | 英语 |
| chi_sim | 简体中文 |
| chi_tra | 繁体中文 |
| jpn | 日语 |
| kor | 韩语 |

---

#### ChineseOCRNode - 中文OCR节点

**节点ID**: `ChineseOCRNode`

专门针对中文优化的OCR节点。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `image` | 输入图像 | Image | 是 | 待识别图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `text` | 识别文本 | String | 识别结果 |
| `confidence` | 置信度 | Number | 识别置信度 |
| `lines` | 行文本 | Array | 每行识别结果 |

---

### 半导体晶圆检测节点

#### WaferDieDetectionNode - 晶粒定位节点

**节点ID**: `WaferDieDetectionNode`

检测晶圆上的晶粒(Die)位置。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `wafer_image` | 晶圆图像 | Image | 是 | 晶圆图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `dies` | 晶粒列表 | Array | WaferDie数组 |
| `die_count` | 晶粒数 | Number | 检测到的晶粒数 |
| `good_count` | 良品数 | Number | 良品晶粒数 |
| `yield` | 良率 | Number | 良品率(%) |

---

#### WaferDefectClassificationNode - 缺陷分类节点

**节点ID**: `WaferDefectClassificationNode`

检测并分类晶圆缺陷。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `wafer_image` | 晶圆图像 | Image | 是 | 晶圆图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `defects` | 缺陷列表 | Array | WaferDefect数组 |
| `defect_count` | 缺陷数 | Number | 检测到的缺陷数 |

**缺陷类型**:
| 类型 | 说明 |
|------|------|
| scratch | 划痕 |
| particle | 颗粒 |
| contamination | 污染 |
| crack | 裂纹 |

---

#### WaferAlignmentNode - 晶圆对准节点

**节点ID**: `WaferAlignmentNode`

检测晶圆对准标记。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `wafer_image` | 晶圆图像 | Image | 是 | 晶圆图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `alignment_marks` | 对准标记 | Array | AlignmentMark数组 |
| `offset_x` | X偏移 | Number | X方向偏移(μm) |
| `offset_y` | Y偏移 | Number | Y方向偏移(μm) |
| `rotation` | 旋转角 | Number | 旋转角度(度) |

---

#### WaferDicingInspectionNode - 切割检测节点

**节点ID**: `WaferDicingInspectionNode`

检测切割道质量。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `dicing_streets` | 切割道 | Array | DicingStreet数组 |
| `has_defect` | 有缺陷 | Boolean | 切割道是否有缺陷 |

---

#### WaferSurfaceInspectionNode - 表面检测节点

**节点ID**: `WaferSurfaceInspectionNode`

检测晶圆表面质量。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `surface_quality` | 表面质量 | Object | SurfaceQuality结构 |
| `is_acceptable` | 是否合格 | Boolean | 表面是否合格 |

---

#### WaferThicknessMeasurementNode - 厚度测量节点

**节点ID**: `WaferThicknessMeasurementNode`

测量晶圆厚度参数。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `thickness_result` | 厚度结果 | Object | ThicknessResult结构 |

**厚度参数**:
| 参数 | 说明 |
|------|------|
| center_thickness | 中心厚度(μm) |
| edge_thickness | 边缘厚度(μm) |
| thickness_variation | 厚度变化(μm) |
| bow | 弯曲度(μm) |
| warp | 翘曲度(μm) |
| ttv | 总厚度变化(TTV) |

---

### 汽车零部件检测节点

#### BodyPanelInspectionNode - 钣金检测节点

**节点ID**: `BodyPanelInspectionNode`

检测钣金表面缺陷和变形。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `panel_image` | 钣金图像 | Image | 是 | 钣金表面图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `defects` | 缺陷列表 | Array | SurfaceDefect数组 |
| `defect_count` | 缺陷数 | Number | 检测到的缺陷数 |
| `quality_score` | 质量分数 | Number | 表面质量分数(0-1) |

**缺陷类型**:
| 类型 | 说明 |
|------|------|
| scratch | 刮痕 |
| dent | 凹陷 |
| pit | 麻点 |
| ripple | 波纹 |
| contamination | 污染 |

---

#### WeldSeamInspectionNode - 焊缝检测节点

**节点ID**: `WeldSeamInspectionNode`

检测焊缝质量，包括气孔、裂纹、咬边等。

**输入端口**:
| 端口ID | 名称 | 类型 | 必须 | 说明 |
|--------|------|------|------|------|
| `weld_image` | 焊缝图像 | Image | 是 | 焊缝图像 |

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `quality_result` | 质量结果 | Object | WeldQualityResult结构 |
| `defects` | 缺陷列表 | Array | WeldDefect数组 |

**焊缝缺陷类型**:
| 类型 | 说明 |
|------|------|
| porosity | 气孔 |
| crack | 裂纹 |
| undercut | 咬边 |
| overlap | 未熔合 |
| spatter | 飞溅 |

---

#### PaintQualityInspectionNode - 涂装检测节点

**节点ID**: `PaintQualityInspectionNode`

检测漆面缺陷，包括橘皮、流挂、色差等。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `paint_quality` | 漆面质量 | Object | PaintQualityResult结构 |
| `defects` | 缺陷列表 | Array | PaintDefect数组 |

**漆面缺陷类型**:
| 类型 | 说明 |
|------|------|
| orange_peel | 橘皮 |
| run | 流挂 |
| sag | 流痕 |
| scratch | 刮痕 |
| pinhole | 针孔 |
| dust | 灰尘 |
| color_defect | 色差 |

---

#### AssemblyVerificationNode - 装配验证节点

**节点ID**: `AssemblyVerificationNode`

验证零件装配完整性。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `assembly_result` | 装配结果 | Object | AssemblyResult结构 |
| `is_complete` | 是否完整 | Boolean | 装配是否完整 |

---

#### GearInspectionNode - 齿轮检测节点

**节点ID**: `GearInspectionNode`

检测齿轮齿形和磨损。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `gear_result` | 齿轮结果 | Object | GearInspectionResult结构 |
| `defects` | 缺陷列表 | Array | GearDefect数组 |

---

#### ConnectorInspectionNode - 连接器检测节点

**节点ID**: `ConnectorInspectionNode`

检测连接器插针位置。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `connector_result` | 连接器结果 | Object | ConnectorResult结构 |

---

#### BoltPresenceCheckNode - 螺栓检测节点

**节点ID**: `BoltPresenceCheckNode`

检测螺栓存在性。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `bolt_result` | 螺栓结果 | Object | BoltResult结构 |

---

#### GapMeasurementNode - 间隙测量节点

**节点ID**: `GapMeasurementNode`

测量面板间隙。

**输出端口**:
| 端口ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| `gap_result` | 间隙结果 | Object | GapResult结构 |

---

## 3. Python SDK API (ovf-sdk-python)

### ovf.Image 图像类

#### 类定义

```python
class ovf.Image:
    def __init__(self, width: int, height: int, channels: int = 1):
        """创建空白图像"""
        
    def __init__(self, data: bytes, width: int, height: int, channels: int = 1):
        """从数据创建图像"""
        
    @property
    def width(self) -> int:
        """图像宽度"""
        
    @property
    def height(self) -> int:
        """图像高度"""
        
    @property
    def channels(self) -> int:
        """通道数"""
        
    @property
    def data(self) -> bytes:
        """图像数据"""
        
    def copy(self) -> 'Image':
        """复制图像"""
        
    def save(self, filepath: str) -> bool:
        """保存图像到文件"""
        
    def load(filepath: str) -> 'Image':
        """从文件加载图像（静态方法）"""
```

#### 使用示例

```python
import ovf

# 创建空白图像
img = ovf.Image(640, 480, 1)

# 从文件加载
img = ovf.Image.load("test.png")

# 获取图像属性
print(f"Size: {img.width}x{img.height}, Channels: {img.channels}")

# 获取图像数据
data = img.data  # bytes类型
print(f"Data size: {len(data)}")

# 保存图像
img.save("output.png")
```

---

### ovf.Flow 流程类

#### 类定义

```python
class ovf.Flow:
    def __init__(self):
        """创建流程引擎"""
        
    def load(self, filepath: str) -> bool:
        """加载流程文件"""
        
    def load_from_json(self, json_str: str) -> bool:
        """从JSON字符串加载"""
        
    def save(self, filepath: str) -> bool:
        """保存流程文件"""
        
    def get_json(self) -> str:
        """获取流程JSON字符串"""
        
    def run(self) -> bool:
        """运行流程"""
        
    def run_node(self, node_id: str) -> bool:
        """运行指定节点"""
        
    def get_node_count(self) -> int:
        """获取节点数量"""
        
    def get_node_ids(self) -> List[str]:
        """获取所有节点ID"""
        
    def get_node_state(self, node_id: str) -> str:
        """获取节点状态"""
        
    def get_node_execute_time(self, node_id: str) -> int:
        """获取节点执行时间（微秒）"""
        
    def set_input_image(self, node_id: str, port_id: str, image: Image) -> bool:
        """设置输入图像"""
        
    def set_input_number(self, node_id: str, port_id: str, value: float) -> bool:
        """设置输入数值"""
        
    def set_input_string(self, node_id: str, port_id: str, value: str) -> bool:
        """设置输入字符串"""
        
    def get_output_image(self, node_id: str, port_id: str) -> Image:
        """获取输出图像"""
        
    def get_output_number(self, node_id: str, port_id: str) -> float:
        """获取输出数值"""
```

#### 使用示例

```python
import ovf

# 创建流程引擎
flow = ovf.Flow()

# 加载流程文件
if flow.load("flow.json"):
    print("Flow loaded successfully")
    
    # 获取节点数量
    print(f"Node count: {flow.get_node_count()}")
    
    # 获取所有节点ID
    node_ids = flow.get_node_ids()
    for id in node_ids:
        print(f"Node: {id}, State: {flow.get_node_state(id)}")
    
    # 设置输入图像
    input_img = ovf.Image.load("input.png")
    flow.set_input_image("camera_1", "output", input_img)
    
    # 运行流程
    if flow.run():
        print("Flow execution successful")
        
        # 获取输出图像
        output_img = flow.get_output_image("output_1", "image")
        output_img.save("result.png")
        
        # 获取执行时间
        for id in node_ids:
            time_us = flow.get_node_execute_time(id)
            print(f"{id}: {time_us} us")
    else:
        print("Flow execution failed")
```

---

### ovf.Node 节点类

#### 类定义

```python
class ovf.Node:
    def __init__(self, type_id: str, instance_id: str):
        """创建节点"""
        
    def set_param_int(self, key: str, value: int) -> bool:
        """设置整数参数"""
        
    def set_param_float(self, key: str, value: float) -> bool:
        """设置浮点参数"""
        
    def set_param_string(self, key: str, value: str) -> bool:
        """设置字符串参数"""
        
    def set_param_bool(self, key: str, value: bool) -> bool:
        """设置布尔参数"""
```

#### 使用示例

```python
import ovf

# 创建节点
node = ovf.Node("ThresholdNode", "threshold_1")

# 设置参数
node.set_param_int("threshold", 128)
node.set_param_string("method", "binary")

# 添加到流程
flow = ovf.Flow()
# 注意：需要先实现 add_node 方法
```

---

### ovf.Camera 相机类

#### 类定义

```python
class ovf.Camera:
    def __init__(self, driver_type: str, device_id: str):
        """打开相机"""
        
    def start_capture(self) -> bool:
        """开始采集"""
        
    def stop_capture(self) -> bool:
        """停止采集"""
        
    def capture_frame(self, timeout_ms: int = 1000) -> Image:
        """采集一帧"""
        
    def set_exposure(self, exposure_us: float) -> bool:
        """设置曝光时间"""
        
    def get_exposure(self) -> float:
        """获取曝光时间"""
        
    def set_gain(self, gain: float) -> bool:
        """设置增益"""
        
    def get_gain(self) -> float:
        """获取增益"""
        
    def send_soft_trigger(self) -> bool:
        """发送软触发"""
        
    def close(self):
        """关闭相机"""
        
    @staticmethod
    def enumerate(driver_type: str = "") -> List[Tuple[str, str]]:
        """枚举相机设备"""
```

#### 使用示例

```python
import ovf

# 枚举相机
cameras = ovf.Camera.enumerate()
for id, name in cameras:
    print(f"Camera: {id}, Name: {name}")

# 打开相机
camera = ovf.Camera("mock", "0")

# 设置参数
camera.set_exposure(10000)  # 10ms
camera.set_gain(1.5)

# 开始采集
camera.start_capture()

# 采集一帧
img = camera.capture_frame(1000)
if img:
    img.save("capture.png")

# 停止采集并关闭
camera.stop_capture()
camera.close()
```

---

### ovf.Runner 运行器类

#### 类定义

```python
class ovf.Runner:
    def __init__(self):
        """创建流程运行器"""
        
    def set_engine(self, engine: Flow) -> bool:
        """设置流程引擎"""
        
    def start_continuous(self) -> bool:
        """启动连续运行"""
        
    def start_triggered(self) -> bool:
        """启动触发模式"""
        
    def stop(self) -> bool:
        """停止运行"""
        
    def trigger(self) -> bool:
        """触发执行"""
        
    def is_running(self) -> bool:
        """检查是否运行"""
        
    def get_stats(self) -> Tuple[int, int, int]:
        """获取统计信息 (total_runs, success_runs, failed_runs)"""
```

#### 使用示例

```python
import ovf
import time

# 创建流程和运行器
flow = ovf.Flow()
flow.load("flow.json")

runner = ovf.Runner()
runner.set_engine(flow)

# 启动连续运行
runner.start_continuous()

# 运行一段时间
time.sleep(10)

# 停止运行
runner.stop()

# 获取统计信息
total, success, failed = runner.get_stats()
print(f"Total: {total}, Success: {success}, Failed: {failed}")

# 触发模式运行
runner.start_triggered()
for i in range(5):
    runner.trigger()
    time.sleep(1)
runner.stop()
```

---

### SDK函数

#### 版本和初始化

```python
# 初始化SDK
ovf.initialize()

# 关闭SDK
ovf.shutdown()

# 获取版本
major, minor, patch = ovf.get_version()
print(f"Version: {major}.{minor}.{patch}")
```

#### 节点类型查询

```python
# 获取注册的节点类型数量
count = ovf.get_registered_node_type_count()

# 获取注册的节点类型列表
types = ovf.get_registered_node_types()

# 获取节点类型信息
name, category, description = ovf.get_node_type_info("ThresholdNode")
```

---

## 4. Web API (ovf-web-server)

### REST API

#### 基础URL

```
http://localhost:8080/api
```

#### 获取节点列表

**请求**:
```
GET /api/nodes
```

**响应**:
```json
{
  "success": true,
  "nodes": [
    {
      "id": "ThresholdNode",
      "name": "阈值分割",
      "category": "ImageProcessing",
      "description": "图像阈值分割"
    }
  ]
}
```

---

#### 获取节点信息

**请求**:
```
GET /api/nodes/{type_id}
```

**响应**:
```json
{
  "success": true,
  "info": {
    "id": "ThresholdNode",
    "name": "阈值分割",
    "category": "ImageProcessing",
    "inputs": [
      {
        "id": "image",
        "name": "输入图像",
        "type": "Image",
        "required": true
      }
    ],
    "outputs": [
      {
        "id": "binary",
        "name": "二值图像",
        "type": "Image"
      }
    ],
    "params": [
      {
        "id": "threshold",
        "name": "阈值",
        "type": "Number",
        "default_value": 128
      }
    ]
  }
}
```

---

#### 加载流程

**请求**:
```
POST /api/flow/load
Content-Type: application/json

{
  "filepath": "flow.json"
}
```

或直接上传JSON内容:
```json
{
  "json_content": {
    "id": "flow1",
    "name": "测试流程",
    "nodes": [...]
  }
}
```

**响应**:
```json
{
  "success": true,
  "message": "Flow loaded successfully"
}
```

---

#### 保存流程

**请求**:
```
POST /api/flow/save
Content-Type: application/json

{
  "filepath": "output/flow.json"
}
```

**响应**:
```json
{
  "success": true,
  "message": "Flow saved successfully"
}
```

---

#### 运行流程

**请求**:
```
POST /api/flow/run
```

**响应**:
```json
{
  "success": true,
  "result": {
    "success": true,
    "total_time_us": 12345,
    "node_times": {
      "camera_1": 5000,
      "threshold_1": 2000,
      "output_1": 100
    }
  }
}
```

---

#### 单步执行

**请求**:
```
POST /api/flow/step
Content-Type: application/json

{
  "action": "begin" | "next" | "end"
}
```

**响应** (next动作):
```json
{
  "success": true,
  "node": {
    "id": "threshold_1",
    "type": "ThresholdNode",
    "state": "Running"
  }
}
```

---

#### 获取执行状态

**请求**:
```
GET /api/flow/status
```

**响应**:
```json
{
  "success": true,
  "status": {
    "is_running": true,
    "current_node": "threshold_1",
    "nodes_completed": 2,
    "total_nodes": 5,
    "execution_time_us": 5000
  }
}
```

---

#### 获取图像

**请求**:
```
GET /api/image/{node_id}/{port_id}
```

**响应**:
```json
{
  "success": true,
  "image": "base64_encoded_image_data",
  "width": 640,
  "height": 480,
  "channels": 1
}
```

---

#### 执行历史查询

**请求**:
```
GET /api/execution/history?limit=10
```

**响应**:
```json
{
  "success": true,
  "sessions": [
    {
      "id": "session_001",
      "timestamp": "2026-07-05T10:30:00",
      "success": true,
      "total_time_us": 12345,
      "node_count": 5
    }
  ]
}
```

---

#### 性能报告

**请求**:
```
GET /api/performance/report?session_id=session_001
```

**响应**:
```json
{
  "success": true,
  "report": {
    "total_time_ms": 12.345,
    "avg_node_time_ms": 2.469,
    "max_node_time_ms": 5.0,
    "min_node_time_ms": 0.1,
    "node_times": {
      "camera_1": 5.0,
      "threshold_1": 2.0,
      "output_1": 0.1
    }
  }
}
```

---

### WebSocket API

#### WebSocket URL

```
ws://localhost:8080/ws
```

#### 实时监控消息

**订阅节点状态**:
```json
{
  "action": "subscribe",
  "topic": "node_state"
}
```

**服务器推送**:
```json
{
  "topic": "node_state",
  "data": {
    "node_id": "threshold_1",
    "state": "Running",
    "timestamp": "2026-07-05T10:30:00"
  }
}
```

---

#### 流程执行进度

**订阅执行进度**:
```json
{
  "action": "subscribe",
  "topic": "execution_progress"
}
```

**服务器推送**:
```json
{
  "topic": "execution_progress",
  "data": {
    "current_node": "threshold_1",
    "nodes_completed": 2,
    "total_nodes": 5,
    "progress_percent": 40
  }
}
```

---

#### 实时图像输出

**订阅图像输出**:
```json
{
  "action": "subscribe",
  "topic": "image_output",
  "node_id": "output_1",
  "port_id": "image"
}
```

**服务器推送**:
```json
{
  "topic": "image_output",
  "data": {
    "node_id": "output_1",
    "port_id": "image",
    "image": "base64_encoded_data",
    "timestamp": "2026-07-05T10:30:01"
  }
}
```

---

### 使用示例（JavaScript）

```javascript
// REST API 调用
async function loadFlow() {
    const response = await fetch('/api/flow/load', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ filepath: 'flow.json' })
    });
    const result = await response.json();
    console.log(result);
}

// WebSocket 连接
const ws = new WebSocket('ws://localhost:8080/ws');

ws.onopen = () => {
    // 订阅节点状态
    ws.send(JSON.stringify({
        action: 'subscribe',
        topic: 'node_state'
    }));
};

ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    console.log('Received:', message.topic, message.data);
    
    if (message.topic === 'node_state') {
        updateNodeState(message.data.node_id, message.data.state);
    }
};
```

---

## 附录

### A. 数据类型映射表

| C++ 类型 | Python 类型 | JSON 类型 |
|----------|-------------|-----------|
| `int32_t` | `int` | `number` |
| `double` | `float` | `number` |
| `bool` | `bool` | `boolean` |
| `String` | `str` | `string` |
| `ImageData` | `ovf.Image` | object + base64 |
| `Vector<T>` | `List[T]` | `array` |
| `HashMap<K,V>` | `Dict[K,V]` | `object` |

---

### B. 图像格式说明

| 格式 | 说明 | 通道数 |
|------|------|--------|
| Mono8 | 8位灰度 | 1 |
| Mono16 | 16位灰度 | 1 |
| RGB8 | 8位RGB | 3 |
| BGR8 | 8位BGR | 3 |
| RGBA8 | 8位RGBA | 4 |
| Float32 | 32位浮点 | 1 |

---

### C. 节点状态说明

| 状态 | 值 | 说明 |
|------|-----|------|
| Idle | 0 | 空闲状态 |
| Running | 1 | 正在执行 |
| Success | 2 | 执行成功 |
| Failed | 3 | 执行失败 |
| Disabled | 4 | 已禁用 |

---

### D. 执行模式说明

| 模式 | 说明 |
|------|------|
| Sequential | 顺序执行（单线程） |
| Parallel | 并行执行（多线程，无依赖关系的节点同时执行） |
| DataDriven | 数据驱动（当输入数据就绪时自动触发执行） |

---

**文档版本**: 0.1.0  
**最后更新**: 2026-07-05  
**作者**: OpenVisionFlow Team