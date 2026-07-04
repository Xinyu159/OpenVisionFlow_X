# OpenVisionFlow 用户手册

**版本**: v1.0  
**更新日期**: 2026-07-05  
**适用平台**: Windows / Linux / 国产操作系统

---

## 目录

1. [快速入门](#1-快速入门)
   - 1.1 系统要求
   - 1.2 安装步骤
   - 1.3 快速开始
2. [Web编辑器使用](#2-web编辑器使用)
   - 2.1 登录和界面
   - 2.2 创建流程
   - 2.3 添加节点
   - 2.4 连接节点
   - 2.5 配置参数
   - 2.6 运行流程
   - 2.7 查看结果
3. [调试功能](#3-调试功能)
   - 3.1 设置断点
   - 3.2 单步调试
   - 3.3 变量监视
   - 3.4 执行追踪
4. [Python SDK使用](#4-python-sdk使用)
   - 4.1 安装SDK
   - 4.2 基本用法
   - 4.3 图像处理示例
   - 4.4 流程执行示例
5. [常见问题FAQ](#5-常见问题faq)
   - 5.1 安装问题
   - 5.2 运行问题
   - 5.3 性能优化

---

## 1. 快速入门

### 1.1 系统要求

OpenVisionFlow 支持以下操作系统和硬件环境：

**操作系统**：
- Windows 10/11（64位）
- Windows Server 2016及以上
- Linux（Ubuntu 20.04+, CentOS 7+, Debian 10+）
- 国产操作系统（如统信UOS、中标麒麟等）

**硬件要求**：
- CPU：Intel Core i5及以上或同等性能AMD处理器
- 内存：最低4GB，推荐8GB及以上
- 硬盘：至少500MB可用空间
- 显卡：无特殊要求（可选GPU加速功能需要NVIDIA显卡）

**软件依赖**：
- CMake >= 3.16
- C++17兼容编译器（MSVC 2019+, GCC 9+, Clang 10+）
- OpenCV >= 4.5（可选，用于高级图像处理）
- Python >= 3.8（使用Python SDK时）

> **[截图说明]** 系统要求检查界面，显示当前系统配置是否满足运行条件

### 1.2 安装步骤

#### Windows平台安装

**步骤1：获取安装包**

从GitHub下载最新版本：
```bash
git clone https://github.com/openvisionflow/OpenVisionFlow.git
cd OpenVisionFlow
```

或直接下载发布的ZIP压缩包。

**步骤2：编译构建**

使用CMake进行编译：

```bash
# 创建构建目录
mkdir build
cd build

# 配置项目（默认Release模式）
cmake ..

# 编译
cmake --build . --config Release

# 安装（可选）
cmake --install .
```

编译完成后，在`build/bin/Release`目录下会生成以下可执行文件：
- `ovf-web-server.exe` - Web编辑器服务器
- `test_all.exe` - 单元测试程序

**步骤3：验证安装**

运行测试程序验证安装是否成功：
```bash
cd build/bin/Release
./test_all.exe
```

> **[截图说明]** 编译进度窗口，显示编译百分比和成功状态

#### Linux平台安装

```bash
# 安装依赖
sudo apt-get update
sudo apt-get install -y cmake build-essential git

# 克隆项目
git clone https://github.com/openvisionflow/OpenVisionFlow.git
cd OpenVisionFlow

# 编译
mkdir build && cd build
cmake ..
make -j$(nproc)

# 安装
sudo make install
```

### 1.3 快速开始

#### 启动Web编辑器

启动Web编辑器服务器：

```bash
# Windows
cd build/bin/Release
ovf-web-server.exe

# Linux
./ovf-web-server
```

服务器默认在端口`8080`启动，打开浏览器访问：

```
http://localhost:8080
```

> **[截图说明]** Web编辑器启动界面，显示服务器运行状态和访问地址

#### 创建第一个流程

通过以下步骤创建一个简单的图像预处理流程：

1. 点击"新建流程"按钮
2. 添加"图像采集"节点
3. 添加"均值滤波"节点
4. 连接两个节点
5. 点击"运行"按钮执行流程

流程示例JSON（保存在`examples/flows/demo_preprocess.json`）：

```json
{
    "id": "demo_flow_001",
    "name": "图像预处理示例流程",
    "description": "演示图像采集、预处理、边缘检测和Blob分析",
    "nodes": [
        {
            "id": "node_image_source",
            "type_id": "ImageSource",
            "name": "图像采集",
            "params": {
                "width": 640,
                "height": 480,
                "channels": 1
            }
        },
        {
            "id": "node_blur",
            "type_id": "Blur",
            "name": "均值滤波",
            "params": {
                "kernel_size": 5,
                "sigma": 1.5
            }
        }
    ]
}
```

---

## 2. Web编辑器使用

### 2.1 登录和界面

#### 启动服务器

Web编辑器基于HTTP服务提供图形化界面，启动命令：

```bash
ovf-web-server [--port=端口号]
```

默认端口为8080，可通过参数指定其他端口：

```bash
ovf-web-server --port=9000
```

#### 界面布局

打开浏览器访问`http://localhost:8080`后，界面主要包含以下区域：

| 区域 | 功能说明 |
|------|---------|
| **左侧节点库面板** | 显示所有可用节点类型，按分类组织 |
| **中间流程画布** | 拖拽节点、连线、编辑流程图 |
| **右侧参数面板** | 显示选中节点的参数配置 |
| **底部状态栏** | 显示执行状态、日志信息 |

> **[截图说明]** Web编辑器主界面布局图，标注各功能区名称

### 2.2 创建流程

#### 新建流程

点击工具栏"新建流程"按钮或使用快捷键`Ctrl+N`，系统会创建一个空白流程。

流程元信息包括：
- 流程ID：唯一标识符
- 流程名称：显示名称
- 描述：功能说明
- 版本：流程版本号

#### 打开现有流程

点击"打开流程"按钮，选择JSON格式的流程文件（`.json`）加载。

API方式加载流程：
```http
POST /api/flow/load
Content-Type: application/json

{
    "filepath": "path/to/flow.json"
}
```

#### 保存流程

点击"保存流程"按钮，流程定义会保存为JSON文件，包含：
- 节点列表及参数
- 节点连接关系
- 元数据信息

### 2.3 添加节点

#### 从节点库拖拽

1. 在左侧节点库面板选择节点分类（如"图像处理"、"测量分析"）
2. 找到需要的节点类型
3. 拖拽节点到中间画布区域
4. 松开鼠标完成添加

#### 可用节点类型

系统提供以下主要节点分类：

| 分类 | 典型节点类型 |
|------|------------|
| **图像源** | ImageSource, ImageLoad, CameraCapture |
| **图像预处理** | Blur, Threshold, Morphology, ColorConvert |
| **特征检测** | EdgeDetection, BlobAnalysis, CornerDetect |
| **几何测量** | GeometryMeasure, CaliperTool, FitLine |
| **模板匹配** | TemplateMatch, ShapeMatch, NCCMatch |
| **标定** | Calibration, HandEyeCalibration |
| **通信输出** | TcpSend, SerialWrite, ImageSave |

> **[截图说明]** 节点库面板截图，展示各分类下的节点图标

### 2.4 连接节点

#### 创建数据连接

节点之间通过端口传递数据：

1. 点击源节点的输出端口（右侧圆点）
2. 拖拽连线到目标节点的输入端口（左侧圆点）
3. 连线成功后显示为绿色箭头

#### 端口类型

端口支持的数据类型：
- **Image** - 图像数据
- **Number** - 数值（整数或浮点数）
- **String** - 字符串
- **Array** - 数组数据
- **Object** - 对象/字典数据

#### 连接规则

- 输出端口类型必须与输入端口类型匹配
- 一个输入端口只能连接一个输出端口
- 一个输出端口可以连接多个输入端口
- 不能创建循环连接（系统自动检测拓扑合法性）

> **[截图说明]** 节点连接示意图，展示数据流向和端口类型匹配

### 2.5 配置参数

#### 参数面板

选中节点后，右侧参数面板显示该节点的可配置参数：

| 参数类型 | 配置方式 |
|---------|---------|
| 整数 | 数值输入框 |
| 浮点数 | 滑块+数值输入 |
| 字符串 | 文本输入框 |
| 布尔值 | 复选框 |
| 选项列表 | 下拉选择框 |
| 数组 | 多项编辑器 |

#### 参数示例

以"均值滤波"节点为例：

```json
{
    "kernel_size": 5,
    "sigma": 1.5
}
```

- `kernel_size`：卷积核大小（整数，范围1-31）
- `sigma`：标准差（浮点数，范围0.1-10.0）

#### 参数验证

系统在配置参数时会进行实时验证：
- 超出范围会显示警告
- 类型不匹配会提示错误
- 缺少必填参数会阻止运行

> **[截图说明]** 参数配置面板截图，展示各类型参数的编辑控件

### 2.6 运行流程

#### 执行控制

工具栏提供以下执行控制按钮：

| 按钮 | 功能 |
|------|------|
| ▶️ 运行 | 执行整个流程 |
| ⏸️ 暂停 | 暂停执行（用于调试） |
| ⏹️ 停止 | 终止执行 |
| 👉 单步 | 单步执行下一个节点 |

#### 执行模式

流程引擎支持三种执行模式：

1. **顺序执行（Sequential）**：按拓扑排序顺序依次执行
2. **并行执行（Parallel）**：无依赖关系的节点并行执行
3. **数据驱动（DataDriven）**：数据到达时触发执行

切换执行模式：

```http
POST /api/flow/run
Content-Type: application/json

{
    "mode": "parallel"
}
```

#### 执行状态监控

运行时底部状态栏显示：
- 当前执行节点
- 已完成节点数/总节点数
- 执行耗时
- 错误信息（如有）

> **[截图说明]** 流程执行状态截图，显示进度条和节点状态指示器

### 2.7 查看结果

#### 图像输出查看

执行完成后，点击节点查看输出图像：

1. 选中节点
2. 在参数面板切换到"输出"标签
3. 查看图像预览

图像可通过API导出：
```http
GET /api/image?node_id=node_blob_analysis&port=output
```

返回Base64编码的图像数据。

#### 数据输出查看

数值、字符串等数据的查看方式：
- 直接在输出标签页显示
- 支持复制到剪贴板
- 支持导出为JSON

#### 执行历史

点击"执行历史"按钮查看历史执行记录：
- 执行时间
- 执行结果（成功/失败）
- 各节点耗时
- 错误日志

> **[截图说明]** 结果查看面板截图，展示图像预览和数据输出区域

---

## 3. 调试功能

OpenVisionFlow 提供完善的调试工具，帮助用户诊断流程问题。

### 3.1 设置断点

#### 添加断点

在流程画布中设置断点：

1. 右键点击节点
2. 选择"添加断点"菜单项
3. 断点图标（红点）出现在节点上

或在调试面板直接添加：
```json
{
    "node_id": "node_threshold",
    "type": "normal",
    "condition": "",
    "hit_target": 0
}
```

#### 断点类型

系统支持四种断点类型：

| 类型 | 说明 |
|------|------|
| **普通断点（Normal）** | 每次执行到该节点时暂停 |
| **条件断点（Conditional）** | 满足条件表达式时暂停，如`area > 100` |
| **临时断点（Temporary）** | 命中一次后自动删除 |
| **命中次数断点（HitCount）** | 达到指定命中次数后暂停 |

#### 断点管理

断点管理器提供以下操作：
- 启用/禁用断点
- 删除断点
- 清空所有断点
- 查看断点列表

> **[截图说明]** 断点设置界面截图，展示断点类型选择和条件编辑框

### 3.2 单步调试

#### 单步执行控制

调试工具栏提供以下控制按钮：

| 按钮 | 功能 | 说明 |
|------|------|------|
| 👉 单步跳过（Step Over） | 执行当前节点，暂停于下一节点 | 不进入子流程 |
| 👇 单步进入（Step Into） | 执行并进入节点内部 | 用于复杂节点调试 |
| 👆 单步退出（Step Out） | 执行完当前子流程，返回上层 | 退出嵌套调用 |
| ▶️ 继续（Continue） | 继续执行直到下一个断点 | 恢复正常执行 |

#### 调试流程

典型调试流程：

1. 设置断点在可疑节点
2. 点击"运行"开始执行
3. 执行到断点自动暂停
4. 查看当前变量状态
5. 使用单步执行逐节点检查
6. 发现问题后修改参数
7. 重新运行验证

#### 调试状态

调试器状态指示：

| 状态 | 含义 |
|------|------|
| Idle | 空闲，未启动 |
| Running | 正在执行 |
| Paused | 已暂停（断点或手动暂停） |
| Stepping | 单步执行中 |
| Stopped | 已停止 |

> **[截图说明]** 单步调试状态截图，显示当前暂停节点和调试控制按钮

### 3.3 变量监视

#### 添加监视变量

在变量监视面板添加需要跟踪的变量：

1. 点击"添加监视"按钮
2. 输入变量名称（如`threshold_value`, `blob_count`）
3. 变量值实时显示

监视变量存储在`VariableWatcher`中，支持：
- 添加/移除监视项
- 清空监视列表
- 实时值更新

#### 变量类型

监视面板显示变量详细信息：

```json
{
    "name": "area",
    "display_value": "1250.5",
    "type": "Number",
    "is_valid": true,
    "last_update": 1656954123456
}
```

- 数值类型显示具体值
- 图像类型显示尺寸信息
- 数组类型显示元素数量
- 对象类型可展开查看子属性

#### 变量更新

变量值在以下时机自动更新：
- 节点执行完成后
- 断点命中时
- 单步执行暂停时

> **[截图说明]** 变量监视面板截图，展示变量列表和实时值显示

### 3.4 执行追踪

#### 执行日志

执行追踪器记录完整执行过程：

每步执行记录包含：
```json
{
    "node_id": "node_blur",
    "node_type": "Blur",
    "input_data": "{...}",
    "output_data": "{...}",
    "start_time": 1656954123000,
    "end_time": 1656954123050,
    "duration_us": 50000,
    "success": true,
    "error_message": ""
}
```

#### 追踪导出

执行追踪可导出为JSON格式：
```http
GET /api/execution/history
```

返回完整的执行历史记录，包含：
- 所有执行步骤
- 每步耗时统计
- 成功/失败统计

#### 性能分析

基于执行追踪进行性能分析：

1. 查看各节点耗时，找出慢节点
2. 分析并行执行效率
3. 定位瓶颈节点进行优化

执行监控API：
```http
GET /api/monitor/realtime
GET /api/performance/report
```

> **[截图说明]** 执行追踪面板截图，展示执行时间线和节点耗时图表

---

## 4. Python SDK使用

### 4.1 安装SDK

#### Windows安装

Python SDK以`.pyd`扩展模块形式提供，位于：
```
build/bin/Release/ovf.pyd
```

将`ovf.pyd`复制到Python模块目录或项目目录：

```bash
# 复制到Python site-packages
copy build\bin\Release\ovf.pyd C:\Python3x\Lib\site-packages\

# 或直接在项目中使用
copy build\bin\Release\ovf.pyd your_project\
```

#### Linux安装

Linux平台生成`.so`共享库：
```
build/bin/Release/ovf.so
```

安装方式：
```bash
sudo cp build/bin/Release/ovf.so /usr/local/lib/python3.x/site-packages/
```

#### 验证安装

在Python中导入验证：

```python
import ovf

# 获取版本
major, minor, patch = ovf.get_version()
print(f"OpenVisionFlow SDK版本: {major}.{minor}.{patch}")

# 初始化SDK
ovf.initialize()
```

### 4.2 基本用法

#### SDK初始化

使用SDK前必须初始化：

```python
import ovf

# 初始化SDK
ovf.initialize()

# 创建流程引擎
engine = ovf.FlowEngine()

# 使用完毕后关闭
ovf.shutdown()
```

#### 查看注册节点类型

获取所有可用节点类型：

```python
# 获取节点类型列表
node_types = ovf.get_registered_node_types()
print(f"可用节点类型: {len(node_types)}个")

for type_id in node_types:
    info = ovf.get_node_type_info(type_id)
    print(f"  {type_id}: {info['name']} - {info['description']}")
```

#### 图像对象

创建和操作图像：

```python
# 创建空白图像
img = ovf.Image(width=640, height=480, channels=1)

# 获取图像属性
print(f"图像尺寸: {img.width} x {img.height}")
print(f"通道数: {img.channels}")

# 设置图像数据
data = bytes([128] * (640 * 480))
img.set_data(data)

# 获取图像数据
img_bytes = img.tobytes()
img_array = img.toarray()  # bytearray类型，可修改

# 从bytes创建图像
img2 = ovf.Image.from_bytes(data, 640, 480, 1)
```

### 4.3 图像处理示例

#### 加载流程处理图像

```python
import ovf

# 初始化
ovf.initialize()

# 创建引擎并加载流程
engine = ovf.FlowEngine()
engine.load_flow("examples/flows/demo_preprocess.json")

# 设置输入图像
input_img = ovf.Image(width=640, height=480, channels=1)
# 填充测试数据...
engine.set_input_image("node_image_source", "input", input_img)

# 运行流程
engine.run()

# 获取输出结果
output_img = engine.get_output_image("node_blob_analysis", "output")
print(f"输出图像尺寸: {output_img.width} x {output_img.height}")

# 关闭
ovf.shutdown()
```

#### 单节点执行

```python
import ovf

ovf.initialize()

engine = ovf.FlowEngine()
engine.load_flow("flow.json")

# 只执行特定节点
engine.run_node("node_blur")

# 获取节点执行时间
time_us = engine.get_node_execute_time("node_blur")
print(f"节点执行耗时: {time_us}微秒")

ovf.shutdown()
```

### 4.4 流程执行示例

#### 连续运行模式

使用`FlowRunner`进行连续执行：

```python
import ovf
import time

ovf.initialize()

# 创建引擎和运行器
engine = ovf.FlowEngine()
engine.load_flow("inspection_flow.json")

runner = ovf.FlowRunner()
runner.set_engine(engine)

# 启动连续运行
runner.start_continuous()

# 运行一段时间
time.sleep(10)

# 停止
runner.stop()

# 获取统计
total, success, failed = runner.get_stats()
print(f"总执行次数: {total}")
print(f"成功次数: {success}")
print(f"失败次数: {failed}")

ovf.shutdown()
```

#### 触发执行模式

外部触发执行流程：

```python
import ovf

ovf.initialize()

engine = ovf.FlowEngine()
engine.load_flow("triggered_flow.json")

runner = ovf.FlowRunner()
runner.set_engine(engine)

# 启动触发模式
runner.start_triggered()

# 外部触发执行
for i in range(5):
    # 等待外部信号（如相机触发）
    # ...
    runner.trigger()  # 触发一次执行

runner.stop()
ovf.shutdown()
```

#### 相机采集与处理

```python
import ovf

ovf.initialize()

# 列举相机
cameras = ovf.enumerate_cameras()
print(f"发现相机: {len(cameras)}")
for cam in cameras:
    print(f"  {cam['id']}: {cam['name']}")

# 打开相机
camera = ovf.Camera(driver_type="mock", device_id="default")

# 设置参数
camera.set_exposure(10000)  # 10毫秒
camera.set_gain(2.0)

# 开始采集
camera.start_capture()

# 创建处理引擎
engine = ovf.FlowEngine()
engine.load_flow("process_flow.json")

# 采集并处理
for frame_id in range(100):
    # 采集一帧
    img = camera.capture_frame(timeout_ms=1000)
    
    # 设置输入并运行
    engine.set_input_image("node_source", "image", img)
    engine.run()
    
    # 获取结果
    result = engine.get_output_number("node_result", "score")

# 停止采集
camera.stop_capture()

ovf.shutdown()
```

> **[截图说明]** Python SDK使用示例代码运行截图，展示输出结果

---

## 5. 常见问题FAQ

### 5.1 安装问题

#### Q1: 编译失败，提示找不到CMake

**原因**：系统未安装CMake或版本过低。

**解决方案**：
1. 安装CMake 3.16或更高版本
2. Windows：从 https://cmake.org/download/ 下载安装
3. Linux：`sudo apt-get install cmake`

#### Q2: 编译错误"C++17标准不支持"

**原因**：编译器版本过低。

**解决方案**：
- Windows：使用Visual Studio 2019或更高版本
- Linux GCC：确保GCC版本 >= 9.0
- 添加编译选项：`cmake -DCMAKE_CXX_STANDARD=17 ..`

#### Q3: 运行时找不到DLL/SO文件

**原因**：动态库路径未配置。

**解决方案**：
- Windows：将`build/bin/Release`目录添加到PATH环境变量
- Linux：设置`LD_LIBRARY_PATH`环境变量
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./build/bin/Release
```

#### Q4: Python导入ovf模块失败

**原因**：`.pyd`或`.so`文件未正确安装。

**解决方案**：
1. 检查文件是否在Python搜索路径中
2. 确认Python版本匹配（SDK编译时的Python版本）
3. 尝试直接指定路径：
```python
import sys
sys.path.append("path/to/ovf.pyd")
import ovf
```

### 5.2 运行问题

#### Q5: 流程运行报错"节点未找到"

**原因**：流程定义中的节点类型未注册。

**解决方案**：
1. 检查`type_id`是否正确
2. 确认相关算法插件已加载
3. 使用`ovf.get_registered_node_types()`查看可用类型

#### Q6: 节点连接报错"端口类型不匹配"

**原因**：尝试连接不同数据类型的端口。

**解决方案**：
1. 检查源节点输出端口类型
2. 检查目标节点输入端口类型
3. 确保类型一致（如Image→Image，Number→Number）
4. 必要时添加类型转换节点

#### Q7: Web编辑器无法访问

**原因**：服务器未启动或端口被占用。

**解决方案**：
1. 确认`ovf-web-server`已启动
2. 检查端口是否被占用：
```bash
# Windows
netstat -ano | findstr :8080

# Linux
netstat -tulpn | grep 8080
```
3. 使用其他端口启动：`ovf-web-server --port=9000`

#### Q8: 图像处理节点执行失败

**原因**：输入图像格式不正确或参数超限。

**解决方案**：
1. 检查输入图像尺寸和通道数
2. 确认参数在有效范围内
3. 查看节点错误信息：`engine.get_node_state("node_id")`
4. 检查日志输出

### 5.3 性能优化

#### Q9: 流程执行速度慢

**解决方案**：

1. **使用并行执行模式**：
```python
# Web API
POST /api/flow/run {"mode": "parallel"}

# Python SDK
engine.set_execution_mode(ovf.ExecutionMode.Parallel)
```

2. **减少图像尺寸**：适当降低图像分辨率

3. **优化节点参数**：
   - 降低滤波核大小
   - 减少迭代次数
   - 使用更快的算法选项

4. **避免重复计算**：优化节点连接拓扑

#### Q10: 内存占用过高

**解决方案**：

1. 及时释放图像对象：
```python
img = engine.get_output_image(...)
# 使用完及时释放（Python自动GC）
```

2. 减少中间图像缓存：
   - 断开不必要的中间输出连接
   - 使用单步执行减少内存峰值

3. 控制图像尺寸：避免处理超大图像

#### Q11: 多线程环境使用注意事项

**解决方案**：

1. FlowEngine和FlowRunner实例不能跨线程共享
2. 每个线程创建独立的引擎实例
3. 使用线程安全的队列传递图像数据
4. 避免在多线程中直接访问节点输出

#### Q12: 如何提高相机采集效率

**解决方案**：

1. 使用触发模式而非连续采集
2. 设置合理的缓冲区大小
3. 调整曝光和增益参数
4. 使用硬件触发代替软触发

---

## 附录

### A. 节点类型快速参考

| 分类 | 节点类型ID | 说明 |
|------|-----------|------|
| 图像源 | ImageSource | 创建空白图像 |
| 图像源 | ImageLoad | 从文件加载图像 |
| 图像源 | CameraCapture | 相机采集 |
| 预处理 | Blur | 均值/高斯滤波 |
| 预处理 | Threshold | 阈值分割 |
| 预处理 | Morphology | 形态学处理 |
| 预处理 | ColorConvert | 颜色空间转换 |
| 特征检测 | EdgeDetection | 边缘检测 |
| 特征检测 | BlobAnalysis | Blob分析 |
| 测量 | GeometryMeasure | 几何测量 |
| 匹配 | TemplateMatch | 模板匹配 |
| 标定 | Calibration | 标定 |
| 输出 | ImageSave | 保存图像 |
| 输出 | TcpSend | TCP发送 |
| 通信 | SerialWrite | 串口输出 |

### B. API接口参考

Web编辑器主要API接口：

| 接口 | 方法 | 说明 |
|------|------|------|
| `/api/nodes` | GET | 获取节点类型列表 |
| `/api/node/info` | GET | 获取节点类型信息 |
| `/api/flow/load` | POST | 加载流程文件 |
| `/api/flow/save` | POST | 保存流程文件 |
| `/api/flow/run` | POST | 运行流程 |
| `/api/flow/step` | POST | 单步执行 |
| `/api/flow/status` | GET | 获取执行状态 |
| `/api/image` | GET | 获取图像输出 |
| `/api/execution/history` | GET | 执行历史记录 |
| `/api/performance/report` | GET | 性能分析报告 |

### C. 错误码参考

Python SDK错误码定义：

| 错误码 | 名称 | 说明 |
|--------|------|------|
| 0 | SUCCESS | 成功 |
| 1 | ERROR_UNKNOWN | 未知错误 |
| 2 | ERROR_INVALID_PARAM | 参数无效 |
| 3 | ERROR_NULL_POINTER | 空指针 |
| 4 | ERROR_OUT_OF_RANGE | 超出范围 |
| 5 | ERROR_NOT_SUPPORTED | 不支持 |
| 6 | ERROR_TIMEOUT | 超时 |
| 100 | ERROR_FLOW_NOT_FOUND | 流程未找到 |
| 101 | ERROR_NODE_NOT_FOUND | 节点未找到 |
| 102 | ERROR_INVALID_FLOW | 流程无效 |
| 105 | ERROR_EXECUTION_FAILED | 执行失败 |
| 200 | ERROR_DEVICE_NOT_FOUND | 设备未找到 |
| 204 | ERROR_CAMERA_CAPTURE_FAILED | 相机采集失败 |

---

**文档结束**

如需更多帮助，请访问：
- GitHub: https://github.com/openvisionflow/OpenVisionFlow
- Issues: https://github.com/openvisionflow/OpenVisionFlow/issues
- 项目Wiki: https://github.com/openvisionflow/OpenVisionFlow/wiki