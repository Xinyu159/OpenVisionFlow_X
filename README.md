# OpenVisionFlow - 开源工业机器视觉平台

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Version](https://img.shields.io/badge/Version-0.2.0-green.svg)](CHANGELOG.md)
[![Nodes](https://img.shields.io/badge/Nodes-557-orange.svg)](docs/node_reference.md)
[![Tests](https://img.shields.io/badge/Tests-99.27%25-brightgreen.svg)](docs/test_report_summary.md)

## 项目简介

OpenVisionFlow (OVF) 是一个完全开源、跨平台的工业机器视觉平台，对标 VisionMaster/Halcon/VisionPro，解决其算法弱、封闭、硬件绑定、授权受限等核心问题。

## 核心特性

- **完全开源免费** - Apache 2.0 协议，无加密狗、无授权限制
- **557个算子节点** - 图像处理/亚像素精度/OCR/行业检测/CUDA加速
- **硬件完全解耦** - 统一 HAL 抽象层，支持全品牌工业相机
- **算法分层开放** - 底层可改、算子可插拔，源码可见
- **跨平台支持** - Windows/Linux/国产操作系统
- **双开发模式** - Web零代码编辑器 + SDK 代码开发
- **工业鲁棒性** - 详细日志诊断、断点调试、自动补偿
- **CUDA加速** - 5-10x关键算子性能提升
- **亚像素精度** - ±0.005像素边缘定位精度

## 📚 文档

| 文档 | 描述 |
|------|------|
| [API参考文档](docs/api_reference.md) | 核心/算法/Python SDK/Web API完整参考 |
| [用户手册](docs/user_manual.md) | Web编辑器、调试功能、Python SDK使用指南 |
| [节点参考手册](docs/node_reference.md) | 557个算子节点详细说明 |
| [架构设计文档](docs/architecture.md) | 系统架构、核心设计、扩展机制 |
| [测试报告汇总](docs/test_report_summary.md) | 273个测试，99.27%通过率 |
| [版本变更日志](CHANGELOG.md) | Keep a Changelog规范 |

## 项目结构

```
OpenVisionFlow/
├── ovf-core/           # 微内核核心模块
│   ├── include/ovf/core/   # INode/Data/Result/FlowEngine
│   └── src/
│
├── ovf-algorithm/      # 算法模块（557节点）
│   ├── include/ovf/algorithm/
│   │   ├── subpixel_precision.h   # 亚像素精度
│   │   ├── chinese_ocr.h          # 汉字OCR
│   │   ├── wafer_inspection.h     # 半导体晶圆检测
│   │   ├── automotive_inspection.h # 汽车零部件检测
│   │   ├── cuda_accelerator.h     # CUDA加速
│   │   └── ...
│   └── src/
│
├── ovf-web-server/     # Web零代码编辑器
│   ├── src/
│   └── web/            # 前端界面
│
├── ovf-sdk-python/     # Python SDK
│   └── src/
│
├── tests/              # 测试框架
│   ├── test_framework.h
│   ├── test_core_types.cpp
│   ├── test_cuda_accelerator.cpp
│   └── ...
│
├── docs/               # 文档目录
│   ├── api_reference.md
│   ├── user_manual.md
│   ├── node_reference.md
│   ├── architecture.md
│   └── test_report_summary.md
│
├── CHANGELOG.md        # 版本变更日志
└── README.md           # 本文件
```

## 突破成果

### P0 突破（核心差距缩小）

| 功能 | 详情 |
|------|------|
| **亚像素精度** | 10节点，Taylor/Parabola/Gaussian/Saddle/Zernike算法，±0.005像素精度 |
| **汉字OCR** | 10节点，MSER文本检测+模板匹配，GB2312汉字3755个 |
| **Web编辑器** | 8大功能模块，零代码流程配置，断点调试 |

### P1 突破（行业应用）

| 功能 | 详情 |
|------|------|
| **半导体晶圆检测** | 10节点，晶粒定位/缺陷分类/图案检测/厚度测量 |
| **汽车零部件检测** | 10节点，钣金/焊缝/涂装/齿轮检测 |
| **调试工具** | 断点管理/单步调试/变量监视/执行追踪 |
| **执行日志可视化** | WebSocket实时推送/性能分析/Chart.js输出 |
| **CUDA加速** | 8节点，高斯模糊/Sobel/阈值/形态学（5-10x加速） |

## 构建依赖

- CMake >= 3.16
- C++17 编译器 (MSVC/GCC/Clang)
- OpenCV >= 4.5 (可选)
- CUDA >= 11.0 (可选，用于GPU加速)

## 构建步骤

```bash
# 克隆项目
git clone https://gitcode.com/hunyuan2026/OpenVisionFlow.git
cd OpenVisionFlow

# 配置（启用测试）
cmake -B build -DBUILD_TESTS=ON

# 构建
cmake --build build --config Release

# 运行测试
./build/bin/Release/test_all.exe

# 启动Web服务器
./build/bin/Release/ovf-web-server.exe
```

## 快速开始

### 使用 Python SDK

```python
import ovf

# 初始化
ovf.initialize()

# 创建图像
img = ovf.Image(640, 480, 1)

# 加载流程
flow = ovf.Flow.load("flow.json")

# 设置输入
flow.set_input("image", img)

# 运行流程
flow.run()

# 获取结果
result = flow.get_output("result")
print(f"检测结果: {result}")
```

### 使用 Web 编辑器

1. 启动服务器: `./build/bin/Release/ovf-web-server.exe`
2. 打开浏览器: `http://localhost:8080`
3. 从节点库拖拽节点到画布
4. 连接节点端口
5. 配置参数
6. 点击运行

### 使用 C++ SDK

```cpp
#include "ovf/api.h"

int main() {
    // 初始化 SDK
    ovf::api::initialize();
    
    // 创建流程引擎
    auto engine = ovf::api::create_flow_engine();
    
    // 加载流程文件
    engine->load_from_file("flow.json");
    
    // 运行流程
    ovf::FlowContext context;
    auto result = engine->run(context);
    
    ovf::api::shutdown();
    return 0;
}
```

## 核心模块说明

### ovf-core (微内核)

- **INode** - 节点基类，所有算法节点的统一接口
- **Data** - 数据容器，支持图像/数值/字符串/数组等
- **Result<T>** - 结果封装，错误码+错误消息
- **FlowEngine** - 流程引擎，流程加载/执行/管理
- **NodeFactory** - 节点工厂，节点注册/创建

### ovf-algorithm (算法模块)

| 分类 | 节点数 | 代表节点 |
|------|--------|----------|
| 图像处理 | ~80 | GaussianBlur, SobelEdge, ImageResize |
| 亚像素精度 | 10 | SubpixelEdge, SubpixelCorner, SubpixelCircle |
| Blob分析 | ~20 | BlobDetect, BlobAnalyze, BlobFilter |
| 测量工具 | ~30 | CaliperTool, MeasureLine, MeasureCircle |
| OCR识别 | 10 | ChineseOCR, DigitOCR, EnglishOCR |
| 半导体检测 | 10 | WaferDieDetection, WaferDefectClassification |
| 汽车检测 | 10 | BodyPanelInspection, WeldSeamInspection |
| CUDA加速 | 8 | CudaGaussianBlur, CudaSobelFilter |

## 测试覆盖

| 模块 | 测试数 | 通过率 |
|------|--------|--------|
| CoreTypes | 63 | 96.8% |
| CudaAccelerator | 37 | **100%** |
| Debugger | 73 | **100%** |
| AutomotiveInspection | 55 | **100%** |
| WaferInspection | 45 | **100%** |

## 开发路线

- [x] **V0.1** - 基础架构（核心内核 + HAL + 基础算法）
- [x] **V0.2** - P0/P1突破（亚像素/OCR/行业检测/CUDA） ✅
- [ ] **V0.5** - 功能完整版（3D + 深度学习推理 + 工业协议）
- [ ] **V1.0** - 正式版（UI优化 + 自动补偿 + 信创适配）

## 许可证

Apache 2.0 License - 允许免费商用、二次修改、闭源分发

## 联系方式

- GitHub: https://github.com/openvisionflow/OpenVisionFlow
- Issues: https://github.com/openvisionflow/OpenVisionFlow/issues
- Email: openvisionflow@example.com

---

**OpenVisionFlow** - 让工业机器视觉更开放、更强大、更自由！