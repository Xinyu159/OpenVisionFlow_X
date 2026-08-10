# OpenVisionFlow 依赖说明

## 核心依赖（必需）

### C++ 编译环境
- **CMake** >= 3.16
- **C++17** 兼容编译器
  - Windows: MSVC 2019+
  - Linux: GCC 9+ / Clang 10+

### 无外部库依赖
OpenVisionFlow 的核心算法模块采用**纯C++实现**，不依赖任何外部图像处理库：
- ✓ 所有图像处理算法均为自研实现
- ✓ 支持基础图像处理、亚像素精度、OCR等功能
- ✓ 轻量级设计，降低部署复杂度

---

## 可选依赖

### 1. CUDA 加速（可选）

#### 版本要求
- **CUDA Toolkit** >= 11.0（推荐）
- 支持最低版本：CUDA 10.0

#### 支持的GPU架构
- Pascal (sm_60) 及以上
- 推荐：Volta (sm_70), Turing (sm_75), Ampere (sm_80)

#### 功能说明
启用CUDA后，以下算子可获得5-10倍性能提升：
- 高斯模糊
- Sobel边缘检测
- 阈值处理
- 形态学运算
- 特征检测

#### 构建选项
```bash
# 启用CUDA（默认）
cmake -B build -DENABLE_CUDA=ON

# 禁用CUDA
cmake -B build -DENABLE_CUDA=OFF
```

#### 验证CUDA支持
```bash
# 构建后检查
./build/bin/Release/test_cuda_accelerator.exe
```

---

### 2. OpenCV（可选）

#### 当前状态
**不依赖OpenCV** - 所有算法均为纯C++实现

#### 未来计划
- V1.0版本将支持OpenCV作为可选后端
- 提供OpenCV加速的算子选项
- 支持OpenCV图像格式互操作

#### 为什么要可选支持OpenCV？
1. **性能优势**: 部分算子OpenCV实现更成熟
2. **兼容性**: 方便从OpenCV项目迁移
3. **生态集成**: 利用OpenCV丰富的工具链

---

### 3. PCL 点云库（风格兼容）

#### 实现说明
- 项目实现了**PCL风格**的点云处理算法
- **无需安装PCL库** - 所有算法为纯C++实现
- API设计参考PCL，但完全独立实现

#### 功能覆盖
- 点云滤波（VoxelGrid、StatisticalOutlierRemoval）
- 点云配准（ICP、NDT）
- 点云分割（欧式聚类、区域生长）
- 点云重建（泊松重建、贪婪三角化）

---

## 第三方库（已集成）

项目已集成以下第三方库，无需单独安装：

### JSON处理
- **nlohmann/json** - 单头文件JSON库
- 位置: `ovf-core/thirdparty/nlohmann/json.hpp`

### 构建系统
- **CMake** - 跨平台构建工具

---

## 依赖决策指南

### 最小依赖方案（推荐）
```bash
# 只需要CMake和C++17编译器
cmake -B build -DENABLE_CUDA=OFF
cmake --build build --config Release
```

**优点**：
- 部署简单
- 依赖最少
- 跨平台兼容性好

**适用场景**：
- CPU性能足够
- 快速原型开发
- 嵌入式部署

---

### GPU加速方案
```bash
# 需要安装CUDA Toolkit 11.0+
cmake -B build -DENABLE_CUDA=ON
cmake --build build --config Release
```

**优点**：
- 关键算子5-10倍加速
- 支持实时处理
- 大批量数据处理

**适用场景**：
- 实时视觉检测
- 高吞吐量生产环境
- GPU服务器部署

---

## 常见问题

### Q1: 必须安装OpenCV吗？
**A**: 不需要。OpenVisionFlow所有算法均为纯C++实现，无需OpenCV。

### Q2: 没有GPU能用吗？
**A**: 完全可以。CPU实现已经过优化，满足大多数应用场景。GPU加速为可选功能。

### Q3: CUDA版本要求严格吗？
**A**: 代码支持CUDA 10.0+，但推荐使用CUDA 11.0+以获得最佳兼容性。

### Q4: 如何在无GPU环境构建？
**A**: 使用 `-DENABLE_CUDA=OFF` 选项即可完全禁用CUDA相关代码。

### Q5: 支持哪些操作系统？
**A**:
- Windows 10/11
- Linux (Ubuntu 20.04+, CentOS 8+)
- 国产操作系统（适配中）

---

## 版本兼容性矩阵

| 组件 | 最低版本 | 推荐版本 | 状态 |
|------|----------|----------|------|
| CMake | 3.16 | 3.20+ | 必需 |
| C++编译器 | C++17 | C++20 | 必需 |
| CUDA | 10.0 | 11.0+ | 可选 |
| OpenCV | - | - | 未来支持 |
| PCL | - | - | 风格兼容（无需安装） |

---

## 技术支持

如有依赖相关问题，请查看：
- [构建指南](../README.md#构建步骤)
- [常见问题](../docs/user_manual.md#常见问题)
- [GitHub Issues](https://gitcode.com/hunyuan2026/OpenVisionFlow/issues)