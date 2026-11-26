# OpenVisionFlow_X — Windows 优先的自建算子平台

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux-lightgrey.svg)]()
[![Tests](https://img.shields.io/badge/Tests-5%2F5%20passed-brightgreen.svg)]()

## 这个仓库现在是什么

以开源工业机器视觉平台 **[OpenVisionFlow](https://gitcode.com/hunyuan2026/OpenVisionFlow)（Apache-2.0，上游 15.4 万行 / 502 算子）** 的引擎为核心，**转向 Windows 产线部署环境**的自建算子平台。

2026-11-26 `windows-rebuild` 重构后的形态：

```
├── ovf-core/          引擎（继承上游 + 本人加固：注册防重名/流程引擎防崩/安全带 API）
├── ovf-nodes/         ★ 自建算子模块（唯一算子来源，从 Threshold 起逐个填充）
├── ovf-sdk/           C++ SDK
├── ovf-web-server/    REST 服务 + Web 编辑器后端
└── tests/             引擎/算子测试（ctest）
```

**上游 502 个算子（ovf-algorithm）、Qt 编辑器、相机插件、Python SDK、算子体检工具已整体移除**——它们仍完整保留在 git 历史（`windows-rebuild` 分支之前的 `main`）与上游仓库中。移除不是否定，而是把平台变成"引擎 + 我自己写的算子"：每个算子都经过自己理解和验证，而不是背一座不认识的山。

> 上游衍生关系的完整记录（Linux 移植、注册加固、防崩、501 算子体检流水线等 ~9300 行工作）见 git 历史与 `docs/hardening/`。原项目版权与许可见 [LICENSE](LICENSE)。

## Windows 构建状态（2026-11-26，实测通过）

工具链：**MinGW-w64 (w64devkit, gcc 14.2)** + CMake ≥ 3.24。

```bash
cmake -G "MinGW Makefiles" -B build -S .
cmake --build build -j8
cd build && ctest --output-on-failure   # 5/5 passed
./bin/Release/ovf-web-server.exe        # /api/nodes 已能返回 nodes.Threshold
```

Windows 适配过程中修掉的真实问题（都会在面试里被问，值得记住）：

| 问题 | 根因 | 修法 |
|---|---|---|
| `remove(iterator)` 编译错误 | Linux 上 `<algorithm>` 被传递包含，MinGW 没有；未限定名落到 C 的 `remove(const char*)` | 显式 `#include <algorithm>` |
| 链接期 `undefined reference to vtable for ovf::INode` | `WINDOWS_EXPORT_ALL_SYMBOLS` 对 MinGW 无法导出虚函数 vtable | ovf-core 在 WIN32 下改静态库 |
| `UserNodesTest` 段错误 | PE/COFF 没有 ELF 弱函数"解析不到即 nullptr"语义，MinGW 误入 `__GNUC__` 弱引用分支 | 弱引用分支排除 `_WIN32` |
| 算子注册表为空（零警告） | 静态库只抽取被引用的目标文件，`OVF_REGISTER_NODE` 注册对象无人引用——Windows 静态库版的 `--as-needed` 问题 | `ovf-nodes-link` 用 `$<LINK_LIBRARY:WHOLE_ARCHIVE,ovf-nodes>`（MinGW→`--whole-archive`，MSVC→`/WHOLEARCHIVE`） |
| ovf-nodes DLL 导入库与静态 core 重定义 | MinGW DLL 自动导出把 core 符号再导一遍 | ovf-nodes 在 WIN32 下也改静态 |

MSVC（VS Build Tools）构建是下一步——上游 CMake 的 MSVC 分支（`/W4 /utf-8`、winsock 双路径）已就位，主要工作量在 154k→引擎子集的 `/W4` 告警清理。

## 如何加一个自己的算子

算子全部住在 `ovf-nodes/src/`（`file(GLOB CONFIGURE_DEPENDS)` 会自动拾取新文件）：

1. 抄 `ovf-nodes/src/Threshold.cpp` 的骨架：继承 `INode`，`OVF_REGISTER_NODE` 静态注册；
2. 用安全带 API 写 execute（`in_image() / p_num() / step_ok() / OVF_TRY_IN`）；
3. 在 `ovf-nodes/tests/test_user_nodes.cpp` 加行为测试；
4. `cmake --build build && cd build && ctest`。

> Linux 上的历史经验（`--as-needed` 丢库、弱引用探测三层防御）写在 `ovf-nodes/CMakeLists.txt` 与 `docs/hardening/stage4_nodes.md`，Windows 对偶问题（whole-archive）的注释在 `ovf-nodes-link` 处。

## 快速开始

```bash
# 启动 Web 服务（编辑器画布 + REST API）
./build/bin/Release/ovf-web-server.exe -p 8080

# 节点清单
curl http://127.0.0.1:8080/api/nodes
```

C++ SDK 用法见 `ovf-sdk/`；上游的完整文档（501 算子参考、用户手册、架构）仍在 `docs/` 下，作为引擎与 Web 层的参考保留。

## 构建依赖

- CMake ≥ 3.24（`LINK_LIBRARY:WHOLE_ARCHIVE`）
- C++17：MinGW-w64 gcc（已验证）/ MSVC（计划） / Linux gcc（历史验证）
- 无其他强制依赖（OpenCV/Qt6/CUDA 均可选）

## 许可证

Apache-2.0（继承上游）。本仓库为衍生作品，上游版权与许可见 [LICENSE](LICENSE)。
