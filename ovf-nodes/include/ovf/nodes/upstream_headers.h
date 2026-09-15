#pragma once
/**
 * @file upstream_headers.h
 * @brief 上游头文件的伞形包含 —— 写自己的算子时 include 这一个就够了
 *
 * 为什么要单独开一个文件，而不是在 `nodes.h` 里直接 include：
 * 上游的头文件清单以后可能变（换路径、加模块），改这里一处就够，
 * **不动上游任何一个文件**。`ovf-algorithm/` 对上游保持零 diff 是硬约束。
 *
 * 依赖方向是单向的：
 *
 *     ovf-nodes  ──依赖──▶  ovf-core  ◀──依赖──  ovf-algorithm
 *
 * `ovf-nodes` **只依赖 ovf-core**，不依赖 ovf-algorithm。理由：501 个老算子
 * 是历史资产，你的新算子没必要跟它们耦合在一起。以后想把 ovf-algorithm
 * 整个换掉、或者只挑一部分进构建，`ovf-nodes/` 一行都不用改。
 *
 * 真要用上游某个算法时，显式加：
 *   - `ovf-nodes/CMakeLists.txt` 里 `target_link_libraries(ovf-nodes PUBLIC ovf-algorithm)`
 *   - 这里 `#include <ovf/algorithm/xxx.h>`
 * 但那是**有意的决定**，不是默认。默认只给核心。
 */

// ---- 核心类型 ----
#include <ovf/core/types.h>      // DataType / ImageData / Region / PointCloudData / ErrorCode
#include <ovf/core/error.h>      // Result<T> / ErrorResult / Exception
#include <ovf/core/data.h>       // Data / DataPort / ParamDef / NodeInfo

// ---- 节点与流程 ----
#include <ovf/core/node.h>       // INode / NodeFactory / OVF_REGISTER_NODE / 安全带 API
#include <ovf/core/flow.h>       // FlowContext / FlowEngine

// ---- 基础设施 ----
#include <ovf/core/logger.h>     // OVF_INFO / OVF_WARN / OVF_ERROR / ILogSink
