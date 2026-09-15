/**
 * @file node_health.h
 * @brief 算子体检结论表 —— 阶段 5 的产出，编辑器与 API 的输入
 *
 * 背景：501 个上游算子是从别处搬来的历史资产，谁也不知道哪一个会在
 * 什么输入下崩溃。`ovf-node-audit` 把它们逐个跑一遍（fork 子进程隔离，
 * 见 tools/node_audit/），结论落成 health/node_health.json 进仓库。
 *
 * 这个类就是那份 JSON 的运行时视图：启动时加载一次，之后只读。
 *
 * ★ 两条设计纪律：
 *
 * 1. **fail-open。** 文件不在、读不动、格式不对 —— 一律当成"没体检过"，
 *    返回空结论，**绝不抛、绝不阻止启动**。体检报告是锦上添花的东西，
 *    不能因为它缺失就让整个平台起不来。
 *
 * 2. **只标注，不改变行为。** 501 个算子全部照常注册、照常可调用；
 *    结论只用来在画布上灰掉 D 档、在 API 里多带一个字段。
 */

#pragma once

#include "node.h"   // OVF_CORE_API + types.h

#include <mutex>

namespace ovf {

/**
 * @brief 一个算子的体检结论
 */
struct NodeHealth {
    String tier;        //!< "A" / "B" / "C" / "D"；空串 = 没体检过
    int    score = 0;   //!< 0–100，越低越糟
    String note;        //!< 一句话说清为什么是这个档

    bool known() const { return !tier.empty(); }
};

/**
 * @brief 体检结论表（单例，进程内只读）
 */
class OVF_CORE_API NodeHealthRegistry {
public:
    static NodeHealthRegistry& instance();

    /**
     * @brief 从 JSON 文件加载
     * @return 真的加载到了内容才返回 true；文件不存在/解析失败 → false
     *
     * @note **失败不是错误。** 调用方通常不需要检查返回值。
     */
    bool load_file(const String& path);

    /// 从 JSON 文本加载（给测试用）
    bool load_json(const String& json_text);

    /**
     * @brief 按默认顺序找一个能用的报告文件
     *
     * 依次尝试：`$OVF_NODE_HEALTH` → `health/node_health.json` →
     * `../health/node_health.json` → `$OVF_SOURCE_DIR/health/node_health.json`。
     * 一个都找不到就保持空表。
     *
     * 之所以要试好几个路径：二进制在 `build/bin/`、报告在仓库根的
     * `health/`，从哪儿启动 cwd 都不一样。找不到也只是没标注而已。
     */
    bool load_default();

    /// 查结论。没体检过 → 默认构造的空 NodeHealth（tier 为空）
    NodeHealth query(const String& type_id) const;

    /// 体检过的条目数
    size_t size() const;

    bool empty() const;

    /// 体检过的 type_id，按字典序（报告要能 diff）
    Vector<String> type_ids() const;

    /// 从哪个文件加载的；空 = 没加载到
    String source() const;

    void clear();

private:
    NodeHealthRegistry() = default;
    NodeHealthRegistry(const NodeHealthRegistry&) = delete;
    NodeHealthRegistry& operator=(const NodeHealthRegistry&) = delete;

    mutable std::mutex mutex_;
    HashMap<String, NodeHealth> table_;
    String source_;
};

/// 档位 → 分数（A=100 B=80 C=50 D=10）。报告和注册表用的是同一个映射。
OVF_CORE_API int node_health_score_for_tier(const String& tier);

} // namespace ovf
