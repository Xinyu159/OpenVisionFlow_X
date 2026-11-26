#!/usr/bin/env python3
"""把同一份二进制的**多趟**体检报告合并成一份 `health/node_health.json`。

    ./tools/node_audit/merge_health.py run1.json run2.json run3.json -o health/node_health.json

## 为什么要合并，而不直接拿最后一趟

有 7 个算子的档位**自己就会变**（TIMEOUT 边界，见 compare_reports.py 里的
`UNSTABLE`）。拿单独一趟去生成标注表，等于把"这次运气好不好"写进仓库：

    CudaResize  三趟实测总耗时 5.3 s / 46 s / 40 s
    同一趟可能给 A、下一趟给 D

而这张表是**画布上灰不灰掉一个节点的唯一依据** —— 把一个会挂死 46 秒的节点
标成 A，就是让人把它拖到画布上。

## 规则：逐算子取**最差**档位（A < B < C < D）

- 对**不抖**的算子，几趟结果本来就一样，取最差 = 原值，没有任何副作用。
- 对**会抖**的算子，只要有任何一趟崩过 / 挂过，它就按 D 标注 —— 这是安全侧。

只合并**同一份二进制、同一组参数**跑出来的报告。修复前后的报告不要混进来，
否则会把已经修好的节点永久钉在旧档位上（那正是对拍脚本要回答的问题，
不是这张表要回答的）。
"""

import json
import sys

ORDER = ["A", "B", "C", "D"]


def worst(tiers):
    """取最差档位。空档（没体检过）不参与投票，除非全空。"""
    known = [t for t in tiers if t in ORDER]
    if not known:
        return tiers[0] if tiers else ""
    return max(known, key=ORDER.index)


def main():
    args = [a for a in sys.argv[1:] if a != "-o"]
    paths, out = [], None
    it = iter(sys.argv[1:])
    for a in it:
        if a == "-o":
            out = next(it, None)
        else:
            paths.append(a)
    if not paths or not out:
        print(__doc__.strip().splitlines()[0], file=sys.stderr)
        print("用法: merge_health.py <报告1.json> <报告2.json> ... -o <health.json>",
              file=sys.stderr)
        return 2

    reports = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            reports.append({n["type_id"]: n for n in json.load(f)["nodes"]})

    # 以第一份为骨架；三份的 type_id 集合必然相同（同一二进制、同参数）。
    ids = set(reports[0])
    for p, r in zip(paths[1:], reports[1:]):
        if set(r) != ids:
            print(f"✗ {p} 的算子集合与 {paths[0]} 不同，不能合并", file=sys.stderr)
            return 2

    nodes, counts, moved = {}, dict.fromkeys(ORDER, 0), []
    for t in sorted(ids):
        entries = [r[t] for r in reports]
        tier = worst([e["tier"] for e in entries])
        # note 取**最差那趟**的 —— 它描述的就是这个档位的由来。
        note = next((e.get("note", "") for e in entries if e["tier"] == tier), "")
        nodes[t] = {"tier": tier, "score": min(e["score"] for e in entries), "note": note}
        counts[tier] = counts.get(tier, 0) + 1
        seen = {e["tier"] for e in entries}
        if len(seen) > 1:
            moved.append((t, "/".join(sorted(seen, key=ORDER.index))))

    doc = {
        "_comment": "ovf-node-audit 生成（多趟取最差）。只标注、不改变行为：501 个算子全部照常注册可调用。",
        "_schema": "type_id -> {tier, score, note}；tier 为空表示没体检过",
        "_tiers": {
            "A": "全过",
            "B": "能跑通但元数据有 Error",
            "C": "不崩但金丝雀跑不通/输出缺失/吃内存",
            "D": "崩溃或挂死",
        },
        "_merged_from": [f"{len(reports)} 趟：{', '.join(paths)}"],
        "_counts": counts,
        "nodes": nodes,
    }
    with open(out, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
        f.write("\n")

    print(f"合并 {len(reports)} 趟 → {out}")
    print("  档位分布: " + "  ".join(f"{k} {counts[k]}" for k in ORDER))
    if moved:
        print(f"  几趟之间自己变过的（已按最差取）：{len(moved)} 个")
        for t, seen in moved:
            print(f"    {t}: {seen}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
