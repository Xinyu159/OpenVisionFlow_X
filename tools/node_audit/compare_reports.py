#!/usr/bin/env python3
"""对拍两份 `ovf-node-audit --full` 报告，回答一个问题：

    **这次改动有没有改变体检结论？**

阶段 6 每修一条缺陷都要回答它。判据是「算法数值不变」，而体检报告是这条判据
唯一能自动化的证据 —— 修完重跑一遍，逐算子比对档位和死亡点。

    ./tools/node_audit/compare_reports.py \
        build/reports/baseline/stage5_audit.json build/reports/node_audit.json

不给路径时默认就是上面这一对。

## 三种差异，含义完全不同

- **档位变化**：某个算子从 A 掉到 D、或从 D 升到 A。**必须逐条解释**。
- **证据变化**：档位没变，但死亡点（场景 / 端口 / 参数 / 取值）变了。
  比如一个算子本来死在 `kernel_size=0`，改完改死在 `iterations=1e9` ——
  档位还是 D，但这是**修复引入的新行为**，不能当成"没变"放过去。
- **判不了案**：`UNSTABLE` 里那几个算子的**运行耗时本身**在几趟之间能差 8 倍
  （见下方注释里的实测数字），档位跟着 TIMEOUT 判定乱跳。
  它们出现在差异里**不是回归**，但也不能静默放过 —— 单独列出来。

退出码：有**判不了案之外**的差异 → 1；否则 0。方便接 CI。
"""

import json
import re
import sys

# ★ 判不了案的算子：它们的**运行耗时本身**在几趟之间能差 8 倍，
#   于是档位跟着 TIMEOUT 判定乱跳。拿它们当裁判等于掷骰子。
#
#   2026-09-15 实测（阶段 6 修 F1 时）：同一个二进制、机器空闲、连跑三趟
#   --full，这 6 个算子自身就在动 —— 其中 CudaResize 总耗时 5.3s / 46s / 40s，
#   stereo_match_ncc 5.4s / 38s / 5.6s，DenoiseNLM 2.4s / 11.8s / 11.9s。
#   它们全都不在 F1 改过的文件里，与改动无关，是工具自身的分辨率极限。
#
#   ⚠️ 这更正了 docs/hardening/stage5_node_audit.md §四「只有 2 个算子抖」的说法
#   —— 那是**低估**，真实数量是 6 个。
UNSTABLE = {
    "CudaResize", "DenoiseNLM", "KMeansSegment",
    "ShapeContext", "WaferDicingInspection", "stereo_match_ncc",
    # 阶段 5 报出来的两个，一并保留（宁多勿少）
    "RandomCrop",
    # stereo_match_bm：档位稳 D，但**死点数**在动（同二进制 10 → 9）。
    # 它和 stereo_match_ncc 是一对，全是 TIMEOUT 死亡点，个数看调度。
    "stereo_match_bm",
}

# 另一类不稳定，**只作用于死法的名字**：档位稳稳是 D，但**具体是哪个信号**
# 会变。2026-09-15 实测（阶段 6 修 F1）：同一个二进制连跑两趟 --full，
# QRCodeDecodeEnhanced 先 SIGABRT、后 SIGSEGV；单独 --only 跑四趟全是 SIGSEGV。
# 原因是这块代码在 rgb 输入下写坏了堆 —— glibc 的堆检查先发现就 abort，
# 野指针先踩到不可写页就 segv，谁先到看堆布局。
#
# ⚠️ 所以这里**只把这两个信号归一化**，其余照常比较：场景 / 端口 / 参数 / 取值
# 变了仍然报，档位 D→C 也仍然报。绝不能把它整个塞进 UNSTABLE —— 那等于
# 将来真把 QR 解码的堆写坏修好了，脚本却当"判不了案"静默放过。
UNSTABLE_SIGNAL = {"QRCodeDecodeEnhanced"}
_SIG_PAIR_RE = re.compile(r"^SIG(?:SEGV|ABRT)\b")

DEFAULT_A = "build/reports/baseline/stage5_audit.json"
DEFAULT_B = "build/reports/node_audit.json"


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def by_type(report):
    return {n["type_id"]: n for n in report["nodes"]}


def death_fingerprint(node, normalize_signal=False):
    """一个算子的死亡点集合，与顺序无关。

    只取「死在哪」的坐标，不取耗时 —— 毫秒数每次都不一样，纳进来等于每次必报差异。
    `death` 标记来自 worker 为每处死亡单独补的证据（见 audit_worker.cpp）。

    `normalize_signal` 给 UNSTABLE_SIGNAL 用：把那对分不清先后的信号抹平成
    一个记号，其余原样保留。
    """
    out = []
    for e in node.get("evidence", []):
        if not e.get("death"):
            continue
        detail = e.get("detail", "")
        if normalize_signal:
            detail = _SIG_PAIR_RE.sub("SIGSEGV/SIGABRT", detail)
        out.append((
            # 键名是短的那个 —— 报告里场景序列化成 "s"（见 audit_report.cpp:270），
            # 不是 "scenario"。写错了不会报错，只会静默漏掉"参数相同但场景不同"的变化。
            e.get("s", ""),
            e.get("flavour", ""),
            e.get("port", ""),
            e.get("param", ""),
            e.get("value", ""),
            # detail 里带了信号名（SIGSEGV / TIMEOUT）和死因描述。纳入比较 ——
            # 同一个参数上从 SIGSEGV 变成 TIMEOUT 是实质变化。
            e.get("detail", ""),
        ))
    return sorted(out)


def fmt_deaths(fp):
    if not fp:
        return "（无死亡点）"
    return "；".join(
        f"{s}/{f or p or '—'}" + (f"={v}" if v else "") + f" [{d}]"
        for s, f, p, _param, v, d in fp
    )


def main():
    a_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_A
    b_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_B

    try:
        a, b = load(a_path), load(b_path)
    except FileNotFoundError as e:
        print(f"✗ 读不到报告：{e.filename}", file=sys.stderr)
        print("  基线用 --json-out 存一份在 build/reports/baseline/ 下。", file=sys.stderr)
        return 2

    na, nb = by_type(a), by_type(b)
    print(f"对拍：{a_path}\n  →  {b_path}\n")

    sa, sb = a.get("summary", {}), b.get("summary", {})
    print("档位分布：")
    for t in "ABCD":
        mark = "" if sa.get(t) == sb.get(t) else "   ← 变了"
        print(f"  {t}  {sa.get(t, 0):4d}  →  {sb.get(t, 0):4d}{mark}")
    print(f"  死亡点 {sa.get('total_deaths', 0)} → {sb.get('total_deaths', 0)}")
    print()

    missing = set(na) - set(nb)
    added = set(nb) - set(na)
    if missing:
        print(f"★ 算子在 B 里没了（{len(missing)} 个）：{sorted(missing)}")
    if added:
        print(f"★ 算子在 B 里多出来了（{len(added)} 个）：{sorted(added)}")

    tier_changed, evidence_changed = [], []
    for t in sorted(set(na) & set(nb)):
        x, y = na[t], nb[t]
        norm = t in UNSTABLE_SIGNAL
        if x["tier"] != y["tier"]:
            tier_changed.append(t)
        elif death_fingerprint(x, norm) != death_fingerprint(y, norm):
            evidence_changed.append(t)
        elif bool(x.get("resource_limit")) != bool(y.get("resource_limit")):
            evidence_changed.append(t)

    flaky_hits = []

    def report(title, names, show):
        real = [t for t in names if t not in UNSTABLE]
        fake = [t for t in names if t in UNSTABLE]
        print(f"{title}（{len(names)} 个）"
              + (f"，其中 {len(fake)} 个是判不了案" if fake else ""))
        for t in real:
            show(t)
        if not names:
            print("  （无）")
        for t in fake:
            flaky_hits.append(t)
            print(f"  ~ {t}：已知 TIMEOUT 抖动，连跑两趟自己就会变 —— 不算回归，但要复核")
        print()

    report("档位变化", tier_changed,
           lambda t: print(f"  {t}: {na[t]['tier']} → {nb[t]['tier']}"
                           f"   (score {na[t]['score']} → {nb[t]['score']})"))
    report("证据变化（档位没变，但死法变了）", evidence_changed,
           lambda t: (print(f"  {t}  [{na[t]['tier']}→{nb[t]['tier']}]"),
                      print(f"      修前: {fmt_deaths(death_fingerprint(na[t]))}"),
                      print(f"      修后: {fmt_deaths(death_fingerprint(nb[t]))}")))

    real_diff = [t for t in tier_changed + evidence_changed if t not in UNSTABLE]
    if not real_diff and not missing and not added:
        if flaky_hits:
            print(f"结论：✓ 除判不了案外没有任何结论变化（抖动 {len(flaky_hits)} 个）")
        else:
            print("结论：✓ 逐算子完全一致 —— 这次改动没有改变任何体检结论")
        return 0
    print(f"结论：✗ 有 {len(real_diff)} 个算子结论变了 —— 必须逐条解释",
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
