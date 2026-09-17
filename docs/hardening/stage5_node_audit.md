# 阶段 5 —— 501 算子体检（fork 子进程）

> 计划书：`~/.claude/plans/majestic-leaping-thimble.md`
> 工具：`tools/node_audit/`（可执行文件 `build/bin/ovf-node-audit`）
> 报告：`build/reports/node_audit.json` / `.md`，标注表 `health/node_health.json`
> 分支：`hardening`

## 一句话结论

502 个算子全部体检完毕：**A 313 / B 11 / C 112 / D 66**。
66 个算子会崩或会挂，共 202 个死亡点；**它们全部保持注册、可调用**，
只是 `/api/nodes` 多了一个 `health` 字段，画布上可以把 D 档灰掉。

**没有摘掉任何一个算子**（计划书硬约束 4）。

---

## 一、验收门：★ 必须复现 `sampling_interval=0` 死循环

计划书的原话是：

> **如果连 `sampling_interval=0` 的死循环都复现不出来，说明 fuzzer 没打到代码，
> 先修 fuzzer，其余结论一律不可信。**

**结论：已复现。**

| 算子 | 源码位置 | 预期 | 实测 |
|---|---|---|---|
| `SurfaceRoughnessInspection` | `automotive_inspection.cpp:1883` | `sampling_interval=0` → 死循环 | ✅ S5 / `sampling_interval=0` → `std::bad_alloc` |
| `ThresholdMorph` | `morphology.cpp:709` | `kernel_size=0` → 空核照用 | ✅ S5 / `kernel_size=1e9` → SIGSEGV |
| `Resize` | `geometry.cpp:226` | 宽高巨值 → `int` 溢出 | ⚠️ 崩了，但触发点是 `scale_x=0`（另一个 bug，见 §三.1） |

### 关于"死循环"表现成 `bad_alloc`

它**不是** TIMEOUT，而是 `execute` 里抛出的 `std::bad_alloc`，被外层 catch 接住、
子进程**活着**退出。原因：这个循环一边转一边往 `profile` 里塞数据，
于是它先撞上 `RLIMIT_AS`（1024 MB）而不是墙钟预算（2000 ms + 3000 ms 宽限）。
实测 441 ms 就炸。

**这比 TIMEOUT 更有价值**：它证明内存上限在真的干活，而且这个缺陷是**可以被
干净捕获的失败**，不是必须杀进程才能收场的挂死。

> ⚠️ 工具因此踩过一个坑：最初的判据只看"子进程有没有死"，会把这个**已经复现**的
> 缺陷判成"没复现"。现在的判据是"死了 **或** 抛了异常"（"抛出"是子进程记录
> 被 catch 住的 C++ 异常时的原话，干净的 `Result::failure` 不含这个词，不会误判）。

---

## 二、结果总览

```
A  313   全过
B   11   能跑通，元数据有 Error 级问题
C  112   不崩，但金丝雀跑不通 / 输出缺失 / 吃内存
D   66   崩溃或挂死          ← 202 个死亡点
```

耗时 **149 s**（16 核并行，502 个算子 × 平均 ~20 步）。
串行不可行：这是 `fork` + `poll` 并行存在的理由。

### 死亡点分布（D 档，按场景）

| 场景 | 死亡点 | 说明 |
|---|---|---|
| **S5 参数 fuzz** | **172** | 绝对主因。极端参数值打出来的 |
| S4 金丝雀 | 21 | 切尺寸/通道切出来的（tiny 8 + gray 8 + rgb 5） |
| S6 输出检查 | 6 | |
| S1 什么都不接 | 2 | |
| S2 空输入 | 1 | |

### 死因

| 信号 | 个数 | 含义 |
|---|---|---|
| TIMEOUT | 36 | 挂死（大多是被极端参数顶成死循环） |
| SIGSEGV | 23 | 段错误 |
| SIGFPE | 6 | 整数除零 |
| SIGABRT | 1 | |

### 最集中的几个触发点

`rack`(9) / `slot`(9) / `roi_height`(7) / `operation`(6) / `db_number`(6) /
`address`(6) / `bit_offset`(6) —— 前两个是 S7 通讯节点的 PLC 地址参数，
后面几个是标定/配置类参数。**绝大多数是"参数取 0 或取极大值时没做边界检查"**，
和计划书阶段 3 要提供的 `p_num` / `p_num_in` 恰好对症。

---

## 三、这次新发现（计划书里没有的）

### 1. `image_utils.cpp:259-260` 除零 → SIGSEGV（**共享工具，影响面最大**）

```cpp
double scale_x = static_cast<double>(src_w - 1) / (dst_w - 1);
double scale_y = static_cast<double>(src_h - 1) / (dst_h - 1);
```

目标宽或高**正好等于 1** 时 `dst_w - 1 == 0` → `scale = inf` → `src_x = 0 * inf = NaN`
→ `(int)NaN` 在 x86 上是 `INT_MIN` → 数组下标飞到天上 → 段错误。

**这不是 `Resize` 一个算子的问题**，`resize_bilinear` 的调用方全都有份
（`advanced_matching.cpp:112,172`、`dl_training.cpp:1219`、`build_pyramid` …）。

而且它**只在一个方向上崩**，这个不对称值得记一笔：

| 目标尺寸 | 下标表达式 | 结果 |
|---|---|---|
| `dst_w=1` | `(y0*256 + INT_MIN)` | 一直是巨大负数 → **SIGSEGV** |
| `dst_h=1` | `(INT_MIN*256 + x)` | `INT_MIN*256` 在 2³² 下**回绕成 0** → 变成 `src[x]`，越界没了，但**静默算出垃圾** |

也就是说 `scale_y=0` 那条路**不崩、但结果是错的** —— 这种"不报警的错"比崩溃更难查。

> 计划书给 `Resize` 的探针写的是"宽高巨值 → `dst_w*dst_h*channels` 在 int 下溢出"。
> 那条路**仍然没被打到**：`use_scale_` 只在宽和高**同时** ≤0 时才为真，单参数 fuzz 够不着。
> 这是已知覆盖盲区，不是 fuzz 坏了（报告里判 ⚠️ 而不是 ✅，就是为了不冒名顶替）。
> 它实测崩在**另一个**真实存在的点上，也就是上面这个除零。

### 2. `ThresholdMorph` 除了 `kernel_size` 还会被 `iterations=1e9` 顶成 TIMEOUT

一次体检挖出同一算子的**两个独立缺陷**（`kernel_size=1e9` → SIGSEGV；
`iterations=1e9` → TIMEOUT）。这正好说明"每个死亡点单独记一行"是必要的 ——
只报第一处的话，第二个坑就被第一处盖住了。

### 3. `WaferDie` `die_width=0` → SIGFPE

整数除零。属于晶圆那条线，和已有的 `wafer_inspection` 测试相邻。

---

## 四、怎么信的（可重复性）

阶段 6 要拿这份报告当"这个修复有没有改变结论"的唯一裁判，所以**必须先证明它自己稳**。

### 4.1 ⚠️ 本节原来的数字是错的（2026-09-15 更正）

原来这里写的是「**500/502 完全一致**，只有 `KMeansSegment`、`RandomCrop` 两个在抖」。
**那是低估。** 阶段 6 修 F1 时把同一二进制在空闲机器上连跑了三趟，实测抖的更多：

| 算子 | 三趟实测总耗时 | 抖动来源 |
|---|---|---|
| `CudaResize` | 5.3 s / 46 s / 40 s | TIMEOUT 边界 |
| `stereo_match_ncc` | 5.4 s / 38 s / 5.6 s | TIMEOUT 边界 |
| `DenoiseNLM` | 2.4 s / 11.8 s / 11.9 s | TIMEOUT 边界 |
| `KMeansSegment`、`ShapeContext`、`WaferDicingInspection`、`RandomCrop` | — | TIMEOUT 边界 |

为什么原来只看见 2 个：**只跑了两趟**，而单个算子的抖是概率性的 ——
跑得少就漏。这也说明「两趟一致」不足以作为稳定性证据。

另：`QRCodeDecodeEnhanced` 是**另一类**不稳 —— 档位稳稳是 D，但**死在哪个信号**
会变：同一二进制连跑两趟，先 `SIGABRT` 后 `SIGSEGV`（单独 `--only` 跑四趟全是
`SIGSEGV`）。它那块代码写坏了堆，glibc 的堆检查先发现就 abort、野指针先踩到
不可写页就 segv，谁先到看堆布局。**死法指纹对它是不可用的**。

结论修正为：
- **档位稳定的 495 个可以当裁判用**；上面那 7 个的档位不要当成确定事实，
  要复核就单独加预算（`--only KMeansSegment --budget-ms 20000`）。
- 总档位数字在几趟之间浮动 ±2～3，来源就是这些边界项。

对拍脚本 `tools/node_audit/compare_reports.py` 把这两类都记了名单，
跑的时候会自动把它们的差异降级成"判不了案"而不是"回归"。

### 4.2 ★ 一个更要紧的方法论坑：**死得早 = 白捡一个好档**

`Mixup` 在阶段 5 报的是 **A 档**，其实是**假 A**。

它的场景链是 S0→S1→S2→S3→S4→S5→S6，而 **S6（"声明过的输出端口必须有产出"）
排在最后**。修 F1 之前，它跑到 S5 第 2 步就 `std::bad_alloc` 了，链条**断在那里，
S6 从来没执行过** —— 于是得分按"已执行的步数"算，白捡一个 A。

修完 F1，链条第一次跑通到底，S6 立刻报出 `mixed_annotation` 端口没有产出 →
档位 A → C。**这不是修坏了，是修之前那份 A 根本不算数。**

所以读这份报告时要注意：**一个算子在 S5 就死掉，它在 S6 上的分是不可信的**。
（`audit_worker.cpp` 记了"跑了 N/M 步"，N < M 就说明后面没跑到。
`note` 里也带着这个数字。）

---

## 五、三处**故意偏离**计划书的地方

改动都在工具里，不动任何算子。理由写全，免得以后被当成 bug。

### (1) S4/S5 用 256×256，不是 8×8；8×8 降级成一个单独的 flavour

计划书的场景矩阵把 S4 金丝雀定成 8×8。实测**不够**：

8×8 对很多算子是**退化输入**而不是"正常输入" —— 测量窗口比图还大，
`start_x = width/2 - measure_length/2` 算出负数、按 uint32 回绕成 42.9 亿。
于是 `SurfaceRoughnessInspection` 在 S4 就崩，**每轮都死、连吃两次重启预算**，
S5 还没跑到 `sampling_interval` 就被掐断 —— ★ 验收门因此一度复现不出来。

现在：S4 基准也用 256，**"小图"作为一个独立的 flavour `tiny` 单列一步**。
这样"小图崩"仍然会被抓到、如实记进报告（实测确实抓到 8 个死亡点），
但它不会再冒充参数 fuzz 的结论，也不会再把 S5 挡在门外。

**副作用**：只有 3 个算子（`GearInspection` / `PaintQualityInspection` / `StainDetect`）
是"仅在小图下崩、别处都好"。也就是说 66 个 D 档**不是**这个新探针刷出来的。

### (2) 硬失败之后**重启**，而不是就此停手

计划书写"硬失败即停"。但那样 S4 一崩，S5 的 fuzz 永远跑不到，恰好丢掉最有价值的结论。
改成：死在哪一步，就从**下一步**重新 `fork` 继续跑。

每一步都是独立的 `begin`/`end`，所以**归因精度不变**（仍然是"死在这一步"）。

默认给 `--max-restarts 32` 而不是 4：一份完整计划也就 ~25 步，
**每步都崩也不过 25 次**，天然有界。定成 4 的时候，`SurfaceRoughnessInspection`
（`measure_length` 三个取值全崩）会在跑到 `sampling_interval` 之前就烧光额度。

### (3) `--max-restarts 0` 可退回严格的"即停"

给需要"第一次崩就停、保留现场"的场合留的出口。

---

## 六、已知覆盖盲区（写出来，不是漏掉）

1. **一次只动一个参数。** 需要两个参数同时越界才出事的路径打不到 ——
   `Resize` 的 `dst_w*dst_h*channels` 溢出就是典型（`use_scale_` 要求宽高同时 ≤0）。
2. **只喂合成数据。** 金丝雀是梯度图 + 小点云，真实图像里的病态内容（全黑、
   高噪声、16 位饱和）打不到。
3. **不测并发。** 每个子进程都是单线程，`INode` 的线程安全问题照旧存在
   （计划书里已明确列为"不做"，真并行是另一个大工程）。
4. **参数 fuzz 的取值集合是固定的** `{0, -1, 1e9}` + 声明范围 ±1。
   像"比图宽大一像素"这种**相对**越界值，靠现在的集合撞不上。

---

## 七、工具本身

```
tools/node_audit/
├── audit.h           共享类型：Scenario/Status/Evidence/NodeVerdict/Options/Step
├── audit_child.cpp   子进程：构造场景、喂输入、执行、把结论写进管道
├── audit_worker.cpp  父进程：fork、poll、超时硬杀、重启、归因、分档
├── audit_report.cpp  JSON + Markdown + 标注表
└── main.cpp          命令行
```

**为什么必须是子进程**：C++ 杀不掉线程；看门狗设 `context.stop()` 只对"配合的"
算子有用，而 501 个里**0 个**轮询 `is_stopped()`；`detach()` 掉卡住的线程
则会泄漏它、它还会在后台继续改全局状态。只有 `fork` 能让一个挂死的算子
**伤不到父进程**。

子进程规矩：单线程（父进程先 `initialize_algorithm_module()`）／只准 `_exit()`
不准 `exit()`／fork 后第一件事清 logger sink／stdout、stderr 重定向到 `/dev/null`
（结论走专用管道 fd）／`RLIMIT_AS`(1G)、`RLIMIT_CPU`、`RLIMIT_FSIZE`、
`RLIMIT_NOFILE`、`RLIMIT_NPROC=0`、**`RLIMIT_CORE=0`**（501 个会崩的子进程
不写 core，不然磁盘会被填满）／`chdir()` 到该算子的临时目录。

**每跑完一步就写一行 JSON 并 flush**，不是最后写一个大 JSON ——
这样它在第 N 步段错误时，父进程仍然握着前 N-1 步的证据和"是 N 哪一步弄死的"。

### 用法

```bash
export LD_LIBRARY_PATH=build/lib

./build/bin/ovf-node-audit --quick                    # 秒级，A/B/C/D 总览
./build/bin/ovf-node-audit --full --update-annotations # 全场景 + 刷新标注表
./build/bin/ovf-node-audit --only Resize --verbose     # 盯一个算子
./build/bin/ovf-node-audit --grep Caliper              # 名字里带 Caliper 的全部
./build/bin/ovf-node-audit --list                      # 只列算子
```

退出码**默认永远 0**（501 个老算子必然有 D 档，非零会让 CTest 变红，那就没人看它了）。
给 CI 用加 `--fail-on-d`。

---

## 八、顺手发现的两件事（不属于阶段 5，但值得记）

1. **`ConsoleLogSink` 写的是 stdout**（计划书 8.7 的实测证据）：
   `ovf-node-audit --list 2>/dev/null` 依然会打出注册期的 WARN 行 ——
   因为它们在 stdout 上，和真正的输出混在一起。
2. **注册期的那批 WARN 是真实缺陷**，不是噪音：V2×14（`NodeInfo.id` 与
   注册 `type_id` 不一致）、V9×1（`standard` 参数重复 id）、V14×8
   （声明 Number 却带 string options）。修它们要动 `ovf-algorithm/`，
   和硬约束 3（对上游零 diff）冲突 —— **已挂到 Kaneo 等拍板**。

---

## 九、下一步

- **阶段 6**：按 Tier 1 → Tier 2 修缺陷，每修一个重跑 `ctest` + 该算子的体检。
  `health/node_health.json` 就是"这次修复有没有改变结论"的裁判。
- **Web 端**已经接好（`/api/nodes` 的每个节点多一个 `health` 字段，
  启动时 `NodeHealthRegistry::load_default()`，**fail-open**：文件不在也只是没标注，
  绝不让服务器起不来）。实测 502 个节点全部带标注，e2e 回归 26/26 通过。
