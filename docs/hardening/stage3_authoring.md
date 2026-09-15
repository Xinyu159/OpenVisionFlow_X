# 阶段 3 —— 写算子的安全带 API

> 分支：`hardening`
> 日期：2026-09-15
> 落点：`ovf-core/include/ovf/core/error.h`、`ovf-core/include/ovf/core/node.h`、
> `ovf-core/src/node.cpp`、`tests/test_node_helpers.cpp`、`tests/CMakeLists.txt`
> **`ovf-algorithm/` 对上游仍然零 diff**（501 个老算子一行未改）

---

## 一、为什么改

阶段 2 修的是「**引擎自己**别崩」。这一阶段修的是另一半：「**写算子的人**别给自己挖坑」。

审计里最刺眼的一条数据是：**501 个算子中，0 个轮询 `is_stopped()`**。也就是说
用户在界面上点「停止」，正在死循环的那个算子根本不知道，流程只能干等。
根因不是写算子的人偷懒 —— 是**写对它太麻烦**：`FlowContext` 只在 `execute(FlowContext&)`
的参数里，要在内层循环里拿到它得自己一层层传下去，还得记得 `break`。

同类问题还有三处，全都是"**拿不到工具，所以只能写错**"：

| 现状 | 后果 |
|---|---|
| `get_input("image")` 拿不到时**静默返回空 `Data{}`** | 算法在空图上跑，输出一片垃圾，没有任何报错 |
| `set_input` 喂错类型，`as_image()` 返回**共享的可变 static 空图** | 类型错当空图处理；那个 static 还是全进程共享的 |
| `ParamDef::min_value/max_value` 原先**根本传不进去**（构造函数写残了） | `sampling_interval=0` 一路走到死循环、`kernel_size=0` 算出垃圾 |
| 参数名拼错 → `get_param` 返回 `Data{}` | 拿 0 去算，查半天查不出是拼错了字母 |

阶段 1 已经把 `ParamDef` 的 `.range()/.doc()/.choices()` 补上了（数据能声明了）。
这一阶段补的是**消费侧**：让新算子一行就能把声明用起来。

**全部是纯增量。** 501 个老算子继续用 `get_input`/`get_param`，一个字符都没动。

---

## 二、新 API 一览

### 2.1 输入：把"静默返回空"换成"带节点名说明原因"

```cpp
Result<Data>           in_data(port);
Result<ImageData>      in_image(port);
Result<Region>         in_region(port);
Result<PointCloudData> in_pointcloud(port);
Result<DepthImageData> in_depth_image(port);
Result<double>         in_number(port);
```

失败分三种，因为**修法完全不同**：

| 情况 | 错误码 | 消息 |
|---|---|---|
| 端口名拼错了 | `NotFound` | `Node 'blur_1': no input port named 'iamge'. Declared inputs: image, mask` |
| 声明了、没连线、没默认值 | `InvalidData` | `Node 'blur_1': input port 'image' has no data (not connected, or the upstream node produced nothing)` |
| 类型不对 | `InvalidData` | `Node 'blur_1': input 'image' expects Image, got Number` |

**每条消息都带节点实例名**。画布上同类节点有二十个的时候，"输入类型不对"等于没说。

端口声明了默认值时按默认值走（与 `get_input()` 的既有语义一致），否则一个带默认值的
端口会被误判成"缺输入"。

### 2.2 参数：声明了范围就自动 clamp + WARN

```cpp
Result<double>  p_num(key);    Result<int32_t> p_int(key);
Result<bool>    p_bool(key);   Result<String>  p_str(key);
Result<double>  p_num_in(key, lo, hi);          // 参数没声明范围时就地约束
Result<int32_t> p_int_in(key, lo, hi);
```

- **参数名不存在 → 直接失败**（`NotFound`，消息里列出所有已声明参数名）。
  老 API 这里是静默返回 `Data{}`，等于拿 0 去算，是最难查的一类。
- **类型不符 → 直接失败**（`p_num("mode")` 而 mode 是 String）。
- **值超出 `min_value/max_value` → 夹到边界 + `OVF_WARN()` 一次**。
  夹取必须留痕：只夹不报就是**静默改行为**，比崩溃更难查（结果悄悄变了，没人知道为什么）。
- **`p_str` 遇到 `options` 且取值不在其中 → 直接失败**。
  悄悄退回第一个选项会让流程走进完全不同的分支，比失败危险得多。
- `p_num_in` 的 `lo > hi` 视为**算子代码自己的 bug**，直接失败，不凑合夹。

### 2.3 循环：`step_ok` —— 一行解决「501 个算子 0 个轮询」

```cpp
for (uint64_t i = 0; i < max_iter; ++i) {
    if (!step_ok(context, i, max_iter)) break;   // 被叫停 / 超预算
    ...
}
```

`max_iterations = 0` 表示不限次数，只受"被叫停"约束。
两种 `false` 的区别是**有意的**：被叫停是正常操作、不打警告；
超预算是异常情况、打一次 `WARN`（结果可能不完整，不能悄悄少算）。

这**不改老算子**，而是让新算子写对这件事的成本降到一行。

### 2.4 失败与传播

```cpp
Result<T> fail_node(ErrorCode, msg)   // 自动带上节点名，并写进 error_message()
ErrorResult Result<T>::error_result() // 让 `if (!r) return r.error_result();` 成立
```

`error_result()` 为什么不是模板成员函数：`Result<ImageData>` 的失败要作为外层函数的
`Result<void>` 返回时，**编译器没法从 return 语句反推** `error_result<U>()` 里的 U ——
返回语句不参与模板实参推导。所以让 `error_result()` 返回一个带**模板化隐式转换**的
小对象 `ErrorResult`，外层返回 `Result<任意类型>` 都成立。

### 2.5 两个宏

```cpp
OVF_TRY_IN(image, in_image("image"));   // 取值 + 失败传播 + 绑定引用
OVF_TRY(validate_inputs());             // 只要传播，不要返回值
```

`OVF_TRY_IN` 展开成三件事：取值 → 失败就 `return r.error_result();`（自动适配本函数
返回类型）→ 成功则 `auto& var = *r;`。上面那个跨类型的例子（`Result<int32_t>` 的失败
变成 `Result<double>` 的失败）由测试实测过。

---

## 三、唯一一处共享布局变化（必须知道）

`INode` 新增了一个成员：

```cpp
mutable std::unordered_set<String> warned_params_;   // warn_once 的限流 key
```

- **安全性**：`INode` 只在库内 `make_shared` 堆分配，**没有插件 ABI 边界在使用**
  （阶段 6 已核实：`OVF_PLUGIN_API` / `dlopen` 全代码库零命中）。
- **代价**：改动它的那一刻起，**`ovf-core` 与 `ovf-algorithm` 必须一起重编**。
  别只重编一边 —— 两边对 `INode` 大小的理解会不一致，那是很难查的堆损坏。
  （本来就在同一棵 CMake 树里，正常构建天然满足。）

---

## 四、验收门实测

### 门：构建通过，且 **501 个 `::execute` 符号都还在**（硬约束）

```
cmake --build build -j16                     → exit 0，0 个 error
nm -C --defined-only build/lib/libovf-algorithm.so | grep " T " \
    | grep -c "::execute(ovf::FlowContext&)"  → 501
grep -rho "OVF_REGISTER_NODE(" ovf-algorithm/src/*.cpp | wc -l  → 501
```

源码口径与产物口径**都是 501，完全对上**。运行时 `/api/health` 也报 `node_count: 501`。

`Result<T>::success(T&&)` 这个右值重载本来最可能引起重载歧义而波及 501 个算子 ——
实测零影响。**全代码库只有一个 `success(std::move(...))` 调用点，就是本阶段新写的那一处**，
所以它对既有行为的影响面是零（不是"没观察到影响"，是"不存在调用点"）。

### 门：新增 `tests/test_node_helpers.cpp` —— 18/18

```
Total Tests: 18   Passed: 18 (100.0%)   Failed: 0
```

| 计划书原文的门 | 对应测试 |
|---|---|
| `in_image` 在未连接端口上失败且消息含节点名 | `InImageOnUnconnectedPortFailsWithNodeName` |
| 喂 `Data(3.0)` 时失败且说 "expects Image, got Number" | `InImageOnWrongTypeSaysExpectedAndGot` |
| `p_num_in` 会 clamp 并记日志 | `PNumInClampsAndLogsOnce` |
| `step_ok` 在超预算后返回 false | `StepOkStopsOnBudget` |

"记日志"那条是真的去抓日志的：测试挂了一个内存 sink，断言 **WARN 级**、正文含参数名
和 `clamped to 10`，并断言**同一个参数只警告一次**。断言日志不是形式主义 ——
clamp 只夹不报就是静默改行为。

### 门：证明这 18 条不是空转（变异测试）

新 API 是纯增量，没有"修复前"可回退，所以改用**变异**验证：把实现改坏三处，
看是不是**恰好**对应的测试挂掉。

| 变异 | 结果 |
|---|---|
| `who_of()` 返回空串（错误消息不再带节点名） | ❌ `InImageOnUnconnectedPortFailsWithNodeName`、`FailNodeCarriesNodeNameAndSetsErrorMessage`（后者不依赖 `who_of`，故仍 PASS，符合预期） |
| `clamp_value()` 直接 `return v`（不夹了） | ❌ `PNumInClampsAndLogsOnce`、`PNumUsesDeclaredRange`、`TryInMacroBindsValueAndPropagatesFailure` |
| `step_ok` 的预算判断前置 `false &&` | ❌ `StepOkStopsOnBudget` |

实测 **18 → 13 通过、5 失败**，失败的正是这 5 条，其余 13 条不受影响 ——
每条断言都是承重的，没有互相掩护的空转项。变异后按 `/tmp` 备份恢复，
`diff` 逐字节一致，重跑回到 18/18。

### 门：无回归

| 检查 | 结果 |
|---|---|
| `test_flow_engine`（阶段 2 的 50 条） | ✅ **50/50** |
| Web 端 e2e（`/tmp/e2e_verify.py`） | ✅ **25/25**，`/api/health` 报 `node_count: 501` |
| 真·运行中叫停（`/tmp/test_stop.py`） | ✅ 全过，仍是 `stopped: true` 且**无 `error` 字段**（阶段 2.3 的判别没被破坏） |
| `git diff baseline-501 --stat -- ovf-algorithm` | ✅ 空 |

---

## 五、给新算子写的模板（改前 / 改后）

以 `geometry.cpp` 的 `RotateNode` 为参照，**新算子**应该这么写
（**不改老算子本体** —— 它们继续用老 API）：

```cpp
Result<void> MyNode::execute(FlowContext& context) override {
    // 输入：缺 / 类型错都在这里当场报出来，消息带节点名
    OVF_TRY_IN(image, in_image("image"));
    const ImageData& src = image;

    // 参数：声明过范围就自动夹，没声明就用 p_num_in 就地约束
    OVF_TRY_IN(angle,    p_num("angle"));          // ParamDef(...).range(-180, 180)
    OVF_TRY_IN(kernel,   p_int_in("kernel", 1, 31));

    // 循环：一行搞定"被叫停"和"超预算"
    for (uint64_t i = 0; i < src.height; ++i) {
        if (!step_ok(context, i, src.height)) break;
        ...
    }

    if (something_failed) {
        return fail_node(ErrorCode::AlgorithmExecFailed, "template matching found no candidate");
    }
    set_output("output", Data(std::move(result)));
    return Result<void>::success();
}
```

---

## 六、遗留

- **501 个老算子仍然 0 个轮询 `is_stopped()`** —— 本阶段提供了工具，但没有、也不会
  去改老算子（硬约束）。真正让老算子可中断是「节点间超时 + `stop()` join 超过 1 秒打 WARN」，
  归阶段 6 的 F18。
- **`Data::as_image()` 的共享可变 static 空图**（F7）—— 在新 API 里已经绕开了
  （`in_image` 从不走那条路），但老路径仍在用。修它归阶段 6。
- **`in_image` 返回的是副本**。与 `get_input()` 同量级（它也是按值返回），
  没有变得更差；但平台整体"每条边拷一次整幅图"的问题依旧，那是更大的重构，
  不在本次范围。
- **`warn_once` 的限流表无锁**。节点本身不是线程安全的（阶段 2 已把 `Parallel`
  改成干净失败），所以这里也不加锁，与既有约定一致。

---

## 七、相关

- 计划书：`/home/asus/.claude/plans/majestic-leaping-thimble.md`
- 上一阶段：`docs/hardening/stage2_engine.md`
- 再上一阶段：`docs/hardening/stage1_registration.md`
- 开发指南：`docs/DEVELOPMENT.md`
- 下一阶段：阶段 4 —— `ovf-nodes/` 独立模块 + `--as-needed` 三层防御
