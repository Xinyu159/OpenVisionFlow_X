# 阶段 2 —— 引擎防崩

> 分支：`hardening`
> 日期：2026-09-15
> 范围：`ovf-core/src/flow.cpp`（主要）、`ovf-web-server/`（第 4 层）、`tests/test_flow_engine.cpp`
> **`ovf-algorithm/` 对上游仍然零 diff**（`git diff baseline-501 -- ovf-algorithm` 为空，本阶段一个字符都没动）

---

## 一、为什么改

这一阶段解决的问题只有一句话：**引擎整条执行路径上，一个 `try/catch` 都没有，而 Web 请求跑在 `detach()` 出去的线程上。**

写算子是手写指针、手写循环的重活 —— 抛异常是常态，不是意外。而在修之前：

```
算子抛异常
  → execute_node 没接住
  → run() 没接住
  → FlowRunner::run_thread（**线程入口**）没接住
  → std::terminate
  → 整个 ovf-web-server 进程当场消失，连是哪个算子弄死的都看不到
```

顺带还有一批**本身就会让引擎自己崩**的缺陷，全是读代码确认过的，不是猜的。

---

## 二、改了什么

### 2.1 四层 try/catch（纵深防御，四层缺一不可）

| 层 | 位置 | 兜住什么 |
|---|---|---|
| 1 | `FlowEngine::execute_node` | **单个算子**抛异常 → 变成带 `failed_node_id` 的 `FlowResult::fail`，流程体面失败 |
| 2 | `FlowEngine::run` | 编排本身（快照、派发、暂停等待、状态回调）抛异常 |
| 3 | `FlowRunner::run_thread` | **线程入口**。这是全代码库最重要的一处 —— 漏了它就是 `std::terminate` |
| 4 | `WebServer::handle_client` | `accept()` 循环 `detach()` 出去的客户端线程；一个畸形请求不该带走整个服务 |

第 1 层实现（`Result<void>` 没有默认构造函数，所以用立即调用 lambda）：

```cpp
auto exec_result = [&]() -> Result<void> {
    try {
        return node->execute(context);
    } catch (const std::exception& e) {
        return Result<void>::failure(
            ErrorCode::ExecutionFailed,
            String("Node '") + node->instance_id() + "' threw an exception: " + e.what());
    } catch (...) {
        return Result<void>::failure(
            ErrorCode::ExecutionFailed,
            "Node '" + node->instance_id() + "' threw a non-standard exception");
    }
}();
```

第 4 层顺带修了一个**异常路径漏 `close()`** 的问题：`handle_client` 拆成
`handle_client`（兜底 + 统一关 socket）和 `handle_client_inner`（真正的处理）。
原先处理中途抛异常会跳过 `CLOSE_SOCKET`，客户端一直挂着等响应。

### 2.2 拓扑排序重复边（本次最有价值的修复）

一个节点的**每个输入端**都会往 `adjacency_list_` 里 push 一条边。所以
`image` 和 `mask` 都接同一个上游时，会得到 `["S", "S"]`：

- 入度按 `pair.second.size()` 算 → `in_degree[X] = 2`
- 递减用 `std::find`，**最多命中一次、只减 1** → 入度永远减不到 0
- → 一个**完全合法的 DAG** 被判成 `"Flow has cyclic dependencies"`，`load_flow` 直接失败

两处都修：建表时去重（`load_flow`）+ 递减改成 `std::count` 计数递减（`topological_sort`）。
第二道保险是故意的 —— 只要还剩一条重复边漏网，入度就会减成负数，所以判据用
`<= 0` 而不是 `== 0`，保证那种情况也能出得来。

### 2.3 `node_times` 全程累积

原先后端**只在失败路径**保留那一个节点的耗时；成功路径整个丢掉 ——
`run()` 结尾重新构造一个 `FlowResult::ok()`，把 `execute_node` 填的
`node_times` 连同 `failed_node_id` 一起扔了。

现在每条退出路径都赋值（成功 / 失败 / 被叫停 / 数据竞争兜底）。

### 2.4 加锁快照，而不是把 `run()` 整个锁住

`run()` 裸读 `nodes_` / `execution_order_` / `execution_mode_`，而
`load_flow` / `clear` / `step_next` / `get_node` 全都上锁 → 数据竞争。

**不能改成全程持锁**：算子的回调和 `node_state_callback_` 会**重入引擎**
（`get_node` / `get_execution_order` / `set_param` 都上锁）→ 自死锁。

所以：在锁内拷快照（`execution_mode_` / `execution_order_` / `nodes_`），**出锁再执行**。
`INode::Ptr` 是 `shared_ptr`，快照之外节点仍然活着。

`run_node()` 同理 —— 原先它持着 `mutex_` 调 `execute_node`，任何重入都会自死锁。
它有 5 个真实调用方（`api_handlers.cpp:707`、`python_bindings`、`ovf_python.cpp:434`、
`editor_main_window.cpp:843`、测试），全部受益。

### 2.5 `Parallel` / `DataDriven` 明确失败

原先这两个函数都是 `return run(context);`，而 `run()` 又按 `execution_mode_`
派发回它们 → **互相递归 → 栈溢出**（实测：SIGSEGV，见下表）。

改成返回清晰的失败信息。**不做真并行** —— 正确的并行需要 `INode` 线程安全，
而全代码库有 611+ 处 `get_input` 写路径、501 个算子一个都没做同步，那是另一个
大得多的工程。明确报"不支持"比装成能跑然后偶发数据损坏强得多。

### 2.6 `nodes_[...]` → `find()`

`operator[]` 查不到会**插入一个空条目**污染 `nodes_` ——
而 `nodes_` 正是拓扑排序算入度的依据。改了 `run()`、`step_next()`。
`step_next()` 失败时**不再推进** `current_step_`，调用方可以重试同一步。

### 2.7 执行顺序确定性

原先 frontier 的入队顺序来自 `unordered_map` 的迭代顺序、出队用 `pop_back()`（LIFO）
→ **同一个流程每次跑的节点顺序都可能不同**。改成 `std::set` + 取字典序最小的先出。

> ⚠️ **这是本阶段唯一会改变现有输出的地方**：无依赖关系的分支之间，
> 执行顺序从"随机"变成"字典序"。数值结果不受影响（49/49 测试全过，
> 含 `FlowEngine_ComplexPipeline` 的 203.0 断言）。

---

## 三、验收门实测

### 门：`ctest -R FlowEngineTest` 通过

```
Total Tests:  49      Passed: 49 (100.0%)      Failed: 0
```

### 门：新增五个断言 —— **并在修复前跑了一遍，证明它们不是空转**

把 `flow.cpp` 暂时 stash 回修复前、重编、重跑，得到下表。这是这一步的意义所在：
**四个门在修复前全都真的会炸，其中两个是段错误。**

| 门 | 断言 | 修复前 | 修复后 |
|---|---|---|---|
| (a) 同一上游喂同一节点两个输入 | 流程能**加载**并跑出正确结果 | ❌ `loaded.is_success() is false`（误判成有环） | ✅ |
| (b) 算子 `execute` 抛异常 | 干净的 `fail` + `failed_node_id`，不崩 | ❌ `ASSERT_NO_THROW failed: exception thrown: simulated operator bug`（真机上= `std::terminate`） | ✅ |
| (c) 5 节点流程 | `node_times.size() == 5` | ❌ `expected 5 but got 0` | ✅ |
| (d) `set_execution_mode(Parallel)` 后 `run()` | 返回失败而不是死掉 | ❌ **SIGSEGV 段错误**，整个测试进程没了（退出码 139） | ✅ |
| (e) 回调重入引擎 | 不自死锁 | 未验（修前会挂死） | ✅ |

### 门：Web 端无回归

```
25/25 通过   (python3 /tmp/e2e_verify.py，服务用新构建重启)
```

> 第一次跑是 24/25，失败项是"文件真的落盘"。查下来**不是回归**：
> 服务是用 `-d examples/flows` 起的，而脚本读的是 `/tmp/ovf-e2e-flows` ——
> 路径对不上。文件其实稳稳落在 `examples/flows/e2e_flow.json`（73 B，
> 是那 46 个字符的 UTF-8 字节数）。用匹配的 `-d` 重起即 25/25。

### 门：上游零 diff

```
git diff baseline-501 --stat -- ovf-algorithm     → 空
```

---

## 四、2.3 `stopped` 掩盖真失败 —— 已按拍板项走完

计划书把这条标了「三项会改变现有输出的修复 —— 不许自行决定」，所以实施到这一步
**先停下来问过用户**，用户选了 **A（只改 API 判定顺序）**。

### 问题

```cpp
// flow.cpp —— 节点报错的同时又被叫停，两个标记一起置上
if (!result.success) {
    result.stopped = context.is_stopped();
    return result;
}

// api_handlers.cpp:654（修前）—— stopped 先判，success 后判
if (result.stopped) {
    response["stopped"] = true;
    response["message"] = "Flow execution stopped by request";
} else if (!result.success) {
    response["error"] = result.error_message;        // ← 被跳过
    response["failed_node"] = result.failed_node_id; // ← 被跳过
}
```

**失效场景**：用户点了「停止」，而这**同一个节点**又真的失败了。此时
`stopped=true` 和 `success=false` 同时成立，API 层先判 `stopped` → 前端显示"已取消"，
**真实的 `error_message` 和 `failed_node_id` 被整个丢掉** —— 那个坏掉的算子
看起来像是被正常取消的，查问题会查错方向。

### 修法：靠 `failed_node_id` 区分两种 `stopped && !success`

走选项 A（只动 API 层），但**加了一个判别**，因为两种情况的 `stopped && !success`
长得一模一样，直接调换判定顺序会把**纯取消**误报成失败：

| 情况 | `failed_node_id` | 按什么报 |
|---|---|---|
| 循环被叫停（纯取消） | **空** —— `run()` 只填 `error_message` | 取消（`stopped: true`，保持原样） |
| 某个节点真失败 + 停止请求在飞 | **非空** —— `run()` 一定填了 | **失败**（`error` + `failed_node`），另带 `stop_requested: true` |

```cpp
const bool stopped_by_request = result.stopped && result.failed_node_id.empty();

if (stopped_by_request) {
    response["stopped"] = true;
    response["message"] = "Flow execution stopped by request";
} else if (!result.success) {
    response["error"] = result.error_message;
    response["failed_node"] = result.failed_node_id;
    if (result.stopped) response["stop_requested"] = true;
} else {
    response["message"] = "Flow executed successfully";
}
```

**不需要改 `FlowResult`** —— 这个判别只用到了引擎本来就填好的字段。

### 前端（`ovf-web-editor/js/editor.js`）

`result.stopped` 字段在纯取消时仍然照发，所以前端原有的「已停止」分支不受影响。
改的是失败分支：原先只显示「流程执行失败」一句话，**等于把后端刚保住的
`error_message` 又吞了一次**。现在把出错节点和错误原文带出来。

### 验收

| 门 | 结果 |
|---|---|
| `test_flow_engine` 新增不变量断言 | ✅ **50/50**（`Stage2_NodeFailureWithStopInFlight_KeepsFailedNodeId`） |
| Web 端 e2e | ✅ **25/25** |
| `test_stop.py`（真·运行中叫停） | ✅ 全过，且响应用的是 `stopped: true` + 无 `error` 字段 —— 纯取消没被误报成失败 |

新增的断言之所以必要：**没有它，这个修复就退化成"纯取消被误报成失败"**。
它同时验证了两侧 —— 真失败时 `failed_node_id` 非空，纯取消时为空。

---

## 五、遗留

- **`FlowRunner::Continuous` 无节流**（计划书 F19）—— 本阶段没动。
  它是空转跑满一个核心，但改成加 sleep 是行为变化，留给阶段 6 一起拍板。
- **501 个算子仍然 0 个轮询 `is_stopped()`**（F18）—— 这是阶段 3 `step_ok()` 要解决的，
  本阶段没有触碰任何算子。
- **`INode` 线程安全** —— 明确不做，`Parallel` 已改为干净失败。

---

## 六、相关

- 计划书：`/home/asus/.claude/plans/majestic-leaping-thimble.md`
- 上一阶段：`docs/hardening/stage1_registration.md`
- 开发指南：`docs/DEVELOPMENT.md`
- 下一个阶段：阶段 3 —— 写算子的安全带 API（`in_image` / `p_num` / `step_ok`）
