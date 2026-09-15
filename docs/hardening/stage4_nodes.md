# 阶段 4 —— `ovf-nodes/` 独立模块 + `--as-needed` 三层防御

> 分支：`hardening`
> 日期：2026-09-15
> 新增：`ovf-nodes/`（整套）、`tests/baseline/`
> 改动：根 `CMakeLists.txt`、`ovf-web-server/CMakeLists.txt`、`ovf-web-server/src/main.cpp`
> **`ovf-algorithm/` 对上游仍然零 diff**（501 个老算子一行未改）

---

## 一、为什么

前三阶段把**平台自己**修好了：注册不再静默覆盖（1）、引擎不再一个异常就带走进程（2）、
写算子的人手里有了工具（3）。

这一阶段解决的是**你的算子住在哪儿、怎么保证它真的被装上**。

答案是新建 `ovf-nodes/`。位置参照已有的 `ovf-algorithm/`，但有一条铁律：

> **`ovf-algorithm/` 对上游保持零 diff。** 501 个老算子是历史资产，
> 不动它，也不往里加东西。

---

## 二、结构

```
ovf-nodes/
├── CMakeLists.txt
├── include/ovf/nodes/
│   ├── nodes.h              # 模块入口 + 三层防御的接口 + OVF_REGISTER_USER_NODE
│   └── upstream_headers.h   # 上游头文件的伞形包含（放这里，不动上游）
├── src/
│   ├── nodes_init.cpp       # 锚点 + 算子清单 + 自检
│   └── Threshold.cpp        # 示例算子（同时是新算子的模板）
└── tests/
    ├── CMakeLists.txt
    └── test_user_nodes.cpp  # 13 条
```

依赖方向是单向的、而且**故意窄**：

```
ovf-nodes ──依赖──▶ ovf-core ◀──依赖── ovf-algorithm
```

`ovf-nodes` **只依赖 `ovf-core`**，不依赖 `ovf-algorithm`。以后想把那 501 个
整体换掉、或者只挑一部分进构建，`ovf-nodes/` 一行都不用改。

---

## 三、`--as-needed` 到底是什么，为什么值得三层防御

每个算子文件里的 `OVF_REGISTER_NODE` 展开出一个匿名 namespace 里的全局对象，
它的构造函数在 **`main()` 之前**往 `NodeFactory` 里登记。这套机制很优雅，
但有一个致命的失败模式：

> **链接器可以把整个 `libovf-nodes.so` 丢掉。**
>
> Ubuntu 的 gcc/ld 默认开 `--as-needed`：一个共享库如果**没有任何被引用的
> 非弱符号**，链接器认为"没人要"，直接在链接期扔掉。库一被扔，静态初始化
> 就不跑，你的算子**一个都不剩** —— 而且**链接期零警告、零报错**，
> 一直到你画完流程点运行，才看见 `Node type 'X' not found`。

这不是理论。**本机实测**（一个只引用 `ovf-core`、不碰用户算子的小程序）：

| 链接方式 | NEEDED 里有 libovf-nodes.so？ | 实跑 `NodeFactory::size()` |
|---|---|---|
| `-lovf-nodes`（裸链） | ❌ 没有 | **0** |
| `-Wl,--no-as-needed -lovf-nodes -Wl,--as-needed` | ✅ 有 | **1** |

两次编译**都没有任何警告**。第一行的 `0` 就是那个静默事故的完整复现。

`$<LINK_LIBRARY:WHOLE_ARCHIVE>` 在本机不可用 —— 需要 CMake ≥ 3.24，
本机是 3.22.1；而且那是**静态库**的概念（强制把 `.a` 的所有成员拉进来）。
对 SHARED 库，正解就是 `--no-as-needed`。

### 三层，每一层挡的东西不一样

| 层 | 在哪 | 机制 | 挡住什么 | 什么时候报 |
|---|---|---|---|---|
| 1 | `ovf-nodes/CMakeLists.txt` | `ovf-nodes-link` 这个 INTERFACE target，自带 `-Wl,--no-as-needed` | 库被链接器丢掉 | 构建期（根本不发生） |
| 2 | `main.cpp` 里的 `initialize_user_nodes()` | ODR-use 一个**强**符号 | 同上，给"没链 `ovf-nodes-link`"的消费方兜底 | 链接期 |
| 3 | `nodes.h` + `nodes_init.cpp` | `require_user_node_module_loaded()`（**弱**引用探测）<br>`require_user_nodes()`（清单比对） | 库整个不在 / 算子被顶掉 / 注册进了另一个 `NodeFactory` 副本 | **运行期，启动时**，消息可操作 |

**唯一的正确姿势：只链 `ovf-nodes-link`，永远不要直接链 `ovf-nodes`。**

```cmake
target_link_libraries(你的目标 PRIVATE ovf-nodes-link)
```

### 第 3 层为什么要有两种检查

它们是两件事，修法也完全不同：

- **`require_user_node_module_loaded()`** —— 库**整个不在**。
  用的是**弱**引用，所以这句话**不参与**"链接器要不要保留这个库"的判定。
  含义是：哪怕第 2 层那句调用被谁删了、库真被丢掉了，这句话照样编得过、
  照样跑得起来，然后把原因和修法原样讲清楚。**这是唯一能在"库已经不在了"
  的前提下还能开口的检查。**

- **`require_user_nodes()`** —— 库**在**，但算子没全注册上。
  拿模块自己维护的算子清单（每个 `OVF_REGISTER_USER_NODE` 在静态初始化期
  登记的那份）去比对真实注册表，分成两类报：
  - `missing` 注册表里根本没有这个 type_id；
  - `hijacked` type_id 在，但**不是我们注册的** —— 撞车了，`FirstWins`
    把我们那份静默丢掉了。这跟"根本没有"是两回事：一个要改链接，一个要改 type_id。

---

## 四、验收门实测

### 门 1：`readelf -d build/bin/ovf-web-server | grep NEEDED | grep ovf-nodes`

```
0x0000000000000001 (NEEDED)             共享库：[libovf-nodes.so]
```

✅ 命中。测试二进制 `test_user_nodes` 同样命中。

### 门 2：反向测试 —— 第 3 层不靠第 2 层也能活

计划书要的是一条。**做了四种组合**，因为"三层各自是不是真的承重"必须逐个证明
（沿用阶段 2 的"stash 回修复前"和阶段 3 的"变异"那条纪律）：

| 组合 | 第 1 层 | 第 2 层 | NEEDED 里有？ | 结果 |
|---|---|---|---|---|
| **A** | 开 (`ovf-nodes-link`) | 有调用 | ✅ 有 | 起来，`node_count = 502` ✅ |
| **B** | **关**（直接链 `ovf-nodes`） | 有调用 | ✅ 有 | 起来，`node_count = 502` —— 被第 2 层的强引用救了 ✅ |
| **C** | **关** | **删掉** | ❌ **没有** | 起不来，**退出码 1**，打印下面那段可操作说明 ✅ |
| **D** | 开 | **删掉** | ✅ 有 | 起来，`node_count = 502` —— 被第 1 层救了 ✅ |

组合 C 的真实输出（这一段就是"可操作"的标准）：

```
[FATAL] [Error 8] ovf-web-server: user node module (libovf-nodes.so) is NOT loaded in this process.

  这个断言几乎总是意味着同一件事：链接器把整个 libovf-nodes.so 丢掉了。
  Ubuntu 的 gcc/ld 默认开 --as-needed —— 一个共享库只要没有被引用的
  非弱符号，链接期就被当成"没人要"扔掉，**零警告**。库一被扔，算子
  的静态初始化就不跑，你的算子全部消失，运行时才报 Node type 'X' not found。

  修法：把链接目标从 ovf-nodes 换成 ovf-nodes-link（它自带
        -Wl,--no-as-needed 把库钉住）：

            target_link_libraries(你的目标 PRIVATE ovf-nodes-link)

  顺带确认构建时开了用户算子模块：-DBUILD_OVF_NODES=ON（默认就是 ON）。
```

组合 A/B/D 证明**单靠任意一层都还活着**；组合 C 证明第 3 层确实会响、
说的确实是能照着做的话。四个格子各有一层承重，没有互相掩护的空转项。

### 门 3：`require_user_nodes()` 真的会响（变异测试）

库在、但算子没注册上的形态，用**幽灵声明**复现：临时加一句
`declare_user_node("nodes.GhostNode")`（声明了，谁也没注册它）。

```
[FATAL] [Error 101] 用户算子模块自检失败：声明 2 个，实际注册上 1 个。

  根本没注册上（注册表里查不到这个 type_id）：
    nodes.GhostNode

  当前注册表里出处标着 "ovf-nodes" 的类型共 1 个：
    nodes.Threshold
  ...（后面是三种原因各自该怎么修）
```

退出码 1。变异按 `/tmp` 备份还原，`diff` 逐字节一致，重跑回到 13/13。

### 门 4：新增 `tests/test_user_nodes.cpp` —— 13/13

最要紧的一条是 `NodesVisibleWithoutExplicitInit`：这个测试文件**从头到尾
没有调用过 `initialize_user_nodes()`**，却断言 `nodes.Threshold` 在注册表里
并且能 `create()` 出来。它验证的就是**第 1 层单独也能让算子注册上**。
哪天有人把 `ovf-nodes-link` 改回直接链 `ovf-nodes`，这条会挂 ——
而且挂的方式正好就是生产环境会发生的那个。

| 覆盖 | 测试 |
|---|---|
| 锚点解析正常 | `ModuleAnchorIsPresent` |
| **第 1 层单独有效** | `NodesVisibleWithoutExplicitInit` |
| 严格注册 | `ModuleIsRegisteredAsStrict` |
| 清单与注册表一致 | `DeclaredMatchesRegistered` |
| `missing` / `hijacked` 分得开 | `CheckReportsMissingTypeId` / `CheckReportsHijackedTypeId` |
| 报错可操作 | `ReportMessageIsActionable` |
| 示例算子行为（4 条） | `ThresholdBinarizesImage` / `RejectsWrongInputType` / `ClampsOutOfRangeParam` / `StopsWhenFlowIsStopped` / `RejectsNonEightBitFormat` |

### 门 5：`-DBUILD_OVF_NODES=OFF` 精确还原今天的构建

在 `/tmp/build-nooff` 里全新配置并构建：

| 检查 | 结果 |
|---|---|
| 生成 `libovf-nodes.so`？ | 没有 ✅ |
| web-server 的 NEEDED 里有 `libovf-nodes`？ | 没有 ✅ |
| 编译参数里有 `-DOVF_WITH_USER_NODES`？ | 没有 ✅ |
| 服务跑起来 `node_count` | **501**（正是阶段 3 的数字）✅ |

### 门 6：新增 `.cpp` 免 reconfigure

放一个 `ovf-nodes/src/_ProbeNode.cpp` 进去，**不手动跑 cmake**，直接
`cmake --build build -j16`：

```
[ 14%] Building CXX object ovf-nodes/CMakeFiles/ovf-nodes.dir/src/_ProbeNode.cpp.o
构建退出码 = 0
node_count = 503
```

✅ `CONFIGURE_DEPENDS` 生效。顺带这次用的是**限定名**写法
`OVF_REGISTER_USER_NODE(ovf::nodes::ProbeNode, ...)`，也编得过 ——
阶段 7 的脚手架要生成的正是这个形式。探针文件已删除，回到 502。

### 门 7：无回归

| 检查 | 结果 |
|---|---|
| `test_node_helpers`（阶段 3 的 18 条） | ✅ **18/18** |
| `test_flow_engine`（阶段 2 的 50 条） | ✅ **50/50** |
| Web 端 e2e（`/tmp/e2e_verify.py`） | ✅ **26/26** |
| 真·运行中叫停（`/tmp/test_stop.py`） | ✅ 全过，仍是 `stopped: true` 且**无 `error` 字段** |
| `git diff baseline-501 --stat -- ovf-algorithm` | ✅ 空 |
| 全量构建警告数（`ovf-nodes/` 部分） | ✅ **0**（另外 17 条全在 `ovf-web-server`，是既有的） |

---

## 五、踩到的坑：弱**变量**符号会段错误

值得单独记一笔，因为它的表现是**段错误**而不是编译错误，很难联想到原因。

锚点最初写成了变量：

```cpp
extern "C" const char* ovf_nodes_module_anchor __attribute__((weak));
...
if (ovf_nodes_module_anchor != nullptr) { ... }      // ← 库一被丢掉，这里就炸
```

**段错误。** 原因：弱**变量**符号解析不到时，符号的**地址**是 0，而读一个
变量就是**通过它的地址取值** —— 空指针解引用。反过来，弱**函数**符号比较的
是函数地址本身，才有"没有就是 `nullptr`"这个语义。

隔离实验（GCC 13 / ld 2.42），三种写法 × 库在/库不在：

| 写法 | 库在 | 库不在 |
|---|---|---|
| `if (fn != nullptr)`（弱**函数**） | 有 | 无 ✅ |
| `if (&var != nullptr)`（弱变量**取地址**） | 有 | 无 ✅ |
| `if (var != nullptr)`（弱变量**读值**） | 有 | **段错误** ❌ |

改成了函数形式（`ovf_nodes_module_anchor()` 返回模块名，顺便比纯布尔有用）。

**这条实验本身也是设计的一部分**：如果没做组合 C，这个段错误会一直潜伏到
某天真的有人把库丢掉的时候才发作，而那时候看到的是"服务启动时莫名其妙段错误"，
比原来的 `Node type 'X' not found` 还难查。

---

## 六、`OVF_REGISTER_USER_NODE` 和 `OVF_REGISTER_NODE` 的两点不同

```cpp
OVF_REGISTER_USER_NODE(ThresholdNode, "nodes.Threshold", ThresholdNode::make_info())
```

1. **严格模式**（等价于 `OVF_REGISTER_NODE_STRICT`）—— 元数据问题
   （`info.id` 和注册用的 type_id 对不上、端口重名……）会在
   `initialize_user_nodes()` 里被升级为致命，一次列全。
   你自己的算子从第一天起就是干净的，不会攒成 501 份历史债。
2. **记进模块清单** —— `require_user_nodes()` 靠它判断"应该有几个算子"。
   没有清单就没法知道"少没少"。

type_id 推荐带 `模块.` 前缀（`nodes.MyBlur`），撞车时一眼可见；
上游那 501 个都没有点，天然分得开。

模块里所有翻译单元由 CMake 打上**编译期**的 `OVF_NODE_ORIGIN="ovf-nodes"` ——
必须是编译期的，因为注册发生在静态初始化期，`main()` 里再 `set_origin()`
对那些早就注册完的算子已经太晚。

---

## 七、给新算子写的模板

`ovf-nodes/src/Threshold.cpp` 就是模板本体。骨架：

```cpp
#include "ovf/nodes/nodes.h"
namespace ovf { namespace nodes {

class MyNode : public INode {
public:
    MyNode(const String& id) : INode(id, make_info()) {}
    static NodeInfo make_info();
    Result<void> execute(FlowContext& ctx) override;
};

NodeInfo MyNode::make_info() {
    NodeInfo i;
    i.id = "nodes.MyNode";  i.name = "我的算子";
    i.category = "图像处理"; i.version = "0.1.0"; i.author = "我";
    i.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    i.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    i.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(128)).range(0, 255));
    return i;
}

Result<void> MyNode::execute(FlowContext& ctx) {
    OVF_TRY_IN(image, in_image("image"));            // 缺/类型错当场报，带节点名
    OVF_TRY_IN(thr,   p_num_in("threshold", 0, 255)); // 越界自动夹 + WARN
    for (uint32_t y = 0; y < image.height; ++y) {
        if (!step_ok(ctx, y, image.height)) break;    // 被叫停 / 超预算
        ...
    }
    set_output("image", Data(std::move(result)));
    return Result<void>::success();
}

OVF_REGISTER_USER_NODE(MyNode, "nodes.MyNode", MyNode::make_info())
}}
```

然后 `cmake --build build -j16` —— **不需要重新跑 cmake**。

---

## 八、上游 501 个 type_id 的基线快照

新增 `tests/baseline/upstream_501_type_ids.txt`（501 行）+ 一份 `README.md`。

**为什么需要它：** 验收脚本原先写的是 `count == 501`。加了用户算子之后这个
判据就废了 —— 总数会变，而**总数的变化恰恰掩盖真正的风险**：某个老算子被顶掉了、
同时用户算子多了一个，总数照样"正常"。

改成**按名字做集合包含**之后，判据与用户算子的多少彻底解耦：

```
上游 501 个 type_id  ⊆  进程注册表里的 type_id
```

同时断言**上游那 501 个的端口/参数数量一个都没变**（2217 / 3414）——
这才是"老算子的算法数值不许变"在接口层面的对应物。

名单是阶段 4 开工**之前**用阶段 3 那份二进制抓的干净快照。**永远不该改它。**

---

## 九、遗留

- **"文件编进去了但忘了写 `OVF_REGISTER_USER_NODE`"检测不到。**
  清单和注册是同一个宏产生的，所以宏被整个删掉时两边一起消失，自检看不见。
  **归阶段 7**：`new_node.sh` 生成的文件里注册宏是现成的，这条路基本被堵死；
  真要做严格检查，得让 CMake 把 `src/*.cpp` 的名单传进来比对。
- **`/tmp/e2e_verify.py` 仍活在 `/tmp` 里。** 它是阶段 2/3/4 的验收脚本，
  但不在仓库里，重装系统就没了。**归阶段 8**（和 `test_all.cpp` 的自递归一起收拾）。
- **`ovf-nodes/` 目前不链 OpenCV。** 真写视觉算子时多半要，那时候显式加
  `target_link_libraries(ovf-nodes PUBLIC ${OpenCV_LIBS})` —— 有意为之，不是漏掉。
- **模块自检是"启动时一次"。** 运行期动态注册（`NodeFactory::register_node`
  直接调）不受它管 —— 平台目前没有这条路径，有了再说。

---

## 十、相关

- 计划书：`/home/asus/.claude/plans/majestic-leaping-thimble.md`
- 上一阶段：`docs/hardening/stage3_authoring.md`
- 再上一阶段：`docs/hardening/stage2_engine.md`
- 开发指南：`docs/DEVELOPMENT.md`
- 下一阶段：阶段 5 —— 501 算子体检（fork 子进程 + 参数 fuzz）
