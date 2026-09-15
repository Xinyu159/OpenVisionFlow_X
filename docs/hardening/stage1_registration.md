# 阶段 1 —— 注册逻辑加固

> 提交：`1204787`（分支 `hardening`）
> 日期：2026-09-15
> 范围：`ovf-core/` 纯增量。**`ovf-algorithm/` 对上游保持零 diff**（`git diff baseline-501 -- ovf-algorithm` 为空）

---

## 一、为什么改

`NodeFactory::register_node` 原先只有两句：

```cpp
void NodeFactory::register_node(const String& type_id, Creator creator, const NodeInfo& info) {
    creators_[type_id] = creator;
    infos_[type_id] = info;
}
```

问题不在"覆盖"，在于**覆盖是静默的、而且结果不确定**：

- 重名**不报错、不告警**，后来者覆盖
- 而 `OVF_REGISTER_NODE` 是**静态初始化期自注册**，**链接顺序不保证**
  → "谁覆盖谁"取决于链接顺序，等于把结果交给运气
- 冲突信息直接丢掉，事后无法回答"到底是哪个被覆盖了"

配套的三个洞：

| 洞 | 后果 |
|---|---|
| `get_all_types()` 遍历 `unordered_map` | 顺序每次不同 → `/api/nodes` 刷新一次变一次、体检报告无法 diff |
| `ParamDef` 的 `min_value`/`max_value` 在构造函数里**根本没有参数能传进去** | 501 个算子几乎全都没声明过参数范围 —— 这是 `sampling_interval = 0` 能一路走到死循环的根因 |
| `OVF_REGISTER_NODE` 按**类名**粘贴注册器名 | 传限定名会展开成 `struct ovf::nodes::XRegistrar`，C++ 不允许 → 编译失败。新算子不在 `ovf::algorithm` 命名空间里，这个坑必须先填 |

---

## 二、改了什么

### 2.1 `register_node` 加锁 + first-wins + 冲突记录

```cpp
void register_node(const String& type_id, Creator creator, const NodeInfo& info,
                   const RegistrationSite& site = {},
                   RegistrationPolicy policy = RegistrationPolicy::FirstWins,
                   bool strict = false);
```

- **加 `std::mutex`**（`get_info` / `get_all_types` / `has_type` / `size` 等全部加锁）
- **first-wins，不是后来者覆盖**。理由：静态初始化顺序未定义，"后来者覆盖"等于把结果
  交给运气；first-wins 至少**结果确定**（链接顺序固定 → 结果固定），而且**冲突被保留下来**。
- 冲突记进 `conflicts()`，日志带**两个注册点的 `文件:行` + 来源标签**
- 保留显式出口 `RegistrationPolicy::Replace`，故意覆盖老算子时必须显式申请
- `create()` 兜住构造函数抛出的异常，不再让进程 `std::terminate`
- 空 `type_id` / 空 `Creator` 直接拒绝并报错（原先会走到 `std::bad_function_call`）

**后三个参数都有默认值**，所以既有的 3 参数调用点（相机插件 2 处、流程引擎测试 4 处）一行未动。

### 2.2 元数据校验 V1–V15（只报告，绝不拒绝注册）

| 级别 | 规则 | 日志 |
|---|---|---|
| Error | V1 `id` 空 / V2 `id` ≠ `type_id` / V3 `name` 空 / V4–V6 端口参数 id 空 / V7–V9 id 重复 / V10 默认值类型不符 / V11 min·max 非数值 / V12 min > max / V13 选项含空串 / V14 非 String 类型带字符串 options | `WARN` |
| Note | V15 缺 category/description/version/author | `DEBUG` |

**为什么 Error 打 WARN 而不是 ERROR、Note 打 DEBUG 而不是 WARN**：上游 501 个算子里
489 个没填 `author`/`version`，全按 WARN 打会刷满整个启动日志，把真正的 23 条 error 淹掉。

**为什么不拒绝注册**：501 个里有多少条违规在体检之前是未知数，设成致命会直接违反
"老算子继续能跑"的硬约束。

### 2.3 严格模式 `OVF_REGISTER_NODE_STRICT`

```cpp
OVF_REGISTER_NODE(MyNode, "MyNode", MyNode::make_info())         // 老算子
OVF_REGISTER_NODE_STRICT(MyNode, "MyNode", MyNode::make_info())  // 新算子
```

STRICT 的元数据问题在 `assert_no_strict_violations()` 升级为致命。
**不在静态初始化期当场抛** —— 那样只会得到 `terminate called after throwing...` + SIGABRT，
看不到是哪个算子、违反了哪条规则。攒到启动自检再抛，能一次把问题连同 `文件:行` 全列出来。

### 2.4 确定性

`get_all_types()` 加 `std::sort`。

### 2.5 流式构造器（只加不改）

```cpp
DataPort("image", "输入图像", DataType::Image, true).doc("单通道或三通道")
ParamDef("threshold", "阈值", DataType::Number, Data(128)).range(0, 255).doc("二值化阈值")
ParamDef("mode", "模式", DataType::String).choices({"fast", "accurate"})
```

原先 `min_value`/`max_value`/`options`/`description` **根本没有途径能在构造时写进去**。
**501 处既有调用点一行未动。**

### 2.6 注册器改按 `__LINE__` 命名

```cpp
#define OVF_CONCAT_IMPL(a, b) a##b
#define OVF_CONCAT(a, b) OVF_CONCAT_IMPL(a, b)

struct OVF_CONCAT(OvfNodeRegistrar_, __LINE__) { ... };
```

老宏是 `struct NodeClass##Registrar`，传限定名会展开成 `struct ovf::nodes::XRegistrar` ——
C++ 不允许用限定名做类定义：

```
error: qualified name does not name a class before '{' token
```

（已实测确认。）改成按行号命名后限定名和非限定名都能用。
**已核实全仓库没有任何一行出现两个以上 `OVF_REGISTER_NODE`**，不会撞名。

### 2.7 `OVF_NODE_ORIGIN` 编译期来源标签

注册发生在静态初始化期、**比 `main()` 早得多**，所以 `main()` 里再 `set_origin()`
对已经注册完的算子已经太晚。来源标签必须能在编译期给：

```cpp
#define OVF_NODE_ORIGIN "ovf-nodes"      // 写在文件头，include node.h 之前
```

### 2.8 `data_type_name()`

错误消息能报"期望 Image，实际 Number"，而不是类型枚举的整数。

### 2.9 启动摘要

`ovf-web-server/src/main.cpp` 的 `initialize_algorithm_module()` 后面加了
`NodeFactory::instance().log_summary()`，启动时打一行：

```
NodeFactory: 501 node type(s) registered, 0 duplicate conflict(s), 23 metadata error(s), 489 metadata note(s)
```

**冲突数非 0 就意味着有算子被静默丢弃，这一行就是发现它的地方。**

---

## 三、验收门实测

| 门 | 命令 | 结果 |
|---|---|---|
| 构建 | `cmake --build build -j16` | ✅ 通过，零错误 |
| 回归 | `ctest --test-dir build/tests -R FlowEngineTest` | ✅ Passed |
| 注册数 | 探针读注册表 | ✅ **501 registered** |
| 冲突 | 同上 | ✅ **0 conflicts** |
| 上游零 diff | `git diff baseline-501 --stat -- ovf-algorithm` | ✅ 空 |

`test_flow_engine.cpp:192/199` 那两个断言（`test.add_one` 的 type_id 与 NodeInfo）**仍然成立**。

> ⚠️ `--test-dir` 必须是 `build/tests`，写 `build` 会报 "No tests were found"
> —— `enable_testing()` 只在 tests 子目录里调了。

**爆炸半径 = 0**：当前 501 个 type_id 零重复，所以这一整套改动**不改变任何既有行为**，
只是把"以后一定会咬你"的坑先填上。

---

## 四、体检发现：512 条元数据问题（23 error + 489 note）

489 条 note 全是 V15（缺 author/version/description），上游本来就这样，不算缺陷。

**23 条 error 是 3 个真缺陷，原计划里没有，是这次体检新发现的：**

### D1 · V2 ×14 · `ovf-algorithm/src/pcl_3d_reconstruction.cpp:3134-3147`

| 注册 type_id | `info().id` |
|---|---|
| `PointCloudFromDepth` | `pcl_pointcloud_from_depth` |
| `PointCloudFromStereo` | `pcl_pointcloud_from_stereo` |
| `PointCloudFromFile` | `pcl_pointcloud_from_file` |
| `PointCloudFilter` | `pcl_pointcloud_filter` |
| `PointCloudDownsample` | `pcl_pointcloud_downsample` |
| `PointCloudNormalEstimation` | `pcl_normal_estimation` |
| `PointCloudICP` | `pcl_icp_registration` |
| `PointCloudNDT` | `pcl_ndt_registration` |
| `PointCloudFeatureMatching` | `pcl_feature_matching_registration` |
| `PointCloudGlobalRegistration` | `pcl_global_registration` |
| `PointCloudSegmentPlane` | `pcl_plane_segmentation` |
| `PointCloudSegmentCluster` | `pcl_cluster_segmentation` |
| `PointCloudMesh` | `pcl_mesh_reconstruction` |
| `PointCloudSurface` | `pcl_surface_reconstruction` |

**实测：14/14 的 `create(info().id)` 全部失败** —— 是断链，不是潜在风险。

消费者拿到的是查不到的名字：

- `ovf-web-server/src/api_handlers.cpp:460` —— `handle_flow_load` 响应里 `n["type"] = node->info().id`
- `ovf-web-server/src/api_handlers.cpp:865` —— step 响应里 `response["node_type"] = node->info().id`

**前端编辑器本身不受影响** —— 它用的是 `/api/nodes` 的 `type_id`（`api_handlers.cpp:393`），
那是注册表的键，是对的。所以这不是"今天就在炸"，是"API 消费者会踩"。

### D2 · V9 ×1 · `barcode.cpp:3281`

`BarcodeGrade` 的参数 id `standard` 出现两次。`ParamSet` 是 `HashMap<String, Data>`，
所以**两个 `ParamDef` 的默认值里有一个静默丢失**，用户在编辑器里改的也只会是其中一个。

### D3 · V14 ×8

位置（部分）：`pcl_3d_reconstruction.cpp` ×6、`ocr.cpp:466`、`3d_advanced.cpp:2096`。

例：`ocr.cpp:466` 的 `min_confidence` 声明 `type = Number` 却带 5 个字符串 options。

前端**不会**把 Number+options 渲染成下拉框 —— 后端的连接/参数存储用的是整数下标，
转了会把存的整数悄悄换成字符串。所以这些参数在编辑器里**没有像样的控件**。

---

## 五、遗留

三个缺陷**都没修**，因为修 D1/D2 大概率要动 `ovf-algorithm/`，
而硬约束是那个目录对上游零 diff。三个选项（改 `info.id` / 改注册名 / 只标注）
和倾向写在 Kaneo 任务 #13 里，**等拍板**。

---

## 六、相关

- 计划书：`/home/asus/.claude/plans/majestic-leaping-thimble.md`
- 开发指南：`docs/DEVELOPMENT.md`
- Kaneo：任务 #3（本阶段）、#13（三个缺陷）
- 下一个阶段：阶段 2 —— 引擎防崩
