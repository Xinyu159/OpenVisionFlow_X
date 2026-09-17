# OpenVisionFlow 开发指南

写给「要在这个平台上长期慢慢写算子」的人（也就是我自己）。

---

## 一、仓库与远程

| 远程名 | 地址 | 用途 |
|---|---|---|
| `origin` | `https://github.com/Xinyu159/OpenVisionFlow_X` | **我的仓库，日常开发都推这里** |
| `upstream` | `https://gitcode.com/hunyuan2026/OpenVisionFlow` | 原项目，**只用于拉上游更新**，不推 |

```bash
# 拉上游的新算子 / 修复（只读）
git fetch upstream
git log --oneline upstream/main ^main      # 看上游多了什么
git cherry-pick <sha>                       # 只挑想要的

# 日常
git push origin main
```

> **网络：这台机器上 git 走 GitHub 只有一个配方能通。**
>
> 直连 `github.com` 会被 TLS 重置（挂满超时、不报错）。但**走 HTTP 代理同样是坏的** ——
> git 2.34.1 这一版是 gnutls 后端，`https_proxy`/`http_proxy` 会让它
> `gnutls_handshake() failed: The TLS connection was non-properly terminated`。
> 写进 `git config http.<url>.proxy` 也一样炸。
>
> 能通的是**只走 SOCKS5、并且把 http(s)_proxy 清空**（它俩优先级更高，会截胡）：
>
> ```bash
> unset https_proxy http_proxy
> export all_proxy=socks5h://127.0.0.1:17897
> git push origin main
> ```
>
> 注意是 `socks5h`（h = DNS 也在代理侧解析），不是 `socks5`。
> 另外已全局设了 `http.version=HTTP/1.1` —— HTTP/2 在这台机器上也会被重置。
>
> **代理是偶发的**：同一条命令可能连成功三次、第四次就握手失败。推送包一层重试：
> ```bash
> for i in 1 2 3 4 5; do
>   git push origin main && break
>   echo "第 $i 次失败，重试…"; sleep 3
> done
> ```
> 另外别用 `curl` 能不能通来判断 git 能不能通 —— curl 是 OpenSSL 后端、
> git 是 gnutls，两者行为不一样。
>
> 凭据走 `credential.helper=store`（`~/.git-credentials`，600 权限，不在仓库里）。
> 令牌是 classic PAT，撤销后重新生成，然后：
> ```bash
> printf 'https://Xinyu159:<新令牌>@github.com\n' > ~/.git-credentials && chmod 600 ~/.git-credentials
> ```

---

## 二、分支模型

```
main        平台主干。永远可构建、可运行。只接受「已完成并验过」的东西。
 │
 ├── hardening    本轮平台加固（阶段 1–8）。做完合回 main，然后删掉。
 │
 └── op/<名字>    以后每写一个算子开一条，例如 op/wafer-edge-find
                  写完 → 编过 → 体检过 → 合回 main → 删分支
```

**为什么给算子单开分支**：这个仓库的定位是「平台」。算子写到一半编不过、
或者把 `NodeFactory` 改坏了，`main` 必须还是好的 —— 否则连编辑器都起不来，
查问题会变成在坏掉的平台上查坏掉的算子。

小改动（改个注释、修个笔误）直接上 `main` 就行，不用开分支。

**里程碑打 tag**：

```bash
git tag -a v0.2-hardened -m "平台加固完成：注册加固 + 引擎防崩 + 新算子流水线"
git push origin v0.2-hardened
```

已有 tag：`baseline-501` = 上游 501 个算子 + Linux 可移植性修复。

---

## 三、写一个新算子

> ✅ `ovf-nodes/` 模块已建好（阶段 4），新算子写在那里。
> `new_node.sh` 脚手架还没建（阶段 7）—— 在那之前先手写，参照
> `ovf-algorithm/src/geometry.cpp` 里 `RotateNode` 的写法。
> **新算子一律落在 `ovf-nodes/`，不要往 `ovf-algorithm/` 里加**（见下方约束 3）。

```bash
git checkout main && git pull
git checkout -b op/my-operator

./ovf-nodes/tools/new_node.sh MyOperator --type MyOperator
# 编辑 ovf-nodes/src/MyOperator.cpp
cmake --build build -j16
./build/bin/ovf-node-audit --only MyOperator     # 必须 A 档
LD_LIBRARY_PATH=build/lib ctest --test-dir build/tests

git add -A && git commit
git checkout main && git merge --no-ff op/my-operator
```

---

## 四、算子注册机制（**必读，这是最容易踩的坑**）

### 4.1 注册是「静态初始化期自注册」

```cpp
OVF_REGISTER_NODE(MyOperator, "MyOperator", MyOperator::make_info())
```

这个宏展开成一个匿名 namespace 里的结构体，它的**构造函数**在共享库被加载时自动跑，
往 `NodeFactory` 单例里登记。**没有一个集中的注册表文件**，所以：

- 加算子不需要改任何 CMake 的源文件列表之外的东西
- 但也意味着 **「这个算子有没有注册上」在编译期是看不出来的**

### 4.2 `--as-needed` 会把你的算子整库丢掉

`libovf-algorithm.so` 靠的就是静态自注册。如果链接方**没有引用它任何符号**，
Ubuntu 的 gcc 默认 `--as-needed` 会**把整个库丢掉** → 静态初始化不跑 →
501 个算子一个不剩，而**链接期零警告**，运行时只报 `Node type 'X' not found`。

三层防御（阶段 4 建，**消费方永远只链 `ovf-nodes-link`，不要直接链 `ovf-nodes`**）：

1. `ovf-nodes-link` 这个 INTERFACE target 自动把库包在
   `-Wl,--no-as-needed` … `-Wl,--as-needed` 中间
2. `initialize_user_nodes()` —— ODR-use 一个符号，库自然被保留
3. `require_user_nodes()` —— 运行时断言，失败时抛出可操作的信息，
   **绝不允许静默少一个节点**

> 本机 CMake 是 3.22.1，**用不了** `$<LINK_LIBRARY:WHOLE_ARCHIVE>`（需要 ≥3.24）。
> 而且那是静态库的概念，共享库的正解就是 `--no-as-needed`。

### 4.3 重名不再静默覆盖（阶段 1 已加固）

`type_id` 重复注册时：

- **首次写入者获胜**，后来的**不生效**
- 冲突记进 `NodeFactory::conflicts()`，并打一条 `ERROR` 日志，
  带**两个注册点的 `文件:行`**
- 想故意覆盖老算子，用 `RegistrationPolicy::Replace` 显式申请

为什么是 first-wins 而不是后来者覆盖：注册发生在**静态初始化期，链接顺序不保证**，
「后来者覆盖」等于把结果交给运气；first-wins 至少结果是确定的，而且冲突被保留下来可查。

### 4.4 新算子用 STRICT

```cpp
// 老算子（od 上游 501 个）用这个：元数据问题只记录、只告警
OVF_REGISTER_NODE(MyOperator, "MyOperator", MyOperator::make_info())

// 我自己的新算子用这个：元数据问题在启动自检时升级为致命
OVF_REGISTER_NODE_STRICT(MyOperator, "MyOperator", MyOperator::make_info())
```

启动时 `assert_no_strict_violations()` 会把所有问题**一次列全**（含 `文件:行`），
而不是在静态初始化期当场抛 —— 后者只会得到一句 `terminate called after throwing...`
和 SIGABRT，看不到是哪个算子、违反了哪条规则。

### 4.5 写 `make_info()` 时注意

用链式构造器，**501 处老调用点一行没动、也不会动**：

```cpp
static NodeInfo make_info() {
    NodeInfo info;
    info.id          = "MyOperator";
    info.name        = "我的算子";
    info.category    = "自定义";
    info.description = "……";
    info.version     = "1.0.0";
    info.author      = "马欣语";

    info.inputs.push_back(
        DataPort("image", "输入图像", DataType::Image, true).doc("单通道或三通道"));
    info.outputs.push_back(
        DataPort("result", "结果", DataType::Region).doc("检出的区域"));

    info.params.push_back(
        ParamDef("threshold", "阈值", DataType::Number, Data(128))
            .range(0, 255)                    // ★ 一定要声明范围
            .doc("二值化阈值"));
    return info;
}
```

**`.range()` 是重点。** 原先 `min_value`/`max_value` 在构造函数里根本没有参数能传进去，
所以 501 个老算子几乎全部没声明过参数范围 —— 这正是 `sampling_interval = 0`
能一路走到死循环的根因。新算子从第一天起就该声明。

### 4.6 元数据校验规则 V1–V15

`NodeFactory::register_node` 会跑一遍校验，**只报告、绝不拒绝注册**：

| 级别 | 规则 | 打什么日志 |
|---|---|---|
| Error | V1 id 空 / V2 id ≠ type_id / V3 name 空 / V4–V6 端口参数 id 空 / V7–V9 id 重复 / V10 默认值类型不符 / V11 min·max 非数值 / V12 min > max / V13 选项含空串 / V14 非 String 类型带字符串 options | `WARN` |
| Note | V15 缺 category/description/version/author | `DEBUG`（默认不显示，免得刷屏） |

---

## 五、提交前检查清单

```bash
# 1. 编过
cmake --build build -j16

# 2. 测试（注意是 build/tests，直接 --test-dir build 会报 "No tests were found"）
LD_LIBRARY_PATH=build/lib ctest --test-dir build/tests --output-on-failure

# 3. 动没动算法数值（判据见约束 3）
git diff baseline-501 --stat -- ovf-algorithm
./build/bin/ovf-node-audit --full --json-out build/reports/node_audit.json
#    然后与 build/reports/baseline/stage5_audit.json 逐算子对拍

# 4. 注册表摘要（冲突数非 0 就要查）
./build/bin/ovf-web-server -p 18090 &             # 启动日志里有 NodeFactory 那一行
```

**已知的基线失败（不是我弄坏的）**：

- `SubpixelPrecisionTest`、`ChineseOCRTest` —— 一直是失败的
- `AllTests` —— 会挂死（`tests/test_all.cpp:447-477` 有 8 个自递归，`-O3` 下变成死循环），阶段 8 修

**判据**：这两个失败**不是回归**；`FlowEngineTest` 失败、或者多出新的失败，**就是回归**，必须停下来查。

---

## 六、硬约束（别越界）

1. **501 个老算子继续能编过、能跑** —— 611 处 `get_input(`、1511 处 `get_param(` 一个都不动，新增 API 只能「只加不改」
2. **老算子的算法数值不许变**（例外只有阶段 6 逐条拍板过的那些）
3. **`ovf-algorithm/` 只修缺陷、不加算子** —— 新算子一律落在 `ovf-nodes/`

   > ★ **2026-09-15 用户拍板：判据从「对上游零 diff」改为「算法数值不变」。**
   > 原因：阶段 5 体检查出的越界写、除零、悬垂引用、整数溢出**全都在 `ovf-algorithm/` 里**，
   > 而原「零 diff」约束与本条自相矛盾 —— 照字面执行等于「知道会崩但一个字节都不许修」。
   > 修法限定为**机械的、不影响正常输入的改动**；每修一个用体检报告对拍，
   > 证明只有预期的那几个算子结论变了（`build/reports/baseline/` 存着修前基线）。
   > 代价如实记：以后 `git fetch upstream` 拉更新时，被改过的文件会有冲突。
4. **不摘任何算子** —— 体检结论只做运行时标注，501 个全部保持注册、可调用

---

## 七、体检结果与已知缺陷（阶段 5 跑出来的）

对 **502 个算子**（501 上游 + `nodes.Threshold`）跑 `ovf-node-audit --full`：

```
A  313   全过
B   11   能跑通，元数据有 Error 级问题   ← 就是下面这三条
C  112   不崩，但金丝雀跑不通 / 输出缺失 / 吃内存
D   66   崩溃或挂死                      ← 202 个死亡点
```

**没有摘掉任何一个算子**（硬约束 4）：502 个全部照常注册、可调用，
只是 `/api/nodes` 每个节点多了一个 `health` 字段，画布上可以把 D 档灰掉。

### 三条元数据真缺陷（阶段 1 校验器查出，23 error）

| 规则 | 数量 | 位置 | 问题 |
|---|---|---|---|
| V2 | 14 | `ovf-algorithm/src/pcl_3d_reconstruction.cpp:3134-3147` | 注册名是 `PointCloudFromDepth`，`info().id` 却是 `pcl_pointcloud_from_depth`。**实测 `create(info().id)` 14/14 全部失败**。任何拿 `info().id` 再喂回 `create()` 的地方（`api_handlers.cpp:460` 的 `n["type"]`、`:865` 的 `node_type`）拿到的是个查不到的名字 |
| V9 | 1 | `ovf-algorithm/src/barcode.cpp:3281` | `BarcodeGrade` 参数 id `standard` 重复。`params_` 是 map，两个默认值里有一个**静默丢失** |
| V14 | 8 | `pcl_3d_reconstruction.cpp`、`ocr.cpp:466`、`3d_advanced.cpp:2096` 等 | 参数声明 `type=Number` 却带字符串 `options`，前端画不出控件（前端不会把 Number+options 渲染成下拉框 —— 转了会把存的整数下标悄悄换成字符串） |

### 崩溃面（66 个 D 档里最值得先看的）

| # | 位置 | 问题 | 状态 |
|---|---|---|---|
| F1 | `color_processing.cpp` ×6、`dl_training.cpp` ×8 | `const ImageData& x = get_input(...).as_image();` —— 对**临时对象**取引用，语句一结束就悬垂 | ✅ 已修 |
| F2 | `geometry.cpp:222` | `dst_w*dst_h*channels` 在 `int` 下溢出 → 堆越界写 | ⬜ |
| F2b | `image_utils.cpp:259-260` | `(src_w-1)/(dst_w-1)` 在目标边长 == 1 时除零 → `NaN` → `(int)NaN = INT_MIN` → 越界。**共享工具**，`resize_bilinear` 的所有调用方都有份；且 `dst_w=1` 必崩、`dst_h=1` 静默算垃圾 | ⬜ |
| F2c | `dl_training.cpp:1357-1361` | `crop_w = (int)(width * crop_ratio)`，`crop_ratio=1e9` 时 `int` 溢出 → `apply_crop` 里全是 UB → SIGSEGV。**修 F1 之后才暴露**（之前这个算子每次都 `bad_alloc`，根本跑不到） | ⬜ |
| F3 | `dl_training.cpp:302,3422` | 未判 `ch>=3`，Mono8 下 2 字节堆溢出 | ⬜ 已拍板 |
| F4 | `3d_matching.cpp:612,809` | 空 vector 上 `size()-1` → `SIZE_MAX` → 立刻越界 | ✅ 已修 |
| F5 | `morphology.cpp:709` | `create_kernel` 的返回值被丢弃，空核照用 | ⬜ |
| F6 | `automotive_inspection.cpp:1883` 等 4 处 | `sampling_interval=0` 死循环、`measure_length` 溢出写坏堆 | ⬜ |
| F27 | `dl_training.cpp:2027` | `mixed_annotation` 端口声明了却**条件产出**。修 F1 前它在 S5 就断了、S6 从没执行过，所以阶段 5 给的是**假 A 档** | ⬜ 待拍板 |

**F1 的坑**：原计划写的是「12 处」，实际是 **14 处** —— 计划里那份扫描的正则用
`[a-z_]*` 匹配变量名，**匹配不到带数字的 `image1`/`image2`**（`MixupNode::execute`）。
重写扫描器后拿 `baseline-501` 的源码验证过它**不是空转**（确实报 14 处），才动手改。

**F2c / F27 是体检自己抓出来的**，不在阶段 5 的原清单里 —— 它们被 F1 挡在后面，
F1 一修就露出来了。这正是「用体检报告当裁判」的价值。

**这些在阶段 6 逐条修**（2026-09-15 用户已拍板：允许改 `ovf-algorithm/` 修缺陷，
判据是「算法数值不变」而不是「零 diff」）。修前基线存在
`build/reports/baseline/stage5_audit.json`，对拍用
`python3 tools/node_audit/compare_reports.py`。

**读报告的注意事项**：一个算子在 S5 就死掉的话，**它在 S6 上的分不可信**
（S6 排在最后，根本轮不到执行）。`note` 里的「跑了 N/M 步」就是看这个的。
