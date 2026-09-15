/**
 * @file Threshold.cpp
 * @brief 示例算子：二值化 —— 同时当"新算子该怎么写"的模板
 *
 * 这个算子本身没什么了不起（就是按阈值切 0/255），它的价值在于**示范写法**：
 * 阶段 3 那套安全带 API 在这里被完整地用了一遍 ——
 * `in_image` / `p_num_in` / `p_bool` / `step_ok` / `fail_node` / `warn_once` / `OVF_TRY_IN`。
 *
 * 对照 501 个老算子的写法（它们继续用 `get_input`/`get_param`，一行不改）：

 * ```cpp
 * // 老写法：缺输入时静默拿到空 Data，算法在空图上跑出一片垃圾，没有任何报错
 * auto d = get_input("image");
 * if (!d.is_image()) { ... }
 * ImageData src = d.as_image();
 * ```
 *
 * ```cpp
 * // 新写法：缺 / 类型错 / 参数越界都在**入口**当场报出来，消息里带节点名
 * OVF_TRY_IN(image, in_image("image"));
 * ```
 */

#include "ovf/nodes/nodes.h"

#include <utility>

namespace ovf {
namespace nodes {

class ThresholdNode : public INode {
public:
    ThresholdNode(const String& instance_id) : INode(instance_id, make_info()) {}

    static NodeInfo make_info();
    Result<void> execute(FlowContext& context) override;
};

NodeInfo ThresholdNode::make_info() {
    NodeInfo info;
    info.id          = "nodes.Threshold";
    info.name        = "二值化（示例算子）";
    info.category    = "图像处理";
    info.description = "把 8 位灰度/RGB 图按阈值切成 0 或 255";
    info.version     = "0.1.0";
    info.author      = "ovf-nodes";

    // 第 4 个参数 = required。声明成必填输入，编辑器会把它画成必连的端口。
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "二值化结果", DataType::Image));

    // 声明了范围，参数越界就会被自动夹住并打一次 WARN（阶段 3 的 p_num）。
    info.params.push_back(
        ParamDef("threshold", "阈值", DataType::Number, Data(128)).range(0, 255));
    info.params.push_back(
        ParamDef("invert", "反相（前景变 0、背景变 255）", DataType::Boolean, Data(false)));
    return info;
}

Result<void> ThresholdNode::execute(FlowContext& context) {
    // ---- 输入 ----
    // 失败消息一定带节点名：画布上同类节点有二十个的时候，"输入类型不对"等于没说。
    OVF_TRY_IN(image, in_image("image"));
    const ImageData& src = image;   // 是 Result 内部的副本，可以安全地读/改

    // ---- 参数 ----
    // threshold 在 ParamDef 里声明了 .range(0,255)，但这里仍然写 p_num_in：
    // 参数的范围约束是**算子自己的**不变量，不该依赖"注册表里的声明有没有写对"。
    // 两层都写，越界就挡在入口。
    // 注意 OVF_TRY_IN 绑的是**引用**（`auto& var = *r`），不是指针 —— 拿到就直接用。
    OVF_TRY_IN(threshold, p_num_in("threshold", 0.0, 255.0));
    OVF_TRY_IN(invert, p_bool("invert"));

    // ---- 校验 ----
    if (src.width == 0 || src.height == 0 || src.data.empty()) {
        return fail_node(ErrorCode::InvalidImage, "input image is empty");
    }
    // data 是字节数组，只有 8 位格式能这么按字节处理。
    // 这里**显式失败**而不是猜 —— 把 Mono16 当 Mono8 处理会得到一张看起来
    // 很像样、其实完全错的图，比报错难查得多。
    switch (src.format) {
        case ImageFormat::Mono8:
        case ImageFormat::RGB8:
        case ImageFormat::RGBA8:
        case ImageFormat::BGR8:
        case ImageFormat::BGRA8:
            break;
        default:
            return fail_node(ErrorCode::NotSupported,
                             "only 8-bit formats are supported, got format " +
                                 std::to_string(static_cast<int>(src.format)));
    }

    const uint32_t width    = src.width;
    const uint32_t height   = src.height;
    const uint32_t channels = src.channels ? src.channels : 1;
    const bool     inv      = invert;
    const uint8_t  on_value = inv ? 0 : 255;
    const uint8_t  off_value= inv ? 255 : 0;

    ImageData dst;
    dst.width    = width;
    dst.height   = height;
    dst.channels = channels;
    dst.format   = src.format;
    dst.data.resize(static_cast<size_t>(width) * height * channels, off_value);

    // ---- 主循环 ----
    // 一行一次 step_ok：被叫停 / 超预算都能在**下一行**生效。
    // 这是"501 个算子 0 个轮询 is_stopped()"的正解 —— 不改老算子，
    // 而是让新算子写对这件事的成本降到一行。
    bool finished = true;
    for (uint32_t y = 0; y < height; ++y) {
        if (!step_ok(context, y, height)) {
            finished = false;
            break;
        }
        for (uint32_t x = 0; x < width; ++x) {
            // 按**像素**推进而不是按字节：直接遍历 data.size() 会把同一像素的
            // RGB 三个通道拆成三次独立的判定，彩色图上结果是错的。
            const size_t base = (static_cast<size_t>(y) * width + x) * channels;
            for (uint32_t c = 0; c < channels; ++c) {
                const uint8_t v = src.data[base + c];
                dst.data[base + c] =
                    (static_cast<double>(v) >= threshold) ? on_value : off_value;
            }
        }
    }

    if (!finished) {
        // 走到这儿是因为 step_ok 说了"停"。**不返回 failure** ——
        // 被叫停是正常操作（阶段 2.3 已经把"取消"和"失败"分开），
        // 超预算那一支 step_ok 自己打过 WARN 了。
        // 但结果确实只有算过的行是准的，这一点要留个痕迹，不能假装算完了。
        warn_once("partial", "stopped before the last row; output is partially computed");
    }

    set_output("image", Data(std::move(dst)));
    return Result<void>::success();
}

// 注册。等价于 OVF_REGISTER_NODE_STRICT + 记进模块清单（require_user_nodes 靠它）。
//
// type_id 带 `nodes.` 前缀是有意的：撞车的时候一眼就能看出来是用户算子，
// 而且和上游那 501 个（都没有点）天然分得开。
OVF_REGISTER_USER_NODE(ThresholdNode, "nodes.Threshold", ThresholdNode::make_info())

} // namespace nodes
} // namespace ovf
