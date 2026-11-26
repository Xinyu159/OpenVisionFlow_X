/**
 * @file LightbarDetect.cpp
 * @brief 灯条/条状特征检测 —— 从 rm_vision 装甲板灯条检测抽象而来
 *
 * 来源：RoboMaster 视觉里的灯条检测链路（阈值 → 连通域 → 几何筛选 → 旋转矩形）。
 * 平台化后去掉 RM 语境，保留通用能力：在灰度图上找"细长的亮（或暗）条"，
 * 输出最优检出（旋转矩形 Region）+ 检出数 + 可视化调试图。
 *
 * 纯 C++ 实现（无 OpenCV）：连通域用 BFS，主轴角用二阶中心矩（PCA 等价），
 * 旋转包围盒用主轴系投影。每个环节自己写一遍，是"算子变成自己的东西"的起点。
 *
 * 几何筛选参数沿用 RM 场景的物理含义：
 *   - 长宽比 range：灯条高/宽 ≈ 2.5~10，平台化后可调
 *   - 面积 range：过滤噪点与大面积反光
 */

#include "ovf/nodes/nodes.h"

#include <cmath>
#include <cstdint>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <deque>
#include <utility>
#include <vector>

namespace ovf {
namespace nodes {

class LightbarDetectNode : public INode {
public:
    LightbarDetectNode(const String& instance_id) : INode(instance_id, make_info()) {}

    static NodeInfo make_info();
    Result<void> execute(FlowContext& context) override;
};

NodeInfo LightbarDetectNode::make_info() {
    NodeInfo info;
    info.id          = "nodes.LightbarDetect";
    info.name        = "灯条/条状特征检测";
    info.category    = "检测/定位";
    info.description = "阈值→8连通域→几何筛选（面积/长宽比）→旋转矩形。"
                       "抽象自 RoboMaster 装甲板灯条检测，适用于细长亮/暗条状特征定位";
    info.version     = "0.1.0";
    info.author      = "ovf-nodes";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("region", "最强检出（旋转矩形）", DataType::Region));
    info.outputs.push_back(DataPort("count", "检出条数", DataType::Number));
    info.outputs.push_back(DataPort("debug", "检出可视化（前景像素展开）", DataType::Image));

    info.params.push_back(
        ParamDef("threshold", "二值化阈值", DataType::Number, Data(180)).range(0, 255));
    info.params.push_back(
        ParamDef("invert", "检测暗条（反相）", DataType::Boolean, Data(false)));
    info.params.push_back(
        ParamDef("min_area", "最小面积(px)", DataType::Number, Data(8)).range(1, 1e6));
    info.params.push_back(
        ParamDef("max_area", "最大面积(px)", DataType::Number, Data(100000)).range(1, 1e8));
    info.params.push_back(
        ParamDef("min_ratio", "最小长宽比", DataType::Number, Data(2.0)).range(1.0, 100.0));
    info.params.push_back(
        ParamDef("max_ratio", "最大长宽比", DataType::Number, Data(12.0)).range(1.0, 100.0));
    return info;
}

namespace {

// 单个连通域的几何描述
struct Blob {
    size_t area = 0;
    double cx = 0.0, cy = 0.0;      // 质心
    double angle = 0.0;             // 主轴角（弧度，x 轴起逆时针）
    double len_a = 0.0, len_b = 0.0; // 主轴系包围盒边长（a=长边）
    std::vector<std::pair<int, int>> pixels;  // 供 debug 图回放
};

// 主轴角：图像二阶中心矩。等价于 PCA 第一主成分方向。
double principal_angle(const Blob& b, double mu_xx, double mu_yy, double mu_xy) {
    (void)b;
    return 0.5 * std::atan2(2.0 * mu_xy, mu_xx - mu_yy);
}

} // namespace

Result<void> LightbarDetectNode::execute(FlowContext& context) {
    // ---- 输入与参数（安全带 API）----
    OVF_TRY_IN(image, in_image("image"));
    OVF_TRY_IN(threshold, p_num_in("threshold", 0.0, 255.0));
    OVF_TRY_IN(invert, p_bool("invert"));
    OVF_TRY_IN(min_area, p_num_in("min_area", 1.0, 1e6));
    OVF_TRY_IN(max_area, p_num_in("max_area", 1.0, 1e8));
    OVF_TRY_IN(min_ratio, p_num_in("min_ratio", 1.0, 100.0));
    OVF_TRY_IN(max_ratio, p_num_in("max_ratio", 1.0, 100.0));

    const ImageData& src = image;
    if (src.width == 0 || src.height == 0 || src.data.empty()) {
        return fail_node(ErrorCode::InvalidImage, "input image is empty");
    }
    if (src.format != ImageFormat::Mono8 && src.format != ImageFormat::RGB8 &&
        src.format != ImageFormat::RGBA8 && src.format != ImageFormat::BGR8 &&
        src.format != ImageFormat::BGRA8) {
        return fail_node(ErrorCode::NotSupported,
                         "only 8-bit formats are supported, got format " +
                             std::to_string(static_cast<int>(src.format)));
    }

    const int W = static_cast<int>(src.width);
    const int H = static_cast<int>(src.height);
    const uint32_t ch = src.channels ? src.channels : 1;

    // ---- 1. 灰度 + 阈值 ----
    std::vector<uint8_t> fg(static_cast<size_t>(W) * H, 0);
    for (int y = 0; y < H; ++y) {
        if (!step_ok(context, y, H)) {
            warn_once("partial", "stopped during thresholding; result incomplete");
            set_output("count", Data(0));
            return Result<void>::success();
        }
        for (int x = 0; x < W; ++x) {
            const size_t base = (static_cast<size_t>(y) * W + x) * ch;
            uint32_t v = 0;
            for (uint32_t c = 0; c < ch; ++c) v += src.data[base + c];
            const double gray = static_cast<double>(v) / ch;
            const bool on = invert ? (gray < threshold) : (gray >= threshold);
            fg[static_cast<size_t>(y) * W + x] = on ? 1 : 0;
        }
    }

    // ---- 2. 8 连通域（BFS）----
    std::vector<uint8_t> visited(static_cast<size_t>(W) * H, 0);
    std::vector<Blob> blobs;
    static const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int y0 = 0; y0 < H; ++y0) {
        for (int x0 = 0; x0 < W; ++x0) {
            const size_t idx0 = static_cast<size_t>(y0) * W + x0;
            if (!fg[idx0] || visited[idx0]) continue;

            Blob b;
            std::deque<std::pair<int, int>> q;
            q.emplace_back(x0, y0);
            visited[idx0] = 1;
            double sx = 0, sy = 0;                 // 累计和（质心用）
            while (!q.empty()) {
                auto [x, y] = q.front();
                q.pop_front();
                b.pixels.emplace_back(x, y);
                ++b.area;
                sx += x;
                sy += y;
                for (int k = 0; k < 8; ++k) {
                    const int nx = x + dx8[k], ny = y + dy8[k];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    const size_t ni = static_cast<size_t>(ny) * W + nx;
                    if (fg[ni] && !visited[ni]) {
                        visited[ni] = 1;
                        q.emplace_back(nx, ny);
                    }
                }
            }
            b.cx = sx / b.area;
            b.cy = sy / b.area;
            blobs.push_back(std::move(b));
        }
    }

    // ---- 3. 几何筛选：面积 + 主轴 + 长宽比 ----
    std::vector<Blob> kept;
    for (auto& b : blobs) {
        if (b.area < min_area || b.area > max_area) continue;

        // 二阶中心矩 → 主轴角
        double mu_xx = 0, mu_yy = 0, mu_xy = 0;
        for (const auto& [x, y] : b.pixels) {
            const double dx = x - b.cx, dy = y - b.cy;
            mu_xx += dx * dx;
            mu_yy += dy * dy;
            mu_xy += dx * dy;
        }
        b.angle = principal_angle(b, mu_xx, mu_yy, mu_xy);

        // 主轴系投影包围盒
        const double ca = std::cos(b.angle), sa = std::sin(b.angle);
        double min_u = 1e18, max_u = -1e18, min_v = 1e18, max_v = -1e18;
        for (const auto& [x, y] : b.pixels) {
            const double dx = x - b.cx, dy = y - b.cy;
            const double u =  dx * ca + dy * sa;   // 主轴分量
            const double v = -dx * sa + dy * ca;   // 副轴分量
            min_u = std::min(min_u, u); max_u = std::max(max_u, u);
            min_v = std::min(min_v, v); max_v = std::max(max_v, v);
        }
        const double len1 = max_u - min_u + 1.0;
        const double len2 = max_v - min_v + 1.0;
        b.len_a = std::max(len1, len2);
        b.len_b = std::min(len1, len2);
        // 主轴是"长边方向"时角度才对；若长边其实是副轴，转 90°
        if (len2 > len1) b.angle += M_PI / 2.0;

        const double ratio = b.len_a / std::max(b.len_b, 1.0);
        if (ratio < min_ratio || ratio > max_ratio) continue;
        kept.push_back(std::move(b));
    }

    // ---- 4. 输出 ----
    set_output("count", Data(static_cast<double>(kept.size())));

    if (!kept.empty()) {
        // 最强检出 = 面积最大者。Region 用外接正矩形（中心 + 主轴宽高），
        // 旋转矩形语义由 angle 字段承载（引擎 Region 本就带 angle）。
        size_t best = 0;
        for (size_t i = 1; i < kept.size(); ++i) {
            if (kept[i].area > kept[best].area) best = i;
        }
        const Blob& k = kept[best];
        Region r(static_cast<int32_t>(k.cx - k.len_b / 2.0),
                 static_cast<int32_t>(k.cy - k.len_a / 2.0),
                 static_cast<int32_t>(std::ceil(k.len_b)),
                 static_cast<int32_t>(std::ceil(k.len_a)),
                 static_cast<float>(k.angle));
        set_output("region", Data(r));
    } else {
        set_output("region", Data(Region()));  // 空 Region（w=h=0）= 无检出
    }

    // debug 图：灰度展开 + 前景像素标白（Mono8，轻量）
    ImageData dbg;
    dbg.width = src.width;
    dbg.height = src.height;
    dbg.channels = 1;
    dbg.format = ImageFormat::Mono8;
    dbg.data.assign(static_cast<size_t>(W) * H, 0);
    for (const auto& b : kept) {
        for (const auto& [x, y] : b.pixels) {
            dbg.data[static_cast<size_t>(y) * W + x] = 255;
        }
    }
    set_output("debug", Data(std::move(dbg)));
    return Result<void>::success();
}

OVF_REGISTER_USER_NODE(LightbarDetectNode, "nodes.LightbarDetect",
                       LightbarDetectNode::make_info())

} // namespace nodes
} // namespace ovf
