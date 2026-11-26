/**
 * @file CropROI.cpp
 * @brief ROI 裁剪算子 —— 真实链路里"先定位(OnnxInfer)→裁剪→精修(LightbarDetect)"的中间环节
 *
 * 支持两种给框方式：参数直接给 x/y/w/h，或接 region 输入（上游检出框，
 * 可用 expand 参数外扩像素）。输出裁剪后的子图。
 */

#include "ovf/nodes/nodes.h"

#include <algorithm>
#include <utility>

namespace ovf {
namespace nodes {

class CropROINode : public INode {
public:
    CropROINode(const String& instance_id) : INode(instance_id, make_info()) {}
    static NodeInfo make_info();
    Result<void> execute(FlowContext& context) override;
};

NodeInfo CropROINode::make_info() {
    NodeInfo info;
    info.id          = "nodes.CropROI";
    info.name        = "ROI 裁剪";
    info.category    = "图像处理";
    info.description = "按参数 x/y/w/h 或上游 region 裁剪子图（expand 可外扩）";
    info.version     = "0.1.0";
    info.author      = "ovf-nodes";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("region", "裁剪框(可选,优先于参数)", DataType::Region));
    info.outputs.push_back(DataPort("image", "裁剪结果", DataType::Image));

    info.params.push_back(ParamDef("x", "左上 x", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("y", "左上 y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("w", "宽", DataType::Number, Data(0.0)).range(0, 65535));
    info.params.push_back(ParamDef("h", "高", DataType::Number, Data(0.0)).range(0, 65535));
    info.params.push_back(ParamDef("expand", "外扩像素", DataType::Number, Data(0.0)).range(0, 1024));
    return info;
}

Result<void> CropROINode::execute(FlowContext& context) {
    OVF_TRY_IN(image, in_image("image"));
    OVF_TRY_IN(px, p_num_in("x", -1e9, 1e9));
    OVF_TRY_IN(py, p_num_in("y", -1e9, 1e9));
    OVF_TRY_IN(pw, p_num_in("w", 0.0, 65535.0));
    OVF_TRY_IN(ph, p_num_in("h", 0.0, 65535.0));
    OVF_TRY_IN(expand, p_num_in("expand", 0.0, 1024.0));

    const ImageData& src = image;
    if (src.width == 0 || src.height == 0 || src.data.empty()) {
        return fail_node(ErrorCode::InvalidImage, "input image is empty");
    }
    const int W = (int)src.width, H = (int)src.height;
    const int ch = src.channels ? (int)src.channels : 1;

    // region 输入优先（有有效框时）
    int rx = (int)px, ry = (int)py, rw = (int)pw, rh = (int)ph;
    const Data rd = get_input("region");
    if (rd.is_region()) {
        const Region r = rd.as_region();
        if (r.width > 0 && r.height > 0) {
            rx = r.x; ry = r.y; rw = r.width; rh = r.height;
        }
    }
    const int e = (int)expand;
    rx -= e; ry -= e; rw += 2 * e; rh += 2 * e;

    // 夹到图内；越界即空结果（显式失败比静默空图好）
    rx = std::max(0, rx); ry = std::max(0, ry);
    rw = std::min(rw, W - rx); rh = std::min(rh, H - ry);
    if (rw <= 0 || rh <= 0) {
        return fail_node(ErrorCode::InvalidParameter,
                         "ROI outside image or empty after clamp");
    }

    ImageData dst;
    dst.width = (uint32_t)rw; dst.height = (uint32_t)rh;
    dst.channels = src.channels; dst.format = src.format;
    dst.data.resize((size_t)rw * rh * ch);
    for (int y = 0; y < rh; ++y) {
        if (!step_ok(context, y, rh)) {
            warn_once("partial", "stopped during crop; output incomplete");
            break;
        }
        std::copy_n(src.data.begin() + ((size_t)(ry + y) * W + rx) * ch,
                    (size_t)rw * ch, dst.data.begin() + (size_t)y * rw * ch);
    }
    set_output("image", Data(std::move(dst)));
    return Result<void>::success();
}

OVF_REGISTER_USER_NODE(CropROINode, "nodes.CropROI", CropROINode::make_info())

} // namespace nodes
} // namespace ovf
