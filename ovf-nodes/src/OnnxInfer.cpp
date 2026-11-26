/**
 * @file OnnxInfer.cpp
 * @brief ONNX 推理算子（onnxruntime，YOLO 系输出解码 + NMS 一体化）
 *
 * 动机：实习日常 = 天天 ONNX 推理（YOLO26m_3204 复判导出）。把这条链路
 * 平台化：模型路径/置信度/NMS 阈值全是参数，输入图像 → 输出检出数/最优框/
 * 带框可视化图，与 LightbarDetect/CropROI 串成完整链：
 *   OnnxInfer(粗定位) → CropROI(裁剪) → LightbarDetect(灯条精修)
 *
 * 解码约定（ultralytics YOLOv5/8/11/26 导出格式）：
 *   输出 (1, 4+nc, N) 或 (1, N, 4+nc)（自动判别），前 4 = cx,cy,w,h（输入坐标系），
 *   其后为各类置信度（v5 多一个 obj 位按 5 处理）。预处理 = 拉伸到 input_size + /255。
 */

#include "ovf/nodes/nodes.h"

#include "ort_mingw_compat.h"
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ovf {
namespace nodes {

namespace {
// Ort::Env 进程级单例（惰性）。
// 教训：作为节点成员在 factory.create（节点构造期）里直接构造 Ort::Env 会段错误
// （MinGW+静态库混合链接下，ORT 的 C++ 包装在构造期取 API 表的路径不稳）；
// 惰性到第一次 execute 再建，且全进程共享一个。
Ort::Env& SharedOrtEnv() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ovf.onnx");
    return env;
}
} // namespace

class OnnxInferNode : public INode {
public:
    OnnxInferNode(const String& instance_id) : INode(instance_id, make_info()) {}

    static NodeInfo make_info();
    Result<void> execute(FlowContext& context) override;

private:
    std::shared_ptr<Ort::Session> session_;   // 模型缓存：同节点实例只加载一次
    String loaded_path_;
};

NodeInfo OnnxInferNode::make_info() {
    NodeInfo info;
    info.id          = "nodes.OnnxInfer";
    info.name        = "ONNX 推理（YOLO 系解码+NMS）";
    info.category    = "深度学习";
    info.description = "onnxruntime CPU 推理。输入图像，输出 count/最优框/带框可视化。"
                       "支持 ultralytics YOLOv5/8/11/26 导出格式";
    info.version     = "0.1.0";
    info.author      = "ovf-nodes";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("count", "检出框数", DataType::Number));
    info.outputs.push_back(DataPort("region", "最优检出框", DataType::Region));
    info.outputs.push_back(DataPort("classes", "检出类别串(cls:conf,...)", DataType::String));
    info.outputs.push_back(DataPort("debug", "带框可视化图", DataType::Image));

    info.params.push_back(ParamDef("model_path", "模型路径(.onnx)", DataType::String,
                                   Data("model.onnx")));
    info.params.push_back(ParamDef("conf", "置信度阈值", DataType::Number, Data(0.35))
                              .range(0.0, 1.0));
    info.params.push_back(ParamDef("iou", "NMS IoU 阈值", DataType::Number, Data(0.45))
                              .range(0.0, 1.0));
    info.params.push_back(ParamDef("input_size", "输入边长(正方形)", DataType::Number,
                                   Data(640.0)).range(64, 2048));
    return info;
}

namespace {

// float32 -> float16（位运算标准转换，fp16 模型用）
inline uint16_t F32ToF16(float f) {
    uint32_t x; std::memcpy(&x, &f, 4);
    const uint32_t sign = (x >> 16) & 0x8000;
    int32_t exp = ((x >> 23) & 0xff) - 127 + 15;
    uint32_t mant = (x >> 13) & 0x3ff;
    if (exp <= 0) return (uint16_t)sign;            // 下溢出→0
    if (exp >= 31) return (uint16_t)(sign | 0x7c00); // 上溢出→inf
    return (uint16_t)(sign | (exp << 10) | mant);
}
inline float F16ToF32(uint16_t h) {
    uint32_t sign = (h & 0x8000) << 16;
    uint32_t exp = (h >> 10) & 0x1f;
    uint32_t mant = h & 0x3ff;
    uint32_t x;
    if (exp == 0) x = sign;                          // 非规格化→按0处理（够用）
    else x = sign | ((exp - 15 + 127) << 23) | (mant << 13);
    float f; std::memcpy(&f, &x, 4);
    return f;
}

struct DetBox {
    float x1, y1, x2, y2, conf;
    int cls;
};

float Iou(const DetBox& a, const DetBox& b) {
    const float ix = std::max(0.0f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    const float iy = std::max(0.0f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    const float inter = ix * iy;
    const float uni = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
    return uni > 0 ? inter / uni : 0;
}

} // namespace

Result<void> OnnxInferNode::execute(FlowContext& context) {
    fprintf(stderr, "[S0] enter execute\n");
    // ---- 输入与参数（安全带 API）----
    OVF_TRY_IN(image, in_image("image"));
    OVF_TRY_IN(path_r, p_str("model_path"));
    OVF_TRY_IN(conf_r, p_num_in("conf", 0.0, 1.0));
    OVF_TRY_IN(iou_r, p_num_in("iou", 0.0, 1.0));
    OVF_TRY_IN(size_r, p_num_in("input_size", 64.0, 2048.0));

    const String model_path = path_r;
    const float conf_th = static_cast<float>(conf_r);
    const float iou_th  = static_cast<float>(iou_r);
    const int S = static_cast<int>(size_r);

    const ImageData& src = image;
    if (src.width == 0 || src.height == 0 || src.data.empty()) {
        return fail_node(ErrorCode::InvalidImage, "input image is empty");
    }
    const int W = (int)src.width, H = (int)src.height;
    const int ch = src.channels ? (int)src.channels : 1;

    // ---- 会话缓存：同一模型路径只加载一次 ----
    fprintf(stderr, "[S1] params ok, model=%s\n", model_path.c_str());
    if (!session_ || loaded_path_ != model_path) {
        try {
            Ort::SessionOptions so;
            so.SetIntraOpNumThreads(1);   // MinGW 下 ORT 默认线程池崩，单线程稳
            std::wstring wpath(model_path.begin(), model_path.end());
            session_ = std::make_shared<Ort::Session>(SharedOrtEnv(), wpath.c_str(), so);
            loaded_path_ = model_path;
        } catch (const Ort::Exception& e) {
            session_.reset();
            return fail_node(ErrorCode::FileNotFound,
                             String("load onnx failed: ") + e.what());
        }
    }

    // ---- 预处理：拉伸到 SxS，/255，CHW float ----
    const int in_ch = 3;
    std::vector<float> tensor((size_t)in_ch * S * S);
    for (int y = 0; y < S; ++y) {
        if (!step_ok(context, y, S)) {
            warn_once("partial", "stopped during preprocess; output incomplete");
            set_output("count", Data(0.0));
            return Result<void>::success();
        }
        const int sy = (int)((int64_t)y * H / S);
        for (int x = 0; x < S; ++x) {
            const int sx = (int)((int64_t)x * W / S);
            const size_t base = ((size_t)sy * W + sx) * ch;
            // 灰度输入复制 3 份；彩色取前 3 通道
            const float r = src.data[base] / 255.0f;
            const float g = ch >= 3 ? src.data[base + 1] / 255.0f : r;
            const float b = ch >= 3 ? src.data[base + 2] / 255.0f : r;
            tensor[(size_t)0 * S * S + (size_t)y * S + x] = r;
            tensor[(size_t)1 * S * S + (size_t)y * S + x] = g;
            tensor[(size_t)2 * S * S + (size_t)y * S + x] = b;
        }
    }

    // ---- 推理 ----
    fprintf(stderr, "[S2] preprocess done, session=%d\n", (int)(session_ != nullptr));
    fprintf(stderr, "[P1] before names\n");
    // 为避免悬挂，这里重新获取并保持在 run 之前
    Ort::AllocatorWithDefaultOptions alloc;
    auto in_name  = session_->GetInputNameAllocated(0, alloc);
    auto out_name = session_->GetOutputNameAllocated(0, alloc);
    const char* in_names2[1]  = { in_name.get() };
    const char* out_names2[1] = { out_name.get() };

    const int64_t dims[5] = {1, in_ch, S, S, 0};
    const int64_t* dims_ptr = dims;
    Ort::MemoryInfo mi = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    // fp16 模型（如深大 RP24 的 0526.onnx）：转半精度再喂
    const auto in_type = session_->GetInputTypeInfo(0)
                             .GetTensorTypeAndShapeInfo().GetElementType();
    std::vector<Ort::Float16_t> tensor_h;
    Ort::Value input = Ort::Value::CreateTensor<float>(
        mi, tensor.data(), tensor.size(), dims_ptr, 4);
    if (in_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        tensor_h.resize(tensor.size());
        for (size_t i = 0; i < tensor.size(); ++i) tensor_h[i].val = F32ToF16(tensor[i]);
        input = Ort::Value::CreateTensor<Ort::Float16_t>(
            mi, tensor_h.data(), tensor_h.size(), dims_ptr, 4);
    }

    std::vector<Ort::Value> outputs;
    try {
        fprintf(stderr, "[P2] before run\n");
        outputs = session_->Run(Ort::RunOptions{nullptr}, in_names2, &input, 1,
                                out_names2, 1);
        fprintf(stderr, "[P3] after run\n");
    } catch (const Ort::Exception& e) {
        return fail_node(ErrorCode::Unknown,
                         String("onnx run failed: ") + e.what());
    }
    if (outputs.empty() || !outputs[0].IsTensor()) {
        return fail_node(ErrorCode::Unknown, "model has no tensor output");
    }

    // ---- 解码：自动判别 (1,C,N) / (1,N,C)，C = 4+nc（v5 则 5+nc）----
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (shape.size() < 2) {
        return fail_node(ErrorCode::NotSupported, "unexpected output rank");
    }
    const int64_t d1 = shape[shape.size() - 2];
    const int64_t d2 = shape[shape.size() - 1];
    const bool channels_first = (d1 < d2);          // (1, C, N)
    const int64_t C = channels_first ? d1 : d2;
    const int64_t N = channels_first ? d2 : d1;
    const int num_cls = static_cast<int>(C) - 4;
    if (num_cls < 1) {
        return fail_node(ErrorCode::NotSupported,
                         "output has no class dims (C=" + std::to_string(C) + ")");
    }
    std::vector<float> out_f16buf;
    const float* out = nullptr;
    const auto out_type = outputs[0].GetTensorTypeAndShapeInfo().GetElementType();
    if (out_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        const Ort::Float16_t* h = outputs[0].GetTensorData<Ort::Float16_t>();
        const size_t total = (size_t)(C * N);
        out_f16buf.resize(total);
        for (size_t i = 0; i < total; ++i) out_f16buf[i] = F16ToF32(h[i].val);
        out = out_f16buf.data();
    } else {
        out = outputs[0].GetTensorData<float>();
    }
    fprintf(stderr, "[ORT] shape=%lldx%lld ch_first=%d C=%lld N=%lld v0..5=%.3f %.3f %.3f %.3f %.3f %.3f\n",
            (long long)d1, (long long)d2, (int)channels_first, (long long)C, (long long)N,
            out[0], out[1], out[2], out[3], out[4], out[5 < C*N ? 5 : 0]);

    std::vector<DetBox> boxes;
    for (int64_t n = 0; n < N; ++n) {
        // 找最大类置信度（v5 的 obj 位在第 4 列，乘上去近似）
        int best_c = 0;
        float best_p = -1.0f;
        for (int c = 0; c < num_cls; ++c) {
            const int64_t idx = channels_first
                ? (int64_t)(4 + c) * N + n
                : n * C + 4 + c;
            if (out[idx] > best_p) { best_p = out[idx]; best_c = c; }
        }
        float score = best_p;
        if (C == (int64_t)(5 + num_cls)) {   // v5：乘 obj
            const int64_t obj_idx = channels_first ? (int64_t)4 * N + n : n * C + 4;
            score *= out[obj_idx];
        }
        if (score < conf_th) continue;

        const int64_t cx_i = channels_first ? n : n * C;
        const float cx = out[channels_first ? cx_i : cx_i + 0];
        const float cy = out[channels_first ? N + n : cx_i + 1];
        const float w  = out[channels_first ? 2 * (int64_t)N + n : cx_i + 2];
        const float h  = out[channels_first ? 3 * (int64_t)N + n : cx_i + 3];
        DetBox b{ (cx - w / 2) * W / S, (cy - h / 2) * H / S,
                  (cx + w / 2) * W / S, (cy + h / 2) * H / S, score, best_c };
        if (b.x2 <= b.x1 || b.y2 <= b.y1) continue;
        boxes.push_back(b);
    }

    // ---- NMS ----
    std::sort(boxes.begin(), boxes.end(),
              [](const DetBox& a, const DetBox& b) { return a.conf > b.conf; });
    std::vector<char> dropped(boxes.size(), 0);
    std::vector<DetBox> keep;
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (dropped[i]) continue;
        keep.push_back(boxes[i]);
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (!dropped[j] && boxes[j].cls == boxes[i].cls &&
                Iou(boxes[i], boxes[j]) > iou_th)
                dropped[j] = 1;
        }
    }

    // ---- 输出 ----
    set_output("count", Data(static_cast<double>(keep.size())));

    String cls_str;
    for (size_t i = 0; i < keep.size(); ++i) {
        if (i) cls_str += ",";
        cls_str += std::to_string(keep[i].cls) + ":" +
                   std::to_string(keep[i].conf).substr(0, 4);
    }
    set_output("classes", Data(cls_str));

    Region best;
    if (!keep.empty()) {
        const DetBox& k = keep.front();
        best = Region((int32_t)k.x1, (int32_t)k.y1,
                      (int32_t)(k.x2 - k.x1), (int32_t)(k.y2 - k.y1));
    }
    set_output("region", Data(best));

    // debug：白框画到灰度展开图上（8 像素十字角标风格，够看）
    ImageData dbg;
    dbg.width = src.width; dbg.height = src.height;
    dbg.channels = 1; dbg.format = ImageFormat::Mono8;
    dbg.data.assign((size_t)W * H, 0);
    for (size_t i = 0; i < (size_t)W * H; ++i) {
        unsigned v = 0;
        for (int c = 0; c < ch && c < 3; ++c) v += src.data[i * ch + c];
        dbg.data[i] = (unsigned char)(ch >= 3 ? v / 3 : v);
    }
    for (const auto& k : keep) {
        for (int x = std::max(0, (int)k.x1); x <= std::min(W - 1, (int)k.x2); ++x) {
            dbg.data[(size_t)std::max(0, (int)k.y1) * W + x] = 255;
            dbg.data[(size_t)std::min(H - 1, (int)k.y2) * W + x] = 255;
        }
        for (int y = std::max(0, (int)k.y1); y <= std::min(H - 1, (int)k.y2); ++y) {
            dbg.data[(size_t)y * W + std::max(0, (int)k.x1)] = 255;
            dbg.data[(size_t)y * W + std::min(W - 1, (int)k.x2)] = 255;
        }
    }
    set_output("debug", Data(std::move(dbg)));
    return Result<void>::success();
}

OVF_REGISTER_USER_NODE(OnnxInferNode, "nodes.OnnxInfer", OnnxInferNode::make_info())

} // namespace nodes
} // namespace ovf
