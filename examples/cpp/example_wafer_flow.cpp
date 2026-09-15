/**
 * @file example_wafer_flow.cpp
 * @brief OVF 可用路径示范：在 C++ 里手搓 FlowDef 并执行完整流程
 *
 * 为什么不用 JSON？—— FlowEngine::load_from_file() 里解析 params 和 inputs 的
 * 代码块是空的（只有注释"这里暂时跳过，后续完善"），所以 JSON 加载出来的流程
 * 既没有参数也没有连线；且 examples/flows/*.json 用的 type_id（ImageSource/
 * Blur/Threshold）根本没注册。JSON 那条路目前是死的。
 *
 * 编译：
 *   g++ -std=c++17 -I ovf-core/include -I ovf-algorithm/include \
 *       -I ovf-core/thirdparty examples/cpp/example_wafer_flow.cpp \
 *       -L build/lib -lovf-algorithm -lovf-core -o example_wafer_flow
 * 运行：
 *   LD_LIBRARY_PATH=build/lib ./example_wafer_flow
 */

#include "ovf/core/flow.h"
#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/algorithm.h"   // initialize_algorithm_module()

#include <cstdio>
#include <iostream>

using namespace ovf;

// 小工具：加一个节点
static FlowDef::NodeInstance make_node(const String& id,
                                       const String& type_id,
                                       const String& name) {
    FlowDef::NodeInstance n;
    n.id = id;
    n.type_id = type_id;
    n.name = name;
    n.enabled = true;
    return n;
}

// 小工具：把 upstream 的某个输出口连到 downstream 的某个输入口
static void connect(FlowDef::NodeInstance& downstream,
                    const String& input_port,
                    const String& upstream_id,
                    const String& output_port) {
    FlowDef::NodeInstance::InputConnection c;
    c.source_node_id = upstream_id;
    c.source_port = output_port;
    downstream.input_connections[input_port] = c;
}

int main() {
    // 必须调用：算法节点靠 libovf-algorithm.so 的静态初始化注册。
    // 若本程序不引用该库的任何符号，链接器 --as-needed 会整库丢弃，
    // 静态初始化不执行 → 一个节点都注册不上（报 "Node type not found"）。
    ovf::algorithm::initialize_algorithm_module();

    FlowEngine engine;

    FlowDef flow;
    flow.id = "wafer_preprocess_demo";
    flow.name = "晶圆图像预处理演示";
    flow.version = "1.0";
    flow.author = "example";

    // ── 1. 图像源：生成测试图 ────────────────────────────────
    auto src = make_node("src", "SimpleImageSource", "图像源");
    src.params.set("width", Data(256));
    src.params.set("height", Data(256));

    // ── 2. 高斯滤波 ─────────────────────────────────────────
    auto blur = make_node("blur", "GaussianFilter", "高斯滤波");
    blur.params.set("kernel_size", Data(5));
    blur.params.set("sigma", Data(1.5));
    connect(blur, "image", "src", "image");

    // ── 3. 阈值化 ───────────────────────────────────────────
    auto thresh = make_node("thresh", "SimpleThreshold", "阈值化");
    thresh.params.set("threshold", Data(128));
    thresh.params.set("max_value", Data(255));
    connect(thresh, "image", "blur", "image");

    // ── 4. Blob 分析 ────────────────────────────────────────
    auto blob = make_node("blob", "BlobAnalysis", "Blob分析");
    blob.params.set("min_area", Data(10));
    blob.params.set("max_area", Data(100000));
    blob.params.set("draw_boxes", Data(true));
    connect(blob, "image", "thresh", "image");

    // ── 5. 保存结果 ─────────────────────────────────────────
    auto save = make_node("save", "ImageSave", "保存图像");
    save.params.set("filepath", Data("/tmp/ovf_wafer_out.raw"));
    save.params.set("format", Data("raw"));
    connect(save, "image", "blob", "image");

    flow.nodes.push_back(src);
    flow.nodes.push_back(blur);
    flow.nodes.push_back(thresh);
    flow.nodes.push_back(blob);
    flow.nodes.push_back(save);

    // ── 加载（这一步才会真正实例化节点、设参数、接线）────────
    auto load_res = engine.load_flow(flow);
    if (!load_res.is_success()) {
        std::printf("load_flow 失败: %s\n", load_res.message().c_str());
        return 1;
    }
    std::printf("load_flow 成功，节点数 = %zu\n", engine.get_all_nodes().size());

    // ── 查看引擎自己算出的执行顺序（由连线拓扑决定）──────────
    auto order = engine.get_execution_order();
    if (order.is_success()) {
        std::printf("拓扑执行顺序: ");
        for (const auto& id : order.value()) std::printf("%s ", id.c_str());
        std::printf("\n");
    }

    // ── 跑 ──────────────────────────────────────────────────
    FlowContext context;
    FlowResult result = engine.run(context);

    std::printf("\n执行结果: %s\n", result.success ? "成功" : "失败");
    if (!result.success && !result.error_message.empty()) {
        std::printf("错误信息: %s\n", result.error_message.c_str());
    }
    std::printf("总耗时: %.3f ms\n", result.total_time_us / 1000.0);

    std::printf("\n各节点耗时:\n");
    for (const auto& kv : result.node_times) {
        std::printf("  %-8s %8.3f ms\n", kv.first.c_str(), kv.second / 1000.0);
    }

    // ── 读回中间结果：Blob 数量 ─────────────────────────────
    auto blob_node = engine.get_node("blob");
    if (blob_node) {
        Data cnt = blob_node->get_output("blob_count");
        if (cnt.is_valid()) {
            std::printf("\nBlob 数量 = %d\n", cnt.as_int());
        }
    }

    return result.success ? 0 : 1;
}
