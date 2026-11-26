/**
 * @file run_cli.cpp
 * @brief ovf-run — 无头跑算子链（数据集批量验证用，不开窗口）
 *
 * 用法: ovf-run <image> [operator_id]
 * 默认 operator_id = nodes.LightbarDetect。输出：count / region + debug.bmp
 */
#include "ovf/core/node.h"
#include "ovf/nodes/nodes.h"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_STATIC
#include "stb_image.h"
#include "stb_image_write.h"

#include <cstdio>
#include <string>
#define TRACE(...) do{ fprintf(stderr,"[T] " __VA_ARGS__); fflush(stderr);}while(0)

using namespace ovf;

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: ovf-run <image> [operator_id]\n"); return 2; }
    TRACE("start %s\n", argv[1]);
    const char* op = argc >= 3 ? argv[2] : "nodes.LightbarDetect";

    int w = 0, h = 0, n = 0;
    unsigned char* px = stbi_load(argv[1], &w, &h, &n, 0);
    if (!px) { std::printf("[ERR] cannot load %s\n", argv[1]); return 1; }

    ImageData img;
    img.width = (uint32_t)w; img.height = (uint32_t)h;
    img.channels = 1; img.format = ImageFormat::Mono8;
    img.data.assign((size_t)w * h, 0);
    const int ch = n > 0 ? n : 1;
    for (size_t i = 0; i < (size_t)w * h; ++i) {
        unsigned v = 0;
        for (int c = 0; c < ch && c < 3; ++c) v += px[i * ch + c];
        img.data[i] = (unsigned char)(ch >= 3 ? v / 3 : v);
    }
    TRACE("decoded %dx%dx%d\n", w, h, n);
    stbi_image_free(px);

    try { ovf::nodes::initialize_user_nodes(); }
    catch (const Exception& e) { std::printf("[ERR] %s\n", e.what()); return 1; }

    auto& factory = NodeFactory::instance();
    if (!factory.has_type(op)) { std::printf("[ERR] unknown op %s\n", op); return 1; }
    auto node = factory.create(op, "cli");
    if (!node) { std::printf("[ERR] create failed\n"); return 1; }

    // 附加参数：argv[i]="key=value"（数值自动判别，否则按字符串）
    for (int i = 3; i < argc; ++i) {
        std::string kv(argv[i]);
        auto eq = kv.find('=');
        if (eq == std::string::npos) continue;
        const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
        char* end = nullptr;
        const double d = strtod(v.c_str(), &end);
        if (end && *end == '\0' && !v.empty()) node->set_param(k, Data(d));
        else node->set_param(k, Data(String(v)));
    }

    for (const auto& in : node->info().inputs)
        if (in.data_type == DataType::Image) { node->set_input(in.id, Data(img)); break; }

    TRACE("executing op %s\n", op);
    FlowContext ctx;
    auto r = node->execute(ctx);
    TRACE("exec done ok=%d\n", (int)r.is_success());
    if (!r.is_success()) { std::printf("[FAIL] %s\n", r.message().c_str()); return 1; }

    TRACE("A: get count\n");
    const double count = node->get_output("count").as_number(-1);
    TRACE("B: count=%f get region\n", count);
    const Region rg = node->get_output("region").as_region();
    TRACE("C: region got\n");
    std::printf("%s\tcount=%.0f\tregion=(%d,%d,%dx%d,%.1frad)\n",
                argv[1], count, rg.x, rg.y, rg.width, rg.height, rg.angle);

    // 按值拷贝：get_output 返回临时 Data，as_image() 的引用经函数返回值
    // 绑定不延长临时生命周期 —— 悬垂引用 use-after-free（大图必炸的根因）
    const ImageData dbg = node->get_output("debug").as_image();
    if (!dbg.data.empty() && dbg.width) {
        stbi_write_bmp("debug.bmp", (int)dbg.width, (int)dbg.height, 1, dbg.data.data());
    TRACE("debug.bmp written\n");
    }
    return 0;
}
