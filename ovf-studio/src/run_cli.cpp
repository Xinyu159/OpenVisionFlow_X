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

using namespace ovf;

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: ovf-run <image> [operator_id]\n"); return 2; }
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
    stbi_image_free(px);

    try { ovf::nodes::initialize_user_nodes(); }
    catch (const Exception& e) { std::printf("[ERR] %s\n", e.what()); return 1; }

    auto& factory = NodeFactory::instance();
    if (!factory.has_type(op)) { std::printf("[ERR] unknown op %s\n", op); return 1; }
    auto node = factory.create(op, "cli");
    if (!node) { std::printf("[ERR] create failed\n"); return 1; }

    for (const auto& in : node->info().inputs)
        if (in.data_type == DataType::Image) { node->set_input(in.id, Data(img)); break; }

    FlowContext ctx;
    auto r = node->execute(ctx);
    if (!r.is_success()) { std::printf("[FAIL] %s\n", r.message().c_str()); return 1; }

    const double count = node->get_output("count").as_number(-1);
    const Region rg = node->get_output("region").as_region();
    std::printf("%s\tcount=%.0f\tregion=(%d,%d,%dx%d,%.1frad)\n",
                argv[1], count, rg.x, rg.y, rg.width, rg.height, rg.angle);

    const ImageData& dbg = node->get_output("debug").as_image();
    if (!dbg.data.empty() && dbg.width) {
        stbi_write_bmp("debug.bmp", (int)dbg.width, (int)dbg.height, 1, dbg.data.data());
    }
    return 0;
}
