/**
 * @file test_algorithm_nodes.cpp
 * @brief 测试算法节点的基本功能
 */

#include <iostream>
#include <vector>
#include "ovf/core/node.h"
#include "ovf/core/types.h"
#include "ovf/core/flow.h"
#include "ovf/algorithm/simple_algo.h"
#include "ovf/algorithm/edge_detection.h"
#include "ovf/algorithm/morphology.h"
#include "ovf/algorithm/template_matching.h"
#include "ovf/algorithm/geometry.h"
#include "ovf/algorithm/blob_analysis.h"

using namespace ovf;
using namespace ovf::algorithm;

// 创建测试图像
ImageData create_test_image(int width, int height, int channels = 1) {
    ImageData img;
    img.width = width;
    img.height = height;
    img.channels = channels;
    img.data.resize(width * height * channels);

    // 创建带有矩形图案的测试图像
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t val = 50; // 背景

            // 绘制矩形
            if (x >= 100 && x < 200 && y >= 100 && y < 200) {
                val = 200; // 前景
            }

            if (channels == 1) {
                img.data[y * width + x] = val;
            } else {
                for (int c = 0; c < channels; ++c) {
                    img.data[(y * width + x) * channels + c] = val;
                }
            }
        }
    }

    return img;
}

// 打印节点信息
void print_node_info(const String& type_id) {
    auto info = NodeFactory::instance().get_node_info(type_id);
    if (info) {
        std::cout << "节点类型: " << info->type << std::endl;
        std::cout << "描述: " << info->description << std::endl;
        std::cout << "输入:" << std::endl;
        for (const auto& input : info->inputs) {
            std::cout << "  - " << input.name << " (" << input.type_name << ")" << std::endl;
        }
        std::cout << "输出:" << std::endl;
        for (const auto& output : info->outputs) {
            std::cout << "  - " << output.name << " (" << output.type_name << ")" << std::endl;
        }
        std::cout << "参数:" << std::endl;
        for (const auto& param : info->params) {
            std::cout << "  - " << param.name << " [" << param.type_name << "]" << std::endl;
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "===== OpenVisionFlow 算法节点测试 =====" << std::endl;
    std::cout << std::endl;

    // 打印所有注册的节点类型
    std::cout << "已注册节点类型:" << std::endl;
    auto node_types = NodeFactory::instance().get_registered_types();
    for (const auto& type : node_types) {
        std::cout << "  - " << type << std::endl;
    }
    std::cout << std::endl;

    // 测试图像源节点
    std::cout << "===== 测试 ImageSourceNode =====" << std::endl;
    print_node_info("ImageSource");

    auto image_source = NodeFactory::instance().create("ImageSource", "test_source");
    if (image_source) {
        nlohmann::json config;
        config["width"] = 320;
        config["height"] = 240;
        config["channels"] = 1;
        image_source->initialize(config);

        FlowContext context;
        InputArray inputs;
        OutputArray outputs;

        auto result = image_source->execute(context);
        if (result.is_success()) {
            std::cout << "执行成功!" << std::endl;
        } else {
            std::cout << "执行失败: " << result.message() << std::endl;
        }
    }
    std::cout << std::endl;

    // 测试阈值节点（使用SimpleThresholdNode）
    std::cout << "===== 测试 Threshold =====" << std::endl;
    print_node_info("Threshold");

    auto test_image = create_test_image(320, 240, 1);
    std::cout << "创建测试图像: " << test_image.width << "x" << test_image.height << std::endl;
    std::cout << std::endl;

    // 测试形态学节点
    std::cout << "===== 测试 MorphologyNode =====" << std::endl;
    print_node_info("Morphology");

    auto morph_node = NodeFactory::instance().create("Morphology", "test_morph");
    if (morph_node) {
        nlohmann::json config;
        config["operation"] = "Erode";
        config["kernel_shape"] = "Rect";
        config["kernel_size"] = 3;
        config["iterations"] = 1;
        morph_node->initialize(config);

        FlowContext context;
        morph_node->set_input("image", test_image);

        auto result = morph_node->execute(context);
        if (result.is_success()) {
            std::cout << "形态学处理成功!" << std::endl;
        }
    }
    std::cout << std::endl;

    // 测试几何变换节点
    std::cout << "===== 测试 RotateNode =====" << std::endl;
    print_node_info("Rotate");

    auto rotate_node = NodeFactory::instance().create("Rotate", "test_rotate");
    if (rotate_node) {
        nlohmann::json config;
        config["angle"] = 45.0;
        config["expand_canvas"] = true;
        rotate_node->initialize(config);

        FlowContext context;
        rotate_node->set_input("image", test_image);

        auto result = rotate_node->execute(context);
        if (result.is_success()) {
            std::cout << "图像旋转成功!" << std::endl;
        }
    }
    std::cout << std::endl;

    // 测试模板匹配节点
    std::cout << "===== 测试 TemplateMatchNode =====" << std::endl;
    print_node_info("TemplateMatch");

    auto template_match_node = NodeFactory::instance().create("TemplateMatch", "test_template");
    if (template_match_node) {
        // 从测试图像中提取模板
        ImageData tmpl;
        tmpl.width = 50;
        tmpl.height = 50;
        tmpl.channels = 1;
        tmpl.data.resize(50 * 50, 200);

        nlohmann::json config;
        config["method"] = "NCC";
        config["threshold"] = 0.7;
        template_match_node->initialize(config);

        // 需要先设置模板
        // template_match_node->set_template(tmpl);

        std::cout << "模板匹配节点已创建" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "===== 测试完成 =====" << std::endl;

    return 0;
}