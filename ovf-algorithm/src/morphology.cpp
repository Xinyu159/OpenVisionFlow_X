#include "ovf/algorithm/morphology.h"
#include "ovf/core/logger.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace ovf {
namespace algorithm {

namespace morph_utils {

ErrorCode create_kernel(KernelShape shape, int ksize, std::vector<uint8_t>& kernel) {
    if (ksize < 1 || ksize % 2 == 0) {
        return ErrorCode::InvalidParameter;
    }

    int size = ksize * ksize;
    kernel.resize(size, 0);

    int cx = ksize / 2;
    int cy = ksize / 2;

    switch (shape) {
        case KernelShape::Rect:
            // 矩形核：所有元素为1
            std::fill(kernel.begin(), kernel.end(), 255);
            break;

        case KernelShape::Ellipse:
            // 椭圆核：根据椭圆方程填充
            for (int y = 0; y < ksize; ++y) {
                for (int x = 0; x < ksize; ++x) {
                    double dx = x - cx;
                    double dy = y - cy;
                    double rx = ksize / 2.0;
                    double ry = ksize / 2.0;
                    if ((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry) <= 1.0) {
                        kernel[y * ksize + x] = 255;
                    }
                }
            }
            break;

        case KernelShape::Cross:
            // 十字核：只有中心和十字线
            for (int y = 0; y < ksize; ++y) {
                for (int x = 0; x < ksize; ++x) {
                    if (x == cx || y == cy) {
                        kernel[y * ksize + x] = 255;
                    }
                }
            }
            break;

        case KernelShape::Diamond:
            // 菱形核
            for (int y = 0; y < ksize; ++y) {
                for (int x = 0; x < ksize; ++x) {
                    int dx = std::abs(x - cx);
                    int dy = std::abs(y - cy);
                    if (dx + dy <= ksize / 2) {
                        kernel[y * ksize + x] = 255;
                    }
                }
            }
            break;

        default:
            return ErrorCode::InvalidParameter;
    }

    return ErrorCode::Success;
}

ErrorCode create_custom_kernel(const uint8_t* data, int width, int height,
                               std::vector<uint8_t>& kernel) {
    if (!data || width < 1 || height < 1) {
        return ErrorCode::InvalidParameter;
    }

    kernel.assign(data, data + width * height);
    return ErrorCode::Success;
}

ErrorCode erode(const uint8_t* src, uint8_t* dst, int width, int height,
                const uint8_t* kernel, int kernel_width, int kernel_height,
                int iterations) {
    if (!src || !dst || !kernel) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> temp(width * height);
    std::vector<uint8_t> temp2(width * height);
    
    // 复制源数据到临时缓冲区
    std::copy(src, src + width * height, temp.begin());
    
    uint8_t* current_src = temp.data();
    uint8_t* current_dst = temp2.data();

    int kcx = kernel_width / 2;
    int kcy = kernel_height / 2;

    for (int iter = 0; iter < iterations; ++iter) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                uint8_t min_val = 255;

                // 遍历结构元素
                for (int ky = 0; ky < kernel_height; ++ky) {
                    for (int kx = 0; kx < kernel_width; ++kx) {
                        if (kernel[ky * kernel_width + kx] > 0) {
                            int sx = x + kx - kcx;
                            int sy = y + ky - kcy;

                            // 边界处理：超出边界视为255
                            if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
                                min_val = std::min(min_val, current_src[sy * width + sx]);
                            }
                        }
                    }
                }

                current_dst[y * width + x] = min_val;
            }
        }

        // 交换缓冲区
        std::swap(current_src, current_dst);
    }

    // 最后将结果复制到目标
    std::copy(current_src, current_src + width * height, dst);

    return ErrorCode::Success;
}

ErrorCode dilate(const uint8_t* src, uint8_t* dst, int width, int height,
                 const uint8_t* kernel, int kernel_width, int kernel_height,
                 int iterations) {
    if (!src || !dst || !kernel) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> temp(width * height);
    std::vector<uint8_t> temp2(width * height);
    
    // 复制源数据到临时缓冲区
    std::copy(src, src + width * height, temp.begin());
    
    uint8_t* current_src = temp.data();
    uint8_t* current_dst = temp2.data();

    int kcx = kernel_width / 2;
    int kcy = kernel_height / 2;

    for (int iter = 0; iter < iterations; ++iter) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                uint8_t max_val = 0;

                // 遍历结构元素
                for (int ky = 0; ky < kernel_height; ++ky) {
                    for (int kx = 0; kx < kernel_width; ++kx) {
                        if (kernel[ky * kernel_width + kx] > 0) {
                            int sx = x + kx - kcx;
                            int sy = y + ky - kcy;

                            // 边界处理：超出边界视为0
                            if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
                                max_val = std::max(max_val, current_src[sy * width + sx]);
                            }
                        }
                    }
                }

                current_dst[y * width + x] = max_val;
            }
        }

        // 交换缓冲区
        std::swap(current_src, current_dst);
    }

    // 最后将结果复制到目标
    std::copy(current_src, current_src + width * height, dst);

    return ErrorCode::Success;
}

ErrorCode open(const uint8_t* src, uint8_t* dst, int width, int height,
               const uint8_t* kernel, int kernel_width, int kernel_height,
               int iterations) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> temp(width * height);

    // 先腐蚀
    ErrorCode err = erode(src, temp.data(), width, height,
                          kernel, kernel_width, kernel_height, iterations);
    if (err != ErrorCode::Success) return err;

    // 再膨胀
    err = dilate(temp.data(), dst, width, height,
                 kernel, kernel_width, kernel_height, iterations);
    return err;
}

ErrorCode close(const uint8_t* src, uint8_t* dst, int width, int height,
                const uint8_t* kernel, int kernel_width, int kernel_height,
                int iterations) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> temp(width * height);

    // 先膨胀
    ErrorCode err = dilate(src, temp.data(), width, height,
                           kernel, kernel_width, kernel_height, iterations);
    if (err != ErrorCode::Success) return err;

    // 再腐蚀
    err = erode(temp.data(), dst, width, height,
                kernel, kernel_width, kernel_height, iterations);
    return err;
}

ErrorCode gradient(const uint8_t* src, uint8_t* dst, int width, int height,
                   const uint8_t* kernel, int kernel_width, int kernel_height) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> dilated(width * height);
    std::vector<uint8_t> eroded(width * height);

    ErrorCode err = dilate(src, dilated.data(), width, height,
                           kernel, kernel_width, kernel_height, 1);
    if (err != ErrorCode::Success) return err;

    err = erode(src, eroded.data(), width, height,
                kernel, kernel_width, kernel_height, 1);
    if (err != ErrorCode::Success) return err;

    // 膨胀 - 腐蚀
    for (int i = 0; i < width * height; ++i) {
        int diff = static_cast<int>(dilated[i]) - static_cast<int>(eroded[i]);
        dst[i] = static_cast<uint8_t>(std::clamp(diff, 0, 255));
    }

    return ErrorCode::Success;
}

ErrorCode top_hat(const uint8_t* src, uint8_t* dst, int width, int height,
                  const uint8_t* kernel, int kernel_width, int kernel_height,
                  int iterations) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> opened(width * height);

    ErrorCode err = open(src, opened.data(), width, height,
                         kernel, kernel_width, kernel_height, iterations);
    if (err != ErrorCode::Success) return err;

    // 原图 - 开运算结果
    for (int i = 0; i < width * height; ++i) {
        int diff = static_cast<int>(src[i]) - static_cast<int>(opened[i]);
        dst[i] = static_cast<uint8_t>(std::clamp(diff, 0, 255));
    }

    return ErrorCode::Success;
}

ErrorCode black_hat(const uint8_t* src, uint8_t* dst, int width, int height,
                    const uint8_t* kernel, int kernel_width, int kernel_height,
                    int iterations) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    std::vector<uint8_t> closed(width * height);

    ErrorCode err = close(src, closed.data(), width, height,
                          kernel, kernel_width, kernel_height, iterations);
    if (err != ErrorCode::Success) return err;

    // 闭运算结果 - 原图
    for (int i = 0; i < width * height; ++i) {
        int diff = static_cast<int>(closed[i]) - static_cast<int>(src[i]);
        dst[i] = static_cast<uint8_t>(std::clamp(diff, 0, 255));
    }

    return ErrorCode::Success;
}

ErrorCode skeleton(const uint8_t* src, uint8_t* dst, int width, int height) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    std::fill(dst, dst + width * height, 0);

    // 使用经典的骨架化算法
    // 需要构建8个方向的结构元素
    std::vector<uint8_t> kernel1 = {0, 0, 0, 255, 255, 0, 255, 255, 255}; // E
    std::vector<uint8_t> kernel2 = {255, 255, 0, 255, 255, 0, 0, 0, 0};   // NE
    std::vector<uint8_t> kernel3 = {255, 255, 255, 255, 255, 0, 0, 0, 0}; // N
    std::vector<uint8_t> kernel4 = {0, 0, 0, 255, 255, 0, 255, 255, 0};   // SE
    std::vector<uint8_t> kernel5 = {0, 0, 0, 0, 255, 255, 0, 255, 255};   // W
    std::vector<uint8_t> kernel6 = {255, 0, 0, 255, 255, 0, 255, 255, 0}; // SW
    std::vector<uint8_t> kernel7 = {255, 255, 0, 255, 255, 0, 255, 255, 0}; // S
    std::vector<uint8_t> kernel8 = {0, 255, 255, 0, 255, 255, 0, 0, 0};   // NW

    std::vector<uint8_t> current(width * height);
    std::copy(src, src + width * height, current.begin());

    std::vector<uint8_t> eroded(width * height);
    std::vector<uint8_t> temp(width * height);
    std::vector<uint8_t> diff(width * height);

    // 迭代直到不再变化
    bool changed = true;
    int max_iter = 100;
    int iter = 0;

    while (changed && iter < max_iter) {
        changed = false;
        iter++;

        // 腐蚀
        erode(current.data(), eroded.data(), width, height, kernel1.data(), 3, 3, 1);

        // 对每个方向的核进行细化
        for (int k = 0; k < 8; ++k) {
            const uint8_t* kernel = nullptr;
            switch (k) {
                case 0: kernel = kernel1.data(); break;
                case 1: kernel = kernel2.data(); break;
                case 2: kernel = kernel3.data(); break;
                case 3: kernel = kernel4.data(); break;
                case 4: kernel = kernel5.data(); break;
                case 5: kernel = kernel6.data(); break;
                case 6: kernel = kernel7.data(); break;
                case 7: kernel = kernel8.data(); break;
            }

            // 腐蚀
            erode(current.data(), temp.data(), width, height, kernel, 3, 3, 1);

            // 取反膨胀（hit-miss）
            for (int i = 0; i < width * height; ++i) {
                diff[i] = current[i] & (255 - temp[i]);
            }

            // 更新骨架
            for (int i = 0; i < width * height; ++i) {
                if (diff[i] > 0 && dst[i] == 0) {
                    dst[i] = 255;
                    changed = true;
                }
            }
        }

        // 更新当前图像
        std::copy(eroded.begin(), eroded.end(), current.begin());
    }

    return ErrorCode::Success;
}

ErrorCode distance_transform(const uint8_t* src, uint8_t* dst, int width, int height) {
    if (!src || !dst) {
        return ErrorCode::InvalidParameter;
    }

    // 使用两遍扫描算法
    std::vector<uint16_t> dist(width * height, 0);

    // 初始化：前景为无穷大，背景为0
    const uint16_t INF = 10000;
    for (int i = 0; i < width * height; ++i) {
        dist[i] = (src[i] > 128) ? INF : 0;
    }

    // 第一遍：从左上到右下
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (dist[y * width + x] > 0) {
                // 检查邻居
                uint16_t min_dist = INF;

                if (x > 0) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[y * width + x - 1] + 1));
                if (y > 0) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[(y - 1) * width + x] + 1));
                if (x > 0 && y > 0) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[(y - 1) * width + x - 1] + 1));
                if (x < width - 1 && y > 0) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[(y - 1) * width + x + 1] + 1));

                dist[y * width + x] = min_dist;
            }
        }
    }

    // 第二遍：从右下到左上
    for (int y = height - 1; y >= 0; --y) {
        for (int x = width - 1; x >= 0; --x) {
            if (dist[y * width + x] > 0) {
                uint16_t min_dist = dist[y * width + x];

                if (x < width - 1) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[y * width + x + 1] + 1));
                if (y < height - 1) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[(y + 1) * width + x] + 1));
                if (x < width - 1 && y < height - 1) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[(y + 1) * width + x + 1] + 1));
                if (x > 0 && y < height - 1) min_dist = std::min<uint16_t>(min_dist, static_cast<uint16_t>(dist[(y + 1) * width + x - 1] + 1));

                dist[y * width + x] = min_dist;
            }
        }
    }

    // 找到最大距离
    uint16_t max_dist = 0;
    for (int i = 0; i < width * height; ++i) {
        max_dist = std::max(max_dist, dist[i]);
    }

    // 归一化到0-255
    if (max_dist > 0) {
        for (int i = 0; i < width * height; ++i) {
            dst[i] = static_cast<uint8_t>(dist[i] * 255 / max_dist);
        }
    }

    return ErrorCode::Success;
}

ErrorCode reconstruct(const uint8_t* marker, const uint8_t* mask,
                      uint8_t* dst, int width, int height) {
    if (!marker || !mask || !dst) {
        return ErrorCode::InvalidParameter;
    }

    // 形态学重建：迭代膨胀直到不再变化
    std::copy(marker, marker + width * height, dst);

    std::vector<uint8_t> kernel = {255, 255, 255, 255, 255, 255, 255, 255, 255};
    std::vector<uint8_t> dilated(width * height);
    std::vector<uint8_t> prev(width * height);

    bool changed = true;
    int max_iter = 100;
    int iter = 0;

    while (changed && iter < max_iter) {
        changed = false;
        iter++;

        // 保存当前状态
        std::copy(dst, dst + width * height, prev.begin());

        // 膨胀
        dilate(dst, dilated.data(), width, height, kernel.data(), 3, 3, 1);

        // 与掩模取交集
        for (int i = 0; i < width * height; ++i) {
            dst[i] = std::min(dilated[i], mask[i]);
        }

        // 检查是否变化
        for (int i = 0; i < width * height; ++i) {
            if (dst[i] != prev[i]) {
                changed = true;
                break;
            }
        }
    }

    return ErrorCode::Success;
}

} // namespace morph_utils

// ========== MorphologyNode Implementation ==========

MorphologyNode::MorphologyNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MorphologyNode::make_info() {
    NodeInfo info;
    info.id = "Morphology";
    info.name = "形态学处理";
    info.category = "图像处理";
    info.description = "形态学处理节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));
    
    info.params.push_back(ParamDef("operation", "操作类型", DataType::String, Data("Erode")));
    info.params.push_back(ParamDef("kernel_shape", "核形状", DataType::String, Data("Rect")));
    info.params.push_back(ParamDef("kernel_size", "核大小", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("iterations", "迭代次数", DataType::Number, Data(1)));
    
    return info;
}

void MorphologyNode::set_kernel(KernelShape shape, int ksize) {
    kernel_shape_ = shape;
    kernel_size_ = ksize;
    if (kernel_size_ % 2 == 0) kernel_size_++;
    morph_utils::create_kernel(kernel_shape_, kernel_size_, kernel_);
}

Result<void> MorphologyNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    String op = get_param("operation", Data("Erode")).as_string();
    if (op == "Erode") operation_ = MorphOp::Erode;
    else if (op == "Dilate") operation_ = MorphOp::Dilate;
    else if (op == "Open") operation_ = MorphOp::Open;
    else if (op == "Close") operation_ = MorphOp::Close;
    else if (op == "Gradient") operation_ = MorphOp::Gradient;
    else if (op == "TopHat") operation_ = MorphOp::TopHat;
    else if (op == "BlackHat") operation_ = MorphOp::BlackHat;
    
    String shape = get_param("kernel_shape", Data("Rect")).as_string();
    if (shape == "Rect") kernel_shape_ = KernelShape::Rect;
    else if (shape == "Ellipse") kernel_shape_ = KernelShape::Ellipse;
    else if (shape == "Cross") kernel_shape_ = KernelShape::Cross;
    else if (shape == "Diamond") kernel_shape_ = KernelShape::Diamond;
    
    kernel_size_ = static_cast<int>(get_param("kernel_size", Data(3)).as_int());
    if (kernel_size_ % 2 == 0) kernel_size_++;
    
    iterations_ = static_cast<int>(get_param("iterations", Data(1)).as_int());
    
    // 创建结构元素
    ErrorCode err = morph_utils::create_kernel(kernel_shape_, kernel_size_, kernel_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Failed to create kernel");
    }

    // 转换为灰度
    std::vector<uint8_t> src_gray;
    int src_w, src_h;

    if (input.channels == 1) {
        src_gray = input.data;
        src_w = input.width;
        src_h = input.height;
    } else {
        src_w = input.width;
        src_h = input.height;
        src_gray.resize(src_w * src_h);
        for (int i = 0; i < src_w * src_h; ++i) {
            int idx = i * input.channels;
            src_gray[i] = static_cast<uint8_t>(
                (input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3);
        }
    }

    std::vector<uint8_t> dst_gray(src_w * src_h);

    switch (operation_) {
        case MorphOp::Erode:
            err = morph_utils::erode(src_gray.data(), dst_gray.data(),
                                     src_w, src_h, kernel_.data(),
                                     kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Dilate:
            err = morph_utils::dilate(src_gray.data(), dst_gray.data(),
                                      src_w, src_h, kernel_.data(),
                                      kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Open:
            err = morph_utils::open(src_gray.data(), dst_gray.data(),
                                    src_w, src_h, kernel_.data(),
                                    kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Close:
            err = morph_utils::close(src_gray.data(), dst_gray.data(),
                                     src_w, src_h, kernel_.data(),
                                     kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Gradient:
            err = morph_utils::gradient(src_gray.data(), dst_gray.data(),
                                        src_w, src_h, kernel_.data(),
                                        kernel_size_, kernel_size_);
            break;
        case MorphOp::TopHat:
            err = morph_utils::top_hat(src_gray.data(), dst_gray.data(),
                                       src_w, src_h, kernel_.data(),
                                       kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::BlackHat:
            err = morph_utils::black_hat(src_gray.data(), dst_gray.data(),
                                         src_w, src_h, kernel_.data(),
                                         kernel_size_, kernel_size_, iterations_);
            break;
    }

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Morphology operation failed");
    }

    // 创建输出图像
    ImageData output;
    output.width = src_w;
    output.height = src_h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = dst_gray;

    set_output("image", Data(output));
    
    OVF_INFO() << "Morphology operation completed: op=" << op;

    return Result<void>::success();
}

// ========== ThresholdMorphNode Implementation ==========

ThresholdMorphNode::ThresholdMorphNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ThresholdMorphNode::make_info() {
    NodeInfo info;
    info.id = "ThresholdMorph";
    info.name = "阈值形态学";
    info.category = "图像处理";
    info.description = "阈值+形态学处理节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "二值图像", DataType::Image));
    
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(128)));
    info.params.push_back(ParamDef("morph_op", "形态学操作", DataType::String, Data("Erode")));
    info.params.push_back(ParamDef("kernel_shape", "核形状", DataType::String, Data("Rect")));
    info.params.push_back(ParamDef("kernel_size", "核大小", DataType::Number, Data(3)));
    info.params.push_back(ParamDef("iterations", "迭代次数", DataType::Number, Data(1)));
    
    return info;
}

Result<void> ThresholdMorphNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 获取参数
    threshold_value_ = static_cast<int>(get_param("threshold", Data(128)).as_int());
    
    String op = get_param("morph_op", Data("Erode")).as_string();
    if (op == "Erode") morph_op_ = MorphOp::Erode;
    else if (op == "Dilate") morph_op_ = MorphOp::Dilate;
    else if (op == "Open") morph_op_ = MorphOp::Open;
    else if (op == "Close") morph_op_ = MorphOp::Close;
    
    String shape = get_param("kernel_shape", Data("Rect")).as_string();
    if (shape == "Rect") kernel_shape_ = KernelShape::Rect;
    else if (shape == "Ellipse") kernel_shape_ = KernelShape::Ellipse;
    else if (shape == "Cross") kernel_shape_ = KernelShape::Cross;
    else if (shape == "Diamond") kernel_shape_ = KernelShape::Diamond;
    
    kernel_size_ = static_cast<int>(get_param("kernel_size", Data(3)).as_int());
    if (kernel_size_ % 2 == 0) kernel_size_++;
    
    iterations_ = static_cast<int>(get_param("iterations", Data(1)).as_int());

    // 转换为灰度并阈值化
    std::vector<uint8_t> binary;
    int w = input.width;
    int h = input.height;
    binary.resize(w * h);

    for (int i = 0; i < w * h; ++i) {
        int gray;
        if (input.channels == 1) {
            gray = input.data[i];
        } else {
            int idx = i * input.channels;
            gray = (input.data[idx] + input.data[idx + 1] + input.data[idx + 2]) / 3;
        }
        binary[i] = (gray > threshold_value_) ? 255 : 0;
    }

    // 形态学处理
    std::vector<uint8_t> kernel;
    morph_utils::create_kernel(kernel_shape_, kernel_size_, kernel);

    std::vector<uint8_t> result(w * h);
    ErrorCode err;

    switch (morph_op_) {
        case MorphOp::Erode:
            err = morph_utils::erode(binary.data(), result.data(), w, h,
                                     kernel.data(), kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Dilate:
            err = morph_utils::dilate(binary.data(), result.data(), w, h,
                                      kernel.data(), kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Open:
            err = morph_utils::open(binary.data(), result.data(), w, h,
                                    kernel.data(), kernel_size_, kernel_size_, iterations_);
            break;
        case MorphOp::Close:
            err = morph_utils::close(binary.data(), result.data(), w, h,
                                     kernel.data(), kernel_size_, kernel_size_, iterations_);
            break;
        default:
            err = ErrorCode::InvalidParameter;
            break;
    }

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Morphology operation failed");
    }

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = result;

    set_output("image", Data(output));
    
    OVF_INFO() << "Threshold morph operation completed: threshold=" << threshold_value_;

    return Result<void>::success();
}

// ========== FillHolesNode Implementation ==========

FillHolesNode::FillHolesNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FillHolesNode::make_info() {
    NodeInfo info;
    info.id = "FillHoles";
    info.name = "孔洞填充";
    info.category = "图像处理";
    info.description = "孔洞填充节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "二值图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "填充后的图像", DataType::Image));
    
    return info;
}

Result<void> FillHolesNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为二值
    std::vector<uint8_t> binary(w * h);
    for (int i = 0; i < w * h; ++i) {
        int val = input.channels == 1 ? input.data[i] :
                  (input.data[i * input.channels] + 
                   input.data[i * input.channels + 1] +
                   input.data[i * input.channels + 2]) / 3;
        binary[i] = (val > 128) ? 255 : 0;
    }

    // 创建标记图像（填充边界）
    std::vector<uint8_t> marker(w * h, 255);

    // 边界初始化为与原图相反
    for (int x = 0; x < w; ++x) {
        marker[0 * w + x] = binary[0 * w + x] == 0 ? 255 : 0;
        marker[(h - 1) * w + x] = binary[(h - 1) * w + x] == 0 ? 255 : 0;
    }
    for (int y = 0; y < h; ++y) {
        marker[y * w + 0] = binary[y * w + 0] == 0 ? 255 : 0;
        marker[y * w + (w - 1)] = binary[y * w + (w - 1)] == 0 ? 255 : 0;
    }

    // 创建掩模（原图的反）
    std::vector<uint8_t> mask(w * h);
    for (int i = 0; i < w * h; ++i) {
        mask[i] = 255 - binary[i];
    }

    // 形态学重建
    std::vector<uint8_t> reconstructed(w * h);
    morph_utils::reconstruct(marker.data(), mask.data(), reconstructed.data(), w, h);

    // 取反得到填充结果
    std::vector<uint8_t> result(w * h);
    for (int i = 0; i < w * h; ++i) {
        result[i] = 255 - reconstructed[i];
    }

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = result;

    set_output("image", Data(output));
    
    OVF_INFO() << "Fill holes completed";

    return Result<void>::success();
}

// ========== ExtractBoundaryNode Implementation ==========

ExtractBoundaryNode::ExtractBoundaryNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ExtractBoundaryNode::make_info() {
    NodeInfo info;
    info.id = "ExtractBoundary";
    info.name = "边界提取";
    info.category = "图像处理";
    info.description = "边界提取节点";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "二值图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "边界图像", DataType::Image));
    
    return info;
}

Result<void> ExtractBoundaryNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int w = input.width;
    int h = input.height;

    // 转换为二值
    std::vector<uint8_t> binary(w * h);
    for (int i = 0; i < w * h; ++i) {
        int val = input.channels == 1 ? input.data[i] :
                  (input.data[i * input.channels] + 
                   input.data[i * input.channels + 1] +
                   input.data[i * input.channels + 2]) / 3;
        binary[i] = (val > 128) ? 255 : 0;
    }

    // 腐蚀
    std::vector<uint8_t> kernel = {255, 255, 255, 255, 255, 255, 255, 255, 255};
    std::vector<uint8_t> eroded(w * h);
    morph_utils::erode(binary.data(), eroded.data(), w, h, kernel.data(), 3, 3, 1);

    // 边界 = 原图 - 腐蚀
    std::vector<uint8_t> boundary(w * h);
    for (int i = 0; i < w * h; ++i) {
        boundary[i] = binary[i] > eroded[i] ? 255 : 0;
    }

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = boundary;

    set_output("image", Data(output));
    
    OVF_INFO() << "Extract boundary completed";

    return Result<void>::success();
}

// ========== ThinningNode Implementation ==========

ThinningNode::ThinningNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ThinningNode::make_info() {
    NodeInfo info;
    info.id = "Thinning";
    info.name = "细化";
    info.category = "图像处理";
    info.description = "细化节点（骨架提取）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "二值图像", DataType::Image, true));
    info.outputs.push_back(DataPort("image", "细化图像", DataType::Image));
    
    info.params.push_back(ParamDef("max_iterations", "最大迭代次数", DataType::Number, Data(100)));
    
    return info;
}

Result<void> ThinningNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    max_iterations_ = static_cast<int>(get_param("max_iterations", Data(100)).as_int());

    int w = input.width;
    int h = input.height;

    // 转换为二值
    std::vector<uint8_t> binary(w * h);
    for (int i = 0; i < w * h; ++i) {
        int val = input.channels == 1 ? input.data[i] :
                  (input.data[i * input.channels] + 
                   input.data[i * input.channels + 1] +
                   input.data[i * input.channels + 2]) / 3;
        binary[i] = (val > 128) ? 255 : 0;
    }

    // 骨架提取
    std::vector<uint8_t> skeleton(w * h);
    morph_utils::skeleton(binary.data(), skeleton.data(), w, h);

    // 输出
    ImageData output;
    output.width = w;
    output.height = h;
    output.channels = 1;
    output.format = ImageFormat::Mono8;
    output.data = skeleton;

    set_output("image", Data(output));
    
    OVF_INFO() << "Thinning completed";

    return Result<void>::success();
}

// ========== Node Registration ==========

OVF_REGISTER_NODE(MorphologyNode, "Morphology", MorphologyNode::make_info())
OVF_REGISTER_NODE(ThresholdMorphNode, "ThresholdMorph", ThresholdMorphNode::make_info())
OVF_REGISTER_NODE(FillHolesNode, "FillHoles", FillHolesNode::make_info())
OVF_REGISTER_NODE(ExtractBoundaryNode, "ExtractBoundary", ExtractBoundaryNode::make_info())
OVF_REGISTER_NODE(ThinningNode, "Thinning", ThinningNode::make_info())

} // namespace algorithm
} // namespace ovf