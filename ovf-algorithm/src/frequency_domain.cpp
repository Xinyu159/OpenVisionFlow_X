#include "ovf/algorithm/frequency_domain.h"
#include "ovf/core/data.h"
#include "ovf/core/logger.h"
#include <cmath>
#include <algorithm>
#include <complex>

namespace ovf {
namespace algorithm {

namespace {
// 读取像素值并转为 float（支持 Float32 和 8 位格式）
inline float pixel_at_f(const ovf::ImageData& img, int y, int x) {
    size_t idx = static_cast<size_t>(y) * img.width + x;
    if (img.format == ovf::ImageFormat::Float32) {
        return reinterpret_cast<const float*>(img.data.data())[idx];
    }
    return static_cast<float>(img.data[idx]);
}

// 获取 Float32 图像像素的可写引用
inline float& float_pixel(ovf::ImageData& img, int y, int x) {
    float* p = reinterpret_cast<float*>(img.data.data());
    return p[static_cast<size_t>(y) * img.width + x];
}

// 读取 Float32 图像像素
inline float float_pixel(const ovf::ImageData& img, int y, int x) {
    const float* p = reinterpret_cast<const float*>(img.data.data());
    return p[static_cast<size_t>(y) * img.width + x];
}

// 获取 8 位图像像素的可写引用
inline uint8_t& uint8_pixel(ovf::ImageData& img, int y, int x) {
    return img.data[static_cast<size_t>(y) * img.width + x];
}

// 创建 Float32 图像
inline ovf::ImageData create_float_image(int width, int height) {
    ovf::ImageData img;
    img.width = width;
    img.height = height;
    img.channels = 1;
    img.format = ovf::ImageFormat::Float32;
    img.data.resize(static_cast<size_t>(width) * height * sizeof(float), 0);
    return img;
}

// 创建 Mono8（UInt8）图像
inline ovf::ImageData create_uint8_image(int width, int height) {
    ovf::ImageData img;
    img.width = width;
    img.height = height;
    img.channels = 1;
    img.format = ovf::ImageFormat::Mono8;
    img.data.resize(static_cast<size_t>(width) * height, 0);
    return img;
}
} // anonymous namespace

// FFT辅助函数 - Cooley-Tukey算法
void fft1d(std::vector<std::complex<float>>& data, bool inverse) {
    int n = data.size();
    if (n <= 1) return;

    // 位反转排列
    int j = 0;
    for (int i = 0; i < n; ++i) {
        if (i < j) std::swap(data[i], data[j]);
        int k = n >> 1;
        while (k >= 1 && j >= k) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    // Cooley-Tukey迭代
    float sign = inverse ? 1.0f : -1.0f;
    for (int len = 2; len <= n; len <<= 1) {
        float angle = sign * 2.0f * 3.14159265358979323846f / len;
        std::complex<float> wlen(std::cos(angle), std::sin(angle));
        for (int i = 0; i < n; i += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (int j = 0; j < len / 2; ++j) {
                std::complex<float> u = data[i + j];
                std::complex<float> v = data[i + j + len / 2] * w;
                data[i + j] = u + v;
                data[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }

    // 逆变换需要归一化
    if (inverse) {
        for (int i = 0; i < n; ++i) {
            data[i] /= n;
        }
    }
}

// 2D FFT
void fft2d(std::vector<std::vector<std::complex<float>>>& data, int rows, int cols, bool inverse) {
    // 行FFT
    for (int i = 0; i < rows; ++i) {
        fft1d(data[i], inverse);
    }

    // 列FFT
    for (int j = 0; j < cols; ++j) {
        std::vector<std::complex<float>> col(rows);
        for (int i = 0; i < rows; ++i) {
            col[i] = data[i][j];
        }
        fft1d(col, inverse);
        for (int i = 0; i < rows; ++i) {
            data[i][j] = col[i];
        }
    }
}

// FFT正变换节点
FFTForwardNode::FFTForwardNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FFTForwardNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FFTForward";
    info.name = "FFT正向变换";
    info.category = "频域处理";
    info.description = "将图像从空间域转换到频域";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("frequency_image", "频域图像（复数）", DataType::Image));
    info.outputs.push_back(DataPort("magnitude", "幅度谱", DataType::Image));
    info.outputs.push_back(DataPort("phase", "相位谱", DataType::Image));
    return info;
}

Result<void> FFTForwardNode::execute(FlowContext& context) {
    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;

    // 转换为复数数组
    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }

    // 执行FFT
    fft2d(freq_data, rows, cols, false);

    // 输出复数频域数据
    auto freq_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    auto mag_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    auto phase_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float real = freq_data[y][x].real();
            float imag = freq_data[y][x].imag();
            float_pixel(*freq_img, y, x) = real; // 简化存储实部
            float_pixel(*mag_img, y, x) = std::sqrt(real * real + imag * imag);
            float_pixel(*phase_img, y, x) = std::atan2(imag, real);
        }
    }

    set_output("frequency_image", Data(*freq_img));
    set_output("magnitude", Data(*mag_img));
    set_output("phase", Data(*phase_img));

    OVF_INFO() << "FFT正变换完成：" << rows << "x" << cols;
    return Result<void>::success();
}

// FFT逆变换节点
FFTInverseNode::FFTInverseNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FFTInverseNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FFTInverse";
    info.name = "FFT逆向变换";
    info.category = "频域处理";
    info.description = "将频域图像转换回空间域";
    info.inputs.push_back(DataPort("frequency_image", "频域图像", DataType::Image, true));
    info.inputs.push_back(DataPort("phase", "相位谱（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("image", "空间域图像", DataType::Image));
    return info;
}

Result<void> FFTInverseNode::execute(FlowContext& context) {
    auto freq_data = get_input("frequency_image");
    if (!freq_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要频域图像");
    }

    auto& freq_img = freq_data.as_image();
    int rows = freq_img.height;
    int cols = freq_img.width;

    // 构建复数数据（假设相位为0或使用提供的相位）
    std::vector<std::vector<std::complex<float>>> spatial_data(rows, std::vector<std::complex<float>>(cols));
    
    auto phase_data = get_input("phase");
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float real = pixel_at_f(freq_img, y, x);
            float imag = 0.0f;
            if (phase_data.is_image()) {
                float phase = pixel_at_f(phase_data.as_image(), y, x);
                float mag = std::abs(real);
                real = mag * std::cos(phase);
                imag = mag * std::sin(phase);
            }
            spatial_data[y][x] = std::complex<float>(real, imag);
        }
    }

    // 执行IFFT
    fft2d(spatial_data, rows, cols, true);

    // 输出空间域图像
    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = spatial_data[y][x].real();
        }
    }

    set_output("image", Data(*output_img));
    OVF_INFO() << "FFT逆变换完成：" << rows << "x" << cols;
    return Result<void>::success();
}

// 理想低通滤波器
IdealLowPassNode::IdealLowPassNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo IdealLowPassNode::make_info() {
    ovf::NodeInfo info;
    info.id = "IdealLowPass";
    info.name = "理想低通滤波";
    info.category = "频域处理";
    info.description = "理想低通滤波器，截止频率外完全抑制";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("cutoff_frequency", "归一化截止频率[0,1]", DataType::Number, Data(0.5f)));
    return info;
}

Result<void> IdealLowPassNode::execute(FlowContext& context) {
    cutoff_frequency_ = static_cast<float>(get_param("cutoff_frequency", Data(cutoff_frequency_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    int cutoff_pixels = static_cast<int>(cutoff_frequency_ * std::min(rows, cols) / 2);

    // FFT
    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移（零频到中心）
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int new_y = (y + center_y) % rows;
            int new_x = (x + center_x) % cols;
            if (y != new_y || x != new_x) {
                std::swap(freq_data[y][x], freq_data[new_y][new_x]);
            }
        }
    }

    // 应用理想低通滤波
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            if (dist > cutoff_pixels) {
                freq_data[y][x] = 0.0f;
            }
        }
    }

    // 频移恢复
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int new_y = (y + center_y) % rows;
            int new_x = (x + center_x) % cols;
            if (y != new_y || x != new_x) {
                std::swap(freq_data[y][x], freq_data[new_y][new_x]);
            }
        }
    }

    // IFFT
    fft2d(freq_data, rows, cols, true);

    // 输出
    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    OVF_INFO() << "理想低通滤波完成，截止频率=" << cutoff_frequency_;
    return Result<void>::success();
}

// 理想高通滤波器
IdealHighPassNode::IdealHighPassNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo IdealHighPassNode::make_info() {
    ovf::NodeInfo info;
    info.id = "IdealHighPass";
    info.name = "理想高通滤波";
    info.category = "频域处理";
    info.description = "理想高通滤波器，截止频率内完全抑制";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("cutoff_frequency", "归一化截止频率[0,1]", DataType::Number, Data(0.1f)));
    return info;
}

Result<void> IdealHighPassNode::execute(FlowContext& context) {
    cutoff_frequency_ = static_cast<float>(get_param("cutoff_frequency", Data(cutoff_frequency_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    int cutoff_pixels = static_cast<int>(cutoff_frequency_ * std::min(rows, cols) / 2);

    // FFT + 频移
    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 高通滤波
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            if (dist < cutoff_pixels) {
                freq_data[y][x] = 0.0f;
            }
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 高斯低通滤波器
GaussLowPassNode::GaussLowPassNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo GaussLowPassNode::make_info() {
    ovf::NodeInfo info;
    info.id = "GaussLowPass";
    info.name = "高斯低通滤波";
    info.category = "频域处理";
    info.description = "高斯低通滤波器，平滑过渡";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("sigma", "高斯标准差", DataType::Number, Data(10.0f)));
    return info;
}

Result<void> GaussLowPassNode::execute(FlowContext& context) {
    sigma_ = static_cast<float>(get_param("sigma", Data(sigma_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 高斯低通
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = std::exp(-(dist * dist) / (2 * sigma_ * sigma_));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 高斯高通滤波器
GaussHighPassNode::GaussHighPassNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo GaussHighPassNode::make_info() {
    ovf::NodeInfo info;
    info.id = "GaussHighPass";
    info.name = "高斯高通滤波";
    info.category = "频域处理";
    info.description = "高斯高通滤波器";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("sigma", "高斯标准差", DataType::Number, Data(10.0f)));
    return info;
}

Result<void> GaussHighPassNode::execute(FlowContext& context) {
    sigma_ = static_cast<float>(get_param("sigma", Data(sigma_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 高斯高通
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = 1.0f - std::exp(-(dist * dist) / (2 * sigma_ * sigma_));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 巴特沃斯低通滤波器
ButterworthLowPassNode::ButterworthLowPassNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo ButterworthLowPassNode::make_info() {
    ovf::NodeInfo info;
    info.id = "ButterworthLowPass";
    info.name = "巴特沃斯低通滤波";
    info.category = "频域处理";
    info.description = "巴特沃斯低通滤波器";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("cutoff_frequency", "归一化截止频率", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("order", "滤波器阶数", DataType::Number, Data(2)));
    return info;
}

Result<void> ButterworthLowPassNode::execute(FlowContext& context) {
    cutoff_frequency_ = static_cast<float>(get_param("cutoff_frequency", Data(cutoff_frequency_)).as_number());
    order_ = get_param("order", Data(order_)).as_int();

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    float d0 = cutoff_frequency_ * std::min(rows, cols) / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 巴特沃斯低通
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = 1.0f / (1.0f + std::pow(dist / d0, 2 * order_));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 巴特沃斯高通滤波器
ButterworthHighPassNode::ButterworthHighPassNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo ButterworthHighPassNode::make_info() {
    ovf::NodeInfo info;
    info.id = "ButterworthHighPass";
    info.name = "巴特沃斯高通滤波";
    info.category = "频域处理";
    info.description = "巴特沃斯高通滤波器";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("cutoff_frequency", "归一化截止频率", DataType::Number, Data(0.1f)));
    info.params.push_back(ParamDef("order", "滤波器阶数", DataType::Number, Data(2)));
    return info;
}

Result<void> ButterworthHighPassNode::execute(FlowContext& context) {
    cutoff_frequency_ = static_cast<float>(get_param("cutoff_frequency", Data(cutoff_frequency_)).as_number());
    order_ = get_param("order", Data(order_)).as_int();

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    float d0 = cutoff_frequency_ * std::min(rows, cols) / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 巴特沃斯高通
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = 1.0f / (1.0f + std::pow(d0 / (dist + 0.001f), 2 * order_));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 带通滤波器
BandPassFilterNode::BandPassFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo BandPassFilterNode::make_info() {
    ovf::NodeInfo info;
    info.id = "BandPass";
    info.name = "带通滤波";
    info.category = "频域处理";
    info.description = "带通滤波器";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("center_frequency", "中心频率", DataType::Number, Data(0.3f)));
    info.params.push_back(ParamDef("bandwidth", "带宽", DataType::Number, Data(0.1f)));
    return info;
}

Result<void> BandPassFilterNode::execute(FlowContext& context) {
    center_frequency_ = static_cast<float>(get_param("center_frequency", Data(center_frequency_)).as_number());
    bandwidth_ = static_cast<float>(get_param("bandwidth", Data(bandwidth_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    float d_center = center_frequency_ * std::min(rows, cols) / 2;
    float d_band = bandwidth_ * std::min(rows, cols) / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 带通滤波
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = std::exp(-((dist - d_center) * (dist - d_center)) / (2 * d_band * d_band));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 带阻滤波器
BandStopFilterNode::BandStopFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo BandStopFilterNode::make_info() {
    ovf::NodeInfo info;
    info.id = "BandStop";
    info.name = "带阻滤波";
    info.category = "频域处理";
    info.description = "带阻滤波器";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("center_frequency", "中心频率", DataType::Number, Data(0.3f)));
    info.params.push_back(ParamDef("bandwidth", "带宽", DataType::Number, Data(0.1f)));
    return info;
}

Result<void> BandStopFilterNode::execute(FlowContext& context) {
    center_frequency_ = static_cast<float>(get_param("center_frequency", Data(center_frequency_)).as_number());
    bandwidth_ = static_cast<float>(get_param("bandwidth", Data(bandwidth_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    float d_center = center_frequency_ * std::min(rows, cols) / 2;
    float d_band = bandwidth_ * std::min(rows, cols) / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 带阻滤波
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = 1.0f - std::exp(-((dist - d_center) * (dist - d_center)) / (2 * d_band * d_band));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 同态滤波
HomomorphicFilterNode::HomomorphicFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo HomomorphicFilterNode::make_info() {
    ovf::NodeInfo info;
    info.id = "HomomorphicFilter";
    info.name = "同态滤波";
    info.category = "频域处理";
    info.description = "同态滤波增强对比度，校正不均匀光照";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("high_frequency_gain", "高频增益", DataType::Number, Data(2.0f)));
    info.params.push_back(ParamDef("low_frequency_gain", "低频增益", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("cutoff_frequency", "截止频率", DataType::Number, Data(0.2f)));
    return info;
}

Result<void> HomomorphicFilterNode::execute(FlowContext& context) {
    high_frequency_gain_ = static_cast<float>(get_param("high_frequency_gain", Data(high_frequency_gain_)).as_number());
    low_frequency_gain_ = static_cast<float>(get_param("low_frequency_gain", Data(low_frequency_gain_)).as_number());
    cutoff_frequency_ = static_cast<float>(get_param("cutoff_frequency", Data(cutoff_frequency_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;
    float d0 = cutoff_frequency_ * std::min(rows, cols) / 2;

    // 对数变换
    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float val = pixel_at_f(img, y, x) + 1.0f; // 避免log(0)
            freq_data[y][x] = std::complex<float>(std::log(val), 0.0f);
        }
    }

    // FFT + 频移
    fft2d(freq_data, rows, cols, false);
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 同态滤波器
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = (high_frequency_gain_ - low_frequency_gain_) * 
                     (1.0f - std::exp(-(dist * dist) / (2 * d0 * d0))) + low_frequency_gain_;
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    // 指数变换
    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = std::exp(freq_data[y][x].real()) - 1.0f;
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 维纳滤波
WienerFilterNode::WienerFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo WienerFilterNode::make_info() {
    ovf::NodeInfo info;
    info.id = "WienerFilter";
    info.name = "维纳滤波";
    info.category = "频域处理";
    info.description = "维纳滤波去模糊";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("h", "退化函数（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("noise_power", "噪声功率谱", DataType::Number, Data(0.01f)));
    return info;
}

Result<void> WienerFilterNode::execute(FlowContext& context) {
    noise_power_ = static_cast<float>(get_param("noise_power", Data(noise_power_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 简化维纳滤波（假设H=1）
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float mag_sq = std::norm(freq_data[y][x]);
            float h_sq = 1.0f; // 假设退化函数为1
            float w = h_sq / (h_sq + noise_power_ / (mag_sq + 0.001f));
            freq_data[y][x] *= w;
        }
    }

    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 频谱可视化
PowerSpectrumNode::PowerSpectrumNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo PowerSpectrumNode::make_info() {
    ovf::NodeInfo info;
    info.id = "PowerSpectrum";
    info.name = "功率谱";
    info.category = "频域处理";
    info.description = "生成频谱幅度图像用于分析";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("power_spectrum", "功率谱图像", DataType::Image));
    info.params.push_back(ParamDef("log_scale", "对数尺度", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("center_shift", "零频移到中心", DataType::Boolean, Data(true)));
    return info;
}

Result<void> PowerSpectrumNode::execute(FlowContext& context) {
    log_scale_ = get_param("log_scale", Data(log_scale_)).as_bool();
    center_shift_ = get_param("center_shift", Data(center_shift_)).as_bool();

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    if (center_shift_) {
        for (int y = 0; y < rows / 2; ++y) {
            for (int x = 0; x < cols; ++x) {
                std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
            }
        }
    }

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    float max_val = 0.0f;
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float power = std::norm(freq_data[y][x]);
            if (log_scale_) {
                power = std::log(power + 1.0f);
            }
            float_pixel(*output_img, y, x) = power;
            max_val = std::max(max_val, power);
        }
    }

    // 归一化到0-255
    if (max_val > 0) {
        auto display_img = std::make_shared<ovf::ImageData>(create_uint8_image(cols, rows));
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                uint8_pixel(*display_img, y, x) = static_cast<uint8_t>(255.0f * float_pixel(*output_img, y, x) / max_val);
            }
        }
        set_output("power_spectrum", Data(*display_img));
    } else {
        set_output("power_spectrum", Data(*output_img));
    }

    return Result<void>::success();
}

// 相位谱
PhaseSpectrumNode::PhaseSpectrumNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo PhaseSpectrumNode::make_info() {
    ovf::NodeInfo info;
    info.id = "PhaseSpectrum";
    info.name = "相位谱";
    info.category = "频域处理";
    info.description = "生成相位谱图像";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("phase_spectrum", "相位谱图像", DataType::Image));
    return info;
}

Result<void> PhaseSpectrumNode::execute(FlowContext& context) {
    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float phase = std::arg(freq_data[y][x]);
            float_pixel(*output_img, y, x) = phase;
        }
    }

    set_output("phase_spectrum", Data(*output_img));
    return Result<void>::success();
}

// 频域乘法
FrequencyMultiplyNode::FrequencyMultiplyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencyMultiplyNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencyMultiply";
    info.name = "频域乘法";
    info.category = "频域处理";
    info.description = "频域乘法（卷积定理）";
    info.inputs.push_back(DataPort("image1", "第一张图像", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "第二张图像", DataType::Image, true));
    info.outputs.push_back(DataPort("result", "乘积结果", DataType::Image));
    return info;
}

Result<void> FrequencyMultiplyNode::execute(FlowContext& context) {
    auto img1_data = get_input("image1");
    auto img2_data = get_input("image2");
    if (!img1_data.is_image() || !img2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要两张输入图像");
    }

    auto& img1 = img1_data.as_image();
    auto& img2 = img2_data.as_image();

    if (img1.width != img2.width || img1.height != img2.height) {
        return Result<void>::failure(ErrorCode::InvalidImage, "图像尺寸必须相同");
    }

    int rows = img1.height;
    int cols = img1.width;

    std::vector<std::vector<std::complex<float>>> freq1(rows, std::vector<std::complex<float>>(cols));
    std::vector<std::vector<std::complex<float>>> freq2(rows, std::vector<std::complex<float>>(cols));
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq1[y][x] = std::complex<float>(pixel_at_f(img1, y, x), 0.0f);
            freq2[y][x] = std::complex<float>(pixel_at_f(img2, y, x), 0.0f);
        }
    }

    fft2d(freq1, rows, cols, false);
    fft2d(freq2, rows, cols, false);

    // 频域乘法
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq1[y][x] *= freq2[y][x];
        }
    }

    fft2d(freq1, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq1[y][x].real();
        }
    }

    set_output("result", Data(*output_img));
    return Result<void>::success();
}

// 频域相关
FrequencyCorrelateNode::FrequencyCorrelateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencyCorrelateNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencyCorrelate";
    info.name = "频域相关";
    info.category = "频域处理";
    info.description = "频域相关计算";
    info.inputs.push_back(DataPort("image1", "第一张图像", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "第二张图像", DataType::Image, true));
    info.outputs.push_back(DataPort("correlation", "相关结果", DataType::Image));
    return info;
}

Result<void> FrequencyCorrelateNode::execute(FlowContext& context) {
    auto img1_data = get_input("image1");
    auto img2_data = get_input("image2");
    if (!img1_data.is_image() || !img2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要两张输入图像");
    }

    auto& img1 = img1_data.as_image();
    auto& img2 = img2_data.as_image();

    int rows = img1.height;
    int cols = img1.width;

    std::vector<std::vector<std::complex<float>>> freq1(rows, std::vector<std::complex<float>>(cols));
    std::vector<std::vector<std::complex<float>>> freq2(rows, std::vector<std::complex<float>>(cols));
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq1[y][x] = std::complex<float>(pixel_at_f(img1, y, x), 0.0f);
            freq2[y][x] = std::complex<float>(pixel_at_f(img2, y, x), 0.0f);
        }
    }

    fft2d(freq1, rows, cols, false);
    fft2d(freq2, rows, cols, false);

    // 相关：F1 * conj(F2)
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq1[y][x] *= std::conj(freq2[y][x]);
        }
    }

    fft2d(freq1, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq1[y][x].real();
        }
    }

    set_output("correlation", Data(*output_img));
    return Result<void>::success();
}

// 相位相关（图像配准）
PhaseCorrelationNode::PhaseCorrelationNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo PhaseCorrelationNode::make_info() {
    ovf::NodeInfo info;
    info.id = "PhaseCorrelation";
    info.name = "相位相关";
    info.category = "频域处理";
    info.description = "相位相关图像配准（平移估计）";
    info.inputs.push_back(DataPort("image1", "参考图像", DataType::Image, true));
    info.inputs.push_back(DataPort("image2", "待配准图像", DataType::Image, true));
    info.outputs.push_back(DataPort("shift_x", "X方向平移", DataType::Number));
    info.outputs.push_back(DataPort("shift_y", "Y方向平移", DataType::Number));
    info.outputs.push_back(DataPort("confidence", "置信度", DataType::Number));
    info.params.push_back(ParamDef("threshold", "峰值阈值", DataType::Number, Data(0.5f)));
    return info;
}

Result<void> PhaseCorrelationNode::execute(FlowContext& context) {
    threshold_ = static_cast<float>(get_param("threshold", Data(threshold_)).as_number());

    auto img1_data = get_input("image1");
    auto img2_data = get_input("image2");
    if (!img1_data.is_image() || !img2_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要两张输入图像");
    }

    auto& img1 = img1_data.as_image();
    auto& img2 = img2_data.as_image();

    int rows = img1.height;
    int cols = img1.width;

    std::vector<std::vector<std::complex<float>>> freq1(rows, std::vector<std::complex<float>>(cols));
    std::vector<std::vector<std::complex<float>>> freq2(rows, std::vector<std::complex<float>>(cols));
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq1[y][x] = std::complex<float>(pixel_at_f(img1, y, x), 0.0f);
            freq2[y][x] = std::complex<float>(pixel_at_f(img2, y, x), 0.0f);
        }
    }

    fft2d(freq1, rows, cols, false);
    fft2d(freq2, rows, cols, false);

    // 相位相关：F1 * conj(F2) / |F1 * F2|
    std::vector<std::vector<std::complex<float>>> cross_power(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            auto prod = freq1[y][x] * std::conj(freq2[y][x]);
            float mag = std::abs(prod);
            if (mag > 0.001f) {
                cross_power[y][x] = prod / mag;
            } else {
                cross_power[y][x] = 0.0f;
            }
        }
    }

    fft2d(cross_power, rows, cols, true);

    // 找峰值位置
    float max_val = 0.0f;
    int peak_x = 0, peak_y = 0;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float val = std::abs(cross_power[y][x]);
            if (val > max_val) {
                max_val = val;
                peak_x = x;
                peak_y = y;
            }
        }
    }

    // 处理边界位移
    if (peak_x > cols / 2) peak_x -= cols;
    if (peak_y > rows / 2) peak_y -= rows;

    float confidence = max_val / (rows * cols);

    set_output("shift_x", Data(peak_x));
    set_output("shift_y", Data(peak_y));
    set_output("confidence", Data(confidence));

    OVF_INFO() << "相位相关平移估计: (" << peak_x << ", " << peak_y << "), 置信度=" << confidence;
    return Result<void>::success();
}

// 频域缺陷检测
FrequencyDefectDetectNode::FrequencyDefectDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencyDefectDetectNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencyDefectDetect";
    info.name = "频域缺陷检测";
    info.category = "频域处理";
    info.description = "频域+空域结合缺陷检测";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像（可选）", DataType::Image, false));
    info.outputs.push_back(DataPort("defect_mask", "缺陷掩码", DataType::Image));
    info.outputs.push_back(DataPort("defect_regions", "缺陷区域列表", DataType::Region));
    info.params.push_back(ParamDef("low_cutoff", "低频截止", DataType::Number, Data(0.05f)));
    info.params.push_back(ParamDef("high_cutoff", "高频截止", DataType::Number, Data(0.4f)));
    info.params.push_back(ParamDef("threshold", "阈值", DataType::Number, Data(30.0f)));
    return info;
}

Result<void> FrequencyDefectDetectNode::execute(FlowContext& context) {
    low_cutoff_ = static_cast<float>(get_param("low_cutoff", Data(low_cutoff_)).as_number());
    high_cutoff_ = static_cast<float>(get_param("high_cutoff", Data(high_cutoff_)).as_number());
    threshold_ = static_cast<float>(get_param("threshold", Data(threshold_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 带通滤波保留缺陷相关频率
    int low_d = static_cast<int>(low_cutoff_ * std::min(rows, cols) / 2);
    int high_d = static_cast<int>(high_cutoff_ * std::min(rows, cols) / 2);
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            if (dist < low_d || dist > high_d) {
                freq_data[y][x] = 0.0f;
            }
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    // 阈值检测
    auto defect_mask = std::make_shared<ovf::ImageData>(create_uint8_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float val = std::abs(freq_data[y][x].real());
            uint8_pixel(*defect_mask, y, x) = (val > threshold_) ? 255 : 0;
        }
    }

    set_output("defect_mask", Data(*defect_mask));
    return Result<void>::success();
}

// 周期性噪声去除
RemovePeriodicNoiseNode::RemovePeriodicNoiseNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo RemovePeriodicNoiseNode::make_info() {
    ovf::NodeInfo info;
    info.id = "RemovePeriodicNoise";
    info.name = "去除周期性噪声";
    info.category = "频域处理";
    info.description = "去除周期性噪声（条纹、摩尔纹）";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("filtered_image", "滤波后图像", DataType::Image));
    info.params.push_back(ParamDef("threshold_ratio", "噪声峰值阈值比例", DataType::Number, Data(0.1f)));
    info.params.push_back(ParamDef("notch_radius", "陷波滤波器半径", DataType::Number, Data(5)));
    return info;
}

Result<void> RemovePeriodicNoiseNode::execute(FlowContext& context) {
    threshold_ratio_ = static_cast<float>(get_param("threshold_ratio", Data(threshold_ratio_)).as_number());
    notch_radius_ = get_param("notch_radius", Data(notch_radius_)).as_int();

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 找噪声峰值（排除中心低频）
    float max_mag = 0.0f;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            max_mag = std::max(max_mag, std::abs(freq_data[y][x]));
        }
    }

    float threshold_mag = max_mag * threshold_ratio_;

    // 陷波滤波（移除峰值噪声点）
    for (int y = center_y + 20; y < rows; ++y) { // 排除中心低频区域
        for (int x = 0; x < cols; ++x) {
            if (std::abs(freq_data[y][x]) > threshold_mag) {
                // 陷波滤波器移除该频率及其对称点
                for (int dy = -notch_radius_; dy <= notch_radius_; ++dy) {
                    for (int dx = -notch_radius_; dx <= notch_radius_; ++dx) {
                        int ny1 = y + dy;
                        int nx1 = x + dx;
                        int ny2 = rows - y + dy;
                        int nx2 = cols - x + dx;
                        
                        if (ny1 >= 0 && ny1 < rows && nx1 >= 0 && nx1 < cols) {
                            freq_data[ny1][nx1] *= 0.0f;
                        }
                        if (ny2 >= 0 && ny2 < rows && nx2 >= 0 && nx2 < cols) {
                            freq_data[ny2][nx2] *= 0.0f;
                        }
                    }
                }
            }
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("filtered_image", Data(*output_img));
    return Result<void>::success();
}

// 纹理分析（FFT）
TextureAnalysisFFTNode::TextureAnalysisFFTNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo TextureAnalysisFFTNode::make_info() {
    ovf::NodeInfo info;
    info.id = "TextureAnalysisFFT";
    info.name = "纹理分析(FFT)";
    info.category = "频域处理";
    info.description = "使用频域方法分析图像纹理";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("features", "纹理特征(逗号分隔字符串)", DataType::String));
    info.params.push_back(ParamDef("num_features", "特征数量", DataType::Number, Data(10)));
    return info;
}

Result<void> TextureAnalysisFFTNode::execute(FlowContext& context) {
    num_features_ = get_param("num_features", Data(num_features_)).as_int();

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 提取纹理特征（频谱统计）
    std::vector<float> features;
    
    // 总能量
    float total_energy = 0.0f;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            total_energy += std::norm(freq_data[y][x]);
        }
    }
    features.push_back(total_energy);

    // 主频率能量
    int center_y = rows / 2;
    int center_x = cols / 2;
    float high_freq_energy = 0.0f;
    float low_freq_energy = 0.0f;
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float dist = std::sqrt((y-center_y)*(y-center_y) + (x-center_x)*(x-center_x));
            float power = std::norm(freq_data[y][x]);
            if (dist > std::min(rows, cols) / 4) {
                high_freq_energy += power;
            } else {
                low_freq_energy += power;
            }
        }
    }
    features.push_back(low_freq_energy);
    features.push_back(high_freq_energy);
    features.push_back(high_freq_energy / (low_freq_energy + 0.001f));

    // 方向性特征（扇形区域能量）
    int num_sectors = 8;
    std::vector<float> sector_energies(num_sectors, 0.0f);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float dy = y - center_y;
            float dx = x - center_x;
            float dist = std::sqrt(dy*dy + dx*dx);
            if (dist > 10 && dist < std::min(rows, cols) / 2) {
                float angle = std::atan2(dy, dx);
                int sector = static_cast<int>((angle + 3.14159265f) / (2 * 3.14159265f) * num_sectors);
                sector = std::max(0, std::min(num_sectors - 1, sector));
                sector_energies[sector] += std::norm(freq_data[y][x]);
            }
        }
    }
    for (int i = 0; i < std::min(num_sectors, num_features_ - 4); ++i) {
        features.push_back(sector_energies[i]);
    }

    // 将特征向量转换为逗号分隔的字符串（Data类不支持std::vector<float>）
    String features_str;
    for (size_t i = 0; i < features.size(); ++i) {
        if (i > 0) features_str += ",";
        features_str += std::to_string(features[i]);
    }
    set_output("features", Data(features_str));
    return Result<void>::success();
}

// 频域锐化
FrequencySharpenNode::FrequencySharpenNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencySharpenNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencySharpen";
    info.name = "频域锐化";
    info.category = "频域处理";
    info.description = "频域图像锐化";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("sharpened", "锐化后图像", DataType::Image));
    info.params.push_back(ParamDef("amount", "锐化强度", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("radius", "锐化半径", DataType::Number, Data(20.0f)));
    return info;
}

Result<void> FrequencySharpenNode::execute(FlowContext& context) {
    amount_ = static_cast<float>(get_param("amount", Data(amount_)).as_number());
    radius_ = static_cast<float>(get_param("radius", Data(radius_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 高频增强
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float boost = 1.0f + amount_ * (1.0f - std::exp(-(dist * dist) / (2 * radius_ * radius_)));
            freq_data[y][x] *= boost;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("sharpened", Data(*output_img));
    return Result<void>::success();
}

// 频域边缘增强
FrequencyEdgeEnhanceNode::FrequencyEdgeEnhanceNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencyEdgeEnhanceNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencyEdgeEnhance";
    info.name = "频域边缘增强";
    info.category = "频域处理";
    info.description = "频域边缘增强";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("enhanced", "增强后图像", DataType::Image));
    info.params.push_back(ParamDef("strength", "增强强度", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("direction", "边缘方向角度", DataType::Number, Data(0.0f)));
    return info;
}

Result<void> FrequencyEdgeEnhanceNode::execute(FlowContext& context) {
    strength_ = static_cast<float>(get_param("strength", Data(strength_)).as_number());
    direction_ = static_cast<float>(get_param("direction", Data(direction_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 方向性边缘增强
    float target_angle = direction_ * 3.14159265f / 180.0f;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float angle = std::atan2(dy, dx);
            float angle_diff = std::abs(angle - target_angle);
            float boost = 1.0f + strength_ * std::exp(-angle_diff * angle_diff);
            freq_data[y][x] *= boost;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("enhanced", Data(*output_img));
    return Result<void>::success();
}

// 频域平滑
FrequencySmoothNode::FrequencySmoothNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencySmoothNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencySmooth";
    info.name = "频域平滑";
    info.category = "频域处理";
    info.description = "频域图像平滑";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("smoothed", "平滑后图像", DataType::Image));
    info.params.push_back(ParamDef("sigma", "平滑强度", DataType::Number, Data(5.0f)));
    return info;
}

Result<void> FrequencySmoothNode::execute(FlowContext& context) {
    sigma_ = static_cast<float>(get_param("sigma", Data(sigma_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = std::complex<float>(pixel_at_f(img, y, x), 0.0f);
        }
    }
    fft2d(freq_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 高斯低通平滑
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = std::exp(-(dist * dist) / (2 * sigma_ * sigma_));
            freq_data[y][x] *= h;
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("smoothed", Data(*output_img));
    return Result<void>::success();
}

// 频域反锐化掩模
FrequencyUnsharpMaskNode::FrequencyUnsharpMaskNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

ovf::NodeInfo FrequencyUnsharpMaskNode::make_info() {
    ovf::NodeInfo info;
    info.id = "FrequencyUnsharpMask";
    info.name = "频域反锐化掩模";
    info.category = "频域处理";
    info.description = "频域反锐化掩模";
    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.outputs.push_back(DataPort("result", "处理结果", DataType::Image));
    info.params.push_back(ParamDef("sigma", "模糊半径", DataType::Number, Data(5.0f)));
    info.params.push_back(ParamDef("amount", "锐化强度", DataType::Number, Data(0.5f)));
    return info;
}

Result<void> FrequencyUnsharpMaskNode::execute(FlowContext& context) {
    sigma_ = static_cast<float>(get_param("sigma", Data(sigma_)).as_number());
    amount_ = static_cast<float>(get_param("amount", Data(amount_)).as_number());

    auto img_data = get_input("image");
    if (!img_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "需要输入图像");
    }

    auto& img = img_data.as_image();
    int rows = img.height;
    int cols = img.width;
    int center_y = rows / 2;
    int center_x = cols / 2;

    std::vector<std::vector<std::complex<float>>> freq_data(rows, std::vector<std::complex<float>>(cols));
    std::vector<std::vector<std::complex<float>>> blur_data(rows, std::vector<std::complex<float>>(cols));
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float val = pixel_at_f(img, y, x);
            freq_data[y][x] = std::complex<float>(val, 0.0f);
            blur_data[y][x] = std::complex<float>(val, 0.0f);
        }
    }

    fft2d(freq_data, rows, cols, false);
    fft2d(blur_data, rows, cols, false);

    // 频移
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
            std::swap(blur_data[y][x], blur_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }

    // 低通滤波
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int dy = y - center_y;
            int dx = x - center_x;
            float dist = std::sqrt(dy * dy + dx * dx);
            float h = std::exp(-(dist * dist) / (2 * sigma_ * sigma_));
            blur_data[y][x] *= h;
        }
    }

    // 反锐化掩模: 原图 + amount * (原图 - 模糊图)
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            freq_data[y][x] = freq_data[y][x] + amount_ * (freq_data[y][x] - blur_data[y][x]);
        }
    }

    // 频移恢复 + IFFT
    for (int y = 0; y < rows / 2; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::swap(freq_data[y][x], freq_data[(y + center_y) % rows][(x + center_x) % cols]);
        }
    }
    fft2d(freq_data, rows, cols, true);

    auto output_img = std::make_shared<ovf::ImageData>(create_float_image(cols, rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float_pixel(*output_img, y, x) = freq_data[y][x].real();
        }
    }

    set_output("result", Data(*output_img));
    return Result<void>::success();
}

// 注册所有节点
OVF_REGISTER_NODE(FFTForwardNode, "FFTForward", FFTForwardNode::make_info())
OVF_REGISTER_NODE(FFTInverseNode, "FFTInverse", FFTInverseNode::make_info())
OVF_REGISTER_NODE(IdealLowPassNode, "IdealLowPass", IdealLowPassNode::make_info())
OVF_REGISTER_NODE(IdealHighPassNode, "IdealHighPass", IdealHighPassNode::make_info())
OVF_REGISTER_NODE(GaussLowPassNode, "GaussLowPass", GaussLowPassNode::make_info())
OVF_REGISTER_NODE(GaussHighPassNode, "GaussHighPass", GaussHighPassNode::make_info())
OVF_REGISTER_NODE(ButterworthLowPassNode, "ButterworthLowPass", ButterworthLowPassNode::make_info())
OVF_REGISTER_NODE(ButterworthHighPassNode, "ButterworthHighPass", ButterworthHighPassNode::make_info())
OVF_REGISTER_NODE(BandPassFilterNode, "BandPass", BandPassFilterNode::make_info())
OVF_REGISTER_NODE(BandStopFilterNode, "BandStop", BandStopFilterNode::make_info())
OVF_REGISTER_NODE(HomomorphicFilterNode, "HomomorphicFilter", HomomorphicFilterNode::make_info())
OVF_REGISTER_NODE(WienerFilterNode, "WienerFilter", WienerFilterNode::make_info())
OVF_REGISTER_NODE(PowerSpectrumNode, "PowerSpectrum", PowerSpectrumNode::make_info())
OVF_REGISTER_NODE(PhaseSpectrumNode, "PhaseSpectrum", PhaseSpectrumNode::make_info())
OVF_REGISTER_NODE(FrequencyMultiplyNode, "FrequencyMultiply", FrequencyMultiplyNode::make_info())
OVF_REGISTER_NODE(FrequencyCorrelateNode, "FrequencyCorrelate", FrequencyCorrelateNode::make_info())
OVF_REGISTER_NODE(PhaseCorrelationNode, "PhaseCorrelation", PhaseCorrelationNode::make_info())
OVF_REGISTER_NODE(FrequencyDefectDetectNode, "FrequencyDefectDetect", FrequencyDefectDetectNode::make_info())
OVF_REGISTER_NODE(RemovePeriodicNoiseNode, "RemovePeriodicNoise", RemovePeriodicNoiseNode::make_info())
OVF_REGISTER_NODE(TextureAnalysisFFTNode, "TextureAnalysisFFT", TextureAnalysisFFTNode::make_info())
OVF_REGISTER_NODE(FrequencySharpenNode, "FrequencySharpen", FrequencySharpenNode::make_info())
OVF_REGISTER_NODE(FrequencyEdgeEnhanceNode, "FrequencyEdgeEnhance", FrequencyEdgeEnhanceNode::make_info())
OVF_REGISTER_NODE(FrequencySmoothNode, "FrequencySmooth", FrequencySmoothNode::make_info())
OVF_REGISTER_NODE(FrequencyUnsharpMaskNode, "FrequencyUnsharpMask", FrequencyUnsharpMaskNode::make_info())

} // namespace algorithm
} // namespace ovf
