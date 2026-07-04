/**
 * @file subpixel_precision.cpp
 * @brief 亚像素精度验证与优化模块 - 节点与核心算法实现
 *
 * 实现5类核心算法：
 *   1. 二阶泰勒展开亚像素边缘定位（参考Halcon edges_sub_pix）
 *   2. 三点抛物线拟合亚像素定位
 *   3. 高斯函数拟合亚像素定位
 *   4. Saddle点角点亚像素定位（二次曲面拟合）
 *   5. Zernike矩亚像素边缘定位（高级算法）
 *
 * 纯C++实现，不依赖OpenCV。
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>
#include <random>
#include <numeric>

#include "ovf/algorithm/subpixel_precision.h"
#include "ovf/core/flow.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

// ============================================================================
// subpixel_utils 实现
// ============================================================================

namespace subpixel_utils {

// ---------------------- 图像基础工具 ----------------------

ImageData to_gray(const ImageData& input) {
    if (input.empty()) {
        return ImageData();
    }
    if (input.channels == 1) {
        return input;
    }

    ImageData gray;
    gray.width = input.width;
    gray.height = input.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(static_cast<size_t>(gray.width) * gray.height);
    gray.timestamp = input.timestamp;
    gray.frame_id = input.frame_id;
    gray.source_id = input.source_id;

    for (size_t i = 0; i < gray.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * input.channels];
            uint8_t g = input.data[i * input.channels + 1];
            uint8_t r = input.data[i * input.channels + 2];
            gray.data[i] = static_cast<uint8_t>(0.114f * b + 0.587f * g + 0.299f * r);
        } else if (input.channels == 2) {
            gray.data[i] = input.data[i * 2];
        } else {
            gray.data[i] = input.data[i * input.channels];
        }
    }
    return gray;
}

float bilinear_sample(const ImageData& image, float x, float y) {
    if (image.empty() || image.channels != 1) {
        return 0.0f;
    }

    // 边界裁剪
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    if (x > static_cast<float>(image.width - 1)) x = static_cast<float>(image.width - 1);
    if (y > static_cast<float>(image.height - 1)) y = static_cast<float>(image.height - 1);

    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    int x1 = std::min(x0 + 1, static_cast<int>(image.width) - 1);
    int y1 = std::min(y0 + 1, static_cast<int>(image.height) - 1);

    float fx = x - static_cast<float>(x0);
    float fy = y - static_cast<float>(y0);

    float v00 = static_cast<float>(image.data[static_cast<size_t>(y0) * image.width + x0]);
    float v01 = static_cast<float>(image.data[static_cast<size_t>(y0) * image.width + x1]);
    float v10 = static_cast<float>(image.data[static_cast<size_t>(y1) * image.width + x0]);
    float v11 = static_cast<float>(image.data[static_cast<size_t>(y1) * image.width + x1]);

    float v0 = v00 + (v01 - v00) * fx;
    float v1 = v10 + (v11 - v10) * fx;
    return v0 + (v1 - v0) * fy;
}

std::vector<float> gaussian_kernel_1d(float sigma, int kernel_size) {
    if (kernel_size <= 0) {
        kernel_size = static_cast<int>(std::ceil(sigma * 6.0f)) | 1;
        if (kernel_size < 3) kernel_size = 3;
        if (kernel_size > 31) kernel_size = 31;
    }
    if (kernel_size % 2 == 0) ++kernel_size;

    std::vector<float> kernel(kernel_size);
    int half = kernel_size / 2;
    float sum = 0.0f;

    for (int i = 0; i < kernel_size; ++i) {
        float x = static_cast<float>(i - half);
        float val = std::exp(-(x * x) / (2.0f * sigma * sigma));
        kernel[i] = val;
        sum += val;
    }
    for (float& k : kernel) k /= sum;
    return kernel;
}

ImageData gaussian_smooth(const ImageData& input, float sigma) {
    if (input.empty() || sigma <= 0.0f) return input;
    if (input.channels != 1) return input;

    auto kernel = gaussian_kernel_1d(sigma);
    int half = static_cast<int>(kernel.size()) / 2;

    ImageData temp;
    temp.width = input.width;
    temp.height = input.height;
    temp.channels = 1;
    temp.format = ImageFormat::Mono8;
    temp.data.resize(static_cast<size_t>(input.width) * input.height);
    temp.timestamp = input.timestamp;
    temp.frame_id = input.frame_id;
    temp.source_id = input.source_id;

    int w = static_cast<int>(input.width);
    int h = static_cast<int>(input.height);

    // 水平方向卷积
    std::vector<float> horiz(static_cast<size_t>(w) * h, 0.0f);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float sum = 0.0f;
            float wsum = 0.0f;
            for (int k = 0; k < static_cast<int>(kernel.size()); ++k) {
                int xx = x + k - half;
                if (xx < 0) xx = 0;
                if (xx >= w) xx = w - 1;
                float val = static_cast<float>(input.data[static_cast<size_t>(y) * w + xx]);
                sum += val * kernel[k];
                wsum += kernel[k];
            }
            horiz[static_cast<size_t>(y) * w + x] = (wsum > 0.0f) ? sum / wsum : 0.0f;
        }
    }

    // 垂直方向卷积
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float sum = 0.0f;
            float wsum = 0.0f;
            for (int k = 0; k < static_cast<int>(kernel.size()); ++k) {
                int yy = y + k - half;
                if (yy < 0) yy = 0;
                if (yy >= h) yy = h - 1;
                float val = horiz[static_cast<size_t>(yy) * w + x];
                sum += val * kernel[k];
                wsum += kernel[k];
            }
            float result = (wsum > 0.0f) ? sum / wsum : 0.0f;
            // 限幅
            if (result < 0.0f) result = 0.0f;
            if (result > 255.0f) result = 255.0f;
            temp.data[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(result);
        }
    }

    return temp;
}

// ---------------------- 梯度计算 ----------------------

void sobel_gradient(const ImageData& image,
                    std::vector<float>& grad_x,
                    std::vector<float>& grad_y,
                    std::vector<float>& magnitude,
                    std::vector<float>& angle) {
    size_t total = static_cast<size_t>(image.width) * image.height;
    grad_x.assign(total, 0.0f);
    grad_y.assign(total, 0.0f);
    magnitude.assign(total, 0.0f);
    angle.assign(total, 0.0f);

    if (image.empty() || image.channels != 1) return;

    int w = static_cast<int>(image.width);
    int h = static_cast<int>(image.height);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            // 3x3 邻域
            float p00 = static_cast<float>(image.data[static_cast<size_t>(y - 1) * w + (x - 1)]);
            float p01 = static_cast<float>(image.data[static_cast<size_t>(y - 1) * w + x]);
            float p02 = static_cast<float>(image.data[static_cast<size_t>(y - 1) * w + (x + 1)]);
            float p10 = static_cast<float>(image.data[static_cast<size_t>(y) * w + (x - 1)]);
            // p11 中心点
            float p12 = static_cast<float>(image.data[static_cast<size_t>(y) * w + (x + 1)]);
            float p20 = static_cast<float>(image.data[static_cast<size_t>(y + 1) * w + (x - 1)]);
            float p21 = static_cast<float>(image.data[static_cast<size_t>(y + 1) * w + x]);
            float p22 = static_cast<float>(image.data[static_cast<size_t>(y + 1) * w + (x + 1)]);

            // Sobel Gx = [-1 0 1; -2 0 2; -1 0 1]
            float gx = (p02 + 2.0f * p12 + p22) - (p00 + 2.0f * p10 + p20);
            // Sobel Gy = [-1 -2 -1; 0 0 0; 1 2 1]
            float gy = (p20 + 2.0f * p21 + p22) - (p00 + 2.0f * p01 + p02);

            size_t idx = static_cast<size_t>(y) * w + x;
            grad_x[idx] = gx;
            grad_y[idx] = gy;
            magnitude[idx] = std::sqrt(gx * gx + gy * gy);
            angle[idx] = std::atan2(gy, gx);
        }
    }
}

// ---------------------- 亚像素定位核心算法 ----------------------

float taylor_subpixel_offset(float g0, float g1, float g2) {
    // 二阶泰勒展开：在峰值点附近，一阶导数接近0
    // g(x) ≈ g1 + g'(0)*x + (1/2)*g''(0)*x^2
    // 一阶导数：g'(x) = g'(0) + g''(0)*x = 0
    // => x = -g'(0) / g''(0)
    // 离散近似：g'(0) ≈ (g2 - g0) / 2，g''(0) ≈ g2 - 2g1 + g0
    float second_deriv = g2 - 2.0f * g1 + g0;
    float first_deriv = (g2 - g0) * 0.5f;

    if (std::abs(second_deriv) < 1e-6f) {
        return 0.0f;
    }

    float offset = -first_deriv / second_deriv;
    if (offset < -0.5f) offset = -0.5f;
    if (offset > 0.5f) offset = 0.5f;
    return offset;
}

float parabola_subpixel_offset(float y0, float y1, float y2) {
    // y = ax^2 + bx + c
    // y(-1) = y0, y(0) = y1, y(1) = y2
    // => a = (y0 + y2 - 2y1)/2, b = (y2 - y0)/2, c = y1
    // 极值点 x = -b / (2a)
    float a = (y0 + y2 - 2.0f * y1) * 0.5f;
    float b = (y2 - y0) * 0.5f;

    if (std::abs(a) < 1e-6f) {
        return 0.0f;
    }

    float offset = -b / (2.0f * a);
    if (offset < -0.5f) offset = -0.5f;
    if (offset > 0.5f) offset = 0.5f;
    return offset;
}

float gaussian_subpixel_offset(float y0, float y1, float y2) {
    // 假设响应为高斯分布 y(x) = A * exp(-(x-mu)^2 / (2*sigma^2))
    // 取对数：ln(y) = ln(A) - (x-mu)^2 / (2*sigma^2)
    // 令 L(x) = ln(y(x))，则 L 为二次函数，极值点即亚像素位置
    // 在三点 (-1, 0, 1) 上对 L 做抛物线拟合
    // 注意：需要保证 y0, y1, y2 > 0
    const float eps = 1e-3f;
    if (y0 < eps || y1 < eps || y2 < eps) {
        return parabola_subpixel_offset(y0, y1, y2);
    }

    float l0 = std::log(y0);
    float l1 = std::log(y1);
    float l2 = std::log(y2);

    float offset = parabola_subpixel_offset(l0, l1, l2);
    if (offset < -0.5f) offset = -0.5f;
    if (offset > 0.5f) offset = 0.5f;
    return offset;
}

bool saddle_subpixel_offset(const float values[9], float& dx, float& dy) {
    // 3x3邻域拟合二次曲面 R(x,y) = a*x^2 + b*y^2 + c*x*y + d*x + e*y + f
    // 使用最小二乘法求解，然后在中心点求梯度=0位置
    //
    // 中心点为 f = values[4]
    // d = (R(1,0) - R(-1,0)) / 2 = (values[5] - values[3]) / 2
    // e = (R(0,1) - R(0,-1)) / 2 = (values[7] - values[1]) / 2
    // a = (R(1,0) + R(-1,0) - 2R(0,0)) / 2 = (values[5] + values[3] - 2*values[4]) / 2
    // b = (R(0,1) + R(0,-1) - 2R(0,0)) / 2 = (values[7] + values[1] - 2*values[4]) / 2
    // c = (R(1,1) - R(1,-1) - R(-1,1) + R(-1,-1)) / 4
    //   = (values[8] - values[6] - values[2] + values[0]) / 4
    //
    // 极值点：dR/dx = 2a*x + c*y + d = 0
    //         dR/dy = 2b*y + c*x + e = 0
    // 解线性方程组：
    //   [2a  c] [x]   [-d]
    //   [c  2b] [y] = [-e]

    float v0 = values[0], v1 = values[1], v2 = values[2];
    float v3 = values[3], v4 = values[4], v5 = values[5];
    float v6 = values[6], v7 = values[7], v8 = values[8];

    float d = (v5 - v3) * 0.5f;
    float e = (v7 - v1) * 0.5f;
    float a = (v5 + v3 - 2.0f * v4) * 0.5f;
    float b = (v7 + v1 - 2.0f * v4) * 0.5f;
    float c = (v8 - v6 - v2 + v0) * 0.25f;

    float det = 4.0f * a * b - c * c;
    if (std::abs(det) < 1e-6f) {
        dx = 0.0f;
        dy = 0.0f;
        return false;
    }

    // 逆矩阵：1/det * [2b  -c; -c  2a]
    dx = (-2.0f * b * d + c * e) / det;
    dy = (c * d - 2.0f * a * e) / det;

    // 限制范围
    if (dx < -0.5f) dx = -0.5f;
    if (dx > 0.5f) dx = 0.5f;
    if (dy < -0.5f) dy = -0.5f;
    if (dy > 0.5f) dy = 0.5f;
    return true;
}

bool zernike_subpixel_edge(const float values[25], float& sub_x, float& sub_y,
                           float& angle, float& strength) {
    // Zernike矩亚像素边缘定位（Ghosal & Mehrotra 1993）
    // 5x5 邻域，单位圆半径 r = 2 (像素)
    //
    // 计算 Z11（复数）, Z20（实数）, Z22（复数）
    // 边缘方向：theta = atan2(Im(Z11), Re(Z11))
    // 边缘距离中心点：l = Z20 / sqrt(Re(Z11)^2 + Im(Z11)^2) * h
    // 其中 h 为像素步长（=1）
    //
    // 标准 5x5 Zernike 核（归一化系数已包含）

    // M11 实部（Re of V_11 = x）
    static const float M11_real[25] = {
        -0.04467f, -0.08504f,  0.0f,       0.08504f,  0.04467f,
        -0.08504f, -0.16194f,  0.0f,       0.16194f,  0.08504f,
        -0.11038f, -0.21014f,  0.0f,       0.21014f,  0.11038f,
        -0.08504f, -0.16194f,  0.0f,       0.16194f,  0.08504f,
        -0.04467f, -0.08504f,  0.0f,       0.08504f,  0.04467f
    };
    // M11 虚部（Im of V_11 = y），注意图像y向下
    static const float M11_imag[25] = {
         0.04467f,  0.08504f,  0.11038f,  0.08504f,  0.04467f,
         0.08504f,  0.16194f,  0.21014f,  0.16194f,  0.08504f,
         0.0f,      0.0f,      0.0f,      0.0f,      0.0f,
        -0.08504f, -0.16194f, -0.21014f, -0.16194f, -0.08504f,
        -0.04467f, -0.08504f, -0.11038f, -0.08504f, -0.04467f
    };
    // M20 (V_20 = 2r^2 - 1)
    static const float M20[25] = {
         0.09175f,  0.10253f,  0.10610f,  0.10253f,  0.09175f,
         0.10253f,  0.11450f,  0.11848f,  0.11450f,  0.10253f,
         0.10610f,  0.11848f,  0.12250f,  0.11848f,  0.10610f,
         0.10253f,  0.11450f,  0.11848f,  0.11450f,  0.10253f,
         0.09175f,  0.10253f,  0.10610f,  0.10253f,  0.09175f
    };

    float z11_real = 0.0f, z11_imag = 0.0f, z20 = 0.0f;
    for (int i = 0; i < 25; ++i) {
        z11_real += values[i] * M11_real[i];
        z11_imag += values[i] * M11_imag[i];
        z20 += values[i] * M20[i];
    }

    float z11_mag = std::sqrt(z11_real * z11_real + z11_imag * z11_imag);
    if (z11_mag < 1e-3f) {
        sub_x = 0.0f;
        sub_y = 0.0f;
        angle = 0.0f;
        strength = 0.0f;
        return false;
    }

    // 边缘方向（梯度方向）
    angle = std::atan2(z11_imag, z11_real);

    // 边缘距离（从中心到边缘的距离，单位：像素）
    // 注意 Z20 与 |Z11| 的比例给出归一化距离
    // 单位圆半径 = 2 像素
    float l = z20 / z11_mag * 2.0f;

    // 限制范围在 [-0.5, 0.5]
    if (l > 0.5f) l = 0.5f;
    if (l < -0.5f) l = -0.5f;

    // 边缘位置 = 中心 + l * (cos(theta), sin(theta))
    sub_x = l * std::cos(angle);
    sub_y = l * std::sin(angle);

    strength = z11_mag;
    return true;
}

// ---------------------- Harris角点响应 ----------------------

float harris_response(const std::vector<float>& grad_x,
                      const std::vector<float>& grad_y,
                      uint32_t width, uint32_t height,
                      uint32_t x, uint32_t y,
                      float k, int window_size) {
    if (window_size < 1) window_size = 1;
    int half = window_size;
    int w = static_cast<int>(width);
    int h = static_cast<int>(height);

    int x0 = static_cast<int>(x) - half;
    int y0 = static_cast<int>(y) - half;
    int x1 = static_cast<int>(x) + half;
    int y1 = static_cast<int>(y) + half;
    if (x0 < 1) x0 = 1;
    if (y0 < 1) y0 = 1;
    if (x1 > w - 1) x1 = w - 1;
    if (y1 > h - 1) y1 = h - 1;

    float sum_ix2 = 0.0f, sum_iy2 = 0.0f, sum_ixiy = 0.0f;
    int count = 0;
    for (int yy = y0; yy <= y1; ++yy) {
        for (int xx = x0; xx <= x1; ++xx) {
            size_t idx = static_cast<size_t>(yy) * w + xx;
            float gx = grad_x[idx];
            float gy = grad_y[idx];
            sum_ix2 += gx * gx;
            sum_iy2 += gy * gy;
            sum_ixiy += gx * gy;
            ++count;
        }
    }
    if (count == 0) return 0.0f;
    sum_ix2 /= count;
    sum_iy2 /= count;
    sum_ixiy /= count;

    // M = [Ix2  IxIy; IxIy  Iy2]
    // det(M) = Ix2*Iy2 - IxIy^2
    // trace(M) = Ix2 + Iy2
    // R = det - k*trace^2
    float det = sum_ix2 * sum_iy2 - sum_ixiy * sum_ixiy;
    float trace = sum_ix2 + sum_iy2;
    return det - k * trace * trace;
}

// ---------------------- 测试图生成与噪声 ----------------------

ImageData generate_test_pattern(uint32_t width, uint32_t height,
                                TestPatternType type,
                                float param1, float param2,
                                uint8_t fg_value, uint8_t bg_value) {
    ImageData img;
    img.width = width;
    img.height = height;
    img.channels = 1;
    img.format = ImageFormat::Mono8;
    img.data.resize(static_cast<size_t>(width) * height, bg_value);

    int w = static_cast<int>(width);
    int h = static_cast<int>(height);

    switch (type) {
        case TestPatternType::VerticalEdge: {
            // param1 = 边缘位置X（亚像素）
            // 在 |x - param1| < 1 范围内做线性过渡（边缘平滑）
            float edge_x = param1;
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    float dist = static_cast<float>(x) - edge_x;
                    // 阶跃边缘：x < edge_x 时为 bg，x > edge_x 时为 fg
                    // 但为了模拟相机拍摄的真实边缘，加 1 像素线性过渡
                    float value;
                    if (dist < -0.5f) {
                        value = static_cast<float>(bg_value);
                    } else if (dist > 0.5f) {
                        value = static_cast<float>(fg_value);
                    } else {
                        // 线性过渡
                        float t = dist + 0.5f;  // [0, 1]
                        value = bg_value + (fg_value - bg_value) * t;
                    }
                    img.data[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(value);
                }
            }
            break;
        }
        case TestPatternType::HorizontalEdge: {
            // param1 = 边缘位置Y（亚像素）
            float edge_y = param1;
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    float dist = static_cast<float>(y) - edge_y;
                    float value;
                    if (dist < -0.5f) {
                        value = static_cast<float>(bg_value);
                    } else if (dist > 0.5f) {
                        value = static_cast<float>(fg_value);
                    } else {
                        float t = dist + 0.5f;
                        value = bg_value + (fg_value - bg_value) * t;
                    }
                    img.data[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(value);
                }
            }
            break;
        }
        case TestPatternType::Circle: {
            // param1 = 圆心X, param2 = 圆心Y + 半径（用 param1+param2 编码）
            // 约定：param1 = center_x, param2 = center_y, 半径固定 = 50（也可作为参数）
            // 这里我们使用 param1, param2 作为圆心和半径
            // 为简化：圆心 = (param1, param2)，半径 = 50
            float cx = param1;
            float cy = param2;
            float radius = 50.0f;
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    float dx = static_cast<float>(x) - cx;
                    float dy = static_cast<float>(y) - cy;
                    float dist = std::sqrt(dx * dx + dy * dy);
                    float value;
                    if (dist < radius - 0.5f) {
                        value = static_cast<float>(fg_value);
                    } else if (dist > radius + 0.5f) {
                        value = static_cast<float>(bg_value);
                    } else {
                        // 边缘过渡
                        float t = (dist - (radius - 0.5f));  // [0, 1]
                        value = fg_value + (bg_value - fg_value) * t;
                    }
                    img.data[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(value);
                }
            }
            break;
        }
        case TestPatternType::Line: {
            // param1 = 斜率（角度，度），param2 = 截距（Y = param1*x + param2 不对，应该是 y = tan(angle)*x + b）
            // 简化：param1 = 角度（度），param2 = 直线经过中心的偏移
            // 直线方向：angle = param1 度
            // 直线经过点 (w/2 + param2*cos, h/2 + param2*sin) 沿方向延伸
            float angle_rad = param1 * static_cast<float>(M_PI) / 180.0f;
            float offset = param2;
            float cx = w * 0.5f;
            float cy = h * 0.5f;
            // 法线方向
            float nx = -std::sin(angle_rad);
            float ny = std::cos(angle_rad);
            // 直线方程：nx*(x - cx) + ny*(y - cy) = offset
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    float dist = nx * (static_cast<float>(x) - cx) +
                                 ny * (static_cast<float>(y) - cy) - offset;
                    float value;
                    if (dist < -0.5f) {
                        value = static_cast<float>(bg_value);
                    } else if (dist > 0.5f) {
                        value = static_cast<float>(fg_value);
                    } else {
                        float t = dist + 0.5f;
                        value = bg_value + (fg_value - bg_value) * t;
                    }
                    img.data[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(value);
                }
            }
            break;
        }
        case TestPatternType::Corner: {
            // param1 = 角点X, param2 = 角点Y
            // 生成 L 形角点：左上为fg，其它为bg
            float corner_x = param1;
            float corner_y = param2;
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    // L 形：x < corner_x 且 y < corner_y 为 fg
                    float dx = static_cast<float>(x) - corner_x;
                    float dy = static_cast<float>(y) - corner_y;
                    bool in_corner = (dx < 0.5f) && (dy < 0.5f);
                    img.data[static_cast<size_t>(y) * w + x] = in_corner ? fg_value : bg_value;
                }
            }
            // 平滑角点边缘
            // 简化处理：保持角点锐利
            break;
        }
        case TestPatternType::Square: {
            // param1 = 中心X, param2 = 中心Y，绘制 100x100 的方块（4个角点）
            float cx = param1;
            float cy = param2;
            int half_size = 50;
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    if (x >= static_cast<int>(cx) - half_size &&
                        x <  static_cast<int>(cx) + half_size &&
                        y >= static_cast<int>(cy) - half_size &&
                        y <  static_cast<int>(cy) + half_size) {
                        img.data[static_cast<size_t>(y) * w + x] = fg_value;
                    } else {
                        img.data[static_cast<size_t>(y) * w + x] = bg_value;
                    }
                }
            }
            break;
        }
    }

    return img;
}

void add_gaussian_noise(ImageData& image, float snr_db) {
    if (image.empty()) return;

    // 计算信号功率
    double signal_power = 0.0;
    for (uint8_t v : image.data) {
        double dv = static_cast<double>(v);
        signal_power += dv * dv;
    }
    signal_power /= static_cast<double>(image.data.size());

    if (signal_power < 1e-6) return;

    // 计算噪声标准差
    // SNR_dB = 10 * log10(signal_power / noise_power)
    // noise_power = signal_power / 10^(SNR_dB / 10)
    double snr_linear = std::pow(10.0, snr_db / 10.0);
    double noise_power = signal_power / snr_linear;
    double noise_std = std::sqrt(noise_power);

    // Box-Muller 生成高斯噪声
    std::mt19937 rng(42);  // 固定种子，确保可复现
    std::normal_distribution<double> dist(0.0, noise_std);

    for (uint8_t& v : image.data) {
        double noisy = static_cast<double>(v) + dist(rng);
        if (noisy < 0.0) noisy = 0.0;
        if (noisy > 255.0) noisy = 255.0;
        v = static_cast<uint8_t>(noisy);
    }
}

PrecisionStats compute_precision_stats(const std::vector<float>& errors,
                                       float target_precision) {
    PrecisionStats stats;
    stats.target_precision = target_precision;
    stats.sample_count = static_cast<uint32_t>(errors.size());

    if (errors.empty()) {
        stats.evaluate();
        return stats;
    }

    double sum = 0.0;
    double sum_sq = 0.0;
    double max_err = 0.0;
    for (float e : errors) {
        double abs_e = std::abs(static_cast<double>(e));
        sum += abs_e;
        sum_sq += abs_e * abs_e;
        if (abs_e > max_err) max_err = abs_e;
    }

    double n = static_cast<double>(errors.size());
    stats.mean_error = static_cast<float>(sum / n);
    double variance = (sum_sq / n) - (stats.mean_error * stats.mean_error);
    if (variance < 0.0) variance = 0.0;
    stats.std_error = static_cast<float>(std::sqrt(variance));
    stats.max_error = static_cast<float>(max_err);
    stats.rms_error = static_cast<float>(std::sqrt(sum_sq / n));

    stats.evaluate();
    return stats;
}

} // namespace subpixel_utils

// ============================================================================
// 节点实现
// ============================================================================

namespace {

// 解析定位方法
SubpixelMethod parse_method(const String& method_str) {
    if (method_str == "taylor" || method_str == "Taylor") return SubpixelMethod::Taylor;
    if (method_str == "parabola" || method_str == "Parabola") return SubpixelMethod::Parabola;
    if (method_str == "gaussian" || method_str == "Gaussian") return SubpixelMethod::Gaussian;
    if (method_str == "saddle" || method_str == "Saddle") return SubpixelMethod::Saddle;
    if (method_str == "zernike" || method_str == "Zernike") return SubpixelMethod::Zernike;
    return SubpixelMethod::Taylor;
}

// 沿梯度方向亚像素细化
SubpixelEdgePoint refine_edge_point(const ImageData& image,
                                    const std::vector<float>& magnitude,
                                    const std::vector<float>& angle_arr,
                                    uint32_t px, uint32_t py,
                                    SubpixelMethod method) {
    SubpixelEdgePoint point;
    int w = static_cast<int>(image.width);
    int h = static_cast<int>(image.height);

    if (static_cast<int>(px) < 1 || static_cast<int>(px) >= w - 1 ||
        static_cast<int>(py) < 1 || static_cast<int>(py) >= h - 1) {
        return point;
    }

    size_t idx = static_cast<size_t>(py) * w + px;
    float gx = std::cos(angle_arr[idx]);
    float gy = std::sin(angle_arr[idx]);

    // 沿梯度方向取三点（间隔 1 像素）
    float cx = static_cast<float>(px);
    float cy = static_cast<float>(py);
    float px_m = cx - gx, py_m = cy - gy;
    float px_p = cx + gx, py_p = cy + gy;

    // 使用梯度幅值的双线性采样
    auto sample_mag = [&](float x, float y) -> float {
        int x0 = static_cast<int>(std::floor(x));
        int y0 = static_cast<int>(std::floor(y));
        int x1 = x0 + 1, y1 = y0 + 1;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x0 >= w) x0 = w - 1;
        if (y0 >= h) y0 = h - 1;
        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x1 >= w) x1 = w - 1;
        if (y1 >= h) y1 = h - 1;
        float v00 = magnitude[static_cast<size_t>(y0) * w + x0];
        float v01 = magnitude[static_cast<size_t>(y0) * w + x1];
        float v10 = magnitude[static_cast<size_t>(y1) * w + x0];
        float v11 = magnitude[static_cast<size_t>(y1) * w + x1];
        float fx = x - std::floor(x);
        float fy = y - std::floor(y);
        float v0 = v00 + (v01 - v00) * fx;
        float v1 = v10 + (v11 - v10) * fx;
        return v0 + (v1 - v0) * fy;
    };

    float g0 = sample_mag(px_m, py_m);
    float g1 = magnitude[idx];
    float g2 = sample_mag(px_p, py_p);

    float offset = 0.0f;
    switch (method) {
        case SubpixelMethod::Taylor:
            offset = subpixel_utils::taylor_subpixel_offset(g0, g1, g2);
            break;
        case SubpixelMethod::Parabola:
            offset = subpixel_utils::parabola_subpixel_offset(g0, g1, g2);
            break;
        case SubpixelMethod::Gaussian:
            offset = subpixel_utils::gaussian_subpixel_offset(g0, g1, g2);
            break;
        case SubpixelMethod::Zernike: {
            // 取 5x5 邻域灰度值
            float values[25];
            bool ok = true;
            for (int dy = -2; dy <= 2 && ok; ++dy) {
                for (int dx = -2; dx <= 2 && ok; ++dx) {
                    int xx = static_cast<int>(px) + dx;
                    int yy = static_cast<int>(py) + dy;
                    if (xx < 0 || xx >= w || yy < 0 || yy >= h) {
                        ok = false;
                        break;
                    }
                    values[(dy + 2) * 5 + (dx + 2)] =
                        static_cast<float>(image.data[static_cast<size_t>(yy) * w + xx]);
                }
            }
            if (ok) {
                float sub_x, sub_y, ang, str;
                if (subpixel_utils::zernike_subpixel_edge(values, sub_x, sub_y, ang, str)) {
                    point.x = cx + sub_x;
                    point.y = cy + sub_y;
                    point.angle = ang;
                    point.strength = str;
                    point.subpixel_offset = std::sqrt(sub_x * sub_x + sub_y * sub_y);
                    point.valid = true;
                    return point;
                }
            }
            // Zernike 失败时回退到泰勒
            offset = subpixel_utils::taylor_subpixel_offset(g0, g1, g2);
            break;
        }
        case SubpixelMethod::Saddle:
            offset = subpixel_utils::taylor_subpixel_offset(g0, g1, g2);
            break;
    }

    point.x = cx + offset * gx;
    point.y = cy + offset * gy;
    point.strength = g1;
    point.angle = angle_arr[idx];
    point.subpixel_offset = offset;
    point.valid = true;
    return point;
}

} // anonymous namespace

// ============================================================================
// SubpixelEdgeNode - 亚像素边缘定位（泰勒展开法）
// ============================================================================

SubpixelEdgeNode::SubpixelEdgeNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelEdgeNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelEdge";
    info.name = "亚像素边缘定位";
    info.category = "亚像素精度";
    info.description = "亚像素边缘定位（对标Halcon edges_sub_pix），支持 taylor/parabola/gaussian/zernike 方法";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("edge_count", "边缘点数量", DataType::Number));
    info.outputs.push_back(DataPort("first_x", "首个边缘点X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("first_y", "首个边缘点Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("first_strength", "首个边缘点强度", DataType::Number));
    info.outputs.push_back(DataPort("mean_x", "边缘点平均X", DataType::Number));
    info.outputs.push_back(DataPort("mean_y", "边缘点平均Y", DataType::Number));

    info.params.push_back(ParamDef("method", "定位方法（taylor/parabola/gaussian/zernike）",
                                   DataType::String, Data(String("taylor"))));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("max_edges", "最大边缘点数", DataType::Number, Data(1000.0)));

    return info;
}

Result<void> SubpixelEdgeNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    String method_str = get_param("method", Data(String("taylor"))).as_string();
    SubpixelMethod method = parse_method(method_str);
    float threshold = static_cast<float>(get_param("threshold", Data(30.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    int max_edges = get_param("max_edges", Data(1000)).as_int();

    // 平滑
    ImageData smoothed = (sigma > 0.0f) ? subpixel_utils::gaussian_smooth(gray, sigma) : gray;

    // 梯度计算
    std::vector<float> grad_x, grad_y, magnitude, angle;
    subpixel_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude, angle);

    // 非极大值抑制 + 阈值化
    int w = static_cast<int>(smoothed.width);
    int h = static_cast<int>(smoothed.height);

    std::vector<SubpixelEdgePoint> edges;
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            float mag = magnitude[idx];
            if (mag < threshold) continue;

            // 沿梯度方向比较邻居（非极大值抑制）
            float ang = angle[idx];
            float gx = std::cos(ang);
            float gy = std::sin(ang);

            size_t n1_idx = static_cast<size_t>(y + static_cast<int>(std::round(gy))) * w +
                            (x + static_cast<int>(std::round(gx)));
            size_t n2_idx = static_cast<size_t>(y - static_cast<int>(std::round(gy))) * w +
                            (x - static_cast<int>(std::round(gx)));
            if (n1_idx >= magnitude.size() || n2_idx >= magnitude.size()) continue;

            if (mag >= magnitude[n1_idx] && mag >= magnitude[n2_idx]) {
                // 局部极大值，进行亚像素细化
                auto edge = refine_edge_point(smoothed, magnitude, angle,
                                               static_cast<uint32_t>(x), static_cast<uint32_t>(y),
                                               method);
                if (edge.valid) {
                    edges.push_back(edge);
                    if (static_cast<int>(edges.size()) >= max_edges) break;
                }
            }
        }
        if (static_cast<int>(edges.size()) >= max_edges) break;
    }

    // 输出结果
    set_output("edge_count", Data(static_cast<int64_t>(edges.size())));

    if (!edges.empty()) {
        set_output("first_x", Data(static_cast<double>(edges[0].x)));
        set_output("first_y", Data(static_cast<double>(edges[0].y)));
        set_output("first_strength", Data(static_cast<double>(edges[0].strength)));

        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& e : edges) {
            sum_x += e.x;
            sum_y += e.y;
        }
        set_output("mean_x", Data(sum_x / edges.size()));
        set_output("mean_y", Data(sum_y / edges.size()));
    } else {
        set_output("first_x", Data(0.0));
        set_output("first_y", Data(0.0));
        set_output("first_strength", Data(0.0));
        set_output("mean_x", Data(0.0));
        set_output("mean_y", Data(0.0));
    }

    OVF_INFO() << "SubpixelEdge: method=" << method_str
               << ", found " << edges.size() << " edges"
               << " (threshold=" << threshold << ", sigma=" << sigma << ")";

    return Result<void>::success();
}

// ============================================================================
// SubpixelCornerNode - 亚像素角点定位（Saddle点法）
// ============================================================================

SubpixelCornerNode::SubpixelCornerNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelCornerNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelCorner";
    info.name = "亚像素角点定位";
    info.category = "亚像素精度";
    info.description = "亚像素角点定位（Saddle点法，Harris响应 + 二次曲面拟合）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("corner_count", "角点数量", DataType::Number));
    info.outputs.push_back(DataPort("first_x", "首个角点X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("first_y", "首个角点Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("first_response", "首个角点响应值", DataType::Number));

    info.params.push_back(ParamDef("threshold", "角点响应阈值", DataType::Number, Data(1000.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("k", "Harris灵敏度系数", DataType::Number, Data(0.04)));
    info.params.push_back(ParamDef("max_corners", "最大角点数", DataType::Number, Data(100.0)));

    return info;
}

Result<void> SubpixelCornerNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    float threshold = static_cast<float>(get_param("threshold", Data(1000.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    float k = static_cast<float>(get_param("k", Data(0.04)).as_number());
    int max_corners = get_param("max_corners", Data(100)).as_int();

    ImageData smoothed = (sigma > 0.0f) ? subpixel_utils::gaussian_smooth(gray, sigma) : gray;

    // 计算梯度
    std::vector<float> grad_x, grad_y, magnitude, angle;
    subpixel_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude, angle);

    int w = static_cast<int>(smoothed.width);
    int h = static_cast<int>(smoothed.height);

    // 计算Harris响应
    std::vector<float> response(static_cast<size_t>(w) * h, 0.0f);
    for (int y = 2; y < h - 2; ++y) {
        for (int x = 2; x < w - 2; ++x) {
            response[static_cast<size_t>(y) * w + x] =
                subpixel_utils::harris_response(grad_x, grad_y,
                                                smoothed.width, smoothed.height,
                                                static_cast<uint32_t>(x), static_cast<uint32_t>(y),
                                                k, 3);
        }
    }

    // 非极大值抑制 + Saddle点细化
    std::vector<SubpixelCornerPoint> corners;
    for (int y = 2; y < h - 2; ++y) {
        for (int x = 2; x < w - 2; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            float r = response[idx];
            if (r < threshold) continue;

            // 检查是否为局部极大值
            bool is_max = true;
            for (int dy = -1; dy <= 1 && is_max; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (response[static_cast<size_t>(y + dy) * w + (x + dx)] > r) {
                        is_max = false;
                        break;
                    }
                }
            }
            if (!is_max) continue;

            // Saddle点亚像素细化
            float values[9];
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    values[(dy + 1) * 3 + (dx + 1)] =
                        response[static_cast<size_t>(y + dy) * w + (x + dx)];
                }
            }
            float dx_off, dy_off;
            SubpixelCornerPoint corner;
            corner.x = static_cast<float>(x);
            corner.y = static_cast<float>(y);
            corner.response = r;
            if (subpixel_utils::saddle_subpixel_offset(values, dx_off, dy_off)) {
                corner.x += dx_off;
                corner.y += dy_off;
            }
            corner.valid = true;
            corners.push_back(corner);
            if (static_cast<int>(corners.size()) >= max_corners) break;
        }
        if (static_cast<int>(corners.size()) >= max_corners) break;
    }

    // 按响应值排序
    std::sort(corners.begin(), corners.end(),
              [](const SubpixelCornerPoint& a, const SubpixelCornerPoint& b) {
                  return a.response > b.response;
              });

    set_output("corner_count", Data(static_cast<int64_t>(corners.size())));
    if (!corners.empty()) {
        set_output("first_x", Data(static_cast<double>(corners[0].x)));
        set_output("first_y", Data(static_cast<double>(corners[0].y)));
        set_output("first_response", Data(static_cast<double>(corners[0].response)));
    } else {
        set_output("first_x", Data(0.0));
        set_output("first_y", Data(0.0));
        set_output("first_response", Data(0.0));
    }

    OVF_INFO() << "SubpixelCorner: found " << corners.size() << " corners"
               << " (threshold=" << threshold << ", sigma=" << sigma << ")";

    return Result<void>::success();
}

// ============================================================================
// SubpixelLineNode - 亚像素直线定位
// ============================================================================

SubpixelLineNode::SubpixelLineNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelLineNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelLine";
    info.name = "亚像素直线定位";
    info.category = "亚像素精度";
    info.description = "提取亚像素边缘点后，使用最小二乘法拟合直线";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("line_a", "直线系数a (ax+by+c=0)", DataType::Number));
    info.outputs.push_back(DataPort("line_b", "直线系数b", DataType::Number));
    info.outputs.push_back(DataPort("line_c", "直线系数c", DataType::Number));
    info.outputs.push_back(DataPort("rms_error", "拟合均方根误差", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "参与拟合的点数", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否拟合成功", DataType::Boolean));

    info.params.push_back(ParamDef("method", "定位方法", DataType::String, Data(String("taylor"))));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("min_points", "最少拟合点数", DataType::Number, Data(10.0)));

    return info;
}

Result<void> SubpixelLineNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    String method_str = get_param("method", Data(String("taylor"))).as_string();
    SubpixelMethod method = parse_method(method_str);
    float threshold = static_cast<float>(get_param("threshold", Data(30.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    int min_points = get_param("min_points", Data(10)).as_int();

    ImageData smoothed = (sigma > 0.0f) ? subpixel_utils::gaussian_smooth(gray, sigma) : gray;

    std::vector<float> grad_x, grad_y, magnitude, angle;
    subpixel_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude, angle);

    int w = static_cast<int>(smoothed.width);
    int h = static_cast<int>(smoothed.height);

    // 收集亚像素边缘点
    std::vector<Point2Df> points;
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            if (magnitude[idx] < threshold) continue;

            float ang = angle[idx];
            float gx = std::cos(ang);
            float gy = std::sin(ang);

            size_t n1 = static_cast<size_t>(y + static_cast<int>(std::round(gy))) * w +
                        (x + static_cast<int>(std::round(gx)));
            size_t n2 = static_cast<size_t>(y - static_cast<int>(std::round(gy))) * w +
                        (x - static_cast<int>(std::round(gx)));
            if (n1 >= magnitude.size() || n2 >= magnitude.size()) continue;

            if (magnitude[idx] >= magnitude[n1] && magnitude[idx] >= magnitude[n2]) {
                auto edge = refine_edge_point(smoothed, magnitude, angle,
                                               static_cast<uint32_t>(x), static_cast<uint32_t>(y),
                                               method);
                if (edge.valid) {
                    points.emplace_back(edge.x, edge.y);
                }
            }
        }
    }

    SubpixelLineResult result;
    if (static_cast<int>(points.size()) >= min_points) {
        result.line = measurement_utils::fit_line(points);
        // 计算 RMS 误差
        double sum_sq = 0.0;
        for (const auto& p : points) {
            double d = result.line.distance_to_point(p);
            sum_sq += d * d;
        }
        result.rms_error = static_cast<float>(std::sqrt(sum_sq / points.size()));
        result.point_count = static_cast<uint32_t>(points.size());
        result.valid = true;
    }

    set_output("line_a", Data(static_cast<double>(result.line.a)));
    set_output("line_b", Data(static_cast<double>(result.line.b)));
    set_output("line_c", Data(static_cast<double>(result.line.c)));
    set_output("rms_error", Data(static_cast<double>(result.rms_error)));
    set_output("point_count", Data(static_cast<int64_t>(result.point_count)));
    set_output("valid", Data(result.valid));

    OVF_INFO() << "SubpixelLine: " << points.size() << " points, rms=" << result.rms_error
               << ", valid=" << result.valid;

    return Result<void>::success();
}

// ============================================================================
// SubpixelCircleNode - 亚像素圆定位
// ============================================================================

SubpixelCircleNode::SubpixelCircleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelCircleNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelCircle";
    info.name = "亚像素圆定位";
    info.category = "亚像素精度";
    info.description = "提取圆弧亚像素边缘点后，使用最小二乘法拟合圆";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("center_x", "圆心X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "圆心Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("radius", "半径（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("rms_error", "拟合均方根误差", DataType::Number));
    info.outputs.push_back(DataPort("point_count", "参与拟合的点数", DataType::Number));
    info.outputs.push_back(DataPort("valid", "是否拟合成功", DataType::Boolean));

    info.params.push_back(ParamDef("method", "定位方法", DataType::String, Data(String("taylor"))));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("min_points", "最少拟合点数", DataType::Number, Data(20.0)));

    return info;
}

Result<void> SubpixelCircleNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    String method_str = get_param("method", Data(String("taylor"))).as_string();
    SubpixelMethod method = parse_method(method_str);
    float threshold = static_cast<float>(get_param("threshold", Data(30.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    int min_points = get_param("min_points", Data(20)).as_int();

    ImageData smoothed = (sigma > 0.0f) ? subpixel_utils::gaussian_smooth(gray, sigma) : gray;

    std::vector<float> grad_x, grad_y, magnitude, angle;
    subpixel_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude, angle);

    int w = static_cast<int>(smoothed.width);
    int h = static_cast<int>(smoothed.height);

    std::vector<Point2Df> points;
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            if (magnitude[idx] < threshold) continue;

            float ang = angle[idx];
            float gx = std::cos(ang);
            float gy = std::sin(ang);

            size_t n1 = static_cast<size_t>(y + static_cast<int>(std::round(gy))) * w +
                        (x + static_cast<int>(std::round(gx)));
            size_t n2 = static_cast<size_t>(y - static_cast<int>(std::round(gy))) * w +
                        (x - static_cast<int>(std::round(gx)));
            if (n1 >= magnitude.size() || n2 >= magnitude.size()) continue;

            if (magnitude[idx] >= magnitude[n1] && magnitude[idx] >= magnitude[n2]) {
                auto edge = refine_edge_point(smoothed, magnitude, angle,
                                               static_cast<uint32_t>(x), static_cast<uint32_t>(y),
                                               method);
                if (edge.valid) {
                    points.emplace_back(edge.x, edge.y);
                }
            }
        }
    }

    SubpixelCircleResult result;
    if (static_cast<int>(points.size()) >= min_points) {
        result.circle = measurement_utils::fit_circle(points);
        // 计算 RMS 误差
        double sum_sq = 0.0;
        for (const auto& p : points) {
            double dx = p.x - result.circle.center_x;
            double dy = p.y - result.circle.center_y;
            double dist = std::sqrt(dx * dx + dy * dy);
            double err = std::abs(dist - result.circle.radius);
            sum_sq += err * err;
        }
        result.rms_error = static_cast<float>(std::sqrt(sum_sq / points.size()));
        result.point_count = static_cast<uint32_t>(points.size());
        result.valid = result.circle.valid;
    }

    set_output("center_x", Data(static_cast<double>(result.circle.center_x)));
    set_output("center_y", Data(static_cast<double>(result.circle.center_y)));
    set_output("radius", Data(static_cast<double>(result.circle.radius)));
    set_output("rms_error", Data(static_cast<double>(result.rms_error)));
    set_output("point_count", Data(static_cast<int64_t>(result.point_count)));
    set_output("valid", Data(result.valid));

    OVF_INFO() << "SubpixelCircle: " << points.size() << " points, center=("
               << result.circle.center_x << "," << result.circle.center_y
               << "), r=" << result.circle.radius
               << ", rms=" << result.rms_error;

    return Result<void>::success();
}

// ============================================================================
// SubpixelMeasureNode - 亚像素测量（高斯插值）
// ============================================================================

SubpixelMeasureNode::SubpixelMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelMeasureNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelMeasure";
    info.name = "亚像素测量";
    info.category = "亚像素精度";
    info.description = "沿给定路径提取灰度轮廓，使用高斯函数拟合边缘响应极值";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("position", "亚像素位置", DataType::Number));
    info.outputs.push_back(DataPort("strength", "边缘强度", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到边缘", DataType::Boolean));

    info.params.push_back(ParamDef("start_x", "起点X", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("start_y", "起点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("end_x", "终点X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("end_y", "终点Y", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("method", "定位方法（gaussian/parabola/taylor）",
                                   DataType::String, Data(String("gaussian"))));

    return info;
}

Result<void> SubpixelMeasureNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    float start_x = static_cast<float>(get_param("start_x", Data(0.0)).as_number());
    float start_y = static_cast<float>(get_param("start_y", Data(0.0)).as_number());
    float end_x = static_cast<float>(get_param("end_x", Data(100.0)).as_number());
    float end_y = static_cast<float>(get_param("end_y", Data(0.0)).as_number());
    float threshold = static_cast<float>(get_param("threshold", Data(10.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    String method_str = get_param("method", Data(String("gaussian"))).as_string();
    SubpixelMethod method = parse_method(method_str);

    // 沿路径采样
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1.0f) {
        set_output("position", Data(0.0));
        set_output("strength", Data(0.0));
        set_output("found", Data(false));
        return Result<void>::success();
    }

    int num_samples = std::max(2, static_cast<int>(length));
    std::vector<float> profile;
    profile.reserve(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / (num_samples - 1);
        float x = start_x + t * dx;
        float y = start_y + t * dy;
        profile.push_back(subpixel_utils::bilinear_sample(gray, x, y));
    }

    // 计算一阶差分（梯度）
    std::vector<float> gradient(num_samples);
    gradient[0] = profile[1] - profile[0];
    for (int i = 1; i < num_samples - 1; ++i) {
        gradient[i] = (profile[i + 1] - profile[i - 1]) * 0.5f;
    }
    gradient[num_samples - 1] = profile[num_samples - 1] - profile[num_samples - 2];

    // 找最大梯度位置
    int max_idx = 0;
    float max_grad = 0.0f;
    for (int i = 1; i < num_samples - 1; ++i) {
        if (std::abs(gradient[i]) > std::abs(max_grad)) {
            max_grad = gradient[i];
            max_idx = i;
        }
    }

    if (std::abs(max_grad) < threshold || max_idx <= 0 || max_idx >= num_samples - 1) {
        set_output("position", Data(0.0));
        set_output("strength", Data(0.0));
        set_output("found", Data(false));
        OVF_WARN() << "SubpixelMeasure: no edge found (max_grad=" << max_grad << ")";
        return Result<void>::success();
    }

    // 亚像素定位
    float y0 = std::abs(gradient[max_idx - 1]);
    float y1 = std::abs(gradient[max_idx]);
    float y2 = std::abs(gradient[max_idx + 1]);

    float offset = 0.0f;
    switch (method) {
        case SubpixelMethod::Gaussian:
            offset = subpixel_utils::gaussian_subpixel_offset(y0, y1, y2);
            break;
        case SubpixelMethod::Parabola:
            offset = subpixel_utils::parabola_subpixel_offset(y0, y1, y2);
            break;
        default:
            offset = subpixel_utils::taylor_subpixel_offset(y0, y1, y2);
            break;
    }

    float sub_pos = static_cast<float>(max_idx) + offset;
    // 转换为图像坐标
    float t = sub_pos / (num_samples - 1);
    float pos_x = start_x + t * dx;
    float pos_y = start_y + t * dy;
    // 对于水平/垂直路径，输出主要坐标
    float position = (std::abs(dx) >= std::abs(dy)) ? pos_x : pos_y;

    set_output("position", Data(static_cast<double>(position)));
    set_output("strength", Data(static_cast<double>(std::abs(max_grad))));
    set_output("found", Data(true));

    OVF_INFO() << "SubpixelMeasure: found edge at " << position
               << " (strength=" << std::abs(max_grad) << ")";

    return Result<void>::success();
}

// ============================================================================
// SubpixelCaliperNode - 亚像素卡尺（抛物线拟合）
// ============================================================================

SubpixelCaliperNode::SubpixelCaliperNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelCaliperNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelCaliper";
    info.name = "亚像素卡尺";
    info.category = "亚像素精度";
    info.description = "亚像素卡尺（抛物线拟合），在梯度最大值附近取3点拟合抛物线求极值";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("edge_x", "边缘X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("edge_y", "边缘Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("strength", "边缘强度", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到边缘", DataType::Boolean));

    info.params.push_back(ParamDef("roi_x", "ROI中心X", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_y", "ROI中心Y", DataType::Number, Data(100.0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(80.0)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(20.0)));
    info.params.push_back(ParamDef("direction", "搜索方向（度）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(10.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));

    return info;
}

Result<void> SubpixelCaliperNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    float roi_x = static_cast<float>(get_param("roi_x", Data(100.0)).as_number());
    float roi_y = static_cast<float>(get_param("roi_y", Data(100.0)).as_number());
    float roi_w = static_cast<float>(get_param("roi_width", Data(80.0)).as_number());
    float roi_h = static_cast<float>(get_param("roi_height", Data(20.0)).as_number());
    float direction = static_cast<float>(get_param("direction", Data(0.0)).as_number());
    float threshold = static_cast<float>(get_param("threshold", Data(10.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());

    // 沿搜索方向采样
    float dir_rad = direction * static_cast<float>(M_PI) / 180.0f;
    float dir_x = std::cos(dir_rad);
    float dir_y = std::sin(dir_rad);
    float perp_x = -dir_y;
    float perp_y = dir_x;

    int half_w = static_cast<int>(roi_w * 0.5f);
    int half_h = static_cast<int>(roi_h * 0.5f);
    int num_samples = std::max(2, half_w * 2);

    std::vector<float> profile;
    profile.reserve(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i - half_w);
        float cx = roi_x + t * dir_x;
        float cy = roi_y + t * dir_y;
        // 在垂直方向上平均
        float sum = 0.0f;
        int count = 0;
        for (int j = -half_h; j <= half_h; ++j) {
            float px = cx + static_cast<float>(j) * perp_x;
            float py = cy + static_cast<float>(j) * perp_y;
            sum += subpixel_utils::bilinear_sample(gray, px, py);
            ++count;
        }
        profile.push_back((count > 0) ? sum / count : 0.0f);
    }

    // 平滑
    if (sigma > 0.0f) {
        auto kernel = subpixel_utils::gaussian_kernel_1d(sigma);
        int khalf = static_cast<int>(kernel.size()) / 2;
        std::vector<float> smoothed(profile.size(), 0.0f);
        for (int i = 0; i < static_cast<int>(profile.size()); ++i) {
            float sum = 0.0f, wsum = 0.0f;
            for (int k = 0; k < static_cast<int>(kernel.size()); ++k) {
                int idx = i + k - khalf;
                if (idx < 0) idx = 0;
                if (idx >= static_cast<int>(profile.size())) idx = static_cast<int>(profile.size()) - 1;
                sum += profile[idx] * kernel[k];
                wsum += kernel[k];
            }
            smoothed[i] = (wsum > 0.0f) ? sum / wsum : 0.0f;
        }
        profile = smoothed;
    }

    // 计算梯度
    int n = static_cast<int>(profile.size());
    std::vector<float> gradient(n);
    gradient[0] = profile[1] - profile[0];
    for (int i = 1; i < n - 1; ++i) {
        gradient[i] = (profile[i + 1] - profile[i - 1]) * 0.5f;
    }
    gradient[n - 1] = profile[n - 1] - profile[n - 2];

    // 找最大梯度位置
    int max_idx = 0;
    float max_grad = 0.0f;
    for (int i = 1; i < n - 1; ++i) {
        if (std::abs(gradient[i]) > std::abs(max_grad)) {
            max_grad = gradient[i];
            max_idx = i;
        }
    }

    if (std::abs(max_grad) < threshold || max_idx <= 0 || max_idx >= n - 1) {
        set_output("edge_x", Data(0.0));
        set_output("edge_y", Data(0.0));
        set_output("strength", Data(0.0));
        set_output("found", Data(false));
        OVF_WARN() << "SubpixelCaliper: no edge found";
        return Result<void>::success();
    }

    // 抛物线拟合
    float y0 = std::abs(gradient[max_idx - 1]);
    float y1 = std::abs(gradient[max_idx]);
    float y2 = std::abs(gradient[max_idx + 1]);
    float offset = subpixel_utils::parabola_subpixel_offset(y0, y1, y2);

    float sub_pos = static_cast<float>(max_idx) + offset - static_cast<float>(half_w);
    float edge_x = roi_x + sub_pos * dir_x;
    float edge_y = roi_y + sub_pos * dir_y;

    set_output("edge_x", Data(static_cast<double>(edge_x)));
    set_output("edge_y", Data(static_cast<double>(edge_y)));
    set_output("strength", Data(static_cast<double>(std::abs(max_grad))));
    set_output("found", Data(true));

    OVF_INFO() << "SubpixelCaliper: edge at (" << edge_x << "," << edge_y
               << "), strength=" << std::abs(max_grad);

    return Result<void>::success();
}

// ============================================================================
// SubpixelContourNode - 亚像素轮廓提取
// ============================================================================

SubpixelContourNode::SubpixelContourNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo SubpixelContourNode::make_info() {
    NodeInfo info;
    info.id = "SubpixelContour";
    info.name = "亚像素轮廓提取";
    info.category = "亚像素精度";
    info.description = "在边缘点列表基础上，沿梯度方向细化得到亚像素级轮廓";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("contour_count", "轮廓点数量", DataType::Number));
    info.outputs.push_back(DataPort("first_x", "首个轮廓点X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("first_y", "首个轮廓点Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("last_x", "末个轮廓点X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("last_y", "末个轮廓点Y（亚像素）", DataType::Number));

    info.params.push_back(ParamDef("method", "定位方法", DataType::String, Data(String("taylor"))));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("max_points", "最大轮廓点数", DataType::Number, Data(2000.0)));

    return info;
}

Result<void> SubpixelContourNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    String method_str = get_param("method", Data(String("taylor"))).as_string();
    SubpixelMethod method = parse_method(method_str);
    float threshold = static_cast<float>(get_param("threshold", Data(30.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    int max_points = get_param("max_points", Data(2000)).as_int();

    ImageData smoothed = (sigma > 0.0f) ? subpixel_utils::gaussian_smooth(gray, sigma) : gray;

    std::vector<float> grad_x, grad_y, magnitude, angle;
    subpixel_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude, angle);

    int w = static_cast<int>(smoothed.width);
    int h = static_cast<int>(smoothed.height);

    std::vector<SubpixelEdgePoint> contour;
    // 逐行扫描，提取每行的最强边缘点（用于生成单像素宽轮廓）
    for (int y = 1; y < h - 1; ++y) {
        int best_x = -1;
        float best_mag = threshold;
        for (int x = 1; x < w - 1; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            if (magnitude[idx] < best_mag) continue;

            // 非极大值抑制
            float ang = angle[idx];
            float gx = std::cos(ang);
            float gy = std::sin(ang);
            size_t n1 = static_cast<size_t>(y + static_cast<int>(std::round(gy))) * w +
                        (x + static_cast<int>(std::round(gx)));
            size_t n2 = static_cast<size_t>(y - static_cast<int>(std::round(gy))) * w +
                        (x - static_cast<int>(std::round(gx)));
            if (n1 >= magnitude.size() || n2 >= magnitude.size()) continue;

            if (magnitude[idx] >= magnitude[n1] && magnitude[idx] >= magnitude[n2]) {
                best_mag = magnitude[idx];
                best_x = x;
            }
        }
        if (best_x > 0) {
            auto edge = refine_edge_point(smoothed, magnitude, angle,
                                           static_cast<uint32_t>(best_x), static_cast<uint32_t>(y),
                                           method);
            if (edge.valid) {
                contour.push_back(edge);
                if (static_cast<int>(contour.size()) >= max_points) break;
            }
        }
    }

    set_output("contour_count", Data(static_cast<int64_t>(contour.size())));
    if (!contour.empty()) {
        set_output("first_x", Data(static_cast<double>(contour.front().x)));
        set_output("first_y", Data(static_cast<double>(contour.front().y)));
        set_output("last_x", Data(static_cast<double>(contour.back().x)));
        set_output("last_y", Data(static_cast<double>(contour.back().y)));
    } else {
        set_output("first_x", Data(0.0));
        set_output("first_y", Data(0.0));
        set_output("last_x", Data(0.0));
        set_output("last_y", Data(0.0));
    }

    OVF_INFO() << "SubpixelContour: " << contour.size() << " contour points";

    return Result<void>::success();
}

// ============================================================================
// PrecisionTestNode - 精度测试节点（生成标准测试图）
// ============================================================================

PrecisionTestNode::PrecisionTestNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PrecisionTestNode::make_info() {
    NodeInfo info;
    info.id = "PrecisionTest";
    info.name = "精度测试图生成";
    info.category = "亚像素精度";
    info.description = "生成已知参数的合成测试图（圆、直线、边缘），用于精度验证";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像（可选，作为基底）", DataType::Image, false));

    info.outputs.push_back(DataPort("image", "生成的测试图", DataType::Image));
    info.outputs.push_back(DataPort("theoretical_x", "理论边缘位置X", DataType::Number));
    info.outputs.push_back(DataPort("theoretical_y", "理论边缘位置Y", DataType::Number));

    info.params.push_back(ParamDef("pattern_type", "测试图类型（vertical/horizontal/circle/line/corner/square）",
                                  DataType::String, Data(String("vertical"))));
    info.params.push_back(ParamDef("width", "图像宽度", DataType::Number, Data(256.0)));
    info.params.push_back(ParamDef("height", "图像高度", DataType::Number, Data(256.0)));
    info.params.push_back(ParamDef("param1", "主参数（如边缘位置X）", DataType::Number, Data(128.5)));
    info.params.push_back(ParamDef("param2", "副参数（如边缘位置Y/圆心）", DataType::Number, Data(128.0)));
    info.params.push_back(ParamDef("noise_level", "噪声水平（dB，0表示无噪声）", DataType::Number, Data(0.0)));
    info.params.push_back(ParamDef("fg_value", "前景灰度值", DataType::Number, Data(200.0)));
    info.params.push_back(ParamDef("bg_value", "背景灰度值", DataType::Number, Data(50.0)));

    return info;
}

Result<void> PrecisionTestNode::execute(FlowContext& context) {
    String pattern_str = get_param("pattern_type", Data(String("vertical"))).as_string();
    uint32_t width = static_cast<uint32_t>(get_param("width", Data(256.0)).as_number());
    uint32_t height = static_cast<uint32_t>(get_param("height", Data(256.0)).as_number());
    float param1 = static_cast<float>(get_param("param1", Data(128.5)).as_number());
    float param2 = static_cast<float>(get_param("param2", Data(128.0)).as_number());
    float noise_db = static_cast<float>(get_param("noise_level", Data(0.0)).as_number());
    uint8_t fg = static_cast<uint8_t>(get_param("fg_value", Data(200.0)).as_number());
    uint8_t bg = static_cast<uint8_t>(get_param("bg_value", Data(50.0)).as_number());

    TestPatternType type = TestPatternType::VerticalEdge;
    if (pattern_str == "vertical" || pattern_str == "Vertical") type = TestPatternType::VerticalEdge;
    else if (pattern_str == "horizontal" || pattern_str == "Horizontal") type = TestPatternType::HorizontalEdge;
    else if (pattern_str == "circle" || pattern_str == "Circle") type = TestPatternType::Circle;
    else if (pattern_str == "line" || pattern_str == "Line") type = TestPatternType::Line;
    else if (pattern_str == "corner" || pattern_str == "Corner") type = TestPatternType::Corner;
    else if (pattern_str == "square" || pattern_str == "Square") type = TestPatternType::Square;

    ImageData image = subpixel_utils::generate_test_pattern(width, height, type,
                                                              param1, param2, fg, bg);
    if (noise_db > 0.0f) {
        subpixel_utils::add_gaussian_noise(image, noise_db);
    }

    set_output("image", Data(image));
    set_output("theoretical_x", Data(static_cast<double>(param1)));
    set_output("theoretical_y", Data(static_cast<double>(param2)));

    OVF_INFO() << "PrecisionTest: generated " << pattern_str << " pattern "
               << width << "x" << height << " (param1=" << param1
               << ", param2=" << param2 << ", noise=" << noise_db << "dB)";

    return Result<void>::success();
}

// ============================================================================
// PrecisionBenchmarkNode - 精度基准测试（与理论值对比）
// ============================================================================

PrecisionBenchmarkNode::PrecisionBenchmarkNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PrecisionBenchmarkNode::make_info() {
    NodeInfo info;
    info.id = "PrecisionBenchmark";
    info.name = "精度基准测试";
    info.category = "亚像素精度";
    info.description = "在测试图上运行亚像素定位算法，与已知理论值对比，输出统计误差";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入测试图", DataType::Image, true));
    info.inputs.push_back(DataPort("theoretical_x", "理论位置X", DataType::Number, false));
    info.inputs.push_back(DataPort("theoretical_y", "理论位置Y", DataType::Number, false));

    info.outputs.push_back(DataPort("measured_x", "测量位置X（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("measured_y", "测量位置Y（亚像素）", DataType::Number));
    info.outputs.push_back(DataPort("error", "误差（像素）", DataType::Number));
    info.outputs.push_back(DataPort("method", "使用的定位方法", DataType::String));

    info.params.push_back(ParamDef("method", "定位方法", DataType::String, Data(String("taylor"))));
    info.params.push_back(ParamDef("threshold", "边缘强度阈值", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("sigma", "平滑参数", DataType::Number, Data(1.0)));
    info.params.push_back(ParamDef("direction", "测量方向（horizontal/vertical）",
                                   DataType::String, Data(String("horizontal"))));

    return info;
}

Result<void> PrecisionBenchmarkNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }

    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    ImageData gray = subpixel_utils::to_gray(input);

    String method_str = get_param("method", Data(String("taylor"))).as_string();
    SubpixelMethod method = parse_method(method_str);
    float threshold = static_cast<float>(get_param("threshold", Data(30.0)).as_number());
    float sigma = static_cast<float>(get_param("sigma", Data(1.0)).as_number());
    String direction = get_param("direction", Data(String("horizontal"))).as_string();

    float theoretical_x = 0.0f, theoretical_y = 0.0f;
    if (has_input("theoretical_x")) {
        theoretical_x = static_cast<float>(get_input("theoretical_x").as_number());
    }
    if (has_input("theoretical_y")) {
        theoretical_y = static_cast<float>(get_input("theoretical_y").as_number());
    }

    ImageData smoothed = (sigma > 0.0f) ? subpixel_utils::gaussian_smooth(gray, sigma) : gray;

    std::vector<float> grad_x, grad_y, magnitude, angle;
    subpixel_utils::sobel_gradient(smoothed, grad_x, grad_y, magnitude, angle);

    int w = static_cast<int>(smoothed.width);
    int h = static_cast<int>(smoothed.height);

    // 找最强边缘点
    float measured_x = 0.0f, measured_y = 0.0f;
    bool found = false;
    float best_mag = threshold;

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            if (magnitude[idx] < best_mag) continue;

            float ang = angle[idx];
            float gx = std::cos(ang);
            float gy = std::sin(ang);
            size_t n1 = static_cast<size_t>(y + static_cast<int>(std::round(gy))) * w +
                        (x + static_cast<int>(std::round(gx)));
            size_t n2 = static_cast<size_t>(y - static_cast<int>(std::round(gy))) * w +
                        (x - static_cast<int>(std::round(gx)));
            if (n1 >= magnitude.size() || n2 >= magnitude.size()) continue;

            if (magnitude[idx] >= magnitude[n1] && magnitude[idx] >= magnitude[n2]) {
                best_mag = magnitude[idx];
                auto edge = refine_edge_point(smoothed, magnitude, angle,
                                               static_cast<uint32_t>(x), static_cast<uint32_t>(y),
                                               method);
                if (edge.valid) {
                    measured_x = edge.x;
                    measured_y = edge.y;
                    found = true;
                }
            }
        }
    }

    float error_val = 0.0f;
    if (found) {
        if (direction == "horizontal" || direction == "Horizontal") {
            error_val = std::abs(measured_y - theoretical_y);
        } else {
            error_val = std::abs(measured_x - theoretical_x);
        }
    }

    set_output("measured_x", Data(static_cast<double>(measured_x)));
    set_output("measured_y", Data(static_cast<double>(measured_y)));
    set_output("error", Data(static_cast<double>(error_val)));
    set_output("method", Data(method_str));

    OVF_INFO() << "PrecisionBenchmark: measured=(" << measured_x << "," << measured_y
               << "), theoretical=(" << theoretical_x << "," << theoretical_y
               << "), error=" << error_val << " px";

    return Result<void>::success();
}

// ============================================================================
// PrecisionReportNode - 精度报告生成
// ============================================================================

PrecisionReportNode::PrecisionReportNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo PrecisionReportNode::make_info() {
    NodeInfo info;
    info.id = "PrecisionReport";
    info.name = "精度报告生成";
    info.category = "亚像素精度";
    info.description = "汇总多次精度测试结果，输出均值误差、标准差、最大误差，并判定达标";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("errors", "误差样本（数值）", DataType::Number, false));

    info.outputs.push_back(DataPort("mean_error", "平均误差（像素）", DataType::Number));
    info.outputs.push_back(DataPort("std_error", "标准差（像素）", DataType::Number));
    info.outputs.push_back(DataPort("max_error", "最大误差（像素）", DataType::Number));
    info.outputs.push_back(DataPort("rms_error", "均方根误差（像素）", DataType::Number));
    info.outputs.push_back(DataPort("sample_count", "样本数量", DataType::Number));
    info.outputs.push_back(DataPort("pass", "是否达标", DataType::Boolean));
    info.outputs.push_back(DataPort("report", "文本报告", DataType::String));

    info.params.push_back(ParamDef("target_precision", "目标精度（像素）",
                                  DataType::Number, Data(0.01)));
    info.params.push_back(ParamDef("sample_count", "预期样本数", DataType::Number, Data(100.0)));

    return info;
}

Result<void> PrecisionReportNode::execute(FlowContext& context) {
    float target = static_cast<float>(get_param("target_precision", Data(0.01)).as_number());

    // 从变量中收集误差样本（从 FlowContext 全局变量读取）
    // 由于本节点接收单个数值输入，我们使用一个简单的累加机制
    // 通过 FlowContext 全局变量 "errors_collected" 来累计
    std::vector<float> errors;
    if (has_input("errors")) {
        float err = static_cast<float>(get_input("errors").as_number());
        errors.push_back(err);
    }

    // 从上下文获取已收集的误差列表
    String errors_key = "_precision_errors_" + instance_id_;
    auto var = context.get_variable(errors_key, Data(0));
    int count = var.as_int();
    for (int i = 0; i < count && i < 10000; ++i) {
        String key = errors_key + "_" + std::to_string(i);
        auto e_var = context.get_variable(key, Data(0.0));
        errors.push_back(static_cast<float>(e_var.as_number()));
    }

    // 添加当前误差并保存
    if (has_input("errors")) {
        String key = errors_key + "_" + std::to_string(count);
        context.set_variable(key, get_input("errors"));
        context.set_variable(errors_key, Data(count + 1));
        errors.push_back(static_cast<float>(get_input("errors").as_number()));
    }

    // 计算统计
    PrecisionStats stats = subpixel_utils::compute_precision_stats(errors, target);

    // 生成文本报告
    std::ostringstream oss;
    oss << "===== 亚像素精度报告 =====\n";
    oss << "样本数量:    " << stats.sample_count << "\n";
    oss << "目标精度:    " << stats.target_precision << " 像素\n";
    oss << "平均误差:    " << stats.mean_error << " 像素\n";
    oss << "标准差:      " << stats.std_error << " 像素\n";
    oss << "最大误差:    " << stats.max_error << " 像素\n";
    oss << "RMS误差:     " << stats.rms_error << " 像素\n";
    oss << "是否达标:    " << (stats.pass ? "是" : "否") << "\n";
    oss << "==========================";

    set_output("mean_error", Data(static_cast<double>(stats.mean_error)));
    set_output("std_error", Data(static_cast<double>(stats.std_error)));
    set_output("max_error", Data(static_cast<double>(stats.max_error)));
    set_output("rms_error", Data(static_cast<double>(stats.rms_error)));
    set_output("sample_count", Data(static_cast<int64_t>(stats.sample_count)));
    set_output("pass", Data(stats.pass));
    set_output("report", Data(oss.str()));

    OVF_INFO() << "PrecisionReport: " << stats.sample_count << " samples"
               << ", mean=" << stats.mean_error
               << ", max=" << stats.max_error
               << ", pass=" << (stats.pass ? "YES" : "NO");

    return Result<void>::success();
}

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(SubpixelEdgeNode, "SubpixelEdge", SubpixelEdgeNode::make_info())
OVF_REGISTER_NODE(SubpixelCornerNode, "SubpixelCorner", SubpixelCornerNode::make_info())
OVF_REGISTER_NODE(SubpixelLineNode, "SubpixelLine", SubpixelLineNode::make_info())
OVF_REGISTER_NODE(SubpixelCircleNode, "SubpixelCircle", SubpixelCircleNode::make_info())
OVF_REGISTER_NODE(SubpixelMeasureNode, "SubpixelMeasure", SubpixelMeasureNode::make_info())
OVF_REGISTER_NODE(SubpixelCaliperNode, "SubpixelCaliper", SubpixelCaliperNode::make_info())
OVF_REGISTER_NODE(SubpixelContourNode, "SubpixelContour", SubpixelContourNode::make_info())
OVF_REGISTER_NODE(PrecisionTestNode, "PrecisionTest", PrecisionTestNode::make_info())
OVF_REGISTER_NODE(PrecisionBenchmarkNode, "PrecisionBenchmark", PrecisionBenchmarkNode::make_info())
OVF_REGISTER_NODE(PrecisionReportNode, "PrecisionReport", PrecisionReportNode::make_info())

} // namespace algorithm
} // namespace ovf
