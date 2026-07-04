#include "ovf/algorithm/image_utils.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace ovf {
namespace algorithm {

namespace image_utils {

void draw_rect(ImageData& image, int x, int y, int width, int height, uint8_t r, uint8_t g, uint8_t b) {
    if (image.data.empty()) return;

    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;

    // 边界检查
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + width > img_w) width = img_w - x;
    if (y + height > img_h) height = img_h - y;

    // 绘制矩形边框
    for (int i = 0; i < width; ++i) {
        // 上边
        if (y >= 0 && y < img_h && x + i >= 0 && x + i < img_w) {
            int idx = (y * img_w + (x + i)) * ch;
            if (ch >= 3) {
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            } else {
                image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
            }
        }
        // 下边
        if (y + height - 1 >= 0 && y + height - 1 < img_h && x + i >= 0 && x + i < img_w) {
            int idx = ((y + height - 1) * img_w + (x + i)) * ch;
            if (ch >= 3) {
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            } else {
                image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
            }
        }
    }

    for (int i = 0; i < height; ++i) {
        // 左边
        if (y + i >= 0 && y + i < img_h && x >= 0 && x < img_w) {
            int idx = ((y + i) * img_w + x) * ch;
            if (ch >= 3) {
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            } else {
                image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
            }
        }
        // 右边
        if (y + i >= 0 && y + i < img_h && x + width - 1 >= 0 && x + width - 1 < img_w) {
            int idx = ((y + i) * img_w + (x + width - 1)) * ch;
            if (ch >= 3) {
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            } else {
                image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
            }
        }
    }
}

void draw_circle(ImageData& image, int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b) {
    if (image.data.empty() || radius <= 0) return;

    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;

    // Bresenham算法绘制圆形
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        // 绘制八个对称点
        auto draw_point = [&](int px, int py) {
            if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
                int idx = (py * img_w + px) * ch;
                if (ch >= 3) {
                    image.data[idx] = r;
                    image.data[idx + 1] = g;
                    image.data[idx + 2] = b;
                } else {
                    image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
                }
            }
        };

        draw_point(cx + x, cy + y);
        draw_point(cx + y, cy + x);
        draw_point(cx - y, cy + x);
        draw_point(cx - x, cy + y);
        draw_point(cx - x, cy - y);
        draw_point(cx - y, cy - x);
        draw_point(cx + y, cy - x);
        draw_point(cx + x, cy - y);

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

void draw_line(ImageData& image, int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b) {
    if (image.data.empty()) return;

    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;

    // Bresenham算法绘制直线
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    int x = x1, y = y1;

    while (true) {
        if (x >= 0 && x < img_w && y >= 0 && y < img_h) {
            int idx = (y * img_w + x) * ch;
            if (ch >= 3) {
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            } else {
                image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
            }
        }

        if (x == x2 && y == y2) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

void draw_point(ImageData& image, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (image.data.empty()) return;

    int img_w = image.width;
    int img_h = image.height;
    int ch = image.channels;

    if (x >= 0 && x < img_w && y >= 0 && y < img_h) {
        int idx = (y * img_w + x) * ch;
        if (ch >= 3) {
            image.data[idx] = r;
            image.data[idx + 1] = g;
            image.data[idx + 2] = b;
        } else {
            image.data[idx] = static_cast<uint8_t>((r + g + b) / 3);
        }
    }
}

void gray_to_rgb(const uint8_t* gray, int width, int height, std::vector<uint8_t>& rgb) {
    if (!gray) return;

    rgb.resize(width * height * 3);
    for (int i = 0; i < width * height; ++i) {
        rgb[i * 3] = gray[i];
        rgb[i * 3 + 1] = gray[i];
        rgb[i * 3 + 2] = gray[i];
    }
}

void rgb_to_gray(const uint8_t* rgb, int width, int height, std::vector<uint8_t>& gray) {
    if (!rgb) return;

    gray.resize(width * height);
    for (int i = 0; i < width * height; ++i) {
        // 使用标准灰度转换公式
        gray[i] = static_cast<uint8_t>(
            0.299 * rgb[i * 3] + 0.587 * rgb[i * 3 + 1] + 0.114 * rgb[i * 3 + 2]);
    }
}

void flip(const uint8_t* src, int width, int height, int channels,
          std::vector<uint8_t>& dst, bool horizontal, bool vertical) {
    if (!src) return;

    dst.resize(width * height * channels);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int src_x = horizontal ? (width - 1 - x) : x;
            int src_y = vertical ? (height - 1 - y) : y;

            int dst_idx = (y * width + x) * channels;
            int src_idx = (src_y * width + src_x) * channels;

            for (int c = 0; c < channels; ++c) {
                dst[dst_idx + c] = src[src_idx + c];
            }
        }
    }
}

void resize_nearest(const uint8_t* src, int src_w, int src_h, int channels,
                    std::vector<uint8_t>& dst, int dst_w, int dst_h) {
    if (!src) return;

    dst.resize(dst_w * dst_h * channels);

    double scale_x = static_cast<double>(src_w) / dst_w;
    double scale_y = static_cast<double>(src_h) / dst_h;

    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            int src_x = static_cast<int>(x * scale_x);
            int src_y = static_cast<int>(y * scale_y);

            // 边界检查
            if (src_x >= src_w) src_x = src_w - 1;
            if (src_y >= src_h) src_y = src_h - 1;

            int dst_idx = (y * dst_w + x) * channels;
            int src_idx = (src_y * src_w + src_x) * channels;

            for (int c = 0; c < channels; ++c) {
                dst[dst_idx + c] = src[src_idx + c];
            }
        }
    }
}

void resize_bilinear(const uint8_t* src, int src_w, int src_h, int channels,
                     std::vector<uint8_t>& dst, int dst_w, int dst_h) {
    if (!src) return;

    dst.resize(dst_w * dst_h * channels);

    double scale_x = static_cast<double>(src_w - 1) / (dst_w - 1);
    double scale_y = static_cast<double>(src_h - 1) / (dst_h - 1);

    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            double src_x = x * scale_x;
            double src_y = y * scale_y;

            int x0 = static_cast<int>(src_x);
            int y0 = static_cast<int>(src_y);
            int x1 = std::min(x0 + 1, src_w - 1);
            int y1 = std::min(y0 + 1, src_h - 1);

            double fx = src_x - x0;
            double fy = src_y - y0;

            int dst_idx = (y * dst_w + x) * channels;

            for (int c = 0; c < channels; ++c) {
                double v00 = src[(y0 * src_w + x0) * channels + c];
                double v01 = src[(y0 * src_w + x1) * channels + c];
                double v10 = src[(y1 * src_w + x0) * channels + c];
                double v11 = src[(y1 * src_w + x1) * channels + c];

                double v = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v01 +
                           (1 - fx) * fy * v10 + fx * fy * v11;

                dst[dst_idx + c] = static_cast<uint8_t>(std::clamp(v, 0.0, 255.0));
            }
        }
    }
}

void build_pyramid(const uint8_t* src, int width, int height, int channels,
                   std::vector<std::vector<uint8_t>>& pyramid, int levels) {
    if (!src || levels < 1) return;

    pyramid.clear();
    pyramid.reserve(levels);

    // 第0层是原始图像
    pyramid.push_back(std::vector<uint8_t>(src, src + width * height * channels));

    int cur_w = width;
    int cur_h = height;

    for (int level = 1; level < levels; ++level) {
        int next_w = cur_w / 2;
        int next_h = cur_h / 2;

        if (next_w < 1 || next_h < 1) break;

        std::vector<uint8_t> next_level;
        resize_bilinear(pyramid[level - 1].data(), cur_w, cur_h, channels,
                        next_level, next_w, next_h);

        pyramid.push_back(next_level);
        cur_w = next_w;
        cur_h = next_h;
    }
}

void split_channels(const uint8_t* src, int width, int height, int channels,
                    std::vector<std::vector<uint8_t>>& separated) {
    if (!src) return;

    separated.clear();
    separated.resize(channels);

    for (int c = 0; c < channels; ++c) {
        separated[c].resize(width * height);
    }

    for (int i = 0; i < width * height; ++i) {
        for (int c = 0; c < channels; ++c) {
            separated[c][i] = src[i * channels + c];
        }
    }
}

void merge_channels(const std::vector<std::vector<uint8_t>>& channels,
                    int width, int height, std::vector<uint8_t>& merged) {
    if (channels.empty()) return;

    int num_channels = static_cast<int>(channels.size());
    merged.resize(width * height * num_channels);

    for (int i = 0; i < width * height; ++i) {
        for (int c = 0; c < num_channels; ++c) {
            merged[i * num_channels + c] = channels[c][i];
        }
    }
}

void calc_histogram(const uint8_t* src, int size, std::vector<int>& histogram, int bins) {
    if (!src || bins <= 0) return;

    histogram.clear();
    histogram.resize(bins, 0);

    for (int i = 0; i < size; ++i) {
        int bin = static_cast<int>(src[i] * bins / 256.0);
        if (bin >= bins) bin = bins - 1;
        histogram[bin]++;
    }
}

void histogram_equalize(uint8_t* src, int size) {
    if (!src) return;

    // 计算直方图
    std::vector<int> hist(256, 0);
    for (int i = 0; i < size; ++i) {
        hist[src[i]]++;
    }

    // 计算累积分布函数
    std::vector<int> cdf(256);
    cdf[0] = hist[0];
    for (int i = 1; i < 256; ++i) {
        cdf[i] = cdf[i - 1] + hist[i];
    }

    // 找到最小非零CDF值
    int cdf_min = 0;
    for (int i = 0; i < 256; ++i) {
        if (cdf[i] > 0) {
            cdf_min = cdf[i];
            break;
        }
    }

    // 应用均衡化
    for (int i = 0; i < size; ++i) {
        int val = src[i];
        src[i] = static_cast<uint8_t>(
            std::clamp(static_cast<double>(cdf[val] - cdf_min) / (size - cdf_min) * 255, 0.0, 255.0));
    }
}

void normalize(uint8_t* src, int size, uint8_t min_val, uint8_t max_val) {
    if (!src || size == 0) return;

    // 找到最小和最大值
    uint8_t cur_min = 255, cur_max = 0;
    for (int i = 0; i < size; ++i) {
        cur_min = std::min(cur_min, src[i]);
        cur_max = std::max(cur_max, src[i]);
    }

    if (cur_max == cur_min) {
        // 所有值相同，设为中间值
        for (int i = 0; i < size; ++i) {
            src[i] = (min_val + max_val) / 2;
        }
        return;
    }

    // 归一化
    double scale = static_cast<double>(max_val - min_val) / (cur_max - cur_min);
    for (int i = 0; i < size; ++i) {
        src[i] = static_cast<uint8_t>(min_val + (src[i] - cur_min) * scale);
    }
}

void calc_mean_std(const uint8_t* src, int size, double& mean, double& std_dev) {
    if (!src || size == 0) {
        mean = 0.0;
        std_dev = 0.0;
        return;
    }

    // 计算均值
    double sum = 0.0;
    for (int i = 0; i < size; ++i) {
        sum += src[i];
    }
    mean = sum / size;

    // 计算方差
    double var = 0.0;
    for (int i = 0; i < size; ++i) {
        double diff = src[i] - mean;
        var += diff * diff;
    }
    std_dev = std::sqrt(var / size);
}

void add(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(a[i]) + static_cast<int>(b[i]), 0, 255));
    }
}

void subtract(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(a[i]) - static_cast<int>(b[i]), 0, 255));
    }
}

void multiply(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(a[i]) * static_cast<int>(b[i]) / 255, 0, 255));
    }
}

void divide(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = static_cast<uint8_t>(std::clamp(b[i] > 0 ? static_cast<int>(a[i]) * 255 / static_cast<int>(b[i]) : 255, 0, 255));
    }
}

void bitwise_and(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = a[i] & b[i];
    }
}

void bitwise_or(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = a[i] | b[i];
    }
}

void bitwise_xor(const uint8_t* a, const uint8_t* b, int size, std::vector<uint8_t>& result) {
    if (!a || !b) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = a[i] ^ b[i];
    }
}

void bitwise_not(const uint8_t* a, int size, std::vector<uint8_t>& result) {
    if (!a) return;

    result.resize(size);
    for (int i = 0; i < size; ++i) {
        result[i] = ~a[i];
    }
}

} // namespace image_utils

} // namespace algorithm
} // namespace ovf