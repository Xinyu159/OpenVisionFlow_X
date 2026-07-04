/**
 * @file chinese_ocr.cpp
 * @brief 印刷体汉字 OCR 识别模块实现
 *
 * 纯 C++ 实现，不依赖 Tesseract / OpenCV。
 */

#include "ovf/algorithm/chinese_ocr.h"
#include "ovf/core/logger.h"
#include "nlohmann/json.hpp"

#include <cmath>
#include <cctype>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <cstring>
#include <queue>
#include <utility>

namespace ovf {
namespace algorithm {

// ============================================================================
// 内部辅助函数
// ============================================================================

namespace {

// 像素访问辅助（灰度图，单通道 uint8）
inline uint8_t gray_at(const ImageData& img, int x, int y) {
    if (img.channels == 1) {
        return img.data[static_cast<size_t>(y * img.width) + x];
    }
    // 多通道取平均
    size_t idx = (static_cast<size_t>(y) * img.width + x) * img.channels;
    int sum = 0;
    for (uint32_t c = 0; c < img.channels; ++c) {
        sum += img.data[idx + c];
    }
    return static_cast<uint8_t>(sum / static_cast<int>(img.channels));
}

inline void gray_set(ImageData& img, int x, int y, uint8_t v) {
    if (img.channels == 1) {
        img.data[static_cast<size_t>(y * img.width) + x] = v;
    } else {
        size_t idx = (static_cast<size_t>(y) * img.width + x) * img.channels;
        for (uint32_t c = 0; c < img.channels; ++c) {
            img.data[idx + c] = v;
        }
    }
}

// RGB 转灰度
inline uint8_t rgb_to_gray(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
}

// 字符框裁剪
CharBox make_char_box(int x, int y, int w, int h) {
    return CharBox(static_cast<int32_t>(x), static_cast<int32_t>(y),
                   static_cast<int32_t>(w), static_cast<int32_t>(h));
}

} // anonymous namespace

// ============================================================================
// OCRModel 方法实现
// ============================================================================

OCRTemplate* OCRModel::find(const String& label) {
    for (auto& t : templates) {
        if (t.label == label) return &t;
    }
    return nullptr;
}

const OCRTemplate* OCRModel::find(const String& label) const {
    for (const auto& t : templates) {
        if (t.label == label) return &t;
    }
    return nullptr;
}

void OCRModel::add_template(const String& label, CharSet cs, const FeatureVector& feat) {
    OCRTemplate* existing = find(label);
    if (existing) {
        // 增量更新：平均特征
        int n = existing->sample_count;
        int new_n = n + 1;
        for (size_t i = 0; i < existing->feature.size() && i < feat.size(); ++i) {
            existing->feature[i] = (existing->feature[i] * n + feat[i]) / new_n;
        }
        existing->sample_count = new_n;
    } else {
        templates.emplace_back(label, cs, feat);
    }
}

// ============================================================================
// 图像预处理
// ============================================================================

ErrorCode chinese_ocr_utils::to_gray(const ImageData& src, ImageData& gray) {
    if (src.empty()) return ErrorCode::InvalidImage;
    if (src.channels == 1) {
        gray = src;
        gray.format = ImageFormat::Mono8;
        return ErrorCode::Success;
    }

    gray.width = src.width;
    gray.height = src.height;
    gray.channels = 1;
    gray.format = ImageFormat::Mono8;
    gray.data.resize(static_cast<size_t>(src.width) * src.height);

    for (uint32_t y = 0; y < src.height; ++y) {
        for (uint32_t x = 0; x < src.width; ++x) {
            size_t idx = (static_cast<size_t>(y) * src.width + x) * src.channels;
            uint8_t v = 0;
            if (src.format == ImageFormat::RGB8 || src.format == ImageFormat::BGR8) {
                v = rgb_to_gray(src.data[idx], src.data[idx + 1], src.data[idx + 2]);
            } else if (src.format == ImageFormat::RGBA8 || src.format == ImageFormat::BGRA8) {
                v = rgb_to_gray(src.data[idx], src.data[idx + 1], src.data[idx + 2]);
            } else {
                // 通用：取平均
                int sum = 0;
                for (uint32_t c = 0; c < src.channels; ++c) sum += src.data[idx + c];
                v = static_cast<uint8_t>(sum / static_cast<int>(src.channels));
            }
            gray.data[static_cast<size_t>(y) * src.width + x] = v;
        }
    }
    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::threshold_otsu(const ImageData& gray, ImageData& binary) {
    if (gray.empty() || gray.channels != 1) return ErrorCode::InvalidImage;

    // 直方图
    int hist[256] = {0};
    for (uint8_t v : gray.data) hist[v]++;

    int total = static_cast<int>(gray.data.size());
    float sum = 0;
    for (int i = 0; i < 256; ++i) sum += i * hist[i];

    float sumB = 0;
    int wB = 0;
    float max_var = -1.0f;
    int threshold = 127;

    for (int i = 0; i < 256; ++i) {
        wB += hist[i];
        if (wB == 0) continue;
        int wF = total - wB;
        if (wF == 0) break;

        sumB += static_cast<float>(i * hist[i]);
        float mB = sumB / wB;
        float mF = (sum - sumB) / wF;
        float var = static_cast<float>(wB) * static_cast<float>(wF) * (mB - mF) * (mB - mF);
        if (var > max_var) {
            max_var = var;
            threshold = i;
        }
    }

    binary.width = gray.width;
    binary.height = gray.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(gray.data.size());

    for (size_t i = 0; i < gray.data.size(); ++i) {
        // 文本通常为深色（低值），故低于阈值置 255（白底黑字归一为前景=255）
        binary.data[i] = gray.data[i] < threshold ? 255 : 0;
    }
    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::threshold_adaptive(const ImageData& gray, ImageData& binary, int block_size) {
    if (gray.empty() || gray.channels != 1) return ErrorCode::InvalidImage;
    if (block_size < 3) block_size = 3;
    if (block_size % 2 == 0) block_size += 1;

    binary.width = gray.width;
    binary.height = gray.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(gray.data.size());

    int half = block_size / 2;
    int C = 5; // 偏移常数

    for (uint32_t y = 0; y < gray.height; ++y) {
        for (uint32_t x = 0; x < gray.width; ++x) {
            // 计算局部均值
            long sum = 0;
            int count = 0;
            for (int dy = -half; dy <= half; ++dy) {
                int yy = static_cast<int>(y) + dy;
                if (yy < 0 || yy >= static_cast<int>(gray.height)) continue;
                for (int dx = -half; dx <= half; ++dx) {
                    int xx = static_cast<int>(x) + dx;
                    if (xx < 0 || xx >= static_cast<int>(gray.width)) continue;
                    sum += gray.data[static_cast<size_t>(yy) * gray.width + xx];
                    ++count;
                }
            }
            int local_mean = count > 0 ? static_cast<int>(sum / count) : 0;
            uint8_t v = gray.data[static_cast<size_t>(y) * gray.width + x];
            binary.data[static_cast<size_t>(y) * gray.width + x] =
                (v < local_mean - C) ? 255 : 0;
        }
    }
    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::resize_image(const ImageData& src, ImageData& dst, int new_w, int new_h) {
    if (src.empty() || new_w <= 0 || new_h <= 0) return ErrorCode::InvalidParameter;

    dst.width = static_cast<uint32_t>(new_w);
    dst.height = static_cast<uint32_t>(new_h);
    dst.channels = src.channels;
    dst.format = src.format;
    dst.data.resize(static_cast<size_t>(new_w) * new_h * src.channels);

    float sx = static_cast<float>(src.width) / new_w;
    float sy = static_cast<float>(src.height) / new_h;

    for (int y = 0; y < new_h; ++y) {
        for (int x = 0; x < new_w; ++x) {
            float src_x = (x + 0.5f) * sx - 0.5f;
            float src_y = (y + 0.5f) * sy - 0.5f;

            int x0 = static_cast<int>(std::floor(src_x));
            int y0 = static_cast<int>(std::floor(src_y));
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            x0 = std::max(0, std::min(x0, static_cast<int>(src.width) - 1));
            y0 = std::max(0, std::min(y0, static_cast<int>(src.height) - 1));
            x1 = std::max(0, std::min(x1, static_cast<int>(src.width) - 1));
            y1 = std::max(0, std::min(y1, static_cast<int>(src.height) - 1));

            float fx = src_x - std::floor(src_x);
            float fy = src_y - std::floor(src_y);
            if (fx < 0) fx = 0;
            if (fy < 0) fy = 0;

            for (uint32_t c = 0; c < src.channels; ++c) {
                size_t i00 = (static_cast<size_t>(y0) * src.width + x0) * src.channels + c;
                size_t i01 = (static_cast<size_t>(y0) * src.width + x1) * src.channels + c;
                size_t i10 = (static_cast<size_t>(y1) * src.width + x0) * src.channels + c;
                size_t i11 = (static_cast<size_t>(y1) * src.width + x1) * src.channels + c;

                float v = (1 - fx) * (1 - fy) * src.data[i00] +
                          fx * (1 - fy) * src.data[i01] +
                          (1 - fx) * fy * src.data[i10] +
                          fx * fy * src.data[i11];
                dst.data[(static_cast<size_t>(y) * new_w + x) * src.channels + c] =
                    static_cast<uint8_t>(std::round(std::max(0.0f, std::min(255.0f, v))));
            }
        }
    }
    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::crop_image(const ImageData& src, ImageData& dst, int x, int y, int w, int h) {
    if (src.empty()) return ErrorCode::InvalidImage;
    if (w <= 0 || h <= 0) return ErrorCode::InvalidParameter;

    // 边界裁剪
    int x0 = std::max(0, x);
    int y0 = std::max(0, y);
    int x1 = std::min(static_cast<int>(src.width), x + w);
    int y1 = std::min(static_cast<int>(src.height), y + h);
    if (x1 <= x0 || y1 <= y0) return ErrorCode::OutOfRange;

    int aw = x1 - x0;
    int ah = y1 - y0;

    dst.width = static_cast<uint32_t>(aw);
    dst.height = static_cast<uint32_t>(ah);
    dst.channels = src.channels;
    dst.format = src.format;
    dst.data.resize(static_cast<size_t>(aw) * ah * src.channels);

    for (int dy = 0; dy < ah; ++dy) {
        for (int dx = 0; dx < aw; ++dx) {
            for (uint32_t c = 0; c < src.channels; ++c) {
                size_t src_idx = ((static_cast<size_t>(y0 + dy) * src.width) + (x0 + dx)) * src.channels + c;
                size_t dst_idx = (static_cast<size_t>(dy) * aw + dx) * src.channels + c;
                dst.data[dst_idx] = src.data[src_idx];
            }
        }
    }
    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::preprocess_char(const ImageData& src, ImageData& dst, int target_size) {
    if (src.empty()) return ErrorCode::InvalidImage;

    // 转灰度
    ImageData gray;
    ErrorCode code = to_gray(src, gray);
    if (code != ErrorCode::Success) return code;

    // 找字符前景区域（非零像素的包围盒）
    int min_x = static_cast<int>(gray.width);
    int min_y = static_cast<int>(gray.height);
    int max_x = 0;
    int max_y = 0;
    bool found = false;

    for (uint32_t y = 0; y < gray.height; ++y) {
        for (uint32_t x = 0; x < gray.width; ++x) {
            if (gray.data[static_cast<size_t>(y) * gray.width + x] > 50) {
                if (static_cast<int>(x) < min_x) min_x = static_cast<int>(x);
                if (static_cast<int>(y) < min_y) min_y = static_cast<int>(y);
                if (static_cast<int>(x) > max_x) max_x = static_cast<int>(x);
                if (static_cast<int>(y) > max_y) max_y = static_cast<int>(y);
                found = true;
            }
        }
    }

    if (!found) {
        min_x = min_y = 0;
        max_x = static_cast<int>(gray.width) - 1;
        max_y = static_cast<int>(gray.height) - 1;
    }

    int cw = std::max(1, max_x - min_x + 1);
    int ch = std::max(1, max_y - min_y + 1);

    // 裁剪字符区域
    ImageData cropped;
    code = crop_image(gray, cropped, min_x, min_y, cw, ch);
    if (code != ErrorCode::Success) return code;

    // 等比缩放到 target_size，并居中填充到 target_size x target_size
    float scale = static_cast<float>(target_size - 2) / std::max(cw, ch);
    int new_w = std::max(1, static_cast<int>(cw * scale));
    int new_h = std::max(1, static_cast<int>(ch * scale));

    ImageData resized;
    code = resize_image(cropped, resized, new_w, new_h);
    if (code != ErrorCode::Success) return code;

    // 居中放置
    dst.width = static_cast<uint32_t>(target_size);
    dst.height = static_cast<uint32_t>(target_size);
    dst.channels = 1;
    dst.format = ImageFormat::Mono8;
    dst.data.assign(static_cast<size_t>(target_size) * target_size, 0);

    int off_x = (target_size - new_w) / 2;
    int off_y = (target_size - new_h) / 2;

    for (int y = 0; y < new_h; ++y) {
        for (int x = 0; x < new_w; ++x) {
            int dx = off_x + x;
            int dy = off_y + y;
            if (dx >= 0 && dx < target_size && dy >= 0 && dy < target_size) {
                dst.data[static_cast<size_t>(dy) * target_size + dx] =
                    resized.data[static_cast<size_t>(y) * new_w + x];
            }
        }
    }
    return ErrorCode::Success;
}

// ============================================================================
// MSER 文本检测
// ============================================================================

namespace {

// 连通域标记（4 邻域），返回每个连通域的包围盒和像素数
struct ConnectedComp {
    int x = 0, y = 0, w = 0, h = 0;
    int area = 0;
    Vector<Point2D<int>> pixels;
};

void label_connected(const ImageData& binary, uint8_t target_val,
                     Vector<ConnectedComp>& comps) {
    int w = static_cast<int>(binary.width);
    int h = static_cast<int>(binary.height);
    Vector<uint8_t> visited(w * h, 0);

    int dx4[4] = {1, -1, 0, 0};
    int dy4[4] = {0, 0, 1, -1};

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            size_t idx = static_cast<size_t>(y) * w + x;
            if (visited[idx]) continue;
            if (binary.data[idx] != target_val) continue;

            // BFS
            ConnectedComp comp;
            int min_x = x, max_x = x, min_y = y, max_y = y;
            std::queue<std::pair<int, int>> q;
            q.push({x, y});
            visited[idx] = 1;

            while (!q.empty()) {
                auto cur = q.front();
                q.pop();
                int cx = cur.first;
                int cy = cur.second;
                comp.pixels.emplace_back(cx, cy);
                comp.area++;
                if (cx < min_x) min_x = cx;
                if (cx > max_x) max_x = cx;
                if (cy < min_y) min_y = cy;
                if (cy > max_y) max_y = cy;

                for (int k = 0; k < 4; ++k) {
                    int nx = cx + dx4[k];
                    int ny = cy + dy4[k];
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    size_t nidx = static_cast<size_t>(ny) * w + nx;
                    if (visited[nidx]) continue;
                    if (binary.data[nidx] != target_val) continue;
                    visited[nidx] = 1;
                    q.push({nx, ny});
                }
            }

            comp.x = min_x;
            comp.y = min_y;
            comp.w = max_x - min_x + 1;
            comp.h = max_y - min_y + 1;
            comps.push_back(std::move(comp));
        }
    }
}

} // anonymous namespace

ErrorCode chinese_ocr_utils::mser_detect(const ImageData& gray, Vector<MSERRegion>& regions,
                                          int delta, int min_area, int max_area) {
    if (gray.empty() || gray.channels != 1) return ErrorCode::InvalidImage;

    regions.clear();

    // 多阈值二值化：阈值从低到高扫描，统计每个阈值下的连通域
    // 计算稳定性：|area(T - delta) - area(T + delta)| / area(T)
    // 稳定性小（变化率小）的区域即为 MSER

    struct RegionTrack {
        int threshold;
        int x, y, w, h;
        int area;
    };

    // 简化版 MSER：扫描若干阈值，找到稳定的连通域
    Vector<RegionTrack> all_tracks;
    const int T_min = 50;
    const int T_max = 200;
    const int step = std::max(1, delta);

    for (int T = T_min; T <= T_max; T += step) {
        ImageData binary;
        binary.width = gray.width;
        binary.height = gray.height;
        binary.channels = 1;
        binary.format = ImageFormat::Mono8;
        binary.data.resize(gray.data.size());

        for (size_t i = 0; i < gray.data.size(); ++i) {
            binary.data[i] = gray.data[i] < T ? 255 : 0;
        }

        Vector<ConnectedComp> comps;
        label_connected(binary, 255, comps);

        for (const auto& c : comps) {
            if (c.area < min_area || c.area > max_area) continue;
            RegionTrack t;
            t.threshold = T;
            t.x = c.x; t.y = c.y; t.w = c.w; t.h = c.h;
            t.area = c.area;
            all_tracks.push_back(t);
        }
    }

    // 按位置聚类区域轨迹，计算稳定性
    // 简化处理：对每个阈值下的连通域，检查相邻阈值是否有相似区域（IoU > 0.5）
    // 并统计面积变化率，保留稳定区域
    Vector<RegionTrack> stable_tracks;
    for (size_t i = 0; i < all_tracks.size(); ++i) {
        const auto& cur = all_tracks[i];
        int T = cur.threshold;
        int T_low = T - step;
        int T_high = T + step;

        // 找相邻阈值的相似区域
        int area_low = -1, area_high = -1;
        for (const auto& other : all_tracks) {
            if (other.threshold == T_low) {
                // IoU 简化判断：中心点距离 + 尺寸相似
                float cx1 = cur.x + cur.w / 2.0f;
                float cy1 = cur.y + cur.h / 2.0f;
                float cx2 = other.x + other.w / 2.0f;
                float cy2 = other.y + other.h / 2.0f;
                float dist = std::sqrt((cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2));
                if (dist < std::max(cur.w, cur.h) * 0.5f) {
                    area_low = other.area;
                }
            }
            if (other.threshold == T_high) {
                float cx1 = cur.x + cur.w / 2.0f;
                float cy1 = cur.y + cur.h / 2.0f;
                float cx2 = other.x + other.w / 2.0f;
                float cy2 = other.y + other.h / 2.0f;
                float dist = std::sqrt((cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2));
                if (dist < std::max(cur.w, cur.h) * 0.5f) {
                    area_high = other.area;
                }
            }
        }

        if (area_low > 0 && area_high > 0 && cur.area > 0) {
            float stability = std::abs(area_low - area_high) / static_cast<float>(cur.area);
            if (stability < 0.5f) {
                MSERRegion r;
                r.x = cur.x; r.y = cur.y;
                r.width = cur.w; r.height = cur.h;
                r.area = cur.area;
                r.stability = stability;
                regions.push_back(r);
            }
        }
    }

    // 去重：相同位置的 MSER 只保留稳定性最好的
    Vector<MSERRegion> deduped;
    for (const auto& r : regions) {
        bool duplicate = false;
        for (auto& kept : deduped) {
            float cx1 = r.x + r.width / 2.0f;
            float cy1 = r.y + r.height / 2.0f;
            float cx2 = kept.x + kept.width / 2.0f;
            float cy2 = kept.y + kept.height / 2.0f;
            float dist = std::sqrt((cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2));
            if (dist < std::max(r.width, r.height) * 0.3f) {
                if (r.stability < kept.stability) {
                    kept = r;
                }
                duplicate = true;
                break;
            }
        }
        if (!duplicate) deduped.push_back(r);
    }
    regions = std::move(deduped);

    OVF_DEBUG() << "MSER: detected " << regions.size() << " stable regions";
    return ErrorCode::Success;
}

float chinese_ocr_utils::compute_iou(const CharBox& a, const CharBox& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);

    int iw = std::max(0, x2 - x1);
    int ih = std::max(0, y2 - y1);
    int inter = iw * ih;

    int ua = a.area() + b.area() - inter;
    if (ua <= 0) return 0.0f;
    return static_cast<float>(inter) / static_cast<float>(ua);
}

ErrorCode chinese_ocr_utils::nms(Vector<CharBox>& boxes, Vector<float>& scores, float threshold) {
    if (boxes.size() != scores.size() || boxes.empty()) return ErrorCode::Success;

    Vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    // 按分数降序排序
    std::sort(indices.begin(), indices.end(),
              [&scores](int a, int b) { return scores[a] > scores[b]; });

    Vector<CharBox> kept_boxes;
    Vector<float> kept_scores;
    Vector<uint8_t> suppressed(boxes.size(), 0);

    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        if (suppressed[idx]) continue;
        kept_boxes.push_back(boxes[idx]);
        kept_scores.push_back(scores[idx]);

        for (size_t j = i + 1; j < indices.size(); ++j) {
            int jdx = indices[j];
            if (suppressed[jdx]) continue;
            float iou = compute_iou(boxes[idx], boxes[jdx]);
            if (iou > threshold) suppressed[jdx] = 1;
        }
    }

    boxes = std::move(kept_boxes);
    scores = std::move(kept_scores);
    return ErrorCode::Success;
}

// ============================================================================
// 字符分割
// ============================================================================

ErrorCode chinese_ocr_utils::segment_by_projection(const ImageData& binary, Vector<CharBox>& chars,
                                                    int min_char_width, int max_char_width) {
    if (binary.empty()) return ErrorCode::InvalidImage;
    chars.clear();

    int w = static_cast<int>(binary.width);
    int h = static_cast<int>(binary.height);

    // 垂直投影：每列前景像素数
    Vector<int> col_proj(w, 0);
    for (int x = 0; x < w; ++x) {
        int cnt = 0;
        for (int y = 0; y < h; ++y) {
            if (binary.data[static_cast<size_t>(y) * w + x] > 0) ++cnt;
        }
        col_proj[x] = cnt;
    }

    // 找投影谷点（cnt == 0 或局部最小），切分字符
    bool in_char = false;
    int char_start = 0;
    int threshold = 1; // 至少 1 个前景像素

    for (int x = 0; x < w; ++x) {
        if (!in_char && col_proj[x] >= threshold) {
            in_char = true;
            char_start = x;
        } else if (in_char && col_proj[x] < threshold) {
            int cw = x - char_start;
            if (cw >= min_char_width && cw <= max_char_width) {
                // 找该列范围内的垂直范围
                int min_y = h, max_y = 0;
                bool found_y = false;
                for (int xx = char_start; xx < x; ++xx) {
                    for (int y = 0; y < h; ++y) {
                        if (binary.data[static_cast<size_t>(y) * w + xx] > 0) {
                            if (y < min_y) min_y = y;
                            if (y > max_y) max_y = y;
                            found_y = true;
                        }
                    }
                }
                if (found_y) {
                    chars.push_back(make_char_box(char_start, min_y, cw, max_y - min_y + 1));
                }
            }
            in_char = false;
        }
    }
    // 处理末尾
    if (in_char) {
        int cw = w - char_start;
        if (cw >= min_char_width && cw <= max_char_width) {
            int min_y = h, max_y = 0;
            bool found_y = false;
            for (int xx = char_start; xx < w; ++xx) {
                for (int y = 0; y < h; ++y) {
                    if (binary.data[static_cast<size_t>(y) * w + xx] > 0) {
                        if (y < min_y) min_y = y;
                        if (y > max_y) max_y = y;
                        found_y = true;
                    }
                }
            }
            if (found_y) {
                chars.push_back(make_char_box(char_start, min_y, cw, max_y - min_y + 1));
            }
        }
    }

    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::segment_by_connected(const ImageData& binary, Vector<CharBox>& chars,
                                                    int min_char_size, int max_char_size) {
    if (binary.empty()) return ErrorCode::InvalidImage;
    chars.clear();

    Vector<ConnectedComp> comps;
    label_connected(binary, 255, comps);

    for (const auto& c : comps) {
        if (c.w < min_char_size || c.w > max_char_size) continue;
        if (c.h < min_char_size || c.h > max_char_size) continue;
        // 字符宽高比约束（一般 0.2 - 2.0）
        float ar = static_cast<float>(c.w) / static_cast<float>(std::max(1, c.h));
        if (ar < 0.2f || ar > 2.0f) continue;
        chars.push_back(make_char_box(c.x, c.y, c.w, c.h));
    }

    // 按 x 坐标排序（从左到右，符合阅读顺序）
    std::sort(chars.begin(), chars.end(),
              [](const CharBox& a, const CharBox& b) { return a.x < b.x; });

    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::segment_chars(const ImageData& binary, Vector<CharBox>& chars,
                                            int min_char_size, int max_char_size) {
    if (binary.empty()) return ErrorCode::InvalidImage;

    // 先用连通域法（更鲁棒），失败则回退到投影法
    ErrorCode code = segment_by_connected(binary, chars, min_char_size, max_char_size);
    if (code != ErrorCode::Success || chars.empty()) {
        OVF_DEBUG() << "SegmentChars: connected method failed, fallback to projection";
        return segment_by_projection(binary, chars, min_char_size, max_char_size);
    }
    return ErrorCode::Success;
}

// ============================================================================
// 特征提取
// ============================================================================

ErrorCode chinese_ocr_utils::extract_grid_feature(const ImageData& char_img, FeatureVector& feature, int grid) {
    if (char_img.empty()) return ErrorCode::InvalidImage;
    feature.assign(grid * grid, 0.0f);

    // 归一化到 grid x grid
    ImageData resized;
    ErrorCode code = resize_image(char_img, resized, grid, grid);
    if (code != ErrorCode::Success) return code;

    // 计算每个网格的像素密度（归一化到 0-1）
    for (int y = 0; y < grid; ++y) {
        for (int x = 0; x < grid; ++x) {
            float v = resized.data[static_cast<size_t>(y) * grid + x] / 255.0f;
            feature[y * grid + x] = v;
        }
    }
    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::extract_direction_feature(const ImageData& char_img, FeatureVector& feature, int bins) {
    if (char_img.empty()) return ErrorCode::InvalidImage;
    feature.assign(bins, 0.0f);

    int w = static_cast<int>(char_img.width);
    int h = static_cast<int>(char_img.height);

    // 4 方向：0=水平, 1=垂直, 2=45°, 3=135°
    // 每方向 bins/4 个 bin
    int bins_per_dir = bins / 4;
    if (bins_per_dir == 0) bins_per_dir = 1;

    Vector<float> dir_mag(4, 0.0f);
    Vector<Vector<float>> dir_hist(4, Vector<float>(bins_per_dir, 0.0f));

    // Sobel 梯度
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            int gx = static_cast<int>(char_img.data[static_cast<size_t>(y) * w + (x + 1)]) -
                     static_cast<int>(char_img.data[static_cast<size_t>(y) * w + (x - 1)]);
            int gy = static_cast<int>(char_img.data[static_cast<size_t>(y + 1) * w + x]) -
                     static_cast<int>(char_img.data[static_cast<size_t>(y - 1) * w + x]);

            float mag = std::sqrt(static_cast<float>(gx * gx + gy * gy));
            if (mag < 30.0f) continue;

            // 角度（0-360）
            float angle = std::atan2(static_cast<float>(gy), static_cast<float>(gx)) * 180.0f / 3.14159265f;
            if (angle < 0) angle += 360.0f;

            // 分配到 4 个方向
            int dir;
            if ((angle >= 0 && angle < 22.5f) || (angle >= 157.5f && angle < 202.5f) || angle >= 337.5f) {
                dir = 0; // 水平
            } else if ((angle >= 67.5f && angle < 112.5f) || (angle >= 247.5f && angle < 292.5f)) {
                dir = 1; // 垂直
            } else if ((angle >= 22.5f && angle < 67.5f) || (angle >= 202.5f && angle < 247.5f)) {
                dir = 2; // 45°
            } else {
                dir = 3; // 135°
            }

            dir_mag[dir] += mag;
            int bin = static_cast<int>(std::min(1.0f, mag / 255.0f) * (bins_per_dir - 1));
            dir_hist[dir][bin] += mag;
        }
    }

    // 归一化并合并到 feature
    float total_mag = std::accumulate(dir_mag.begin(), dir_mag.end(), 0.0f);
    if (total_mag < 1e-6f) total_mag = 1.0f;

    int idx = 0;
    for (int d = 0; d < 4; ++d) {
        float dir_sum = std::accumulate(dir_hist[d].begin(), dir_hist[d].end(), 0.0f);
        if (dir_sum < 1e-6f) dir_sum = 1.0f;
        for (int b = 0; b < bins_per_dir; ++b) {
            feature[idx++] = dir_hist[d][b] / dir_sum;
        }
    }

    // 全局归一化
    float max_v = *std::max_element(feature.begin(), feature.end());
    if (max_v > 1e-6f) {
        for (auto& v : feature) v /= max_v;
    }

    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::extract_projection_feature(const ImageData& char_img, FeatureVector& feature, int bins) {
    if (char_img.empty()) return ErrorCode::InvalidImage;
    feature.assign(bins, 0.0f);

    int w = static_cast<int>(char_img.width);
    int h = static_cast<int>(char_img.height);
    int half = bins / 2;

    // 水平投影（每行像素和）→ half 维
    Vector<float> h_proj(h, 0.0f);
    for (int y = 0; y < h; ++y) {
        float sum = 0;
        for (int x = 0; x < w; ++x) {
            sum += char_img.data[static_cast<size_t>(y) * w + x];
        }
        h_proj[y] = sum;
    }
    float h_max = *std::max_element(h_proj.begin(), h_proj.end());
    if (h_max < 1e-6f) h_max = 1.0f;
    for (int b = 0; b < half; ++b) {
        float acc = 0;
        int cnt = 0;
        int y0 = b * h / half;
        int y1 = (b + 1) * h / half;
        for (int y = y0; y < y1 && y < h; ++y) {
            acc += h_proj[y];
            ++cnt;
        }
        feature[b] = (cnt > 0 ? acc / cnt : 0) / h_max;
    }

    // 垂直投影（每列像素和）→ half 维
    Vector<float> v_proj(w, 0.0f);
    for (int x = 0; x < w; ++x) {
        float sum = 0;
        for (int y = 0; y < h; ++y) {
            sum += char_img.data[static_cast<size_t>(y) * w + x];
        }
        v_proj[x] = sum;
    }
    float v_max = *std::max_element(v_proj.begin(), v_proj.end());
    if (v_max < 1e-6f) v_max = 1.0f;
    for (int b = 0; b < half; ++b) {
        float acc = 0;
        int cnt = 0;
        int x0 = b * w / half;
        int x1 = (b + 1) * w / half;
        for (int x = x0; x < x1 && x < w; ++x) {
            acc += v_proj[x];
            ++cnt;
        }
        feature[half + b] = (cnt > 0 ? acc / cnt : 0) / v_max;
    }

    return ErrorCode::Success;
}

ErrorCode chinese_ocr_utils::extract_features(const ImageData& char_img, FeatureVector& feature) {
    if (char_img.empty()) return ErrorCode::InvalidImage;

    // 预处理：归一化到 32x32
    ImageData normalized;
    ErrorCode code = preprocess_char(char_img, normalized, 32);
    if (code != ErrorCode::Success) return code;

    // 网格特征 64 维
    FeatureVector grid_feat;
    code = extract_grid_feature(normalized, grid_feat, 8);
    if (code != ErrorCode::Success) return code;

    // 方向特征 32 维
    FeatureVector dir_feat;
    code = extract_direction_feature(normalized, dir_feat, 32);
    if (code != ErrorCode::Success) return code;

    // 投影特征 32 维
    FeatureVector proj_feat;
    code = extract_projection_feature(normalized, proj_feat, 32);
    if (code != ErrorCode::Success) return code;

    // 合并
    feature.clear();
    feature.reserve(grid_feat.size() + dir_feat.size() + proj_feat.size());
    feature.insert(feature.end(), grid_feat.begin(), grid_feat.end());
    feature.insert(feature.end(), dir_feat.begin(), dir_feat.end());
    feature.insert(feature.end(), proj_feat.begin(), proj_feat.end());

    // L2 归一化
    float norm = 0.0f;
    for (float v : feature) norm += v * v;
    norm = std::sqrt(norm);
    if (norm > 1e-6f) {
        for (auto& v : feature) v /= norm;
    }

    return ErrorCode::Success;
}

int chinese_ocr_utils::get_feature_dim() {
    return 64 + 32 + 32; // 128
}

// ============================================================================
// 模板匹配
// ============================================================================

float chinese_ocr_utils::cosine_similarity(const FeatureVector& a, const FeatureVector& b) {
    if (a.size() != b.size() || a.empty()) return 0.0f;
    float dot = 0.0f;
    float na = 0.0f;
    float nb = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) {
        dot += a[i] * b[i];
        na += a[i] * a[i];
        nb += b[i] * b[i];
    }
    if (na < 1e-10f || nb < 1e-10f) return 0.0f;
    return dot / (std::sqrt(na) * std::sqrt(nb));
}

ErrorCode chinese_ocr_utils::match_template(const FeatureVector& feature, const OCRModel& model,
                                            CharResult& result, float threshold) {
    result = CharResult();

    if (feature.empty()) return ErrorCode::InvalidParameter;
    if (model.empty()) {
        return ErrorCode::InvalidModel;
    }

    float best_sim = -1.0f;
    String best_label;
    CharSet best_cs = model.char_set;

    for (const auto& tpl : model.templates) {
        // 混合模型可以匹配任意字符集；否则要求字符集匹配
        if (model.char_set != CharSet::Mixed && tpl.char_set != model.char_set) continue;

        float sim = cosine_similarity(feature, tpl.feature);
        if (sim > best_sim) {
            best_sim = sim;
            best_label = tpl.label;
            best_cs = tpl.char_set;
        }
    }

    if (best_label.empty() || best_sim < threshold) {
        result.character = "?";
        result.confidence = std::max(0.0f, best_sim * 100.0f);
        return ErrorCode::Success;
    }

    // 余弦相似度范围 [-1, 1]，归一化到 [0, 100]
    float confidence = std::max(0.0f, std::min(100.0f, best_sim * 100.0f));
    result.character = best_label;
    result.confidence = confidence;
    result.char_set = best_cs;
    return ErrorCode::Success;
}

// ============================================================================
// 模型 I/O（JSON 格式）
// ============================================================================

ErrorCode chinese_ocr_utils::save_model(const OCRModel& model, const String& path) {
    if (path.empty()) return ErrorCode::InvalidParameter;

    try {
        nlohmann::json j;
        j["name"] = model.name;
        j["char_set"] = static_cast<int>(model.char_set);
        j["feature_dim"] = model.feature_dim;
        j["version"] = model.version;
        j["created_time"] = model.created_time;

        nlohmann::json templates_arr = nlohmann::json::array();
        for (const auto& tpl : model.templates) {
            nlohmann::json t;
            t["label"] = tpl.label;
            t["char_set"] = static_cast<int>(tpl.char_set);
            t["sample_count"] = tpl.sample_count;

            nlohmann::json feat_arr = nlohmann::json::array();
            for (float v : tpl.feature) {
                feat_arr.push_back(v);
            }
            t["feature"] = feat_arr;
            templates_arr.push_back(t);
        }
        j["templates"] = templates_arr;

        std::ofstream ofs(path, std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) {
            OVF_ERROR() << "SaveModel: cannot open file: " << path;
            return ErrorCode::FileOpenFailed;
        }
        ofs << j.dump(2);
        ofs.close();

        OVF_INFO() << "SaveModel: saved " << model.templates.size() << " templates to " << path;
        return ErrorCode::Success;
    } catch (const std::exception& e) {
        OVF_ERROR() << "SaveModel: exception: " << e.what();
        return ErrorCode::FileWriteFailed;
    }
}

ErrorCode chinese_ocr_utils::load_model(OCRModel& model, const String& path) {
    if (path.empty()) return ErrorCode::InvalidParameter;

    try {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            OVF_ERROR() << "LoadModel: cannot open file: " << path;
            return ErrorCode::FileNotFound;
        }

        std::stringstream ss;
        ss << ifs.rdbuf();
        String content = ss.str();

        nlohmann::json j = nlohmann::json::parse(content);

        model.name = j.value("name", String("default"));
        model.char_set = static_cast<CharSet>(j.value("char_set", 0));
        model.feature_dim = j.value("feature_dim", get_feature_dim());
        model.version = j.value("version", String("0.1.0"));
        model.created_time = j.value("created_time", String(""));
        model.templates.clear();

        if (j.contains("templates") && j["templates"].is_array()) {
            for (const auto& t : j["templates"]) {
                OCRTemplate tpl;
                tpl.label = t.value("label", String(""));
                tpl.char_set = static_cast<CharSet>(t.value("char_set", 0));
                tpl.sample_count = t.value("sample_count", 1);

                if (t.contains("feature") && t["feature"].is_array()) {
                    for (const auto& fv : t["feature"]) {
                        if (fv.is_number_float()) {
                            tpl.feature.push_back(fv.get_double());
                        } else if (fv.is_number()) {
                            tpl.feature.push_back(static_cast<float>(fv.get_double()));
                        }
                    }
                }
                if (!tpl.label.empty() && !tpl.feature.empty()) {
                    model.templates.push_back(std::move(tpl));
                }
            }
        }

        OVF_INFO() << "LoadModel: loaded " << model.templates.size() << " templates from " << path;
        return ErrorCode::Success;
    } catch (const std::exception& e) {
        OVF_ERROR() << "LoadModel: exception: " << e.what();
        return ErrorCode::FileParseFailed;
    }
}

OCRModel chinese_ocr_utils::create_default_model(CharSet char_set) {
    OCRModel model;
    model.name = "default_" + char_set_name(char_set);
    model.char_set = char_set;
    model.feature_dim = get_feature_dim();
    model.version = "0.1.0";
    model.created_time = current_time_string();
    return model;
}

// ============================================================================
// 字符集工具
// ============================================================================

String chinese_ocr_utils::char_set_name(CharSet char_set) {
    switch (char_set) {
        case CharSet::Chinese: return "chinese";
        case CharSet::Digit:   return "digit";
        case CharSet::English: return "english";
        case CharSet::Mixed:   return "mixed";
        default: return "unknown";
    }
}

CharSet chinese_ocr_utils::parse_char_set(const String& name) {
    if (name == "chinese" || name == "Chinese" || name == "CN") return CharSet::Chinese;
    if (name == "digit" || name == "Digit" || name == "num")   return CharSet::Digit;
    if (name == "english" || name == "English" || name == "en") return CharSet::English;
    if (name == "mixed" || name == "Mixed")                    return CharSet::Mixed;
    return CharSet::Chinese;
}

Vector<String> chinese_ocr_utils::get_char_set(CharSet char_set) {
    Vector<String> chars;
    switch (char_set) {
        case CharSet::Digit: {
            for (char c = '0'; c <= '9'; ++c) {
                chars.push_back(String(1, c));
            }
            break;
        }
        case CharSet::English: {
            for (char c = 'A'; c <= 'Z'; ++c) {
                chars.push_back(String(1, c));
            }
            for (char c = 'a'; c <= 'z'; ++c) {
                chars.push_back(String(1, c));
            }
            break;
        }
        case CharSet::Chinese: {
            // GB2312 一级常用汉字 3755 个（按区位码 16-55 区）
            // 实际部署应通过 OCRTrainNode 训练生成完整模型
            // 这里返回常用子集以避免过大（约 100 个高频字）
            const char* common[] = {
                "的", "一", "是", "在", "不", "了", "有", "和", "人", "这",
                "中", "大", "为", "上", "个", "国", "我", "以", "要", "他",
                "时", "来", "用", "们", "生", "到", "作", "地", "于", "出",
                "就", "分", "对", "成", "会", "可", "主", "发", "年", "动",
                "同", "工", "也", "能", "下", "过", "子", "说", "产", "种",
                "面", "而", "方", "后", "多", "定", "行", "学", "法", "所",
                "民", "得", "经", "十", "三", "之", "进", "着", "等", "部",
                "度", "家", "电", "力", "里", "如", "水", "化", "高", "自",
                "二", "理", "起", "小", "物", "现", "实", "加", "量", "都",
                "两", "体", "制", "机", "当", "使", "点", "从", "业", "本",
                "回", "情", "然", "日", "前", "意", "用", "想", "政", "见"
            };
            for (const char* s : common) {
                chars.push_back(String(s));
            }
            break;
        }
        case CharSet::Mixed: {
            // 数字 + 英文 + 常用汉字
            Vector<String> digits = get_char_set(CharSet::Digit);
            Vector<String> english = get_char_set(CharSet::English);
            Vector<String> chinese = get_char_set(CharSet::Chinese);
            chars.insert(chars.end(), digits.begin(), digits.end());
            chars.insert(chars.end(), english.begin(), english.end());
            chars.insert(chars.end(), chinese.begin(), chinese.end());
            break;
        }
    }
    return chars;
}

String chinese_ocr_utils::current_time_string() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return String(buf);
}

// ============================================================================
// 通用 OCR 识别流程辅助函数（被各 OCR 节点复用）
// ============================================================================

namespace {

// 完整 OCR 流程：检测 → 分割 → 识别 → 拼接文本
ErrorCode run_ocr_pipeline(const ImageData& image, const OCRModel& model,
                           ChineseOCRResult& result,
                           int min_char_size, int max_char_size,
                           float confidence_threshold) {
    result = ChineseOCRResult();
    result.char_set = model.char_set;

    if (image.empty()) return ErrorCode::InvalidImage;
    if (model.empty()) {
        OVF_WARN() << "OCR pipeline: model is empty, results will be unknown";
    }

    // 1. 转灰度
    ImageData gray;
    ErrorCode code = chinese_ocr_utils::to_gray(image, gray);
    if (code != ErrorCode::Success) return code;

    // 2. 二值化（Otsu）
    ImageData binary;
    code = chinese_ocr_utils::threshold_otsu(gray, binary);
    if (code != ErrorCode::Success) return code;

    // 3. 字符分割
    Vector<CharBox> char_boxes;
    code = chinese_ocr_utils::segment_chars(binary, char_boxes, min_char_size, max_char_size);
    if (code != ErrorCode::Success) return code;

    if (char_boxes.empty()) {
        OVF_WARN() << "OCR pipeline: no characters segmented";
        result.text = "";
        result.confidence = 0.0f;
        return ErrorCode::Success;
    }

    // 4. 逐字符识别
    float total_conf = 0.0f;
    int recognized_count = 0;
    std::ostringstream text_oss;

    for (const auto& box : char_boxes) {
        ImageData char_img;
        code = chinese_ocr_utils::crop_image(gray, char_img, box.x, box.y, box.width, box.height);
        if (code != ErrorCode::Success) continue;

        FeatureVector feat;
        code = chinese_ocr_utils::extract_features(char_img, feat);
        if (code != ErrorCode::Success) continue;

        CharResult cr;
        cr.box = box;
        code = chinese_ocr_utils::match_template(feat, model, cr, confidence_threshold / 100.0f);
        if (code != ErrorCode::Success) continue;

        text_oss << cr.character;
        total_conf += cr.confidence;
        ++recognized_count;
        result.chars.push_back(std::move(cr));
    }

    result.text = text_oss.str();
    result.confidence = recognized_count > 0 ? total_conf / recognized_count : 0.0f;

    return ErrorCode::Success;
}

// 加载模型（若路径有效）
bool load_model_if_exists(OCRModel& model, bool& loaded_flag, const String& path, CharSet cs) {
    if (loaded_flag) return true;
    if (path.empty()) {
        model = chinese_ocr_utils::create_default_model(cs);
        loaded_flag = true;
        return true;
    }
    ErrorCode code = chinese_ocr_utils::load_model(model, path);
    if (code != ErrorCode::Success) {
        OVF_WARN() << "Load model failed, using default empty model: " << path;
        model = chinese_ocr_utils::create_default_model(cs);
    }
    loaded_flag = true;
    return true;
}

} // anonymous namespace

// ============================================================================
// CharDetectNode 实现（字符区域检测，MSER + NMS）
// ============================================================================

CharDetectNode::CharDetectNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CharDetectNode::make_info() {
    NodeInfo info;
    info.id = "CharDetect";
    info.name = "字符区域检测";
    info.category = "文字识别";
    info.description = "MSER + NMS 检测图像中的字符候选区域";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("char_region", "首个字符区域", DataType::Region));
    info.outputs.push_back(DataPort("region_count", "区域数量", DataType::Number));
    info.outputs.push_back(DataPort("text", "检测概要", DataType::String));

    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(120)));
    info.params.push_back(ParamDef("delta", "MSER阈值步长", DataType::Number, Data(5)));

    return info;
}

Result<void> CharDetectNode::init() {
    return Result<void>::success();
}

Result<void> CharDetectNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int min_size = get_param("min_char_size", Data(8)).as_int();
    int max_size = get_param("max_char_size", Data(120)).as_int();
    int delta = get_param("delta", Data(5)).as_int();

    // 转灰度
    ImageData gray;
    ErrorCode code = chinese_ocr_utils::to_gray(image, gray);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "to_gray failed");
    }

    // MSER 检测
    Vector<chinese_ocr_utils::MSERRegion> regions;
    int min_area = min_size * min_size;
    int max_area = max_size * max_size;
    code = chinese_ocr_utils::mser_detect(gray, regions, delta, min_area, max_area);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "MSER detection failed");
    }

    // 转换为 CharBox + 置信度（基于稳定性）
    Vector<CharBox> boxes;
    Vector<float> scores;
    for (const auto& r : regions) {
        CharBox b(r.x, r.y, r.width, r.height);
        boxes.push_back(b);
        // 稳定性越小越好 → 转换为分数
        float score = 1.0f / (1.0f + r.stability);
        scores.push_back(score);
    }

    // NMS 去重
    code = chinese_ocr_utils::nms(boxes, scores, 0.3f);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "NMS failed");
    }

    // 输出
    if (!boxes.empty()) {
        Region r = boxes[0].to_region();
        set_output("char_region", Data(r));
    } else {
        Region empty;
        set_output("char_region", Data(empty));
    }
    set_output("region_count", Data(static_cast<int32_t>(boxes.size())));

    std::ostringstream oss;
    oss << "CharDetect: detected " << boxes.size() << " regions"
        << " (min_size=" << min_size << ", max_size=" << max_size << ")";
    set_output("text", Data(oss.str()));

    OVF_INFO() << oss.str();
    return Result<void>::success();
}

// ============================================================================
// CharSegmentNode 实现（字符分割，投影法 + 连通域）
// ============================================================================

CharSegmentNode::CharSegmentNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CharSegmentNode::make_info() {
    NodeInfo info;
    info.id = "CharSegment";
    info.name = "字符分割";
    info.category = "文字识别";
    info.description = "通过投影法 + 连通域分析分割字符";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("char_count", "字符数量", DataType::Number));
    info.outputs.push_back(DataPort("first_char_region", "首个字符框", DataType::Region));
    info.outputs.push_back(DataPort("text", "分割概要", DataType::String));

    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(100)));
    Vector<String> methods;
    methods.push_back("auto");
    methods.push_back("projection");
    methods.push_back("connected");
    ParamDef method_p("method", "分割方法", DataType::String, Data("auto"));
    method_p.options = methods;
    info.params.push_back(method_p);

    return info;
}

Result<void> CharSegmentNode::init() {
    return Result<void>::success();
}

Result<void> CharSegmentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int min_size = get_param("min_char_size", Data(5)).as_int();
    int max_size = get_param("max_char_size", Data(100)).as_int();
    String method = get_param("method", Data("auto")).as_string();

    // 转灰度 + 二值化
    ImageData gray, binary;
    ErrorCode code = chinese_ocr_utils::to_gray(image, gray);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "to_gray failed");
    }
    code = chinese_ocr_utils::threshold_otsu(gray, binary);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "threshold failed");
    }

    Vector<CharBox> chars;
    if (method == "projection") {
        code = chinese_ocr_utils::segment_by_projection(binary, chars, min_size, max_size);
    } else if (method == "connected") {
        code = chinese_ocr_utils::segment_by_connected(binary, chars, min_size, max_size);
    } else {
        code = chinese_ocr_utils::segment_chars(binary, chars, min_size, max_size);
    }
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "segmentation failed");
    }

    // 输出
    set_output("char_count", Data(static_cast<int32_t>(chars.size())));
    if (!chars.empty()) {
        set_output("first_char_region", Data(chars[0].to_region()));
    } else {
        Region empty;
        set_output("first_char_region", Data(empty));
    }

    std::ostringstream oss;
    oss << "CharSegment: " << chars.size() << " chars"
        << " (method=" << method << ")";
    set_output("text", Data(oss.str()));

    OVF_INFO() << oss.str();
    return Result<void>::success();
}

// ============================================================================
// CharBoxNode 实现（字符框定位）
// ============================================================================

CharBoxNode::CharBoxNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CharBoxNode::make_info() {
    NodeInfo info;
    info.id = "CharBox";
    info.name = "字符框定位";
    info.category = "文字识别";
    info.description = "定位图像中的字符框（基于连通域 + 宽高比约束）";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("char_box", "主字符框", DataType::Region));
    info.outputs.push_back(DataPort("box_count", "字符框数量", DataType::Number));
    info.outputs.push_back(DataPort("text", "定位概要", DataType::String));

    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("aspect_ratio", "字符宽高比约束", DataType::Number, Data(2.0)));

    return info;
}

Result<void> CharBoxNode::init() {
    return Result<void>::success();
}

Result<void> CharBoxNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    int min_size = get_param("min_char_size", Data(5)).as_int();
    int max_size = get_param("max_char_size", Data(100)).as_int();
    double ar_limit = get_param("aspect_ratio", Data(2.0)).as_number();

    // 转灰度 + 二值化
    ImageData gray, binary;
    ErrorCode code = chinese_ocr_utils::to_gray(image, gray);
    if (code != ErrorCode::Success) return Result<void>::failure(code, "to_gray failed");
    code = chinese_ocr_utils::threshold_otsu(gray, binary);
    if (code != ErrorCode::Success) return Result<void>::failure(code, "threshold failed");

    // 连通域分割
    Vector<CharBox> chars;
    code = chinese_ocr_utils::segment_by_connected(binary, chars, min_size, max_size);
    if (code != ErrorCode::Success) return Result<void>::failure(code, "segment failed");

    // 宽高比过滤
    Vector<CharBox> filtered;
    for (const auto& c : chars) {
        float ar = c.aspect_ratio();
        if (ar >= 1.0f / static_cast<float>(ar_limit) && ar <= static_cast<float>(ar_limit)) {
            filtered.push_back(c);
        }
    }

    // 输出
    if (!filtered.empty()) {
        set_output("char_box", Data(filtered[0].to_region()));
    } else {
        Region empty;
        set_output("char_box", Data(empty));
    }
    set_output("box_count", Data(static_cast<int32_t>(filtered.size())));

    std::ostringstream oss;
    oss << "CharBox: " << filtered.size() << " boxes located";
    set_output("text", Data(oss.str()));

    OVF_INFO() << oss.str();
    return Result<void>::success();
}

// ============================================================================
// ChineseOCRNode 实现（汉字 OCR 识别）
// ============================================================================

ChineseOCRNode::ChineseOCRNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ChineseOCRNode::make_info() {
    NodeInfo info;
    info.id = "ChineseOCR";
    info.name = "汉字OCR识别";
    info.category = "文字识别";
    info.description = "印刷体汉字识别（模板匹配法，GB2312 一级常用汉字）";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "待识别图像", DataType::Image, true));

    info.outputs.push_back(DataPort("text", "识别文本", DataType::String));
    info.outputs.push_back(DataPort("confidence", "整体置信度", DataType::Number));
    info.outputs.push_back(DataPort("chars", "字符结果（首个字符框）", DataType::Region));

    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("chinese")));
    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(8)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(120)));
    info.params.push_back(ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("")));

    return info;
}

Result<void> ChineseOCRNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    load_model_if_exists(model_, model_loaded_, model_path, CharSet::Chinese);
    return Result<void>::success();
}

Result<void> ChineseOCRNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    // 若模型路径变化，重新加载
    String model_path = get_param("model_path", Data("")).as_string();
    if (!model_path.empty() && !model_loaded_) {
        load_model_if_exists(model_, model_loaded_, model_path, CharSet::Chinese);
    }

    int min_size = get_param("min_char_size", Data(8)).as_int();
    int max_size = get_param("max_char_size", Data(120)).as_int();
    double conf_threshold = get_param("confidence_threshold", Data(50.0)).as_number();

    ChineseOCRResult ocr_result;
    ErrorCode code = run_ocr_pipeline(image, model_, ocr_result,
                                       min_size, max_size,
                                       static_cast<float>(conf_threshold));
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "OCR pipeline failed");
    }

    set_output("text", Data(ocr_result.text));
    set_output("confidence", Data(static_cast<double>(ocr_result.confidence)));
    if (!ocr_result.chars.empty()) {
        set_output("chars", Data(ocr_result.chars[0].box.to_region()));
    } else {
        Region empty;
        set_output("chars", Data(empty));
    }

    OVF_INFO() << "ChineseOCR: text='" << ocr_result.text
               << "', conf=" << ocr_result.confidence
               << ", chars=" << ocr_result.chars.size();
    return Result<void>::success();
}

// ============================================================================
// DigitOCRNode 实现（数字 OCR 识别）
// ============================================================================

DigitOCRNode::DigitOCRNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DigitOCRNode::make_info() {
    NodeInfo info;
    info.id = "DigitOCR";
    info.name = "数字OCR识别";
    info.category = "文字识别";
    info.description = "数字 0-9 模板匹配识别";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "待识别图像", DataType::Image, true));

    info.outputs.push_back(DataPort("text", "识别文本", DataType::String));
    info.outputs.push_back(DataPort("confidence", "整体置信度", DataType::Number));
    info.outputs.push_back(DataPort("chars", "字符结果（首个字符框）", DataType::Region));

    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("digit")));
    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(80)));
    info.params.push_back(ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("")));

    return info;
}

Result<void> DigitOCRNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    load_model_if_exists(model_, model_loaded_, model_path, CharSet::Digit);
    return Result<void>::success();
}

Result<void> DigitOCRNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    String model_path = get_param("model_path", Data("")).as_string();
    if (!model_path.empty() && !model_loaded_) {
        load_model_if_exists(model_, model_loaded_, model_path, CharSet::Digit);
    }

    int min_size = get_param("min_char_size", Data(5)).as_int();
    int max_size = get_param("max_char_size", Data(80)).as_int();
    double conf_threshold = get_param("confidence_threshold", Data(50.0)).as_number();

    ChineseOCRResult ocr_result;
    ErrorCode code = run_ocr_pipeline(image, model_, ocr_result,
                                       min_size, max_size,
                                       static_cast<float>(conf_threshold));
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "OCR pipeline failed");
    }

    set_output("text", Data(ocr_result.text));
    set_output("confidence", Data(static_cast<double>(ocr_result.confidence)));
    if (!ocr_result.chars.empty()) {
        set_output("chars", Data(ocr_result.chars[0].box.to_region()));
    } else {
        Region empty;
        set_output("chars", Data(empty));
    }

    OVF_INFO() << "DigitOCR: text='" << ocr_result.text
               << "', conf=" << ocr_result.confidence;
    return Result<void>::success();
}

// ============================================================================
// EnglishOCRNode 实现（英文字母识别）
// ============================================================================

EnglishOCRNode::EnglishOCRNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo EnglishOCRNode::make_info() {
    NodeInfo info;
    info.id = "EnglishOCR";
    info.name = "英文字母识别";
    info.category = "文字识别";
    info.description = "英文字母 A-Z 模板匹配识别";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "待识别图像", DataType::Image, true));

    info.outputs.push_back(DataPort("text", "识别文本", DataType::String));
    info.outputs.push_back(DataPort("confidence", "整体置信度", DataType::Number));
    info.outputs.push_back(DataPort("chars", "字符结果（首个字符框）", DataType::Region));

    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("english")));
    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(80)));
    info.params.push_back(ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("")));

    return info;
}

Result<void> EnglishOCRNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    load_model_if_exists(model_, model_loaded_, model_path, CharSet::English);
    return Result<void>::success();
}

Result<void> EnglishOCRNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    String model_path = get_param("model_path", Data("")).as_string();
    if (!model_path.empty() && !model_loaded_) {
        load_model_if_exists(model_, model_loaded_, model_path, CharSet::English);
    }

    int min_size = get_param("min_char_size", Data(5)).as_int();
    int max_size = get_param("max_char_size", Data(80)).as_int();
    double conf_threshold = get_param("confidence_threshold", Data(50.0)).as_number();

    ChineseOCRResult ocr_result;
    ErrorCode code = run_ocr_pipeline(image, model_, ocr_result,
                                       min_size, max_size,
                                       static_cast<float>(conf_threshold));
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "OCR pipeline failed");
    }

    set_output("text", Data(ocr_result.text));
    set_output("confidence", Data(static_cast<double>(ocr_result.confidence)));
    if (!ocr_result.chars.empty()) {
        set_output("chars", Data(ocr_result.chars[0].box.to_region()));
    } else {
        Region empty;
        set_output("chars", Data(empty));
    }

    OVF_INFO() << "EnglishOCR: text='" << ocr_result.text
               << "', conf=" << ocr_result.confidence;
    return Result<void>::success();
}

// ============================================================================
// MixedOCRNode 实现（混合字符识别）
// ============================================================================

MixedOCRNode::MixedOCRNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MixedOCRNode::make_info() {
    NodeInfo info;
    info.id = "MixedOCR";
    info.name = "混合字符识别";
    info.category = "文字识别";
    info.description = "混合字符识别（汉字 + 数字 + 英文）";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "待识别图像", DataType::Image, true));

    info.outputs.push_back(DataPort("text", "识别文本", DataType::String));
    info.outputs.push_back(DataPort("confidence", "整体置信度", DataType::Number));
    info.outputs.push_back(DataPort("chars", "字符结果（首个字符框）", DataType::Region));

    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("mixed")));
    info.params.push_back(ParamDef("min_char_size", "最小字符尺寸", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_char_size", "最大字符尺寸", DataType::Number, Data(120)));
    info.params.push_back(ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("")));

    return info;
}

Result<void> MixedOCRNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    load_model_if_exists(model_, model_loaded_, model_path, CharSet::Mixed);
    return Result<void>::success();
}

Result<void> MixedOCRNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    ImageData image = input_data.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    String model_path = get_param("model_path", Data("")).as_string();
    if (!model_path.empty() && !model_loaded_) {
        load_model_if_exists(model_, model_loaded_, model_path, CharSet::Mixed);
    }

    int min_size = get_param("min_char_size", Data(5)).as_int();
    int max_size = get_param("max_char_size", Data(120)).as_int();
    double conf_threshold = get_param("confidence_threshold", Data(50.0)).as_number();

    ChineseOCRResult ocr_result;
    ErrorCode code = run_ocr_pipeline(image, model_, ocr_result,
                                       min_size, max_size,
                                       static_cast<float>(conf_threshold));
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "OCR pipeline failed");
    }

    set_output("text", Data(ocr_result.text));
    set_output("confidence", Data(static_cast<double>(ocr_result.confidence)));
    if (!ocr_result.chars.empty()) {
        set_output("chars", Data(ocr_result.chars[0].box.to_region()));
    } else {
        Region empty;
        set_output("chars", Data(empty));
    }

    OVF_INFO() << "MixedOCR: text='" << ocr_result.text
               << "', conf=" << ocr_result.confidence
               << ", chars=" << ocr_result.chars.size();
    return Result<void>::success();
}

// ============================================================================
// OCRTrainNode 实现（OCR 字符训练）
// ============================================================================

OCRTrainNode::OCRTrainNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo OCRTrainNode::make_info() {
    NodeInfo info;
    info.id = "OCRTrain";
    info.name = "OCR字符训练";
    info.category = "文字识别";
    info.description = "收集字符样本，提取特征，增量训练到模板库（JSON）";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "字符样本图像", DataType::Image, true));
    info.inputs.push_back(DataPort("label", "字符标签", DataType::String, true));

    info.outputs.push_back(DataPort("success", "是否成功", DataType::Boolean));
    info.outputs.push_back(DataPort("model_path", "模型库路径", DataType::String));
    info.outputs.push_back(DataPort("sample_count", "模板总数", DataType::Number));

    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("chinese")));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("./chinese_ocr_model.json")));
    info.params.push_back(ParamDef("confidence_threshold", "去重阈值", DataType::Number, Data(95.0)));

    return info;
}

Result<void> OCRTrainNode::init() {
    return Result<void>::success();
}

Result<void> OCRTrainNode::execute(FlowContext& context) {
    auto input_image = get_input("image");
    auto input_label = get_input("label");

    if (!input_image.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    if (!input_label.is_string()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Input label is not a string");
    }

    ImageData image = input_image.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    String label = input_label.as_string();
    if (label.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Label is empty");
    }

    String model_path = get_param("model_path", Data("./chinese_ocr_model.json")).as_string();
    String cs_str = get_param("char_set", Data("chinese")).as_string();
    CharSet cs = chinese_ocr_utils::parse_char_set(cs_str);
    double dedup_threshold = get_param("confidence_threshold", Data(95.0)).as_number();

    // 加载已有模型
    OCRModel model;
    ErrorCode code = chinese_ocr_utils::load_model(model, model_path);
    if (code != ErrorCode::Success) {
        OVF_INFO() << "OCRTrain: create new model (file not found)";
        model = chinese_ocr_utils::create_default_model(cs);
    }

    // 提取特征
    FeatureVector feat;
    code = chinese_ocr_utils::extract_features(image, feat);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "feature extraction failed");
    }

    // 检查是否已存在相同字符的模板（去重）
    OCRTemplate* existing = model.find(label);
    if (existing) {
        // 计算与已有特征的相似度，若已超过去重阈值则跳过
        float sim = chinese_ocr_utils::cosine_similarity(feat, existing->feature);
        if (sim * 100.0f >= static_cast<float>(dedup_threshold)) {
            OVF_INFO() << "OCRTrain: sample skipped (duplicate, sim=" << sim << ")";
            set_output("success", Data(true));
            set_output("model_path", Data(model_path));
            set_output("sample_count", Data(static_cast<int32_t>(model.size())));
            return Result<void>::success();
        }
    }

    // 增量添加
    model.add_template(label, cs, feat);

    // 保存
    code = chinese_ocr_utils::save_model(model, model_path);
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "save model failed");
    }

    set_output("success", Data(true));
    set_output("model_path", Data(model_path));
    set_output("sample_count", Data(static_cast<int32_t>(model.size())));

    OVF_INFO() << "OCRTrain: trained '" << label << "'"
               << ", total templates=" << model.size();
    return Result<void>::success();
}

// ============================================================================
// OCRModelNode 实现（OCR 模型管理）
// ============================================================================

OCRModelNode::OCRModelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo OCRModelNode::make_info() {
    NodeInfo info;
    info.id = "OCRModel";
    info.name = "OCR模型管理";
    info.category = "文字识别";
    info.description = "加载 / 保存 / 清空 / 查询 OCR 模型";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "可选样本图像", DataType::Image, false));

    info.outputs.push_back(DataPort("model_size", "模板数量", DataType::Number));
    info.outputs.push_back(DataPort("model_path", "模型路径", DataType::String));
    info.outputs.push_back(DataPort("model_name", "模型名称", DataType::String));

    info.params.push_back(ParamDef("action", "操作", DataType::String, Data("info")));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("./chinese_ocr_model.json")));
    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("chinese")));
    info.params.push_back(ParamDef("model_name", "模型名称", DataType::String, Data("default")));

    Vector<String> actions;
    actions.push_back("info");
    actions.push_back("load");
    actions.push_back("save");
    actions.push_back("clear");
    info.params[0].options = actions;

    return info;
}

Result<void> OCRModelNode::init() {
    return Result<void>::success();
}

Result<void> OCRModelNode::execute(FlowContext& context) {
    String action = get_param("action", Data("info")).as_string();
    String model_path = get_param("model_path", Data("./chinese_ocr_model.json")).as_string();
    String cs_str = get_param("char_set", Data("chinese")).as_string();
    String model_name = get_param("model_name", Data("default")).as_string();
    CharSet cs = chinese_ocr_utils::parse_char_set(cs_str);

    if (action == "info") {
        // 加载并输出信息
        OCRModel model;
        ErrorCode code = chinese_ocr_utils::load_model(model, model_path);
        if (code != ErrorCode::Success) {
            model = chinese_ocr_utils::create_default_model(cs);
        }
        set_output("model_size", Data(static_cast<int32_t>(model.size())));
        set_output("model_path", Data(model_path));
        set_output("model_name", Data(model.name));
        OVF_INFO() << "OCRModel info: " << model.size() << " templates, name=" << model.name;
        return Result<void>::success();
    }

    if (action == "load") {
        OCRModel model;
        ErrorCode code = chinese_ocr_utils::load_model(model, model_path);
        if (code != ErrorCode::Success) {
            return Result<void>::failure(code, "load model failed");
        }
        set_output("model_size", Data(static_cast<int32_t>(model.size())));
        set_output("model_path", Data(model_path));
        set_output("model_name", Data(model.name));
        OVF_INFO() << "OCRModel load: " << model.size() << " templates";
        return Result<void>::success();
    }

    if (action == "save") {
        // 保存默认空模型（或追加输入样本）
        OCRModel model = chinese_ocr_utils::create_default_model(cs);
        model.name = model_name;

        // 若有输入图像，作为新模板
        if (has_input("image")) {
            auto input_data = get_input("image");
            if (input_data.is_image()) {
                ImageData img = input_data.as_image();
                if (!img.empty()) {
                    FeatureVector feat;
                    ErrorCode code = chinese_ocr_utils::extract_features(img, feat);
                    if (code == ErrorCode::Success) {
                        model.add_template("sample", cs, feat);
                    }
                }
            }
        }

        ErrorCode code = chinese_ocr_utils::save_model(model, model_path);
        if (code != ErrorCode::Success) {
            return Result<void>::failure(code, "save model failed");
        }
        set_output("model_size", Data(static_cast<int32_t>(model.size())));
        set_output("model_path", Data(model_path));
        set_output("model_name", Data(model.name));
        return Result<void>::success();
    }

    if (action == "clear") {
        OCRModel model = chinese_ocr_utils::create_default_model(cs);
        model.name = model_name;
        ErrorCode code = chinese_ocr_utils::save_model(model, model_path);
        if (code != ErrorCode::Success) {
            return Result<void>::failure(code, "clear model failed");
        }
        set_output("model_size", Data(static_cast<int32_t>(0)));
        set_output("model_path", Data(model_path));
        set_output("model_name", Data(model.name));
        OVF_INFO() << "OCRModel clear: model reset";
        return Result<void>::success();
    }

    return Result<void>::failure(ErrorCode::InvalidParameter,
                                  "Unknown action: " + action);
}

// ============================================================================
// OCRVerifyNode 实现（OCR 识别验证）
// ============================================================================

OCRVerifyNode::OCRVerifyNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo OCRVerifyNode::make_info() {
    NodeInfo info;
    info.id = "OCRVerify";
    info.name = "OCR识别验证";
    info.category = "文字识别";
    info.description = "OCR 识别 + 与期望文本对比，输出匹配率和准确率";
    info.version = "0.1.0";
    info.author = "OpenVisionFlow Team";

    info.inputs.push_back(DataPort("image", "待识别图像", DataType::Image, true));
    info.inputs.push_back(DataPort("expected_text", "期望文本", DataType::String, true));

    info.outputs.push_back(DataPort("matched", "是否匹配", DataType::Boolean));
    info.outputs.push_back(DataPort("accuracy", "准确率", DataType::Number));
    info.outputs.push_back(DataPort("actual_text", "实际识别文本", DataType::String));
    info.outputs.push_back(DataPort("confidence", "识别置信度", DataType::Number));

    info.params.push_back(ParamDef("char_set", "字符集", DataType::String, Data("chinese")));
    info.params.push_back(ParamDef("confidence_threshold", "置信度阈值", DataType::Number, Data(50.0)));
    info.params.push_back(ParamDef("model_path", "模型库路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("case_sensitive", "区分大小写", DataType::Boolean, Data(false)));

    return info;
}

Result<void> OCRVerifyNode::init() {
    String model_path = get_param("model_path", Data("")).as_string();
    load_model_if_exists(model_, model_loaded_, model_path, CharSet::Chinese);
    return Result<void>::success();
}

Result<void> OCRVerifyNode::execute(FlowContext& context) {
    auto input_image = get_input("image");
    auto input_expected = get_input("expected_text");

    if (!input_image.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    if (!input_expected.is_string()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "expected_text is not a string");
    }

    ImageData image = input_image.as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    String expected = input_expected.as_string();

    String model_path = get_param("model_path", Data("")).as_string();
    String cs_str = get_param("char_set", Data("chinese")).as_string();
    CharSet cs = chinese_ocr_utils::parse_char_set(cs_str);
    if (!model_path.empty() && !model_loaded_) {
        load_model_if_exists(model_, model_loaded_, model_path, cs);
    }

    double conf_threshold = get_param("confidence_threshold", Data(50.0)).as_number();
    bool case_sensitive = get_param("case_sensitive", Data(false)).as_bool();

    // OCR 识别
    ChineseOCRResult ocr_result;
    ErrorCode code = run_ocr_pipeline(image, model_, ocr_result,
                                       5, 120,
                                       static_cast<float>(conf_threshold));
    if (code != ErrorCode::Success) {
        return Result<void>::failure(code, "OCR pipeline failed");
    }

    // 计算准确率（按字符）
    String actual = ocr_result.text;
    String expected_norm = expected;
    String actual_norm = actual;
    if (!case_sensitive) {
        // 转小写（仅对 ASCII 字母有效）
        std::transform(expected_norm.begin(), expected_norm.end(), expected_norm.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        std::transform(actual_norm.begin(), actual_norm.end(), actual_norm.begin(),
                       [](unsigned char c) { return std::tolower(c); });
    }

    bool matched = (expected_norm == actual_norm);
    double accuracy = 0.0;
    if (!expected_norm.empty()) {
        // 编辑距离的简化版本：按字符对齐计算匹配率
        size_t min_len = std::min(expected_norm.size(), actual_norm.size());
        size_t match_count = 0;
        for (size_t i = 0; i < min_len; ++i) {
            if (expected_norm[i] == actual_norm[i]) ++match_count;
        }
        size_t max_len = std::max(expected_norm.size(), actual_norm.size());
        accuracy = max_len > 0 ? static_cast<double>(match_count) / max_len * 100.0 : 0.0;
    } else if (actual_norm.empty()) {
        accuracy = 100.0;
    }

    set_output("matched", Data(matched));
    set_output("accuracy", Data(accuracy));
    set_output("actual_text", Data(actual));
    set_output("confidence", Data(static_cast<double>(ocr_result.confidence)));

    OVF_INFO() << "OCRVerify: expected='" << expected
               << "', actual='" << actual
               << "', matched=" << (matched ? "true" : "false")
               << ", accuracy=" << accuracy;
    return Result<void>::success();
}

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(CharDetectNode,   "CharDetect",   CharDetectNode::make_info());
OVF_REGISTER_NODE(CharSegmentNode,  "CharSegment",  CharSegmentNode::make_info());
OVF_REGISTER_NODE(CharBoxNode,      "CharBox",      CharBoxNode::make_info());
OVF_REGISTER_NODE(ChineseOCRNode,    "ChineseOCR",   ChineseOCRNode::make_info());
OVF_REGISTER_NODE(DigitOCRNode,     "DigitOCR",     DigitOCRNode::make_info());
OVF_REGISTER_NODE(EnglishOCRNode,   "EnglishOCR",   EnglishOCRNode::make_info());
OVF_REGISTER_NODE(MixedOCRNode,     "MixedOCR",     MixedOCRNode::make_info());
OVF_REGISTER_NODE(OCRTrainNode,     "OCRTrain",     OCRTrainNode::make_info());
OVF_REGISTER_NODE(OCRModelNode,      "OCRModel",     OCRModelNode::make_info());
OVF_REGISTER_NODE(OCRVerifyNode,    "OCRVerify",    OCRVerifyNode::make_info());

} // namespace algorithm
} // namespace ovf
