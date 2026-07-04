/**
 * @file advanced_matching.cpp
 * @brief Halcon风格的高级模板匹配算子实现（纯C++实现）
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "ovf/algorithm/advanced_matching.h"
#include "ovf/algorithm/image_utils.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <limits>
#include <fstream>
#include <numeric>

namespace ovf {
namespace algorithm {

// ========== 工具函数实现 ==========

namespace advanced_match_utils {

void compute_gradient(const uint8_t* src, int width, int height,
                      std::vector<float>& grad_x, std::vector<float>& grad_y) {
    grad_x.resize(width * height, 0.0f);
    grad_y.resize(width * height, 0.0f);

    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int idx = y * width + x;
            // Sobel算子
            grad_x[idx] = static_cast<float>(
                -src[(y - 1) * width + (x - 1)] + src[(y - 1) * width + (x + 1)]
                - 2 * src[y * width + (x - 1)] + 2 * src[y * width + (x + 1)]
                - src[(y + 1) * width + (x - 1)] + src[(y + 1) * width + (x + 1)]
            ) / 8.0f;
            grad_y[idx] = static_cast<float>(
                -src[(y - 1) * width + (x - 1)] - 2 * src[(y - 1) * width + x] - src[(y - 1) * width + (x + 1)]
                + src[(y + 1) * width + (x - 1)] + 2 * src[(y + 1) * width + x] + src[(y + 1) * width + (x + 1)]
            ) / 8.0f;
        }
    }
}

void detect_edges(const uint8_t* src, int width, int height,
                  std::vector<ShapeModel::EdgePoint>& edges,
                  float min_contrast) {
    edges.clear();

    std::vector<float> grad_x, grad_y;
    compute_gradient(src, width, height, grad_x, grad_y);

    // 非极大值抑制
    for (int y = 2; y < height - 2; ++y) {
        for (int x = 2; x < width - 2; ++x) {
            int idx = y * width + x;
            float gx = grad_x[idx];
            float gy = grad_y[idx];
            float magnitude = std::sqrt(gx * gx + gy * gy);

            if (magnitude < min_contrast) continue;

            // 方向量化（0, 45, 90, 135度）
            float direction = std::atan2(gy, gx);
            float abs_dir = std::abs(direction);
            int neighbor1 = idx, neighbor2 = idx;

            if (abs_dir < 0.3927f || abs_dir > 2.7489f) { // 水平方向
                neighbor1 = idx - 1;
                neighbor2 = idx + 1;
            } else if (abs_dir < 1.1781f) { // 45度
                neighbor1 = idx - width - 1;
                neighbor2 = idx + width + 1;
            } else if (abs_dir < 1.9635f) { // 垂直方向
                neighbor1 = idx - width;
                neighbor2 = idx + width;
            } else { // 135度
                neighbor1 = idx - width + 1;
                neighbor2 = idx + width - 1;
            }

            float mag1 = std::sqrt(grad_x[neighbor1] * grad_x[neighbor1] + grad_y[neighbor1] * grad_y[neighbor1]);
            float mag2 = std::sqrt(grad_x[neighbor2] * grad_x[neighbor2] + grad_y[neighbor2] * grad_y[neighbor2]);

            if (magnitude >= mag1 && magnitude >= mag2) {
                ShapeModel::EdgePoint edge;
                edge.x = x;
                edge.y = y;
                edge.direction = direction;
                edge.magnitude = magnitude;
                edges.push_back(edge);
            }
        }
    }
}

void build_image_pyramid(const uint8_t* src, int width, int height,
                         std::vector<std::vector<uint8_t>>& pyramid, int levels) {
    pyramid.resize(levels);
    pyramid[0].assign(src, src + width * height);

    int cur_w = width;
    int cur_h = height;

    for (int level = 1; level < levels; ++level) {
        int new_w = cur_w / 2;
        int new_h = cur_h / 2;
        if (new_w < 8 || new_h < 8) break;

        pyramid[level].resize(new_w * new_h);
        image_utils::resize_bilinear(pyramid[level - 1].data(), cur_w, cur_h, 1,
                                     pyramid[level], new_w, new_h);
        cur_w = new_w;
        cur_h = new_h;
    }
}

void rotate_image(const uint8_t* src, int width, int height,
                  std::vector<uint8_t>& dst, float angle_deg) {
    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);

    // 计算旋转后图像的尺寸
    int new_w = static_cast<int>(std::abs(width * cos_a) + std::abs(height * sin_a) + 0.5f);
    int new_h = static_cast<int>(std::abs(width * sin_a) + std::abs(height * cos_a) + 0.5f);

    dst.resize(new_w * new_h);
    std::fill(dst.begin(), dst.end(), 0);

    float cx = width / 2.0f;
    float cy = height / 2.0f;
    float new_cx = new_w / 2.0f;
    float new_cy = new_h / 2.0f;

    for (int y = 0; y < new_h; ++y) {
        for (int x = 0; x < new_w; ++x) {
            // 逆变换
            float src_x = (x - new_cx) * cos_a + (y - new_cy) * sin_a + cx;
            float src_y = -(x - new_cx) * sin_a + (y - new_cy) * cos_a + cy;

            // 双线性插值
            if (src_x >= 0 && src_x < width - 1 && src_y >= 0 && src_y < height - 1) {
                int x0 = static_cast<int>(src_x);
                int y0 = static_cast<int>(src_y);
                float dx = src_x - x0;
                float dy = src_y - y0;

                float v00 = src[y0 * width + x0];
                float v01 = src[y0 * width + x0 + 1];
                float v10 = src[(y0 + 1) * width + x0];
                float v11 = src[(y0 + 1) * width + x0 + 1];

                dst[y * new_w + x] = static_cast<uint8_t>(
                    v00 * (1 - dx) * (1 - dy) + v01 * dx * (1 - dy) +
                    v10 * (1 - dx) * dy + v11 * dx * dy
                );
            }
        }
    }
}

void scale_image(const uint8_t* src, int width, int height,
                 std::vector<uint8_t>& dst, float scale) {
    int new_w = static_cast<int>(width * scale + 0.5f);
    int new_h = static_cast<int>(height * scale + 0.5f);
    if (new_w < 1) new_w = 1;
    if (new_h < 1) new_h = 1;

    dst.resize(new_w * new_h);
    image_utils::resize_bilinear(src, width, height, 1, dst, new_w, new_h);
}

float compute_ncc(const uint8_t* src, int src_w, int src_h,
                  const uint8_t* tmpl, int tmpl_w, int tmpl_h,
                  int x, int y) {
    if (x < 0 || y < 0 || x + tmpl_w > src_w || y + tmpl_h > src_h) {
        return -1.0f;
    }

    int tmpl_size = tmpl_w * tmpl_h;

    // 计算模板均值
    double tmpl_sum = 0.0;
    for (int i = 0; i < tmpl_size; ++i) {
        tmpl_sum += tmpl[i];
    }
    double tmpl_mean = tmpl_sum / tmpl_size;

    // 计算模板方差
    double tmpl_var = 0.0;
    for (int i = 0; i < tmpl_size; ++i) {
        double diff = tmpl[i] - tmpl_mean;
        tmpl_var += diff * diff;
    }
    double tmpl_std = std::sqrt(tmpl_var);

    // 计算源图像区域均值
    double src_sum = 0.0;
    for (int ty = 0; ty < tmpl_h; ++ty) {
        for (int tx = 0; tx < tmpl_w; ++tx) {
            src_sum += src[(y + ty) * src_w + (x + tx)];
        }
    }
    double src_mean = src_sum / tmpl_size;

    // 计算NCC
    double numerator = 0.0;
    double src_var = 0.0;

    for (int ty = 0; ty < tmpl_h; ++ty) {
        for (int tx = 0; tx < tmpl_w; ++tx) {
            int s_idx = (y + ty) * src_w + (x + tx);
            int t_idx = ty * tmpl_w + tx;

            double src_diff = src[s_idx] - src_mean;
            double tmpl_diff = tmpl[t_idx] - tmpl_mean;

            numerator += src_diff * tmpl_diff;
            src_var += src_diff * src_diff;
        }
    }

    double src_std = std::sqrt(src_var);
    double denominator = src_std * tmpl_std;

    if (denominator < 1e-10) {
        return 0.0f;
    }

    return static_cast<float>(numerator / denominator);
}

float compute_sad(const uint8_t* src, int src_w, int src_h,
                  const uint8_t* tmpl, int tmpl_w, int tmpl_h,
                  int x, int y) {
    if (x < 0 || y < 0 || x + tmpl_w > src_w || y + tmpl_h > src_h) {
        return std::numeric_limits<float>::max();
    }

    uint32_t sad = 0;
    for (int ty = 0; ty < tmpl_h; ++ty) {
        for (int tx = 0; tx < tmpl_w; ++tx) {
            int s_idx = (y + ty) * src_w + (x + tx);
            int t_idx = ty * tmpl_w + tx;
            int diff = static_cast<int>(src[s_idx]) - static_cast<int>(tmpl[t_idx]);
            sad += static_cast<uint32_t>(std::abs(diff));
        }
    }

    return static_cast<float>(sad) / (tmpl_w * tmpl_h);
}

void pyramid_search(const uint8_t* src, int width, int height,
                    const ShapeModel& model,
                    std::vector<AdvancedMatchResult>& results,
                    float min_score, int max_matches) {
    results.clear();

    // 构建图像金字塔
    std::vector<std::vector<uint8_t>> pyramid;
    build_image_pyramid(src, width, height, pyramid, model.num_levels);

    // 从最顶层开始搜索
    std::vector<std::pair<int, int>> candidates;

    int top_level = static_cast<int>(pyramid.size()) - 1;
    int pyr_w = width >> top_level;
    int pyr_h = height >> top_level;
    int tmpl_w = model.width >> top_level;
    int tmpl_h = model.height >> top_level;

    if (tmpl_w < 4 || tmpl_h < 4) {
        top_level = 0;
        pyr_w = width;
        pyr_h = height;
        tmpl_w = model.width;
        tmpl_h = model.height;
    }

    // 顶层粗搜索
    for (int y = 0; y <= pyr_h - tmpl_h; y += 2) {
        for (int x = 0; x <= pyr_w - tmpl_w; x += 2) {
            float score = compute_ncc(pyramid[top_level].data(), pyr_w, pyr_h,
                                      model.template_data.data(), model.width, model.height,
                                      x << top_level, y << top_level);
            if (score >= min_score * 0.5f) {
                candidates.push_back({x << top_level, y << top_level});
            }
        }
    }

    // 从顶层向下细化
    for (int level = top_level - 1; level >= 0; --level) {
        std::vector<std::pair<int, int>> new_candidates;

        int scale_factor = 1 << level;
        int cur_w = width >> level;
        int cur_h = height >> level;
        int cur_tmpl_w = model.width >> level;
        int cur_tmpl_h = model.height >> level;

        for (auto& cand : candidates) {
            // 细化搜索范围
            int search_range = (level == 0) ? 2 : 4;

            for (int dy = -search_range; dy <= search_range; ++dy) {
                for (int dx = -search_range; dx <= search_range; ++dx) {
                    int new_x = (cand.first >> 1) + dx * scale_factor;
                    int new_y = (cand.second >> 1) + dy * scale_factor;

                    if (new_x < 0 || new_y < 0 ||
                        new_x + cur_tmpl_w > cur_w ||
                        new_y + cur_tmpl_h > cur_h) {
                        continue;
                    }

                    float score = compute_ncc(src, width, height,
                                              model.template_data.data(), model.width, model.height,
                                              new_x, new_y);

                    if (score >= min_score) {
                        new_candidates.push_back({new_x, new_y});
                    }
                }
            }
        }

        candidates = new_candidates;
    }

    // 转换为结果
    for (auto& cand : candidates) {
        AdvancedMatchResult result;
        result.x = static_cast<float>(cand.first);
        result.y = static_cast<float>(cand.second);
        result.score = compute_ncc(src, width, height,
                                   model.template_data.data(), model.width, model.height,
                                   cand.first, cand.second);
        result.angle = 0.0f;
        result.scale = 1.0f;
        result.model_id = model.model_id;
        results.push_back(result);
    }

    // 按分数排序
    std::sort(results.begin(), results.end(),
              [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                  return a.score > b.score;
              });

    // 非极大值抑制
    non_max_suppression(results, model.max_overlap, width, height);

    // 限制数量
    if (results.size() > max_matches) {
        results.resize(max_matches);
    }
}

void detect_feature_points(const uint8_t* src, int width, int height,
                           std::vector<FeaturePoint>& features,
                           int max_features) {
    features.clear();

    // 使用简化版的Harris角点检测
    std::vector<float> grad_x, grad_y;
    compute_gradient(src, width, height, grad_x, grad_y);

    std::vector<float> Ixx(width * height, 0.0f);
    std::vector<float> Iyy(width * height, 0.0f);
    std::vector<float> Ixy(width * height, 0.0f);

    for (int i = 0; i < width * height; ++i) {
        Ixx[i] = grad_x[i] * grad_x[i];
        Iyy[i] = grad_y[i] * grad_y[i];
        Ixy[i] = grad_x[i] * grad_y[i];
    }

    // 计算Harris响应
    std::vector<float> harris_response(width * height, 0.0f);
    int window_size = 5;
    float k = 0.04f;

    for (int y = window_size; y < height - window_size; ++y) {
        for (int x = window_size; x < width - window_size; ++x) {
            float sum_Ixx = 0.0f;
            float sum_Iyy = 0.0f;
            float sum_Ixy = 0.0f;

            for (int wy = -window_size; wy <= window_size; ++wy) {
                for (int wx = -window_size; wx <= window_size; ++wx) {
                    int idx = (y + wy) * width + (x + wx);
                    sum_Ixx += Ixx[idx];
                    sum_Iyy += Iyy[idx];
                    sum_Ixy += Ixy[idx];
                }
            }

            float det = sum_Ixx * sum_Iyy - sum_Ixy * sum_Ixy;
            float trace = sum_Ixx + sum_Iyy;
            harris_response[y * width + x] = det - k * trace * trace;
        }
    }

    // 非极大值抑制
    float threshold = 1000.0f;
    int nms_size = 3;

    for (int y = nms_size; y < height - nms_size; ++y) {
        for (int x = nms_size; x < width - nms_size; ++x) {
            float response = harris_response[y * width + x];
            if (response < threshold) continue;

            bool is_max = true;
            for (int ny = -nms_size; ny <= nms_size && is_max; ++ny) {
                for (int nx = -nms_size; nx <= nms_size && is_max; ++nx) {
                    if (nx == 0 && ny == 0) continue;
                    if (harris_response[(y + ny) * width + (x + nx)] > response) {
                        is_max = false;
                    }
                }
            }

            if (is_max) {
                FeaturePoint fp;
                fp.x = static_cast<float>(x);
                fp.y = static_cast<float>(y);
                fp.scale = 1.0f;
                fp.orientation = std::atan2(grad_y[y * width + x], grad_x[y * width + x]);
                fp.response = response;
                features.push_back(fp);
            }
        }
    }

    // 按响应值排序并限制数量
    std::sort(features.begin(), features.end(),
              [](const FeaturePoint& a, const FeaturePoint& b) {
                  return a.response > b.response;
              });

    if (features.size() > max_features) {
        features.resize(max_features);
    }
}

void compute_descriptors(const uint8_t* src, int width, int height,
                         std::vector<FeaturePoint>& features) {
    const int descriptor_size = 32;  // 简化描述符大小（SIFT为128）

    std::vector<float> grad_x, grad_y;
    compute_gradient(src, width, height, grad_x, grad_y);

    for (auto& fp : features) {
        fp.descriptor.resize(descriptor_size, 0.0f);

        int cx = static_cast<int>(fp.x);
        int cy = static_cast<int>(fp.y);
        int radius = 8;

        // 在特征点周围8x8区域内计算梯度方向直方图
        std::vector<float> hist(8, 0.0f);

        for (int dy = -radius; dy < radius; ++dy) {
            for (int dx = -radius; dx < radius; ++dx) {
                int px = cx + dx;
                int py = cy + dy;
                if (px < 0 || px >= width || py < 0 || py >= height) continue;

                int idx = py * width + px;
                float gx = grad_x[idx];
                float gy = grad_y[idx];
                float magnitude = std::sqrt(gx * gx + gy * gy);
                float orientation = std::atan2(gy, gx) + static_cast<float>(M_PI);

                int bin = static_cast<int>(orientation * 4 / static_cast<float>(M_PI));
                bin = bin % 8;
                hist[bin] += magnitude;
            }
        }

        // 归一化
        float sum = 0.0f;
        for (float h : hist) sum += h;
        if (sum > 0) {
            for (float& h : hist) h /= sum;
        }

        // 存入描述符
        for (int i = 0; i < 8; ++i) {
            fp.descriptor[i] = hist[i];
        }

        // 扩展到32维（复制并加噪声）
        for (int i = 8; i < descriptor_size; ++i) {
            fp.descriptor[i] = hist[i % 8] * (1.0f + 0.01f * (i - 8));
        }
    }
}

void match_features(const std::vector<FeaturePoint>& features1,
                    const std::vector<FeaturePoint>& features2,
                    std::vector<std::pair<int, int>>& matches,
                    float threshold) {
    matches.clear();

    for (size_t i = 0; i < features1.size(); ++i) {
        float best_dist = std::numeric_limits<float>::max();
        float second_dist = std::numeric_limits<float>::max();
        int best_idx = -1;

        for (size_t j = 0; j < features2.size(); ++j) {
            // 计算描述符距离（欧氏距离）
            float dist = 0.0f;
            for (size_t k = 0; k < features1[i].descriptor.size(); ++k) {
                float diff = features1[i].descriptor[k] - features2[j].descriptor[k];
                dist += diff * diff;
            }
            dist = std::sqrt(dist);

            if (dist < best_dist) {
                second_dist = best_dist;
                best_dist = dist;
                best_idx = static_cast<int>(j);
            } else if (dist < second_dist) {
                second_dist = dist;
            }
        }

        // Lowe's ratio test
        if (best_idx >= 0 && best_dist < threshold * second_dist) {
            matches.push_back({static_cast<int>(i), best_idx});
        }
    }
}

void non_max_suppression(std::vector<AdvancedMatchResult>& results,
                         float max_overlap, int width, int height) {
    if (results.empty()) return;

    std::vector<bool> suppressed(results.size(), false);

    for (size_t i = 0; i < results.size(); ++i) {
        if (suppressed[i]) continue;

        for (size_t j = i + 1; j < results.size(); ++j) {
            if (suppressed[j]) continue;

            // 计算重叠率
            float dx = std::abs(results[i].x - results[j].x);
            float dy = std::abs(results[i].y - results[j].y);

            // 简化：假设模板大小相同
            float overlap_w = std::max(0.0f, static_cast<float>(width) - dx);
            float overlap_h = std::max(0.0f, static_cast<float>(height) - dy);
            float overlap = (overlap_w * overlap_h) / (width * height);

            if (overlap > max_overlap) {
                suppressed[j] = true;
            }
        }
    }

    // 移除被抑制的结果
    std::vector<AdvancedMatchResult> filtered;
    for (size_t i = 0; i < results.size(); ++i) {
        if (!suppressed[i]) {
            filtered.push_back(results[i]);
        }
    }
    results = filtered;
}

void refine_position(const uint8_t* src, int src_w, int src_h,
                     const uint8_t* tmpl, int tmpl_w, int tmpl_h,
                     float& x, float& y) {
    // 使用3x3邻域的曲面拟合进行亚像素细化
    int ix = static_cast<int>(x);
    int iy = static_cast<int>(y);

    if (ix < 1 || iy < 1 || ix >= src_w - tmpl_w - 1 ||
        iy >= src_h - tmpl_h - 1) {
        return;
    }

    // 计算中心及周围8个点的匹配分数
    std::vector<float> scores(9);
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int idx = (dy + 1) * 3 + (dx + 1);
            scores[idx] = compute_ncc(src, src_w, src_h, tmpl, tmpl_w, tmpl_h,
                                      ix + dx, iy + dy);
        }
    }

    // 计算梯度
    float dx = (scores[5] - scores[3]) / 2.0f;
    float dy = (scores[7] - scores[1]) / 2.0f;

    // 计算二阶导数
    float dxx = scores[3] - 2 * scores[4] + scores[5];
    float dyy = scores[1] - 2 * scores[4] + scores[7];

    if (std::abs(dxx) > 1e-6f) {
        x = ix - dx / dxx;
    }
    if (std::abs(dyy) > 1e-6f) {
        y = iy - dy / dyy;
    }
}

} // namespace advanced_match_utils

// ========== 辅助函数：灰度转换 ==========

static void to_gray(const ImageData& input, std::vector<uint8_t>& gray,
                    int& width, int& height) {
    width = input.width;
    height = input.height;
    gray.resize(width * height);

    if (input.channels == 1) {
        gray = input.data;
    } else if (input.channels >= 3) {
        for (size_t i = 0; i < gray.size(); ++i) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            gray[i] = static_cast<uint8_t>(0.11f * b + 0.59f * g + 0.30f * r);
        }
    }
}

// ========== ShapeMatchScaleNode ==========

ShapeMatchScaleNode::ShapeMatchScaleNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeMatchScaleNode::make_info() {
    NodeInfo info;
    info.id = "ShapeMatchScale";
    info.name = "带缩放的形状匹配";
    info.category = "高级匹配";
    info.description = "支持缩放的基于形状的模板匹配（缩放范围0.5-2.0）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("matches", "匹配结果列表", DataType::String));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("best_scale", "最佳缩放比例", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("scale_min", "最小缩放", DataType::Number, Data(0.8f)));
    info.params.push_back(ParamDef("scale_max", "最大缩放", DataType::Number, Data(1.2f)));
    info.params.push_back(ParamDef("scale_step", "缩放步长", DataType::Number, Data(0.05f)));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数量", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("max_overlap", "最大重叠率", DataType::Number, Data(0.5f)));

    return info;
}

Result<void> ShapeMatchScaleNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    auto tmpl_data = get_input("template");
    if (!tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "模板不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    if (input.empty() || tmpl.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "图像为空");
    }

    // 转灰度
    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    // 参数
    float scale_min = get_param("scale_min", Data(0.8f)).as_number();
    float scale_max = get_param("scale_max", Data(1.2f)).as_number();
    float scale_step = get_param("scale_step", Data(0.05f)).as_number();
    float min_score = get_param("min_score", Data(0.7f)).as_number();
    int num_matches = get_param("num_matches", Data(5)).as_int();
    float max_overlap = get_param("max_overlap", Data(0.5f)).as_number();

    results_.clear();

    // 多尺度搜索
    for (float scale = scale_min; scale <= scale_max; scale += scale_step) {
        std::vector<uint8_t> scaled_tmpl;
        advanced_match_utils::scale_image(tmpl_gray.data(), tmpl_w, tmpl_h,
                                          scaled_tmpl, scale);

        int scaled_w = static_cast<int>(tmpl_w * scale);
        int scaled_h = static_cast<int>(tmpl_h * scale);

        if (scaled_w < 4 || scaled_h < 4 || scaled_w > src_w || scaled_h > src_h) {
            continue;
        }

        // 搜索
        for (int y = 0; y <= src_h - scaled_h; y += 4) {
            for (int x = 0; x <= src_w - scaled_w; x += 4) {
                float score = advanced_match_utils::compute_ncc(
                    src_gray.data(), src_w, src_h,
                    scaled_tmpl.data(), scaled_w, scaled_h,
                    x, y);

                if (score >= min_score) {
                    AdvancedMatchResult result;
                    result.x = static_cast<float>(x);
                    result.y = static_cast<float>(y);
                    result.score = score;
                    result.angle = 0.0f;
                    result.scale = scale;
                    results_.push_back(result);
                }
            }
        }
    }

    // 排序和非极大值抑制
    std::sort(results_.begin(), results_.end(),
              [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                  return a.score > b.score;
              });

    advanced_match_utils::non_max_suppression(results_, max_overlap, tmpl_w, tmpl_h);

    if (results_.size() > num_matches) {
        results_.resize(num_matches);
    }

    // 亚像素细化
    for (auto& result : results_) {
        advanced_match_utils::refine_position(
            src_gray.data(), src_w, src_h,
            tmpl_gray.data(), tmpl_w, tmpl_h,
            result.x, result.y);
    }

    // 输出
    ImageData output = input;
    for (const auto& result : results_) {
        int w = static_cast<int>(tmpl_w * result.scale);
        int h = static_cast<int>(tmpl_h * result.scale);
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y), w, h, 0, 255, 0);
    }

    String match_str = "Found " + std::to_string(results_.size()) + " matches";
    set_output("matches", Data(match_str));

    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
        set_output("best_scale", Data(results_[0].scale));
    } else {
        set_output("best_x", Data(0));
        set_output("best_y", Data(0));
        set_output("best_score", Data(0.0f));
        set_output("best_scale", Data(1.0f));
    }

    set_output("image", Data(output));

    OVF_INFO() << "ShapeMatchScale: " << results_.size() << " matches found";

    return Result<void>::success();
}

// ========== ShapeMatchRotateNode ==========

ShapeMatchRotateNode::ShapeMatchRotateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeMatchRotateNode::make_info() {
    NodeInfo info;
    info.id = "ShapeMatchRotate";
    info.name = "带旋转的形状匹配";
    info.category = "高级匹配";
    info.description = "支持旋转的基于形状的模板匹配（旋转范围0-360度）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("matches", "匹配结果列表", DataType::String));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("best_angle", "最佳旋转角度", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("angle_start", "起始角度", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("angle_extent", "角度范围", DataType::Number, Data(360.0f)));
    info.params.push_back(ParamDef("angle_step", "角度步长", DataType::Number, Data(10.0f)));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数量", DataType::Number, Data(5)));

    return info;
}

Result<void> ShapeMatchRotateNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    auto tmpl_data = get_input("template");
    if (!tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "模板不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    float angle_start = get_param("angle_start", Data(0.0f)).as_number();
    float angle_extent = get_param("angle_extent", Data(360.0f)).as_number();
    float angle_step = get_param("angle_step", Data(10.0f)).as_number();
    float min_score = get_param("min_score", Data(0.7f)).as_number();
    int num_matches = get_param("num_matches", Data(5)).as_int();

    results_.clear();

    // 多角度搜索
    for (float angle = angle_start; angle <= angle_start + angle_extent; angle += angle_step) {
        std::vector<uint8_t> rotated_tmpl;
        advanced_match_utils::rotate_image(tmpl_gray.data(), tmpl_w, tmpl_h,
                                           rotated_tmpl, angle);

        // 计算旋转后的尺寸
        float angle_rad = angle * static_cast<float>(M_PI) / 180.0f;
        float cos_a = std::abs(std::cos(angle_rad));
        float sin_a = std::abs(std::sin(angle_rad));
        int rot_w = static_cast<int>(tmpl_w * cos_a + tmpl_h * sin_a);
        int rot_h = static_cast<int>(tmpl_w * sin_a + tmpl_h * cos_a);

        if (rot_w > src_w || rot_h > src_h) continue;

        // 搜索
        for (int y = 0; y <= src_h - rot_h; y += 8) {
            for (int x = 0; x <= src_w - rot_w; x += 8) {
                float score = advanced_match_utils::compute_ncc(
                    src_gray.data(), src_w, src_h,
                    rotated_tmpl.data(), rot_w, rot_h,
                    x, y);

                if (score >= min_score) {
                    AdvancedMatchResult result;
                    result.x = static_cast<float>(x);
                    result.y = static_cast<float>(y);
                    result.score = score;
                    result.angle = angle;
                    result.scale = 1.0f;
                    results_.push_back(result);
                }
            }
        }
    }

    std::sort(results_.begin(), results_.end(),
              [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                  return a.score > b.score;
              });

    advanced_match_utils::non_max_suppression(results_, 0.5f, tmpl_w, tmpl_h);

    if (results_.size() > num_matches) {
        results_.resize(num_matches);
    }

    ImageData output = input;
    for (const auto& result : results_) {
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y), tmpl_w, tmpl_h, 0, 255, 0);
    }

    set_output("matches", Data("Found " + std::to_string(results_.size()) + " matches"));
    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
        set_output("best_angle", Data(results_[0].angle));
    } else {
        set_output("best_x", Data(0));
        set_output("best_y", Data(0));
        set_output("best_score", Data(0.0f));
        set_output("best_angle", Data(0.0f));
    }
    set_output("image", Data(output));

    OVF_INFO() << "ShapeMatchRotate: " << results_.size() << " matches found";

    return Result<void>::success();
}

// ========== ShapeMatchScaleRotateNode ==========

ShapeMatchScaleRotateNode::ShapeMatchScaleRotateNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeMatchScaleRotateNode::make_info() {
    NodeInfo info;
    info.id = "ShapeMatchScaleRotate";
    info.name = "带缩放+旋转的形状匹配";
    info.category = "高级匹配";
    info.description = "支持缩放和旋转的形状匹配（最完整的形状匹配）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("matches", "匹配结果", DataType::String));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("best_angle", "最佳旋转角度", DataType::Number));
    info.outputs.push_back(DataPort("best_scale", "最佳缩放比例", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("scale_min", "最小缩放", DataType::Number, Data(0.8f)));
    info.params.push_back(ParamDef("scale_max", "最大缩放", DataType::Number, Data(1.2f)));
    info.params.push_back(ParamDef("scale_step", "缩放步长", DataType::Number, Data(0.1f)));
    info.params.push_back(ParamDef("angle_start", "起始角度", DataType::Number, Data(-30.0f)));
    info.params.push_back(ParamDef("angle_extent", "角度范围", DataType::Number, Data(60.0f)));
    info.params.push_back(ParamDef("angle_step", "角度步长", DataType::Number, Data(15.0f)));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.6f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数量", DataType::Number, Data(3)));

    return info;
}

Result<void> ShapeMatchScaleRotateNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto tmpl_data = get_input("template");

    if (!input_data.is_image() || !tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    float scale_min = get_param("scale_min", Data(0.8f)).as_number();
    float scale_max = get_param("scale_max", Data(1.2f)).as_number();
    float scale_step = get_param("scale_step", Data(0.1f)).as_number();
    float angle_start = get_param("angle_start", Data(-30.0f)).as_number();
    float angle_extent = get_param("angle_extent", Data(60.0f)).as_number();
    float angle_step = get_param("angle_step", Data(15.0f)).as_number();
    float min_score = get_param("min_score", Data(0.6f)).as_number();
    int num_matches = get_param("num_matches", Data(3)).as_int();

    results_.clear();

    // 多尺度+多角度搜索
    for (float scale = scale_min; scale <= scale_max; scale += scale_step) {
        // 先缩放
        std::vector<uint8_t> scaled_tmpl;
        advanced_match_utils::scale_image(tmpl_gray.data(), tmpl_w, tmpl_h,
                                          scaled_tmpl, scale);
        int scaled_w = static_cast<int>(tmpl_w * scale);
        int scaled_h = static_cast<int>(tmpl_h * scale);

        for (float angle = angle_start; angle <= angle_start + angle_extent; angle += angle_step) {
            // 再旋转
            std::vector<uint8_t> rotated_tmpl;
            advanced_match_utils::rotate_image(scaled_tmpl.data(), scaled_w, scaled_h,
                                               rotated_tmpl, angle);

            float angle_rad = angle * static_cast<float>(M_PI) / 180.0f;
            int rot_w = static_cast<int>(std::abs(scaled_w * std::cos(angle_rad)) +
                                         std::abs(scaled_h * std::sin(angle_rad)));
            int rot_h = static_cast<int>(std::abs(scaled_w * std::sin(angle_rad)) +
                                         std::abs(scaled_h * std::cos(angle_rad)));

            if (rot_w > src_w || rot_h > src_h) continue;

            // 粗搜索
            for (int y = 0; y <= src_h - rot_h; y += 16) {
                for (int x = 0; x <= src_w - rot_w; x += 16) {
                    float score = advanced_match_utils::compute_ncc(
                        src_gray.data(), src_w, src_h,
                        rotated_tmpl.data(), rot_w, rot_h,
                        x, y);

                    if (score >= min_score) {
                        AdvancedMatchResult result;
                        result.x = static_cast<float>(x);
                        result.y = static_cast<float>(y);
                        result.score = score;
                        result.angle = angle;
                        result.scale = scale;
                        results_.push_back(result);
                    }
                }
            }
        }
    }

    std::sort(results_.begin(), results_.end(),
              [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                  return a.score > b.score;
              });

    advanced_match_utils::non_max_suppression(results_, 0.3f, tmpl_w, tmpl_h);

    if (results_.size() > num_matches) {
        results_.resize(num_matches);
    }

    ImageData output = input;
    for (const auto& result : results_) {
        int w = static_cast<int>(tmpl_w * result.scale);
        int h = static_cast<int>(tmpl_h * result.scale);
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y), w, h, 255, 0, 0);
    }

    set_output("matches", Data("Found " + std::to_string(results_.size()) + " matches"));
    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
        set_output("best_angle", Data(results_[0].angle));
        set_output("best_scale", Data(results_[0].scale));
    }
    set_output("image", Data(output));

    OVF_INFO() << "ShapeMatchScaleRotate: " << results_.size() << " matches found";

    return Result<void>::success();
}

// ========== CreateShapeModelNode ==========

CreateShapeModelNode::CreateShapeModelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CreateShapeModelNode::make_info() {
    NodeInfo info;
    info.id = "CreateShapeModel";
    info.name = "创建形状模板";
    info.category = "高级匹配";
    info.description = "从图像创建基于形状的模板模型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("model_id", "模型ID", DataType::Number));
    info.outputs.push_back(DataPort("num_edges", "边缘点数量", DataType::Number));
    info.outputs.push_back(DataPort("width", "模板宽度", DataType::Number));
    info.outputs.push_back(DataPort("height", "模板高度", DataType::Number));

    info.params.push_back(ParamDef("roi_x", "ROI起始X", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("roi_y", "ROI起始Y", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("roi_width", "ROI宽度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("roi_height", "ROI高度", DataType::Number, Data(100)));
    info.params.push_back(ParamDef("num_levels", "金字塔层数", DataType::Number, Data(4)));
    info.params.push_back(ParamDef("min_contrast", "最小对比度", DataType::Number, Data(30.0f)));
    info.params.push_back(ParamDef("angle_start", "起始角度", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("angle_extent", "角度范围", DataType::Number, Data(360.0f)));
    info.params.push_back(ParamDef("angle_step", "角度步长", DataType::Number, Data(1.0f)));

    return info;
}

Result<void> CreateShapeModelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();

    // 参数
    int roi_x = get_param("roi_x", Data(0)).as_int();
    int roi_y = get_param("roi_y", Data(0)).as_int();
    int roi_width = get_param("roi_width", Data(100)).as_int();
    int roi_height = get_param("roi_height", Data(100)).as_int();
    int num_levels = get_param("num_levels", Data(4)).as_int();
    float min_contrast = get_param("min_contrast", Data(30.0f)).as_number();
    float angle_start = get_param("angle_start", Data(0.0f)).as_number();
    float angle_extent = get_param("angle_extent", Data(360.0f)).as_number();
    float angle_step = get_param("angle_step", Data(1.0f)).as_number();

    // 边界检查
    if (roi_x < 0 || roi_y < 0 ||
        roi_x + roi_width > input.width ||
        roi_y + roi_height > input.height) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "ROI超出图像边界");
    }

    // 提取ROI
    std::vector<uint8_t> roi_data(roi_width * roi_height);
    for (int y = 0; y < roi_height; ++y) {
        for (int x = 0; x < roi_width; ++x) {
            size_t src_idx = (roi_y + y) * input.width + (roi_x + x);
            if (input.channels >= 3) {
                src_idx *= input.channels;
                roi_data[y * roi_width + x] = static_cast<uint8_t>(
                    0.11f * input.data[src_idx] +
                    0.59f * input.data[src_idx + 1] +
                    0.30f * input.data[src_idx + 2]);
            } else {
                roi_data[y * roi_width + x] = input.data[src_idx];
            }
        }
    }

    // 创建模型
    static uint32_t next_model_id = 1;
    model_.model_id = next_model_id++;
    model_.width = roi_width;
    model_.height = roi_height;
    model_.num_levels = num_levels;
    model_.angle_start = angle_start;
    model_.angle_extent = angle_extent;
    model_.angle_step = angle_step;
    model_.min_contrast = min_contrast;
    model_.template_data = roi_data;

    // 检测边缘点
    advanced_match_utils::detect_edges(roi_data.data(), roi_width, roi_height,
                                        model_.edge_points, min_contrast);

    // 构建金字塔
    std::vector<std::vector<uint8_t>> pyramid;
    advanced_match_utils::build_image_pyramid(roi_data.data(), roi_width, roi_height,
                                               pyramid, num_levels);

    // 检测各层金字塔的边缘
    model_.pyramid_edges.resize(pyramid.size());
    for (size_t level = 0; level < pyramid.size(); ++level) {
        int level_w = roi_width >> level;
        int level_h = roi_height >> level;
        if (level_w < 8 || level_h < 8) break;

        advanced_match_utils::detect_edges(
            pyramid[level].data(), level_w, level_h,
            model_.pyramid_edges[level],
            min_contrast / (level + 1));
    }

    // 输出
    set_output("model_id", Data(static_cast<int>(model_.model_id)));
    set_output("num_edges", Data(static_cast<int>(model_.edge_points.size())));
    set_output("width", Data(static_cast<int>(model_.width)));
    set_output("height", Data(static_cast<int>(model_.height)));

    OVF_INFO() << "CreateShapeModel: model_id=" << model_.model_id
               << ", edges=" << model_.edge_points.size();

    return Result<void>::success();
}

// ========== FindShapeModelNode ==========

FindShapeModelNode::FindShapeModelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FindShapeModelNode::make_info() {
    NodeInfo info;
    info.id = "FindShapeModel";
    info.name = "查找形状模板";
    info.category = "高级匹配";
    info.description = "在图像中查找形状模板";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("model_id", "模型ID", DataType::Number, false));

    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("positions", "匹配位置列表", DataType::String));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("best_angle", "最佳旋转角度", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数量", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("max_overlap", "最大重叠率", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("angle_start", "搜索起始角度", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("angle_extent", "搜索角度范围", DataType::Number, Data(360.0f)));

    return info;
}

Result<void> FindShapeModelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();

    std::vector<uint8_t> src_gray;
    int src_w, src_h;
    to_gray(input, src_gray, src_w, src_h);

    float min_score = get_param("min_score", Data(0.7f)).as_number();
    int num_matches = get_param("num_matches", Data(1)).as_int();
    float max_overlap = get_param("max_overlap", Data(0.5f)).as_number();

    // 使用金字塔搜索
    advanced_match_utils::pyramid_search(src_gray.data(), src_w, src_h,
                                          model_, results_,
                                          min_score, num_matches);

    // 输出
    ImageData output = input;
    for (const auto& result : results_) {
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y),
                               model_.width, model_.height, 0, 255, 0);
    }

    set_output("num_matches", Data(static_cast<int>(results_.size())));

    String positions_str;
    for (const auto& r : results_) {
        positions_str += "(" + std::to_string(r.x) + "," + std::to_string(r.y) +
                         " s=" + std::to_string(r.score) + ") ";
    }
    set_output("positions", Data(positions_str));

    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
        set_output("best_angle", Data(results_[0].angle));
    } else {
        set_output("best_x", Data(0));
        set_output("best_y", Data(0));
        set_output("best_score", Data(0.0f));
        set_output("best_angle", Data(0.0f));
    }
    set_output("image", Data(output));

    OVF_INFO() << "FindShapeModel: " << results_.size() << " matches found";

    return Result<void>::success();
}

// ========== WriteShapeModelNode ==========

WriteShapeModelNode::WriteShapeModelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo WriteShapeModelNode::make_info() {
    NodeInfo info;
    info.id = "WriteShapeModel";
    info.name = "保存形状模板";
    info.category = "高级匹配";
    info.description = "将形状模板保存到文件";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model_id", "模型ID", DataType::Number, true));

    info.outputs.push_back(DataPort("success", "是否成功", DataType::Boolean));
    info.outputs.push_back(DataPort("filepath", "保存路径", DataType::String));

    info.params.push_back(ParamDef("filepath", "保存路径", DataType::String, Data("shape_model.bin")));

    return info;
}

Result<void> WriteShapeModelNode::execute(FlowContext& context) {
    String filepath = get_param("filepath", Data("shape_model.bin")).as_string();

    // 这里简化实现：假设model_id可以从上下文获取
    // 实际应用中需要从模型管理器获取模型
    ShapeModel dummy_model;
    dummy_model.model_id = 1;
    dummy_model.width = 100;
    dummy_model.height = 100;
    dummy_model.num_levels = 4;

    bool success = serialize_model(dummy_model, filepath);

    set_output("success", Data(success));
    set_output("filepath", Data(filepath));

    OVF_INFO() << "WriteShapeModel: " << (success ? "success" : "failed");

    return Result<void>::success();
}

bool WriteShapeModelNode::serialize_model(const ShapeModel& model, const String& filepath) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    // 写入头部信息
    file.write(reinterpret_cast<const char*>(&model.model_id), sizeof(model.model_id));
    file.write(reinterpret_cast<const char*>(&model.width), sizeof(model.width));
    file.write(reinterpret_cast<const char*>(&model.height), sizeof(model.height));
    file.write(reinterpret_cast<const char*>(&model.num_levels), sizeof(model.num_levels));
    file.write(reinterpret_cast<const char*>(&model.angle_start), sizeof(model.angle_start));
    file.write(reinterpret_cast<const char*>(&model.angle_extent), sizeof(model.angle_extent));
    file.write(reinterpret_cast<const char*>(&model.angle_step), sizeof(model.angle_step));
    file.write(reinterpret_cast<const char*>(&model.min_contrast), sizeof(model.min_contrast));

    // 写入边缘点数量和数据
    size_t num_edges = model.edge_points.size();
    file.write(reinterpret_cast<const char*>(&num_edges), sizeof(num_edges));

    for (const auto& edge : model.edge_points) {
        file.write(reinterpret_cast<const char*>(&edge.x), sizeof(edge.x));
        file.write(reinterpret_cast<const char*>(&edge.y), sizeof(edge.y));
        file.write(reinterpret_cast<const char*>(&edge.direction), sizeof(edge.direction));
        file.write(reinterpret_cast<const char*>(&edge.magnitude), sizeof(edge.magnitude));
    }

    // 写入模板图像数据
    size_t data_size = model.template_data.size();
    file.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
    file.write(reinterpret_cast<const char*>(model.template_data.data()), data_size);

    file.close();
    return true;
}

// ========== ComponentMatchNode ==========

ComponentMatchNode::ComponentMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ComponentMatchNode::make_info() {
    NodeInfo info;
    info.id = "ComponentMatch";
    info.name = "组件匹配";
    info.category = "高级匹配";
    info.description = "基于组件的模板匹配";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("num_components", "组件数量", DataType::Number, Data(4)));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));

    return info;
}

Result<void> ComponentMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto tmpl_data = get_input("template");

    if (!input_data.is_image() || !tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    // 简化实现：使用基本的模板匹配
    float min_score = get_param("min_score", Data(0.7f)).as_number();

    results_.clear();

    // 使用金字塔搜索
    ShapeModel temp_model;
    temp_model.model_id = 1;
    temp_model.width = tmpl_w;
    temp_model.height = tmpl_h;
    temp_model.num_levels = 3;
    temp_model.template_data = tmpl_gray;
    temp_model.max_overlap = 0.5f;

    advanced_match_utils::pyramid_search(src_gray.data(), src_w, src_h,
                                          temp_model, results_,
                                          min_score, 5);

    ImageData output = input;
    for (const auto& result : results_) {
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y),
                               tmpl_w, tmpl_h, 0, 255, 0);
    }

    set_output("num_matches", Data(static_cast<int>(results_.size())));
    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
    }
    set_output("image", Data(output));

    return Result<void>::success();
}

// ========== TrainComponentModelNode ==========

TrainComponentModelNode::TrainComponentModelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo TrainComponentModelNode::make_info() {
    NodeInfo info;
    info.id = "TrainComponentModel";
    info.name = "训练组件模型";
    info.category = "高级匹配";
    info.description = "从模板图像训练组件模型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("model_id", "模型ID", DataType::Number));
    info.outputs.push_back(DataPort("num_components", "组件数量", DataType::Number));
    info.outputs.push_back(DataPort("model_info", "模型信息", DataType::String));

    info.params.push_back(ParamDef("num_components", "组件数量", DataType::Number, Data(4)));
    info.params.push_back(ParamDef("min_contrast", "最小对比度", DataType::Number, Data(30.0f)));

    return info;
}

Result<void> TrainComponentModelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();

    std::vector<uint8_t> src_gray;
    int src_w, src_h;
    to_gray(input, src_gray, src_w, src_h);

    int num_components = get_param("num_components", Data(4)).as_int();
    float min_contrast = get_param("min_contrast", Data(30.0f)).as_number();

    // 创建组件模型
    static uint32_t next_model_id = 1;
    model_.model_id = next_model_id++;
    model_.num_components = num_components;
    model_.min_contrast = min_contrast;

    // 分解为组件
    decompose_into_components(src_gray.data(), src_w, src_h, num_components);

    set_output("model_id", Data(static_cast<int>(model_.model_id)));
    set_output("num_components", Data(model_.num_components));
    set_output("model_info", Data("Components: " + std::to_string(model_.num_components)));

    OVF_INFO() << "TrainComponentModel: model_id=" << model_.model_id;

    return Result<void>::success();
}

void TrainComponentModelNode::decompose_into_components(const uint8_t* src, int width, int height,
                                                         int num_components) {
    model_.components.clear();

    // 将图像划分为多个区域作为组件
    int grid_x = static_cast<int>(std::sqrt(static_cast<float>(num_components)));
    int grid_y = num_components / grid_x;

    int comp_w = width / grid_x;
    int comp_h = height / grid_y;

    int comp_id = 0;
    for (int gy = 0; gy < grid_y; ++gy) {
        for (int gx = 0; gx < grid_x; ++gx) {
            ComponentModel::Component comp;
            comp.component_id = comp_id++;
            comp.x = gx * comp_w - width / 2;
            comp.y = gy * comp_h - height / 2;
            comp.width = comp_w;
            comp.height = comp_h;

            comp.template_data.resize(comp_w * comp_h);
            for (int y = 0; y < comp_h; ++y) {
                for (int x = 0; x < comp_w; ++x) {
                    int src_x = gx * comp_w + x;
                    int src_y = gy * comp_h + y;
                    comp.template_data[y * comp_w + x] = src[src_y * width + src_x];
                }
            }

            // 检测边缘
            advanced_match_utils::detect_edges(comp.template_data.data(), comp_w, comp_h,
                                               comp.edge_points, model_.min_contrast);

            model_.components.push_back(comp);
        }
    }

    // 计算组件间关系
    for (size_t i = 0; i < model_.components.size(); ++i) {
        for (size_t j = i + 1; j < model_.components.size(); ++j) {
            ComponentModel::ComponentRelation rel;
            rel.comp1_id = model_.components[i].component_id;
            rel.comp2_id = model_.components[j].component_id;

            float dx = model_.components[j].x - model_.components[i].x;
            float dy = model_.components[j].y - model_.components[i].y;
            rel.distance = std::sqrt(dx * dx + dy * dy);
            rel.angle = std::atan2(dy, dx);

            model_.relations.push_back(rel);
        }
    }
}

// ========== FindComponentModelNode ==========

FindComponentModelNode::FindComponentModelNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FindComponentModelNode::make_info() {
    NodeInfo info;
    info.id = "FindComponentModel";
    info.name = "查找组件模型";
    info.category = "高级匹配";
    info.description = "在图像中查找组件模型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));

    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数量", DataType::Number, Data(1)));

    return info;
}

Result<void> FindComponentModelNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();

    std::vector<uint8_t> src_gray;
    int src_w, src_h;
    to_gray(input, src_gray, src_w, src_h);

    float min_score = get_param("min_score", Data(0.7f)).as_number();

    results_.clear();

    // 简化实现：匹配每个组件
    if (!model_.components.empty()) {
        auto& first_comp = model_.components[0];

        for (int y = 0; y <= src_h - first_comp.height; y += 4) {
            for (int x = 0; x <= src_w - first_comp.width; x += 4) {
                float score = advanced_match_utils::compute_ncc(
                    src_gray.data(), src_w, src_h,
                    first_comp.template_data.data(), first_comp.width, first_comp.height,
                    x, y);

                if (score >= min_score) {
                    AdvancedMatchResult result;
                    result.x = static_cast<float>(x + first_comp.width / 2);
                    result.y = static_cast<float>(y + first_comp.height / 2);
                    result.score = score;
                    result.angle = 0.0f;
                    result.scale = 1.0f;
                    result.model_id = model_.model_id;
                    results_.push_back(result);
                }
            }
        }
    }

    std::sort(results_.begin(), results_.end(),
              [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                  return a.score > b.score;
              });

    int num_matches = get_param("num_matches", Data(1)).as_int();
    if (results_.size() > num_matches) {
        results_.resize(num_matches);
    }

    ImageData output = input;
    for (const auto& result : results_) {
        image_utils::draw_circle(output, static_cast<int>(result.x),
                                 static_cast<int>(result.y), 10, 255, 0, 0);
    }

    set_output("num_matches", Data(static_cast<int>(results_.size())));
    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
    }
    set_output("image", Data(output));

    return Result<void>::success();
}

// ========== InspectComponentNode ==========

InspectComponentNode::InspectComponentNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo InspectComponentNode::make_info() {
    NodeInfo info;
    info.id = "InspectComponent";
    info.name = "组件检测";
    info.category = "高级匹配";
    info.description = "检测组件是否存在及其状态";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("expected_x", "期望位置X", DataType::Number, false));
    info.inputs.push_back(DataPort("expected_y", "期望位置Y", DataType::Number, false));

    info.outputs.push_back(DataPort("num_found", "找到的组件数量", DataType::Number));
    info.outputs.push_back(DataPort("inspection_result", "检测结果", DataType::String));
    info.outputs.push_back(DataPort("all_found", "是否全部找到", DataType::Boolean));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("tolerance", "位置容差", DataType::Number, Data(10.0f)));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.6f)));

    return info;
}

Result<void> InspectComponentNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();

    std::vector<uint8_t> src_gray;
    int src_w, src_h;
    to_gray(input, src_gray, src_w, src_h);

    float tolerance = get_param("tolerance", Data(10.0f)).as_number();
    float min_score = get_param("min_score", Data(0.6f)).as_number();

    // 简化实现：检测单个区域
    inspection_results_.clear();

    InspectionResult result;
    result.component_id = 0;
    result.found = false;
    result.score = 0.0f;
    result.x = 0;
    result.y = 0;
    result.status = "Not Found";

    // 在期望位置附近搜索
    int exp_x = get_input("expected_x").as_int();
    int exp_y = get_input("expected_y").as_int();

    if (exp_x == 0 && exp_y == 0) {
        // 如果没有期望位置，搜索整个图像
        for (int y = 0; y < src_h - 20; y += 10) {
            for (int x = 0; x < src_w - 20; x += 10) {
                // 计算区域均值作为简单检测
                float sum = 0.0f;
                for (int ty = 0; ty < 20; ++ty) {
                    for (int tx = 0; tx < 20; ++tx) {
                        sum += src_gray[(y + ty) * src_w + (x + tx)];
                    }
                }
                float mean = sum / 400.0f;

                if (mean > 100.0f) {
                    result.found = true;
                    result.x = x;
                    result.y = y;
                    result.score = mean / 255.0f;
                    result.status = "Found";
                    break;
                }
            }
            if (result.found) break;
        }
    } else {
        // 在期望位置附近搜索
        int search_range = static_cast<int>(tolerance);
        for (int dy = -search_range; dy <= search_range && !result.found; ++dy) {
            for (int dx = -search_range; dx <= search_range; ++dx) {
                int x = exp_x + dx;
                int y = exp_y + dy;
                if (x < 0 || y < 0 || x >= src_w - 20 || y >= src_h - 20) continue;

                float sum = 0.0f;
                for (int ty = 0; ty < 20; ++ty) {
                    for (int tx = 0; tx < 20; ++tx) {
                        sum += src_gray[(y + ty) * src_w + (x + tx)];
                    }
                }
                float mean = sum / 400.0f;

                if (mean / 255.0f >= min_score) {
                    result.found = true;
                    result.x = x;
                    result.y = y;
                    result.score = mean / 255.0f;
                    result.status = "Found (at expected position)";
                }
            }
        }
    }

    inspection_results_.push_back(result);

    int num_found = 0;
    for (const auto& r : inspection_results_) {
        if (r.found) num_found++;
    }

    bool all_found = num_found == static_cast<int>(inspection_results_.size());

    ImageData output = input;
    for (const auto& r : inspection_results_) {
        if (r.found) {
            image_utils::draw_circle(output, r.x + 10, r.y + 10, 15, 0, 255, 0);
        } else {
            image_utils::draw_circle(output, exp_x + 10, exp_y + 10, 15, 255, 0, 0);
        }
    }

    set_output("num_found", Data(num_found));
    set_output("inspection_result", Data(result.status));
    set_output("all_found", Data(all_found));
    set_output("image", Data(output));

    OVF_INFO() << "InspectComponent: " << num_found << " found";

    return Result<void>::success();
}

// ========== GrayMatchNode ==========

GrayMatchNode::GrayMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo GrayMatchNode::make_info() {
    NodeInfo info;
    info.id = "GrayMatch";
    info.name = "灰度匹配";
    info.category = "高级匹配";
    info.description = "基于灰度的模板匹配（NCC/SAD方法）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("best_x", "最佳匹配X", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳匹配Y", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("method", "匹配方法", DataType::String, Data("NCC")));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数量", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("use_pyramid", "使用金字塔加速", DataType::Boolean, Data(true)));

    return info;
}

Result<void> GrayMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto tmpl_data = get_input("template");

    if (!input_data.is_image() || !tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    String method = get_param("method", Data("NCC")).as_string();
    float min_score = get_param("min_score", Data(0.7f)).as_number();
    int num_matches = get_param("num_matches", Data(5)).as_int();
    bool use_pyramid = get_param("use_pyramid", Data(true)).as_bool();

    results_.clear();

    if (use_pyramid && tmpl_w >= 16 && tmpl_h >= 16) {
        // 使用金字塔搜索
        ShapeModel temp_model;
        temp_model.width = tmpl_w;
        temp_model.height = tmpl_h;
        temp_model.num_levels = 3;
        temp_model.template_data = tmpl_gray;
        temp_model.max_overlap = 0.5f;

        advanced_match_utils::pyramid_search(src_gray.data(), src_w, src_h,
                                              temp_model, results_,
                                              min_score, num_matches);
    } else {
        // 直接搜索
        for (int y = 0; y <= src_h - tmpl_h; y += 2) {
            for (int x = 0; x <= src_w - tmpl_w; x += 2) {
                float score;
                if (method == "NCC") {
                    score = advanced_match_utils::compute_ncc(
                        src_gray.data(), src_w, src_h,
                        tmpl_gray.data(), tmpl_w, tmpl_h, x, y);
                } else {
                    score = 1.0f - advanced_match_utils::compute_sad(
                        src_gray.data(), src_w, src_h,
                        tmpl_gray.data(), tmpl_w, tmpl_h, x, y) / 255.0f;
                }

                if (score >= min_score) {
                    AdvancedMatchResult result;
                    result.x = static_cast<float>(x);
                    result.y = static_cast<float>(y);
                    result.score = score;
                    result.angle = 0.0f;
                    result.scale = 1.0f;
                    results_.push_back(result);
                }
            }
        }

        std::sort(results_.begin(), results_.end(),
                  [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                      return a.score > b.score;
                  });

        advanced_match_utils::non_max_suppression(results_, 0.5f, tmpl_w, tmpl_h);

        if (results_.size() > num_matches) {
            results_.resize(num_matches);
        }
    }

    ImageData output = input;
    for (const auto& result : results_) {
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y), tmpl_w, tmpl_h, 0, 255, 0);
    }

    set_output("num_matches", Data(static_cast<int>(results_.size())));
    if (!results_.empty()) {
        set_output("best_x", Data(results_[0].x));
        set_output("best_y", Data(results_[0].y));
        set_output("best_score", Data(results_[0].score));
    }
    set_output("image", Data(output));

    OVF_INFO() << "GrayMatch: " << results_.size() << " matches found";

    return Result<void>::success();
}

// ========== BestMatchNode ==========

BestMatchNode::BestMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo BestMatchNode::make_info() {
    NodeInfo info;
    info.id = "BestMatch";
    info.name = "最佳匹配";
    info.category = "高级匹配";
    info.description = "寻找最佳的单个匹配目标";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("x", "匹配位置X", DataType::Number));
    info.outputs.push_back(DataPort("y", "匹配位置Y", DataType::Number));
    info.outputs.push_back(DataPort("score", "匹配分数", DataType::Number));
    info.outputs.push_back(DataPort("found", "是否找到", DataType::Boolean));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("method", "匹配方法", DataType::String, Data("NCC")));
    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.7f)));

    return info;
}

Result<void> BestMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto tmpl_data = get_input("template");

    if (!input_data.is_image() || !tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    String method = get_param("method", Data("NCC")).as_string();
    float min_score = get_param("min_score", Data(0.7f)).as_number();

    best_result_.score = -1.0f;
    best_result_.x = 0.0f;
    best_result_.y = 0.0f;

    // 金字塔搜索寻找最佳匹配
    ShapeModel temp_model;
    temp_model.width = tmpl_w;
    temp_model.height = tmpl_h;
    temp_model.num_levels = 4;
    temp_model.template_data = tmpl_gray;

    std::vector<AdvancedMatchResult> all_results;
    advanced_match_utils::pyramid_search(src_gray.data(), src_w, src_h,
                                          temp_model, all_results,
                                          min_score, 1);

    bool found = false;
    if (!all_results.empty()) {
        best_result_ = all_results[0];
        found = true;

        // 亚像素细化
        advanced_match_utils::refine_position(
            src_gray.data(), src_w, src_h,
            tmpl_gray.data(), tmpl_w, tmpl_h,
            best_result_.x, best_result_.y);
    }

    ImageData output = input;
    if (found) {
        image_utils::draw_rect(output, static_cast<int>(best_result_.x),
                               static_cast<int>(best_result_.y),
                               tmpl_w, tmpl_h, 0, 255, 0);
    }

    set_output("x", Data(best_result_.x));
    set_output("y", Data(best_result_.y));
    set_output("score", Data(best_result_.score));
    set_output("found", Data(found));
    set_output("image", Data(output));

    OVF_INFO() << "BestMatch: found=" << found << ", score=" << best_result_.score;

    return Result<void>::success();
}

// ========== AllMatchNode ==========

AllMatchNode::AllMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AllMatchNode::make_info() {
    NodeInfo info;
    info.id = "AllMatch";
    info.name = "所有匹配";
    info.category = "高级匹配";
    info.description = "寻找所有匹配目标（多目标匹配）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("positions", "匹配位置列表", DataType::String));
    info.outputs.push_back(DataPort("scores", "匹配分数列表", DataType::String));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("min_score", "最小匹配分数", DataType::Number, Data(0.6f)));
    info.params.push_back(ParamDef("max_overlap", "最大重叠率", DataType::Number, Data(0.3f)));
    info.params.push_back(ParamDef("max_matches", "最大匹配数量", DataType::Number, Data(20)));

    return info;
}

Result<void> AllMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto tmpl_data = get_input("template");

    if (!input_data.is_image() || !tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    float min_score = get_param("min_score", Data(0.6f)).as_number();
    float max_overlap = get_param("max_overlap", Data(0.3f)).as_number();
    int max_matches = get_param("max_matches", Data(20)).as_int();

    all_results_.clear();

    // 全图搜索
    for (int y = 0; y <= src_h - tmpl_h; y += 4) {
        for (int x = 0; x <= src_w - tmpl_w; x += 4) {
            float score = advanced_match_utils::compute_ncc(
                src_gray.data(), src_w, src_h,
                tmpl_gray.data(), tmpl_w, tmpl_h, x, y);

            if (score >= min_score) {
                AdvancedMatchResult result;
                result.x = static_cast<float>(x);
                result.y = static_cast<float>(y);
                result.score = score;
                all_results_.push_back(result);
            }
        }
    }

    // 排序和非极大值抑制
    std::sort(all_results_.begin(), all_results_.end(),
              [](const AdvancedMatchResult& a, const AdvancedMatchResult& b) {
                  return a.score > b.score;
              });

    advanced_match_utils::non_max_suppression(all_results_, max_overlap, tmpl_w, tmpl_h);

    if (all_results_.size() > max_matches) {
        all_results_.resize(max_matches);
    }

    ImageData output = input;
    for (const auto& result : all_results_) {
        image_utils::draw_rect(output, static_cast<int>(result.x),
                               static_cast<int>(result.y), tmpl_w, tmpl_h,
                               0, static_cast<uint8_t>(255 * result.score), 0);
    }

    String positions, scores;
    for (const auto& r : all_results_) {
        positions += "(" + std::to_string(static_cast<int>(r.x)) + "," +
                     std::to_string(static_cast<int>(r.y)) + ") ";
        scores += std::to_string(r.score) + " ";
    }

    set_output("num_matches", Data(static_cast<int>(all_results_.size())));
    set_output("positions", Data(positions));
    set_output("scores", Data(scores));
    set_output("image", Data(output));

    OVF_INFO() << "AllMatch: " << all_results_.size() << " matches found";

    return Result<void>::success();
}

// ========== DescriptorMatchNode ==========

DescriptorMatchNode::DescriptorMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo DescriptorMatchNode::make_info() {
    NodeInfo info;
    info.id = "DescriptorMatch";
    info.name = "描述符匹配";
    info.category = "高级匹配";
    info.description = "基于特征描述符的匹配（简化SIFT）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("template", "模板图像", DataType::Image, true));

    info.outputs.push_back(DataPort("num_matches", "匹配特征点数量", DataType::Number));
    info.outputs.push_back(DataPort("result_x", "结果位置X", DataType::Number));
    info.outputs.push_back(DataPort("result_y", "结果位置Y", DataType::Number));
    info.outputs.push_back(DataPort("result_angle", "结果角度", DataType::Number));
    info.outputs.push_back(DataPort("result_scale", "结果缩放", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("max_features", "最大特征点数量", DataType::Number, Data(300)));
    info.params.push_back(ParamDef("match_threshold", "匹配阈值", DataType::Number, Data(0.75f)));

    return info;
}

Result<void> DescriptorMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    auto tmpl_data = get_input("template");

    if (!input_data.is_image() || !tmpl_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();
    ImageData tmpl = tmpl_data.as_image();

    std::vector<uint8_t> src_gray, tmpl_gray;
    int src_w, src_h, tmpl_w, tmpl_h;
    to_gray(input, src_gray, src_w, src_h);
    to_gray(tmpl, tmpl_gray, tmpl_w, tmpl_h);

    int max_features = get_param("max_features", Data(300)).as_int();
    float match_threshold = get_param("match_threshold", Data(0.75f)).as_number();

    // 检测特征点
    advanced_match_utils::detect_feature_points(src_gray.data(), src_w, src_h,
                                                 image_features_, max_features);
    advanced_match_utils::detect_feature_points(tmpl_gray.data(), tmpl_w, tmpl_h,
                                                 template_features_, max_features);

    // 计算描述符
    advanced_match_utils::compute_descriptors(src_gray.data(), src_w, src_h, image_features_);
    advanced_match_utils::compute_descriptors(tmpl_gray.data(), tmpl_w, tmpl_h, template_features_);

    // 匹配特征点
    advanced_match_utils::match_features(template_features_, image_features_,
                                          matches_, match_threshold);

    result_.x = 0.0f;
    result_.y = 0.0f;
    result_.angle = 0.0f;
    result_.scale = 1.0f;
    result_.score = 0.0f;

    // 计算变换
    if (matches_.size() >= 4) {
        float tx = 0.0f, ty = 0.0f, angle = 0.0f, scale = 1.0f;

        // 简化的变换估计：使用匹配点的中心偏移
        float sum_dx = 0.0f, sum_dy = 0.0f;
        for (const auto& match : matches_) {
            const auto& tmpl_pt = template_features_[match.first];
            const auto& img_pt = image_features_[match.second];
            sum_dx += img_pt.x - tmpl_pt.x;
            sum_dy += img_pt.y - tmpl_pt.y;
        }

        result_.x = sum_dx / matches_.size();
        result_.y = sum_dy / matches_.size();
        result_.score = static_cast<float>(matches_.size()) / template_features_.size();
    }

    ImageData output = input;
    for (const auto& match : matches_) {
        const auto& img_pt = image_features_[match.second];
        image_utils::draw_circle(output, static_cast<int>(img_pt.x),
                                 static_cast<int>(img_pt.y), 5, 0, 255, 0);
    }

    set_output("num_matches", Data(static_cast<int>(matches_.size())));
    set_output("result_x", Data(result_.x));
    set_output("result_y", Data(result_.y));
    set_output("result_angle", Data(result_.angle));
    set_output("result_scale", Data(result_.scale));
    set_output("image", Data(output));

    OVF_INFO() << "DescriptorMatch: " << matches_.size() << " feature matches";

    return Result<void>::success();
}

// ========== FeaturePointMatchNode ==========

FeaturePointMatchNode::FeaturePointMatchNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FeaturePointMatchNode::make_info() {
    NodeInfo info;
    info.id = "FeaturePointMatch";
    info.name = "特征点匹配";
    info.category = "高级匹配";
    info.description = "特征点检测与匹配";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("reference", "参考图像", DataType::Image, false));

    info.outputs.push_back(DataPort("num_features", "特征点数量", DataType::Number));
    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("transform_tx", "平移X", DataType::Number));
    info.outputs.push_back(DataPort("transform_ty", "平移Y", DataType::Number));
    info.outputs.push_back(DataPort("transform_angle", "旋转角度", DataType::Number));
    info.outputs.push_back(DataPort("transform_scale", "缩放比例", DataType::Number));
    info.outputs.push_back(DataPort("image", "结果图像", DataType::Image));

    info.params.push_back(ParamDef("max_features", "最大特征点数量", DataType::Number, Data(500)));
    info.params.push_back(ParamDef("method", "特征类型", DataType::String, Data("Harris")));

    return info;
}

Result<void> FeaturePointMatchNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "输入不是图像");
    }

    ImageData input = input_data.as_image();

    std::vector<uint8_t> src_gray;
    int src_w, src_h;
    to_gray(input, src_gray, src_w, src_h);

    int max_features = get_param("max_features", Data(500)).as_int();

    // 检测特征点
    advanced_match_utils::detect_feature_points(src_gray.data(), src_w, src_h,
                                                 features_, max_features);

    float tx = 0.0f, ty = 0.0f, angle = 0.0f, scale = 1.0f;

    // 如果有参考图像，进行匹配
    auto ref_data = get_input("reference");
    if (ref_data.is_image()) {
        ImageData ref = ref_data.as_image();

        std::vector<uint8_t> ref_gray;
        int ref_w, ref_h;
        to_gray(ref, ref_gray, ref_w, ref_h);

        std::vector<FeaturePoint> ref_features;
        advanced_match_utils::detect_feature_points(ref_gray.data(), ref_w, ref_h,
                                                     ref_features, max_features);

        advanced_match_utils::compute_descriptors(src_gray.data(), src_w, src_h, features_);
        advanced_match_utils::compute_descriptors(ref_gray.data(), ref_w, ref_h, ref_features);

        advanced_match_utils::match_features(ref_features, features_, matches_, 0.75f);

        if (matches_.size() >= 4) {
            compute_transform(ref_features, features_, matches_, tx, ty, angle, scale);
        }
    } else {
        matches_.clear();
    }

    ImageData output = input;
    for (const auto& fp : features_) {
        image_utils::draw_circle(output, static_cast<int>(fp.x),
                                 static_cast<int>(fp.y), 3, 255, 0, 0);
    }

    for (const auto& match : matches_) {
        const auto& img_pt = features_[match.second];
        image_utils::draw_circle(output, static_cast<int>(img_pt.x),
                                 static_cast<int>(img_pt.y), 5, 0, 255, 0);
    }

    set_output("num_features", Data(static_cast<int>(features_.size())));
    set_output("num_matches", Data(static_cast<int>(matches_.size())));
    set_output("transform_tx", Data(tx));
    set_output("transform_ty", Data(ty));
    set_output("transform_angle", Data(angle));
    set_output("transform_scale", Data(scale));
    set_output("image", Data(output));

    OVF_INFO() << "FeaturePointMatch: " << features_.size() << " features, "
               << matches_.size() << " matches";

    return Result<void>::success();
}

bool FeaturePointMatchNode::compute_transform(
    const std::vector<FeaturePoint>& src_points,
    const std::vector<FeaturePoint>& dst_points,
    const std::vector<std::pair<int, int>>& matches,
    float& tx, float& ty, float& angle, float& scale) {

    if (matches.size() < 4) return false;

    // 计算平移（均值）
    float sum_dx = 0.0f, sum_dy = 0.0f;
    for (const auto& match : matches) {
        const auto& src_pt = src_points[match.first];
        const auto& dst_pt = dst_points[match.second];
        sum_dx += dst_pt.x - src_pt.x;
        sum_dy += dst_pt.y - src_pt.y;
    }
    tx = sum_dx / matches.size();
    ty = sum_dy / matches.size();

    // 计算缩放（距离比的均值）
    float sum_scale = 0.0f;
    int scale_count = 0;
    for (size_t i = 0; i < matches.size(); ++i) {
        for (size_t j = i + 1; j < matches.size(); ++j) {
            const auto& src1 = src_points[matches[i].first];
            const auto& src2 = src_points[matches[j].first];
            const auto& dst1 = dst_points[matches[i].second];
            const auto& dst2 = dst_points[matches[j].second];

            float src_dist = std::sqrt((src1.x - src2.x) * (src1.x - src2.x) +
                                       (src1.y - src2.y) * (src1.y - src2.y));
            float dst_dist = std::sqrt((dst1.x - dst2.x) * (dst1.x - dst2.x) +
                                       (dst1.y - dst2.y) * (dst1.y - dst2.y));

            if (src_dist > 1.0f) {
                sum_scale += dst_dist / src_dist;
                scale_count++;
            }
        }
    }
    if (scale_count > 0) {
        scale = sum_scale / scale_count;
    }

    // 计算旋转（角度差的均值）
    float sum_angle = 0.0f;
    for (const auto& match : matches) {
        const auto& src_pt = src_points[match.first];
        const auto& dst_pt = dst_points[match.second];
        float angle_diff = dst_pt.orientation - src_pt.orientation;
        sum_angle += angle_diff;
    }
    angle = sum_angle / matches.size() * 180.0f / static_cast<float>(M_PI);

    return true;
}

// ========== Node Registration ==========

OVF_REGISTER_NODE(ShapeMatchScaleNode, "ShapeMatchScale", ShapeMatchScaleNode::make_info())
OVF_REGISTER_NODE(ShapeMatchRotateNode, "ShapeMatchRotate", ShapeMatchRotateNode::make_info())
OVF_REGISTER_NODE(ShapeMatchScaleRotateNode, "ShapeMatchScaleRotate", ShapeMatchScaleRotateNode::make_info())
OVF_REGISTER_NODE(CreateShapeModelNode, "CreateShapeModel", CreateShapeModelNode::make_info())
OVF_REGISTER_NODE(FindShapeModelNode, "FindShapeModel", FindShapeModelNode::make_info())
OVF_REGISTER_NODE(WriteShapeModelNode, "WriteShapeModel", WriteShapeModelNode::make_info())

OVF_REGISTER_NODE(ComponentMatchNode, "ComponentMatch", ComponentMatchNode::make_info())
OVF_REGISTER_NODE(TrainComponentModelNode, "TrainComponentModel", TrainComponentModelNode::make_info())
OVF_REGISTER_NODE(FindComponentModelNode, "FindComponentModel", FindComponentModelNode::make_info())
OVF_REGISTER_NODE(InspectComponentNode, "InspectComponent", InspectComponentNode::make_info())

OVF_REGISTER_NODE(GrayMatchNode, "GrayMatch", GrayMatchNode::make_info())
OVF_REGISTER_NODE(BestMatchNode, "BestMatch", BestMatchNode::make_info())
OVF_REGISTER_NODE(AllMatchNode, "AllMatch", AllMatchNode::make_info())

OVF_REGISTER_NODE(DescriptorMatchNode, "DescriptorMatch", DescriptorMatchNode::make_info())
OVF_REGISTER_NODE(FeaturePointMatchNode, "FeaturePointMatch", FeaturePointMatchNode::make_info())

} // namespace algorithm
} // namespace ovf