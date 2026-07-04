/**
 * @file pcl_3d_reconstruction.cpp
 * @brief 高级3D重建模块实现 - PCL风格点云处理算法
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#include "ovf/algorithm/pcl_3d_reconstruction.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <random>
#include <fstream>
#include <sstream>
#include <queue>
#include <limits>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {
namespace pcl_3d_utils {

// ============================================================================
// 辅助数学函数
// ============================================================================

inline float sqr(float x) { return x * x; }

// 按维度索引访问 Point3Df 的坐标（dim: 0=x, 1=y, 2=z）
inline float get_coord(const Point3Df& p, int dim) {
    return (dim == 0) ? p.x : (dim == 1) ? p.y : p.z;
}

inline Point3Df cross_product(const Point3Df& a, const Point3Df& b) {
    return Point3Df(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

inline float dot_product(const Point3Df& a, const Point3Df& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float distance_squared(const Point3Df& a, const Point3Df& b) {
    return sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z);
}

inline float distance_3d(const Point3Df& a, const Point3Df& b) {
    return std::sqrt(distance_squared(a, b));
}

// 计算协方差矩阵
void compute_covariance_matrix(const Vector<Point3Df>& points, 
                               const Point3Df& centroid,
                               float cov[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov[i][j] = 0.0f;
        }
    }
    
    if (points.empty()) return;
    
    for (const auto& pt : points) {
        float dx = pt.x - centroid.x;
        float dy = pt.y - centroid.y;
        float dz = pt.z - centroid.z;
        
        cov[0][0] += dx * dx;
        cov[0][1] += dx * dy;
        cov[0][2] += dx * dz;
        cov[1][1] += dy * dy;
        cov[1][2] += dy * dz;
        cov[2][2] += dz * dz;
    }
    
    // 补全对称部分
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];
    
    float n = static_cast<float>(points.size());
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov[i][j] /= n;
        }
    }
}

// PCA求解最小特征值对应的特征向量（法向）
Point3Df solve_pca_normal(const float cov[3][3]) {
    // 使用雅可比方法求解特征值和特征向量
    float eigenvalues[3];
    float eigenvectors[3][3];
    
    // 初始化特征向量矩阵为单位矩阵
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigenvectors[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 复制协方差矩阵
    float matrix[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            matrix[i][j] = cov[i][j];
        }
    }
    
    // 雅可比迭代
    for (int iter = 0; iter < 50; ++iter) {
        // 找最大非对角元素
        int p = 0, q = 1;
        float max_val = std::abs(matrix[0][1]);
        
        if (std::abs(matrix[0][2]) > max_val) {
            p = 0; q = 2; max_val = std::abs(matrix[0][2]);
        }
        if (std::abs(matrix[1][2]) > max_val) {
            p = 1; q = 2; max_val = std::abs(matrix[1][2]);
        }
        
        if (max_val < 1e-10f) break;  // 收敛
        
        // 计算旋转角度
        float theta = 0.0f;
        if (std::abs(matrix[p][p] - matrix[q][q]) > 1e-10f) {
            theta = 0.5f * std::atan2(2.0f * matrix[p][q], matrix[p][p] - matrix[q][q]);
        } else {
            theta = M_PI / 4.0f;
        }
        
        float c = std::cos(theta);
        float s = std::sin(theta);
        
        // 应用旋转
        for (int i = 0; i < 3; ++i) {
            float temp1 = c * matrix[i][p] - s * matrix[i][q];
            float temp2 = s * matrix[i][p] + c * matrix[i][q];
            matrix[i][p] = temp1;
            matrix[i][q] = temp2;
        }
        
        for (int j = 0; j < 3; ++j) {
            float temp1 = c * matrix[p][j] - s * matrix[q][j];
            float temp2 = s * matrix[p][j] + c * matrix[q][j];
            matrix[p][j] = temp1;
            matrix[q][j] = temp2;
        }
        
        // 更新特征向量
        for (int i = 0; i < 3; ++i) {
            float temp1 = c * eigenvectors[i][p] - s * eigenvectors[i][q];
            float temp2 = s * eigenvectors[i][p] + c * eigenvectors[i][q];
            eigenvectors[i][p] = temp1;
            eigenvectors[i][q] = temp2;
        }
        
        matrix[p][q] = matrix[q][p] = 0.0f;
    }
    
    // 特征值
    eigenvalues[0] = matrix[0][0];
    eigenvalues[1] = matrix[1][1];
    eigenvalues[2] = matrix[2][2];
    
    // 找最小特征值
    int min_idx = 0;
    if (eigenvalues[1] < eigenvalues[min_idx]) min_idx = 1;
    if (eigenvalues[2] < eigenvalues[min_idx]) min_idx = 2;
    
    return Point3Df(eigenvectors[0][min_idx], eigenvectors[1][min_idx], eigenvectors[2][min_idx]).normalized();
}

// ============================================================================
// KdTree 实现
// ============================================================================

SimpleKdTree::SimpleKdTree() {}

SimpleKdTree::~SimpleKdTree() {
    clear();
}

void SimpleKdTree::build(const PointCloudData& cloud) {
    clear();
    
    if (cloud.empty()) return;
    
    // 复制点数据（避免修改原始数据）
    points_ = new Vector<Point3Df>(cloud.points);
    nodes_.resize(cloud.size());
    
    Vector<int> indices(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        indices[i] = static_cast<int>(i);
    }
    
    build_recursive(0, static_cast<int>(cloud.size()), 0);
}

void SimpleKdTree::clear() {
    if (points_) {
        delete points_;
        points_ = nullptr;
    }
    nodes_.clear();
}

bool SimpleKdTree::empty() const {
    return nodes_.empty();
}

size_t SimpleKdTree::size() const {
    return points_ ? points_->size() : 0;
}

int SimpleKdTree::build_recursive(int start, int end, int depth) {
    if (start >= end) return -1;
    
    int split_dim = depth % 3;
    int mid = (start + end) / 2;
    
    // 找分割值（中位数）
    std::nth_element(nodes_.begin() + start, nodes_.begin() + mid, nodes_.begin() + end,
        [this, split_dim](const Node& a, const Node& b) {
            return get_coord((*points_)[a.idx], split_dim) < get_coord((*points_)[b.idx], split_dim);
        });
    
    // 简化：直接使用索引构建
    int node_idx = mid;
    nodes_[node_idx].split_dim = split_dim;
    nodes_[node_idx].split_val = get_coord((*points_)[nodes_[node_idx].idx], split_dim);
    
    nodes_[node_idx].left = build_recursive(start, mid, depth + 1);
    nodes_[node_idx].right = build_recursive(mid + 1, end, depth + 1);
    
    return node_idx;
}

ErrorCode SimpleKdTree::nearest_k_search(const Point3Df& query, int k,
                                          Vector<int>& indices, Vector<float>& distances) const {
    if (empty() || k <= 0) return ErrorCode::InvalidParameter;
    
    indices.clear();
    distances.clear();
    
    // 优先队列实现K近邻
    Vector<std::pair<float, int>> heap;  // (距离, 索引)
    float max_dist_sq = std::numeric_limits<float>::max();
    
    // 递归搜索
    search_recursive(0, query, k, indices, distances, max_dist_sq);
    
    // 按距离排序
    std::sort(indices.begin(), indices.end(), [&distances](int a, int b) {
        return distances[a] < distances[b];
    });
    
    return ErrorCode::Success;
}

void SimpleKdTree::search_recursive(int node_idx, const Point3Df& query, int k,
                                     Vector<int>& indices, Vector<float>& distances,
                                     float max_dist_sq) const {
    if (node_idx < 0 || node_idx >= static_cast<int>(nodes_.size())) return;
    
    const Node& node = nodes_[node_idx];
    const Point3Df& pt = (*points_)[node.idx];
    
    float dist_sq = distance_squared(query, pt);
    
    // 更新K近邻列表
    if (static_cast<int>(indices.size()) < k) {
        indices.push_back(node.idx);
        distances.push_back(std::sqrt(dist_sq));
        max_dist_sq = dist_sq;
    } else if (dist_sq < max_dist_sq) {
        // 找最大距离的元素并替换
        int max_idx = 0;
        float max_d = distances[0];
        for (int i = 1; i < k; ++i) {
            if (distances[i] > max_d) {
                max_d = distances[i];
                max_idx = i;
            }
        }
        if (dist_sq < sqr(max_d)) {
            indices[max_idx] = node.idx;
            distances[max_idx] = std::sqrt(dist_sq);
            max_dist_sq = sqr(max_d);
        }
    }
    
    // 决定搜索顺序
    float diff = get_coord(query, node.split_dim) - node.split_val;
    int first = (diff < 0) ? node.left : node.right;
    int second = (diff < 0) ? node.right : node.left;
    
    // 搜索较近的子树
    search_recursive(first, query, k, indices, distances, max_dist_sq);
    
    // 检查是否需要搜索另一子树
    if (sqr(diff) < max_dist_sq) {
        search_recursive(second, query, k, indices, distances, max_dist_sq);
    }
}

ErrorCode SimpleKdTree::radius_search(const Point3Df& query, float radius,
                                       Vector<int>& indices) const {
    if (empty() || radius <= 0) return ErrorCode::InvalidParameter;
    
    indices.clear();
    float radius_sq = radius * radius;
    
    radius_search_recursive(0, query, radius_sq, indices);
    
    return ErrorCode::Success;
}

void SimpleKdTree::radius_search_recursive(int node_idx, const Point3Df& query,
                                            float radius_sq, Vector<int>& indices) const {
    if (node_idx < 0 || node_idx >= static_cast<int>(nodes_.size())) return;
    
    const Node& node = nodes_[node_idx];
    const Point3Df& pt = (*points_)[node.idx];
    
    float dist_sq = distance_squared(query, pt);
    
    if (dist_sq <= radius_sq) {
        indices.push_back(node.idx);
    }
    
    float diff = get_coord(query, node.split_dim) - node.split_val;
    
    // 搜索较近的子树
    if (diff < 0) {
        radius_search_recursive(node.left, query, radius_sq, indices);
        if (sqr(diff) <= radius_sq) {
            radius_search_recursive(node.right, query, radius_sq, indices);
        }
    } else {
        radius_search_recursive(node.right, query, radius_sq, indices);
        if (sqr(diff) <= radius_sq) {
            radius_search_recursive(node.left, query, radius_sq, indices);
        }
    }
}

// ============================================================================
// 点云获取
// ============================================================================

ErrorCode depth_to_pointcloud_advanced(const DepthImageData& depth_img, 
                                        PointCloudData& cloud,
                                        float depth_scale) {
    if (depth_img.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    if (depth_img.focal_length_x <= 0 || depth_img.focal_length_y <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    uint32_t width = depth_img.depth.width;
    uint32_t height = depth_img.depth.height;
    
    cloud.clear();
    cloud.width = width;
    cloud.height = height;
    cloud.is_organized = true;
    
    float fx = depth_img.focal_length_x;
    float fy = depth_img.focal_length_y;
    float cx = depth_img.center_x > 0 ? depth_img.center_x : width / 2.0f;
    float cy = depth_img.center_y > 0 ? depth_img.center_y : height / 2.0f;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            float z = depth_img.get_depth(static_cast<int>(x), static_cast<int>(y)) * depth_scale;
            
            if (z > 0) {
                float px = (static_cast<float>(x) - cx) * z / fx;
                float py = (static_cast<float>(y) - cy) * z / fy;
                float pz = z;
                
                cloud.add_point(px, py, pz);
            } else {
                cloud.add_point(0, 0, 0);
            }
        }
    }
    
    // 添加颜色
    if (depth_img.has_color() && 
        depth_img.color.width == width && 
        depth_img.color.height == height) {
        cloud.colors.resize(cloud.size());
        
        for (size_t i = 0; i < cloud.size(); ++i) {
            uint32_t x = i % width;
            uint32_t y = i / width;
            
            if (depth_img.color.format == ImageFormat::RGB8 && depth_img.color.channels == 3) {
                size_t idx = (y * width + x) * 3;
                cloud.colors[i] = ColorRGB(
                    depth_img.color.data[idx],
                    depth_img.color.data[idx + 1],
                    depth_img.color.data[idx + 2]
                );
            } else if (depth_img.color.format == ImageFormat::BGR8 && depth_img.color.channels == 3) {
                size_t idx = (y * width + x) * 3;
                cloud.colors[i] = ColorRGB(
                    depth_img.color.data[idx + 2],
                    depth_img.color.data[idx + 1],
                    depth_img.color.data[idx]
                );
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode stereo_to_pointcloud(const ImageData& left_img,
                               const ImageData& right_img,
                               PointCloudData& cloud,
                               float baseline,
                               float focal_length) {
    if (left_img.empty() || right_img.empty()) {
        return ErrorCode::InvalidImage;
    }
    
    if (baseline <= 0 || focal_length <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    // 简化实现：假设输入是视差图
    // 实际应用中需要先计算视差图
    
    uint32_t width = left_img.width;
    uint32_t height = left_img.height;
    
    cloud.clear();
    cloud.width = width;
    cloud.height = height;
    cloud.is_organized = true;
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            size_t idx = y * width + x;
            
            // 假设left_img是视差图
            float disparity = 0.0f;
            if (left_img.format == ImageFormat::Mono16) {
                const uint16_t* ptr = reinterpret_cast<const uint16_t*>(left_img.data.data());
                disparity = static_cast<float>(ptr[idx]);
            } else if (left_img.format == ImageFormat::Float32) {
                const float* ptr = reinterpret_cast<const float*>(left_img.data.data());
                disparity = ptr[idx];
            } else {
                disparity = static_cast<float>(left_img.data[idx]);
            }
            
            if (disparity > 0) {
                float z = focal_length * baseline / disparity;
                float px = (static_cast<float>(x) - width / 2.0f) * z / focal_length;
                float py = (static_cast<float>(y) - height / 2.0f) * z / focal_length;
                
                cloud.add_point(px, py, z);
                
                // 从右图获取颜色
                if (right_img.channels >= 3) {
                    size_t color_idx = idx * 3;
                    if (right_img.format == ImageFormat::RGB8) {
                        cloud.colors.emplace_back(
                            right_img.data[color_idx],
                            right_img.data[color_idx + 1],
                            right_img.data[color_idx + 2]
                        );
                    } else if (right_img.format == ImageFormat::BGR8) {
                        cloud.colors.emplace_back(
                            right_img.data[color_idx + 2],
                            right_img.data[color_idx + 1],
                            right_img.data[color_idx]
                        );
                    }
                }
            } else {
                cloud.add_point(0, 0, 0);
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode load_pcd_file(const String& filepath, PointCloudData& cloud) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return ErrorCode::FileNotFound;
    }
    
    // 读取PCD头
    String line;
    int width = 0, height = 0;
    size_t point_count = 0;
    bool has_color = false;
    Vector<String> fields;
    
    while (std::getline(file, line)) {
        if (line.find("FIELDS") != String::npos) {
            std::istringstream iss(line.substr(7));
            String field;
            while (iss >> field) {
                fields.push_back(field);
                if (field == "rgb" || field == "rgba") {
                    has_color = true;
                }
            }
        } else if (line.find("WIDTH") != String::npos) {
            width = std::stoi(line.substr(6));
        } else if (line.find("HEIGHT") != String::npos) {
            height = std::stoi(line.substr(7));
        } else if (line.find("POINTS") != String::npos) {
            point_count = std::stoul(line.substr(7));
        } else if (line.find("DATA") != String::npos) {
            break;  // 数据开始
        }
    }
    
    if (point_count == 0) {
        return ErrorCode::FileParseFailed;
    }
    
    cloud.clear();
    cloud.width = width;
    cloud.height = height;
    cloud.is_organized = (height > 1);
    
    // 解析数据格式
    String data_type = line.substr(5);
    if (data_type.find("binary") != String::npos) {
        // 二进制数据
        int point_size = 0;
        for (const auto& f : fields) {
            if (f == "x" || f == "y" || f == "z") point_size += 4;
            else if (f == "rgb" || f == "rgba") point_size += 4;
            else point_size += 4;  // 默认float
        }
        
        for (size_t i = 0; i < point_count; ++i) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), 4);
            file.read(reinterpret_cast<char*>(&y), 4);
            file.read(reinterpret_cast<char*>(&z), 4);
            
            cloud.add_point(x, y, z);
            
            if (has_color) {
                float rgb;
                file.read(reinterpret_cast<char*>(&rgb), 4);
                uint32_t rgb_int = *reinterpret_cast<uint32_t*>(&rgb);
                cloud.colors.emplace_back(
                    (rgb_int >> 0) & 0xFF,
                    (rgb_int >> 8) & 0xFF,
                    (rgb_int >> 16) & 0xFF
                );
            }
            
            // 跳过其他字段
            int remaining = point_size - 12 - (has_color ? 4 : 0);
            if (remaining > 0) {
                file.ignore(remaining);
            }
        }
    } else {
        // ASCII数据
        for (size_t i = 0; i < point_count; ++i) {
            std::getline(file, line);
            std::istringstream iss(line);
            
            float x, y, z;
            iss >> x >> y >> z;
            cloud.add_point(x, y, z);
            
            if (has_color) {
                float rgb;
                iss >> rgb;
                uint32_t rgb_int = *reinterpret_cast<uint32_t*>(&rgb);
                cloud.colors.emplace_back(
                    (rgb_int >> 0) & 0xFF,
                    (rgb_int >> 8) & 0xFF,
                    (rgb_int >> 16) & 0xFF
                );
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode load_ply_file(const String& filepath, PointCloudData& cloud) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return ErrorCode::FileNotFound;
    }
    
    String line;
    int vertex_count = 0;
    bool has_color = false;
    bool is_binary = false;
    
    // 解析PLY头
    while (std::getline(file, line)) {
        if (line.find("element vertex") != String::npos) {
            vertex_count = std::stoi(line.substr(15));
        } else if (line.find("property uchar red") != String::npos ||
                   line.find("property uchar green") != String::npos ||
                   line.find("property uchar blue") != String::npos) {
            has_color = true;
        } else if (line.find("format binary") != String::npos) {
            is_binary = true;
        } else if (line == "end_header") {
            break;
        }
    }
    
    if (vertex_count == 0) {
        return ErrorCode::FileParseFailed;
    }
    
    cloud.clear();
    cloud.is_organized = false;
    
    if (is_binary) {
        // 二进制格式
        for (int i = 0; i < vertex_count; ++i) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), 4);
            file.read(reinterpret_cast<char*>(&y), 4);
            file.read(reinterpret_cast<char*>(&z), 4);
            
            cloud.add_point(x, y, z);
            
            if (has_color) {
                uint8_t r, g, b;
                file.read(reinterpret_cast<char*>(&r), 1);
                file.read(reinterpret_cast<char*>(&g), 1);
                file.read(reinterpret_cast<char*>(&b), 1);
                cloud.colors.emplace_back(r, g, b);
            }
        }
    } else {
        // ASCII格式
        for (int i = 0; i < vertex_count; ++i) {
            std::getline(file, line);
            std::istringstream iss(line);
            
            float x, y, z;
            iss >> x >> y >> z;
            cloud.add_point(x, y, z);
            
            if (has_color) {
                int r, g, b;
                iss >> r >> g >> b;
                cloud.colors.emplace_back(
                    static_cast<uint8_t>(r),
                    static_cast<uint8_t>(g),
                    static_cast<uint8_t>(b)
                );
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode load_pointcloud_file(const String& filepath, PointCloudData& cloud) {
    if (filepath.find(".pcd") != String::npos) {
        return load_pcd_file(filepath, cloud);
    } else if (filepath.find(".ply") != String::npos) {
        return load_ply_file(filepath, cloud);
    } else {
        // 尝试两种格式
        ErrorCode err = load_pcd_file(filepath, cloud);
        if (err != ErrorCode::Success) {
            err = load_ply_file(filepath, cloud);
        }
        return err;
    }
}

ErrorCode save_pcd_file(const String& filepath, const PointCloudData& cloud) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return ErrorCode::FileOpenFailed;
    }
    
    // 写入PCD头
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z";
    if (cloud.has_colors()) file << " rgb";
    file << "\n";
    file << "SIZE 4 4 4";
    if (cloud.has_colors()) file << " 4";
    file << "\n";
    file << "TYPE F F F";
    if (cloud.has_colors()) file << " F";
    file << "\n";
    file << "COUNT 1 1 1";
    if (cloud.has_colors()) file << " 1";
    file << "\n";
    file << "WIDTH " << cloud.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud.size() << "\n";
    file << "DATA binary\n";
    
    // 写入点数据
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.points[i];
        file.write(reinterpret_cast<const char*>(&pt.x), 4);
        file.write(reinterpret_cast<const char*>(&pt.y), 4);
        file.write(reinterpret_cast<const char*>(&pt.z), 4);
        
        if (cloud.has_colors()) {
            uint32_t rgb = (cloud.colors[i].r) | 
                          (cloud.colors[i].g << 8) | 
                          (cloud.colors[i].b << 16);
            file.write(reinterpret_cast<const char*>(&rgb), 4);
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode save_ply_file(const String& filepath, const PointCloudData& cloud) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return ErrorCode::FileOpenFailed;
    }
    
    // 写入PLY头
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    if (cloud.has_colors()) {
        file << "property uchar red\n";
        file << "property uchar green\n";
        file << "property uchar blue\n";
    }
    file << "end_header\n";
    
    // 写入点数据
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.points[i];
        file.write(reinterpret_cast<const char*>(&pt.x), 4);
        file.write(reinterpret_cast<const char*>(&pt.y), 4);
        file.write(reinterpret_cast<const char*>(&pt.z), 4);
        
        if (cloud.has_colors()) {
            file.write(reinterpret_cast<const char*>(&cloud.colors[i].r), 1);
            file.write(reinterpret_cast<const char*>(&cloud.colors[i].g), 1);
            file.write(reinterpret_cast<const char*>(&cloud.colors[i].b), 1);
        }
    }
    
    return ErrorCode::Success;
}

// ============================================================================
// 点云预处理
// ============================================================================

ErrorCode voxel_grid_downsample(PointCloudData& cloud, float voxel_size) {
    if (cloud.empty() || voxel_size <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    // 体素键
    struct VoxelKey {
        int x, y, z;
        
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    
    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            return static_cast<size_t>(k.x) ^ 
                   (static_cast<size_t>(k.y) << 10) ^ 
                   (static_cast<size_t>(k.z) << 20);
        }
    };
    
    struct VoxelData {
        Point3Df centroid;
        ColorRGB color;
        int count;
        bool has_color;
    };
    
    std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels;
    
    // 分配点到体素
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& pt = cloud.points[i];
        
        VoxelKey key;
        key.x = static_cast<int>(std::floor(pt.x / voxel_size));
        key.y = static_cast<int>(std::floor(pt.y / voxel_size));
        key.z = static_cast<int>(std::floor(pt.z / voxel_size));
        
        auto& voxel = voxels[key];
        voxel.centroid.x += pt.x;
        voxel.centroid.y += pt.y;
        voxel.centroid.z += pt.z;
        voxel.count++;
        
        if (cloud.has_colors()) {
            voxel.color.r += cloud.colors[i].r;
            voxel.color.g += cloud.colors[i].g;
            voxel.color.b += cloud.colors[i].b;
            voxel.has_color = true;
        }
    }
    
    // 重建点云
    cloud.clear();
    for (const auto& pair : voxels) {
        const auto& voxel = pair.second;
        float inv_count = 1.0f / voxel.count;
        
        cloud.add_point(voxel.centroid.x * inv_count,
                        voxel.centroid.y * inv_count,
                        voxel.centroid.z * inv_count);
        
        if (voxel.has_color) {
            cloud.colors.emplace_back(
                static_cast<uint8_t>(voxel.color.r * inv_count),
                static_cast<uint8_t>(voxel.color.g * inv_count),
                static_cast<uint8_t>(voxel.color.b * inv_count)
            );
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode statistical_outlier_removal(PointCloudData& cloud, 
                                       int mean_k, 
                                       float std_threshold) {
    if (cloud.empty() || mean_k <= 0 || std_threshold <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t n = cloud.size();
    Vector<float> distances(n);
    
    // 计算每个点到K邻域的平均距离
    for (size_t i = 0; i < n; ++i) {
        Vector<float> all_distances(n - 1);
        size_t idx = 0;
        
        for (size_t j = 0; j < n; ++j) {
            if (i != j) {
                all_distances[idx++] = distance_3d(cloud.points[i], cloud.points[j]);
            }
        }
        
        // 取前K个最小距离
        std::partial_sort(all_distances.begin(), 
                          all_distances.begin() + std::min(mean_k, static_cast<int>(n - 1)),
                          all_distances.end());
        
        float sum = 0.0f;
        int k_count = std::min(mean_k, static_cast<int>(n - 1));
        for (int k = 0; k < k_count; ++k) {
            sum += all_distances[k];
        }
        distances[i] = sum / k_count;
    }
    
    // 计算均值和标准差
    float sum = 0.0f;
    for (float d : distances) sum += d;
    float mean = sum / n;
    
    float sum_sq = 0.0f;
    for (float d : distances) sum_sq += sqr(d - mean);
    float stddev = std::sqrt(sum_sq / n);
    
    // 过滤离群点
    float threshold = mean + std_threshold * stddev;
    
    PointCloudData filtered;
    for (size_t i = 0; i < n; ++i) {
        if (distances[i] <= threshold) {
            filtered.add_point(cloud.points[i]);
            if (cloud.has_colors()) {
                filtered.colors.push_back(cloud.colors[i]);
            }
        }
    }
    
    cloud = std::move(filtered);
    return ErrorCode::Success;
}

ErrorCode moving_least_squares(PointCloudData& cloud, 
                                PointCloudData& smoothed,
                                float search_radius,
                                bool compute_normals) {
    if (cloud.empty() || search_radius <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    smoothed.clear();
    float radius_sq = search_radius * search_radius;
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.points[i];
        
        // 找半径内的邻近点
        Vector<Point3Df> neighbors;
        Vector<size_t> neighbor_indices;
        
        for (size_t j = 0; j < cloud.size(); ++j) {
            if (distance_squared(pt, cloud.points[j]) <= radius_sq) {
                neighbors.push_back(cloud.points[j]);
                neighbor_indices.push_back(j);
            }
        }
        
        if (neighbors.size() < 3) {
            smoothed.add_point(pt);
            if (cloud.has_colors()) {
                smoothed.colors.push_back(cloud.colors[i]);
            }
            continue;
        }
        
        // 计算局部平面（使用PCA）
        Point3Df centroid(0, 0, 0);
        for (const auto& npt : neighbors) {
            centroid.x += npt.x;
            centroid.y += npt.y;
            centroid.z += npt.z;
        }
        float inv_n = 1.0f / neighbors.size();
        centroid.x *= inv_n;
        centroid.y *= inv_n;
        centroid.z *= inv_n;
        
        // 计算协方差矩阵
        float cov[3][3];
        compute_covariance_matrix(neighbors, centroid, cov);
        
        // PCA求解法向
        Point3Df normal = solve_pca_normal(cov);
        
        // 投影点到局部平面
        Point3Df projected = pt - normal * dot_product(pt - centroid, normal);
        
        smoothed.add_point(projected);
        if (cloud.has_colors()) {
            smoothed.colors.push_back(cloud.colors[i]);
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode estimate_normals(const PointCloudData& cloud,
                           Vector<PointCloudNormal>& normals,
                           float search_radius,
                           int k_neighbors) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    normals.clear();
    normals.resize(cloud.size());
    
    float radius_sq = sqr(search_radius);
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.points[i];
        
        // 找邻近点
        Vector<Point3Df> neighbors;
        
        if (k_neighbors > 0) {
            // K邻域搜索
            Vector<float> distances(cloud.size());
            Vector<size_t> indices(cloud.size());
            
            for (size_t j = 0; j < cloud.size(); ++j) {
                indices[j] = j;
                distances[j] = distance_squared(pt, cloud.points[j]);
            }
            
            // 排序取前K个
            std::partial_sort(indices.begin(), indices.begin() + std::min(k_neighbors, static_cast<int>(cloud.size())),
                              indices.end(), [&distances](size_t a, size_t b) {
                                  return distances[a] < distances[b];
                              });
            
            int k_count = std::min(k_neighbors, static_cast<int>(cloud.size()));
            for (int k = 0; k < k_count; ++k) {
                neighbors.push_back(cloud.points[indices[k]]);
            }
        } else {
            // 半径搜索
            for (size_t j = 0; j < cloud.size(); ++j) {
                if (distance_squared(pt, cloud.points[j]) <= radius_sq) {
                    neighbors.push_back(cloud.points[j]);
                }
            }
        }
        
        if (neighbors.size() < 3) {
            normals[i] = PointCloudNormal(0, 0, 1, 0);
            continue;
        }
        
        // 计算重心
        Point3Df centroid(0, 0, 0);
        for (const auto& npt : neighbors) {
            centroid = centroid + npt;
        }
        float inv_n = 1.0f / neighbors.size();
        centroid = centroid * inv_n;
        
        // 协方差矩阵
        float cov[3][3];
        compute_covariance_matrix(neighbors, centroid, cov);
        
        // PCA求解法向
        Point3Df normal = solve_pca_normal(cov);
        
        // 计算曲率（最小特征值 / 特征值之和）
        float eigen_sum = cov[0][0] + cov[1][1] + cov[2][2];
        float curvature = (eigen_sum > 0) ? std::min({cov[0][0], cov[1][1], cov[2][2]}) / eigen_sum : 0.0f;
        
        normals[i] = PointCloudNormal(normal.x, normal.y, normal.z, curvature);
    }
    
    return ErrorCode::Success;
}

ErrorCode estimate_normals_knn(const PointCloudData& cloud,
                                Vector<PointCloudNormal>& normals,
                                int k_neighbors) {
    return estimate_normals(cloud, normals, 0.0f, k_neighbors);
}

ErrorCode find_k_nearest_neighbors(const PointCloudData& cloud,
                                    int query_idx,
                                    int k,
                                    Vector<int>& neighbor_indices,
                                    Vector<float>& distances) {
    if (cloud.empty() || query_idx < 0 || query_idx >= static_cast<int>(cloud.size()) || k <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    const auto& query_pt = cloud.points[query_idx];
    
    Vector<std::pair<float, int>> all_dists;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (static_cast<int>(i) != query_idx) {
            float d = distance_3d(query_pt, cloud.points[i]);
            all_dists.emplace_back(d, static_cast<int>(i));
        }
    }
    
    std::partial_sort(all_dists.begin(), 
                      all_dists.begin() + std::min(k, static_cast<int>(all_dists.size())),
                      all_dists.end());
    
    neighbor_indices.clear();
    distances.clear();
    
    int k_count = std::min(k, static_cast<int>(all_dists.size()));
    for (int i = 0; i < k_count; ++i) {
        neighbor_indices.push_back(all_dists[i].second);
        distances.push_back(all_dists[i].first);
    }
    
    return ErrorCode::Success;
}

ErrorCode compute_knn_graph(const PointCloudData& cloud,
                             int k,
                             Vector<Vector<int>>& neighbor_indices) {
    if (cloud.empty() || k <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    neighbor_indices.resize(cloud.size());
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        Vector<float> dists;
        find_k_nearest_neighbors(cloud, static_cast<int>(i), k, neighbor_indices[i], dists);
    }
    
    return ErrorCode::Success;
}

// ============================================================================
// 点云配准
// ============================================================================

// SVD求解变换矩阵
void solve_transformation_svd(const Vector<Point3Df>& source_pts,
                               const Vector<Point3Df>& target_pts,
                               Transform3D& transform) {
    if (source_pts.size() != target_pts.size() || source_pts.empty()) {
        return;
    }
    
    // 计算重心
    Point3Df src_centroid(0, 0, 0);
    Point3Df tgt_centroid(0, 0, 0);
    
    for (size_t i = 0; i < source_pts.size(); ++i) {
        src_centroid = src_centroid + source_pts[i];
        tgt_centroid = tgt_centroid + target_pts[i];
    }
    
    float inv_n = 1.0f / source_pts.size();
    src_centroid = src_centroid * inv_n;
    tgt_centroid = tgt_centroid * inv_n;
    
    // 计算H矩阵
    float H[3][3] = {{0}};
    for (size_t i = 0; i < source_pts.size(); ++i) {
        Point3Df src_centered = source_pts[i] - src_centroid;
        Point3Df tgt_centered = target_pts[i] - tgt_centroid;
        
        H[0][0] += src_centered.x * tgt_centered.x;
        H[0][1] += src_centered.x * tgt_centered.y;
        H[0][2] += src_centered.x * tgt_centered.z;
        H[1][0] += src_centered.y * tgt_centered.x;
        H[1][1] += src_centered.y * tgt_centered.y;
        H[1][2] += src_centered.y * tgt_centered.z;
        H[2][0] += src_centered.z * tgt_centered.x;
        H[2][1] += src_centered.z * tgt_centered.y;
        H[2][2] += src_centered.z * tgt_centered.z;
    }
    
    // SVD分解（简化版）
    // 使用雅可比方法求解
    float U[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float S[3] = {0};
    
    // 计算H^T H
    float HtH[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            HtH[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                HtH[i][j] += H[k][i] * H[k][j];
            }
        }
    }
    
    // 对HtH进行特征分解得到V
    // (简化实现，使用雅可比迭代)
    for (int iter = 0; iter < 50; ++iter) {
        int p = 0, q = 1;
        float max_val = std::abs(HtH[0][1]);
        if (std::abs(HtH[0][2]) > max_val) { p = 0; q = 2; max_val = std::abs(HtH[0][2]); }
        if (std::abs(HtH[1][2]) > max_val) { p = 1; q = 2; max_val = std::abs(HtH[1][2]); }
        
        if (max_val < 1e-10f) break;
        
        float theta = 0.5f * std::atan2(2.0f * HtH[p][q], HtH[p][p] - HtH[q][q]);
        float c = std::cos(theta);
        float s = std::sin(theta);
        
        for (int i = 0; i < 3; ++i) {
            float temp1 = c * HtH[i][p] - s * HtH[i][q];
            float temp2 = s * HtH[i][p] + c * HtH[i][q];
            HtH[i][p] = temp1;
            HtH[i][q] = temp2;
        }
        
        for (int j = 0; j < 3; ++j) {
            float temp1 = c * HtH[p][j] - s * HtH[q][j];
            float temp2 = s * HtH[p][j] + c * HtH[q][j];
            HtH[p][j] = temp1;
            HtH[q][j] = temp2;
        }
        
        for (int i = 0; i < 3; ++i) {
            float temp1 = c * V[i][p] - s * V[i][q];
            float temp2 = s * V[i][p] + c * V[i][q];
            V[i][p] = temp1;
            V[i][q] = temp2;
        }
    }
    
    S[0] = HtH[0][0];
    S[1] = HtH[1][1];
    S[2] = HtH[2][2];
    
    // 计算U = H V S^-1
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            U[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                if (S[k] > 1e-10f) {
                    U[i][j] += H[i][k] * V[k][j] / std::sqrt(S[k]);
                }
            }
        }
    }
    
    // R = U V^T
    float R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                R[i][j] += U[i][k] * V[j][k];
            }
        }
    }
    
    // t = tgt_centroid - R * src_centroid
    Point3Df t(
        tgt_centroid.x - (R[0][0] * src_centroid.x + R[0][1] * src_centroid.y + R[0][2] * src_centroid.z),
        tgt_centroid.y - (R[1][0] * src_centroid.x + R[1][1] * src_centroid.y + R[1][2] * src_centroid.z),
        tgt_centroid.z - (R[2][0] * src_centroid.x + R[2][1] * src_centroid.y + R[2][2] * src_centroid.z)
    );
    
    // 构建变换矩阵
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transform.m[i][j] = R[i][j];
        }
    }
    transform.m[0][3] = t.x;
    transform.m[1][3] = t.y;
    transform.m[2][3] = t.z;
    transform.m[3][0] = transform.m[3][1] = transform.m[3][2] = 0.0f;
    transform.m[3][3] = 1.0f;
}

ErrorCode icp_registration(const PointCloudData& source,
                           const PointCloudData& target,
                           RegistrationResult& result,
                           int max_iterations,
                           float tolerance,
                           float max_correspondence_distance) {
    if (source.empty() || target.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.converged = false;
    result.transform = Transform3D();  // 初始化为单位矩阵
    
    PointCloudData transformed = source;  // 变换后的源点云
    float prev_error = std::numeric_limits<float>::max();
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 找对应点
        Vector<Point3Df> src_matched;
        Vector<Point3Df> tgt_matched;
        float total_error = 0.0f;
        int match_count = 0;
        
        for (size_t i = 0; i < transformed.size(); ++i) {
            const auto& src_pt = transformed.points[i];
            
            // 找最近的靶点
            float min_dist = std::numeric_limits<float>::max();
            int min_idx = -1;
            
            for (size_t j = 0; j < target.size(); ++j) {
                float d = distance_3d(src_pt, target.points[j]);
                if (d < min_dist) {
                    min_dist = d;
                    min_idx = static_cast<int>(j);
                }
            }
            
            if (min_idx >= 0 && min_dist <= max_correspondence_distance) {
                src_matched.push_back(src_pt);
                tgt_matched.push_back(target.points[min_idx]);
                total_error += min_dist;
                match_count++;
            }
        }
        
        if (match_count < 3) {
            break;
        }
        
        float avg_error = total_error / match_count;
        
        // 检查收敛
        if (std::abs(prev_error - avg_error) < tolerance) {
            result.converged = true;
            break;
        }
        
        prev_error = avg_error;
        
        // 计算变换
        Transform3D delta_transform;
        solve_transformation_svd(src_matched, tgt_matched, delta_transform);
        
        // 更新累积变换
        Transform3D combined;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                combined.m[i][j] = 0;
                for (int k = 0; k < 4; ++k) {
                    combined.m[i][j] += delta_transform.m[i][k] * result.transform.m[k][j];
                }
            }
        }
        result.transform = combined;
        
        // 更新变换后的点云
        for (auto& pt : transformed.points) {
            pt = result.transform.transform(pt);
        }
    }
    
    // 计算最终结果
    float total_error = 0.0f;
    int inlier_count = 0;
    
    for (size_t i = 0; i < transformed.size(); ++i) {
        float min_dist = std::numeric_limits<float>::max();
        for (size_t j = 0; j < target.size(); ++j) {
            float d = distance_3d(transformed.points[i], target.points[j]);
            min_dist = std::min(min_dist, d);
        }
        if (min_dist <= max_correspondence_distance) {
            total_error += sqr(min_dist);
            inlier_count++;
        }
    }
    
    result.rmse = std::sqrt(total_error / std::max(1, inlier_count));
    result.fitness_score = static_cast<float>(inlier_count) / source.size();
    result.inlier_count = inlier_count;
    
    return ErrorCode::Success;
}

ErrorCode icp_point_to_plane(const PointCloudData& source,
                              const PointCloudData& target,
                              const Vector<PointCloudNormal>& target_normals,
                              RegistrationResult& result,
                              int max_iterations,
                              float tolerance,
                              float max_correspondence_distance) {
    if (source.empty() || target.empty() || target_normals.size() != target.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.converged = false;
    result.transform = Transform3D();
    
    PointCloudData transformed = source;
    float prev_error = std::numeric_limits<float>::max();
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        Vector<Point3Df> src_matched;
        Vector<Point3Df> tgt_matched;
        float total_error = 0.0f;
        int match_count = 0;
        
        for (size_t i = 0; i < transformed.size(); ++i) {
            const auto& src_pt = transformed.points[i];
            
            float min_dist = std::numeric_limits<float>::max();
            int min_idx = -1;
            
            for (size_t j = 0; j < target.size(); ++j) {
                float d = distance_3d(src_pt, target.points[j]);
                if (d < min_dist) {
                    min_dist = d;
                    min_idx = static_cast<int>(j);
                }
            }
            
            if (min_idx >= 0 && min_dist <= max_correspondence_distance) {
                // 点到面距离
                const auto& tgt_pt = target.points[min_idx];
                const auto& tgt_normal = target_normals[min_idx].normal;
                
                float pt2plane_dist = std::abs(dot_product(src_pt - tgt_pt, tgt_normal));
                
                // 投影到平面
                Point3Df projected = tgt_pt + tgt_normal * dot_product(src_pt - tgt_pt, tgt_normal);
                
                src_matched.push_back(src_pt);
                tgt_matched.push_back(projected);
                total_error += pt2plane_dist;
                match_count++;
            }
        }
        
        if (match_count < 3) break;
        
        float avg_error = total_error / match_count;
        if (std::abs(prev_error - avg_error) < tolerance) {
            result.converged = true;
            break;
        }
        
        prev_error = avg_error;
        
        Transform3D delta_transform;
        solve_transformation_svd(src_matched, tgt_matched, delta_transform);
        
        Transform3D combined;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                combined.m[i][j] = 0;
                for (int k = 0; k < 4; ++k) {
                    combined.m[i][j] += delta_transform.m[i][k] * result.transform.m[k][j];
                }
            }
        }
        result.transform = combined;
        
        for (auto& pt : transformed.points) {
            pt = result.transform.transform(pt);
        }
    }
    
    // 计算最终结果
    float total_error = 0.0f;
    int inlier_count = 0;
    
    for (size_t i = 0; i < transformed.size(); ++i) {
        float min_dist = std::numeric_limits<float>::max();
        int min_idx = -1;
        
        for (size_t j = 0; j < target.size(); ++j) {
            float d = distance_3d(transformed.points[i], target.points[j]);
            if (d < min_dist) {
                min_dist = d;
                min_idx = static_cast<int>(j);
            }
        }
        
        if (min_idx >= 0 && min_dist <= max_correspondence_distance) {
            float pt2plane_dist = std::abs(dot_product(
                transformed.points[i] - target.points[min_idx],
                target_normals[min_idx].normal));
            total_error += sqr(pt2plane_dist);
            inlier_count++;
        }
    }
    
    result.rmse = std::sqrt(total_error / std::max(1, inlier_count));
    result.fitness_score = static_cast<float>(inlier_count) / source.size();
    result.inlier_count = inlier_count;
    
    return ErrorCode::Success;
}

ErrorCode ndt_registration(const PointCloudData& source,
                           const PointCloudData& target,
                           RegistrationResult& result,
                           float voxel_size,
                           int max_iterations,
                           float tolerance) {
    if (source.empty() || target.empty() || voxel_size <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    result.converged = false;
    result.transform = Transform3D();
    
    // 构建靶点云的NDT网格
    struct NDTCell {
        Point3Df mean;
        float cov[3][3];
        int count;
        bool valid;
        
        NDTCell() : count(0), valid(false) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    cov[i][j] = 0;
                }
            }
        }
    };
    
    std::unordered_map<int64_t, NDTCell> ndt_grid;
    
    auto get_cell_key = [voxel_size](const Point3Df& pt) -> int64_t {
        int64_t ix = static_cast<int64_t>(std::floor(pt.x / voxel_size));
        int64_t iy = static_cast<int64_t>(std::floor(pt.y / voxel_size));
        int64_t iz = static_cast<int64_t>(std::floor(pt.z / voxel_size));
        return (ix & 0xFFFFF) | ((iy & 0xFFFFF) << 20) | ((iz & 0xFFFFF) << 40);
    };
    
    // 填充NDT网格
    for (const auto& pt : target.points) {
        int64_t key = get_cell_key(pt);
        auto& cell = ndt_grid[key];
        
        cell.mean.x += pt.x;
        cell.mean.y += pt.y;
        cell.mean.z += pt.z;
        cell.count++;
    }
    
    // 计算每个格子的均值和协方差
    for (auto& pair : ndt_grid) {
        auto& cell = pair.second;
        if (cell.count >= 3) {
            float inv_n = 1.0f / cell.count;
            cell.mean.x *= inv_n;
            cell.mean.y *= inv_n;
            cell.mean.z *= inv_n;
            
            // 计算协方差（需要重新遍历）
            Vector<Point3Df> cell_points;
            // 简化：使用已存储的点
            // 实际需要存储点数据
            
            cell.valid = true;
        }
    }
    
    PointCloudData transformed = source;
    float prev_score = -std::numeric_limits<float>::max();
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算NDT得分
        float score = 0.0f;
        
        for (const auto& pt : transformed.points) {
            int64_t key = get_cell_key(pt);
            auto it = ndt_grid.find(key);
            
            if (it != ndt_grid.end() && it->second.valid) {
                const auto& cell = it->second;
                Point3Df diff = pt - cell.mean;
                
                // 简化的得分计算
                float dist_sq = sqr(diff.x) + sqr(diff.y) + sqr(diff.z);
                score -= dist_sq;
            }
        }
        
        if (std::abs(prev_score - score) < tolerance) {
            result.converged = true;
            break;
        }
        
        prev_score = score;
        
        // 简化梯度下降更新
        // 实际NDT需要计算雅可比矩阵和Hessian矩阵
        // 这里使用简化版本
    }
    
    result.fitness_score = static_cast<float>(result.inlier_count) / source.size();
    
    return ErrorCode::Success;
}

ErrorCode compute_fpfh_features(const PointCloudData& cloud,
                                 const Vector<PointCloudNormal>& normals,
                                 Vector<FPFHFeature>& features,
                                 float search_radius) {
    if (cloud.empty() || normals.size() != cloud.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    features.resize(cloud.size());
    float radius_sq = sqr(search_radius);
    
    // 计算SPFH（简化点特征直方图）
    Vector<Vector<float>> spfh(cloud.size(), Vector<float>(12, 0.0f));
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt_i = cloud.points[i];
        const auto& normal_i = normals[i].normal;
        
        Vector<size_t> neighbors;
        for (size_t j = 0; j < cloud.size(); ++j) {
            if (distance_squared(pt_i, cloud.points[j]) <= radius_sq) {
                neighbors.push_back(j);
            }
        }
        
        for (size_t neighbor_idx : neighbors) {
            if (neighbor_idx == i) continue;
            
            const auto& pt_j = cloud.points[neighbor_idx];
            const auto& normal_j = normals[neighbor_idx].normal;
            
            Point3Df diff = pt_j - pt_i;
            float dist = diff.length();
            if (dist < 1e-10f) continue;

            // 归一化的方向向量（Point3Df 不支持 / 运算符，使用 * 代替）
            Point3Df diff_normalized = diff * (1.0f / dist);

            Point3Df u = normal_i;
            Point3Df v = cross_product(u, diff).normalized();
            Point3Df w = cross_product(u, v);
            
            // 计算角度特征
            float alpha = std::acos(std::max(-1.0f, std::min(1.0f, dot_product(v, normal_j))));
            float phi = std::acos(std::max(-1.0f, std::min(1.0f, dot_product(u, diff_normalized))));
            float theta = std::acos(std::max(-1.0f, std::min(1.0f, dot_product(v, cross_product(normal_j, diff).normalized()))));
            
            // 简化的直方图累加
            spfh[i][0] += alpha;
            spfh[i][1] += phi;
            spfh[i][2] += theta;
        }
        
        // 归一化
        float sum = spfh[i][0] + spfh[i][1] + spfh[i][2];
        if (sum > 0) {
            spfh[i][0] /= sum;
            spfh[i][1] /= sum;
            spfh[i][2] /= sum;
        }
    }
    
    // 计算FPFH（加权融合邻居的SPFH）
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt_i = cloud.points[i];
        
        Vector<size_t> neighbors;
        Vector<float> neighbor_dists;
        
        for (size_t j = 0; j < cloud.size(); ++j) {
            float d_sq = distance_squared(pt_i, cloud.points[j]);
            if (d_sq <= radius_sq && j != i) {
                neighbors.push_back(j);
                neighbor_dists.push_back(std::sqrt(d_sq));
            }
        }
        
        // FPFH = SPFH + weighted neighbor SPFH
        features[i] = FPFHFeature();
        
        // 复制SPFH到FPFH
        for (int k = 0; k < 12; ++k) {
            features[i][k] = spfh[i][k];
        }
        
        // 加权邻居贡献
        if (neighbors.size() > 0) {
            float dist_sum = 0.0f;
            for (float d : neighbor_dists) dist_sum += d;
            
            if (dist_sum > 0) {
                for (size_t n = 0; n < neighbors.size(); ++n) {
                    float weight = 1.0f - neighbor_dists[n] / dist_sum;
                    for (int k = 0; k < 12; ++k) {
                        features[i][k] += weight * spfh[neighbors[n]][k];
                    }
                }
            }
        }
        
        // 扩展到33维（简化）
        for (int k = 12; k < 33; ++k) {
            features[i][k] = features[i][k % 12];
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode compute_sift3d_features(const PointCloudData& cloud,
                                   Vector<SIFT3DFeature>& features,
                                   float min_scale,
                                   float min_contrast) {
    // 简化实现
    features.clear();
    
    if (cloud.empty()) return ErrorCode::InvalidParameter;
    
    // 使用曲率作为关键点检测指标
    Vector<PointCloudNormal> normals;
    estimate_normals(cloud, normals, min_scale * 3.0f);
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (normals[i].curvature > min_contrast) {
            SIFT3DFeature feat;
            feat.position = cloud.points[i];
            feat.scale = min_scale;
            feat.normal = normals[i].normal;
            feat.descriptor.resize(128, 0.0f);  // 简化
            features.push_back(feat);
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode sac_ia_registration(const PointCloudData& source,
                               const PointCloudData& target,
                               const Vector<FPFHFeature>& source_features,
                               const Vector<FPFHFeature>& target_features,
                               RegistrationResult& result,
                               int max_iterations,
                               float min_sample_distance) {
    if (source.empty() || target.empty() || 
        source_features.size() != source.size() || 
        target_features.size() != target.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.converged = false;
    result.transform = Transform3D();
    
    // 计算特征距离
    auto feature_distance = [](const FPFHFeature& a, const FPFHFeature& b) -> float {
        float dist = 0.0f;
        for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
            dist += sqr(a[i] - b[i]);
        }
        return std::sqrt(dist);
    };
    
    float best_error = std::numeric_limits<float>::max();
    Transform3D best_transform;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> src_dis(0, source.size() - 1);
    std::uniform_int_distribution<size_t> tgt_dis(0, target.size() - 1);
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择3个源点
        Vector<size_t> src_indices(3);
        Vector<size_t> tgt_indices(3);
        
        for (int k = 0; k < 3; ++k) {
            src_indices[k] = src_dis(gen);
            
            // 找特征最相似的靶点
            float min_feat_dist = std::numeric_limits<float>::max();
            size_t best_tgt_idx = 0;
            
            for (size_t j = 0; j < target.size(); ++j) {
                float fd = feature_distance(source_features[src_indices[k]], target_features[j]);
                if (fd < min_feat_dist) {
                    min_feat_dist = fd;
                    best_tgt_idx = j;
                }
            }
            tgt_indices[k] = best_tgt_idx;
        }
        
        // 检查采样距离
        bool valid = true;
        for (int k = 0; k < 3; ++k) {
            for (int l = k + 1; l < 3; ++l) {
                float d = distance_3d(source.points[src_indices[k]], source.points[src_indices[l]]);
                if (d < min_sample_distance) {
                    valid = false;
                    break;
                }
            }
        }
        
        if (!valid) continue;
        
        // 计算变换
        Vector<Point3Df> src_pts(3);
        Vector<Point3Df> tgt_pts(3);
        
        for (int k = 0; k < 3; ++k) {
            src_pts[k] = source.points[src_indices[k]];
            tgt_pts[k] = target.points[tgt_indices[k]];
        }
        
        Transform3D candidate;
        solve_transformation_svd(src_pts, tgt_pts, candidate);
        
        // 评估变换
        float error = 0.0f;
        for (size_t i = 0; i < source.size(); ++i) {
            Point3Df transformed = candidate.transform(source.points[i]);
            float min_dist = std::numeric_limits<float>::max();
            
            for (const auto& tgt_pt : target.points) {
                min_dist = std::min(min_dist, distance_3d(transformed, tgt_pt));
            }
            error += min_dist;
        }
        
        if (error < best_error) {
            best_error = error;
            best_transform = candidate;
        }
    }
    
    result.transform = best_transform;
    result.converged = (best_error < std::numeric_limits<float>::max());
    result.rmse = best_error / source.size();
    
    return ErrorCode::Success;
}

ErrorCode feature_matching_registration(const PointCloudData& source,
                                         const PointCloudData& target,
                                         RegistrationResult& result) {
    // 计算法向和特征
    Vector<PointCloudNormal> source_normals, target_normals;
    estimate_normals(source, source_normals, 0.05f);
    estimate_normals(target, target_normals, 0.05f);
    
    Vector<FPFHFeature> source_features, target_features;
    compute_fpfh_features(source, source_normals, source_features, 0.05f);
    compute_fpfh_features(target, target_normals, target_features, 0.05f);
    
    return sac_ia_registration(source, target, source_features, target_features, result);
}

ErrorCode global_registration(Vector<PointCloudData>& clouds,
                               Vector<Transform3D>& transforms,
                               Vector<RegistrationResult>& results) {
    if (clouds.size() < 2) {
        return ErrorCode::InvalidParameter;
    }
    
    transforms.resize(clouds.size());
    results.resize(clouds.size());
    
    // 第一个点云作为参考
    transforms[0] = Transform3D();
    results[0].converged = true;
    
    // 逐对配准
    for (size_t i = 1; i < clouds.size(); ++i) {
        RegistrationResult pair_result;
        icp_registration(clouds[i], clouds[0], pair_result);
        
        transforms[i] = pair_result.transform;
        results[i] = pair_result;
        
        // 变换点云
        for (auto& pt : clouds[i].points) {
            pt = transforms[i].transform(pt);
        }
    }
    
    return ErrorCode::Success;
}

// ============================================================================
// 点云分割
// ============================================================================

ErrorCode ransac_plane_segmentation(PointCloudData& cloud,
                                     PlaneSegmentResult& result,
                                     float distance_threshold,
                                     int max_iterations,
                                     int min_inliers) {
    if (cloud.size() < 3) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t n = cloud.size();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, n - 1);
    
    int best_inlier_count = 0;
    Point3Df best_normal;
    float best_d = 0.0f;
    Vector<int> best_inliers;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选3点
        size_t i1 = dis(gen), i2 = dis(gen), i3 = dis(gen);
        while (i2 == i1) i2 = dis(gen);
        while (i3 == i1 || i3 == i2) i3 = dis(gen);
        
        const auto& p1 = cloud.points[i1];
        const auto& p2 = cloud.points[i2];
        const auto& p3 = cloud.points[i3];
        
        // 计算平面
        Point3Df v1 = p2 - p1;
        Point3Df v2 = p3 - p1;
        Point3Df normal = cross_product(v1, v2).normalized();
        
        if (normal.length() < 0.1f) continue;
        
        float d = -dot_product(normal, p1);
        
        // 找内点
        Vector<int> inliers;
        for (size_t i = 0; i < n; ++i) {
            float dist = std::abs(dot_product(normal, cloud.points[i]) + d);
            if (dist <= distance_threshold) {
                inliers.push_back(static_cast<int>(i));
            }
        }
        
        if (inliers.size() > best_inlier_count) {
            best_inlier_count = static_cast<int>(inliers.size());
            best_normal = normal;
            best_d = d;
            best_inliers = inliers;
        }
    }
    
    if (best_inlier_count < min_inliers) {
        result.plane.valid = false;
        return ErrorCode::Success;
    }
    
    // 构建结果
    result.plane.normal = best_normal;
    result.plane.d = best_d;
    result.plane.valid = true;
    result.plane_indices = best_inliers;
    
    // 计算平面中心
    Point3Df center(0, 0, 0);
    for (int idx : best_inliers) {
        center = center + cloud.points[idx];
    }
    result.plane.center = center * (1.0f / best_inliers.size());
    
    // 分离点云
    Vector<bool> is_plane(n, false);
    for (int idx : best_inliers) {
        is_plane[idx] = true;
        result.plane_cloud.add_point(cloud.points[idx]);
        if (cloud.has_colors()) {
            result.plane_cloud.colors.push_back(cloud.colors[idx]);
        }
    }
    
    for (size_t i = 0; i < n; ++i) {
        if (!is_plane[i]) {
            result.remaining.add_point(cloud.points[i]);
            if (cloud.has_colors()) {
                result.remaining.colors.push_back(cloud.colors[i]);
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode euclidean_cluster_extraction(PointCloudData& cloud,
                                        ClusterSegmentResult& result,
                                        float tolerance,
                                        int min_cluster_size,
                                        int max_cluster_size) {
    if (cloud.empty() || tolerance <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    result.clusters.clear();
    result.cluster_indices.clear();
    
    size_t n = cloud.size();
    float tolerance_sq = sqr(tolerance);
    
    Vector<bool> visited(n, false);
    
    for (size_t i = 0; i < n; ++i) {
        if (visited[i]) continue;
        
        // BFS聚类
        Vector<int> cluster_indices;
        std::queue<size_t> queue;
        queue.push(i);
        visited[i] = true;
        
        while (!queue.empty()) {
            size_t current = queue.front();
            queue.pop();
            cluster_indices.push_back(static_cast<int>(current));
            
            // 找邻近点
            for (size_t j = 0; j < n; ++j) {
                if (!visited[j] && distance_squared(cloud.points[current], cloud.points[j]) <= tolerance_sq) {
                    visited[j] = true;
                    queue.push(j);
                }
            }
        }
        
        // 检查聚类大小
        if (cluster_indices.size() >= min_cluster_size && 
            cluster_indices.size() <= max_cluster_size) {
            
            PointCloudData cluster;
            for (int idx : cluster_indices) {
                cluster.add_point(cloud.points[idx]);
                if (cloud.has_colors()) {
                    cluster.colors.push_back(cloud.colors[idx]);
                }
            }
            
            result.clusters.push_back(std::move(cluster));
            result.cluster_indices.push_back(std::move(cluster_indices));
        }
    }
    
    result.cluster_count = static_cast<int>(result.clusters.size());
    
    return ErrorCode::Success;
}

// ============================================================================
// 点云重建
// ============================================================================

ErrorCode poisson_reconstruction(const PointCloudData& cloud,
                                  const Vector<PointCloudNormal>& normals,
                                  PointCloudMesh& mesh,
                                  int depth,
                                  int solver_divide,
                                  int iso_divide,
                                  float point_weight) {
    if (cloud.empty() || normals.size() != cloud.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    mesh.clear();
    
    // 简化实现：Marching Cubes变体
    int grid_size = 1 << depth;
    float cell_size = 1.0f / grid_size;
    
    // 计算包围盒
    Point3Df min_pt(std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max());
    Point3Df max_pt(std::numeric_limits<float>::lowest(),
                    std::numeric_limits<float>::lowest(),
                    std::numeric_limits<float>::lowest());
    
    for (const auto& pt : cloud.points) {
        min_pt.x = std::min(min_pt.x, pt.x);
        min_pt.y = std::min(min_pt.y, pt.y);
        min_pt.z = std::min(min_pt.z, pt.z);
        max_pt.x = std::max(max_pt.x, pt.x);
        max_pt.y = std::max(max_pt.y, pt.y);
        max_pt.z = std::max(max_pt.z, pt.z);
    }
    
    Point3Df extent = max_pt - min_pt;
    float max_extent = std::max(extent.x, std::max(extent.y, extent.z));
    
    // 构建隐式函数网格
    Vector<Vector<Vector<float>>> implicit_grid(grid_size + 1,
        Vector<Vector<float>>(grid_size + 1, Vector<float>(grid_size + 1, 0.0f)));
    
    // 计算每个格子的隐式函数值
    for (int i = 0; i <= grid_size; ++i) {
        for (int j = 0; j <= grid_size; ++j) {
            for (int k = 0; k <= grid_size; ++k) {
                Point3Df grid_pt(
                    min_pt.x + i * max_extent / grid_size,
                    min_pt.y + j * max_extent / grid_size,
                    min_pt.z + k * max_extent / grid_size
                );
                
                // 找最近的点
                float min_dist = std::numeric_limits<float>::max();
                int nearest_idx = -1;
                
                for (size_t p = 0; p < cloud.size(); ++p) {
                    float d = distance_3d(grid_pt, cloud.points[p]);
                    if (d < min_dist) {
                        min_dist = d;
                        nearest_idx = static_cast<int>(p);
                    }
                }
                
                if (nearest_idx >= 0) {
                    // 使用法向计算隐式值
                    Point3Df diff = grid_pt - cloud.points[nearest_idx];
                    implicit_grid[i][j][k] = dot_product(diff, normals[nearest_idx].normal);
                }
            }
        }
    }
    
    // Marching Cubes
    // 边查找表
    static const int edgeTable[256] = {
        0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
        // ... (简化，只使用前8个)
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
    };
    
    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            for (int k = 0; k < grid_size; ++k) {
                // 计算立方体顶点的配置索引
                int cube_index = 0;
                if (implicit_grid[i][j][k] > 0) cube_index |= 1;
                if (implicit_grid[i+1][j][k] > 0) cube_index |= 2;
                if (implicit_grid[i+1][j+1][k] > 0) cube_index |= 4;
                if (implicit_grid[i][j+1][k] > 0) cube_index |= 8;
                if (implicit_grid[i][j][k+1] > 0) cube_index |= 16;
                if (implicit_grid[i+1][j][k+1] > 0) cube_index |= 32;
                if (implicit_grid[i+1][j+1][k+1] > 0) cube_index |= 64;
                if (implicit_grid[i][j+1][k+1] > 0) cube_index |= 128;
                
                if (cube_index == 0 || cube_index == 255) continue;
                
                // 计算顶点位置（简化）
                Point3Df v0(min_pt.x + i * max_extent / grid_size,
                            min_pt.y + j * max_extent / grid_size,
                            min_pt.z + k * max_extent / grid_size);
                
                // 简化：只添加立方体顶点作为网格顶点
                mesh.add_vertex(v0);
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode greedy_triangulation(const PointCloudData& cloud,
                                const Vector<PointCloudNormal>& normals,
                                PointCloudMesh& mesh,
                                float search_radius,
                                float mu,
                                int max_neighbors,
                                float max_angle,
                                float min_angle) {
    if (cloud.empty() || normals.size() != cloud.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    mesh.clear();
    
    float radius_sq = sqr(search_radius);
    float max_angle_rad = max_angle * M_PI / 180.0f;
    float min_angle_rad = min_angle * M_PI / 180.0f;
    
    // 添加所有点作为顶点
    for (const auto& pt : cloud.points) {
        mesh.add_vertex(pt);
    }
    
    if (cloud.has_colors()) {
        mesh.vertex_colors = cloud.colors;
    }
    
    // 构建三角形
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt_i = cloud.points[i];
        
        // 找邻近点
        Vector<size_t> neighbors;
        for (size_t j = 0; j < cloud.size(); ++j) {
            if (distance_squared(pt_i, cloud.points[j]) <= radius_sq) {
                neighbors.push_back(j);
            }
        }
        
        // 排序邻居
        std::sort(neighbors.begin(), neighbors.end(), [&cloud, &pt_i](size_t a, size_t b) {
            return distance_3d(pt_i, cloud.points[a]) < distance_3d(pt_i, cloud.points[b]);
        });
        
        // 构建三角形
        if (neighbors.size() >= 2) {
            // 取最近的邻居构建三角形
            size_t j = neighbors[0];
            size_t k = neighbors[1];
            
            // 检查角度约束
            Point3Df v1 = cloud.points[j] - pt_i;
            Point3Df v2 = cloud.points[k] - pt_i;
            
            float angle = std::acos(std::max(-1.0f, std::min(1.0f, 
                dot_product(v1.normalized(), v2.normalized()))));
            
            if (angle >= min_angle_rad && angle <= max_angle_rad) {
                mesh.add_face(static_cast<uint32_t>(i), 
                              static_cast<uint32_t>(j), 
                              static_cast<uint32_t>(k));
            }
        }
    }
    
    return ErrorCode::Success;
}

ErrorCode surface_reconstruction(const PointCloudData& cloud,
                                  PointCloudMesh& mesh,
                                  float grid_resolution) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 先计算法向
    Vector<PointCloudNormal> normals;
    estimate_normals(cloud, normals, grid_resolution * 3.0f);
    
    // 使用简化泊松重建
    return poisson_reconstruction(cloud, normals, mesh, 6);
}

} // namespace pcl_3d_utils

// ============================================================================
// 节点实现
// ============================================================================

// ----- 点云获取节点 -----

PointCloudFromDepthNode::PointCloudFromDepthNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudFromDepthNode::make_info() {
    NodeInfo info;
    info.id = "pcl_pointcloud_from_depth";
    info.name = "深度图转点云";
    info.category = "3D重建/点云获取";
    info.description = "从深度图像生成3D点云";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("depth_image", "深度图", DataType::DepthImage, true);
    info.inputs.emplace_back("color_image", "彩色图", DataType::Image, false);
    
    info.outputs.emplace_back("pointcloud", "点云", DataType::PointCloud);
    
    info.params.emplace_back("depth_scale", "深度缩放因子", DataType::Number, Data(1.0f));
    info.params.emplace_back("focal_length_x", "焦距X", DataType::Number, Data(0.0f));
    info.params.emplace_back("focal_length_y", "焦距Y", DataType::Number, Data(0.0f));
    info.params.emplace_back("center_x", "光心X", DataType::Number, Data(0.0f));
    info.params.emplace_back("center_y", "光心Y", DataType::Number, Data(0.0f));
    
    return info;
}

Result<void> PointCloudFromDepthNode::execute(FlowContext& context) {
    auto depth_input = get_input("depth_image");
    if (!depth_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少深度图输入");
    }
    
    depth_scale_ = static_cast<float>(get_param("depth_scale", Data(1.0f)).as_number());
    focal_length_x_ = static_cast<float>(get_param("focal_length_x", Data(0.0f)).as_number());
    focal_length_y_ = static_cast<float>(get_param("focal_length_y", Data(0.0f)).as_number());
    center_x_ = static_cast<float>(get_param("center_x", Data(0.0f)).as_number());
    center_y_ = static_cast<float>(get_param("center_y", Data(0.0f)).as_number());
    
    DepthImageData depth_img = depth_input.as_depth_image();
    depth_img.depth_scale = depth_scale_;
    depth_img.focal_length_x = focal_length_x_;
    depth_img.focal_length_y = focal_length_y_;
    depth_img.center_x = center_x_;
    depth_img.center_y = center_y_;
    
    auto color_input = get_input("color_image");
    if (color_input.is_valid()) {
        depth_img.color = color_input.as_image();
    }
    
    PointCloudData cloud;
    ErrorCode err = pcl_3d_utils::depth_to_pointcloud_advanced(depth_img, cloud, depth_scale_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "深度图转点云失败");
    }
    
    set_output("pointcloud", Data(std::move(cloud)));
    return Result<void>::success();
}

PointCloudFromStereoNode::PointCloudFromStereoNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudFromStereoNode::make_info() {
    NodeInfo info;
    info.id = "pcl_pointcloud_from_stereo";
    info.name = "立体视觉转点云";
    info.category = "3D重建/点云获取";
    info.description = "从立体视觉图像生成3D点云";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("left_image", "左图（视差图）", DataType::Image, true);
    info.inputs.emplace_back("right_image", "右图（彩色图）", DataType::Image, true);
    
    info.outputs.emplace_back("pointcloud", "点云", DataType::PointCloud);
    
    info.params.emplace_back("baseline", "基线距离", DataType::Number, Data(0.0f));
    info.params.emplace_back("focal_length", "焦距", DataType::Number, Data(0.0f));
    info.params.emplace_back("disparity_scale", "视差缩放因子", DataType::Number, Data(1.0f));
    
    return info;
}

Result<void> PointCloudFromStereoNode::execute(FlowContext& context) {
    auto left_input = get_input("left_image");
    auto right_input = get_input("right_image");
    
    if (!left_input.is_valid() || !right_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少立体图像输入");
    }
    
    baseline_ = static_cast<float>(get_param("baseline", Data(0.0f)).as_number());
    focal_length_ = static_cast<float>(get_param("focal_length", Data(0.0f)).as_number());
    disparity_scale_ = static_cast<float>(get_param("disparity_scale", Data(1.0f)).as_number());
    
    PointCloudData cloud;
    ErrorCode err = pcl_3d_utils::stereo_to_pointcloud(
        left_input.as_image(), right_input.as_image(), 
        cloud, baseline_, focal_length_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "立体视觉转点云失败");
    }
    
    set_output("pointcloud", Data(std::move(cloud)));
    return Result<void>::success();
}

PointCloudFromFileNode::PointCloudFromFileNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudFromFileNode::make_info() {
    NodeInfo info;
    info.id = "pcl_pointcloud_from_file";
    info.name = "加载点云文件";
    info.category = "3D重建/点云获取";
    info.description = "从PCD/PLY文件加载点云数据";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("filepath", "文件路径", DataType::String, true);
    
    info.outputs.emplace_back("pointcloud", "点云", DataType::PointCloud);
    
    info.params.emplace_back("file_format", "文件格式", DataType::Number, Data(0));
    Vector<String> format_options;
    format_options.push_back("自动检测");
    format_options.push_back("PCD格式");
    format_options.push_back("PLY格式");
    info.params.back().options = format_options;
    
    return info;
}

Result<void> PointCloudFromFileNode::execute(FlowContext& context) {
    auto filepath_input = get_input("filepath");
    if (!filepath_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少文件路径输入");
    }
    
    filepath_ = filepath_input.as_string();
    file_format_ = get_param("file_format", Data(0)).as_int();
    
    PointCloudData cloud;
    ErrorCode err = ErrorCode::Success;
    
    if (file_format_ == 1) {
        err = pcl_3d_utils::load_pcd_file(filepath_, cloud);
    } else if (file_format_ == 2) {
        err = pcl_3d_utils::load_ply_file(filepath_, cloud);
    } else {
        err = pcl_3d_utils::load_pointcloud_file(filepath_, cloud);
    }
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "加载点云文件失败: " + filepath_);
    }
    
    set_output("pointcloud", Data(std::move(cloud)));
    return Result<void>::success();
}

// ----- 点云预处理节点 -----

PointCloudFilterNode::PointCloudFilterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudFilterNode::make_info() {
    NodeInfo info;
    info.id = "pcl_pointcloud_filter";
    info.name = "点云滤波";
    info.category = "3D重建/点云预处理";
    info.description = "点云滤波处理（去除离群点、噪声点）";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("filtered_pointcloud", "滤波后点云", DataType::PointCloud);
    
    info.params.emplace_back("method", "滤波方法", DataType::Number, Data(0));
    Vector<String> method_options;
    method_options.push_back("统计离群点去除");
    method_options.push_back("半径离群点去除");
    method_options.push_back("体素网格滤波");
    info.params.back().options = method_options;
    
    info.params.emplace_back("mean_k", "统计滤波K邻域", DataType::Number, Data(50));
    info.params.emplace_back("std_threshold", "标准差阈值", DataType::Number, Data(1.0f));
    info.params.emplace_back("radius", "半径滤波半径", DataType::Number, Data(0.05f));
    info.params.emplace_back("min_neighbors", "最小邻居数", DataType::Number, Data(5));
    info.params.emplace_back("voxel_size", "体素大小", DataType::Number, Data(0.01f));
    
    return info;
}

Result<void> PointCloudFilterNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    method_ = static_cast<FilterMethod>(get_param("method", Data(0)).as_int());
    mean_k_ = get_param("mean_k", Data(50)).as_int();
    std_threshold_ = static_cast<float>(get_param("std_threshold", Data(1.0f)).as_number());
    radius_ = static_cast<float>(get_param("radius", Data(0.05f)).as_number());
    min_neighbors_ = get_param("min_neighbors", Data(5)).as_int();
    voxel_size_ = static_cast<float>(get_param("voxel_size", Data(0.01f)).as_number());
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    ErrorCode err = ErrorCode::Success;
    
    switch (method_) {
        case FilterMethod::Statistical:
            err = pcl_3d_utils::statistical_outlier_removal(cloud, mean_k_, std_threshold_);
            break;
        case FilterMethod::Radius: {
            // 半径滤波简化实现
            size_t n = cloud.size();
            float radius_sq = radius_ * radius_;
            PointCloudData filtered;
            
            for (size_t i = 0; i < n; ++i) {
                int count = 0;
                for (size_t j = 0; j < n && count < min_neighbors_; ++j) {
                    if (i != j && pcl_3d_utils::distance_squared(cloud.points[i], cloud.points[j]) < radius_sq) {
                        ++count;
                    }
                }
                if (count >= min_neighbors_) {
                    filtered.add_point(cloud.points[i]);
                    if (cloud.has_colors()) {
                        filtered.colors.push_back(cloud.colors[i]);
                    }
                }
            }
            cloud = std::move(filtered);
            break;
        }
        case FilterMethod::VoxelGrid:
            err = pcl_3d_utils::voxel_grid_downsample(cloud, voxel_size_);
            break;
    }
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云滤波失败");
    }
    
    set_output("filtered_pointcloud", Data(std::move(cloud)));
    return Result<void>::success();
}

PointCloudDownsampleNode::PointCloudDownsampleNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudDownsampleNode::make_info() {
    NodeInfo info;
    info.id = "pcl_pointcloud_downsample";
    info.name = "点云下采样";
    info.category = "3D重建/点云预处理";
    info.description = "VoxelGrid点云下采样";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("downsampled_pointcloud", "下采样后点云", DataType::PointCloud);
    
    info.params.emplace_back("voxel_size", "体素大小", DataType::Number, Data(0.01f));
    
    return info;
}

Result<void> PointCloudDownsampleNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    voxel_size_ = static_cast<float>(get_param("voxel_size", Data(0.01f)).as_number());
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    ErrorCode err = pcl_3d_utils::voxel_grid_downsample(cloud, voxel_size_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云下采样失败");
    }
    
    set_output("downsampled_pointcloud", Data(std::move(cloud)));
    return Result<void>::success();
}

PointCloudNormalEstimationNode::PointCloudNormalEstimationNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudNormalEstimationNode::make_info() {
    NodeInfo info;
    info.id = "pcl_normal_estimation";
    info.name = "法向估计";
    info.category = "3D重建/点云预处理";
    info.description = "点云法向估计（PCA方法）";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("normals_count", "法向数量", DataType::Number);
    info.outputs.emplace_back("avg_curvature", "平均曲率", DataType::Number);
    
    info.params.emplace_back("method", "估计方法", DataType::Number, Data(0));
    Vector<String> method_options;
    method_options.push_back("PCA方法");
    method_options.push_back("K邻域方法");
    method_options.push_back("半径搜索方法");
    info.params.back().options = method_options;
    
    info.params.emplace_back("search_radius", "搜索半径", DataType::Number, Data(0.03f));
    info.params.emplace_back("k_neighbors", "K邻域数", DataType::Number, Data(20));
    
    return info;
}

Result<void> PointCloudNormalEstimationNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    method_ = static_cast<EstimationMethod>(get_param("method", Data(0)).as_int());
    search_radius_ = static_cast<float>(get_param("search_radius", Data(0.03f)).as_number());
    k_neighbors_ = get_param("k_neighbors", Data(20)).as_int();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    Vector<PointCloudNormal> normals;
    
    ErrorCode err;
    if (method_ == EstimationMethod::KNN) {
        err = pcl_3d_utils::estimate_normals_knn(cloud, normals, k_neighbors_);
    } else {
        err = pcl_3d_utils::estimate_normals(cloud, normals, search_radius_, 
                                              (method_ == EstimationMethod::PCA) ? 0 : k_neighbors_);
    }
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "法向估计失败");
    }
    
    // 计算平均曲率
    float avg_curvature = 0.0f;
    for (const auto& n : normals) {
        avg_curvature += n.curvature;
    }
    avg_curvature /= normals.size();
    
    set_output("normals_count", Data(static_cast<int>(normals.size())));
    set_output("avg_curvature", Data(avg_curvature));
    
    return Result<void>::success();
}

// ----- 点云配准节点 -----

PointCloudICPNode::PointCloudICPNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudICPNode::make_info() {
    NodeInfo info;
    info.id = "pcl_icp_registration";
    info.name = "ICP配准";
    info.category = "3D重建/点云配准";
    info.description = "ICP迭代最近点配准（精配准）";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "靶点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_normals", "靶点云法向", DataType::Array, false);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    info.outputs.emplace_back("rmse", "均方根误差", DataType::Number);
    
    info.params.emplace_back("method", "ICP方法", DataType::Number, Data(0));
    Vector<String> method_options;
    method_options.push_back("点到点");
    method_options.push_back("点到面");
    info.params.back().options = method_options;
    
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(50));
    info.params.emplace_back("tolerance", "收敛阈值", DataType::Number, Data(1e-6f));
    info.params.emplace_back("max_correspondence_distance", "最大对应距离", DataType::Number, Data(0.05f));
    
    return info;
}

Result<void> PointCloudICPNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    method_ = static_cast<ICPMethod>(get_param("method", Data(0)).as_int());
    max_iterations_ = get_param("max_iterations", Data(50)).as_int();
    tolerance_ = static_cast<float>(get_param("tolerance", Data(1e-6f)).as_number());
    max_correspondence_distance_ = static_cast<float>(get_param("max_correspondence_distance", Data(0.05f)).as_number());
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    RegistrationResult result;
    ErrorCode err;
    
    if (method_ == ICPMethod::PointToPlane) {
        auto normals_input = get_input("target_normals");
        Vector<PointCloudNormal> target_normals;
        
        if (normals_input.is_valid()) {
            // 使用输入的法向（简化处理）
            pcl_3d_utils::estimate_normals(target, target_normals, max_correspondence_distance_ * 2);
        } else {
            pcl_3d_utils::estimate_normals(target, target_normals, max_correspondence_distance_ * 2);
        }
        
        err = pcl_3d_utils::icp_point_to_plane(source, target, target_normals, result,
                                                max_iterations_, tolerance_, max_correspondence_distance_);
    } else {
        err = pcl_3d_utils::icp_registration(source, target, result,
                                              max_iterations_, tolerance_, max_correspondence_distance_);
    }
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "ICP配准失败");
    }
    
    // 变换源点云
    PointCloudData transformed;
    for (const auto& pt : source.points) {
        transformed.add_point(result.transform.transform(pt));
    }
    if (source.has_colors()) {
        transformed.colors = source.colors;
    }
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    set_output("rmse", Data(result.rmse));
    
    return Result<void>::success();
}

PointCloudNDTNode::PointCloudNDTNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudNDTNode::make_info() {
    NodeInfo info;
    info.id = "pcl_ndt_registration";
    info.name = "NDT配准";
    info.category = "3D重建/点云配准";
    info.description = "NDT正态分布变换配准";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "靶点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    
    info.params.emplace_back("voxel_size", "体素大小", DataType::Number, Data(1.0f));
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(50));
    info.params.emplace_back("tolerance", "收敛阈值", DataType::Number, Data(1e-6f));
    info.params.emplace_back("step_size", "步长", DataType::Number, Data(0.1f));
    
    return info;
}

Result<void> PointCloudNDTNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    voxel_size_ = static_cast<float>(get_param("voxel_size", Data(1.0f)).as_number());
    max_iterations_ = get_param("max_iterations", Data(50)).as_int();
    tolerance_ = static_cast<float>(get_param("tolerance", Data(1e-6f)).as_number());
    step_size_ = static_cast<float>(get_param("step_size", Data(0.1f)).as_number());
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    RegistrationResult result;
    ErrorCode err = pcl_3d_utils::ndt_registration(source, target, result,
                                                    voxel_size_, max_iterations_, tolerance_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "NDT配准失败");
    }
    
    PointCloudData transformed;
    for (const auto& pt : source.points) {
        transformed.add_point(result.transform.transform(pt));
    }
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    
    return Result<void>::success();
}

PointCloudFeatureMatchingNode::PointCloudFeatureMatchingNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudFeatureMatchingNode::make_info() {
    NodeInfo info;
    info.id = "pcl_feature_matching_registration";
    info.name = "特征匹配配准";
    info.category = "3D重建/点云配准";
    info.description = "基于特征匹配的点云配准（粗配准）";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "靶点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    
    info.params.emplace_back("feature_type", "特征类型", DataType::Number, Data(0));
    Vector<String> feature_options;
    feature_options.push_back("FPFH特征");
    feature_options.push_back("SIFT 3D特征");
    info.params.back().options = feature_options;
    
    info.params.emplace_back("feature_radius", "特征半径", DataType::Number, Data(0.05f));
    info.params.emplace_back("sac_iterations", "SAC迭代次数", DataType::Number, Data(100));
    
    return info;
}

Result<void> PointCloudFeatureMatchingNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    feature_type_ = static_cast<FeatureType>(get_param("feature_type", Data(0)).as_int());
    feature_radius_ = static_cast<float>(get_param("feature_radius", Data(0.05f)).as_number());
    sac_iterations_ = get_param("sac_iterations", Data(100)).as_int();
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    RegistrationResult result;
    ErrorCode err = pcl_3d_utils::feature_matching_registration(source, target, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "特征匹配配准失败");
    }
    
    PointCloudData transformed;
    for (const auto& pt : source.points) {
        transformed.add_point(result.transform.transform(pt));
    }
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    
    return Result<void>::success();
}

PointCloudGlobalRegistrationNode::PointCloudGlobalRegistrationNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudGlobalRegistrationNode::make_info() {
    NodeInfo info;
    info.id = "pcl_global_registration";
    info.name = "全局配准";
    info.category = "3D重建/点云配准";
    info.description = "多视角点云全局配准融合";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("clouds_array", "点云数组", DataType::Array, true);
    
    info.outputs.emplace_back("merged_cloud", "合并点云", DataType::PointCloud);
    info.outputs.emplace_back("transforms_count", "变换数量", DataType::Number);
    
    info.params.emplace_back("use_pairwise", "使用逐对配准", DataType::Boolean, Data(true));
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(50));
    info.params.emplace_back("convergence_threshold", "收敛阈值", DataType::Number, Data(1e-6f));
    
    return info;
}

Result<void> PointCloudGlobalRegistrationNode::execute(FlowContext& context) {
    auto clouds_input = get_input("clouds_array");
    if (!clouds_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云数组输入");
    }
    
    use_pairwise_ = get_param("use_pairwise", Data(true)).as_bool();
    max_iterations_ = get_param("max_iterations", Data(50)).as_int();
    convergence_threshold_ = static_cast<float>(get_param("convergence_threshold", Data(1e-6f)).as_number());
    
    // 简化：假设输入是点云数量
    int clouds_count = clouds_input.as_int();
    
    // 创建模拟点云数组
    Vector<PointCloudData> clouds(clouds_count);
    Vector<Transform3D> transforms;
    Vector<RegistrationResult> results;
    
    ErrorCode err = pcl_3d_utils::global_registration(clouds, transforms, results);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "全局配准失败");
    }
    
    // 合并所有变换后的点云
    PointCloudData merged;
    for (const auto& cloud : clouds) {
        for (const auto& pt : cloud.points) {
            merged.add_point(pt);
        }
    }
    
    set_output("merged_cloud", Data(std::move(merged)));
    set_output("transforms_count", Data(static_cast<int>(transforms.size())));
    
    return Result<void>::success();
}

// ----- 点云分割节点 -----

PointCloudSegmentPlaneNode::PointCloudSegmentPlaneNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudSegmentPlaneNode::make_info() {
    NodeInfo info;
    info.id = "pcl_plane_segmentation";
    info.name = "平面分割";
    info.category = "3D重建/点云分割";
    info.description = "RANSAC平面分割";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("plane_cloud", "平面点云", DataType::PointCloud);
    info.outputs.emplace_back("remaining_cloud", "剩余点云", DataType::PointCloud);
    info.outputs.emplace_back("plane_normal", "平面法向", DataType::Point);
    
    info.params.emplace_back("distance_threshold", "距离阈值", DataType::Number, Data(0.01f));
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(100));
    info.params.emplace_back("min_inliers", "最小内点数", DataType::Number, Data(100));
    
    return info;
}

Result<void> PointCloudSegmentPlaneNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    distance_threshold_ = static_cast<float>(get_param("distance_threshold", Data(0.01f)).as_number());
    max_iterations_ = get_param("max_iterations", Data(100)).as_int();
    min_inliers_ = get_param("min_inliers", Data(100)).as_int();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    PlaneSegmentResult result;
    
    ErrorCode err = pcl_3d_utils::ransac_plane_segmentation(cloud, result,
                                                             distance_threshold_, max_iterations_, min_inliers_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "平面分割失败");
    }
    
    set_output("plane_cloud", Data(std::move(result.plane_cloud)));
    set_output("remaining_cloud", Data(std::move(result.remaining)));
    set_output("plane_normal", Data(result.plane.normal));
    
    return Result<void>::success();
}

PointCloudSegmentClusterNode::PointCloudSegmentClusterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudSegmentClusterNode::make_info() {
    NodeInfo info;
    info.id = "pcl_cluster_segmentation";
    info.name = "聚类分割";
    info.category = "3D重建/点云分割";
    info.description = "欧式聚类分割";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("cluster_count", "聚类数量", DataType::Number);
    info.outputs.emplace_back("largest_cluster", "最大聚类", DataType::PointCloud);
    
    info.params.emplace_back("tolerance", "聚类容差", DataType::Number, Data(0.02f));
    info.params.emplace_back("min_cluster_size", "最小聚类大小", DataType::Number, Data(100));
    info.params.emplace_back("max_cluster_size", "最大聚类大小", DataType::Number, Data(25000));
    
    return info;
}

Result<void> PointCloudSegmentClusterNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    tolerance_ = static_cast<float>(get_param("tolerance", Data(0.02f)).as_number());
    min_cluster_size_ = get_param("min_cluster_size", Data(100)).as_int();
    max_cluster_size_ = get_param("max_cluster_size", Data(25000)).as_int();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    ClusterSegmentResult result;
    
    ErrorCode err = pcl_3d_utils::euclidean_cluster_extraction(cloud, result,
                                                                tolerance_, min_cluster_size_, max_cluster_size_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "聚类分割失败");
    }
    
    set_output("cluster_count", Data(result.cluster_count));
    
    // 返回最大聚类
    if (!result.clusters.empty()) {
        size_t largest_idx = 0;
        size_t largest_size = result.clusters[0].size();
        
        for (size_t i = 1; i < result.clusters.size(); ++i) {
            if (result.clusters[i].size() > largest_size) {
                largest_size = result.clusters[i].size();
                largest_idx = i;
            }
        }
        
        set_output("largest_cluster", Data(std::move(result.clusters[largest_idx])));
    }
    
    return Result<void>::success();
}

// ----- 点云重建节点 -----

PointCloudMeshNode::PointCloudMeshNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudMeshNode::make_info() {
    NodeInfo info;
    info.id = "pcl_mesh_reconstruction";
    info.name = "点云网格化";
    info.category = "3D重建/点云重建";
    info.description = "点云网格化重建（泊松重建、贪婪三角化）";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    info.inputs.emplace_back("normals", "法向", DataType::Array, false);
    
    info.outputs.emplace_back("vertex_count", "顶点数量", DataType::Number);
    info.outputs.emplace_back("face_count", "面片数量", DataType::Number);
    
    info.params.emplace_back("method", "重建方法", DataType::Number, Data(0));
    Vector<String> method_options;
    method_options.push_back("泊松重建");
    method_options.push_back("贪婪三角化");
    method_options.push_back("Marching Cubes");
    info.params.back().options = method_options;
    
    info.params.emplace_back("poisson_depth", "泊松深度", DataType::Number, Data(8));
    info.params.emplace_back("search_radius", "搜索半径", DataType::Number, Data(0.025f));
    info.params.emplace_back("grid_resolution", "网格分辨率", DataType::Number, Data(0.01f));
    
    return info;
}

Result<void> PointCloudMeshNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    method_ = static_cast<MeshMethod>(get_param("method", Data(0)).as_int());
    poisson_depth_ = get_param("poisson_depth", Data(8)).as_int();
    search_radius_ = static_cast<float>(get_param("search_radius", Data(0.025f)).as_number());
    grid_resolution_ = static_cast<float>(get_param("grid_resolution", Data(0.01f)).as_number());
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    PointCloudMesh mesh;
    ErrorCode err;
    
    // 获取或计算法向
    Vector<PointCloudNormal> normals;
    auto normals_input = get_input("normals");
    if (normals_input.is_valid()) {
        pcl_3d_utils::estimate_normals(cloud, normals, search_radius_);
    } else {
        pcl_3d_utils::estimate_normals(cloud, normals, search_radius_);
    }
    
    switch (method_) {
        case MeshMethod::Poisson:
            err = pcl_3d_utils::poisson_reconstruction(cloud, normals, mesh, poisson_depth_);
            break;
        case MeshMethod::GreedyTriangle:
            err = pcl_3d_utils::greedy_triangulation(cloud, normals, mesh, search_radius_);
            break;
        case MeshMethod::MarchingCubes:
            err = pcl_3d_utils::surface_reconstruction(cloud, mesh, grid_resolution_);
            break;
    }
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云网格化失败");
    }
    
    set_output("vertex_count", Data(static_cast<int>(mesh.vertex_count())));
    set_output("face_count", Data(static_cast<int>(mesh.face_count())));
    
    return Result<void>::success();
}

PointCloudSurfaceNode::PointCloudSurfaceNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudSurfaceNode::make_info() {
    NodeInfo info;
    info.id = "pcl_surface_reconstruction";
    info.name = "表面重建";
    info.category = "3D重建/点云重建";
    info.description = "点云表面重建";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("vertex_count", "顶点数量", DataType::Number);
    info.outputs.emplace_back("face_count", "面片数量", DataType::Number);
    
    info.params.emplace_back("resolution", "分辨率", DataType::Number, Data(0.01f));
    info.params.emplace_back("smooth_surface", "平滑表面", DataType::Boolean, Data(true));
    
    return info;
}

Result<void> PointCloudSurfaceNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    resolution_ = static_cast<float>(get_param("resolution", Data(0.01f)).as_number());
    smooth_surface_ = get_param("smooth_surface", Data(true)).as_bool();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    PointCloudMesh mesh;
    
    ErrorCode err = pcl_3d_utils::surface_reconstruction(cloud, mesh, resolution_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "表面重建失败");
    }
    
    set_output("vertex_count", Data(static_cast<int>(mesh.vertex_count())));
    set_output("face_count", Data(static_cast<int>(mesh.face_count())));
    
    return Result<void>::success();
}

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(PointCloudFromDepthNode, "PointCloudFromDepth", PointCloudFromDepthNode::make_info())
OVF_REGISTER_NODE(PointCloudFromStereoNode, "PointCloudFromStereo", PointCloudFromStereoNode::make_info())
OVF_REGISTER_NODE(PointCloudFromFileNode, "PointCloudFromFile", PointCloudFromFileNode::make_info())
OVF_REGISTER_NODE(PointCloudFilterNode, "PointCloudFilter", PointCloudFilterNode::make_info())
OVF_REGISTER_NODE(PointCloudDownsampleNode, "PointCloudDownsample", PointCloudDownsampleNode::make_info())
OVF_REGISTER_NODE(PointCloudNormalEstimationNode, "PointCloudNormalEstimation", PointCloudNormalEstimationNode::make_info())
OVF_REGISTER_NODE(PointCloudICPNode, "PointCloudICP", PointCloudICPNode::make_info())
OVF_REGISTER_NODE(PointCloudNDTNode, "PointCloudNDT", PointCloudNDTNode::make_info())
OVF_REGISTER_NODE(PointCloudFeatureMatchingNode, "PointCloudFeatureMatching", PointCloudFeatureMatchingNode::make_info())
OVF_REGISTER_NODE(PointCloudGlobalRegistrationNode, "PointCloudGlobalRegistration", PointCloudGlobalRegistrationNode::make_info())
OVF_REGISTER_NODE(PointCloudSegmentPlaneNode, "PointCloudSegmentPlane", PointCloudSegmentPlaneNode::make_info())
OVF_REGISTER_NODE(PointCloudSegmentClusterNode, "PointCloudSegmentCluster", PointCloudSegmentClusterNode::make_info())
OVF_REGISTER_NODE(PointCloudMeshNode, "PointCloudMesh", PointCloudMeshNode::make_info())
OVF_REGISTER_NODE(PointCloudSurfaceNode, "PointCloudSurface", PointCloudSurfaceNode::make_info())

} // namespace algorithm
} // namespace ovf