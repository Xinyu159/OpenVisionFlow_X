/**
 * @file pointcloud.cpp
 * @brief 3D点云处理模块实现
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#include "ovf/algorithm/pointcloud.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <random>
#include <unordered_map>
#include <cmath>
#include <ctime>

// Windows上需要定义M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {
namespace pointcloud_utils {

// 深度图转点云
ErrorCode depth_to_pointcloud(const DepthImageData& depth_img, PointCloudData& cloud) {
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
            float z = depth_img.get_depth(x, y);
            
            if (z > 0) {
                float px = (static_cast<float>(x) - cx) * z / fx;
                float py = (static_cast<float>(y) - cy) * z / fy;
                float pz = z;
                
                cloud.add_point(px, py, pz);
            } else {
                // 对于无效深度，仍然添加点但z为0（保持organized结构）
                cloud.add_point(0, 0, 0);
            }
        }
    }
    
    // 如果有彩色图，添加颜色
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
            } else {
                // 灰度图转为灰度颜色
                cloud.colors[i] = ColorRGB(depth_img.color.data[y * width + x], 
                                          depth_img.color.data[y * width + x], 
                                          depth_img.color.data[y * width + x]);
            }
        }
    }
    
    return ErrorCode::Success;
}

// 体素滤波
ErrorCode voxel_filter(PointCloudData& cloud, float voxel_size) {
    if (cloud.empty() || voxel_size <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    // 使用哈希表存储体素
    struct VoxelKey {
        int x, y, z;
        
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    
    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            return static_cast<size_t>(k.x) ^ (static_cast<size_t>(k.y) << 10) ^ (static_cast<size_t>(k.z) << 20);
        }
    };
    
    struct VoxelData {
        Point3Df centroid;
        ColorRGB color;
        uint8_t intensity;
        int count;
        bool has_color;
        bool has_intensity;
    };
    
    std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels;
    
    // 将点分配到体素
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
        
        if (cloud.has_intensities()) {
            voxel.intensity += cloud.intensities[i];
            voxel.has_intensity = true;
        }
    }
    
    // 计算每个体素的中心点
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
        
        if (voxel.has_intensity) {
            cloud.intensities.emplace_back(static_cast<uint8_t>(voxel.intensity * inv_count));
        }
    }
    
    return ErrorCode::Success;
}

// 统计滤波
ErrorCode statistical_filter(PointCloudData& cloud, int neighbors, float std_threshold) {
    if (cloud.empty() || neighbors <= 0 || std_threshold <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t n = cloud.size();
    Vector<float> distances(n);
    
    // 计算每个点到其邻近点的平均距离
    for (size_t i = 0; i < n; ++i) {
        // 找最近的k个点
        Vector<float> local_dists(n);
        for (size_t j = 0; j < n; ++j) {
            if (i != j) {
                float dx = cloud.points[i].x - cloud.points[j].x;
                float dy = cloud.points[i].y - cloud.points[j].y;
                float dz = cloud.points[i].z - cloud.points[j].z;
                local_dists[j] = std::sqrt(dx * dx + dy * dy + dz * dz);
            } else {
                local_dists[j] = std::numeric_limits<float>::max();
            }
        }
        
        // 排序并取前k个
        std::partial_sort(local_dists.begin(), local_dists.begin() + neighbors, local_dists.end());
        
        float sum = 0.0f;
        for (int k = 0; k < neighbors && k < static_cast<int>(n); ++k) {
            sum += local_dists[k];
        }
        distances[i] = sum / neighbors;
    }
    
    // 计算均值和标准差
    float sum = 0.0f;
    for (float d : distances) {
        sum += d;
    }
    float mean = sum / n;
    
    float sum_sq = 0.0f;
    for (float d : distances) {
        sum_sq += (d - mean) * (d - mean);
    }
    float stddev = std::sqrt(sum_sq / n);
    
    // 过滤离群点
    float threshold = mean + std_threshold * stddev;
    
    PointCloudData filtered;
    for (size_t i = 0; i < n; ++i) {
        if (distances[i] < threshold) {
            filtered.add_point(cloud.points[i]);
            
            if (cloud.has_colors()) {
                filtered.colors.push_back(cloud.colors[i]);
            }
            if (cloud.has_intensities()) {
                filtered.intensities.push_back(cloud.intensities[i]);
            }
        }
    }
    
    cloud = std::move(filtered);
    return ErrorCode::Success;
}

// 半径滤波
ErrorCode radius_filter(PointCloudData& cloud, float radius, int min_neighbors) {
    if (cloud.empty() || radius <= 0 || min_neighbors <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t n = cloud.size();
    Vector<bool> keep(n, false);
    float radius_sq = radius * radius;
    
    for (size_t i = 0; i < n; ++i) {
        int count = 0;
        for (size_t j = 0; j < n && count < min_neighbors; ++j) {
            if (i != j) {
                float dx = cloud.points[i].x - cloud.points[j].x;
                float dy = cloud.points[i].y - cloud.points[j].y;
                float dz = cloud.points[i].z - cloud.points[j].z;
                float dist_sq = dx * dx + dy * dy + dz * dz;
                
                if (dist_sq < radius_sq) {
                    ++count;
                }
            }
        }
        keep[i] = (count >= min_neighbors);
    }
    
    PointCloudData filtered;
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            filtered.add_point(cloud.points[i]);
            
            if (cloud.has_colors()) {
                filtered.colors.push_back(cloud.colors[i]);
            }
            if (cloud.has_intensities()) {
                filtered.intensities.push_back(cloud.intensities[i]);
            }
        }
    }
    
    cloud = std::move(filtered);
    return ErrorCode::Success;
}

// 点云变换
ErrorCode transform_pointcloud(PointCloudData& cloud, const float matrix[4][4]) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    for (auto& pt : cloud.points) {
        float x = matrix[0][0] * pt.x + matrix[0][1] * pt.y + matrix[0][2] * pt.z + matrix[0][3];
        float y = matrix[1][0] * pt.x + matrix[1][1] * pt.y + matrix[1][2] * pt.z + matrix[1][3];
        float z = matrix[2][0] * pt.x + matrix[2][1] * pt.y + matrix[2][2] * pt.z + matrix[2][3];
        pt.x = x;
        pt.y = y;
        pt.z = z;
    }
    
    return ErrorCode::Success;
}

// 点云旋转
ErrorCode rotate_pointcloud(PointCloudData& cloud, float rx, float ry, float rz) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    Transform3D transform;
    transform.set_rotation_x(rx);
    
    Transform3D rot_y;
    rot_y.set_rotation_y(ry);
    
    Transform3D rot_z;
    rot_z.set_rotation_z(rz);
    
    // 组合旋转矩阵（Z * Y * X）
    float combined[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            combined[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                combined[i][j] += rot_z.m[i][k] * rot_y.m[k][j];
            }
        }
    }
    
    float final_matrix[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            final_matrix[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                final_matrix[i][j] += combined[i][k] * transform.m[k][j];
            }
        }
    }
    
    return transform_pointcloud(cloud, final_matrix);
}

// 点云平移
ErrorCode translate_pointcloud(PointCloudData& cloud, float tx, float ty, float tz) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    for (auto& pt : cloud.points) {
        pt.x += tx;
        pt.y += ty;
        pt.z += tz;
    }
    
    return ErrorCode::Success;
}

// 平面检测（RANSAC）
ErrorCode find_plane(PointCloudData& cloud, Plane3D& plane, 
                     float distance_threshold, int max_iterations) {
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
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择3个点
        size_t i1 = dis(gen);
        size_t i2 = dis(gen);
        size_t i3 = dis(gen);
        
        while (i2 == i1) i2 = dis(gen);
        while (i3 == i1 || i3 == i2) i3 = dis(gen);
        
        const auto& p1 = cloud.points[i1];
        const auto& p2 = cloud.points[i2];
        const auto& p3 = cloud.points[i3];
        
        // 计算平面参数
        Point3Df v1 = p2 - p1;
        Point3Df v2 = p3 - p1;
        Point3Df normal = v1.cross(v2).normalized();
        
        if (normal.length() < 0.1f) {
            continue;  // 三点共线
        }
        
        float d = -(normal.x * p1.x + normal.y * p1.y + normal.z * p1.z);
        
        // 计算内点数量
        int inlier_count = 0;
        for (const auto& pt : cloud.points) {
            float dist = std::abs(normal.x * pt.x + normal.y * pt.y + normal.z * pt.z + d);
            if (dist < distance_threshold) {
                ++inlier_count;
            }
        }
        
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_normal = normal;
            best_d = d;
        }
    }
    
    if (best_inlier_count < 3) {
        plane.valid = false;
        return ErrorCode::AlgorithmExecFailed;
    }
    
    // 计算平面中心点（内点的平均）
    plane.center = Point3Df(0, 0, 0);
    int count = 0;
    
    for (const auto& pt : cloud.points) {
        float dist = std::abs(best_normal.x * pt.x + best_normal.y * pt.y + best_normal.z * pt.z + best_d);
        if (dist < distance_threshold) {
            plane.center.x += pt.x;
            plane.center.y += pt.y;
            plane.center.z += pt.z;
            ++count;
        }
    }
    
    if (count > 0) {
        plane.center.x /= count;
        plane.center.y /= count;
        plane.center.z /= count;
    }
    
    plane.normal = best_normal;
    plane.d = best_d;
    
    // 计算拟合误差
    float total_error = 0.0f;
    for (const auto& pt : cloud.points) {
        float dist = std::abs(best_normal.x * pt.x + best_normal.y * pt.y + best_normal.z * pt.z + best_d);
        if (dist < distance_threshold) {
            total_error += dist;
        }
    }
    plane.fit_error = total_error / count;
    plane.valid = true;
    
    return ErrorCode::Success;
}

// 点云聚类
ErrorCode find_clusters(PointCloudData& cloud, 
                        Vector<PointCloudData>& clusters, 
                        float tolerance,
                        int min_cluster_size,
                        int max_cluster_size) {
    if (cloud.empty() || tolerance <= 0) {
        return ErrorCode::InvalidParameter;
    }
    
    clusters.clear();
    size_t n = cloud.size();
    
    // 标记已访问的点
    Vector<bool> visited(n, false);
    Vector<int> cluster_labels(n, -1);
    float tolerance_sq = tolerance * tolerance;
    
    int current_cluster_id = 0;
    
    for (size_t i = 0; i < n; ++i) {
        if (visited[i]) {
            continue;
        }
        
        // 开始新的聚类
        Vector<size_t> cluster_indices;
        Vector<size_t> seed_queue;
        seed_queue.push_back(i);
        visited[i] = true;
        
        while (!seed_queue.empty()) {
            size_t current = seed_queue.back();
            seed_queue.pop_back();
            cluster_indices.push_back(current);
            
            // 找邻近点
            for (size_t j = 0; j < n; ++j) {
                if (!visited[j]) {
                    float dx = cloud.points[current].x - cloud.points[j].x;
                    float dy = cloud.points[current].y - cloud.points[j].y;
                    float dz = cloud.points[current].z - cloud.points[j].z;
                    float dist_sq = dx * dx + dy * dy + dz * dz;
                    
                    if (dist_sq < tolerance_sq) {
                        visited[j] = true;
                        seed_queue.push_back(j);
                    }
                }
            }
        }
        
        // 检查聚类大小
        int cluster_size = static_cast<int>(cluster_indices.size());
        if (cluster_size >= min_cluster_size && cluster_size <= max_cluster_size) {
            PointCloudData cluster;
            for (size_t idx : cluster_indices) {
                cluster.add_point(cloud.points[idx]);
                
                if (cloud.has_colors()) {
                    cluster.colors.push_back(cloud.colors[idx]);
                }
                if (cloud.has_intensities()) {
                    cluster.intensities.push_back(cloud.intensities[idx]);
                }
            }
            clusters.push_back(std::move(cluster));
            ++current_cluster_id;
        }
    }
    
    return ErrorCode::Success;
}

// 计算包围盒
ErrorCode compute_bounding_box(PointCloudData& cloud, Point3Df& min_pt, Point3Df& max_pt) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    min_pt = Point3Df(std::numeric_limits<float>::max(), 
                      std::numeric_limits<float>::max(), 
                      std::numeric_limits<float>::max());
    max_pt = Point3Df(std::numeric_limits<float>::lowest(), 
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
    
    return ErrorCode::Success;
}

// 计算两点距离
float compute_distance(const Point3Df& p1, const Point3Df& p2) {
    return p1.distance_to(p2);
}

// 计算点云高度
float compute_height(PointCloudData& cloud, const Point3Df& plane_normal) {
    if (cloud.empty()) {
        return 0.0f;
    }
    
    // 找到距离平面最远的点
    Point3Df n = plane_normal.normalized();
    
    float max_height = 0.0f;
    for (const auto& pt : cloud.points) {
        float height = std::abs(n.x * pt.x + n.y * pt.y + n.z * pt.z);
        max_height = std::max(max_height, height);
    }
    
    return max_height;
}

// 计算重心
Point3Df compute_centroid(const PointCloudData& cloud) {
    if (cloud.empty()) {
        return Point3Df(0, 0, 0);
    }
    
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (const auto& pt : cloud.points) {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_z += pt.z;
    }
    
    float n = static_cast<float>(cloud.size());
    return Point3Df(sum_x / n, sum_y / n, sum_z / n);
}

// 点云归一化
void normalize_pointcloud(PointCloudData& cloud) {
    if (cloud.empty()) {
        return;
    }
    
    Point3Df centroid = compute_centroid(cloud);
    
    for (auto& pt : cloud.points) {
        pt.x -= centroid.x;
        pt.y -= centroid.y;
        pt.z -= centroid.z;
    }
}

// 计算密度
float compute_density(PointCloudData& cloud, float radius) {
    if (cloud.empty() || radius <= 0) {
        return 0.0f;
    }
    
    size_t n = cloud.size();
    float radius_sq = radius * radius;
    
    // 计算平均邻居数
    float total_neighbors = 0.0f;
    int sample_count = std::min(static_cast<int>(n), 100);  // 采样以提高性能
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, n - 1);
    
    for (int s = 0; s < sample_count; ++s) {
        size_t i = dis(gen);
        int neighbors = 0;
        
        for (size_t j = 0; j < n; ++j) {
            if (i != j) {
                float dx = cloud.points[i].x - cloud.points[j].x;
                float dy = cloud.points[i].y - cloud.points[j].y;
                float dz = cloud.points[i].z - cloud.points[j].z;
                float dist_sq = dx * dx + dy * dy + dz * dz;
                
                if (dist_sq < radius_sq) {
                    ++neighbors;
                }
            }
        }
        
        total_neighbors += static_cast<float>(neighbors);
    }
    
    float avg_neighbors = total_neighbors / sample_count;
    float volume = (4.0f / 3.0f) * static_cast<float>(M_PI) * radius * radius * radius;
    
    return avg_neighbors / volume;
}

// 点云合并
ErrorCode merge_pointclouds(const PointCloudData& cloud1, 
                            const PointCloudData& cloud2, 
                            PointCloudData& result) {
    if (cloud1.empty() && cloud2.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.clear();
    
    // 添加cloud1的点
    for (const auto& pt : cloud1.points) {
        result.add_point(pt);
    }
    for (const auto& c : cloud1.colors) {
        result.colors.push_back(c);
    }
    for (const auto& i : cloud1.intensities) {
        result.intensities.push_back(i);
    }
    
    // 添加cloud2的点
    for (const auto& pt : cloud2.points) {
        result.add_point(pt);
    }
    for (const auto& c : cloud2.colors) {
        result.colors.push_back(c);
    }
    for (const auto& i : cloud2.intensities) {
        result.intensities.push_back(i);
    }
    
    return ErrorCode::Success;
}

// 点云裁剪
ErrorCode crop_pointcloud(PointCloudData& cloud, 
                          const Point3Df& min_pt, 
                          const Point3Df& max_pt) {
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    PointCloudData cropped;
    
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& pt = cloud.points[i];
        
        if (pt.x >= min_pt.x && pt.x <= max_pt.x &&
            pt.y >= min_pt.y && pt.y <= max_pt.y &&
            pt.z >= min_pt.z && pt.z <= max_pt.z) {
            
            cropped.add_point(pt);
            
            if (cloud.has_colors()) {
                cropped.colors.push_back(cloud.colors[i]);
            }
            if (cloud.has_intensities()) {
                cropped.intensities.push_back(cloud.intensities[i]);
            }
        }
    }
    
    cloud = std::move(cropped);
    return ErrorCode::Success;
}

// 点云采样
ErrorCode sample_pointcloud(PointCloudData& cloud, float sample_ratio) {
    if (cloud.empty() || sample_ratio <= 0 || sample_ratio >= 1) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t n = cloud.size();
    size_t sample_size = static_cast<size_t>(n * sample_ratio);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, n - 1);
    
    Vector<bool> selected(n, false);
    PointCloudData sampled;
    
    while (sampled.size() < sample_size) {
        size_t idx = dis(gen);
        if (!selected[idx]) {
            selected[idx] = true;
            sampled.add_point(cloud.points[idx]);
            
            if (cloud.has_colors()) {
                sampled.colors.push_back(cloud.colors[idx]);
            }
            if (cloud.has_intensities()) {
                sampled.intensities.push_back(cloud.intensities[idx]);
            }
        }
    }
    
    cloud = std::move(sampled);
    return ErrorCode::Success;
}

} // namespace pointcloud_utils

// ============================================================================
// 节点实现
// ============================================================================

// DepthToPointCloudNode
DepthToPointCloudNode::DepthToPointCloudNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DepthToPointCloudNode::make_info() {
    NodeInfo info;
    info.id = "depth_to_pointcloud";
    info.name = "深度图转点云";
    info.category = "3D处理";
    info.description = "将深度图像转换为3D点云数据";
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

Result<void> DepthToPointCloudNode::execute(FlowContext& context) {
    auto depth_input = get_input("depth_image");
    if (!depth_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少深度图输入");
    }
    
    // 获取参数
    depth_scale_ = static_cast<float>(get_param("depth_scale", Data(1.0f)).as_number());
    focal_length_x_ = static_cast<float>(get_param("focal_length_x", Data(0.0f)).as_number());
    focal_length_y_ = static_cast<float>(get_param("focal_length_y", Data(0.0f)).as_number());
    center_x_ = static_cast<float>(get_param("center_x", Data(0.0f)).as_number());
    center_y_ = static_cast<float>(get_param("center_y", Data(0.0f)).as_number());
    
    // 构建深度图像数据
    DepthImageData depth_img;
    depth_img.depth = depth_input.as_image();
    depth_img.depth_scale = depth_scale_;
    depth_img.focal_length_x = focal_length_x_;
    depth_img.focal_length_y = focal_length_y_;
    depth_img.center_x = center_x_;
    depth_img.center_y = center_y_;
    
    // 检查是否有彩色图输入
    auto color_input = get_input("color_image");
    if (color_input.is_valid()) {
        depth_img.color = color_input.as_image();
    }
    
    // 转换为点云
    PointCloudData cloud;
    ErrorCode err = pointcloud_utils::depth_to_pointcloud(depth_img, cloud);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "深度图转点云失败");
    }
    
    // 设置输出
    set_output("pointcloud", Data(cloud));
    
    return Result<void>::success();
}

// PlaneDetectionNode
PlaneDetectionNode::PlaneDetectionNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PlaneDetectionNode::make_info() {
    NodeInfo info;
    info.id = "plane_detection";
    info.name = "平面检测";
    info.category = "3D处理";
    info.description = "检测点云中的平面";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("plane_center", "平面中心", DataType::Point);
    info.outputs.emplace_back("plane_normal", "平面法向量", DataType::Point);
    info.outputs.emplace_back("fit_error", "拟合误差", DataType::Number);
    
    info.params.emplace_back("distance_threshold", "距离阈值", DataType::Number, Data(0.01f));
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(100));
    
    return info;
}

Result<void> PlaneDetectionNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    distance_threshold_ = static_cast<float>(get_param("distance_threshold", Data(0.01f)).as_number());
    max_iterations_ = get_param("max_iterations", Data(100)).as_int();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    Plane3D plane;
    ErrorCode err = pointcloud_utils::find_plane(cloud, plane, distance_threshold_, max_iterations_);
    
    if (err != ErrorCode::Success || !plane.valid) {
        return Result<void>::failure(err, "平面检测失败");
    }
    
    set_output("plane_center", Data(plane.center));
    set_output("plane_normal", Data(plane.normal));
    set_output("fit_error", Data(plane.fit_error));
    
    return Result<void>::success();
}

// DepthMeasureNode
DepthMeasureNode::DepthMeasureNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DepthMeasureNode::make_info() {
    NodeInfo info;
    info.id = "depth_measure";
    info.name = "深度测量";
    info.category = "3D处理";
    info.description = "基于深度图测量两点间的3D距离";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("depth_image", "深度图", DataType::DepthImage, true);
    info.inputs.emplace_back("point1", "点1", DataType::Point, true);
    info.inputs.emplace_back("point2", "点2", DataType::Point, true);
    
    info.outputs.emplace_back("distance_3d", "3D距离", DataType::Number);
    
    info.params.emplace_back("depth_scale", "深度缩放因子", DataType::Number, Data(1.0f));
    info.params.emplace_back("focal_length_x", "焦距X", DataType::Number, Data(0.0f));
    info.params.emplace_back("focal_length_y", "焦距Y", DataType::Number, Data(0.0f));
    info.params.emplace_back("center_x", "光心X", DataType::Number, Data(0.0f));
    info.params.emplace_back("center_y", "光心Y", DataType::Number, Data(0.0f));
    
    return info;
}

Result<void> DepthMeasureNode::execute(FlowContext& context) {
    auto depth_input = get_input("depth_image");
    auto pt1_input = get_input("point1");
    auto pt2_input = get_input("point2");
    
    if (!depth_input.is_valid() || !pt1_input.is_valid() || !pt2_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少输入数据");
    }
    
    // 获取参数
    depth_scale_ = static_cast<float>(get_param("depth_scale", Data(1.0f)).as_number());
    focal_length_x_ = static_cast<float>(get_param("focal_length_x", Data(0.0f)).as_number());
    focal_length_y_ = static_cast<float>(get_param("focal_length_y", Data(0.0f)).as_number());
    center_x_ = static_cast<float>(get_param("center_x", Data(0.0f)).as_number());
    center_y_ = static_cast<float>(get_param("center_y", Data(0.0f)).as_number());
    
    // 构建深度图像数据
    DepthImageData depth_img;
    depth_img.depth = depth_input.as_depth_image().depth;
    depth_img.depth_scale = depth_scale_;
    depth_img.focal_length_x = focal_length_x_;
    depth_img.focal_length_y = focal_length_y_;
    depth_img.center_x = center_x_;
    depth_img.center_y = center_y_;
    
    // 获取两点（图像坐标）
    Point3Df pt1_img = pt1_input.as_point();
    Point3Df pt2_img = pt2_input.as_point();
    
    // 获取深度值并转换为3D点
    float z1 = depth_img.get_depth(static_cast<int>(pt1_img.x), static_cast<int>(pt1_img.y));
    float z2 = depth_img.get_depth(static_cast<int>(pt2_img.x), static_cast<int>(pt2_img.y));
    
    float fx = focal_length_x_ > 0 ? focal_length_x_ : depth_img.depth.width / 2.0f;
    float fy = focal_length_y_ > 0 ? focal_length_y_ : depth_img.depth.height / 2.0f;
    float cx = center_x_ > 0 ? center_x_ : depth_img.depth.width / 2.0f;
    float cy = center_y_ > 0 ? center_y_ : depth_img.depth.height / 2.0f;
    
    Point3Df p1_3d((pt1_img.x - cx) * z1 / fx, (pt1_img.y - cy) * z1 / fy, z1);
    Point3Df p2_3d((pt2_img.x - cx) * z2 / fx, (pt2_img.y - cy) * z2 / fy, z2);
    
    // 计算3D距离
    float distance = pointcloud_utils::compute_distance(p1_3d, p2_3d);
    
    set_output("distance_3d", Data(distance));
    
    return Result<void>::success();
}

// PointCloudVisualizeNode
PointCloudVisualizeNode::PointCloudVisualizeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudVisualizeNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_visualize";
    info.name = "点云可视化";
    info.category = "3D处理";
    info.description = "将点云渲染为2D图像";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("visualization_image", "可视化图像", DataType::Image);
    
    info.params.emplace_back("image_width", "图像宽度", DataType::Number, Data(800));
    info.params.emplace_back("image_height", "图像高度", DataType::Number, Data(600));
    info.params.emplace_back("view_distance", "视点距离", DataType::Number, Data(500.0f));
    info.params.emplace_back("rotation_x", "旋转X(度)", DataType::Number, Data(30.0f));
    info.params.emplace_back("rotation_y", "旋转Y(度)", DataType::Number, Data(45.0f));
    info.params.emplace_back("rotation_z", "旋转Z(度)", DataType::Number, Data(0.0f));
    info.params.emplace_back("zoom", "缩放", DataType::Number, Data(1.0f));
    info.params.emplace_back("show_colors", "显示颜色", DataType::Boolean, Data(true));
    
    return info;
}

Result<void> PointCloudVisualizeNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    // 获取参数
    image_width_ = get_param("image_width", Data(800)).as_int();
    image_height_ = get_param("image_height", Data(600)).as_int();
    view_distance_ = static_cast<float>(get_param("view_distance", Data(500.0f)).as_number());
    rotation_x_ = static_cast<float>(get_param("rotation_x", Data(30.0f)).as_number());
    rotation_y_ = static_cast<float>(get_param("rotation_y", Data(45.0f)).as_number());
    rotation_z_ = static_cast<float>(get_param("rotation_z", Data(0.0f)).as_number());
    zoom_ = static_cast<float>(get_param("zoom", Data(1.0f)).as_number());
    show_colors_ = get_param("show_colors", Data(true)).as_bool();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    if (cloud.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "点云为空");
    }
    
    // 计算包围盒
    Point3Df min_pt, max_pt;
    pointcloud_utils::compute_bounding_box(cloud, min_pt, max_pt);
    
    Point3Df center = pointcloud_utils::compute_centroid(cloud);
    
    // 创建输出图像
    ImageData output;
    output.width = image_width_;
    output.height = image_height_;
    output.channels = 3;
    output.format = ImageFormat::RGB8;
    output.data.resize(image_width_ * image_height_ * 3, 0);  // 黑色背景
    
    // 设置变换矩阵
    Transform3D transform;
    transform.set_rotation_x(rotation_x_ * static_cast<float>(M_PI) / 180.0f);
    transform.set_rotation_y(rotation_y_ * static_cast<float>(M_PI) / 180.0f);
    transform.set_rotation_z(rotation_z_ * static_cast<float>(M_PI) / 180.0f);
    transform.set_translation(-center.x, -center.y, -center.z);
    
    // 计算缩放因子
    float max_extent = std::max(max_pt.x - min_pt.x, std::max(max_pt.y - min_pt.y, max_pt.z - min_pt.z));
    float scale = (std::min(image_width_, image_height_) * 0.8f * zoom_) / (max_extent + view_distance_);
    
    // 投影每个点
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        Point3Df transformed = transform.transform(cloud.points[i]);
        
        // 简单投影（忽略透视）
        int px = static_cast<int>((transformed.x * scale + view_distance_) * image_width_ / (2.0f * view_distance_));
        int py = static_cast<int>((transformed.y * scale + view_distance_) * image_height_ / (2.0f * view_distance_));
        
        px = image_width_ / 2 + px;
        py = image_height_ / 2 - py;  // Y轴翻转
        
        if (px >= 0 && px < image_width_ && py >= 0 && py < image_height_) {
            size_t idx = (py * image_width_ + px) * 3;
            
            ColorRGB color(255, 255, 255);  // 默认白色
            if (show_colors_ && cloud.has_colors()) {
                color = cloud.colors[i];
            } else {
                // 根据深度设置灰度
                float depth_ratio = (transformed.z - min_pt.z) / (max_pt.z - min_pt.z + 1.0f);
                uint8_t gray = static_cast<uint8_t>(255 * depth_ratio);
                color = ColorRGB(gray, gray, gray);
            }
            
            output.data[idx] = color.r;
            output.data[idx + 1] = color.g;
            output.data[idx + 2] = color.b;
        }
    }
    
    set_output("visualization_image", Data(std::move(output)));
    
    return Result<void>::success();
}

// PointCloudTransformNode
PointCloudTransformNode::PointCloudTransformNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudTransformNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_transform";
    info.name = "点云变换";
    info.category = "3D处理";
    info.description = "对点云进行旋转和平移变换";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_pointcloud", "变换后点云", DataType::PointCloud);
    
    info.params.emplace_back("tx", "平移X", DataType::Number, Data(0.0f));
    info.params.emplace_back("ty", "平移Y", DataType::Number, Data(0.0f));
    info.params.emplace_back("tz", "平移Z", DataType::Number, Data(0.0f));
    info.params.emplace_back("rx", "旋转X(弧度)", DataType::Number, Data(0.0f));
    info.params.emplace_back("ry", "旋转Y(弧度)", DataType::Number, Data(0.0f));
    info.params.emplace_back("rz", "旋转Z(弧度)", DataType::Number, Data(0.0f));
    
    return info;
}

Result<void> PointCloudTransformNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    tx_ = static_cast<float>(get_param("tx", Data(0.0f)).as_number());
    ty_ = static_cast<float>(get_param("ty", Data(0.0f)).as_number());
    tz_ = static_cast<float>(get_param("tz", Data(0.0f)).as_number());
    rx_ = static_cast<float>(get_param("rx", Data(0.0f)).as_number());
    ry_ = static_cast<float>(get_param("ry", Data(0.0f)).as_number());
    rz_ = static_cast<float>(get_param("rz", Data(0.0f)).as_number());
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    ErrorCode err = pointcloud_utils::rotate_pointcloud(cloud, rx_, ry_, rz_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云旋转失败");
    }
    
    err = pointcloud_utils::translate_pointcloud(cloud, tx_, ty_, tz_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云平移失败");
    }
    
    set_output("transformed_pointcloud", Data(cloud));
    
    return Result<void>::success();
}

// PointCloudClusterNode
PointCloudClusterNode::PointCloudClusterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudClusterNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_cluster";
    info.name = "点云聚类";
    info.category = "3D处理";
    info.description = "对点云进行聚类分割";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("clusters", "聚类结果", DataType::Array);
    
    info.params.emplace_back("tolerance", "聚类容差", DataType::Number, Data(0.02f));
    info.params.emplace_back("min_cluster_size", "最小聚类大小", DataType::Number, Data(10));
    info.params.emplace_back("max_cluster_size", "最大聚类大小", DataType::Number, Data(100000));
    
    return info;
}

Result<void> PointCloudClusterNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    tolerance_ = static_cast<float>(get_param("tolerance", Data(0.02f)).as_number());
    min_cluster_size_ = get_param("min_cluster_size", Data(10)).as_int();
    max_cluster_size_ = get_param("max_cluster_size", Data(100000)).as_int();
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    Vector<PointCloudData> clusters;
    ErrorCode err = pointcloud_utils::find_clusters(cloud, clusters, tolerance_, 
                                                    min_cluster_size_, max_cluster_size_);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云聚类失败");
    }
    
    // TODO: 需要将clusters数组正确设置为输出
    // 目前简化为返回聚类数量
    set_output("clusters", Data(static_cast<int>(clusters.size())));
    
    return Result<void>::success();
}

// PointCloudBoundingBoxNode
PointCloudBoundingBoxNode::PointCloudBoundingBoxNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudBoundingBoxNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_boundingbox";
    info.name = "点云包围盒";
    info.category = "3D处理";
    info.description = "计算点云的3D包围盒";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("min_point", "最小点", DataType::Point);
    info.outputs.emplace_back("max_point", "最大点", DataType::Point);
    info.outputs.emplace_back("center", "中心点", DataType::Point);
    info.outputs.emplace_back("size", "尺寸", DataType::Point);
    
    return info;
}

Result<void> PointCloudBoundingBoxNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    Point3Df min_pt, max_pt;
    ErrorCode err = pointcloud_utils::compute_bounding_box(cloud, min_pt, max_pt);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "计算包围盒失败");
    }
    
    Point3Df center(
        (min_pt.x + max_pt.x) * 0.5f,
        (min_pt.y + max_pt.y) * 0.5f,
        (min_pt.z + max_pt.z) * 0.5f
    );
    
    Point3Df size(
        max_pt.x - min_pt.x,
        max_pt.y - min_pt.y,
        max_pt.z - min_pt.z
    );
    
    set_output("min_point", Data(min_pt));
    set_output("max_point", Data(max_pt));
    set_output("center", Data(center));
    set_output("size", Data(size));
    
    return Result<void>::success();
}

// 节点注册
OVF_REGISTER_NODE(DepthToPointCloudNode, "depth_to_pointcloud", DepthToPointCloudNode::make_info())
OVF_REGISTER_NODE(PlaneDetectionNode, "plane_detection", PlaneDetectionNode::make_info())
OVF_REGISTER_NODE(DepthMeasureNode, "depth_measure", DepthMeasureNode::make_info())
OVF_REGISTER_NODE(PointCloudVisualizeNode, "pointcloud_visualize", PointCloudVisualizeNode::make_info())
OVF_REGISTER_NODE(PointCloudTransformNode, "pointcloud_transform", PointCloudTransformNode::make_info())
OVF_REGISTER_NODE(PointCloudClusterNode, "pointcloud_cluster", PointCloudClusterNode::make_info())
OVF_REGISTER_NODE(PointCloudBoundingBoxNode, "pointcloud_boundingbox", PointCloudBoundingBoxNode::make_info())

} // namespace algorithm
} // namespace ovf