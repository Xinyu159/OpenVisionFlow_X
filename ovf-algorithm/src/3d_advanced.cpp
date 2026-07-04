/**
 * @file 3d_advanced.cpp
 * @brief 高级3D算法模块实现 - 点云配准、拼接、网格重建和处理
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#include "ovf/algorithm/3d_advanced.h"
#include "ovf/core/logger.h"
#include <algorithm>
#include <random>
#include <cmath>
#include <limits>
#include <queue>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {
namespace advanced_3d_utils {

// ============================================================================
// 辅助函数
// ============================================================================

// 找最近点
static size_t find_nearest_point(const Point3Df& pt, const PointCloudData& cloud, float max_dist_sq) {
    size_t best_idx = -1;
    float best_dist_sq = max_dist_sq;
    
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        float dx = pt.x - cloud.points[i].x;
        float dy = pt.y - cloud.points[i].y;
        float dz = pt.z - cloud.points[i].z;
        float dist_sq = dx * dx + dy * dy + dz * dz;
        
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    
    return best_idx;
}

// 找半径内的所有点
static Vector<size_t> find_points_in_radius(const Point3Df& pt, const PointCloudData& cloud, float radius) {
    Vector<size_t> indices;
    float radius_sq = radius * radius;
    
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        float dx = pt.x - cloud.points[i].x;
        float dy = pt.y - cloud.points[i].y;
        float dz = pt.z - cloud.points[i].z;
        float dist_sq = dx * dx + dy * dy + dz * dz;
        
        if (dist_sq < radius_sq) {
            indices.push_back(i);
        }
    }
    
    return indices;
}

// 计算变换矩阵的逆
static Transform3D inverse_transform(const Transform3D& t) {
    Transform3D inv;
    
    // 提取旋转矩阵R和平移向量T
    // 变换矩阵: [R|T; 0|1]
    // 逆矩阵: [R^T|-R^T*T; 0|1]
    
    // 计算R^T
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            inv.m[i][j] = t.m[j][i];
        }
    }
    
    // 计算-R^T*T
    float tx = t.m[0][3];
    float ty = t.m[1][3];
    float tz = t.m[2][3];
    
    inv.m[0][3] = -(inv.m[0][0] * tx + inv.m[0][1] * ty + inv.m[0][2] * tz);
    inv.m[1][3] = -(inv.m[1][0] * tx + inv.m[1][1] * ty + inv.m[1][2] * tz);
    inv.m[2][3] = -(inv.m[2][0] * tx + inv.m[2][1] * ty + inv.m[2][2] * tz);
    
    // 最后一行
    inv.m[3][0] = 0;
    inv.m[3][1] = 0;
    inv.m[3][2] = 0;
    inv.m[3][3] = 1;
    
    return inv;
}

// 应用变换到点云
static void transform_cloud(PointCloudData& cloud, const Transform3D& transform) {
    for (auto& pt : cloud.points) {
        pt = transform.transform(pt);
    }
}

// ============================================================================
// 点云配准算法实现
// ============================================================================

// ICP配准 - 迭代最近点算法
ErrorCode icp_register(const PointCloudData& source,
                       const PointCloudData& target,
                       const Transform3D& initial_transform,
                       int max_iterations,
                       float tolerance,
                       ICPResult& result) {
    
    if (source.empty() || target.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 初始化变换矩阵
    result.transformation = initial_transform;
    result.converged = false;
    result.iterations = 0;
    result.fitness_score = std::numeric_limits<float>::max();
    
    // 复制源点云并应用初始变换
    PointCloudData transformed_source = source;
    transform_cloud(transformed_source, initial_transform);
    
    float prev_error = std::numeric_limits<float>::max();
    float max_dist_sq = 0.1f;  // 最大对应点距离
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations = iter + 1;
        
        // 找对应点
        Vector<Point3Df> source_matched;
        Vector<Point3Df> target_matched;
        
        float total_error = 0.0f;
        int match_count = 0;
        
        for (const auto& src_pt : transformed_source.points) {
            size_t nearest_idx = find_nearest_point(src_pt, target, max_dist_sq);
            if (nearest_idx != static_cast<size_t>(-1)) {
                source_matched.push_back(src_pt);
                target_matched.push_back(target.points[nearest_idx]);
                
                float dx = src_pt.x - target.points[nearest_idx].x;
                float dy = src_pt.y - target.points[nearest_idx].y;
                float dz = src_pt.z - target.points[nearest_idx].z;
                total_error += dx * dx + dy * dy + dz * dz;
                ++match_count;
            }
        }
        
        if (match_count < 3) {
            return ErrorCode::AlgorithmExecFailed;
        }
        
        float mean_error = total_error / match_count;
        result.fitness_score = mean_error;
        
        // 检查收敛
        if (std::abs(prev_error - mean_error) < tolerance) {
            result.converged = true;
            break;
        }
        
        prev_error = mean_error;
        
        // 计算新的变换（基于对应点的最小二乘）
        // 计算源和目标的重心
        Point3Df src_centroid(0, 0, 0);
        Point3Df tgt_centroid(0, 0, 0);
        
        for (const auto& pt : source_matched) {
            src_centroid.x += pt.x;
            src_centroid.y += pt.y;
            src_centroid.z += pt.z;
        }
        for (const auto& pt : target_matched) {
            tgt_centroid.x += pt.x;
            tgt_centroid.y += pt.y;
            tgt_centroid.z += pt.z;
        }
        
        float n = static_cast<float>(match_count);
        src_centroid.x /= n; src_centroid.y /= n; src_centroid.z /= n;
        tgt_centroid.x /= n; tgt_centroid.y /= n; tgt_centroid.z /= n;
        
        // 计算去中心化的点
        Vector<Point3Df> src_centered, tgt_centered;
        for (const auto& pt : source_matched) {
            src_centered.push_back(pt - src_centroid);
        }
        for (const auto& pt : target_matched) {
            tgt_centered.push_back(pt - tgt_centroid);
        }
        
        // 计算H矩阵（用于SVD求解旋转）
        float H[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        for (size_t i = 0; i < src_centered.size(); ++i) {
            H[0][0] += src_centered[i].x * tgt_centered[i].x;
            H[0][1] += src_centered[i].x * tgt_centered[i].y;
            H[0][2] += src_centered[i].x * tgt_centered[i].z;
            H[1][0] += src_centered[i].y * tgt_centered[i].x;
            H[1][1] += src_centered[i].y * tgt_centered[i].y;
            H[1][2] += src_centered[i].y * tgt_centered[i].z;
            H[2][0] += src_centered[i].z * tgt_centered[i].x;
            H[2][1] += src_centered[i].z * tgt_centered[i].y;
            H[2][2] += src_centered[i].z * tgt_centered[i].z;
        }
        
        // 简化的旋转矩阵计算（使用Kabsch算法近似）
        // 这里使用简化的方法计算旋转矩阵
        Transform3D delta_transform;
        
        // 基于特征值分解近似计算旋转（简化实现）
        // 计算迹来判断是否有镜像
        float det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) -
                   H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
                   H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);
        
        if (det < 0) {
            // 需要镜像校正
            H[2][0] = -H[2][0];
            H[2][1] = -H[2][1];
            H[2][2] = -H[2][2];
        }
        
        // 计算旋转矩阵（简化：使用极分解近似）
        // R ≈ H * (H^T * H)^{-0.5}
        // 简化实现：直接归一化H的列
        float col0_len = std::sqrt(H[0][0] * H[0][0] + H[1][0] * H[1][0] + H[2][0] * H[2][0]);
        float col1_len = std::sqrt(H[0][1] * H[0][1] + H[1][1] * H[1][1] + H[2][1] * H[2][1]);
        float col2_len = std::sqrt(H[0][2] * H[0][2] + H[1][2] * H[1][2] + H[2][2] * H[2][2]);
        
        if (col0_len > 1e-6f && col1_len > 1e-6f && col2_len > 1e-6f) {
            delta_transform.m[0][0] = H[0][0] / col0_len;
            delta_transform.m[1][0] = H[1][0] / col0_len;
            delta_transform.m[2][0] = H[2][0] / col0_len;
            
            delta_transform.m[0][1] = H[0][1] / col1_len;
            delta_transform.m[1][1] = H[1][1] / col1_len;
            delta_transform.m[2][1] = H[2][1] / col1_len;
            
            delta_transform.m[0][2] = H[0][2] / col2_len;
            delta_transform.m[1][2] = H[1][2] / col2_len;
            delta_transform.m[2][2] = H[2][2] / col2_len;
        }
        
        // 计算平移
        delta_transform.m[0][3] = tgt_centroid.x - (delta_transform.m[0][0] * src_centroid.x +
                                                    delta_transform.m[0][1] * src_centroid.y +
                                                    delta_transform.m[0][2] * src_centroid.z);
        delta_transform.m[1][3] = tgt_centroid.y - (delta_transform.m[1][0] * src_centroid.x +
                                                    delta_transform.m[1][1] * src_centroid.y +
                                                    delta_transform.m[1][2] * src_centroid.z);
        delta_transform.m[2][3] = tgt_centroid.z - (delta_transform.m[2][0] * src_centroid.x +
                                                    delta_transform.m[2][1] * src_centroid.y +
                                                    delta_transform.m[2][2] * src_centroid.z);
        
        // 更新变换矩阵
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += delta_transform.m[i][k] * result.transformation.m[k][j];
                }
                result.transformation.m[i][j] = sum;
            }
        }
        
        // 更新变换后的源点云
        transformed_source = source;
        transform_cloud(transformed_source, result.transformation);
    }
    
    return ErrorCode::Success;
}

// 特征配准（简化实现：基于关键点匹配）
ErrorCode feature_register(const PointCloudData& source,
                           const PointCloudData& target,
                           ICPResult& result) {
    
    if (source.empty() || target.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 简化实现：使用几何特征（基于采样点的距离和角度）
    // 1. 从源和目标点云中选择特征点
    size_t n_source = source.size();
    size_t n_target = target.size();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis_src(0, n_source - 1);
    std::uniform_int_distribution<size_t> dis_tgt(0, n_target - 1);
    
    // 选择一些采样点作为特征点
    int num_features = std::min(100, static_cast<int>(std::min(n_source, n_target)));
    
    Vector<Point3Df> src_features, tgt_features;
    for (int i = 0; i < num_features; ++i) {
        src_features.push_back(source.points[dis_src(gen)]);
        tgt_features.push_back(target.points[dis_tgt(gen)]);
    }
    
    // 计算特征描述符（简化的距离特征）
    // 构建距离矩阵用于匹配
    Vector<float> src_distances;
    for (size_t i = 0; i < src_features.size(); ++i) {
        for (size_t j = i + 1; j < src_features.size(); ++j) {
            src_distances.push_back(src_features[i].distance_to(src_features[j]));
        }
    }
    
    Vector<float> tgt_distances;
    for (size_t i = 0; i < tgt_features.size(); ++i) {
        for (size_t j = i + 1; j < tgt_features.size(); ++j) {
            tgt_distances.push_back(tgt_features[i].distance_to(tgt_features[j]));
        }
    }
    
    // 基于特征匹配初始化变换（简化：使用点云重心对齐）
    Point3Df src_centroid = pointcloud_utils::compute_centroid(source);
    Point3Df tgt_centroid = pointcloud_utils::compute_centroid(target);
    
    // 初始变换：将源点云移动到目标点云中心
    Transform3D initial;
    initial.set_translation(tgt_centroid.x - src_centroid.x,
                            tgt_centroid.y - src_centroid.y,
                            tgt_centroid.z - src_centroid.z);
    
    // 然后使用ICP进行精细配准
    return icp_register(source, target, initial, 30, 0.001f, result);
}

// NDT配准（正态分布变换）
ErrorCode ndt_register(const PointCloudData& source,
                       const PointCloudData& target,
                       float voxel_size,
                       float step_size,
                       int max_iterations,
                       ICPResult& result) {
    
    if (source.empty() || target.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 初始化结果
    result.transformation = Transform3D();  // 单位矩阵
    result.converged = false;
    result.iterations = 0;
    result.fitness_score = std::numeric_limits<float>::max();
    
    // 构建目标点云的NDT体素
    struct NDTVoxel {
        Point3Df mean;
        float cov[3][3];  // 协方差矩阵
        int point_count;
        bool valid;
    };
    
    // 体素索引哈希
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
    
    std::unordered_map<VoxelKey, NDTVoxel, VoxelKeyHash> target_voxels;
    
    // 将目标点云分配到体素
    for (const auto& pt : target.points) {
        VoxelKey key;
        key.x = static_cast<int>(std::floor(pt.x / voxel_size));
        key.y = static_cast<int>(std::floor(pt.y / voxel_size));
        key.z = static_cast<int>(std::floor(pt.z / voxel_size));
        
        auto& voxel = target_voxels[key];
        voxel.mean.x += pt.x;
        voxel.mean.y += pt.y;
        voxel.mean.z += pt.z;
        voxel.point_count++;
    }
    
    // 计算每个体素的均值和协方差
    for (auto& pair : target_voxels) {
        auto& voxel = pair.second;
        if (voxel.point_count > 5) {
            float inv_n = 1.0f / voxel.point_count;
            voxel.mean.x *= inv_n;
            voxel.mean.y *= inv_n;
            voxel.mean.z *= inv_n;
            
            // 计算协方差
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    voxel.cov[i][j] = 0.0f;
                }
            }
            
            VoxelKey key = pair.first;
            // 重新收集该体素中的点计算协方差
            for (const auto& pt : target.points) {
                VoxelKey pt_key;
                pt_key.x = static_cast<int>(std::floor(pt.x / voxel_size));
                pt_key.y = static_cast<int>(std::floor(pt.y / voxel_size));
                pt_key.z = static_cast<int>(std::floor(pt.z / voxel_size));
                
                if (pt_key == key) {
                    float dx = pt.x - voxel.mean.x;
                    float dy = pt.y - voxel.mean.y;
                    float dz = pt.z - voxel.mean.z;
                    
                    voxel.cov[0][0] += dx * dx;
                    voxel.cov[0][1] += dx * dy;
                    voxel.cov[0][2] += dx * dz;
                    voxel.cov[1][0] += dy * dx;
                    voxel.cov[1][1] += dy * dy;
                    voxel.cov[1][2] += dy * dz;
                    voxel.cov[2][0] += dz * dx;
                    voxel.cov[2][1] += dz * dy;
                    voxel.cov[2][2] += dz * dz;
                }
            }
            
            // 归一化协方差
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    voxel.cov[i][j] *= inv_n;
                }
            }
            
            voxel.valid = true;
        } else {
            voxel.valid = false;
        }
    }
    
    // NDT优化迭代
    PointCloudData transformed_source = source;
    float prev_score = std::numeric_limits<float>::max();
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations = iter + 1;
        
        // 计算得分（概率密度之和）
        float total_score = 0.0f;
        int valid_count = 0;
        
        Vector<Point3Df> gradient_points;
        Vector<Point3Df> gradient_dirs;
        
        for (const auto& pt : transformed_source.points) {
            VoxelKey key;
            key.x = static_cast<int>(std::floor(pt.x / voxel_size));
            key.y = static_cast<int>(std::floor(pt.y / voxel_size));
            key.z = static_cast<int>(std::floor(pt.z / voxel_size));
            
            auto it = target_voxels.find(key);
            if (it != target_voxels.end() && it->second.valid) {
                const auto& voxel = it->second;
                
                // 计算概率密度（简化的高斯分布）
                float dx = pt.x - voxel.mean.x;
                float dy = pt.y - voxel.mean.y;
                float dz = pt.z - voxel.mean.z;
                
                // 简化的得分：距离平方的负值
                float score = -(dx * dx + dy * dy + dz * dz);
                total_score += score;
                ++valid_count;
                
                // 计算梯度方向
                gradient_points.push_back(pt);
                gradient_dirs.push_back(Point3Df(-dx, -dy, -dz));
            }
        }
        
        if (valid_count > 0) {
            result.fitness_score = total_score / valid_count;
        }
        
        // 检查收敛
        if (std::abs(prev_score - result.fitness_score) < 0.0001f) {
            result.converged = true;
            break;
        }
        
        prev_score = result.fitness_score;
        
        // 更新变换（基于梯度方向）
        if (!gradient_dirs.empty()) {
            // 计算平均梯度方向作为移动方向
            Point3Df avg_gradient(0, 0, 0);
            for (const auto& g : gradient_dirs) {
                avg_gradient.x += g.x;
                avg_gradient.y += g.y;
                avg_gradient.z += g.z;
            }
            
            float n = static_cast<float>(gradient_dirs.size());
            avg_gradient.x /= n;
            avg_gradient.y /= n;
            avg_gradient.z /= n;
            
            // 归一化并应用步长
            float len = avg_gradient.length();
            if (len > 1e-6f) {
                avg_gradient = avg_gradient * (step_size / len);
                
                // 更新变换的平移部分
                result.transformation.m[0][3] += avg_gradient.x;
                result.transformation.m[1][3] += avg_gradient.y;
                result.transformation.m[2][3] += avg_gradient.z;
            }
        }
        
        // 重新变换源点云
        transformed_source = source;
        transform_cloud(transformed_source, result.transformation);
    }
    
    return ErrorCode::Success;
}

// 全局配准（粗配准）
ErrorCode global_register(const PointCloudData& source,
                          const PointCloudData& target,
                          ICPResult& result) {
    
    if (source.empty() || target.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 使用RANSAC进行全局配准
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis_src(0, source.size() - 1);
    std::uniform_int_distribution<size_t> dis_tgt(0, target.size() - 1);
    
    int max_iterations = 1000;
    float best_score = std::numeric_limits<float>::max();
    Transform3D best_transform;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择3个点对
        size_t i0_src = dis_src(gen), i1_src = dis_src(gen), i2_src = dis_src(gen);
        size_t i0_tgt = dis_tgt(gen), i1_tgt = dis_tgt(gen), i2_tgt = dis_tgt(gen);
        
        // 确保选择的点不相同
        while (i1_src == i0_src) i1_src = dis_src(gen);
        while (i2_src == i0_src || i2_src == i1_src) i2_src = dis_src(gen);
        while (i1_tgt == i0_tgt) i1_tgt = dis_tgt(gen);
        while (i2_tgt == i0_tgt || i2_tgt == i1_tgt) i2_tgt = dis_tgt(gen);
        
        // 计算变换矩阵（将源点映射到目标点）
        const auto& s0 = source.points[i0_src];
        const auto& s1 = source.points[i1_src];
        const auto& s2 = source.points[i2_src];
        const auto& t0 = target.points[i0_tgt];
        const auto& t1 = target.points[i1_tgt];
        const auto& t2 = target.points[i2_tgt];
        
        // 计算源和目标的局部坐标系
        Point3Df src_origin = s0;
        Point3Df src_x = (s1 - s0).normalized();
        Point3Df src_y = ((s2 - s0) - src_x * ((s2 - s0).dot(src_x))).normalized();
        Point3Df src_z = src_x.cross(src_y);
        
        Point3Df tgt_origin = t0;
        Point3Df tgt_x = (t1 - t0).normalized();
        Point3Df tgt_y = ((t2 - t0) - tgt_x * ((t2 - t0).dot(tgt_x))).normalized();
        Point3Df tgt_z = tgt_x.cross(tgt_y);
        
        // 构建变换矩阵
        Transform3D transform;
        for (int i = 0; i < 3; ++i) {
            transform.m[i][0] = tgt_x.x * src_x.x + tgt_y.x * src_y.x + tgt_z.x * src_z.x;
            transform.m[i][1] = tgt_x.y * src_x.y + tgt_y.y * src_y.y + tgt_z.y * src_z.y;
            transform.m[i][2] = tgt_x.z * src_x.z + tgt_y.z * src_y.z + tgt_z.z * src_z.z;
        }
        
        // 计算平移
        Point3Df src_center = pointcloud_utils::compute_centroid(source);
        Point3Df tgt_center = pointcloud_utils::compute_centroid(target);
        transform.m[0][3] = tgt_center.x - src_center.x;
        transform.m[1][3] = tgt_center.y - src_center.y;
        transform.m[2][3] = tgt_center.z - src_center.z;
        
        // 计算得分
        PointCloudData transformed = source;
        transform_cloud(transformed, transform);
        
        float score = 0.0f;
        int match_count = 0;
        float max_dist_sq = 0.05f * 0.05f;
        
        for (const auto& pt : transformed.points) {
            size_t nearest = find_nearest_point(pt, target, max_dist_sq);
            if (nearest != static_cast<size_t>(-1)) {
                float dx = pt.x - target.points[nearest].x;
                float dy = pt.y - target.points[nearest].y;
                float dz = pt.z - target.points[nearest].z;
                score += dx * dx + dy * dy + dz * dz;
                ++match_count;
            }
        }
        
        if (match_count > transformed.size() * 0.3f) {
            score /= match_count;
            if (score < best_score) {
                best_score = score;
                best_transform = transform;
            }
        }
    }
    
    result.transformation = best_transform;
    result.fitness_score = best_score;
    result.iterations = max_iterations;
    result.converged = (best_score < std::numeric_limits<float>::max());
    
    return ErrorCode::Success;
}

// ============================================================================
// 点云拼接算法实现
// ============================================================================

// 点云合并拼接
ErrorCode merge_pointclouds(const Vector<PointCloudData>& clouds,
                            PointCloudData& result) {
    
    if (clouds.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.clear();
    
    for (const auto& cloud : clouds) {
        for (const auto& pt : cloud.points) {
            result.add_point(pt);
        }
        
        if (cloud.has_colors()) {
            for (const auto& c : cloud.colors) {
                result.colors.push_back(c);
            }
        }
        
        if (cloud.has_intensities()) {
            for (const auto& i : cloud.intensities) {
                result.intensities.push_back(i);
            }
        }
    }
    
    return ErrorCode::Success;
}

// 多视角点云拼接
ErrorCode multiview_merge(const Vector<PointCloudData>& clouds,
                          const Vector<Transform3D>& transforms,
                          PointCloudData& result) {
    
    if (clouds.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    if (transforms.size() != clouds.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.clear();
    
    for (size_t i = 0; i < clouds.size(); ++i) {
        PointCloudData transformed = clouds[i];
        transform_cloud(transformed, transforms[i]);
        
        for (const auto& pt : transformed.points) {
            result.add_point(pt);
        }
        
        if (transformed.has_colors()) {
            for (const auto& c : transformed.colors) {
                result.colors.push_back(c);
            }
        }
        
        if (transformed.has_intensities()) {
            for (const auto& i : transformed.intensities) {
                result.intensities.push_back(i);
            }
        }
    }
    
    return ErrorCode::Success;
}

// 扫描线点云拼接
ErrorCode scan_merge(const Vector<PointCloudData>& scan_lines,
                     PointCloudData& result) {
    
    if (scan_lines.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.clear();
    
    // 简化实现：直接合并所有扫描线
    // 实际应用中应该考虑扫描线之间的对齐
    for (const auto& line : scan_lines) {
        for (const auto& pt : line.points) {
            result.add_point(pt);
        }
        
        if (line.has_colors()) {
            for (const auto& c : line.colors) {
                result.colors.push_back(c);
            }
        }
    }
    
    return ErrorCode::Success;
}

// ============================================================================
// 网格重建算法实现
// ============================================================================

// Poisson网格重建（简化实现）
ErrorCode poisson_reconstruct(const PointCloudData& cloud,
                              const Vector<Point3Df>& normals,
                              int depth,
                              MeshData& mesh) {
    
    if (cloud.empty() || normals.empty() || normals.size() != cloud.size()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 简化实现：使用体素网格和移动立方体近似
    // 实际Poisson重建需要复杂的八叉树和隐式表面构建
    
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
    
    // 扩展包围盒
    float padding = 0.1f;
    min_pt.x -= padding; min_pt.y -= padding; min_pt.z -= padding;
    max_pt.x += padding; max_pt.y += padding; max_pt.z += padding;
    
    // 根据深度计算体素网格分辨率
    int resolution = 1 << depth;  // 2^depth
    float voxel_size_x = (max_pt.x - min_pt.x) / resolution;
    float voxel_size_y = (max_pt.y - min_pt.y) / resolution;
    float voxel_size_z = (max_pt.z - min_pt.z) / resolution;
    float voxel_size = std::max(voxel_size_x, std::max(voxel_size_y, voxel_size_z));
    
    // 使用移动立方体方法生成网格
    return marching_cubes_reconstruct(cloud, voxel_size, mesh);
}

// Marching Cubes重建
ErrorCode marching_cubes_reconstruct(const PointCloudData& cloud,
                                     float voxel_size,
                                     MeshData& mesh) {
    
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    mesh.clear();
    
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
    
    // 扩展包围盒
    float padding = voxel_size * 2;
    min_pt.x -= padding; min_pt.y -= padding; min_pt.z -= padding;
    max_pt.x += padding; max_pt.y += padding; max_pt.z += padding;
    
    // 计算体素网格尺寸
    int nx = static_cast<int>(std::ceil((max_pt.x - min_pt.x) / voxel_size));
    int ny = static_cast<int>(std::ceil((max_pt.y - min_pt.y) / voxel_size));
    int nz = static_cast<int>(std::ceil((max_pt.z - min_pt.z) / voxel_size));
    
    // 创建距离场
    Vector<float> distance_field(nx * ny * nz, std::numeric_limits<float>::max());
    
    // 计算每个体素中心到最近点的距离
    for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                Point3Df voxel_center(
                    min_pt.x + (ix + 0.5f) * voxel_size,
                    min_pt.y + (iy + 0.5f) * voxel_size,
                    min_pt.z + (iz + 0.5f) * voxel_size
                );
                
                float min_dist = std::numeric_limits<float>::max();
                for (const auto& pt : cloud.points) {
                    float dist = voxel_center.distance_to(pt);
                    min_dist = std::min(min_dist, dist);
                }
                
                distance_field[iz * nx * ny + iy * nx + ix] = min_dist;
            }
        }
    }
    
    // Marching Cubes提取等值面
    float iso_value = voxel_size;  // 等值面阈值
    
    // 简化实现：直接使用点云的点作为网格顶点
    // 添加顶点
    for (const auto& pt : cloud.points) {
        mesh.add_vertex(pt);
    }
    
    // 简化：使用Delaunay风格构建三角形
    // 为相邻点构建三角形（基于最近邻）
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        Vector<size_t> neighbors = find_points_in_radius(cloud.points[i], cloud, voxel_size * 2);
        
        if (neighbors.size() >= 3) {
            // 简化：选择前3个邻居构建三角形
            mesh.add_triangle(static_cast<uint32_t>(i),
                             static_cast<uint32_t>(neighbors[0]),
                             static_cast<uint32_t>(neighbors[1]));
        }
    }
    
    return ErrorCode::Success;
}

// Delaunay三角化（简化实现）
ErrorCode delaunay_triangulation(const PointCloudData& cloud,
                                 MeshData& mesh) {
    
    if (cloud.size() < 4) {
        return ErrorCode::InvalidParameter;
    }
    
    mesh.clear();
    
    // 添加顶点
    for (const auto& pt : cloud.points) {
        mesh.add_vertex(pt);
    }
    
    // 简化实现：基于邻近点构建三角形
    // 计算平均点间距
    float avg_spacing = 0.0f;
    int sample_count = std::min(100, static_cast<int>(cloud.size()));
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, cloud.size() - 1);
    
    for (int s = 0; s < sample_count; ++s) {
        size_t i = dis(gen);
        size_t nearest = find_nearest_point(cloud.points[i], cloud, std::numeric_limits<float>::max());
        if (nearest != static_cast<size_t>(-1) && nearest != i) {
            avg_spacing += cloud.points[i].distance_to(cloud.points[nearest]);
        }
    }
    
    avg_spacing /= sample_count;
    float search_radius = avg_spacing * 3.0f;
    
    // 为每个点构建三角形
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        Vector<size_t> neighbors = find_points_in_radius(cloud.points[i], cloud, search_radius);
        
        // 选择最近的几个邻居构建三角形
        if (neighbors.size() >= 3) {
            // 排序邻居按距离
            Vector<std::pair<float, size_t>> sorted_neighbors;
            for (size_t j : neighbors) {
                float dist = cloud.points[i].distance_to(cloud.points[j]);
                sorted_neighbors.push_back({dist, j});
            }
            std::sort(sorted_neighbors.begin(), sorted_neighbors.end());
            
            // 构建三角形
            for (size_t k = 0; k + 1 < sorted_neighbors.size(); ++k) {
                mesh.add_triangle(static_cast<uint32_t>(i),
                                 static_cast<uint32_t>(sorted_neighbors[k].second),
                                 static_cast<uint32_t>(sorted_neighbors[k + 1].second));
            }
        }
    }
    
    return ErrorCode::Success;
}

// 网格简化（简化实现）
ErrorCode simplify_mesh(const MeshData& mesh,
                        float target_ratio,
                        MeshData& result) {
    
    if (mesh.empty() || target_ratio <= 0.0f || target_ratio >= 1.0f) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t target_vertex_count = static_cast<size_t>(mesh.vertex_count() * target_ratio);
    
    // 简化实现：随机采样顶点
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(0, static_cast<uint32_t>(mesh.vertex_count() - 1));
    
    Vector<bool> selected(mesh.vertex_count(), false);
    result.clear();
    
    // 选择目标数量的顶点
    Vector<uint32_t> new_indices(mesh.vertex_count(), -1);
    uint32_t new_idx = 0;
    
    while (result.vertex_count() < target_vertex_count) {
        uint32_t old_idx = dis(gen);
        if (!selected[old_idx]) {
            selected[old_idx] = true;
            result.add_vertex(mesh.vertices[old_idx]);
            
            if (mesh.has_normals()) {
                result.normals.push_back(mesh.normals[old_idx]);
            }
            if (mesh.has_colors()) {
                result.colors.push_back(mesh.colors[old_idx]);
            }
            
            new_indices[old_idx] = new_idx++;
        }
    }
    
    // 更新三角形索引
    for (size_t i = 0; i < mesh.triangles.size(); i += 3) {
        uint32_t i0 = mesh.triangles[i];
        uint32_t i1 = mesh.triangles[i + 1];
        uint32_t i2 = mesh.triangles[i + 2];
        
        // 只有当所有三个顶点都被选中时才保留三角形
        if (selected[i0] && selected[i1] && selected[i2]) {
            result.add_triangle(new_indices[i0], new_indices[i1], new_indices[i2]);
        }
    }
    
    return ErrorCode::Success;
}

// ============================================================================
// 点云处理算法实现
// ============================================================================

// 计算点云法向量（PCA方法）
ErrorCode compute_normals(const PointCloudData& cloud,
                          float search_radius,
                          Vector<Point3Df>& normals) {
    
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    normals.clear();
    normals.resize(cloud.size());
    
    // 对每个点，计算其邻域的PCA以估计法向量
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        Vector<size_t> neighbors = find_points_in_radius(cloud.points[i], cloud, search_radius);
        
        if (neighbors.size() < 3) {
            // 如果邻域太小，使用默认法向量
            normals[i] = Point3Df(0, 0, 1);
            continue;
        }
        
        // 计算邻域的重心
        Point3Df centroid(0, 0, 0);
        for (size_t j : neighbors) {
            centroid.x += cloud.points[j].x;
            centroid.y += cloud.points[j].y;
            centroid.z += cloud.points[j].z;
        }
        
        float n = static_cast<float>(neighbors.size());
        centroid.x /= n;
        centroid.y /= n;
        centroid.z /= n;
        
        // 计算协方差矩阵
        float cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        for (size_t j : neighbors) {
            float dx = cloud.points[j].x - centroid.x;
            float dy = cloud.points[j].y - centroid.y;
            float dz = cloud.points[j].z - centroid.z;
            
            cov[0][0] += dx * dx;
            cov[0][1] += dx * dy;
            cov[0][2] += dx * dz;
            cov[1][0] += dy * dx;
            cov[1][1] += dy * dy;
            cov[1][2] += dy * dz;
            cov[2][0] += dz * dx;
            cov[2][1] += dz * dy;
            cov[2][2] += dz * dz;
        }
        
        // 简化的PCA：找到最小特征值对应的特征向量
        // 使用幂迭代法近似
        
        // 初始向量
        Point3Df v(1, 0, 0);
        
        // 幂迭代（求最小特征向量，使用逆矩阵近似）
        for (int iter = 0; iter < 10; ++iter) {
            // 简化：使用协方差矩阵的最小列方向
            float col0_len = std::sqrt(cov[0][0] + cov[1][0] * cov[1][0] + cov[2][0] * cov[2][0]);
            float col1_len = std::sqrt(cov[0][1] * cov[0][1] + cov[1][1] + cov[2][1] * cov[2][1]);
            float col2_len = std::sqrt(cov[0][2] * cov[0][2] + cov[1][2] * cov[1][2] + cov[2][2]);
            
            // 选择最短的列作为法向量方向
            if (col0_len <= col1_len && col0_len <= col2_len) {
                v = Point3Df(cov[0][0], cov[1][0], cov[2][0]).normalized();
            } else if (col1_len <= col0_len && col1_len <= col2_len) {
                v = Point3Df(cov[0][1], cov[1][1], cov[2][1]).normalized();
            } else {
                v = Point3Df(cov[0][2], cov[1][2], cov[2][2]).normalized();
            }
        }
        
        normals[i] = v;
    }
    
    return ErrorCode::Success;
}

// 点云平滑（移动最小二乘）
ErrorCode smooth_pointcloud(const PointCloudData& cloud,
                            float search_radius,
                            PointCloudData& result) {
    
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    result.clear();
    
    // 对每个点，计算其邻域的加权平均位置
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        Vector<size_t> neighbors = find_points_in_radius(cloud.points[i], cloud, search_radius);
        
        if (neighbors.empty()) {
            result.add_point(cloud.points[i]);
            continue;
        }
        
        // 计算加权平均位置
        Point3Df smoothed(0, 0, 0);
        float total_weight = 0.0f;
        
        for (size_t j : neighbors) {
            float dist = cloud.points[i].distance_to(cloud.points[j]);
            // 使用距离加权（距离越近权重越大）
            float weight = 1.0f / (1.0f + dist * dist);
            
            smoothed.x += cloud.points[j].x * weight;
            smoothed.y += cloud.points[j].y * weight;
            smoothed.z += cloud.points[j].z * weight;
            total_weight += weight;
        }
        
        if (total_weight > 0) {
            smoothed.x /= total_weight;
            smoothed.y /= total_weight;
            smoothed.z /= total_weight;
        }
        
        result.add_point(smoothed);
        
        if (cloud.has_colors()) {
            result.colors.push_back(cloud.colors[i]);
        }
        if (cloud.has_intensities()) {
            result.intensities.push_back(cloud.intensities[i]);
        }
    }
    
    return ErrorCode::Success;
}

// 点云离群点去除（统计滤波）
ErrorCode remove_outliers(const PointCloudData& cloud,
                          int neighbors,
                          float std_threshold,
                          PointCloudData& result) {
    
    if (cloud.empty()) {
        return ErrorCode::InvalidParameter;
    }
    
    // 使用统计滤波去除离群点
    // 计算每个点到其邻近点的平均距离
    size_t n = cloud.size();
    Vector<float> avg_distances(n);
    
    for (size_t i = 0; i < n; ++i) {
        // 找最近的k个邻居
        Vector<std::pair<float, size_t>> all_dists;
        for (size_t j = 0; j < n; ++j) {
            if (i != j) {
                float dist = cloud.points[i].distance_to(cloud.points[j]);
                all_dists.push_back({dist, j});
            }
        }
        
        // 排序并取前k个
        std::sort(all_dists.begin(), all_dists.end());
        
        float sum = 0.0f;
        int count = std::min(neighbors, static_cast<int>(all_dists.size()));
        for (int k = 0; k < count; ++k) {
            sum += all_dists[k].first;
        }
        
        avg_distances[i] = sum / count;
    }
    
    // 计算全局均值和标准差
    float mean = 0.0f;
    for (float d : avg_distances) {
        mean += d;
    }
    mean /= n;
    
    float variance = 0.0f;
    for (float d : avg_distances) {
        variance += (d - mean) * (d - mean);
    }
    float stddev = std::sqrt(variance / n);
    
    // 过滤离群点
    float threshold = mean + std_threshold * stddev;
    
    result.clear();
    for (size_t i = 0; i < n; ++i) {
        if (avg_distances[i] < threshold) {
            result.add_point(cloud.points[i]);
            
            if (cloud.has_colors()) {
                result.colors.push_back(cloud.colors[i]);
            }
            if (cloud.has_intensities()) {
                result.intensities.push_back(cloud.intensities[i]);
            }
        }
    }
    
    return ErrorCode::Success;
}

// 点云采样
ErrorCode sample_pointcloud(const PointCloudData& cloud,
                            float sample_ratio,
                            int method,
                            PointCloudData& result) {
    
    if (cloud.empty() || sample_ratio <= 0.0f || sample_ratio >= 1.0f) {
        return ErrorCode::InvalidParameter;
    }
    
    size_t target_count = static_cast<size_t>(cloud.size() * sample_ratio);
    
    result.clear();
    
    if (method == 1) {  // 随机采样
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dis(0, cloud.size() - 1);
        
        Vector<bool> selected(cloud.size(), false);
        
        while (result.size() < target_count) {
            size_t idx = dis(gen);
            if (!selected[idx]) {
                selected[idx] = true;
                result.add_point(cloud.points[idx]);
                
                if (cloud.has_colors()) {
                    result.colors.push_back(cloud.colors[idx]);
                }
                if (cloud.has_intensities()) {
                    result.intensities.push_back(cloud.intensities[idx]);
                }
            }
        }
    } else {  // 均匀采样
        size_t step = static_cast<size_t>(1.0f / sample_ratio);
        for (size_t i = 0; i < cloud.size() && result.size() < target_count; i += step) {
            result.add_point(cloud.points[i]);
            
            if (cloud.has_colors()) {
                result.colors.push_back(cloud.colors[i]);
            }
            if (cloud.has_intensities()) {
                result.intensities.push_back(cloud.intensities[i]);
            }
        }
    }
    
    return ErrorCode::Success;
}

} // namespace advanced_3d_utils

// ============================================================================
// 节点实现 - 点云配准节点（4个）
// ============================================================================

// ICPRegisterNode
ICPRegisterNode::ICPRegisterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ICPRegisterNode::make_info() {
    NodeInfo info;
    info.id = "icp_register";
    info.name = "ICP点云配准";
    info.category = "3D高级处理";
    info.description = "使用迭代最近点(ICP)算法进行点云配准";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "目标点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    info.outputs.emplace_back("iterations", "迭代次数", DataType::Number);
    
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(50));
    info.params.emplace_back("tolerance", "收敛容差", DataType::Number, Data(0.001f));
    info.params.emplace_back("max_distance", "最大对应点距离", DataType::Number, Data(0.05f));
    
    return info;
}

Result<void> ICPRegisterNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    max_iterations_ = params_.get_int("max_iterations", 50);
    tolerance_ = static_cast<float>(params_.get_number("tolerance", 0.001f));
    max_distance_ = static_cast<float>(params_.get_number("max_distance", 0.05f));
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    advanced_3d_utils::ICPResult result;
    Transform3D initial;  // 单位矩阵
    
    ErrorCode err = advanced_3d_utils::icp_register(source, target, initial,
                                                     max_iterations_, tolerance_, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "ICP配准失败");
    }
    
    // 应用变换到源点云
    PointCloudData transformed = source;
    advanced_3d_utils::transform_cloud(transformed, result.transformation);
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    set_output("iterations", Data(result.iterations));
    
    return Result<void>::success();
}

// FeatureRegisterNode
FeatureRegisterNode::FeatureRegisterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FeatureRegisterNode::make_info() {
    NodeInfo info;
    info.id = "feature_register";
    info.name = "特征点云配准";
    info.category = "3D高级处理";
    info.description = "基于特征匹配的点云配准";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "目标点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    
    info.params.emplace_back("feature_radius", "特征半径", DataType::Number, Data(0.05f));
    
    return info;
}

Result<void> FeatureRegisterNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    feature_radius_ = static_cast<float>(params_.get_number("feature_radius", 0.05f));
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    advanced_3d_utils::ICPResult result;
    ErrorCode err = advanced_3d_utils::feature_register(source, target, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "特征配准失败");
    }
    
    PointCloudData transformed = source;
    advanced_3d_utils::transform_cloud(transformed, result.transformation);
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    
    return Result<void>::success();
}

// NDTRegisterNode
NDTRegisterNode::NDTRegisterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo NDTRegisterNode::make_info() {
    NodeInfo info;
    info.id = "ndt_register";
    info.name = "NDT配准";
    info.category = "3D高级处理";
    info.description = "使用正态分布变换(NDT)算法进行点云配准";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "目标点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    
    info.params.emplace_back("voxel_size", "体素尺寸", DataType::Number, Data(0.1f));
    info.params.emplace_back("step_size", "步长大小", DataType::Number, Data(0.1f));
    info.params.emplace_back("max_iterations", "最大迭代次数", DataType::Number, Data(50));
    
    return info;
}

Result<void> NDTRegisterNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    voxel_size_ = static_cast<float>(params_.get_number("voxel_size", 0.1f));
    step_size_ = static_cast<float>(params_.get_number("step_size", 0.1f));
    max_iterations_ = params_.get_int("max_iterations", 50);
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    advanced_3d_utils::ICPResult result;
    ErrorCode err = advanced_3d_utils::ndt_register(source, target, voxel_size_,
                                                     step_size_, max_iterations_, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "NDT配准失败");
    }
    
    PointCloudData transformed = source;
    advanced_3d_utils::transform_cloud(transformed, result.transformation);
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    
    return Result<void>::success();
}

// GlobalRegisterNode
GlobalRegisterNode::GlobalRegisterNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo GlobalRegisterNode::make_info() {
    NodeInfo info;
    info.id = "global_register";
    info.name = "全局配准";
    info.category = "3D高级处理";
    info.description = "使用RANSAC进行全局粗配准";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("source_cloud", "源点云", DataType::PointCloud, true);
    info.inputs.emplace_back("target_cloud", "目标点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("transformed_cloud", "变换后点云", DataType::PointCloud);
    info.outputs.emplace_back("fitness_score", "配准得分", DataType::Number);
    
    info.params.emplace_back("distance_threshold", "距离阈值", DataType::Number, Data(0.05f));
    
    return info;
}

Result<void> GlobalRegisterNode::execute(FlowContext& context) {
    auto source_input = get_input("source_cloud");
    auto target_input = get_input("target_cloud");
    
    if (!source_input.is_valid() || !target_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    distance_threshold_ = static_cast<float>(params_.get_number("distance_threshold", 0.05f));
    
    PointCloudData source = source_input.as_pointcloud();
    PointCloudData target = target_input.as_pointcloud();
    
    advanced_3d_utils::ICPResult result;
    ErrorCode err = advanced_3d_utils::global_register(source, target, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "全局配准失败");
    }
    
    PointCloudData transformed = source;
    advanced_3d_utils::transform_cloud(transformed, result.transformation);
    
    set_output("transformed_cloud", Data(std::move(transformed)));
    set_output("fitness_score", Data(result.fitness_score));
    
    return Result<void>::success();
}

// ============================================================================
// 节点实现 - 点云拼接节点（3个）
// ============================================================================

// PointCloudMergeNode
PointCloudMergeNode::PointCloudMergeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudMergeNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_merge";
    info.name = "点云合并拼接";
    info.category = "3D高级处理";
    info.description = "合并多个点云数据";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("cloud1", "点云1", DataType::PointCloud, true);
    info.inputs.emplace_back("cloud2", "点云2", DataType::PointCloud, true);
    
    info.outputs.emplace_back("merged_cloud", "合并点云", DataType::PointCloud);
    
    return info;
}

Result<void> PointCloudMergeNode::execute(FlowContext& context) {
    auto cloud1_input = get_input("cloud1");
    auto cloud2_input = get_input("cloud2");
    
    if (!cloud1_input.is_valid() || !cloud2_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    PointCloudData cloud1 = cloud1_input.as_pointcloud();
    PointCloudData cloud2 = cloud2_input.as_pointcloud();
    
    Vector<PointCloudData> clouds;
    clouds.push_back(cloud1);
    clouds.push_back(cloud2);
    
    PointCloudData result;
    ErrorCode err = advanced_3d_utils::merge_pointclouds(clouds, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云合并失败");
    }
    
    set_output("merged_cloud", Data(std::move(result)));
    
    return Result<void>::success();
}

// MultiViewMergeNode
MultiViewMergeNode::MultiViewMergeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MultiViewMergeNode::make_info() {
    NodeInfo info;
    info.id = "multiview_merge";
    info.name = "多视角点云拼接";
    info.category = "3D高级处理";
    info.description = "多视角点云数据拼接";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("clouds", "点云列表", DataType::Array, true);
    
    info.outputs.emplace_back("merged_cloud", "拼接点云", DataType::PointCloud);
    
    info.params.emplace_back("overlap_threshold", "重叠阈值", DataType::Number, Data(0.1f));
    
    return info;
}

Result<void> MultiViewMergeNode::execute(FlowContext& context) {
    auto clouds_input = get_input("clouds");
    
    if (!clouds_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    overlap_threshold_ = static_cast<float>(params_.get_number("overlap_threshold", 0.1f));
    
    // 简化实现：直接合并点云（假设变换已知）
    // 实际应用中需要进行配准
    PointCloudData result;
    
    // 简化：使用clouds_input中的点云数据
    // 如果是数组，遍历合并
    // 目前简化为合并两个点云
    PointCloudData cloud1 = clouds_input.as_pointcloud();
    
    // 尝试获取第二个点云
    auto cloud2_input = get_input("cloud2");
    if (cloud2_input.is_valid()) {
        PointCloudData cloud2 = cloud2_input.as_pointcloud();
        Vector<PointCloudData> clouds;
        clouds.push_back(cloud1);
        clouds.push_back(cloud2);
        
        ErrorCode err = advanced_3d_utils::merge_pointclouds(clouds, result);
        if (err != ErrorCode::Success) {
            return Result<void>::failure(err, "多视角拼接失败");
        }
    } else {
        result = cloud1;
    }
    
    set_output("merged_cloud", Data(std::move(result)));
    
    return Result<void>::success();
}

// ScanMergeNode
ScanMergeNode::ScanMergeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ScanMergeNode::make_info() {
    NodeInfo info;
    info.id = "scan_merge";
    info.name = "扫描线点云拼接";
    info.category = "3D高级处理";
    info.description = "扫描线点云数据拼接";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("scan_lines", "扫描线数据", DataType::Array, true);
    
    info.outputs.emplace_back("merged_cloud", "拼接点云", DataType::PointCloud);
    
    info.params.emplace_back("line_distance", "扫描线间距", DataType::Number, Data(0.01f));
    
    return info;
}

Result<void> ScanMergeNode::execute(FlowContext& context) {
    auto scan_input = get_input("scan_lines");
    
    if (!scan_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少扫描线输入");
    }
    
    line_distance_ = static_cast<float>(params_.get_number("line_distance", 0.01f));
    
    PointCloudData result;
    
    // 简化实现：直接合并
    PointCloudData scan_data = scan_input.as_pointcloud();
    result = scan_data;
    
    set_output("merged_cloud", Data(std::move(result)));
    
    return Result<void>::success();
}

// ============================================================================
// 节点实现 - 网格重建节点（4个）
// ============================================================================

// PoissonReconstructNode
PoissonReconstructNode::PoissonReconstructNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PoissonReconstructNode::make_info() {
    NodeInfo info;
    info.id = "poisson_reconstruct";
    info.name = "Poisson网格重建";
    info.category = "3D高级处理";
    info.description = "使用Poisson算法进行网格重建";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("mesh", "网格数据", DataType::Object);
    info.outputs.emplace_back("vertex_count", "顶点数", DataType::Number);
    info.outputs.emplace_back("triangle_count", "三角形数", DataType::Number);
    
    info.params.emplace_back("depth", "重建深度", DataType::Number, Data(8));
    info.params.emplace_back("search_radius", "搜索半径", DataType::Number, Data(0.05f));
    
    return info;
}

Result<void> PoissonReconstructNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    depth_ = params_.get_int("depth", 8);
    search_radius_ = static_cast<float>(params_.get_number("search_radius", 0.05f));
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    // 计算法向量
    Vector<Point3Df> normals;
    ErrorCode err = advanced_3d_utils::compute_normals(cloud, search_radius_, normals);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "法向量计算失败");
    }
    
    advanced_3d_utils::MeshData mesh;
    err = advanced_3d_utils::poisson_reconstruct(cloud, normals, depth_, mesh);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Poisson重建失败");
    }
    
    set_output("vertex_count", Data(static_cast<int>(mesh.vertex_count())));
    set_output("triangle_count", Data(static_cast<int>(mesh.triangle_count())));
    
    // 网格数据转换为点云输出（简化）
    PointCloudData mesh_cloud;
    for (const auto& v : mesh.vertices) {
        mesh_cloud.add_point(v);
    }
    set_output("mesh", Data(std::move(mesh_cloud)));
    
    return Result<void>::success();
}

// MarchingCubesNode
MarchingCubesNode::MarchingCubesNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MarchingCubesNode::make_info() {
    NodeInfo info;
    info.id = "marching_cubes";
    info.name = "Marching Cubes重建";
    info.category = "3D高级处理";
    info.description = "使用Marching Cubes算法进行网格重建";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("mesh", "网格数据", DataType::Object);
    info.outputs.emplace_back("vertex_count", "顶点数", DataType::Number);
    
    info.params.emplace_back("voxel_size", "体素尺寸", DataType::Number, Data(0.01f));
    
    return info;
}

Result<void> MarchingCubesNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    voxel_size_ = static_cast<float>(params_.get_number("voxel_size", 0.01f));
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    advanced_3d_utils::MeshData mesh;
    ErrorCode err = advanced_3d_utils::marching_cubes_reconstruct(cloud, voxel_size_, mesh);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Marching Cubes重建失败");
    }
    
    set_output("vertex_count", Data(static_cast<int>(mesh.vertex_count())));
    
    PointCloudData mesh_cloud;
    for (const auto& v : mesh.vertices) {
        mesh_cloud.add_point(v);
    }
    set_output("mesh", Data(std::move(mesh_cloud)));
    
    return Result<void>::success();
}

// DelaunayTriNode
DelaunayTriNode::DelaunayTriNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DelaunayTriNode::make_info() {
    NodeInfo info;
    info.id = "delaunay_triangulation";
    info.name = "Delaunay三角化";
    info.category = "3D高级处理";
    info.description = "对点云进行Delaunay三角化";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("mesh", "网格数据", DataType::Object);
    info.outputs.emplace_back("triangle_count", "三角形数", DataType::Number);
    
    return info;
}

Result<void> DelaunayTriNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    advanced_3d_utils::MeshData mesh;
    ErrorCode err = advanced_3d_utils::delaunay_triangulation(cloud, mesh);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Delaunay三角化失败");
    }
    
    set_output("triangle_count", Data(static_cast<int>(mesh.triangle_count())));
    
    PointCloudData mesh_cloud;
    for (const auto& v : mesh.vertices) {
        mesh_cloud.add_point(v);
    }
    set_output("mesh", Data(std::move(mesh_cloud)));
    
    return Result<void>::success();
}

// MeshSimplifyNode
MeshSimplifyNode::MeshSimplifyNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo MeshSimplifyNode::make_info() {
    NodeInfo info;
    info.id = "mesh_simplify";
    info.name = "网格简化";
    info.category = "3D高级处理";
    info.description = "简化网格以减少顶点数量";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("mesh", "网格数据", DataType::Object, true);
    
    info.outputs.emplace_back("simplified_mesh", "简化网格", DataType::Object);
    info.outputs.emplace_back("vertex_count", "顶点数", DataType::Number);
    
    info.params.emplace_back("target_ratio", "目标比例", DataType::Number, Data(0.5f));
    
    return info;
}

Result<void> MeshSimplifyNode::execute(FlowContext& context) {
    auto mesh_input = get_input("mesh");
    
    if (!mesh_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少网格输入");
    }
    
    target_ratio_ = static_cast<float>(params_.get_number("target_ratio", 0.5f));
    
    // 从点云构建网格（简化）
    PointCloudData cloud = mesh_input.as_pointcloud();
    
    advanced_3d_utils::MeshData mesh;
    for (const auto& pt : cloud.points) {
        mesh.add_vertex(pt);
    }
    
    advanced_3d_utils::MeshData simplified;
    ErrorCode err = advanced_3d_utils::simplify_mesh(mesh, target_ratio_, simplified);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "网格简化失败");
    }
    
    set_output("vertex_count", Data(static_cast<int>(simplified.vertex_count())));
    
    PointCloudData result_cloud;
    for (const auto& v : simplified.vertices) {
        result_cloud.add_point(v);
    }
    set_output("simplified_mesh", Data(std::move(result_cloud)));
    
    return Result<void>::success();
}

// ============================================================================
// 节点实现 - 点云处理节点（4个）
// ============================================================================

// PointCloudNormalsNode
PointCloudNormalsNode::PointCloudNormalsNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudNormalsNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_normals";
    info.name = "点云法向量计算";
    info.category = "3D高级处理";
    info.description = "使用PCA方法计算点云法向量";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("pointcloud_with_normals", "带法向量点云", DataType::PointCloud);
    
    info.params.emplace_back("search_radius", "搜索半径", DataType::Number, Data(0.05f));
    
    return info;
}

Result<void> PointCloudNormalsNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    search_radius_ = static_cast<float>(params_.get_number("search_radius", 0.05f));
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    Vector<Point3Df> normals;
    ErrorCode err = advanced_3d_utils::compute_normals(cloud, search_radius_, normals);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "法向量计算失败");
    }
    
    // 将法向量添加到点云（使用intensities字段临时存储）
    // 实际应用中可能需要扩展PointCloudData结构
    // 这里简化处理，直接返回原始点云
    set_output("pointcloud_with_normals", Data(cloud));
    
    return Result<void>::success();
}

// PointCloudSmoothNode
PointCloudSmoothNode::PointCloudSmoothNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudSmoothNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_smooth";
    info.name = "点云平滑";
    info.category = "3D高级处理";
    info.description = "对点云进行平滑处理";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("smoothed_cloud", "平滑点云", DataType::PointCloud);
    
    info.params.emplace_back("search_radius", "搜索半径", DataType::Number, Data(0.03f));
    
    return info;
}

Result<void> PointCloudSmoothNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    search_radius_ = static_cast<float>(params_.get_number("search_radius", 0.03f));
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    PointCloudData result;
    ErrorCode err = advanced_3d_utils::smooth_pointcloud(cloud, search_radius_, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云平滑失败");
    }
    
    set_output("smoothed_cloud", Data(std::move(result)));
    
    return Result<void>::success();
}

// PointCloudOutlierNode
PointCloudOutlierNode::PointCloudOutlierNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudOutlierNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_outlier";
    info.name = "点云离群点去除";
    info.category = "3D高级处理";
    info.description = "去除点云中的离群点";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("cleaned_cloud", "清理点云", DataType::PointCloud);
    info.outputs.emplace_back("removed_count", "去除数量", DataType::Number);
    
    info.params.emplace_back("neighbors", "邻近点数量", DataType::Number, Data(20));
    info.params.emplace_back("std_threshold", "标准差阈值", DataType::Number, Data(1.0f));
    
    return info;
}

Result<void> PointCloudOutlierNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    neighbors_ = params_.get_int("neighbors", 20);
    std_threshold_ = static_cast<float>(params_.get_number("std_threshold", 1.0f));
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    size_t original_size = cloud.size();
    
    PointCloudData result;
    ErrorCode err = advanced_3d_utils::remove_outliers(cloud, neighbors_, std_threshold_, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "离群点去除失败");
    }
    
    int removed_count = static_cast<int>(original_size - result.size());
    set_output("cleaned_cloud", Data(std::move(result)));
    set_output("removed_count", Data(removed_count));
    
    return Result<void>::success();
}

// PointCloudSampleNode
PointCloudSampleNode::PointCloudSampleNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo PointCloudSampleNode::make_info() {
    NodeInfo info;
    info.id = "pointcloud_sample";
    info.name = "点云采样";
    info.category = "3D高级处理";
    info.description = "对点云进行采样（均匀/随机）";
    info.version = "0.1.0";
    
    info.inputs.emplace_back("pointcloud", "点云", DataType::PointCloud, true);
    
    info.outputs.emplace_back("sampled_cloud", "采样点云", DataType::PointCloud);
    
    info.params.emplace_back("sample_ratio", "采样比例", DataType::Number, Data(0.5f));
    info.params.emplace_back("method", "采样方法", DataType::Number, Data(0));
    
    Vector<String> method_options;
    method_options.push_back("均匀采样");
    method_options.push_back("随机采样");
    info.params.back().options = method_options;
    
    return info;
}

Result<void> PointCloudSampleNode::execute(FlowContext& context) {
    auto cloud_input = get_input("pointcloud");
    
    if (!cloud_input.is_valid()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "缺少点云输入");
    }
    
    sample_ratio_ = static_cast<float>(params_.get_number("sample_ratio", 0.5f));
    int method_int = params_.get_int("method", 0);
    method_ = static_cast<SampleMethod>(method_int);
    
    PointCloudData cloud = cloud_input.as_pointcloud();
    
    PointCloudData result;
    ErrorCode err = advanced_3d_utils::sample_pointcloud(cloud, sample_ratio_, method_int, result);
    
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "点云采样失败");
    }
    
    set_output("sampled_cloud", Data(std::move(result)));
    
    return Result<void>::success();
}

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(ICPRegisterNode, "icp_register", ICPRegisterNode::make_info())
OVF_REGISTER_NODE(FeatureRegisterNode, "feature_register", FeatureRegisterNode::make_info())
OVF_REGISTER_NODE(NDTRegisterNode, "ndt_register", NDTRegisterNode::make_info())
OVF_REGISTER_NODE(GlobalRegisterNode, "global_register", GlobalRegisterNode::make_info())

OVF_REGISTER_NODE(PointCloudMergeNode, "pointcloud_merge", PointCloudMergeNode::make_info())
OVF_REGISTER_NODE(MultiViewMergeNode, "multiview_merge", MultiViewMergeNode::make_info())
OVF_REGISTER_NODE(ScanMergeNode, "scan_merge", ScanMergeNode::make_info())

OVF_REGISTER_NODE(PoissonReconstructNode, "poisson_reconstruct", PoissonReconstructNode::make_info())
OVF_REGISTER_NODE(MarchingCubesNode, "marching_cubes", MarchingCubesNode::make_info())
OVF_REGISTER_NODE(DelaunayTriNode, "delaunay_triangulation", DelaunayTriNode::make_info())
OVF_REGISTER_NODE(MeshSimplifyNode, "mesh_simplify", MeshSimplifyNode::make_info())

OVF_REGISTER_NODE(PointCloudNormalsNode, "pointcloud_normals", PointCloudNormalsNode::make_info())
OVF_REGISTER_NODE(PointCloudSmoothNode, "pointcloud_smooth", PointCloudSmoothNode::make_info())
OVF_REGISTER_NODE(PointCloudOutlierNode, "pointcloud_outlier", PointCloudOutlierNode::make_info())
OVF_REGISTER_NODE(PointCloudSampleNode, "pointcloud_sample", PointCloudSampleNode::make_info())

} // namespace algorithm
} // namespace ovf