/**
 * @file 3d_advanced.h
 * @brief 高级3D算法模块 - 点云配准、拼接、网格重建和处理
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/algorithm/pointcloud.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 高级3D算法工具函数
 */
namespace advanced_3d_utils {

/**
 * @brief ICP配准结果
 */
struct ICPResult {
    Transform3D transformation;  // 变换矩阵
    float fitness_score;         // 配准得分
    int iterations;              // 迭代次数
    bool converged;              // 是否收敛
};

/**
 * @brief 网格数据结构
 */
struct MeshData {
    Vector<Point3Df> vertices;       // 顶点
    Vector<Point3Df> normals;        // 法向量（可选）
    Vector<ColorRGB> colors;         // 颜色（可选）
    Vector<uint32_t> triangles;      // 三角形索引（每3个为一组）
    
    bool empty() const { return vertices.empty(); }
    size_t vertex_count() const { return vertices.size(); }
    size_t triangle_count() const { return triangles.size() / 3; }
    void clear() {
        vertices.clear();
        normals.clear();
        colors.clear();
        triangles.clear();
    }
    
    bool has_normals() const { return !normals.empty() && normals.size() == vertices.size(); }
    bool has_colors() const { return !colors.empty() && colors.size() == vertices.size(); }
    
    void add_vertex(const Point3Df& v) { vertices.push_back(v); }
    void add_triangle(uint32_t i0, uint32_t i1, uint32_t i2) {
        triangles.push_back(i0);
        triangles.push_back(i1);
        triangles.push_back(i2);
    }
};

/**
 * @brief ICP点云配准
 * @param source 源点云
 * @param target 目标点云
 * @param initial_transform 初始变换矩阵
 * @param max_iterations 最大迭代次数
 * @param tolerance 收敛容差
 * @param result 输出配准结果
 * @return 错误码
 */
ErrorCode icp_register(const PointCloudData& source,
                       const PointCloudData& target,
                       const Transform3D& initial_transform,
                       int max_iterations,
                       float tolerance,
                       ICPResult& result);

/**
 * @brief 特征点云配准（基于特征匹配）
 * @param source 源点云
 * @param target 目标点云
 * @param result 输出配准结果
 * @return 错误码
 */
ErrorCode feature_register(const PointCloudData& source,
                           const PointCloudData& target,
                           ICPResult& result);

/**
 * @brief NDT配准（正态分布变换）
 * @param source 源点云
 * @param target 目标点云
 * @param voxel_size 体素尺寸
 * @param step_size 步长大小
 * @param max_iterations 最大迭代次数
 * @param result 输出配准结果
 * @return 错误码
 */
ErrorCode ndt_register(const PointCloudData& source,
                       const PointCloudData& target,
                       float voxel_size,
                       float step_size,
                       int max_iterations,
                       ICPResult& result);

/**
 * @brief 全局配准（粗配准）
 * @param source 源点云
 * @param target 目标点云
 * @param result 输出配准结果
 * @return 错误码
 */
ErrorCode global_register(const PointCloudData& source,
                          const PointCloudData& target,
                          ICPResult& result);

/**
 * @brief 点云合并拼接
 * @param clouds 点云列表
 * @param result 输出合并结果
 * @return 错误码
 */
ErrorCode merge_pointclouds(const Vector<PointCloudData>& clouds,
                            PointCloudData& result);

/**
 * @brief 多视角点云拼接
 * @param clouds 点云列表
 * @param transforms 变换矩阵列表
 * @param result 输出拼接结果
 * @return 错误码
 */
ErrorCode multiview_merge(const Vector<PointCloudData>& clouds,
                          const Vector<Transform3D>& transforms,
                          PointCloudData& result);

/**
 * @brief 扫描线点云拼接
 * @param scan_lines 扫描线数据
 * @param result 输出拼接结果
 * @return 错误码
 */
ErrorCode scan_merge(const Vector<PointCloudData>& scan_lines,
                     PointCloudData& result);

/**
 * @brief Poisson网格重建
 * @param cloud 输入点云
 * @param normals 点云法向量
 * @param depth 重建深度
 * @param mesh 输出网格
 * @return 错误码
 */
ErrorCode poisson_reconstruct(const PointCloudData& cloud,
                              const Vector<Point3Df>& normals,
                              int depth,
                              MeshData& mesh);

/**
 * @brief Marching Cubes重建
 * @param cloud 输入点云
 * @param voxel_size 体素尺寸
 * @param mesh 输出网格
 * @return 错误码
 */
ErrorCode marching_cubes_reconstruct(const PointCloudData& cloud,
                                     float voxel_size,
                                     MeshData& mesh);

/**
 * @brief Delaunay三角化
 * @param cloud 输入点云
 * @param mesh 输出网格
 * @return 错误码
 */
ErrorCode delaunay_triangulation(const PointCloudData& cloud,
                                 MeshData& mesh);

/**
 * @brief 网格简化
 * @param mesh 输入网格
 * @param target_ratio 目标简化比例
 * @param result 输出简化网格
 * @return 错误码
 */
ErrorCode simplify_mesh(const MeshData& mesh,
                        float target_ratio,
                        MeshData& result);

/**
 * @brief 计算点云法向量（PCA方法）
 * @param cloud 输入点云
 * @param search_radius 搜索半径
 * @param normals 输出法向量
 * @return 错误码
 */
ErrorCode compute_normals(const PointCloudData& cloud,
                          float search_radius,
                          Vector<Point3Df>& normals);

/**
 * @brief 点云平滑（移动最小二乘）
 * @param cloud 输入点云
 * @param search_radius 搜索半径
 * @param result 输出平滑点云
 * @return 错误码
 */
ErrorCode smooth_pointcloud(const PointCloudData& cloud,
                            float search_radius,
                            PointCloudData& result);

/**
 * @brief 点云离群点去除（统计滤波）
 * @param cloud 输入点云
 * @param neighbors 邻近点数量
 * @param std_threshold 标准差阈值
 * @param result 输出清理点云
 * @return 错误码
 */
ErrorCode remove_outliers(const PointCloudData& cloud,
                          int neighbors,
                          float std_threshold,
                          PointCloudData& result);

/**
 * @brief 点云采样
 * @param cloud 输入点云
 * @param sample_ratio 采样比例
 * @param method 采样方法（0=均匀，1=随机）
 * @param result 输出采样点云
 * @return 错误码
 */
ErrorCode sample_pointcloud(const PointCloudData& cloud,
                            float sample_ratio,
                            int method,
                            PointCloudData& result);

} // namespace advanced_3d_utils

// ============================================================================
// 点云配准节点（4个）
// ============================================================================

/**
 * @brief ICP点云配准节点
 */
class ICPRegisterNode : public INode {
public:
    ICPRegisterNode(const String& instance_id);
    ~ICPRegisterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int max_iterations_ = 50;
    float tolerance_ = 0.001f;
    float max_distance_ = 0.05f;
};

/**
 * @brief 特征点云配准节点
 */
class FeatureRegisterNode : public INode {
public:
    FeatureRegisterNode(const String& instance_id);
    ~FeatureRegisterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float feature_radius_ = 0.05f;
};

/**
 * @brief NDT配准节点
 */
class NDTRegisterNode : public INode {
public:
    NDTRegisterNode(const String& instance_id);
    ~NDTRegisterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float voxel_size_ = 0.1f;
    float step_size_ = 0.1f;
    int max_iterations_ = 50;
};

/**
 * @brief 全局配准节点
 */
class GlobalRegisterNode : public INode {
public:
    GlobalRegisterNode(const String& instance_id);
    ~GlobalRegisterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float distance_threshold_ = 0.05f;
};

// ============================================================================
// 点云拼接节点（3个）
// ============================================================================

/**
 * @brief 点云合并拼接节点
 */
class PointCloudMergeNode : public INode {
public:
    PointCloudMergeNode(const String& instance_id);
    ~PointCloudMergeNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 多视角点云拼接节点
 */
class MultiViewMergeNode : public INode {
public:
    MultiViewMergeNode(const String& instance_id);
    ~MultiViewMergeNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float overlap_threshold_ = 0.1f;
};

/**
 * @brief 扫描线点云拼接节点
 */
class ScanMergeNode : public INode {
public:
    ScanMergeNode(const String& instance_id);
    ~ScanMergeNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float line_distance_ = 0.01f;
};

// ============================================================================
// 网格重建节点（4个）
// ============================================================================

/**
 * @brief Poisson网格重建节点
 */
class PoissonReconstructNode : public INode {
public:
    PoissonReconstructNode(const String& instance_id);
    ~PoissonReconstructNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int depth_ = 8;
    float search_radius_ = 0.05f;
};

/**
 * @brief Marching Cubes重建节点
 */
class MarchingCubesNode : public INode {
public:
    MarchingCubesNode(const String& instance_id);
    ~MarchingCubesNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float voxel_size_ = 0.01f;
};

/**
 * @brief Delaunay三角化节点
 */
class DelaunayTriNode : public INode {
public:
    DelaunayTriNode(const String& instance_id);
    ~DelaunayTriNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 网格简化节点
 */
class MeshSimplifyNode : public INode {
public:
    MeshSimplifyNode(const String& instance_id);
    ~MeshSimplifyNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float target_ratio_ = 0.5f;
};

// ============================================================================
// 点云处理节点（4个）
// ============================================================================

/**
 * @brief 点云法向量计算节点
 */
class PointCloudNormalsNode : public INode {
public:
    PointCloudNormalsNode(const String& instance_id);
    ~PointCloudNormalsNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float search_radius_ = 0.05f;
};

/**
 * @brief 点云平滑节点
 */
class PointCloudSmoothNode : public INode {
public:
    PointCloudSmoothNode(const String& instance_id);
    ~PointCloudSmoothNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float search_radius_ = 0.03f;
};

/**
 * @brief 点云离群点去除节点
 */
class PointCloudOutlierNode : public INode {
public:
    PointCloudOutlierNode(const String& instance_id);
    ~PointCloudOutlierNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int neighbors_ = 20;
    float std_threshold_ = 1.0f;
};

/**
 * @brief 点云采样节点
 */
class PointCloudSampleNode : public INode {
public:
    enum class SampleMethod {
        Uniform = 0,  // 均匀采样
        Random = 1    // 随机采样
    };
    
    PointCloudSampleNode(const String& instance_id);
    ~PointCloudSampleNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float sample_ratio_ = 0.5f;
    SampleMethod method_ = SampleMethod::Uniform;
};

} // namespace algorithm
} // namespace ovf