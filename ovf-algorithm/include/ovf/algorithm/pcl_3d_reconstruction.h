/**
 * @file pcl_3d_reconstruction.h
 * @brief 高级3D重建模块 - PCL风格点云处理算法
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>
#include <unordered_map>

namespace ovf {
namespace algorithm {

// ============================================================================
// 数据结构定义
// ============================================================================

/**
 * @brief 点云法向数据
 */
struct PointCloudNormal {
    Point3Df normal;          // 法向量
    float curvature = 0.0f;   // 曲率
    
    PointCloudNormal() = default;
    PointCloudNormal(float nx, float ny, float nz, float curv = 0.0f)
        : normal(nx, ny, nz), curvature(curv) {}
};

/**
 * @brief 网格数据（三角网格）
 */
struct PointCloudMesh {
    Vector<Point3Df> vertices;       // 顶点
    Vector<Point3Df> vertex_normals; // 顶点法向
    Vector<ColorRGB> vertex_colors;  // 顶点颜色
    Vector<uint32_t> faces;          // 面片索引（每3个uint32_t为一个三角形）
    
    bool empty() const { return vertices.empty(); }
    size_t vertex_count() const { return vertices.size(); }
    size_t face_count() const { return faces.size() / 3; }
    void clear() {
        vertices.clear();
        vertex_normals.clear();
        vertex_colors.clear();
        faces.clear();
    }
    
    void add_vertex(const Point3Df& v) { vertices.push_back(v); }
    void add_face(uint32_t i0, uint32_t i1, uint32_t i2) {
        faces.push_back(i0);
        faces.push_back(i1);
        faces.push_back(i2);
    }
};

/**
 * @brief FPFH特征描述子（快速点特征直方图）
 */
struct FPFHFeature {
    Vector<float> histogram;  // 33维直方图
    
    FPFHFeature() : histogram(33, 0.0f) {}
    explicit FPFHFeature(const Vector<float>& hist) : histogram(hist) {}
    
    float operator[](size_t i) const { return histogram[i]; }
    float& operator[](size_t i) { return histogram[i]; }
    size_t size() const { return histogram.size(); }
};

/**
 * @brief SIFT 3D特征点
 */
struct SIFT3DFeature {
    Point3Df position;        // 特征点位置
    float scale;              // 尺度
    Point3Df normal;          // 法向（可选）
    Vector<float> descriptor; // 描述子（128维）
    
    SIFT3DFeature() : scale(0.0f) {}
};

/**
 * @brief 点云特征数据集
 */
struct PointCloudFeatures {
    Vector<FPFHFeature> fpfh_features;    // FPFH特征
    Vector<SIFT3DFeature> sift_features;  // SIFT特征
    
    bool empty() const { return fpfh_features.empty() && sift_features.empty(); }
    void clear() {
        fpfh_features.clear();
        sift_features.clear();
    }
};

/**
 * @brief 配准结果
 */
struct RegistrationResult {
    Transform3D transform;      // 变换矩阵
    float fitness_score = 0.0f; // 配准得分（内点比例）
    float rmse = 0.0f;          // 均方根误差
    int inlier_count = 0;       // 内点数量
    bool converged = false;     // 是否收敛
};

/**
 * @brief 平面分割结果
 */
struct PlaneSegmentResult {
    Plane3D plane;                // 检测到的平面
    PointCloudData plane_cloud;   // 平面上的点
    PointCloudData remaining;     // 剩余点
    Vector<int> plane_indices;    // 平面点的索引
};

/**
 * @brief 聚类分割结果
 */
struct ClusterSegmentResult {
    Vector<PointCloudData> clusters;      // 聚类结果
    Vector<Vector<int>> cluster_indices;  // 每个聚类的点索引
    int cluster_count = 0;                // 聚类数量
};

// ============================================================================
// 点云处理工具函数
// ============================================================================

namespace pcl_3d_utils {

// ========== 点云获取 ==========

/**
 * @brief 从深度图生成点云
 */
ErrorCode depth_to_pointcloud_advanced(const DepthImageData& depth_img, 
                                        PointCloudData& cloud,
                                        float depth_scale = 1.0f);

/**
 * @brief 从立体视觉生成点云
 */
ErrorCode stereo_to_pointcloud(const ImageData& left_img,
                               const ImageData& right_img,
                               PointCloudData& cloud,
                               float baseline = 0.0f,
                               float focal_length = 0.0f);

/**
 * @brief 从PCD文件加载点云
 */
ErrorCode load_pcd_file(const String& filepath, PointCloudData& cloud);

/**
 * @brief 从PLY文件加载点云
 */
ErrorCode load_ply_file(const String& filepath, PointCloudData& cloud);

/**
 * @brief 从文件加载点云（自动检测格式）
 */
ErrorCode load_pointcloud_file(const String& filepath, PointCloudData& cloud);

/**
 * @brief 保存点云到PCD文件
 */
ErrorCode save_pcd_file(const String& filepath, const PointCloudData& cloud);

/**
 * @brief 保存点云到PLY文件
 */
ErrorCode save_ply_file(const String& filepath, const PointCloudData& cloud);

// ========== 点云预处理 ==========

/**
 * @brief VoxelGrid下采样（三维网格滤波）
 */
ErrorCode voxel_grid_downsample(PointCloudData& cloud, float voxel_size);

/**
 * @brief StatisticalOutlierRemoval（统计离群点去除）
 */
ErrorCode statistical_outlier_removal(PointCloudData& cloud, 
                                       int mean_k = 50, 
                                       float std_threshold = 1.0f);

/**
 * @brief 移动最小二乘法平滑（MLS曲面重建）
 */
ErrorCode moving_least_squares(PointCloudData& cloud, 
                                PointCloudData& smoothed,
                                float search_radius = 0.03f,
                                bool compute_normals = true);

/**
 * @brief 法向估计（PCA方法）
 */
ErrorCode estimate_normals(const PointCloudData& cloud,
                           Vector<PointCloudNormal>& normals,
                           float search_radius = 0.03f,
                           int k_neighbors = 0);

/**
 * @brief 法向估计（使用K邻域）
 */
ErrorCode estimate_normals_knn(const PointCloudData& cloud,
                                Vector<PointCloudNormal>& normals,
                                int k_neighbors = 20);

// ========== 点云配准 ==========

/**
 * @brief ICP配准（点到点）
 */
ErrorCode icp_registration(const PointCloudData& source,
                           const PointCloudData& target,
                           RegistrationResult& result,
                           int max_iterations = 50,
                           float tolerance = 1e-6f,
                           float max_correspondence_distance = 0.05f);

/**
 * @brief ICP配准（点到面）
 */
ErrorCode icp_point_to_plane(const PointCloudData& source,
                              const PointCloudData& target,
                              const Vector<PointCloudNormal>& target_normals,
                              RegistrationResult& result,
                              int max_iterations = 50,
                              float tolerance = 1e-6f,
                              float max_correspondence_distance = 0.05f);

/**
 * @brief NDT配准（正态分布变换）
 */
ErrorCode ndt_registration(const PointCloudData& source,
                           const PointCloudData& target,
                           RegistrationResult& result,
                           float voxel_size = 1.0f,
                           int max_iterations = 50,
                           float tolerance = 1e-6f);

/**
 * @brief FPFH特征提取
 */
ErrorCode compute_fpfh_features(const PointCloudData& cloud,
                                 const Vector<PointCloudNormal>& normals,
                                 Vector<FPFHFeature>& features,
                                 float search_radius = 0.05f);

/**
 * @brief SIFT 3D特征提取
 */
ErrorCode compute_sift3d_features(const PointCloudData& cloud,
                                   Vector<SIFT3DFeature>& features,
                                   float min_scale = 0.01f,
                                   float min_contrast = 0.05f);

/**
 * @brief SAC-IA初始配准（采样一致性初始配准）
 */
ErrorCode sac_ia_registration(const PointCloudData& source,
                               const PointCloudData& target,
                               const Vector<FPFHFeature>& source_features,
                               const Vector<FPFHFeature>& target_features,
                               RegistrationResult& result,
                               int max_iterations = 100,
                               float min_sample_distance = 0.01f);

/**
 * @brief 特征匹配配准（粗配准）
 */
ErrorCode feature_matching_registration(const PointCloudData& source,
                                         const PointCloudData& target,
                                         RegistrationResult& result);

/**
 * @brief 全局配准（多视角融合）
 */
ErrorCode global_registration(Vector<PointCloudData>& clouds,
                               Vector<Transform3D>& transforms,
                               Vector<RegistrationResult>& results);

// ========== 点云分割 ==========

/**
 * @brief RANSAC平面分割
 */
ErrorCode ransac_plane_segmentation(PointCloudData& cloud,
                                     PlaneSegmentResult& result,
                                     float distance_threshold = 0.01f,
                                     int max_iterations = 100,
                                     int min_inliers = 100);

/**
 * @brief 欧式聚类分割
 */
ErrorCode euclidean_cluster_extraction(PointCloudData& cloud,
                                        ClusterSegmentResult& result,
                                        float tolerance = 0.02f,
                                        int min_cluster_size = 100,
                                        int max_cluster_size = 25000);

// ========== 点云重建 ==========

/**
 * @brief 泊松重建（Marching Cubes变体）
 */
ErrorCode poisson_reconstruction(const PointCloudData& cloud,
                                  const Vector<PointCloudNormal>& normals,
                                  PointCloudMesh& mesh,
                                  int depth = 8,
                                  int solver_divide = 8,
                                  int iso_divide = 8,
                                  float point_weight = 4.0f);

/**
 * @brief 贪婪三角化
 */
ErrorCode greedy_triangulation(const PointCloudData& cloud,
                                const Vector<PointCloudNormal>& normals,
                                PointCloudMesh& mesh,
                                float search_radius = 0.025f,
                                float mu = 2.5f,
                                int max_neighbors = 100,
                                float max_angle = 120.0f,
                                float min_angle = 15.0f);

/**
 * @brief 表面重建（简化版泊松）
 */
ErrorCode surface_reconstruction(const PointCloudData& cloud,
                                  PointCloudMesh& mesh,
                                  float grid_resolution = 0.01f);

// ========== 辅助函数 ==========

/**
 * @brief 计算K近邻（暴力搜索）
 */
ErrorCode find_k_nearest_neighbors(const PointCloudData& cloud,
                                    int query_idx,
                                    int k,
                                    Vector<int>& neighbor_indices,
                                    Vector<float>& distances);

/**
 * @brief 计算K近邻（所有点）
 */
ErrorCode compute_knn_graph(const PointCloudData& cloud,
                             int k,
                             Vector<Vector<int>>& neighbor_indices);

/**
 * @brief 构建Kd树（简化版）
 */
class SimpleKdTree {
public:
    SimpleKdTree();
    ~SimpleKdTree();
    
    void build(const PointCloudData& cloud);
    void clear();
    
    ErrorCode nearest_k_search(const Point3Df& query, int k,
                               Vector<int>& indices, Vector<float>& distances) const;
    
    ErrorCode radius_search(const Point3Df& query, float radius,
                            Vector<int>& indices) const;
    
    bool empty() const;
    size_t size() const;
    
private:
    struct Node {
        int idx = -1;
        int left = -1;
        int right = -1;
        int split_dim = 0;
        float split_val = 0.0f;
    };
    
    Vector<Node> nodes_;
    Vector<Point3Df>* points_ = nullptr;
    
    int build_recursive(int start, int end, int depth);
    void search_recursive(int node_idx, const Point3Df& query, int k,
                          Vector<int>& indices, Vector<float>& distances,
                          float max_dist_sq) const;
    void radius_search_recursive(int node_idx, const Point3Df& query,
                                 float radius_sq, Vector<int>& indices) const;
};

} // namespace pcl_3d_utils

// ============================================================================
// 节点定义 - 点云获取（3个）
// ============================================================================

/**
 * @brief 从深度图生成点云节点
 */
class PointCloudFromDepthNode : public INode {
public:
    PointCloudFromDepthNode(const String& instance_id);
    ~PointCloudFromDepthNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float depth_scale_ = 1.0f;
    float focal_length_x_ = 0.0f;
    float focal_length_y_ = 0.0f;
    float center_x_ = 0.0f;
    float center_y_ = 0.0f;
};

/**
 * @brief 从立体视觉生成点云节点
 */
class PointCloudFromStereoNode : public INode {
public:
    PointCloudFromStereoNode(const String& instance_id);
    ~PointCloudFromStereoNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float baseline_ = 0.0f;
    float focal_length_ = 0.0f;
    float disparity_scale_ = 1.0f;
};

/**
 * @brief 从文件加载点云节点
 */
class PointCloudFromFileNode : public INode {
public:
    PointCloudFromFileNode(const String& instance_id);
    ~PointCloudFromFileNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    String filepath_;
    int file_format_ = 0;  // 0: auto, 1: PCD, 2: PLY
};

// ============================================================================
// 节点定义 - 点云预处理（4个）
// ============================================================================

/**
 * @brief 点云滤波节点（去除离群点、噪声点）
 */
class PointCloudFilterNode : public INode {
public:
    enum class FilterMethod {
        Statistical = 0,    // 统计离群点去除
        Radius = 1,         // 半径离群点去除
        VoxelGrid = 2       // 体素网格滤波
    };
    
    PointCloudFilterNode(const String& instance_id);
    ~PointCloudFilterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    FilterMethod method_ = FilterMethod::Statistical;
    int mean_k_ = 50;
    float std_threshold_ = 1.0f;
    float radius_ = 0.05f;
    int min_neighbors_ = 5;
    float voxel_size_ = 0.01f;
};

/**
 * @brief 点云下采样节点（VoxelGrid）
 */
class PointCloudDownsampleNode : public INode {
public:
    PointCloudDownsampleNode(const String& instance_id);
    ~PointCloudDownsampleNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float voxel_size_ = 0.01f;
};

/**
 * @brief 点云平滑节点（移动最小二乘法MLS）
 */
class PointCloudSmoothNode : public INode {
public:
    PointCloudSmoothNode(const String& instance_id);
    ~PointCloudSmoothNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float search_radius_ = 0.03f;
    bool compute_normals_ = true;
    int polynomial_order_ = 2;
};

/**
 * @brief 法向估计节点
 */
class PointCloudNormalEstimationNode : public INode {
public:
    enum class EstimationMethod {
        PCA = 0,     // PCA方法
        KNN = 1,     // K邻域方法
        Radius = 2   // 半径搜索方法
    };
    
    PointCloudNormalEstimationNode(const String& instance_id);
    ~PointCloudNormalEstimationNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    EstimationMethod method_ = EstimationMethod::PCA;
    float search_radius_ = 0.03f;
    int k_neighbors_ = 20;
};

// ============================================================================
// 节点定义 - 点云配准（4个）
// ============================================================================

/**
 * @brief ICP配准节点（精配准）
 */
class PointCloudICPNode : public INode {
public:
    enum class ICPMethod {
        PointToPoint = 0,   // 点到点
        PointToPlane = 1    // 点到面
    };
    
    PointCloudICPNode(const String& instance_id);
    ~PointCloudICPNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    ICPMethod method_ = ICPMethod::PointToPoint;
    int max_iterations_ = 50;
    float tolerance_ = 1e-6f;
    float max_correspondence_distance_ = 0.05f;
};

/**
 * @brief NDT配准节点（正态分布变换）
 */
class PointCloudNDTNode : public INode {
public:
    PointCloudNDTNode(const String& instance_id);
    ~PointCloudNDTNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float voxel_size_ = 1.0f;
    int max_iterations_ = 50;
    float tolerance_ = 1e-6f;
    float step_size_ = 0.1f;
};

/**
 * @brief 特征匹配配准节点（粗配准）
 */
class PointCloudFeatureMatchingNode : public INode {
public:
    enum class FeatureType {
        FPFH = 0,    // FPFH特征
        SIFT3D = 1   // SIFT 3D特征
    };
    
    PointCloudFeatureMatchingNode(const String& instance_id);
    ~PointCloudFeatureMatchingNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    FeatureType feature_type_ = FeatureType::FPFH;
    float feature_radius_ = 0.05f;
    int sac_iterations_ = 100;
};

/**
 * @brief 全局配准节点（多视角融合）
 */
class PointCloudGlobalRegistrationNode : public INode {
public:
    PointCloudGlobalRegistrationNode(const String& instance_id);
    ~PointCloudGlobalRegistrationNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    bool use_pairwise_ = true;
    int max_iterations_ = 50;
    float convergence_threshold_ = 1e-6f;
};

// ============================================================================
// 节点定义 - 点云分割（2个）
// ============================================================================

/**
 * @brief 平面分割节点（RANSAC）
 */
class PointCloudSegmentPlaneNode : public INode {
public:
    PointCloudSegmentPlaneNode(const String& instance_id);
    ~PointCloudSegmentPlaneNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float distance_threshold_ = 0.01f;
    int max_iterations_ = 100;
    int min_inliers_ = 100;
};

/**
 * @brief 聚类分割节点（欧式聚类）
 */
class PointCloudSegmentClusterNode : public INode {
public:
    PointCloudSegmentClusterNode(const String& instance_id);
    ~PointCloudSegmentClusterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float tolerance_ = 0.02f;
    int min_cluster_size_ = 100;
    int max_cluster_size_ = 25000;
};

// ============================================================================
// 节点定义 - 点云重建（2个）
// ============================================================================

/**
 * @brief 点云网格化节点（泊松重建、贪婪三角化）
 */
class PointCloudMeshNode : public INode {
public:
    enum class MeshMethod {
        Poisson = 0,         // 泊松重建
        GreedyTriangle = 1,  // 贪婪三角化
        MarchingCubes = 2    // Marching Cubes
    };
    
    PointCloudMeshNode(const String& instance_id);
    ~PointCloudMeshNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    MeshMethod method_ = MeshMethod::Poisson;
    int poisson_depth_ = 8;
    float search_radius_ = 0.025f;
    float grid_resolution_ = 0.01f;
};

/**
 * @brief 表面重建节点
 */
class PointCloudSurfaceNode : public INode {
public:
    PointCloudSurfaceNode(const String& instance_id);
    ~PointCloudSurfaceNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float resolution_ = 0.01f;
    bool smooth_surface_ = true;
};

} // namespace algorithm
} // namespace ovf