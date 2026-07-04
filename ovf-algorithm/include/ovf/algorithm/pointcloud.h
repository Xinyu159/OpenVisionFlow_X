/**
 * @file pointcloud.h
 * @brief 3D点云处理模块
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

/**
 * @brief 点云处理工具函数
 */
namespace pointcloud_utils {

/**
 * @brief 深度图转点云
 * @param depth_img 深度图像数据
 * @param cloud 输出点云数据
 * @return 错误码
 */
ErrorCode depth_to_pointcloud(const DepthImageData& depth_img, PointCloudData& cloud);

/**
 * @brief 体素滤波（下采样）
 * @param cloud 输入点云
 * @param voxel_size 体素尺寸
 * @return 错误码
 */
ErrorCode voxel_filter(PointCloudData& cloud, float voxel_size);

/**
 * @brief 统计滤波（去除离群点）
 * @param cloud 输入点云
 * @param neighbors 邻近点数量
 * @param std_threshold 标准差阈值
 * @return 错误码
 */
ErrorCode statistical_filter(PointCloudData& cloud, int neighbors, float std_threshold);

/**
 * @brief 半径滤波
 * @param cloud 输入点云
 * @param radius 搜索半径
 * @param min_neighbors 最小邻近点数
 * @return 错误码
 */
ErrorCode radius_filter(PointCloudData& cloud, float radius, int min_neighbors);

/**
 * @brief 点云变换（应用4x4变换矩阵）
 * @param cloud 输入点云
 * @param matrix 4x4变换矩阵
 * @return 错误码
 */
ErrorCode transform_pointcloud(PointCloudData& cloud, const float matrix[4][4]);

/**
 * @brief 点云旋转
 * @param cloud 输入点云
 * @param rx 绕X轴旋转角度（弧度）
 * @param ry 绕Y轴旋转角度（弧度）
 * @param rz 绕Z轴旋转角度（弧度）
 * @return 错误码
 */
ErrorCode rotate_pointcloud(PointCloudData& cloud, float rx, float ry, float rz);

/**
 * @brief 点云平移
 * @param cloud 输入点云
 * @param tx X方向平移
 * @param ty Y方向平移
 * @param tz Z方向平移
 * @return 错误码
 */
ErrorCode translate_pointcloud(PointCloudData& cloud, float tx, float ty, float tz);

/**
 * @brief 平面检测（RANSAC方法）
 * @param cloud 输入点云
 * @param plane 输出平面参数
 * @param distance_threshold 距离阈值
 * @param max_iterations 最大迭代次数
 * @return 错误码
 */
ErrorCode find_plane(PointCloudData& cloud, Plane3D& plane, 
                     float distance_threshold = 0.01f, 
                     int max_iterations = 100);

/**
 * @brief 点云聚类
 * @param cloud 输入点云
 * @param clusters 输出聚类结果
 * @param tolerance 聚类距离容差
 * @param min_cluster_size 最小聚类大小
 * @param max_cluster_size 最大聚类大小
 * @return 错误码
 */
ErrorCode find_clusters(PointCloudData& cloud, 
                        Vector<PointCloudData>& clusters, 
                        float tolerance,
                        int min_cluster_size = 10,
                        int max_cluster_size = 100000);

/**
 * @brief 计算包围盒
 * @param cloud 输入点云
 * @param min_pt 最小点
 * @param max_pt 最大点
 * @return 错误码
 */
ErrorCode compute_bounding_box(PointCloudData& cloud, Point3Df& min_pt, Point3Df& max_pt);

/**
 * @brief 计算两点距离
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 欧氏距离
 */
float compute_distance(const Point3Df& p1, const Point3Df& p2);

/**
 * @brief 计算点云高度（相对于平面）
 * @param cloud 输入点云
 * @param plane_normal 平面法向量
 * @return 高度值
 */
float compute_height(PointCloudData& cloud, const Point3Df& plane_normal);

/**
 * @brief 计算点云重心
 * @param cloud 输入点云
 * @return 重心点
 */
Point3Df compute_centroid(const PointCloudData& cloud);

/**
 * @brief 点云归一化（将重心移到原点）
 * @param cloud 输入点云
 */
void normalize_pointcloud(PointCloudData& cloud);

/**
 * @brief 计算点云密度
 * @param cloud 输入点云
 * @param radius 搜索半径
 * @return 平均密度（点数/体积）
 */
float compute_density(PointCloudData& cloud, float radius);

/**
 * @brief 点云合并
 * @param cloud1 第一个点云
 * @param cloud2 第二个点云
 * @param result 输出合并结果
 */
ErrorCode merge_pointclouds(const PointCloudData& cloud1, 
                            const PointCloudData& cloud2, 
                            PointCloudData& result);

/**
 * @brief 点云裁剪（保留指定范围内的点）
 * @param cloud 输入点云
 * @param min_pt 范围最小点
 * @param max_pt 范围最大点
 */
ErrorCode crop_pointcloud(PointCloudData& cloud, 
                          const Point3Df& min_pt, 
                          const Point3Df& max_pt);

/**
 * @brief 点云采样（均匀采样）
 * @param cloud 输入点云
 * @param sample_ratio 采样比例 (0.0-1.0)
 */
ErrorCode sample_pointcloud(PointCloudData& cloud, float sample_ratio);

} // namespace pointcloud_utils

/**
 * @brief 深度图转点云节点
 */
class DepthToPointCloudNode : public INode {
public:
    DepthToPointCloudNode(const String& instance_id);
    ~DepthToPointCloudNode() override = default;
    
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
 * @brief 点云滤波节点
 */
class PointCloudFilterNode : public INode {
public:
    enum class FilterType {
        Voxel = 0,      // 体素滤波
        Statistical = 1, // 统计滤波
        Radius = 2      // 半径滤波
    };
    
    PointCloudFilterNode(const String& instance_id);
    ~PointCloudFilterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    FilterType filter_type_ = FilterType::Voxel;
    float voxel_size_ = 0.01f;
    int statistical_neighbors_ = 20;
    float statistical_threshold_ = 1.0f;
    float radius_ = 0.05f;
    int radius_min_neighbors_ = 5;
};

/**
 * @brief 平面检测节点
 */
class PlaneDetectionNode : public INode {
public:
    PlaneDetectionNode(const String& instance_id);
    ~PlaneDetectionNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float distance_threshold_ = 0.01f;
    int max_iterations_ = 100;
};

/**
 * @brief 3D测量节点
 */
class DepthMeasureNode : public INode {
public:
    DepthMeasureNode(const String& instance_id);
    ~DepthMeasureNode() override = default;
    
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
 * @brief 点云可视化节点
 */
class PointCloudVisualizeNode : public INode {
public:
    PointCloudVisualizeNode(const String& instance_id);
    ~PointCloudVisualizeNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int image_width_ = 800;
    int image_height_ = 600;
    float view_distance_ = 500.0f;
    float rotation_x_ = 30.0f;  // 绕X轴旋转角度（度）
    float rotation_y_ = 45.0f;  // 绕Y轴旋转角度（度）
    float rotation_z_ = 0.0f;   // 绕Z轴旋转角度（度）
    float zoom_ = 1.0f;
    bool show_colors_ = true;
};

/**
 * @brief 点云变换节点
 */
class PointCloudTransformNode : public INode {
public:
    PointCloudTransformNode(const String& instance_id);
    ~PointCloudTransformNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float tx_ = 0.0f;  // 平移X
    float ty_ = 0.0f;  // 平移Y
    float tz_ = 0.0f;  // 平移Z
    float rx_ = 0.0f;  // 旋转X（弧度）
    float ry_ = 0.0f;  // 旋转Y（弧度）
    float rz_ = 0.0f;  // 旋转Z（弧度）
};

/**
 * @brief 点云聚类节点
 */
class PointCloudClusterNode : public INode {
public:
    PointCloudClusterNode(const String& instance_id);
    ~PointCloudClusterNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float tolerance_ = 0.02f;
    int min_cluster_size_ = 10;
    int max_cluster_size_ = 100000;
};

/**
 * @brief 点云包围盒节点
 */
class PointCloudBoundingBoxNode : public INode {
public:
    PointCloudBoundingBoxNode(const String& instance_id);
    ~PointCloudBoundingBoxNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf