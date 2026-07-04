/**
 * @file stereo_vision.h
 * @brief 双目立体视觉算子模块
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
 * @brief 立体相机内参结构
 */
struct StereoIntrinsics {
    // 左相机内参
    float fx_left = 0.0f;    // 左相机焦距X
    float fy_left = 0.0f;    // 左相机焦距Y
    float cx_left = 0.0f;    // 左相机光心X
    float cy_left = 0.0f;    // 左相机光心Y
    
    // 右相机内参
    float fx_right = 0.0f;   // 右相机焦距X
    float fy_right = 0.0f;   // 右相机焦距Y
    float cx_right = 0.0f;   // 右相机光心X
    float cy_right = 0.0f;   // 右相机光心Y
    
    // 图像尺寸
    uint32_t image_width = 0;
    uint32_t image_height = 0;
    
    bool valid = false;
};

/**
 * @brief 立体相机外参结构（相对位姿）
 */
struct StereoExtrinsics {
    // 旋转矩阵（3x3）- 从左相机到右相机
    float R[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    
    // 平移向量（3x1）- 从左相机到右相机
    float T[3] = {0, 0, 0};  // T[0]为基线距离
    
    // 基线距离（水平偏移）
    float baseline = 0.0f;
    
    bool valid = false;
};

/**
 * @brief 立体校正参数
 */
struct StereoRectifyParams {
    // 左相机校正映射
    Vector<float> map_x_left;
    Vector<float> map_y_left;
    
    // 右相机校正映射
    Vector<float> map_x_right;
    Vector<float> map_y_right;
    
    // 校正后的有效区域（ROI）
    int32_t roi_left_x = 0;
    int32_t roi_left_y = 0;
    int32_t roi_left_width = 0;
    int32_t roi_left_height = 0;
    
    int32_t roi_right_x = 0;
    int32_t roi_right_y = 0;
    int32_t roi_right_width = 0;
    int32_t roi_right_height = 0;
    
    uint32_t image_width = 0;
    uint32_t image_height = 0;
    
    bool valid = false;
};

/**
 * @brief 立体标定结果
 */
struct StereoCalibrationResult {
    StereoIntrinsics intrinsics;
    StereoExtrinsics extrinsics;
    StereoRectifyParams rectify_params;
    
    // 标定误差
    float reprojection_error = 0.0f;  // 重投影误差
    float max_error = 0.0f;
    
    bool valid = false;
};

/**
 * @brief 视差图数据
 */
struct DisparityData {
    ImageData disparity;           // 视差图（通常为16位整数，实际视差值=值/16）
    float min_disparity = 0.0f;    // 最小视差
    float max_disparity = 0.0f;    // 最大视差
    float disparity_scale = 16.0f; // 视差缩放因子（通常为16）
    float baseline = 0.0f;         // 基线距离
    float focal_length = 0.0f;     // 焦距
    
    bool valid = false;
    
    // 获取视差值
    float get_disparity(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(disparity.width) ||
            y < 0 || y >= static_cast<int>(disparity.height)) {
            return 0.0f;
        }
        
        float raw_disp = 0.0f;
        if (disparity.format == ImageFormat::Mono16) {
            const uint16_t* ptr = reinterpret_cast<const uint16_t*>(disparity.data.data());
            raw_disp = static_cast<float>(ptr[y * disparity.width + x]);
        } else if (disparity.format == ImageFormat::Float32) {
            const float* ptr = reinterpret_cast<const float*>(disparity.data.data());
            raw_disp = ptr[y * disparity.width + x];
        } else {
            raw_disp = static_cast<float>(disparity.data[y * disparity.width + x]);
        }
        
        return raw_disp / disparity_scale;
    }
    
    // 视差转深度
    float get_depth(int x, int y) const {
        float disp = get_disparity(x, y);
        if (disp <= 0.0f) {
            return 0.0f;
        }
        return baseline * focal_length / disp;
    }
};

/**
 * @brief 立体视觉工具函数
 */
namespace stereo_utils {

/**
 * @brief 立体标定（计算内外参）
 * @param left_images 左相机图像列表
 * @param right_images 右相机图像列表
 * @param corners_left 左相机角点检测结果
 * @param corners_right 右相机角点检测结果
 * @param pattern_width 标定板宽度（角点数）
 * @param pattern_height 标定板高度（角点数）
 * @param square_size 标定板方格尺寸（mm）
 * @param result 标定结果
 * @return 错误码
 */
ErrorCode stereo_calibrate(const Vector<ImageData>& left_images,
                           const Vector<ImageData>& right_images,
                           const Vector<Vector<Point2D<float>>>& corners_left,
                           const Vector<Vector<Point2D<float>>>& corners_right,
                           int pattern_width, int pattern_height,
                           float square_size,
                           StereoCalibrationResult& result);

/**
 * @brief 极线校正（计算校正映射）
 * @param intrinsics 内参
 * @param extrinsics 外参
 * @param rectify_params 校正参数输出
 * @param image_width 图像宽度
 * @param image_height 图像高度
 * @return 错误码
 */
ErrorCode stereo_rectify(const StereoIntrinsics& intrinsics,
                         const StereoExtrinsics& extrinsics,
                         StereoRectifyParams& rectify_params,
                         uint32_t image_width, uint32_t image_height);

/**
 * @brief 应用校正映射
 * @param input 输入图像
 * @param map_x X映射
 * @param map_y Y映射
 * @param output 输出图像
 * @return 错误码
 */
ErrorCode apply_rectify_map(const ImageData& input,
                            const Vector<float>& map_x,
                            const Vector<float>& map_y,
                            ImageData& output);

/**
 * @brief 计算立体参数（基线、焦距）
 * @param extrinsics 外参
 * @param intrinsics 内参
 * @param baseline 基线输出
 * @param focal_length 焦距输出
 * @return 错误码
 */
ErrorCode compute_stereo_params(const StereoExtrinsics& extrinsics,
                                const StereoIntrinsics& intrinsics,
                                float& baseline, float& focal_length);

/**
 * @brief Block Matching视差计算
 * @param left_image 左图（校正后）
 * @param right_image 右图（校正后）
 * @param num_disparities 视差搜索范围（必须是16的倍数）
 * @param block_size 块大小（通常为5-21）
 * @param min_disparity 最小视差
 * @param disparity 输出视差图
 * @return 错误码
 */
ErrorCode stereo_match_bm(const ImageData& left_image,
                          const ImageData& right_image,
                          int num_disparities, int block_size,
                          int min_disparity,
                          DisparityData& disparity);

/**
 * @brief Semi-Global Block Matching视差计算
 * @param left_image 左图（校正后）
 * @param right_image 右图（校正后）
 * @param num_disparities 视差搜索范围
 * @param block_size 块大小
 * @param min_disparity 最小视差
 * @param P1 平滑惩罚参数1
 * @param P2 平滑惩罚参数2
 * @param uniqueness_ratio 唯一性比率
 * @param disparity 输出视差图
 * @return 错误码
 */
ErrorCode stereo_match_sgbm(const ImageData& left_image,
                            const ImageData& right_image,
                            int num_disparities, int block_size,
                            int min_disparity,
                            int P1, int P2,
                            int uniqueness_ratio,
                            DisparityData& disparity);

/**
 * @brief NCC视差计算（归一化交叉相关）
 * @param left_image 左图（校正后）
 * @param right_image 右图（校正后）
 * @param num_disparities 视差搜索范围
 * @param window_size 窗口大小
 * @param min_disparity 最小视差
 * @param disparity 输出视差图
 * @return 错误码
 */
ErrorCode stereo_match_ncc(const ImageData& left_image,
                           const ImageData& right_image,
                           int num_disparities, int window_size,
                           int min_disparity,
                           DisparityData& disparity);

/**
 * @brief 视差转深度
 * @param disparity 视差图
 * @param baseline 基线距离
 * @param focal_length 焦距
 * @param depth 输出深度图
 * @return 错误码
 */
ErrorCode disparity_to_depth(const DisparityData& disparity,
                             float baseline, float focal_length,
                             DepthImageData& depth);

/**
 * @brief 深度图重建
 * @param disparity 视差图
 * @param intrinsics 内参
 * @param extrinsics 外参
 * @param depth 输出深度图
 * @return 错误码
 */
ErrorCode depth_reconstruct(const DisparityData& disparity,
                            const StereoIntrinsics& intrinsics,
                            const StereoExtrinsics& extrinsics,
                            DepthImageData& depth);

/**
 * @brief 双目点云生成
 * @param disparity 视差图
 * @param left_image 左图（用于颜色）
 * @param intrinsics 内参
 * @param extrinsics 外参
 * @param cloud 输出点云
 * @return 错误码
 */
ErrorCode pointcloud_stereo(const DisparityData& disparity,
                            const ImageData& left_image,
                            const StereoIntrinsics& intrinsics,
                            const StereoExtrinsics& extrinsics,
                            PointCloudData& cloud);

/**
 * @brief 三角化重建
 * @param left_point 左图点
 * @param right_point 右图点
 * @param intrinsics 内参
 * @param extrinsics 外参
 * @param point_3d 输出3D点
 * @return 错误码
 */
ErrorCode triangulate(const Point2D<float>& left_point,
                      const Point2D<float>& right_point,
                      const StereoIntrinsics& intrinsics,
                      const StereoExtrinsics& extrinsics,
                      Point3Df& point_3d);

/**
 * @brief 立体测量（3D距离）
 * @param point1 第一个3D点
 * @param point2 第二个3D点
 * @return 3D距离
 */
float stereo_measure(const Point3Df& point1, const Point3Df& point2);

/**
 * @brief 障碍物检测（基于深度阈值）
 * @param depth 深度图
 * @param min_depth 最小深度阈值
 * @param max_depth 最大深度阈值
 * @param obstacle_regions 输出障碍物区域列表
 * @return 错误码
 */
ErrorCode detect_obstacles(const DepthImageData& depth,
                           float min_depth, float max_depth,
                           Vector<Region>& obstacle_regions);

/**
 * @brief 计算视差图的SAD匹配代价
 * @param left_block 左图块
 * @param right_block 右图块
 * @return SAD代价
 */
float compute_sad(const Vector<uint8_t>& left_block, const Vector<uint8_t>& right_block);

/**
 * @brief 计算NCC匹配代价
 * @param left_block 左图块
 * @param right_block 右图块
 * @return NCC值（-1到1，越大越匹配）
 */
float compute_ncc(const Vector<uint8_t>& left_block, const Vector<uint8_t>& right_block);

/**
 * @brief 亚像素视差精细化
 * @param disparity 视差图
 * @param refined_disparity 精细化视差图
 * @return 错误码
 */
ErrorCode refine_disparity(const DisparityData& disparity, DisparityData& refined_disparity);

/**
 * @brief 视差图滤波（去除噪声）
 * @param disparity 视差图
 * @param filter_type 滤波类型
 * @param filter_size 滤波尺寸
 * @param filtered_disparity 输出滤波后视差图
 * @return 错误码
 */
ErrorCode filter_disparity(const DisparityData& disparity,
                           int filter_type, int filter_size,
                           DisparityData& filtered_disparity);

/**
 * @brief 检查视差有效性
 * @param disparity 视差图
 * @param valid_ratio 有效视差比例
 * @return 错误码
 */
ErrorCode check_disparity_validity(const DisparityData& disparity, float& valid_ratio);

} // namespace stereo_utils

// ============================================================================
// 立体标定节点（3个）
// ============================================================================

/**
 * @brief 双目相机标定节点
 */
class StereoCalibrateNode : public INode {
public:
    StereoCalibrateNode(const String& instance_id);
    ~StereoCalibrateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int pattern_width_ = 9;      // 标定板宽度（角点数）
    int pattern_height_ = 6;     // 标定板高度（角点数）
    float square_size_ = 25.0f;  // 方格尺寸（mm）
    int min_images_ = 10;        // 最少图像数
    StereoCalibrationResult result_;
};

/**
 * @brief 立体校正节点（极线校正）
 */
class StereoRectifyNode : public INode {
public:
    StereoRectifyNode(const String& instance_id);
    ~StereoRectifyNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    StereoRectifyParams rectify_params_;
};

/**
 * @brief 立体参数计算节点（基线、焦距）
 */
class StereoParamsNode : public INode {
public:
    StereoParamsNode(const String& instance_id);
    ~StereoParamsNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float baseline_ = 0.0f;
    float focal_length_ = 0.0f;
};

// ============================================================================
// 视差计算节点（4个）
// ============================================================================

/**
 * @brief Block Matching视差计算节点
 */
class StereoMatchBMNode : public INode {
public:
    StereoMatchBMNode(const String& instance_id);
    ~StereoMatchBMNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int num_disparities_ = 64;   // 视差搜索范围
    int block_size_ = 15;        // 块大小
    int min_disparity_ = 0;      // 最小视差
    float baseline_ = 0.0f;
    float focal_length_ = 0.0f;
};

/**
 * @brief Semi-Global Block Matching视差计算节点
 */
class StereoMatchSGBMNode : public INode {
public:
    StereoMatchSGBMNode(const String& instance_id);
    ~StereoMatchSGBMNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int num_disparities_ = 64;
    int block_size_ = 3;
    int min_disparity_ = 0;
    int P1_ = 8;                 // 平滑惩罚参数1
    int P2_ = 32;                // 平滑惩罚参数2
    int uniqueness_ratio_ = 10;  // 唯一性比率
    float baseline_ = 0.0f;
    float focal_length_ = 0.0f;
};

/**
 * @brief NCC视差计算节点
 */
class StereoMatchNCCNode : public INode {
public:
    StereoMatchNCCNode(const String& instance_id);
    ~StereoMatchNCCNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int num_disparities_ = 64;
    int window_size_ = 9;
    int min_disparity_ = 0;
    float baseline_ = 0.0f;
    float focal_length_ = 0.0f;
};

/**
 * @brief 视差转深度节点
 */
class DisparityToDepthNode : public INode {
public:
    DisparityToDepthNode(const String& instance_id);
    ~DisparityToDepthNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float baseline_ = 0.0f;
    float focal_length_ = 0.0f;
};

// ============================================================================
// 深度重建节点（3个）
// ============================================================================

/**
 * @brief 深度图重建节点
 */
class DepthReconstructNode : public INode {
public:
    DepthReconstructNode(const String& instance_id);
    ~DepthReconstructNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    StereoIntrinsics intrinsics_;
    StereoExtrinsics extrinsics_;
};

/**
 * @brief 双目点云生成节点
 */
class PointCloudStereoNode : public INode {
public:
    PointCloudStereoNode(const String& instance_id);
    ~PointCloudStereoNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    StereoIntrinsics intrinsics_;
    StereoExtrinsics extrinsics_;
    bool use_color_ = true;
};

/**
 * @brief 三角化重建节点
 */
class TriangulateNode : public INode {
public:
    TriangulateNode(const String& instance_id);
    ~TriangulateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    StereoIntrinsics intrinsics_;
    StereoExtrinsics extrinsics_;
};

// ============================================================================
// 立体应用节点（2个）
// ============================================================================

/**
 * @brief 立体测量节点（3D距离）
 */
class StereoMeasureNode : public INode {
public:
    StereoMeasureNode(const String& instance_id);
    ~StereoMeasureNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float depth_scale_ = 1.0f;
};

/**
 * @brief 障碍物检测节点（深度阈值）
 */
class StereoObstacleNode : public INode {
public:
    StereoObstacleNode(const String& instance_id);
    ~StereoObstacleNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float min_depth_ = 0.0f;     // 最小深度阈值
    float max_depth_ = 1000.0f;  // 最大深度阈值
    int min_area_ = 100;         // 最小障碍物面积
};

} // namespace algorithm
} // namespace ovf