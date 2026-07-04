/**
 * @file photometric_stereo.h
 * @brief 光度立体(Photometric Stereo)算子模块
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
 * @brief 光照方向结构
 */
struct LightDirection {
    float lx = 0.0f;    // 光源方向X分量
    float ly = 0.0f;    // 光源方向Y分量
    float lz = 1.0f;    // 光源方向Z分量（通常指向物体）
    bool valid = false;
    
    // 归一化
    void normalize() {
        float len = std::sqrt(lx * lx + ly * ly + lz * lz);
        if (len > 1e-6f) {
            lx /= len;
            ly /= len;
            lz /= len;
        }
    }
    
    // 获取归一化后的方向
    LightDirection normalized() const {
        LightDirection result = *this;
        result.normalize();
        return result;
    }
};

/**
 * @brief 光度立体标定结果
 */
struct PhotometricCalibrationResult {
    Vector<LightDirection> light_directions;  // 光照方向列表
    Vector<float> light_intensities;          // 光源强度列表（可选）
    
    // 标定目标类型
    enum class TargetType {
        Sphere,     // 球面目标
        Plane,      // 平面目标
        Unknown
    };
    TargetType target_type = TargetType::Unknown;
    
    // 标定目标参数
    float sphere_radius = 0.0f;      // 球面半径（mm）
    Point3Df sphere_center;          // 球面中心
    
    // 标定误差
    float calibration_error = 0.0f;
    
    bool valid = false;
};

/**
 * @brief 表面法向图数据
 */
struct SurfaceNormalData {
    ImageData normal_x;    // 法向X分量图（Float32）
    ImageData normal_y;    // 法向Y分量图（Float32）
    ImageData normal_z;    // 法向Z分量图（Float32）
    
    uint32_t width = 0;
    uint32_t height = 0;
    
    bool valid = false;
    
    // 获取指定位置的法向
    Point3Df get_normal(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(width) ||
            y < 0 || y >= static_cast<int>(height)) {
            return Point3Df(0, 0, 1);
        }
        
        float nx = 0.0f, ny = 0.0f, nz = 1.0f;
        
        if (normal_x.format == ImageFormat::Float32) {
            const float* ptr_x = reinterpret_cast<const float*>(normal_x.data.data());
            const float* ptr_y = reinterpret_cast<const float*>(normal_y.data.data());
            const float* ptr_z = reinterpret_cast<const float*>(normal_z.data.data());
            
            nx = ptr_x[y * width + x];
            ny = ptr_y[y * width + x];
            nz = ptr_z[y * width + x];
        }
        
        return Point3Df(nx, ny, nz).normalized();
    }
    
    // 设置指定位置的法向
    void set_normal(int x, int y, const Point3Df& normal) {
        if (x < 0 || x >= static_cast<int>(width) ||
            y < 0 || y >= static_cast<int>(height)) {
            return;
        }
        
        if (normal_x.format == ImageFormat::Float32) {
            float* ptr_x = reinterpret_cast<float*>(normal_x.data.data());
            float* ptr_y = reinterpret_cast<float*>(normal_y.data.data());
            float* ptr_z = reinterpret_cast<float*>(normal_z.data.data());
            
            ptr_x[y * width + x] = normal.x;
            ptr_y[y * width + x] = normal.y;
            ptr_z[y * width + x] = normal.z;
        }
    }
};

/**
 * @brief 表面高度图数据
 */
struct SurfaceHeightData {
    ImageData height;       // 高度图（Float32）
    
    float min_height = 0.0f;
    float max_height = 0.0f;
    float height_scale = 1.0f;  // 高度缩放因子
    
    bool valid = false;
    
    // 获取指定位置的高度
    float get_height(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(height.width) ||
            y < 0 || y >= static_cast<int>(height.height)) {
            return 0.0f;
        }
        
        if (height.format == ImageFormat::Float32) {
            const float* ptr = reinterpret_cast<const float*>(height.data.data());
            return ptr[y * height.width + x];
        }
        
        return 0.0f;
    }
};

/**
 * @brief 表面梯度数据
 */
struct SurfaceGradientData {
    ImageData gradient_x;   // X方向梯度（偏导数dz/dx）
    ImageData gradient_y;   // Y方向梯度（偏导数dz/dy）
    
    uint32_t width = 0;
    uint32_t height = 0;
    
    bool valid = false;
    
    // 获取指定位置的梯度
    Point2D<float> get_gradient(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(width) ||
            y < 0 || y >= static_cast<int>(height)) {
            return Point2D<float>(0, 0);
        }
        
        float gx = 0.0f, gy = 0.0f;
        
        if (gradient_x.format == ImageFormat::Float32) {
            const float* ptr_x = reinterpret_cast<const float*>(gradient_x.data.data());
            const float* ptr_y = reinterpret_cast<const float*>(gradient_y.data.data());
            
            gx = ptr_x[y * width + x];
            gy = ptr_y[y * width + x];
        }
        
        return Point2D<float>(gx, gy);
    }
};

/**
 * @brief 表面曲率数据
 */
struct SurfaceCurvatureData {
    ImageData curvature_x;      // X方向曲率
    ImageData curvature_y;      // Y方向曲率
    ImageData mean_curvature;   // 平均曲率
    ImageData gaussian_curvature; // 高斯曲率
    
    uint32_t width = 0;
    uint32_t height = 0;
    
    bool valid = false;
};

/**
 * @brief 反照率数据
 */
struct AlbedoData {
    ImageData albedo;   // 反照率图（表面反射率）
    
    float min_albedo = 0.0f;
    float max_albedo = 1.0f;
    
    bool valid = false;
};

/**
 * @brief 光度立体缺陷检测结果
 */
struct PhotometricDefectResult {
    Vector<Region> defect_regions;      // 缺陷区域
    Vector<float> defect_severities;    // 缺陷严重程度
    
    // 缺陷类型
    enum class DefectType {
        SurfaceAnomaly,     // 表面异常
        NormalDiscontinuity, // 法向不连续
        AlbedoVariation,    // 反照率变化
        CurvatureAnomaly,   // 曲率异常
        HeightDiscontinuity // 高度不连续
    };
    Vector<DefectType> defect_types;
    
    int total_defects = 0;
    bool valid = false;
};

/**
 * @brief 光度立体工具函数
 */
namespace photometric_utils {

/**
 * @brief 光度立体标定（使用球面目标）
 * @param images 多光照图像列表（不同光照方向下的球面图像）
 * @param sphere_radius 球面半径（mm）
 * @param sphere_center 球面中心坐标
 * @param result 标定结果
 * @return 错误码
 */
ErrorCode calibrate_with_sphere(const Vector<ImageData>& images,
                                float sphere_radius,
                                const Point3Df& sphere_center,
                                PhotometricCalibrationResult& result);

/**
 * @brief 光度立体标定（使用平面目标）
 * @param images 多光照图像列表
 * @param plane_normal 平面法向
 * @param result 标定结果
 * @return 错误码
 */
ErrorCode calibrate_with_plane(const Vector<ImageData>& images,
                               const Point3Df& plane_normal,
                               PhotometricCalibrationResult& result);

/**
 * @brief 从光照图像估计光照方向
 * @param image 单光照图像
 * @param known_albedo 已知反照率（可选）
 * @param light_direction 光照方向输出
 * @return 错误码
 */
ErrorCode estimate_light_direction(const ImageData& image,
                                   float known_albedo,
                                   LightDirection& light_direction);

/**
 * @brief 光度立体表面重建（核心算法）
 * 使用线性最小二乘法求解表面法向
 * @param images 多光照图像列表（至少3张不同光照）
 * @param light_directions 光照方向列表
 * @param normals 表面法向输出
 * @return 错误码
 */
ErrorCode photometric_stereo_reconstruct(const Vector<ImageData>& images,
                                         const Vector<LightDirection>& light_directions,
                                         SurfaceNormalData& normals);

/**
 * @brief 从表面法向估计反照率
 * @param images 多光照图像列表
 * @param normals 表面法向
 * @param light_directions 光照方向列表
 * @param albedo 反照率输出
 * @return 错误码
 */
ErrorCode estimate_albedo(const Vector<ImageData>& images,
                          const SurfaceNormalData& normals,
                          const Vector<LightDirection>& light_directions,
                          AlbedoData& albedo);

/**
 * @brief 从法向计算表面梯度
 * @param normals 表面法向
 * @param gradient 表面梯度输出
 * @return 错误码
 */
ErrorCode compute_surface_gradient(const SurfaceNormalData& normals,
                                   SurfaceGradientData& gradient);

/**
 * @brief 从梯度重建表面高度（Frankot-Chellappa算法）
 * @param gradient 表面梯度
 * @param height 表面高度输出
 * @return 错误码
 */
ErrorCode reconstruct_height_frankot_chellappa(const SurfaceGradientData& gradient,
                                               SurfaceHeightData& height);

/**
 * @brief 从梯度重建表面高度（简单积分法）
 * @param gradient 表面梯度
 * @param height 表面高度输出
 * @return 错误码
 */
ErrorCode reconstruct_height_integration(const SurfaceGradientData& gradient,
                                         SurfaceHeightData& height);

/**
 * @brief 计算表面曲率
 * @param normals 表面法向
 * @param gradient 表面梯度
 * @param curvature 表面曲率输出
 * @return 错误码
 */
ErrorCode compute_surface_curvature(const SurfaceNormalData& normals,
                                    const SurfaceGradientData& gradient,
                                    SurfaceCurvatureData& curvature);

/**
 * @brief 从阴影恢复形状（Shape from Shading）
 * 单光照图像的形状恢复
 * @param image 单光照图像
 * @param light_direction 光照方向
 * @param albedo 已知反照率（可选）
 * @param normals 表面法向输出
 * @param height 表面高度输出
 * @return 错误码
 */
ErrorCode shape_from_shading(const ImageData& image,
                             const LightDirection& light_direction,
                             float albedo,
                             SurfaceNormalData& normals,
                             SurfaceHeightData& height);

/**
 * @brief 镜面反射去除
 * @param images 多光照图像列表
 * @param diffuse_images 去除镜面反射后的漫反射图像输出
 * @return 错误码
 */
ErrorCode remove_specularity(const Vector<ImageData>& images,
                             Vector<ImageData>& diffuse_images);

/**
 * @brief 光度立体缺陷检测
 * @param normals 表面法向
 * @param albedo 反照率
 * @param curvature 表面曲率
 * @param height 表面高度
 * @param thresholds 各类型阈值
 * @param result 缺陷检测结果
 * @return 错误码
 */
ErrorCode detect_photometric_defects(const SurfaceNormalData& normals,
                                     const AlbedoData& albedo,
                                     const SurfaceCurvatureData& curvature,
                                     const SurfaceHeightData& height,
                                     const HashMap<String, float>& thresholds,
                                     PhotometricDefectResult& result);

/**
 * @brief 光度立体数据归一化
 * 参考Halcon的photometric_stereo_normalize
 * @param normals 表面法向
 * @param albedo 反照率
 * @param normalize_albedo 是否归一化反照率
 * @return 错误码
 */
ErrorCode photometric_stereo_normalize(SurfaceNormalData& normals,
                                       AlbedoData& albedo,
                                       bool normalize_albedo);

/**
 * @brief 线性最小二乘求解法向
 * 核心：I = albedo * (N · L)，求解N
 * @param intensities 多光照强度值
 * @param light_directions 光照方向列表
 * @param normal 法向输出
 * @param albedo 反照率输出
 * @return 错误码
 */
ErrorCode solve_normal_ls(const Vector<float>& intensities,
                          const Vector<LightDirection>& light_directions,
                          Point3Df& normal,
                          float& albedo);

/**
 * @brief 图像转灰度
 * @param image 输入图像
 * @param gray 灰度图像输出
 * @return 错误码
 */
ErrorCode convert_to_gray(const ImageData& image, ImageData& gray);

/**
 * @brief 计算图像梯度（用于高度积分）
 * @param height 输入高度图
 * @param gradient_x X方向梯度输出
 * @param gradient_y Y方向梯度输出
 * @return 错误码
 */
ErrorCode compute_height_gradient(const SurfaceHeightData& height,
                                  ImageData& gradient_x,
                                  ImageData& gradient_y);

} // namespace photometric_utils

// ============================================================================
// 光度立体核心节点（2个）
// ============================================================================

/**
 * @brief 光度立体表面重建节点（核心）
 * 多光照表面重建，计算表面法向和反照率
 */
class PhotometricStereoReconstructNode : public INode {
public:
    PhotometricStereoReconstructNode(const String& instance_id);
    ~PhotometricStereoReconstructNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    Vector<LightDirection> light_directions_;
    bool estimate_albedo_ = true;
    bool normalize_result_ = true;
};

/**
 * @brief 光度立体标定节点
 * 光照方向标定（使用球面或平面目标）
 */
class PhotometricStereoCalibrateNode : public INode {
public:
    PhotometricStereoCalibrateNode(const String& instance_id);
    ~PhotometricStereoCalibrateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    PhotometricCalibrationResult calibration_result_;
    int target_type_ = 0;  // 0:球面, 1:平面
    float sphere_radius_ = 25.0f;
};

// ============================================================================
// 表面重建节点（3个）
// ============================================================================

/**
 * @brief 表面法向估计节点
 * 从光度立体结果提取法向图
 */
class SurfaceNormalEstimateNode : public INode {
public:
    SurfaceNormalEstimateNode(const String& instance_id);
    ~SurfaceNormalEstimateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    bool smooth_normals_ = true;
    int smooth_window_size_ = 3;
};

/**
 * @brief 表面高度重建节点
 * 从法向图重建高度图（积分法）
 */
class SurfaceHeightReconstructNode : public INode {
public:
    SurfaceHeightReconstructNode(const String& instance_id);
    ~SurfaceHeightReconstructNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int integration_method_ = 0;  // 0:Frankot-Chellappa, 1:简单积分
    float height_scale_ = 1.0f;
};

/**
 * @brief 反照率估计节点
 * 从光度立体结果估计表面反照率
 */
class AlbedoEstimateNode : public INode {
public:
    AlbedoEstimateNode(const String& instance_id);
    ~AlbedoEstimateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    bool normalize_albedo_ = true;
    float albedo_min_ = 0.0f;
    float albedo_max_ = 1.0f;
};

// ============================================================================
// 光照处理节点（2个）
// ============================================================================

/**
 * @brief 光照方向估计节点
 * 从单光照图像估计光照方向
 */
class LightDirectionEstimateNode : public INode {
public:
    LightDirectionEstimateNode(const String& instance_id);
    ~LightDirectionEstimateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float known_albedo_ = 0.5f;  // 已知反照率（用于估计）
    LightDirection estimated_direction_;
};

/**
 * @brief 多光照图像采集节点
 * 采集多张不同光照条件下的图像
 */
class MultipleLightAcquireNode : public INode {
public:
    MultipleLightAcquireNode(const String& instance_id);
    ~MultipleLightAcquireNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int num_light_directions_ = 4;  // 光照方向数量
    Vector<ImageData> acquired_images_;
};

// ============================================================================
// 表面分析节点（3个）
// ============================================================================

/**
 * @brief 表面曲率计算节点
 * 从法向和梯度计算表面曲率
 */
class SurfaceCurvatureNode : public INode {
public:
    SurfaceCurvatureNode(const String& instance_id);
    ~SurfaceCurvatureNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    bool compute_mean_curvature_ = true;
    bool compute_gaussian_curvature_ = true;
};

/**
 * @brief 表面梯度计算节点
 * 从法向计算表面梯度
 */
class SurfaceGradientNode : public INode {
public:
    SurfaceGradientNode(const String& instance_id);
    ~SurfaceGradientNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float gradient_scale_ = 1.0f;
};

/**
 * @brief 从阴影恢复形状节点
 * Shape from Shading算法
 */
class ShapeFromShadingNode : public INode {
public:
    ShapeFromShadingNode(const String& instance_id);
    ~ShapeFromShadingNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    LightDirection light_direction_;
    float albedo_ = 0.5f;
    int iterations_ = 100;  // SFS迭代次数
};

// ============================================================================
// 图像处理与缺陷检测节点（2个）
// ============================================================================

/**
 * @brief 镜面反射去除节点
 * 去除图像中的镜面反射成分
 */
class SpecularityRemoveNode : public INode {
public:
    SpecularityRemoveNode(const String& instance_id);
    ~SpecularityRemoveNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int method_ = 0;  // 0:多光照差异法, 1:颜色分析法
    float threshold_ = 0.1f;
};

/**
 * @brief 光度立体缺陷检测节点
 * 基于光度立体数据的缺陷检测
 */
class PhotometricDefectDetectNode : public INode {
public:
    PhotometricDefectDetectNode(const String& instance_id);
    ~PhotometricDefectDetectNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float normal_threshold_ = 0.3f;     // 法向异常阈值
    float albedo_threshold_ = 0.2f;     // 反照率变化阈值
    float curvature_threshold_ = 0.5f;  // 曲率异常阈值
    float height_threshold_ = 1.0f;     // 高度不连续阈值
    int min_defect_area_ = 10;          // 最小缺陷面积
    
    PhotometricDefectResult result_;
};

} // namespace algorithm
} // namespace ovf