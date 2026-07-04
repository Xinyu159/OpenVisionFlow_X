/**
 * @file 3d_matching.h
 * @brief 3D匹配模块 - 基于DXF CAD模型的6自由度匹配
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
 * @brief 相机内参结构
 */
struct CameraParams {
    float focal_length_x = 0.0f;    // 焦距X (像素单位)
    float focal_length_y = 0.0f;    // 焦距Y (像素单位)
    float center_x = 0.0f;          // 光心X坐标
    float center_y = 0.0f;          // 光心Y坐标
    float pixel_width = 0.0f;       // 像素宽度 (物理单位)
    float pixel_height = 0.0f;      // 像素高度 (物理单位)
    float skew = 0.0f;              // 像素倾斜系数
    
    // 畸变参数
    float k1 = 0.0f;                // 径向畸变系数k1
    float k2 = 0.0f;                // 径向畸变系数k2
    float k3 = 0.0f;                // 径向畸变系数k3
    float p1 = 0.0f;                // 切向畸变系数p1
    float p2 = 0.0f;                // 切向畸变系数p2
    
    bool valid() const {
        return focal_length_x > 0 && focal_length_y > 0;
    }
};

/**
 * @brief DXF几何实体类型
 */
enum class DXFEntityType {
    Point = 0,
    Line = 1,
    Arc = 2,
    Circle = 3,
    Polyline = 4,
    LWPolyline = 5,
    Spline = 6,
    Ellipse = 7,
    Text = 8,
    Solid3D = 9,
    Face3D = 10,
    Unknown = 255
};

/**
 * @brief DXF几何实体
 */
struct DXFEntity {
    DXFEntityType type = DXFEntityType::Unknown;
    Vector<Point3Df> points;        // 点集
    float radius = 0.0f;            // 半径（圆/弧）
    float start_angle = 0.0f;       // 起始角度（弧）
    float end_angle = 0.0f;         // 结束角度（弧）
    float height = 0.0f;            // 高度（文字/3D）
    float width = 0.0f;             // 宽度
    Vector<float> knots;            // B样条节点向量
    Vector<float> weights;          // 权重
    String layer;                   // 图层名称
    ColorRGB color;                 // 颜色
    bool hidden = false;            // 是否隐藏
};

/**
 * @brief DXF几何数据（CAD模型）
 */
struct DXFGeometry {
    Vector<DXFEntity> entities;     // 几何实体列表
    mutable Point3Df min_pt;        // 最小边界点（可由update_bounds缓存更新）
    mutable Point3Df max_pt;        // 最大边界点（可由update_bounds缓存更新）
    mutable Point3Df center;        // 中心点（可由update_bounds缓存更新）
    float scale = 1.0f;             // 缩放比例
    String filename;                // 文件名

    bool empty() const { return entities.empty(); }
    size_t size() const { return entities.size(); }
    void clear() { entities.clear(); }

    // 更新边界
    void update_bounds() const;
};

/**
 * @brief 视图模板（渲染视图）
 */
struct ViewTemplate {
    ImageData rendered_image;       // 渲染图像
    ImageData edge_image;           // 边缘图像
    Transform3D pose;               // 视图姿态（相机相对于模型）
    Vector<Point2D<int>> edge_points; // 边缘点坐标
    Vector<Point3Df> correspondences; // 对应的3D点
    float coverage_score = 0.0f;    // 覆盖得分
    int view_index = 0;             // 视图索引
    
    bool valid() const { return !rendered_image.empty(); }
};

/**
 * @brief 3D形状模型
 */
struct ShapeModel3D {
    DXFGeometry cad_geometry;       // CAD几何数据
    Vector<ViewTemplate> views;     // 多视角模板
    CameraParams camera_params;     // 相机内参
    Transform3D reference_pose;     // 参考姿态
    
    // 匹配参数
    float min_score = 0.5f;         // 最小匹配得分
    float greediness = 0.8f;        // 贪婪度
    int num_levels = 4;             // 金字塔层数
    float angle_step = 0.5f;        // 角度步长（度）
    float distance_step = 0.5f;     // 距离步长
    float min_angle = -180.0f;      // 最小旋转角度（度）
    float max_angle = 180.0f;       // 最大旋转角度（度）
    float min_distance = 0.0f;      // 最小距离
    float max_distance = 1000.0f;   // 最大距离
    
    Point3Df centroid;              // 模型中心
    BoundingBox3D bounding_box;     // 3D包围盒
    float model_diameter = 0.0f;    // 模型直径
    
    bool valid() const { return !views.empty() && !cad_geometry.empty(); }
    size_t view_count() const { return views.size(); }
    void clear() {
        cad_geometry.clear();
        views.clear();
    }
};

/**
 * @brief 3D匹配结果（6自由度姿态）
 */
struct Match3DResult {
    // 6自由度姿态
    float x = 0.0f;                 // X位置
    float y = 0.0f;                 // Y位置
    float z = 0.0f;                 // Z位置
    float rx = 0.0f;                // 绕X轴旋转（弧度）
    float ry = 0.0f;                // 绕Y轴旋转（弧度）
    float rz = 0.0f;                // 绕Z轴旋转（弧度）
    
    // 变换矩阵
    Transform3D transformation;     // 4x4变换矩阵
    
    // 匹配质量
    float score = 0.0f;             // 匹配得分
    float confidence = 0.0f;        // 置信度
    int matched_view = 0;           // 匹配的视图索引
    
    // 验证信息
    bool verified = false;          // 是否经过验证
    int num_matched_points = 0;     // 匹配点数量
    float reprojection_error = 0.0f; // 重投影误差
    
    // 附加信息
    String model_name;              // 模型名称
    int match_index = 0;            // 匹配序号
    uint64_t timestamp = 0;         // 时间戳
    
    // 获取Pose结构
    Pose to_pose() const {
        Pose p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.rx = rx;
        p.ry = ry;
        p.rz = rz;
        return p;
    }
    
    // 从Pose结构设置
    void from_pose(const Pose& p) {
        x = static_cast<float>(p.x);
        y = static_cast<float>(p.y);
        z = static_cast<float>(p.z);
        rx = static_cast<float>(p.rx);
        ry = static_cast<float>(p.ry);
        rz = static_cast<float>(p.rz);
    }
};

/**
 * @brief 3D匹配结果集合
 */
struct Match3DResults {
    Vector<Match3DResult> matches;  // 匹配结果列表
    int best_match_index = 0;       // 最佳匹配索引
    float best_score = 0.0f;        // 最佳得分
    int num_matches = 0;            // 匹配数量
    
    bool empty() const { return matches.empty(); }
    size_t size() const { return matches.size(); }
    void clear() { matches.clear(); }
    
    Match3DResult& best() {
        if (!empty()) return matches[best_match_index];
        static Match3DResult empty_result;
        return empty_result;
    }
    
    const Match3DResult& best() const {
        if (!empty()) return matches[best_match_index];
        static Match3DResult empty_result;
        return empty_result;
    }
};

/**
 * @brief 3D匹配工具函数
 */
namespace matching_3d_utils {

// ========== DXF解析 ==========

/**
 * @brief 解析DXF文件
 * @param filepath DXF文件路径
 * @param geometry 输出几何数据
 * @return 错误码
 */
ErrorCode parse_dxf(const String& filepath, DXFGeometry& geometry);

/**
 * @brief 解析DXF文件内容
 * @param content DXF文件内容（字符串）
 * @param geometry 输出几何数据
 * @return 错误码
 */
ErrorCode parse_dxf_content(const String& content, DXFGeometry& geometry);

/**
 * @brief 提取DXF实体边缘点
 * @param entity DXF实体
 * @param edge_points 输出边缘点
 * @param sample_step 采样步长
 * @return 错误码
 */
ErrorCode extract_entity_edges(const DXFEntity& entity, 
                               Vector<Point3Df>& edge_points,
                               float sample_step = 1.0f);

/**
 * @brief 将DXF几何转换为点云
 * @param geometry DXF几何数据
 * @param cloud 输出点云
 * @param sample_step 采样步长
 * @return 错误码
 */
ErrorCode dxf_to_pointcloud(const DXFGeometry& geometry, 
                            PointCloudData& cloud,
                            float sample_step = 1.0f);

// ========== 视图生成 ==========

/**
 * @brief 生成多视角视图模板
 * @param geometry DXF几何数据
 * @param camera_params 相机内参
 * @param views 输出视图模板列表
 * @param num_views 视图数量
 * @param image_width 渲染图像宽度
 * @param image_height 渲染图像高度
 * @return 错误码
 */
ErrorCode generate_views(const DXFGeometry& geometry,
                        const CameraParams& camera_params,
                        Vector<ViewTemplate>& views,
                        int num_views = 20,
                        int image_width = 640,
                        int image_height = 480);

/**
 * @brief 渲染3D几何到2D图像
 * @param geometry DXF几何数据
 * @param pose 相机姿态
 * @param camera_params 相机内参
 * @param image 输出图像
 * @param width 图像宽度
 * @param height 图像高度
 * @return 错误码
 */
ErrorCode render_geometry(const DXFGeometry& geometry,
                         const Transform3D& pose,
                         const CameraParams& camera_params,
                         ImageData& image,
                         int width,
                         int height);

/**
 * @brief 提取边缘图像
 * @param image 输入图像
 * @param edge_image 输出边缘图像
 * @param low_threshold 低阈值
 * @param high_threshold 高阈值
 * @return 错误码
 */
ErrorCode extract_edges(const ImageData& image,
                        ImageData& edge_image,
                        float low_threshold = 50.0f,
                        float high_threshold = 150.0f);

/**
 * @brief 从边缘图像提取边缘点
 * @param edge_image 边缘图像
 * @param edge_points 输出边缘点
 * @return 错误码
 */
ErrorCode get_edge_points(const ImageData& edge_image,
                          Vector<Point2D<int>>& edge_points);

/**
 * @brief 计算视图覆盖得分
 * @param view 视图模板
 * @param geometry 几何数据
 * @return 覆盖得分
 */
float compute_view_coverage(const ViewTemplate& view, const DXFGeometry& geometry);

// ========== 3D匹配 ==========

/**
 * @brief 创建3D形状模型
 * @param geometry DXF几何数据
 * @param camera_params 相机内参
 * @param model 输出3D模型
 * @param num_views 视图数量
 * @return 错误码
 */
ErrorCode create_shape_model_3d(const DXFGeometry& geometry,
                               const CameraParams& camera_params,
                               ShapeModel3D& model,
                               int num_views = 20);

/**
 * @brief 训练3D形状模型（生成视图）
 * @param model 3D模型
 * @param num_views 视图数量
 * @param image_width 图像宽度
 * @param image_height 图像高度
 * @return 错误码
 */
ErrorCode train_shape_model_3d(ShapeModel3D& model,
                              int num_views = 20,
                              int image_width = 640,
                              int image_height = 480);

/**
 * @brief 3D形状匹配（6自由度）
 * @param image 输入图像
 * @param model 3D形状模型
 * @param camera_params 相机内参
 * @param results 输出匹配结果
 * @param min_score 最小得分
 * @param greediness 贪婪度
 * @param num_matches 最大匹配数量
 * @return 错误码
 */
ErrorCode find_shape_model_3d(const ImageData& image,
                             const ShapeModel3D& model,
                             const CameraParams& camera_params,
                             Match3DResults& results,
                             float min_score = 0.5f,
                             float greediness = 0.8f,
                             int num_matches = 1);

/**
 * @brief 乱序场景3D匹配
 * @param image 输入图像
 * @param model 3D形状模型
 * @param camera_params 相机内参
 * @param results 输出匹配结果
 * @param min_score 最小得分
 * @param clutter_factor 乱序因子
 * @return 错误码
 */
ErrorCode find_shape_model_3d_cluttered(const ImageData& image,
                                       const ShapeModel3D& model,
                                       const CameraParams& camera_params,
                                       Match3DResults& results,
                                       float min_score = 0.5f,
                                       float clutter_factor = 0.3f);

/**
 * @brief 多3D模型匹配
 * @param image 输入图像
 * @param models 3D模型列表
 * @param camera_params 相机内参
 * @param results 输出匹配结果列表
 * @param min_score 最小得分
 * @return 错误码
 */
ErrorCode find_shape_model_3d_multi(const ImageData& image,
                                   const Vector<ShapeModel3D>& models,
                                   const CameraParams& camera_params,
                                   Vector<Match3DResults>& results,
                                   float min_score = 0.5f);

/**
 * @brief 2D到3D姿态估计（PnP问题）
 * @param points_2d 2D点坐标
 * @param points_3d 对应的3D点坐标
 * @param camera_params 相机内参
 * @param pose 输出姿态
 * @return 错误码
 */
ErrorCode estimate_pose_pnp(const Vector<Point2D<float>>& points_2d,
                           const Vector<Point3Df>& points_3d,
                           const CameraParams& camera_params,
                           Transform3D& pose);

/**
 * @brief 假设验证（Hypothesis Verification）
 * @param image 输入图像
 * @param model 3D模型
 * @param pose 假设姿态
 * @param camera_params 相机内参
 * @param verified_score 输出验证得分
 * @return 是否验证通过
 */
bool verify_hypothesis(const ImageData& image,
                      const ShapeModel3D& model,
                      const Transform3D& pose,
                      const CameraParams& camera_params,
                      float& verified_score);

// ========== 优化 ==========

/**
 * @brief ICP优化3D匹配结果
 * @param matched_points 匹配的2D点
 * @param model_points 对应的3D模型点
 * @param pose 输入/输出姿态
 * @param camera_params 相机内参
 * @param max_iterations 最大迭代次数
 * @param tolerance 收敛容差
 * @return 错误码
 */
ErrorCode refine_match_3d(const Vector<Point2D<float>>& matched_points,
                         const Vector<Point3Df>& model_points,
                         Transform3D& pose,
                         const CameraParams& camera_params,
                         int max_iterations = 50,
                         float tolerance = 0.001f);

/**
 * @brief 优化单次匹配结果
 * @param image 输入图像
 * @param model 3D模型
 * @param result 输入/输出匹配结果
 * @param camera_params 相机内参
 * @param max_iterations 最大迭代次数
 * @return 错误码
 */
ErrorCode refine_single_match(const ImageData& image,
                             const ShapeModel3D& model,
                             Match3DResult& result,
                             const CameraParams& camera_params,
                             int max_iterations = 50);

// ========== 投影和变换 ==========

/**
 * @brief 投影3D模型到2D图像
 * @param model 3D模型
 * @param pose 姿态
 * @param camera_params 相机内参
 * @param image 输出图像
 * @param width 图像宽度
 * @param height 图像高度
 * @return 错误码
 */
ErrorCode project_model_3d(const ShapeModel3D& model,
                          const Transform3D& pose,
                          const CameraParams& camera_params,
                          ImageData& image,
                          int width,
                          int height);

/**
 * @brief 变换3D模型
 * @param model 输入3D模型
 * @param pose 变换姿态
 * @param transformed_model 输出变换模型
 * @return 错误码
 */
ErrorCode transform_model_3d(const ShapeModel3D& model,
                            const Transform3D& pose,
                            ShapeModel3D& transformed_model);

/**
 * @brief 应用姿态到点
 * @param point 输入3D点
 * @param pose 姿态变换
 * @return 变换后的3D点
 */
Point3Df apply_pose(const Point3Df& point, const Transform3D& pose);

/**
 * @brief 投影3D点到2D
 * @param point_3d 3D点
 * @param pose 相机姿态
 * @param camera_params 相机内参
 * @return 2D点坐标
 */
Point2D<float> project_point_3d_to_2d(const Point3Df& point_3d,
                                      const Transform3D& pose,
                                      const CameraParams& camera_params);

/**
 * @brief 反投影2D点到3D射线
 * @param point_2d 2D点
 * @param pose 相机姿态
 * @param camera_params 相机内参
 * @param ray_origin 输出射线起点
 * @param ray_direction 输出射线方向
 * @return 错误码
 */
ErrorCode backproject_point_2d_to_3d(const Point2D<float>& point_2d,
                                    const Transform3D& pose,
                                    const CameraParams& camera_params,
                                    Point3Df& ray_origin,
                                    Point3Df& ray_direction);

// ========== 序列化 ==========

/**
 * @brief 序列化3D模型到文件
 * @param model 3D模型
 * @param filepath 文件路径
 * @return 错误码
 */
ErrorCode serialize_shape_model_3d(const ShapeModel3D& model, const String& filepath);

/**
 * @brief 从文件反序列化3D模型
 * @param filepath 文件路径
 * @param model 输出3D模型
 * @return 错误码
 */
ErrorCode deserialize_shape_model_3d(const String& filepath, ShapeModel3D& model);

/**
 * @brief 序列化3D模型到字节流
 * @param model 3D模型
 * @param data 输出字节流
 * @return 错误码
 */
ErrorCode serialize_model_to_bytes(const ShapeModel3D& model, ByteArray& data);

/**
 * @brief 从字节流反序列化3D模型
 * @param data 输入字节流
 * @param model 输出3D模型
 * @return 错误码
 */
ErrorCode deserialize_model_from_bytes(const ByteArray& data, ShapeModel3D& model);

// ========== 可视化 ==========

/**
 * @brief 可视化3D匹配结果
 * @param image 输入图像
 * @param model 3D模型
 * @param result 匹配结果
 * @param camera_params 相机内参
 * @param output_image 输出可视化图像
 * @return 错误码
 */
ErrorCode visualize_match_3d(const ImageData& image,
                            const ShapeModel3D& model,
                            const Match3DResult& result,
                            const CameraParams& camera_params,
                            ImageData& output_image);

/**
 * @brief 在图像上绘制3D投影轮廓
 * @param image 输入/输出图像
 * @param geometry DXF几何
 * @param pose 姿态
 * @param camera_params 相机内参
 * @param color 绘制颜色
 * @param thickness 线宽
 * @return 错误码
 */
ErrorCode draw_projected_contour(ImageData& image,
                                const DXFGeometry& geometry,
                                const Transform3D& pose,
                                const CameraParams& camera_params,
                                const ColorRGB& color,
                                int thickness = 2);

// ========== 数学工具 ==========

/**
 * @brief 从6自由度参数构建变换矩阵
 * @param x X位置
 * @param y Y位置
 * @param z Z位置
 * @param rx 绕X轴旋转（弧度）
 * @param ry 绕Y轴旋转（弧度）
 * @param rz 绕Z轴旋转（弧度）
 * @return 4x4变换矩阵
 */
Transform3D build_transform(float x, float y, float z, float rx, float ry, float rz);

/**
 * @brief 从变换矩阵提取6自由度参数
 * @param transform 变换矩阵
 * @param x 输出X位置
 * @param y 输出Y位置
 * @param z 输出Z位置
 * @param rx 输出绕X轴旋转（弧度）
 * @param ry 输出绕Y轴旋转（弧度）
 * @param rz 输出绕Z轴旋转（弧度）
 */
void extract_pose_params(const Transform3D& transform,
                        float& x, float& y, float& z,
                        float& rx, float& ry, float& rz);

/**
 * @brief 矩阵乘法
 * @param a 左矩阵
 * @param b 右矩阵
 * @return 结果矩阵
 */
Transform3D multiply_transform(const Transform3D& a, const Transform3D& b);

/**
 * @brief 矩阵求逆
 * @param transform 输入矩阵
 * @return 逆矩阵
 */
Transform3D inverse_transform(const Transform3D& transform);

/**
 * @brief 计算模型直径
 * @param geometry 几何数据
 * @return 直径
 */
float compute_model_diameter(const DXFGeometry& geometry);

/**
 * @brief 计算两点云之间的距离误差
 * @param cloud1 点云1
 * @param cloud2 点云2
 * @return 平均距离误差
 */
float compute_point_distance_error(const Vector<Point3Df>& cloud1,
                                   const Vector<Point3Df>& cloud2);

} // namespace matching_3d_utils

// ============================================================================
// 3D模型处理节点（4个）
// ============================================================================

/**
 * @brief DXF文件加载节点
 */
class DXFLoaderNode : public INode {
public:
    DXFLoaderNode(const String& instance_id);
    ~DXFLoaderNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    String filepath_;
    float scale_ = 1.0f;
    String layer_filter_;
    bool extract_edges_ = true;
    float edge_sample_step_ = 1.0f;
};

/**
 * @brief 创建3D形状模型节点
 */
class CreateShapeModel3DNode : public INode {
public:
    CreateShapeModel3DNode(const String& instance_id);
    ~CreateShapeModel3DNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int num_views_ = 20;
    int image_width_ = 640;
    int image_height_ = 480;
    float min_score_ = 0.5f;
    float greediness_ = 0.8f;
};

/**
 * @brief 3D模型训练节点（生成视图）
 */
class ShapeModel3DTrainNode : public INode {
public:
    ShapeModel3DTrainNode(const String& instance_id);
    ~ShapeModel3DTrainNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int num_views_ = 20;
    int image_width_ = 640;
    int image_height_ = 480;
    bool use_sphere_sampling_ = true;  // 使用球面采样
    float view_distance_ = 500.0f;     // 视图距离
};

/**
 * @brief 3D模型序列化节点（保存/加载）
 */
class ShapeModel3DSerializeNode : public INode {
public:
    ShapeModel3DSerializeNode(const String& instance_id);
    ~ShapeModel3DSerializeNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    String filepath_;
    bool save_mode_ = true;  // true=保存, false=加载
};

// ============================================================================
// 3D匹配执行节点（4个）
// ============================================================================

/**
 * @brief 3D形状匹配节点（6自由度）
 */
class FindShapeModel3DNode : public INode {
public:
    FindShapeModel3DNode(const String& instance_id);
    ~FindShapeModel3DNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float min_score_ = 0.5f;
    float greediness_ = 0.8f;
    int num_matches_ = 1;
    bool use_verification_ = true;
    bool use_refinement_ = true;
    int refinement_iterations_ = 50;
};

/**
 * @brief 乱序3D匹配节点
 */
class FindShapeModel3DClutteredNode : public INode {
public:
    FindShapeModel3DClutteredNode(const String& instance_id);
    ~FindShapeModel3DClutteredNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float min_score_ = 0.5f;
    float clutter_factor_ = 0.3f;
    int num_matches_ = 5;
    float overlap_threshold_ = 0.1f;
};

/**
 * @brief 多3D模型匹配节点
 */
class FindShapeModel3DMultiNode : public INode {
public:
    FindShapeModel3DMultiNode(const String& instance_id);
    ~FindShapeModel3DMultiNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    float min_score_ = 0.5f;
    int max_matches_per_model_ = 3;
};

/**
 * @brief 3D匹配优化节点
 */
class Match3DRefineNode : public INode {
public:
    Match3DRefineNode(const String& instance_id);
    ~Match3DRefineNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int max_iterations_ = 50;
    float tolerance_ = 0.001f;
    bool use_icp_ = true;
    bool use_boundary_optimization_ = true;
};

// ============================================================================
// 3D匹配结果节点（4个）
// ============================================================================

/**
 * @brief 获取匹配结果节点
 */
class GetShapeModel3DMatchesNode : public INode {
public:
    GetShapeModel3DMatchesNode(const String& instance_id);
    ~GetShapeModel3DMatchesNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int match_index_ = 0;        // 获取第几个匹配结果
    bool get_best_ = true;       // 获取最佳匹配
    bool output_pose_ = true;    // 输出Pose结构
    bool output_matrix_ = true;  // 输出变换矩阵
};

/**
 * @brief 3D模型投影节点（到2D）
 */
class ShapeModel3DProjectNode : public INode {
public:
    ShapeModel3DProjectNode(const String& instance_id);
    ~ShapeModel3DProjectNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int output_width_ = 640;
    int output_height_ = 480;
    bool draw_edges_ = true;
    bool draw_filled_ = false;
    ColorRGB edge_color_{0, 255, 0};
    int edge_thickness_ = 2;
};

/**
 * @brief 3D模型变换节点
 */
class ShapeModel3DTransformNode : public INode {
public:
    ShapeModel3DTransformNode(const String& instance_id);
    ~ShapeModel3DTransformNode() override = default;
    
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
 * @brief 3D可视化节点
 */
class ShapeModel3DVisualizeNode : public INode {
public:
    ShapeModel3DVisualizeNode(const String& instance_id);
    ~ShapeModel3DVisualizeNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();

private:
    int output_width_ = 800;
    int output_height_ = 600;
    bool show_model_ = true;
    bool show_matches_ = true;
    bool show_axes_ = true;
    bool show_bounding_box_ = false;
    ColorRGB match_color_{0, 255, 0};
    ColorRGB model_color_{255, 255, 255};
    float axis_length_ = 50.0f;
};

} // namespace algorithm
} // namespace ovf