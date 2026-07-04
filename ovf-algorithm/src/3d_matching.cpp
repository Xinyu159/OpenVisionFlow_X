/**
 * @file 3d_matching.cpp
 * @brief 3D匹配模块实现 - 基于DXF CAD模型的6自由度匹配
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "ovf/algorithm/3d_matching.h"
#include "ovf/algorithm/image_utils.h"
#include "ovf/core/logger.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace ovf {
namespace algorithm {

// ============================================================================
// DXFGeometry 方法实现
// ============================================================================

void DXFGeometry::update_bounds() const {
    if (entities.empty()) {
        min_pt = Point3Df(0, 0, 0);
        max_pt = Point3Df(0, 0, 0);
        center = Point3Df(0, 0, 0);
        return;
    }

    min_pt = Point3Df(std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max());
    max_pt = Point3Df(std::numeric_limits<float>::lowest(),
                     std::numeric_limits<float>::lowest(),
                     std::numeric_limits<float>::lowest());

    for (const auto& entity : entities) {
        for (const auto& pt : entity.points) {
            min_pt.x = std::min(min_pt.x, pt.x);
            min_pt.y = std::min(min_pt.y, pt.y);
            min_pt.z = std::min(min_pt.z, pt.z);
            max_pt.x = std::max(max_pt.x, pt.x);
            max_pt.y = std::max(max_pt.y, pt.y);
            max_pt.z = std::max(max_pt.z, pt.z);

            // 对于圆/弧，考虑半径
            if (entity.type == DXFEntityType::Circle || entity.type == DXFEntityType::Arc) {
                if (!entity.points.empty()) {
                    Point3Df center_pt = entity.points[0];
                    float r = entity.radius;
                    min_pt.x = std::min(min_pt.x, center_pt.x - r);
                    min_pt.y = std::min(min_pt.y, center_pt.y - r);
                    max_pt.x = std::max(max_pt.x, center_pt.x + r);
                    max_pt.y = std::max(max_pt.y, center_pt.y + r);
                }
            }
        }
    }

    center = Point3Df(
        (min_pt.x + max_pt.x) * 0.5f,
        (min_pt.y + max_pt.y) * 0.5f,
        (min_pt.z + max_pt.z) * 0.5f
    );
}

// ============================================================================
// matching_3d_utils 工具函数实现
// ============================================================================

namespace matching_3d_utils {

// 前向声明：边缘匹配得分计算辅助函数
static float compute_edge_match_score(const Vector<Point2D<int>>& input_edges,
                                      const Vector<Point2D<int>>& view_edges,
                                      int input_w, int input_h,
                                      int view_w, int view_h);

// ========== 数学工具 ==========

Transform3D build_transform(float x, float y, float z, float rx, float ry, float rz) {
    Transform3D transform;

    // 计算旋转矩阵（先绕X，再绕Y，最后绕Z）
    float cx = std::cos(rx), sx = std::sin(rx);
    float cy = std::cos(ry), sy = std::sin(ry);
    float cz = std::cos(rz), sz = std::sin(rz);

    // 组合旋转矩阵 R = Rz * Ry * Rx
    transform.m[0][0] = cy * cz;
    transform.m[0][1] = sx * sy * cz - cx * sz;
    transform.m[0][2] = cx * sy * cz + sx * sz;
    transform.m[0][3] = x;

    transform.m[1][0] = cy * sz;
    transform.m[1][1] = sx * sy * sz + cx * cz;
    transform.m[1][2] = cx * sy * sz - sx * cz;
    transform.m[1][3] = y;

    transform.m[2][0] = -sy;
    transform.m[2][1] = sx * cy;
    transform.m[2][2] = cx * cy;
    transform.m[2][3] = z;

    transform.m[3][0] = 0.0f;
    transform.m[3][1] = 0.0f;
    transform.m[3][2] = 0.0f;
    transform.m[3][3] = 1.0f;

    return transform;
}

void extract_pose_params(const Transform3D& transform,
                        float& x, float& y, float& z,
                        float& rx, float& ry, float& rz) {
    // 提取平移
    x = transform.m[0][3];
    y = transform.m[1][3];
    z = transform.m[2][3];

    // 提取旋转角度（使用ZYX顺序）
    float sy = -transform.m[2][0];

    if (std::abs(sy) > 0.99999f) {
        // Gimbal lock情况
        rx = 0.0f;
        ry = sy > 0 ? static_cast<float>(M_PI / 2) : static_cast<float>(-M_PI / 2);
        rz = std::atan2(-transform.m[0][1], transform.m[0][0]);
    } else {
        rx = std::atan2(transform.m[2][1], transform.m[2][2]);
        ry = std::asin(sy);
        rz = std::atan2(transform.m[1][0], transform.m[0][0]);
    }
}

Transform3D multiply_transform(const Transform3D& a, const Transform3D& b) {
    Transform3D result;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                result.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }

    return result;
}

Transform3D inverse_transform(const Transform3D& transform) {
    Transform3D result;

    // 提取旋转矩阵R和平移向量t
    float R[3][3];
    float t[3];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = transform.m[i][j];
        }
        t[i] = transform.m[i][3];
    }

    // 计算R的逆（因为是旋转矩阵，逆等于转置）
    float R_inv[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_inv[i][j] = R[j][i];
        }
    }

    // 计算 -R_inv * t
    float t_inv[3];
    for (int i = 0; i < 3; ++i) {
        t_inv[i] = 0.0f;
        for (int j = 0; j < 3; ++j) {
            t_inv[i] -= R_inv[i][j] * t[j];
        }
    }

    // 构建逆矩阵
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = R_inv[i][j];
        }
        result.m[i][3] = t_inv[i];
    }

    result.m[3][0] = 0.0f;
    result.m[3][1] = 0.0f;
    result.m[3][2] = 0.0f;
    result.m[3][3] = 1.0f;

    return result;
}

Point3Df apply_pose(const Point3Df& point, const Transform3D& pose) {
    return pose.transform(point);
}

Point2D<float> project_point_3d_to_2d(const Point3Df& point_3d,
                                      const Transform3D& pose,
                                      const CameraParams& camera_params) {
    // 先应用姿态变换（将点从模型坐标系转到相机坐标系）
    Point3Df pt_cam = pose.transform(point_3d);

    // 如果点在相机后面，返回无效点
    if (pt_cam.z <= 0.001f) {
        return Point2D<float>(-1.0f, -1.0f);
    }

    // 投影到图像平面
    float u = camera_params.focal_length_x * pt_cam.x / pt_cam.z + camera_params.center_x;
    float v = camera_params.focal_length_y * pt_cam.y / pt_cam.z + camera_params.center_y;

    return Point2D<float>(u, v);
}

ErrorCode backproject_point_2d_to_3d(const Point2D<float>& point_2d,
                                    const Transform3D& pose,
                                    const CameraParams& camera_params,
                                    Point3Df& ray_origin,
                                    Point3Df& ray_direction) {
    if (!camera_params.valid()) {
        return ErrorCode::InvalidParameter;
    }

    // 计算相机坐标系中的射线方向
    float x = (point_2d.x - camera_params.center_x) / camera_params.focal_length_x;
    float y = (point_2d.y - camera_params.center_y) / camera_params.focal_length_y;

    Point3Df ray_dir_cam(x, y, 1.0f);
    ray_dir_cam = ray_dir_cam.normalized();

    // 射线起点在相机坐标系中是(0,0,0)
    Point3Df ray_origin_cam(0, 0, 0);

    // 获取姿态的逆变换（从相机坐标系转到模型坐标系）
    Transform3D pose_inv = inverse_transform(pose);

    // 将射线转换到模型坐标系
    ray_origin = pose_inv.transform(ray_origin_cam);
    ray_direction = pose_inv.transform(ray_dir_cam) - ray_origin;
    ray_direction = ray_direction.normalized();

    return ErrorCode::Success;
}

float compute_model_diameter(const DXFGeometry& geometry) {
    if (geometry.empty()) {
        return 0.0f;
    }

    // 找到最远的两个点
    float max_dist = 0.0f;

    // 简化：使用边界框对角线长度作为直径
    float dx = geometry.max_pt.x - geometry.min_pt.x;
    float dy = geometry.max_pt.y - geometry.min_pt.y;
    float dz = geometry.max_pt.z - geometry.min_pt.z;

    max_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    return max_dist;
}

float compute_point_distance_error(const Vector<Point3Df>& cloud1,
                                   const Vector<Point3Df>& cloud2) {
    if (cloud1.empty() || cloud2.empty()) {
        return std::numeric_limits<float>::max();
    }

    float total_error = 0.0f;
    int count = 0;

    // 计算cloud1每个点到cloud2最近点的平均距离
    for (const auto& p1 : cloud1) {
        float min_dist = std::numeric_limits<float>::max();
        for (const auto& p2 : cloud2) {
            float dist = p1.distance_to(p2);
            min_dist = std::min(min_dist, dist);
        }
        total_error += min_dist;
        count++;
    }

    return count > 0 ? total_error / count : std::numeric_limits<float>::max();
}

// ========== DXF解析 ==========

ErrorCode parse_dxf(const String& filepath, DXFGeometry& geometry) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        OVF_ERROR() << "Failed to open DXF file: " << filepath;
        return ErrorCode::FileOpenFailed;
    }

    String content((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());
    file.close();

    return parse_dxf_content(content, geometry);
}

ErrorCode parse_dxf_content(const String& content, DXFGeometry& geometry) {
    std::istringstream stream(content);
    String line;
    String current_section;
    String current_entity_type;
    DXFEntity current_entity;
    int group_code = 0;
    String group_value;

    geometry.clear();

    enum class ParseState {
        Section,
        Entity,
        GroupCode,
        GroupValue
    };

    ParseState state = ParseState::GroupCode;

    while (std::getline(stream, line)) {
        // 去除空白字符
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.empty()) continue;

        switch (state) {
            case ParseState::GroupCode:
                try {
                    group_code = std::stoi(line);
                } catch (...) {
                    group_code = 0;
                }
                state = ParseState::GroupValue;
                break;

            case ParseState::GroupValue:
                group_value = line;
                state = ParseState::GroupCode;

                // 解析DXF组
                switch (group_code) {
                    case 0:  // 实体类型或段结束标记
                        if (group_value == "SECTION") {
                            current_section.clear();
                        } else if (group_value == "ENDSEC") {
                            current_section.clear();
                            current_entity_type.clear();
                        } else if (group_value == "EOF") {
                            // 结束解析
                        } else if (current_section == "ENTITIES" || current_section == "BLOCKS") {
                            // 保存当前实体（如果有）
                            if (!current_entity.points.empty() && 
                                current_entity.type != DXFEntityType::Unknown) {
                                geometry.entities.push_back(current_entity);
                            }
                            // 开始新实体
                            current_entity = DXFEntity();
                            current_entity_type = group_value;

                            // 设置实体类型
                            if (group_value == "LINE") {
                                current_entity.type = DXFEntityType::Line;
                            } else if (group_value == "POINT") {
                                current_entity.type = DXFEntityType::Point;
                            } else if (group_value == "ARC") {
                                current_entity.type = DXFEntityType::Arc;
                            } else if (group_value == "CIRCLE") {
                                current_entity.type = DXFEntityType::Circle;
                            } else if (group_value == "POLYLINE") {
                                current_entity.type = DXFEntityType::Polyline;
                            } else if (group_value == "LWPOLYLINE") {
                                current_entity.type = DXFEntityType::LWPolyline;
                            } else if (group_value == "SPLINE") {
                                current_entity.type = DXFEntityType::Spline;
                            } else if (group_value == "ELLIPSE") {
                                current_entity.type = DXFEntityType::Ellipse;
                            } else if (group_value == "TEXT" || group_value == "MTEXT") {
                                current_entity.type = DXFEntityType::Text;
                            } else if (group_value == "3DFACE") {
                                current_entity.type = DXFEntityType::Face3D;
                            } else if (group_value == "3DSOLID") {
                                current_entity.type = DXFEntityType::Solid3D;
                            }
                        }
                        break;

                    case 2:  // 名称/段名
                        if (current_section.empty()) {
                            current_section = group_value;
                        } else if (current_entity.type != DXFEntityType::Unknown) {
                            current_entity.layer = group_value;
                        }
                        break;

                    case 8:  // 图层名
                        current_entity.layer = group_value;
                        break;

                    case 10:  // X坐标
                        try {
                            float x = std::stof(group_value);
                            if (current_entity.points.empty()) {
                                current_entity.points.push_back(Point3Df(x, 0, 0));
                            } else {
                                current_entity.points.back().x = x;
                            }
                        } catch (...) {}
                        break;

                    case 11:  // X坐标（终点）
                    case 12:
                    case 13:
                        try {
                            float x = std::stof(group_value);
                            current_entity.points.push_back(Point3Df(x, 0, 0));
                        } catch (...) {}
                        break;

                    case 20:  // Y坐标
                        try {
                            float y = std::stof(group_value);
                            if (!current_entity.points.empty()) {
                                current_entity.points.back().y = y;
                            }
                        } catch (...) {}
                        break;

                    case 21:  // Y坐标（终点）
                    case 22:
                    case 23:
                        try {
                            float y = std::stof(group_value);
                            if (current_entity.points.size() > 1) {
                                current_entity.points[current_entity.points.size() - 1].y = y;
                            }
                        } catch (...) {}
                        break;

                    case 30:  // Z坐标
                        try {
                            float z = std::stof(group_value);
                            if (!current_entity.points.empty()) {
                                current_entity.points.back().z = z;
                            }
                        } catch (...) {}
                        break;

                    case 31:  // Z坐标（终点）
                    case 32:
                    case 33:
                        try {
                            float z = std::stof(group_value);
                            if (current_entity.points.size() > 1) {
                                current_entity.points[current_entity.points.size() - 1].z = z;
                            }
                        } catch (...) {}
                        break;

                    case 40:  // 半径/大小
                        try {
                            current_entity.radius = std::stof(group_value);
                        } catch (...) {}
                        break;

                    case 50:  // 起始角度
                        try {
                            current_entity.start_angle = std::stof(group_value) * static_cast<float>(M_PI) / 180.0f;
                        } catch (...) {}
                        break;

                    case 51:  // 结束角度
                        try {
                            current_entity.end_angle = std::stof(group_value) * static_cast<float>(M_PI) / 180.0f;
                        } catch (...) {}
                        break;

                    case 62:  // 颜色索引
                        try {
                            int color_idx = std::stoi(group_value);
                            // DXF颜色索引转换（简化版）
                            if (color_idx >= 1 && color_idx <= 7) {
                                static ColorRGB standard_colors[] = {
                                    ColorRGB(255, 0, 0),     // 1 = Red
                                    ColorRGB(255, 255, 0),   // 2 = Yellow
                                    ColorRGB(0, 255, 0),     // 3 = Green
                                    ColorRGB(0, 255, 255),   // 4 = Cyan
                                    ColorRGB(0, 0, 255),     // 5 = Blue
                                    ColorRGB(255, 0, 255),   // 6 = Magenta
                                    ColorRGB(255, 255, 255)  // 7 = White
                                };
                                current_entity.color = standard_colors[color_idx - 1];
                            }
                        } catch (...) {}
                        break;

                    case 70:  // 实体标志（多段线闭合等）
                        try {
                            int flags = std::stoi(group_value);
                            // 处理闭合标志等
                        } catch (...) {}
                        break;
                }
                break;
        }
    }

    // 保存最后一个实体
    if (!current_entity.points.empty() && current_entity.type != DXFEntityType::Unknown) {
        geometry.entities.push_back(current_entity);
    }

    geometry.update_bounds();
    geometry.filename = "";

    OVF_INFO() << "DXF parsed: " << geometry.entities.size() << " entities found";
    return ErrorCode::Success;
}

ErrorCode extract_entity_edges(const DXFEntity& entity, 
                               Vector<Point3Df>& edge_points,
                               float sample_step) {
    edge_points.clear();

    switch (entity.type) {
        case DXFEntityType::Point:
            // 点实体：直接添加点
            for (const auto& pt : entity.points) {
                edge_points.push_back(pt);
            }
            break;

        case DXFEntityType::Line:
            // 线实体：采样起点到终点
            if (entity.points.size() >= 2) {
                const Point3Df& p1 = entity.points[0];
                const Point3Df& p2 = entity.points[1];
                float length = p1.distance_to(p2);
                int num_samples = static_cast<int>(length / sample_step) + 1;

                for (int i = 0; i <= num_samples; ++i) {
                    float t = static_cast<float>(i) / num_samples;
                    Point3Df pt(
                        p1.x + t * (p2.x - p1.x),
                        p1.y + t * (p2.y - p1.y),
                        p1.z + t * (p2.z - p1.z)
                    );
                    edge_points.push_back(pt);
                }
            }
            break;

        case DXFEntityType::Circle:
            // 圆实体：采样圆周
            if (!entity.points.empty()) {
                const Point3Df& center = entity.points[0];
                float r = entity.radius;
                float circumference = 2.0f * static_cast<float>(M_PI) * r;
                int num_samples = static_cast<int>(circumference / sample_step) + 1;

                for (int i = 0; i < num_samples; ++i) {
                    float angle = 2.0f * static_cast<float>(M_PI) * i / num_samples;
                    Point3Df pt(
                        center.x + r * std::cos(angle),
                        center.y + r * std::sin(angle),
                        center.z
                    );
                    edge_points.push_back(pt);
                }
            }
            break;

        case DXFEntityType::Arc:
            // 弧实体：采样弧段
            if (!entity.points.empty()) {
                const Point3Df& center = entity.points[0];
                float r = entity.radius;
                float angle_span = entity.end_angle - entity.start_angle;
                if (angle_span < 0) angle_span += 2.0f * static_cast<float>(M_PI);
                float arc_length = r * angle_span;
                int num_samples = static_cast<int>(arc_length / sample_step) + 1;

                for (int i = 0; i <= num_samples; ++i) {
                    float angle = entity.start_angle + angle_span * i / num_samples;
                    Point3Df pt(
                        center.x + r * std::cos(angle),
                        center.y + r * std::sin(angle),
                        center.z
                    );
                    edge_points.push_back(pt);
                }
            }
            break;

        case DXFEntityType::Polyline:
        case DXFEntityType::LWPolyline:
            // 多段线：依次采样每段
            for (size_t i = 0; i < entity.points.size() - 1; ++i) {
                const Point3Df& p1 = entity.points[i];
                const Point3Df& p2 = entity.points[i + 1];
                float length = p1.distance_to(p2);
                int num_samples = static_cast<int>(length / sample_step) + 1;

                for (int j = 0; j <= num_samples; ++j) {
                    float t = static_cast<float>(j) / num_samples;
                    Point3Df pt(
                        p1.x + t * (p2.x - p1.x),
                        p1.y + t * (p2.y - p1.y),
                        p1.z + t * (p2.z - p1.z)
                    );
                    edge_points.push_back(pt);
                }
            }
            break;

        case DXFEntityType::Face3D:
            // 3D面：采样四条边
            if (entity.points.size() >= 3) {
                for (size_t i = 0; i < entity.points.size(); ++i) {
                    size_t next_i = (i + 1) % entity.points.size();
                    const Point3Df& p1 = entity.points[i];
                    const Point3Df& p2 = entity.points[next_i];
                    float length = p1.distance_to(p2);
                    int num_samples = static_cast<int>(length / sample_step) + 1;

                    for (int j = 0; j <= num_samples; ++j) {
                        float t = static_cast<float>(j) / num_samples;
                        Point3Df pt(
                            p1.x + t * (p2.x - p1.x),
                            p1.y + t * (p2.y - p1.y),
                            p1.z + t * (p2.z - p1.z)
                        );
                        edge_points.push_back(pt);
                    }
                }
            }
            break;

        default:
            // 其他类型：直接复制点
            for (const auto& pt : entity.points) {
                edge_points.push_back(pt);
            }
            break;
    }

    return ErrorCode::Success;
}

ErrorCode dxf_to_pointcloud(const DXFGeometry& geometry, 
                            PointCloudData& cloud,
                            float sample_step) {
    cloud.clear();

    for (const auto& entity : geometry.entities) {
        Vector<Point3Df> entity_points;
        extract_entity_edges(entity, entity_points, sample_step);

        for (const auto& pt : entity_points) {
            cloud.add_point(pt);
        }
    }

    return ErrorCode::Success;
}

// ========== 视图生成 ==========

ErrorCode generate_views(const DXFGeometry& geometry,
                        const CameraParams& camera_params,
                        Vector<ViewTemplate>& views,
                        int num_views,
                        int image_width,
                        int image_height) {
    views.clear();

    if (geometry.empty() || !camera_params.valid()) {
        return ErrorCode::InvalidParameter;
    }

    // 计算模型中心和直径
    geometry.update_bounds();
    Point3Df center = geometry.center;
    float diameter = compute_model_diameter(geometry);
    float view_distance = diameter * 2.5f;  // 视图距离约为直径的2.5倍

    // 使用球面采样生成视图位置
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < num_views; ++i) {
        ViewTemplate view;
        view.view_index = i;

        // 球面均匀采样（使用黄金角方法）
        float phi = static_cast<float>(M_PI) * (1.0f + std::sqrt(5.0f)) / 2.0f;
        float y = 1.0f - (2.0f * i + 1.0f) / num_views;  // -1到1的均匀分布
        float radius = std::sqrt(1.0f - y * y);
        float theta = phi * i;

        // 相机位置（在球面上）
        Point3Df camera_pos(
            center.x + view_distance * radius * std::cos(theta),
            center.y + view_distance * radius * std::sin(theta),
            center.z + view_distance * y
        );

        // 计算相机姿态（看向模型中心）
        Point3Df view_dir = (center - camera_pos).normalized();
        Point3Df up(0, 0, 1);
        if (std::abs(view_dir.z) > 0.99f) {
            up = Point3Df(0, 1, 0);  // 避免奇异情况
        }

        // 构建相机坐标系
        Point3Df cam_z = view_dir;  // 相机Z轴（光轴）
        Point3Df cam_x = up.cross(cam_z).normalized();  // 相机X轴
        Point3Df cam_y = cam_z.cross(cam_x);  // 相机Y轴

        // 构建姿态矩阵（从模型坐标系到相机坐标系）
        view.pose = Transform3D();
        view.pose.m[0][0] = cam_x.x;
        view.pose.m[0][1] = cam_x.y;
        view.pose.m[0][2] = cam_x.z;
        view.pose.m[0][3] = -(cam_x.x * camera_pos.x + cam_x.y * camera_pos.y + cam_x.z * camera_pos.z);

        view.pose.m[1][0] = cam_y.x;
        view.pose.m[1][1] = cam_y.y;
        view.pose.m[1][2] = cam_y.z;
        view.pose.m[1][3] = -(cam_y.x * camera_pos.x + cam_y.y * camera_pos.y + cam_y.z * camera_pos.z);

        view.pose.m[2][0] = cam_z.x;
        view.pose.m[2][1] = cam_z.y;
        view.pose.m[2][2] = cam_z.z;
        view.pose.m[2][3] = -(cam_z.x * camera_pos.x + cam_z.y * camera_pos.y + cam_z.z * camera_pos.z);

        view.pose.m[3][0] = 0.0f;
        view.pose.m[3][1] = 0.0f;
        view.pose.m[3][2] = 0.0f;
        view.pose.m[3][3] = 1.0f;

        // 渲染几何到图像
        ErrorCode err = render_geometry(geometry, view.pose, camera_params,
                                        view.rendered_image, image_width, image_height);
        if (err != ErrorCode::Success) {
            continue;
        }

        // 提取边缘
        err = extract_edges(view.rendered_image, view.edge_image);
        if (err != ErrorCode::Success) {
            continue;
        }

        // 获取边缘点
        err = get_edge_points(view.edge_image, view.edge_points);
        if (err != ErrorCode::Success) {
            continue;
        }

        // 计算覆盖得分
        view.coverage_score = compute_view_coverage(view, geometry);

        views.push_back(view);
    }

    OVF_INFO() << "Generated " << views.size() << " view templates";
    return ErrorCode::Success;
}

ErrorCode render_geometry(const DXFGeometry& geometry,
                         const Transform3D& pose,
                         const CameraParams& camera_params,
                         ImageData& image,
                         int width,
                         int height) {
    if (!camera_params.valid()) {
        return ErrorCode::InvalidParameter;
    }

    // 创建空白图像（白色背景）
    image.width = width;
    image.height = height;
    image.channels = 1;
    image.format = ImageFormat::Mono8;
    image.data.resize(width * height, 255);  // 白色背景

    // 投影每个实体
    for (const auto& entity : geometry.entities) {
        // 获取实体边缘点
        Vector<Point3Df> edge_points;
        extract_entity_edges(entity, edge_points, 2.0f);

        // 投影并绘制
        for (size_t i = 0; i < edge_points.size() - 1; ++i) {
            Point2D<float> p1 = project_point_3d_to_2d(edge_points[i], pose, camera_params);
            Point2D<float> p2 = project_point_3d_to_2d(edge_points[i + 1], pose, camera_params);

            // 检查投影有效性
            if (p1.x < 0 || p1.y < 0 || p2.x < 0 || p2.y < 0) continue;
            if (p1.x >= width || p1.y >= height || p2.x >= width || p2.y >= height) continue;

            // 绘制线段（简化版）
            int x1 = static_cast<int>(p1.x);
            int y1 = static_cast<int>(p1.y);
            int x2 = static_cast<int>(p2.x);
            int y2 = static_cast<int>(p2.y);

            // Bresenham线段绘制
            int dx = std::abs(x2 - x1);
            int dy = std::abs(y2 - y1);
            int sx = x1 < x2 ? 1 : -1;
            int sy = y1 < y2 ? 1 : -1;
            int err = dx - dy;

            while (true) {
                if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height) {
                    image.data[y1 * width + x1] = 0;  // 黑色线条
                }

                if (x1 == x2 && y1 == y2) break;

                int e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x1 += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y1 += sy;
                }
            }
        }
    }

    return ErrorCode::Success;
}

ErrorCode extract_edges(const ImageData& image,
                        ImageData& edge_image,
                        float low_threshold,
                        float high_threshold) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    // 简化的边缘检测（使用差分方法）
    edge_image.width = image.width;
    edge_image.height = image.height;
    edge_image.channels = 1;
    edge_image.format = ImageFormat::Mono8;
    edge_image.data.resize(image.width * image.height, 0);

    int w = image.width;
    int h = image.height;
    int c = image.channels;

    // 辅助函数：获取灰度值
    auto get_pixel = [&image, w, c](int py, int px) -> uint8_t {
        int idx = py * w + px;
        if (c == 1) {
            return image.data[idx];
        } else {
            return static_cast<uint8_t>(
                (image.data[idx * c] + image.data[idx * c + 1] + image.data[idx * c + 2]) / 3);
        }
    };

    // Sobel边缘检测（简化版）
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            int gx = -get_pixel(y-1, x-1) + get_pixel(y-1, x+1)
                    -2*get_pixel(y, x-1) + 2*get_pixel(y, x+1)
                    -get_pixel(y+1, x-1) + get_pixel(y+1, x+1);

            int gy = -get_pixel(y-1, x-1) - 2*get_pixel(y-1, x) - get_pixel(y-1, x+1)
                    +get_pixel(y+1, x-1) + 2*get_pixel(y+1, x) + get_pixel(y+1, x+1);

            float magnitude = std::sqrt(static_cast<float>(gx * gx + gy * gy));

            // 阈值化
            if (magnitude > high_threshold) {
                edge_image.data[y * w + x] = 255;
            } else if (magnitude > low_threshold) {
                edge_image.data[y * w + x] = 128;
            }
        }
    }

    return ErrorCode::Success;
}

ErrorCode get_edge_points(const ImageData& edge_image,
                          Vector<Point2D<int>>& edge_points) {
    edge_points.clear();

    if (edge_image.empty()) {
        return ErrorCode::InvalidImage;
    }

    for (uint32_t y = 0; y < edge_image.height; ++y) {
        for (uint32_t x = 0; x < edge_image.width; ++x) {
            if (edge_image.data[y * edge_image.width + x] > 0) {
                edge_points.push_back(Point2D<int>(static_cast<int>(x), static_cast<int>(y)));
            }
        }
    }

    return ErrorCode::Success;
}

float compute_view_coverage(const ViewTemplate& view, const DXFGeometry& geometry) {
    if (view.edge_points.empty() || geometry.empty()) {
        return 0.0f;
    }

    // 简化的覆盖得分计算
    int edge_count = static_cast<int>(view.edge_points.size());
    int total_possible = geometry.entities.size() * 100;  // 简化估计

    float coverage = static_cast<float>(edge_count) / static_cast<float>(total_possible);
    return std::min(coverage, 1.0f);
}

// ========== 3D匹配 ==========

ErrorCode create_shape_model_3d(const DXFGeometry& geometry,
                               const CameraParams& camera_params,
                               ShapeModel3D& model,
                               int num_views) {
    model.clear();
    model.cad_geometry = geometry;
    model.camera_params = camera_params;

    // 更新几何边界
    geometry.update_bounds();
    model.cad_geometry.update_bounds();

    // 计算模型中心和直径
    model.centroid = geometry.center;
    model.model_diameter = compute_model_diameter(geometry);

    // 计算包围盒
    model.bounding_box.min_pt = geometry.min_pt;
    model.bounding_box.max_pt = geometry.max_pt;
    model.bounding_box.update();

    // 训练模型（生成视图）
    ErrorCode err = train_shape_model_3d(model, num_views);
    if (err != ErrorCode::Success) {
        return err;
    }

    return ErrorCode::Success;
}

ErrorCode train_shape_model_3d(ShapeModel3D& model,
                              int num_views,
                              int image_width,
                              int image_height) {
    // 生成视图
    ErrorCode err = generate_views(model.cad_geometry, model.camera_params,
                                   model.views, num_views, image_width, image_height);
    if (err != ErrorCode::Success) {
        return err;
    }

    // 更新模型参数
    model.min_score = 0.5f;
    model.greediness = 0.8f;
    model.num_levels = 4;

    OVF_INFO() << "3D shape model trained with " << model.views.size() << " views";
    return ErrorCode::Success;
}

ErrorCode find_shape_model_3d(const ImageData& image,
                             const ShapeModel3D& model,
                             const CameraParams& camera_params,
                             Match3DResults& results,
                             float min_score,
                             float greediness,
                             int num_matches) {
    results.clear();

    if (image.empty() || !model.valid()) {
        return ErrorCode::InvalidParameter;
    }

    // 提取输入图像的边缘
    ImageData input_edges;
    ErrorCode err = extract_edges(image, input_edges);
    if (err != ErrorCode::Success) {
        return err;
    }

    Vector<Point2D<int>> input_edge_points;
    err = get_edge_points(input_edges, input_edge_points);
    if (err != ErrorCode::Success) {
        return err;
    }

    // 对每个视图进行匹配
    Vector<Match3DResult> candidates;

    for (size_t view_idx = 0; view_idx < model.views.size(); ++view_idx) {
        const ViewTemplate& view = model.views[view_idx];

        // 提取视图边缘点
        Vector<Point2D<int>> view_edge_points;
        err = get_edge_points(view.edge_image, view_edge_points);
        if (err != ErrorCode::Success || view_edge_points.empty()) {
            continue;
        }

        // 计算边缘匹配得分（简化版）
        float score = compute_edge_match_score(input_edge_points, view_edge_points,
                                               image.width, image.height,
                                               view.rendered_image.width, view.rendered_image.height);

        if (score >= min_score) {
            Match3DResult candidate;
            candidate.score = score;
            candidate.matched_view = static_cast<int>(view_idx);
            candidate.match_index = static_cast<int>(candidates.size());

            // 从视图姿态估计初始姿态
            candidate.transformation = view.pose;

            // 提取姿态参数
            extract_pose_params(candidate.transformation,
                               candidate.x, candidate.y, candidate.z,
                               candidate.rx, candidate.ry, candidate.rz);

            candidates.push_back(candidate);
        }
    }

    // 按得分排序
    std::sort(candidates.begin(), candidates.end(),
              [](const Match3DResult& a, const Match3DResult& b) {
                  return a.score > b.score;
              });

    // 取前N个匹配
    int actual_matches = std::min(num_matches, static_cast<int>(candidates.size()));
    for (int i = 0; i < actual_matches; ++i) {
        candidates[i].match_index = i;
        results.matches.push_back(candidates[i]);
    }

    if (!results.empty()) {
        results.best_match_index = 0;
        results.best_score = results.matches[0].score;
        results.num_matches = static_cast<int>(results.matches.size());
    }

    OVF_INFO() << "3D matching found " << results.num_matches << " matches";
    return ErrorCode::Success;
}

// 边缘匹配得分计算（辅助函数）
static float compute_edge_match_score(const Vector<Point2D<int>>& input_edges,
                                      const Vector<Point2D<int>>& view_edges,
                                      int input_w, int input_h,
                                      int view_w, int view_h) {
    if (input_edges.empty() || view_edges.empty()) {
        return 0.0f;
    }

    // 简化的边缘匹配：使用距离变换的近似
    // 计算输入边缘点到视图边缘点的平均最近距离

    float total_distance = 0.0f;
    int matched_count = 0;
    float threshold = 10.0f;  // 距离阈值

    // 为了效率，采样一部分点
    int sample_size = std::min(1000, static_cast<int>(input_edges.size()));

    for (int i = 0; i < sample_size; ++i) {
        const auto& pt = input_edges[i * input_edges.size() / sample_size];

        // 在视图边缘中找最近点
        float min_dist = std::numeric_limits<float>::max();

        for (const auto& view_pt : view_edges) {
            float dx = pt.x - view_pt.x;
            float dy = pt.y - view_pt.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            min_dist = std::min(min_dist, dist);
        }

        if (min_dist < threshold) {
            total_distance += min_dist;
            matched_count++;
        }
    }

    // 计算得分
    if (matched_count == 0) {
        return 0.0f;
    }

    float avg_distance = total_distance / matched_count;
    float coverage = static_cast<float>(matched_count) / sample_size;

    // 得分 = 覆盖率 * (1 - 平均距离/阈值)
    float score = coverage * (1.0f - avg_distance / threshold);
    return std::max(0.0f, std::min(1.0f, score));
}

ErrorCode find_shape_model_3d_cluttered(const ImageData& image,
                                       const ShapeModel3D& model,
                                       const CameraParams& camera_params,
                                       Match3DResults& results,
                                       float min_score,
                                       float clutter_factor) {
    // 乱序场景匹配：降低阈值，寻找多个实例
    ErrorCode err = find_shape_model_3d(image, model, camera_params, results,
                                        min_score * (1.0f - clutter_factor), model.greediness, 10);

    // 后处理：去除重叠的匹配
    if (err == ErrorCode::Success && results.matches.size() > 1) {
        Vector<Match3DResult> filtered;
        float overlap_threshold = 0.3f * model.model_diameter;

        for (const auto& match : results.matches) {
            bool is_unique = true;
            for (const auto& existing : filtered) {
                float dist = std::sqrt(
                    (match.x - existing.x) * (match.x - existing.x) +
                    (match.y - existing.y) * (match.y - existing.y) +
                    (match.z - existing.z) * (match.z - existing.z)
                );
                if (dist < overlap_threshold) {
                    is_unique = false;
                    break;
                }
            }
            if (is_unique) {
                filtered.push_back(match);
            }
        }

        results.matches = filtered;
        results.num_matches = static_cast<int>(filtered.size());
    }

    return err;
}

ErrorCode find_shape_model_3d_multi(const ImageData& image,
                                   const Vector<ShapeModel3D>& models,
                                   const CameraParams& camera_params,
                                   Vector<Match3DResults>& results,
                                   float min_score) {
    results.clear();

    if (image.empty() || models.empty()) {
        return ErrorCode::InvalidParameter;
    }

    // 对每个模型进行匹配
    for (const auto& model : models) {
        Match3DResults model_results;
        ErrorCode err = find_shape_model_3d(image, model, camera_params,
                                            model_results, min_score, model.greediness, 5);
        if (err == ErrorCode::Success) {
            results.push_back(model_results);
        } else {
            results.push_back(Match3DResults{});  // 添加空结果
        }
    }

    return ErrorCode::Success;
}

ErrorCode estimate_pose_pnp(const Vector<Point2D<float>>& points_2d,
                           const Vector<Point3Df>& points_3d,
                           const CameraParams& camera_params,
                           Transform3D& pose) {
    if (points_2d.size() < 4 || points_3d.size() < 4 ||
        points_2d.size() != points_3d.size() || !camera_params.valid()) {
        return ErrorCode::InvalidParameter;
    }

    // 简化的PnP解法（使用DLT方法）
    // 构建方程组 Ax = b

    size_t n = points_2d.size();
    Vector<float> A(12 * n, 0.0f);
    Vector<float> b(2 * n, 0.0f);

    for (size_t i = 0; i < n; ++i) {
        float u = points_2d[i].x;
        float v = points_2d[i].y;
        float X = points_3d[i].x;
        float Y = points_3d[i].y;
        float Z = points_3d[i].z;

        float fx = camera_params.focal_length_x;
        float fy = camera_params.focal_length_y;
        float cx = camera_params.center_x;
        float cy = camera_params.center_y;

        // 归一化坐标
        float xn = (u - cx) / fx;
        float yn = (v - cy) / fy;

        // DLT方程
        size_t row1 = 2 * i;
        size_t row2 = 2 * i + 1;

        A[row1 * 12 + 0] = X;
        A[row1 * 12 + 1] = Y;
        A[row1 * 12 + 2] = Z;
        A[row1 * 12 + 3] = 1;
        A[row1 * 12 + 8] = -xn * X;
        A[row1 * 12 + 9] = -xn * Y;
        A[row1 * 12 + 10] = -xn * Z;
        A[row1 * 12 + 11] = -xn;

        A[row2 * 12 + 4] = X;
        A[row2 * 12 + 5] = Y;
        A[row2 * 12 + 6] = Z;
        A[row2 * 12 + 7] = 1;
        A[row2 * 12 + 8] = -yn * X;
        A[row2 * 12 + 9] = -yn * Y;
        A[row2 * 12 + 10] = -yn * Z;
        A[row2 * 12 + 11] = -yn;
    }

    // 简化的求解（这里使用近似方法）
    // 在实际实现中应使用SVD求解

    // 估计平移（使用质心）
    Point3Df centroid_3d(0, 0, 0);
    Point2D<float> centroid_2d(0, 0);
    for (size_t i = 0; i < n; ++i) {
        centroid_3d.x += points_3d[i].x;
        centroid_3d.y += points_3d[i].y;
        centroid_3d.z += points_3d[i].z;
        centroid_2d.x += points_2d[i].x;
        centroid_2d.y += points_2d[i].y;
    }
    centroid_3d.x /= n;
    centroid_3d.y /= n;
    centroid_3d.z /= n;
    centroid_2d.x /= n;
    centroid_2d.y /= n;

    // 估计深度（简化）
    float avg_depth = camera_params.focal_length_x * centroid_3d.x / (centroid_2d.x - camera_params.center_x);
    if (avg_depth < 0.1f) avg_depth = 500.0f;  // 默认深度

    // 设置初始姿态
    pose = build_transform(0, 0, avg_depth, 0, 0, 0);

    return ErrorCode::Success;
}

bool verify_hypothesis(const ImageData& image,
                      const ShapeModel3D& model,
                      const Transform3D& pose,
                      const CameraParams& camera_params,
                      float& verified_score) {
    // 假设验证：渲染模型并与图像比较
    ImageData rendered;
    ErrorCode err = render_geometry(model.cad_geometry, pose, camera_params,
                                    rendered, image.width, image.height);
    if (err != ErrorCode::Success) {
        verified_score = 0.0f;
        return false;
    }

    // 提取两者的边缘
    ImageData input_edges, rendered_edges;
    extract_edges(image, input_edges);
    extract_edges(rendered, rendered_edges);

    // 计算边缘匹配得分
    Vector<Point2D<int>> input_pts, render_pts;
    get_edge_points(input_edges, input_pts);
    get_edge_points(rendered_edges, render_pts);

    verified_score = compute_edge_match_score(input_pts, render_pts,
                                               image.width, image.height,
                                               image.width, image.height);

    return verified_score > model.min_score;
}

// ========== 优化 ==========

ErrorCode refine_match_3d(const Vector<Point2D<float>>& matched_points,
                         const Vector<Point3Df>& model_points,
                         Transform3D& pose,
                         const CameraParams& camera_params,
                         int max_iterations,
                         float tolerance) {
    if (matched_points.size() < 4 || model_points.size() < 4 ||
        matched_points.size() != model_points.size()) {
        return ErrorCode::InvalidParameter;
    }

    // 简化的ICP优化
    float prev_error = std::numeric_limits<float>::max();

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 投影模型点
        Vector<Point2D<float>> projected;
        for (const auto& pt : model_points) {
            projected.push_back(project_point_3d_to_2d(pt, pose, camera_params));
        }

        // 计算重投影误差
        float total_error = 0.0f;
        for (size_t i = 0; i < matched_points.size(); ++i) {
            float dx = matched_points[i].x - projected[i].x;
            float dy = matched_points[i].y - projected[i].y;
            total_error += dx * dx + dy * dy;
        }
        float avg_error = total_error / matched_points.size();

        // 检查收敛
        if (std::abs(prev_error - avg_error) < tolerance) {
            break;
        }
        prev_error = avg_error;

        // 更新姿态（简化：调整平移）
        float dx = 0, dy = 0, dz = 0;
        for (size_t i = 0; i < matched_points.size(); ++i) {
            dx += (matched_points[i].x - projected[i].x) / camera_params.focal_length_x;
            dy += (matched_points[i].y - projected[i].y) / camera_params.focal_length_y;
        }
        dx /= matched_points.size();
        dy /= matched_points.size();

        // 更新平移
        pose.m[0][3] += dx * pose.m[2][3] * 0.1f;
        pose.m[1][3] += dy * pose.m[2][3] * 0.1f;
    }

    return ErrorCode::Success;
}

ErrorCode refine_single_match(const ImageData& image,
                             const ShapeModel3D& model,
                             Match3DResult& result,
                             const CameraParams& camera_params,
                             int max_iterations) {
    // 获取匹配的视图
    if (result.matched_view >= static_cast<int>(model.views.size())) {
        return ErrorCode::InvalidParameter;
    }

    const ViewTemplate& view = model.views[result.matched_view];

    // 提取图像边缘点
    ImageData edge_image;
    extract_edges(image, edge_image);
    Vector<Point2D<int>> edge_points;
    get_edge_points(edge_image, edge_points);

    // 构建匹配点对
    Vector<Point2D<float>> matched_2d;
    Vector<Point3Df> matched_3d;

    // 简化：使用视图边缘点
    for (const auto& pt : edge_points) {
        matched_2d.push_back(Point2D<float>(static_cast<float>(pt.x), static_cast<float>(pt.y)));
    }

    // 对应的3D点（从视图模板）
    for (const auto& pt : view.correspondences) {
        matched_3d.push_back(pt);
    }

    // 确保数量匹配
    size_t num_points = std::min(matched_2d.size(), matched_3d.size());
    if (num_points < 4) {
        return ErrorCode::InvalidParameter;
    }

    matched_2d.resize(num_points);
    matched_3d.resize(num_points);

    // 优化
    ErrorCode err = refine_match_3d(matched_2d, matched_3d, result.transformation,
                                   camera_params, max_iterations);

    if (err == ErrorCode::Success) {
        extract_pose_params(result.transformation,
                           result.x, result.y, result.z,
                           result.rx, result.ry, result.rz);
    }

    return err;
}

// ========== 投影和变换 ==========

ErrorCode project_model_3d(const ShapeModel3D& model,
                          const Transform3D& pose,
                          const CameraParams& camera_params,
                          ImageData& image,
                          int width,
                          int height) {
    return render_geometry(model.cad_geometry, pose, camera_params, image, width, height);
}

ErrorCode transform_model_3d(const ShapeModel3D& model,
                            const Transform3D& pose,
                            ShapeModel3D& transformed_model) {
    transformed_model = model;

    // 变换CAD几何中的所有点
    for (auto& entity : transformed_model.cad_geometry.entities) {
        for (auto& pt : entity.points) {
            pt = pose.transform(pt);
        }
    }

    // 更新边界
    transformed_model.cad_geometry.update_bounds();
    transformed_model.centroid = transformed_model.cad_geometry.center;
    transformed_model.bounding_box.min_pt = transformed_model.cad_geometry.min_pt;
    transformed_model.bounding_box.max_pt = transformed_model.cad_geometry.max_pt;
    transformed_model.bounding_box.update();

    return ErrorCode::Success;
}

// ========== 序列化 ==========

ErrorCode serialize_shape_model_3d(const ShapeModel3D& model, const String& filepath) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return ErrorCode::FileOpenFailed;
    }

    ByteArray data;
    ErrorCode err = serialize_model_to_bytes(model, data);
    if (err != ErrorCode::Success) {
        return err;
    }

    file.write(reinterpret_cast<const char*>(data.data()), data.size());
    file.close();

    return ErrorCode::Success;
}

ErrorCode deserialize_shape_model_3d(const String& filepath, ShapeModel3D& model) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return ErrorCode::FileOpenFailed;
    }

    ByteArray data((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());
    file.close();

    return deserialize_model_from_bytes(data, model);
}

ErrorCode serialize_model_to_bytes(const ShapeModel3D& model, ByteArray& data) {
    data.clear();

    // 简化的序列化（写入关键参数）
    // 格式：版本号(4) + 实体数(4) + 视图数(4) + ...

    uint32_t version = 1;
    uint32_t num_entities = static_cast<uint32_t>(model.cad_geometry.entities.size());
    uint32_t num_views = static_cast<uint32_t>(model.views.size());

    // 写入版本和计数
    data.insert(data.end(), reinterpret_cast<uint8_t*>(&version), reinterpret_cast<uint8_t*>(&version) + 4);
    data.insert(data.end(), reinterpret_cast<uint8_t*>(&num_entities), reinterpret_cast<uint8_t*>(&num_entities) + 4);
    data.insert(data.end(), reinterpret_cast<uint8_t*>(&num_views), reinterpret_cast<uint8_t*>(&num_views) + 4);

    // 写入模型参数
    float params[] = {
        model.min_score, model.greediness, static_cast<float>(model.num_levels),
        model.angle_step, model.distance_step,
        model.min_angle, model.max_angle, model.min_distance, model.max_distance,
        model.centroid.x, model.centroid.y, model.centroid.z,
        model.model_diameter
    };
    data.insert(data.end(), reinterpret_cast<uint8_t*>(params), reinterpret_cast<uint8_t*>(params) + sizeof(params));

    // 写入相机参数
    float cam_params[] = {
        model.camera_params.focal_length_x, model.camera_params.focal_length_y,
        model.camera_params.center_x, model.camera_params.center_y
    };
    data.insert(data.end(), reinterpret_cast<uint8_t*>(cam_params), reinterpret_cast<uint8_t*>(cam_params) + sizeof(cam_params));

    // 写入实体数据（简化）
    for (const auto& entity : model.cad_geometry.entities) {
        uint32_t type = static_cast<uint32_t>(entity.type);
        uint32_t num_points = static_cast<uint32_t>(entity.points.size());
        float radius = entity.radius;

        data.insert(data.end(), reinterpret_cast<uint8_t*>(&type), reinterpret_cast<uint8_t*>(&type) + 4);
        data.insert(data.end(), reinterpret_cast<uint8_t*>(&num_points), reinterpret_cast<uint8_t*>(&num_points) + 4);
        data.insert(data.end(), reinterpret_cast<uint8_t*>(&radius), reinterpret_cast<uint8_t*>(&radius) + 4);

        for (const auto& pt : entity.points) {
            float coords[] = {pt.x, pt.y, pt.z};
            data.insert(data.end(), reinterpret_cast<uint8_t*>(coords), reinterpret_cast<uint8_t*>(coords) + sizeof(coords));
        }
    }

    return ErrorCode::Success;
}

ErrorCode deserialize_model_from_bytes(const ByteArray& data, ShapeModel3D& model) {
    model.clear();

    if (data.size() < 12) {
        return ErrorCode::FileParseFailed;
    }

    size_t offset = 0;

    // 读取版本和计数
    uint32_t version = *reinterpret_cast<const uint32_t*>(data.data() + offset);
    offset += 4;

    uint32_t num_entities = *reinterpret_cast<const uint32_t*>(data.data() + offset);
    offset += 4;

    uint32_t num_views = *reinterpret_cast<const uint32_t*>(data.data() + offset);
    offset += 4;

    // 读取模型参数
    const float* params = reinterpret_cast<const float*>(data.data() + offset);
    model.min_score = params[0];
    model.greediness = params[1];
    model.num_levels = static_cast<int>(params[2]);
    model.angle_step = params[3];
    model.distance_step = params[4];
    model.min_angle = params[5];
    model.max_angle = params[6];
    model.min_distance = params[7];
    model.max_distance = params[8];
    model.centroid.x = params[9];
    model.centroid.y = params[10];
    model.centroid.z = params[11];
    model.model_diameter = params[12];
    offset += 13 * sizeof(float);

    // 读取相机参数
    const float* cam_params = reinterpret_cast<const float*>(data.data() + offset);
    model.camera_params.focal_length_x = cam_params[0];
    model.camera_params.focal_length_y = cam_params[1];
    model.camera_params.center_x = cam_params[2];
    model.camera_params.center_y = cam_params[3];
    offset += 4 * sizeof(float);

    // 读取实体数据
    for (uint32_t i = 0; i < num_entities; ++i) {
        DXFEntity entity;

        uint32_t type = *reinterpret_cast<const uint32_t*>(data.data() + offset);
        offset += 4;
        entity.type = static_cast<DXFEntityType>(type);

        uint32_t num_points = *reinterpret_cast<const uint32_t*>(data.data() + offset);
        offset += 4;

        entity.radius = *reinterpret_cast<const float*>(data.data() + offset);
        offset += 4;

        for (uint32_t j = 0; j < num_points; ++j) {
            const float* coords = reinterpret_cast<const float*>(data.data() + offset);
            entity.points.push_back(Point3Df(coords[0], coords[1], coords[2]));
            offset += 3 * sizeof(float);
        }

        model.cad_geometry.entities.push_back(entity);
    }

    model.cad_geometry.update_bounds();

    return ErrorCode::Success;
}

// ========== 可视化 ==========

ErrorCode visualize_match_3d(const ImageData& image,
                            const ShapeModel3D& model,
                            const Match3DResult& result,
                            const CameraParams& camera_params,
                            ImageData& output_image) {
    output_image = image;

    // 在图像上绘制投影轮廓
    return draw_projected_contour(output_image, model.cad_geometry,
                                 result.transformation, camera_params,
                                 ColorRGB(0, 255, 0), 2);
}

ErrorCode draw_projected_contour(ImageData& image,
                                const DXFGeometry& geometry,
                                const Transform3D& pose,
                                const CameraParams& camera_params,
                                const ColorRGB& color,
                                int thickness) {
    if (image.empty()) {
        return ErrorCode::InvalidImage;
    }

    // 遍历实体并绘制
    for (const auto& entity : geometry.entities) {
        Vector<Point3Df> edge_points;
        extract_entity_edges(entity, edge_points, 2.0f);

        // 投影并绘制
        for (size_t i = 0; i < edge_points.size(); ++i) {
            Point2D<float> p1 = project_point_3d_to_2d(edge_points[i], pose, camera_params);

            size_t next_i = (i + 1) % edge_points.size();
            Point2D<float> p2 = project_point_3d_to_2d(edge_points[next_i], pose, camera_params);

            // 绘制线段
            if (image.channels >= 3) {
                int x1 = static_cast<int>(p1.x);
                int y1 = static_cast<int>(p1.y);
                int x2 = static_cast<int>(p2.x);
                int y2 = static_cast<int>(p2.y);

                // 简化绘制（这里可以调用image_utils的绘图函数）
                if (x1 >= 0 && x1 < image.width && y1 >= 0 && y1 < image.height) {
                    size_t idx = (y1 * image.width + x1) * image.channels;
                    image.data[idx] = color.r;
                    if (image.channels >= 2) image.data[idx + 1] = color.g;
                    if (image.channels >= 3) image.data[idx + 2] = color.b;
                }
            }
        }
    }

    return ErrorCode::Success;
}

} // namespace matching_3d_utils

// ============================================================================
// 节点实现
// ============================================================================

// ========== DXFLoaderNode ==========

DXFLoaderNode::DXFLoaderNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo DXFLoaderNode::make_info() {
    NodeInfo info;
    info.id = "DXFLoader";
    info.name = "DXF文件加载";
    info.category = "3D模型处理";
    info.description = "加载并解析DXF CAD模型文件";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("filepath", "DXF文件路径", DataType::String, false));

    info.outputs.push_back(DataPort("geometry", "DXF几何数据", DataType::Any));
    info.outputs.push_back(DataPort("num_entities", "实体数量", DataType::Number));
    info.outputs.push_back(DataPort("center_x", "中心X", DataType::Number));
    info.outputs.push_back(DataPort("center_y", "中心Y", DataType::Number));
    info.outputs.push_back(DataPort("center_z", "中心Z", DataType::Number));

    info.params.push_back(ParamDef("filepath", "文件路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("scale", "缩放比例", DataType::Number, Data(1.0f)));
    info.params.push_back(ParamDef("layer_filter", "图层过滤", DataType::String, Data("")));
    info.params.push_back(ParamDef("extract_edges", "提取边缘", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("edge_sample_step", "边缘采样步长", DataType::Number, Data(1.0f)));

    return info;
}

Result<void> DXFLoaderNode::execute(FlowContext& context) {
    filepath_ = get_param("filepath", Data("")).as_string();
    scale_ = static_cast<float>(get_param("scale", Data(1.0f)).as_number());
    layer_filter_ = get_param("layer_filter", Data("")).as_string();
    extract_edges_ = get_param("extract_edges", Data(true)).as_bool();
    edge_sample_step_ = static_cast<float>(get_param("edge_sample_step", Data(1.0f)).as_number());

    // 也可以从输入端口获取文件路径
    if (has_input("filepath")) {
        filepath_ = get_input("filepath").as_string();
    }

    if (filepath_.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "DXF file path is empty");
    }

    DXFGeometry geometry;
    ErrorCode err = matching_3d_utils::parse_dxf(filepath_, geometry);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Failed to parse DXF file");
    }

    // 应用缩放
    if (scale_ != 1.0f) {
        for (auto& entity : geometry.entities) {
            for (auto& pt : entity.points) {
                pt.x *= scale_;
                pt.y *= scale_;
                pt.z *= scale_;
            }
            entity.radius *= scale_;
        }
        geometry.scale = scale_;
        geometry.update_bounds();
    }

    // 应用图层过滤
    if (!layer_filter_.empty()) {
        Vector<DXFEntity> filtered;
        for (const auto& entity : geometry.entities) {
            if (entity.layer.empty() || entity.layer == layer_filter_) {
                filtered.push_back(entity);
            }
        }
        geometry.entities = filtered;
        geometry.update_bounds();
    }

    // 设置输出
    set_output("geometry", Data("DXFGeometry[" + std::to_string(geometry.entities.size()) + " entities]"));
    set_output("num_entities", Data(static_cast<int>(geometry.entities.size())));
    set_output("center_x", Data(geometry.center.x));
    set_output("center_y", Data(geometry.center.y));
    set_output("center_z", Data(geometry.center.z));

    OVF_INFO() << "DXF loaded: " << geometry.entities.size() << " entities from " << filepath_;

    return Result<void>::success();
}

// ========== CreateShapeModel3DNode ==========

CreateShapeModel3DNode::CreateShapeModel3DNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo CreateShapeModel3DNode::make_info() {
    NodeInfo info;
    info.id = "CreateShapeModel3D";
    info.name = "创建3D形状模型";
    info.category = "3D模型处理";
    info.description = "从DXF几何创建3D形状匹配模型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("geometry", "DXF几何数据", DataType::Any, true));
    info.inputs.push_back(DataPort("cam_param_fx", "焦距X", DataType::Number, false));
    info.inputs.push_back(DataPort("cam_param_fy", "焦距Y", DataType::Number, false));
    info.inputs.push_back(DataPort("cam_param_cx", "光心X", DataType::Number, false));
    info.inputs.push_back(DataPort("cam_param_cy", "光心Y", DataType::Number, false));

    info.outputs.push_back(DataPort("model", "3D形状模型", DataType::Any));
    info.outputs.push_back(DataPort("num_views", "视图数量", DataType::Number));
    info.outputs.push_back(DataPort("model_diameter", "模型直径", DataType::Number));

    info.params.push_back(ParamDef("num_views", "视图数量", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("image_width", "图像宽度", DataType::Number, Data(640)));
    info.params.push_back(ParamDef("image_height", "图像高度", DataType::Number, Data(480)));
    info.params.push_back(ParamDef("min_score", "最小得分", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("greediness", "贪婪度", DataType::Number, Data(0.8f)));

    return info;
}

Result<void> CreateShapeModel3DNode::execute(FlowContext& context) {
    num_views_ = static_cast<int>(get_param("num_views", Data(20)).as_int());
    image_width_ = static_cast<int>(get_param("image_width", Data(640)).as_int());
    image_height_ = static_cast<int>(get_param("image_height", Data(480)).as_int());
    min_score_ = static_cast<float>(get_param("min_score", Data(0.5f)).as_number());
    greediness_ = static_cast<float>(get_param("greediness", Data(0.8f)).as_number());

    // 获取相机参数
    CameraParams camera_params;
    camera_params.focal_length_x = static_cast<float>(get_param("cam_param_fx", has_input("cam_param_fx") ? get_input("cam_param_fx") : Data(1000.0f)).as_number());
    camera_params.focal_length_y = static_cast<float>(get_param("cam_param_fy", has_input("cam_param_fy") ? get_input("cam_param_fy") : Data(1000.0f)).as_number());
    camera_params.center_x = static_cast<float>(get_param("cam_param_cx", has_input("cam_param_cx") ? get_input("cam_param_cx") : Data(320.0f)).as_number());
    camera_params.center_y = static_cast<float>(get_param("cam_param_cy", has_input("cam_param_cy") ? get_input("cam_param_cy") : Data(240.0f)).as_number());

    if (!has_input("geometry")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No DXF geometry input");
    }

    // 从输入获取几何数据（这里需要自定义类型支持）
    // 简化：假设从文件路径重新加载
    DXFGeometry geometry;
    auto geom_data = get_input("geometry");
    // 实际实现需要正确的类型转换

    ShapeModel3D model;
    ErrorCode err = matching_3d_utils::create_shape_model_3d(geometry, camera_params, model, num_views_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Failed to create 3D shape model");
    }

    model.min_score = min_score_;
    model.greediness = greediness_;

    // 设置输出
    set_output("model", Data("ShapeModel3D[" + std::to_string(model.views.size()) + " views]"));
    set_output("num_views", Data(static_cast<int>(model.views.size())));
    set_output("model_diameter", Data(model.model_diameter));

    OVF_INFO() << "3D shape model created: " << model.views.size() << " views";

    return Result<void>::success();
}

// ========== ShapeModel3DTrainNode ==========

ShapeModel3DTrainNode::ShapeModel3DTrainNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ShapeModel3DTrainNode::make_info() {
    NodeInfo info;
    info.id = "ShapeModel3DTrain";
    info.name = "3D模型训练";
    info.category = "3D模型处理";
    info.description = "训练3D形状模型，生成多视角模板";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, true));

    info.outputs.push_back(DataPort("model", "训练后的模型", DataType::Any));
    info.outputs.push_back(DataPort("num_views", "视图数量", DataType::Number));

    info.params.push_back(ParamDef("num_views", "视图数量", DataType::Number, Data(20)));
    info.params.push_back(ParamDef("image_width", "图像宽度", DataType::Number, Data(640)));
    info.params.push_back(ParamDef("image_height", "图像高度", DataType::Number, Data(480)));
    info.params.push_back(ParamDef("use_sphere_sampling", "球面采样", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("view_distance", "视图距离", DataType::Number, Data(500.0f)));

    return info;
}

Result<void> ShapeModel3DTrainNode::execute(FlowContext& context) {
    num_views_ = static_cast<int>(get_param("num_views", Data(20)).as_int());
    image_width_ = static_cast<int>(get_param("image_width", Data(640)).as_int());
    image_height_ = static_cast<int>(get_param("image_height", Data(480)).as_int());
    use_sphere_sampling_ = get_param("use_sphere_sampling", Data(true)).as_bool();
    view_distance_ = static_cast<float>(get_param("view_distance", Data(500.0f)).as_number());

    if (!has_input("model")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No model input");
    }

    // 获取模型并训练
    ShapeModel3D model;
    // 从输入获取模型（需要类型转换支持）

    ErrorCode err = matching_3d_utils::train_shape_model_3d(model, num_views_, image_width_, image_height_);
    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Failed to train 3D model");
    }

    set_output("model", Data("ShapeModel3D[" + std::to_string(model.views.size()) + " views]"));
    set_output("num_views", Data(static_cast<int>(model.views.size())));

    return Result<void>::success();
}

// ========== ShapeModel3DSerializeNode ==========

ShapeModel3DSerializeNode::ShapeModel3DSerializeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ShapeModel3DSerializeNode::make_info() {
    NodeInfo info;
    info.id = "ShapeModel3DSerialize";
    info.name = "3D模型序列化";
    info.category = "3D模型处理";
    info.description = "保存或加载3D形状模型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "3D模型（保存时）", DataType::Any, false));

    info.outputs.push_back(DataPort("model", "3D模型（加载时）", DataType::Any));
    info.outputs.push_back(DataPort("success", "操作成功", DataType::Boolean));

    info.params.push_back(ParamDef("filepath", "文件路径", DataType::String, Data("")));
    info.params.push_back(ParamDef("mode", "模式（save/load）", DataType::String, Data("save")));

    return info;
}

Result<void> ShapeModel3DSerializeNode::execute(FlowContext& context) {
    filepath_ = get_param("filepath", Data("")).as_string();
    String mode = get_param("mode", Data("save")).as_string();
    save_mode_ = (mode == "save");

    if (filepath_.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "File path is empty");
    }

    ErrorCode err;

    if (save_mode_) {
        // 保存模型
        if (!has_input("model")) {
            return Result<void>::failure(ErrorCode::InvalidParameter, "No model to save");
        }

        ShapeModel3D model;
        // 从输入获取模型

        err = matching_3d_utils::serialize_shape_model_3d(model, filepath_);
        set_output("success", Data(err == ErrorCode::Success));
    } else {
        // 加载模型
        ShapeModel3D model;
        err = matching_3d_utils::deserialize_shape_model_3d(filepath_, model);

        if (err == ErrorCode::Success) {
            set_output("model", Data("ShapeModel3D[" + std::to_string(model.views.size()) + " views]"));
        }
        set_output("success", Data(err == ErrorCode::Success));
    }

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Serialization failed");
    }

    return Result<void>::success();
}

// ========== FindShapeModel3DNode ==========

FindShapeModel3DNode::FindShapeModel3DNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FindShapeModel3DNode::make_info() {
    NodeInfo info;
    info.id = "FindShapeModel3D";
    info.name = "3D形状匹配";
    info.category = "3D匹配执行";
    info.description = "在图像中查找3D模型（6自由度匹配）";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, true));
    info.inputs.push_back(DataPort("cam_param_fx", "焦距X", DataType::Number, false));
    info.inputs.push_back(DataPort("cam_param_fy", "焦距Y", DataType::Number, false));
    info.inputs.push_back(DataPort("cam_param_cx", "光心X", DataType::Number, false));
    info.inputs.push_back(DataPort("cam_param_cy", "光心Y", DataType::Number, false));

    info.outputs.push_back(DataPort("results", "匹配结果", DataType::Any));
    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));
    info.outputs.push_back(DataPort("best_x", "最佳X位置", DataType::Number));
    info.outputs.push_back(DataPort("best_y", "最佳Y位置", DataType::Number));
    info.outputs.push_back(DataPort("best_z", "最佳Z位置", DataType::Number));
    info.outputs.push_back(DataPort("best_score", "最佳得分", DataType::Number));

    info.params.push_back(ParamDef("min_score", "最小得分", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("greediness", "贪婪度", DataType::Number, Data(0.8f)));
    info.params.push_back(ParamDef("num_matches", "匹配数量", DataType::Number, Data(1)));
    info.params.push_back(ParamDef("use_verification", "使用验证", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("use_refinement", "使用优化", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("refinement_iterations", "优化迭代次数", DataType::Number, Data(50)));

    return info;
}

Result<void> FindShapeModel3DNode::execute(FlowContext& context) {
    min_score_ = static_cast<float>(get_param("min_score", Data(0.5f)).as_number());
    greediness_ = static_cast<float>(get_param("greediness", Data(0.8f)).as_number());
    num_matches_ = static_cast<int>(get_param("num_matches", Data(1)).as_int());
    use_verification_ = get_param("use_verification", Data(true)).as_bool();
    use_refinement_ = get_param("use_refinement", Data(true)).as_bool();
    refinement_iterations_ = static_cast<int>(get_param("refinement_iterations", Data(50)).as_int());

    // 检查输入
    if (!has_input("image")) {
        return Result<void>::failure(ErrorCode::InvalidImage, "No input image");
    }

    ImageData image = get_input("image").as_image();
    if (image.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }

    if (!has_input("model")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No 3D model");
    }

    // 获取相机参数
    CameraParams camera_params;
    camera_params.focal_length_x = static_cast<float>(get_param("cam_param_fx", has_input("cam_param_fx") ? get_input("cam_param_fx") : Data(1000.0f)).as_number());
    camera_params.focal_length_y = static_cast<float>(get_param("cam_param_fy", has_input("cam_param_fy") ? get_input("cam_param_fy") : Data(1000.0f)).as_number());
    camera_params.center_x = static_cast<float>(get_param("cam_param_cx", has_input("cam_param_cx") ? get_input("cam_param_cx") : Data(image.width / 2.0f)).as_number());
    camera_params.center_y = static_cast<float>(get_param("cam_param_cy", has_input("cam_param_cy") ? get_input("cam_param_cy") : Data(image.height / 2.0f)).as_number());

    // 获取模型
    ShapeModel3D model;
    // 需要类型转换

    // 执行匹配
    Match3DResults results;
    ErrorCode err = matching_3d_utils::find_shape_model_3d(image, model, camera_params,
                                                           results, min_score_, greediness_, num_matches_);

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "3D matching failed");
    }

    // 假设验证和优化
    if (use_verification_ || use_refinement_) {
        for (auto& match : results.matches) {
            if (use_verification_) {
                float verified_score;
                match.verified = matching_3d_utils::verify_hypothesis(image, model,
                                                                      match.transformation, camera_params,
                                                                      verified_score);
                if (match.verified) {
                    match.score = verified_score;
                }
            }

            if (use_refinement_) {
                matching_3d_utils::refine_single_match(image, model, match,
                                                       camera_params, refinement_iterations_);
            }
        }
    }

    // 设置输出
    set_output("results", Data("Match3DResults[" + std::to_string(results.num_matches) + " matches]"));
    set_output("num_matches", Data(results.num_matches));

    if (!results.empty()) {
        const auto& best = results.best();
        set_output("best_x", Data(best.x));
        set_output("best_y", Data(best.y));
        set_output("best_z", Data(best.z));
        set_output("best_score", Data(best.score));
    }

    OVF_INFO() << "3D matching: found " << results.num_matches << " matches";

    return Result<void>::success();
}

// ========== FindShapeModel3DClutteredNode ==========

FindShapeModel3DClutteredNode::FindShapeModel3DClutteredNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FindShapeModel3DClutteredNode::make_info() {
    NodeInfo info;
    info.id = "FindShapeModel3DCluttered";
    info.name = "乱序3D匹配";
    info.category = "3D匹配执行";
    info.description = "在乱序场景中查找多个3D模型实例";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, true));

    info.outputs.push_back(DataPort("results", "匹配结果", DataType::Any));
    info.outputs.push_back(DataPort("num_matches", "匹配数量", DataType::Number));

    info.params.push_back(ParamDef("min_score", "最小得分", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("clutter_factor", "乱序因子", DataType::Number, Data(0.3f)));
    info.params.push_back(ParamDef("num_matches", "最大匹配数", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("overlap_threshold", "重叠阈值", DataType::Number, Data(0.1f)));

    return info;
}

Result<void> FindShapeModel3DClutteredNode::execute(FlowContext& context) {
    min_score_ = static_cast<float>(get_param("min_score", Data(0.5f)).as_number());
    clutter_factor_ = static_cast<float>(get_param("clutter_factor", Data(0.3f)).as_number());
    num_matches_ = static_cast<int>(get_param("num_matches", Data(5)).as_int());
    overlap_threshold_ = static_cast<float>(get_param("overlap_threshold", Data(0.1f)).as_number());

    if (!has_input("image") || !has_input("model")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Missing inputs");
    }

    ImageData image = get_input("image").as_image();
    ShapeModel3D model;
    CameraParams camera_params;

    Match3DResults results;
    ErrorCode err = matching_3d_utils::find_shape_model_3d_cluttered(image, model,
                                                                     camera_params, results,
                                                                     min_score_, clutter_factor_);

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Cluttered 3D matching failed");
    }

    set_output("results", Data("Match3DResults[" + std::to_string(results.num_matches) + " matches]"));
    set_output("num_matches", Data(results.num_matches));

    return Result<void>::success();
}

// ========== FindShapeModel3DMultiNode ==========

FindShapeModel3DMultiNode::FindShapeModel3DMultiNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FindShapeModel3DMultiNode::make_info() {
    NodeInfo info;
    info.id = "FindShapeModel3DMulti";
    info.name = "多3D模型匹配";
    info.category = "3D匹配执行";
    info.description = "同时匹配多个不同的3D模型";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("models", "3D模型列表", DataType::Array, true));

    info.outputs.push_back(DataPort("results", "各模型匹配结果", DataType::Array));
    info.outputs.push_back(DataPort("total_matches", "总匹配数", DataType::Number));

    info.params.push_back(ParamDef("min_score", "最小得分", DataType::Number, Data(0.5f)));
    info.params.push_back(ParamDef("max_matches_per_model", "每模型最大匹配数", DataType::Number, Data(3)));

    return info;
}

Result<void> FindShapeModel3DMultiNode::execute(FlowContext& context) {
    min_score_ = static_cast<float>(get_param("min_score", Data(0.5f)).as_number());
    max_matches_per_model_ = static_cast<int>(get_param("max_matches_per_model", Data(3)).as_int());

    if (!has_input("image") || !has_input("models")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Missing inputs");
    }

    ImageData image = get_input("image").as_image();
    Vector<ShapeModel3D> models;
    CameraParams camera_params;

    Vector<Match3DResults> results;
    ErrorCode err = matching_3d_utils::find_shape_model_3d_multi(image, models,
                                                                  camera_params, results,
                                                                  min_score_);

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Multi-model 3D matching failed");
    }

    int total = 0;
    for (const auto& r : results) {
        total += r.num_matches;
    }

    set_output("results", Data("Match3DResults[" + std::to_string(total) + " total matches]"));
    set_output("total_matches", Data(total));

    return Result<void>::success();
}

// ========== Match3DRefineNode ==========

Match3DRefineNode::Match3DRefineNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo Match3DRefineNode::make_info() {
    NodeInfo info;
    info.id = "Match3DRefine";
    info.name = "3D匹配优化";
    info.category = "3D匹配执行";
    info.description = "使用ICP等方法优化3D匹配结果";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, true));
    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, true));
    info.inputs.push_back(DataPort("result", "初始匹配结果", DataType::Any, true));

    info.outputs.push_back(DataPort("result", "优化后的匹配结果", DataType::Any));
    info.outputs.push_back(DataPort("improved", "是否改进", DataType::Boolean));

    info.params.push_back(ParamDef("max_iterations", "最大迭代次数", DataType::Number, Data(50)));
    info.params.push_back(ParamDef("tolerance", "收敛容差", DataType::Number, Data(0.001f)));
    info.params.push_back(ParamDef("use_icp", "使用ICP", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("use_boundary_optimization", "边界优化", DataType::Boolean, Data(true)));

    return info;
}

Result<void> Match3DRefineNode::execute(FlowContext& context) {
    max_iterations_ = static_cast<int>(get_param("max_iterations", Data(50)).as_int());
    tolerance_ = static_cast<float>(get_param("tolerance", Data(0.001f)).as_number());
    use_icp_ = get_param("use_icp", Data(true)).as_bool();
    use_boundary_optimization_ = get_param("use_boundary_optimization", Data(true)).as_bool();

    if (!has_input("image") || !has_input("model") || !has_input("result")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "Missing inputs");
    }

    ImageData image = get_input("image").as_image();
    ShapeModel3D model;
    Match3DResult result;
    CameraParams camera_params;

    float prev_score = result.score;

    ErrorCode err = matching_3d_utils::refine_single_match(image, model, result,
                                                           camera_params, max_iterations_);

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Refinement failed");
    }

    bool improved = result.score > prev_score;

    set_output("result", Data("Match3DResult[score=" + std::to_string(result.score) + "]"));
    set_output("improved", Data(improved));

    return Result<void>::success();
}

// ========== GetShapeModel3DMatchesNode ==========

GetShapeModel3DMatchesNode::GetShapeModel3DMatchesNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo GetShapeModel3DMatchesNode::make_info() {
    NodeInfo info;
    info.id = "GetShapeModel3DMatches";
    info.name = "获取匹配结果";
    info.category = "3D匹配结果";
    info.description = "从匹配结果集合中获取单个结果";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("results", "匹配结果集合", DataType::Any, true));

    info.outputs.push_back(DataPort("x", "位置X", DataType::Number));
    info.outputs.push_back(DataPort("y", "位置Y", DataType::Number));
    info.outputs.push_back(DataPort("z", "位置Z", DataType::Number));
    info.outputs.push_back(DataPort("rx", "旋转X", DataType::Number));
    info.outputs.push_back(DataPort("ry", "旋转Y", DataType::Number));
    info.outputs.push_back(DataPort("rz", "旋转Z", DataType::Number));
    info.outputs.push_back(DataPort("score", "得分", DataType::Number));
    info.outputs.push_back(DataPort("pose", "位姿", DataType::Pose));

    info.params.push_back(ParamDef("match_index", "匹配索引", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("get_best", "获取最佳", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("output_pose", "输出Pose", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("output_matrix", "输出矩阵", DataType::Boolean, Data(true)));

    return info;
}

Result<void> GetShapeModel3DMatchesNode::execute(FlowContext& context) {
    match_index_ = static_cast<int>(get_param("match_index", Data(0)).as_int());
    get_best_ = get_param("get_best", Data(true)).as_bool();
    output_pose_ = get_param("output_pose", Data(true)).as_bool();
    output_matrix_ = get_param("output_matrix", Data(true)).as_bool();

    if (!has_input("results")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No results input");
    }

    Match3DResults results;
    // 从输入获取

    if (results.empty()) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No matches found");
    }

    Match3DResult match;
    if (get_best_) {
        match = results.best();
    } else {
        if (match_index_ >= results.num_matches) {
            return Result<void>::failure(ErrorCode::OutOfRange, "Match index out of range");
        }
        match = results.matches[match_index_];
    }

    // 设置输出
    set_output("x", Data(match.x));
    set_output("y", Data(match.y));
    set_output("z", Data(match.z));
    set_output("rx", Data(match.rx));
    set_output("ry", Data(match.ry));
    set_output("rz", Data(match.rz));
    set_output("score", Data(match.score));

    if (output_pose_) {
        set_output("pose", Data(match.to_pose()));
    }

    return Result<void>::success();
}

// ========== ShapeModel3DProjectNode ==========

ShapeModel3DProjectNode::ShapeModel3DProjectNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ShapeModel3DProjectNode::make_info() {
    NodeInfo info;
    info.id = "ShapeModel3DProject";
    info.name = "3D模型投影";
    info.category = "3D匹配结果";
    info.description = "将3D模型投影到2D图像";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, true));
    info.inputs.push_back(DataPort("pose", "位姿", DataType::Pose, false));

    info.outputs.push_back(DataPort("image", "投影图像", DataType::Image));

    info.params.push_back(ParamDef("output_width", "输出宽度", DataType::Number, Data(640)));
    info.params.push_back(ParamDef("output_height", "输出高度", DataType::Number, Data(480)));
    info.params.push_back(ParamDef("draw_edges", "绘制边缘", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("draw_filled", "填充绘制", DataType::Boolean, Data(false)));
    info.params.push_back(ParamDef("edge_color_r", "边缘颜色R", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("edge_color_g", "边缘颜色G", DataType::Number, Data(255)));
    info.params.push_back(ParamDef("edge_color_b", "边缘颜色B", DataType::Number, Data(0)));
    info.params.push_back(ParamDef("edge_thickness", "边缘线宽", DataType::Number, Data(2)));

    return info;
}

Result<void> ShapeModel3DProjectNode::execute(FlowContext& context) {
    output_width_ = static_cast<int>(get_param("output_width", Data(640)).as_int());
    output_height_ = static_cast<int>(get_param("output_height", Data(480)).as_int());
    draw_edges_ = get_param("draw_edges", Data(true)).as_bool();
    draw_filled_ = get_param("draw_filled", Data(false)).as_bool();

    edge_color_.r = static_cast<uint8_t>(get_param("edge_color_r", Data(0)).as_int());
    edge_color_.g = static_cast<uint8_t>(get_param("edge_color_g", Data(255)).as_int());
    edge_color_.b = static_cast<uint8_t>(get_param("edge_color_b", Data(0)).as_int());

    edge_thickness_ = static_cast<int>(get_param("edge_thickness", Data(2)).as_int());

    if (!has_input("model")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No model input");
    }

    ShapeModel3D model;
    Transform3D pose;

    if (has_input("pose")) {
        Pose p = get_input("pose").as_pose();
        pose = matching_3d_utils::build_transform(
            static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
            static_cast<float>(p.rx), static_cast<float>(p.ry), static_cast<float>(p.rz)
        );
    }

    CameraParams camera_params = model.camera_params;

    ImageData image;
    ErrorCode err = matching_3d_utils::project_model_3d(model, pose, camera_params,
                                                        image, output_width_, output_height_);

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Projection failed");
    }

    set_output("image", Data(image));

    return Result<void>::success();
}

// ========== ShapeModel3DTransformNode ==========

ShapeModel3DTransformNode::ShapeModel3DTransformNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ShapeModel3DTransformNode::make_info() {
    NodeInfo info;
    info.id = "ShapeModel3DTransform";
    info.name = "3D模型变换";
    info.category = "3D匹配结果";
    info.description = "对3D模型应用平移和旋转变换";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, true));

    info.outputs.push_back(DataPort("model", "变换后的模型", DataType::Any));

    info.params.push_back(ParamDef("tx", "平移X", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("ty", "平移Y", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("tz", "平移Z", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("rx", "旋转X（弧度）", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("ry", "旋转Y（弧度）", DataType::Number, Data(0.0f)));
    info.params.push_back(ParamDef("rz", "旋转Z（弧度）", DataType::Number, Data(0.0f)));

    return info;
}

Result<void> ShapeModel3DTransformNode::execute(FlowContext& context) {
    tx_ = static_cast<float>(get_param("tx", Data(0.0f)).as_number());
    ty_ = static_cast<float>(get_param("ty", Data(0.0f)).as_number());
    tz_ = static_cast<float>(get_param("tz", Data(0.0f)).as_number());
    rx_ = static_cast<float>(get_param("rx", Data(0.0f)).as_number());
    ry_ = static_cast<float>(get_param("ry", Data(0.0f)).as_number());
    rz_ = static_cast<float>(get_param("rz", Data(0.0f)).as_number());

    if (!has_input("model")) {
        return Result<void>::failure(ErrorCode::InvalidParameter, "No model input");
    }

    ShapeModel3D model;
    Transform3D pose = matching_3d_utils::build_transform(tx_, ty_, tz_, rx_, ry_, rz_);

    ShapeModel3D transformed_model;
    ErrorCode err = matching_3d_utils::transform_model_3d(model, pose, transformed_model);

    if (err != ErrorCode::Success) {
        return Result<void>::failure(err, "Transform failed");
    }

    set_output("model", Data("ShapeModel3D[" + std::to_string(transformed_model.views.size()) + " views]"));

    return Result<void>::success();
}

// ========== ShapeModel3DVisualizeNode ==========

ShapeModel3DVisualizeNode::ShapeModel3DVisualizeNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo ShapeModel3DVisualizeNode::make_info() {
    NodeInfo info;
    info.id = "ShapeModel3DVisualize";
    info.name = "3D可视化";
    info.category = "3D匹配结果";
    info.description = "可视化3D模型和匹配结果";
    info.version = "0.1.0";

    info.inputs.push_back(DataPort("image", "背景图像", DataType::Image, false));
    info.inputs.push_back(DataPort("model", "3D模型", DataType::Any, false));
    info.inputs.push_back(DataPort("result", "匹配结果", DataType::Any, false));

    info.outputs.push_back(DataPort("image", "可视化图像", DataType::Image));

    info.params.push_back(ParamDef("output_width", "输出宽度", DataType::Number, Data(800)));
    info.params.push_back(ParamDef("output_height", "输出高度", DataType::Number, Data(600)));
    info.params.push_back(ParamDef("show_model", "显示模型", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("show_matches", "显示匹配", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("show_axes", "显示坐标轴", DataType::Boolean, Data(true)));
    info.params.push_back(ParamDef("show_bounding_box", "显示包围盒", DataType::Boolean, Data(false)));
    info.params.push_back(ParamDef("axis_length", "坐标轴长度", DataType::Number, Data(50.0f)));

    return info;
}

Result<void> ShapeModel3DVisualizeNode::execute(FlowContext& context) {
    output_width_ = static_cast<int>(get_param("output_width", Data(800)).as_int());
    output_height_ = static_cast<int>(get_param("output_height", Data(600)).as_int());
    show_model_ = get_param("show_model", Data(true)).as_bool();
    show_matches_ = get_param("show_matches", Data(true)).as_bool();
    show_axes_ = get_param("show_axes", Data(true)).as_bool();
    show_bounding_box_ = get_param("show_bounding_box", Data(false)).as_bool();
    axis_length_ = static_cast<float>(get_param("axis_length", Data(50.0f)).as_number());

    // 获取输入
    ImageData base_image;
    if (has_input("image")) {
        base_image = get_input("image").as_image();
    }

    // 创建输出图像
    ImageData output_image;
    if (base_image.empty()) {
        output_image.width = output_width_;
        output_image.height = output_height_;
        output_image.channels = 3;
        output_image.format = ImageFormat::RGB8;
        output_image.data.resize(output_width_ * output_height_ * 3, 128);  // 灰色背景
    } else {
        output_image = base_image;
    }

    ShapeModel3D model;
    Match3DResult result;
    CameraParams camera_params;

    // 从输入获取数据（如果存在）

    if (show_matches_ && has_input("result") && has_input("model")) {
        ErrorCode err = matching_3d_utils::visualize_match_3d(output_image, model, result,
                                                              camera_params, output_image);
        if (err != ErrorCode::Success) {
            OVF_WARN() << "Visualization failed";
        }
    }

    set_output("image", Data(output_image));

    return Result<void>::success();
}

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(DXFLoaderNode, "DXFLoader", DXFLoaderNode::make_info())
OVF_REGISTER_NODE(CreateShapeModel3DNode, "CreateShapeModel3D", CreateShapeModel3DNode::make_info())
OVF_REGISTER_NODE(ShapeModel3DTrainNode, "ShapeModel3DTrain", ShapeModel3DTrainNode::make_info())
OVF_REGISTER_NODE(ShapeModel3DSerializeNode, "ShapeModel3DSerialize", ShapeModel3DSerializeNode::make_info())
OVF_REGISTER_NODE(FindShapeModel3DNode, "FindShapeModel3D", FindShapeModel3DNode::make_info())
OVF_REGISTER_NODE(FindShapeModel3DClutteredNode, "FindShapeModel3DCluttered", FindShapeModel3DClutteredNode::make_info())
OVF_REGISTER_NODE(FindShapeModel3DMultiNode, "FindShapeModel3DMulti", FindShapeModel3DMultiNode::make_info())
OVF_REGISTER_NODE(Match3DRefineNode, "Match3DRefine", Match3DRefineNode::make_info())
OVF_REGISTER_NODE(GetShapeModel3DMatchesNode, "GetShapeModel3DMatches", GetShapeModel3DMatchesNode::make_info())
OVF_REGISTER_NODE(ShapeModel3DProjectNode, "ShapeModel3DProject", ShapeModel3DProjectNode::make_info())
OVF_REGISTER_NODE(ShapeModel3DTransformNode, "ShapeModel3DTransform", ShapeModel3DTransformNode::make_info())
OVF_REGISTER_NODE(ShapeModel3DVisualizeNode, "ShapeModel3DVisualize", ShapeModel3DVisualizeNode::make_info())

} // namespace algorithm
} // namespace ovf