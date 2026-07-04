#pragma once

#include "ovf/core/node.h"
#include <vector>

namespace ovf {
namespace algorithm {

/**
 * @brief 几何变换节点
 */
class RotateNode : public INode {
public:
    RotateNode(const String& instance_id);
    ~RotateNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

    void set_angle(float angle) { angle_ = angle; }
    void set_center(float cx, float cy) { center_x_ = cx; center_y_ = cy; }

private:
    float angle_ = 0.0f;         // 旋转角度（度）
    float center_x_ = -1.0f;     // 旋转中心X（-1表示图像中心）
    float center_y_ = -1.0f;     // 旋转中心Y
    bool expand_canvas_ = true;  // 是否扩展画布以保持完整图像
};

/**
 * @brief 缩放节点
 */
class ResizeNode : public INode {
public:
    ResizeNode(const String& instance_id);
    ~ResizeNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

    void set_size(int width, int height) { new_width_ = width; new_height_ = height; }
    void set_scale(float scale) { scale_x_ = scale; scale_y_ = scale; use_scale_ = true; }

private:
    int new_width_ = 0;
    int new_height_ = 0;
    float scale_x_ = 1.0f;
    float scale_y_ = 1.0f;
    bool use_scale_ = false;
    String interpolation_ = "bilinear"; // nearest, bilinear
};

/**
 * @brief 平移节点
 */
class TranslateNode : public INode {
public:
    TranslateNode(const String& instance_id);
    ~TranslateNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    int offset_x_ = 0;
    int offset_y_ = 0;
    uint8_t fill_value_ = 0;
};

/**
 * @brief 仿射变换节点
 */
class AffineTransformNode : public INode {
public:
    AffineTransformNode(const String& instance_id);
    ~AffineTransformNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

    // 设置变换矩阵（2x3）
    void set_matrix(const std::vector<float>& matrix);

private:
    std::vector<float> transform_matrix_;
};

/**
 * @brief 透视变换节点
 */
class PerspectiveTransformNode : public INode {
public:
    PerspectiveTransformNode(const String& instance_id);
    ~PerspectiveTransformNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    // 四个源点和四个目标点
    std::vector<float> src_points_;
    std::vector<float> dst_points_;
};

/**
 * @brief 翻转节点
 */
class FlipNode : public INode {
public:
    FlipNode(const String& instance_id);
    ~FlipNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    bool horizontal_ = true;
    bool vertical_ = false;
};

/**
 * @brief 裁剪节点
 */
class CropNode : public INode {
public:
    CropNode(const String& instance_id);
    ~CropNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    int roi_x_ = 0;
    int roi_y_ = 0;
    int roi_width_ = 100;
    int roi_height_ = 100;
};

/**
 * @brief 坐标映射节点（自定义变换）
 */
class RemapNode : public INode {
public:
    RemapNode(const String& instance_id);
    ~RemapNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    std::vector<float> map_x_;
    std::vector<float> map_y_;
    int map_width_ = 0;
    int map_height_ = 0;
};

/**
 * @brief 几何校准节点（标定）
 */
class GeometricCalibrationNode : public INode {
public:
    GeometricCalibrationNode(const String& instance_id);
    ~GeometricCalibrationNode() override = default;

    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    std::vector<float> calibration_matrix_;
    float pixel_size_x_ = 1.0f;
    float pixel_size_y_ = 1.0f;
    float distortion_k1_ = 0.0f;
    float distortion_k2_ = 0.0f;
};

} // namespace algorithm
} // namespace ovf