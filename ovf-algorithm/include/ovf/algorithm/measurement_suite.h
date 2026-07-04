/**
 * @file measurement_suite.h
 * @brief 测量套件 - 完整的测量流程节点
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

// ==================== 测量数据结构 ====================

/**
 * @brief 测量对象类型
 */
enum class MeasureObjectType : uint8_t {
    Line = 0,       // 直线测量
    Circle = 1,     // 圆测量
    Rectangle = 2,  // 矩形测量
    Arc = 3         // 弧测量
};

/**
 * @brief 边缘类型
 */
enum class TransitionType : uint8_t {
    Positive = 0,   // 从暗到亮
    Negative = 1,   // 从亮到暗
    All = 2,        // 所有边缘
    Strongest = 3   // 最强边缘
};

/**
 * @brief 选择策略
 */
enum class SelectStrategy : uint8_t {
    First = 0,      // 第一个
    Last = 1,       // 最后一个
    All = 2,        // 所有
    Strongest = 3   // 最强
};

/**
 * @brief 测量类型
 */
enum class MeasureType : uint8_t {
    Position = 0,   // 位置测量
    Distance = 1,    // 距离测量
    Angle = 2,       // 角度测量
    Diameter = 3,    // 直径测量
    Radius = 4       // 半径测量
};

/**
 * @brief 测量状态
 */
enum class MeasureStatus : uint8_t {
    OK = 0,          // 合格
    NG = 1,          // 不合格
    Warning = 2,     // 警告
    Error = 3        // 错误
};

/**
 * @brief 边缘点
 */
struct EdgePoint {
    double row = 0.0;           // 行坐标（亚像素）
    double col = 0.0;           // 列坐标（亚像素）
    double amplitude = 0.0;     // 边缘幅度
    double distance = 0.0;      // 距离（到参考点的距离）
    int transition = 0;         // 边缘类型：1=正边缘, -1=负边缘

    EdgePoint() = default;
    EdgePoint(double r, double c, double amp, double dist, int trans)
        : row(r), col(c), amplitude(amp), distance(dist), transition(trans) {}
};

/**
 * @brief 测量对象 - 单个测量项
 */
struct MeasureObject {
    String id;                           // 对象ID
    MeasureObjectType type;              // 对象类型

    // 几何参数
    double center_row = 0.0;             // 中心行坐标
    double center_col = 0.0;             // 中心列坐标
    double length1 = 0.0;                // 长度1（线长/矩形长/半径）
    double length2 = 0.0;                // 长度2（矩形宽/角度范围）
    double angle = 0.0;                  // 角度（弧度）
    double radius = 0.0;                 // 半径（圆/弧）
    double start_angle = 0.0;            // 起始角度（弧度，用于弧）
    double end_angle = 0.0;               // 终止角度（弧度，用于弧）

    // 测量参数
    TransitionType transition = TransitionType::All;    // 边缘类型
    SelectStrategy select = SelectStrategy::All;         // 选择策略
    double threshold = 30.0;            // 边缘阈值
    double sigma = 1.0;                  // 高斯平滑参数
    int num_points = 10;                 // 测量点数

    // 测量类型
    MeasureType measure_type = MeasureType::Position;

    // 公差范围
    double tolerance_min = 0.0;          // 最小值
    double tolerance_max = 0.0;          // 最大值
    double nominal_value = 0.0;          // 标称值

    // ROI参数（用于测量）
    double roi_row1 = 0.0;               // ROI起始行
    double roi_col1 = 0.0;               // ROI起始列
    double roi_row2 = 0.0;               // ROI终止行
    double roi_col2 = 0.0;               // ROI终止列

    MeasureObject() = default;
};

/**
 * @brief 测量结果 - 单个测量对象的结果
 */
struct MeasureResult {
    String object_id;                    // 对应的测量对象ID
    MeasureStatus status = MeasureStatus::OK;  // 测量状态

    // 测量值
    double value = 0.0;                  // 测量值
    double error = 0.0;                  // 误差（与标称值的差）
    double deviation = 0.0;              // 偏差

    // 边缘点列表
    Vector<EdgePoint> edge_points;

    // 拟合结果
    double fit_row = 0.0;                // 拟合中心行
    double fit_col = 0.0;                // 拟合中心列
    double fit_angle = 0.0;              // 拟合角度
    double fit_radius = 0.0;             // 拟合半径
    double fit_error = 0.0;              // 拟合误差

    // 额外信息
    double first_edge_pos = 0.0;         // 第一个边缘位置
    double last_edge_pos = 0.0;          // 最后一个边缘位置
    double edge_distance = 0.0;          // 边缘间距离
    int num_edges = 0;                   // 检测到的边缘数

    MeasureResult() = default;
};

/**
 * @brief 测量模型 - 包含多个测量对象
 */
struct MeasureModel {
    String id;                           // 模型ID
    String name;                         // 模型名称
    Vector<MeasureObject> objects;      // 测量对象列表

    // 模型参数
    double pixel_size = 1.0;             // 像素尺寸（mm/pixel）
    bool calibrated = false;             // 是否已标定

    // 统计信息
    int measure_count = 0;               // 测量次数

    MeasureModel() = default;

    void add_object(const MeasureObject& obj) {
        objects.push_back(obj);
    }

    MeasureObject* find_object(const String& id) {
        for (auto& obj : objects) {
            if (obj.id == id) return &obj;
        }
        return nullptr;
    }

    void clear() {
        objects.clear();
        measure_count = 0;
    }
};

/**
 * @brief 测量统计结果
 */
struct MeasureStatistics {
    String object_id;                    // 对象ID
    int sample_count = 0;                // 样本数

    // 统计值
    double mean = 0.0;                   // 均值
    double std_dev = 0.0;                // 标准差
    double min_value = 0.0;              // 最小值
    double max_value = 0.0;              // 最大值
    double range = 0.0;                  // 极差

    // 过程能力
    double cp = 0.0;                     // 过程能力指数
    double cpk = 0.0;                    // 过程能力指数（修正）
    double cpu = 0.0;                    // 上侧过程能力
    double cpl = 0.0;                    // 下侧过程能力

    // 公差
    double lsl = 0.0;                    // 下规格限
    double usl = 0.0;                    // 上规格限
    double nominal = 0.0;                // 标称值

    MeasureStatistics() = default;
};

/**
 * @brief 测量报告
 */
struct MeasureReport {
    String report_id;                         // 报告ID
    uint64_t timestamp = 0;                   // 时间戳

    // 总体结果
    MeasureStatus overall_status = MeasureStatus::OK;  // 总体状态
    int pass_count = 0;                       // 合格项数
    int fail_count = 0;                       // 不合格项数
    int warning_count = 0;                    // 警告项数

    // 详细结果
    Vector<MeasureResult> results;            // 各项测量结果
    Vector<MeasureStatistics> statistics;     // 统计信息

    // 报告信息
    String model_name;                        // 模型名称
    String inspection_id;                     // 检测ID

    MeasureReport() {
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

// ==================== 测量辅助函数 ====================

/**
 * @brief 亚像素边缘检测
 */
class EdgeDetector {
public:
    // 检测边缘点
    static Vector<EdgePoint> detect_edges_1d(
        const uint8_t* profile,
        int length,
        double threshold,
        double sigma,
        TransitionType transition);

    // 高斯平滑
    static Vector<double> gaussian_smooth(
        const Vector<double>& data,
        double sigma);

    // 计算梯度
    static Vector<double> compute_gradient(
        const Vector<double>& data);

    // 亚像素精确定位
    static double subpixel_position(
        const Vector<double>& gradient,
        int index);
};

/**
 * @brief 几何拟合
 */
class GeometryFitter {
public:
    // 拟合直线
    static bool fit_line(
        const Vector<EdgePoint>& points,
        double& center_row,
        double& center_col,
        double& angle,
        double& error);

    // 拟合圆
    static bool fit_circle(
        const Vector<EdgePoint>& points,
        double& center_row,
        double& center_col,
        double& radius,
        double& error);

    // 拟合圆弧
    static bool fit_arc(
        const Vector<EdgePoint>& points,
        double& center_row,
        double& center_col,
        double& radius,
        double& start_angle,
        double& end_angle,
        double& error);

    // 计算点到直线距离
    static double point_to_line_distance(
        double row, double col,
        double line_row, double line_col, double line_angle);

    // 计算两点距离
    static double distance(double r1, double c1, double r2, double c2);

    // 计算角度
    static double angle_between_points(
        double r1, double c1, double r2, double c2);
};

// ==================== 测量模型创建节点 ====================

/**
 * @brief 创建测量模型节点
 */
class MeasureModelCreateNode : public INode {
public:
    MeasureModelCreateNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 添加直线测量对象节点
 */
class MeasureModelAddLineNode : public INode {
public:
    MeasureModelAddLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 添加圆测量对象节点
 */
class MeasureModelAddCircleNode : public INode {
public:
    MeasureModelAddCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 添加矩形测量对象节点
 */
class MeasureModelAddRectangleNode : public INode {
public:
    MeasureModelAddRectangleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 添加弧测量对象节点
 */
class MeasureModelAddArcNode : public INode {
public:
    MeasureModelAddArcNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ==================== 测量执行节点 ====================

/**
 * @brief 应用测量模型节点
 */
class MeasureModelApplyNode : public INode {
public:
    MeasureModelApplyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();

private:
    // 执行单个测量对象
    Result<void> measure_object(
        const ImageData& image,
        const MeasureObject& obj,
        MeasureResult& result);

    // 提取测量轮廓
    Vector<double> extract_profile(
        const ImageData& image,
        const MeasureObject& obj);

    // 在轮廓上检测边缘
    Vector<EdgePoint> detect_edges_in_profile(
        const Vector<double>& profile,
        const MeasureObject& obj);
};

/**
 * @brief 测量位置节点
 */
class MeasurePosNode : public INode {
public:
    MeasurePosNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 测量阈值节点
 */
class MeasureThresholdNode : public INode {
public:
    MeasureThresholdNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 测量轮廓节点
 */
class MeasureProfileNode : public INode {
public:
    MeasureProfileNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 模糊测量节点
 */
class MeasureFuzzyNode : public INode {
public:
    MeasureFuzzyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

// ==================== 测量结果处理节点 ====================

/**
 * @brief 获取测量结果节点
 */
class MeasureResultsGetNode : public INode {
public:
    MeasureResultsGetNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 结果转区域节点
 */
class MeasureResultsToRegionsNode : public INode {
public:
    MeasureResultsToRegionsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 测量统计节点
 */
class MeasureStatisticsNode : public INode {
public:
    MeasureStatisticsNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 测量比较节点（与标准值对比）
 */
class MeasureCompareNode : public INode {
public:
    MeasureCompareNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

/**
 * @brief 测量报告生成节点
 */
class MeasureReportNode : public INode {
public:
    MeasureReportNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;

    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf