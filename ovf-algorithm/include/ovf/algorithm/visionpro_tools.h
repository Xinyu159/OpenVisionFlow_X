/**
 * @file visionpro_tools.h
 * @brief VisionPro风格检测工具（纯C++实现）
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include "ovf/core/types.h"
#include <vector>
#include <memory>

namespace ovf {
namespace algorithm {

// ==================== VisionPro 核心数据结构 ====================

/**
 * @brief VisionPro风格的匹配结果（亚像素精度）
 */
struct CogMatchResult {
    double x = 0.0;             // 匹配位置X（亚像素）
    double y = 0.0;             // 匹配位置Y（亚像素）
    double angle = 0.0;         // 旋转角度（弧度）
    double scale = 1.0;         // 缩放比例
    double score = 0.0;         // 匹配分数 (0-100)
    double origin_x = 0.0;      // 原点X
    double origin_y = 0.0;      // 原点Y
    bool found = false;         // 是否找到
    uint32_t match_time_ms = 0; // 匹配耗时
};

/**
 * @brief VisionPro风格的边缘点
 */
struct CogEdgePoint {
    double x = 0.0;             // 边缘位置X（亚像素）
    double y = 0.0;             // 边缘位置Y（亚像素）
    double gradient = 0.0;      // 梯度强度
    double direction = 0.0;     // 边缘方向（弧度）
    int polarity = 0;           // 极性：1=亮到暗，-1=暗到亮
    bool valid = false;
};

/**
 * @brief VisionPro风格的拟合直线
 */
struct CogLine {
    double a = 0.0;             // 直线参数a
    double b = 0.0;             // 直线参数b
    double c = 0.0;             // 直线参数c
    double start_x = 0.0;       // 起点X
    double start_y = 0.0;       // 起点Y
    double end_x = 0.0;         // 终点X
    double end_y = 0.0;         // 终点Y
    double fit_error = 0.0;     // 拟合误差
    uint32_t point_count = 0;   // 拟合点数
    bool valid = false;
};

/**
 * @brief VisionPro风格的拟合圆
 */
struct CogCircle {
    double center_x = 0.0;      // 圆心X
    double center_y = 0.0;      // 圆心Y
    double radius = 0.0;        // 半径
    double fit_error = 0.0;     // 拟合误差
    uint32_t point_count = 0;   // 拟合点数
    bool valid = false;
};

/**
 * @brief VisionPro风格的Blob结果
 */
struct CogBlob {
    uint32_t id = 0;
    double area = 0.0;          // 面积（像素）
    double center_x = 0.0;      // 中心X
    double center_y = 0.0;      // 中心Y
    double min_x = 0.0;         // 边界框最小X
    double max_x = 0.0;         // 边界框最大X
    double min_y = 0.0;         // 边界框最小Y
    double max_y = 0.0;         // 边界框最大Y
    double width = 0.0;         // 宽度
    double height = 0.0;        // 高度
    double circularity = 0.0;   // 圆度
    double compactness = 0.0;   // 紧凑度
    double elongation = 0.0;    // 延伸率
    double orientation = 0.0;   // 方向角
    bool valid = false;
};

/**
 * @brief VisionPro风格的直方图结果
 */
struct CogHistogram {
    std::vector<uint32_t> bins; // 直方图数据
    double min_value = 0.0;     // 最小值
    double max_value = 255.0;   // 最大值
    double mean = 0.0;          // 均值
    double std_dev = 0.0;       // 标准差
    double median = 0.0;        // 中值
    uint32_t bin_count = 256;   // 直方图桶数
    bool valid = false;
};

/**
 * @brief VisionPro风格的模板信息
 */
struct CogTemplate {
    ImageData image;            // 模板图像
    double origin_x = 0.0;      // 原点X
    double origin_y = 0.0;      // 原点Y
    std::vector<double> features; // 特征描述
    bool trained = false;
};

// ==================== 模式检测工具 ====================

/**
 * @brief PatInspect节点 - PatInspect模式检测（模板对比检测缺陷）
 *
 * VisionPro风格的模板对比检测，用于检测产品缺陷
 *
 * 输入: image - 待检测图像
 *       template - 模板图像（可选，如果已训练则不需要）
 * 输出: result_image - 结果图像（标注缺陷）
 *       defect_count - 缺陷数量
 *       defect_areas - 缺陷区域列表
 *       pass_fail - 是否合格
 * 参数: threshold - 差异阈值
 *       min_defect_area - 最小缺陷面积
 *       max_defect_area - 最大缺陷面积
 */
class PatInspectNode : public INode {
public:
    PatInspectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief PatMax节点 - PatMax高精度定位
 *
 * VisionPro核心算法，高精度几何模式定位，支持亚像素定位
 *
 * 输入: image - 待检测图像
 * 输出: match_x - 匹配位置X
 *       match_y - 匹配位置Y
 *       match_angle - 匹配角度
 *       match_score - 匹配分数
 *       found - 是否找到
 * 参数: accept_threshold - 接受阈值
 *       search_angle - 搜索角度范围
 *       search_scale - 搜索缩放范围
 */
class PatMaxNode : public INode {
public:
    PatMaxNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief PMAlign节点 - PMAlign模式匹配（VisionPro核心）
 *
 * VisionPro最核心的定位工具，高精度模式匹配定位
 *
 * 输入: image - 待检测图像
 * 输出: match_result - 匹配结果
 *       position_x - 位置X
 *       position_y - 位置Y
 *       rotation - 旋转角度
 *       score - 匹配分数
 *       found - 是否找到
 * 参数: train_image - 训练图像路径
 *       accept_threshold - 接受阈值(0-100)
 *       polarity - 极性模式
 *       search_region - 搜索区域
 */
class PMAlignNode : public INode {
public:
    PMAlignNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief SearchMax节点 - SearchMax搜索最大化
 *
 * 在区域内搜索最佳匹配位置
 *
 * 输入: image - 待检测图像
 * 输出: best_x - 最佳位置X
 *       best_y - 最佳位置Y
 *       best_score - 最佳分数
 *       match_count - 匹配数量
 * 参数: search_region_x - 搜索区域X
 *       search_region_y - 搜索区域Y
 *       search_region_w - 搜索区域宽度
 *       search_region_h - 搜索区域高度
 *       threshold - 阈值
 */
class SearchMaxNode : public INode {
public:
    SearchMaxNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== ID识别工具 ====================

/**
 * @brief IDReaderNode - ID读取器（一维码+二维码）
 *
 * VisionPro风格的条码/二维码识别
 *
 * 输入: image - 待识别图像
 * 输出: barcode_data - 条码数据
 *       barcode_type - 条码类型
 *       position_x - 位置X
 *       position_y - 位置Y
 *       confidence - 置信度
 *       success - 是否成功
 * 参数: code_type - 码类型（auto/barcode/qrcode）
 *       orientation - 方向模式
 */
class IDReaderNode : public INode {
public:
    IDReaderNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief OCRVerifyNode - OCR验证器
 *
 * VisionPro风格的OCR文字验证
 *
 * 输入: image - 待识别图像
 *       expected_text - 期望文本（可选）
 * 输出: recognized_text - 识别文本
 *       confidence - 置信度
 *       match - 是否匹配期望文本
 *       text_regions - 文本区域列表
 * 参数: expected_text - 期望文本
 *       tolerance - 容差（允许字符差异数）
 *       language - 语言
 */
class OCRVerifyNode : public INode {
public:
    OCRVerifyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief FontTrainNode - 字体训练工具
 *
 * VisionPro风格的字体训练工具
 *
 * 输入: sample_image - 样本图像
 *       character - 字符标注
 * 输出: font_model - 字体模型
 *       training_status - 训练状态
 *       char_count - 字符数量
 * 参数: font_name - 字体名称
 *       output_path - 输出路径
 *       char_set - 字符集
 */
class FontTrainNode : public INode {
public:
    FontTrainNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 测量工具 ====================

/**
 * @brief CogCaliperNode - Caliper卡尺工具（VisionPro风格）
 *
 * VisionPro风格的卡尺工具，沿投影方向测量边缘
 *
 * 输入: image - 输入图像
 * 输出: edge_count - 边缘数量
 *       edge_points - 边缘点列表
 *       first_edge_x - 第一个边缘X
 *       first_edge_y - 第一个边缘Y
 *       distance - 测量距离
 * 参数: start_x - 起始点X
 *       start_y - 联起始点Y
 *       end_x - 结束点X
 *       end_y - 结束点Y
 *       width - 搜索宽度
 *       threshold - 边缘阈值
 *       polarity - 极性
 */
class CogCaliperNode : public INode {
public:
    CogCaliperNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief CogFindLineNode - 找线工具
 *
 * VisionPro风格的找线工具，检测并拟合直线
 *
 * 输入: image - 输入图像
 *       region - 搜索区域（可选）
 * 输出: line_a - 直线参数a
 *       line_b - 直线参数b
 *       line_c - 直线参数c
 *       line_result - 直线结果
 *       fit_error - 拟合误差
 *       point_count - 点数量
 * 参数: caliper_count - 卡尺数量
 *       caliper_length - 卡尺长度
 *       caliper_width - 卡尺宽度
 *       edge_threshold - 边缘阈值
 *       polarity - 极性
 */
class CogFindLineNode : public INode {
public:
    CogFindLineNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief CogFindCircleNode - 找圆工具
 *
 * VisionPro风格的找圆工具，检测并拟合圆
 *
 * 输入: image - 输入图像
 *       region - 搜索区域（可选）
 * 输出: center_x - 圆心X
 *       center_y - 圆心Y
 *       radius - 半径
 *       circle_result - 圆结果
 *       fit_error - 拟合误差
 *       point_count - 点数量
 * 参数: caliper_count - 卡尺数量
 *       expected_center_x - 预期圆心X
 *       expected_center_y - 预期圆心Y
 *       expected_radius - 预期半径
 *       edge_threshold - 边缘阈值
 *       polarity - 极性
 */
class CogFindCircleNode : public INode {
public:
    CogFindCircleNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== 分析工具 ====================

/**
 * @brief CogBlobAnalysisNode - Blob分析工具（VisionPro风格）
 *
 * VisionPro风格的Blob分析工具
 *
 * 输入: image - 输入图像（二值）
 * 输出: blob_count - Blob数量
 *       blob_list - Blob列表
 *       result_image - 结果图像
 *       total_area - 总面积
 * 参数: min_area - 最小面积
 *       max_area - 最大面积
 *       min_circularity - 最小圆度
 *       max_circularity - 最大圆度
 *       connectivity - 连通性（4/8）
 */
class CogBlobAnalysisNode : public INode {
public:
    CogBlobAnalysisNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief CogHistogramNode - 直方图分析工具
 *
 * VisionPro风格的直方图分析工具
 *
 * 输入: image - 输入图像
 * 输出: histogram - 直方图数据
 *       mean - 均值
 *       std_dev - 标准差
 *       min_value - 最小值
 *       max_value - 最大值
 *       median - 中值
 *       entropy - 信息熵
 * 参数: region - 计算区域（可选）
 *       bin_count - 直方图桶数
 *       channel - 通道（0=灰度，1=R，2=G，3=B）
 */
class CogHistogramNode : public INode {
public:
    CogHistogramNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief CogInspectNode - 综合检测工具
 *
 * VisionPro风格的综合检测工具，结合多种检测方法
 *
 * 输入: image - 输入图像
 *       reference - 参考图像（可选）
 * 输出: pass_fail - 是否合格
 *       defect_count - 缺陷数量
 *       defect_types - 缺陷类型列表
 *       inspect_time - 检测耗时
 *       result_image - 结果图像
 * 参数: mode - 检测模式（template/edge/color/combined）
 *       threshold - 检测阈值
 *       sensitivity - 敏感度
 *       roi - 检测区域
 */
class CogInspectNode : public INode {
public:
    CogInspectNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

/**
 * @brief CogResultAnalysisNode - 结果分析工具
 *
 * VisionPro风格的结果分析工具，汇总和分析检测结果
 *
 * 输入: inspect_results - 检测结果列表
 * 输出: pass_rate - 合格率
 *       total_count - 总数量
 *       pass_count - 合格数量
 *       fail_count - 不合格数量
 *       statistics - 统计数据
 *       report - 分析报告
 * 参数: pass_threshold - 合格阈值
 *       analysis_mode - 分析模式
 */
class CogResultAnalysisNode : public INode {
public:
    CogResultAnalysisNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
};

// ==================== VisionPro工具函数 ====================

namespace visionpro_utils {

/**
 * @brief PatInspect差异检测算法
 */
ErrorCode pat_inspect_detect(const ImageData& image, const ImageData& template_img,
                             double threshold, std::vector<Region>& defects);

/**
 * @brief PatMax高精度定位算法
 */
ErrorCode pat_max_locate(const ImageData& image, const CogTemplate& template_info,
                         double accept_threshold, double angle_range, double scale_range,
                         CogMatchResult& result);

/**
 * @brief PMAlign模式匹配算法
 */
ErrorCode pmalign_match(const ImageData& image, const ImageData& template_img,
                        double accept_threshold, int polarity_mode,
                        CogMatchResult& result);

/**
 * @brief SearchMax最佳搜索算法
 */
ErrorCode search_max_find(const ImageData& image, const ImageData& template_img,
                          int region_x, int region_y, int region_w, int region_h,
                          double threshold, std::vector<CogMatchResult>& results);

/**
 * @brief VisionPro风格卡尺边缘搜索
 */
std::vector<CogEdgePoint> cog_caliper_search(const ImageData& image,
                                              double start_x, double start_y,
                                              double end_x, double end_y,
                                              int width, double threshold, int polarity);

/**
 * @brief VisionPro风格直线拟合
 */
CogLine cog_fit_line(const std::vector<CogEdgePoint>& points);

/**
 * @brief VisionPro风格圆拟合
 */
CogCircle cog_fit_circle(const std::vector<CogEdgePoint>& points, 
                         double expected_cx, double expected_cy, double expected_r);

/**
 * @brief VisionPro风格Blob分析
 */
std::vector<CogBlob> cog_blob_analyze(const ImageData& binary_img,
                                       double min_area, double max_area,
                                       double min_circularity, double max_circularity,
                                       int connectivity);

/**
 * @brief VisionPro风格直方图计算
 */
CogHistogram cog_histogram_compute(const ImageData& image,
                                    int region_x, int region_y, int region_w, int region_h,
                                    uint32_t bin_count, int channel);

/**
 * @brief 亚像素边缘定位
 */
double subpixel_edge_position(const uint8_t* profile, int length, int edge_index);

/**
 * @brief 综合检测算法
 */
ErrorCode cog_inspect(const ImageData& image, const ImageData& reference,
                      int mode, double threshold, double sensitivity,
                      std::vector<Region>& defects, std::vector<String>& defect_types);

} // namespace visionpro_utils

} // namespace algorithm
} // namespace ovf