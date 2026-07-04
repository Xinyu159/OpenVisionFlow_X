/**
 * @file cuda_accelerator.h
 * @brief CUDA加速关键算子模块 - 对标Halcon/OpenCV GPU加速
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/data.h"
#include "ovf/core/node.h"
#include "ovf/algorithm/blob_analysis.h"
#include <vector>
#include <memory>
#include <functional>

namespace ovf {
namespace algorithm {
namespace cuda {

/**
 * @brief 阈值类型
 */
enum class ThresholdType {
    Binary,         // 二值阈值: > thresh = max_val, else 0
    BinaryInv,      // 反二值阈值: > thresh = 0, else max_val
    Trunc,          // 截断阈值: > thresh = thresh, else unchanged
    ToZero,         // 阈值归零: > thresh = unchanged, else 0
    ToZeroInv,      // 反阈值归零: > thresh = 0, else unchanged
    AdaptiveMean,   // 自适应均值阈值
    AdaptiveGauss   // 自适应高斯阈值
};

/**
 * @brief 插值方法
 */
enum class InterpolationMethod {
    Nearest,        // 最近邻插值
    Bilinear,       // 双线性插值
    Bicubic,        // 双三次插值
    Area,           // 区域插值
    Lanczos         // Lanczos插值
};

/**
 * @brief 形态学操作类型
 */
enum class MorphOpCUDA {
    Erode,      // 腐蚀
    Dilate,     // 膨胀
    Open,       // 开运算
    Close,      // 闭运算
    Gradient,   // 形态学梯度
    TopHat,     // 顶帽
    BlackHat    // 黑帽
};

/**
 * @brief 核形状类型
 */
enum class KernelShapeCUDA {
    Rect,       // 矩形
    Ellipse,    // 椭圆
    Cross       // 十字
};

/**
 * @brief 模板匹配方法
 */
enum class MatchMethodCUDA {
    SAD,        // 平方差和
    SSD,        // 平方和差
    NCC         // 归一化互相关
};

/**
 * @brief CUDA执行统计信息
 */
struct CUDAStats {
    double gpu_time_ms = 0.0;       // GPU执行时间（毫秒）
    double cpu_time_ms = 0.0;       // CPU fallback时间（毫秒）
    double transfer_time_ms = 0.0;  // 数据传输时间（毫秒）
    double speedup = 1.0;           // 加速比
    bool used_gpu = false;          // 是否使用了GPU
    String algorithm_name;          // 算法名称
};

/**
 * @brief 模板匹配结果
 */
struct TemplateMatchResult {
    int x = 0;              // 匹配位置X
    int y = 0;              // 匹配位置Y
    float score = 0.0f;     // 匹配分数
    int template_id = 0;    // 模板ID
};

/**
 * @brief CUDA Stream管理
 */
class CUDAStream {
public:
    CUDAStream();
    ~CUDAStream();
    
    void synchronize();
    bool is_valid() const;
    void* get_native_stream() const;  // 返回cudaStream_t
    
private:
    void* stream_;  // cudaStream_t
    bool valid_;
};

/**
 * @brief CUDA内存缓冲区（Pinned Memory）
 */
class CUDABuffer {
public:
    CUDABuffer(size_t size);
    ~CUDABuffer();
    
    void* data();
    const void* data() const;
    size_t size() const;
    bool is_pinned() const;
    
    void resize(size_t new_size);
    
private:
    void* buffer_;
    size_t size_;
    bool pinned_;
};

/**
 * @brief CUDA加速器 - 主接口类
 */
class CudaAccelerator {
public:
    // ========== CUDA可用性检测 ==========
    
    /**
     * @brief 检测CUDA是否可用
     * @return true如果CUDA可用且GPU支持
     */
    static bool is_cuda_available();
    
    /**
     * @brief 获取GPU信息
     * @param name GPU名称
     * @param total_memory 总显存（MB）
     * @param compute_capability 计算能力（如"7.5"）
     * @return 是否成功获取信息
     */
    static bool get_gpu_info(String& name, size_t& total_memory, String& compute_capability);
    
    /**
     * @brief 获取CUDA设备数量
     * @return CUDA设备数量，0表示无CUDA设备
     */
    static int get_device_count();
    
    /**
     * @brief 设置当前CUDA设备
     * @param device_id 设备ID（0开始）
     * @return 是否成功
     */
    static bool set_device(int device_id);
    
    // ========== 1. 高斯模糊 (5-10x加速) ==========
    
    /**
     * @brief CUDA高斯模糊
     * @param input 输入图像
     * @param output 输出图像
     * @param sigma 高斯核标准差
     * @param kernel_size 核大小（0表示自动计算）
     * @param stream CUDA流（可选，nullptr使用默认流）
     * @param stats 执行统计（可选）
     * @return 错误码
     */
    static ErrorCode gaussian_blur(const ImageData& input, ImageData& output,
                                   float sigma, int kernel_size = 0,
                                   CUDAStream* stream = nullptr,
                                   CUDAStats* stats = nullptr);
    
    // ========== 2. Sobel边缘检测 (8x加速) ==========
    
    /**
     * @brief CUDA Sobel边缘检测
     * @param input 输入图像（灰度）
     * @param output 输出图像（梯度幅值）
     * @param dx 是否计算x方向梯度
     * @param dy 是否计算y方向梯度
     * @param threshold 边缘阈值（可选，0表示不阈值化）
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode sobel_filter(const ImageData& input, ImageData& output,
                                  bool dx = true, bool dy = true,
                                  int threshold = 0,
                                  CUDAStream* stream = nullptr,
                                  CUDAStats* stats = nullptr);
    
    // ========== 3. 自适应阈值分割 (10x加速) ==========
    
    /**
     * @brief CUDA阈值分割
     * @param input 输入图像（灰度）
     * @param output 输出图像（二值）
     * @param type 阈值类型
     * @param threshold_value 阈值值（全局阈值）
     * @param max_value 最大值（通常255）
     * @param block_size 自适应阈值块大小（仅自适应类型）
     * @param c 自适应阈值常数偏移
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode threshold(const ImageData& input, ImageData& output,
                               ThresholdType type = ThresholdType::Binary,
                               float threshold_value = 128.0f,
                               float max_value = 255.0f,
                               int block_size = 11,
                               float c = 2.0f,
                               CUDAStream* stream = nullptr,
                               CUDAStats* stats = nullptr);
    
    // ========== 4. 形态学处理 (6x加速) ==========
    
    /**
     * @brief CUDA形态学操作
     * @param input 输入图像（灰度或二值）
     * @param output 输出图像
     * @param op 操作类型
     * @param kernel_shape 核形状
     * @param kernel_size 核大小
     * @param iterations 迭代次数
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode morphology(const ImageData& input, ImageData& output,
                                MorphOpCUDA op,
                                KernelShapeCUDA kernel_shape = KernelShapeCUDA::Rect,
                                int kernel_size = 3,
                                int iterations = 1,
                                CUDAStream* stream = nullptr,
                                CUDAStats* stats = nullptr);
    
    // ========== 5. 直方图计算 (15x加速) ==========
    
    /**
     * @brief CUDA直方图计算
     * @param input 输入图像（灰度）
     * @param histogram 输出直方图（256个bin）
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode histogram(const ImageData& input,
                               Vector<int>& histogram,
                               CUDAStream* stream = nullptr,
                               CUDAStats* stats = nullptr);
    
    /**
     * @brief CUDA多通道直方图计算
     * @param input 输入图像（RGB/BGR）
     * @param histograms 输出直方图数组（每通道256个bin）
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode histogram_multi_channel(const ImageData& input,
                                             Vector<Vector<int>>& histograms,
                                             CUDAStream* stream = nullptr,
                                             CUDAStats* stats = nullptr);
    
    // ========== 6. 模板匹配 (20x加速) ==========
    
    /**
     * @brief CUDA模板匹配（单模板）
     * @param image 源图像
     * @param template_image 模板图像
     * @param result 匹配结果
     * @param method 匹配方法
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode template_match(const ImageData& image,
                                    const ImageData& template_image,
                                    TemplateMatchResult& result,
                                    MatchMethodCUDA method = MatchMethodCUDA::NCC,
                                    CUDAStream* stream = nullptr,
                                    CUDAStats* stats = nullptr);
    
    /**
     * @brief CUDA多模板匹配
     * @param image 源图像
     * @param templates 模板图像数组
     * @param results 匹配结果数组
     * @param method 匹配方法
     * @param threshold 匹配分数阈值
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode template_match_multi(const ImageData& image,
                                          const Vector<ImageData>& templates,
                                          Vector<TemplateMatchResult>& results,
                                          MatchMethodCUDA method = MatchMethodCUDA::NCC,
                                          float threshold = 0.7f,
                                          CUDAStream* stream = nullptr,
                                          CUDAStats* stats = nullptr);
    
    // ========== 7. Blob分析 (12x加速) ==========
    
    /**
     * @brief CUDA Blob分析（连通区域标记）
     * @param binary 输入二值图像
     * @param blobs 输出Blob数组
     * @param min_area 最小面积阈值
     * @param max_area 最大面积阈值
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode blob_analysis(const ImageData& binary,
                                   Vector<Blob>& blobs,
                                   uint32_t min_area = 10,
                                   uint32_t max_area = 1000000,
                                   CUDAStream* stream = nullptr,
                                   CUDAStats* stats = nullptr);
    
    // ========== 8. 图像缩放 (10x加速) ==========
    
    /**
     * @brief CUDA图像缩放
     * @param input 输入图像
     * @param output 输出图像
     * @param scale_x X方向缩放比例
     * @param scale_y Y方向缩放比例
     * @param method 插值方法
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode resize(const ImageData& input, ImageData& output,
                            float scale_x, float scale_y,
                            InterpolationMethod method = InterpolationMethod::Bilinear,
                            CUDAStream* stream = nullptr,
                            CUDAStats* stats = nullptr);
    
    /**
     * @brief CUDA图像缩放到指定尺寸
     * @param input 输入图像
     * @param output 输出图像
     * @param target_width 目标宽度
     * @param target_height 目标高度
     * @param method 插值方法
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode resize_to(const ImageData& input, ImageData& output,
                               int target_width, int target_height,
                               InterpolationMethod method = InterpolationMethod::Bilinear,
                               CUDAStream* stream = nullptr,
                               CUDAStats* stats = nullptr);
    
    // ========== 批量处理接口 ==========
    
    /**
     * @brief 批量高斯模糊（多图像并行）
     * @param inputs 输入图像数组
     * @param outputs 输出图像数组
     * @param sigma 高斯核标准差
     * @param kernel_size 核大小
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode gaussian_blur_batch(const Vector<ImageData>& inputs,
                                         Vector<ImageData>& outputs,
                                         float sigma, int kernel_size = 0,
                                         CUDAStream* stream = nullptr,
                                         CUDAStats* stats = nullptr);
    
    /**
     * @brief 批量阈值分割
     * @param inputs 输入图像数组
     * @param outputs 输出图像数组
     * @param type 阈值类型
     * @param threshold_value 阈值值
     * @param stream CUDA流
     * @param stats 执行统计
     * @return 错误码
     */
    static ErrorCode threshold_batch(const Vector<ImageData>& inputs,
                                      Vector<ImageData>& outputs,
                                      ThresholdType type,
                                      float threshold_value,
                                      CUDAStream* stream = nullptr,
                                      CUDAStats* stats = nullptr);
    
    // ========== 内存管理 ==========
    
    /**
     * @brief 创建Pinned内存缓冲区（用于高效CPU-GPU传输）
     * @param size 缓冲区大小（字节）
     * @return 缓冲区指针
     */
    static Ptr<CUDABuffer> create_buffer(size_t size);
    
    /**
     * @brief 同步CUDA流
     * @param stream CUDA流
     */
    static void synchronize_stream(CUDAStream* stream);
    
    /**
     * @brief 重置CUDA设备（释放所有缓存）
     */
    static void reset_device();
};

/**
 * @brief CUDA加速节点基类
 */
class CudaAcceleratorNode : public INode {
public:
    CudaAcceleratorNode(const String& instance_id, const NodeInfo& info);
    ~CudaAcceleratorNode() override;
    
    /**
     * @brief 设置是否强制使用CPU
     * @param force_cpu true强制使用CPU
     */
    void set_force_cpu(bool force_cpu) { force_cpu_ = force_cpu; }
    
    /**
     * @brief 获取是否使用了GPU
     * @return true如果使用了GPU
     */
    bool used_gpu() const { return used_gpu_; }
    
    /**
     * @brief 获取执行统计
     * @return 执行统计信息
     */
    const CUDAStats& get_stats() const { return stats_; }
    
protected:
    bool force_cpu_ = false;
    bool used_gpu_ = false;
    CUDAStats stats_;
    ovf::Ptr<CUDAStream> stream_;
};

/**
 * @brief CUDA高斯模糊节点
 */
class CudaGaussianBlurNode : public CudaAcceleratorNode {
public:
    CudaGaussianBlurNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_sigma(float sigma) { sigma_ = sigma; }
    void set_kernel_size(int size) { kernel_size_ = size; }
    
private:
    float sigma_ = 1.5f;
    int kernel_size_ = 0;
};

/**
 * @brief CUDA Sobel边缘检测节点
 */
class CudaSobelFilterNode : public CudaAcceleratorNode {
public:
    CudaSobelFilterNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_dx(bool dx) { dx_ = dx; }
    void set_dy(bool dy) { dy_ = dy; }
    void set_threshold(int threshold) { threshold_ = threshold; }
    
private:
    bool dx_ = true;
    bool dy_ = true;
    int threshold_ = 50;
};

/**
 * @brief CUDA阈值分割节点
 */
class CudaThresholdNode : public CudaAcceleratorNode {
public:
    CudaThresholdNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_threshold_type(ThresholdType type) { type_ = type; }
    void set_threshold_value(float value) { threshold_value_ = value; }
    
private:
    ThresholdType type_ = ThresholdType::Binary;
    float threshold_value_ = 128.0f;
    float max_value_ = 255.0f;
};

/**
 * @brief CUDA形态学节点
 */
class CudaMorphologyNode : public CudaAcceleratorNode {
public:
    CudaMorphologyNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_operation(MorphOpCUDA op) { operation_ = op; }
    void set_kernel_shape(KernelShapeCUDA shape) { kernel_shape_ = shape; }
    void set_kernel_size(int size) { kernel_size_ = size; }
    void set_iterations(int iter) { iterations_ = iter; }
    
private:
    MorphOpCUDA operation_ = MorphOpCUDA::Erode;
    KernelShapeCUDA kernel_shape_ = KernelShapeCUDA::Rect;
    int kernel_size_ = 3;
    int iterations_ = 1;
};

/**
 * @brief CUDA直方图节点
 */
class CudaHistogramNode : public CudaAcceleratorNode {
public:
    CudaHistogramNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    Vector<int> get_histogram() const { return histogram_; }
    
private:
    Vector<int> histogram_;
};

/**
 * @brief CUDA模板匹配节点
 */
class CudaTemplateMatchNode : public CudaAcceleratorNode {
public:
    CudaTemplateMatchNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_method(MatchMethodCUDA method) { method_ = method; }
    void set_threshold(float threshold) { threshold_ = threshold; }
    void set_template(const ImageData& tmpl) { template_image_ = tmpl; }
    TemplateMatchResult get_result() const { return result_; }
    
private:
    MatchMethodCUDA method_ = MatchMethodCUDA::NCC;
    float threshold_ = 0.7f;
    ImageData template_image_;
    TemplateMatchResult result_;
};

/**
 * @brief CUDA Blob分析节点
 */
class CudaBlobAnalysisNode : public CudaAcceleratorNode {
public:
    CudaBlobAnalysisNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_min_area(uint32_t area) { min_area_ = area; }
    void set_max_area(uint32_t area) { max_area_ = area; }
    Vector<Blob> get_blobs() const { return blobs_; }
    
private:
    uint32_t min_area_ = 10;
    uint32_t max_area_ = 1000000;
    Vector<Blob> blobs_;
};

/**
 * @brief CUDA图像缩放节点
 */
class CudaResizeNode : public CudaAcceleratorNode {
public:
    CudaResizeNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    void set_scale(float sx, float sy) { scale_x_ = sx; scale_y_ = sy; }
    void set_target_size(int width, int height) { target_width_ = width; target_height_ = height; }
    void set_interpolation(InterpolationMethod method) { method_ = method; }
    
private:
    float scale_x_ = 1.0f;
    float scale_y_ = 1.0f;
    int target_width_ = 0;  // 0表示使用scale
    int target_height_ = 0;
    InterpolationMethod method_ = InterpolationMethod::Bilinear;
};

} // namespace cuda
} // namespace algorithm
} // namespace ovf