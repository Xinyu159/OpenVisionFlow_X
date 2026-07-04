#pragma once

#include "ovf/core/node.h"
#include "ovf/core/types.h"
#include <complex>
#include <vector>

namespace ovf {
namespace algorithm {

/**
 * @brief 频域处理节点 - FFT变换
 *
 * 实现快速傅里叶变换，将图像从空间域转换到频域
 * 参考Halcon: fft_image, fft_image_inv
 */
class FFTForwardNode : public ovf::INode {
public:
    explicit FFTForwardNode(const String& instance_id);
    ~FFTForwardNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;
};

/**
 * @brief 频域处理节点 - FFT逆变换
 *
 * 将频域图像转换回空间域
 */
class FFTInverseNode : public ovf::INode {
public:
    explicit FFTInverseNode(const String& instance_id);
    ~FFTInverseNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;
};

/**
 * @brief 频域滤波节点 - 理想低通滤波器
 *
 * 参考Halcon: gen_lowpass
 */
class IdealLowPassNode : public ovf::INode {
public:
    explicit IdealLowPassNode(const String& instance_id);
    ~IdealLowPassNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float cutoff_frequency_ = 0.5f; // 归一化截止频率 [0,1]
};

/**
 * @brief 频域滤波节点 - 理想高通滤波器
 *
 * 参考Halcon: gen_highpass
 */
class IdealHighPassNode : public ovf::INode {
public:
    explicit IdealHighPassNode(const String& instance_id);
    ~IdealHighPassNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float cutoff_frequency_ = 0.1f;
};

/**
 * @brief 频域滤波节点 - 高斯低通滤波器
 *
 * 参考Halcon: gen_gauss_filter
 */
class GaussLowPassNode : public ovf::INode {
public:
    explicit GaussLowPassNode(const String& instance_id);
    ~GaussLowPassNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float sigma_ = 10.0f; // 高斯标准差
};

/**
 * @brief 频域滤波节点 - 高斯高通滤波器
 */
class GaussHighPassNode : public ovf::INode {
public:
    explicit GaussHighPassNode(const String& instance_id);
    ~GaussHighPassNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float sigma_ = 10.0f;
};

/**
 * @brief 频域滤波节点 - 巴特沃斯低通滤波器
 *
 * 参考Halcon: gen_bandpass
 */
class ButterworthLowPassNode : public ovf::INode {
public:
    explicit ButterworthLowPassNode(const String& instance_id);
    ~ButterworthLowPassNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float cutoff_frequency_ = 0.5f;
    int order_ = 2; // 巴特沃斯阶数
};

/**
 * @brief 频域滤波节点 - 巴特沃斯高通滤波器
 */
class ButterworthHighPassNode : public ovf::INode {
public:
    explicit ButterworthHighPassNode(const String& instance_id);
    ~ButterworthHighPassNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float cutoff_frequency_ = 0.1f;
    int order_ = 2;
};

/**
 * @brief 频域滤波节点 - 带通滤波器
 *
 * 参考Halcon: gen_bandpass, gen_bandstop
 */
class BandPassFilterNode : public ovf::INode {
public:
    explicit BandPassFilterNode(const String& instance_id);
    ~BandPassFilterNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float center_frequency_ = 0.3f;
    float bandwidth_ = 0.1f;
};

/**
 * @brief 频域滤波节点 - 带阻滤波器
 */
class BandStopFilterNode : public ovf::INode {
public:
    explicit BandStopFilterNode(const String& instance_id);
    ~BandStopFilterNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float center_frequency_ = 0.3f;
    float bandwidth_ = 0.1f;
};

/**
 * @brief 同态滤波节点
 *
 * 增强图像对比度，校正不均匀光照
 * 参考Halcon: hom_mat2d
 */
class HomomorphicFilterNode : public ovf::INode {
public:
    explicit HomomorphicFilterNode(const String& instance_id);
    ~HomomorphicFilterNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float high_frequency_gain_ = 2.0f;
    float low_frequency_gain_ = 0.5f;
    float cutoff_frequency_ = 0.2f;
};

/**
 * @brief 维纳滤波节点（去模糊）
 *
 * 参考Halcon: wiener_filter
 */
class WienerFilterNode : public ovf::INode {
public:
    explicit WienerFilterNode(const String& instance_id);
    ~WienerFilterNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float noise_power_ = 0.01f; // 噪声功率谱
};

/**
 * @brief 频谱分析节点 - 频谱可视化
 *
 * 生成频谱幅度图像用于分析
 * 参考Halcon: power_real, power_ln
 */
class PowerSpectrumNode : public ovf::INode {
public:
    explicit PowerSpectrumNode(const String& instance_id);
    ~PowerSpectrumNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    bool log_scale_ = true; // 使用对数尺度显示
    bool center_shift_ = true; // 将零频移到中心
};

/**
 * @brief 频谱分析节点 - 相位谱
 */
class PhaseSpectrumNode : public ovf::INode {
public:
    explicit PhaseSpectrumNode(const String& instance_id);
    ~PhaseSpectrumNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;
};

/**
 * @brief 频域乘法节点
 *
 * 在频域对两个图像进行乘法运算（卷积定理）
 * 参考Halcon: fft_image_inv
 */
class FrequencyMultiplyNode : public ovf::INode {
public:
    explicit FrequencyMultiplyNode(const String& instance_id);
    ~FrequencyMultiplyNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;
};

/**
 * @brief 频域相关节点
 *
 * 计算两幅图像的频域相关
 * 参考Halcon: phase_correlation
 */
class FrequencyCorrelateNode : public ovf::INode {
public:
    explicit FrequencyCorrelateNode(const String& instance_id);
    ~FrequencyCorrelateNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;
};

/**
 * @brief 相位相关节点（图像配准）
 *
 * 使用相位相关进行平移估计
 * 参考Halcon: phase_correlation
 */
class PhaseCorrelationNode : public ovf::INode {
public:
    explicit PhaseCorrelationNode(const String& instance_id);
    ~PhaseCorrelationNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float threshold_ = 0.5f; // 相关峰值阈值
};

/**
 * @brief 缺陷检测节点 - 频域差分
 *
 * 结合频域和空域进行缺陷检测
 * 参考Halcon缺陷检测方法：频域+空间域
 */
class FrequencyDefectDetectNode : public ovf::INode {
public:
    explicit FrequencyDefectDetectNode(const String& instance_id);
    ~FrequencyDefectDetectNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float low_cutoff_ = 0.05f;
    float high_cutoff_ = 0.4f;
    float threshold_ = 30.0f;
};

/**
 * @brief 周期性噪声去除节点
 *
 * 在频域去除周期性噪声（条纹、摩尔纹等）
 * 参考Halcon: detect_periodic_pattern
 */
class RemovePeriodicNoiseNode : public ovf::INode {
public:
    explicit RemovePeriodicNoiseNode(const String& instance_id);
    ~RemovePeriodicNoiseNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float threshold_ratio_ = 0.1f; // 噪声峰值阈值比例
    int notch_radius_ = 5; // 陷波滤波器半径
};

/**
 * @brief 纹理分析节点（频域）
 *
 * 使用频域方法分析图像纹理
 */
class TextureAnalysisFFTNode : public ovf::INode {
public:
    explicit TextureAnalysisFFTNode(const String& instance_id);
    ~TextureAnalysisFFTNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    int num_features_ = 10; // 提取的特征数量
};

/**
 * @brief 图像锐化节点（频域）
 *
 * 在频域进行图像锐化
 */
class FrequencySharpenNode : public ovf::INode {
public:
    explicit FrequencySharpenNode(const String& instance_id);
    ~FrequencySharpenNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float amount_ = 1.0f;
    float radius_ = 20.0f;
};

/**
 * @brief 边缘增强节点（频域）
 *
 * 在频域进行边缘增强
 */
class FrequencyEdgeEnhanceNode : public ovf::INode {
public:
    explicit FrequencyEdgeEnhanceNode(const String& instance_id);
    ~FrequencyEdgeEnhanceNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float strength_ = 1.0f;
    float direction_ = 0.0f; // 边缘方向角度
};

/**
 * @brief 频域平滑节点
 *
 * 在频域进行图像平滑
 */
class FrequencySmoothNode : public ovf::INode {
public:
    explicit FrequencySmoothNode(const String& instance_id);
    ~FrequencySmoothNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float sigma_ = 5.0f;
};

/**
 * @brief 反锐化掩模节点（频域）
 *
 * 使用频域方法进行反锐化掩模
 */
class FrequencyUnsharpMaskNode : public ovf::INode {
public:
    explicit FrequencyUnsharpMaskNode(const String& instance_id);
    ~FrequencyUnsharpMaskNode() override = default;

    static ovf::NodeInfo make_info();
    ovf::Result<void> execute(ovf::FlowContext& context) override;

private:
    float sigma_ = 5.0f;
    float amount_ = 0.5f;
    float threshold_ = 0.0f;
};

} // namespace algorithm
} // namespace ovf