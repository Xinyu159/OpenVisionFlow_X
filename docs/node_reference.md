# OpenVisionFlow 节点参考手册

> 本手册提供所有节点的完整技术参考，包含节点ID、功能描述、输入输出端口、参数配置和使用示例。

## 目录

1. [图像处理节点](#1-图像处理节点)
2. [亚像素精度节点](#2-亚像素精度节点)
3. [Blob分析节点](#3-blob分析节点)
4. [测量工具节点](#4-测量工具节点)
5. [OCR识别节点](#5-ocr识别节点)
6. [半导体晶圆检测节点](#6-半导体晶圆检测节点)
7. [汽车零部件检测节点](#7-汽车零部件检测节点)
8. [CUDA加速节点](#8-cuda加速节点)
9. [特征检测节点](#9-特征检测节点)
10. [分割算法节点](#10-分割算法节点)
11. [缺陷检测节点](#11-缺陷检测节点)
12. [条码识别节点](#12-条码识别节点)
13. [模板匹配节点](#13-模板匹配节点)
14. [图像运算节点](#14-图像运算节点)
15. [标定节点](#15-标定节点)

---

## 1. 图像处理节点

### 1.1 图像增强节点

#### MedianFilterNode - 中值滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.median_filter` |
| 功能描述 | 使用优化算法进行快速中值滤波，有效去除椒盐噪声，保边缘 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData) |
| 参数 | `kernel_size` - 核大小 (默认: 3, 范围: 3-15)<br>`channels` - 处理通道数 (默认: 全部) |

```json
{
  "id": "median_filter_1",
  "type": "algorithm.median_filter",
  "params": {
    "kernel_size": 5
  }
}
```

---

#### BilateralFilterNode - 双边滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.bilateral_filter` |
| 功能描述 | 保边缘去噪滤波，同时平滑图像并保留边缘细节 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData) |
| 参数 | `d` - 邻域直径 (默认: 9)<br>`sigma_color` - 颜色空间标准差 (默认: 75.0)<br>`sigma_space` - 坐标空间标准差 (默认: 75.0) |

---

#### GuidedFilterNode - 导向滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.guided_filter` |
| 功能描述 | 导向滤波，用于图像平滑、去雾、边缘保持等 |
| 输入端口 | `image` - 输入图像 (ImageData)<br>`guide` - 导向图像 (ImageData, 可选) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData) |
| 参数 | `radius` - 局部窗口半径 (默认: 8)<br>`epsilon` - 正则化参数 (默认: 0.01) |

---

#### GaussianFilterNode - 高斯滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.gaussian_filter` |
| 功能描述 | 高斯低通滤波，用于图像平滑、去噪 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData) |
| 参数 | `kernel_size` - 核大小 (默认: 5)<br>`sigma` - 标准差 (默认: 1.5) |

---

#### BoxFilterNode - 方框滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.box_filter` |
| 功能描述 | 方框滤波/均值滤波，计算邻域平均值 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData) |
| 参数 | `kernel_size` - 核大小 (默认: 3) |

---

#### LaplacianFilterNode - 拉普拉斯滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.laplacian_filter` |
| 功能描述 | 拉普拉斯滤波，用于边缘增强和锐化 |
| 输入端口 | `image` - 输入图像 (ImageData, 灰度) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData) |

---

#### SharpenFilterNode - 锐化滤波节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.sharpen_filter` |
| 功能描述 | 图像锐化，增强边缘和细节 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 锐化后图像 (ImageData) |
| 参数 | `strength` - 锐化强度 (默认: 1.0, 范围: 0.1-5.0) |

---

#### UnsharpMaskNode - 反锐化掩蔽节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.unsharp_mask` |
| 功能描述 | 反锐化掩蔽，使用原图减去模糊图实现锐化 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 处理后图像 (ImageData) |
| 参数 | `radius` - 模糊半径 (默认: 5)<br>`amount` - 锐化量 (默认: 1.0)<br>`threshold` - 阈值 (默认: 0) |

---

### 1.2 去噪节点

#### DenoiseBilateralNode - 双边去噪节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.denoise_bilateral` |
| 功能描述 | 双边滤波去噪，保边缘 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `denoised_image` - 去噪后图像 (ImageData) |
| 参数 | `d` - 邻域直径 (默认: 9)<br>`sigma_color` - 颜色标准差 (默认: 75)<br>`sigma_space` - 空间标准差 (默认: 75) |

---

#### DenoiseNLMNode - 非局部均值去噪节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.denoise_nlm` |
| 功能描述 | 非局部均值去噪，利用图像自相似性 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `denoised_image` - 去噪后图像 (ImageData) |
| 参数 | `search_window` - 搜索窗口大小 (默认: 21)<br>`template_size` - 模板大小 (默认: 7)<br>`h` - 去噪强度参数 (默认: 10) |

---

#### DenoiseWaveletNode - 小波去噪节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.denoise_wavelet` |
| 功能描述 | 小波变换去噪（简化实现） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `denoised_image` - 去噪后图像 (ImageData) |
| 参数 | `threshold` - 阈值系数 (默认: 0.1) |

---

#### DenoiseAdaptiveNode - 自适应去噪节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.denoise_adaptive` |
| 功能描述 | 根据局部方差自适应调整去噪强度 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `denoised_image` - 去噪后图像 (ImageData) |
| 参数 | `window_size` - 窗口大小 (默认: 15)<br>`noise_level` - 噪声水平估计 (默认: 10) |

---

### 1.3 增强节点

#### ContrastEnhanceNode - 对比度增强节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.contrast_enhance` |
| 功能描述 | 线性对比度增强，拉伸灰度范围 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `enhanced_image` - 增强后图像 (ImageData) |
| 参数 | `factor` - 对比度因子 (默认: 1.5, 范围: 0.1-5.0) |

---

#### BrightnessAdjustNode - 亮度调整节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.brightness_adjust` |
| 功能描述 | 调整图像整体亮度 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `adjusted_image` - 调整后图像 (ImageData) |
| 参数 | `delta` - 亮度偏移值 (默认: 0, 范围: -255~255) |

---

#### GammaCorrectNode - Gamma校正节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.gamma_correct` |
| 功能描述 | Gamma非线性校正，调整图像亮度响应曲线 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `corrected_image` - 校正后图像 (ImageData) |
| 参数 | `gamma` - Gamma值 (默认: 1.0, 范围: 0.1-5.0) |

---

#### HistogramEqualizeNode - 直方图均衡化节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.histogram_equalize` |
| 功能描述 | 全局直方图均衡化，增强图像对比度 |
| 输入端口 | `image` - 输入图像 (ImageData, 灰度) |
| 输出端口 | `equalized_image` - 均衡化后图像 (ImageData) |

---

#### AdaptiveHistogramEqualizeNode - 自适应直方图均衡节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.adaptive_histogram_equalize` |
| 功能描述 | CLAHE自适应直方图均衡，局部对比度增强 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `equalized_image` - 均衡化后图像 (ImageData) |
| 参数 | `clip_limit` - 截断限制 (默认: 2.0)<br>`tile_size` - 分块大小 (默认: 8) |

---

### 1.4 色彩调整节点

#### SaturationAdjustNode - 饱和度调整节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.saturation_adjust` |
| 功能描述 | 在HSV空间调整图像饱和度 |
| 输入端口 | `image` - 输入RGB图像 (ImageData) |
| 输出端口 | `adjusted_image` - 调整后图像 (ImageData) |
| 参数 | `factor` - 饱和度因子 (默认: 1.0, 范围: 0-3.0) |

---

#### HueShiftNode - 色相偏移节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.hue_shift` |
| 功能描述 | 调整图像整体色相 |
| 输入端口 | `image` - 输入RGB图像 (ImageData) |
| 输出端口 | `adjusted_image` - 调整后图像 (ImageData) |
| 参数 | `delta_h` - 色相偏移角度 (默认: 0, 范围: -180~180) |

---

#### WhiteBalanceNode - 白平衡校正节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.white_balance` |
| 功能描述 | 自动或手动白平衡校正 |
| 输入端口 | `image` - 输入RGB图像 (ImageData) |
| 输出端口 | `corrected_image` - 校正后图像 (ImageData) |
| 参数 | `method` - 校正方法 (默认: "gray_world", 选项: gray_world/white_patch/perfect_reflector) |

---

### 1.5 颜色处理节点

#### ColorConvertNode - 颜色空间转换节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_convert` |
| 功能描述 | 图像颜色空间转换（RGB/HSV/HSL/LAB/YUV/GRAY） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `converted_image` - 转换后图像 (ImageData) |
| 参数 | `source_space` - 源颜色空间 (默认: "RGB")<br>`target_space` - 目标颜色空间 (必填) |

---

#### ColorSegmentNode - 颜色分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_segment` |
| 功能描述 | 根据目标颜色进行颜色分割 |
| 输入端口 | `image` - 输入RGB图像 (ImageData) |
| 输出端口 | `mask` - 分割掩码 (ImageData)<br>`segmented_image` - 分割后彩色图像 (ImageData) |
| 参数 | `target_color` - 目标颜色 (格式: "r,g,b" 或 "#RRGGBB")<br>`tolerance` - 容差 (默认: 30, 范围: 0-255)<br>`color_space` - 颜色空间 (默认: RGB) |

---

#### ColorRecognizeNode - 颜色识别节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_recognize` |
| 功能描述 | 从图像中提取主颜色 |
| 输入端口 | `image` - 输入图像 (ImageData)<br>`roi` - ROI区域 (Region, 可选) |
| 输出端口 | `dominant_color` - 主颜色 (ColorRGB)<br>`color_histogram` - 颜色直方图<br>`color_name` - 颜色名称<br>`dominant_colors` - 主颜色列表 (JSON数组) |
| 参数 | `max_colors` - 最大颜色数量 (默认: 5)<br>`method` - 识别方法 (默认: histogram) |

---

#### ColorMatchNode - 颜色匹配节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_match` |
| 功能描述 | 在图像中匹配指定颜色区域 |
| 输入端口 | `image` - 输入图像 (ImageData)<br>`reference_color` - 参考颜色 |
| 输出端口 | `match_result` - 匹配结果<br>`match_region` - 匹配区域<br>`similarity` - 相似度 |
| 参数 | `tolerance` - 容差 (默认: 30)<br>`min_area` - 最小匹配面积 (默认: 100)<br>`color_space` - 颜色空间 (默认: RGB) |

---

#### ColorCorrectNode - 颜色校正节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_correct` |
| 功能描述 | 颜色校正处理 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `corrected_image` - 校正后图像 (ImageData)<br>`white_point` - 检测到的白点 |
| 参数 | `method` - 校正方法 (white_balance/gamma/auto_levels)<br>`gamma` - Gamma值 (Gamma模式)<br>`low_clip` - 低端裁剪 (自动色阶)<br>`high_clip` - 高端裁剪 (自动色阶) |

---

#### ColorHistogramNode - 颜色直方图节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_histogram` |
| 功能描述 | 计算图像颜色直方图 |
| 输入端口 | `image` - 输入图像 (ImageData)<br>`roi` - ROI区域 (Region, 可选) |
| 输出端口 | `histogram_r` - 红通道直方图<br>`histogram_g` - 绿通道直方图<br>`histogram_b` - 蓝通道直方图<br>`mean_color` - 平均颜色<br>`std_dev` - 标准差 |
| 参数 | `bins` - 直方图箱子数 (默认: 256)<br>`compute_stats` - 是否计算统计信息 (默认: true) |

---

### 1.6 边缘检测节点

#### SobelEdgeNode - Sobel边缘检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.sobel_edge` |
| 功能描述 | Sobel算子边缘检测 |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `edge_image` - 边缘图像 (ImageData) |
| 参数 | `threshold` - 边缘阈值 (默认: 50, 范围: 0-255) |

---

#### CannyEdgeNode - Canny边缘检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.canny_edge` |
| 功能描述 | Canny边缘检测（简化版，含双阈值处理） |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `edge_image` - 边缘图像 (ImageData) |
| 参数 | `low_threshold` - 低阈值 (默认: 50)<br>`high_threshold` - 高阈值 (默认: 100) |

---

### 1.7 形态学节点

#### MorphologyNode - 形态学处理节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.morphology` |
| 功能描述 | 通用形态学操作（腐蚀/膨胀/开运算/闭运算等） |
| 输入端口 | `image` - 输入二值或灰度图像 (ImageData) |
| 输出端口 | `output_image` - 处理后图像 (ImageData) |
| 参数 | `operation` - 操作类型 (Erode/Dilate/Open/Close/Gradient/TopHat/BlackHat)<br>`kernel_shape` - 核形状 (Rect/Ellipse/Cross/Diamond)<br>`kernel_size` - 核大小 (默认: 3)<br>`iterations` - 迭代次数 (默认: 1) |

---

#### ThresholdMorphNode - 阈值形态学节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.threshold_morph` |
| 功能描述 | 先阈值化再进行形态学操作 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `output_image` - 处理后图像 (ImageData) |
| 参数 | `threshold_value` - 阈值 (默认: 128)<br>`morph_op` - 形态学操作<br>`kernel_shape` - 核形状<br>`kernel_size` - 核大小 |

---

#### FillHolesNode - 孔洞填充节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.fill_holes` |
| 功能描述 | 填充二值图像中的孔洞 |
| 输入端口 | `image` - 输入二值图像 (ImageData) |
| 输出端口 | `filled_image` - 填充后图像 (ImageData) |

---

#### ExtractBoundaryNode - 边界提取节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.extract_boundary` |
| 功能描述 | 提取二值区域的边界轮廓 |
| 输入端口 | `image` - 输入二值图像 (ImageData) |
| 输出端口 | `boundary_image` - 边界图像 (ImageData) |

---

#### ThinningNode - 细化节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.thinning` |
| 功能描述 | 骨架细化，将二值图像细化为单像素宽度骨架 |
| 输入端口 | `image` - 输入二值图像 (ImageData) |
| 输出端口 | `thinned_image` - 细化后图像 (ImageData) |
| 参数 | `max_iterations` - 最大迭代次数 (默认: 100) |

---

### 1.8 几何变换节点

#### RotateNode - 旋转节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.rotate` |
| 功能描述 | 图像旋转变换 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `rotated_image` - 旋转后图像 (ImageData) |
| 参数 | `angle` - 旋转角度（度） (默认: 0)<br>`center_x` - 旋转中心X (-1表示图像中心)<br>`center_y` - 旋转中心Y<br>`expand_canvas` - 是否扩展画布 (默认: true) |

---

#### ResizeNode - 缩放节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.resize` |
| 功能描述 | 图像缩放变换 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `resized_image` - 缩放后图像 (ImageData) |
| 参数 | `width` - 目标宽度 (与scale二选一)<br>`height` - 目标高度<br>`scale` - 缩放比例<br>`interpolation` - 插值方法 (nearest/bilinear, 默认: bilinear) |

---

#### TranslateNode - 平移节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.translate` |
| 功能描述 | 图像平移变换 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `translated_image` - 平移后图像 (ImageData) |
| 参数 | `offset_x` - X偏移 (默认: 0)<br>`offset_y` - Y偏移 (默认: 0)<br>`fill_value` - 填充值 (默认: 0) |

---

#### AffineTransformNode - 仿射变换节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.affine_transform` |
| 功能描述 | 仿射变换（使用2x3变换矩阵） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `transformed_image` - 变换后图像 (ImageData) |
| 参数 | `matrix` - 变换矩阵 [a, b, c, d, e, f] |

---

#### PerspectiveTransformNode - 透视变换节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.perspective_transform` |
| 功能描述 | 透视变换（四点映射） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `transformed_image` - 变换后图像 (ImageData) |
| 参数 | `src_points` - 源四点坐标<br>`dst_points` - 目标四点坐标 |

---

#### FlipNode - 翻转节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.flip` |
| 功能描述 | 图像翻转 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `flipped_image` - 翻转后图像 (ImageData) |
| 参数 | `horizontal` - 水平翻转 (默认: true)<br>`vertical` - 垂直翻转 (默认: false) |

---

#### CropNode - 裁剪节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.crop` |
| 功能描述 | ROI裁剪 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `cropped_image` - 裁剪后图像 (ImageData) |
| 参数 | `roi_x` - ROI起始X (默认: 0)<br>`roi_y` - ROI起始Y (默认: 0)<br>`roi_width` - ROI宽度 (默认: 100)<br>`roi_height` - ROI高度 (默认: 100) |

---

#### RemapNode - 坐标映射节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.remap` |
| 功能描述 | 自定义坐标映射变换 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `remapped_image` - 映射后图像 (ImageData) |
| 参数 | `map_x` - X坐标映射表<br>`map_y` - Y坐标映射表 |

---

#### GeometricCalibrationNode - 几何校准节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.geometric_calibration` |
| 功能描述 | 使用标定矩阵进行几何校准 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `calibrated_image` - 校准后图像 (ImageData) |
| 参数 | `calibration_matrix` - 标定矩阵<br>`pixel_size_x` - X像素尺寸<br>`pixel_size_y` - Y像素尺寸<br>`distortion_k1` - 畸变系数k1<br>`distortion_k2` - 畸变系数k2 |

---

## 2. 亚像素精度节点

### 2.1 亚像素定位节点

#### SubpixelEdgeNode - 亚像素边缘定位节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.subpixel_edge` |
| 功能描述 | 亚像素边缘定位（泰勒展开法，对标Halcon edges_sub_pix） |
| 精度目标 | ±0.01像素 |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `edge_points` - 亚像素边缘点数组 (Vector<SubpixelEdgePoint>)<br>`edge_count` - 边缘点数量 |
| 参数 | `method` - 定位方法 (taylor/parabola/gaussian/zernike, 默认: taylor)<br>`threshold` - 边缘阈值 (默认: 30)<br>`sigma` - 平滑参数 (默认: 1.0) |

---

#### SubpixelCornerNode - 亚像素角点定位节点

| 属性 | 说明 |
|------|------|
| 芒点ID | `algorithm.subpixel_corner` |
| 功能描述 | 亚像素角点定位（Saddle点法） |
| 精度目标 | ±0.03像素 |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `corner_points` - 亚像素角点数组 (Vector<SubpixelCornerPoint>)<br>`corner_count` - 角点数量 |
| 参数 | `k` - Harris系数 (默认: 0.04)<br>`window_size` - 窗口大小 (默认: 5)<br>`threshold` - 角点阈值 |

---

#### SubpixelLineNode - 亚像素直线定位节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.subpixel_line` |
| 功能描述 | 提取亚像素边缘点后拟合直线 |
| 输入端口 | `image` - 输入灰度图像 (ImageData)<br>`edge_points` - 边缘点数组（可选） |
| 输出端口 | `line_result` - 亚像素直线结果 (SubpixelLineResult)<br>`rms_error` - 拟合RMS误差 |
| 参数 | `threshold` - 边缘阈值<br>`min_points` - 最少拟合点数 (默认: 10) |

---

#### SubpixelCircleNode - 亚像素圆定位节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.subpixel_circle` |
| 功能描述 | 提取圆弧亚像素边缘点后拟合圆 |
| 输入端口 | `image` - 输入灰度图像 (ImageData)<br>`edge_points` - 边缘点数组（可选） |
| 输出端口 | `circle_result` - 亚像素圆结果 (SubpixelCircleResult)<br>`rms_error` - 拟合RMS误差 |
| 参数 | `threshold` - 边缘阈值<br>`min_points` - 最少拟合点数 (默认: 20) |

---

### 2.2 亚像素测量节点

#### SubpixelMeasureNode - 亚像素测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.subpixel_measure` |
| 功能描述 | 沿路径提取灰度轮廓，使用高斯拟合定位边缘 |
| 精度目标 | ±0.01像素 |
| 输入端口 | `image` - 输入灰度图像 (ImageData)<br>`path` - 测量路径 (起点/终点) |
| 输出端口 | `edge_positions` - 亚像素边缘位置<br>`profile` - 灰度轮廓 |
| 参数 | `sigma` - 平滑参数<br>`threshold` - 边缘阈值 |

---

#### SubpixelCaliperNode - 亚像素卡尺节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.subpixel_caliper` |
| 功能描述 | 卡尺工具，抛物线拟合亚像素定位 |
| 精度目标 | ±0.02像素 |
| 输入端口 | `image` - 输入灰度图像 (ImageData)<br>`start` - 起始点<br>`end` - 结束点 |
| 输出端口 | `edge_position` - 亚像素边缘位置<br>`edge_amplitude` - 边缘幅度 |
| 参数 | `width` - 搜索宽度<br>`threshold` - 边缘阈值<br>`polarity` - 极性 (1=亮到暗, -1=暗到亮, 0=双向) |

---

#### SubpixelContourNode - 亚像素轮廓提取节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.subpixel_contour` |
| 功能描述 | 沿梯度方向细化得到亚像素级轮廓 |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `contour_points` - 亚像素轮廓点数组 |
| 参数 | `threshold` - 边缘阈值<br>`min_length` - 最小轮廓长度 |

---

### 2.3 精度验证节点

#### PrecisionTestNode - 精度测试节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.precision_test` |
| 功能描述 | 生成已知参数的合成测试图（圆/直线/边缘） |
| 输入端口 | 无（自动生成） |
| 输出端口 | `test_image` - 测试图像 (ImageData)<br>`ground_truth` - 理论真值 (JSON) |
| 参数 | `width` - 图像宽度 (默认: 512)<br>`height` - 图像高度 (默认: 512)<br>`pattern_type` - 测试图类型 (VerticalEdge/HorizontalEdge/Circle/Line/Corner/Square)<br>`param1` - 主参数（如圆心X）<br>`param2` - 副参数（如圆心Y）<br>`snr_db` - 信噪比（添加噪声） |

---

#### PrecisionBenchmarkNode - 精度基准测试节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.precision_benchmark` |
| 功能描述 | 在测试图上运行亚像素定位算法，与理论值对比 |
| 输入端口 | `test_image` - 测试图像 (ImageData)<br>`ground_truth` - 理论真值 |
| 输出端口 | `measured_value` - 测量值<br>`error` - 误差<br>`stats` - 统计信息 (PrecisionStats) |
| 参数 | `algorithm` - 测试算法 (edge/corner/line/circle)<br>`target_precision` - 目标精度 (默认: 0.01) |

---

#### PrecisionReportNode - 精度报告生成节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.precision_report` |
| 功能描述 | 汇总多次精度测试结果，判定是否达标 |
| 输入端口 | `errors` - 误差样本数组 |
| 输出端口 | `report` - 精度报告 (JSON)<br>`pass` - 是否达标<br>`mean_error` - 平均误差<br>`max_error` - 最大误差<br>`std_error` - 标准差 |
| 参数 | `target_precision` - 目标精度 (默认: 0.01) |

---

## 3. Blob分析节点

#### BlobAnalysisNode - Blob分析节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.blob_analysis` |
| 功能描述 | 连通区域分析，检测二值图像中的Blob并计算属性 |
| 输入端口 | `binary_image` - 输入二值图像 (ImageData) |
| 输出端口 | `blobs` - Blob数组 (Vector<Blob>)<br>`blob_count` - Blob数量<br>`annotated_image` - 标注图像（可选） |
| 参数 | `min_area` - 最小面积 (默认: 10)<br>`max_area` - 最大面积 (默认: 1000000)<br>`min_circularity` - 最小圆度（可选）<br>`max_aspect_ratio` - 最大长宽比（可选）<br>`draw_boxes` - 是否绘制边界框 (默认: false) |

**Blob属性结构：**

| 属性 | 类型 | 说明 |
|------|------|------|
| id | uint32_t | Blob ID |
| area | uint32_t | 面积（像素数） |
| x, y | uint32_t | 中心坐标 |
| min_x, max_x, min_y, max_y | uint32_t | 边界框 |
| width, height | uint32_t | 尺寸 |
| circularity | double | 圆度 (0-1) |
| aspect_ratio | double | 长宽比 |
| orientation | double | 方向角（弧度） |

---

## 4. 测量工具节点

### 4.1 基本测量节点

#### CaliperToolNode - 卡尺工具节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.caliper_tool` |
| 功能描述 | 沿投影方向搜索边缘跳变点 |
| 输入端口 | `image` - 输入灰度图像 (ImageData)<br>`start` - 起始点<br>`end` - 结束点 |
| 输出端口 | `edge_points` - 边缘点集合 (Vector<Point2Df>)<br>`edge_count` - 边缘数量 |
| 参数 | `width` - 搜索宽度 (默认: 5)<br>`threshold` - 边缘阈值 (默认: 30)<br>`polarity` - 极性 (1=亮到暗, -1=暗到亮, 0=双向) |

---

#### LineFitNode - 直线拟合节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.line_fit` |
| 功能描述 | 最小二乘法拟合直线 |
| 输入端口 | `points` - 输入点集 (Vector<Point2Df>) |
| 输出端口 | `line` - 拟合直线 (Line2D)<br>`error` - 拟合误差 |
| 参数 | `min_points` - 最少点数 (默认: 2) |

---

#### CircleFitNode - 圆拟合节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.circle_fit` |
| 功能描述 | 最小二乘法拟合圆（Kasa方法） |
| 输入端口 | `points` - 输入点集 (Vector<Point2Df>) |
| 输出端口 | `circle` - 拟合圆 (Circle2D)<br>`error` - 拟合误差 |
| 参数 | `min_points` - 最少点数 (默认: 3) |

---

#### DistanceMeasureNode - 距离测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.distance_measure` |
| 功能描述 | 测量两点或点到直线距离 |
| 输入端口 | `point1` - 第一点<br>`point2` - 第二点（点-点模式）<br>`line` - 直线（点-线模式） |
| 输出端口 | `distance` - 距离值 |
| 参数 | `mode` - 测量模式 (point_point/point_line) |

---

#### AngleMeasureNode - 角度测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.angle_measure` |
| 功能描述 | 测量两条直线的夹角 |
| 输入端口 | `line1` - 第一条直线<br>`line2` - 第二条直线 |
| 输出端口 | `angle` - 夹角（弧度）<br>`angle_deg` - 夹角（度） |

---

### 4.2 测量模型节点

#### MeasureModelCreateNode - 创建测量模型节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.measure_model_create` |
| 功能描述 | 创建测量模型容器 |
| 输入端口 | 无 |
| 输出端口 | `model` - 测量模型 (MeasureModel)<br>`model_id` - 模型ID |
| 参数 | `name` - 模型名称<br>`pixel_size` - 像素尺寸 (mm/pixel)<br>`calibrated` - 是否已标定 |

---

#### MeasureModelAddLineNode - 添加直线测量对象节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.measure_model_add_line` |
| 功能描述 | 向测量模型添加直线测量对象 |
| 输入端口 | `model` - 测量模型 |
| 输出端口 | `model` - 更新后的模型<br>`object_id` - 对象ID |
| 参数 | `id` - 对象ID<br>`center_row` - 中心行坐标<br>`center_col` - 中心列坐标<br>`length` - 线长<br>`angle` - 角度<br>`threshold` - 边缘阈值<br>`transition` - 边缘类型<br>`select` - 选择策略 |

---

#### MeasureModelAddCircleNode - 添加圆测量对象节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.measure_model_add_circle` |
| 功能描述 | 向测量模型添加圆测量对象 |
| 输入端口 | `model` - 测量模型 |
| 输出端口 | `model` - 更新后的模型<br>`object_id` - 对象ID |
| 参数 | `id` - 对象ID<br>`center_row` - 中心行坐标<br>`center_col` - 中心列坐标<br>`radius` - 半径<br>`threshold` - 边缘阈值<br>`num_points` - 测量点数 |

---

#### MeasureModelApplyNode - 应用测量模型节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.measure_model_apply` |
| 功能描述 | 在图像上应用测量模型，执行所有测量对象 |
| 输入端口 | `image` - 输入图像<br>`model` - 测量模型 |
| 输出端口 | `results` - 测量结果数组 (Vector<MeasureResult>)<br>`report` - 测量报告 |
| 参数 | 无 |

---

### 4.3 几何测量节点

#### AreaMeasureNode - 面积测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.area_measure` |
| 功能描述 | 测量区域面积 |
| 输入端口 | `region` - 输入区域或轮廓 |
| 输出端口 | `area` - 面积值 |
| 参数 | `unit` - 单位 (pixel/mm) |

---

#### PerimeterMeasureNode - 周长测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.perimeter_measure` |
| 功能描述 | 测量轮廓周长 |
| 输入端口 | `contour` - 输入轮廓 |
| 输出端口 | `perimeter` - 周长值 |

---

#### CircleMeasureNode - 圆测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.circle_measure` |
| 功能描述 | 测量圆度、直径 |
| 输入端口 | `contour` - 输入轮廓 |
| 输出端口 | `circularity` - 圆度<br>`diameter` - 直径<br>`center` - 圆心 |

---

#### EllipseMeasureNode - 椭圆测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.ellipse_measure` |
| 功能描述 | 拟合椭圆并测量参数 |
| 输入端口 | `points` - 输入点集或轮廓 |
| 输出端口 | `ellipse` - 椭圆参数 (Ellipse2D)<br>`major_axis` - 长轴<br>`minor_axis` - 短轴<br>`angle` - 旋转角度 |

---

## 5. OCR识别节点

### 5.1 通用OCR节点

#### OCRNode - OCR识别节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.ocr` |
| 功能描述 | 通用OCR文字识别（调用Tesseract） |
| 输入端口 | `image` - 待识别图像 (ImageData)<br>`roi` - ROI区域（可选） |
| 输出端口 | `text` - 识别文本<br>`confidence` - 置信度 (0-100)<br>`text_regions` - 文本区域列表 |
| 参数 | `language` - 语言代码 (eng/chi_sim/chi_tra等, 默认: eng)<br>`whitelist` - 白名单字符<br>`blacklist` - 黑名单字符<br>`model_path` - 模型路径 |

---

#### TextLocateNode - 文本定位节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.text_locate` |
| 功能描述 | 检测图像中的文本区域，不进行识别 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `text_regions` - 文本区域列表<br>`region_count` - 区域数量 |
| 参数 | `min_area` - 最小区域面积<br>`max_area` - 最大区域面积<br>`sensitivity` - 检测敏感度 |

---

### 5.2 中文OCR节点

#### CharDetectNode - 字符区域检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.char_detect` |
| 功能描述 | MSER文本检测，定位字符区域 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `char_region` - 第一个检测到的字符区域 (Region)<br>`region_count` - 检测到的字符区域数量<br>`text` - 检测概要信息 |
| 参数 | `min_char_size` - 最小字符尺寸 (默认: 8)<br>`max_char_size` - 最大字符尺寸 (默认: 100)<br>`delta` - MSER阈值步长 (默认: 5) |

---

#### CharSegmentNode - 字符分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.char_segment` |
| 功能描述 | 投影法+连通域字符分割 |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `char_count` - 分割出的字符数量<br>`first_char_region` - 第一个字符框 (Region)<br>`text` - 分割概要 |
| 参数 | `min_char_size` - 最小字符尺寸 (默认: 8)<br>`max_char_size` - 最大字符尺寸 (默认: 100)<br>`method` - 分割方法 (projection/connected/auto) |

---

#### ChineseOCRNode - 汉字OCR识别节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.chinese_ocr` |
| 功能描述 | 印刷体汉字识别（模板匹配法，网格+方向+投影特征） |
| 字符集 | GB2312一级常用汉字3755个 |
| 输入端口 | `image` - 待识别图像 (ImageData) |
| 输出端口 | `text` - 识别的文本字符串<br>`confidence` - 整体置信度<br>`chars` - 字符结果数组 |
| 参数 | `char_set` - 字符集 (固定: chinese)<br>`min_char_size` - 最小字符尺寸<br>`max_char_size` - 最大字符尺寸<br>`confidence_threshold` - 置信度阈值<br>`model_path` - 模型库路径 |

---

#### DigitOCRNode - 数字OCR识别节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.digit_ocr` |
| 功能描述 | 数字0-9识别 |
| 输入端口 | `image` - 待识别图像 (ImageData) |
| 输出端口 | `text` - 识别文本<br>`confidence` - 置信度<br>`chars` - 字符结果 |
| 参数 | `confidence_threshold` - 置信度阈值<br>`model_path` - 模型路径 |

---

#### EnglishOCRNode - 英文字母识别节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.english_ocr` |
| 功能描述 | 英文字母A-Z识别 |
| 输入端口 | `image` - 待识别图像 (ImageData) |
| 输出端口 | `text` - 识别文本<br>`confidence` - 置信度<br>`chars` - 字符结果 |
| 参数 | `confidence_threshold` - 置信度阈值<br>`model_path` - 模型路径 |

---

#### MixedOCRNode - 混合字符识别节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.mixed_ocr` |
| 功能描述 | 混合字符识别（汉字+数字+英文） |
| 输入端口 | `image` - 待识别图像 (ImageData) |
| 输出端口 | `text` - 识别文本<br>`confidence` - 置信度<br>`chars` - 字符结果 |
| 参数 | `confidence_threshold` - 置信度阈值<br>`model_path` - 模型路径 |

---

#### OCRVerifyNode - OCR验证节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.ocr_verify` |
| 功能描述 | OCR识别验证，对比期望文本 |
| 输入端口 | `image` - 待识别图像<br>`expected_text` - 期望文本 |
| 输出端口 | `matched` - 是否匹配<br>`accuracy` - 准确率<br>`actual_text` - 实际识别文本<br>`confidence` - 识别置信度 |
| 参数 | `case_sensitive` - 是否区分大小写<br>`confidence_threshold` - 置信度阈值 |

---

## 6. 半导体晶圆检测节点

### 6.1 晶粒检测节点

#### WaferDieDetectionNode - 晶粒定位检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_die_detection` |
| 功能描述 | 晶圆晶粒定位检测，识别晶粒网格 |
| 输入端口 | `image` - 晶圆图像 (ImageData) |
| 输出端口 | `dies` - 晶粒数组 (Vector<WaferDie>)<br>`die_count` - 晶粒数量<br>`yield_rate` - 良品率 |
| 参数 | `die_width` - 晶粒宽度（预估）<br>`die_height` - 晶粒高度（预估）<br>`margin` - 边缘余量 |

**WaferDie结构：**

| 属性 | 类型 | 说明 |
|------|------|------|
| id | uint32_t | 晶粒ID |
| row, col | uint32_t | 行号/列号 |
| center_x, center_y | double | 中心坐标 |
| width, height | uint32_t | 尺寸 |
| rotation | double | 旋转角度 |
| is_good | bool | 是否良品 |
| confidence | double | 置信度 |

---

### 6.2 缺陷检测节点

#### WaferDefectClassificationNode - 缺陷分类节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_defect_classification` |
| 功能描述 | 晶圆缺陷分类（划痕/颗粒/污染/裂纹） |
| 输入端口 | `image` - 晶圆图像 (ImageData)<br>`dies` - 晶粒数组（可选） |
| 输出端口 | `defects` - 缺陷数组 (Vector<WaferDefect>)<br>`defect_count` - 缺陷数量<br>`defect_types` - 缺陷类型统计 |
| 参数 | `min_defect_size` - 最小缺陷尺寸<br>`max_defect_size` - 最大缺陷尺寸<br>`sensitivity` - 检测敏感度 |

---

#### WaferPatternInspectionNode - 图案检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_pattern_inspection` |
| 功能描述 | 晶圆图案缺陷检测 |
| 输入端口 | `image` - 晶圆图像<br>`template` - 标准模板图像（可选） |
| 输出端口 | `defects` - 缺陷列表<br>`pattern_score` - 图案质量分数 |

---

#### WaferEdgeInspectionNode - 边缘检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_edge_inspection` |
| 功能描述 | 晶圆边缘完整性检测 |
| 输入端口 | `image` - 晶圆图像 |
| 输出端口 | `edge_quality` - 边缘质量分数<br>`edge_defects` - 边缘缺陷列表 |

---

#### WaferAlignmentNode - 晶圆对准节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_alignment` |
| 功能描述 | 晶圆对准标记检测 |
| 输入端口 | `image` - 晶圆图像 |
| 输出端口 | `alignment_marks` - 对准标记数组 (Vector<AlignmentMark>)<br>`rotation` - 检测到的旋转角度<br>`offset` - 偏移量 |
| 参数 | `mark_type` - 标记类型 (cross/lattice/h alignment) |

---

### 6.3 表面检测节点

#### WaferSurfaceInspectionNode - 表面检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_surface_inspection` |
| 功能描述 | 晶圆表面质量检测 |
| 输入端口 | `image` - 晶圆图像 |
| 输出端口 | `surface_quality` - 表面质量结果<br>`roughness` - 粗糙度<br>`flatness` - 平坦度 |

---

#### WaferContaminationDetectionNode - 污染检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_contamination_detection` |
| 功能描述 | 晶圆表面污染检测 |
| 输入端口 | `image` - 晶圆图像 |
| 输出端口 | `contaminations` - 污染区域列表<br>`contamination_count` - 污染数量 |

---

#### WaferCrackDetectionNode - 裂纹检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_crack_detection` |
| 功能描述 | 晶圆裂纹检测 |
| 输入端口 | `image` - 晶圆图像 |
| 输出端口 | `cracks` - 裂纹列表<br>`crack_count` - 裂纹数量<br>`severity` - 总体严重程度 |

---

#### WaferThicknessMeasurementNode - 厚度测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.wafer_thickness_measurement` |
| 功能描述 | 晶圆厚度测量（TTV/Bow/Warp） |
| 输入端口 | `image` - 晶圆图像或干涉图 |
| 输出端口 | `thickness_result` - 厚度结果 (ThicknessResult)<br>`center_thickness` - 中心厚度<br>`ttv` - 总厚度变化<br>`bow` - 弯曲度<br>`warp` - 翘曲度 |

---

## 7. 汽车零部件检测节点

### 7.1 钣金检测节点

#### BodyPanelInspectionNode - 钣金检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.body_panel_inspection` |
| 功能描述 | 汽车钣金表面缺陷检测（划痕/凹陷/凹坑/波纹/污染） |
| 输入端口 | `image` - 钣金图像 (ImageData) |
| 输出端口 | `defects` - 缺陷数组 (Vector<SurfaceDefect>)<br>`defect_count` - 缺陷数量<br>`quality_score` - 质量分数 (0-1)<br>`is_acceptable` - 是否合格 |
| 参数 | `min_defect_size` - 最小缺陷尺寸<br>`max_defect_size` - 最大缺陷尺寸<br>`severity_threshold` - 严重程度阈值 |

---

### 7.2 焊缝检测节点

#### WeldSeamInspectionNode - 焊缝检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.weld_seam_inspection` |
| 功能描述 | 焊缝质量评估（气孔/裂纹/咬边/焊瘤/飞溅检测） |
| 输入端口 | `image` - 焊缝图像 (ImageData) |
| 输出端口 | `defects` - 焊缝缺陷数组 (Vector<WeldDefect>)<br>`quality_result` - 焊缝质量结果 (WeldQualityResult)<br>`weld_width` - 焊缝宽度<br>`penetration` - 熔深百分比<br>`porosity_rate` - 气孔率 |
| 参数 | `weld_type` - 焊缝类型 (butt/fillet/lap)<br>`min_porosity_size` - 最小气孔尺寸 |

---

### 7.3 涂装检测节点

#### PaintQualityInspectionNode - 涂装检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.paint_quality_inspection` |
| 功能描述 | 漆面缺陷检测（橘皮/流挂/划痕/针孔/色差） |
| 输入端口 | `image` - 漆面图像 (ImageData) |
| 输出端口 | `defects` - 漆面缺陷数组 (Vector<PaintDefect>)<br>`quality_result` - 漆面质量结果 (PaintQualityResult)<br>`gloss_level` - 光泽度 (0-100)<br>`smoothness` - 平滑度 (0-100)<br>`color_consistency` - 色差一致性 |
| 参数 | `inspection_type` - 检测类型 (color/texture/defect) |

---

### 7.4 装配检测节点

#### AssemblyVerificationNode - 装配验证节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.assembly_verification` |
| 功能描述 | 零件完整性检测，验证装配是否正确 |
| 输入端口 | `image` - 装配图像<br>`expected_parts` - 期望零件列表（可选） |
| 输出端口 | `result` - 装配结果 (AssemblyResult)<br>`detected_parts` - 检测到零件数<br>`missing_parts` - 缺失零件数<br>`extra_parts` - 多余零件数<br>`completeness` - 完整性分数 (0-1) |
| 参数 | `expected_part_count` - 期望零件数量 |

---

### 7.5 尺寸检测节点

#### DimensionalInspectionNode - 尺寸检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.dimensional_inspection` |
| 功能描述 | 零件尺寸公差测量 |
| 输入端口 | `image` - 零件图像<br>`calibration` - 标定参数（可选） |
| 输出端口 | `measurements` - 测量结果数组 (Vector<DimensionResult>)<br>`in_tolerance` - 是否在公差范围 |
| 参数 | 测量对象配置（长度/宽度/孔径等） |

---

### 7.6 其他检测节点

#### GearInspectionNode - 齿轮检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.gear_inspection` |
| 功能描述 | 齿轮检测（齿形/磨损检测，齿厚/齿距测量） |
| 输入端口 | `image` - 齿轮图像 |
| 输出端口 | `result` - 齿轮检测结果 (GearInspectionResult)<br>`total_teeth` - 总齿数<br>`good_teeth` - 良好齿数<br>`tooth_thickness` - 齿厚<br>`pitch_error` - 齿距误差<br>`wear_level` - 磨损程度 |

---

#### ConnectorInspectionNode - 连接器检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.connector_inspection` |
| 功能描述 | 连接器插针位置检测 |
| 输入端口 | `image` - 连接器图像 |
| 输出端口 | `result` - 连接器结果 (ConnectorResult)<br>`total_pins` - 总插针数<br>`bent_pins` - 弯曲插针数<br>`missing_pins` - 缺失插针数<br>`alignment_score` - 对准分数 |

---

#### BoltPresenceCheckNode - 螺栓检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.bolt_presence_check` |
| 功能描述 | 螺栓漏装检测（存在性验证） |
| 输入端口 | `image` - 待检测图像<br>`expected_positions` - 期望螺栓位置（可选） |
| 输出端口 | `result` - 螺栓结果 (BoltResult)<br>`detected_bolts` - 检测到螺栓数<br>`missing_bolts` - 缺失螺栓数<br>`presence_rate` - 存在率 |

---

#### SurfaceRoughnessInspectionNode - 表面粗糙度检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.surface_roughness_inspection` |
| 功能描述 | 表面粗糙度检测（Ra/Rz/Rp/Rv/Rq） |
| 输入端口 | `image` - 表面图像 |
| 输出端口 | `result` - 粗糙度结果 (RoughnessResult)<br>`ra` - Ra粗糙度(μm)<br>`rz` - Rz粗糙度(μm)<br>`peak_to_valley` - 峰谷差(μm) |

---

#### GapMeasurementNode - 间隙测量节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.gap_measurement` |
| 功能描述 | 面板间隙公差检测 |
| 输入端口 | `image` - 面板图像 |
| 输出端口 | `result` - 间隙结果 (GapResult)<br>`gap_width` - 间隙宽度<br>`uniformity` - 均匀性分数<br>`is_acceptable` - 是否合格 |
| 参数 | `nominal_gap` - 标称间隙<br>`tolerance` - 公差 |

---

## 8. CUDA加速节点

### 8.1 滤波节点

#### CudaGaussianBlurNode - CUDA高斯模糊节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.gaussian_blur` |
| 功能描述 | CUDA加速高斯模糊（5-10x加速） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `filtered_image` - 滤波后图像 (ImageData)<br>`stats` - 执行统计 (CUDAStats) |
| 参数 | `sigma` - 高斯标准差 (默认: 1.5)<br>`kernel_size` - 核大小 (0=自动)<br>`force_cpu` - 强制使用CPU (默认: false) |

---

#### CudaSobelFilterNode - CUDA Sobel边缘检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.sobel_filter` |
| 功能描述 | CUDA加速Sobel边缘检测（8x加速） |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `edge_image` - 边缘图像 (ImageData)<br>`stats` - 执行统计 |
| 参数 | `dx` - 是否计算X梯度 (默认: true)<br>`dy` - 是否计算Y梯度 (默认: true)<br>`threshold` - 边缘阈值 (默认: 50) |

---

### 8.2 分割节点

#### CudaThresholdNode - CUDA阈值分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.threshold` |
| 功能描述 | CUDA加速阈值分割（10x加速） |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `binary_image` - 二值图像 (ImageData)<br>`stats` - 执行统计 |
| 参数 | `threshold_type` - 阈值类型 (Binary/BinaryInv/AdaptiveMean/AdaptiveGauss)<br>`threshold_value` - 阈值值 (默认: 128)<br>`block_size` - 自适应块大小<br>`c` - 常数偏移 |

---

#### CudaMorphologyNode - CUDA形态学节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.morphology` |
| 功能描述 | CUDA加速形态学操作（6x加速） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `output_image` - 处理后图像 (ImageData)<br>`stats` - 执行统计 |
| 参数 | `operation` - 操作类型 (Erode/Dilate/Open/Close/Gradient/TopHat/BlackHat)<br>`kernel_shape` - 核形状 (Rect/Ellipse/Cross)<br>`kernel_size` - 核大小 (默认: 3)<br>`iterations` - 迭代次数 (默认: 1) |

---

### 8.3 其他CUDA节点

#### CudaHistogramNode - CUDA直方图节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.histogram` |
| 功能描述 | CUDA加速直方图计算（15x加速） |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `histogram` - 直方图数据 (Vector<int>, 256 bins)<br>`stats` - 执行统计 |

---

#### CudaTemplateMatchNode - CUDA模板匹配节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.template_match` |
| 功能描述 | CUDA加速模板匹配（20x加速） |
| 输入端口 | `image` - 源图像<br>`template` - 模板图像 |
| 输出端口 | `result` - 匹配结果 (TemplateMatchResult)<br>`stats` - 执行统计 |
| 参数 | `method` - 匹配方法 (SAD/SSD/NCC)<br>`threshold` - 匹配分数阈值 (默认: 0.7) |

---

#### CudaBlobAnalysisNode - CUDA Blob分析节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.blob_analysis` |
| 功能描述 | CUDA加速Blob分析（12x加速） |
| 输入端口 | `binary` - 输入二值图像 (ImageData) |
| 输出端口 | `blobs` - Blob数组 (Vector<Blob>)<br>`stats` - 执行统计 |
| 参数 | `min_area` - 最小面积 (默认: 10)<br>`max_area` - 最大面积 (默认: 1000000) |

---

#### CudaResizeNode - CUDA图像缩放节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.cuda.resize` |
| 功能描述 | CUDA加速图像缩放（10x加速） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `resized_image` - 缩放后图像 (ImageData)<br>`stats` - 执行统计 |
| 参数 | `scale_x` - X缩放比例<br>`scale_y` - Y缩放比例<br>`target_width` - 目标宽度<br>`target_height` - 目标高度<br>`interpolation` - 插值方法 (Nearest/Bilinear/Bicubic) |

---

## 9. 特征检测节点

### 9.1 角点检测节点

#### HarrisCornerNode - Harris角点检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.harris_corner` |
| 功能描述 | Harris角点检测 |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `corners` - 角点数组 (Vector<Corner>)<br>`corner_count` - 角点数量 |
| 参数 | `k` - Harris系数 (默认: 0.04)<br>`threshold` - 响应阈值 (默认: 50)<br>`block_size` - 邻域块大小 (默认: 3) |

---

#### FastCornerNode - FAST角点检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.fast_corner` |
| 功能描述 | FAST角点检测（9点圆周比较） |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `corners` - 角点数组<br>`corner_count` - 角点数量 |
| 参数 | `threshold` - FAST阈值 (默认: 20) |

---

#### SobelCornerNode - Sobel角点检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.sobel_corner` |
| 功能描述 | 基于梯度方向变化的角点检测 |
| 输入端口 | `image` - 输入灰度图像 |
| 输出端口 | `corners` - 角点数组 |
| 参数 | `threshold` - 阈值 (默认: 50) |

---

#### MoravecCornerNode - Moravec角点检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.moravec_corner` |
| 功能描述 | Moravec角点检测 |
| 输入端口 | `image` - 输入灰度图像 |
| 输出端口 | `corners` - 角点数组 |
| 参数 | `threshold` - 阈值 (默认: 10000)<br>`window_size` - 窗口大小 (默认: 3) |

---

### 9.2 轮廓检测节点

#### ContourDetectNode - 轮廓检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.contour_detect` |
| 功能描述 | 基于边界跟踪的轮廓检测 |
| 输入端口 | `binary_image` - 输入二值图像 |
| 输出端口 | `contours` - 轮廓数组 (Vector<Contour>)<br>`contour_count` - 轮廓数量 |
| 参数 | `find_hierarchy` - 是否检测层次结构 (默认: true) |

---

#### ContourApproxNode - 轮廓近似节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.contour_approx` |
| 功能描述 | 多边形拟合近似轮廓 |
| 输入端口 | `contour` - 输入轮廓 |
| 输出端口 | `approx_contour` - 近似轮廓 |
| 参数 | `epsilon` - 近似精度 (默认: 2.0) |

---

#### ContourPropertyNode - 轮廓属性节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.contour_property` |
| 功能描述 | 计算轮廓属性（面积/周长/圆度/凸性等） |
| 输入端口 | `contour` - 输入轮廓 |
| 输出端口 | `property` - 轮廓属性 (ContourProperty)<br>`area` - 面积<br>`perimeter` - 周长<br>`circularity` - 圆度<br>`convexity` - 凸性<br>`centroid` - 质心 |

---

### 9.3 关键点检测节点

#### BlobDetectNode - Blob检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.blob_detect` |
| 功能描述 | 基于连通区域的Blob检测 |
| 输入端口 | `binary_image` - 输入二值图像 |
| 输出端口 | `keypoints` - 关键点数组 (Vector<KeyPoint>)<br>`keypoint_count` - 关键点数量 |
| 参数 | `min_area` - 最小面积 (默认: 10)<br>`max_area` - 最大面积 (默认: 100000) |

---

#### SIFTNode - SIFT关键点检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.sift` |
| 功能描述 | SIFT关键点检测（简化版，基于DoG） |
| 输入端口 | `image` - 输入灰度图像 |
| 输出端口 | `keypoints` - 关键点数组<br>`descriptors` - 描述符数组 |
| 参数 | `n_octaves` - octave层数 (默认: 4)<br>`threshold` - 响应阈值 (默认: 10) |

---

### 9.4 特征匹配节点

#### FeatureMatchNode - 特征匹配节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.feature_match` |
| 功能描述 | 关键点匹配（基于描述符欧氏距离） |
| 输入端口 | `keypoints1` - 第一图关键点<br>`descriptors1` - 第一图描述符<br>`keypoints2` - 第二图关键点<br>`descriptors2` - 第二图描述符 |
| 输出端口 | `matches` - 匹配结果数组 (Vector<FeatureMatch>)<br>`match_count` - 匹配数量 |
| 参数 | `ratio_threshold` - 比率测试阈值 (默认: 0.75) |

---

#### FeatureTrackNode - 特征跟踪节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.feature_track` |
| 功能描述 | Lucas-Kanade光流特征跟踪 |
| 输入端口 | `prev_image` - 前一帧灰度图<br>`curr_image` - 当前帧灰度图<br>`prev_keypoints` - 前一帧关键点 |
| 输出端口 | `tracks` - 跟踪结果数组 (Vector<TrackResult>)<br>`tracked_count` - 成功跟踪数量 |
| 参数 | `window_size` - 窗口大小 (默认: 15)<br>`max_iter` - 最大迭代次数 (默认: 20) |

---

## 10. 分割算法节点

### 10.1 阈值分割节点

#### ThresholdSegmentNode - 阈值分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.threshold_segment` |
| 功能描述 | 固定阈值分割，支持Otsu自动阈值 |
| 输入端口 | `image` - 输入灰度图像 (ImageData) |
| 输出端口 | `binary` - 二值图像 (ImageData)<br>`threshold_value` - 使用的阈值 |
| 参数 | `threshold_type` - 阈值类型 (Binary/BinaryInv/Trunc/ToZero/ToZeroInv)<br>`threshold_value` - 阈值值<br>`max_value` - 最大值 (默认: 255)<br>`auto_threshold` - 自动Otsu阈值 (默认: false) |

---

#### AdaptiveThresholdNode - 自适应阈值分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.adaptive_threshold` |
| 功能描述 | 局部自适应阈值分割 |
| 输入端口 | `image` - 输入灰度图像 |
| 输出端口 | `binary` - 二值图像 |
| 参数 | `method` - 自适应方法 (Mean/Gaussian)<br>`block_size` - 邻域块大小 (默认: 31)<br>`c` - 常数偏移 (默认: 5) |

---

### 10.2 高级分割节点

#### WatershedNode - 分水岭分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.watershed` |
| 功能描述 | 分水岭分割算法 |
| 输入端口 | `image` - 输入图像<br>`markers` - 标记图像（可选） |
| 输出端口 | `segmented` - 分割结果<br>`boundaries` - 分割边界<br>`num_regions` - 区域数量 |
| 参数 | `marker_method` - 标记生成方法 (Auto/User/Gradient)<br>`min_region_size` - 最小区域大小 |

---

#### RegionGrowingNode - 区域生长分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.region_growing` |
| 功能描述 | 区域生长分割 |
| 输入端口 | `image` - 输入图像<br>`seed_point` - 种子点（可选） |
| 输出端口 | `region` - 生长区域掩码<br>`region_area` - 区域面积<br>`region_mean` - 区域平均值 |
| 参数 | `seed_x` - 种子点X坐标<br>`seed_y` - 种子点Y坐标<br>`threshold` - 生长阈值 (默认: 10)<br>`connectivity` - 连通性 (4/8)<br>`auto_seed` - 自动选择种子点 |

---

#### KMeansSegmentNode - K均值聚类分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.kmeans_segment` |
| 功能描述 | K均值颜色聚类分割 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `segmented` - 分割结果（量化图像）<br>`labels` - 像素标签<br>`centers` - 聚类中心颜色 |
| 参数 | `k` - 聚类数量 (默认: 4)<br>`max_iterations` - 最大迭代次数 (默认: 100)<br>`convergence_threshold` - 收敛阈值 |

---

#### GrabCutNode - GrabCut分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.grabcut` |
| 功能描述 | GrabCut前景提取 |
| 输入端口 | `image` - 输入RGB图像<br>`rect` - 矩形区域（可选） |
| 输出端口 | `mask` - 分割掩码 (0:背景, 1:前景)<br>`foreground` - 前景提取图像<br>`background` - 背景图像 |
| 参数 | `rect_x` - 矩形左上角X<br>`rect_y` - 矩形左上角Y<br>`rect_w` - 矩形宽度<br>`rect_h` - 矩形高度<br>`iterations` - 迭代次数 (默认: 5) |

---

#### MeanShiftSegmentNode - MeanShift分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.meanshift_segment` |
| 功能描述 | MeanShift分割 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `segmented` - 分割结果<br>`labels` - 区域标签<br>`num_regions` - 区域数量 |
| 参数 | `spatial_radius` - 空间半径 (默认: 10)<br>`color_radius` - 颜色半径 (默认: 20)<br>`min_density` - 最小密度 (默认: 50) |

---

#### SuperpixelNode - 超像素分割节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.superpixel` |
| 功能描述 | SLIC超像素分割 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `labels` - 超像素标签<br>`boundaries` - 超像素边界<br>`num_superpixels` - 超像素数量<br>`superpixel_image` - 超像素平均颜色图像 |
| 参数 | `num_superpixels` - 超像素数量 (默认: 100)<br>`compactness` - 紧致度 (默认: 10)<br>`iterations` - 迭代次数 (默认: 10) |

---

## 11. 缺陷检测节点

### 11.1 表面缺陷节点

#### SurfaceDefectNode - 表面缺陷检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.surface_defect` |
| 功能描述 | 通用表面缺陷检测 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `defects` - 缺陷数组 (Vector<DefectResult>)<br>`defect_count` - 缺陷数量 |
| 参数 | `min_size` - 最小缺陷尺寸<br>`max_size` - 最大缺陷尺寸 |

---

#### ScratchDetectNode - 划痕检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.scratch_detect` |
| 功能描述 | 划痕缺陷检测 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `defects` - 划痕列表<br>`scratch_count` - 划痕数量 |

---

#### CrackDetectNode - 裂纹检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.crack_detect` |
| 功能描述 | 裂纹缺陷检测 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `defects` - 裂纹列表<br>`crack_count` - 裂纹数量 |

---

#### PitDetectNode - 凹坑检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.pit_detect` |
| 功能描述 | 凹坑缺陷检测 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `defects` - 凹坑列表<br>`pit_count` - 凹坑数量 |

---

### 11.2 形状缺陷节点

#### ShapeDefectNode - 形状缺陷检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.shape_defect` |
| 功能描述 | 形状异常缺陷检测 |
| 输入端口 | `image` - 输入图像<br>`reference_shape` - 参考形状（可选） |
| 输出端口 | `defects` - 形状缺陷列表 |

---

#### DimensionDefectNode - 尺寸缺陷检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.dimension_defect` |
| 功能描述 | 尺寸超差缺陷检测 |
| 输入端口 | `image` - 输入图像<br>`nominal_dimensions` - 标称尺寸 |
| 输出端口 | `defects` - 尺寸缺陷列表 |

---

### 11.3 外观缺陷节点

#### ColorDefectNode - 颜色缺陷检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.color_defect` |
| 功能描述 | 颜色异常缺陷检测 |
| 输入端口 | `image` - 输入图像<br>`reference_color` - 参考颜色 |
| 输出端口 | `defects` - 颜色缺陷列表 |

---

#### TextureDefectNode - 纹理缺陷检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.texture_defect` |
| 功能描述 | 纹理异常缺陷检测 |
| 输入端口 | `image` - 输入图像<br>`reference_texture` - 参考纹理（可选） |
| 输出端口 | `defects` - 纹理缺陷列表 |

---

#### ForeignObjectNode - 异物检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.foreign_object` |
| 功能描述 | 异物检测 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `defects` - 异物列表<br>`foreign_count` - 异物数量 |

---

#### MissingObjectNode - 缺失检测节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.missing_object` |
| 功能描述 | 缺失对象检测 |
| 输入端口 | `image` - 输入图像<br>`expected_objects` - 期望对象列表 |
| 输出端口 | `missing_list` - 缺失对象列表<br>`missing_count` - 缺失数量 |

---

## 12. 条码识别节点

### 12.1 通用条码节点

#### BarcodeDecodeNode - 条码解码节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.barcode_decode` |
| 功能描述 | 通用条码解码（支持一维码和二维码） |
| 输入端口 | `image` - 输入图像 (ImageData) |
| 输出端口 | `result` - 解码结果 (BarcodeResult)<br>`data` - 解码内容<br>`type` - 条码类型<br>`corners` - 四角坐标<br>`quality` - 质量/置信度 |
| 参数 | `barcode_type` - 条码类型 (AUTO/CODE128/QR_CODE等, 默认: AUTO) |

---

#### QRCodeNode - QR码解码节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.qr_code` |
| 功能描述 | QR码专用解码 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `data` - 解码内容<br>`corners` - 定位点坐标<br>`quality` - 解码质量 |

---

#### DataMatrixNode - DataMatrix码解码节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.data_matrix` |
| 功能描述 | DataMatrix码专用解码 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `data` - 解码内容<br>`corners` - 定位点坐标 |

---

#### QRGenerateNode - QR码生成节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.qr_generate` |
| 功能描述 | QR码生成 |
| 输入端口 | `data` - 待编码数据 |
| 输出端口 | `qr_image` - QR码图像 (ImageData) |
| 参数 | `size` - 图像尺寸 (默认: 200) |

---

### 12.2 条码验证节点

#### BarcodeVerifyNode - 条码验证节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.barcode_verify` |
| 功能描述 | 条码质量检测和验证 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `is_valid` - 是否有效<br>`quality_score` - 质量分数<br>`error_details` - 错误详情 |

---

#### BarcodeGradeNode - 条码评级节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.barcode_grade` |
| 功能描述 | ISO/IEC标准条码质量评级 |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `grade` - 评级结果 (BarcodeGradeResult)<br>`overall_grade` - 总体评级 (0.0-4.0)<br>`grade_letter` - 等级字母 (A/B/C/D/F)<br>`symbol_contrast` - 符号对比度<br>`modulation` - 调制<br>`defects` - 缺陷<br>`decodability` - 可解码性 |

---

## 13. 模板匹配节点

#### TemplateMatchNode - 模板匹配节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.template_match` |
| 功能描述 | 单模板匹配 |
| 输入端口 | `image` - 源图像<br>`template` - 模板图像（可通过参数设置） |
| 输出端口 | `result` - 匹配结果 (MatchResult)<br>`x` - 匹配位置X<br>`y` - 匹配位置Y<br>`score` - 匹配分数<br>`angle` - 旋转角度<br>`scale` - 缩放比例 |
| 参数 | `method` - 匹配方法 (SAD/SSD/NCC/NCC_SAD, 默认: NCC)<br>`threshold` - 匹配分数阈值 (默认: 0.7)<br>`template_image` - 模板图像（可设置） |

---

#### TemplateTrainNode - 模板训练节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.template_train` |
| 功能描述 | 从ROI区域创建模板 |
| 输入端口 | `image` - 源图像 |
| 输出端口 | `template` - 生成的模板图像 |
| 参数 | `roi_x` - ROI起始X<br>`roi_y` - ROI起始Y<br>`roi_width` - ROI宽度<br>`roi_height` - ROI高度 |

---

#### MultiTemplateMatchNode - 多模板匹配节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.multi_template_match` |
| 功能描述 | 多模板同时匹配 |
| 输入端口 | `image` - 源图像<br>`templates` - 模板图像数组 |
| 输出端口 | `results` - 匹配结果数组<br>`best_match` - 最佳匹配结果 |
| 参数 | `method` - 匹配方法<br>`threshold` - 匹配阈值 |

---

## 14. 图像运算节点

### 14.1 算术运算节点

#### ImageAddNode - 图像加法节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_add` |
| 功能描述 | 两图像相加 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |
| 参数 | `scale_factor` - 缩放因子 (默认: 1.0)<br>`offset` - 偏移值 (默认: 0.0) |

---

#### ImageSubtractNode - 图像减法节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_subtract` |
| 功能描述 | 两图像相减（差分） |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |

---

#### ImageMultiplyNode - 图像乘法节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_multiply` |
| 功能描述 | 两图像相乘 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |

---

#### ImageDivideNode - 图像除法节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_divide` |
| 功能描述 | 两图像相除 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |

---

#### ImageAbsDiffNode - 绝对差值节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_abs_diff` |
| 功能描述 | 两图像绝对差值 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 绝对差值图像 |

---

#### ImageMinNode - 图像最小值节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_min` |
| 功能描述 | 两图像逐像素取最小值 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 最小值图像 |

---

#### ImageMaxNode - 图像最大值节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_max` |
| 功能描述 | 两图像逐像素取最大值 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 最大值图像 |

---

### 14.2 位运算节点

#### ImageAndNode - 位与运算节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_and` |
| 功能描述 | 两图像位与运算 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |

---

#### ImageOrNode - 位或运算节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_or` |
| 功能描述 | 两图像位或运算 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |

---

#### ImageXorNode - 位异或运算节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_xor` |
| 功能描述 | 两图像位异或运算 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 结果图像 |

---

#### ImageNotNode - 位非运算节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_not` |
| 功能描述 | 图像位非运算（反转） |
| 输入端口 | `image` - 输入图像 |
| 输出端口 | `result` - 反转图像 |

---

### 14.3 混合运算节点

#### ImageBlendNode - 图像混合节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.image_blend` |
| 功能描述 | 两图像加权混合 |
| 输入端口 | `image_a` - 第一图像<br>`image_b` - 第二图像 |
| 输出端口 | `result` - 混合图像 |
| 参数 | `blend_weight` - 第一图像权重 (默认: 0.5, 范围: 0.0-1.0) |

---

## 15. 标定节点

#### NinePointCalibrationNode - 九点标定节点

| 属性 | 说明 |
|------|------|
| 节点ID | `algorithm.nine_point_calibration` |
| 功能描述 | 九点标定算法（最小二乘法求解仿射变换） |
| 输入端口 | `calibration_points` - 标定点数组 (Vector<CalibrationPoint>) |
| 输出端口 | `result` - 标定结果 (CalibrationResult)<br>`a, b, c, d, e, f` - 仿射变换参数<br>`error` - 平均误差<br>`max_error` - 最大误差<br>`valid` - 是否有效 |
| 参数 | 无（标定点通过输入端口提供） |

**标定应用：**

```
world_x = a * image_x + b * image_y + c
world_y = d * image_x + e * image_y + f
```

---

## 附录

### A. 节点总数统计

| 分类 | 节点数量 |
|------|----------|
| 图像处理节点 | 52 |
| 亚像素精度节点 | 10 |
| Blob分析节点 | 1 |
| 测量工具节点 | 30 |
| OCR识别节点 | 13 |
| 半导体晶圆检测节点 | 10 |
| 汽车零部件检测节点 | 10 |
| CUDA加速节点 | 8 |
| 特征检测节点 | 15 |
| 分割算法节点 | 12 |
| 缺陷检测节点 | 15 |
| 条码识别节点 | 12 |
| 模板匹配节点 | 3 |
| 图像运算节点 | 12 |
| 标定节点 | 1 |
| **总计** | **约200+** |

### B. 基础数据类型

#### ImageData - 图像数据结构

| 属性 | 类型 | 说明 |
|------|------|------|
| width | uint32_t | 图像宽度 |
| height | uint32_t | 图像高度 |
| channels | uint32_t | 通道数 |
| format | ImageFormat | 图像格式 |
| data | Vector<uint8_t> | 像素数据 |
| timestamp | uint64_t | 时间戳 |
| frame_id | uint32_t | 帧ID |
| source_id | String | 源ID |

#### Region - 区域结构

| 属性 | 类型 | 说明 |
|------|------|------|
| x | int32_t | 区域起始X |
| y | int32_t | 区域起始Y |
| width | int32_t | 区域宽度 |
| height | int32_t | 区域高度 |

### C. 图像格式枚举

| 格式 | 说明 |
|------|------|
| Mono8 | 8位灰度 |
| Mono16 | 16位灰度 |
| RGB8 | 24位RGB |
| BGR8 | 24位BGR |
| RGBA8 | 32位RGBA |
| Float32 | 32位浮点 |

---

**版本**: v0.1.0
**更新日期**: 2025-07-05
**文档维护**: OpenVisionFlow Team