# OpenVisionFlow Web编辑器 - 图像处理示例

## 目录
1. [基础概念](#基础概念)
2. [示例1: 简单图像生成](#示例1-简单图像生成)
3. [示例2: 图像预处理流程](#示例2-图像预处理流程)
4. [示例3: 阈值处理流程](#示例3-阈值处理流程)
5. [示例4: 3D模型加载与匹配](#示例4-3d模型加载与匹配)
6. [示例5: 条码识别流程](#示例5-条码识别流程)

---

## 基础概念

### Web编辑器界面
- **左侧面板**: 节点类型列表（501个可用节点）
- **中间画布**: 流程图编辑区域
- **右侧面板**: 节点属性配置
- **顶部菜单**: 新建/保存/运行/单步调试

### 节点类型
- **输入节点**: 从文件/摄像头获取图像
- **处理节点**: 图像处理、特征提取、测量等
- **输出节点**: 显示结果、保存文件

### 连接规则
- **输出端口**（右侧圆点）→ **输入端口**（左侧圆点）
- 数据从左向右流动
- 一个输出可以连接多个输入

---

## 示例1: 简单图像生成

### 目标
生成一个测试图像用于后续处理

### 步骤

#### 1. 添加图像源节点
```
节点类型: SimpleImageSource
位置: 图像采集 → 图像源
操作: 从左侧面板拖拽到画布
```

#### 2. 配置参数
在右侧属性面板中设置：
```
宽度 (width): 640
高度 (height): 480
```

#### 3. 查看输出
运行流程后，图像源节点会生成：
- 640x480的RGB测试图像
- 包含渐变背景和红色圆形

### JSON配置
```json
{
  "nodes": [
    {
      "id": "node_001",
      "type": "SimpleImageSource",
      "name": "图像源",
      "x": 100,
      "y": 200,
      "params": {
        "width": 640,
        "height": 480
      }
    }
  ],
  "connections": []
}
```

---

## 示例2: 图像预处理流程

### 目标
对图像进行模糊处理，减少噪声

### 流程图
```
[图像源] → [图像预处理] → (输出)
```

### 步骤

#### 1. 创建图像源节点
```
节点: SimpleImageSource
参数: width=512, height=512
```

#### 2. 添加预处理节点
```
节点类型: SimpleImagePreprocess
位置: 图像处理 → 图像预处理
操作: 拖拽到图像源右侧
```

#### 3. 连接节点
- 鼠标悬停在图像源的**输出端口**（右侧圆点）
- 按住左键拖拽到预处理节点的**输入端口**（左侧圆点）
- 释放鼠标完成连接

#### 4. 运行流程
点击顶部菜单的"运行"按钮，系统会：
1. 生成测试图像
2. 应用3x3均值滤波
3. 输出模糊后的图像

### JSON配置
```json
{
  "nodes": [
    {
      "id": "node_001",
      "type": "SimpleImageSource",
      "name": "图像源",
      "x": 100,
      "y": 200,
      "params": {
        "width": 512,
        "height": 512
      }
    },
    {
      "id": "node_002",
      "type": "SimpleImagePreprocess",
      "name": "图像预处理",
      "x": 350,
      "y": 200,
      "params": {}
    }
  ],
  "connections": [
    {
      "id": "conn_001",
      "fromNode": "node_001",
      "fromPort": 0,
      "toNode": "node_002",
      "toPort": 0
    }
  ]
}
```

---

## 示例3: 阈值处理流程

### 目标
将灰度图像转换为二值图像

### 流程图
```
[图像源] → [图像预处理] → [阈值化] → (输出)
```

### 步骤

#### 1-3. 完成示例2的流程

#### 4. 添加阈值化节点
```
节点类型: SimpleThreshold
位置: 图像处理 → 阈值化
参数:
  - threshold: 128
  - max_value: 255
```

#### 5. 连接节点
```
预处理节点输出 → 阈值化节点输入
```

#### 6. 运行并查看结果
- 像素值 ≥ 128 → 变为255（白色）
- 像素值 < 128 → 变为0（黑色）

### 参数说明
| 参数 | 含义 | 取值范围 |
|------|------|----------|
| threshold | 分割阈值 | 0-255 |
| max_value | 最大值 | 0-255 |

### JSON配置
```json
{
  "nodes": [
    {
      "id": "node_001",
      "type": "SimpleImageSource",
      "name": "图像源",
      "x": 100,
      "y": 200,
      "params": {
        "width": 512,
        "height": 512
      }
    },
    {
      "id": "node_002",
      "type": "SimpleImagePreprocess",
      "name": "图像预处理",
      "x": 350,
      "y": 200,
      "params": {}
    },
    {
      "id": "node_003",
      "type": "SimpleThreshold",
      "name": "阈值化",
      "x": 600,
      "y": 200,
      "params": {
        "threshold": 128,
        "max_value": 255
      }
    }
  ],
  "connections": [
    {
      "id": "conn_001",
      "fromNode": "node_001",
      "fromPort": 0,
      "toNode": "node_002",
      "toPort": 0
    },
    {
      "id": "conn_002",
      "fromNode": "node_002",
      "fromPort": 0,
      "toNode": "node_003",
      "toPort": 0
    }
  ]
}
```

---

## 示例4: 3D模型加载与匹配

### 目标
加载DXF格式的3D CAD模型并进行形状匹配

### 流程图
```
[DXF文件加载] → [创建3D形状模型] → [3D模型训练] → [3D形状匹配]
```

### 节点说明

#### 1. DXF文件加载节点
```
节点: DXFLoader
功能: 加载DXF格式的CAD模型文件
参数:
  - file_path: "path/to/model.dxf"
输出: 3D几何数据
```

#### 2. 创建3D形状模型节点
```
节点: CreateShapeModel3D
功能: 从几何数据创建可匹配的3D模型
输入: DXF几何数据
输出: 3D形状模型
```

#### 3. 3D模型训练节点
```
节点: ShapeModel3DTrain
功能: 优化模型参数，提高匹配精度
参数:
  - optimization_level: "high"
  - pyramid_levels: 4
```

#### 4. 3D形状匹配节点
```
节点: FindShapeModel3D
功能: 在图像中搜索3D模型的位置和姿态
输入:
  - 图像数据
  - 训练好的3D模型
输出:
  - 匹配位置 (x, y, z)
  - 旋转角度 (roll, pitch, yaw)
  - 匹配分数
```

### 应用场景
- 工业零件定位
- 机器人抓取引导
- 质量检测

---

## 示例5: 条码识别流程

### 目标
识别图像中的二维码/条形码

### 流程图
```
[图像源] → [QR码解码] → (文本结果)
```

### 步骤

#### 1. 准备包含二维码的图像
```
选项A: 使用摄像头实时采集
选项B: 加载包含二维码的图片文件
选项C: 使用QR码生成节点创建测试二维码
```

#### 2. 添加QR码解码节点
```
节点: QRCodeDecode
位置: 条码识别 → QR码解码
输入: 图像数据
输出:
  - decoded_text: 解码文本
  - format: 二维码格式
  - position: 二维码位置
```

#### 3. 查看解码结果
右侧属性面板会显示：
```
解码文本: "https://example.com"
格式: "QR_CODE"
位置: (x=100, y=150, w=200, h=200)
```

### 扩展：批量条码验证
```
[图像源] → [条码解码] → [条码验证] → [条码评级]
```

#### 条码验证节点
```
节点: BarcodeVerify
功能: 验证条码是否符合标准
参数:
  - standard: "ISO/IEC 15415"
输出: 验证结果（通过/失败）
```

#### 条码评级节点
```
节点: BarcodeGrade
功能: 按照ISO标准对条码质量评级
输出:
  - grade_letter: "A", "B", "C", "D", "F"
  - symbol_contrast: 符号对比度
  - modulation: 调制系数
  - defects: 缺陷等级
```

### JSON配置
```json
{
  "nodes": [
    {
      "id": "node_001",
      "type": "SimpleImageSource",
      "name": "图像源",
      "x": 100,
      "y": 200,
      "params": {
        "width": 640,
        "height": 480
      }
    },
    {
      "id": "node_002",
      "type": "QRCodeDecode",
      "name": "QR码解码",
      "x": 350,
      "y": 200,
      "params": {}
    }
  ],
  "connections": [
    {
      "id": "conn_001",
      "fromNode": "node_001",
      "fromPort": 0,
      "toNode": "node_002",
      "toPort": 0
    }
  ]
}
```

---

## 高级技巧

### 1. 使用搜索功能
在节点面板顶部搜索框中输入关键词：
```
示例:
- 输入"qr" → 显示所有QR码相关节点
- 输入"3d" → 显示所有3D处理节点
- 输入"测量" → 显示所有测量节点
```

### 2. 快捷键
| 快捷键 | 功能 |
|--------|------|
| Delete | 删除选中节点 |
| Ctrl+C | 复制节点 |
| Ctrl+V | 粘贴节点 |
| Ctrl+S | 保存流程 |
| Space+拖拽 | 平移画布 |
| 滚轮 | 缩放视图 |

### 3. 节点状态
- **灰色**: 就绪状态
- **绿色**: 执行成功
- **红色**: 执行失败
- **黄色**: 正在执行
- **半透明**: 已禁用

### 4. 调试技巧

#### 单步调试
```
1. 选中某个节点
2. 点击"单步"按钮
3. 查看该节点的执行结果
```

#### 查看中间结果
```
1. 点击已执行的节点
2. 右侧属性面板显示输出数据
3. 图像节点会显示缩略图
```

### 5. 批量处理
对于批量图像处理：
```
[图像源1] ─┐
[图像源2] ─┼→ [数据合并] → [批量处理] → (结果)
[图像源3] ─┘
```

---

## 常见问题

### Q1: 节点无法连接？
**A**: 检查数据类型是否匹配：
- 图像输出 → 图像输入 ✓
- 数值输出 → 数值输入 ✓
- 图像输出 → 数值输入 ✗

### Q2: 节点执行失败？
**A**: 检查输入数据：
1. 右键点击节点 → 查看详细信息
2. 确认前置节点是否成功执行
3. 检查参数设置是否合理

### Q3: 如何保存流程？
**A**: 两种方式：
1. 点击"保存"按钮 → 导出JSON文件
2. 使用 Ctrl+S 快捷键

### Q4: 如何加载已有流程？
**A**: 点击"打开"按钮 → 选择.json文件

### Q5: 如何分享流程？
**A**: 将导出的JSON文件分享给他人，对方导入即可使用

---

## 访问地址

**Web编辑器**: http://localhost:8080/

**API文档**: http://localhost:8080/api/nodes

---

## 节点分类索引

### 图像采集 (5个节点)
- SimpleImageSource - 图像源
- ImageLoad - 图像加载
- CameraCapture - 摄像头采集
- VideoSource - 视频源
- ImageSequence - 图像序列

### 图像处理 (14个节点)
- SimpleImagePreprocess - 图像预处理
- SimpleThreshold - 阈值化
- GrayscaleConvert - 灰度转换
- ColorSpaceConvert - 颜色空间转换
- ImageResize - 图像缩放
- ImageRotate - 图像旋转
- ImageCrop - 图像裁剪
- ImageFlip - 图像翻转
- BrightnessAdjust - 亮度调整
- ContrastAdjust - 对比度调整
- GammaCorrect - Gamma校正
- HistogramEqualize - 直方图均衡
- WhiteBalance - 白平衡
- Denoise - 去噪

### 特征检测 (15个节点)
- EdgeDetection - 边缘检测
- CornerDetection - 角点检测
- BlobDetection - 斑点检测
- ContourDetection - 轮廓检测
- LineDetection - 直线检测
- CircleDetection - 圆检测
- EllipseDetection - 椭圆检测
- RectangleDetection - 矩形检测
- TriangleDetection - 三角形检测
- PolygonDetection - 多边形检测
- FeatureMatch - 特征匹配
- TemplateMatch - 模板匹配
- SIFTFeature - SIFT特征
- SURFFeature - SURF特征
- ORBFeature - ORB特征

### 测量 (20个节点)
- DistanceMeasure - 距离测量
- AngleMeasure - 角度测量
- AreaMeasure - 面积测量
- PerimeterMeasure - 周长测量
- DiameterMeasure - 直径测量
- RadiusMeasure - 半径测量
- CenterMeasure - 中心点测量
- WidthMeasure - 宽度测量
- HeightMeasure - 高度测量
- ThicknessMeasure - 厚度测量
- GapMeasure - 间隙测量
- PositionMeasure - 位置测量
- OrientationMeasure - 方向测量
- SymmetryMeasure - 对称性测量
- EccentricityMeasure - 偏心率测量
- CompactnessMeasure - 紧凑度测量
- CircularityMeasure - 圆度测量
- RectangularityMeasure - 矩形度测量
- AspectRatioMeasure - 长宽比测量
- StraightnessMeasure - 直线度测量

### 条码识别 (7个节点)
- QRCodeDecode - QR码解码
- QRGenerate - QR码生成
- BarcodeDecode - 条码解码
- DataMatrixDecode - DataMatrix解码
- BarcodeTrain - 条码训练
- BarcodeVerify - 条码验证
- BarcodeGrade - 条码评级

### 3D匹配 (12个节点)
- DXFLoader - DXF文件加载
- CreateShapeModel3D - 创建3D形状模型
- ShapeModel3DTrain - 3D模型训练
- ShapeModel3DSerialize - 3D模型序列化
- FindShapeModel3D - 3D形状匹配
- FindShapeModel3DCluttered - 乱序3D匹配
- FindShapeModel3DMulti - 多3D模型匹配
- Match3DRefine - 3D匹配优化
- GetShapeModel3DMatches - 获取匹配结果
- ShapeModel3DProject - 3D模型投影
- ShapeModel3DTransform - 3D模型变换
- ShapeModel3DVisualize - 3D可视化

---

## 技术支持

如需更多帮助，请查看：
- 项目文档: `docs/` 目录
- API参考: `docs/api_reference.md`
- 架构说明: `docs/architecture.md`