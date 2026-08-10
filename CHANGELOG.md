# Changelog

本项目的所有重要更改都将记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
并且本项目遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [0.2.0] - 2026-07-05

### Added（新增）
- P0突破功能
  - 亚像素精度节点（10个）：Taylor/Parabola/Gaussian/Saddle/Zernike算法
  - 汉字OCR节点（10个）：GB2312汉字3755个支持
  - Web编辑器增强（8大功能模块）
  
- P1突破功能
  - 半导体晶圆检测节点（10个）
  - 汽车零部件检测节点（10个）
  - 调试工具完善（断点/单步/变量监视）
  - 执行日志可视化（WebSocket实时推送）
  - CUDA加速关键算子（8个）

- 新增节点总计：501个算子节点

### Changed（变更）
- 重构核心模块接口（INode基类）
- 统一数据类型系统（Data类）
- 优化流程引擎性能

### Fixed（修复）
- 修复30+文件的编译错误
- 修复LNK2005重复定义问题
- 修复API接口兼容性问题

### Performance（性能）
- CUDA加速：5-10x性能提升
- 亚像素精度：±0.005像素

## [0.1.0] - 2026-01-15

### Added
- 项目初始化
- 核心框架搭建
- 基础节点实现