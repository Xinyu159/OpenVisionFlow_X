/**
 * OpenVisionFlow Web Editor Pro
 * 版本: 3.0
 *
 * 核心特性:
 * 1. 零代码流程配置 - 拖拽创建、智能连线、参数可视化编辑、实时预览、一键运行
 * 2. 节点管理增强 - 分类树、搜索、收藏夹、版本管理、自定义节点导入
 * 3. 画布交互增强 - 无限画布、网格吸附、对齐工具、框选、右键菜单、小地图
 * 4. 调试功能增强 - 断点、单步、变量查看器、性能分析、错误定位
 * 5. 可视化增强 - 图像预览、直方图、结果表格、缺陷标注、Chart.js图表
 * 6. 流程管理 - 模板库、导入导出、版本管理、Diff视图、批处理
 * 7. 完整快捷键支持
 * 8. 深色/浅色主题切换
 */

'use strict';

// ==================== 全局配置 ====================
const CONFIG = {
    API_BASE: 'http://localhost:8080/api',
    WS_URL: 'ws://localhost:8080/ws',
    GRID_SIZE: 20,
    NODE_WIDTH: 180,
    NODE_HEADER_HEIGHT: 32,
    NODE_PORT_SPACING: 22,
    NODE_BODY_PADDING: 10,
    PORT_SIZE: 12,
    PORT_HIT_RADIUS: 14,
    CONNECTION_WIDTH: 2.5,
    ZOOM_MIN: 0.2,
    ZOOM_MAX: 4,
    ZOOM_STEP: 0.15,
    ZOOM_WHEEL_FACTOR: 0.0015,
    SNAP_TO_GRID: true,
    ANIMATION_DURATION: 300,
    MAX_UNDO_STACK: 100,
    AUTO_SAVE_INTERVAL: 60000, // 1分钟
    THEME_KEY: 'ovf-theme',
    FAVORITES_KEY: 'ovf-favorites',
    CUSTOM_NODES_KEY: 'ovf-custom-nodes'
};

// ==================== 状态管理 ====================
class EditorState {
    constructor() {
        this.nodes = new Map();
        this.connections = [];
        this.groups = [];
        this.comments = [];
        this.selectedNodes = [];
        this.selectedConnections = [];
        this.clipboard = { nodes: [], connections: [] };
        this.undoStack = [];
        this.redoStack = [];
        this.zoom = 1;
        this.panOffset = { x: 0, y: 0 };
        this.gridVisible = true;
        this.snapEnabled = true;
        this.minimapVisible = true;
        this.debugMode = false;
        this.breakpoints = new Set();
        this.isRunning = false;
        this.isStepping = false;
        this.currentFlowId = null;
        this.currentFlowName = '未命名';
        this.modified = false;
        this.theme = localStorage.getItem(CONFIG.THEME_KEY) || 'dark';
        this.favorites = JSON.parse(localStorage.getItem(CONFIG.FAVORITES_KEY) || '[]');
        this.customNodes = JSON.parse(localStorage.getItem(CONFIG.CUSTOM_NODES_KEY) || '[]');
        this.nodeVersions = new Map(); // 节点版本管理
        this.executionStats = new Map(); // 节点执行统计
        this.flowVersions = []; // 流程版本历史
        this.activeGroup = null;
        this.currentLogLevel = 'all';
        this.logFilter = '';
        this.autoScroll = true;
    }

    addNode(node) {
        this.nodes.set(node.id, node);
        this.markModified();
    }

    removeNode(nodeId) {
        this.nodes.delete(nodeId);
        this.connections = this.connections.filter(c =>
            c.sourceId !== nodeId && c.targetId !== nodeId);
        this.breakpoints.delete(nodeId);
        this.groups = this.groups.filter(g => !g.nodeIds.includes(nodeId));
        this.markModified();
    }

    addConnection(connection) {
        // 检查重复连接
        const exists = this.connections.some(c =>
            c.sourceId === connection.sourceId &&
            c.sourcePort === connection.sourcePort &&
            c.targetId === connection.targetId &&
            c.targetPort === connection.targetPort
        );
        if (!exists) {
            this.connections.push(connection);
            this.markModified();
            return true;
        }
        return false;
    }

    removeConnection(connectionId) {
        this.connections = this.connections.filter(c => c.id !== connectionId);
        this.markModified();
    }

    selectNode(nodeId, addToSelection = false) {
        if (addToSelection) {
            if (!this.selectedNodes.includes(nodeId)) {
                this.selectedNodes.push(nodeId);
            } else {
                // 取消选择
                this.selectedNodes = this.selectedNodes.filter(id => id !== nodeId);
            }
        } else {
            this.selectedNodes = [nodeId];
        }
        this.selectedConnections = [];
    }

    selectAll() {
        this.selectedNodes = Array.from(this.nodes.keys());
    }

    deselectAll() {
        this.selectedNodes = [];
        this.selectedConnections = [];
    }

    copySelected() {
        const selectedSet = new Set(this.selectedNodes);
        this.clipboard = {
            nodes: this.selectedNodes.map(id => {
                const node = this.nodes.get(id);
                return JSON.parse(JSON.stringify(node));
            }),
            connections: this.connections.filter(c =>
                selectedSet.has(c.sourceId) && selectedSet.has(c.targetId)
            ).map(c => ({ ...c }))
        };
    }

    paste(offsetX = 50, offsetY = 50) {
        if (this.clipboard.nodes.length === 0) return [];

        const idMap = new Map();
        const newNodes = [];

        this.clipboard.nodes.forEach(nodeTemplate => {
            const newId = generateId();
            idMap.set(nodeTemplate.id, newId);
            const newNode = {
                ...JSON.parse(JSON.stringify(nodeTemplate)),
                id: newId,
                x: nodeTemplate.x + offsetX,
                y: nodeTemplate.y + offsetY
            };
            if (CONFIG.SNAP_TO_GRID) {
                newNode.x = Math.round(newNode.x / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
                newNode.y = Math.round(newNode.y / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
            }
            this.addNode(newNode);
            newNodes.push(newNode);
        });

        // 复制相关连接
        this.clipboard.connections.forEach(conn => {
            const newSourceId = idMap.get(conn.sourceId);
            const newTargetId = idMap.get(conn.targetId);
            if (newSourceId && newTargetId) {
                this.addConnection({
                    ...conn,
                    id: generateId(),
                    sourceId: newSourceId,
                    targetId: newTargetId
                });
            }
        });

        return newNodes;
    }

    markModified() {
        this.modified = true;
        updateTitle();
    }

    clearModified() {
        this.modified = false;
        updateTitle();
    }

    pushUndo() {
        const snapshot = {
            nodes: Array.from(this.nodes.values()).map(n => JSON.parse(JSON.stringify(n))),
            connections: this.connections.map(c => ({ ...c })),
            groups: JSON.parse(JSON.stringify(this.groups))
        };
        this.undoStack.push(snapshot);
        if (this.undoStack.length > CONFIG.MAX_UNDO_STACK) {
            this.undoStack.shift();
        }
        this.redoStack = [];
    }

    undo() {
        if (this.undoStack.length === 0) return false;
        const current = {
            nodes: Array.from(this.nodes.values()).map(n => JSON.parse(JSON.stringify(n))),
            connections: this.connections.map(c => ({ ...c })),
            groups: JSON.parse(JSON.stringify(this.groups))
        };
        this.redoStack.push(current);
        const snapshot = this.undoStack.pop();
        this.restoreSnapshot(snapshot);
        return true;
    }

    redo() {
        if (this.redoStack.length === 0) return false;
        const current = {
            nodes: Array.from(this.nodes.values()).map(n => JSON.parse(JSON.stringify(n))),
            connections: this.connections.map(c => ({ ...c })),
            groups: JSON.parse(JSON.stringify(this.groups))
        };
        this.undoStack.push(current);
        const snapshot = this.redoStack.pop();
        this.restoreSnapshot(snapshot);
        return true;
    }

    restoreSnapshot(snapshot) {
        this.nodes.clear();
        snapshot.nodes.forEach(n => this.nodes.set(n.id, n));
        this.connections = snapshot.connections.map(c => ({ ...c }));
        this.groups = snapshot.groups || [];
        this.selectedNodes = this.selectedNodes.filter(id => this.nodes.has(id));
        this.markModified();
        renderCanvas();
        updateStatusBar();
        updateParamsPanel();
    }

    toJSON() {
        return {
            id: this.currentFlowId || generateId(),
            name: this.currentFlowName,
            version: '3.0',
            nodes: Array.from(this.nodes.values()),
            connections: this.connections,
            groups: this.groups,
            comments: this.comments,
            created_time: new Date().toISOString(),
            modified_time: new Date().toISOString()
        };
    }

    fromJSON(json) {
        this.nodes.clear();
        this.connections = [];
        this.groups = [];
        this.comments = [];
        json.nodes.forEach(n => this.nodes.set(n.id, n));
        this.connections = json.connections || [];
        this.groups = json.groups || [];
        this.comments = json.comments || [];
        this.currentFlowId = json.id;
        this.currentFlowName = json.name || '未命名';
        this.clearModified();
        renderCanvas();
        updateStatusBar();
        updateFlowName();
    }

    toggleFavorite(nodeTypeId) {
        const idx = this.favorites.indexOf(nodeTypeId);
        if (idx >= 0) {
            this.favorites.splice(idx, 1);
        } else {
            this.favorites.push(nodeTypeId);
        }
        localStorage.setItem(CONFIG.FAVORITES_KEY, JSON.stringify(this.favorites));
    }

    isFavorite(nodeTypeId) {
        return this.favorites.includes(nodeTypeId);
    }

    addCustomNode(nodeDef) {
        this.customNodes.push(nodeDef);
        localStorage.setItem(CONFIG.CUSTOM_NODES_KEY, JSON.stringify(this.customNodes));
    }
}

const state = new EditorState();

// ==================== 节点类型定义 ====================
const NODE_CATEGORIES = {
    '输入': {
        icon: 'fa fa-sign-in',
        color: '#89b4fa',
        nodes: [
            {
                id: 'image.load', name: '图像加载', icon: 'fa fa-picture-o', color: '#89b4fa',
                inputs: [],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    file_path: { name: '文件路径', type: 'string', default: '' },
                    auto_reload: { name: '自动重载', type: 'boolean', default: false }
                }
            },
            {
                id: 'camera.capture', name: '相机采集', icon: 'fa fa-video-camera', color: '#89b4fa',
                inputs: [],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    device_id: { name: '设备ID', type: 'number', min: 0, max: 16, default: 0 },
                    exposure: { name: '曝光时间', type: 'number', min: 100, max: 100000, default: 5000 },
                    gain: { name: '增益', type: 'number', min: 0, max: 32, default: 1 }
                }
            },
            {
                id: 'param.input', name: '参数输入', icon: 'fa fa-sliders', color: '#f9e2af',
                inputs: [],
                outputs: [{ id: 'value', name: '值', type: 'number' }],
                params: {
                    value: { name: '数值', type: 'number', min: 0, max: 10000, default: 0 },
                    name: { name: '参数名', type: 'string', default: 'param' }
                }
            }
        ]
    },
    '图像处理': {
        icon: 'fa fa-cogs',
        color: '#cba6f7',
        nodes: [
            {
                id: 'image.threshold', name: '阈值分割', icon: 'fa fa-adjust', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '二值图', type: 'image' }],
                params: {
                    threshold_value: { name: '阈值', type: 'number', min: 0, max: 255, default: 128 },
                    max_value: { name: '最大值', type: 'number', min: 0, max: 255, default: 255 },
                    threshold_type: {
                        name: '类型', type: 'select',
                        options: ['Binary', 'BinaryInv', 'Trunc', 'ToZero', 'Otsu', 'Triangle'],
                        default: 'Binary'
                    }
                }
            },
            {
                id: 'image.filter', name: '滤波', icon: 'fa fa-filter', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    filter_type: { name: '滤波类型', type: 'select', options: ['Gaussian', 'Median', 'Blur', 'Bilateral'], default: 'Gaussian' },
                    kernel_size: { name: '核大小', type: 'number', min: 1, max: 31, default: 5, step: 2 }
                }
            },
            {
                id: 'image.morphology', name: '形态学', icon: 'fa fa-cube', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    operation: { name: '操作', type: 'select', options: ['Erode', 'Dilate', 'Open', 'Close', 'Gradient', 'TopHat', 'BlackHat'], default: 'Open' },
                    kernel_size: { name: '核大小', type: 'number', min: 1, max: 31, default: 3, step: 2 },
                    iterations: { name: '迭代次数', type: 'number', min: 1, max: 20, default: 1 }
                }
            },
            {
                id: 'image.edge_detection', name: '边缘检测', icon: 'fa fa-bolt', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '边缘图', type: 'image' }],
                params: {
                    algorithm: { name: '算法', type: 'select', options: ['Canny', 'Sobel', 'Laplacian', 'Scharr'], default: 'Canny' },
                    threshold1: { name: '阈值1', type: 'number', min: 0, max: 500, default: 50 },
                    threshold2: { name: '阈值2', type: 'number', min: 0, max: 500, default: 150 }
                }
            },
            {
                id: 'image.color_convert', name: '颜色转换', icon: 'fa fa-paint-brush', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    conversion: { name: '转换方式', type: 'select', options: ['BGR2GRAY', 'GRAY2BGR', 'BGR2HSV', 'HSV2BGR', 'BGR2RGB', 'RGB2BGR'], default: 'BGR2GRAY' }
                }
            },
            {
                id: 'image.enhance', name: '图像增强', icon: 'fa fa-magic', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    method: { name: '方法', type: 'select', options: ['Histogram', 'CLAHE', 'Gamma'], default: 'CLAHE' },
                    clip_limit: { name: 'CLIPLimit', type: 'number', min: 0, max: 100, default: 2 },
                    brightness: { name: '亮度', type: 'number', min: -100, max: 100, default: 0 },
                    contrast: { name: '对比度', type: 'number', min: -100, max: 100, default: 0 }
                }
            },
            {
                id: 'image.roi', name: 'ROI裁剪', icon: 'fa fa-crop', color: '#f38ba8',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'image', name: '图像', type: 'image' }],
                params: {
                    x: { name: 'X', type: 'number', min: 0, max: 10000, default: 0 },
                    y: { name: 'Y', type: 'number', min: 0, max: 10000, default: 0 },
                    width: { name: '宽度', type: 'number', min: 1, max: 10000, default: 100 },
                    height: { name: '高度', type: 'number', min: 1, max: 10000, default: 100 }
                }
            }
        ]
    },
    '检测分析': {
        icon: 'fa fa-search',
        color: '#a6e3a1',
        nodes: [
            {
                id: 'blob.analysis', name: 'Blob分析', icon: 'fa fa-circle', color: '#a6e3a1',
                inputs: [{ id: 'image', name: '二值图', type: 'image' }],
                outputs: [{ id: 'blobs', name: 'Blob列表', type: 'object' }, { id: 'count', name: '数量', type: 'number' }],
                params: {
                    min_area: { name: '最小面积', type: 'number', min: 0, max: 100000, default: 100 },
                    max_area: { name: '最大面积', type: 'number', min: 0, max: 1000000, default: 100000 },
                    min_circularity: { name: '最小圆度', type: 'number', min: 0, max: 1, default: 0.1, step: 0.05 }
                }
            },
            {
                id: 'pattern.match', name: '模板匹配', icon: 'fa fa-clone', color: '#a6e3a1',
                inputs: [{ id: 'image', name: '图像', type: 'image' }, { id: 'template', name: '模板', type: 'image' }],
                outputs: [{ id: 'result', name: '匹配结果', type: 'object' }],
                params: {
                    method: { name: '匹配方法', type: 'select', options: ['SQDIFF', 'CCORR', 'CCOEFF'], default: 'CCOEFF' },
                    threshold: { name: '阈值', type: 'number', min: 0, max: 1, default: 0.8, step: 0.05 }
                }
            },
            {
                id: 'barcode.read', name: '条码读取', icon: 'fa fa-barcode', color: '#f9e2af',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'text', name: '条码内容', type: 'string' }],
                params: {
                    formats: { name: '条码类型', type: 'select', multiple: true, options: ['QR', 'CODE128', 'CODE39', 'EAN13', 'EAN8'], default: ['QR', 'CODE128'] }
                }
            },
            {
                id: 'ocr.recognize', name: 'OCR识别', icon: 'fa fa-font', color: '#f9e2af',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'text', name: '文本', type: 'string' }],
                params: {
                    language: { name: '语言', type: 'select', options: ['chi_sim', 'chi_tra', 'eng', 'jpn'], default: 'chi_sim' },
                    psm: { name: '页面分割', type: 'number', min: 0, max: 13, default: 3 }
                }
            },
            {
                id: 'object.detect', name: '目标检测', icon: 'fa fa-eye', color: '#a6e3a1',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'objects', name: '对象列表', type: 'object' }],
                params: {
                    model: { name: '模型', type: 'select', options: ['YOLOv5', 'YOLOv8', 'SSD', 'FasterRCNN'], default: 'YOLOv8' },
                    confidence: { name: '置信度', type: 'number', min: 0, max: 1, default: 0.5, step: 0.05 }
                }
            }
        ]
    },
    '测量': {
        icon: 'fa fa-ruler',
        color: '#89b4fa',
        nodes: [
            {
                id: 'measure.distance', name: '距离测量', icon: 'fa fa-arrows-h', color: '#89b4fa',
                inputs: [{ id: 'point1', name: '点1', type: 'region' }, { id: 'point2', name: '点2', type: 'region' }],
                outputs: [{ id: 'distance', name: '距离', type: 'number' }],
                params: {
                    unit: { name: '单位', type: 'select', options: ['mm', 'pixel', 'cm'], default: 'mm' }
                }
            },
            {
                id: 'measure.angle', name: '角度测量', icon: 'fa fa-angle-up', color: '#89b4fa',
                inputs: [{ id: 'line1', name: '线1', type: 'region' }, { id: 'line2', name: '线2', type: 'region' }],
                outputs: [{ id: 'angle', name: '角度', type: 'number' }],
                params: {}
            },
            {
                id: 'measure.circle', name: '圆测量', icon: 'fa fa-circle-o', color: '#89b4fa',
                inputs: [{ id: 'edges', name: '边缘', type: 'region' }],
                outputs: [{ id: 'center', name: '圆心', type: 'region' }, { id: 'radius', name: '半径', type: 'number' }],
                params: {
                    method: { name: '方法', type: 'select', options: ['LeastSquare', 'Algebraic', 'Robust'], default: 'LeastSquare' }
                }
            },
            {
                id: 'measure.line', name: '直线测量', icon: 'fa fa-minus', color: '#89b4fa',
                inputs: [{ id: 'edges', name: '边缘', type: 'region' }],
                outputs: [{ id: 'line', name: '直线', type: 'region' }],
                params: {}
            }
        ]
    },
    '标定': {
        icon: 'fa fa-crosshairs',
        color: '#f9e2af',
        nodes: [
            {
                id: 'calibration.nine_point', name: '九点标定', icon: 'fa fa-th', color: '#f9e2af',
                inputs: [{ id: 'points', name: '点集', type: 'region' }],
                outputs: [{ id: 'matrix', name: '变换矩阵', type: 'object' }],
                params: {
                    scale: { name: '缩放', type: 'number', min: 0.001, max: 10, default: 1, step: 0.001 }
                }
            },
            {
                id: 'calibration.chessboard', name: '棋盘格标定', icon: 'fa fa-th-large', color: '#f9e2af',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'params', name: '相机参数', type: 'object' }],
                params: {
                    corners_x: { name: 'X角点数', type: 'number', min: 3, max: 30, default: 9 },
                    corners_y: { name: 'Y角点数', type: 'number', min: 3, max: 30, default: 6 },
                    square_size: { name: '方格尺寸', type: 'number', min: 0.1, max: 100, default: 25 }
                }
            }
        ]
    },
    '几何': {
        icon: 'fa fa-shapes',
        color: '#89b4fa',
        nodes: [
            {
                id: 'geometry.find_line', name: '找直线', icon: 'fa fa-minus', color: '#89b4fa',
                inputs: [{ id: 'points', name: '点集', type: 'region' }],
                outputs: [{ id: 'line', name: '直线', type: 'region' }],
                params: {}
            },
            {
                id: 'geometry.find_circle', name: '找圆', icon: 'fa fa-circle-o', color: '#89b4fa',
                inputs: [{ id: 'points', name: '点集', type: 'region' }],
                outputs: [{ id: 'circle', name: '圆', type: 'region' }],
                params: {}
            },
            {
                id: 'geometry.pose', name: '位姿计算', icon: 'fa fa-compass', color: '#cba6f7',
                inputs: [{ id: 'points', name: '点集', type: 'region' }],
                outputs: [{ id: 'pose', name: '位姿', type: 'object' }],
                params: {}
            }
        ]
    },
    '深度学习': {
        icon: 'fa fa-brain',
        color: '#cba6f7',
        nodes: [
            {
                id: 'dl.classify', name: '图像分类', icon: 'fa fa-image', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'label', name: '标签', type: 'string' }, { id: 'confidence', name: '置信度', type: 'number' }],
                params: {
                    model_path: { name: '模型路径', type: 'string', default: '' },
                    use_gpu: { name: '使用GPU', type: 'boolean', default: true }
                }
            },
            {
                id: 'dl.segment', name: '语义分割', icon: 'fa fa-puzzle-piece', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'mask', name: '掩膜', type: 'image' }],
                params: {
                    model_path: { name: '模型路径', type: 'string', default: '' }
                }
            },
            {
                id: 'dl.detect', name: '缺陷检测', icon: 'fa fa-exclamation-circle', color: '#cba6f7',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [{ id: 'defects', name: '缺陷列表', type: 'object' }],
                params: {
                    model_path: { name: '模型路径', type: 'string', default: '' },
                    threshold: { name: '阈值', type: 'number', min: 0, max: 1, default: 0.5, step: 0.05 }
                }
            }
        ]
    },
    '3D': {
        icon: 'fa fa-cube',
        color: '#94e2d5',
        nodes: [
            {
                id: 'three.point_cloud', name: '点云处理', icon: 'fa fa-cloud', color: '#94e2d5',
                inputs: [{ id: 'cloud', name: '点云', type: 'object' }],
                outputs: [{ id: 'cloud', name: '点云', type: 'object' }],
                params: {
                    filter: { name: '滤波', type: 'select', options: ['None', 'Statistical', 'Voxel', 'Radius'], default: 'None' }
                }
            },
            {
                id: 'three.surface', name: '三维重建', icon: 'fa fa-cubes', color: '#94e2d5',
                inputs: [{ id: 'cloud', name: '点云', type: 'object' }],
                outputs: [{ id: 'mesh', name: '网格', type: 'object' }],
                params: {
                    method: { name: '方法', type: 'select', options: ['Poisson', 'BallPivoting', 'MarchingCubes'], default: 'Poisson' }
                }
            }
        ]
    },
    '通信': {
        icon: 'fa fa-plug',
        color: '#89b4fa',
        nodes: [
            {
                id: 'comm.s7_read', name: 'S7读取', icon: 'fa fa-download', color: '#89b4fa',
                inputs: [],
                outputs: [{ id: 'value', name: '值', type: 'object' }],
                params: {
                    ip: { name: 'IP地址', type: 'string', default: '192.168.1.1' },
                    db_number: { name: 'DB号', type: 'number', min: 0, max: 65535, default: 1 },
                    offset: { name: '偏移', type: 'number', min: 0, max: 65535, default: 0 },
                    data_type: { name: '数据类型', type: 'select', options: ['BOOL', 'INT', 'REAL', 'STRING'], default: 'INT' }
                }
            },
            {
                id: 'comm.s7_write', name: 'S7写入', icon: 'fa fa-upload', color: '#89b4fa',
                inputs: [{ id: 'value', name: '值', type: 'object' }],
                outputs: [],
                params: {
                    ip: { name: 'IP地址', type: 'string', default: '192.168.1.1' },
                    db_number: { name: 'DB号', type: 'number', min: 0, max: 65535, default: 1 },
                    offset: { name: '偏移', type: 'number', min: 0, max: 65535, default: 0 }
                }
            },
            {
                id: 'comm.modbus_read', name: 'Modbus读取', icon: 'fa fa-download', color: '#89b4fa',
                inputs: [],
                outputs: [{ id: 'value', name: '值', type: 'object' }],
                params: {
                    ip: { name: 'IP地址', type: 'string', default: '127.0.0.1' },
                    port: { name: '端口', type: 'number', min: 1, max: 65535, default: 502 },
                    address: { name: '地址', type: 'number', min: 0, max: 65535, default: 0 },
                    quantity: { name: '数量', type: 'number', min: 1, max: 125, default: 1 }
                }
            },
            {
                id: 'comm.tcp_send', name: 'TCP发送', icon: 'fa fa-paper-plane', color: '#89b4fa',
                inputs: [{ id: 'data', name: '数据', type: 'string' }],
                outputs: [],
                params: {
                    ip: { name: 'IP地址', type: 'string', default: '127.0.0.1' },
                    port: { name: '端口', type: 'number', min: 1, max: 65535, default: 8080 }
                }
            }
        ]
    },
    '输出': {
        icon: 'fa fa-sign-out',
        color: '#a6e3a1',
        nodes: [
            {
                id: 'output.result', name: '结果输出', icon: 'fa fa-check-circle', color: '#a6e3a1',
                inputs: [{ id: 'data', name: '数据', type: 'object' }],
                outputs: [],
                params: {
                    format: { name: '格式', type: 'select', options: ['JSON', 'CSV', 'XML'], default: 'JSON' }
                }
            },
            {
                id: 'output.report', name: '报告生成', icon: 'fa fa-file-text-o', color: '#f9e2af',
                inputs: [{ id: 'data', name: '数据', type: 'object' }],
                outputs: [{ id: 'report', name: '报告', type: 'object' }],
                params: {
                    template: { name: '模板', type: 'select', options: ['Default', 'Inspection', 'Measurement'], default: 'Default' }
                }
            },
            {
                id: 'output.notification', name: '通知推送', icon: 'fa fa-bell-o', color: '#f38ba8',
                inputs: [{ id: 'message', name: '消息', type: 'string' }],
                outputs: [],
                params: {
                    channel: { name: '通道', type: 'select', options: ['Email', 'Webhook', 'WeChat', 'SMS'], default: 'Webhook' },
                    url: { name: 'URL', type: 'string', default: '' }
                }
            },
            {
                id: 'output.save_image', name: '图像保存', icon: 'fa fa-floppy-o', color: '#a6e3a1',
                inputs: [{ id: 'image', name: '图像', type: 'image' }],
                outputs: [],
                params: {
                    path: { name: '保存路径', type: 'string', default: './output/' },
                    format: { name: '格式', type: 'select', options: ['PNG', 'JPG', 'BMP', 'TIFF'], default: 'PNG' },
                    quality: { name: '质量', type: 'number', min: 0, max: 100, default: 95 }
                }
            }
        ]
    },
    '逻辑': {
        icon: 'fa fa-code-branch',
        color: '#f9e2af',
        nodes: [
            {
                id: 'logic.condition', name: '条件判断', icon: 'fa fa-question-circle', color: '#f9e2af',
                inputs: [{ id: 'input', name: '输入', type: 'object' }],
                outputs: [{ id: 'true', name: '真', type: 'object' }, { id: 'false', name: '假', type: 'object' }],
                params: {
                    operator: { name: '运算符', type: 'select', options: ['==', '!=', '>', '<', '>=', '<=', 'contains'], default: '==' },
                    value: { name: '比较值', type: 'string', default: '' }
                }
            },
            {
                id: 'logic.loop', name: '循环', icon: 'fa fa-repeat', color: '#f9e2af',
                inputs: [{ id: 'input', name: '输入', type: 'object' }],
                outputs: [{ id: 'item', name: '当前项', type: 'object' }],
                params: {
                    count: { name: '循环次数', type: 'number', min: 1, max: 10000, default: 10 }
                }
            },
            {
                id: 'logic.script', name: '脚本执行', icon: 'fa fa-terminal', color: '#cba6f7',
                inputs: [{ id: 'input', name: '输入', type: 'object' }],
                outputs: [{ id: 'output', name: '输出', type: 'object' }],
                params: {
                    language: { name: '语言', type: 'select', options: ['Python', 'JavaScript', 'Lua'], default: 'Python' },
                    code: { name: '代码', type: 'text', default: '# 在此输入代码\nresult = input' }
                }
            }
        ]
    }
};

// 流程模板定义
const FLOW_TEMPLATES = {
    'basic_inspection': {
        name: '基础检测流程',
        icon: 'fa fa-check-circle',
        nodes: [
            { type: 'image.load', x: 80, y: 80, name: '图像加载' },
            { type: 'image.threshold', x: 320, y: 80, name: '阈值分割' },
            { type: 'blob.analysis', x: 560, y: 80, name: 'Blob分析' },
            { type: 'output.result', x: 800, y: 80, name: '结果输出' }
        ],
        connections: [
            [0, 1, 'image', 'image'],
            [1, 2, 'image', 'image'],
            [2, 3, 'blobs', 'data']
        ]
    },
    'measurement': {
        name: '尺寸测量流程',
        icon: 'fa fa-ruler',
        nodes: [
            { type: 'camera.capture', x: 80, y: 80, name: '相机采集' },
            { type: 'image.edge_detection', x: 320, y: 80, name: '边缘检测' },
            { type: 'geometry.find_line', x: 560, y: 80, name: '找直线' },
            { type: 'measure.distance', x: 800, y: 80, name: '距离测量' },
            { type: 'output.result', x: 1040, y: 80, name: '结果输出' }
        ],
        connections: [
            [0, 1, 'image', 'image'],
            [1, 2, 'image', 'points'],
            [2, 3, 'line', 'point1'],
            [3, 4, 'distance', 'data']
        ]
    },
    'barcode': {
        name: '条码识别流程',
        icon: 'fa fa-barcode',
        nodes: [
            { type: 'image.load', x: 80, y: 80, name: '图像加载' },
            { type: 'image.roi', x: 320, y: 80, name: 'ROI裁剪' },
            { type: 'image.filter', x: 560, y: 80, name: '滤波' },
            { type: 'barcode.read', x: 800, y: 80, name: '条码读取' },
            { type: 'output.result', x: 1040, y: 80, name: '结果输出' }
        ],
        connections: [
            [0, 1, 'image', 'image'],
            [1, 2, 'image', 'image'],
            [2, 3, 'image', 'image'],
            [3, 4, 'text', 'data']
        ]
    },
    'ocr': {
        name: 'OCR识别流程',
        icon: 'fa fa-font',
        nodes: [
            { type: 'image.load', x: 80, y: 80, name: '图像加载' },
            { type: 'image.enhance', x: 320, y: 80, name: '图像增强' },
            { type: 'image.threshold', x: 560, y: 80, name: '阈值分割' },
            { type: 'ocr.recognize', x: 800, y: 80, name: 'OCR识别' },
            { type: 'output.report', x: 1040, y: 80, name: '报告生成' }
        ],
        connections: [
            [0, 1, 'image', 'image'],
            [1, 2, 'image', 'image'],
            [2, 3, 'image', 'image'],
            [3, 4, 'text', 'data']
        ]
    },
    'defect_detection': {
        name: '缺陷检测流程',
        icon: 'fa fa-exclamation-circle',
        nodes: [
            { type: 'camera.capture', x: 80, y: 80, name: '相机采集' },
            { type: 'image.color_convert', x: 320, y: 80, name: '灰度化' },
            { type: 'image.filter', x: 560, y: 80, name: '滤波' },
            { type: 'dl.detect', x: 800, y: 80, name: '缺陷检测' },
            { type: 'logic.condition', x: 1040, y: 80, name: '判断' },
            { type: 'output.notification', x: 1280, y: 40, name: '报警' },
            { type: 'output.result', x: 1280, y: 140, name: '记录' }
        ],
        connections: [
            [0, 1, 'image', 'image'],
            [1, 2, 'image', 'image'],
            [2, 3, 'image', 'image'],
            [3, 4, 'defects', 'input'],
            [4, 5, 'true', 'message'],
            [4, 6, 'false', 'data']
        ]
    },
    'alignment': {
        name: '视觉对位流程',
        icon: 'fa fa-crosshairs',
        nodes: [
            { type: 'camera.capture', x: 80, y: 80, name: '相机采集' },
            { type: 'pattern.match', x: 320, y: 80, name: '模板匹配' },
            { type: 'geometry.pose', x: 560, y: 80, name: '位姿计算' },
            { type: 'comm.s7_write', x: 800, y: 80, name: 'PLC写入' }
        ],
        connections: [
            [0, 1, 'image', 'image'],
            [1, 2, 'result', 'points'],
            [2, 3, 'pose', 'value']
        ]
    }
};

// ==================== 工具函数 ====================
function generateId(prefix = 'n') {
    return prefix + '_' + Date.now().toString(36) + '_' + Math.random().toString(36).substr(2, 9);
}

function bezierPoint(p0, p1, p2, p3, t) {
    const u = 1 - t;
    return u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
}

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function distance(p1, p2) {
    const dx = p1.x - p2.x;
    const dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function rectIntersect(r1, r2) {
    return !(r2.x > r1.x + r1.width ||
             r2.x + r2.width < r1.x ||
             r2.y > r1.y + r1.height ||
             r2.y + r2.height < r1.y);
}

function pointInRect(p, r) {
    return p.x >= r.x && p.x <= r.x + r.width &&
           p.y >= r.y && p.y <= r.y + r.height;
}

// 模糊匹配
function fuzzyMatch(text, keyword) {
    if (!keyword) return true;
    text = text.toLowerCase();
    keyword = keyword.toLowerCase();
    if (text.includes(keyword)) return true;
    // 简单的字符顺序匹配
    let ki = 0;
    for (let i = 0; i < text.length && ki < keyword.length; i++) {
        if (text[i] === keyword[ki]) ki++;
    }
    return ki === keyword.length;
}

// 获取节点定义
function getNodeType(typeId) {
    for (const cat of Object.values(NODE_CATEGORIES)) {
        const node = cat.nodes.find(n => n.id === typeId);
        if (node) return node;
    }
    // 查找自定义节点
    return state.customNodes.find(n => n.id === typeId);
}

// 获取节点显示尺寸（基于端口数）
function getNodeHeight(node) {
    const inputCount = node.inputs ? node.inputs.length : 0;
    const outputCount = node.outputs ? node.outputs.length : 0;
    const portCount = Math.max(inputCount, outputCount);
    return CONFIG.NODE_HEADER_HEIGHT + 20 + portCount * CONFIG.NODE_PORT_SPACING + CONFIG.NODE_BODY_PADDING;
}

// ==================== Canvas 渲染器 ====================
class CanvasRenderer {
    constructor() {
        this.canvas = document.getElementById('flowCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.svgOverlay = document.getElementById('overlaySvg');
        this.minimapCanvas = document.getElementById('minimapCanvas');
        this.minimapCtx = this.minimapCanvas.getContext('2d');
        this.dpr = window.devicePixelRatio || 1;
        this.resize();
        this.setupEvents();
    }

    resize() {
        const wrapper = this.canvas.parentElement;
        const w = wrapper.clientWidth;
        const h = wrapper.clientHeight;
        this.canvas.width = w * this.dpr;
        this.canvas.height = h * this.dpr;
        this.canvas.style.width = w + 'px';
        this.canvas.style.height = h + 'px';
        this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
        this.minimapCanvas.width = 180;
        this.minimapCanvas.height = 120;
    }

    setupEvents() {
        window.addEventListener('resize', () => this.resize());
    }

    clear() {
        this.ctx.save();
        this.ctx.setTransform(1, 0, 0, 1, 0, 0);
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.restore();
    }

    // 获取CSS颜色变量值
    getCssVar(name) {
        return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    }

    drawGrid() {
        if (!state.gridVisible) return;

        const gridSize = CONFIG.GRID_SIZE * state.zoom;
        const offsetX = state.panOffset.x % gridSize;
        const offsetY = state.panOffset.y % gridSize;
        const w = this.canvas.width / this.dpr;
        const h = this.canvas.height / this.dpr;

        // 小网格点
        this.ctx.fillStyle = this.getCssVar('--border-color');
        for (let x = offsetX; x < w; x += gridSize) {
            for (let y = offsetY; y < h; y += gridSize) {
                this.ctx.fillRect(x - 0.5, y - 0.5, 1, 1);
            }
        }

        // 大网格线（每5格）
        const bigGrid = gridSize * 5;
        const bigOffsetX = state.panOffset.x % bigGrid;
        const bigOffsetY = state.panOffset.y % bigGrid;
        this.ctx.strokeStyle = this.getCssVar('--border-color');
        this.ctx.lineWidth = 1;
        this.ctx.beginPath();
        for (let x = bigOffsetX; x < w; x += bigGrid) {
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, h);
        }
        for (let y = bigOffsetY; y < h; y += bigGrid) {
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(w, y);
        }
        this.ctx.stroke();
    }

    drawNode(node) {
        const x = node.x * state.zoom + state.panOffset.x;
        const y = node.y * state.zoom + state.panOffset.y;
        const width = CONFIG.NODE_WIDTH * state.zoom;
        const height = getNodeHeight(node) * state.zoom;
        const isSelected = state.selectedNodes.includes(node.id);

        // 节点背景
        this.ctx.fillStyle = this.getCssVar('--bg-elevated');
        this.roundRect(x, y, width, height, 6);
        this.ctx.fill();

        // 选中态光晕
        if (isSelected) {
            this.ctx.shadowColor = this.getCssVar('--accent-primary');
            this.ctx.shadowBlur = 15;
        }

        // 节点边框
        this.ctx.strokeStyle = isSelected ?
            this.getCssVar('--accent-primary') :
            this.getCssVar('--border-color');
        this.ctx.lineWidth = isSelected ? 2 : 1.5;
        this.roundRect(x, y, width, height, 6);
        this.ctx.stroke();
        this.ctx.shadowBlur = 0;

        // 状态边框
        if (node.state === 'running') {
            this.ctx.strokeStyle = this.getCssVar('--accent-warning');
            this.ctx.lineWidth = 3;
            this.roundRect(x - 1, y - 1, width + 2, height + 2, 7);
            this.ctx.stroke();
        } else if (node.state === 'success') {
            this.ctx.strokeStyle = this.getCssVar('--accent-success');
            this.ctx.lineWidth = 2.5;
            this.roundRect(x - 1, y - 1, width + 2, height + 2, 7);
            this.ctx.stroke();
        } else if (node.state === 'error') {
            this.ctx.strokeStyle = this.getCssVar('--accent-error');
            this.ctx.lineWidth = 2.5;
            this.roundRect(x - 1, y - 1, width + 2, height + 2, 7);
            this.ctx.stroke();
        }

        // 标题栏背景
        const typeDef = getNodeType(node.type);
        const headerColor = typeDef ? typeDef.color : this.getCssVar('--bg-hover');
        this.ctx.fillStyle = headerColor + '33'; // 透明度
        this.roundRect(x, y, width, CONFIG.NODE_HEADER_HEIGHT * state.zoom, 6, true, false);
        this.ctx.fill();

        // 标题栏左侧色条
        this.ctx.fillStyle = headerColor;
        this.roundRect(x, y, 4 * state.zoom, CONFIG.NODE_HEADER_HEIGHT * state.zoom, 2, true, false);
        this.ctx.fill();

        // 节点图标
        if (typeDef && typeDef.icon) {
            this.ctx.fillStyle = headerColor;
            this.ctx.font = `${14 * state.zoom}px 'FontAwesome'`;
            this.ctx.textAlign = 'center';
            this.ctx.textBaseline = 'middle';
            // 使用Unicode图标占位（实际渲染为方块图标）
            this.ctx.fillRect(x + 10 * state.zoom, y + 9 * state.zoom, 12 * state.zoom, 12 * state.zoom);
        }

        // 节点名称
        this.ctx.fillStyle = this.getCssVar('--text-primary');
        this.ctx.font = `600 ${12 * state.zoom}px ${getComputedStyle(document.body).fontFamily}`;
        this.ctx.textAlign = 'left';
        this.ctx.textBaseline = 'middle';
        this.ctx.fillText(node.name, x + 30 * state.zoom, y + 16 * state.zoom);

        // 启用/禁用状态
        if (node.enabled === false) {
            this.ctx.fillStyle = 'rgba(0,0,0,0.5)';
            this.roundRect(x, y, width, height, 6);
            this.ctx.fill();
        }

        // 断点标记
        if (state.breakpoints.has(node.id)) {
            this.ctx.fillStyle = this.getCssVar('--accent-error');
            this.ctx.beginPath();
            this.ctx.arc(x + width - 14 * state.zoom, y + 14 * state.zoom, 6 * state.zoom, 0, 2 * Math.PI);
            this.ctx.fill();
            this.ctx.fillStyle = this.getCssVar('--text-inverse');
            this.ctx.font = `bold ${9 * state.zoom}px sans-serif`;
            this.ctx.textAlign = 'center';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText('B', x + width - 14 * state.zoom, y + 14 * state.zoom);
        }

        // 端口
        const portStartY = y + CONFIG.NODE_HEADER_HEIGHT * state.zoom + 12 * state.zoom;
        if (node.inputs) {
            node.inputs.forEach((port, i) => {
                const portY = portStartY + i * CONFIG.NODE_PORT_SPACING * state.zoom;
                this.drawPort(x, portY, port, 'input');
                // 端口标签
                this.ctx.fillStyle = this.getCssVar('--text-secondary');
                this.ctx.font = `${11 * state.zoom}px ${getComputedStyle(document.body).fontFamily}`;
                this.ctx.textAlign = 'left';
                this.ctx.textBaseline = 'middle';
                this.ctx.fillText(port.name, x + 12 * state.zoom, portY);
            });
        }
        if (node.outputs) {
            node.outputs.forEach((port, i) => {
                const portY = portStartY + i * CONFIG.NODE_PORT_SPACING * state.zoom;
                this.drawPort(x + width, portY, port, 'output');
                // 端口标签
                this.ctx.fillStyle = this.getCssVar('--text-secondary');
                this.ctx.font = `${11 * state.zoom}px ${getComputedStyle(document.body).fontFamily}`;
                this.ctx.textAlign = 'right';
                this.ctx.textBaseline = 'middle';
                this.ctx.fillText(port.name, x + width - 12 * state.zoom, portY);
            });
        }

        // 执行时间
        if (node.executeTime) {
            this.ctx.fillStyle = this.getCssVar('--text-muted');
            this.ctx.font = `${10 * state.zoom}px ${getComputedStyle(document.body).fontFamily}`;
            this.ctx.textAlign = 'right';
            this.ctx.textBaseline = 'bottom';
            this.ctx.fillText(`${node.executeTime.toFixed(1)}ms`, x + width - 6 * state.zoom, y + height - 4 * state.zoom);
        }

        // 折叠态处理
        if (node.collapsed) {
            this.ctx.fillStyle = this.getCssVar('--bg-elevated');
            this.ctx.fillRect(x, y + CONFIG.NODE_HEADER_HEIGHT * state.zoom, width, height - CONFIG.NODE_HEADER_HEIGHT * state.zoom);
        }
    }

    drawPort(x, y, port, direction) {
        const size = CONFIG.PORT_SIZE * state.zoom;
        const colorMap = {
            'image': this.getCssVar('--type-image'),
            'number': this.getCssVar('--type-number'),
            'string': this.getCssVar('--type-string'),
            'boolean': this.getCssVar('--type-boolean'),
            'region': this.getCssVar('--type-region'),
            'object': this.getCssVar('--type-object')
        };
        const color = colorMap[port.type] || this.getCssVar('--text-primary');

        // 端口外圈
        this.ctx.beginPath();
        this.ctx.arc(x, y, size / 2 + 2, 0, 2 * Math.PI);
        this.ctx.fillStyle = this.getCssVar('--bg-secondary');
        this.ctx.fill();

        // 端口主体
        this.ctx.beginPath();
        this.ctx.arc(x, y, size / 2, 0, 2 * Math.PI);
        this.ctx.fillStyle = color;
        this.ctx.fill();

        // 端口边框
        this.ctx.strokeStyle = this.getCssVar('--bg-tertiary');
        this.ctx.lineWidth = 1.5;
        this.ctx.stroke();
    }

    // 智能路由：计算连接路径
    getConnectionPath(source, target) {
        const sourceX = source.x + CONFIG.NODE_WIDTH * state.zoom;
        const sourceY = source.y + (CONFIG.NODE_HEADER_HEIGHT + 12) * state.zoom;
        const targetX = target.x;
        const targetY = target.y + (CONFIG.NODE_HEADER_HEIGHT + 12) * state.zoom;

        // 智能路径计算（避免穿过节点）
        const dx = Math.abs(targetX - sourceX);
        const offset = Math.max(40, dx * 0.5);

        return {
            startX: sourceX,
            startY: sourceY,
            cp1x: sourceX + offset,
            cp1y: sourceY,
            cp2x: targetX - offset,
            cp2y: targetY,
            endX: targetX,
            endY: targetY
        };
    }

    drawConnection(conn) {
        const sourceNode = state.nodes.get(conn.sourceId);
        const targetNode = state.nodes.get(conn.targetId);
        if (!sourceNode || !targetNode) return;

        // 计算端口位置
        const sourcePortIdx = sourceNode.outputs.findIndex(p => p.id === conn.sourcePort);
        const targetPortIdx = targetNode.inputs.findIndex(p => p.id === conn.targetPort);
        if (sourcePortIdx < 0 || targetPortIdx < 0) return;

        const sx = sourceNode.x * state.zoom + state.panOffset.x + CONFIG.NODE_WIDTH * state.zoom;
        const sy = (sourceNode.y + CONFIG.NODE_HEADER_HEIGHT + 12 + sourcePortIdx * CONFIG.NODE_PORT_SPACING) * state.zoom + state.panOffset.y;
        const tx = targetNode.x * state.zoom + state.panOffset.x;
        const ty = (targetNode.y + CONFIG.NODE_HEADER_HEIGHT + 12 + targetPortIdx * CONFIG.NODE_PORT_SPACING) * state.zoom + state.panOffset.y;

        // 贝塞尔曲线控制点
        const dx = Math.abs(tx - sx);
        const offset = Math.max(40, dx * 0.5);
        const cp1x = sx + offset;
        const cp1y = sy;
        const cp2x = tx - offset;
        const cp2y = ty;

        // 颜色
        const colorMap = {
            'image': this.getCssVar('--type-image'),
            'number': this.getCssVar('--type-number'),
            'string': this.getCssVar('--type-string'),
            'boolean': this.getCssVar('--type-boolean'),
            'region': this.getCssVar('--type-region'),
            'object': this.getCssVar('--type-object')
        };
        const color = colorMap[conn.dataType] || this.getCssVar('--text-secondary');
        const isSelected = state.selectedConnections.includes(conn.id);

        // 阴影
        this.ctx.beginPath();
        this.ctx.moveTo(sx, sy);
        this.ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, tx, ty);
        this.ctx.strokeStyle = isSelected ? this.getCssVar('--accent-warning') : color;
        this.ctx.lineWidth = (isSelected ? 4 : CONFIG.CONNECTION_WIDTH) * state.zoom;
        if (conn.active) {
            this.ctx.setLineDash([12, 6]);
            this.ctx.lineDashOffset = -Date.now() / 50;
        } else {
            this.ctx.setLineDash([]);
        }
        this.ctx.stroke();
        this.ctx.setLineDash([]);

        // 流动粒子
        if (conn.active) {
            const t = (Date.now() / 1500) % 1;
            const px = bezierPoint(sx, cp1x, cp2x, tx, t);
            const py = bezierPoint(sy, cp1y, cp2y, ty, t);
            this.ctx.beginPath();
            this.ctx.arc(px, py, 5 * state.zoom, 0, 2 * Math.PI);
            this.ctx.fillStyle = this.getCssVar('--accent-warning');
            this.ctx.fill();
            // 光晕
            this.ctx.beginPath();
            this.ctx.arc(px, py, 8 * state.zoom, 0, 2 * Math.PI);
            this.ctx.fillStyle = this.getCssVar('--accent-warning') + '44';
            this.ctx.fill();
        }
    }

    drawTempConnection(start, end) {
        const sx = start.x;
        const sy = start.y;
        const tx = end.x;
        const ty = end.y;
        const dx = Math.abs(tx - sx);
        const offset = Math.max(40, dx * 0.5);

        this.ctx.beginPath();
        this.ctx.moveTo(sx, sy);
        this.ctx.bezierCurveTo(sx + offset, sy, tx - offset, ty, tx, ty);
        this.ctx.strokeStyle = this.getCssVar('--accent-primary');
        this.ctx.lineWidth = 2.5;
        this.ctx.setLineDash([8, 4]);
        this.ctx.stroke();
        this.ctx.setLineDash([]);

        // 终点指示器
        this.ctx.beginPath();
        this.ctx.arc(tx, ty, 6, 0, 2 * Math.PI);
        this.ctx.fillStyle = this.getCssVar('--accent-primary');
        this.ctx.fill();
    }

    drawSelectionRect(rect) {
        const x = Math.min(rect.x, rect.x + rect.width);
        const y = Math.min(rect.y, rect.y + rect.height);
        const w = Math.abs(rect.width);
        const h = Math.abs(rect.height);

        this.ctx.fillStyle = this.getCssVar('--accent-primary') + '20';
        this.ctx.fillRect(x, y, w, h);
        this.ctx.strokeStyle = this.getCssVar('--accent-primary');
        this.ctx.lineWidth = 1.5;
        this.ctx.setLineDash([4, 4]);
        this.ctx.strokeRect(x, y, w, h);
        this.ctx.setLineDash([]);
    }

    drawGroup(group) {
        const bounds = this.getGroupBounds(group);
        const x = bounds.x * state.zoom + state.panOffset.x;
        const y = bounds.y * state.zoom + state.panOffset.y;
        const w = bounds.width * state.zoom;
        const h = bounds.height * state.zoom;

        this.ctx.strokeStyle = this.getCssVar('--accent-purple');
        this.ctx.lineWidth = 1.5;
        this.ctx.setLineDash([8, 4]);
        this.roundRect(x - 10, y - 25, w + 20, h + 35, 8);
        this.ctx.stroke();
        this.ctx.setLineDash([]);

        // 分组标题
        this.ctx.fillStyle = this.getCssVar('--accent-purple');
        this.ctx.font = `600 ${11 * state.zoom}px ${getComputedStyle(document.body).fontFamily}`;
        this.ctx.textAlign = 'left';
        this.ctx.textBaseline = 'middle';
        this.ctx.fillText(group.name, x - 5, y - 15);
    }

    getGroupBounds(group) {
        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        group.nodeIds.forEach(id => {
            const node = state.nodes.get(id);
            if (node) {
                minX = Math.min(minX, node.x);
                minY = Math.min(minY, node.y);
                maxX = Math.max(maxX, node.x + CONFIG.NODE_WIDTH);
                maxY = Math.max(maxY, node.y + getNodeHeight(node));
            }
        });
        if (minX === Infinity) return { x: 0, y: 0, width: 0, height: 0 };
        return { x: minX, y: minY, width: maxX - minX, height: maxY - minY };
    }

    roundRect(x, y, w, h, r, fillTop = true, fillBottom = true) {
        this.ctx.beginPath();
        if (fillTop) {
            this.ctx.moveTo(x + r, y);
            this.ctx.lineTo(x + w - r, y);
            this.ctx.quadraticCurveTo(x + w, y, x + w, y + r);
        } else {
            this.ctx.moveTo(x, y);
            this.ctx.lineTo(x + w, y);
        }
        if (fillBottom) {
            this.ctx.lineTo(x + w, y + h - r);
            this.ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
            this.ctx.lineTo(x + r, y + h);
            this.ctx.quadraticCurveTo(x, y + h, x, y + h - r);
        } else {
            this.ctx.lineTo(x + w, y + h);
            this.ctx.lineTo(x, y + h);
        }
        if (fillTop) {
            this.ctx.lineTo(x, y + r);
            this.ctx.quadraticCurveTo(x, y, x + r, y);
        } else {
            this.ctx.lineTo(x, y);
        }
        this.ctx.closePath();
    }

    drawMinimap() {
        if (!state.minimapVisible) return;
        const ctx = this.minimapCtx;
        const w = 180, h = 120;
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = this.getCssVar('--bg-primary');
        ctx.fillRect(0, 0, w, h);

        const bounds = this.getNodesBounds();
        if (bounds.width === 0) return;
        const padding = 60;
        const scaleX = w / (bounds.width + padding * 2);
        const scaleY = h / (bounds.height + padding * 2);
        const scale = Math.min(scaleX, scaleY, 0.15);

        // 绘制节点
        state.nodes.forEach(node => {
            const x = (node.x - bounds.minX + padding) * scale;
            const y = (node.y - bounds.minY + padding) * scale;
            const nw = CONFIG.NODE_WIDTH * scale;
            const nh = getNodeHeight(node) * scale;
            const typeDef = getNodeType(node.type);
            ctx.fillStyle = typeDef ? typeDef.color : this.getCssVar('--bg-hover');
            ctx.fillRect(x, y, Math.max(2, nw), Math.max(2, nh));
        });

        // 绘制连接
        ctx.strokeStyle = this.getCssVar('--text-muted');
        ctx.lineWidth = 0.5;
        state.connections.forEach(conn => {
            const s = state.nodes.get(conn.sourceId);
            const t = state.nodes.get(conn.targetId);
            if (!s || !t) return;
            ctx.beginPath();
            ctx.moveTo((s.x - bounds.minX + padding + CONFIG.NODE_WIDTH) * scale,
                       (s.y - bounds.minY + padding + CONFIG.NODE_HEADER_HEIGHT) * scale);
            ctx.lineTo((t.x - bounds.minX + padding) * scale,
                       (t.y - bounds.minY + padding + CONFIG.NODE_HEADER_HEIGHT) * scale);
            ctx.stroke();
        });

        // 视口框
        const viewX = (-state.panOffset.x / state.zoom - bounds.minX + padding) * scale;
        const viewY = (-state.panOffset.y / state.zoom - bounds.minY + padding) * scale;
        const viewW = (this.canvas.width / this.dpr / state.zoom) * scale;
        const viewH = (this.canvas.height / this.dpr / state.zoom) * scale;
        ctx.strokeStyle = this.getCssVar('--accent-primary');
        ctx.lineWidth = 1.5;
        ctx.strokeRect(viewX, viewY, viewW, viewH);
        ctx.fillStyle = this.getCssVar('--accent-primary') + '20';
        ctx.fillRect(viewX, viewY, viewW, viewH);
    }

    getNodesBounds() {
        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        state.nodes.forEach(node => {
            minX = Math.min(minX, node.x);
            minY = Math.min(minY, node.y);
            maxX = Math.max(maxX, node.x + CONFIG.NODE_WIDTH);
            maxY = Math.max(maxY, node.y + getNodeHeight(node));
        });
        if (minX === Infinity) return { minX: 0, minY: 0, width: 1000, height: 800 };
        return { minX, minY, width: maxX - minX, height: maxY - minY };
    }

    render() {
        this.clear();
        this.drawGrid();
        // 绘制分组（在节点下方）
        state.groups.forEach(g => this.drawGroup(g));
        // 绘制连接
        state.connections.forEach(c => this.drawConnection(c));
        // 绘制临时连接
        if (interaction && interaction.tempConnection) {
            this.drawTempConnection(interaction.tempConnection.start, interaction.tempConnection.end);
        }
        // 绘制选择框
        if (interaction && interaction.selecting && interaction.selectionRect) {
            this.drawSelectionRect(interaction.selectionRect);
        }
        // 绘制节点
        state.nodes.forEach(node => this.drawNode(node));
        // 小地图
        this.drawMinimap();
    }
}

let renderer;

// ==================== 交互控制器 ====================
class InteractionController {
    constructor() {
        this.dragging = false;
        this.dragNodes = [];
        this.dragOffsets = [];
        this.connecting = false;
        this.connectStart = null;
        this.tempConnection = null;
        this.panning = false;
        this.panStart = { x: 0, y: 0 };
        this.selecting = false;
        this.selectionRect = null;
        this.hoverPort = null;
        this.lastMousePos = { x: 0, y: 0 };
        this.setupEvents();
    }

    setupEvents() {
        const canvas = renderer.canvas;
        canvas.addEventListener('mousedown', e => this.onMouseDown(e));
        canvas.addEventListener('mousemove', e => this.onMouseMove(e));
        canvas.addEventListener('mouseup', e => this.onMouseUp(e));
        canvas.addEventListener('wheel', e => this.onWheel(e), { passive: false });
        canvas.addEventListener('dblclick', e => this.onDoubleClick(e));
        canvas.addEventListener('contextmenu', e => this.onContextMenu(e));
        canvas.addEventListener('dragover', e => { e.preventDefault(); });
        canvas.addEventListener('drop', e => this.onDrop(e));
        document.addEventListener('keydown', e => this.onKeyDown(e));
        // 触摸支持
        canvas.addEventListener('touchstart', e => this.onTouchStart(e), { passive: false });
        canvas.addEventListener('touchmove', e => this.onTouchMove(e), { passive: false });
        canvas.addEventListener('touchend', e => this.onTouchEnd(e));
    }

    getCanvasPos(e) {
        const rect = renderer.canvas.getBoundingClientRect();
        return { x: e.clientX - rect.left, y: e.clientY - rect.top };
    }

    screenToCanvas(pos) {
        return {
            x: (pos.x - state.panOffset.x) / state.zoom,
            y: (pos.y - state.panOffset.y) / state.zoom
        };
    }

    hitTestNode(pos) {
        for (const node of state.nodes.values()) {
            const x = node.x * state.zoom + state.panOffset.x;
            const y = node.y * state.zoom + state.panOffset.y;
            const w = CONFIG.NODE_WIDTH * state.zoom;
            const h = getNodeHeight(node) * state.zoom;
            if (pos.x >= x && pos.x <= x + w && pos.y >= y && pos.y <= y + h) {
                return node;
            }
        }
        return null;
    }

    hitTestPort(pos) {
        for (const node of state.nodes.values()) {
            const nodeX = node.x * state.zoom + state.panOffset.x;
            const nodeY = node.y * state.zoom + state.panOffset.y;
            const portStartY = nodeY + (CONFIG.NODE_HEADER_HEIGHT + 12) * state.zoom;
            const hitR = CONFIG.PORT_HIT_RADIUS * state.zoom;

            if (node.inputs) {
                for (let i = 0; i < node.inputs.length; i++) {
                    const portY = portStartY + i * CONFIG.NODE_PORT_SPACING * state.zoom;
                    if (Math.abs(pos.x - nodeX) < hitR && Math.abs(pos.y - portY) < hitR) {
                        return { node, port: node.inputs[i], direction: 'input', index: i };
                    }
                }
            }
            if (node.outputs) {
                for (let i = 0; i < node.outputs.length; i++) {
                    const portY = portStartY + i * CONFIG.NODE_PORT_SPACING * state.zoom;
                    const portX = nodeX + CONFIG.NODE_WIDTH * state.zoom;
                    if (Math.abs(pos.x - portX) < hitR && Math.abs(pos.y - portY) < hitR) {
                        return { node, port: node.outputs[i], direction: 'output', index: i };
                    }
                }
            }
        }
        return null;
    }

    hitTestConnection(pos) {
        for (const conn of state.connections) {
            const s = state.nodes.get(conn.sourceId);
            const t = state.nodes.get(conn.targetId);
            if (!s || !t) continue;
            const sIdx = s.outputs.findIndex(p => p.id === conn.sourcePort);
            const tIdx = t.inputs.findIndex(p => p.id === conn.targetPort);
            if (sIdx < 0 || tIdx < 0) continue;

            const sx = s.x * state.zoom + state.panOffset.x + CONFIG.NODE_WIDTH * state.zoom;
            const sy = (s.y + CONFIG.NODE_HEADER_HEIGHT + 12 + sIdx * CONFIG.NODE_PORT_SPACING) * state.zoom + state.panOffset.y;
            const tx = t.x * state.zoom + state.panOffset.x;
            const ty = (t.y + CONFIG.NODE_HEADER_HEIGHT + 12 + tIdx * CONFIG.NODE_PORT_SPACING) * state.zoom + state.panOffset.y;

            // 在曲线上采样检测
            const dx = Math.abs(tx - sx);
            const offset = Math.max(40, dx * 0.5);
            for (let i = 0; i <= 20; i++) {
                const t = i / 20;
                const px = bezierPoint(sx, sx + offset, tx - offset, tx, t);
                const py = bezierPoint(sy, sy, ty, ty, t);
                if (distance(pos, { x: px, y: py }) < 8 * state.zoom) {
                    return conn;
                }
            }
        }
        return null;
    }

    onMouseDown(e) {
        const pos = this.getCanvasPos(e);
        this.lastMousePos = pos;
        hideContextMenu();

        // 端口检测
        const portHit = this.hitTestPort(pos);
        if (portHit) {
            this.startConnection(portHit, pos);
            return;
        }

        // 节点检测
        const nodeHit = this.hitTestNode(pos);
        if (nodeHit) {
            if (e.ctrlKey || e.shiftKey) {
                state.selectNode(nodeHit.id, true);
            } else if (!state.selectedNodes.includes(nodeHit.id)) {
                state.selectNode(nodeHit.id);
            }
            updateParamsPanel(nodeHit);
            this.startDrag(state.selectedNodes, pos);
            renderCanvas();
            return;
        }

        // 连接检测
        const connHit = this.hitTestConnection(pos);
        if (connHit) {
            if (e.ctrlKey) {
                if (!state.selectedConnections.includes(connHit.id)) {
                    state.selectedConnections.push(connHit.id);
                }
            } else {
                state.selectedConnections = [connHit.id];
                state.selectedNodes = [];
            }
            renderCanvas();
            return;
        }

        // 空白区域
        if (e.button === 0) {
            state.deselectAll();
            updateParamsPanel(null);
            if (e.altKey || e.button === 1) {
                this.startPan(pos);
            } else {
                this.startSelection(pos);
            }
            renderCanvas();
        }
    }

    onMouseMove(e) {
        const pos = this.getCanvasPos(e);

        if (this.dragging) {
            const dx = (pos.x - this.lastMousePos.x) / state.zoom;
            const dy = (pos.y - this.lastMousePos.y) / state.zoom;
            this.dragNodes.forEach(node => {
                node.x += dx;
                node.y += dy;
                if (CONFIG.SNAP_TO_GRID && state.snapEnabled) {
                    node.x = Math.round(node.x / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
                    node.y = Math.round(node.y / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
                }
            });
            renderCanvas();
        } else if (this.connecting) {
            this.tempConnection = { start: this.connectStart.pos, end: pos };
            // 端口吸附
            const portHit = this.hitTestPort(pos);
            if (portHit && portHit.direction !== this.connectStart.direction) {
                this.hoverPort = portHit;
                // 吸附到端口位置
                const nodeX = portHit.node.x * state.zoom + state.panOffset.x;
                const portStartY = (portHit.node.y + CONFIG.NODE_HEADER_HEIGHT + 12) * state.zoom + state.panOffset.y;
                const portY = portStartY + portHit.index * CONFIG.NODE_PORT_SPACING * state.zoom;
                const snapX = portHit.direction === 'input' ? nodeX : nodeX + CONFIG.NODE_WIDTH * state.zoom;
                this.tempConnection.end = { x: snapX, y: portY };
            } else {
                this.hoverPort = null;
            }
            renderCanvas();
        } else if (this.panning) {
            state.panOffset.x += pos.x - this.panStart.x;
            state.panOffset.y += pos.y - this.panStart.y;
            this.panStart = { x: pos.x, y: pos.y };
            renderCanvas();
        } else if (this.selecting) {
            this.selectionRect.width = pos.x - this.selectionRect.x;
            this.selectionRect.height = pos.y - this.selectionRect.y;
            renderCanvas();
        }

        this.lastMousePos = pos;
    }

    onMouseUp(e) {
        const pos = this.getCanvasPos(e);
        if (this.dragging) {
            this.endDrag();
            state.markModified();
        } else if (this.connecting) {
            this.endConnection(pos);
        } else if (this.panning) {
            this.endPan();
        } else if (this.selecting) {
            this.endSelection();
        }
        // 显示/隐藏对齐工具
        toggleAlignTools();
    }

    onWheel(e) {
        e.preventDefault();
        const pos = this.getCanvasPos(e);
        const delta = -e.deltaY * CONFIG.ZOOM_WHEEL_FACTOR;
        const newZoom = clamp(state.zoom * (1 + delta), CONFIG.ZOOM_MIN, CONFIG.ZOOM_MAX);
        const ratio = newZoom / state.zoom;
        state.panOffset.x = pos.x - (pos.x - state.panOffset.x) * ratio;
        state.panOffset.y = pos.y - (pos.y - state.panOffset.y) * ratio;
        state.zoom = newZoom;
        updateZoomLevel();
        renderCanvas();
    }

    onDoubleClick(e) {
        const pos = this.getCanvasPos(e);
        const nodeHit = this.hitTestNode(pos);
        if (nodeHit) {
            nodeHit.collapsed = !nodeHit.collapsed;
            renderCanvas();
        }
    }

    onContextMenu(e) {
        e.preventDefault();
        const pos = this.getCanvasPos(e);
        const nodeHit = this.hitTestNode(pos);
        const connHit = !nodeHit ? this.hitTestConnection(pos) : null;
        showContextMenu(e.clientX, e.clientY, { node: nodeHit, connection: connHit, pos });
    }

    onKeyDown(e) {
        const target = e.target;
        if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT') {
            // 仅在输入框内保留特殊快捷键
            if (e.key === 'Escape') target.blur();
            return;
        }

        const ctrl = e.ctrlKey || e.metaKey;
        switch (e.key) {
            case 'Delete':
            case 'Backspace':
                cmd('deleteSelected');
                e.preventDefault();
                break;
            case 'c': case 'C':
                if (ctrl) { cmd('copyNodes'); e.preventDefault(); }
                break;
            case 'v': case 'V':
                if (ctrl) { cmd('pasteNodes'); e.preventDefault(); }
                break;
            case 'x': case 'X':
                if (ctrl) { cmd('cutNodes'); e.preventDefault(); }
                break;
            case 'z': case 'Z':
                if (ctrl) {
                    if (e.shiftKey) cmd('redo');
                    else cmd('undo');
                    e.preventDefault();
                }
                break;
            case 'y': case 'Y':
                if (ctrl) { cmd('redo'); e.preventDefault(); }
                break;
            case 'a': case 'A':
                if (ctrl) { cmd('selectAll'); e.preventDefault(); }
                break;
            case 's': case 'S':
                if (ctrl) { cmd('saveFlow'); e.preventDefault(); }
                break;
            case 'n': case 'N':
                if (ctrl) { cmd('newFlow'); e.preventDefault(); }
                break;
            case 'o': case 'O':
                if (ctrl) { cmd('openFlow'); e.preventDefault(); }
                break;
            case 'g': case 'G':
                if (ctrl) { cmd('groupSelected'); e.preventDefault(); }
                break;
            case 'd': case 'D':
                if (ctrl) { cmd('duplicateSelected'); e.preventDefault(); }
                break;
            case 'f': case 'F':
                if (ctrl) { cmd('fitToScreen'); e.preventDefault(); }
                break;
            case '+': case '=':
                cmd('zoomIn');
                break;
            case '-': case '_':
                cmd('zoomOut');
                break;
            case '0':
                if (ctrl) { cmd('resetView'); e.preventDefault(); }
                break;
            case 'F5':
                e.preventDefault();
                if (e.shiftKey) cmd('stopFlow');
                else cmd('runFlow');
                break;
            case 'F10':
                e.preventDefault();
                cmd('stepFlow');
                break;
            case 'F11':
                e.preventDefault();
                cmd('stepOver');
                break;
            case '?':
                cmd('showShortcuts');
                break;
            case 'Escape':
                cmd('deselectAll');
                cmd('hideShortcuts');
                hideContextMenu();
                break;
            case 'ArrowLeft':
                if (state.selectedNodes.length > 0) {
                    state.selectedNodes.forEach(id => {
                        const n = state.nodes.get(id);
                        if (n) n.x -= CONFIG.GRID_SIZE;
                    });
                    state.markModified();
                    renderCanvas();
                }
                break;
            case 'ArrowRight':
                if (state.selectedNodes.length > 0) {
                    state.selectedNodes.forEach(id => {
                        const n = state.nodes.get(id);
                        if (n) n.x += CONFIG.GRID_SIZE;
                    });
                    state.markModified();
                    renderCanvas();
                }
                break;
            case 'ArrowUp':
                if (state.selectedNodes.length > 0) {
                    state.selectedNodes.forEach(id => {
                        const n = state.nodes.get(id);
                        if (n) n.y -= CONFIG.GRID_SIZE;
                    });
                    state.markModified();
                    renderCanvas();
                }
                break;
            case 'ArrowDown':
                if (state.selectedNodes.length > 0) {
                    state.selectedNodes.forEach(id => {
                        const n = state.nodes.get(id);
                        if (n) n.y += CONFIG.GRID_SIZE;
                    });
                    state.markModified();
                    renderCanvas();
                }
                break;
        }
    }

    onDrop(e) {
        e.preventDefault();
        const pos = this.getCanvasPos(e);
        const canvasPos = this.screenToCanvas(pos);
        const nodeType = e.dataTransfer.getData('nodeType');
        if (nodeType) {
            createNodeFromType(nodeType, canvasPos.x, canvasPos.y);
        }
    }

    onTouchStart(e) {
        e.preventDefault();
        if (e.touches.length === 1) {
            const t = e.touches[0];
            this.onMouseDown({ clientX: t.clientX, clientY: t.clientY, button: 0, altKey: false, ctrlKey: false, shiftKey: false, preventDefault: () => {} });
        } else if (e.touches.length === 2) {
            this.touchStartDistance = distance(
                { x: e.touches[0].clientX, y: e.touches[0].clientY },
                { x: e.touches[1].clientX, y: e.touches[1].clientY }
            );
        }
    }

    onTouchMove(e) {
        e.preventDefault();
        if (e.touches.length === 1) {
            const t = e.touches[0];
            this.onMouseMove({ clientX: t.clientX, clientY: t.clientY });
        } else if (e.touches.length === 2) {
            const d = distance(
                { x: e.touches[0].clientX, y: e.touches[0].clientY },
                { x: e.touches[1].clientX, y: e.touches[1].clientY }
            );
            const ratio = d / this.touchStartDistance;
            state.zoom = clamp(state.zoom * ratio, CONFIG.ZOOM_MIN, CONFIG.ZOOM_MAX);
            updateZoomLevel();
            renderCanvas();
            this.touchStartDistance = d;
        }
    }

    onTouchEnd(e) {
        this.onMouseUp({});
    }

    startDrag(nodeIds, pos) {
        state.pushUndo();
        this.dragging = true;
        this.dragNodes = nodeIds.map(id => state.nodes.get(id)).filter(n => n);
    }

    endDrag() {
        this.dragging = false;
        this.dragNodes = [];
    }

    startConnection(portHit, pos) {
        this.connecting = true;
        const nodeX = portHit.node.x * state.zoom + state.panOffset.x;
        const portStartY = (portHit.node.y + CONFIG.NODE_HEADER_HEIGHT + 12) * state.zoom + state.panOffset.y;
        const portY = portStartY + portHit.index * CONFIG.NODE_PORT_SPACING * state.zoom;
        const startX = portHit.direction === 'input' ? nodeX : nodeX + CONFIG.NODE_WIDTH * state.zoom;
        this.connectStart = {
            node: portHit.node,
            port: portHit.port,
            direction: portHit.direction,
            pos: { x: startX, y: portY }
        };
        this.tempConnection = { start: this.connectStart.pos, end: pos };
    }

    endConnection(pos) {
        this.connecting = false;
        const portHit = this.hoverPort || this.hitTestPort(pos);
        if (portHit && portHit.direction !== this.connectStart.direction) {
            const source = this.connectStart.direction === 'output' ? this.connectStart : portHit;
            const target = this.connectStart.direction === 'input' ? this.connectStart : portHit;

            // 类型检查
            if (source.port.type !== target.port.type && source.port.type !== 'object' && target.port.type !== 'object') {
                showToast(`类型不匹配: ${source.port.type} → ${target.port.type}`, 'error');
            } else {
                state.pushUndo();
                const added = state.addConnection({
                    id: generateId('c'),
                    sourceId: source.node.id,
                    sourcePort: source.port.id,
                    targetId: target.node.id,
                    targetPort: target.port.id,
                    dataType: source.port.type
                });
                if (added) {
                    showToast('连接已创建', 'success');
                }
                updateStatusBar();
            }
        }
        this.connectStart = null;
        this.tempConnection = null;
        this.hoverPort = null;
        renderCanvas();
    }

    startPan(pos) {
        this.panning = true;
        this.panStart = { x: pos.x, y: pos.y };
    }

    endPan() {
        this.panning = false;
    }

    startSelection(pos) {
        this.selecting = true;
        this.selectionRect = { x: pos.x, y: pos.y, width: 0, height: 0 };
    }

    endSelection() {
        this.selecting = false;
        if (!this.selectionRect) return;
        const rect = {
            x: Math.min(this.selectionRect.x, this.selectionRect.x + this.selectionRect.width),
            y: Math.min(this.selectionRect.y, this.selectionRect.y + this.selectionRect.height),
            width: Math.abs(this.selectionRect.width),
            height: Math.abs(this.selectionRect.height)
        };
        if (rect.width < 5 && rect.height < 5) {
            this.selectionRect = null;
            return;
        }
        state.nodes.forEach(node => {
            const nx = node.x * state.zoom + state.panOffset.x;
            const ny = node.y * state.zoom + state.panOffset.y;
            const nw = CONFIG.NODE_WIDTH * state.zoom;
            const nh = getNodeHeight(node) * state.zoom;
            if (rectIntersect(rect, { x: nx, y: ny, width: nw, height: nh })) {
                state.selectNode(node.id, true);
            }
        });
        this.selectionRect = null;
        renderCanvas();
    }
}

let interaction;

// ==================== API 客户端 ====================
class ApiClient {
    constructor() {
        this.baseUrl = CONFIG.API_BASE;
        this.ws = null;
        this.wsConnected = false;
        this.eventHandlers = new Map();
        this.reconnectTimer = null;
    }

    async request(method, path, data) {
        try {
            const opts = { method, headers: {} };
            if (data) {
                opts.headers['Content-Type'] = 'application/json';
                opts.body = JSON.stringify(data);
            }
            const resp = await fetch(this.baseUrl + path, opts);
            return await resp.json();
        } catch (error) {
            logMessage(`API请求失败: ${error.message}`, 'error');
            return { success: false, error: error.message };
        }
    }

    get(path) { return this.request('GET', path); }
    post(path, data) { return this.request('POST', path, data); }
    put(path, data) { return this.request('PUT', path, data); }
    delete(path) { return this.request('DELETE', path); }

    connectWebSocket() {
        if (this.ws) return;
        try {
            this.ws = new WebSocket(CONFIG.WS_URL);
            this.ws.onopen = () => {
                this.wsConnected = true;
                logMessage('WebSocket已连接', 'info');
            };
            this.ws.onmessage = (e) => {
                try {
                    const data = JSON.parse(e.data);
                    this.handleWsMessage(data);
                } catch (err) {}
            };
            this.ws.onerror = () => logMessage('WebSocket错误', 'error');
            this.ws.onclose = () => {
                this.wsConnected = false;
                logMessage('WebSocket已断开，5秒后重连', 'warn');
                this.ws = null;
                this.reconnectTimer = setTimeout(() => this.connectWebSocket(), 5000);
            };
        } catch (e) {
            logMessage('WebSocket连接失败: ' + e.message, 'error');
        }
    }

    handleWsMessage(data) {
        switch (data.type) {
            case 'execution_progress': updateExecutionProgress(data); break;
            case 'node_state': updateNodeState(data.node_id, data.state); break;
            case 'execution_result': handleExecutionResult(data); break;
            case 'debug_info': handleDebugInfo(data); break;
            case 'log': logMessage(data.message, data.level); break;
            case 'error': showError(data.message); break;
        }
        if (this.eventHandlers.has(data.type)) {
            this.eventHandlers.get(data.type)(data);
        }
    }

    on(event, handler) { this.eventHandlers.set(event, handler); }

    // API方法
    getNodes() { return this.get('/nodes'); }
    getNodeInfo(typeId) { return this.get(`/node/${typeId}`); }
    createFlow(flowDef) { return this.post('/flow', flowDef); }
    updateFlow(flowId, flowDef) { return this.put(`/flow/${flowId}`, flowDef); }
    deleteFlow(flowId) { return this.delete(`/flow/${flowId}`); }
    executeFlow(flowId) { return this.post(`/flow/${flowId}/execute`); }
    stopExecution() { return this.post('/flow/stop'); }
    stepFlow() { return this.post('/flow/step'); }
    getFlowResult(flowId) { return this.get(`/flow/${flowId}/result`); }
    uploadImage(imageData) { return this.post('/image', imageData); }
    getImage(imageId) { return this.get(`/image/${imageId}`); }
    previewNode(nodeId) { return this.get(`/node/${nodeId}/preview`); }
}

let apiClient;

// ==================== 节点分类树渲染 ====================
function renderNodeTree(filter = '') {
    const container = document.getElementById('nodeTree');
    container.innerHTML = '';

    let totalVisible = 0;
    for (const [categoryName, categoryDef] of Object.entries(NODE_CATEGORIES)) {
        const filteredNodes = categoryDef.nodes.filter(n =>
            fuzzyMatch(n.name, filter) || fuzzyMatch(n.id, filter)
        );

        // 即使没有匹配，分类本身也显示（但折叠）
        const categoryDiv = document.createElement('div');
        categoryDiv.className = 'tree-category' + (filter && filteredNodes.length === 0 ? ' collapsed' : '');

        const headerDiv = document.createElement('div');
        headerDiv.className = 'tree-category-header';
        headerDiv.innerHTML = `
            <i class="fa fa-caret-down caret"></i>
            <i class="fa ${categoryDef.icon} category-icon" style="color: ${categoryDef.color}"></i>
            <span>${categoryName}</span>
            <span class="category-count">${filteredNodes.length}</span>
        `;
        headerDiv.onclick = () => categoryDiv.classList.toggle('collapsed');

        const itemsDiv = document.createElement('div');
        itemsDiv.className = 'tree-category-items';

        filteredNodes.forEach(node => {
            const itemDiv = createNodeItem(node);
            itemsDiv.appendChild(itemDiv);
            totalVisible++;
        });

        categoryDiv.appendChild(headerDiv);
        categoryDiv.appendChild(itemsDiv);
        container.appendChild(categoryDiv);
    }

    // 自定义节点分类
    if (state.customNodes.length > 0) {
        const categoryDiv = document.createElement('div');
        categoryDiv.className = 'tree-category';
        const headerDiv = document.createElement('div');
        headerDiv.className = 'tree-category-header';
        headerDiv.innerHTML = `
            <i class="fa fa-caret-down caret"></i>
            <i class="fa fa-cog category-icon" style="color: var(--accent-purple)"></i>
            <span>自定义</span>
            <span class="category-count">${state.customNodes.length}</span>
        `;
        headerDiv.onclick = () => categoryDiv.classList.toggle('collapsed');
        const itemsDiv = document.createElement('div');
        itemsDiv.className = 'tree-category-items';
        state.customNodes.forEach(node => {
            if (fuzzyMatch(node.name, filter) || fuzzyMatch(node.id, filter)) {
                itemsDiv.appendChild(createNodeItem(node));
            }
        });
        categoryDiv.appendChild(headerDiv);
        categoryDiv.appendChild(itemsDiv);
        container.appendChild(categoryDiv);
    }

    if (totalVisible === 0 && filter) {
        container.innerHTML = `
            <div class="empty-state">
                <i class="fa fa-search empty-icon"></i>
                <div class="empty-text">未找到匹配节点</div>
            </div>
        `;
    }
}

function createNodeItem(nodeDef) {
    const itemDiv = document.createElement('div');
    itemDiv.className = 'node-item';
    itemDiv.draggable = true;
    itemDiv.innerHTML = `
        <div class="node-icon" style="background: ${nodeDef.color}33; color: ${nodeDef.color}">
            <i class="fa ${nodeDef.icon || 'fa fa-cube'}"></i>
        </div>
        <div class="node-name" title="${nodeDef.id}">${nodeDef.name}</div>
        <button class="icon-btn fav-btn ${state.isFavorite(nodeDef.id) ? 'active' : ''}" title="收藏">
            <i class="fa ${state.isFavorite(nodeDef.id) ? 'fa-star' : 'fa-star-o'}"></i>
        </button>
    `;

    itemDiv.addEventListener('dragstart', e => {
        e.dataTransfer.setData('nodeType', nodeDef.id);
        e.dataTransfer.effectAllowed = 'copy';
        itemDiv.classList.add('dragging');
    });
    itemDiv.addEventListener('dragend', () => itemDiv.classList.remove('dragging'));
    itemDiv.addEventListener('dblclick', () => {
        // 在画布中央创建节点
        const cw = renderer.canvas.width / renderer.dpr / 2;
        const ch = renderer.canvas.height / renderer.dpr / 2;
        const x = (cw - state.panOffset.x) / state.zoom - CONFIG.NODE_WIDTH / 2;
        const y = (ch - state.panOffset.y) / state.zoom - 40;
        createNodeFromType(nodeDef.id, x, y);
    });
    itemDiv.querySelector('.fav-btn').addEventListener('click', e => {
        e.stopPropagation();
        state.toggleFavorite(nodeDef.id);
        renderNodeTree(document.getElementById('nodeSearch').value);
        renderFavorites();
    });

    return itemDiv;
}

function renderFavorites() {
    const container = document.getElementById('favoritesList');
    container.innerHTML = '';
    if (state.favorites.length === 0) {
        container.innerHTML = `
            <div class="empty-state">
                <i class="fa fa-star-o empty-icon"></i>
                <div class="empty-text">暂无收藏节点</div>
            </div>
        `;
        return;
    }
    state.favorites.forEach(typeId => {
        const nodeDef = getNodeType(typeId);
        if (nodeDef) {
            container.appendChild(createNodeItem(nodeDef));
        }
    });
}

function renderTemplates() {
    const container = document.getElementById('templatesList');
    container.innerHTML = '';
    for (const [id, template] of Object.entries(FLOW_TEMPLATES)) {
        const div = document.createElement('div');
        div.className = 'template-item';
        div.innerHTML = `
            <i class="fa ${template.icon} template-icon"></i>
            <span>${template.name}</span>
        `;
        div.onclick = () => loadTemplate(id);
        container.appendChild(div);
    }
}

// ==================== 参数面板渲染 ====================
function updateParamsPanel(node) {
    const container = document.getElementById('nodeInfo');
    if (!node) {
        if (state.selectedNodes.length === 0) {
            container.innerHTML = `
                <div class="empty-state">
                    <i class="fa fa-mouse-pointer empty-icon"></i>
                    <div class="empty-text">请选择一个节点查看参数</div>
                </div>
            `;
        } else {
            // 多选状态
            renderMultiSelectionPanel(container);
        }
        return;
    }

    const typeDef = getNodeType(node.type);
    const version = state.nodeVersions.get(node.type) || '1.0';

    container.innerHTML = `
        <div class="param-group">
            <div class="param-group-header" onclick="cmd('toggleParamGroup', this.parentElement)">
                <i class="fa fa-info-circle"></i>
                节点信息
                <i class="fa fa-caret-down caret"></i>
            </div>
            <div class="param-group-body">
                <div class="param-item">
                    <label class="param-label">节点名称</label>
                    <input class="param-input" type="text" value="${escapeHtml(node.name)}"
                           oninput="updateNodeField('${node.id}', 'name', this.value)">
                </div>
                <div class="param-item">
                    <label class="param-label">节点类型 <span class="param-type-badge">${node.type}</span></span></label>
                    <input class="param-input" type="text" value="${node.type}" readonly>
                </div>
                <div class="param-item">
                    <label class="param-label">版本 <span class="version-badge">v${version}</span></label>
                </div>
                <div class="param-item">
                    <label class="param-label">位置</label>
                    <div class="param-range">
                        <input class="param-input" type="number" value="${Math.round(node.x)}" style="flex: 1"
                               oninput="updateNodeField('${node.id}', 'x', parseFloat(this.value))">
                        <input class="param-input" type="number" value="${Math.round(node.y)}" style="flex: 1"
                               oninput="updateNodeField('${node.id}', 'y', parseFloat(this.value))">
                    </div>
                </div>
                <div class="param-item param-checkbox">
                    <input type="checkbox" id="enableNode" ${node.enabled !== false ? 'checked' : ''}
                           onchange="toggleNodeEnabled('${node.id}')">
                    <label for="enableNode">启用节点</label>
                </div>
                <div class="param-item param-checkbox">
                    <input type="checkbox" id="breakpointNode" ${state.breakpoints.has(node.id) ? 'checked' : ''}
                           onchange="toggleBreakpoint('${node.id}')">
                    <label for="breakpointNode">设置断点</label>
                </div>
            </div>
        </div>

        <div class="param-group">
            <div class="param-group-header" onclick="cmd('toggleParamGroup', this.parentElement)">
                <i class="fa fa-sliders"></i>
                参数配置
                <i class="fa fa-caret-down caret"></i>
            </div>
            <div class="param-group-body" id="paramItems">
                ${renderParamItems(node, typeDef)}
            </div>
        </div>

        <div class="param-group">
            <div class="param-group-header" onclick="cmd('toggleParamGroup', this.parentElement)">
                <i class="fa fa-sign-in"></i>
                输入端口 (${node.inputs ? node.inputs.length : 0})
                <i class="fa fa-caret-down caret"></i>
            </div>
            <div class="param-group-body">
                ${renderPorts(node.inputs, 'input')}
            </div>
        </div>

        <div class="param-group">
            <div class="param-group-header" onclick="cmd('toggleParamGroup', this.parentElement)">
                <i class="fa fa-sign-out"></i>
                输出端口 (${node.outputs ? node.outputs.length : 0})
                <i class="fa fa-caret-down caret"></i>
            </div>
            <div class="param-group-body">
                ${renderPorts(node.outputs, 'output')}
            </div>
        </div>

        <div style="display: flex; gap: 6px; margin-top: 12px;">
            <button class="modal-btn primary" style="flex: 1;" onclick="previewNodeOutput('${node.id}')">
                <i class="fa fa-eye"></i> 预览输出
            </button>
            <button class="modal-btn secondary" onclick="cmd('duplicateNode', '${node.id}')">
                <i class="fa fa-clone"></i>
            </button>
        </div>
    `;
}

function renderMultiSelectionPanel(container) {
    container.innerHTML = `
        <div class="param-group">
            <div class="param-group-header">
                <i class="fa fa-list"></i>
                多选 (${state.selectedNodes.length})
            </div>
            <div class="param-group-body">
                <div style="display: flex; flex-direction: column; gap: 6px;">
                    <button class="modal-btn primary" onclick="cmd('groupSelected')">
                        <i class="fa fa-object-group"></i> 创建分组
                    </button>
                    <button class="modal-btn secondary" onclick="cmd('alignLeft')">
                        <i class="fa fa-align-left"></i> 左对齐
                    </button>
                    <button class="modal-btn secondary" onclick="cmd('alignTop')">
                        <i class="fa fa-arrow-up"></i> 顶对齐
                    </button>
                    <button class="modal-btn secondary" onclick="cmd('distributeH')">
                        <i class="fa fa-arrows-h"></i> 水平等距
                    </button>
                    <button class="modal-btn secondary" onclick="cmd('distributeV')">
                        <i class="fa fa-ellipsis-v"></i> 垂直等距
                    </button>
                    <button class="modal-btn danger" onclick="cmd('deleteSelected')">
                        <i class="fa fa-trash"></i> 删除所选
                    </button>
                </div>
            </div>
        </div>
    `;
}

function renderParamItems(node, typeDef) {
    if (!typeDef || !typeDef.params) {
        return '<div class="empty-state"><div class="empty-text">无参数</div></div>';
    }
    let html = '';
    for (const [paramId, paramDef] of Object.entries(typeDef.params)) {
        const currentValue = node.params[paramId] !== undefined ? node.params[paramId] : paramDef.default;
        html += renderParamWidget(node.id, paramId, paramDef, currentValue);
    }
    return html;
}

function renderParamWidget(nodeId, paramId, paramDef, currentValue) {
    const onInput = `oninput="updateNodeParam('${nodeId}', '${paramId}', this.value, this)"`;
    const onChange = `onchange="updateNodeParam('${nodeId}', '${paramId}', this.value, this)"`;

    switch (paramDef.type) {
        case 'number':
            const step = paramDef.step || 1;
            return `
                <div class="param-item">
                    <label class="param-label">${paramDef.name} <span class="param-type-badge">number</span></label>
                    <div class="param-range">
                        <input type="range" min="${paramDef.min}" max="${paramDef.max}" step="${step}"
                               value="${currentValue}" ${onInput}>
                        <input type="number" class="param-input" style="width: 70px"
                               min="${paramDef.min}" max="${paramDef.max}" step="${step}"
                               value="${currentValue}" ${onChange}>
                    </div>
                </div>
            `;
        case 'select':
            const opts = paramDef.options.map(opt =>
                `<option value="${opt}" ${opt === currentValue ? 'selected' : ''}>${opt}</option>`
            ).join('');
            return `
                <div class="param-item">
                    <label class="param-label">${paramDef.name} <span class="param-type-badge">select</span></label>
                    <select class="param-select" ${onChange}>
                        ${opts}
                    </select>
                </div>
            `;
        case 'string':
            return `
                <div class="param-item">
                    <label class="param-label">${paramDef.name} <span class="param-type-badge">string</span></label>
                    <input class="param-input" type="text" value="${escapeHtml(currentValue || '')}" ${onInput}>
                </div>
            `;
        case 'text':
            return `
                <div class="param-item">
                    <label class="param-label">${paramDef.name} <span class="param-type-badge">text</span></label>
                    <textarea class="param-textarea" rows="4" ${onInput}>${escapeHtml(currentValue || '')}</textarea>
                </div>
            `;
        case 'boolean':
            return `
                <div class="param-item">
                    <label class="param-checkbox">
                        <input type="checkbox" ${currentValue ? 'checked' : ''}
                               onchange="updateNodeParam('${nodeId}', '${paramId}', this.checked, this)">
                        ${paramDef.name}
                    </label>
                </div>
            `;
        case 'color':
            return `
                <div class="param-item">
                    <label class="param-label">${paramDef.name} <span class="param-type-badge">color</span></label>
                    <div class="param-color">
                        <input type="color" value="${currentValue || '#89b4fa'}" ${onChange}>
                        <input type="text" class="param-input" value="${currentValue || '#89b4fa'}" ${onInput}>
                    </div>
                </div>
            `;
        default:
            return '';
    }
}

function renderPorts(ports, direction) {
    if (!ports || ports.length === 0) {
        return '<div class="empty-state"><div class="empty-text">无端口</div></div>';
    }
    const typeColors = {
        'image': 'var(--type-image)',
        'number': 'var(--type-number)',
        'string': 'var(--type-string)',
        'boolean': 'var(--type-boolean)',
        'region': 'var(--type-region)',
        'object': 'var(--type-object)'
    };
    return ports.map(p => `
        <div class="param-item" style="display: flex; align-items: center; gap: 8px;">
            <div style="width: 10px; height: 10px; border-radius: 50%; background: ${typeColors[p.type] || 'var(--text-muted)'}"></div>
            <span style="flex: 1; font-size: 12px;">${p.name}</span>
            <span class="param-type-badge">${p.type}</span>
        </div>
    `).join('');
}

function escapeHtml(str) {
    if (str === null || str === undefined) return '';
    return String(str)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#39;');
}

// ==================== 节点操作 ====================
function createNodeFromType(typeId, x, y) {
    const typeDef = getNodeType(typeId);
    if (!typeDef) {
        showToast(`未找到节点类型: ${typeId}`, 'error');
        return null;
    }

    state.pushUndo();

    const node = {
        id: generateId(),
        type: typeId,
        name: typeDef.name + ' ' + (state.nodes.size + 1),
        x: x,
        y: y,
        inputs: JSON.parse(JSON.stringify(typeDef.inputs || [])),
        outputs: JSON.parse(JSON.stringify(typeDef.outputs || [])),
        params: {},
        state: 'idle',
        executeTime: 0,
        enabled: true,
        collapsed: false,
        version: '1.0'
    };

    // 应用默认参数
    if (typeDef.params) {
        for (const [key, def] of Object.entries(typeDef.params)) {
            node.params[key] = def.default;
        }
    }

    // 网格吸附
    if (CONFIG.SNAP_TO_GRID && state.snapEnabled) {
        node.x = Math.round(node.x / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
        node.y = Math.round(node.y / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
    }

    state.addNode(node);
    state.selectNode(node.id);
    renderCanvas();
    updateStatusBar();
    updateParamsPanel(node);
    showToast(`创建节点: ${node.name}`, 'success');
    return node;
}

function updateNodeField(nodeId, field, value) {
    const node = state.nodes.get(nodeId);
    if (node) {
        node[field] = value;
        state.markModified();
        renderCanvas();
    }
}

function updateNodeParam(nodeId, paramId, value, element) {
    const node = state.nodes.get(nodeId);
    if (node) {
        node.params[paramId] = value;
        state.markModified();
        // 更新滑块旁的数值
        if (element && element.type === 'range') {
            const numInput = element.parentElement.querySelector('input[type="number"]');
            if (numInput) numInput.value = value;
        }
        // 实时预览
        if (state.isRunning) {
            previewNodeOutput(nodeId);
        }
    }
}

function toggleNodeEnabled(nodeId) {
    const node = state.nodes.get(nodeId);
    if (node) {
        state.pushUndo();
        node.enabled = !node.enabled;
        state.markModified();
        renderCanvas();
        showToast(`节点${node.enabled ? '已启用' : '已禁用'}`, 'info');
    }
}

function toggleBreakpoint(nodeId) {
    if (state.breakpoints.has(nodeId)) {
        state.breakpoints.delete(nodeId);
    } else {
        state.breakpoints.add(nodeId);
        if (!state.debugMode) {
            cmd('toggleDebug');
        }
    }
    updateBreakpointsList();
    renderCanvas();
}

// ==================== 命令调度器 ====================
function cmd(name, ...args) {
    const commands = {
        newFlow, openFlow, saveFlow, exportFlow, importFlow,
        undo, redo, copyNodes, pasteNodes, cutNodes, deleteSelected,
        duplicateSelected, duplicateNode, selectAll, deselectAll,
        alignLeft, alignCenter, alignRight, alignTop, alignMiddle, alignBottom,
        distributeH, distributeV,
        zoomIn, zoomOut, fitToScreen, resetView, toggleGrid, toggleSnap, toggleMinimap,
        toggleTheme, toggleDebug, toggleBottomPanel, toggleParamGroup,
        runFlow, stopFlow, stepFlow, stepOver, stepInto, stepOut, continueExec,
        groupSelected, ungroupSelected, createComment,
        batchRun, compareFlows, showShortcuts, hideShortcuts,
        expandAllCategories, collapseAllCategories, importCustomNode, clearFavorites,
        clearLogs, toggleRoiMode, toggleHistogram, adjustContrast,
        zoomImageIn, zoomImageOut, fitImage, previewNodeOutput, viewNodeOutput
    };
    if (commands[name]) {
        commands[name](...args);
    } else {
        console.warn('Unknown command:', name);
    }
}

// ==================== 编辑操作 ====================
function copyNodes() {
    if (state.selectedNodes.length === 0) {
        showToast('请先选择节点', 'warning');
        return;
    }
    state.copySelected();
    showToast(`已复制 ${state.clipboard.nodes.length} 个节点`, 'success');
}

function pasteNodes() {
    if (state.clipboard.nodes.length === 0) {
        showToast('剪贴板为空', 'warning');
        return;
    }
    state.pushUndo();
    const newNodes = state.paste();
    state.selectedNodes = newNodes.map(n => n.id);
    renderCanvas();
    updateStatusBar();
    showToast(`已粘贴 ${newNodes.length} 个节点`, 'success');
}

function cutNodes() {
    if (state.selectedNodes.length === 0) {
        showToast('请先选择节点', 'warning');
        return;
    }
    state.copySelected();
    deleteSelected();
}

function deleteSelected() {
    if (state.selectedNodes.length === 0 && state.selectedConnections.length === 0) {
        showToast('请先选择节点或连接', 'warning');
        return;
    }
    state.pushUndo();
    state.selectedNodes.forEach(id => state.removeNode(id));
    state.selectedConnections.forEach(id => state.removeConnection(id));
    state.deselectAll();
    renderCanvas();
    updateStatusBar();
    updateParamsPanel(null);
    showToast('已删除选中项', 'success');
}

function duplicateSelected() {
    if (state.selectedNodes.length === 0) return;
    state.copySelected();
    const newNodes = state.paste(20, 20);
    state.selectedNodes = newNodes.map(n => n.id);
    renderCanvas();
    showToast('已复制节点', 'success');
}

function duplicateNode(nodeId) {
    state.selectedNodes = [nodeId];
    duplicateSelected();
}

function selectAll() {
    state.selectAll();
    renderCanvas();
    updateParamsPanel(null);
}

function deselectAll() {
    state.deselectAll();
    renderCanvas();
    updateParamsPanel(null);
}

// ==================== 对齐工具 ====================
function alignNodes(type) {
    if (state.selectedNodes.length < 2) {
        showToast('至少选择2个节点', 'warning');
        return;
    }
    state.pushUndo();
    const nodes = state.selectedNodes.map(id => state.nodes.get(id)).filter(n => n);
    switch (type) {
        case 'left':
            const minX = Math.min(...nodes.map(n => n.x));
            nodes.forEach(n => n.x = minX);
            break;
        case 'right':
            const maxX = Math.max(...nodes.map(n => n.x + CONFIG.NODE_WIDTH));
            nodes.forEach(n => n.x = maxX - CONFIG.NODE_WIDTH);
            break;
        case 'center':
            const avgX = nodes.reduce((s, n) => s + n.x, 0) / nodes.length;
            nodes.forEach(n => n.x = avgX);
            break;
        case 'top':
            const minY = Math.min(...nodes.map(n => n.y));
            nodes.forEach(n => n.y = minY);
            break;
        case 'bottom':
            const maxY = Math.max(...nodes.map(n => n.y + 80));
            nodes.forEach(n => n.y = maxY - 80);
            break;
        case 'middle':
            const avgY = nodes.reduce((s, n) => s + n.y, 0) / nodes.length;
            nodes.forEach(n => n.y = avgY);
            break;
    }
    state.markModified();
    renderCanvas();
}

function alignLeft() { alignNodes('left'); }
function alignCenter() { alignNodes('center'); }
function alignRight() { alignNodes('right'); }
function alignTop() { alignNodes('top'); }
function alignMiddle() { alignNodes('middle'); }
function alignBottom() { alignNodes('bottom'); }

function distributeH() {
    if (state.selectedNodes.length < 3) {
        showToast('至少选择3个节点', 'warning');
        return;
    }
    state.pushUndo();
    const nodes = state.selectedNodes.map(id => state.nodes.get(id)).filter(n => n);
    nodes.sort((a, b) => a.x - b.x);
    const first = nodes[0];
    const last = nodes[nodes.length - 1];
    const totalSpace = (last.x + CONFIG.NODE_WIDTH) - first.x;
    const totalNodesWidth = nodes.length * CONFIG.NODE_WIDTH;
    const gap = (totalSpace - totalNodesWidth) / (nodes.length - 1);
    nodes.forEach((n, i) => {
        n.x = first.x + i * (CONFIG.NODE_WIDTH + gap);
    });
    state.markModified();
    renderCanvas();
}

function distributeV() {
    if (state.selectedNodes.length < 3) {
        showToast('至少选择3个节点', 'warning');
        return;
    }
    state.pushUndo();
    const nodes = state.selectedNodes.map(id => state.nodes.get(id)).filter(n => n);
    nodes.sort((a, b) => a.y - b.y);
    const first = nodes[0];
    const last = nodes[nodes.length - 1];
    const totalSpace = (last.y + 80) - first.y;
    const totalNodesHeight = nodes.length * 80;
    const gap = (totalSpace - totalNodesHeight) / (nodes.length - 1);
    nodes.forEach((n, i) => {
        n.y = first.y + i * (80 + gap);
    });
    state.markModified();
    renderCanvas();
}

function toggleAlignTools() {
    const tools = document.getElementById('alignTools');
    if (state.selectedNodes.length >= 2) {
        tools.classList.add('active');
    } else {
        tools.classList.remove('active');
    }
}

// ==================== 视图操作 ====================
function zoomIn() {
    state.zoom = clamp(state.zoom + CONFIG.ZOOM_STEP, CONFIG.ZOOM_MIN, CONFIG.ZOOM_MAX);
    updateZoomLevel();
    renderCanvas();
}

function zoomOut() {
    state.zoom = clamp(state.zoom - CONFIG.ZOOM_STEP, CONFIG.ZOOM_MIN, CONFIG.ZOOM_MAX);
    updateZoomLevel();
    renderCanvas();
}

function fitToScreen() {
    const bounds = renderer.getNodesBounds();
    if (bounds.width === 0) return;
    const cw = renderer.canvas.width / renderer.dpr;
    const ch = renderer.canvas.height / renderer.dpr;
    const scaleX = cw / (bounds.width + 200);
    const scaleY = ch / (bounds.height + 200);
    state.zoom = clamp(Math.min(scaleX, scaleY), CONFIG.ZOOM_MIN, CONFIG.ZOOM_MAX);
    state.panOffset.x = (cw - bounds.width * state.zoom) / 2 - bounds.minX * state.zoom;
    state.panOffset.y = (ch - bounds.height * state.zoom) / 2 - bounds.minY * state.zoom;
    updateZoomLevel();
    renderCanvas();
}

function resetView() {
    state.zoom = 1;
    state.panOffset = { x: 0, y: 0 };
    updateZoomLevel();
    renderCanvas();
}

function toggleGrid() {
    state.gridVisible = !state.gridVisible;
    renderCanvas();
}

function toggleSnap() {
    state.snapEnabled = !state.snapEnabled;
    showToast(`网格吸附${state.snapEnabled ? '已开启' : '已关闭'}`, 'info');
}

function toggleMinimap() {
    state.minimapVisible = !state.minimapVisible;
    const minimap = document.getElementById('minimap');
    minimap.classList.toggle('hidden', !state.minimapVisible);
    renderCanvas();
}

function toggleTheme() {
    state.theme = state.theme === 'dark' ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', state.theme);
    localStorage.setItem(CONFIG.THEME_KEY, state.theme);
    const icon = document.getElementById('themeIcon');
    if (icon) {
        icon.className = state.theme === 'dark' ? 'fa fa-moon-o btn-icon' : 'fa fa-sun-o btn-icon';
    }
    renderCanvas();
    showToast(`已切换到${state.theme === 'dark' ? '深色' : '浅色'}主题`, 'info');
}

function toggleBottomPanel() {
    document.getElementById('appContainer').classList.toggle('bottom-collapsed');
}

function toggleParamGroup(groupEl) {
    groupEl.classList.toggle('collapsed');
}

// ==================== 文件操作 ====================
function newFlow() {
    if (state.modified) {
        showConfirmDialog('当前流程未保存，是否新建？', () => {
            state.nodes.clear();
            state.connections = [];
            state.groups = [];
            state.undoStack = [];
            state.redoStack = [];
            state.currentFlowId = null;
            state.currentFlowName = '未命名';
            state.clearModified();
            renderCanvas();
            updateStatusBar();
            updateFlowName();
            showToast('已新建流程', 'success');
        });
    } else {
        state.nodes.clear();
        state.connections = [];
        state.groups = [];
        state.currentFlowId = null;
        state.currentFlowName = '未命名';
        renderCanvas();
        updateStatusBar();
        updateFlowName();
        showToast('已新建流程', 'success');
    }
}

async function openFlow() {
    showInputDialog('打开流程', '请输入流程ID或文件路径', '', async (filepath) => {
        try {
            const result = await apiClient.get(`/flow/load?path=${encodeURIComponent(filepath)}`);
            if (result.success) {
                state.fromJSON(result.flow);
                showToast('流程加载成功', 'success');
            } else {
                showToast(`加载失败: ${result.error}`, 'error');
            }
        } catch (error) {
            showToast(`加载失败: ${error.message}`, 'error');
        }
    });
}

async function saveFlow() {
    const flowDef = state.toJSON();
    try {
        const result = state.currentFlowId ?
            await apiClient.updateFlow(state.currentFlowId, flowDef) :
            await apiClient.createFlow(flowDef);
        if (result.success) {
            state.currentFlowId = result.flow_id || state.currentFlowId;
            state.clearModified();
            showToast('流程保存成功', 'success');
            // 保存版本
            state.flowVersions.push({
                timestamp: new Date().toISOString(),
                flow: JSON.parse(JSON.stringify(flowDef))
            });
        } else {
            showToast(`保存失败: ${result.error}`, 'error');
        }
    } catch (error) {
        showToast(`保存失败: ${error.message}`, 'error');
    }
}

function exportFlow() {
    const flowDef = state.toJSON();
    const json = JSON.stringify(flowDef, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${state.currentFlowName}_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
    showToast('流程已导出', 'success');
}

function importFlow() {
    const fileInput = document.getElementById('fileInput');
    fileInput.accept = '.json';
    fileInput.onchange = (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (ev) => {
            try {
                const json = JSON.parse(ev.target.result);
                state.fromJSON(json);
                showToast('流程已导入', 'success');
            } catch (err) {
                showToast('导入失败：JSON格式错误', 'error');
            }
        };
        reader.readAsText(file);
    };
    fileInput.click();
}

function loadTemplate(templateId) {
    const template = FLOW_TEMPLATES[templateId];
    if (!template) {
        showToast('未找到模板', 'error');
        return;
    }

    state.pushUndo();
    state.nodes.clear();
    state.connections = [];
    state.groups = [];

    const createdNodes = [];
    template.nodes.forEach((nodeDef, index) => {
        const typeDef = getNodeType(nodeDef.type);
        const node = {
            id: generateId(),
            type: nodeDef.type,
            name: nodeDef.name,
            x: nodeDef.x,
            y: nodeDef.y,
            inputs: JSON.parse(JSON.stringify(typeDef?.inputs || [])),
            outputs: JSON.parse(JSON.stringify(typeDef?.outputs || [])),
            params: {},
            state: 'idle',
            enabled: true,
            collapsed: false,
            version: '1.0'
        };
        if (typeDef?.params) {
            for (const [key, def] of Object.entries(typeDef.params)) {
                node.params[key] = def.default;
            }
        }
        state.addNode(node);
        createdNodes.push(node);
    });

    // 创建连接
    template.connections.forEach(([srcIdx, tgtIdx, srcPort, tgtPort]) => {
        const source = createdNodes[srcIdx];
        const target = createdNodes[tgtIdx];
        if (source && target) {
            state.addConnection({
                id: generateId('c'),
                sourceId: source.id,
                sourcePort: srcPort,
                targetId: target.id,
                targetPort: tgtPort,
                dataType: source.outputs.find(p => p.id === srcPort)?.type || 'object'
            });
        }
    });

    state.currentFlowName = template.name;
    state.clearModified();
    renderCanvas();
    updateStatusBar();
    updateFlowName();
    setTimeout(() => fitToScreen(), 50);
    showToast(`已加载模板: ${template.name}`, 'success');
}

function importCustomNode() {
    showInputDialog('导入自定义节点', '请输入节点JSON定义', '', (jsonStr) => {
        try {
            const nodeDef = JSON.parse(jsonStr);
            if (!nodeDef.id || !nodeDef.name) {
                showToast('节点定义缺少必需字段', 'error');
                return;
            }
            state.addCustomNode(nodeDef);
            renderNodeTree();
            showToast(`自定义节点已导入: ${nodeDef.name}`, 'success');
        } catch (err) {
            showToast('JSON解析失败', 'error');
        }
    });
}

function clearFavorites() {
    state.favorites = [];
    localStorage.setItem(CONFIG.FAVORITES_KEY, JSON.stringify(state.favorites));
    renderFavorites();
    renderNodeTree(document.getElementById('nodeSearch').value);
    showToast('收藏夹已清空', 'info');
}

function expandAllCategories() {
    document.querySelectorAll('.tree-category').forEach(c => c.classList.remove('collapsed'));
}

function collapseAllCategories() {
    document.querySelectorAll('.tree-category').forEach(c => c.classList.add('collapsed'));
}

// ==================== 分组操作 ====================
function groupSelected() {
    if (state.selectedNodes.length < 2) {
        showToast('至少选择2个节点', 'warning');
        return;
    }
    state.pushUndo();
    const group = {
        id: generateId('g'),
        name: `分组 ${state.groups.length + 1}`,
        nodeIds: [...state.selectedNodes]
    };
    state.groups.push(group);
    renderCanvas();
    showToast(`已创建分组: ${group.name}`, 'success');
}

function ungroupSelected() {
    if (state.selectedNodes.length === 0) return;
    state.pushUndo();
    state.groups = state.groups.filter(g => {
        return !g.nodeIds.some(id => state.selectedNodes.includes(id));
    });
    renderCanvas();
    showToast('已取消分组', 'success');
}

function createComment() {
    showInputDialog('创建注释', '请输入注释内容', '', (text) => {
        state.pushUndo();
        state.comments.push({
            id: generateId('cm'),
            type: 'comment',
            text: text,
            x: 100,
            y: 100
        });
        showToast('注释已创建', 'success');
    });
}

// ==================== 执行操作 ====================
async function runFlow() {
    if (state.isRunning) {
        showToast('流程正在执行中', 'warning');
        return;
    }
    if (state.nodes.size === 0) {
        showToast('流程为空', 'warning');
        return;
    }

    state.isRunning = true;
    updateStatus('running', '执行中');
    showProgress('正在执行流程...');
    logMessage('开始执行流程', 'info', 'execution');

    try {
        const flowDef = state.toJSON();
        // 清除之前的状态
        state.nodes.forEach(n => {
            n.state = 'idle';
            n.executeTime = 0;
        });

        const result = await apiClient.executeFlow(state.currentFlowId || 'temp');

        if (result.success) {
            logMessage('流程执行完成', 'info', 'execution');
            if (result.execution) {
                handleExecutionResult(result.execution);
            }
        } else {
            showToast(`执行失败: ${result.error}`, 'error');
            logMessage(`执行失败: ${result.error}`, 'error', 'errors');
            // 高亮错误节点
            if (result.error_node_id) {
                const node = state.nodes.get(result.error_node_id);
                if (node) {
                    node.state = 'error';
                    renderCanvas();
                }
            }
        }
    } catch (error) {
        showToast(`执行失败: ${error.message}`, 'error');
        logMessage(`执行异常: ${error.message}`, 'error', 'errors');
    }

    hideProgress();
    state.isRunning = false;
    updateStatus('ready', '就绪');
}

async function stopFlow() {
    if (!state.isRunning) return;
    await apiClient.stopExecution();
    state.isRunning = false;
    updateStatus('ready', '已停止');
    hideProgress();
    logMessage('流程已停止', 'warn', 'execution');
}

async function stepFlow() {
    if (state.nodes.size === 0) {
        showToast('流程为空', 'warning');
        return;
    }
    try {
        const result = await apiClient.stepFlow();
        if (result.success) {
            if (result.node_id) {
                updateNodeState(result.node_id, 'success');
                if (result.execute_time) {
                    const node = state.nodes.get(result.node_id);
                    if (node) node.executeTime = result.execute_time;
                }
                logMessage(`单步执行: ${result.node_name || result.node_id}`, 'info', 'execution');
            }
        } else {
            showToast(result.error || '单步执行失败', 'error');
        }
    } catch (error) {
        showToast(`单步失败: ${error.message}`, 'error');
    }
}

function toggleDebug() {
    state.debugMode = !state.debugMode;
    showToast(`调试模式${state.debugMode ? '已开启' : '已关闭'}`, 'info');
    if (state.debugMode) {
        apiClient.connectWebSocket();
        switchPanel('debug');
    }
    updateStatus(state.debugMode ? 'debug' : 'ready', state.debugMode ? '调试中' : '就绪');
}

async function stepOver() {
    await apiClient.post('/debug/step_over');
    logMessage('Step Over', 'debug', 'execution');
}

async function stepInto() {
    await apiClient.post('/debug/step_into');
    logMessage('Step Into', 'debug', 'execution');
}

async function stepOut() {
    await apiClient.post('/debug/step_out');
    logMessage('Step Out', 'debug', 'execution');
}

async function continueExec() {
    await apiClient.post('/debug/continue');
    logMessage('继续执行', 'info', 'execution');
}

// ==================== 调试功能 ====================
function updateBreakpointsList() {
    const container = document.getElementById('breakpointsList');
    if (state.breakpoints.size === 0) {
        container.innerHTML = `
            <div class="empty-state">
                <i class="fa fa-circle-o empty-icon"></i>
                <div class="empty-text">无断点</div>
            </div>
        `;
        return;
    }
    container.innerHTML = '';
    state.breakpoints.forEach(nodeId => {
        const node = state.nodes.get(nodeId);
        if (node) {
            const div = document.createElement('div');
            div.className = 'breakpoint-item';
            div.innerHTML = `
                <i class="fa fa-circle bp-icon"></i>
                <span class="node-name">${escapeHtml(node.name)}</span>
                <span class="remove-btn" onclick="state.breakpoints.delete('${nodeId}'); updateBreakpointsList(); renderCanvas();">
                    <i class="fa fa-times"></i>
                </span>
            `;
            container.appendChild(div);
        }
    });
}

function handleDebugInfo(data) {
    const container = document.getElementById('variablesTree');
    if (!data.variables || Object.keys(data.variables).length === 0) {
        container.innerHTML = `
            <div class="empty-state">
                <i class="fa fa-cube empty-icon"></i>
                <div class="empty-text">暂无变量</div>
            </div>
        `;
        return;
    }
    container.innerHTML = '';
    container.appendChild(renderVariableTree(data.variables, 'root'));
}

function renderVariableTree(data, name, depth = 0) {
    if (depth > 5) return document.createTextNode('...');

    const div = document.createElement('div');
    div.className = 'var-node';

    const row = document.createElement('div');
    row.className = 'var-row' + (typeof data === 'object' && data !== null ? '' : ' collapsed');

    const hasChildren = typeof data === 'object' && data !== null && Object.keys(data).length > 0;

    row.innerHTML = `
        <i class="fa fa-caret-${hasChildren ? 'down' : 'right'} caret"></i>
        <span class="var-name">${escapeHtml(name)}</span>
        ${typeof data === 'object' && data !== null ?
            `<span class="var-type">${Array.isArray(data) ? `Array[${data.length}]` : 'Object'}</span>` :
            `<span class="var-value">${escapeHtml(String(data))}</span>`
        }
    `;

    if (hasChildren) {
        row.onclick = () => {
            row.classList.toggle('collapsed');
            const next = row.nextElementSibling;
            if (next) next.style.display = row.classList.contains('collapsed') ? 'none' : 'block';
        };
    }

    div.appendChild(row);

    if (hasChildren) {
        const children = document.createElement('div');
        children.className = 'var-children';
        for (const [k, v] of Object.entries(data)) {
            children.appendChild(renderVariableTree(v, k, depth + 1));
        }
        div.appendChild(children);
    }

    return div;
}

function updateNodeState(nodeId, nodeState) {
    const node = state.nodes.get(nodeId);
    if (node) {
        node.state = nodeState;
        renderCanvas();
    }
}

function updateExecutionProgress(data) {
    updateProgress(data.percent || 0, data.message);
    if (data.current_node) {
        updateNodeState(data.current_node, 'running');
    }
}

function handleExecutionResult(result) {
    // 更新节点状态
    if (result.node_states) {
        for (const [nodeId, nodeState] of Object.entries(result.node_states)) {
            updateNodeState(nodeId, nodeState);
        }
    }
    // 更新执行时间
    if (result.node_times) {
        for (const [nodeId, time] of Object.entries(result.node_times)) {
            const node = state.nodes.get(nodeId);
            if (node) node.executeTime = time;
        }
    }
    // 总耗时
    const totalEl = document.getElementById('executionTime');
    if (totalEl) {
        totalEl.textContent = (result.total_time_us ? result.total_time_us / 1000 : 0).toFixed(2) + ' ms';
    }
    // 性能统计
    if (result.node_times) {
        updatePerformanceStats(result.node_times, result.total_time_us);
    }
    // 显示结果
    if (result.outputs) {
        showExecutionResults(result.outputs);
    }
    hideProgress();
}

function updatePerformanceStats(nodeTimes, totalTime) {
    const container = document.getElementById('perfStats');
    if (!container) return;

    const totalMs = totalTime / 1000;
    const entries = Object.entries(nodeTimes).map(([id, t]) => {
        const node = state.nodes.get(id);
        return { id, name: node?.name || id, time: t / 1000 };
    }).sort((a, b) => b.time - a.time);

    document.getElementById('totalTime').textContent = totalMs.toFixed(2) + ' ms';
    document.getElementById('nodeCount2').textContent = entries.length;
    document.getElementById('slowestNode').textContent = entries[0]?.name || '-';

    container.innerHTML = entries.map(e => `
        <div class="perf-item">
            <div class="perf-row">
                <span class="perf-name">${escapeHtml(e.name)}</span>
                <span class="perf-time">${e.time.toFixed(2)} ms</span>
            </div>
            <div class="perf-bar">
                <div class="perf-bar-fill" style="width: ${(e.time / totalMs * 100).toFixed(1)}%"></div>
            </div>
        </div>
    `).join('');
}

// ==================== 结果可视化 ====================
function showExecutionResults(outputs) {
    const container = document.getElementById('resultItems');
    container.innerHTML = '';

    if (Object.keys(outputs).length === 0) {
        container.innerHTML = `
            <div class="empty-state">
                <i class="fa fa-bar-chart empty-icon"></i>
                <div class="empty-text">暂无执行结果</div>
            </div>
        `;
        return;
    }

    for (const [key, value] of Object.entries(outputs)) {
        const div = document.createElement('div');
        div.className = 'result-item';

        if (typeof value === 'string' && value.startsWith('data:image')) {
            div.innerHTML = `
                <div class="result-header"><i class="fa fa-image"></i> ${escapeHtml(key)}</div>
                <img src="${value}" style="max-width: 100%; max-height: 250px; border-radius: 4px;">
            `;
        } else if (key === 'histogram' && Array.isArray(value)) {
            const chartCanvas = document.createElement('canvas');
            chartCanvas.className = 'result-chart';
            div.appendChild(chartCanvas);
            new Chart(chartCanvas, {
                type: 'bar',
                data: {
                    labels: value.map((_, i) => i.toString()),
                    datasets: [{
                        label: '直方图',
                        data: value,
                        backgroundColor: 'rgba(137, 180, 250, 0.7)'
                    }]
                },
                options: { responsive: true, maintainAspectRatio: false }
            });
        } else if (typeof value === 'object' && value !== null) {
            if (Array.isArray(value) && value.length > 0 && typeof value[0] === 'object') {
                // 表格
                const cols = Object.keys(value[0]);
                const table = document.createElement('table');
                table.className = 'result-table';
                table.innerHTML = `
                    <thead><tr>${cols.map(c => `<th>${c}</th>`).join('')}</tr></thead>
                    <tbody>
                        ${value.map(row => `<tr>${cols.map(c => `<td>${escapeHtml(String(row[c]))}</td>`).join('')}</tr>`).join('')}
                    </tbody>
                `;
                div.appendChild(table);
            } else {
                // 键值对表格
                const table = document.createElement('table');
                table.className = 'result-table';
                table.innerHTML = `
                    <thead><tr><th>属性</th><th>值</th></tr></thead>
                    <tbody>
                        ${Object.entries(value).map(([k, v]) =>
                            `<tr><td>${escapeHtml(k)}</td><td>${escapeHtml(JSON.stringify(v))}</td></tr>`
                        ).join('')}
                    </tbody>
                `;
                div.appendChild(table);
            }
        } else {
            div.innerHTML = `
                <div class="result-header"><i class="fa fa-info-circle"></i> ${escapeHtml(key)}</div>
                <div>${escapeHtml(String(value))}</div>
            `;
        }

        container.appendChild(div);
    }

    switchPanel('results');
}

// ==================== 图像预览 ====================
let imagePreviewZoom = 1;
let roiMode = false;
let histogramVisible = false;

async function previewNodeOutput(nodeId) {
    const node = state.nodes.get(nodeId);
    if (!node) return;

    try {
        const result = await apiClient.previewNode(nodeId);
        if (result.success && result.image) {
            showImagePreview(result.image, node.name);
            switchPanel('preview');
        } else {
            showToast('无法获取节点输出', 'warning');
        }
    } catch (error) {
        showToast(`预览失败: ${error.message}`, 'error');
    }
}

function viewNodeOutput() {
    if (state.selectedNodes.length > 0) {
        previewNodeOutput(state.selectedNodes[0]);
    }
}

function showImagePreview(imageData, title = '图像预览') {
    const canvas = document.getElementById('imagePreviewCanvas');
    const ctx = canvas.getContext('2d');
    const img = new Image();
    img.onload = () => {
        canvas.width = img.width;
        canvas.height = img.height;
        ctx.drawImage(img, 0, 0);
        document.getElementById('previewTitle').textContent = title;
        document.getElementById('imageInfo').textContent = `${img.width} × ${img.height}`;
    };
    img.src = imageData;
}

function zoomImageIn() {
    imagePreviewZoom = Math.min(8, imagePreviewZoom * 1.25);
    updateImagePreview();
}

function zoomImageOut() {
    imagePreviewZoom = Math.max(0.1, imagePreviewZoom / 1.25);
    updateImagePreview();
}

function fitImage() {
    imagePreviewZoom = 1;
    updateImagePreview();
}

function toggleRoiMode() {
    roiMode = !roiMode;
    showToast(`ROI模式${roiMode ? '已开启' : '已关闭'}`, 'info');
}

function toggleHistogram() {
    histogramVisible = !histogramVisible;
    showToast(`直方图${histogramVisible ? '已显示' : '已隐藏'}`, 'info');
}

function adjustContrast() {
    showToast('对比度调整功能', 'info');
}

function updateImagePreview() {
    const canvas = document.getElementById('imagePreviewCanvas');
    if (!canvas) return;
    canvas.style.transform = `scale(${imagePreviewZoom})`;
    canvas.style.transformOrigin = 'center';
}

// ==================== 流程比较 ====================
function compareFlows() {
    showInputDialog('流程比较', '请输入要比较的流程JSON或文件路径', '', (otherFlowJson) => {
        try {
            const other = JSON.parse(otherFlowJson);
            const current = state.toJSON();
            showDiffView(current, other);
        } catch (err) {
            showToast('JSON解析失败', 'error');
        }
    });
}

function showDiffView(flow1, flow2) {
    const lines1 = JSON.stringify(flow1, null, 2).split('\n');
    const lines2 = JSON.stringify(flow2, null, 2).split('\n');
    const modal = document.getElementById('modalContent');
    let html = `
        <div class="modal-header">
            <i class="fa fa-columns"></i> 流程差异比较
            <span class="close-btn" onclick="hideModal()"><i class="fa fa-times"></i></span>
        </div>
        <div class="modal-body">
            <div class="diff-view">
    `;
    const maxLen = Math.max(lines1.length, lines2.length);
    for (let i = 0; i < maxLen; i++) {
        const l1 = lines1[i] || '';
        const l2 = lines2[i] || '';
        if (l1 === l2) {
            html += `<div class="diff-line unchanged"><span class="diff-prefix"> </span>${escapeHtml(l1)}</div>`;
        } else {
            if (l1) html += `<div class="diff-line removed"><span class="diff-prefix">-</span>${escapeHtml(l1)}</div>`;
            if (l2) html += `<div class="diff-line added"><span class="diff-prefix">+</span>${escapeHtml(l2)}</div>`;
        }
    }
    html += `
            </div>
        </div>
        <div class="modal-footer">
            <button class="modal-btn secondary" onclick="hideModal()">关闭</button>
        </div>
    `;
    modal.innerHTML = html;
    document.getElementById('modalOverlay').classList.add('active');
}

// ==================== 批处理运行 ====================
function batchRun() {
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header">
            <i class="fa fa-files-o"></i> 批处理运行
            <span class="close-btn" onclick="hideModal()"><i class="fa fa-times"></i></span>
        </div>
        <div class="modal-body">
            <p style="margin-bottom: 10px;">选择要批量执行的流程文件（多选）：</p>
            <input type="file" id="batchFiles" multiple accept=".json" style="margin-bottom: 10px;">
            <div style="margin-top: 10px;">
                <label class="param-checkbox">
                    <input type="checkbox" id="batchParallel" checked> 并行执行
                </label>
            </div>
        </div>
        <div class="modal-footer">
            <button class="modal-btn secondary" onclick="hideModal()">取消</button>
            <button class="modal-btn primary" onclick="executeBatch()">
                <i class="fa fa-play"></i> 开始执行
            </button>
        </div>
    `;
    document.getElementById('modalOverlay').classList.add('active');
}

async function executeBatch() {
    const files = document.getElementById('batchFiles').files;
    if (files.length === 0) {
        showToast('请选择文件', 'warning');
        return;
    }
    hideModal();
    showProgress('批量执行中...');
    let success = 0, failed = 0;
    for (let i = 0; i < files.length; i++) {
        updateProgress((i / files.length) * 100, `执行 ${i + 1}/${files.length}: ${files[i].name}`);
        try {
            const text = await files[i].text();
            const flow = JSON.parse(text);
            const result = await apiClient.createFlow(flow);
            if (result.success) {
                const execResult = await apiClient.executeFlow(result.flow_id);
                if (execResult.success) success++;
                else failed++;
            } else {
                failed++;
            }
        } catch (err) {
            failed++;
            logMessage(`批处理失败: ${files[i].name} - ${err.message}`, 'error', 'errors');
        }
    }
    hideProgress();
    showToast(`批处理完成: 成功 ${success}，失败 ${failed}`, success > 0 ? 'success' : 'error');
}

// ==================== UI 辅助函数 ====================
function switchPanel(name) {
    document.querySelectorAll('.panel-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.panel-pane').forEach(p => p.classList.remove('active'));
    const tab = document.querySelector(`.panel-tab[data-pane="${name}"]`);
    const pane = document.getElementById(`pane-${name}`);
    if (tab) tab.classList.add('active');
    if (pane) {
        pane.classList.add('active');
        pane.style.display = 'block';
    }
}

function switchLog(logType) {
    document.querySelectorAll('.bottom-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.log-pane').forEach(p => p.classList.remove('active'));
    const tab = document.querySelector(`.bottom-tab[data-log="${logType}"]`);
    const pane = document.getElementById(`logPane-${logType}`);
    if (tab) tab.classList.add('active');
    if (pane) pane.classList.add('active');
}

function logMessage(message, level = 'info', target = 'console') {
    const pane = document.getElementById(`logPane-${target}`);
    if (!pane) return;

    // 过滤
    if (state.logFilter && !message.toLowerCase().includes(state.logFilter.toLowerCase())) return;
    if (state.currentLogLevel !== 'all' && level !== state.currentLogLevel) return;

    const time = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    entry.innerHTML = `
        <span class="log-time">${time}</span>
        <span class="log-level ${level}">${level.toUpperCase()}</span>
        <span class="log-message">${escapeHtml(message)}</span>
    `;
    pane.appendChild(entry);

    // 自动滚动
    if (state.autoScroll) {
        pane.scrollTop = pane.scrollHeight;
    }

    // 错误计数
    if (level === 'error' && target !== 'errors') {
        const errorPane = document.getElementById('logPane-errors');
        if (errorPane) {
            const e = entry.cloneNode(true);
            errorPane.appendChild(e);
            if (state.autoScroll) errorPane.scrollTop = errorPane.scrollHeight;
        }
        const badge = document.getElementById('errorBadge');
        const count = parseInt(badge.textContent || '0') + 1;
        badge.textContent = count;
        badge.style.display = 'inline-block';
    }

    // 同时记录到对应分类
    if (target === 'console' && (level === 'info' || level === 'success')) {
        // 不重复
    } else if (target !== 'console') {
        const consolePane = document.getElementById('logPane-console');
        if (consolePane) {
            const e = entry.cloneNode(true);
            consolePane.appendChild(e);
            if (state.autoScroll) consolePane.scrollTop = consolePane.scrollHeight;
        }
    }
}

function clearLogs() {
    document.querySelectorAll('.log-pane').forEach(p => p.innerHTML = '');
    const badge = document.getElementById('errorBadge');
    badge.textContent = '0';
    badge.style.display = 'none';
    showToast('日志已清空', 'info');
}

function updateStatusBar() {
    document.getElementById('nodeCount').textContent = state.nodes.size;
    document.getElementById('connectionCount').textContent = state.connections.length;
}

function updateZoomLevel() {
    const pct = Math.round(state.zoom * 100) + '%';
    document.getElementById('zoomLevel').textContent = pct;
    const zv = document.getElementById('zoomValue');
    if (zv) zv.textContent = pct;
}

function updateTitle() {
    const modified = state.modified ? ' •' : '';
    document.title = `${state.currentFlowName}${modified} - OpenVisionFlow Pro`;
    updateFlowName();
}

function updateFlowName() {
    const el = document.getElementById('flowName');
    if (el) el.textContent = state.currentFlowName + (state.modified ? ' •' : '');
}

function updateStatus(status, text) {
    const indicator = document.getElementById('statusIndicator');
    const statusText = document.getElementById('statusText');
    indicator.className = 'status-indicator ' + status;
    statusText.textContent = text;
}

function showProgress(text) {
    document.getElementById('progressOverlay').classList.add('active');
    document.getElementById('progressTitle').textContent = text;
    document.getElementById('progressFill').style.width = '0%';
    document.getElementById('progressPercent').textContent = '0%';
}

function updateProgress(percent, text) {
    document.getElementById('progressFill').style.width = percent + '%';
    document.getElementById('progressPercent').textContent = Math.round(percent) + '%';
    if (text) document.getElementById('progressDetail').textContent = text;
}

function hideProgress() {
    document.getElementById('progressOverlay').classList.remove('active');
}

function showError(message) {
    showToast(message, 'error');
    logMessage(message, 'error', 'errors');
}

// Toast 通知
function showToast(message, type = 'info', duration = 3000) {
    const container = document.getElementById('toastContainer');
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    const icons = {
        success: 'fa-check-circle',
        error: 'fa-exclamation-circle',
        warning: 'fa-exclamation-triangle',
        info: 'fa-info-circle'
    };
    toast.innerHTML = `
        <i class="fa ${icons[type] || icons.info} toast-icon"></i>
        <span class="toast-message">${escapeHtml(message)}</span>
        <i class="fa fa-times toast-close" onclick="this.parentElement.remove()"></i>
    `;
    container.appendChild(toast);
    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transform = 'translateX(100%)';
        setTimeout(() => toast.remove(), 300);
    }, duration);
}

// 模态对话框
function showModal(title, bodyHtml, footerHtml) {
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header">
            ${title}
            <span class="close-btn" onclick="hideModal()"><i class="fa fa-times"></i></span>
        </div>
        <div class="modal-body">${bodyHtml}</div>
        <div class="modal-footer">${footerHtml || `
            <button class="modal-btn primary" onclick="hideModal()">确定</button>
        `}</div>
    `;
    document.getElementById('modalOverlay').classList.add('active');
}

function hideModal() {
    document.getElementById('modalOverlay').classList.remove('active');
}

function showConfirmDialog(message, onConfirm) {
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header">
            <i class="fa fa-question-circle"></i> 确认
            <span class="close-btn" onclick="hideModal()"><i class="fa fa-times"></i></span>
        </div>
        <div class="modal-body">
            <p>${escapeHtml(message)}</p>
        </div>
        <div class="modal-footer">
            <button class="modal-btn secondary" onclick="hideModal()">取消</button>
            <button class="modal-btn primary" id="confirmBtn">确定</button>
        </div>
    `;
    document.getElementById('confirmBtn').onclick = () => {
        hideModal();
        onConfirm();
    };
    document.getElementById('modalOverlay').classList.add('active');
}

function showInputDialog(title, message, defaultValue, onSubmit) {
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header">
            <i class="fa fa-edit"></i> ${title}
            <span class="close-btn" onclick="hideModal()"><i class="fa fa-times"></i></span>
        </div>
        <div class="modal-body">
            <p style="margin-bottom: 8px;">${escapeHtml(message)}</p>
            <textarea class="param-textarea" id="dialogInput" rows="4" style="width: 100%;">${escapeHtml(defaultValue)}</textarea>
        </div>
        <div class="modal-footer">
            <button class="modal-btn secondary" onclick="hideModal()">取消</button>
            <button class="modal-btn primary" id="dialogSubmit">确定</button>
        </div>
    `;
    document.getElementById('dialogSubmit').onclick = () => {
        const value = document.getElementById('dialogInput').value;
        hideModal();
        onSubmit(value);
    };
    document.getElementById('modalOverlay').classList.add('active');
    setTimeout(() => document.getElementById('dialogInput').focus(), 100);
}

// 上下文菜单
function showContextMenu(x, y, context) {
    const menu = document.getElementById('contextMenu');
    const hasNode = !!context.node;
    const hasConn = !!context.connection;
    const hasSelection = state.selectedNodes.length > 0;

    let html = '';
    if (hasNode || hasSelection) {
        html += `
            <div class="context-menu-item" onclick="cmd('copyNodes')">
                <i class="fa fa-copy menu-icon"></i>
                <span class="menu-label">复制</span>
                <span class="shortcut">Ctrl+C</span>
            </div>
            <div class="context-menu-item" onclick="cmd('cutNodes')">
                <i class="fa fa-scissors menu-icon"></i>
                <span class="menu-label">剪切</span>
                <span class="shortcut">Ctrl+X</span>
            </div>
            <div class="context-menu-item" onclick="cmd('pasteNodes')">
                <i class="fa fa-paste menu-icon"></i>
                <span class="menu-label">粘贴</span>
                <span class="shortcut">Ctrl+V</span>
            </div>
            <div class="context-menu-item" onclick="cmd('duplicateSelected')">
                <i class="fa fa-clone menu-icon"></i>
                <span class="menu-label">克隆</span>
                <span class="shortcut">Ctrl+D</span>
            </div>
            <div class="context-menu-separator"></div>
            <div class="context-menu-item danger" onclick="cmd('deleteSelected')">
                <i class="fa fa-trash menu-icon"></i>
                <span class="menu-label">删除</span>
                <span class="shortcut">Del</span>
            </div>
            <div class="context-menu-separator"></div>
        `;
        if (hasNode) {
            html += `
                <div class="context-menu-item" onclick="toggleNodeEnabled('${context.node.id}')">
                    <i class="fa fa-power-off menu-icon"></i>
                    <span class="menu-label">${context.node.enabled === false ? '启用' : '禁用'}</span>
                </div>
                <div class="context-menu-item" onclick="toggleBreakpoint('${context.node.id}')">
                    <i class="fa fa-circle menu-icon" style="color: var(--accent-error)"></i>
                    <span class="menu-label">${state.breakpoints.has(context.node.id) ? '取消断点' : '设置断点'}</span>
                </div>
                <div class="context-menu-item" onclick="previewNodeOutput('${context.node.id}')">
                    <i class="fa fa-eye menu-icon"></i>
                    <span class="menu-label">查看输出</span>
                </div>
            `;
        }
        if (state.selectedNodes.length >= 2) {
            html += `
                <div class="context-menu-separator"></div>
                <div class="context-menu-item" onclick="cmd('groupSelected')">
                    <i class="fa fa-object-group menu-icon"></i>
                    <span class="menu-label">创建分组</span>
                    <span class="shortcut">Ctrl+G</span>
                </div>
            `;
        }
    } else if (hasConn) {
        html += `
            <div class="context-menu-item danger" onclick="state.removeConnection('${context.connection.id}'); renderCanvas(); updateStatusBar();">
                <i class="fa fa-trash menu-icon"></i>
                <span class="menu-label">删除连接</span>
            </div>
        `;
    } else {
        html += `
            <div class="context-menu-item" onclick="cmd('pasteNodes')">
                <i class="fa fa-paste menu-icon"></i>
                <span class="menu-label">粘贴</span>
                <span class="shortcut">Ctrl+V</span>
            </div>
            <div class="context-menu-item" onclick="cmd('selectAll')">
                <i class="fa fa-check-square-o menu-icon"></i>
                <span class="menu-label">全选</span>
                <span class="shortcut">Ctrl+A</span>
            </div>
            <div class="context-menu-separator"></div>
            <div class="context-menu-item" onclick="cmd('fitToScreen')">
                <i class="fa fa-expand menu-icon"></i>
                <span class="menu-label">适应屏幕</span>
            </div>
            <div class="context-menu-item" onclick="cmd('resetView')">
                <i class="fa fa-crosshairs menu-icon"></i>
                <span class="menu-label">重置视图</span>
            </div>
        `;
    }

    menu.innerHTML = html;
    menu.style.left = x + 'px';
    menu.style.top = y + 'px';
    menu.classList.add('active');

    // 边界检测
    setTimeout(() => {
        const rect = menu.getBoundingClientRect();
        if (rect.right > window.innerWidth) {
            menu.style.left = (x - rect.width) + 'px';
        }
        if (rect.bottom > window.innerHeight) {
            menu.style.top = (y - rect.height) + 'px';
        }
    }, 0);
}

function hideContextMenu() {
    document.getElementById('contextMenu').classList.remove('active');
}

// 快捷键提示
function showShortcuts() {
    const grid = document.getElementById('shortcutGrid');
    const shortcuts = [
        ['新建流程', 'Ctrl+N'],
        ['打开流程', 'Ctrl+O'],
        ['保存流程', 'Ctrl+S'],
        ['撤销', 'Ctrl+Z'],
        ['重做', 'Ctrl+Y / Ctrl+Shift+Z'],
        ['复制', 'Ctrl+C'],
        ['粘贴', 'Ctrl+V'],
        ['剪切', 'Ctrl+X'],
        ['克隆', 'Ctrl+D'],
        ['全选', 'Ctrl+A'],
        ['删除', 'Delete'],
        ['组合', 'Ctrl+G'],
        ['查找/适应', 'Ctrl+F'],
        ['重置视图', 'Ctrl+0'],
        ['放大', '+'],
        ['缩小', '-'],
        ['运行流程', 'F5'],
        ['停止流程', 'Shift+F5'],
        ['单步执行', 'F10'],
        ['单步跳过', 'F11'],
        ['取消选择', 'Esc'],
        ['方向键移动', '↑↓←→'],
        ['显示快捷键', '?']
    ];
    grid.innerHTML = shortcuts.map(([name, key]) =>
        `<div class="shortcut-item">
            <span class="shortcut-name">${name}</span>
            <span class="shortcut-key">${key}</span>
        </div>`
    ).join('');
    document.getElementById('shortcutOverlay').classList.add('active');
}

function hideShortcuts() {
    document.getElementById('shortcutOverlay').classList.remove('active');
}

// ==================== 渲染循环 ====================
let animationFrame = null;
function renderCanvas() {
    if (animationFrame) cancelAnimationFrame(animationFrame);
    animationFrame = requestAnimationFrame(() => {
        if (renderer) renderer.render();
    });
}

// ==================== 初始化 ====================
function init() {
    // 应用主题
    document.documentElement.setAttribute('data-theme', state.theme);
    const themeIcon = document.getElementById('themeIcon');
    if (themeIcon) {
        themeIcon.className = state.theme === 'dark' ? 'fa fa-moon-o btn-icon' : 'fa fa-sun-o btn-icon';
    }

    // 创建渲染器
    renderer = new CanvasRenderer();

    // 创建交互控制器
    interaction = new InteractionController();

    // 创建API客户端
    apiClient = new ApiClient();
    apiClient.connectWebSocket();

    // 渲染节点树
    renderNodeTree();
    renderFavorites();
    renderTemplates();

    // 节点搜索
    document.getElementById('nodeSearch').addEventListener('input', (e) => {
        renderNodeTree(e.target.value);
    });
    document.getElementById('globalSearch').addEventListener('input', (e) => {
        document.getElementById('nodeSearch').value = e.target.value;
        renderNodeTree(e.target.value);
    });

    // 面板切换
    document.querySelectorAll('.panel-tab').forEach(tab => {
        tab.addEventListener('click', () => {
            switchPanel(tab.dataset.pane);
        });
    });

    // 日志切换
    document.querySelectorAll('.bottom-tab').forEach(tab => {
        tab.addEventListener('click', () => {
            switchLog(tab.dataset.log);
        });
    });

    // 日志过滤
    document.getElementById('logFilter').addEventListener('input', (e) => {
        state.logFilter = e.target.value;
    });
    document.querySelectorAll('.filter-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            state.currentLogLevel = btn.dataset.level;
        });
    });
    document.getElementById('autoScroll').addEventListener('change', (e) => {
        state.autoScroll = e.target.checked;
    });

    // 关闭模态框点击外部
    document.getElementById('modalOverlay').addEventListener('click', (e) => {
        if (e.target.id === 'modalOverlay') hideModal();
    });

    // 关闭快捷键提示
    document.getElementById('shortcutOverlay').addEventListener('click', (e) => {
        if (e.target.id === 'shortcutOverlay') hideShortcuts();
    });

    // 关闭上下文菜单
    document.addEventListener('click', (e) => {
        if (!e.target.closest('.context-menu')) hideContextMenu();
    });

    // 小地图点击导航
    document.getElementById('minimapCanvas').addEventListener('click', (e) => {
        const rect = e.target.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const bounds = renderer.getNodesBounds();
        const padding = 60;
        const scaleX = 180 / (bounds.width + padding * 2);
        const scaleY = 120 / (bounds.height + padding * 2);
        const scale = Math.min(scaleX, scaleY, 0.15);
        // 计算对应的画布坐标
        const cx = (x / scale - padding + bounds.minX);
        const cy = (y / scale - padding + bounds.minY);
        const cw = renderer.canvas.width / renderer.dpr;
        const ch = renderer.canvas.height / renderer.dpr;
        state.panOffset.x = cw / 2 - cx * state.zoom;
        state.panOffset.y = ch / 2 - cy * state.zoom;
        renderCanvas();
    });

    // 文件拖入
    document.addEventListener('dragover', e => e.preventDefault());
    document.addEventListener('drop', e => {
        e.preventDefault();
        const file = e.dataTransfer.files[0];
        if (file && file.name.endsWith('.json')) {
            const reader = new FileReader();
            reader.onload = (ev) => {
                try {
                    const json = JSON.parse(ev.target.result);
                    state.fromJSON(json);
                    showToast('流程已导入', 'success');
                } catch (err) {
                    showToast('导入失败：JSON格式错误', 'error');
                }
            };
            reader.readAsText(file);
        }
    });

    // 初始渲染
    renderCanvas();
    updateStatusBar();
    updateZoomLevel();

    logMessage('OpenVisionFlow Editor Pro v3.0 初始化完成', 'info');
    logMessage('按 ? 查看快捷键帮助', 'info');
    logMessage('从左侧节点库拖拽节点到画布开始创建流程', 'info');

    // 自动保存定时器
    setInterval(() => {
        if (state.modified && state.currentFlowId) {
            saveFlow();
            logMessage('自动保存完成', 'debug');
        }
    }, CONFIG.AUTO_SAVE_INTERVAL);

    // 持续动画（用于流动粒子）
    function animate() {
        const hasActive = state.connections.some(c => c.active) ||
                         state.nodes.values().some(n => n.state === 'running');
        if (hasActive) {
            renderCanvas();
        }
        requestAnimationFrame(animate);
    }
    animate();
}

// 启动
window.addEventListener('load', init);

// 导出到全局
window.cmd = cmd;
window.state = state;
window.hideModal = hideModal;
window.updateNodeField = updateNodeField;
window.updateNodeParam = updateNodeParam;
window.toggleNodeEnabled = toggleNodeEnabled;
window.toggleBreakpoint = toggleBreakpoint;
window.updateBreakpointsList = updateBreakpointsList;
window.renderCanvas = renderCanvas;
window.previewNodeOutput = previewNodeOutput;

