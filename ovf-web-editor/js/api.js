/**
 * OpenVisionFlow API 封装
 * 提供与后端API交互的接口
 */

const OVF_API = {
    // 默认后端地址
    baseURL: 'http://localhost:8080',

    // API 端点
    endpoints: {
        // 节点相关
        nodes: '/api/nodes',
        nodeTypes: '/api/nodes/types',

        // 流程相关
        flows: '/api/flows',
        flowExecute: '/api/flows/execute',
        flowStep: '/api/flows/step',
        flowStop: '/api/flows/stop',

        // 文件相关
        files: '/api/files',
        fileLoad: '/api/files/load',
        fileSave: '/api/files/save'
    },

    /**
     * 设置基础URL
     */
    setBaseURL(url) {
        this.baseURL = url.replace(/\/$/, '');
    },

    /**
     * 通用请求方法
     */
    async request(endpoint, options = {}) {
        const url = this.baseURL + endpoint;
        const defaultOptions = {
            mode: 'cors',
            cache: 'no-cache',
            credentials: 'same-origin',
            headers: {
                'Content-Type': 'application/json'
            }
        };

        const mergedOptions = { ...defaultOptions, ...options };
        if (options.headers) {
            mergedOptions.headers = { ...defaultOptions.headers, ...options.headers };
        }

        try {
            const response = await fetch(url, mergedOptions);

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`HTTP ${response.status}: ${errorText}`);
            }

            // 检查是否有响应内容
            const contentType = response.headers.get('content-type');
            if (contentType && contentType.includes('application/json')) {
                return await response.json();
            }

            return await response.text();
        } catch (error) {
            console.error('API请求失败:', error);
            throw error;
        }
    },

    /**
     * GET请求
     */
    async get(endpoint, params = {}) {
        const url = new URL(this.baseURL + endpoint);
        Object.keys(params).forEach(key => {
            if (params[key] !== undefined && params[key] !== null) {
                url.searchParams.append(key, params[key]);
            }
        });

        return this.request(endpoint + url.search, { method: 'GET' });
    },

    /**
     * POST请求
     */
    async post(endpoint, data = {}) {
        return this.request(endpoint, {
            method: 'POST',
            body: JSON.stringify(data)
        });
    },

    /**
     * PUT请求
     */
    async put(endpoint, data = {}) {
        return this.request(endpoint, {
            method: 'PUT',
            body: JSON.stringify(data)
        });
    },

    /**
     * DELETE请求
     */
    async delete(endpoint) {
        return this.request(endpoint, { method: 'DELETE' });
    },

    // ==================== 节点相关API ====================

    /**
     * 获取所有节点类型
     */
    async getNodeTypes() {
        try {
            const result = await this.get(this.endpoints.nodeTypes);
            return result;
        } catch (error) {
            console.warn('获取节点类型失败，使用默认类型:', error);
            return this.getDefaultNodeTypes();
        }
    },

    /**
     * 获取默认节点类型（离线使用）
     */
    getDefaultNodeTypes() {
        return [
            {
                type: 'input',
                name: '输入节点',
                category: '数据',
                description: '从文件或摄像头获取图像输入',
                icon: '📷',
                color: '#3b82f6',
                inputs: [],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: [
                    { name: 'source', type: 'select', label: '输入源', options: ['file', 'camera'], default: 'file' },
                    { name: 'path', type: 'string', label: '文件路径', default: '' }
                ]
            },
            {
                type: 'output',
                name: '输出节点',
                category: '数据',
                description: '输出处理结果',
                icon: '💾',
                color: '#10b981',
                inputs: [{ name: 'data', type: 'any', label: '输入数据' }],
                outputs: [],
                params: [
                    { name: 'type', type: 'select', label: '输出类型', options: ['display', 'save', 'both'], default: 'display' },
                    { name: 'path', type: 'string', label: '保存路径', default: '' }
                ]
            },
            {
                type: 'grayscale',
                name: '灰度转换',
                category: '图像处理',
                description: '将图像转换为灰度图',
                icon: '🎨',
                color: '#6366f1',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: []
            },
            {
                type: 'blur',
                name: '图像模糊',
                category: '图像处理',
                description: '对图像进行模糊处理',
                icon: '💨',
                color: '#8b5cf6',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: [
                    { name: 'kernelSize', type: 'number', label: '核大小', default: 5, min: 1, max: 31 },
                    { name: 'sigma', type: 'number', label: 'Sigma', default: 1.0, min: 0.1, max: 10 }
                ]
            },
            {
                type: 'threshold',
                name: '阈值处理',
                category: '图像处理',
                description: '对图像进行阈值处理',
                icon: '⬛',
                color: '#ec4899',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: [
                    { name: 'threshold', type: 'number', label: '阈值', default: 127, min: 0, max: 255 },
                    { name: 'type', type: 'select', label: '类型', options: ['binary', 'binary_inv', 'trunc', 'tozero', 'tozero_inv'], default: 'binary' }
                ]
            },
            {
                type: 'canny',
                name: '边缘检测',
                category: '特征检测',
                description: 'Canny边缘检测算法',
                icon: '🔍',
                color: '#f59e0b',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'edges', type: 'image', label: '边缘图像' }],
                params: [
                    { name: 'threshold1', type: 'number', label: '低阈值', default: 50, min: 0, max: 255 },
                    { name: 'threshold2', type: 'number', label: '高阈值', default: 150, min: 0, max: 255 }
                ]
            },
            {
                type: 'contours',
                name: '轮廓检测',
                category: '特征检测',
                description: '检测图像中的轮廓',
                icon: '📐',
                color: '#14b8a6',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'contours', type: 'contours', label: '轮廓数据' }],
                params: [
                    { name: 'mode', type: 'select', label: '检索模式', options: ['external', 'list', 'ccomp', 'tree'], default: 'external' },
                    { name: 'method', type: 'select', label: '近似方法', options: ['none', 'simple', 'tc89'], default: 'simple' }
                ]
            },
            {
                type: 'filter',
                name: '自定义滤波',
                category: '图像处理',
                description: '使用自定义卷积核进行滤波',
                icon: '🔲',
                color: '#f97316',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: [
                    { name: 'kernel', type: 'text', label: '卷积核(JSON)', default: '[[0,-1,0],[-1,5,-1],[0,-1,0]]' }
                ]
            },
            {
                type: 'resize',
                name: '图像缩放',
                category: '图像处理',
                description: '调整图像大小',
                icon: '↔',
                color: '#06b6d4',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: [
                    { name: 'width', type: 'number', label: '宽度', default: 800, min: 1 },
                    { name: 'height', type: 'number', label: '高度', default: 600, min: 1 },
                    { name: 'interpolation', type: 'select', label: '插值方法', options: ['nearest', 'linear', 'cubic', 'area', 'lanczos'], default: 'linear' }
                ]
            },
            {
                type: 'rotate',
                name: '图像旋转',
                category: '图像处理',
                description: '旋转图像',
                icon: '🔄',
                color: '#a855f7',
                inputs: [{ name: 'image', type: 'image', label: '输入图像' }],
                outputs: [{ name: 'image', type: 'image', label: '输出图像' }],
                params: [
                    { name: 'angle', type: 'number', label: '旋转角度', default: 0, min: -360, max: 360 },
                    { name: 'scale', type: 'number', label: '缩放比例', default: 1.0, min: 0.1, max: 10 }
                ]
            },
            {
                type: 'merge',
                name: '数据合并',
                category: '流程控制',
                description: '合并多个输入数据',
                icon: '🔀',
                color: '#64748b',
                inputs: [
                    { name: 'input1', type: 'any', label: '输入1' },
                    { name: 'input2', type: 'any', label: '输入2' }
                ],
                outputs: [{ name: 'output', type: 'array', label: '合并输出' }],
                params: []
            },
            {
                type: 'script',
                name: '自定义脚本',
                category: '高级',
                description: '执行自定义Python脚本',
                icon: '📜',
                color: '#ef4444',
                inputs: [{ name: 'input', type: 'any', label: '输入数据' }],
                outputs: [{ name: 'output', type: 'any', label: '输出数据' }],
                params: [
                    { name: 'script', type: 'textarea', label: 'Python脚本', default: '# 在此编写Python代码\nresult = input_data' }
                ]
            }
        ];
    },

    /**
     * 创建节点实例
     */
    async createNode(nodeType, x, y) {
        return {
            id: this.generateId(),
            type: nodeType.type,
            name: nodeType.name,
            x: x,
            y: y,
            width: 200,
            height: 100,
            inputs: nodeType.inputs ? [...nodeType.inputs] : [],
            outputs: nodeType.outputs ? [...nodeType.outputs] : [],
            params: {},
            status: 'idle', // idle, running, success, error, disabled
            icon: nodeType.icon,
            color: nodeType.color
        };
    },

    // ==================== 流程相关API ====================

    /**
     * 保存流程
     */
    async saveFlow(flowData) {
        try {
            const result = await this.post(this.endpoints.fileSave, {
                filename: flowData.name || 'untitled.json',
                content: JSON.stringify(flowData, null, 2)
            });
            return result;
        } catch (error) {
            console.warn('保存到后端失败，使用本地存储:', error);
            // 使用本地存储
            localStorage.setItem('ovf_flow_' + (flowData.name || 'untitled'), JSON.stringify(flowData));
            return { success: true, message: '已保存到本地存储' };
        }
    },

    /**
     * 加载流程
     */
    async loadFlow(filename) {
        try {
            const result = await this.get(this.endpoints.fileLoad, { filename });
            return typeof result === 'string' ? JSON.parse(result) : result;
        } catch (error) {
            console.warn('从后端加载失败，尝试本地存储:', error);
            const localData = localStorage.getItem('ovf_flow_' + filename);
            if (localData) {
                return JSON.parse(localData);
            }
            throw error;
        }
    },

    /**
     * 执行流程
     */
    async executeFlow(flowData) {
        return await this.post(this.endpoints.flowExecute, flowData);
    },

    /**
     * 单步执行
     */
    async stepFlow(flowData, nodeId) {
        return await this.post(this.endpoints.flowStep, {
            flow: flowData,
            nodeId: nodeId
        });
    },

    /**
     * 停止执行
     */
    async stopFlow() {
        return await this.post(this.endpoints.flowStop);
    },

    // ==================== 工具方法 ====================

    /**
     * 生成唯一ID
     */
    generateId() {
        return 'node_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
    },

    /**
     * 导出流程为JSON文件
     */
    exportFlowJSON(flowData, filename = 'flow.json') {
        const dataStr = JSON.stringify(flowData, null, 2);
        const blob = new Blob([dataStr], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    },

    /**
     * 从JSON文件导入流程
     */
    importFlowJSON(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    const data = JSON.parse(e.target.result);
                    resolve(data);
                } catch (error) {
                    reject(new Error('无效的JSON文件'));
                }
            };
            reader.onerror = () => reject(new Error('文件读取失败'));
            reader.readAsText(file);
        });
    },

    /**
     * 检查后端连接状态
     */
    async checkConnection() {
        try {
            await this.get('/api/health');
            return true;
        } catch (error) {
            return false;
        }
    }
};

// 导出API对象
window.OVF_API = OVF_API;