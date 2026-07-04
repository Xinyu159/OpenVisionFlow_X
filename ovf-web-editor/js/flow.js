/**
 * OpenVisionFlow 流程管理
 * 管理流程节点、连接和数据流
 */

const OVF_Flow = {
    // 流程数据
    flowData: {
        id: null,
        name: 'untitled',
        nodes: [],
        connections: [],
        metadata: {
            created: new Date().toISOString(),
            modified: new Date().toISOString(),
            version: '1.0'
        }
    },

    // 当前选中的节点
    selectedNodes: [],

    // 当前选中的连接
    selectedConnection: null,

    // 节点类型缓存
    nodeTypes: [],

    /**
     * 初始化流程管理器
     */
    async init() {
        // 加载节点类型
        this.nodeTypes = await OVF_API.getNodeTypes();
        console.log('已加载节点类型:', this.nodeTypes.length);

        // 创建新流程
        this.newFlow();
    },

    /**
     * 创建新流程
     */
    newFlow() {
        this.flowData = {
            id: OVF_API.generateId(),
            name: 'untitled',
            nodes: [],
            connections: [],
            metadata: {
                created: new Date().toISOString(),
                modified: new Date().toISOString(),
                version: '1.0'
            }
        };
        this.selectedNodes = [];
        this.selectedConnection = null;
    },

    /**
     * 添加节点到流程
     */
    addNode(nodeType, x, y) {
        const node = {
            id: OVF_API.generateId(),
            type: nodeType.type,
            name: nodeType.name,
            x: x,
            y: y,
            width: 200,
            height: this.calculateNodeHeight(nodeType),
            inputs: nodeType.inputs ? nodeType.inputs.map((inp, idx) => ({
                ...inp,
                id: `${node.id}_input_${idx}`,
                connected: false
            })) : [],
            outputs: nodeType.outputs ? nodeType.outputs.map((out, idx) => ({
                ...out,
                id: `${node.id}_output_${idx}`,
                connected: false
            })) : [],
            params: this.getDefaultParams(nodeType),
            status: 'idle',
            icon: nodeType.icon,
            color: nodeType.color,
            disabled: false
        };

        this.flowData.nodes.push(node);
        this.flowData.metadata.modified = new Date().toISOString();

        return node;
    },

    /**
     * 计算节点高度
     */
    calculateNodeHeight(nodeType) {
        const inputCount = nodeType.inputs ? nodeType.inputs.length : 0;
        const outputCount = nodeType.outputs ? nodeType.outputs.length : 0;
        const maxPorts = Math.max(inputCount, outputCount, 1);
        return 80 + maxPorts * 24; // 基础高度 + 端口高度
    },

    /**
     * 获取默认参数值
     */
    getDefaultParams(nodeType) {
        const params = {};
        if (nodeType.params) {
            nodeType.params.forEach(param => {
                params[param.name] = param.default;
            });
        }
        return params;
    },

    /**
     * 删除节点
     */
    deleteNode(nodeId) {
        // 删除相关的连接
        this.flowData.connections = this.flowData.connections.filter(conn => {
            return conn.fromNode !== nodeId && conn.toNode !== nodeId;
        });

        // 删除节点
        this.flowData.nodes = this.flowData.nodes.filter(node => node.id !== nodeId);

        // 从选中列表移除
        this.selectedNodes = this.selectedNodes.filter(id => id !== nodeId);

        this.flowData.metadata.modified = new Date().toISOString();
    },

    /**
     * 复制节点
     */
    copyNode(nodeId) {
        const originalNode = this.getNodeById(nodeId);
        if (!originalNode) return null;

        const newNode = JSON.parse(JSON.stringify(originalNode));
        newNode.id = OVF_API.generateId();
        newNode.x += 50; // 偏移位置
        newNode.y += 50;
        newNode.status = 'idle';

        // 重新生成端口ID
        newNode.inputs = newNode.inputs.map((inp, idx) => ({
            ...inp,
            id: `${newNode.id}_input_${idx}`,
            connected: false
        }));
        newNode.outputs = newNode.outputs.map((out, idx) => ({
            ...out,
            id: `${newNode.id}_output_${idx}`,
            connected: false
        }));

        this.flowData.nodes.push(newNode);
        this.flowData.metadata.modified = new Date().toISOString();

        return newNode;
    },

    /**
     * 禁用/启用节点
     */
    toggleNodeDisabled(nodeId) {
        const node = this.getNodeById(nodeId);
        if (node) {
            node.disabled = !node.disabled;
            node.status = node.disabled ? 'disabled' : 'idle';
            this.flowData.metadata.modified = new Date().toISOString();
        }
    },

    /**
     * 更新节点位置
     */
    updateNodePosition(nodeId, x, y) {
        const node = this.getNodeById(nodeId);
        if (node) {
            node.x = x;
            node.y = y;
            this.flowData.metadata.modified = new Date().toISOString();
        }
    },

    /**
     * 更新节点参数
     */
    updateNodeParams(nodeId, params) {
        const node = this.getNodeById(nodeId);
        if (node) {
            node.params = { ...node.params, ...params };
            this.flowData.metadata.modified = new Date().toISOString();
        }
    },

    /**
     * 添加连接
     */
    addConnection(fromNodeId, fromPortIndex, toNodeId, toPortIndex) {
        // 检查连接是否已存在
        const existing = this.flowData.connections.find(conn =>
            conn.fromNode === fromNodeId &&
            conn.fromPort === fromPortIndex &&
            conn.toNode === toNodeId &&
            conn.toPort === toPortIndex
        );

        if (existing) return existing;

        // 检查输入端口是否已连接（一个输入只能连接一个输出）
        const inputConnected = this.flowData.connections.find(conn =>
            conn.toNode === toNodeId &&
            conn.toPort === toPortIndex
        );

        if (inputConnected) {
            // 删除旧连接
            this.deleteConnection(inputConnected.id);
        }

        const connection = {
            id: OVF_API.generateId(),
            fromNode: fromNodeId,
            fromPort: fromPortIndex,
            toNode: toNodeId,
            toPort: toPortIndex
        };

        this.flowData.connections.push(connection);

        // 更新端口连接状态
        const fromNode = this.getNodeById(fromNodeId);
        const toNode = this.getNodeById(toNodeId);

        if (fromNode && fromNode.outputs[fromPortIndex]) {
            fromNode.outputs[fromPortIndex].connected = true;
        }
        if (toNode && toNode.inputs[toPortIndex]) {
            toNode.inputs[toPortIndex].connected = true;
        }

        this.flowData.metadata.modified = new Date().toISOString();

        return connection;
    },

    /**
     * 删除连接
     */
    deleteConnection(connectionId) {
        const connection = this.getConnectionById(connectionId);
        if (!connection) return;

        // 更新端口连接状态
        const fromNode = this.getNodeById(connection.fromNode);
        const toNode = this.getNodeById(connection.toNode);

        if (fromNode && fromNode.outputs[connection.fromPort]) {
            // 检查是否还有其他连接使用此端口
            const otherConnections = this.flowData.connections.filter(conn =>
                conn.fromNode === connection.fromNode &&
                conn.fromPort === connection.fromPort &&
                conn.id !== connectionId
            );
            fromNode.outputs[connection.fromPort].connected = otherConnections.length > 0;
        }

        if (toNode && toNode.inputs[connection.toPort]) {
            toNode.inputs[connection.toPort].connected = false;
        }

        this.flowData.connections = this.flowData.connections.filter(conn => conn.id !== connectionId);

        if (this.selectedConnection === connectionId) {
            this.selectedConnection = null;
        }

        this.flowData.metadata.modified = new Date().toISOString();
    },

    /**
     * 根据ID获取节点
     */
    getNodeById(nodeId) {
        return this.flowData.nodes.find(node => node.id === nodeId);
    },

    /**
     * 根据ID获取连接
     */
    getConnectionById(connectionId) {
        return this.flowData.connections.find(conn => conn.id === connectionId);
    },

    /**
     * 获取节点类型定义
     */
    getNodeTypeByType(type) {
        return this.nodeTypes.find(nt => nt.type === type);
    },

    /**
     * 选中节点
     */
    selectNode(nodeId, addToSelection = false) {
        if (addToSelection) {
            if (!this.selectedNodes.includes(nodeId)) {
                this.selectedNodes.push(nodeId);
            }
        } else {
            this.selectedNodes = [nodeId];
        }
    },

    /**
     * 取消选中节点
     */
    deselectNode(nodeId) {
        this.selectedNodes = this.selectedNodes.filter(id => id !== nodeId);
    },

    /**
     * 清除所有选中
     */
    clearSelection() {
        this.selectedNodes = [];
        this.selectedConnection = null;
    },

    /**
     * 判断节点是否选中
     */
    isNodeSelected(nodeId) {
        return this.selectedNodes.includes(nodeId);
    },

    /**
     * 选中连接
     */
    selectConnection(connectionId) {
        this.selectedConnection = connectionId;
        this.selectedNodes = [];
    },

    /**
     * 获取选中的节点
     */
    getSelectedNodes() {
        return this.selectedNodes.map(id => this.getNodeById(id)).filter(n => n);
    },

    /**
     * 执行流程
     */
    async executeFlow() {
        try {
            // 重置所有节点状态
            this.flowData.nodes.forEach(node => {
                if (!node.disabled) {
                    node.status = 'idle';
                }
            });

            const result = await OVF_API.executeFlow(this.flowData);

            // 更新节点状态
            if (result.nodeStates) {
                result.nodeStates.forEach(state => {
                    const node = this.getNodeById(state.nodeId);
                    if (node) {
                        node.status = state.status;
                    }
                });
            }

            return result;
        } catch (error) {
            console.error('流程执行失败:', error);
            throw error;
        }
    },

    /**
     * 单步执行
     */
    async stepFlow(nodeId) {
        try {
            // 设置节点为运行状态
            const node = this.getNodeById(nodeId);
            if (node) {
                node.status = 'running';
            }

            const result = await OVF_API.stepFlow(this.flowData, nodeId);

            // 更新节点状态
            if (result.nodeStates) {
                result.nodeStates.forEach(state => {
                    const n = this.getNodeById(state.nodeId);
                    if (n) {
                        n.status = state.status;
                    }
                });
            }

            return result;
        } catch (error) {
            console.error('单步执行失败:', error);
            const node = this.getNodeById(nodeId);
            if (node) {
                node.status = 'error';
            }
            throw error;
        }
    },

    /**
     * 停止执行
     */
    async stopFlow() {
        try {
            await OVF_API.stopFlow();

            // 重置所有节点状态
            this.flowData.nodes.forEach(node => {
                if (node.status === 'running') {
                    node.status = 'idle';
                }
            });
        } catch (error) {
            console.error('停止执行失败:', error);
            throw error;
        }
    },

    /**
     * 保存流程
     */
    async saveFlow() {
        await OVF_API.saveFlow(this.flowData);
    },

    /**
     * 加载流程
     */
    async loadFlow(filename) {
        const data = await OVF_API.loadFlow(filename);
        this.flowData = data;
        this.selectedNodes = [];
        this.selectedConnection = null;
    },

    /**
     * 导出流程数据
     */
    exportFlow() {
        return JSON.parse(JSON.stringify(this.flowData));
    },

    /**
     * 导入流程数据
     */
    importFlow(data) {
        this.flowData = data;
        this.selectedNodes = [];
        this.selectedConnection = null;
    },

    /**
     * 获取连接的端口位置
     */
    getPortPosition(nodeId, portType, portIndex) {
        const node = this.getNodeById(nodeId);
        if (!node) return null;

        const ports = portType === 'input' ? node.inputs : node.outputs;
        const port = ports[portIndex];

        if (!port) return null;

        const nodeElement = document.querySelector(`[data-node-id="${nodeId}"]`);
        if (!nodeElement) return null;

        const portElement = nodeElement.querySelector(`[data-port-type="${portType}"][data-port-index="${portIndex}"]`);
        if (!portElement) return null;

        const nodeRect = nodeElement.getBoundingClientRect();
        const portRect = portElement.getBoundingClientRect();
        const canvasWrapper = document.getElementById('canvas-wrapper');
        const canvasRect = canvasWrapper.getBoundingClientRect();

        return {
            x: portRect.left - canvasRect.left + portRect.width / 2,
            y: portRect.top - canvasRect.top + portRect.height / 2
        };
    },

    /**
     * 验证流程是否完整
     */
    validateFlow() {
        const errors = [];

        // 检查是否有未连接的必需输入端口
        this.flowData.nodes.forEach(node => {
            if (!node.disabled) {
                node.inputs.forEach((input, idx) => {
                    const connected = this.flowData.connections.some(conn =>
                        conn.toNode === node.id && conn.toPort === idx
                    );

                    if (!connected && input.required !== false) {
                        errors.push({
                            nodeId: node.id,
                            message: `节点"${node.name}"的输入端口"${input.label}"未连接`
                        });
                    }
                });
            }
        });

        return {
            valid: errors.length === 0,
            errors: errors
        };
    },

    /**
     * 获取流程统计信息
     */
    getFlowStats() {
        return {
            nodeCount: this.flowData.nodes.length,
            connectionCount: this.flowData.connections.length,
            enabledCount: this.flowData.nodes.filter(n => !n.disabled).length,
            disabledCount: this.flowData.nodes.filter(n => n.disabled).length
        };
    }
};

// 导出流程管理器
window.OVF_Flow = OVF_Flow;