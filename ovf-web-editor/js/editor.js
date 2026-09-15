/**
 * OpenVisionFlow 编辑器核心逻辑
 * 实现画布交互、节点拖拽、连线等功能
 */

const OVF_Editor = {
    // DOM 元素引用
    elements: {
        canvas: null,
        canvasWrapper: null,
        connectionSVG: null,
        nodeList: null,
        propertyPanel: null,
        propertyContent: null,
        contextMenu: null,
        nodeDialog: null,
        loadingOverlay: null,
        zoomLevel: null
    },

    // 状态变量
    state: {
        zoom: 1,
        offsetX: 0,
        offsetY: 0,
        isDragging: false,
        dragTarget: null,
        dragStartX: 0,
        dragStartY: 0,
        connecting: false,
        connectionStart: null,
        selectedNode: null,
        isPanning: false,
        panStartX: 0,
        panStartY: 0,
        lastMouseX: 0,
        lastMouseY: 0
    },

    // 执行状态
    executionState: {
        isRunning: false,
        currentNodeId: null,
        startTime: null
    },

    /**
     * 初始化编辑器
     */
    async init() {
        console.log('初始化 OpenVisionFlow 编辑器...');

        // 获取DOM元素
        this.initElements();

        // 初始化流程管理器
        await OVF_Flow.init();

        // 渲染节点类型列表
        this.renderNodeTypes();

        // 初始化画布
        this.initCanvas();

        // 绑定事件
        this.bindEvents();

        // 更新状态栏
        this.updateStatus('就绪', 'idle');
        this.updateTime();

        console.log('编辑器初始化完成');
    },

    /**
     * 初始化DOM元素引用
     */
    initElements() {
        this.elements.canvas = document.getElementById('flow-canvas');
        this.elements.canvasWrapper = document.getElementById('canvas-wrapper');
        this.elements.connectionSVG = document.getElementById('connection-svg');
        this.elements.nodeList = document.getElementById('node-list');
        this.elements.propertyContent = document.getElementById('property-content');
        this.elements.contextMenu = document.getElementById('context-menu');
        this.elements.nodeDialog = document.getElementById('node-dialog');
        this.elements.loadingOverlay = document.getElementById('loading-overlay');
        this.elements.zoomLevel = document.getElementById('zoom-level');
    },

    /**
     * 初始化画布
     */
    initCanvas() {
        const wrapper = this.elements.canvasWrapper;
        const canvas = this.elements.canvas;

        // 设置画布尺寸
        canvas.width = wrapper.clientWidth;
        canvas.height = wrapper.clientHeight;

        // 设置SVG尺寸
        this.elements.connectionSVG.setAttribute('width', wrapper.clientWidth);
        this.elements.connectionSVG.setAttribute('height', wrapper.clientHeight);

        // 绘制初始状态
        this.render();
    },

    /**
     * 渲染节点类型列表
     */
    renderNodeTypes() {
        const nodeTypes = OVF_Flow.nodeTypes;
        const nodeList = this.elements.nodeList;

        // 按类别分组
        const categories = {};
        nodeTypes.forEach(type => {
            if (!categories[type.category]) {
                categories[type.category] = [];
            }
            categories[type.category].push(type);
        });

        // 渲染
        nodeList.innerHTML = '';
        Object.keys(categories).forEach(category => {
            const categoryDiv = document.createElement('div');
            categoryDiv.className = 'node-category';

            const categoryTitle = document.createElement('div');
            categoryTitle.className = 'node-category-title';
            categoryTitle.textContent = category;
            categoryDiv.appendChild(categoryTitle);

            categories[category].forEach(nodeType => {
                const nodeItem = document.createElement('div');
                nodeItem.className = 'node-item';
                nodeItem.draggable = true;
                nodeItem.dataset.nodeType = nodeType.type;

                const icon = document.createElement('div');
                icon.className = 'node-item-icon';
                icon.style.backgroundColor = nodeType.color;
                icon.textContent = nodeType.icon;

                const info = document.createElement('div');
                info.className = 'node-item-info';

                const name = document.createElement('div');
                name.className = 'node-item-name';
                name.textContent = nodeType.name;

                const desc = document.createElement('div');
                desc.className = 'node-item-desc';
                desc.textContent = nodeType.description;

                info.appendChild(name);
                info.appendChild(desc);

                nodeItem.appendChild(icon);
                nodeItem.appendChild(info);

                categoryDiv.appendChild(nodeItem);
            });

            nodeList.appendChild(categoryDiv);
        });
    },

    /**
     * 绑定事件
     */
    bindEvents() {
        // 节点拖拽
        this.bindNodeDragEvents();

        // 画布事件
        this.bindCanvasEvents();

        // 菜单按钮事件
        this.bindMenuEvents();

        // 工具栏事件
        this.bindToolbarEvents();

        // 右键菜单事件
        this.bindContextMenuEvents();

        // 对话框事件
        this.bindDialogEvents();

        // 搜索事件
        this.bindSearchEvents();

        // 窗口大小调整
        window.addEventListener('resize', () => this.handleResize());

        // 键盘事件
        document.addEventListener('keydown', (e) => this.handleKeyDown(e));
    },

    /**
     * 绑定节点拖拽事件
     */
    bindNodeDragEvents() {
        const nodeList = this.elements.nodeList;

        nodeList.addEventListener('dragstart', (e) => {
            if (e.target.classList.contains('node-item')) {
                const nodeType = e.target.dataset.nodeType;
                e.dataTransfer.setData('nodeType', nodeType);
                e.dataTransfer.effectAllowed = 'copy';

                // 创建拖拽预览
                const preview = e.target.cloneNode(true);
                preview.style.position = 'fixed';
                preview.style.width = '200px';
                preview.style.opacity = '0.8';
                preview.style.pointerEvents = 'none';
                document.body.appendChild(preview);
                e.dataTransfer.setDragImage(preview, 100, 30);

                setTimeout(() => document.body.removeChild(preview), 0);
            }
        });

        nodeList.addEventListener('dragend', (e) => {
            // 清理
        });
    },

    /**
     * 绑定画布事件
     */
    bindCanvasEvents() {
        const wrapper = this.elements.canvasWrapper;

        // 拖放节点到画布
        wrapper.addEventListener('dragover', (e) => {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'copy';
        });

        wrapper.addEventListener('drop', (e) => {
            e.preventDefault();
            const nodeTypeStr = e.dataTransfer.getData('nodeType');
            if (nodeTypeStr) {
                const nodeType = OVF_Flow.getNodeTypeByType(nodeTypeStr);
                if (nodeType) {
                    const rect = wrapper.getBoundingClientRect();
                    const x = (e.clientX - rect.left) / this.state.zoom - this.state.offsetX;
                    const y = (e.clientY - rect.top) / this.state.zoom - this.state.offsetY;

                    const node = OVF_Flow.addNode(nodeType, x, y);
                    this.render();
                    this.selectNode(node.id);
                    this.showMessage(`已添加节点: ${node.name}`);
                }
            }
        });

        // 画布鼠标事件
        wrapper.addEventListener('mousedown', (e) => this.handleCanvasMouseDown(e));
        wrapper.addEventListener('mousemove', (e) => this.handleCanvasMouseMove(e));
        wrapper.addEventListener('mouseup', (e) => this.handleCanvasMouseUp(e));
        wrapper.addEventListener('mouseleave', (e) => this.handleCanvasMouseLeave(e));

        // 右键菜单
        wrapper.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            this.showContextMenu(e.clientX, e.clientY, null);
        });

        // 双击节点
        wrapper.addEventListener('dblclick', (e) => {
            const nodeElement = e.target.closest('.canvas-node');
            if (nodeElement) {
                const nodeId = nodeElement.dataset.nodeId;
                this.openNodeDialog(nodeId);
            }
        });

        // 点击空白区域
        wrapper.addEventListener('click', (e) => {
            if (e.target === wrapper || e.target.tagName === 'CANVAS') {
                OVF_Flow.clearSelection();
                this.render();
                this.updatePropertyPanel(null);
            }
        });
    },

    /**
     * 处理画布鼠标按下
     */
    handleCanvasMouseDown(e) {
        const wrapper = this.elements.canvasWrapper;
        const rect = wrapper.getBoundingClientRect();

        // 中键或空格键+左键拖拽画布
        if (e.button === 1 || (e.button === 0 && e.shiftKey)) {
            this.state.isPanning = true;
            this.state.panStartX = e.clientX - rect.left - this.state.offsetX * this.state.zoom;
            this.state.panStartY = e.clientY - rect.top - this.state.offsetY * this.state.zoom;
            wrapper.style.cursor = 'grabbing';
            return;
        }

        // 检查是否点击了节点
        const nodeElement = e.target.closest('.canvas-node');
        if (nodeElement) {
            const nodeId = nodeElement.dataset.nodeId;

            // 检查是否点击了端口
            const portElement = e.target.closest('.port');
            if (portElement) {
                const portType = portElement.dataset.portType;
                const portIndex = parseInt(portElement.dataset.portIndex);

                if (portType === 'output') {
                    // 开始连线
                    this.state.connecting = true;
                    this.state.connectionStart = {
                        nodeId: nodeId,
                        portType: portType,
                        portIndex: portIndex,
                        x: e.clientX - rect.left,
                        y: e.clientY - rect.top
                    };
                }
                return;
            }

            // 拖拽节点
            this.state.isDragging = true;
            this.state.dragTarget = nodeId;
            this.state.dragStartX = e.clientX;
            this.state.dragStartY = e.clientY;

            // 选中节点
            this.selectNode(nodeId, e.ctrlKey);
            wrapper.style.cursor = 'move';
            return;
        }

        // 检查是否点击了连线
        const connectionElement = e.target.closest('.connection-line');
        if (connectionElement) {
            const connectionId = connectionElement.dataset.connectionId;
            if (connectionId) {
                OVF_Flow.selectConnection(connectionId);
                this.render();
            }
            return;
        }
    },

    /**
     * 处理画布鼠标移动
     */
    handleCanvasMouseMove(e) {
        const wrapper = this.elements.canvasWrapper;
        const rect = wrapper.getBoundingClientRect();

        // 更新鼠标位置显示
        const x = Math.round((e.clientX - rect.left) / this.state.zoom - this.state.offsetX);
        const y = Math.round((e.clientY - rect.top) / this.state.zoom - this.state.offsetY);
        this.updatePosition(x, y);

        // 拖拽画布
        if (this.state.isPanning) {
            this.state.offsetX = (e.clientX - rect.left - this.state.panStartX) / this.state.zoom;
            this.state.offsetY = (e.clientY - rect.top - this.state.panStartY) / this.state.zoom;
            this.render();
            return;
        }

        // 拖拽节点
        if (this.state.isDragging && this.state.dragTarget) {
            const dx = (e.clientX - this.state.dragStartX) / this.state.zoom;
            const dy = (e.clientY - this.state.dragStartY) / this.state.zoom;

            const node = OVF_Flow.getNodeById(this.state.dragTarget);
            if (node) {
                node.x += dx;
                node.y += dy;
                this.state.dragStartX = e.clientX;
                this.state.dragStartY = e.clientY;
                this.render();
            }
            return;
        }

        // 连线过程
        if (this.state.connecting) {
            this.drawTemporaryConnection(
                this.state.connectionStart.x,
                this.state.connectionStart.y,
                e.clientX - rect.left,
                e.clientY - rect.top
            );
        }
    },

    /**
     * 处理画布鼠标释放
     */
    handleCanvasMouseUp(e) {
        const wrapper = this.elements.canvasWrapper;
        wrapper.style.cursor = 'default';

        // 结束拖拽画布
        if (this.state.isPanning) {
            this.state.isPanning = false;
            return;
        }

        // 结束节点拖拽
        if (this.state.isDragging) {
            this.state.isDragging = false;
            this.state.dragTarget = null;
            return;
        }

        // 结束连线
        if (this.state.connecting) {
            const portElement = e.target.closest('.port');
            if (portElement) {
                const portType = portElement.dataset.portType;
                const portIndex = parseInt(portElement.dataset.portIndex);
                const nodeElement = portElement.closest('.canvas-node');
                const nodeId = nodeElement.dataset.nodeId;

                if (portType === 'input') {
                    // 创建连接
                    OVF_Flow.addConnection(
                        this.state.connectionStart.nodeId,
                        this.state.connectionStart.portIndex,
                        nodeId,
                        portIndex
                    );
                    this.showMessage('已创建连接');
                }
            }

            // 清除临时连线
            this.clearTemporaryConnection();
            this.state.connecting = false;
            this.state.connectionStart = null;
            this.render();
        }
    },

    /**
     * 处理画布鼠标离开
     */
    handleCanvasMouseLeave(e) {
        if (this.state.connecting) {
            this.clearTemporaryConnection();
            this.state.connecting = false;
            this.state.connectionStart = null;
        }

        if (this.state.isDragging) {
            this.state.isDragging = false;
            this.state.dragTarget = null;
        }

        if (this.state.isPanning) {
            this.state.isPanning = false;
        }

        this.elements.canvasWrapper.style.cursor = 'default';
    },

    /**
     * 绑定菜单事件
     */
    bindMenuEvents() {
        // 新建
        document.getElementById('btn-new').addEventListener('click', () => {
            if (confirm('是否创建新流程？未保存的内容将丢失。')) {
                OVF_Flow.newFlow();
                this.render();
                this.updatePropertyPanel(null);
                this.showMessage('已创建新流程');
            }
        });

        // 打开
        document.getElementById('btn-open').addEventListener('click', () => {
            const input = document.createElement('input');
            input.type = 'file';
            input.accept = '.json';
            input.onchange = async (e) => {
                const file = e.target.files[0];
                if (file) {
                    try {
                        this.showLoading(true);
                        const data = await OVF_API.importFlowJSON(file);
                        OVF_Flow.importFlow(data);
                        this.render();
                        this.updatePropertyPanel(null);
                        this.showMessage(`已加载流程: ${file.name}`);
                    } catch (error) {
                        this.showError('加载失败: ' + error.message);
                    } finally {
                        this.showLoading(false);
                    }
                }
            };
            input.click();
        });

        // 保存
        document.getElementById('btn-save').addEventListener('click', () => {
            const data = OVF_Flow.exportFlow();
            OVF_API.exportFlowJSON(data, data.name + '.json');
            this.showMessage('流程已导出');
        });

        // 运行
        document.getElementById('btn-run').addEventListener('click', async () => {
            await this.executeFlow();
        });

        // 单步
        document.getElementById('btn-step').addEventListener('click', async () => {
            const selectedNodes = OVF_Flow.getSelectedNodes();
            if (selectedNodes.length > 0) {
                await this.stepFlow(selectedNodes[0].id);
            } else {
                this.showError('请先选择一个节点');
            }
        });

        // 停止
        document.getElementById('btn-stop').addEventListener('click', async () => {
            await this.stopFlow();
        });
    },

    /**
     * 绑定工具栏事件
     */
    bindToolbarEvents() {
        // 放大
        document.getElementById('btn-zoom-in').addEventListener('click', () => {
            this.state.zoom = Math.min(this.state.zoom * 1.2, 3);
            this.updateZoomLevel();
            this.render();
        });

        // 缩小
        document.getElementById('btn-zoom-out').addEventListener('click', () => {
            this.state.zoom = Math.max(this.state.zoom / 1.2, 0.3);
            this.updateZoomLevel();
            this.render();
        });

        // 适应视图
        document.getElementById('btn-fit-view').addEventListener('click', () => {
            this.fitView();
        });
    },

    /**
     * 绑定右键菜单事件
     */
    bindContextMenuEvents() {
        const menu = this.elements.contextMenu;

        menu.querySelectorAll('.context-menu-item').forEach(item => {
            item.addEventListener('click', (e) => {
                const action = item.dataset.action;
                this.handleContextMenuAction(action);
                this.hideContextMenu();
            });
        });

        // 点击其他地方隐藏菜单
        document.addEventListener('click', (e) => {
            if (!menu.contains(e.target)) {
                this.hideContextMenu();
            }
        });
    },

    /**
     * 绑定对话框事件
     */
    bindDialogEvents() {
        const dialog = this.elements.nodeDialog;

        dialog.querySelector('.modal-close').addEventListener('click', () => {
            this.closeNodeDialog();
        });

        dialog.querySelector('.modal-overlay').addEventListener('click', () => {
            this.closeNodeDialog();
        });

        dialog.querySelector('#dialog-cancel').addEventListener('click', () => {
            this.closeNodeDialog();
        });

        dialog.querySelector('#dialog-ok').addEventListener('click', () => {
            this.saveNodeDialog();
        });
    },

    /**
     * 绑定搜索事件
     */
    bindSearchEvents() {
        const searchInput = document.getElementById('node-search');

        searchInput.addEventListener('input', (e) => {
            const keyword = e.target.value.toLowerCase();
            this.filterNodeTypes(keyword);
        });
    },

    /**
     * 过滤节点类型
     */
    filterNodeTypes(keyword) {
        const nodeItems = this.elements.nodeList.querySelectorAll('.node-item');

        nodeItems.forEach(item => {
            const name = item.querySelector('.node-item-name').textContent.toLowerCase();
            const desc = item.querySelector('.node-item-desc').textContent.toLowerCase();

            if (keyword === '' || name.includes(keyword) || desc.includes(keyword)) {
                item.style.display = '';
            } else {
                item.style.display = 'none';
            }
        });
    },

    /**
     * 处理键盘事件
     */
    handleKeyDown(e) {
        // Delete键删除选中节点
        if (e.key === 'Delete' || e.key === 'Backspace') {
            const selectedNodes = OVF_Flow.getSelectedNodes();
            if (selectedNodes.length > 0) {
                selectedNodes.forEach(node => {
                    OVF_Flow.deleteNode(node.id);
                });
                this.render();
                this.updatePropertyPanel(null);
                this.showMessage('已删除节点');
            }

            if (OVF_Flow.selectedConnection) {
                OVF_Flow.deleteConnection(OVF_Flow.selectedConnection);
                this.render();
                this.showMessage('已删除连接');
            }
        }

        // Ctrl+C 复制
        if (e.ctrlKey && e.key === 'c') {
            const selectedNodes = OVF_Flow.getSelectedNodes();
            if (selectedNodes.length > 0) {
                this.state.clipboardNodes = selectedNodes.map(n => OVF_Flow.copyNode(n.id));
                this.render();
                this.showMessage('已复制节点');
            }
        }

        // Ctrl+V 粘贴
        if (e.ctrlKey && e.key === 'v') {
            if (this.state.clipboardNodes && this.state.clipboardNodes.length > 0) {
                this.state.clipboardNodes.forEach(nodeId => {
                    const node = OVF_Flow.copyNode(nodeId);
                    if (node) {
                        OVF_Flow.addNode(OVF_Flow.getNodeTypeByType(node.type), node.x, node.y);
                    }
                });
                this.render();
                this.showMessage('已粘贴节点');
            }
        }

        // Escape 取消选择
        if (e.key === 'Escape') {
            OVF_Flow.clearSelection();
            this.render();
            this.updatePropertyPanel(null);
            this.hideContextMenu();
            this.closeNodeDialog();
        }

        // Ctrl+S 保存
        if (e.ctrlKey && e.key === 's') {
            e.preventDefault();
            document.getElementById('btn-save').click();
        }
    },

    /**
     * 处理窗口大小调整
     */
    handleResize() {
        const wrapper = this.elements.canvasWrapper;
        const canvas = this.elements.canvas;

        canvas.width = wrapper.clientWidth;
        canvas.height = wrapper.clientHeight;

        this.elements.connectionSVG.setAttribute('width', wrapper.clientWidth);
        this.elements.connectionSVG.setAttribute('height', wrapper.clientHeight);

        this.render();
    },

    /**
     * 渲染画布
     */
    render() {
        this.renderConnections();
        this.renderNodes();
    },

    /**
     * 渲染节点
     */
    renderNodes() {
        const wrapper = this.elements.canvasWrapper;

        // 清除旧节点
        wrapper.querySelectorAll('.canvas-node').forEach(n => n.remove());

        // 渲染所有节点
        OVF_Flow.flowData.nodes.forEach(node => {
            const nodeElement = this.createNodeElement(node);
            wrapper.appendChild(nodeElement);
        });
    },

    /**
     * 创建节点DOM元素
     */
    createNodeElement(node) {
        const nodeElement = document.createElement('div');
        nodeElement.className = 'canvas-node';
        nodeElement.dataset.nodeId = node.id;

        // 应用位置和缩放
        nodeElement.style.left = (node.x * this.state.zoom + this.state.offsetX * this.state.zoom) + 'px';
        nodeElement.style.top = (node.y * this.state.zoom + this.state.offsetY * this.state.zoom) + 'px';
        nodeElement.style.width = (node.width * this.state.zoom) + 'px';
        nodeElement.style.minHeight = (node.height * this.state.zoom) + 'px';
        nodeElement.style.transform = `scale(${this.state.zoom})`;
        nodeElement.style.transformOrigin = 'top left';

        // 应用状态样式
        if (OVF_Flow.isNodeSelected(node.id)) {
            nodeElement.classList.add('selected');
        }
        if (node.status === 'running') {
            nodeElement.classList.add('running');
        }
        if (node.status === 'success') {
            nodeElement.classList.add('success');
        }
        if (node.status === 'error') {
            nodeElement.classList.add('error');
        }
        if (node.disabled) {
            nodeElement.classList.add('disabled');
        }

        // 创建节点头部
        const header = document.createElement('div');
        header.className = 'node-header';
        header.style.backgroundColor = node.color;

        const icon = document.createElement('span');
        icon.className = 'node-icon';
        icon.textContent = node.icon;

        const title = document.createElement('span');
        title.className = 'node-title';
        title.textContent = node.name;

        const statusDot = document.createElement('span');
        statusDot.className = 'node-status';
        if (node.status === 'running') statusDot.classList.add('running');
        if (node.status === 'success') statusDot.classList.add('success');
        if (node.status === 'error') statusDot.classList.add('error');

        header.appendChild(icon);
        header.appendChild(title);
        header.appendChild(statusDot);

        // 创建节点主体
        const body = document.createElement('div');
        body.className = 'node-body';

        // 创建端口
        const portsContainer = document.createElement('div');
        portsContainer.className = 'node-ports';

        // 输入端口
        if (node.inputs.length > 0) {
            node.inputs.forEach((input, idx) => {
                const portRow = document.createElement('div');
                portRow.className = 'node-port-row';

                const port = document.createElement('div');
                port.className = 'port port-input';
                port.dataset.portType = 'input';
                port.dataset.portIndex = idx;
                if (input.connected) port.classList.add('connected');

                const label = document.createElement('span');
                label.className = 'port-label';
                label.textContent = input.label;

                portRow.appendChild(port);
                portRow.appendChild(label);

                // 如果有输出端口，添加间隔
                if (node.outputs.length > idx) {
                    const output = node.outputs[idx];
                    const outputPort = document.createElement('div');
                    outputPort.className = 'port port-output';
                    outputPort.dataset.portType = 'output';
                    outputPort.dataset.portIndex = idx;
                    if (output.connected) outputPort.classList.add('connected');

                    const outputLabel = document.createElement('span');
                    outputLabel.className = 'port-label';
                    outputLabel.textContent = output.label;

                    portRow.appendChild(outputLabel);
                    portRow.appendChild(outputPort);
                }

                portsContainer.appendChild(portRow);
            });
        } else if (node.outputs.length > 0) {
            // 只有输出端口的情况
            node.outputs.forEach((output, idx) => {
                const portRow = document.createElement('div');
                portRow.className = 'node-port-row';
                portRow.style.justifyContent = 'flex-end';

                const label = document.createElement('span');
                label.className = 'port-label';
                label.textContent = output.label;

                const port = document.createElement('div');
                port.className = 'port port-output';
                port.dataset.portType = 'output';
                port.dataset.portIndex = idx;
                if (output.connected) port.classList.add('connected');

                portRow.appendChild(label);
                portRow.appendChild(port);
                portsContainer.appendChild(portRow);
            });
        }

        body.appendChild(portsContainer);

        nodeElement.appendChild(header);
        nodeElement.appendChild(body);

        // 右键菜单
        nodeElement.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            e.stopPropagation();
            this.state.selectedNode = node.id;
            this.showContextMenu(e.clientX, e.clientY, node.id);
        });

        return nodeElement;
    },

    /**
     * 渲染连接线
     */
    renderConnections() {
        const svg = this.elements.connectionSVG;
        svg.innerHTML = '';

        OVF_Flow.flowData.connections.forEach(conn => {
            const fromNode = OVF_Flow.getNodeById(conn.fromNode);
            const toNode = OVF_Flow.getNodeById(conn.toNode);

            if (!fromNode || !toNode) return;

            const fromPos = this.calculatePortPosition(fromNode, 'output', conn.fromPort);
            const toPos = this.calculatePortPosition(toNode, 'input', conn.toPort);

            if (!fromPos || !toPos) return;

            const path = this.createConnectionPath(fromPos, toPos);
            path.dataset.connectionId = conn.id;

            if (OVF_Flow.selectedConnection === conn.id) {
                path.classList.add('selected');
            }

            // 点击连线
            path.addEventListener('click', (e) => {
                e.stopPropagation();
                OVF_Flow.selectConnection(conn.id);
                this.render();
            });

            // 右键连线删除
            path.addEventListener('contextmenu', (e) => {
                e.preventDefault();
                OVF_Flow.deleteConnection(conn.id);
                this.render();
                this.showMessage('已删除连接');
            });

            svg.appendChild(path);
        });
    },

    /**
     * 计算端口位置
     */
    calculatePortPosition(node, portType, portIndex) {
        const nodeWidth = node.width * this.state.zoom;
        const nodeHeight = node.height * this.state.zoom;
        const offsetX = node.x * this.state.zoom + this.state.offsetX * this.state.zoom;
        const offsetY = node.y * this.state.zoom + this.state.offsetY * this.state.zoom;

        const headerHeight = 32 * this.state.zoom;
        const portSpacing = 24 * this.state.zoom;
        const portSize = 12 * this.state.zoom;
        const portOffset = 6 * this.state.zoom;

        const y = offsetY + headerHeight + 12 * this.state.zoom + portIndex * portSpacing + portSize / 2;

        if (portType === 'input') {
            return {
                x: offsetX - portOffset,
                y: y
            };
        } else {
            return {
                x: offsetX + nodeWidth + portOffset,
                y: y
            };
        }
    },

    /**
     * 创建连接路径
     */
    createConnectionPath(fromPos, toPos) {
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.classList.add('connection-line');

        const d = this.calculatePathD(fromPos, toPos);
        path.setAttribute('d', d);

        return path;
    },

    /**
     * 计算路径D属性
     */
    calculatePathD(fromPos, toPos) {
        const dx = Math.abs(toPos.x - fromPos.x);
        const dy = Math.abs(toPos.y - fromPos.y);
        const tension = Math.max(dx * 0.5, 50);

        const cp1x = fromPos.x + tension;
        const cp1y = fromPos.y;
        const cp2x = toPos.x - tension;
        const cp2y = toPos.y;

        return `M ${fromPos.x} ${fromPos.y} C ${cp1x} ${cp1y}, ${cp2x} ${cp2y}, ${toPos.x} ${toPos.y}`;
    },

    /**
     * 绘制临时连接线
     */
    drawTemporaryConnection(startX, startY, endX, endY) {
        this.clearTemporaryConnection();

        const svg = this.elements.connectionSVG;
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.classList.add('temporary-connection');

        const d = this.calculatePathD({ x: startX, y: startY }, { x: endX, y: endY });
        path.setAttribute('d', d);

        svg.appendChild(path);
    },

    /**
     * 清除临时连接线
     */
    clearTemporaryConnection() {
        const svg = this.elements.connectionSVG;
        const tempPath = svg.querySelector('.temporary-connection');
        if (tempPath) {
            tempPath.remove();
        }
    },

    /**
     * 选中节点
     */
    selectNode(nodeId, addToSelection = false) {
        OVF_Flow.selectNode(nodeId, addToSelection);
        this.render();
        this.updatePropertyPanel(nodeId);
    },

    /**
     * 更新属性面板
     */
    updatePropertyPanel(nodeId) {
        const content = this.elements.propertyContent;

        if (!nodeId) {
            content.innerHTML = `
                <div class="no-selection">
                    <p>选择一个节点查看属性</p>
                </div>
            `;
            return;
        }

        const node = OVF_Flow.getNodeById(nodeId);
        const nodeType = OVF_Flow.getNodeTypeByType(node.type);

        if (!node || !nodeType) return;

        let html = `
            <div class="property-group">
                <div class="property-group-title">基本信息</div>
                <div class="property-item">
                    <label class="property-label">节点名称</label>
                    <input class="property-input" type="text" value="${node.name}" data-param="name">
                </div>
                <div class="property-item">
                    <label class="property-label">节点类型</label>
                    <input class="property-input" type="text" value="${nodeType.name}" disabled>
                </div>
                <div class="property-item">
                    <label class="property-label">节点ID</label>
                    <input class="property-input" type="text" value="${node.id}" disabled>
                </div>
                <div class="property-item">
                    <label class="property-label">状态</label>
                    <input class="property-input" type="text" value="${node.status}" disabled>
                </div>
            </div>
        `;

        // 参数配置
        if (nodeType.params && nodeType.params.length > 0) {
            html += '<div class="property-group"><div class="property-group-title">参数配置</div>';

            nodeType.params.forEach(param => {
                const value = node.params[param.name] || param.default;

                html += `<div class="property-item"><label class="property-label">${param.label}</label>`;

                if (param.type === 'select') {
                    html += `<select class="property-select" data-param="${param.name}">`;
                    param.options.forEach(opt => {
                        html += `<option value="${opt}" ${value === opt ? 'selected' : ''}>${opt}</option>`;
                    });
                    html += '</select>';
                } else if (param.type === 'number') {
                    html += `<input class="property-input" type="number" data-param="${param.name}" value="${value}"`;
                    if (param.min !== undefined) html += ` min="${param.min}"`;
                    if (param.max !== undefined) html += ` max="${param.max}"`;
                    html += '>';
                } else if (param.type === 'textarea') {
                    html += `<textarea class="property-textarea" data-param="${param.name}">${value}</textarea>`;
                } else if (param.type === 'checkbox') {
                    html += `<label class="property-checkbox">
                        <input type="checkbox" data-param="${param.name}" ${value ? 'checked' : ''}>
                        <span>${param.label}</span>
                    </label>`;
                } else {
                    html += `<input class="property-input" type="text" data-param="${param.name}" value="${value}">`;
                }

                html += '</div>';
            });

            html += '</div>';
        }

        // 端口信息
        html += `
            <div class="property-group">
                <div class="property-group-title">端口信息</div>
                <div class="property-item">
                    <label class="property-label">输入端口</label>
                    <div style="font-size: 12px; color: var(--text-secondary);">
                        ${node.inputs.map(inp => inp.label).join(', ') || '无'}
                    </div>
                </div>
                <div class="property-item">
                    <label class="property-label">输出端口</label>
                    <div style="font-size: 12px; color: var(--text-secondary);">
                        ${node.outputs.map(out => out.label).join(', ') || '无'}
                    </div>
                </div>
            </div>
        `;

        content.innerHTML = html;

        // 绑定参数修改事件
        content.querySelectorAll('[data-param]').forEach(input => {
            input.addEventListener('change', (e) => {
                const paramName = e.target.dataset.param;
                let value;

                if (e.target.type === 'checkbox') {
                    value = e.target.checked;
                } else if (e.target.type === 'number') {
                    value = parseFloat(e.target.value);
                } else {
                    value = e.target.value;
                }

                if (paramName === 'name') {
                    node.name = value;
                    this.render();
                } else {
                    OVF_Flow.updateNodeParams(nodeId, { [paramName]: value });
                }

                this.showMessage('参数已更新');
            });
        });
    },

    /**
     * 显示右键菜单
     */
    showContextMenu(x, y, nodeId) {
        const menu = this.elements.contextMenu;
        menu.style.left = x + 'px';
        menu.style.top = y + 'px';
        menu.classList.add('show');

        this.state.selectedNode = nodeId;
    },

    /**
     * 隐藏右键菜单
     */
    hideContextMenu() {
        this.elements.contextMenu.classList.remove('show');
    },

    /**
     * 处理右键菜单动作
     */
    handleContextMenuAction(action) {
        if (!this.state.selectedNode) return;

        const nodeId = this.state.selectedNode;

        switch (action) {
            case 'copy':
                OVF_Flow.copyNode(nodeId);
                this.render();
                this.showMessage('节点已复制');
                break;

            case 'delete':
                OVF_Flow.deleteNode(nodeId);
                this.render();
                this.updatePropertyPanel(null);
                this.showMessage('节点已删除');
                break;

            case 'disable':
                OVF_Flow.toggleNodeDisabled(nodeId);
                this.render();
                this.updatePropertyPanel(nodeId);
                this.showMessage('节点状态已更改');
                break;
        }
    },

    /**
     * 打开节点参数对话框
     */
    openNodeDialog(nodeId) {
        const node = OVF_Flow.getNodeById(nodeId);
        const nodeType = OVF_Flow.getNodeTypeByType(node.type);

        if (!node || !nodeType) return;

        const dialog = this.elements.nodeDialog;
        const title = document.getElementById('dialog-title');
        const body = document.getElementById('dialog-body');

        title.textContent = node.name + ' - 参数设置';
        body.innerHTML = this.createDialogForm(node, nodeType);

        dialog.classList.add('show');
        this.state.dialogNodeId = nodeId;
    },

    /**
     * 创建对话框表单
     */
    createDialogForm(node, nodeType) {
        let html = '';

        if (!nodeType.params || nodeType.params.length === 0) {
            html = '<p style="color: var(--text-tertiary); text-align: center;">此节点没有可配置的参数</p>';
        } else {
            nodeType.params.forEach(param => {
                const value = node.params[param.name] || param.default;

                html += `<div class="property-item"><label class="property-label">${param.label}</label>`;

                if (param.type === 'select') {
                    html += `<select class="property-select" data-dialog-param="${param.name}">`;
                    param.options.forEach(opt => {
                        html += `<option value="${opt}" ${value === opt ? 'selected' : ''}>${opt}</option>`;
                    });
                    html += '</select>';
                } else if (param.type === 'number') {
                    html += `<input class="property-input" type="number" data-dialog-param="${param.name}" value="${value}"`;
                    if (param.min !== undefined) html += ` min="${param.min}"`;
                    if (param.max !== undefined) html += ` max="${param.max}"`;
                    html += '>';
                } else if (param.type === 'textarea') {
                    html += `<textarea class="property-textarea" data-dialog-param="${param.name}" style="min-height: 150px;">${value}</textarea>`;
                } else {
                    html += `<input class="property-input" type="text" data-dialog-param="${param.name}" value="${value}">`;
                }

                if (param.description) {
                    html += `<div style="font-size: 11px; color: var(--text-tertiary); margin-top: 4px;">${param.description}</div>`;
                }

                html += '</div>';
            });
        }

        return html;
    },

    /**
     * 关闭节点参数对话框
     */
    closeNodeDialog() {
        this.elements.nodeDialog.classList.remove('show');
        this.state.dialogNodeId = null;
    },

    /**
     * 保存节点参数对话框
     */
    saveNodeDialog() {
        if (!this.state.dialogNodeId) return;

        const body = document.getElementById('dialog-body');
        const params = {};

        body.querySelectorAll('[data-dialog-param]').forEach(input => {
            const paramName = input.dataset.dialogParam;

            if (input.type === 'checkbox') {
                params[paramName] = input.checked;
            } else if (input.type === 'number') {
                params[paramName] = parseFloat(input.value);
            } else {
                params[paramName] = input.value;
            }
        });

        OVF_Flow.updateNodeParams(this.state.dialogNodeId, params);
        this.updatePropertyPanel(this.state.dialogNodeId);
        this.closeNodeDialog();
        this.showMessage('参数已保存');
    },

    /**
     * 执行流程
     */
    async executeFlow() {
        try {
            this.showLoading(true, '执行流程...');
            this.executionState.isRunning = true;
            this.executionState.startTime = Date.now();

            this.updateStatus('执行中', 'running');
            document.getElementById('btn-stop').disabled = false;
            document.getElementById('btn-run').disabled = true;
            document.getElementById('btn-step').disabled = true;

            const result = await OVF_Flow.executeFlow();

            this.render();

            if (result.stopped) {
                // 被叫停：没跑完也不算失败，说清楚是"停的"
                this.showMessage('流程已停止（未执行完）');
                this.updateStatus('已停止', 'idle');
            } else if (result.success) {
                this.showMessage('流程执行成功');
                this.updateStatus('完成', 'success');
            } else {
                this.showError('流程执行失败');
                this.updateStatus('错误', 'error');
            }
        } catch (error) {
            this.showError('执行失败: ' + error.message);
            this.updateStatus('错误', 'error');
        } finally {
            this.showLoading(false);
            this.executionState.isRunning = false;
            document.getElementById('btn-stop').disabled = true;
            document.getElementById('btn-run').disabled = false;
            document.getElementById('btn-step').disabled = false;

            const elapsed = Date.now() - this.executionState.startTime;
            this.updateTime(elapsed);
        }
    },

    /**
     * 单步执行
     */
    async stepFlow(nodeId) {
        try {
            this.executionState.isRunning = true;

            const result = await OVF_Flow.stepFlow(nodeId);

            this.render();

            if (result.success) {
                this.showMessage(`节点执行成功`);
                this.updateStatus(`节点 ${nodeId}`, 'running');
            } else {
                this.showError('节点执行失败');
                this.updateStatus('错误', 'error');
            }
        } catch (error) {
            this.showError('单步执行失败: ' + error.message);
        } finally {
            this.executionState.isRunning = false;
        }
    },

    /**
     * 停止执行
     */
    async stopFlow() {
        try {
            await OVF_Flow.stopFlow();
            this.executionState.isRunning = false;
            this.render();
            this.showMessage('执行已停止');
            this.updateStatus('已停止', 'idle');
        } catch (error) {
            this.showError('停止失败: ' + error.message);
        }
    },

    /**
     * 适应视图
     */
    fitView() {
        const nodes = OVF_Flow.flowData.nodes;

        if (nodes.length === 0) {
            this.state.zoom = 1;
            this.state.offsetX = 0;
            this.state.offsetY = 0;
            this.updateZoomLevel();
            this.render();
            return;
        }

        // 计算节点边界
        let minX = Infinity, minY = Infinity;
        let maxX = -Infinity, maxY = -Infinity;

        nodes.forEach(node => {
            minX = Math.min(minX, node.x);
            minY = Math.min(minY, node.y);
            maxX = Math.max(maxX, node.x + node.width);
            maxY = Math.max(maxY, node.y + node.height);
        });

        const wrapper = this.elements.canvasWrapper;
        const width = wrapper.clientWidth;
        const height = wrapper.clientHeight;

        const contentWidth = maxX - minX;
        const contentHeight = maxY - minY;

        // 计算缩放比例
        const scaleX = (width - 100) / contentWidth;
        const scaleY = (height - 100) / contentHeight;
        this.state.zoom = Math.min(scaleX, scaleY, 1);

        // 计算偏移
        this.state.offsetX = -(minX - 50);
        this.state.offsetY = -(minY - 50);

        this.updateZoomLevel();
        this.render();
    },

    /**
     * 更新缩放级别显示
     */
    updateZoomLevel() {
        this.elements.zoomLevel.textContent = Math.round(this.state.zoom * 100) + '%';
    },

    /**
     * 显示消息
     */
    showMessage(message) {
        document.getElementById('status-message').textContent = message;
    },

    /**
     * 显示错误
     */
    showError(message) {
        this.showMessage(message);
        this.updateStatus('错误', 'error');
    },

    /**
     * 显示加载
     */
    showLoading(show, text = '加载中...') {
        const overlay = this.elements.loadingOverlay;
        if (show) {
            overlay.querySelector('.loading-text').textContent = text;
            overlay.classList.add('show');
        } else {
            overlay.classList.remove('show');
        }
    },

    /**
     * 更新执行状态
     */
    updateStatus(status, type) {
        const statusEl = document.getElementById('status-execution');
        const dot = statusEl.querySelector('.status-dot');
        const text = statusEl.querySelector('span:last-child');

        text.textContent = status;

        dot.classList.remove('running', 'error');
        if (type === 'running') {
            dot.classList.add('running');
        } else if (type === 'error') {
            dot.classList.add('error');
        }
    },

    /**
     * 更新时间显示
     */
    updateTime(elapsed = null) {
        const timeEl = document.getElementById('status-time');

        if (elapsed !== null) {
            const seconds = Math.floor(elapsed / 1000);
            const ms = elapsed % 1000;
            timeEl.textContent = `耗时: ${seconds}.${ms}s`;
        } else {
            const now = new Date();
            timeEl.textContent = now.toLocaleTimeString();
        }
    },

    /**
     * 更新位置显示
     */
    updatePosition(x, y) {
        document.getElementById('status-position').textContent = `X: ${x}, Y: ${y}`;
    }
};

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', () => {
    OVF_Editor.init();
});

// 导出编辑器
window.OVF_Editor = OVF_Editor;