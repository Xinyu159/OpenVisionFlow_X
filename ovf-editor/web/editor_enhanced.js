/**
 * OpenVisionFlow Web Editor - Enhanced JavaScript
 * 版本: 2.0
 */

// ==================== 全局配置 ====================
const CONFIG = {
    API_BASE: 'http://localhost:8080/api',
    WS_URL: 'ws://localhost:8080/ws',
    GRID_SIZE: 20,
    NODE_WIDTH: 180,
    NODE_HEIGHT: 80,
    PORT_SIZE: 12,
    CONNECTION_WIDTH: 2,
    ZOOM_MIN: 0.25,
    ZOOM_MAX: 4,
    ZOOM_STEP: 0.15,
    SNAP_TO_GRID: true,
    ANIMATION_DURATION: 300
};

// ==================== 状态管理 ====================
class EditorState {
    constructor() {
        this.nodes = new Map();
        this.connections = [];
        this.selectedNodes = [];
        this.selectedConnection = null;
        this.clipboard = [];
        this.undoStack = [];
        this.redoStack = [];
        this.zoom = 1;
        this.panOffset = { x: 0, y: 0 };
        this.gridVisible = true;
        this.minimapVisible = true;
        this.debugMode = false;
        this.breakpoints = new Set();
        this.isRunning = false;
        this.currentFlowId = null;
        this.modified = false;
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
        this.markModified();
    }

    addConnection(connection) {
        this.connections.push(connection);
        this.markModified();
    }

    removeConnection(connectionId) {
        this.connections = this.connections.filter(c => c.id !== connectionId);
        this.markModified();
    }

    selectNode(nodeId, addToSelection = false) {
        if (addToSelection) {
            if (!this.selectedNodes.includes(nodeId)) {
                this.selectedNodes.push(nodeId);
            }
        } else {
            this.selectedNodes = [nodeId];
        }
    }

    deselectAll() {
        this.selectedNodes = [];
        this.selectedConnection = null;
    }

    copySelected() {
        this.clipboard = this.selectedNodes.map(id => {
            const node = this.nodes.get(id);
            return { ...node, id: null };
        });
    }

    paste(offsetX = 50, offsetY = 50) {
        const newNodes = [];
        this.clipboard.forEach(nodeTemplate => {
            const newNode = {
                ...nodeTemplate,
                id: generateId(),
                x: nodeTemplate.x + offsetX,
                y: nodeTemplate.y + offsetY
            };
            this.addNode(newNode);
            newNodes.push(newNode);
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
            nodes: Array.from(this.nodes.values()).map(n => ({...n})),
            connections: this.connections.map(c => ({...c}))
        };
        this.undoStack.push(snapshot);
        if (this.undoStack.length > 50) {
            this.undoStack.shift();
        }
        this.redoStack = [];
    }

    undo() {
        if (this.undoStack.length === 0) return false;
        const snapshot = this.undoStack.pop();
        this.redoStack.push({
            nodes: Array.from(this.nodes.values()).map(n => ({...n})),
            connections: this.connections.map(c => ({...c}))
        });
        this.restoreSnapshot(snapshot);
        return true;
    }

    redo() {
        if (this.redoStack.length === 0) return false;
        const snapshot = this.redoStack.pop();
        this.undoStack.push({
            nodes: Array.from(this.nodes.values()).map(n => ({...n})),
            connections: this.connections.map(c => ({...c}))
        });
        this.restoreSnapshot(snapshot);
        return true;
    }

    restoreSnapshot(snapshot) {
        this.nodes.clear();
        snapshot.nodes.forEach(n => this.nodes.set(n.id, n));
        this.connections = snapshot.connections;
        this.markModified();
        renderCanvas();
    }

    toJSON() {
        return {
            id: this.currentFlowId || generateId(),
            name: '未命名流程',
            nodes: Array.from(this.nodes.values()),
            connections: this.connections,
            created_time: new Date().toISOString(),
            modified_time: new Date().toISOString()
        };
    }

    fromJSON(json) {
        this.nodes.clear();
        this.connections = [];
        json.nodes.forEach(n => this.nodes.set(n.id, n));
        this.connections = json.connections || [];
        this.currentFlowId = json.id;
        this.clearModified();
        renderCanvas();
    }
}

const state = new EditorState();

// ==================== 画布渲染 ====================
class CanvasRenderer {
    constructor() {
        this.canvas = document.getElementById('flowCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.svgOverlay = document.getElementById('overlaySvg');
        this.minimapCanvas = document.getElementById('minimapCanvas');
        this.minimapCtx = this.minimapCanvas.getContext('2d');
        this.resize();
        this.setupEvents();
    }

    resize() {
        const wrapper = this.canvas.parentElement;
        this.canvas.width = wrapper.clientWidth;
        this.canvas.height = wrapper.clientHeight;
        this.minimapCanvas.width = 150;
        this.minimapCanvas.height = 100;
    }

    setupEvents() {
        window.addEventListener('resize', () => this.resize());
    }

    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }

    drawGrid() {
        if (!state.gridVisible) return;

        const gridSize = CONFIG.GRID_SIZE * state.zoom;
        this.ctx.strokeStyle = '#313244';
        this.ctx.lineWidth = 1;

        const offsetX = state.panOffset.x % gridSize;
        const offsetY = state.panOffset.y % gridSize;

        for (let x = offsetX; x < this.canvas.width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }

        for (let y = offsetY; y < this.canvas.height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
    }

    drawNode(node) {
        const x = node.x * state.zoom + state.panOffset.x;
        const y = node.y * state.zoom + state.panOffset.y;
        const width = CONFIG.NODE_WIDTH * state.zoom;
        const height = CONFIG.NODE_HEIGHT * state.zoom;

        // 背景
        this.ctx.fillStyle = '#313244';
        this.ctx.strokeStyle = state.selectedNodes.includes(node.id) ? '#89b4fa' : '#45475a';
        this.ctx.lineWidth = 2;

        this.roundRect(x, y, width, height, 8);
        this.ctx.fill();
        this.ctx.stroke();

        // 状态边框
        if (node.state === 'running') {
            this.ctx.strokeStyle = '#f9e2af';
            this.ctx.lineWidth = 3;
            this.roundRect(x, y, width, height, 8);
            this.ctx.stroke();
        } else if (node.state === 'success') {
            this.ctx.strokeStyle = '#a6e3a1';
            this.ctx.lineWidth = 3;
            this.roundRect(x, y, width, height, 8);
            this.ctx.stroke();
        } else if (node.state === 'error') {
            this.ctx.strokeStyle = '#f38ba8';
            this.ctx.lineWidth = 3;
            this.roundRect(x, y, width, height, 8);
            this.ctx.stroke();
        }

        // 标题栏
        this.ctx.fillStyle = '#45475a';
        this.roundRect(x, y, width, 25 * state.zoom, 8, true, false);
        this.ctx.fill();

        // 节点名称
        this.ctx.fillStyle = '#cdd6f4';
        this.ctx.font = `${12 * state.zoom}px 'Segoe UI'`;
        this.ctx.fillText(node.name, x + 10 * state.zoom, y + 17 * state.zoom);

        // 输入端口
        if (node.inputs) {
            node.inputs.forEach((port, i) => {
                const portY = y + 35 * state.zoom + i * 20 * state.zoom;
                this.drawPort(x - 6 * state.zoom, portY, port.type, 'input');
            });
        }

        // 输出端口
        if (node.outputs) {
            node.outputs.forEach((port, i) => {
                const portY = y + 35 * state.zoom + i * 20 * state.zoom;
                this.drawPort(x + width - 6 * state.zoom, portY, port.type, 'output');
            });
        }

        // 断点标记
        if (state.breakpoints.has(node.id)) {
            this.ctx.fillStyle = '#f38ba8';
            this.ctx.beginPath();
            this.ctx.arc(x + width - 15 * state.zoom, y + 12 * state.zoom, 5 * state.zoom, 0, 2 * Math.PI);
            this.ctx.fill();
        }

        // 执行时间
        if (node.executeTime) {
            this.ctx.fillStyle = '#6c7086';
            this.ctx.font = `${10 * state.zoom}px 'Segoe UI'`;
            this.ctx.fillText(`${node.executeTime}ms`, x + 10 * state.zoom, y + height - 10 * state.zoom);
        }
    }

    drawPort(x, y, type, direction) {
        const size = CONFIG.PORT_SIZE * state.zoom;

        this.ctx.beginPath();
        this.ctx.arc(x, y, size / 2, 0, 2 * Math.PI);

        // 根据数据类型选择颜色
        const colorMap = {
            'image': '#89b4fa',
            'number': '#f9e2af',
            'string': '#a6e3a1',
            'boolean': '#cba6f7',
            'region': '#f38ba8'
        };

        this.ctx.fillStyle = '#45475a';
        this.ctx.fill();
        this.ctx.strokeStyle = colorMap[type] || '#cdd6f4';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
    }

    drawConnection(conn) {
        const sourceNode = state.nodes.get(conn.sourceId);
        const targetNode = state.nodes.get(conn.targetId);

        if (!sourceNode || !targetNode) return;

        // 计算起点和终点
        const startX = sourceNode.x * state.zoom + state.panOffset.x + CONFIG.NODE_WIDTH * state.zoom;
        const startY = sourceNode.y * state.zoom + state.panOffset.y + 35 * state.zoom;
        const endX = targetNode.x * state.zoom + state.panOffset.x;
        const endY = targetNode.y * state.zoom + state.panOffset.y + 35 * state.zoom;

        // 贝塞尔曲线
        const midX = (startX + endX) / 2;

        this.ctx.beginPath();
        this.ctx.moveTo(startX, startY);
        this.ctx.bezierCurveTo(midX, startY, midX, endY, endX, endY);

        // 根据数据类型选择颜色
        const colorMap = {
            'image': '#89b4fa',
            'number': '#f9e2af',
            'string': '#a6e3a1',
            'boolean': '#cba6f7'
        };

        this.ctx.strokeStyle = colorMap[conn.dataType] || '#45475a';
        this.ctx.lineWidth = CONFIG.CONNECTION_WIDTH * state.zoom;

        if (conn.active) {
            this.ctx.setLineDash([10, 5]);
            this.ctx.strokeStyle = '#f9e2af';
        } else {
            this.ctx.setLineDash([]);
        }

        this.ctx.stroke();

        // 流动动画效果（在活动连接上）
        if (conn.active) {
            this.drawFlowParticles(startX, startY, midX, endX, endY);
        }
    }

    drawFlowParticles(startX, startY, midX, endX, endY) {
        // 简化的流动粒子效果
        const time = Date.now() / 1000;
        const t = (time % 1);

        // 计算贝塞尔曲线上的点
        const x = bezierPoint(startX, midX, midX, endX, t);
        const y = bezierPoint(startY, startY, endY, endY, t);

        this.ctx.beginPath();
        this.ctx.arc(x, y, 4 * state.zoom, 0, 2 * Math.PI);
        this.ctx.fillStyle = '#f9e2af';
        this.ctx.fill();
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

        this.minimapCtx.clearRect(0, 0, 150, 100);

        // 计算缩放比例
        const bounds = this.getNodesBounds();
        const scaleX = 150 / (bounds.width + 100);
        const scaleY = 100 / (bounds.height + 100);
        const scale = Math.min(scaleX, scaleY, 0.1);

        // 绘制节点
        state.nodes.forEach(node => {
            const x = (node.x - bounds.minX + 50) * scale;
            const y = (node.y - bounds.minY + 50) * scale;
            const w = CONFIG.NODE_WIDTH * scale;
            const h = CONFIG.NODE_HEIGHT * scale;

            this.minimapCtx.fillStyle = '#45475a';
            this.minimapCtx.fillRect(x, y, w, h);
        });

        // 绘制视口框
        const viewX = (-state.panOffset.x / state.zoom - bounds.minX + 50) * scale;
        const viewY = (-state.panOffset.y / state.zoom - bounds.minY + 50) * scale;
        const viewW = (this.canvas.width / state.zoom) * scale;
        const viewH = (this.canvas.height / state.zoom) * scale;

        this.minimapCtx.strokeStyle = '#89b4fa';
        this.minimapCtx.lineWidth = 2;
        this.minimapCtx.strokeRect(viewX, viewY, viewW, viewH);
    }

    getNodesBounds() {
        let minX = Infinity, minY = Infinity;
        let maxX = -Infinity, maxY = -Infinity;

        state.nodes.forEach(node => {
            minX = Math.min(minX, node.x);
            minY = Math.min(minY, node.y);
            maxX = Math.max(maxX, node.x + CONFIG.NODE_WIDTH);
            maxY = Math.max(maxY, node.y + CONFIG.NODE_HEIGHT);
        });

        if (minX === Infinity) {
            return { minX: 0, minY: 0, width: 1000, height: 800 };
        }

        return {
            minX,
            minY,
            width: maxX - minX,
            height: maxY - minY
        };
    }

    render() {
        this.clear();
        this.drawGrid();

        // 绘制连接
        state.connections.forEach(conn => this.drawConnection(conn));

        // 绘制节点
        state.nodes.forEach(node => this.drawNode(node));

        // 绘制小地图
        this.drawMinimap();
    }
}

let renderer;

// ==================== 交互控制器 ====================
class InteractionController {
    constructor() {
        this.dragging = false;
        this.dragNode = null;
        this.dragOffset = { x: 0, y: 0 };
        this.connecting = false;
        this.connectStart = null;
        this.tempConnection = null;
        this.panning = false;
        this.panStart = { x: 0, y: 0 };
        this.selecting = false;
        this.selectionRect = null;
        this.lastMousePos = { x: 0, y: 0 };

        this.setupEvents();
    }

    setupEvents() {
        const canvas = renderer.canvas;

        canvas.addEventListener('mousedown', e => this.onMouseDown(e));
        canvas.addEventListener('mousemove', e => this.onMouseMove(e));
        canvas.addEventListener('mouseup', e => this.onMouseUp(e));
        canvas.addEventListener('wheel', e => this.onWheel(e));
        canvas.addEventListener('dblclick', e => this.onDoubleClick(e));
        canvas.addEventListener('contextmenu', e => this.onContextMenu(e));

        // 键盘事件
        document.addEventListener('keydown', e => this.onKeyDown(e));

        // 拖放事件
        canvas.addEventListener('dragover', e => e.preventDefault());
        canvas.addEventListener('drop', e => this.onDrop(e));

        // 触摸事件（移动端支持）
        canvas.addEventListener('touchstart', e => this.onTouchStart(e));
        canvas.addEventListener('touchmove', e => this.onTouchMove(e));
        canvas.addEventListener('touchend', e => this.onTouchEnd(e));
    }

    onMouseDown(e) {
        const pos = this.getCanvasPos(e);
        this.lastMousePos = pos;

        // 检查是否点击了端口
        const portHit = this.hitTestPort(pos);
        if (portHit) {
            this.startConnection(portHit, pos);
            return;
        }

        // 检查是否点击了节点
        const nodeHit = this.hitTestNode(pos);
        if (nodeHit) {
            if (e.ctrlKey) {
                // Ctrl+点击：添加到选择
                state.selectNode(nodeHit.id, true);
            } else if (state.selectedNodes.includes(nodeHit.id)) {
                // 已选中的节点：开始拖动
                this.startDrag(nodeHit, pos);
            } else {
                // 未选中的节点：单独选择
                state.selectNode(nodeHit.id);
                this.startDrag(nodeHit, pos);
            }
            updateParamsPanel(nodeHit);
            renderCanvas();
            return;
        }

        // 检查是否点击了连接
        const connHit = this.hitTestConnection(pos);
        if (connHit) {
            state.selectedConnection = connHit;
            renderCanvas();
            return;
        }

        // 空白区域：开始框选或平移
        if (e.button === 0) {
            state.deselectAll();
            renderCanvas();
            updateParamsPanel(null);

            if (e.altKey) {
                this.startPan(pos);
            } else {
                this.startSelection(pos);
            }
        }
    }

    onMouseMove(e) {
        const pos = this.getCanvasPos(e);

        if (this.dragging) {
            this.dragNode.x = (pos.x - this.dragOffset.x) / state.zoom - state.panOffset.x / state.zoom;
            this.dragNode.y = (pos.y - this.dragOffset.y) / state.zoom - state.panOffset.y / state.zoom;

            if (CONFIG.SNAP_TO_GRID) {
                this.dragNode.x = Math.round(this.dragNode.x / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
                this.dragNode.y = Math.round(this.dragNode.y / CONFIG.GRID_SIZE) * CONFIG.GRID_SIZE;
            }

            renderCanvas();
        } else if (this.connecting) {
            this.updateTempConnection(pos);
        } else if (this.panning) {
            state.panOffset.x += pos.x - this.panStart.x;
            state.panOffset.y += pos.y - this.panStart.y;
            this.panStart = { x: pos.x, y: pos.y };
            renderCanvas();
        } else if (this.selecting) {
            this.updateSelectionRect(pos);
        }

        this.lastMousePos = pos;
    }

    onMouseUp(e) {
        const pos = this.getCanvasPos(e);

        if (this.dragging) {
            this.endDrag();
        } else if (this.connecting) {
            this.endConnection(pos);
        } else if (this.panning) {
            this.endPan();
        } else if (this.selecting) {
            this.endSelection();
        }
    }

    onWheel(e) {
        e.preventDefault();

        const delta = e.deltaY > 0 ? -CONFIG.ZOOM_STEP : CONFIG.ZOOM_STEP;
        const newZoom = Math.max(CONFIG.ZOOM_MIN, Math.min(CONFIG.ZOOM_MAX, state.zoom + delta));

        // 以鼠标位置为中心缩放
        const pos = this.getCanvasPos(e);
        const zoomRatio = newZoom / state.zoom;

        state.panOffset.x = pos.x - (pos.x - state.panOffset.x) * zoomRatio;
        state.panOffset.y = pos.y - (pos.y - state.panOffset.y) * zoomRatio;

        state.zoom = newZoom;
        updateZoomLevel();
        renderCanvas();
    }

    onDoubleClick(e) {
        const pos = this.getCanvasPos(e);
        const nodeHit = this.hitTestNode(pos);

        if (nodeHit) {
            // 双击节点：折叠/展开
            nodeHit.collapsed = !nodeHit.collapsed;
            renderCanvas();
        }
    }

    onContextMenu(e) {
        e.preventDefault();

        const pos = this.getCanvasPos(e);
        const nodeHit = this.hitTestNode(pos);

        if (nodeHit) {
            showContextMenu(e.clientX, e.clientY, nodeHit);
        } else {
            showContextMenu(e.clientX, e.clientY, null);
        }
    }

    onKeyDown(e) {
        // 防止在输入框中触发
        if (e.target.tagName === 'INPUT') return;

        switch (e.key) {
            case 'Delete':
            case 'Backspace':
                deleteSelected();
                break;
            case 'c':
                if (e.ctrlKey) copyNodes();
                break;
            case 'v':
                if (e.ctrlKey) pasteNodes();
                break;
            case 'x':
                if (e.ctrlKey) cutNodes();
                break;
            case 'z':
                if (e.ctrlKey) {
                    if (e.shiftKey) redo();
                    else undo();
                }
                break;
            case 'y':
                if (e.ctrlKey) redo();
                break;
            case 'n':
                if (e.ctrlKey) newFlow();
                break;
            case 'o':
                if (e.ctrlKey) openFlow();
                break;
            case 's':
                if (e.ctrlKey) saveFlow();
                break;
            case '+':
            case '=':
                zoomIn();
                break;
            case '-':
                zoomOut();
                break;
            case 'F5':
                if (e.shiftKey) stopFlow();
                else runFlow();
                break;
            case 'F10':
                stepFlow();
                break;
            case '?':
                toggleShortcutHint();
                break;
        }
    }

    onDrop(e) {
        e.preventDefault();

        const pos = this.getCanvasPos(e);
        const nodeType = e.dataTransfer.getData('nodeType');

        if (nodeType) {
            const nodeX = pos.x / state.zoom - state.panOffset.x / state.zoom;
            const nodeY = pos.y / state.zoom - state.panOffset.y / state.zoom;

            createNodeFromType(nodeType, nodeX, nodeY);
        }
    }

    onTouchStart(e) {
        if (e.touches.length === 1) {
            const touch = e.touches[0];
            const pos = { x: touch.clientX, y: touch.clientY };
            this.onMouseDown({ ...e, clientX: touch.clientX, clientY: touch.clientY });
        } else if (e.touches.length === 2) {
            // 双指缩放
            this.touchStartDistance = this.getTouchDistance(e.touches);
        }
    }

    onTouchMove(e) {
        if (e.touches.length === 1) {
            const touch = e.touches[0];
            this.onMouseMove({ ...e, clientX: touch.clientX, clientY: touch.clientY });
        } else if (e.touches.length === 2) {
            const distance = this.getTouchDistance(e.touches);
            const delta = distance - this.touchStartDistance;
            state.zoom = Math.max(CONFIG.ZOOM_MIN, Math.min(CONFIG.ZOOM_MAX,
                state.zoom + delta * 0.01));
            this.touchStartDistance = distance;
            updateZoomLevel();
            renderCanvas();
        }
    }

    onTouchEnd(e) {
        this.onMouseUp(e);
    }

    getTouchDistance(touches) {
        const dx = touches[0].clientX - touches[1].clientX;
        const dy = touches[0].clientY - touches[1].clientY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    getCanvasPos(e) {
        const rect = renderer.canvas.getBoundingClientRect();
        return {
            x: e.clientX - rect.left,
            y: e.clientY - rect.top
        };
    }

    hitTestNode(pos) {
        for (const node of state.nodes.values()) {
            const x = node.x * state.zoom + state.panOffset.x;
            const y = node.y * state.zoom + state.panOffset.y;
            const width = CONFIG.NODE_WIDTH * state.zoom;
            const height = CONFIG.NODE_HEIGHT * state.zoom;

            if (pos.x >= x && pos.x <= x + width &&
                pos.y >= y && pos.y <= y + height) {
                return node;
            }
        }
        return null;
    }

    hitTestPort(pos) {
        for (const node of state.nodes.values()) {
            const nodeX = node.x * state.zoom + state.panOffset.x;
            const nodeY = node.y * state.zoom + state.panOffset.y;

            // 检查输入端口
            if (node.inputs) {
                node.inputs.forEach((port, i) => {
                    const portY = nodeY + 35 * state.zoom + i * 20 * state.zoom;
                    const portX = nodeX - 6 * state.zoom;

                    if (this.hitTestCircle(pos, portX, portY, CONFIG.PORT_SIZE * state.zoom)) {
                        return { node, port, direction: 'input', index: i };
                    }
                });
            }

            // 检查输出端口
            if (node.outputs) {
                node.outputs.forEach((port, i) => {
                    const portY = nodeY + 35 * state.zoom + i * 20 * state.zoom;
                    const portX = nodeX + CONFIG.NODE_WIDTH * state.zoom - 6 * state.zoom;

                    if (this.hitTestCircle(pos, portX, portY, CONFIG.PORT_SIZE * state.zoom)) {
                        return { node, port, direction: 'output', index: i };
                    }
                });
            }
        }
        return null;
    }

    hitTestCircle(pos, x, y, radius) {
        const dx = pos.x - x;
        const dy = pos.y - y;
        return dx * dx + dy * dy <= radius * radius;
    }

    hitTestConnection(pos) {
        // 简化的连接线碰撞检测
        for (const conn of state.connections) {
            const sourceNode = state.nodes.get(conn.sourceId);
            const targetNode = state.nodes.get(conn.targetId);

            if (!sourceNode || !targetNode) continue;

            const startX = sourceNode.x * state.zoom + state.panOffset.x + CONFIG.NODE_WIDTH * state.zoom;
            const startY = sourceNode.y * state.zoom + state.panOffset.y + 35 * state.zoom;
            const endX = targetNode.x * state.zoom + state.panOffset.x;
            const endY = targetNode.y * state.zoom + state.panOffset.y + 35 * state.zoom;

            // 检查点到贝塞尔曲线的距离（简化为直线）
            const dist = this.pointToLineDistance(pos, startX, startY, endX, endY);
            if (dist < 10 * state.zoom) {
                return conn;
            }
        }
        return null;
    }

    pointToLineDistance(point, x1, y1, x2, y2) {
        const A = point.x - x1;
        const B = point.y - y1;
        const C = x2 - x1;
        const D = y2 - y1;

        const dot = A * C + B * D;
        const lenSq = C * C + D * D;
        let param = -1;

        if (lenSq !== 0) param = dot / lenSq;

        let xx, yy;

        if (param < 0) {
            xx = x1; yy = y1;
        } else if (param > 1) {
            xx = x2; yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        const dx = point.x - xx;
        const dy = point.y - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }

    startDrag(node, pos) {
        state.pushUndo();
        this.dragging = true;
        this.dragNode = node;
        this.dragOffset = {
            x: pos.x - (node.x * state.zoom + state.panOffset.x),
            y: pos.y - (node.y * state.zoom + state.panOffset.y)
        };
    }

    endDrag() {
        this.dragging = false;
        this.dragNode = null;
    }

    startConnection(portHit, pos) {
        this.connecting = true;
        this.connectStart = {
            node: portHit.node,
            port: portHit.port,
            direction: portHit.direction,
            pos: pos
        };
    }

    updateTempConnection(pos) {
        // 绘制临时连接线
        renderCanvas();

        const startX = this.connectStart.node.x * state.zoom + state.panOffset.x +
            (this.connectStart.direction === 'output' ? CONFIG.NODE_WIDTH * state.zoom : 0);
        const startY = this.connectStart.node.y * state.zoom + state.panOffset.y + 35 * state.zoom;

        renderer.ctx.beginPath();
        renderer.ctx.moveTo(startX, startY);
        renderer.ctx.lineTo(pos.x, pos.y);
        renderer.ctx.strokeStyle = '#89b4fa';
        renderer.ctx.lineWidth = 2;
        renderer.ctx.stroke();
    }

    endConnection(pos) {
        this.connecting = false;

        const portHit = this.hitTestPort(pos);
        if (portHit && portHit.direction !== this.connectStart.direction) {
            // 创建连接
            const source = this.connectStart.direction === 'output' ? this.connectStart : portHit;
            const target = this.connectStart.direction === 'input' ? this.connectStart : portHit;

            state.pushUndo();
            state.addConnection({
                id: generateId(),
                sourceId: source.node.id,
                sourcePort: source.port.id,
                targetId: target.node.id,
                targetPort: target.port.id,
                dataType: source.port.type
            });

            renderCanvas();
            updateStatusBar();
        }

        this.connectStart = null;
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

    updateSelectionRect(pos) {
        this.selectionRect.width = pos.x - this.selectionRect.x;
        this.selectionRect.height = pos.y - this.selectionRect.y;

        renderCanvas();

        // 绘制选择框
        renderer.ctx.strokeStyle = '#89b4fa';
        renderer.ctx.lineWidth = 1;
        renderer.ctx.setLineDash([5, 5]);
        renderer.ctx.strokeRect(
            this.selectionRect.x,
            this.selectionRect.y,
            this.selectionRect.width,
            this.selectionRect.height
        );
        renderer.ctx.setLineDash([]);
    }

    endSelection() {
        this.selecting = false;

        // 选择框内的节点
        state.nodes.forEach(node => {
            const x = node.x * state.zoom + state.panOffset.x;
            const y = node.y * state.zoom + state.panOffset.y;
            const width = CONFIG.NODE_WIDTH * state.zoom;
            const height = CONFIG.NODE_HEIGHT * state.zoom;

            const rect2 = { x, y, width, height };

            if (this.rectsIntersect(this.selectionRect, rect2)) {
                state.selectNode(node.id, true);
            }
        });

        this.selectionRect = null;
        renderCanvas();
    }

    rectsIntersect(r1, r2) {
        return !(r2.x > r1.x + r1.width ||
                 r2.x + r2.width < r1.x ||
                 r2.y > r1.y + r1.height ||
                 r2.y + r2.height < r1.y);
    }
}

let interaction;

// ==================== API客户端 ====================
class ApiClient {
    constructor() {
        this.baseUrl = CONFIG.API_BASE;
        this.ws = null;
        this.wsConnected = false;
        this.eventHandlers = new Map();
    }

    async get(path) {
        try {
            const response = await fetch(this.baseUrl + path);
            return await response.json();
        } catch (error) {
            logMessage(`API请求失败: ${error.message}`, 'error');
            return { success: false, error: error.message };
        }
    }

    async post(path, data) {
        try {
            const response = await fetch(this.baseUrl + path, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            });
            return await response.json();
        } catch (error) {
            logMessage(`API请求失败: ${error.message}`, 'error');
            return { success: false, error: error.message };
        }
    }

    async put(path, data) {
        try {
            const response = await fetch(this.baseUrl + path, {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            });
            return await response.json();
        } catch (error) {
            logMessage(`API请求失败: ${error.message}`, 'error');
            return { success: false, error: error.message };
        }
    }

    async delete(path) {
        try {
            const response = await fetch(this.baseUrl + path, {
                method: 'DELETE'
            });
            return await response.json();
        } catch (error) {
            logMessage(`API请求失败: ${error.message}`, 'error');
            return { success: false, error: error.message };
        }
    }

    connectWebSocket() {
        if (this.ws) return;

        this.ws = new WebSocket(CONFIG.WS_URL);

        this.ws.onopen = () => {
            this.wsConnected = true;
            logMessage('WebSocket连接成功', 'info');
        };

        this.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            this.handleWsMessage(data);
        };

        this.ws.onerror = (error) => {
            logMessage('WebSocket错误', 'error');
        };

        this.ws.onclose = () => {
            this.wsConnected = false;
            logMessage('WebSocket断开连接', 'warn');

            // 5秒后重连
            setTimeout(() => this.connectWebSocket(), 5000);
        };
    }

    handleWsMessage(data) {
        switch (data.type) {
            case 'execution_progress':
                updateExecutionProgress(data);
                break;
            case 'node_state':
                updateNodeState(data.node_id, data.state);
                break;
            case 'execution_result':
                handleExecutionResult(data);
                break;
            case 'debug_info':
                handleDebugInfo(data);
                break;
            case 'log':
                logMessage(data.message, data.level);
                break;
            case 'error':
                showError(data.message);
                break;
        }

        // 触发自定义事件处理
        if (this.eventHandlers.has(data.type)) {
            this.eventHandlers.get(data.type)(data);
        }
    }

    on(event, handler) {
        this.eventHandlers.set(event, handler);
    }

    // API方法
    async getNodes() {
        return await this.get('/nodes');
    }

    async getNodeInfo(typeId) {
        return await this.get(`/node/${typeId}`);
    }

    async createFlow(flowDef) {
        return await this.post('/flow', flowDef);
    }

    async updateFlow(flowId, flowDef) {
        return await this.put(`/flow/${flowId}`, flowDef);
    }

    async deleteFlow(flowId) {
        return await this.delete(`/flow/${flowId}`);
    }

    async executeFlow(flowId) {
        return await this.post(`/flow/${flowId}/execute`);
    }

    async getFlowResult(flowId) {
        return await this.get(`/flow/${flowId}/result`);
    }

    async uploadImage(imageData) {
        return await this.post('/image', imageData);
    }

    async getImage(imageId) {
        return await this.get(`/image/${imageId}`);
    }
}

let apiClient;

// ==================== 节点类型管理 ====================
const nodeTypes = {
    '输入': [
        { id: 'web.image_input', name: '图像输入', icon: '📷', color: '#89b4fa' },
        { id: 'web.parameter_input', name: '参数输入', icon: '📊', color: '#f9e2af' },
        { id: 'camera.capture', name: '相机采集', icon: '📹', color: '#89b4fa' },
        { id: 'file.image_load', name: '文件加载', icon: '📁', color: '#a6e3a1' }
    ],
    '图像处理': [
        { id: 'image.threshold', name: '阈值分割', icon: 'Threshold', color: '#cba6f7' },
        { id: 'image.filter', name: '滤波', icon: 'Filter', color: '#cba6f7' },
        { id: 'image.morphology', name: '形态学', icon: 'Morph', color: '#cba6f7' },
        { id: 'image.edge_detection', name: '边缘检测', icon: 'Edge', color: '#cba6f7' },
        { id: 'image.color_convert', name: '颜色转换', icon: 'Color', color: '#cba6f7' },
        { id: 'image.enhance', name: '图像增强', icon: 'Enhance', color: '#cba6f7' },
        { id: 'image.roi', name: 'ROI裁剪', icon: 'ROI', color: '#f38ba8' }
    ],
    '检测分析': [
        { id: 'blob.analysis', name: 'Blob分析', icon: 'Blob', color: '#a6e3a1' },
        { id: 'pattern.match', name: '模板匹配', icon: 'Pattern', color: '#a6e3a1' },
        { id: 'barcode.read', name: '条码读取', icon: 'Barcode', color: '#f9e2af' },
        { id: 'ocr.recognize', name: 'OCR识别', icon: 'OCR', color: '#f9e2af' },
        { id: 'object.detect', name: '目标检测', icon: 'Object', color: '#a6e3a1' }
    ],
    '测量': [
        { id: 'measure.distance', name: '距离测量', icon: '📏', color: '#89b4fa' },
        { id: 'measure.angle', name: '角度测量', icon: 'Angle', color: '#89b4fa' },
        { id: 'measure.circle', name: '圆测量', icon: 'Circle', color: '#89b4fa' },
        { id: 'measure.line', name: '直线测量', icon: 'Line', color: '#89b4fa' },
        { id: 'calibration.nine_point', name: '九点标定', icon: 'Calib', color: '#f9e2af' }
    ],
    '几何': [
        { id: 'geometry.find_line', name: '找直线', icon: 'Line', color: '#89b4fa' },
        { id: 'geometry.find_circle', name: '找圆', icon: 'Circle', color: '#89b4fa' },
        { id: 'geometry.find_rectangle', name: '找矩形', icon: 'Rect', color: '#89b4fa' },
        { id: 'geometry.pose', name: '位姿计算', icon: 'Pose', color: '#cba6f7' }
    ],
    '输出': [
        { id: 'web.result_output', name: '结果输出', icon: '📤', color: '#a6e3a1' },
        { id: 'web.report_generate', name: '报告生成', icon: '📄', color: '#f9e2af' },
        { id: 'web.notification', name: '通知推送', icon: '🔔', color: '#f38ba8' },
        { id: 'file.image_save', name: '图像保存', icon: '💾', color: '#a6e3a1' }
    ],
    '逻辑': [
        { id: 'logic.condition', name: '条件判断', icon: 'If', color: '#f9e2af' },
        { id: 'logic.loop', name: '循环', icon: 'Loop', color: '#f9e2af' },
        { id: 'script.execute', name: '脚本执行', icon: 'Script', color: '#cba6f7' },
        { id: 'toolblock.container', name: '工具块', icon: 'Block', color: '#45475a' }
    ],
    '通信': [
        { id: 's7.read', name: 'S7读取', icon: 'S7', color: '#89b4fa' },
        { id: 's7.write', name: 'S7写入', icon: 'S7', color: '#89b4fa' },
        { id: 'modbus.read', name: 'Modbus读取', icon: 'Modbus', color: '#89b4fa' },
        { id: 'tcp.send', name: 'TCP发送', icon: 'TCP', color: '#89b4fa' }
    ]
};

function populateNodeCategories() {
    const container = document.getElementById('nodeCategories');
    container.innerHTML = '';

    for (const [category, nodes] of Object.entries(nodeTypes)) {
        const categoryDiv = document.createElement('div');
        categoryDiv.className = 'category';

        const headerDiv = document.createElement('div');
        headerDiv.className = 'category-header';
        headerDiv.innerHTML = `
            <span>${category}</span>
            <span class="icon">▼</span>
        `;
        headerDiv.onclick = () => {
            categoryDiv.classList.toggle('collapsed');
        };

        const itemsDiv = document.createElement('div');
        itemsDiv.className = 'category-items';

        nodes.forEach(node => {
            const itemDiv = document.createElement('div');
            itemDiv.className = 'node-item';
            itemDiv.draggable = true;
            itemDiv.innerHTML = `
                <div class="node-icon" style="color: ${node.color}">${node.icon}</div>
                <div>${node.name}</div>
            `;

            itemDiv.addEventListener('dragstart', e => {
                e.dataTransfer.setData('nodeType', node.id);
                itemDiv.classList.add('dragging');
            });

            itemDiv.addEventListener('dragend', () => {
                itemDiv.classList.remove('dragging');
            });

            itemDiv.addEventListener('dblclick', () => {
                createNodeFromType(node.id, 100, 100);
            });

            itemsDiv.appendChild(itemDiv);
        });

        categoryDiv.appendChild(headerDiv);
        categoryDiv.appendChild(itemsDiv);
        container.appendChild(categoryDiv);
    }
}

// ==================== 工具函数 ====================
function generateId() {
    return 'node_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
}

function bezierPoint(p0, p1, p2, p3, t) {
    const u = 1 - t;
    return u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
}

// ==================== 主功能函数 ====================
function renderCanvas() {
    renderer.render();
}

function createNodeFromType(typeId, x, y) {
    state.pushUndo();

    // 从类型定义中获取节点信息
    let nodeInfo = null;
    for (const nodes of Object.values(nodeTypes)) {
        nodeInfo = nodes.find(n => n.id === typeId);
        if (nodeInfo) break;
    }

    if (!nodeInfo) {
        logMessage(`未找到节点类型: ${typeId}`, 'error');
        return;
    }

    // 创建节点实例
    const node = {
        id: generateId(),
        type: typeId,
        name: nodeInfo.name,
        x: x,
        y: y,
        inputs: [],
        outputs: [],
        params: {},
        state: 'idle',
        executeTime: 0,
        enabled: true,
        collapsed: false
    };

    // 添加默认端口
    if (typeId.includes('image') || typeId.includes('camera') || typeId.includes('file')) {
        node.outputs.push({ id: 'image', name: '图像', type: 'image' });
    } else {
        node.inputs.push({ id: 'image', name: '图像', type: 'image' });
    }

    if (typeId.includes('result') || typeId.includes('output') || typeId.includes('report')) {
        node.inputs.push({ id: 'data', name: '数据', type: 'object' });
    } else {
        node.outputs.push({ id: 'result', name: '结果', type: 'object' });
    }

    state.addNode(node);
    renderCanvas();
    updateStatusBar();
    logMessage(`创建节点: ${nodeInfo.name}`, 'info');
}

function updateParamsPanel(node) {
    const infoDiv = document.getElementById('nodeInfo');
    const paramsDiv = document.getElementById('nodeParams');
    const paramItemsDiv = document.getElementById('paramItems');

    if (!node) {
        infoDiv.innerHTML = '<p style="color: #6c7086; text-align: center; padding: 20px;">请选择一个节点</p>';
        paramsDiv.style.display = 'none';
        return;
    }

    // 显示节点信息
    infoDiv.innerHTML = `
        <div class="param-item">
            <label class="param-label">节点类型</label>
            <input class="param-input" type="text" value="${node.type}" readonly>
        </div>
        <div class="param-item">
            <label class="param-label">节点名称</label>
            <input class="param-input" type="text" value="${node.name}"
                   onchange="updateNodeName('${node.id}', this.value)">
        </div>
        <div class="param-item">
            <label class="param-label">节点ID</label>
            <input class="param-input" type="text" value="${node.id}" readonly>
        </div>
        <div class="param-item param-checkbox">
            <label class="param-label">
                <input type="checkbox" ${node.enabled ? 'checked' : ''}
                       onchange="toggleNodeEnabled('${node.id}')">
                启用节点
            </label>
        </div>
    `;

    // 显示参数（如果有）
    paramsDiv.style.display = 'block';
    paramItemsDiv.innerHTML = '';

    // 根据节点类型添加特定参数
    const typeParams = getNodeDefaultParams(node.type);
    for (const [paramId, paramDef] of Object.entries(typeParams)) {
        const paramDiv = createParamWidget(node, paramId, paramDef);
        paramItemsDiv.appendChild(paramDiv);
    }
}

function getNodeDefaultParams(type) {
    // 返回节点类型的默认参数定义
    const paramsMap = {
        'image.threshold': {
            'threshold_value': { name: '阈值', type: 'number', min: 0, max: 255, default: 128 },
            'threshold_type': { name: '类型', type: 'select', options: ['Binary', 'BinaryInv', 'Trunc', 'ToZero'], default: 'Binary' }
        },
        'image.filter': {
            'filter_type': { name: '滤波类型', type: 'select', options: ['Gaussian', 'Median', 'Blur'], default: 'Gaussian' },
            'kernel_size': { name: '核大小', type: 'number', min: 1, max: 31, default: 5 }
        },
        'blob.analysis': {
            'min_area': { name: '最小面积', type: 'number', min: 0, max: 10000, default: 100 },
            'max_area': { name: '最大面积', type: 'number', min: 0, max: 100000, default: 10000 }
        },
        'measure.distance': {
            'unit': { name: '单位', type: 'select', options: ['mm', 'pixel'], default: 'mm' }
        }
    };

    return paramsMap[type] || {};
}

function createParamWidget(node, paramId, paramDef) {
    const div = document.createElement('div');
    div.className = 'param-item';

    const currentValue = node.params[paramId] || paramDef.default;

    switch (paramDef.type) {
        case 'number':
            div.innerHTML = `
                <label class="param-label">${paramDef.name}</label>
                <div class="param-range">
                    <input type="range" min="${paramDef.min}" max="${paramDef.max}"
                           value="${currentValue}"
                           onchange="updateNodeParam('${node.id}', '${paramId}', this.value)">
                    <span class="param-range-value">${currentValue}</span>
                </div>
            `;
            break;
        case 'select':
            const optionsHtml = paramDef.options.map(opt =>
                `<option value="${opt}" ${opt === currentValue ? 'selected' : ''}>${opt}</option>`
            ).join('');
            div.innerHTML = `
                <label class="param-label">${paramDef.name}</label>
                <select class="param-select" onchange="updateNodeParam('${node.id}', '${paramId}', this.value)">
                    ${optionsHtml}
                </select>
            `;
            break;
        case 'string':
            div.innerHTML = `
                <label class="param-label">${paramDef.name}</label>
                <input class="param-input" type="text" value="${currentValue}"
                       onchange="updateNodeParam('${node.id}', '${paramId}', this.value)">
            `;
            break;
        case 'boolean':
            div.innerHTML = `
                <label class="param-checkbox">
                    <input type="checkbox" ${currentValue ? 'checked' : ''}
                           onchange="updateNodeParam('${node.id}', '${paramId}', this.checked)">
                    ${paramDef.name}
                </label>
            `;
            break;
    }

    return div;
}

function updateNodeName(nodeId, name) {
    const node = state.nodes.get(nodeId);
    if (node) {
        state.pushUndo();
        node.name = name;
        state.markModified();
        renderCanvas();
    }
}

function updateNodeParam(nodeId, paramId, value) {
    const node = state.nodes.get(nodeId);
    if (node) {
        node.params[paramId] = value;
        state.markModified();

        // 实时预览（如果节点有输出图像）
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
    }
}

function updateStatusBar() {
    document.getElementById('nodeCount').textContent = state.nodes.size;
    document.getElementById('connectionCount').textContent = state.connections.length;
}

function updateZoomLevel() {
    document.getElementById('zoomLevel').textContent = Math.round(state.zoom * 100) + '%';
}

function updateTitle() {
    const title = state.modified ? 'OpenVisionFlow Editor [*]' : 'OpenVisionFlow Editor';
    document.title = title;
}

// ==================== 文件操作 ====================
function newFlow() {
    if (state.modified) {
        showConfirmDialog('当前流程未保存，是否继续？', () => {
            state.nodes.clear();
            state.connections = [];
            state.undoStack = [];
            state.redoStack = [];
            state.currentFlowId = null;
            state.clearModified();
            renderCanvas();
            updateStatusBar();
            logMessage('新建流程', 'info');
        });
    } else {
        state.nodes.clear();
        state.connections = [];
        state.undoStack = [];
        state.redoStack = [];
        state.currentFlowId = null;
        renderCanvas();
        updateStatusBar();
        logMessage('新建流程', 'info');
    }
}

function openFlow() {
    showInputDialog('打开流程', '请输入流程文件路径', '', async (filepath) => {
        try {
            // 从API加载流程
            const result = await apiClient.get(`/flow/load?path=${encodeURIComponent(filepath)}`);
            if (result.success) {
                state.fromJSON(result.flow);
                logMessage(`加载流程: ${filepath}`, 'info');
            } else {
                showError(`加载失败: ${result.error}`);
            }
        } catch (error) {
            showError(`加载失败: ${error.message}`);
        }
    });
}

async function saveFlow() {
    const flowDef = state.toJSON();

    try {
        const result = await apiClient.updateFlow(state.currentFlowId || 'new', flowDef);
        if (result.success) {
            state.currentFlowId = result.flow_id;
            state.clearModified();
            logMessage('流程保存成功', 'info');
        } else {
            showError(`保存失败: ${result.error}`);
        }
    } catch (error) {
        showError(`保存失败: ${error.message}`);
    }
}

function exportFlow() {
    const flowDef = state.toJSON();
    const json = JSON.stringify(flowDef, null, 2);

    // 下载JSON文件
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'flow_export.json';
    a.click();
    URL.revokeObjectURL(url);

    logMessage('流程导出成功', 'info');
}

function loadTemplate(templateName) {
    const templates = {
        'basic_inspection': [
            { type: 'camera.capture', x: 100, y: 100, name: '相机采集' },
            { type: 'image.threshold', x: 300, y: 100, name: '阈值分割' },
            { type: 'blob.analysis', x: 500, y: 100, name: 'Blob分析' },
            { type: 'web.result_output', x: 700, y: 100, name: '结果输出' }
        ],
        'measurement': [
            { type: 'camera.capture', x: 100, y: 100, name: '相机采集' },
            { type: 'image.edge_detection', x: 300, y: 100, name: '边缘检测' },
            { type: 'measure.distance', x: 500, y: 100, name: '距离测量' },
            { type: 'web.result_output', x: 700, y: 100, name: '结果输出' }
        ],
        'barcode': [
            { type: 'camera.capture', x: 100, y: 100, name: '相机采集' },
            { type: 'image.roi', x: 300, y: 100, name: 'ROI裁剪' },
            { type: 'barcode.read', x: 500, y: 100, name: '条码读取' },
            { type: 'web.result_output', x: 700, y: 100, name: '结果输出' }
        ],
        'ocr': [
            { type: 'camera.capture', x: 100, y: 100, name: '相机采集' },
            { type: 'image.enhance', x: 300, y: 100, name: '图像增强' },
            { type: 'ocr.recognize', x: 500, y: 100, name: 'OCR识别' },
            { type: 'web.result_output', x: 700, y: 100, name: '结果输出' }
        ],
        'alignment': [
            { type: 'camera.capture', x: 100, y: 100, name: '相机采集' },
            { type: 'pattern.match', x: 300, y: 100, name: '模板匹配' },
            { type: 'geometry.pose', x: 500, y: 100, name: '位姿计算' },
            { type: 's7.write', x: 700, y: 100, name: 'PLC写入' }
        ]
    };

    const template = templates[templateName];
    if (!template) {
        showError('未找到模板');
        return;
    }

    state.nodes.clear();
    state.connections = [];

    // 创建节点
    template.forEach((nodeDef, index) => {
        const node = {
            id: generateId(),
            type: nodeDef.type,
            name: nodeDef.name,
            x: nodeDef.x,
            y: nodeDef.y,
            inputs: [{ id: 'image', name: '图像', type: 'image' }],
            outputs: [{ id: 'image', name: '图像', type: 'image' }, { id: 'result', name: '结果', type: 'object' }],
            params: {},
            state: 'idle',
            enabled: true
        };
        state.addNode(node);
    });

    // 创建连接
    const nodes = Array.from(state.nodes.values());
    for (let i = 0; i < nodes.length - 1; i++) {
        state.addConnection({
            id: generateId(),
            sourceId: nodes[i].id,
            sourcePort: 'image',
            targetId: nodes[i + 1].id,
            targetPort: 'image',
            dataType: 'image'
        });
    }

    renderCanvas();
    updateStatusBar();
    fitToScreen();
    logMessage(`加载模板: ${templateName}`, 'info');
}

// ==================== 编辑操作 ====================
function copyNodes() {
    state.copySelected();
    logMessage(`复制 ${state.clipboard.length} 个节点`, 'info');
}

function pasteNodes() {
    const newNodes = state.paste();
    state.selectedNodes = newNodes.map(n => n.id);
    renderCanvas();
    logMessage(`粘贴 ${newNodes.length} 个节点`, 'info');
}

function cutNodes() {
    state.copySelected();
    deleteSelected();
}

function deleteSelected() {
    state.pushUndo();
    state.selectedNodes.forEach(id => state.removeNode(id));
    state.deselectAll();
    renderCanvas();
    updateStatusBar();
    logMessage('删除节点', 'info');
}

function undo() {
    if (state.undo()) {
        renderCanvas();
        logMessage('撤销', 'info');
    }
}

function redo() {
    if (state.redo()) {
        renderCanvas();
        logMessage('重做', 'info');
    }
}

// ==================== 视图操作 ====================
function zoomIn() {
    state.zoom = Math.min(CONFIG.ZOOM_MAX, state.zoom + CONFIG.ZOOM_STEP);
    updateZoomLevel();
    renderCanvas();
}

function zoomOut() {
    state.zoom = Math.max(CONFIG.ZOOM_MIN, state.zoom - CONFIG.ZOOM_STEP);
    updateZoomLevel();
    renderCanvas();
}

function fitToScreen() {
    const bounds = renderer.getNodesBounds();
    const scaleX = renderer.canvas.width / (bounds.width + 200);
    const scaleY = renderer.canvas.height / (bounds.height + 200);
    state.zoom = Math.min(scaleX, scaleY, 1);

    state.panOffset.x = (renderer.canvas.width - bounds.width * state.zoom) / 2 - bounds.minX * state.zoom;
    state.panOffset.y = (renderer.canvas.height - bounds.height * state.zoom) / 2 - bounds.minY * state.zoom;

    updateZoomLevel();
    renderCanvas();
}

function toggleGrid() {
    state.gridVisible = !state.gridVisible;
    renderCanvas();
}

function toggleMinimap() {
    state.minimapVisible = !state.minimapVisible;
    document.getElementById('minimap').style.display = state.minimapVisible ? 'block' : 'none';
    renderCanvas();
}

// ==================== 执行操作 ====================
async function runFlow() {
    if (state.isRunning) {
        showError('流程正在执行中');
        return;
    }

    state.isRunning = true;
    updateStatus('running', '执行中');

    showProgress('执行流程...');

    try {
        const flowDef = state.toJSON();
        const result = await apiClient.executeFlow(state.currentFlowId || 'temp');

        if (result.success) {
            logMessage('流程执行成功', 'info');
            handleExecutionResult(result);
        } else {
            showError(`执行失败: ${result.error}`);
        }
    } catch (error) {
        showError(`执行失败: ${error.message}`);
    }

    hideProgress();
    state.isRunning = false;
    updateStatus('ready', '就绪');
}

function stopFlow() {
    if (!state.isRunning) return;

    apiClient.post('/flow/stop');
    state.isRunning = false;
    updateStatus('ready', '已停止');
    hideProgress();
    logMessage('流程执行已停止', 'warn');
}

async function stepFlow() {
    const result = await apiClient.post('/flow/step');

    if (result.success) {
        updateNodeState(result.node_id, 'success');
        logMessage(`单步执行: ${result.node_name}`, 'info');
    }
}

function toggleDebug() {
    state.debugMode = !state.debugMode;
    logMessage(`调试模式: ${state.debugMode ? '开启' : '关闭'}`, 'info');

    if (state.debugMode) {
        apiClient.connectWebSocket();
        switchPanel('debug');
    }
}

function addBreakpoint() {
    if (state.selectedNodes.length > 0) {
        state.selectedNodes.forEach(id => state.breakpoints.add(id));
        updateBreakpointsList();
        renderCanvas();
        logMessage(`添加断点: ${state.selectedNodes.length} 个节点`, 'info');
    }
}

function updateBreakpointsList() {
    const container = document.getElementById('breakpoints');
    if (state.breakpoints.size === 0) {
        container.innerHTML = '<p style="color: #6c7086; padding: 10px;">无断点</p>';
        return;
    }

    container.innerHTML = '';
    state.breakpoints.forEach(nodeId => {
        const node = state.nodes.get(nodeId);
        if (node) {
            const div = document.createElement('div');
            div.className = 'breakpoint-item';
            div.innerHTML = `
                <span class="node-name">${node.name}</span>
                <span class="remove-btn" onclick="removeBreakpoint('${nodeId}')">✕</span>
            `;
            container.appendChild(div);
        }
    });
}

function removeBreakpoint(nodeId) {
    state.breakpoints.delete(nodeId);
    updateBreakpointsList();
    renderCanvas();
}

function stepOver() { apiClient.post('/debug/step_over'); }
function stepInto() { apiClient.post('/debug/step_into'); }
function stepOut() { apiClient.post('/debug/step_out'); }
function continueExec() { apiClient.post('/debug/continue'); }

// ==================== 分组操作 ====================
function groupSelected() {
    if (state.selectedNodes.length < 2) {
        showError('至少选择2个节点');
        return;
    }

    logMessage('创建分组', 'info');
}

function ungroupSelected() {
    logMessage('取消分组', 'info');
}

function createComment() {
    const comment = {
        id: generateId(),
        type: 'comment',
        name: '注释',
        x: 100,
        y: 100,
        text: '在此输入注释...'
    };
    state.addNode(comment);
    renderCanvas();
}

// ==================== UI辅助函数 ====================
function switchPanel(panel) {
    const panels = ['paramsPanel', 'previewPanel', 'debugPanel', 'resultsPanel'];
    const tabs = document.querySelectorAll('.panel-tab');

    panels.forEach(p => {
        document.getElementById(p).classList.remove('active');
    });

    tabs.forEach(t => t.classList.remove('active'));

    document.getElementById(panel + 'Panel').classList.add('active');
    tabs[panels.indexOf(panel + 'Panel')].classList.add('active');
}

function switchLog(logType) {
    const tabs = document.querySelectorAll('.log-tab');
    tabs.forEach(t => t.classList.remove('active'));
    tabs.forEach(t => {
        if (t.textContent.toLowerCase().includes(logType)) {
            t.classList.add('active');
        }
    });
}

function logMessage(message, level = 'info') {
    const container = document.getElementById('logContent');
    const time = new Date().toLocaleTimeString();

    const entry = document.createElement('div');
    entry.className = 'log-entry';
    entry.innerHTML = `
        <span class="log-time">${time}</span>
        <span class="log-level ${level}">${level.toUpperCase()}</span>
        <span class="log-message">${message}</span>
    `;

    container.appendChild(entry);
    container.scrollTop = container.scrollHeight;
}

function updateStatus(status, text) {
    const indicator = document.getElementById('statusIndicator');
    const statusText = document.getElementById('statusText');

    indicator.className = 'status-indicator ' + status;
    statusText.textContent = text;
}

function showProgress(text) {
    document.getElementById('progressOverlay').classList.add('active');
    document.getElementById('progressText').textContent = text;
    document.getElementById('progressFill').style.width = '0%';
}

function updateProgress(percent, text) {
    document.getElementById('progressFill').style.width = percent + '%';
    if (text) document.getElementById('progressText').textContent = text;
}

function hideProgress() {
    document.getElementById('progressOverlay').classList.remove('active');
}

function showError(message) {
    logMessage(message, 'error');

    // 显示错误模态框
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header" style="color: #f38ba8">错误</div>
        <div class="modal-body">
            <p>${message}</p>
        </div>
        <div class="modal-footer">
            <button class="modal-btn primary" onclick="hideModal()">确定</button>
        </div>
    `;
    document.getElementById('modalOverlay').classList.add('active');
}

function showConfirmDialog(message, onConfirm) {
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header">确认</div>
        <div class="modal-body">
            <p>${message}</p>
        </div>
        <div class="modal-footer">
            <button class="modal-btn secondary" onclick="hideModal()">取消</button>
            <button class="modal-btn primary" onclick="hideModal(); (${onConfirm})()">确定</button>
        </div>
    `;
    document.getElementById('modalOverlay').classList.add('active');
}

function showInputDialog(title, message, defaultValue, onSubmit) {
    const modal = document.getElementById('modalContent');
    modal.innerHTML = `
        <div class="modal-header">${title}</div>
        <div class="modal-body">
            <p>${message}</p>
            <input class="param-input" type="text" id="dialogInput" value="${defaultValue}">
        </div>
        <div class="modal-footer">
            <button class="modal-btn secondary" onclick="hideModal()">取消</button>
            <button class="modal-btn primary" onclick="hideModal(); (${onSubmit})(document.getElementById('dialogInput').value)">确定</button>
        </div>
    `;
    document.getElementById('modalOverlay').classList.add('active');
}

function hideModal() {
    document.getElementById('modalOverlay').classList.remove('active');
}

function showContextMenu(x, y, node) {
    const menu = document.getElementById('contextMenu');
    menu.style.left = x + 'px';
    menu.style.top = y + 'px';
    menu.classList.add('active');

    // 根据是否选中节点显示不同选项
    const items = menu.querySelectorAll('.context-menu-item');
    if (!node) {
        items.forEach(item => {
            if (item.textContent.includes('断点') ||
                item.textContent.includes('输出') ||
                item.textContent.includes('分组')) {
                item.style.display = 'none';
            }
        });
    }
}

function hideContextMenu() {
    document.getElementById('contextMenu').classList.remove('active');
}

function toggleShortcutHint() {
    const hint = document.getElementById('shortcutHint');
    hint.classList.toggle('active');

    setTimeout(() => {
        hint.classList.remove('active');
    }, 5000);
}

// ==================== 图像预览 ====================
let imagePreviewZoom = 1;
let roiMode = false;

function showImagePreview(imageData) {
    const canvas = document.getElementById('imagePreviewCanvas');
    const ctx = canvas.getContext('2d');

    if (!imageData) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        return;
    }

    // 创建图像
    const img = new Image();
    img.onload = () => {
        canvas.width = img.width * imagePreviewZoom;
        canvas.height = img.height * imagePreviewZoom;
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    };
    img.src = imageData;
}

function zoomImageIn() {
    imagePreviewZoom = Math.min(4, imagePreviewZoom + 0.25);
    updateImagePreview();
}

function zoomImageOut() {
    imagePreviewZoom = Math.max(0.25, imagePreviewZoom - 0.25);
    updateImagePreview();
}

function fitImage() {
    imagePreviewZoom = 1;
    updateImagePreview();
}

function toggleRoiMode() {
    roiMode = !roiMode;
    logMessage(`ROI模式: ${roiMode ? '开启' : '关闭'}`, 'info');
}

function updateImagePreview() {
    // 更新图像预览显示
}

// ==================== 执行结果处理 ====================
function handleExecutionResult(result) {
    // 更新节点状态
    if (result.node_states) {
        for (const [nodeId, nodeState] of Object.entries(result.node_states)) {
            updateNodeState(nodeId, nodeState);
        }
    }

    // 更新执行时间
    document.getElementById('executionTime').textContent =
        (result.total_time_us / 1000).toFixed(2) + 'ms';

    // 显示结果
    if (result.outputs) {
        showExecutionResults(result.outputs);
    }

    hideProgress();
}

function updateNodeState(nodeId, nodeState) {
    const node = state.nodes.get(nodeId);
    if (node) {
        node.state = nodeState;
        renderCanvas();
    }
}

function updateExecutionProgress(data) {
    updateProgress(data.percent, data.message);

    // 更新当前执行节点
    if (data.current_node) {
        updateNodeState(data.current_node, 'running');
    }
}

function showExecutionResults(outputs) {
    const container = document.getElementById('resultItems');
    container.innerHTML = '';

    for (const [key, value] of Object.entries(outputs)) {
        const div = document.createElement('div');
        div.className = 'result-item';

        if (key === 'image' && value.startsWith('data:image')) {
            // 图像结果
            div.innerHTML = `
                <div class="result-header">图像输出</div>
                <img src="${value}" style="max-width: 100%; max-height: 200px;">
            `;
        } else if (key === 'histogram') {
            // 直方图结果
            const chartCanvas = document.createElement('canvas');
            chartCanvas.className = 'result-chart';
            div.appendChild(chartCanvas);

            const chart = new Chart(chartCanvas, {
                type: 'bar',
                data: value,
                options: {
                    responsive: true,
                    maintainAspectRatio: false
                }
            });
        } else if (typeof value === 'object') {
            // 表格结果
            const table = document.createElement('table');
            table.className = 'result-table';
            table.innerHTML = `
                <thead>
                    <tr>
                        <th>属性</th>
                        <th>值</th>
                    </tr>
                </thead>
                <tbody>
                    ${Object.entries(value).map(([k, v]) =>
                        `<tr><td>${k}</td><td>${JSON.stringify(v)}</td></tr>`
                    ).join('')}
                </tbody>
            `;
            div.appendChild(table);
        } else {
            // 文本结果
            div.innerHTML = `
                <div class="result-header">${key}</div>
                <div>${JSON.stringify(value)}</div>
            `;
        }

        container.appendChild(div);
    }

    switchPanel('results');
}

function handleDebugInfo(data) {
    // 更新变量列表
    const container = document.getElementById('variables');
    container.innerHTML = '';

    for (const [name, value] of Object.entries(data.variables)) {
        const div = document.createElement('div');
        div.className = 'variable-item';
        div.innerHTML = `
            <span class="variable-name">${name}</span>
            <span class="variable-value">${JSON.stringify(value)}</span>
        `;
        container.appendChild(div);
    }

    // 高亮当前执行行
    if (data.current_line) {
        // 高亮源码行（如果有代码编辑器）
    }
}

function previewNodeOutput(nodeId) {
    // 实时预览节点输出
    const node = state.nodes.get(nodeId);
    if (node && node.outputs.length > 0) {
        apiClient.get(`/node/${nodeId}/preview`).then(result => {
            if (result.success && result.image) {
                showImagePreview(result.image);
                switchPanel('preview');
            }
        });
    }
}

function viewNodeOutput() {
    if (state.selectedNodes.length > 0) {
        previewNodeOutput(state.selectedNodes[0]);
    }
}

// ==================== 初始化 ====================
function init() {
    // 创建渲染器
    renderer = new CanvasRenderer();

    // 创建交互控制器
    interaction = new InteractionController();

    // 创建API客户端
    apiClient = new ApiClient();
    apiClient.connectWebSocket();

    // 填充节点分类
    populateNodeCategories();

    // 设置搜索过滤
    document.getElementById('nodeSearch').addEventListener('input', (e) => {
        const keyword = e.target.value.toLowerCase();
        filterNodeCategories(keyword);
    });

    // 点击空白区域关闭上下文菜单
    document.addEventListener('click', (e) => {
        if (!e.target.closest('.context-menu')) {
            hideContextMenu();
        }
    });

    // 初始渲染
    renderCanvas();
    updateStatusBar();

    logMessage('OpenVisionFlow Web Editor 初始化完成', 'info');
    logMessage('按 ? 查看快捷键帮助', 'info');
}

function filterNodeCategories(keyword) {
    const categories = document.querySelectorAll('.category');

    categories.forEach(category => {
        const items = category.querySelectorAll('.node-item');
        let hasMatch = false;

        items.forEach(item => {
            const name = item.textContent.toLowerCase();
            if (name.includes(keyword)) {
                item.style.display = 'flex';
                hasMatch = true;
            } else {
                item.style.display = 'none';
            }
        });

        // 如果有匹配项，展开分类
        if (hasMatch) {
            category.classList.remove('collapsed');
        } else {
            category.classList.add('collapsed');
        }
    });
}

// 页面加载完成后初始化
window.addEventListener('load', init);

// 导出到全局（用于HTML onclick）
window.newFlow = newFlow;
window.openFlow = openFlow;
window.saveFlow = saveFlow;
window.exportFlow = exportFlow;
window.loadTemplate = loadTemplate;
window.copyNodes = copyNodes;
window.pasteNodes = pasteNodes;
window.deleteSelected = deleteSelected;
window.undo = undo;
window.redo = redo;
window.zoomIn = zoomIn;
window.zoomOut = zoomOut;
window.fitToScreen = fitToScreen;
window.toggleGrid = toggleGrid;
window.toggleMinimap = toggleMinimap;
window.runFlow = runFlow;
window.stopFlow = stopFlow;
window.stepFlow = stepFlow;
window.toggleDebug = toggleDebug;
window.groupSelected = groupSelected;
window.ungroupSelected = ungroupSelected;
window.createComment = createComment;
window.switchPanel = switchPanel;
window.switchLog = switchLog;
window.zoomImageIn = zoomImageIn;
window.zoomImageOut = zoomImageOut;
window.fitImage = fitImage;
window.toggleRoiMode = toggleRoiMode;
window.addBreakpoint = addBreakpoint;
window.removeBreakpoint = removeBreakpoint;
window.stepOver = stepOver;
window.stepInto = stepInto;
window.stepOut = stepOut;
window.continueExec = continueExec;
window.hideModal = hideModal;
window.copySelected = copyNodes;
window.toggleNodeEnabled = toggleNodeEnabled;
window.viewNodeOutput = viewNodeOutput;
window.updateNodeName = updateNodeName;
window.updateNodeParam = updateNodeParam;
window.toggleShortcutHint = toggleShortcutHint;