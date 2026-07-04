/**
 * @file editor_main_window.cpp
 * @brief 图形化编辑器实现
 */

#include "editor_main_window.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/algorithm.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <QStandardItemModel>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QTextEdit>
#include <QApplication>
#include <QDateTime>

#include <opencv2/opencv.hpp>
#include <QImage>

namespace ovf {
namespace editor {

// ============== FlowCanvasView ==============

FlowCanvasView::FlowCanvasView(QWidget* parent)
    : QGraphicsView(parent) {
    scene_ = new QGraphicsScene(this);
    setScene(scene_);
    
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::RubberBandDrag);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
    
    // 设置背景
    setBackgroundBrush(QBrush(QColor(240, 240, 240)));
    
    // 设置场景大小
    scene_->setSceneRect(-2000, -2000, 4000, 4000);
}

FlowCanvasView::~FlowCanvasView() {}

void FlowCanvasView::set_engine(FlowEngine::Ptr engine) {
    engine_ = engine;
    // 刷新画布
    scene_->clear();
    node_items_.clear();
    
    // 添加现有节点
    for (auto node : engine_->get_all_nodes()) {
        // 创建节点图形项
        // 暂时简化实现
    }
}

void FlowCanvasView::add_node(const String& type_id, const QPointF& pos) {
    if (!engine_) return;
    
    String instance_id = type_id + "_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    
    auto node = NodeFactory::instance().create(type_id, instance_id);
    if (!node) {
        OVF_ERROR() << "Failed to create node: " << type_id;
        return;
    }
    
    // 创建节点图形项
    auto* item = new QGraphicsRectItem(pos.x(), pos.y(), 120, 80);
    item->setBrush(QBrush(QColor(200, 220, 240)));
    item->setPen(QPen(QColor(100, 100, 100), 2));
    item->setData(0, QVariant::fromValue(QString::fromStdString(instance_id)));
    item->setFlag(QGraphicsItem::ItemIsMovable);
    item->setFlag(QGraphicsItem::ItemIsSelectable);
    
    // 添加标题
    auto* text_item = new QGraphicsTextItem(QString::fromStdString(node->info().name), item);
    text_item->setPos(pos.x() + 10, pos.y() + 10);
    
    scene_->addItem(item);
    node_items_[instance_id] = item;
    
    OVF_INFO() << "Node created: " << type_id << " at " << pos.x() << "," << pos.y();
}

void FlowCanvasView::remove_node(const String& instance_id) {
    auto it = node_items_.find(instance_id);
    if (it != node_items_.end()) {
        scene_->removeItem(it->second);
        node_items_.erase(it);
    }
}

void FlowCanvasView::select_node(const String& instance_id) {
    auto it = node_items_.find(instance_id);
    if (it != node_items_.end()) {
        scene_->clearSelection();
        it->second->setSelected(true);
        emit node_selected(instance_id);
    }
}

void FlowCanvasView::connect_nodes(const String& source_id, const String& source_port,
                                    const String& target_id, const String& target_port) {
    // 绘制连接线
    auto source_it = node_items_.find(source_id);
    auto target_it = node_items_.find(target_id);
    
    if (source_it != node_items_.end() && target_it != node_items_.end()) {
        auto* source_item = source_it->second;
        auto* target_item = target_it->second;
        
        QPointF source_pos = source_item->rect().right() + QPointF(10, 40);
        QPointF target_pos = target_item->rect().left() + QPointF(-10, 40);
        
        auto* line = new QGraphicsLineItem(source_pos.x(), source_pos.y(),
                                           target_pos.x(), target_pos.y());
        line->setPen(QPen(QColor(80, 80, 200), 2));
        scene_->addItem(line);
        
        emit connection_created(source_id, source_port, target_id, target_port);
    }
}

void FlowCanvasView::mousePressEvent(QMouseEvent* event) {
    QGraphicsView::mousePressEvent(event);
}

void FlowCanvasView::mouseMoveEvent(QMouseEvent* event) {
    QGraphicsView::mouseMoveEvent(event);
}

void FlowCanvasView::mouseReleaseEvent(QMouseEvent* event) {
    QGraphicsView::mouseReleaseEvent(event);
}

void FlowCanvasView::wheelEvent(QWheelEvent* event) {
    double factor = 1.15;
    if (event->angleDelta().y() > 0) {
        scale(factor, factor);
    } else {
        scale(1.0 / factor, 1.0 / factor);
    }
}

// ============== NodeToolboxPanel ==============

NodeToolboxPanel::NodeToolboxPanel(QWidget* parent)
    : QDockWidget("节点工具箱", parent) {
    setup_ui();
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
}

NodeToolboxPanel::~NodeToolboxPanel() {}

void NodeToolboxPanel::setup_ui() {
    tree_view_ = new QTreeView(this);
    model_ = new QStandardItemModel(this);
    model_->setHorizontalHeaderLabels({"节点类型"});
    
    tree_view_->setModel(model_);
    tree_view_->header()->setStretchLastSection(true);
    
    setWidget(tree_view_);
    
    // 连接双击事件
    connect(tree_view_, &QTreeView::doubleClicked, this, [this](const QModelIndex& index) {
        QString type_id = index.data().toString();
        emit node_create_requested(type_id.toStdString());
    });
}

void NodeToolboxPanel::refresh_nodes() {
    model_->clear();
    model_->setHorizontalHeaderLabels({"节点类型"});
    
    // 添加节点分类
    HashMap<String, QStandardItem*> categories;
    
    auto types = NodeFactory::instance().get_all_types();
    for (const auto& type_id : types) {
        auto info = NodeFactory::instance().get_info(type_id);
        if (!info) continue;
        
        // 获取或创建分类项
        QStandardItem* category_item = nullptr;
        auto cat_it = categories.find(info->category);
        if (cat_it != categories.end()) {
            category_item = cat_it->second;
        } else {
            category_item = new QStandardItem(QString::fromStdString(info->category));
            model_->appendRow(category_item);
            categories[info->category] = category_item;
        }
        
        // 添加节点项
        auto* node_item = new QStandardItem(QString::fromStdString(info->name));
        node_item->setData(QString::fromStdString(type_id));
        category_item->appendRow(node_item);
    }
    
    tree_view_->expandAll();
}

// ============== NodePropertyPanel ==============

NodePropertyPanel::NodePropertyPanel(QWidget* parent)
    : QDockWidget("节点属性", parent) {
    setup_ui();
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
}

NodePropertyPanel::~NodePropertyPanel() {}

void NodePropertyPanel::setup_ui() {
    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setFrameShape(QFrame::NoFrame);
    
    properties_widget_ = new QWidget();
    scroll_area_->setWidget(properties_widget_);
    
    setWidget(scroll_area_);
}

void NodePropertyPanel::set_node(INode::Ptr node) {
    current_node_ = node;
    update_properties();
}

void NodePropertyPanel::update_properties() {
    // 清除旧控件
    param_widgets_.clear();
    
    auto layout = new QVBoxLayout(properties_widget_);
    layout->setSpacing(10);
    
    if (!current_node_) {
        layout->addWidget(new QLabel("未选中节点"));
        properties_widget_->setLayout(layout);
        return;
    }
    
    // 显示节点信息
    auto* info_label = new QLabel(QString::fromStdString(current_node_->info().description));
    info_label->setWordWrap(true);
    layout->addWidget(info_label);
    
    // 添加参数控件
    for (const auto& param : current_node_->info().params) {
        auto* param_widget = create_param_widget(param);
        if (param_widget) {
            layout->addWidget(param_widget);
            param_widgets_[param.id] = param_widget;
        }
    }
    
    layout->addStretch();
    properties_widget_->setLayout(layout);
}

QWidget* NodePropertyPanel::create_param_widget(const ParamDef& param) {
    auto* widget = new QWidget();
    auto* layout = new QHBoxLayout(widget);
    layout->setContentsMargins(5, 5, 5, 5);
    
    auto* label = new QLabel(QString::fromStdString(param.name));
    label->setFixedWidth(100);
    layout->addWidget(label);
    
    QWidget* value_widget = nullptr;
    
    switch (param.type) {
        case DataType::Number: {
            auto* spinbox = new QDoubleSpinBox();
            spinbox->setRange(-999999, 999999);
            spinbox->setValue(current_node_->get_param(param.id).as_number());
            connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
                    this, [this, param_id = param.id](double value) {
                emit parameter_changed(param_id, Data(value));
            });
            value_widget = spinbox;
            break;
        }
        case DataType::String: {
            auto* line_edit = new QLineEdit();
            line_edit->setText(QString::fromStdString(current_node_->get_param(param.id).as_string()));
            connect(line_edit, &QLineEdit::textChanged, 
                    this, [this, param_id = param.id](const QString& text) {
                emit parameter_changed(param_id, Data(text.toStdString()));
            });
            value_widget = line_edit;
            break;
        }
        case DataType::Boolean: {
            auto* checkbox = new QCheckBox();
            checkbox->setChecked(current_node_->get_param(param.id).as_bool());
            connect(checkbox, &QCheckBox::toggled, 
                    this, [this, param_id = param.id](bool checked) {
                emit parameter_changed(param_id, Data(checked));
            });
            value_widget = checkbox;
            break;
        }
        default:
            value_widget = new QLabel("不支持类型");
    }
    
    if (value_widget) {
        layout->addWidget(value_widget);
    }
    
    widget->setLayout(layout);
    return widget;
}

// ============== ImagePreviewPanel ==============

ImagePreviewPanel::ImagePreviewPanel(QWidget* parent)
    : QDockWidget("图像预览", parent) {
    setup_ui();
    setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
}

ImagePreviewPanel::~ImagePreviewPanel() {}

void ImagePreviewPanel::setup_ui() {
    auto* main_widget = new QWidget();
    auto* layout = new QVBoxLayout(main_widget);
    
    // 选择控件
    auto* select_layout = new QHBoxLayout();
    node_combo_ = new QComboBox();
    port_combo_ = new QComboBox();
    select_layout->addWidget(new QLabel("节点:"));
    select_layout->addWidget(node_combo_);
    select_layout->addWidget(new QLabel("端口:"));
    select_layout->addWidget(port_combo_);
    layout->addLayout(select_layout);
    
    // 图像显示
    scroll_area_ = new QScrollArea();
    scroll_area_->setWidgetResizable(true);
    
    image_label_ = new QLabel();
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setMinimumSize(200, 200);
    image_label_->setStyleSheet("background-color: #333;");
    
    scroll_area_->setWidget(image_label_);
    layout->addWidget(scroll_area_);
    
    setWidget(main_widget);
}

void ImagePreviewPanel::set_image(const ImageData& image) {
    current_image_ = image;
    update_display();
}

void ImagePreviewPanel::set_node_output(INode::Ptr node, const String& port_id) {
    if (!node) return;
    
    auto output_data = node->get_output(port_id);
    if (output_data.is_image()) {
        set_image(output_data.as_image());
    }
}

void ImagePreviewPanel::clear() {
    image_label_->clear();
    image_label_->setText("无图像");
}

void ImagePreviewPanel::update_display() {
    if (current_image_.empty()) {
        clear();
        return;
    }
    
    // 转换ImageData到QImage
    cv::Mat mat;
    if (current_image_.channels == 1) {
        mat = cv::Mat(current_image_.height, current_image_.width, CV_8UC1, 
                      current_image_.data.data());
    } else if (current_image_.channels == 3) {
        mat = cv::Mat(current_image_.height, current_image_.width, CV_8UC3, 
                      current_image_.data.data());
        cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    } else if (current_image_.channels == 4) {
        mat = cv::Mat(current_image_.height, current_image_.width, CV_8UC4, 
                      current_image_.data.data());
        cv::cvtColor(mat, mat, cv::COLOR_BGRA2RGBA);
    }
    
    QImage qimage(mat.data, mat.cols, mat.rows, mat.step, 
                  current_image_.channels == 1 ? QImage::Format_Grayscale8 :
                  current_image_.channels == 3 ? QImage::Format_RGB888 :
                  QImage::Format_RGBA8888);
    
    image_label_->setPixmap(QPixmap::fromImage(qimage.copy()));
    image_label_->resize(current_image_.width, current_image_.height);
}

// ============== ExecutionControlPanel ==============

ExecutionControlPanel::ExecutionControlPanel(QWidget* parent)
    : QDockWidget("执行控制", parent) {
    setup_ui();
    setAllowedAreas(Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
}

ExecutionControlPanel::~ExecutionControlPanel() {}

void ExecutionControlPanel::setup_ui() {
    auto* main_widget = new QWidget();
    auto* layout = new QVBoxLayout(main_widget);
    
    // 控制按钮
    auto* btn_layout = new QHBoxLayout();
    
    run_btn_ = new QPushButton("▶ 运行");
    run_btn_->setIcon(QIcon(":/icons/run.png"));
    run_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 8px; }");
    
    stop_btn_ = new QPushButton("■ 停止");
    stop_btn_->setIcon(QIcon(":/icons/stop.png"));
    stop_btn_->setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 8px; }");
    
    step_btn_ = new QPushButton("→ 单步");
    step_btn_->setIcon(QIcon(":/icons/step.png"));
    
    trigger_btn_ = new QPushButton("⚡ 触发");
    trigger_btn_->setIcon(QIcon(":/icons/trigger.png"));
    
    btn_layout->addWidget(run_btn_);
    btn_layout->addWidget(stop_btn_);
    btn_layout->addWidget(step_btn_);
    btn_layout->addWidget(trigger_btn_);
    layout->addLayout(btn_layout);
    
    // 运行模式
    auto* mode_layout = new QHBoxLayout();
    mode_layout->addWidget(new QLabel("模式:"));
    mode_combo_ = new QComboBox();
    mode_combo_->addItem("单次", 0);
    mode_combo_->addItem("连续", 1);
    mode_combo_->addItem("触发", 2);
    mode_layout->addWidget(mode_combo_);
    mode_layout->addStretch();
    layout->addLayout(mode_layout);
    
    // 统计信息
    auto* stats_group = new QGroupBox("统计信息");
    auto* stats_layout = new QGridLayout(stats_group);
    
    status_label_ = new QLabel("就绪");
    runs_label_ = new QLabel("0");
    success_label_ = new QLabel("0");
    failed_label_ = new QLabel("0");
    time_label_ = new QLabel("0 ms");
    
    stats_layout->addWidget(new QLabel("状态:"), 0, 0);
    stats_layout->addWidget(status_label_, 0, 1);
    stats_layout->addWidget(new QLabel("总执行:"), 1, 0);
    stats_layout->addWidget(runs_label_, 1, 1);
    stats_layout->addWidget(new QLabel("成功:"), 2, 0);
    stats_layout->addWidget(success_label_, 2, 1);
    stats_layout->addWidget(new QLabel("失败:"), 3, 0);
    stats_layout->addWidget(failed_label_, 3, 1);
    stats_layout->addWidget(new QLabel("平均耗时:"), 4, 0);
    stats_layout->addWidget(time_label_, 4, 1);
    
    layout->addWidget(stats_group);
    layout->addStretch();
    
    setWidget(main_widget);
    
    // 连接信号
    connect(run_btn_, &QPushButton::clicked, this, &ExecutionControlPanel::run_requested);
    connect(stop_btn_, &QPushButton::clicked, this, &ExecutionControlPanel::stop_requested);
    connect(step_btn_, &QPushButton::clicked, this, &ExecutionControlPanel::step_requested);
    connect(trigger_btn_, &QPushButton::clicked, this, &ExecutionControlPanel::trigger_requested);
}

void ExecutionControlPanel::set_runner(FlowRunner::Ptr runner) {
    runner_ = runner;
    update_statistics();
}

void ExecutionControlPanel::update_statistics() {
    if (!runner_) return;
    
    runs_label_->setText(QString::number(runner_->total_runs()));
    success_label_->setText(QString::number(runner_->success_runs()));
    failed_label_->setText(QString::number(runner_->failed_runs()));
    time_label_->setText(QString::number(runner_->average_time_ms(), 'f', 2) + " ms");
    
    if (runner_->is_running()) {
        status_label_->setText("运行中");
        status_label_->setStyleSheet("color: green;");
    } else {
        status_label_->setText("就绪");
        status_label_->setStyleSheet("color: black;");
    }
}

// ============== OutputLogPanel ==============

OutputLogPanel::OutputLogPanel(QWidget* parent)
    : QDockWidget("输出日志", parent) {
    setup_ui();
    setAllowedAreas(Qt::BottomDockWidgetArea);
}

OutputLogPanel::~OutputLogPanel() {}

void OutputLogPanel::setup_ui() {
    auto* main_widget = new QWidget();
    auto* layout = new QVBoxLayout(main_widget);
    
    log_text_ = new QTextEdit();
    log_text_->setReadOnly(true);
    log_text_->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas; }");
    
    auto* btn_layout = new QHBoxLayout();
    clear_btn_ = new QPushButton("清除");
    auto_scroll_check_ = new QCheckBox("自动滚动");
    auto_scroll_check_->setChecked(true);
    
    btn_layout->addWidget(clear_btn_);
    btn_layout->addWidget(auto_scroll_check_);
    btn_layout->addStretch();
    
    layout->addWidget(log_text_);
    layout->addLayout(btn_layout);
    
    setWidget(main_widget);
    
    connect(clear_btn_, &QPushButton::clicked, this, &OutputLogPanel::clear);
}

void OutputLogPanel::append_log(const String& message, LogLevel level) {
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString level_str;
    QString color;
    
    switch (level) {
        case LogLevel::Trace: level_str = "TRACE"; color = "#808080"; break;
        case LogLevel::Debug: level_str = "DEBUG"; color = "#6080a0"; break;
        case LogLevel::Info: level_str = "INFO"; color = "#d4d4d4"; break;
        case LogLevel::Warning: level_str = "WARN"; color = "#ffa500"; break;
        case LogLevel::Error: level_str = "ERROR"; color = "#ff4444"; break;
        case LogLevel::Fatal: level_str = "FATAL"; color = "#ff0000"; break;
        default: level_str = "INFO"; color = "#d4d4d4";
    }
    
    QString html = QString("<span style='color: %1;'>[%2] [%3] %4</span>")
                   .arg(color, timestamp, level_str, QString::fromStdString(message));
    
    log_text_->append(html);
    
    if (auto_scroll_check_->isChecked()) {
        log_text_->verticalScrollBar()->setValue(log_text_->verticalScrollBar()->maximum());
    }
}

void OutputLogPanel::clear() {
    log_text_->clear();
}

// ============== EditorMainWindow ==============

EditorMainWindow::EditorMainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setup_ui();
    setup_menus();
    setup_toolbars();
    setup_dock_widgets();
    setup_connections();
    
    // 初始化引擎和运行器
    engine_ = std::make_shared<FlowEngine>();
    runner_ = std::make_shared<FlowRunner>();
    runner_->set_engine(engine_);
    
    canvas_view_->set_engine(engine_);
    control_panel_->set_runner(runner_);
    toolbox_panel_->refresh_nodes();
    
    resize(1280, 800);
    update_title();
}

EditorMainWindow::~EditorMainWindow() {}

void EditorMainWindow::setup_ui() {
    setWindowTitle("OpenVisionFlow Editor");
    setWindowIcon(QIcon(":/icons/app.png"));
    
    // 中央画布
    canvas_view_ = new FlowCanvasView(this);
    setCentralWidget(canvas_view_);
}

void EditorMainWindow::setup_menus() {
    // 文件菜单
    auto* file_menu = menuBar()->addMenu("文件");
    
    file_menu->addAction("新建", this, &EditorMainWindow::on_new_flow, QKeySequence::New);
    file_menu->addAction("打开...", this, &EditorMainWindow::on_open_flow, QKeySequence::Open);
    file_menu->addAction("保存", this, &EditorMainWindow::on_save_flow, QKeySequence::Save);
    file_menu->addAction("另存为...", this, &EditorMainWindow::on_save_flow_as, QKeySequence::SaveAs);
    file_menu->addSeparator();
    file_menu->addAction("退出", this, &QMainWindow::close, QKeySequence::Quit);
    
    // 编辑菜单
    auto* edit_menu = menuBar()->addMenu("编辑");
    edit_menu->addAction("撤销", this, []{}, QKeySequence::Undo);
    edit_menu->addAction("重做", this, []{}, QKeySequence::Redo);
    edit_menu->addSeparator();
    edit_menu->addAction("删除", this, []{}, QKeySequence::Delete);
    
    // 运行菜单
    auto* run_menu = menuBar()->addMenu("运行");
    run_menu->addAction("运行", this, &EditorMainWindow::on_run, QKeySequence(Qt::Key_F5));
    run_menu->addAction("停止", this, &EditorMainWindow::on_stop, QKeySequence(Qt::SHIFT + Qt::Key_F5));
    run_menu->addAction("单步", this, &EditorMainWindow::on_step, QKeySequence(Qt::Key_F10));
    run_menu->addAction("触发", this, &EditorMainWindow::on_trigger, QKeySequence(Qt::Key_F6));
    
    // 视图菜单
    auto* view_menu = menuBar()->addMenu("视图");
    view_menu->addAction(toolbox_panel_->toggleViewAction());
    view_menu->addAction(property_panel_->toggleViewAction());
    view_menu->addAction(preview_panel_->toggleViewAction());
    view_menu->addAction(control_panel_->toggleViewAction());
    view_menu->addAction(log_panel_->toggleViewAction());
    
    // 帮助菜单
    auto* help_menu = menuBar()->addMenu("帮助");
    help_menu->addAction("关于", this, []{
        QMessageBox::about(nullptr, "关于 OpenVisionFlow",
            "OpenVisionFlow v0.1.0\n开源工业机器视觉平台\n\n© 2026 OpenVisionFlow Team");
    });
}

void EditorMainWindow::setup_toolbars() {
    // 文件工具栏
    file_toolbar_ = addToolBar("文件");
    file_toolbar_->addAction(QIcon(":/icons/new.png"), "新建", this, &EditorMainWindow::on_new_flow);
    file_toolbar_->addAction(QIcon(":/icons/open.png"), "打开", this, &EditorMainWindow::on_open_flow);
    file_toolbar_->addAction(QIcon(":/icons/save.png"), "保存", this, &EditorMainWindow::on_save_flow);
    
    // 运行工具栏
    run_toolbar_ = addToolBar("运行");
    run_toolbar_->addAction(QIcon(":/icons/run.png"), "运行", this, &EditorMainWindow::on_run);
    run_toolbar_->addAction(QIcon(":/icons/stop.png"), "停止", this, &EditorMainWindow::on_stop);
    run_toolbar_->addAction(QIcon(":/icons/step.png"), "单步", this, &EditorMainWindow::on_step);
    run_toolbar_->addAction(QIcon(":/icons/trigger.png"), "触发", this, &EditorMainWindow::on_trigger);
}

void EditorMainWindow::setup_dock_widgets() {
    toolbox_panel_ = new NodeToolboxPanel(this);
    addDockWidget(Qt::LeftDockWidgetArea, toolbox_panel_);
    
    property_panel_ = new NodePropertyPanel(this);
    addDockWidget(Qt::LeftDockWidgetArea, property_panel_);
    
    preview_panel_ = new ImagePreviewPanel(this);
    addDockWidget(Qt::BottomDockWidgetArea, preview_panel_);
    
    control_panel_ = new ExecutionControlPanel(this);
    addDockWidget(Qt::TopDockWidgetArea, control_panel_);
    
    log_panel_ = new OutputLogPanel(this);
    addDockWidget(Qt::BottomDockWidgetArea, log_panel_);
    
    // 布局调整
    splitDockWidget(preview_panel_, log_panel_, Qt::Horizontal);
}

void EditorMainWindow::setup_connections() {
    connect(toolbox_panel_, &NodeToolboxPanel::node_create_requested,
            canvas_view_, &FlowCanvasView::add_node);
    
    connect(canvas_view_, &FlowCanvasView::node_selected,
            this, &EditorMainWindow::on_node_selected);
    
    connect(property_panel_, &NodePropertyPanel::parameter_changed,
            this, &EditorMainWindow::on_parameter_changed);
    
    connect(control_panel_, &ExecutionControlPanel::run_requested,
            this, &EditorMainWindow::on_run);
    connect(control_panel_, &ExecutionControlPanel::stop_requested,
            this, &EditorMainWindow::on_stop);
    connect(control_panel_, &ExecutionControlPanel::step_requested,
            this, &EditorMainWindow::on_step);
    connect(control_panel_, &ExecutionControlPanel::trigger_requested,
            this, &EditorMainWindow::on_trigger);
    
    runner_->set_result_callback([this](const FlowResult& result) {
        on_flow_result(result);
    });
    
    engine_->set_node_state_callback([this](const String& node_id, NodeState state) {
        on_node_state_changed(node_id, state);
    });
}

void EditorMainWindow::set_engine(FlowEngine::Ptr engine) {
    engine_ = engine;
    canvas_view_->set_engine(engine);
    runner_->set_engine(engine);
}

void EditorMainWindow::set_runner(FlowRunner::Ptr runner) {
    runner_ = runner;
    runner_->set_engine(engine_);
    control_panel_->set_runner(runner_);
}

bool EditorMainWindow::open_flow(const String& filepath) {
    auto result = engine_->load_from_file(filepath);
    if (result.is_failure()) {
        QMessageBox::warning(this, "打开失败", 
            QString::fromStdString("无法打开文件: " + result.message()));
        return false;
    }
    
    current_filepath_ = filepath;
    modified_ = false;
    canvas_view_->set_engine(engine_);
    toolbox_panel_->refresh_nodes();
    update_title();
    log_panel_->append_log("流程已加载: " + filepath);
    
    return true;
}

bool EditorMainWindow::save_flow(const String& filepath) {
    auto result = engine_->save_to_file(filepath);
    if (result.is_failure()) {
        QMessageBox::warning(this, "保存失败",
            QString::fromStdString("无法保存文件: " + result.message()));
        return false;
    }
    
    current_filepath_ = filepath;
    modified_ = false;
    update_title();
    log_panel_->append_log("流程已保存: " + filepath);
    
    return true;
}

void EditorMainWindow::new_flow() {
    engine_->clear();
    current_filepath_.clear();
    modified_ = false;
    canvas_view_->set_engine(engine_);
    update_title();
    log_panel_->append_log("新建流程");
}

void EditorMainWindow::on_new_flow() {
    if (modified_) {
        auto reply = QMessageBox::question(this, "保存?", "是否保存当前流程?",
            QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
        
        if (reply == QMessageBox::Cancel) return;
        if (reply == QMessageBox::Yes) {
            on_save_flow();
        }
    }
    
    new_flow();
}

void EditorMainWindow::on_open_flow() {
    QString filepath = QFileDialog::getOpenFileName(this, "打开流程",
        "", "流程文件 (*.json *.yaml);;所有文件 (*)");
    
    if (!filepath.isEmpty()) {
        open_flow(filepath.toStdString());
    }
}

void EditorMainWindow::on_save_flow() {
    if (current_filepath_.empty()) {
        on_save_flow_as();
    } else {
        save_flow(current_filepath_);
    }
}

void EditorMainWindow::on_save_flow_as() {
    QString filepath = QFileDialog::getSaveFileName(this, "保存流程",
        "", "流程文件 (*.json);;所有文件 (*)");
    
    if (!filepath.isEmpty()) {
        save_flow(filepath.toStdString());
    }
}

void EditorMainWindow::on_run() {
    if (!engine_ || !runner_) return;
    
    auto mode = static_cast<FlowRunner::RunMode>(control_panel_->findChild<QComboBox*>()->currentData().toInt());
    
    auto result = runner_->start(mode);
    if (result.is_failure()) {
        log_panel_->append_log("启动失败: " + result.message(), LogLevel::Error);
    } else {
        log_panel_->append_log("流程开始运行", LogLevel::Info);
    }
}

void EditorMainWindow::on_stop() {
    if (runner_) {
        runner_->stop();
        log_panel_->append_log("流程已停止", LogLevel::Info);
    }
}

void EditorMainWindow::on_step() {
    if (!engine_) return;
    
    engine_->step_begin();
    auto node_result = engine_->step_next();
    
    if (node_result.is_success()) {
        auto node = node_result.value();
        auto exec_result = engine_->run_node(node->instance_id(), context_);
        
        if (exec_result.success) {
            log_panel_->append_log("单步执行: " + node->info().name + " 成功", LogLevel::Info);
            preview_panel_->set_node_output(node, "image");
        } else {
            log_panel_->append_log("单步执行失败: " + exec_result.error_message, LogLevel::Error);
        }
    }
    
    if (!engine_->step_has_more()) {
        engine_->step_end();
        log_panel_->append_log("流程执行完毕", LogLevel::Info);
    }
}

void EditorMainWindow::on_trigger() {
    if (runner_) {
        runner_->trigger();
        log_panel_->append_log("触发执行", LogLevel::Info);
    }
}

void EditorMainWindow::on_node_selected(const String& instance_id) {
    auto node = engine_->get_node(instance_id);
    property_panel_->set_node(node);
    
    // 更新图像预览
    if (node) {
        preview_panel_->set_node_output(node, "image");
    }
}

void EditorMainWindow::on_node_created(const String& type_id) {
    modified_ = true;
    update_title();
    log_panel_->append_log("创建节点: " + type_id);
}

void EditorMainWindow::on_parameter_changed(const String& param_id, const Data& value) {
    if (property_panel_->current_node()) {
        property_panel_->current_node()->set_param(param_id, value);
        modified_ = true;
        update_title();
    }
}

void EditorMainWindow::on_flow_result(const FlowResult& result) {
    control_panel_->update_statistics();
    
    if (result.success) {
        log_panel_->append_log("流程执行成功, 耗时: " + 
            std::to_string(result.total_time_us / 1000.0) + " ms", LogLevel::Info);
    } else {
        log_panel_->append_log("流程执行失败: " + result.error_message + 
            " (节点: " + result.failed_node_id + ")", LogLevel::Error);
    }
}

void EditorMainWindow::on_node_state_changed(const String& node_id, NodeState state) {
    QString state_str;
    switch (state) {
        case NodeState::Idle: state_str = "空闲"; break;
        case NodeState::Running: state_str = "运行"; break;
        case NodeState::Success: state_str = "成功"; break;
        case NodeState::Failed: state_str = "失败"; break;
        case NodeState::Disabled: state_str = "禁用"; break;
    }
    
    log_panel_->append_log("节点 [" + node_id + "] 状态: " + state_str.toStdString());
}

void EditorMainWindow::update_title() {
    QString title = "OpenVisionFlow Editor";
    
    if (!current_filepath_.empty()) {
        title += " - " + QString::fromStdString(current_filepath_);
    } else {
        title += " - 未命名流程";
    }
    
    if (modified_) {
        title += " [*]";
    }
    
    setWindowTitle(title);
    setWindowModified(modified_);
}

void EditorMainWindow::update_status() {
    statusBar()->showMessage("就绪");
}

} // namespace editor
} // namespace ovf