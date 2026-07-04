/**
 * @file editor_main_window.h
 * @brief OpenVisionFlow 图形化编辑器主窗口
 */

#pragma once

#include <QMainWindow>
#include <QDockWidget>
#include <QTreeView>
#include <QGraphicsView>
#include <QTableView>
#include <QLabel>
#include <QToolBar>
#include <QStatusBar>
#include <QMenuBar>
#include <QSplitter>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>

#include "ovf/core/flow.h"
#include "ovf/core/node.h"

namespace ovf {
namespace editor {

/**
 * @brief 流程画布视图
 */
class FlowCanvasView : public QGraphicsView {
    Q_OBJECT
    
public:
    explicit FlowCanvasView(QWidget* parent = nullptr);
    ~FlowCanvasView();
    
    void set_engine(FlowEngine::Ptr engine);
    FlowEngine::Ptr engine() const { return engine_; }
    
    // 节点操作
    void add_node(const String& type_id, const QPointF& pos);
    void remove_node(const String& instance_id);
    void select_node(const String& instance_id);
    
    // 连接操作
    void connect_nodes(const String& source_id, const String& source_port,
                       const String& target_id, const String& target_port);
    void disconnect_nodes(const String& source_id, const String& source_port,
                          const String& target_id, const String& target_port);
    
signals:
    void node_selected(const String& instance_id);
    void connection_created(const String& source_id, const String& source_port,
                            const String& target_id, const String& target_port);
    
protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    
private:
    FlowEngine::Ptr engine_;
    QGraphicsScene* scene_;
    
    // 节点图形项映射
    HashMap<String, QGraphicsItem*> node_items_;
    
    // 拖拽状态
    bool dragging_node_ = false;
    String drag_node_id_;
    QPointF drag_start_pos_;
    
    // 连接拖拽
    bool connecting_ = false;
    String connect_source_id_;
    String connect_source_port_;
    QGraphicsLineItem* temp_line_ = nullptr;
};

/**
 * @brief 节点工具箱面板
 */
class NodeToolboxPanel : public QDockWidget {
    Q_OBJECT
    
public:
    explicit NodeToolboxPanel(QWidget* parent = nullptr);
    ~NodeToolboxPanel();
    
    void refresh_nodes();
    
signals:
    void node_create_requested(const String& type_id);
    
private:
    void setup_ui();
    
    QTreeView* tree_view_;
    QStandardItemModel* model_;
};

/**
 * @brief 节点属性面板
 */
class NodePropertyPanel : public QDockWidget {
    Q_OBJECT
    
public:
    explicit NodePropertyPanel(QWidget* parent = nullptr);
    ~NodePropertyPanel();
    
    void set_node(INode::Ptr node);
    INode::Ptr current_node() const { return current_node_; }
    
signals:
    void parameter_changed(const String& param_id, const Data& value);
    
private:
    void setup_ui();
    void update_properties();
    QWidget* create_param_widget(const ParamDef& param);
    
    INode::Ptr current_node_;
    
    QWidget* properties_widget_;
    QScrollArea* scroll_area_;
    HashMap<String, QWidget*> param_widgets_;
};

/**
 * @brief 图像预览面板
 */
class ImagePreviewPanel : public QDockWidget {
    Q_OBJECT
    
public:
    explicit ImagePreviewPanel(QWidget* parent = nullptr);
    ~ImagePreviewPanel();
    
    void set_image(const ImageData& image);
    void set_node_output(INode::Ptr node, const String& port_id);
    void clear();
    
private:
    void setup_ui();
    void update_display();
    
    QLabel* image_label_;
    QScrollArea* scroll_area_;
    QComboBox* node_combo_;
    QComboBox* port_combo_;
    
    ImageData current_image_;
};

/**
 * @brief 执行控制面板
 */
class ExecutionControlPanel : public QDockWidget {
    Q_OBJECT
    
public:
    explicit ExecutionControlPanel(QWidget* parent = nullptr);
    ~ExecutionControlPanel();
    
    void set_runner(FlowRunner::Ptr runner);
    void update_statistics();
    
signals:
    void run_requested();
    void stop_requested();
    void step_requested();
    void trigger_requested();
    
private:
    void setup_ui();
    
    FlowRunner::Ptr runner_;
    
    QPushButton* run_btn_;
    QPushButton* stop_btn_;
    QPushButton* step_btn_;
    QPushButton* trigger_btn_;
    
    QLabel* status_label_;
    QLabel* runs_label_;
    QLabel* success_label_;
    QLabel* failed_label_;
    QLabel* time_label_;
    
    QComboBox* mode_combo_;
};

/**
 * @brief 输出日志面板
 */
class OutputLogPanel : public QDockWidget {
    Q_OBJECT
    
public:
    explicit OutputLogPanel(QWidget* parent = nullptr);
    ~OutputLogPanel();
    
    void append_log(const String& message, LogLevel level = LogLevel::Info);
    void clear();
    
private:
    void setup_ui();
    
    QTextEdit* log_text_;
    QPushButton* clear_btn_;
    QCheckBox* auto_scroll_check_;
};

/**
 * @brief 编辑器主窗口
 */
class EditorMainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    explicit EditorMainWindow(QWidget* parent = nullptr);
    ~EditorMainWindow();
    
    void set_engine(FlowEngine::Ptr engine);
    void set_runner(FlowRunner::Ptr runner);
    
    // 文件操作
    bool open_flow(const String& filepath);
    bool save_flow(const String& filepath);
    void new_flow();
    
private slots:
    void on_new_flow();
    void on_open_flow();
    void on_save_flow();
    void on_save_flow_as();
    
    void on_run();
    void on_stop();
    void on_step();
    void on_trigger();
    
    void on_node_selected(const String& instance_id);
    void on_node_created(const String& type_id);
    void on_parameter_changed(const String& param_id, const Data& value);
    
    void on_flow_result(const FlowResult& result);
    void on_node_state_changed(const String& node_id, NodeState state);
    
private:
    void setup_ui();
    void setup_menus();
    void setup_toolbars();
    void setup_dock_widgets();
    void setup_connections();
    
    void update_title();
    void update_status();
    
    // 核心组件
    FlowEngine::Ptr engine_;
    FlowRunner::Ptr runner_;
    FlowContext context_;
    
    // UI组件
    FlowCanvasView* canvas_view_;
    NodeToolboxPanel* toolbox_panel_;
    NodePropertyPanel* property_panel_;
    ImagePreviewPanel* preview_panel_;
    ExecutionControlPanel* control_panel_;
    OutputLogPanel* log_panel_;
    
    // 工具栏
    QToolBar* file_toolbar_;
    QToolBar* edit_toolbar_;
    QToolBar* run_toolbar_;
    
    // 当前文件
    String current_filepath_;
    bool modified_ = false;
};

} // namespace editor
} // namespace ovf