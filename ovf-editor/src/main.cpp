/**
 * @file main.cpp
 * @brief OpenVisionFlow 编辑器入口
 */

#include "editor_main_window.h"
#include "ovf/core/logger.h"
#include "ovf/core/node.h"
#include "ovf/algorithm/algorithm.h"

#include <QApplication>
#include <QFile>

namespace ovf {
namespace editor {

// 初始化内置节点
void initialize_builtin_nodes() {
    // 注册算法节点
    // 节点已在algorithm.cpp中通过OVF_REGISTER_NODE宏注册
}

} // namespace editor
} // namespace ovf

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    
    // 设置应用信息
    app.setApplicationName("OpenVisionFlow Editor");
    app.setApplicationVersion("0.1.0");
    app.setOrganizationName("OpenVisionFlow");
    
    // 加载样式
    QFile style_file(":/styles/default.qss");
    if (style_file.open(QFile::ReadOnly)) {
        QString style = style_file.readAll();
        app.setStyleSheet(style);
    }
    
    // 初始化日志
    ovf::Logger::instance().set_level(ovf::LogLevel::Debug);
    
    // 初始化内置节点
    ovf::editor::initialize_builtin_nodes();
    
    // 创建主窗口
    ovf::editor::EditorMainWindow window;
    window.show();
    
    OVF_INFO() << "OpenVisionFlow Editor started";
    
    return app.exec();
}