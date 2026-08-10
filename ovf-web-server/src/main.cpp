/**
 * @file main.cpp
 * @brief Web服务器主入口
 */

#include "ovf/web_server.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/algorithm.h"
#include <iostream>
#include <string>
#include <csignal>

using namespace ovf;
using namespace ovf::web;

// 全局服务器实例
std::shared_ptr<WebServer> g_server;

// 信号处理
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nShutting down server..." << std::endl;
        if (g_server) {
            g_server->stop();
        }
    }
}

// 打印帮助信息
void print_help() {
    std::cout << "OpenVisionFlow Web Server" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "Usage: ovf-web-server [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -p, --port <port>    Server port (default: 8080)" << std::endl;
    std::cout << "  -h, --help           Show this help message" << std::endl;
    std::cout << "  -v, --verbose        Enable verbose logging" << std::endl;
    std::cout << std::endl;
    std::cout << "API Endpoints:" << std::endl;
    std::cout << "  GET  /api/nodes          - Get all available node types" << std::endl;
    std::cout << "  GET  /api/node/{type}    - Get node details" << std::endl;
    std::cout << "  POST /api/flow/load      - Load flow file" << std::endl;
    std::cout << "  POST /api/flow/save      - Save flow file" << std::endl;
    std::cout << "  POST /api/flow/run       - Execute flow" << std::endl;
    std::cout << "  POST /api/flow/step      - Single step execution" << std::endl;
    std::cout << "  GET  /api/flow/status    - Get execution status" << std::endl;
    std::cout << "  GET  /api/image          - Get current image (base64)" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    int port = 8080;
    bool verbose = false;
    
    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            print_help();
            return 0;
        }
        
        if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            continue;
        }
        
        if (arg == "-p" || arg == "--port") {
            if (i + 1 < argc) {
                port = std::stoi(argv[++i]);
            } else {
                std::cerr << "Error: Missing port number" << std::endl;
                return 1;
            }
        }
    }
    
    // 设置日志级别
    if (verbose) {
        Logger::instance().set_level(LogLevel::Debug);
    } else {
        Logger::instance().set_level(LogLevel::Info);
    }
    
    std::cout << "OpenVisionFlow Web Server v" << VERSION << std::endl;
    std::cout << "================================" << std::endl;

    // 初始化算法模块
    OVF_INFO() << "Initializing algorithm module...";
    ovf::algorithm::initialize_algorithm_module();

    // 创建服务器
    g_server = std::make_shared<WebServer>();

    // 设置静态文件目录（Web编辑器前端）
    // 从构建目录 build/bin/Release 出发，需要 ../../../ovf-web-editor
    String static_dir = "../../../ovf-web-editor";
    g_server->set_static_dir(static_dir);
    OVF_INFO() << "Static files served from: " << static_dir;
    
    // 注册信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // 启动服务器
    OVF_INFO() << "Starting Web Server on port " << port;
    
    if (!g_server->start(port)) {
        OVF_ERROR() << "Failed to start Web Server";
        return 1;
    }
    
    std::cout << "Server started on port " << port << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << std::endl;
    std::cout << "API endpoints available:" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/nodes" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/node/{type}" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/flow/load" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/flow/save" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/flow/run" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/flow/step" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/flow/status" << std::endl;
    std::cout << "  http://localhost:" << port << "/api/image" << std::endl;
    
    // 等待服务器运行
    while (g_server->is_running()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    OVF_INFO() << "Server shutdown complete";
    return 0;
}