/**
 * @file main.cpp
 * @brief Web服务器主入口
 */

#include "ovf/web_server.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/algorithm.h"
#ifdef OVF_WITH_USER_NODES
#include "ovf/nodes/nodes.h"
#endif
#include <iostream>
#include <string>
#include <csignal>
#include <filesystem>
#include <system_error>

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
    std::cout << "  -d, --flows-dir <dir>  Where /api/files/* stores flows (default: flows)" << std::endl;
    std::cout << "  -w, --web-dir <dir>    Frontend dir to serve (default: auto-detect ovf-web-editor)" << std::endl;
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
    std::cout << "Web editor endpoints (ovf-web-editor):" << std::endl;
    std::cout << "  GET  /api/health         - Health probe" << std::endl;
    std::cout << "  POST /api/flows/execute  - Run a frontend-format flow" << std::endl;
    std::cout << "  POST /api/flows/step     - Run one node of the loaded flow" << std::endl;
    std::cout << "  POST /api/flows/stop     - Cancel the running flow" << std::endl;
    std::cout << "  POST /api/files/save     - Save flow JSON to the flows dir" << std::endl;
    std::cout << "  GET  /api/files/load     - Load flow JSON from the flows dir" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    int port = 8080;
    bool verbose = false;
    std::string flows_dir = "flows";  // /api/files/* 的落盘目录，可用 -d 覆盖
    std::string web_dir;              // 前端目录，留空则自动探测

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

        if (arg == "-d" || arg == "--flows-dir") {
            if (i + 1 < argc) {
                flows_dir = argv[++i];
            } else {
                std::cerr << "Error: Missing flows directory" << std::endl;
                return 1;
            }
        }

        if (arg == "-w" || arg == "--web-dir") {
            if (i + 1 < argc) {
                web_dir = argv[++i];
            } else {
                std::cerr << "Error: Missing web directory" << std::endl;
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

#ifdef OVF_WITH_USER_NODES
    // 初始化用户算子模块（ovf-nodes/，你自己写的算子）。
    //
    // 这里两句是**故意分开写的**，因为它们承的是不同的重（详见表见
    // docs/hardening/stage4_nodes.md）：

    OVF_INFO() << "Initializing user node module...";
    try {
        // ① 第 3 层（探测）：libovf-nodes.so 到底进没进这个进程。
        //
        //    这句用的是**弱引用**（见 ovf-nodes/include/ovf/nodes/nodes.h），
        //    所以它**不参与**"链接器要不要保留这个库"的判定。含义是：
        //    就算下面那句 ② 被谁删了、库真被 --as-needed 丢掉了，这句
        //    照样编得过、照样跑得起来，然后把原因和修法原样讲清楚。
        //    这是唯一能在"库已经不在了"的前提下还能开口的检查。
        ovf::nodes::require_user_node_module_loaded("ovf-web-server");

        // ② 第 2 层：ODR-use 一个**强**符号，链接器因此必须保留整个
        //    libovf-nodes.so，算子的静态初始化才有机会跑。
        //    顺带把第 3 层的另一半做掉：拿模块自己的算子清单比对真实
        //    注册表，少一个就在**启动时**抛可操作错误，绝不静默放行。
        ovf::nodes::initialize_user_nodes();
    } catch (const Exception& e) {
        // 用户算子出问题是**构建/配置**层面的事，不是运行期偶发 —— 起不来就是
        // 起不来，带着可操作的说明退出，比"起来了但少一半算子"强得多。
        std::cerr << "\n[FATAL] " << e.what() << "\n" << std::endl;
        return 1;
    }
#endif

    // 注册表摘要：节点类型数 / type_id 冲突 / 元数据问题数。
    // 放在这里而不是 initialize_algorithm_module() 里面，是因为后者在
    // ovf-algorithm/ —— 那个目录要对上游保持零 diff。
    // 冲突数非 0 意味着有算子被静默丢弃，这一行就是发现它的地方。
    ovf::NodeFactory::instance().log_summary();

    // 创建服务器
    g_server = std::make_shared<WebServer>();

    // 设置静态文件目录（Web编辑器前端）。
    // 原先写死 "../../../ovf-web-editor"，那是从 Windows 的 build/bin/Release
    // 数的层级；Linux 下二进制在 build/bin，多退了一层，实际指向
    // <仓库上级>/ovf-web-editor —— 目录不存在，前端整站 404。
    // 改成按候选列表探测，从哪个目录启动都能找到；也可以用 -w 显式指定。
    String static_dir = web_dir;
    if (static_dir.empty()) {
        const char* candidates[] = {
            "ovf-web-editor",            // 仓库根目录下启动
            "../ovf-web-editor",         // build/ 下启动
            "../../ovf-web-editor",      // build/bin/ 下启动（标准）
            "../../../ovf-web-editor",   // build/bin/Release/ 下启动
            "../../../../ovf-web-editor"
        };
        for (const char* c : candidates) {
            if (std::filesystem::is_directory(c)) {
                static_dir = c;
                break;
            }
        }
        if (static_dir.empty()) static_dir = "../../ovf-web-editor";
    }
    g_server->set_static_dir(static_dir);
    if (std::filesystem::is_directory(static_dir)) {
        OVF_INFO() << "Static files served from: "
                   << std::filesystem::absolute(static_dir).string();
    } else {
        OVF_WARN() << "Static dir not found: " << static_dir
                   << " (use -w to point at ovf-web-editor)";
    }

    // 流程文件存储目录（/api/files/save 与 /api/files/load 用）。
    // 目录不存在就建一个 —— 否则第一次保存流程会写失败。
    std::error_code ec;
    std::filesystem::create_directories(flows_dir, ec);
    if (ec) {
        OVF_ERROR() << "Failed to create flows dir '" << flows_dir << "': " << ec.message();
    } else {
        g_server->set_storage_dir(flows_dir);
        OVF_INFO() << "Flow files stored in: "
                   << std::filesystem::absolute(flows_dir).string();
    }

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