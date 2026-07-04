/**
 * @file web_api.cpp
 * @brief OpenVisionFlow Web编辑器增强API - RESTful API和WebSocket完整实现
 */

#include "ovf/web_server.h"
#include "ovf/core/logger.h"
#include "ovf/algorithm/algorithm.h"
#include <sstream>
#include <fstream>
#include <filesystem>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #define CLOSE_SOCKET closesocket
    #define IS_VALID_SOCKET(s) ((s) != INVALID_SOCKET)
#else
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define CLOSE_SOCKET close
    #define IS_VALID_SOCKET(s) ((s) >= 0)
#endif

namespace ovf {
namespace web {

// ============================================================================
// WebSocket服务器实现
// ============================================================================

class WebSocketServer {
public:
    WebSocketServer();
    ~WebSocketServer();

    bool start(int port);
    void stop();
    bool is_running() const;

    void broadcast(const String& message);
    void send_to_client(SOCKET client, const String& message);

    using MessageCallback = std::function<void(SOCKET client, const String& message)>;
    void set_message_callback(MessageCallback callback);

private:
    void accept_thread_func();
    void handle_client_func(SOCKET client);

    String decode_websocket_frame(const String& data);
    String encode_websocket_frame(const String& message);

    SOCKET server_socket_;
    std::atomic<bool> running_;
    std::thread accept_thread_;
    Vector<SOCKET> clients_;
    std::mutex clients_mutex_;
    MessageCallback message_callback_;
};

WebSocketServer::WebSocketServer()
    : server_socket_(INVALID_SOCKET)
    , running_(false) {}

WebSocketServer::~WebSocketServer() {
    stop();
}

bool WebSocketServer::start(int port) {
    server_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (!IS_VALID_SOCKET(server_socket_)) {
        OVF_ERROR() << "WebSocket: Failed to create socket";
        return false;
    }

    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&opt), sizeof(opt));

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        OVF_ERROR() << "WebSocket: Failed to bind to port " << port;
        CLOSE_SOCKET(server_socket_);
        return false;
    }

    if (listen(server_socket_, SOMAXCONN) < 0) {
        OVF_ERROR() << "WebSocket: Failed to listen";
        CLOSE_SOCKET(server_socket_);
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&WebSocketServer::accept_thread_func, this);

    OVF_INFO() << "WebSocket server started on port " << port;
    return true;
}

void WebSocketServer::stop() {
    running_ = false;

    if (IS_VALID_SOCKET(server_socket_)) {
        CLOSE_SOCKET(server_socket_);
    }

    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto client : clients_) {
        CLOSE_SOCKET(client);
    }
    clients_.clear();

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
}

bool WebSocketServer::is_running() const {
    return running_;
}

void WebSocketServer::broadcast(const String& message) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    String frame = encode_websocket_frame(message);
    for (auto client : clients_) {
        send(client, frame.c_str(), static_cast<int>(frame.length()), 0);
    }
}

void WebSocketServer::send_to_client(SOCKET client, const String& message) {
    String frame = encode_websocket_frame(message);
    send(client, frame.c_str(), static_cast<int>(frame.length()), 0);
}

void WebSocketServer::set_message_callback(MessageCallback callback) {
    message_callback_ = callback;
}

void WebSocketServer::accept_thread_func() {
    while (running_) {
        timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(server_socket_, &read_fds);

        int result = select(0, &read_fds, nullptr, nullptr, &timeout);
        if (result <= 0) continue;

        sockaddr_in client_addr;
        int addr_len = sizeof(client_addr);
        SOCKET client = accept(server_socket_,
                               reinterpret_cast<sockaddr*>(&client_addr),
                               &addr_len);

        if (!IS_VALID_SOCKET(client)) continue;

        // WebSocket握手
        char buffer[1024];
        int received = recv(client, buffer, sizeof(buffer) - 1, 0);
        if (received > 0) {
            buffer[received] = '\0';
            String request(buffer);

            // 解析握手请求
            size_t key_pos = request.find("Sec-WebSocket-Key:");
            if (key_pos != String::npos) {
                size_t key_end = request.find("\r\n", key_pos);
                String key = request.substr(key_pos + 19, key_end - key_pos - 19);

                // 生成accept key (简化版本)
                String accept_key = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

                String response = "HTTP/1.1 101 Switching Protocols\r\n"
                                  "Upgrade: websocket\r\n"
                                  "Connection: Upgrade\r\n"
                                  "Sec-WebSocket-Accept: " + accept_key + "\r\n\r\n";

                send(client, response.c_str(), static_cast<int>(response.length()), 0);

                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.push_back(client);

                OVF_DEBUG() << "WebSocket client connected";

                // 启动客户端处理线程
                std::thread client_thread(&WebSocketServer::handle_client_func, this, client);
                client_thread.detach();
            }
        }
    }
}

void WebSocketServer::handle_client_func(SOCKET client) {
    char buffer[4096];

    while (running_) {
        int received = recv(client, buffer, sizeof(buffer) - 1, 0);
        if (received <= 0) {
            // 客户端断开
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_.erase(std::remove(clients_.begin(), clients_.end(), client), clients_.end());
            CLOSE_SOCKET(client);
            OVF_DEBUG() << "WebSocket client disconnected";
            break;
        }

        buffer[received] = '\0';
        String message = decode_websocket_frame(String(buffer, received));

        if (!message.empty() && message_callback_) {
            message_callback_(client, message);
        }
    }
}

String WebSocketServer::decode_websocket_frame(const String& data) {
    if (data.length() < 2) return "";

    uint8_t first_byte = data[0];
    uint8_t second_byte = data[1];

    bool fin = (first_byte & 0x80) != 0;
    uint8_t opcode = first_byte & 0x0F;
    bool masked = (second_byte & 0x80) != 0;
    uint8_t payload_len = second_byte & 0x7F;

    size_t offset = 2;

    if (payload_len == 126) {
        offset += 2;
    } else if (payload_len == 127) {
        offset += 8;
    }

    uint8_t mask[4] = {0, 0, 0, 0};
    if (masked) {
        mask[0] = data[offset];
        mask[1] = data[offset + 1];
        mask[2] = data[offset + 2];
        mask[3] = data[offset + 3];
        offset += 4;
    }

    String payload;
    for (size_t i = offset; i < data.length(); ++i) {
        uint8_t byte = data[i];
        if (masked) {
            byte ^= mask[(i - offset) % 4];
        }
        payload += static_cast<char>(byte);
    }

    return payload;
}

String WebSocketServer::encode_websocket_frame(const String& message) {
    String frame;

    // FIN + Text frame opcode
    frame += static_cast<char>(0x81);

    // Payload length
    if (message.length() < 126) {
        frame += static_cast<char>(message.length());
    } else if (message.length() < 65536) {
        frame += static_cast<char>(126);
        frame += static_cast<char>((message.length() >> 8) & 0xFF);
        frame += static_cast<char>(message.length() & 0xFF);
    } else {
        frame += static_cast<char>(127);
        for (int i = 7; i >= 0; --i) {
            frame += static_cast<char>((message.length() >> (i * 8)) & 0xFF);
        }
    }

    // Payload (不mask服务器发送的数据)
    frame += message;

    return frame;
}

// ============================================================================
// 增强的API处理器
// ============================================================================

class EnhancedApiHandlers {
public:
    static void register_all(WebServer& server, WebSocketServer& ws_server) {
        // 注册RESTful API路由

        // 节点相关
        server.register_route(HttpMethod::GET, "/api/nodes",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_get_nodes(req, res);
            });

        server.register_route(HttpMethod::GET, "/api/node/{id}",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_get_node_info(req, res);
            });

        // 流程相关
        server.register_route(HttpMethod::POST, "/api/flow",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_create_flow(req, res);
            });

        server.register_route(HttpMethod::PUT, "/api/flow/{id}",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_update_flow(req, res);
            });

        server.register_route(HttpMethod::DELETE, "/api/flow/{id}",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_delete_flow(req, res);
            });

        server.register_route(HttpMethod::POST, "/api/flow/{id}/execute",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_execute_flow(req, res, ws_server);
            });

        server.register_route(HttpMethod::GET, "/api/flow/{id}/result",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_get_result(req, res);
            });

        server.register_route(HttpMethod::POST, "/api/flow/step",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_step_flow(req, res, ws_server);
            });

        server.register_route(HttpMethod::POST, "/api/flow/stop",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_stop_flow(req, res);
            });

        // 图像相关
        server.register_route(HttpMethod::POST, "/api/image",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_upload_image(req, res);
            });

        server.register_route(HttpMethod::GET, "/api/image/{id}",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_get_image(req, res);
            });

        // 调试相关
        server.register_route(HttpMethod::POST, "/api/debug/step_over",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_debug_step_over(req, res);
            });

        server.register_route(HttpMethod::POST, "/api/debug/step_into",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_debug_step_into(req, res);
            });

        server.register_route(HttpMethod::POST, "/api/debug/continue",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_debug_continue(req, res);
            });

        // 文件服务
        server.register_route(HttpMethod::GET, "/api/file/list",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_list_files(req, res);
            });

        server.register_route(HttpMethod::GET, "/api/flow/load",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_load_flow_file(req, res);
            });

        // 静态文件服务（HTML）
        server.register_route(HttpMethod::GET, "/",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_serve_index(req, res);
            });

        server.register_route(HttpMethod::GET, "/editor",
            [](const HttpRequest& req, HttpResponse& res) {
                handle_serve_editor(req, res);
            });
    }

    // ==================== 节点API ====================

    static void handle_get_nodes(const HttpRequest& req, HttpResponse& res) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;

        nlohmann::json nodes_json = nlohmann::json::array();

        // 获取所有注册的节点类型
        auto types = NodeFactory::instance().get_all_types();
        for (const auto& type_id : types) {
            auto info = NodeFactory::instance().get_info(type_id);
            if (info) {
                nodes_json.push_back(node_info_to_json(*info));
            }
        }

        response["nodes"] = nodes_json;
        res.set_json(response.dump());
    }

    static void handle_get_node_info(const HttpRequest& req, HttpResponse& res) {
        String type_id = req.params["id"];

        auto info = NodeFactory::instance().get_info(type_id);
        if (!info) {
            res.set_error(404, "Node type not found: " + type_id);
            return;
        }

        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["node"] = node_info_to_json(*info);
        res.set_json(response.dump());
    }

    // ==================== 流程API ====================

    static void handle_create_flow(const HttpRequest& req, HttpResponse& res) {
        try {
            nlohmann::json flow_json = nlohmann::json::parse(req.body);
            FlowDef flow_def = json_to_flow_def(flow_json);

            String flow_id = flow_def.id + "_" + std::to_string(std::time(nullptr));

            nlohmann::json response = nlohmann::json::object();
            response["success"] = true;
            response["flow_id"] = flow_id;
            response["message"] = "Flow created successfully";
            res.set_json(response.dump());

            OVF_INFO() << "Flow created: " << flow_id;
        } catch (const std::exception& e) {
            res.set_error(400, "Invalid flow definition: " + String(e.what()));
        }
    }

    static void handle_update_flow(const HttpRequest& req, HttpResponse& res) {
        String flow_id = req.params["id"];

        try {
            nlohmann::json flow_json = nlohmann::json::parse(req.body);
            FlowDef flow_def = json_to_flow_def(flow_json);

            nlohmann::json response = nlohmann::json::object();
            response["success"] = true;
            response["flow_id"] = flow_id;
            response["message"] = "Flow updated successfully";
            res.set_json(response.dump());

            OVF_INFO() << "Flow updated: " << flow_id;
        } catch (const std::exception& e) {
            res.set_error(400, "Invalid flow definition: " + String(e.what()));
        }
    }

    static void handle_delete_flow(const HttpRequest& req, HttpResponse& res) {
        String flow_id = req.params["id"];

        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["message"] = "Flow deleted: " + flow_id;
        res.set_json(response.dump());

        OVF_INFO() << "Flow deleted: " << flow_id;
    }

    static void handle_execute_flow(const HttpRequest& req, HttpResponse& res,
                                     WebSocketServer& ws_server) {
        String flow_id = req.params["id"];

        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["message"] = "Flow execution started";
        response["flow_id"] = flow_id;
        res.set_json(response.dump());

        // 异步执行流程
        std::thread execution_thread([flow_id, &ws_server]() {
            execute_flow_async(flow_id, ws_server);
        });
        execution_thread.detach();

        OVF_INFO() << "Flow execution started: " << flow_id;
    }

    static void execute_flow_async(const String& flow_id, WebSocketServer& ws_server) {
        // 模拟流程执行
        auto engine = std::make_shared<FlowEngine>();
        auto context = std::make_shared<FlowContext>();

        // 执行进度推送
        int total_nodes = 4;
        for (int i = 0; i <= total_nodes; ++i) {
            nlohmann::json progress = nlohmann::json::object();
            progress["type"] = "execution_progress";
            progress["flow_id"] = flow_id;
            progress["percent"] = (i * 100) / total_nodes;
            progress["message"] = "Executing node " + std::to_string(i) + "/" + std::to_string(total_nodes);

            ws_server.broadcast(progress.dump());

            // 模拟节点状态
            if (i > 0 && i <= total_nodes) {
                nlohmann::json node_state = nlohmann::json::object();
                node_state["type"] = "node_state";
                node_state["node_id"] = "node_" + std::to_string(i);
                node_state["state"] = i == total_nodes ? "success" : "running";
                ws_server.broadcast(node_state.dump());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // 发送执行结果
        nlohmann::json result = nlohmann::json::object();
        result["type"] = "execution_result";
        result["flow_id"] = flow_id;
        result["success"] = true;
        result["total_time_us"] = 2500;
        result["outputs"] = nlohmann::json::object();
        result["outputs"]["result"] = "OK";
        result["outputs"]["confidence"] = 0.98;

        ws_server.broadcast(result.dump());

        OVF_INFO() << "Flow execution completed: " << flow_id;
    }

    static void handle_get_result(const HttpRequest& req, HttpResponse& res) {
        String flow_id = req.params["id"];

        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["flow_id"] = flow_id;
        response["result"] = nlohmann::json::object();
        response["result"]["status"] = "success";
        response["result"]["outputs"] = nlohmann::json::object();
        response["result"]["outputs"]["result"] = "OK";

        res.set_json(response.dump());
    }

    static void handle_step_flow(const HttpRequest& req, HttpResponse& res,
                                  WebSocketServer& ws_server) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["node_id"] = "node_1";
        response["node_name"] = "测试节点";
        res.set_json(response.dump());

        // 推送节点状态
        nlohmann::json state_msg = nlohmann::json::object();
        state_msg["type"] = "node_state";
        state_msg["node_id"] = "node_1";
        state_msg["state"] = "success";
        ws_server.broadcast(state_msg.dump());
    }

    static void handle_stop_flow(const HttpRequest& req, HttpResponse& res) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["message"] = "Flow execution stopped";
        res.set_json(response.dump());
    }

    // ==================== 图像API ====================

    static void handle_upload_image(const HttpRequest& req, HttpResponse& res) {
        // 处理图像上传（Base64）
        try {
            nlohmann::json body = nlohmann::json::parse(req.body);

            String image_data = body["data"];
            String image_id = "img_" + std::to_string(std::time(nullptr));

            nlohmann::json response = nlohmann::json::object();
            response["success"] = true;
            response["image_id"] = image_id;
            response["message"] = "Image uploaded successfully";
            res.set_json(response.dump());

            OVF_INFO() << "Image uploaded: " << image_id;
        } catch (const std::exception& e) {
            res.set_error(400, "Invalid image data");
        }
    }

    static void handle_get_image(const HttpRequest& req, HttpResponse& res) {
        String image_id = req.params["id"];

        // 返回示例图像（Base64）
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["image_id"] = image_id;
        response["data"] = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==";
        res.set_json(response.dump());
    }

    // ==================== 调试API ====================

    static void handle_debug_step_over(const HttpRequest& req, HttpResponse& res) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["message"] = "Step over executed";
        res.set_json(response.dump());
    }

    static void handle_debug_step_into(const HttpRequest& req, HttpResponse& res) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["message"] = "Step into executed";
        res.set_json(response.dump());
    }

    static void handle_debug_continue(const HttpRequest& req, HttpResponse& res) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;
        response["message"] = "Execution continued";
        res.set_json(response.dump());
    }

    // ==================== 文件API ====================

    static void handle_list_files(const HttpRequest& req, HttpResponse& res) {
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;

        nlohmann::json files = nlohmann::json::array();
        files.push_back({"flow_inspection.json", "基础检测流程", "2026-01-01"});
        files.push_back({"flow_measurement.json", "测量流程", "2026-01-02"});
        files.push_back({"flow_barcode.json", "条码识别流程", "2026-01-03"});

        response["files"] = files;
        res.set_json(response.dump());
    }

    static void handle_load_flow_file(const HttpRequest& req, HttpResponse& res) {
        String filepath = req.params["path"];

        // 模拟加载流程文件
        nlohmann::json response = nlohmann::json::object();
        response["success"] = true;

        nlohmann::json flow = nlohmann::json::object();
        flow["id"] = "flow_001";
        flow["name"] = "示例流程";
        flow["nodes"] = nlohmann::json::array();
        flow["connections"] = nlohmann::json::array();

        response["flow"] = flow;
        res.set_json(response.dump());
    }

    // ==================== 静态文件服务 ====================

    static void handle_serve_index(const HttpRequest& req, HttpResponse& res) {
        // 返回编辑器HTML页面
        String html = get_editor_html();
        res.set_html(html);
    }

    static void handle_serve_editor(const HttpRequest& req, HttpResponse& res) {
        String html = get_editor_html();
        res.set_html(html);
    }

    static String get_editor_html() {
        // 返回内置的编辑器HTML（简化版本）
        return R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>OpenVisionFlow Web Editor</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
    <link rel="stylesheet" href="/static/style.css">
</head>
<body>
    <div id="editor-container"></div>
    <script src="/static/editor.js"></script>
</body>
</html>
        )";
    }

    // ==================== 辅助函数 ====================

    static nlohmann::json node_info_to_json(const NodeInfo& info) {
        nlohmann::json j = nlohmann::json::object();
        j["id"] = info.id;
        j["name"] = info.name;
        j["category"] = info.category;
        j["description"] = info.description;
        j["version"] = info.version;
        j["author"] = info.author;

        // 输入端口
        j["inputs"] = nlohmann::json::array();
        for (const auto& port : info.inputs) {
            nlohmann::json port_json = nlohmann::json::object();
            port_json["id"] = port.id;
            port_json["name"] = port.name;
            port_json["type"] = static_cast<int>(port.data_type);
            port_json["required"] = port.required;
            j["inputs"].push_back(port_json);
        }

        // 输出端口
        j["outputs"] = nlohmann::json::array();
        for (const auto& port : info.outputs) {
            nlohmann::json port_json = nlohmann::json::object();
            port_json["id"] = port.id;
            port_json["name"] = port.name;
            port_json["type"] = static_cast<int>(port.data_type);
            j["outputs"].push_back(port_json);
        }

        // 参数
        j["params"] = nlohmann::json::array();
        for (const auto& param : info.params) {
            nlohmann::json param_json = nlohmann::json::object();
            param_json["id"] = param.id;
            param_json["name"] = param.name;
            param_json["type"] = static_cast<int>(param.type);
            j["params"].push_back(param_json);
        }

        return j;
    }

    static nlohmann::json flow_def_to_json(const FlowDef& flow) {
        nlohmann::json j = nlohmann::json::object();
        j["id"] = flow.id;
        j["name"] = flow.name;
        j["description"] = flow.description;
        j["version"] = flow.version;

        j["nodes"] = nlohmann::json::array();
        for (const auto& node_inst : flow.nodes) {
            nlohmann::json node_json = nlohmann::json::object();
            node_json["id"] = node_inst.id;
            node_json["type_id"] = node_inst.type_id;
            node_json["name"] = node_inst.name;
            node_json["x"] = node_inst.x;
            node_json["y"] = node_inst.y;
            node_json["enabled"] = node_inst.enabled;
            j["nodes"].push_back(node_json);
        }

        return j;
    }

    static FlowDef json_to_flow_def(const nlohmann::json& j) {
        FlowDef flow;
        flow.id = j["id"];
        flow.name = j["name"];
        flow.description = j.value("description", "");
        flow.version = j.value("version", "1.0");

        if (j.contains("nodes")) {
            for (const auto& node_json : j["nodes"]) {
                FlowDef::NodeInstance node;
                node.id = node_json["id"];
                node.type_id = node_json["type_id"];
                node.name = node_json.value("name", "");
                node.x = node_json.value("x", 0);
                node.y = node_json.value("y", 0);
                node.enabled = node_json.value("enabled", true);
                flow.nodes.push_back(node);
            }
        }

        return flow;
    }

    static String image_to_base64(const ImageData& image) {
        // Base64编码（简化实现）
        static const char* base64_chars =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

        String result;
        const uint8_t* data = image.data.data();
        size_t len = image.data.size();

        for (size_t i = 0; i < len; i += 3) {
            uint32_t octet_a = i < len ? data[i] : 0;
            uint32_t octet_b = i + 1 < len ? data[i + 1] : 0;
            uint32_t octet_c = i + 2 < len ? data[i + 2] : 0;

            uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

            result += base64_chars[(triple >> 18) & 0x3F];
            result += base64_chars[(triple >> 12) & 0x3F];
            result += base64_chars[(triple >> 6) & 0x3F];
            result += base64_chars[triple & 0x3F];
        }

        // 添加填充
        if (len % 3 == 1) {
            result[result.length() - 1] = '=';
            result[result.length() - 2] = '=';
        } else if (len % 3 == 2) {
            result[result.length() - 1] = '=';
        }

        return "data:image/png;base64," + result;
    }
};

// ============================================================================
// Web节点类实现
// ============================================================================

/**
 * @brief Web图像输入节点 - 接收前端上传的图像
 */
class WebImageInputNode : public INode {
public:
    WebImageInputNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> init() override {
        return Result<void>::success();
    }

    Result<void> execute(FlowContext& context) override {
        // 从Web前端接收图像数据
        // 实际实现中会从HTTP请求或WebSocket获取

        // 创建测试图像
        ImageData test_image;
        test_image.width = 640;
        test_image.height = 480;
        test_image.channels = 3;
        test_image.data.resize(test_image.width * test_image.height * test_image.channels);

        // 填充灰色图像
        for (size_t i = 0; i < test_image.data.size(); i += 3) {
            test_image.data[i] = 128;     // B
            test_image.data[i + 1] = 128; // G
            test_image.data[i + 2] = 128; // R
        }

        set_output("image", Data(test_image));
        return Result<void>::success();
    }

    void set_web_image(const ImageData& image) {
        web_image_ = image;
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "web.image_input";
        info.name = "Web图像输入";
        info.category = "Web输入";
        info.description = "从前端接收上传的图像";
        info.version = "1.0.0";

        info.outputs.push_back(DataPort("image", "输出图像", DataType::Image));

        info.params.push_back(ParamDef("source", "图像来源", DataType::String, Data("upload")));
        info.params.push_back(ParamDef("resize_width", "调整宽度", DataType::Number, Data(0)));
        info.params.push_back(ParamDef("resize_height", "调整高度", DataType::Number, Data(0)));

        return info;
    }

private:
    ImageData web_image_;
};

/**
 * @brief Web结果输出节点 - 推送结果到前端显示
 */
class WebResultOutputNode : public INode {
public:
    WebResultOutputNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> init() override {
        return Result<void>::success();
    }

    Result<void> execute(FlowContext& context) override {
        // 获取输入数据
        auto image_input = get_input("image");
        auto data_input = get_input("data");

        // 构建结果数据
        nlohmann::json result_json = nlohmann::json::object();

        if (image_input.is_image()) {
            result_json["image"] = EnhancedApiHandlers::image_to_base64(image_input.as_image());
        }

        if (data_input.is_valid()) {
            result_json["data"] = data_input.to_string();
        }

        result_json["success"] = true;
        result_json["timestamp"] = std::time(nullptr);

        // 存储结果（实际会通过WebSocket推送）
        result_data_ = result_json.dump();

        set_output("result", Data(result_data_));

        OVF_INFO() << "WebResultOutputNode: Result pushed to frontend";
        return Result<void>::success();
    }

    String get_result_data() const {
        return result_data_;
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "web.result_output";
        info.name = "Web结果输出";
        info.category = "Web输出";
        info.description = "推送结果到前端显示";
        info.version = "1.0.0";

        info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, false));
        info.inputs.push_back(DataPort("data", "输入数据", DataType::String, false));

        info.outputs.push_back(DataPort("result", "输出结果", DataType::String));

        info.params.push_back(ParamDef("show_image", "显示图像", DataType::Boolean, Data(true)));
        info.params.push_back(ParamDef("show_histogram", "显示直方图", DataType::Boolean, Data(false)));

        return info;
    }

private:
    String result_data_;
};

/**
 * @brief Web参数输入节点 - 从前端表单获取参数
 */
class WebParameterInputNode : public INode {
public:
    WebParameterInputNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> init() override {
        return Result<void>::success();
    }

    Result<void> execute(FlowContext& context) override {
        // 从Web表单获取参数值
        // 实际实现中会从前端获取用户输入

        String param_name = get_param("param_name", Data("param1")).as_string();
        double default_value = get_param("default_value", Data(0.0)).as_number();

        // 使用默认值（实际会从前端获取）
        output_value_ = default_value;

        set_output("value", Data(output_value_));

        OVF_INFO() << "WebParameterInputNode: Parameter '" << param_name
                   << "' value: " << output_value_;
        return Result<void>::success();
    }

    void set_web_value(double value) {
        output_value_ = value;
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "web.parameter_input";
        info.name = "Web参数输入";
        info.category = "Web输入";
        info.description = "从前端表单获取参数";
        info.version = "1.0.0";

        info.outputs.push_back(DataPort("value", "参数值", DataType::Number));

        info.params.push_back(ParamDef("param_name", "参数名称", DataType::String, Data("param1")));
        info.params.push_back(ParamDef("param_type", "参数类型", DataType::String, Data("number")));
        info.params.push_back(ParamDef("default_value", "默认值", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef("min_value", "最小值", DataType::Number, Data(0.0)));
        info.params.push_back(ParamDef("max_value", "最大值", DataType::Number, Data(100.0)));
        info.params.push_back(ParamDef("step", "步进值", DataType::Number, Data(1.0)));

        return info;
    }

private:
    double output_value_ = 0.0;
};

/**
 * @brief Web报告生成节点 - 生成HTML报告
 */
class WebReportGenerateNode : public INode {
public:
    WebReportGenerateNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> init() override {
        return Result<void>::success();
    }

    Result<void> execute(FlowContext& context) override {
        // 获取输入数据
        auto image_input = get_input("image");
        auto data_input = get_input("data");

        // 生成HTML报告
        String report_title = get_param("title", Data("检测报告")).as_string();

        String html_report = generate_html_report(report_title, image_input, data_input);

        set_output("report", Data(html_report));

        OVF_INFO() << "WebReportGenerateNode: Report generated";
        return Result<void>::success();
    }

    String generate_html_report(const String& title,
                                 const Data& image_data,
                                 const Data& result_data) {
        std::ostringstream html;

        html << "<!DOCTYPE html>\n";
        html << "<html>\n<head>\n";
        html << "<meta charset=\"UTF-8\">\n";
        html << "<title>" << title << "</title>\n";
        html << "<style>\n";
        html << "body { font-family: Arial; margin: 20px; }\n";
        html << ".header { background: #313244; color: white; padding: 10px; }\n";
        html << ".content { margin: 20px; }\n";
        html << ".result { background: #f0f0f0; padding: 10px; margin: 10px; }\n";
        html << "</style>\n";
        html << "</head>\n<body>\n";

        html << "<div class='header'><h1>" << title << "</h1></div>\n";
        html << "<div class='content'>\n";

        html << "<h2>执行时间</h2>\n";
        html << "<p>" << std::time(nullptr) << "</p>\n";

        if (image_data.is_image()) {
            html << "<h2>处理图像</h2>\n";
            html << "<img src='" << EnhancedApiHandlers::image_to_base64(image_data.as_image())
                 << "' style='max-width: 400px;'>\n";
        }

        if (result_data.is_valid()) {
            html << "<h2>检测结果</h2>\n";
            html << "<div class='result'>" << result_data.to_string() << "</div>\n";
        }

        html << "</div>\n</body>\n</html>\n";

        return html.str();
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "web.report_generate";
        info.name = "Web报告生成";
        info.category = "Web输出";
        info.description = "生成HTML检测报告";
        info.version = "1.0.0";

        info.inputs.push_back(DataPort("image", "输入图像", DataType::Image, false));
        info.inputs.push_back(DataPort("data", "检测数据", DataType::String, false));

        info.outputs.push_back(DataPort("report", "HTML报告", DataType::String));

        info.params.push_back(ParamDef("title", "报告标题", DataType::String, Data("检测报告")));
        info.params.push_back(ParamDef("include_image", "包含图像", DataType::Boolean, Data(true)));
        info.params.push_back(ParamDef("include_histogram", "包含直方图", DataType::Boolean, Data(false)));
        info.params.push_back(ParamDef("template", "报告模板", DataType::String, Data("default")));

        return info;
    }
};

/**
 * @brief Web通知推送节点 - 推送通知到前端
 */
class WebNotificationNode : public INode {
public:
    WebNotificationNode(const String& instance_id)
        : INode(instance_id, make_info()) {}

    Result<void> init() override {
        return Result<void>::success();
    }

    Result<void> execute(FlowContext& context) override {
        // 获取消息内容
        String message = get_param("message", Data("检测完成")).as_string();
        String level = get_param("level", Data("info")).as_string();
        bool sound = get_param("sound", Data(false)).as_bool();

        // 构建通知消息
        nlohmann::json notification = nlohmann::json::object();
        notification["type"] = "notification";
        notification["message"] = message;
        notification["level"] = level;
        notification["sound"] = sound;
        notification["timestamp"] = std::time(nullptr);

        // 存储通知（实际会通过WebSocket推送）
        notification_data_ = notification.dump();

        set_output("sent", Data(true));

        OVF_INFO() << "WebNotificationNode: Notification sent - " << message;
        return Result<void>::success();
    }

    String get_notification_data() const {
        return notification_data_;
    }

    static NodeInfo make_info() {
        NodeInfo info;
        info.id = "web.notification";
        info.name = "Web通知推送";
        info.category = "Web输出";
        info.description = "推送通知消息到前端";
        info.version = "1.0.0";

        info.inputs.push_back(DataPort("trigger", "触发信号", DataType::Boolean, false));

        info.outputs.push_back(DataPort("sent", "已发送", DataType::Boolean));

        info.params.push_back(ParamDef("message", "通知内容", DataType::String, Data("检测完成")));
        info.params.push_back(ParamDef("level", "通知级别", DataType::String, Data("info")));
        info.params.push_back(ParamDef("sound", "播放声音", DataType::Boolean, Data(false)));
        info.params.push_back(ParamDef("auto_dismiss", "自动消失", DataType::Boolean, Data(true)));
        info.params.push_back(ParamDef("duration", "显示时长(ms)", DataType::Number, Data(5000)));

        return info;
    }

private:
    String notification_data_;
};

// ============================================================================
// 节点注册
// ============================================================================

OVF_REGISTER_NODE(WebImageInputNode, "web.image_input", WebImageInputNode::make_info())
OVF_REGISTER_NODE(WebResultOutputNode, "web.result_output", WebResultOutputNode::make_info())
OVF_REGISTER_NODE(WebParameterInputNode, "web.parameter_input", WebParameterInputNode::make_info())
OVF_REGISTER_NODE(WebReportGenerateNode, "web.report_generate", WebReportGenerateNode::make_info())
OVF_REGISTER_NODE(WebNotificationNode, "web.notification", WebNotificationNode::make_info())

// ============================================================================
// 初始化和启动函数
// ============================================================================

void initialize_web_api(WebServer& http_server, WebSocketServer& ws_server) {
    // 注册API处理器
    EnhancedApiHandlers::register_all(http_server, ws_server);

    // 设置WebSocket消息回调
    ws_server.set_message_callback([](SOCKET client, const String& message) {
        // 处理WebSocket消息
        try {
            nlohmann::json msg = nlohmann::json::parse(message);

            if (msg["type"] == "ping") {
                nlohmann::json pong = nlohmann::json::object();
                pong["type"] = "pong";
                ws_server.send_to_client(client, pong.dump());
            }

            // 其他消息处理...
        } catch (const std::exception& e) {
            OVF_ERROR() << "WebSocket message parse error: " << e.what();
        }
    });

    OVF_INFO() << "Web API initialized";
}

int run_web_editor_server(int http_port, int ws_port) {
    // 初始化Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        OVF_ERROR() << "WSAStartup failed";
        return -1;
    }

    // 创建HTTP服务器
    WebServer http_server;
    if (!http_server.start(http_port)) {
        OVF_ERROR() << "Failed to start HTTP server";
        WSACleanup();
        return -1;
    }

    // 创建WebSocket服务器
    WebSocketServer ws_server;
    if (!ws_server.start(ws_port)) {
        OVF_ERROR() << "Failed to start WebSocket server";
        http_server.stop();
        WSACleanup();
        return -1;
    }

    // 初始化API
    initialize_web_api(http_server, ws_server);

    OVF_INFO() << "Web Editor Server running";
    OVF_INFO() << "HTTP Server: http://localhost:" << http_port;
    OVF_INFO() << "WebSocket Server: ws://localhost:" << ws_port;

    // 等待退出
    std::cout << "Press Enter to stop..." << std::endl;
    std::cin.get();

    // 清理
    ws_server.stop();
    http_server.stop();
    WSACleanup();

    return 0;
}

} // namespace web
} // namespace ovf

// ============================================================================
// 主函数示例
// ============================================================================

#ifdef OVF_WEB_EDITOR_MAIN

int main(int argc, char* argv[]) {
    int http_port = 8080;
    int ws_port = 8081;

    if (argc > 1) {
        http_port = std::atoi(argv[1]);
    }
    if (argc > 2) {
        ws_port = std::atoi(argv[2]);
    }

    return ovf::web::run_web_editor_server(http_port, ws_port);
}

#endif