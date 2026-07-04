/**
 * @file web_server.cpp
 * @brief Web服务器实现 - 使用Windows Winsock API
 */

#include "ovf/web_server.h"
#include "ovf/core/logger.h"
#include <sstream>
#include <algorithm>
#include <cctype>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #define CLOSE_SOCKET closesocket
    #define IS_VALID_SOCKET(s) ((s) != INVALID_SOCKET)
    #define SOCKET_ERROR_CODE SOCKET_ERROR
#else
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define CLOSE_SOCKET close
    #define IS_VALID_SOCKET(s) ((s) >= 0)
    #define SOCKET_ERROR_CODE -1
#endif

namespace ovf {
namespace web {

// WebSocket初始化计数器
static int winsock_init_count = 0;

// 内部工具函数
namespace detail {

// Trim字符串
String trim(const String& str) {
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == String::npos) return "";
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

// 分割字符串
std::vector<String> split(const String& str, char delimiter) {
    std::vector<String> parts;
    std::stringstream ss(str);
    String part;
    while (std::getline(ss, part, delimiter)) {
        parts.push_back(trim(part));
    }
    return parts;
}

// URL编码字符转义
char hex_to_char(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

} // namespace detail

// HttpResponse实现
void HttpResponse::set_json(const String& json) {
    body = json;
    content_type = "application/json";
}

void HttpResponse::set_error(int code, const String& message) {
    status_code = code;
    status_text = (code == 200) ? "OK" : 
                  (code == 400) ? "Bad Request" :
                  (code == 404) ? "Not Found" :
                  (code == 500) ? "Internal Server Error" : "Error";
    
    nlohmann::json error_json = nlohmann::json::object();
    error_json["success"] = false;
    error_json["error"] = message;
    error_json["code"] = code;
    set_json(error_json.dump());
}

void HttpResponse::set_html(const String& html) {
    body = html;
    content_type = "text/html; charset=utf-8";
}

// WebServer实现
WebServer::WebServer()
    : server_socket_(INVALID_SOCKET)
    , port_(0)
    , running_(false)
{
}

WebServer::~WebServer() {
    stop();
}

bool WebServer::init_winsock() {
#ifdef _WIN32
    if (winsock_init_count == 0) {
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            OVF_ERROR() << "WSAStartup failed: " << result;
            return false;
        }
    }
    winsock_init_count++;
    return true;
#else
    return true;
#endif
}

void WebServer::cleanup_winsock() {
#ifdef _WIN32
    winsock_init_count--;
    if (winsock_init_count == 0) {
        WSACleanup();
    }
#endif
}

bool WebServer::create_server_socket(int port) {
    server_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (!IS_VALID_SOCKET(server_socket_)) {
        OVF_ERROR() << "Failed to create socket";
        return false;
    }
    
    // 允许地址重用
    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, 
               reinterpret_cast<const char*>(&opt), sizeof(opt));
    
    // 绑定地址
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);
    
    if (bind(server_socket_, reinterpret_cast<sockaddr*>(&server_addr), 
             sizeof(server_addr)) == SOCKET_ERROR_CODE) {
        OVF_ERROR() << "Failed to bind socket to port " << port;
        CLOSE_SOCKET(server_socket_);
        server_socket_ = INVALID_SOCKET;
        return false;
    }
    
    // 开始监听
    if (listen(server_socket_, SOMAXCONN) == SOCKET_ERROR_CODE) {
        OVF_ERROR() << "Failed to listen on socket";
        CLOSE_SOCKET(server_socket_);
        server_socket_ = INVALID_SOCKET;
        return false;
    }
    
    port_ = port;
    OVF_INFO() << "Server listening on port " << port;
    return true;
}

bool WebServer::start(int port) {
    if (running_) {
        OVF_WARN() << "Server is already running";
        return true;
    }
    
    if (!init_winsock()) {
        return false;
    }
    
    if (!create_server_socket(port)) {
        cleanup_winsock();
        return false;
    }
    
    running_ = true;
    
    // 创建流程引擎
    flow_engine_ = std::make_shared<ovf::FlowEngine>();
    
    // 创建执行监控器
    execution_monitor_ = std::make_shared<FlowExecutionMonitor>();
    
    // 注册API路由
    ovf::web::ApiHandlers::register_all(*this);
    
    // 启动接受连接线程
    accept_thread_ = std::thread(&WebServer::accept_connections, this);
    
    OVF_INFO() << "WebServer started successfully on port " << port;
    return true;
}

void WebServer::stop() {
    if (!running_) return;
    
    running_ = false;
    
    // 关闭服务器socket
    if (IS_VALID_SOCKET(server_socket_)) {
        CLOSE_SOCKET(server_socket_);
        server_socket_ = INVALID_SOCKET;
    }
    
    // 等待线程结束
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    
    cleanup_winsock();
    OVF_INFO() << "WebServer stopped";
}

bool WebServer::is_running() const {
    return running_;
}

void WebServer::register_route(HttpMethod method, const String& path, ApiHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    switch (method) {
        case HttpMethod::GET:
            get_routes_[path] = handler;
            break;
        case HttpMethod::POST:
            post_routes_[path] = handler;
            break;
        case HttpMethod::PUT:
            put_routes_[path] = handler;
            break;
        case HttpMethod::DELETE:
            delete_routes_[path] = handler;
            break;
        default:
            break;
    }
}

void WebServer::set_flow_engine(ovf::FlowEngine::Ptr engine) {
    std::lock_guard<std::mutex> lock(mutex_);
    flow_engine_ = engine;
}

void WebServer::accept_connections() {
    while (running_) {
        // 设置超时，避免阻塞在accept上
        timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(server_socket_, &read_fds);
        
        int select_result = select(0, &read_fds, nullptr, nullptr, &timeout);
        if (select_result == SOCKET_ERROR_CODE) {
            if (running_) {
                OVF_ERROR() << "select() failed";
            }
            continue;
        }
        
        if (select_result == 0) {
            // 超时，继续循环检查running状态
            continue;
        }
        
        // 接受新连接
        sockaddr_in client_addr;
        int client_addr_len = sizeof(client_addr);
        SOCKET client_socket = accept(server_socket_, 
                                       reinterpret_cast<sockaddr*>(&client_addr), 
                                       &client_addr_len);
        
        if (!IS_VALID_SOCKET(client_socket)) {
            if (running_) {
                OVF_ERROR() << "accept() failed";
            }
            continue;
        }
        
        // 获取客户端IP
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        OVF_DEBUG() << "Client connected: " << client_ip;
        
        // 处理客户端（在新线程中）
        std::thread client_thread(&WebServer::handle_client, this, client_socket);
        client_thread.detach();
    }
}

void WebServer::handle_client(SOCKET client_socket) {
    // 接收请求
    char buffer[4096];
    int bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    
    if (bytes_received <= 0) {
        CLOSE_SOCKET(client_socket);
        return;
    }
    
    buffer[bytes_received] = '\0';
    String raw_request(buffer);
    
    // 解析请求
    HttpRequest request;
    HttpResponse response;
    
    if (!parse_request(raw_request, request)) {
        response.set_error(400, "Invalid HTTP request");
    } else {
        // 匹配路由并处理
        ApiHandler handler;
        if (match_route(request, handler)) {
            handler(request, response);
        } else {
            response.set_error(404, "Not found: " + request.path);
        }
    }
    
    // 发送响应
    String response_str = build_response(response);
    send(client_socket, response_str.c_str(), static_cast<int>(response_str.length()), 0);
    
    CLOSE_SOCKET(client_socket);
}

bool WebServer::parse_request(const String& raw_request, HttpRequest& request) {
    // 分离请求行和头部/正文
    size_t header_end = raw_request.find("\r\n\r\n");
    if (header_end == String::npos) {
        header_end = raw_request.find("\n\n");
    }
    
    String header_section;
    if (header_end != String::npos) {
        header_section = raw_request.substr(0, header_end);
        request.body = raw_request.substr(header_end + 4); // 或 +2
    } else {
        header_section = raw_request;
    }
    
    // 分离行
    auto lines = detail::split(header_section, '\n');
    if (lines.empty()) return false;
    
    // 解析请求行
    auto request_line = detail::split(lines[0], ' ');
    if (request_line.size() < 2) return false;
    
    String method_str = request_line[0];
    if (method_str == "GET") request.method = HttpMethod::GET;
    else if (method_str == "POST") request.method = HttpMethod::POST;
    else if (method_str == "PUT") request.method = HttpMethod::PUT;
    else if (method_str == "DELETE") request.method = HttpMethod::DELETE;
    else if (method_str == "OPTIONS") request.method = HttpMethod::OPTIONS;
    else return false;
    
    // 解析路径和查询字符串
    String full_path = request_line[1];
    size_t query_pos = full_path.find('?');
    if (query_pos != String::npos) {
        request.path = full_path.substr(0, query_pos);
        request.query_string = full_path.substr(query_pos + 1);
        parse_query_params(request.query_string, request.params);
    } else {
        request.path = full_path;
    }
    
    // 解析头部
    for (size_t i = 1; i < lines.size(); ++i) {
        const String& line = lines[i];
        if (line.empty()) continue;
        
        size_t colon_pos = line.find(':');
        if (colon_pos != String::npos) {
            String key = detail::trim(line.substr(0, colon_pos));
            String value = detail::trim(line.substr(colon_pos + 1));
            request.headers[key] = value;
        }
    }
    
    return true;
}

String WebServer::build_response(const HttpResponse& response) {
    std::ostringstream oss;
    
    oss << "HTTP/1.1 " << response.status_code << " " << response.status_text << "\r\n";
    oss << "Content-Type: " << response.content_type << "\r\n";
    oss << "Content-Length: " << response.body.length() << "\r\n";
    oss << "Connection: close\r\n";
    oss << "Access-Control-Allow-Origin: *\r\n";
    oss << "Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS\r\n";
    oss << "Access-Control-Allow-Headers: Content-Type\r\n";
    
    // 自定义头部
    for (const auto& header : response.headers) {
        oss << header.first << ": " << header.second << "\r\n";
    }
    
    oss << "\r\n";
    oss << response.body;
    
    return oss.str();
}

bool WebServer::match_route(HttpRequest& request, ApiHandler& handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::map<String, String> captures;
    
    std::map<String, ApiHandler>* routes = nullptr;
    switch (request.method) {
        case HttpMethod::GET: routes = &get_routes_; break;
        case HttpMethod::POST: routes = &post_routes_; break;
        case HttpMethod::PUT: routes = &put_routes_; break;
        case HttpMethod::DELETE: routes = &delete_routes_; break;
        case HttpMethod::OPTIONS: 
            // OPTIONS请求特殊处理，直接返回成功
            return false;
        default: return false;
    }
    
    if (!routes) return false;
    
    // 精确匹配
    auto it = routes->find(request.path);
    if (it != routes->end()) {
        handler = it->second;
        return true;
    }
    
    // 通配符匹配（支持 {param} 形式）
    for (const auto& route : *routes) {
        if (path_match(route.first, request.path, captures)) {
            // 将捕获的参数加入请求参数
            for (const auto& cap : captures) {
                request.params[cap.first] = cap.second;
            }
            handler = route.second;
            return true;
        }
    }
    
    return false;
}

bool WebServer::path_match(const String& pattern, const String& path, 
                           std::map<String, String>& captures) {
    auto pattern_parts = detail::split(pattern, '/');
    auto path_parts = detail::split(path, '/');
    
    if (pattern_parts.size() != path_parts.size()) return false;
    
    for (size_t i = 0; i < pattern_parts.size(); ++i) {
        const String& p_part = pattern_parts[i];
        const String& path_part = path_parts[i];
        
        // 检查是否是参数占位符 {xxx}
        if (p_part.size() >= 2 && p_part[0] == '{' && p_part.back() == '}') {
            String param_name = p_part.substr(1, p_part.size() - 2);
            captures[param_name] = path_part;
        } else if (p_part != path_part) {
            return false;
        }
    }
    
    return true;
}

void WebServer::parse_query_params(const String& query, std::map<String, String>& params) {
    auto pairs = detail::split(query, '&');
    for (const String& pair : pairs) {
        size_t eq_pos = pair.find('=');
        if (eq_pos != String::npos) {
            String key = url_decode(pair.substr(0, eq_pos));
            String value = url_decode(pair.substr(eq_pos + 1));
            params[key] = value;
        } else {
            params[url_decode(pair)] = "";
        }
    }
}

String WebServer::url_decode(const String& encoded) {
    String result;
    for (size_t i = 0; i < encoded.size(); ++i) {
        if (encoded[i] == '%' && i + 2 < encoded.size()) {
            char c1 = encoded[i + 1];
            char c2 = encoded[i + 2];
            char decoded = static_cast<char>((detail::hex_to_char(c1) << 4) | detail::hex_to_char(c2));
            result += decoded;
            i += 2;
        } else if (encoded[i] == '+') {
            result += ' ';
        } else {
            result += encoded[i];
        }
    }
    return result;
}

nlohmann::json WebServer::parse_json_body(const String& body) {
    try {
        return nlohmann::json::parse(body);
    } catch (const std::exception& e) {
        OVF_ERROR() << "JSON parse error: " << e.what();
        return nlohmann::json();
    }
}

} // namespace web
} // namespace ovf