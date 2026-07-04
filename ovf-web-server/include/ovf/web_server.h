/**
 * @file web_server.h
 * @brief OpenVisionFlow Web图形化编辑器后端API服务
 */

#pragma once

#ifdef _WIN32
    // 在包含Windows头文件前定义必要的宏
    #define _WIN32_WINNT 0x0601
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    // 取消Windows宏与C++代码冲突的定义
    #ifdef DELETE
        #undef DELETE
    #endif
#endif

#include "ovf/core/types.h"
#include "ovf/core/flow.h"
#include "ovf/core/node.h"
#include "ovf/flow_execution_monitor.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>

namespace ovf {
namespace web {

/**
 * @brief HTTP请求方法
 */
enum class HttpMethod {
    GET,
    POST,
    PUT,
    DELETE,
    OPTIONS
};

/**
 * @brief HTTP请求结构
 */
struct HttpRequest {
    HttpMethod method;
    String path;
    String query_string;
    String body;
    std::map<String, String> headers;
    std::map<String, String> params;
};

/**
 * @brief HTTP响应结构
 */
struct HttpResponse {
    int status_code = 200;
    String status_text = "OK";
    String body;
    String content_type = "application/json";
    std::map<String, String> headers;
    
    void set_json(const String& json);
    void set_error(int code, const String& message);
    void set_html(const String& html);
};

/**
 * @brief API路由处理函数
 */
using ApiHandler = std::function<void(const HttpRequest&, HttpResponse&)>;

/**
 * @brief 流程执行状态
 */
struct FlowExecutionStatus {
    bool is_running = false;
    String current_node;
    int nodes_completed = 0;
    int total_nodes = 0;
    String last_error;
    uint64_t execution_time_us = 0;
    ovf::FlowResult last_result;
};

/**
 * @brief Web服务器类
 */
class WebServer {
public:
    using Ptr = std::shared_ptr<WebServer>;
    
    WebServer();
    ~WebServer();
    
    /**
     * @brief 启动服务器
     * @param port 端口号
     * @return 是否成功
     */
    bool start(int port = 8080);
    
    /**
     * @brief 停止服务器
     */
    void stop();
    
    /**
     * @brief 检查服务器是否运行
     */
    bool is_running() const;
    
    /**
     * @brief 注册API路由
     * @param method HTTP方法
     * @param path 路径
     * @param handler 处理函数
     */
    void register_route(HttpMethod method, const String& path, ApiHandler handler);
    
    /**
     * @brief 设置流程引擎
     */
    void set_flow_engine(ovf::FlowEngine::Ptr engine);
    
    /**
     * @brief 获取流程引擎
     */
    ovf::FlowEngine::Ptr flow_engine() const { return flow_engine_; }
    
    /**
     * @brief 获取执行状态
     */
    FlowExecutionStatus& execution_status() { return execution_status_; }
    const FlowExecutionStatus& execution_status() const { return execution_status_; }
    
    /**
     * @brief 获取执行监控器
     */
    FlowExecutionMonitor::Ptr execution_monitor() const { return execution_monitor_; }

private:
    // 初始化Winsock
    bool init_winsock();
    
    // 清理Winsock
    void cleanup_winsock();
    
    // 创建服务器socket
    bool create_server_socket(int port);
    
    // 接受连接
    void accept_connections();
    
    // 处理客户端连接
    void handle_client(SOCKET client_socket);
    
    // 解析HTTP请求
    bool parse_request(const String& raw_request, HttpRequest& request);
    
    // 构建HTTP响应
    String build_response(const HttpResponse& response);
    
    // 路由匹配
    bool match_route(HttpRequest& request, ApiHandler& handler);
    
    // 解析查询参数
    void parse_query_params(const String& query, std::map<String, String>& params);
    
    // 解析JSON body
    nlohmann::json parse_json_body(const String& body);
    
    // URL解码
    String url_decode(const String& encoded);
    
    // 路径匹配（支持简单通配符）
    bool path_match(const String& pattern, const String& path, std::map<String, String>& captures);

private:
    SOCKET server_socket_;
    int port_;
    std::atomic<bool> running_;
    std::thread accept_thread_;
    
    std::map<String, ApiHandler> get_routes_;
    std::map<String, ApiHandler> post_routes_;
    std::map<String, ApiHandler> put_routes_;
    std::map<String, ApiHandler> delete_routes_;
    
    ovf::FlowEngine::Ptr flow_engine_;
    ovf::FlowContext flow_context_;
    FlowExecutionStatus execution_status_;
    FlowExecutionMonitor::Ptr execution_monitor_;
    
    std::mutex mutex_;
};

/**
 * @brief API处理器 - 实现各个API接口
 */
class ApiHandlers {
public:
    /**
     * @brief 注册所有API路由到服务器
     */
    static void register_all(WebServer& server);
    
    // API接口实现
    static void handle_get_nodes(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_get_node_info(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_flow_load(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_flow_save(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_flow_run(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_flow_step(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_flow_status(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_get_image(const HttpRequest& req, HttpResponse& res, WebServer& server);
    
    // 执行日志可视化相关API
    static void handle_execution_history(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_execution_session(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_performance_report(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_monitor_realtime(const HttpRequest& req, HttpResponse& res, WebServer& server);
    static void handle_export_logs(const HttpRequest& req, HttpResponse& res, WebServer& server);
    
    // 辅助函数
    static nlohmann::json node_info_to_json(const ovf::NodeInfo& info);
    static nlohmann::json flow_def_to_json(const ovf::FlowDef& flow);
    static ovf::FlowDef json_to_flow_def(const nlohmann::json& j);
    static String image_to_base64(const ovf::ImageData& image);
};

} // namespace web
} // namespace ovf