/**
 * @file api_handlers.cpp
 * @brief API接口处理器实现
 */

#include "ovf/web_server.h"
#include "ovf/core/logger.h"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace ovf {
namespace web {

using json = nlohmann::json;

// Base64编码表
static const char base64_chars[] = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Base64编码函数
String encode_base64(const ByteArray& data) {
    String result;
    int i = 0;
    int j = 0;
    uint8_t arr3[3];
    uint8_t arr4[4];
    size_t in_len = data.size();
    const uint8_t* bytes = data.data();
    
    while (in_len--) {
        arr3[i++] = bytes[j++];
        if (i == 3) {
            arr4[0] = (arr3[0] & 0xfc) >> 2;
            arr4[1] = ((arr3[0] & 0x03) << 4) + ((arr3[1] & 0xf0) >> 4);
            arr4[2] = ((arr3[1] & 0x0f) << 2) + ((arr3[2] & 0xc0) >> 6);
            arr4[3] = arr3[2] & 0x3f;
            
            for (i = 0; i < 4; i++)
                result += base64_chars[arr4[i]];
            i = 0;
        }
    }
    
    if (i) {
        for (j = i; j < 3; j++)
            arr3[j] = 0;
        
        arr4[0] = (arr3[0] & 0xfc) >> 2;
        arr4[1] = ((arr3[0] & 0x03) << 4) + ((arr3[1] & 0xf0) >> 4);
        arr4[2] = ((arr3[1] & 0x0f) << 2) + ((arr3[2] & 0xc0) >> 6);
        arr4[3] = arr3[2] & 0x3f;
        
        for (j = 0; j < i + 1; j++)
            result += base64_chars[arr4[j]];
        
        while (i++ < 3)
            result += '=';
    }
    
    return result;
}

// 获取DataType字符串表示
String data_type_to_string(DataType type) {
    switch (type) {
        case DataType::None: return "none";
        case DataType::Image: return "image";
        case DataType::Number: return "number";
        case DataType::String: return "string";
        case DataType::Boolean: return "boolean";
        case DataType::Array: return "array";
        case DataType::Object: return "object";
        case DataType::Point: return "point";
        case DataType::Region: return "region";
        case DataType::Pose: return "pose";
        case DataType::Any: return "any";
        default: return "unknown";
    }
}

// NodeInfo转JSON
json ApiHandlers::node_info_to_json(const NodeInfo& info) {
    json j = json::object();
    j["id"] = info.id;
    j["name"] = info.name;
    j["category"] = info.category;
    j["description"] = info.description;
    j["version"] = info.version;
    j["author"] = info.author;
    
    // 输入端口
    json inputs = json::array();
    for (const auto& port : info.inputs) {
        json p = json::object();
        p["id"] = port.id;
        p["name"] = port.name;
        p["type"] = data_type_to_string(port.data_type);
        p["required"] = port.required;
        p["description"] = port.description;
        inputs.push_back(p);
    }
    j["inputs"] = inputs;
    
    // 输出端口
    json outputs = json::array();
    for (const auto& port : info.outputs) {
        json p = json::object();
        p["id"] = port.id;
        p["name"] = port.name;
        p["type"] = data_type_to_string(port.data_type);
        p["description"] = port.description;
        outputs.push_back(p);
    }
    j["outputs"] = outputs;
    
    // 参数定义
    json params = json::array();
    for (const auto& param : info.params) {
        json p = json::object();
        p["id"] = param.id;
        p["name"] = param.name;
        p["type"] = data_type_to_string(param.type);
        p["description"] = param.description;
        
        // 默认值
        if (param.default_value.is_valid()) {
            if (param.default_value.is_number()) {
                p["default"] = param.default_value.as_number();
            } else if (param.default_value.is_string()) {
                p["default"] = param.default_value.as_string();
            } else if (param.default_value.is_bool()) {
                p["default"] = param.default_value.as_bool();
            }
        }
        
        // 范围限制
        if (param.min_value.is_number()) {
            p["min"] = param.min_value.as_number();
        }
        if (param.max_value.is_number()) {
            p["max"] = param.max_value.as_number();
        }
        
        // 选项列表
        if (!param.options.empty()) {
            json opts = json::array();
            for (const auto& opt : param.options) {
                opts.push_back(json(opt));
            }
            p["options"] = opts;
        }
        
        params.push_back(p);
    }
    j["params"] = params;
    
    return j;
}

// FlowDef转JSON
json ApiHandlers::flow_def_to_json(const FlowDef& flow) {
    json j = json::object();
    j["id"] = flow.id;
    j["name"] = flow.name;
    j["description"] = flow.description;
    j["version"] = flow.version;
    j["created_time"] = flow.created_time;
    j["modified_time"] = flow.modified_time;
    j["author"] = flow.author;
    
    // 节点列表
    json nodes = json::array();
    for (const auto& node_inst : flow.nodes) {
        json n = json::object();
        n["id"] = node_inst.id;
        n["type_id"] = node_inst.type_id;
        n["name"] = node_inst.name;
        n["x"] = node_inst.x;
        n["y"] = node_inst.y;
        n["enabled"] = node_inst.enabled;
        
        // 参数
        json params = json::object();
        const NodeInfo* info = NodeFactory::instance().get_info(node_inst.type_id);
        if (info) {
            for (const auto& param_def : info->params) {
                Data value = node_inst.params.get(param_def.id, param_def.default_value);
                if (value.is_number()) {
                    params[param_def.id] = value.as_number();
                } else if (value.is_string()) {
                    params[param_def.id] = value.as_string();
                } else if (value.is_bool()) {
                    params[param_def.id] = value.as_bool();
                }
            }
        }
        n["params"] = params;
        
        // 输入连接
        json inputs = json::object();
        for (const auto& conn : node_inst.input_connections) {
            json c = json::object();
            c["source_node"] = conn.second.source_node_id;
            c["source_port"] = conn.second.source_port;
            inputs[conn.first] = c;
        }
        n["inputs"] = inputs;
        
        nodes.push_back(n);
    }
    j["nodes"] = nodes;
    
    return j;
}

// JSON转FlowDef
FlowDef ApiHandlers::json_to_flow_def(const json& j) {
    FlowDef flow;
    
    flow.id = j.value("id", std::string(""));
    flow.name = j.value("name", std::string(""));
    flow.description = j.value("description", std::string(""));
    flow.version = j.value("version", std::string("1.0"));
    flow.created_time = j.value("created_time", std::string(""));
    flow.modified_time = j.value("modified_time", std::string(""));
    flow.author = j.value("author", std::string(""));
    
    if (j.contains("nodes") && j["nodes"].is_array()) {
        const json& nodes_j = j["nodes"];
        for (size_t i = 0; i < nodes_j.size(); ++i) {
            const json& n = nodes_j[i];
            FlowDef::NodeInstance node_inst;
            node_inst.id = n.value("id", std::string(""));
            node_inst.type_id = n.value("type_id", std::string(""));
            node_inst.name = n.value("name", std::string(""));
            node_inst.x = n.value("x", 0);
            node_inst.y = n.value("y", 0);
            node_inst.enabled = n.value("enabled", true);
            
            flow.nodes.push_back(node_inst);
        }
    }
    
    return flow;
}

// 图像转Base64
String ApiHandlers::image_to_base64(const ImageData& image) {
    if (image.empty()) return "";
    return encode_base64(image.data);
}

// 注册所有API路由
void ApiHandlers::register_all(WebServer& server) {
    // GET /api/nodes - 获取所有节点类型
    server.register_route(HttpMethod::GET, "/api/nodes",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_get_nodes(req, res, server);
        });
    
    // GET /api/node/{type} - 获取节点详细信息
    server.register_route(HttpMethod::GET, "/api/node/{type}",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_get_node_info(req, res, server);
        });
    
    // POST /api/flow/load - 加载流程文件
    server.register_route(HttpMethod::POST, "/api/flow/load",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_flow_load(req, res, server);
        });
    
    // POST /api/flow/save - 保存流程文件
    server.register_route(HttpMethod::POST, "/api/flow/save",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_flow_save(req, res, server);
        });
    
    // POST /api/flow/run - 执行流程
    server.register_route(HttpMethod::POST, "/api/flow/run",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_flow_run(req, res, server);
        });
    
    // POST /api/flow/step - 单步执行
    server.register_route(HttpMethod::POST, "/api/flow/step",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_flow_step(req, res, server);
        });
    
    // GET /api/flow/status - 获取执行状态
    server.register_route(HttpMethod::GET, "/api/flow/status",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_flow_status(req, res, server);
        });
    
    // GET /api/image - 获取当前图像
    server.register_route(HttpMethod::GET, "/api/image",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_get_image(req, res, server);
        });
    
    // ========== 执行日志可视化API ==========
    
    // GET /api/execution/history - 获取执行历史
    server.register_route(HttpMethod::GET, "/api/execution/history",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_execution_history(req, res, server);
        });
    
    // GET /api/execution/session/{session_id} - 获取指定会话详情
    server.register_route(HttpMethod::GET, "/api/execution/session/{session_id}",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_execution_session(req, res, server);
        });
    
    // GET /api/performance/report - 获取性能报告
    server.register_route(HttpMethod::GET, "/api/performance/report",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_performance_report(req, res, server);
        });
    
    // GET /api/monitor/realtime - 获取实时监控数据
    server.register_route(HttpMethod::GET, "/api/monitor/realtime",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_monitor_realtime(req, res, server);
        });
    
    // GET /api/logs/export - 导出日志
    server.register_route(HttpMethod::GET, "/api/logs/export",
        [&server](const HttpRequest& req, HttpResponse& res) {
            handle_export_logs(req, res, server);
        });
    
    OVF_INFO() << "API routes registered";
}

// 从map中获取值的安全函数
static String get_param_value(const std::map<String, String>& params, const String& key, const String& default_val = "") {
    auto it = params.find(key);
    return it != params.end() ? it->second : default_val;
}

// GET /api/nodes - 获取所有节点类型
void ApiHandlers::handle_get_nodes(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    response["success"] = true;
    
    auto types = ovf::NodeFactory::instance().get_all_types();
    
    json nodes = json::array();
    for (const auto& type : types) {
        const NodeInfo* info = ovf::NodeFactory::instance().get_info(type);
        if (info) {
            json node = json::object();
            node["type_id"] = type;
            node["name"] = info->name;
            node["category"] = info->category;
            node["description"] = info->description;
            nodes.push_back(node);
        }
    }
    
    response["nodes"] = nodes;
    response["count"] = static_cast<int>(nodes.size());
    
    res.set_json(response.dump());
    OVF_DEBUG() << "GET /api/nodes returned " << nodes.size() << " nodes";
}

// GET /api/node/{type} - 获取节点详细信息
void ApiHandlers::handle_get_node_info(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    String type = get_param_value(req.params, "type");
    
    const NodeInfo* info = ovf::NodeFactory::instance().get_info(type);
    if (!info) {
        res.set_error(404, "Node type not found: " + type);
        return;
    }
    
    json response = json::object();
    response["success"] = true;
    response["node"] = node_info_to_json(*info);
    
    res.set_json(response.dump());
    OVF_DEBUG() << "GET /api/node/" << type << " returned node info";
}

// POST /api/flow/load - 加载流程文件
void ApiHandlers::handle_flow_load(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    try {
        json request_body = json::parse(req.body);
        
        std::string filepath = request_body.value("filepath", std::string(""));
        
        if (filepath.empty()) {
            res.set_error(400, "Missing filepath parameter");
            return;
        }
        
        auto engine = server.flow_engine();
        if (!engine) {
            res.set_error(500, "Flow engine not initialized");
            return;
        }
        
        auto result = engine->load_from_file(filepath);
        if (!result.is_success()) {
            res.set_error(500, "Failed to load flow: " + result.message());
            return;
        }
        
        response["success"] = true;
        response["message"] = "Flow loaded successfully";
        
        auto nodes = engine->get_all_nodes();
        json flow_json = json::object();
        flow_json["node_count"] = static_cast<int>(nodes.size());
        
        json node_list = json::array();
        for (const auto& node : nodes) {
            json n = json::object();
            n["id"] = node->instance_id();
            n["type"] = node->info().id;
            n["name"] = node->info().name;
            n["state"] = static_cast<int>(node->state());
            node_list.push_back(n);
        }
        flow_json["nodes"] = node_list;
        
        response["flow"] = flow_json;
        
        res.set_json(response.dump());
        OVF_INFO() << "Flow loaded: " << filepath;
        
    } catch (const std::exception& e) {
        res.set_error(400, "Invalid JSON: " + String(e.what()));
    }
}

// POST /api/flow/save - 保存流程文件
void ApiHandlers::handle_flow_save(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    try {
        json request_body = json::parse(req.body);
        
        std::string filepath = request_body.value("filepath", std::string(""));
        
        if (filepath.empty()) {
            res.set_error(400, "Missing filepath parameter");
            return;
        }
        
        auto engine = server.flow_engine();
        if (!engine) {
            res.set_error(500, "Flow engine not initialized");
            return;
        }
        
        auto result = engine->save_to_file(filepath);
        if (!result.is_success()) {
            res.set_error(500, "Failed to save flow: " + result.message());
            return;
        }
        
        response["success"] = true;
        response["message"] = "Flow saved successfully";
        response["filepath"] = filepath;
        
        res.set_json(response.dump());
        OVF_INFO() << "Flow saved: " << filepath;
        
    } catch (const std::exception& e) {
        res.set_error(400, "Invalid JSON: " + String(e.what()));
    }
}

// POST /api/flow/run - 执行流程
void ApiHandlers::handle_flow_run(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto engine = server.flow_engine();
    if (!engine) {
        res.set_error(500, "Flow engine not initialized");
        return;
    }
    
    auto& status = server.execution_status();
    
    if (status.is_running) {
        res.set_error(400, "Flow is already running");
        return;
    }
    
    ovf::FlowContext context;
    
    engine->set_node_state_callback([&status](const String& node_id, NodeState state) {
        status.current_node = node_id;
        if (state == NodeState::Success) {
            status.nodes_completed++;
        }
    });
    
    status.is_running = true;
    status.nodes_completed = 0;
    status.total_nodes = static_cast<int>(engine->get_all_nodes().size());
    status.last_error.clear();
    
    auto result = engine->run(context);
    
    status.is_running = false;
    status.last_result = result;
    status.execution_time_us = result.total_time_us;
    
    if (!result.success) {
        status.last_error = result.error_message;
        response["success"] = false;
        response["error"] = result.error_message;
        response["failed_node"] = result.failed_node_id;
    } else {
        response["success"] = true;
        response["message"] = "Flow executed successfully";
        response["execution_time_ms"] = static_cast<double>(result.total_time_us) / 1000.0;
        
        json node_times = json::object();
        for (const auto& pair : result.node_times) {
            node_times[pair.first] = static_cast<double>(pair.second) / 1000.0;
        }
        response["node_times"] = node_times;
    }
    
    res.set_json(response.dump());
    OVF_INFO() << "Flow execution completed: " << (result.success ? "success" : "failed");
}

// POST /api/flow/step - 单步执行
void ApiHandlers::handle_flow_step(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto engine = server.flow_engine();
    if (!engine) {
        res.set_error(500, "Flow engine not initialized");
        return;
    }
    
    try {
        json request_body = json::parse(req.body);
        
        std::string action = request_body.value("action", std::string("next"));
        
        if (action == "begin") {
            engine->step_begin();
            
            response["success"] = true;
            response["message"] = "Step mode started";
            response["has_more"] = engine->step_has_more();
            
            res.set_json(response.dump());
            OVF_DEBUG() << "Step mode started";
            return;
        }
        
        if (action == "next") {
            auto node_result = engine->step_next();
            
            if (!node_result.is_success()) {
                response["success"] = false;
                response["error"] = node_result.message();
                response["has_more"] = false;
            } else {
                auto node = node_result.value();
                
                response["success"] = true;
                response["node_id"] = node->instance_id();
                response["node_type"] = node->info().id;
                response["node_name"] = node->info().name;
                response["node_state"] = static_cast<int>(node->state());
                response["has_more"] = engine->step_has_more();
            }
            
            res.set_json(response.dump());
            return;
        }
        
        if (action == "end") {
            engine->step_end();
            
            response["success"] = true;
            response["message"] = "Step mode ended";
            
            res.set_json(response.dump());
            OVF_DEBUG() << "Step mode ended";
            return;
        }
        
        res.set_error(400, "Invalid action: " + action);
        
    } catch (const std::exception& e) {
        res.set_error(400, "Invalid JSON: " + String(e.what()));
    }
}

// GET /api/flow/status - 获取执行状态
void ApiHandlers::handle_flow_status(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    const auto& status = server.execution_status();
    
    response["success"] = true;
    response["is_running"] = status.is_running;
    response["current_node"] = status.current_node;
    response["nodes_completed"] = status.nodes_completed;
    response["total_nodes"] = status.total_nodes;
    response["execution_time_ms"] = static_cast<double>(status.execution_time_us) / 1000.0;
    
    if (!status.last_error.empty()) {
        response["last_error"] = status.last_error;
    }
    
    if (!status.is_running && status.last_result.success) {
        response["last_result"] = "success";
    } else if (!status.is_running && !status.last_result.success) {
        response["last_result"] = "failed";
    }
    
    res.set_json(response.dump());
}

// GET /api/image - 获取当前图像
void ApiHandlers::handle_get_image(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto engine = server.flow_engine();
    if (!engine) {
        res.set_error(500, "Flow engine not initialized");
        return;
    }
    
    std::string node_id = get_param_value(req.params, "node_id");
    std::string port_id = get_param_value(req.params, "port_id", "output");
    
    ImageData image;
    
    if (!node_id.empty()) {
        auto node = engine->get_node(node_id);
        if (node) {
            auto output_data = node->get_output(port_id);
            if (output_data.is_image()) {
                image = output_data.as_image();
            }
        }
    } else {
        auto nodes = engine->get_all_nodes();
        for (const auto& node : nodes) {
            for (const auto& port : node->info().outputs) {
                auto output_data = node->get_output(port.id);
                if (output_data.is_image() && !output_data.as_image().empty()) {
                    image = output_data.as_image();
                    node_id = node->instance_id();
                    port_id = port.id;
                    break;
                }
            }
            if (!image.empty()) break;
        }
    }
    
    if (image.empty()) {
        response["success"] = false;
        response["error"] = "No image available";
        response["has_image"] = false;
    } else {
        response["success"] = true;
        response["has_image"] = true;
        response["width"] = static_cast<int>(image.width);
        response["height"] = static_cast<int>(image.height);
        response["channels"] = static_cast<int>(image.channels);
        response["node_id"] = node_id;
        response["port_id"] = port_id;
        response["format"] = static_cast<int>(image.format);
        
        String base64_data = image_to_base64(image);
        response["data"] = base64_data;
    }
    
    res.set_json(response.dump());
}

// ========== 执行日志可视化API实现 ==========

// GET /api/execution/history - 获取执行历史
void ApiHandlers::handle_execution_history(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto monitor = server.execution_monitor();
    if (!monitor) {
        res.set_error(500, "Execution monitor not initialized");
        return;
    }
    
    String limit_str = get_param_value(req.params, "limit", "100");
    int limit = std::stoi(limit_str);
    
    auto history = monitor->get_execution_history(limit);
    
    json sessions = json::array();
    for (const auto& session : history) {
        json s = json::object();
        s["session_id"] = session.session_id;
        s["flow_id"] = session.flow_id;
        s["flow_name"] = session.flow_name;
        s["start_timestamp"] = session.start_timestamp;
        s["end_timestamp"] = session.end_timestamp;
        s["success"] = session.success;
        s["total_nodes"] = session.total_nodes;
        s["completed_nodes"] = session.completed_nodes;
        s["failed_nodes"] = session.failed_nodes;
        s["node_count"] = static_cast<int>(session.node_records.size());
        sessions.push_back(s);
    }
    
    response["success"] = true;
    response["sessions"] = sessions;
    response["count"] = static_cast<int>(sessions.size());
    
    res.set_json(response.dump());
    OVF_DEBUG() << "GET /api/execution/history returned " << sessions.size() << " sessions";
}

// GET /api/execution/session/{session_id} - 获取指定会话详情
void ApiHandlers::handle_execution_session(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto monitor = server.execution_monitor();
    if (!monitor) {
        res.set_error(500, "Execution monitor not initialized");
        return;
    }
    
    String session_id = get_param_value(req.params, "session_id");
    
    FlowExecutionSession session;
    if (!monitor->get_session_detail(session_id, session)) {
        res.set_error(404, "Session not found: " + session_id);
        return;
    }
    
    response["success"] = true;
    response["session_id"] = session.session_id;
    response["flow_id"] = session.flow_id;
    response["flow_name"] = session.flow_name;
    response["start_timestamp"] = session.start_timestamp;
    response["end_timestamp"] = session.end_timestamp;
    response["success"] = session.success;
    response["total_nodes"] = session.total_nodes;
    response["completed_nodes"] = session.completed_nodes;
    response["failed_nodes"] = session.failed_nodes;
    
    // 节点执行记录详情
    json nodes = json::array();
    for (const auto& record : session.node_records) {
        json n = json::object();
        n["node_id"] = record.node_id;
        n["node_type"] = record.node_type;
        n["node_name"] = record.node_name;
        
        // 状态转换
        String state_str;
        switch (record.state) {
            case NodeExecutionState::Pending: state_str = "pending"; break;
            case NodeExecutionState::Running: state_str = "running"; break;
            case NodeExecutionState::Success: state_str = "success"; break;
            case NodeExecutionState::Error: state_str = "error"; break;
            case NodeExecutionState::Skipped: state_str = "skipped"; break;
            default: state_str = "unknown"; break;
        }
        n["state"] = state_str;
        n["start_timestamp"] = record.start_timestamp;
        n["end_timestamp"] = record.end_timestamp;
        n["duration_ms"] = static_cast<double>(record.duration_us) / 1000.0;
        n["error_code"] = record.error_code;
        n["error_message"] = record.error_message;
        
        // 输入输出摘要
        json input_summary = json::object();
        for (const auto& pair : record.input_summary) {
            input_summary[pair.first] = pair.second;
        }
        n["input_summary"] = input_summary;
        
        json output_summary = json::object();
        for (const auto& pair : record.output_summary) {
            output_summary[pair.first] = pair.second;
        }
        n["output_summary"] = output_summary;
        
        nodes.push_back(n);
    }
    response["node_records"] = nodes;
    
    res.set_json(response.dump());
    OVF_DEBUG() << "GET /api/execution/session/" << session_id << " returned session details";
}

// GET /api/performance/report - 获取性能报告
void ApiHandlers::handle_performance_report(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto monitor = server.execution_monitor();
    if (!monitor) {
        res.set_error(500, "Execution monitor not initialized");
        return;
    }
    
    String history_seconds_str = get_param_value(req.params, "history_seconds", "60");
    int history_seconds = std::stoi(history_seconds_str);
    
    // 获取Chart.js格式的性能报告
    response["success"] = true;
    response["performance_report"] = monitor->get_performance_report_json(history_seconds);
    
    // 获取性能瓶颈
    auto bottlenecks = monitor->get_performance_report(history_seconds).bottlenecks;
    json bn_list = json::array();
    for (const auto& bn : bottlenecks) {
        json b = json::object();
        b["node_id"] = bn.node_id;
        b["node_type"] = bn.node_type;
        b["bottleneck_type"] = bn.bottleneck_type;
        b["description"] = bn.description;
        b["severity"] = bn.severity;
        b["impact_score"] = bn.impact_score;
        bn_list.push_back(b);
    }
    response["bottlenecks"] = bn_list;
    
    res.set_json(response.dump());
    OVF_DEBUG() << "GET /api/performance/report returned performance report";
}

// GET /api/monitor/realtime - 获取实时监控数据
void ApiHandlers::handle_monitor_realtime(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    json response = json::object();
    
    auto monitor = server.execution_monitor();
    if (!monitor) {
        res.set_error(500, "Execution monitor not initialized");
        return;
    }
    
    auto realtime_data = monitor->get_realtime_data();
    
    // 状态转换
    String state_str;
    switch (realtime_data.state) {
        case FlowState::Idle: state_str = "idle"; break;
        case FlowState::Running: state_str = "running"; break;
        case FlowState::Paused: state_str = "paused"; break;
        case FlowState::Completed: state_str = "completed"; break;
        case FlowState::Failed: state_str = "failed"; break;
        case FlowState::Cancelled: state_str = "cancelled"; break;
        default: state_str = "unknown"; break;
    }
    
    response["success"] = true;
    response["state"] = state_str;
    response["current_node"] = realtime_data.current_node;
    response["progress"] = realtime_data.progress_percent;
    response["elapsed_time_ms"] = realtime_data.elapsed_time_ms;
    response["estimated_remaining_ms"] = realtime_data.estimated_remaining_ms;
    response["session_id"] = realtime_data.session_id;
    response["flow_id"] = realtime_data.flow_id;
    response["flow_name"] = realtime_data.flow_name;
    response["fps"] = realtime_data.current_fps;
    response["memory_mb"] = realtime_data.current_memory_mb;
    response["throughput"] = realtime_data.throughput;
    
    // 最近异常
    json exceptions = json::array();
    for (const auto& ex : realtime_data.recent_exceptions) {
        json e = json::object();
        e["timestamp"] = ex.timestamp;
        e["node_id"] = ex.node_id;
        e["node_type"] = ex.node_type;
        e["error_code"] = ex.error_code;
        e["error_message"] = ex.error_message;
        exceptions.push_back(e);
    }
    response["recent_exceptions"] = exceptions;
    
    // 执行统计
    auto stats = monitor->get_statistics();
    response["statistics"] = json::object();
    response["statistics"]["total_executions"] = stats.total_executions;
    response["statistics"]["successful_executions"] = stats.successful_executions;
    response["statistics"]["failed_executions"] = stats.failed_executions;
    response["statistics"]["success_rate"] = stats.success_rate;
    response["statistics"]["avg_execution_time_ms"] = stats.avg_execution_time_ms;
    response["statistics"]["min_execution_time_ms"] = stats.min_execution_time_ms;
    response["statistics"]["max_execution_time_ms"] = stats.max_execution_time_ms;
    response["statistics"]["peak_memory_mb"] = stats.peak_memory_mb;
    response["statistics"]["avg_fps"] = stats.avg_fps;
    
    res.set_json(response.dump());
    OVF_DEBUG() << "GET /api/monitor/realtime returned realtime data";
}

// GET /api/logs/export - 导出日志
void ApiHandlers::handle_export_logs(const HttpRequest& req, HttpResponse& res, WebServer& server) {
    auto monitor = server.execution_monitor();
    if (!monitor) {
        res.set_error(500, "Execution monitor not initialized");
        return;
    }
    
    String format = get_param_value(req.params, "format", "json");
    String session_id = get_param_value(req.params, "session_id", "");
    
    String exported_data;
    
    if (format == "csv") {
        exported_data = monitor->export_logs_csv(session_id);
        res.content_type = "text/csv";
        
        // 如果请求下载，设置附件头
        String filename = session_id.empty() ? "execution_logs.csv" : "session_" + session_id + ".csv";
        res.headers["Content-Disposition"] = "attachment; filename=\"" + filename + "\"";
    } else if (format == "json") {
        exported_data = monitor->export_logs_json(session_id);
        res.content_type = "application/json";
        
        String filename = session_id.empty() ? "execution_logs.json" : "session_" + session_id + ".json";
        res.headers["Content-Disposition"] = "attachment; filename=\"" + filename + "\"";
    } else if (format == "performance") {
        exported_data = monitor->export_performance_json();
        res.content_type = "application/json";
        res.headers["Content-Disposition"] = "attachment; filename=\"performance_report.json\"";
    } else {
        res.set_error(400, "Invalid format: " + format + ". Supported formats: json, csv, performance");
        return;
    }
    
    res.status_code = 200;
    res.status_text = "OK";
    res.body = exported_data;
    
    OVF_DEBUG() << "GET /api/logs/export exported logs in " << format << " format";
}

} // namespace web
} // namespace ovf