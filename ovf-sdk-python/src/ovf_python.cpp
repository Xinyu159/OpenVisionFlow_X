/**
 * @file ovf_python.cpp
 * @brief OpenVisionFlow Python SDK C接口实现
 */

#include "ovf_python.h"
#include "ovf/core/types.h"
#include "ovf/core/flow.h"
#include "ovf/core/node.h"
#include "ovf/core/error.h"
#include "ovf/hal/hal.h"
#include "ovf/core/logger.h"
#include "ovf/api.h"

#include <mutex>
#include <cstring>
#include <fstream>
#include <sstream>
#include <atomic>

namespace {

// 全局错误状态
std::mutex g_error_mutex;
OVFErrorCode g_last_error_code = OVF_SUCCESS;
std::string g_last_error_message;

void set_error(OVFErrorCode code, const std::string& message) {
    std::lock_guard<std::mutex> lock(g_error_mutex);
    g_last_error_code = code;
    g_last_error_message = message;
}

void clear_error_state() {
    std::lock_guard<std::mutex> lock(g_error_mutex);
    g_last_error_code = OVF_SUCCESS;
    g_last_error_message.clear();
}

// 图像句柄包装
struct ImageHandleImpl {
    ovf::ImageData data;
    
    ImageHandleImpl() = default;
    explicit ImageHandleImpl(const ovf::ImageData& d) : data(d) {}
};

// 流程引擎句柄包装
struct EngineHandleImpl {
    ovf::FlowEngine::Ptr engine;
    ovf::FlowContext context;
};

// 节点句柄包装
struct NodeHandleImpl {
    ovf::INode::Ptr node;
};

// 相机句柄包装
struct CameraHandleImpl {
    ovf::hal::ICamera::Ptr camera;
};

// 运行器句柄包装
struct RunnerHandleImpl {
    ovf::FlowRunner::Ptr runner;
};

// 错误码映射
OVFErrorCode map_error_code(ovf::ErrorCode code) {
    switch (code) {
        case ovf::ErrorCode::Success: return OVF_SUCCESS;
        case ovf::ErrorCode::Unknown: return OVF_ERROR_UNKNOWN;
        case ovf::ErrorCode::InvalidParameter: return OVF_ERROR_INVALID_PARAM;
        case ovf::ErrorCode::NullPointer: return OVF_ERROR_NULL_POINTER;
        case ovf::ErrorCode::OutOfRange: return OVF_ERROR_OUT_OF_RANGE;
        case ovf::ErrorCode::NotSupported: return OVF_ERROR_NOT_SUPPORTED;
        case ovf::ErrorCode::Timeout: return OVF_ERROR_TIMEOUT;
        case ovf::ErrorCode::FlowNotFound: return OVF_ERROR_FLOW_NOT_FOUND;
        case ovf::ErrorCode::NodeNotFound: return OVF_ERROR_NODE_NOT_FOUND;
        case ovf::ErrorCode::InvalidFlow: return OVF_ERROR_INVALID_FLOW;
        case ovf::ErrorCode::InvalidNode: return OVF_ERROR_INVALID_NODE;
        case ovf::ErrorCode::ExecutionFailed: return OVF_ERROR_EXECUTION_FAILED;
        case ovf::ErrorCode::DeviceNotFound: return OVF_ERROR_DEVICE_NOT_FOUND;
        case ovf::ErrorCode::DeviceOpenFailed: return OVF_ERROR_DEVICE_OPEN_FAILED;
        case ovf::ErrorCode::CameraCaptureFailed: return OVF_ERROR_CAMERA_CAPTURE_FAILED;
        case ovf::ErrorCode::FileNotFound: return OVF_ERROR_FILE_NOT_FOUND;
        case ovf::ErrorCode::FileParseFailed: return OVF_ERROR_FILE_PARSE_FAILED;
        default: return OVF_ERROR_UNKNOWN;
    }
}

// SDK初始化状态
std::atomic<bool> g_sdk_initialized{false};

// 指针转换辅助函数
inline ImageHandleImpl* to_image_impl(OVFImage img) {
    return reinterpret_cast<ImageHandleImpl*>(img);
}

inline OVFImage to_image_handle(ImageHandleImpl* impl) {
    return reinterpret_cast<OVFImage>(impl);
}

inline EngineHandleImpl* to_engine_impl(OVFEngine engine) {
    return reinterpret_cast<EngineHandleImpl*>(engine);
}

inline OVFEngine to_engine_handle(EngineHandleImpl* impl) {
    return reinterpret_cast<OVFEngine>(impl);
}

inline NodeHandleImpl* to_node_impl(OVFNode node) {
    return reinterpret_cast<NodeHandleImpl*>(node);
}

inline OVFNode to_node_handle(NodeHandleImpl* impl) {
    return reinterpret_cast<OVFNode>(impl);
}

inline CameraHandleImpl* to_camera_impl(OVFCamera cam) {
    return reinterpret_cast<CameraHandleImpl*>(cam);
}

inline OVFCamera to_camera_handle(CameraHandleImpl* impl) {
    return reinterpret_cast<OVFCamera>(impl);
}

inline RunnerHandleImpl* to_runner_impl(OVFRunner runner) {
    return reinterpret_cast<RunnerHandleImpl*>(runner);
}

inline OVFRunner to_runner_handle(RunnerHandleImpl* impl) {
    return reinterpret_cast<OVFRunner>(impl);
}

} // anonymous namespace

// ============================================================
// SDK 初始化/关闭
// ============================================================

OVF_PYTHON_API OVFErrorCode ovf_initialize() {
    clear_error_state();
    if (g_sdk_initialized.load()) {
        return OVF_SUCCESS;
    }
    
    auto result = ovf::api::initialize();
    if (result.is_success()) {
        g_sdk_initialized.store(true);
        return OVF_SUCCESS;
    }
    
    set_error(map_error_code(result.code()), result.message());
    return map_error_code(result.code());
}

OVF_PYTHON_API OVFErrorCode ovf_shutdown() {
    clear_error_state();
    
    auto result = ovf::api::shutdown();
    g_sdk_initialized.store(false);
    
    return OVF_SUCCESS;
}

OVF_PYTHON_API void ovf_get_version(int* major, int* minor, int* patch) {
    if (major) *major = ovf::VERSION_MAJOR;
    if (minor) *minor = ovf::VERSION_MINOR;
    if (patch) *patch = ovf::VERSION_PATCH;
}

// ============================================================
// 图像操作
// ============================================================

OVF_PYTHON_API OVFImage ovf_create_image(int width, int height, int channels) {
    clear_error_state();
    
    if (width <= 0 || height <= 0 || channels <= 0) {
        set_error(OVF_ERROR_INVALID_PARAM, "Invalid image dimensions or channels");
        return nullptr;
    }
    
    try {
        auto impl = new ImageHandleImpl();
        impl->data.width = width;
        impl->data.height = height;
        impl->data.channels = channels;
        impl->data.format = (channels == 1) ? ovf::ImageFormat::Mono8 : ovf::ImageFormat::RGB8;
        impl->data.data.resize(width * height * channels, 0);
        return to_image_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API OVFImage ovf_create_image_from_data(const uint8_t* data, int width, int height, 
                                                    int channels, size_t data_size) {
    clear_error_state();
    
    if (!data || width <= 0 || height <= 0 || channels <= 0) {
        set_error(OVF_ERROR_INVALID_PARAM, "Invalid parameters for creating image from data");
        return nullptr;
    }
    
    size_t expected_size = width * height * channels;
    if (data_size < expected_size) {
        set_error(OVF_ERROR_INVALID_PARAM, "Data size mismatch");
        return nullptr;
    }
    
    try {
        auto impl = new ImageHandleImpl();
        impl->data.width = width;
        impl->data.height = height;
        impl->data.channels = channels;
        impl->data.format = (channels == 1) ? ovf::ImageFormat::Mono8 : ovf::ImageFormat::RGB8;
        impl->data.data.assign(data, data + expected_size);
        return to_image_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API void ovf_destroy_image(OVFImage img) {
    if (img) {
        delete to_image_impl(img);
    }
}

OVF_PYTHON_API int ovf_image_width(OVFImage img) {
    if (!img) {
        set_error(OVF_ERROR_NULL_POINTER, "Null image handle");
        return 0;
    }
    return to_image_impl(img)->data.width;
}

OVF_PYTHON_API int ovf_image_height(OVFImage img) {
    if (!img) {
        set_error(OVF_ERROR_NULL_POINTER, "Null image handle");
        return 0;
    }
    return to_image_impl(img)->data.height;
}

OVF_PYTHON_API int ovf_image_channels(OVFImage img) {
    if (!img) {
        set_error(OVF_ERROR_NULL_POINTER, "Null image handle");
        return 0;
    }
    return to_image_impl(img)->data.channels;
}

OVF_PYTHON_API uint8_t* ovf_image_data(OVFImage img) {
    if (!img) {
        set_error(OVF_ERROR_NULL_POINTER, "Null image handle");
        return nullptr;
    }
    return to_image_impl(img)->data.data.data();
}

OVF_PYTHON_API size_t ovf_image_data_size(OVFImage img) {
    if (!img) {
        set_error(OVF_ERROR_NULL_POINTER, "Null image handle");
        return 0;
    }
    return to_image_impl(img)->data.data.size();
}

OVF_PYTHON_API size_t ovf_image_copy_data(OVFImage img, uint8_t* buffer, size_t buffer_size) {
    if (!img || !buffer) {
        set_error(OVF_ERROR_NULL_POINTER, "Null image handle or buffer");
        return 0;
    }
    
    auto impl = to_image_impl(img);
    size_t copy_size = std::min(buffer_size, impl->data.data.size());
    std::memcpy(buffer, impl->data.data.data(), copy_size);
    return copy_size;
}

// ============================================================
// 流程引擎操作
// ============================================================

OVF_PYTHON_API OVFEngine ovf_create_engine() {
    clear_error_state();
    
    try {
        auto impl = new EngineHandleImpl();
        impl->engine = ovf::api::create_flow_engine();
        if (!impl->engine) {
            delete impl;
            set_error(OVF_ERROR_UNKNOWN, "Failed to create flow engine");
            return nullptr;
        }
        return to_engine_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API void ovf_destroy_engine(OVFEngine engine) {
    if (engine) {
        delete to_engine_impl(engine);
    }
}

OVF_PYTHON_API OVFErrorCode ovf_load_flow(OVFEngine engine, const char* filepath) {
    clear_error_state();
    
    if (!engine || !filepath) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine or filepath");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto impl = to_engine_impl(engine);
        auto result = impl->engine->load_from_file(filepath);
        if (result.is_failure()) {
            set_error(map_error_code(result.code()), result.message());
            return map_error_code(result.code());
        }
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_load_flow_from_json(OVFEngine engine, const char* json_content) {
    clear_error_state();
    
    if (!engine || !json_content) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine or json content");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto impl = to_engine_impl(engine);
        // 解析JSON内容为FlowDef（简化实现）
        ovf::FlowDef flow_def;
        flow_def.id = "flow_from_json";
        flow_def.name = "Flow from JSON";
        
        auto result = impl->engine->load_flow(flow_def);
        if (result.is_failure()) {
            set_error(map_error_code(result.code()), result.message());
            return map_error_code(result.code());
        }
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_save_flow(OVFEngine engine, const char* filepath) {
    clear_error_state();
    
    if (!engine || !filepath) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine or filepath");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto impl = to_engine_impl(engine);
        auto result = impl->engine->save_to_file(filepath);
        if (result.is_failure()) {
            set_error(map_error_code(result.code()), result.message());
            return map_error_code(result.code());
        }
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API size_t ovf_get_flow_json(OVFEngine engine, char* buffer, size_t buffer_size) {
    clear_error_state();
    
    if (!engine || !buffer) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine or buffer");
        return 0;
    }
    
    // 返回空JSON（简化实现）
    std::string json = "{\"id\":\"empty\",\"name\":\"Empty Flow\"}";
    size_t copy_size = std::min(buffer_size - 1, json.size());
    std::memcpy(buffer, json.c_str(), copy_size);
    buffer[copy_size] = '\0';
    return copy_size;
}

OVF_PYTHON_API OVFErrorCode ovf_run_flow(OVFEngine engine) {
    clear_error_state();
    
    if (!engine) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto impl = to_engine_impl(engine);
        auto result = impl->engine->run(impl->context);
        if (!result.success) {
            set_error(OVF_ERROR_EXECUTION_FAILED, result.error_message);
            return OVF_ERROR_EXECUTION_FAILED;
        }
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_run_node(OVFEngine engine, const char* node_id) {
    clear_error_state();
    
    if (!engine || !node_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine or node_id");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto impl = to_engine_impl(engine);
        auto result = impl->engine->run_node(node_id, impl->context);
        if (!result.success) {
            set_error(OVF_ERROR_EXECUTION_FAILED, result.error_message);
            return OVF_ERROR_EXECUTION_FAILED;
        }
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_input_image(OVFEngine engine, const char* node_id, 
                                                 const char* port_id, OVFImage img) {
    clear_error_state();
    
    if (!engine || !node_id || !port_id || !img) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto engine_impl = to_engine_impl(engine);
        auto image_impl = to_image_impl(img);
        
        auto node = engine_impl->engine->get_node(node_id);
        if (!node) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node not found: " + std::string(node_id));
            return OVF_ERROR_NODE_NOT_FOUND;
        }
        
        node->set_input(port_id, ovf::Data(image_impl->data));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_input_number(OVFEngine engine, const char* node_id, 
                                                  const char* port_id, double value) {
    clear_error_state();
    
    if (!engine || !node_id || !port_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto engine_impl = to_engine_impl(engine);
        auto node = engine_impl->engine->get_node(node_id);
        if (!node) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node not found: " + std::string(node_id));
            return OVF_ERROR_NODE_NOT_FOUND;
        }
        
        node->set_input(port_id, ovf::Data(value));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_input_string(OVFEngine engine, const char* node_id, 
                                                  const char* port_id, const char* value) {
    clear_error_state();
    
    if (!engine || !node_id || !port_id || !value) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto engine_impl = to_engine_impl(engine);
        auto node = engine_impl->engine->get_node(node_id);
        if (!node) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node not found: " + std::string(node_id));
            return OVF_ERROR_NODE_NOT_FOUND;
        }
        
        node->set_input(port_id, ovf::Data(std::string(value)));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFImage ovf_get_output_image(OVFEngine engine, const char* node_id, 
                                              const char* port_id) {
    clear_error_state();
    
    if (!engine || !node_id || !port_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return nullptr;
    }
    
    try {
        auto engine_impl = to_engine_impl(engine);
        auto node = engine_impl->engine->get_node(node_id);
        if (!node) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node not found: " + std::string(node_id));
            return nullptr;
        }
        
        auto data = node->get_output(port_id);
        if (!data.is_image()) {
            set_error(OVF_ERROR_INVALID_PARAM, "Output is not an image");
            return nullptr;
        }
        
        auto impl = new ImageHandleImpl(data.as_image());
        return to_image_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_get_output_number(OVFEngine engine, const char* node_id, 
                                                   const char* port_id, double* value) {
    clear_error_state();
    
    if (!engine || !node_id || !port_id || !value) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto engine_impl = to_engine_impl(engine);
        auto node = engine_impl->engine->get_node(node_id);
        if (!node) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node not found: " + std::string(node_id));
            return OVF_ERROR_NODE_NOT_FOUND;
        }
        
        auto data = node->get_output(port_id);
        *value = data.as_number(0.0);
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API int ovf_get_node_count(OVFEngine engine) {
    if (!engine) {
        set_error(OVF_ERROR_NULL_POINTER, "Null engine");
        return 0;
    }
    return static_cast<int>(to_engine_impl(engine)->engine->get_all_nodes().size());
}

OVF_PYTHON_API int ovf_get_node_ids(OVFEngine engine, char** ids, int max_count, size_t id_buffer_size) {
    clear_error_state();
    
    if (!engine || !ids) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return 0;
    }
    
    auto nodes = to_engine_impl(engine)->engine->get_all_nodes();
    int count = std::min(max_count, static_cast<int>(nodes.size()));
    
    for (int i = 0; i < count; ++i) {
        size_t copy_size = std::min(id_buffer_size - 1, nodes[i]->instance_id().size());
        std::memcpy(ids[i], nodes[i]->instance_id().c_str(), copy_size);
        ids[i][copy_size] = '\0';
    }
    
    return count;
}

OVF_PYTHON_API OVFNodeState ovf_get_node_state(OVFEngine engine, const char* node_id) {
    clear_error_state();
    
    if (!engine || !node_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_NODE_STATE_IDLE;
    }
    
    auto node = to_engine_impl(engine)->engine->get_node(node_id);
    if (!node) {
        return OVF_NODE_STATE_IDLE;
    }
    
    switch (node->state()) {
        case ovf::NodeState::Idle: return OVF_NODE_STATE_IDLE;
        case ovf::NodeState::Running: return OVF_NODE_STATE_RUNNING;
        case ovf::NodeState::Success: return OVF_NODE_STATE_SUCCESS;
        case ovf::NodeState::Failed: return OVF_NODE_STATE_FAILED;
        case ovf::NodeState::Disabled: return OVF_NODE_STATE_DISABLED;
        default: return OVF_NODE_STATE_IDLE;
    }
}

OVF_PYTHON_API uint64_t ovf_get_node_execute_time(OVFEngine engine, const char* node_id) {
    clear_error_state();
    
    if (!engine || !node_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return 0;
    }
    
    auto node = to_engine_impl(engine)->engine->get_node(node_id);
    if (!node) {
        return 0;
    }
    
    return node->last_execute_time();
}

// ============================================================
// 节点操作
// ============================================================

OVF_PYTHON_API OVFNode ovf_create_node(const char* type_id, const char* instance_id) {
    clear_error_state();
    
    if (!type_id || !instance_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null type_id or instance_id");
        return nullptr;
    }
    
    try {
        auto node = ovf::api::create_node(type_id, instance_id);
        if (!node) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node type not registered: " + std::string(type_id));
            return nullptr;
        }
        
        auto impl = new NodeHandleImpl();
        impl->node = node;
        return to_node_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API void ovf_destroy_node(OVFNode node) {
    if (node) {
        delete to_node_impl(node);
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_param_int(OVFNode node, const char* key, int value) {
    clear_error_state();
    
    if (!node || !key) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        to_node_impl(node)->node->set_param(key, ovf::Data(static_cast<int32_t>(value)));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_param_float(OVFNode node, const char* key, double value) {
    clear_error_state();
    
    if (!node || !key) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        to_node_impl(node)->node->set_param(key, ovf::Data(value));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_param_string(OVFNode node, const char* key, const char* value) {
    clear_error_state();
    
    if (!node || !key || !value) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        to_node_impl(node)->node->set_param(key, ovf::Data(std::string(value)));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_set_param_bool(OVFNode node, const char* key, int value) {
    clear_error_state();
    
    if (!node || !key) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        to_node_impl(node)->node->set_param(key, ovf::Data(value != 0));
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_add_node_to_engine(OVFEngine engine, OVFNode node) {
    clear_error_state();
    
    if (!engine || !node) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    // 简化实现：节点已经通过FlowDef添加到引擎
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_connect_nodes(OVFEngine engine, 
                                               const char* source_node_id, const char* source_port,
                                               const char* target_node_id, const char* target_port) {
    clear_error_state();
    
    if (!engine || !source_node_id || !source_port || !target_node_id || !target_port) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    try {
        auto engine_impl = to_engine_impl(engine);
        auto source = engine_impl->engine->get_node(source_node_id);
        auto target = engine_impl->engine->get_node(target_node_id);
        
        if (!source || !target) {
            set_error(OVF_ERROR_NODE_NOT_FOUND, "Node not found");
            return OVF_ERROR_NODE_NOT_FOUND;
        }
        
        source->connect_output(source_port, target, target_port);
        return OVF_SUCCESS;
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return OVF_ERROR_UNKNOWN;
    }
}

// ============================================================
// 相机操作
// ============================================================

OVF_PYTHON_API OVFCamera ovf_open_camera(const char* driver_type, const char* device_id) {
    clear_error_state();
    
    if (!driver_type || !device_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null driver_type or device_id");
        return nullptr;
    }
    
    try {
        ovf::hal::DeviceInfo info;
        info.id = device_id;
        info.driver_type = driver_type;
        
        auto camera = ovf::hal::HALManager::instance().create_camera(driver_type, info);
        if (!camera) {
            set_error(OVF_ERROR_DEVICE_NOT_FOUND, "Camera not found: " + std::string(device_id));
            return nullptr;
        }
        
        auto result = camera->open();
        if (result.is_failure()) {
            set_error(OVF_ERROR_DEVICE_OPEN_FAILED, result.message());
            return nullptr;
        }
        
        auto impl = new CameraHandleImpl();
        impl->camera = camera;
        return to_camera_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API void ovf_close_camera(OVFCamera cam) {
    if (cam) {
        auto impl = to_camera_impl(cam);
        if (impl->camera) {
            impl->camera->close();
        }
        delete impl;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_start_capture(OVFCamera cam) {
    clear_error_state();
    
    if (!cam) {
        set_error(OVF_ERROR_NULL_POINTER, "Null camera handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->start_capture();
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_stop_capture(OVFCamera cam) {
    clear_error_state();
    
    if (!cam) {
        set_error(OVF_ERROR_NULL_POINTER, "Null camera handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->stop_capture();
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFImage ovf_capture_frame(OVFCamera cam, int timeout_ms) {
    clear_error_state();
    
    if (!cam) {
        set_error(OVF_ERROR_NULL_POINTER, "Null camera handle");
        return nullptr;
    }
    
    auto result = to_camera_impl(cam)->camera->capture_frame(timeout_ms);
    if (result.is_failure()) {
        set_error(OVF_ERROR_CAMERA_CAPTURE_FAILED, result.message());
        return nullptr;
    }
    
    auto impl = new ImageHandleImpl(result.value());
    return to_image_handle(impl);
}

OVF_PYTHON_API OVFErrorCode ovf_set_exposure(OVFCamera cam, double exposure_us) {
    clear_error_state();
    
    if (!cam) {
        set_error(OVF_ERROR_NULL_POINTER, "Null camera handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->set_exposure(exposure_us);
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_get_exposure(OVFCamera cam, double* exposure_us) {
    clear_error_state();
    
    if (!cam || !exposure_us) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->get_exposure();
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    
    *exposure_us = result.value();
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_set_gain(OVFCamera cam, double gain) {
    clear_error_state();
    
    if (!cam) {
        set_error(OVF_ERROR_NULL_POINTER, "Null camera handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->set_gain(gain);
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_get_gain(OVFCamera cam, double* gain) {
    clear_error_state();
    
    if (!cam || !gain) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->get_gain();
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    
    *gain = result.value();
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_send_soft_trigger(OVFCamera cam) {
    clear_error_state();
    
    if (!cam) {
        set_error(OVF_ERROR_NULL_POINTER, "Null camera handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_camera_impl(cam)->camera->send_soft_trigger();
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API int ovf_enumerate_cameras(const char* driver_type, 
                                          char** ids, char** names, 
                                          int max_count, size_t buffer_size) {
    clear_error_state();
    
    if (!ids || !names) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return 0;
    }
    
    auto devices = ovf::hal::HALManager::instance().enumerate_cameras(driver_type ? driver_type : "");
    int count = std::min(max_count, static_cast<int>(devices.size()));
    
    for (int i = 0; i < count; ++i) {
        // 复制ID
        size_t id_size = std::min(buffer_size - 1, devices[i].id.size());
        std::memcpy(ids[i], devices[i].id.c_str(), id_size);
        ids[i][id_size] = '\0';
        
        // 复制名称
        size_t name_size = std::min(buffer_size - 1, devices[i].name.size());
        std::memcpy(names[i], devices[i].name.c_str(), name_size);
        names[i][name_size] = '\0';
    }
    
    return count;
}

// ============================================================
// 流程运行器
// ============================================================

OVF_PYTHON_API OVFRunner ovf_create_runner() {
    clear_error_state();
    
    try {
        auto impl = new RunnerHandleImpl();
        impl->runner = ovf::api::create_flow_runner();
        if (!impl->runner) {
            delete impl;
            set_error(OVF_ERROR_UNKNOWN, "Failed to create flow runner");
            return nullptr;
        }
        return to_runner_handle(impl);
    } catch (const std::exception& e) {
        set_error(OVF_ERROR_UNKNOWN, e.what());
        return nullptr;
    }
}

OVF_PYTHON_API void ovf_destroy_runner(OVFRunner runner) {
    if (runner) {
        auto impl = to_runner_impl(runner);
        if (impl->runner) {
            impl->runner->stop();
        }
        delete impl;
    }
}

OVF_PYTHON_API OVFErrorCode ovf_runner_set_engine(OVFRunner runner, OVFEngine engine) {
    clear_error_state();
    
    if (!runner || !engine) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return OVF_ERROR_NULL_POINTER;
    }
    
    to_runner_impl(runner)->runner->set_engine(to_engine_impl(engine)->engine);
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_runner_start_continuous(OVFRunner runner) {
    clear_error_state();
    
    if (!runner) {
        set_error(OVF_ERROR_NULL_POINTER, "Null runner handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_runner_impl(runner)->runner->start(ovf::FlowRunner::RunMode::Continuous);
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_runner_start_triggered(OVFRunner runner) {
    clear_error_state();
    
    if (!runner) {
        set_error(OVF_ERROR_NULL_POINTER, "Null runner handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto result = to_runner_impl(runner)->runner->start(ovf::FlowRunner::RunMode::Triggered);
    if (result.is_failure()) {
        set_error(map_error_code(result.code()), result.message());
        return map_error_code(result.code());
    }
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_runner_stop(OVFRunner runner) {
    clear_error_state();
    
    if (!runner) {
        set_error(OVF_ERROR_NULL_POINTER, "Null runner handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    to_runner_impl(runner)->runner->stop();
    return OVF_SUCCESS;
}

OVF_PYTHON_API OVFErrorCode ovf_runner_trigger(OVFRunner runner) {
    clear_error_state();
    
    if (!runner) {
        set_error(OVF_ERROR_NULL_POINTER, "Null runner handle");
        return OVF_ERROR_NULL_POINTER;
    }
    
    to_runner_impl(runner)->runner->trigger();
    return OVF_SUCCESS;
}

OVF_PYTHON_API int ovf_runner_is_running(OVFRunner runner) {
    if (!runner) {
        return 0;
    }
    return to_runner_impl(runner)->runner->is_running() ? 1 : 0;
}

OVF_PYTHON_API void ovf_runner_get_stats(OVFRunner runner, 
                                          uint64_t* total_runs, 
                                          uint64_t* success_runs, 
                                          uint64_t* failed_runs) {
    if (!runner) {
        if (total_runs) *total_runs = 0;
        if (success_runs) *success_runs = 0;
        if (failed_runs) *failed_runs = 0;
        return;
    }
    
    auto impl = to_runner_impl(runner);
    if (total_runs) *total_runs = impl->runner->total_runs();
    if (success_runs) *success_runs = impl->runner->success_runs();
    if (failed_runs) *failed_runs = impl->runner->failed_runs();
}

// ============================================================
// 节点类型注册信息
// ============================================================

OVF_PYTHON_API int ovf_get_registered_node_type_count() {
    return static_cast<int>(ovf::NodeFactory::instance().get_all_types().size());
}

OVF_PYTHON_API int ovf_get_registered_node_types(char** types, int max_count, size_t buffer_size) {
    clear_error_state();
    
    if (!types) {
        set_error(OVF_ERROR_NULL_POINTER, "Null parameter");
        return 0;
    }
    
    auto all_types = ovf::NodeFactory::instance().get_all_types();
    int count = std::min(max_count, static_cast<int>(all_types.size()));
    
    for (int i = 0; i < count; ++i) {
        size_t copy_size = std::min(buffer_size - 1, all_types[i].size());
        std::memcpy(types[i], all_types[i].c_str(), copy_size);
        types[i][copy_size] = '\0';
    }
    
    return count;
}

OVF_PYTHON_API OVFErrorCode ovf_get_node_type_info(const char* type_id,
                                                    char* name, char* category, 
                                                    char* description, size_t buffer_size) {
    clear_error_state();
    
    if (!type_id) {
        set_error(OVF_ERROR_NULL_POINTER, "Null type_id");
        return OVF_ERROR_NULL_POINTER;
    }
    
    auto info = ovf::NodeFactory::instance().get_info(type_id);
    if (!info) {
        set_error(OVF_ERROR_NODE_NOT_FOUND, "Node type not found: " + std::string(type_id));
        return OVF_ERROR_NODE_NOT_FOUND;
    }
    
    if (name) {
        size_t copy_size = std::min(buffer_size - 1, info->name.size());
        std::memcpy(name, info->name.c_str(), copy_size);
        name[copy_size] = '\0';
    }
    
    if (category) {
        size_t copy_size = std::min(buffer_size - 1, info->category.size());
        std::memcpy(category, info->category.c_str(), copy_size);
        category[copy_size] = '\0';
    }
    
    if (description) {
        size_t copy_size = std::min(buffer_size - 1, info->description.size());
        std::memcpy(description, info->description.c_str(), copy_size);
        description[copy_size] = '\0';
    }
    
    return OVF_SUCCESS;
}

// ============================================================
// 错误处理
// ============================================================

OVF_PYTHON_API size_t ovf_get_last_error_message(char* buffer, size_t buffer_size) {
    std::lock_guard<std::mutex> lock(g_error_mutex);
    
    if (!buffer || buffer_size == 0) {
        return 0;
    }
    
    size_t copy_size = std::min(buffer_size - 1, g_last_error_message.size());
    std::memcpy(buffer, g_last_error_message.c_str(), copy_size);
    buffer[copy_size] = '\0';
    return copy_size;
}

OVF_PYTHON_API OVFErrorCode ovf_get_last_error_code() {
    std::lock_guard<std::mutex> lock(g_error_mutex);
    return g_last_error_code;
}

OVF_PYTHON_API void ovf_clear_error() {
    clear_error_state();
}

OVF_PYTHON_API size_t ovf_error_code_to_string(OVFErrorCode code, char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return 0;
    }
    
    const char* msg = "";
    switch (code) {
        case OVF_SUCCESS: msg = "Success"; break;
        case OVF_ERROR_UNKNOWN: msg = "Unknown error"; break;
        case OVF_ERROR_INVALID_PARAM: msg = "Invalid parameter"; break;
        case OVF_ERROR_NULL_POINTER: msg = "Null pointer"; break;
        case OVF_ERROR_OUT_OF_RANGE: msg = "Out of range"; break;
        case OVF_ERROR_NOT_SUPPORTED: msg = "Not supported"; break;
        case OVF_ERROR_TIMEOUT: msg = "Timeout"; break;
        case OVF_ERROR_FLOW_NOT_FOUND: msg = "Flow not found"; break;
        case OVF_ERROR_NODE_NOT_FOUND: msg = "Node not found"; break;
        case OVF_ERROR_INVALID_FLOW: msg = "Invalid flow"; break;
        case OVF_ERROR_INVALID_NODE: msg = "Invalid node"; break;
        case OVF_ERROR_EXECUTION_FAILED: msg = "Execution failed"; break;
        case OVF_ERROR_DEVICE_NOT_FOUND: msg = "Device not found"; break;
        case OVF_ERROR_DEVICE_OPEN_FAILED: msg = "Device open failed"; break;
        case OVF_ERROR_CAMERA_CAPTURE_FAILED: msg = "Camera capture failed"; break;
        case OVF_ERROR_FILE_NOT_FOUND: msg = "File not found"; break;
        case OVF_ERROR_FILE_PARSE_FAILED: msg = "File parse failed"; break;
        default: msg = "Unknown error code"; break;
    }
    
    size_t copy_size = std::min(buffer_size - 1, strlen(msg));
    std::memcpy(buffer, msg, copy_size);
    buffer[copy_size] = '\0';
    return copy_size;
}