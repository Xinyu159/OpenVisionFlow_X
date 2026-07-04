/**
 * @file api.cpp
 * @brief SDK API实现
 */

#include "ovf/api.h"
#include "ovf/core/logger.h"

namespace ovf {
namespace api {

SDKVersion get_sdk_version() {
    return {VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, ""};
}

Result<void> initialize() {
    OVF_INFO() << "OpenVisionFlow SDK v" << VERSION << " initialized";
    return Result<void>::success();
}

Result<void> shutdown() {
    OVF_INFO() << "OpenVisionFlow SDK shutdown";
    return Result<void>::success();
}

FlowEngine::Ptr create_flow_engine() {
    return std::make_shared<FlowEngine>();
}

FlowRunner::Ptr create_flow_runner() {
    return std::make_shared<FlowRunner>();
}

Result<FlowDef> load_flow_file(const String& filepath) {
    // 暂时返回空流程定义
    FlowDef def;
    def.id = "flow_001";
    def.name = "默认流程";
    return Result<FlowDef>::success(def);
}

Result<void> save_flow_file(const String& filepath, const FlowDef& flow) {
    return Result<void>::success();
}

Vector<String> get_registered_node_types() {
    return NodeFactory::instance().get_all_types();
}

const NodeInfo* get_node_info(const String& type_id) {
    return NodeFactory::instance().get_info(type_id);
}

INode::Ptr create_node(const String& type_id, const String& instance_id) {
    return NodeFactory::instance().create(type_id, instance_id);
}

hal::HALManager& get_hal_manager() {
    return hal::HALManager::instance();
}

Vector<hal::DeviceInfo> enumerate_cameras() {
    return hal::HALManager::instance().enumerate_cameras();
}

hal::ICamera::Ptr create_camera(const String& driver_type, const hal::DeviceInfo& info) {
    return hal::HALManager::instance().create_camera(driver_type, info);
}

} // namespace api
} // namespace ovf