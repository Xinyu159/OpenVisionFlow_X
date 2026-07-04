/**
 * @file api.h
 * @brief OpenVisionFlow C++ SDK 公共API
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/flow.h"
#include "ovf/core/node.h"
#include "ovf/hal/hal.h"

namespace ovf {
namespace api {

/**
 * @brief SDK版本信息
 */
struct SDKVersion {
    int major;
    int minor;
    int patch;
    String build;
};

/**
 * @brief 获取SDK版本
 */
SDKVersion get_sdk_version();

/**
 * @brief 初始化SDK
 */
Result<void> initialize();

/**
 * @brief 关闭SDK
 */
Result<void> shutdown();

/**
 * @brief 创建流程引擎
 */
FlowEngine::Ptr create_flow_engine();

/**
 * @brief 创建流程运行器
 */
FlowRunner::Ptr create_flow_runner();

/**
 * @brief 加载流程文件
 */
Result<FlowDef> load_flow_file(const String& filepath);

/**
 * @brief 保存流程文件
 */
Result<void> save_flow_file(const String& filepath, const FlowDef& flow);

/**
 * @brief 获取所有注册的节点类型
 */
Vector<String> get_registered_node_types();

/**
 * @brief 获取节点类型信息
 */
const NodeInfo* get_node_info(const String& type_id);

/**
 * @brief 创建节点
 */
INode::Ptr create_node(const String& type_id, const String& instance_id);

/**
 * @brief 获取硬件管理器
 */
hal::HALManager& get_hal_manager();

/**
 * @brief 枚举所有相机
 */
Vector<hal::DeviceInfo> enumerate_cameras();

/**
 * @brief 创建相机
 */
hal::ICamera::Ptr create_camera(const String& driver_type, const hal::DeviceInfo& info);

} // namespace api
} // namespace ovf

// 导出宏
#if defined(_WIN32) || defined(__CYGWIN__)
    #ifdef OVF_SDK_EXPORTS
        #define OVF_API __declspec(dllexport)
    #else
        #define OVF_API __declspec(dllimport)
    #endif
#else
    #define OVF_API __attribute__((visibility("default")))
#endif