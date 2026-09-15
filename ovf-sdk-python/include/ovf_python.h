/**
 * @file ovf_python.h
 * @brief OpenVisionFlow Python SDK C接口 - 供Python绑定使用
 * 
 * 纯C接口设计，便于Python调用
 */

#pragma once

#include <stdint.h>
#include <stddef.h>  // size_t —— MSVC 传递包含，GCC 需显式引入

#ifdef __cplusplus
extern "C" {
#endif

// 导出宏
#if defined(_WIN32) || defined(__CYGWIN__)
    #ifdef OVF_PYTHON_EXPORTS
        #define OVF_PYTHON_API __declspec(dllexport)
    #else
        #define OVF_PYTHON_API __declspec(dllimport)
    #endif
#else
    #define OVF_PYTHON_API __attribute__((visibility("default")))
#endif

// 不透明句柄类型
typedef struct OVFImageHandle* OVFImage;
typedef struct OVFEngineHandle* OVFEngine;
typedef struct OVFNodeHandle* OVFNode;
typedef struct OVFCameraHandle* OVFCamera;
typedef struct OVFRunnerHandle* OVFRunner;

// 错误码定义
typedef enum {
    OVF_SUCCESS = 0,
    OVF_ERROR_UNKNOWN = 1,
    OVF_ERROR_INVALID_PARAM = 2,
    OVF_ERROR_NULL_POINTER = 3,
    OVF_ERROR_OUT_OF_RANGE = 4,
    OVF_ERROR_NOT_SUPPORTED = 5,
    OVF_ERROR_TIMEOUT = 6,
    OVF_ERROR_FLOW_NOT_FOUND = 100,
    OVF_ERROR_NODE_NOT_FOUND = 101,
    OVF_ERROR_INVALID_FLOW = 102,
    OVF_ERROR_INVALID_NODE = 103,
    OVF_ERROR_EXECUTION_FAILED = 105,
    OVF_ERROR_DEVICE_NOT_FOUND = 200,
    OVF_ERROR_DEVICE_OPEN_FAILED = 201,
    OVF_ERROR_CAMERA_CAPTURE_FAILED = 204,
    OVF_ERROR_FILE_NOT_FOUND = 500,
    OVF_ERROR_FILE_PARSE_FAILED = 502
} OVFErrorCode;

// 图像格式
typedef enum {
    OVF_IMAGE_FORMAT_UNKNOWN = 0,
    OVF_IMAGE_FORMAT_MONO8 = 1,
    OVF_IMAGE_FORMAT_MONO16 = 2,
    OVF_IMAGE_FORMAT_RGB8 = 3,
    OVF_IMAGE_FORMAT_BGR8 = 5
} OVFImageFormat;

// 节点状态
typedef enum {
    OVF_NODE_STATE_IDLE = 0,
    OVF_NODE_STATE_RUNNING = 1,
    OVF_NODE_STATE_SUCCESS = 2,
    OVF_NODE_STATE_FAILED = 3,
    OVF_NODE_STATE_DISABLED = 4
} OVFNodeState;

// ============================================================
// SDK 初始化/关闭
// ============================================================

/**
 * @brief 初始化SDK
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_initialize();

/**
 * @brief 关闭SDK
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_shutdown();

/**
 * @brief 获取SDK版本
 * @param major 主版本号
 * @param minor 次版本号
 * @param patch 补丁版本号
 */
OVF_PYTHON_API void ovf_get_version(int* major, int* minor, int* patch);

// ============================================================
// 图像操作
// ============================================================

/**
 * @brief 创建图像
 * @param width 宽度
 * @param height 高度
 * @param channels 通道数 (1=灰度, 3=RGB)
 * @return 图像句柄，失败返回NULL
 */
OVF_PYTHON_API OVFImage ovf_create_image(int width, int height, int channels);

/**
 * @brief 从数据创建图像
 * @param data 图像数据指针
 * @param width 宽度
 * @param height 高度
 * @param channels 通道数
 * @param data_size 数据大小
 * @return 图像句柄
 */
OVF_PYTHON_API OVFImage ovf_create_image_from_data(const uint8_t* data, int width, int height, 
                                                    int channels, size_t data_size);

/**
 * @brief 销毁图像
 * @param img 图像句柄
 */
OVF_PYTHON_API void ovf_destroy_image(OVFImage img);

/**
 * @brief 获取图像宽度
 * @param img 图像句柄
 * @return 宽度
 */
OVF_PYTHON_API int ovf_image_width(OVFImage img);

/**
 * @brief 获取图像高度
 * @param img 图像句柄
 * @return 高度
 */
OVF_PYTHON_API int ovf_image_height(OVFImage img);

/**
 * @brief 获取图像通道数
 * @param img 图像句柄
 * @return 通道数
 */
OVF_PYTHON_API int ovf_image_channels(OVFImage img);

/**
 * @brief 获取图像数据指针
 * @param img 图像句柄
 * @return 数据指针
 */
OVF_PYTHON_API uint8_t* ovf_image_data(OVFImage img);

/**
 * @brief 获取图像数据大小
 * @param img 图像句柄
 * @return 数据大小
 */
OVF_PYTHON_API size_t ovf_image_data_size(OVFImage img);

/**
 * @brief 复制图像数据到指定缓冲区
 * @param img 图像句柄
 * @param buffer 目标缓冲区
 * @param buffer_size 缓冲区大小
 * @return 实际复制的大小
 */
OVF_PYTHON_API size_t ovf_image_copy_data(OVFImage img, uint8_t* buffer, size_t buffer_size);

// ============================================================
// 流程引擎操作
// ============================================================

/**
 * @brief 创建流程引擎
 * @return 流程引擎句柄
 */
OVF_PYTHON_API OVFEngine ovf_create_engine();

/**
 * @brief 销毁流程引擎
 * @param engine 流程引擎句柄
 */
OVF_PYTHON_API void ovf_destroy_engine(OVFEngine engine);

/**
 * @brief 加载流程文件
 * @param engine 流程引擎句柄
 * @param filepath JSON流程文件路径
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_load_flow(OVFEngine engine, const char* filepath);

/**
 * @brief 从JSON字符串加载流程
 * @param engine 流程引擎句柄
 * @param json_content JSON内容字符串
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_load_flow_from_json(OVFEngine engine, const char* json_content);

/**
 * @brief 保存流程文件
 * @param engine 流程引擎句柄
 * @param filepath 目标文件路径
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_save_flow(OVFEngine engine, const char* filepath);

/**
 * @brief 获取流程JSON字符串
 * @param engine 流程引擎句柄
 * @param buffer 缓冲区
 * @param buffer_size 缓冲区大小
 * @return JSON字符串长度
 */
OVF_PYTHON_API size_t ovf_get_flow_json(OVFEngine engine, char* buffer, size_t buffer_size);

/**
 * @brief 运行流程
 * @param engine 流程引擎句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_run_flow(OVFEngine engine);

/**
 * @brief 运行指定节点
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_run_node(OVFEngine engine, const char* node_id);

/**
 * @brief 设置输入图像
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @param port_id 端口ID
 * @param img 图像句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_input_image(OVFEngine engine, const char* node_id, 
                                                 const char* port_id, OVFImage img);

/**
 * @brief 设置输入数值
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @param port_id 端口ID
 * @param value 数值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_input_number(OVFEngine engine, const char* node_id, 
                                                  const char* port_id, double value);

/**
 * @brief 设置输入字符串
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @param port_id 端口ID
 * @param value 字符串值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_input_string(OVFEngine engine, const char* node_id, 
                                                  const char* port_id, const char* value);

/**
 * @brief 获取输出图像
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @param port_id 端口ID
 * @return 图像句柄，失败返回NULL
 */
OVF_PYTHON_API OVFImage ovf_get_output_image(OVFEngine engine, const char* node_id, 
                                              const char* port_id);

/**
 * @brief 获取输出数值
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @param port_id 端口ID
 * @param value 输出数值指针
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_get_output_number(OVFEngine engine, const char* node_id, 
                                                   const char* port_id, double* value);

/**
 * @brief 获取节点数量
 * @param engine 流程引擎句柄
 * @return 节点数量
 */
OVF_PYTHON_API int ovf_get_node_count(OVFEngine engine);

/**
 * @brief 获取节点ID列表
 * @param engine 流程引擎句柄
 * @param ids 字符串数组缓冲区
 * @param max_count 最大数量
 * @param id_buffer_size 每个ID缓冲区大小
 * @return 实际数量
 */
OVF_PYTHON_API int ovf_get_node_ids(OVFEngine engine, char** ids, int max_count, size_t id_buffer_size);

/**
 * @brief 获取节点状态
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @return 节点状态
 */
OVF_PYTHON_API OVFNodeState ovf_get_node_state(OVFEngine engine, const char* node_id);

/**
 * @brief 获取节点执行时间(微秒)
 * @param engine 流程引擎句柄
 * @param node_id 节点实例ID
 * @return 执行时间
 */
OVF_PYTHON_API uint64_t ovf_get_node_execute_time(OVFEngine engine, const char* node_id);

// ============================================================
// 节点操作
// ============================================================

/**
 * @brief 创建节点
 * @param type_id 节点类型ID
 * @param instance_id 节点实例ID
 * @return 节点句柄
 */
OVF_PYTHON_API OVFNode ovf_create_node(const char* type_id, const char* instance_id);

/**
 * @brief 销毁节点
 * @param node 节点句柄
 */
OVF_PYTHON_API void ovf_destroy_node(OVFNode node);

/**
 * @brief 设置节点参数(整数)
 * @param node 节点句柄
 * @param key 参数键
 * @param value 参数值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_param_int(OVFNode node, const char* key, int value);

/**
 * @brief 设置节点参数(浮点数)
 * @param node 节点句柄
 * @param key 参数键
 * @param value 参数值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_param_float(OVFNode node, const char* key, double value);

/**
 * @brief 设置节点参数(字符串)
 * @param node 节点句柄
 * @param key 参数键
 * @param value 参数值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_param_string(OVFNode node, const char* key, const char* value);

/**
 * @brief 设置节点参数(布尔)
 * @param node 节点句柄
 * @param key 参数键
 * @param value 参数值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_param_bool(OVFNode node, const char* key, int value);

/**
 * @brief 添加节点到流程引擎
 * @param engine 流程引擎句柄
 * @param node 节点句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_add_node_to_engine(OVFEngine engine, OVFNode node);

/**
 * @brief 连接节点端口
 * @param engine 流程引擎句柄
 * @param source_node_id 源节点ID
 * @param source_port 源端口
 * @param target_node_id 目标节点ID
 * @param target_port 目标端口
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_connect_nodes(OVFEngine engine, 
                                               const char* source_node_id, const char* source_port,
                                               const char* target_node_id, const char* target_port);

// ============================================================
// 相机操作
// ============================================================

/**
 * @brief 打开相机
 * @param driver_type 驾动类型 (如 "mock", "genicam")
 * @param device_id 设备ID或IP地址
 * @return 相机句柄
 */
OVF_PYTHON_API OVFCamera ovf_open_camera(const char* driver_type, const char* device_id);

/**
 * @brief 关闭相机
 * @param cam 相机句柄
 */
OVF_PYTHON_API void ovf_close_camera(OVFCamera cam);

/**
 * @brief 开始采集
 * @param cam 相机句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_start_capture(OVFCamera cam);

/**
 * @brief 停止采集
 * @param cam 相机句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_stop_capture(OVFCamera cam);

/**
 * @brief 采集一帧
 * @param cam 相机句柄
 * @param timeout_ms 超时时间(毫秒)
 * @return 图像句柄
 */
OVF_PYTHON_API OVFImage ovf_capture_frame(OVFCamera cam, int timeout_ms);

/**
 * @brief 设置曝光时间
 * @param cam 相机句柄
 * @param exposure_us 曝光时间(微秒)
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_exposure(OVFCamera cam, double exposure_us);

/**
 * @brief 获取曝光时间
 * @param cam 相机句柄
 * @param exposure_us 输出曝光时间
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_get_exposure(OVFCamera cam, double* exposure_us);

/**
 * @brief 设置增益
 * @param cam 相机句柄
 * @param gain 增益值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_set_gain(OVFCamera cam, double gain);

/**
 * @brief 获取增益
 * @param cam 相机句柄
 * @param gain 输出增益值
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_get_gain(OVFCamera cam, double* gain);

/**
 * @brief 发送软触发
 * @param cam 相机句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_send_soft_trigger(OVFCamera cam);

/**
 * @brief 枚举相机设备
 * @param driver_type 驾动类型(空字符串表示所有)
 * @param ids 设备ID数组缓冲区
 * @param names 设备名称数组缓冲区
 * @param max_count 最大数量
 * @param buffer_size 每个字符串缓冲区大小
 * @return 实际数量
 */
OVF_PYTHON_API int ovf_enumerate_cameras(const char* driver_type, 
                                          char** ids, char** names, 
                                          int max_count, size_t buffer_size);

// ============================================================
// 流程运行器
// ============================================================

/**
 * @brief 创建流程运行器
 * @return 运行器句柄
 */
OVF_PYTHON_API OVFRunner ovf_create_runner();

/**
 * @brief 销毁流程运行器
 * @param runner 运行器句柄
 */
OVF_PYTHON_API void ovf_destroy_runner(OVFRunner runner);

/**
 * @brief 设置运行器关联的流程引擎
 * @param runner 运行器句柄
 * @param engine 流程引擎句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_runner_set_engine(OVFRunner runner, OVFEngine engine);

/**
 * @brief 启动连续运行
 * @param runner 运行器句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_runner_start_continuous(OVFRunner runner);

/**
 * @brief 启动触发模式
 * @param runner 运行器句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_runner_start_triggered(OVFRunner runner);

/**
 * @brief 停止运行
 * @param runner 运行器句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_runner_stop(OVFRunner runner);

/**
 * @brief 触发执行
 * @param runner 运行器句柄
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_runner_trigger(OVFRunner runner);

/**
 * @brief 检查运行器是否正在运行
 * @param runner 运行器句柄
 * @return 1=运行中, 0=停止
 */
OVF_PYTHON_API int ovf_runner_is_running(OVFRunner runner);

/**
 * @brief 获取运行次数统计
 * @param runner 运行器句柄
 * @param total_runs 总运行次数
 * @param success_runs 成功次数
 * @param failed_runs 失败次数
 */
OVF_PYTHON_API void ovf_runner_get_stats(OVFRunner runner, 
                                          uint64_t* total_runs, 
                                          uint64_t* success_runs, 
                                          uint64_t* failed_runs);

// ============================================================
// 节点类型注册信息
// ============================================================

/**
 * @brief 获取注册的节点类型数量
 * @return 类型数量
 */
OVF_PYTHON_API int ovf_get_registered_node_type_count();

/**
 * @brief 获取注册的节点类型ID列表
 * @param types 类型ID数组缓冲区
 * @param max_count 最大数量
 * @param buffer_size 每个字符串缓冲区大小
 * @return 实际数量
 */
OVF_PYTHON_API int ovf_get_registered_node_types(char** types, int max_count, size_t buffer_size);

/**
 * @brief 获取节点类型信息
 * @param type_id 节点类型ID
 * @param name 输出名称缓冲区
 * @param category 输出分类缓冲区
 * @param description 输出描述缓冲区
 * @param buffer_size 缓冲区大小
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_get_node_type_info(const char* type_id,
                                                    char* name, char* category, 
                                                    char* description, size_t buffer_size);

// ============================================================
// 错误处理
// ============================================================

/**
 * @brief 获取最后错误消息
 * @param buffer 缓冲区
 * @param buffer_size 缓冲区大小
 * @return 错误消息长度
 */
OVF_PYTHON_API size_t ovf_get_last_error_message(char* buffer, size_t buffer_size);

/**
 * @brief 获取最后错误码
 * @return 错误码
 */
OVF_PYTHON_API OVFErrorCode ovf_get_last_error_code();

/**
 * @brief 清除错误状态
 */
OVF_PYTHON_API void ovf_clear_error();

/**
 * @brief 错误码转字符串
 * @param code 错误码
 * @param buffer 缓冲区
 * @param buffer_size 缓冲区大小
 * @return 字符串长度
 */
OVF_PYTHON_API size_t ovf_error_code_to_string(OVFErrorCode code, char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif