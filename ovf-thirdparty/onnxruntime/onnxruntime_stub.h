/**
 * @file onnxruntime_stub.h
 * @brief ONNX Runtime API Stub - 提供ONNX Runtime接口定义，无需实际链接库
 * 
 * 说明：
 * - 此文件仅提供ONNX Runtime API的类型和接口定义
 * - 实际运行时通过动态加载DLL实现推理功能
 * - 不需要链接ONNX Runtime库即可编译
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace ovf {
namespace onnx_runtime {

// ONNX Runtime API版本
constexpr const char* ONNX_RUNTIME_VERSION = "1.16.0";

// 执行提供器类型
enum class ExecutionProvider : int {
    CPU = 0,
    CUDA = 1,
    CUDA_DNNL = 2,
    OpenCL = 3,
    TensorRT = 4,
    DirectML = 5,
    Auto = 6  // 自动选择
};

// 数据类型（ONNX标准）
enum class ONNXTensorElementDataType : int {
    UNDEFINED = 0,
    FLOAT = 1,
    UINT8 = 2,
    INT8 = 3,
    UINT16 = 4,
    INT16 = 5,
    INT32 = 6,
    INT64 = 7,
    STRING = 8,
    BOOL = 9,
    FLOAT16 = 10,
    DOUBLE = 11,
    UINT32 = 12,
    UINT64 = 13,
    COMPLEX64 = 14,
    COMPLEX128 = 15,
    BFLOAT16 = 16
};

// 图优化级别
enum class GraphOptimizationLevel : int {
    DISABLE_ALL = 0,
    ENABLE_BASIC = 1,
    ENABLE_EXTENDED = 2,
    ENABLE_ALL = 99
};

// 内存模式
enum class MemoryPattern : int {
    DISABLE = 0,
    ENABLE = 1
};

// 运行选项 - stub结构
struct RunOptions {
    ExecutionProvider provider = ExecutionProvider::CPU;
    int num_threads = 4;
    GraphOptimizationLevel optimization_level = GraphOptimizationLevel::ENABLE_ALL;
    MemoryPattern memory_pattern = MemoryPattern::ENABLE;
    bool enable_profiling = false;
    bool enable_memory_pattern = true;
};

// 张量信息
struct TensorInfo {
    ONNXTensorElementDataType data_type;
    Vector<int64_t> shape;
    String name;
    size_t element_count;
    size_t byte_size;
};

// 会话选项 - stub结构
struct SessionOptions {
    ExecutionProvider execution_provider = ExecutionProvider::CPU;
    int intra_op_num_threads = 4;
    int inter_op_num_threads = 1;
    GraphOptimizationLevel graph_optimization_level = GraphOptimizationLevel::ENABLE_ALL;
    bool enable_profiling = false;
    String profile_file_prefix = "onnx_profile_";
    bool enable_memory_pattern = true;
    bool enable_sequential_execution = false;
    String log_id = "OpenVisionFlow";
};

} // namespace onnx_runtime
} // namespace ovf