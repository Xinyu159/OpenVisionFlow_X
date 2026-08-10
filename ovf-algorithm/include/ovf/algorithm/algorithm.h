/**
 * @file algorithm.h
 * @brief OpenVisionFlow 算法模块总头文件
 * @author OpenVisionFlow Team
 * @version 0.2.0
 *
 * 此头文件包含了主要的算法模块（仅包含已验证无编译错误的模块）
 */

#pragma once

#include <ovf/core/node.h>
#include <ovf/core/types.h>

namespace ovf {
namespace algorithm {

/**
 * @brief 初始化算法模块，注册所有可用节点
 *
 * 此函数会自动注册所有算法模块中的节点到NodeFactory
 */
void initialize_algorithm_module();

} // namespace algorithm
} // namespace ovf