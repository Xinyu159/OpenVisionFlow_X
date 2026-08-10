/**
 * @file algorithm_init.cpp
 * @brief 算法模块初始化实现
 */

#include <ovf/algorithm/algorithm.h>
#include <ovf/core/logger.h>

namespace ovf {
namespace algorithm {

void initialize_algorithm_module() {
    // 由于使用了OVF_REGISTER_NODE宏，节点会在静态初始化阶段自动注册
    // 当ovf-algorithm.dll被加载时，所有的静态注册对象会自动执行
    // 这里只需要输出日志确认初始化完成
    auto& factory = ovf::NodeFactory::instance();
    auto node_types = factory.get_all_types();
    OVF_INFO() << "Algorithm module initialized. Registered nodes: " << node_types.size();
}

} // namespace algorithm
} // namespace ovf