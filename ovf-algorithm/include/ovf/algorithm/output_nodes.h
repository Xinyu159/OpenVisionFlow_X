/**
 * @file output_nodes.h
 * @brief 输出节点（图像保存、数据输出）
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/data.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 图像保存节点
 */
class ImageSaveNode : public INode {
public:
    ImageSaveNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 结果输出节点
 */
class ResultOutputNode : public INode {
public:
    ResultOutputNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 图像显示节点（控制台输出图像信息）
 */
class ImageDisplayNode : public INode {
public:
    ImageDisplayNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief ROI裁剪节点
 */
class ROICropNode : public INode {
public:
    ROICropNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

/**
 * @brief 图像拼接节点
 */
class ImageConcatNode : public INode {
public:
    ImageConcatNode(const String& instance_id);
    Result<void> execute(FlowContext& context) override;
    
    static NodeInfo make_info();
};

} // namespace algorithm
} // namespace ovf