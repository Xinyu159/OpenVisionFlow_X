/**
 * @file camera_sync_nodes.h
 * @brief 相机同步节点接口定义
 */

#pragma once

#include "ovf/core/node.h"
#include "ovf/core/clock_sync.h"

namespace ovf {
namespace algorithm {

/**
 * @brief 多相机同步采集节点
 * 
 * 支持多相机同步采集，包括：
 * - 软同步：软件触发同步采集
 * - 硬同步：硬件触发线同步
 * - 主从模式：主相机触发从相机
 */
class MultiCameraAcquireNode : public INode {
public:
    explicit MultiCameraAcquireNode(const String& instance_id);
    
    static NodeInfo make_info();
    
    Result<void> init() override;
    Result<void> execute(FlowContext& context) override;
    Result<void> reset() override;

private:
    std::shared_ptr<core::MultiCameraSync> sync_manager_;
    std::shared_ptr<core::TimestampedBuffer> buffer_;
    
    Result<void> parse_camera_config(const String& config);
};

/**
 * @brief 帧同步验证节点
 * 
 * 验证多帧是否在时间容差内同步，
 * 输出同步状态和丢帧信息
 */
class FrameSyncCheckNode : public INode {
public:
    explicit FrameSyncCheckNode(const String& instance_id);
    
    static NodeInfo make_info();
    
    Result<void> execute(FlowContext& context) override;

private:
    uint32_t check_count_ = 0;
    uint32_t sync_success_count_ = 0;
    uint32_t sync_failure_count_ = 0;
};

/**
 * @brief 时间戳对齐节点
 * 
 * 将帧时间戳对齐到全局时钟，
 * 支持多种时钟源（系统、硬件、PTP、NTP）
 */
class TimestampAlignNode : public INode {
public:
    explicit TimestampAlignNode(const String& instance_id);
    
    static NodeInfo make_info();
    
    Result<void> execute(FlowContext& context) override;

private:
    core::ClockTimestamp reference_time_ = 0;
};

} // namespace algorithm
} // namespace ovf