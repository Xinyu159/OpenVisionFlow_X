/**
 * @file camera_sync_nodes.cpp
 * @brief 相机同步节点实现
 */

#include "ovf/core/node.h"
#include "ovf/core/clock_sync.h"
#include "ovf/core/logger.h"
#include "ovf/core/data.h"
#include <algorithm>
#include <sstream>

namespace ovf {
namespace algorithm {

/**
 * @brief 多相机同步采集节点
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
    
    // 解析相机配置
    Result<void> parse_camera_config(const String& config);
};

MultiCameraAcquireNode::MultiCameraAcquireNode(const String& instance_id)
    : INode(instance_id, make_info()) {
    sync_manager_ = std::make_shared<core::MultiCameraSync>();
    buffer_ = std::make_shared<core::TimestampedBuffer>(100);
}

NodeInfo MultiCameraAcquireNode::make_info() {
    NodeInfo info;
    info.id = "MultiCameraAcquire";
    info.name = "多相机同步采集";
    info.category = "相机同步";
    info.description = "多相机同步采集节点，支持软同步、硬同步和主从模式";
    info.version = "0.1.0";
    
    // 输出端口
    info.outputs.push_back(DataPort("images", "图像数组", DataType::Array));
    info.outputs.push_back(DataPort("timestamps", "时间戳数组", DataType::Array));
    info.outputs.push_back(DataPort("camera_ids", "相机ID数组", DataType::Array));
    info.outputs.push_back(DataPort("sync_status", "同步状态", DataType::Object));
    
    // 参数定义
    info.params.push_back(ParamDef("sync_mode", "同步模式", DataType::String, Data("software")));
    info.params.push_back(ParamDef("frame_rate", "帧率", DataType::Number, Data(30.0)));
    info.params.push_back(ParamDef("cameras_config", "相机配置", DataType::String, Data("")));
    info.params.push_back(ParamDef("tolerance_us", "同步容差(us)", DataType::Number, Data(1000.0)));
    info.params.push_back(ParamDef("timeout_ms", "超时时间(ms)", DataType::Number, Data(1000)));
    info.params.push_back(ParamDef("continuous", "持续采集", DataType::Boolean, Data(false)));
    info.params.push_back(ParamDef("buffer_size", "缓冲区大小", DataType::Number, Data(100)));
    
    return info;
}

Result<void> MultiCameraAcquireNode::init() {
    // 获取参数
    String sync_mode_str = get_param("sync_mode", Data("software")).as_string();
    float frame_rate = get_param("frame_rate", Data(30.0)).as_number();
    String cameras_config = get_param("cameras_config", Data("")).as_string();
    float tolerance_us = get_param("tolerance_us", Data(1000.0)).as_number();
    int buffer_size = get_param("buffer_size", Data(100)).as_int();
    
    // 设置同步模式
    SyncMode mode = SyncMode::Software;
    if (sync_mode_str == "hardware") {
        mode = SyncMode::Hardware;
    } else if (sync_mode_str == "master_slave" || sync_mode_str == "masterslave") {
        mode = SyncMode::MasterSlave;
    }
    sync_manager_->set_sync_mode(mode);
    
    // 设置帧率
    sync_manager_->set_frame_rate(frame_rate);
    
    // 设置同步容差
    sync_manager_->set_tolerance(tolerance_us);
    
    // 解析相机配置
    auto result = parse_camera_config(cameras_config);
    if (result.is_failure()) {
        return result;
    }
    
    // 调整缓冲区大小
    buffer_ = std::make_shared<core::TimestampedBuffer>(buffer_size);
    
    // 设置同步回调
    sync_manager_->set_sync_callback([this](const Vector<core::SyncFrame>& frames) {
        for (const auto& frame : frames) {
            buffer_->push(frame);
        }
    });
    
    OVF_INFO() << "MultiCameraAcquireNode initialized: mode=" << sync_mode_str 
                 << ", fps=" << frame_rate << ", cameras=" << sync_manager_->get_camera_ids().size();
    
    return Result<void>::success();
}

Result<void> MultiCameraAcquireNode::execute(FlowContext& context) {
    bool continuous = get_param("continuous", Data(false)).as_bool();
    int timeout_ms = get_param("timeout_ms", Data(1000)).as_int();
    
    // 如果是持续采集模式，检查状态
    if (continuous) {
        if (!sync_manager_->is_running()) {
            ErrorCode result = sync_manager_->start_continuous();
            if (result != ErrorCode::Success) {
                return Result<void>::failure(result, "Failed to start continuous sync capture");
            }
        }
        
        // 从缓冲区获取最新的同步帧组
        Vector<core::SyncFrame> frames;
        core::SyncFrame frame;
        while (buffer_->pop_oldest(frame)) {
            frames.push_back(frame);
        }
        
        if (frames.empty()) {
            return Result<void>::failure(ErrorCode::SyncNotReady, "No synced frames available");
        }
        
        // 输出数据
        Vector<Data> images;
        Vector<Data> timestamps;
        Vector<Data> camera_ids;
        
        for (const auto& f : frames) {
            images.push_back(Data(f.image));
            timestamps.push_back(Data(static_cast<double>(f.capture_time)));
            camera_ids.push_back(Data(static_cast<double>(f.camera_id)));
        }
        
        // 输出同步状态
        core::SyncStatus status = sync_manager_->get_status();
        
        // 设置输出（需要扩展Data类来支持数组和对象）
        // 这里简化实现，输出单帧
        if (!frames.empty()) {
            set_output("images", Data(frames[0].image));
            set_output("timestamps", Data(static_cast<double>(frames[0].capture_time)));
            set_output("camera_ids", Data(static_cast<double>(frames[0].camera_id)));
        }
        
        return Result<void>::success();
    } else {
        // 单次采集模式
        Vector<core::SyncFrame> frames;
        ErrorCode result = sync_manager_->sync_capture(frames, timeout_ms);
        
        if (result != ErrorCode::Success) {
            return Result<void>::failure(result, "Sync capture failed");
        }
        
        if (frames.empty()) {
            return Result<void>::failure(ErrorCode::CameraCaptureFailed, "No frames captured");
        }
        
        // 输出数据
        Vector<Data> images;
        Vector<Data> timestamps;
        Vector<Data> camera_ids;
        
        for (const auto& f : frames) {
            images.push_back(Data(f.image));
            timestamps.push_back(Data(static_cast<double>(f.capture_time)));
            camera_ids.push_back(Data(static_cast<double>(f.camera_id)));
        }
        
        // 简化输出：输出第一个帧
        set_output("images", Data(frames[0].image));
        set_output("timestamps", Data(static_cast<double>(frames[0].capture_time)));
        set_output("camera_ids", Data(static_cast<double>(frames[0].camera_id)));
        
        // 输出同步状态
        core::SyncStatus status = sync_manager_->get_status();
        
        OVF_INFO() << "MultiCameraAcquireNode executed: " << frames.size() 
                     << " frames captured, synced=" << status.synced_count 
                     << ", dropped=" << status.dropped_count;
        
        return Result<void>::success();
    }
}

Result<void> MultiCameraAcquireNode::reset() {
    if (sync_manager_->is_running()) {
        sync_manager_->stop_continuous();
    }
    buffer_->clear();
    return Result<void>::success();
}

Result<void> MultiCameraAcquireNode::parse_camera_config(const String& config) {
    if (config.empty()) {
        // 默认配置：添加默认相机
        sync_manager_->add_camera(0, 0.0f);
        sync_manager_->add_camera(1, 0.0f);
        return Result<void>::success();
    }
    
    // 解析配置字符串（格式："id1:offset1,id2:offset2,...")
    std::istringstream iss(config);
    String token;
    
    while (std::getline(iss, token, ',')) {
        size_t pos = token.find(':');
        if (pos != String::npos) {
            uint32_t camera_id = static_cast<uint32_t>(std::stoul(token.substr(0, pos)));
            float offset_us = static_cast<float>(std::stod(token.substr(pos + 1)));
            sync_manager_->add_camera(camera_id, offset_us);
        } else {
            uint32_t camera_id = static_cast<uint32_t>(std::stoul(token));
            sync_manager_->add_camera(camera_id, 0.0f);
        }
    }
    
    return Result<void>::success();
}

/**
 * @brief 帧同步验证节点
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

FrameSyncCheckNode::FrameSyncCheckNode(const String& instance_id)
    : INode(instance_id, make_info()) {
}

NodeInfo FrameSyncCheckNode::make_info() {
    NodeInfo info;
    info.id = "FrameSyncCheck";
    info.name = "帧同步验证";
    info.category = "相机同步";
    info.description = "验证多帧是否在时间容差内同步";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("frames", "帧数据", DataType::Array, true));
    info.inputs.push_back(DataPort("timestamps", "时间戳", DataType::Array, true));
    
    // 输出端口
    info.outputs.push_back(DataPort("sync_status", "同步状态", DataType::Boolean));
    info.outputs.push_back(DataPort("synced_frames", "同步的帧", DataType::Array));
    info.outputs.push_back(DataPort("dropped_indices", "丢帧索引", DataType::Array));
    info.outputs.push_back(DataPort("sync_offset", "同步偏差(us)", DataType::Number));
    
    // 参数定义
    info.params.push_back(ParamDef("tolerance_us", "容差(us)", DataType::Number, Data(1000.0)));
    
    return info;
}

Result<void> FrameSyncCheckNode::execute(FlowContext& context) {
    float tolerance_us = get_param("tolerance_us", Data(1000.0)).as_number();
    
    // 获取输入的时间戳
    auto timestamps_data = get_input("timestamps");
    
    // 简化实现：假设输入是单个时间戳
    Vector<core::SyncFrame> frames;
    
    // 这里需要实际从输入获取帧数据
    // 目前创建示例帧用于演示
    core::SyncFrame frame;
    frame.capture_time = static_cast<core::ClockTimestamp>(timestamps_data.as_number());
    frame.camera_id = 0;
    frame.is_synced = true;
    frames.push_back(frame);
    
    // 检查同步状态
    bool is_synced = core::is_frame_synced(frames, tolerance_us);
    float sync_offset = core::calculate_sync_offset(frames);
    
    check_count_++;
    
    if (is_synced) {
        sync_success_count_++;
    } else {
        sync_failure_count_++;
    }
    
    // 输出结果
    set_output("sync_status", Data(is_synced));
    set_output("sync_offset", Data(sync_offset));
    
    OVF_INFO() << "FrameSyncCheckNode: check #" << check_count_
                 << ", synced=" << is_synced 
                 << ", offset=" << sync_offset << " us"
                 << ", success_rate=" << (static_cast<float>(sync_success_count_) / check_count_ * 100.0f) << "%";
    
    return Result<void>::success();
}

/**
 * @brief 时间戳对齐节点
 */
class TimestampAlignNode : public INode {
public:
    explicit TimestampAlignNode(const String& instance_id);
    
    static NodeInfo make_info();
    
    Result<void> execute(FlowContext& context) override;

private:
    core::ClockTimestamp reference_time_ = 0;
};

TimestampAlignNode::TimestampAlignNode(const String& instance_id)
    : INode(instance_id, make_info()) {
    reference_time_ = core::GlobalClock::instance().now();
}

NodeInfo TimestampAlignNode::make_info() {
    NodeInfo info;
    info.id = "TimestampAlign";
    info.name = "时间戳对齐";
    info.category = "相机同步";
    info.description = "将帧时间戳对齐到全局时钟";
    info.version = "0.1.0";
    
    // 输入端口
    info.inputs.push_back(DataPort("frame", "输入帧", DataType::Image, true));
    info.inputs.push_back(DataPort("reference_timestamp", "参考时间戳", DataType::Number, false));
    
    // 输出端口
    info.outputs.push_back(DataPort("aligned_frame", "对齐后的帧", DataType::Image));
    info.outputs.push_back(DataPort("aligned_timestamp", "对齐后的时间戳", DataType::Number));
    info.outputs.push_back(DataPort("time_offset", "时间偏移(us)", DataType::Number));
    
    // 参数定义
    info.params.push_back(ParamDef("clock_source", "时钟源", DataType::String, Data("system")));
    
    return info;
}

Result<void> TimestampAlignNode::execute(FlowContext& context) {
    // 获取时钟源参数
    String clock_source_str = get_param("clock_source", Data("system")).as_string();
    
    ClockSource source = ClockSource::System;
    if (clock_source_str == "hardware") {
        source = ClockSource::Hardware;
    } else if (clock_source_str == "ptp") {
        source = ClockSource::PTP;
    } else if (clock_source_str == "ntp") {
        source = ClockSource::NTP;
    }
    
    core::GlobalClock::instance().set_clock_source(source);
    
    // 获取输入帧
    auto frame_data = get_input("frame");
    if (!frame_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData frame = frame_data.as_image();
    
    // 获取参考时间戳（如果有）
    auto ref_ts_data = get_input("reference_timestamp");
    if (ref_ts_data.is_number()) {
        reference_time_ = static_cast<core::ClockTimestamp>(ref_ts_data.as_number());
    } else {
        reference_time_ = core::GlobalClock::instance().now();
    }
    
    // 获取当前全局时间
    core::ClockTimestamp current_time = core::GlobalClock::instance().now();
    
    // 计算时间偏移
    core::ClockTimestamp time_offset = 0;
    if (frame.timestamp > 0) {
        time_offset = current_time - static_cast<core::ClockTimestamp>(frame.timestamp);
    }
    
    // 对齐时间戳
    core::ClockTimestamp aligned_time = current_time;
    
    // 更新帧时间戳
    frame.timestamp = aligned_time;
    
    // 输出对齐后的帧
    set_output("aligned_frame", Data(frame));
    set_output("aligned_timestamp", Data(static_cast<double>(aligned_time)));
    set_output("time_offset", Data(static_cast<double>(time_offset)));
    
    OVF_INFO() << "TimestampAlignNode: aligned_time=" << core::timestamp_to_string(aligned_time)
                 << ", offset=" << time_offset << " us"
                 << ", clock_source=" << clock_source_str;
    
    return Result<void>::success();
}

// 注册节点
OVF_REGISTER_NODE(MultiCameraAcquireNode, "MultiCameraAcquire", MultiCameraAcquireNode::make_info());
OVF_REGISTER_NODE(FrameSyncCheckNode, "FrameSyncCheck", FrameSyncCheckNode::make_info());
OVF_REGISTER_NODE(TimestampAlignNode, "TimestampAlign", TimestampAlignNode::make_info());

} // namespace algorithm
} // namespace ovf