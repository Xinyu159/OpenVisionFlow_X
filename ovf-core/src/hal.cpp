/**
 * @file hal.cpp
 * @brief 硬件抽象层实现
 */

#include "ovf/hal/hal.h"
#include "ovf/core/logger.h"
#include <mutex>

namespace ovf {
namespace hal {

// ============== HALManager 实现 ==============

ICamera::Ptr HALManager::create_camera(const String& driver_type, const DeviceInfo& info) {
    auto it = creators_.find(driver_type);
    if (it == creators_.end()) {
        OVF_ERROR() << "Camera driver not found: " << driver_type;
        return nullptr;
    }
    
    auto device = it->second(info);
    if (device) {
        add_device(device);
    }
    
    // 尝试转换为ICamera
    return std::dynamic_pointer_cast<ICamera>(device);
}

ILightSource::Ptr HALManager::create_light_source(const String& driver_type, const DeviceInfo& info) {
    auto it = creators_.find(driver_type);
    if (it == creators_.end()) {
        return nullptr;
    }
    
    auto device = it->second(info);
    if (device) {
        add_device(device);
    }
    
    return std::dynamic_pointer_cast<ILightSource>(device);
}

IMotionController::Ptr HALManager::create_motion_controller(const String& driver_type, const DeviceInfo& info) {
    auto it = creators_.find(driver_type);
    if (it == creators_.end()) {
        return nullptr;
    }
    
    auto device = it->second(info);
    if (device) {
        add_device(device);
    }
    
    return std::dynamic_pointer_cast<IMotionController>(device);
}

IIODevice::Ptr HALManager::create_io_device(const String& driver_type, const DeviceInfo& info) {
    auto it = creators_.find(driver_type);
    if (it == creators_.end()) {
        return nullptr;
    }
    
    auto device = it->second(info);
    if (device) {
        add_device(device);
    }
    
    return std::dynamic_pointer_cast<IIODevice>(device);
}

Vector<DeviceInfo> HALManager::enumerate_cameras(const String& driver_type) {
    // 暂时返回空列表，实际需要通过驱动枚举
    Vector<DeviceInfo> result;
    
    // 添加模拟相机
    if (driver_type.empty() || driver_type == "mock_camera") {
        DeviceInfo mock_info;
        mock_info.id = "mock_camera_001";
        mock_info.name = "Mock Camera";
        mock_info.vendor = "OVF";
        mock_info.model = "Mock";
        mock_info.driver_type = "mock_camera";
        result.push_back(mock_info);
    }
    
    return result;
}

Vector<DeviceInfo> HALManager::enumerate_devices(const String& device_type) {
    return enumerate_cameras(device_type);
}

void HALManager::add_device(IDevice::Ptr device) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (device) {
        devices_[device->info().id] = device;
    }
}

void HALManager::remove_device(const String& device_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    devices_.erase(device_id);
}

IDevice::Ptr HALManager::get_device(const String& device_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = devices_.find(device_id);
    return it != devices_.end() ? it->second : nullptr;
}

Vector<IDevice::Ptr> HALManager::get_all_devices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<IDevice::Ptr> result;
    for (const auto& pair : devices_) {
        result.push_back(pair.second);
    }
    return result;
}

Vector<ICamera::Ptr> HALManager::get_all_cameras() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<ICamera::Ptr> result;
    for (const auto& pair : devices_) {
        auto camera = std::dynamic_pointer_cast<ICamera>(pair.second);
        if (camera) {
            result.push_back(camera);
        }
    }
    return result;
}

Vector<ILightSource::Ptr> HALManager::get_all_light_sources() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<ILightSource::Ptr> result;
    for (const auto& pair : devices_) {
        auto light = std::dynamic_pointer_cast<ILightSource>(pair.second);
        if (light) {
            result.push_back(light);
        }
    }
    return result;
}

Vector<IMotionController::Ptr> HALManager::get_all_motion_controllers() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<IMotionController::Ptr> result;
    for (const auto& pair : devices_) {
        auto motion = std::dynamic_pointer_cast<IMotionController>(pair.second);
        if (motion) {
            result.push_back(motion);
        }
    }
    return result;
}

Vector<IIODevice::Ptr> HALManager::get_all_io_devices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<IIODevice::Ptr> result;
    for (const auto& pair : devices_) {
        auto io = std::dynamic_pointer_cast<IIODevice>(pair.second);
        if (io) {
            result.push_back(io);
        }
    }
    return result;
}

} // namespace hal
} // namespace ovf