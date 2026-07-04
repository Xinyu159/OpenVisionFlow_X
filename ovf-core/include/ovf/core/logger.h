/**
 * @file logger.h
 * @brief OpenVisionFlow 日志系统
 */

#pragma once

#include "types.h"
#include <memory>
#include <sstream>
#include <mutex>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <iostream>

namespace ovf {

// 日志级别
enum class LogLevel : uint8_t {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warning = 3,
    Error = 4,
    Fatal = 5,
    None = 255
};

// 日志记录器接口
class ILogSink {
public:
    virtual ~ILogSink() = default;
    virtual void write(LogLevel level, const String& message, 
                      const String& file, int line, const String& function) = 0;
};

// 控制台日志输出
class ConsoleLogSink : public ILogSink {
public:
    void write(LogLevel level, const String& message, 
               const String& file, int line, const String& function) override {
        const char* level_str[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::cout << "[" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
                  << "." << std::setfill('0') << std::setw(3) << ms.count() << "]"
                  << "[" << level_str[static_cast<int>(level)] << "]"
                  << " " << message;
        
        if (level >= LogLevel::Warning) {
            std::cout << " (" << file << ":" << line << " in " << function << ")";
        }
        std::cout << std::endl;
    }
};

// 文件日志输出
class FileLogSink : public ILogSink {
public:
    explicit FileLogSink(const String& filename) {
        file_.open(filename, std::ios::app);
    }
    
    ~FileLogSink() {
        if (file_.is_open()) {
            file_.close();
        }
    }
    
    void write(LogLevel level, const String& message, 
               const String& file, int line, const String& function) override {
        if (!file_.is_open()) return;
        
        const char* level_str[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        
        std::lock_guard<std::mutex> lock(mutex_);
        file_ << "[" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
              << "][" << level_str[static_cast<int>(level)] << "]"
              << " " << message
              << " (" << file << ":" << line << " in " << function << ")"
              << std::endl;
    }

private:
    std::ofstream file_;
    std::mutex mutex_;
};

// 日志管理器
class Logger {
public:
    static Logger& instance() {
        static Logger logger;
        return logger;
    }
    
    void set_level(LogLevel level) { level_ = level; }
    LogLevel level() const { return level_; }
    
    void add_sink(Ptr<ILogSink> sink) {
        std::lock_guard<std::mutex> lock(mutex_);
        sinks_.push_back(sink);
    }
    
    void clear_sinks() {
        std::lock_guard<std::mutex> lock(mutex_);
        sinks_.clear();
    }
    
    void log(LogLevel level, const String& message,
             const String& file, int line, const String& function) {
        if (level < level_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& sink : sinks_) {
            sink->write(level, message, file, line, function);
        }
    }

private:
    Logger() : level_(LogLevel::Info) {
        add_sink(std::make_shared<ConsoleLogSink>());
    }
    
    LogLevel level_;
    Vector<Ptr<ILogSink>> sinks_;
    std::mutex mutex_;
};

// 日志流辅助类
class LogStream {
public:
    LogStream(LogLevel level, const String& file, int line, const String& function)
        : level_(level), file_(file), line_(line), function_(function) {}
    
    ~LogStream() {
        Logger::instance().log(level_, stream_.str(), file_, line_, function_);
    }
    
    template<typename T>
    LogStream& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    LogLevel level_;
    String file_;
    int line_;
    String function_;
    std::ostringstream stream_;
};

// 日志宏
#define OVF_TRACE() ovf::LogStream(ovf::LogLevel::Trace, __FILE__, __LINE__, __FUNCTION__)
#define OVF_DEBUG() ovf::LogStream(ovf::LogLevel::Debug, __FILE__, __LINE__, __FUNCTION__)
#define OVF_INFO() ovf::LogStream(ovf::LogLevel::Info, __FILE__, __LINE__, __FUNCTION__)
#define OVF_WARN() ovf::LogStream(ovf::LogLevel::Warning, __FILE__, __LINE__, __FUNCTION__)
#define OVF_ERROR() ovf::LogStream(ovf::LogLevel::Error, __FILE__, __LINE__, __FUNCTION__)
#define OVF_FATAL() ovf::LogStream(ovf::LogLevel::Fatal, __FILE__, __LINE__, __FUNCTION__)

} // namespace ovf