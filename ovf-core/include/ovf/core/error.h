/**
 * @file error.h
 * @brief OpenVisionFlow 错误处理模块
 */

#pragma once

#include "types.h"
#include <string>
#include <sstream>
#include <vector>
#include <stdexcept>

namespace ovf {

/**
 * @brief 异常类
 */
class Exception : public std::runtime_error {
public:
    Exception(ErrorCode code, const String& message)
        : std::runtime_error(format_message(code, message))
        , code_(code)
        , message_(message) {}
    
    ErrorCode code() const { return code_; }
    const String& message() const { return message_; }
    
    static String format_message(ErrorCode code, const String& msg) {
        return "[Error " + std::to_string(static_cast<int>(code)) + "] " + msg;
    }

private:
    ErrorCode code_;
    String message_;
};

/**
 * @brief 结果类 - 用于返回操作结果
 */
template<typename T = void>
class Result {
public:
    // 成功构造
    static Result<T> success(const T& value = T{}) {
        return Result<T>(value, ErrorCode::Success, "");
    }
    
    // 失败构造
    static Result<T> failure(ErrorCode code, const String& message = "") {
        return Result<T>(T{}, code, message);
    }
    
    bool is_success() const { return code_ == ErrorCode::Success; }
    bool is_failure() const { return code_ != ErrorCode::Success; }
    
    const T& value() const { 
        if (is_failure()) {
            throw Exception(code_, message_);
        }
        return value_; 
    }
    
    T& value() { 
        if (is_failure()) {
            throw Exception(code_, message_);
        }
        return value_; 
    }
    
    ErrorCode code() const { return code_; }
    const String& message() const { return message_; }
    
    // 操作符重载
    explicit operator bool() const { return is_success(); }
    const T& operator*() const { return value(); }
    T& operator*() { return value(); }
    const T* operator->() const { return &value_; }
    T* operator->() { return &value_; }

private:
    Result(const T& value, ErrorCode code, const String& message)
        : value_(value), code_(code), message_(message) {}
    
    T value_;
    ErrorCode code_;
    String message_;
};

// void 特化
template<>
class Result<void> {
public:
    static Result<void> success() {
        return Result<void>(ErrorCode::Success, "");
    }
    
    static Result<void> failure(ErrorCode code, const String& message = "") {
        return Result<void>(code, message);
    }
    
    bool is_success() const { return code_ == ErrorCode::Success; }
    bool is_failure() const { return code_ != ErrorCode::Success; }
    ErrorCode code() const { return code_; }
    const String& message() const { return message_; }
    explicit operator bool() const { return is_success(); }

private:
    Result(ErrorCode code, const String& message)
        : code_(code), message_(message) {}
    
    ErrorCode code_;
    String message_;
};

/**
 * @brief 错误信息结构
 */
struct ErrorInfo {
    ErrorCode code = ErrorCode::Success;
    String message;
    String file;
    int line = 0;
    String function;
    std::vector<String> call_stack;
    
    String to_string() const {
        std::ostringstream oss;
        oss << "[Error " << static_cast<int>(code) << "] " << message;
        if (!file.empty()) {
            oss << "\n  at " << file << ":" << line;
            if (!function.empty()) {
                oss << " in " << function;
            }
        }
        if (!call_stack.empty()) {
            oss << "\n  Call Stack:";
            for (const auto& frame : call_stack) {
                oss << "\n    " << frame;
            }
        }
        return oss.str();
    }
};

// 错误处理宏
#define OVF_THROW(code, message) \
    throw ovf::Exception(code, message)

#define OVF_CHECK(condition, code, message) \
    if (!(condition)) { \
        throw ovf::Exception(code, message); \
    }

#define OVF_RETURN_IF_ERROR(result) \
    if (result.is_failure()) { \
        return result; \
    }

} // namespace ovf