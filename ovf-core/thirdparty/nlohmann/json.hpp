/**
 * @file json.hpp
 * @brief Simplified JSON library for OpenVisionFlow
 * 
 * This is a minimal JSON implementation for internal use.
 * In production, use the full nlohmann/json library.
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include <initializer_list>

namespace nlohmann {

class json {
public:
    // 类型枚举
    enum class value_t {
        null,
        object,
        array,
        string,
        boolean,
        number_integer,
        number_unsigned,
        number_float,
        discarded
    };

    // 构造函数
    json() : type_(value_t::null) {}
    json(std::nullptr_t) : type_(value_t::null) {}
    json(bool val) : type_(value_t::boolean), bool_val_(val) {}
    json(int val) : type_(value_t::number_integer), int_val_(val) {}
    json(int64_t val) : type_(value_t::number_integer), int_val_(val) {}
    json(uint64_t val) : type_(value_t::number_unsigned), uint_val_(val) {}
    json(double val) : type_(value_t::number_float), float_val_(val) {}
    json(const char* val) : type_(value_t::string), string_val_(val) {}
    json(const std::string& val) : type_(value_t::string), string_val_(val) {}
    json(const std::vector<json>& val) : type_(value_t::array), array_val_(val) {}
    json(std::initializer_list<json> init) : type_(value_t::array), array_val_(init) {}
    
    // 复制和移动
    json(const json& other) { copy_from(other); }
    json(json&& other) noexcept { move_from(std::move(other)); }
    json& operator=(const json& other) { copy_from(other); return *this; }
    json& operator=(json&& other) noexcept { move_from(std::move(other)); return *this; }
    
    ~json() { destroy(); }

    // 类型检查
    bool is_null() const { return type_ == value_t::null; }
    bool is_boolean() const { return type_ == value_t::boolean; }
    bool is_number() const { return type_ == value_t::number_integer || 
                                    type_ == value_t::number_unsigned || 
                                    type_ == value_t::number_float; }
    bool is_number_integer() const { return type_ == value_t::number_integer || type_ == value_t::number_unsigned; }
    bool is_number_unsigned() const { return type_ == value_t::number_unsigned; }
    bool is_number_float() const { return type_ == value_t::number_float; }
    bool is_string() const { return type_ == value_t::string; }
    bool is_array() const { return type_ == value_t::array; }
    bool is_object() const { return type_ == value_t::object; }
    
    // 获取值
    bool get_bool() const {
        check_type(value_t::boolean);
        return bool_val_;
    }
    
    int64_t get_int64() const {
        if (type_ == value_t::number_integer) return int_val_;
        if (type_ == value_t::number_unsigned) return static_cast<int64_t>(uint_val_);
        if (type_ == value_t::number_float) return static_cast<int64_t>(float_val_);
        throw std::runtime_error("Invalid type for integer conversion");
    }
    
    int get_int() const { return static_cast<int>(get_int64()); }
    
    uint64_t get_uint64() const {
        if (type_ == value_t::number_unsigned) return uint_val_;
        if (type_ == value_t::number_integer) return static_cast<uint64_t>(int_val_);
        if (type_ == value_t::number_float) return static_cast<uint64_t>(float_val_);
        throw std::runtime_error("Invalid type for unsigned conversion");
    }
    
    double get_double() const {
        if (type_ == value_t::number_float) return float_val_;
        if (type_ == value_t::number_integer) return static_cast<double>(int_val_);
        if (type_ == value_t::number_unsigned) return static_cast<double>(uint_val_);
        throw std::runtime_error("Invalid type for float conversion");
    }
    
    std::string get_string() const {
        check_type(value_t::string);
        return string_val_;
    }
    
    const std::string& get_ref_string() const {
        check_type(value_t::string);
        return string_val_;
    }
    
    // 数组访问
    json& operator[](size_t idx) {
        check_type(value_t::array);
        if (idx >= array_val_.size()) {
            throw std::out_of_range("Array index out of range");
        }
        return array_val_[idx];
    }
    
    const json& operator[](size_t idx) const {
        check_type(value_t::array);
        if (idx >= array_val_.size()) {
            throw std::out_of_range("Array index out of range");
        }
        return array_val_[idx];
    }
    
    // 对象访问 - string
    json& operator[](const std::string& key) {
        if (is_null()) {
            type_ = value_t::object;
            object_val_ = std::map<std::string, json>();
        }
        check_type(value_t::object);
        return object_val_[key];
    }
    
    const json& operator[](const std::string& key) const {
        check_type(value_t::object);
        auto it = object_val_.find(key);
        if (it == object_val_.end()) {
            throw std::out_of_range("Key not found: " + key);
        }
        return it->second;
    }
    
    // 对象访问 - const char* (解决歧义问题)
    json& operator[](const char* key) {
        return operator[](std::string(key));
    }
    
    const json& operator[](const char* key) const {
        return operator[](std::string(key));
    }
    
    // 迭代器
    using iterator = std::vector<json>::iterator;
    using const_iterator = std::vector<json>::const_iterator;
    
    iterator begin() {
        check_type(value_t::array);
        return array_val_.begin();
    }
    
    iterator end() {
        check_type(value_t::array);
        return array_val_.end();
    }
    
    const_iterator begin() const {
        check_type(value_t::array);
        return array_val_.begin();
    }
    
    const_iterator end() const {
        check_type(value_t::array);
        return array_val_.end();
    }
    
    // 对象迭代器
    using object_iterator = std::map<std::string, json>::iterator;
    using object_const_iterator = std::map<std::string, json>::const_iterator;
    
    object_iterator object_begin() {
        check_type(value_t::object);
        return object_val_.begin();
    }
    
    object_iterator object_end() {
        check_type(value_t::object);
        return object_val_.end();
    }
    
    object_const_iterator object_begin() const {
        check_type(value_t::object);
        return object_val_.begin();
    }
    
    object_const_iterator object_end() const {
        check_type(value_t::object);
        return object_val_.end();
    }
    
    // 大小
    size_t size() const {
        if (is_array()) return array_val_.size();
        if (is_object()) return object_val_.size();
        if (is_string()) return string_val_.size();
        return 0;
    }
    
    bool empty() const {
        if (is_array()) return array_val_.empty();
        if (is_object()) return object_val_.empty();
        if (is_string()) return string_val_.empty();
        return is_null();
    }
    
    // 添加元素
    void push_back(const json& val) {
        if (is_null()) {
            type_ = value_t::array;
            array_val_ = std::vector<json>();
        }
        check_type(value_t::array);
        array_val_.push_back(val);
    }
    
    void push_back(json&& val) {
        if (is_null()) {
            type_ = value_t::array;
            array_val_ = std::vector<json>();
        }
        check_type(value_t::array);
        array_val_.push_back(std::move(val));
    }
    
    // 查找
    object_iterator find(const std::string& key) {
        check_type(value_t::object);
        return object_val_.find(key);
    }
    
    object_const_iterator find(const std::string& key) const {
        check_type(value_t::object);
        return object_val_.find(key);
    }
    
    bool contains(const std::string& key) const {
        if (!is_object()) return false;
        return object_val_.find(key) != object_val_.end();
    }
    
    // 类型转换
    operator bool() const { return !is_null(); }
    operator int() const { return get_int(); }
    operator int64_t() const { return get_int64(); }
    operator uint64_t() const { return get_uint64(); }
    operator double() const { return get_double(); }
    operator std::string() const { return get_string(); }
    
    // 值访问（带默认值）
    template<typename T>
    T value(const std::string& key, const T& default_val) const {
        if (!is_object()) return default_val;
        auto it = object_val_.find(key);
        if (it == object_val_.end()) return default_val;
        try {
            return static_cast<T>(it->second);
        } catch (...) {
            return default_val;
        }
    }
    
    std::string value(const std::string& key, const std::string& default_val) const {
        if (!is_object()) return default_val;
        auto it = object_val_.find(key);
        if (it == object_val_.end()) return default_val;
        if (it->second.is_string()) return it->second.get_string();
        return default_val;
    }
    
    int value(const std::string& key, int default_val) const {
        if (!is_object()) return default_val;
        auto it = object_val_.find(key);
        if (it == object_val_.end()) return default_val;
        if (it->second.is_number_integer()) return it->second.get_int();
        return default_val;
    }
    
    bool value(const std::string& key, bool default_val) const {
        if (!is_object()) return default_val;
        auto it = object_val_.find(key);
        if (it == object_val_.end()) return default_val;
        if (it->second.is_boolean()) return it->second.get_bool();
        return default_val;
    }
    
    // 序列化
    std::string dump(int indent = -1) const {
        std::ostringstream oss;
        serialize(oss, indent >= 0, indent >= 0 ? indent : 0);
        return oss.str();
    }
    
    // 静态方法
    static json object() {
        json j;
        j.type_ = value_t::object;
        j.object_val_ = std::map<std::string, json>();
        return j;
    }
    
    static json array() {
        json j;
        j.type_ = value_t::array;
        j.array_val_ = std::vector<json>();
        return j;
    }
    
    static json parse(const std::string& str) {
        // 简化解析实现
        size_t pos = 0;
        return parse_value(str, pos);
    }
    
    static json parse(std::istream& is) {
        std::string str;
        std::getline(is, str, '\0');
        return parse(str);
    }

private:
    value_t type_ = value_t::null;
    
    bool bool_val_ = false;
    int64_t int_val_ = 0;
    uint64_t uint_val_ = 0;
    double float_val_ = 0.0;
    std::string string_val_;
    std::vector<json> array_val_;
    std::map<std::string, json> object_val_;
    
    void destroy() {
        type_ = value_t::null;
        string_val_.clear();
        array_val_.clear();
        object_val_.clear();
    }
    
    void copy_from(const json& other) {
        type_ = other.type_;
        bool_val_ = other.bool_val_;
        int_val_ = other.int_val_;
        uint_val_ = other.uint_val_;
        float_val_ = other.float_val_;
        string_val_ = other.string_val_;
        array_val_ = other.array_val_;
        object_val_ = other.object_val_;
    }
    
    void move_from(json&& other) noexcept {
        type_ = other.type_;
        bool_val_ = other.bool_val_;
        int_val_ = other.int_val_;
        uint_val_ = other.uint_val_;
        float_val_ = other.float_val_;
        string_val_ = std::move(other.string_val_);
        array_val_ = std::move(other.array_val_);
        object_val_ = std::move(other.object_val_);
        other.destroy();
    }
    
    void check_type(value_t expected) const {
        if (type_ != expected) {
            throw std::runtime_error("Invalid JSON type");
        }
    }
    
    void serialize(std::ostream& os, bool pretty, int level) const {
        std::string indent_str = pretty ? std::string(level, ' ') : "";
        std::string next_indent = pretty ? std::string(level + 2, ' ') : "";
        
        switch (type_) {
            case value_t::null:
                os << "null";
                break;
                
            case value_t::boolean:
                os << (bool_val_ ? "true" : "false");
                break;
                
            case value_t::number_integer:
                os << int_val_;
                break;
                
            case value_t::number_unsigned:
                os << uint_val_;
                break;
                
            case value_t::number_float:
                os << float_val_;
                break;
                
            case value_t::string:
                os << "\"" << escape_string(string_val_) << "\"";
                break;
                
            case value_t::array:
                if (array_val_.empty()) {
                    os << "[]";
                } else {
                    os << "[";
                    if (pretty) os << "\n";
                    for (size_t i = 0; i < array_val_.size(); ++i) {
                        if (pretty) os << next_indent;
                        array_val_[i].serialize(os, pretty, level + 2);
                        if (i + 1 < array_val_.size()) {
                            os << ",";
                        }
                        if (pretty) os << "\n";
                    }
                    if (pretty) os << indent_str;
                    os << "]";
                }
                break;
                
            case value_t::object:
                if (object_val_.empty()) {
                    os << "{}";
                } else {
                    os << "{";
                    if (pretty) os << "\n";
                    size_t count = 0;
                    for (const auto& pair : object_val_) {
                        if (pretty) os << next_indent;
                        os << "\"" << escape_string(pair.first) << "\":";
                        if (pretty) os << " ";
                        pair.second.serialize(os, pretty, level + 2);
                        if (count + 1 < object_val_.size()) {
                            os << ",";
                        }
                        if (pretty) os << "\n";
                        ++count;
                    }
                    if (pretty) os << indent_str;
                    os << "}";
                }
                break;
                
            default:
                os << "null";
        }
    }
    
    std::string escape_string(const std::string& s) const {
        std::string result;
        for (char c : s) {
            switch (c) {
                case '"': result += "\\\""; break;
                case '\\': result += "\\\\"; break;
                case '\n': result += "\\n"; break;
                case '\r': result += "\\r"; break;
                case '\t': result += "\\t"; break;
                case '\b': result += "\\b"; break;
                case '\f': result += "\\f"; break;
                default:
                    // 其余控制字符必须转义，否则直接吐原始字节 = 非法 JSON。
                    // 0x80 及以上的字节原样输出 —— 那就是合法的 UTF-8。
                    if (static_cast<unsigned char>(c) < 0x20) {
                        static const char* hex = "0123456789abcdef";
                        result += "\\u00";
                        result += hex[(static_cast<unsigned char>(c) >> 4) & 0xF];
                        result += hex[static_cast<unsigned char>(c) & 0xF];
                    } else {
                        result += c;
                    }
            }
        }
        return result;
    }
    
    static json parse_value(const std::string& str, size_t& pos) {
        skip_whitespace(str, pos);
        
        if (pos >= str.size()) {
            return json();
        }
        
        char c = str[pos];
        
        if (c == '{') {
            return parse_object(str, pos);
        } else if (c == '[') {
            return parse_array(str, pos);
        } else if (c == '"') {
            return parse_string(str, pos);
        } else if (c == 't' || c == 'f') {
            return parse_boolean(str, pos);
        } else if (c == 'n') {
            return parse_null(str, pos);
        } else if (c == '-' || std::isdigit(c)) {
            return parse_number(str, pos);
        }
        
        throw std::runtime_error("Invalid JSON at position " + std::to_string(pos));
    }
    
    static void skip_whitespace(const std::string& str, size_t& pos) {
        while (pos < str.size() && std::isspace(str[pos])) {
            ++pos;
        }
    }
    
    static json parse_object(const std::string& str, size_t& pos) {
        json obj = json::object();
        ++pos; // skip '{'
        skip_whitespace(str, pos);
        
        if (str[pos] == '}') {
            ++pos;
            return obj;
        }
        
        while (pos < str.size()) {
            skip_whitespace(str, pos);
            
            // Parse key
            std::string key = parse_string(str, pos).get_string();
            
            skip_whitespace(str, pos);
            if (str[pos] != ':') {
                throw std::runtime_error("Expected ':' in object");
            }
            ++pos;
            
            // Parse value
            obj[key] = parse_value(str, pos);
            
            skip_whitespace(str, pos);
            if (str[pos] == '}') {
                ++pos;
                return obj;
            }
            if (str[pos] == ',') {
                ++pos;
            } else {
                throw std::runtime_error("Expected ',' or '}' in object");
            }
        }
        
        throw std::runtime_error("Unterminated object");
    }
    
    static json parse_array(const std::string& str, size_t& pos) {
        json arr = json::array();
        ++pos; // skip '['
        skip_whitespace(str, pos);
        
        if (str[pos] == ']') {
            ++pos;
            return arr;
        }
        
        while (pos < str.size()) {
            arr.push_back(parse_value(str, pos));
            
            skip_whitespace(str, pos);
            if (str[pos] == ']') {
                ++pos;
                return arr;
            }
            if (str[pos] == ',') {
                ++pos;
            } else {
                throw std::runtime_error("Expected ',' or ']' in array");
            }
        }
        
        throw std::runtime_error("Unterminated array");
    }
    
    // 把 Unicode 码点按 UTF-8 追加到结果（\uXXXX 转义要用）
    static void append_utf8(std::string& out, uint32_t cp) {
        if (cp <= 0x7F) {
            out += static_cast<char>(cp);
        } else if (cp <= 0x7FF) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp <= 0xFFFF) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    // 从 s[pos] 起读 4 位十六进制，成功返回 true
    static bool read_hex4(const std::string& s, size_t pos, uint32_t& out) {
        if (pos + 4 > s.size()) return false;
        out = 0;
        for (int i = 0; i < 4; ++i) {
            char h = s[pos + i];
            out <<= 4;
            if (h >= '0' && h <= '9')      out |= static_cast<uint32_t>(h - '0');
            else if (h >= 'a' && h <= 'f') out |= static_cast<uint32_t>(h - 'a' + 10);
            else if (h >= 'A' && h <= 'F') out |= static_cast<uint32_t>(h - 'A' + 10);
            else return false;
        }
        return true;
    }

    static json parse_string(const std::string& str, size_t& pos) {
        ++pos; // skip opening '"'
        std::string result;

        while (pos < str.size() && str[pos] != '"') {
            if (str[pos] == '\\') {
                ++pos;
                if (pos < str.size()) {
                    char esc = str[pos];
                    switch (esc) {
                        case '"': result += '"'; break;
                        case '\\': result += '\\'; break;
                        case '/': result += '/'; break;
                        case 'n': result += '\n'; break;
                        case 'r': result += '\r'; break;
                        case 't': result += '\t'; break;
                        case 'b': result += '\b'; break;
                        case 'f': result += '\f'; break;
                        case 'u': {
                            // \uXXXX。原先落进 default 分支，只把反斜杠丢掉、留下 "u524d"，
                            // 中文名走 python/requests 这类默认 ensure_ascii 的客户端存进
                            // 流程文件就成了乱码 —— 浏览器 JSON.stringify 不转义非 ASCII，
                            // 所以这条只在部分客户端上爆，更容易被漏掉。
                            uint32_t cp = 0;
                            if (!read_hex4(str, pos + 1, cp)) {
                                result += esc;  // 非法转义，按原样保留
                                break;
                            }
                            pos += 4;
                            // UTF-16 代理对：高位 D800-DBFF 后面紧跟 \uDC00-\uDFFF
                            if (cp >= 0xD800 && cp <= 0xDBFF && pos + 2 < str.size() &&
                                str[pos + 1] == '\\' && str[pos + 2] == 'u') {
                                uint32_t lo = 0;
                                if (read_hex4(str, pos + 3, lo) && lo >= 0xDC00 && lo <= 0xDFFF) {
                                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                                    pos += 6;
                                }
                            }
                            append_utf8(result, cp);
                            break;
                        }
                        default: result += esc;
                    }
                    ++pos;
                }
            } else {
                result += str[pos];
                ++pos;
            }
        }
        
        if (pos >= str.size()) {
            throw std::runtime_error("Unterminated string");
        }
        ++pos; // skip closing '"'
        
        return json(result);
    }
    
    static json parse_boolean(const std::string& str, size_t& pos) {
        if (str.substr(pos, 4) == "true") {
            pos += 4;
            return json(true);
        }
        if (str.substr(pos, 5) == "false") {
            pos += 5;
            return json(false);
        }
        throw std::runtime_error("Invalid boolean value");
    }
    
    static json parse_null(const std::string& str, size_t& pos) {
        if (str.substr(pos, 4) == "null") {
            pos += 4;
            return json();
        }
        throw std::runtime_error("Invalid null value");
    }
    
    static json parse_number(const std::string& str, size_t& pos) {
        size_t start = pos;
        
        if (str[pos] == '-') ++pos;
        
        while (pos < str.size() && std::isdigit(str[pos])) ++pos;
        
        bool is_float = false;
        if (pos < str.size() && str[pos] == '.') {
            is_float = true;
            ++pos;
            while (pos < str.size() && std::isdigit(str[pos])) ++pos;
        }
        
        if (pos < str.size() && (str[pos] == 'e' || str[pos] == 'E')) {
            is_float = true;
            ++pos;
            if (pos < str.size() && (str[pos] == '+' || str[pos] == '-')) ++pos;
            while (pos < str.size() && std::isdigit(str[pos])) ++pos;
        }
        
        std::string num_str = str.substr(start, pos - start);
        
        if (is_float) {
            return json(std::stod(num_str));
        } else {
            int64_t val = std::stoll(num_str);
            if (val >= 0) {
                return json(static_cast<uint64_t>(val));
            }
            return json(val);
        }
    }
};

} // namespace nlohmann