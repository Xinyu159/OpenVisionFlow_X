/**
 * @file data.h
 * @brief OpenVisionFlow 数据容器
 */

#pragma once

#include "types.h"
#include <any>
#include <variant>

namespace ovf {

/**
 * @brief 数据容器 - 用于节点间数据传递
 */
class Data {
public:
    Data() : type_(DataType::None) {}
    
    // 从各种类型构造
    explicit Data(int32_t value) : type_(DataType::Number), value_(static_cast<double>(value)) {}
    explicit Data(int64_t value) : type_(DataType::Number), value_(static_cast<double>(value)) {}
    explicit Data(float value) : type_(DataType::Number), value_(static_cast<double>(value)) {}
    explicit Data(double value) : type_(DataType::Number), value_(value) {}
    explicit Data(bool value) : type_(DataType::Boolean), value_(value) {}
    explicit Data(const char* value) : type_(DataType::String), value_(String(value)) {}
    explicit Data(const String& value) : type_(DataType::String), value_(value) {}
    explicit Data(String&& value) : type_(DataType::String), value_(std::move(value)) {}
    explicit Data(const ImageData& value) : type_(DataType::Image), value_(value) {}
    explicit Data(ImageData&& value) : type_(DataType::Image), value_(std::move(value)) {}
    explicit Data(const Region& value) : type_(DataType::Region), value_(value) {}
    explicit Data(const Pose& value) : type_(DataType::Pose), value_(value) {}
    explicit Data(const Point3Df& value) : type_(DataType::Point), value_(value) {}
    explicit Data(const PointCloudData& value) : type_(DataType::PointCloud), value_(value) {}
    explicit Data(PointCloudData&& value) : type_(DataType::PointCloud), value_(std::move(value)) {}
    explicit Data(const DepthImageData& value) : type_(DataType::DepthImage), value_(value) {}
    explicit Data(DepthImageData&& value) : type_(DataType::DepthImage), value_(std::move(value)) {}
    
    // 类型检查
    DataType type() const { return type_; }
    bool is_valid() const { return type_ != DataType::None; }
    bool is_none() const { return type_ == DataType::None; }
    bool is_number() const { return type_ == DataType::Number; }
    bool is_string() const { return type_ == DataType::String; }
    bool is_bool() const { return type_ == DataType::Boolean; }
    bool is_image() const { return type_ == DataType::Image; }
    bool is_region() const { return type_ == DataType::Region; }
    bool is_pose() const { return type_ == DataType::Pose; }
    bool is_point() const { return type_ == DataType::Point; }
    bool is_pointcloud() const { return type_ == DataType::PointCloud; }
    bool is_depth_image() const { return type_ == DataType::DepthImage; }
    
    // 获取值
    double as_number(double default_val = 0.0) const {
        return is_number() ? std::get<double>(value_) : default_val;
    }
    
    int32_t as_int(int32_t default_val = 0) const {
        return is_number() ? static_cast<int32_t>(std::get<double>(value_)) : default_val;
    }
    
    const String& as_string(const String& default_val = "") const {
        return is_string() ? std::get<String>(value_) : default_val;
    }
    
    bool as_bool(bool default_val = false) const {
        return is_bool() ? std::get<bool>(value_) : default_val;
    }
    
    const ImageData& as_image() const {
        static ImageData empty;
        return is_image() ? std::get<ImageData>(value_) : empty;
    }
    
    ImageData& as_image() {
        static ImageData empty;
        return is_image() ? std::get<ImageData>(value_) : empty;
    }
    
    const Region& as_region() const {
        static Region empty;
        return is_region() ? std::get<Region>(value_) : empty;
    }
    
    const Pose& as_pose() const {
        static Pose empty;
        return is_pose() ? std::get<Pose>(value_) : empty;
    }
    
    const Point3Df& as_point() const {
        static Point3Df empty;
        return is_point() ? std::get<Point3Df>(value_) : empty;
    }
    
    Point3Df& as_point() {
        static Point3Df empty;
        return is_point() ? std::get<Point3Df>(value_) : empty;
    }
    
    const PointCloudData& as_pointcloud() const {
        static PointCloudData empty;
        return is_pointcloud() ? std::get<PointCloudData>(value_) : empty;
    }
    
    PointCloudData& as_pointcloud() {
        static PointCloudData empty;
        return is_pointcloud() ? std::get<PointCloudData>(value_) : empty;
    }
    
    const DepthImageData& as_depth_image() const {
        static DepthImageData empty;
        return is_depth_image() ? std::get<DepthImageData>(value_) : empty;
    }
    
    DepthImageData& as_depth_image() {
        static DepthImageData empty;
        return is_depth_image() ? std::get<DepthImageData>(value_) : empty;
    }
    
    // 转换为字符串
    String to_string() const {
        switch (type_) {
            case DataType::None: return "None";
            case DataType::Number: return std::to_string(std::get<double>(value_));
            case DataType::String: return std::get<String>(value_);
            case DataType::Boolean: return std::get<bool>(value_) ? "true" : "false";
            case DataType::Image: return "Image[" + 
                std::to_string(as_image().width) + "x" + 
                std::to_string(as_image().height) + "]";
            case DataType::Region: {
                const auto& r = std::get<Region>(value_);
                return "Region(" + std::to_string(r.x) + "," + std::to_string(r.y) + "," +
                       std::to_string(r.width) + "," + std::to_string(r.height) + ")";
            }
            case DataType::Pose: {
                const auto& p = std::get<Pose>(value_);
                return "Pose(" + std::to_string(p.x) + "," + std::to_string(p.y) + "," + 
                       std::to_string(p.z) + ")";
            }
            case DataType::Point: {
                const auto& pt = std::get<Point3Df>(value_);
                return "Point3D(" + std::to_string(pt.x) + "," + std::to_string(pt.y) + "," +
                       std::to_string(pt.z) + ")";
            }
            case DataType::PointCloud: {
                const auto& cloud = std::get<PointCloudData>(value_);
                return "PointCloud[" + std::to_string(cloud.size()) + " points]";
            }
            case DataType::DepthImage: {
                const auto& di = std::get<DepthImageData>(value_);
                return "DepthImage[" + std::to_string(di.depth.width) + "x" + 
                       std::to_string(di.depth.height) + "]";
            }
            default: return "Unknown";
        }
    }
    
    // 复制
    Data clone() const {
        Data d;
        d.type_ = type_;
        d.value_ = value_;
        return d;
    }

private:
    DataType type_;
    std::variant<
        std::monostate,
        double,
        bool,
        String,
        ImageData,
        Region,
        Pose,
        Point3Df,
        PointCloudData,
        DepthImageData
    > value_;
};

/**
 * @brief 数据端口 - 节点的输入/输出端口
 */
struct DataPort {
    String id;              // 端口ID
    String name;            // 端口名称
    DataType data_type;     // 数据类型
    bool required;          // 是否必须
    Data default_value;     // 默认值
    String description;     // 描述
    
    DataPort(const String& id, const String& name, DataType type,
             bool required = false, const Data& default_val = Data{})
        : id(id), name(name), data_type(type), required(required),
          default_value(default_val) {}

    // ---- 链式构造器（只增不改：501 处既有的 4 参数调用点一行都不用动）----
    // 用法： DataPort("image", "输入图像", DataType::Image, true).doc("单通道或三通道")
    DataPort& doc(const String& text)       { description = text;   return *this; }
    DataPort& default_to(const Data& value) { default_value = value; return *this; }
    DataPort& mandatory()                   { required = true;      return *this; }
    DataPort& optional()                    { required = false;     return *this; }
};

/**
 * @brief 参数定义 - 节点参数配置
 */
struct ParamDef {
    String id;              // 参数ID
    String name;            // 参数名称
    DataType type;          // 参数类型
    Data default_value;     // 默认值
    Data min_value;         // 最小值（数值类型）
    Data max_value;         // 最大值（数值类型）
    Vector<String> options; // 选项列表（枚举类型）
    String description;     // 描述
    
    ParamDef(const String& id, const String& name, DataType type,
             const Data& default_val = Data{})
        : id(id), name(name), type(type), default_value(default_val) {}

    // ---- 链式构造器（只增不改）----
    // 用法： ParamDef("threshold", "阈值", DataType::Number, Data(128)).range(0, 255).doc("二值化阈值")
    //
    // range() 是重点：原先 min_value/max_value 没有任何途径能在构造时写进去，
    // 所以 501 个算子几乎全都没声明过参数范围 —— 这正是 sampling_interval=0
    // 能一路走到死循环的根因。新算子从第一天起就应该声明范围。
    ParamDef& doc(const String& text)       { description = text;    return *this; }
    ParamDef& default_to(const Data& value) { default_value = value; return *this; }
    ParamDef& range(double lo, double hi)   { min_value = Data(lo); max_value = Data(hi); return *this; }
    ParamDef& min_value_of(double lo)       { min_value = Data(lo);  return *this; }
    ParamDef& max_value_of(double hi)       { max_value = Data(hi);  return *this; }
    // 成员叫 options，函数不能再叫 options，故取名 choices
    ParamDef& choices(const Vector<String>& opts) { options = opts; return *this; }
};

/**
 * @brief 参数集合
 */
class ParamSet {
public:
    void set(const String& key, const Data& value) {
        params_[key] = value;
    }
    
    void set(const String& key, Data&& value) {
        params_[key] = std::move(value);
    }
    
    Data get(const String& key, const Data& default_val = Data{}) const {
        auto it = params_.find(key);
        return it != params_.end() ? it->second : default_val;
    }
    
    bool has(const String& key) const {
        return params_.find(key) != params_.end();
    }
    
    void remove(const String& key) {
        params_.erase(key);
    }
    
    void clear() {
        params_.clear();
    }
    
    size_t size() const {
        return params_.size();
    }
    
    // 类型便捷方法
    double get_number(const String& key, double default_val = 0.0) const {
        return get(key).as_number(default_val);
    }
    
    int32_t get_int(const String& key, int32_t default_val = 0) const {
        return get(key).as_int(default_val);
    }
    
    String get_string(const String& key, const String& default_val = "") const {
        return get(key).as_string(default_val);
    }
    
    bool get_bool(const String& key, bool default_val = false) const {
        return get(key).as_bool(default_val);
    }
    
    // 获取所有参数（用于遍历）
    const HashMap<String, Data>& get_all() const {
        return params_;
    }
    
    // 迭代器支持
    auto begin() const { return params_.begin(); }
    auto end() const { return params_.end(); }

private:
    HashMap<String, Data> params_;
};

} // namespace ovf