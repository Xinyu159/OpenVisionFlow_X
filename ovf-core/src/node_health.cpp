/**
 * @file node_health.cpp
 * @brief 算子体检结论表的实现
 */

#include "ovf/core/node_health.h"

#include "nlohmann/json.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace ovf {

int node_health_score_for_tier(const String& tier) {
    if (tier == "A") return 100;
    if (tier == "B") return 80;
    if (tier == "C") return 50;
    if (tier == "D") return 10;
    return 0;   // 没体检过
}

NodeHealthRegistry& NodeHealthRegistry::instance() {
    static NodeHealthRegistry registry;
    return registry;
}

bool NodeHealthRegistry::load_json(const String& json_text) {
    HashMap<String, NodeHealth> parsed;
    try {
        nlohmann::json doc = nlohmann::json::parse(json_text);
        if (!doc.is_object()) return false;

        const nlohmann::json& nodes = doc["nodes"];
        if (!nodes.is_object()) return false;

        // 遍历：本仓库自带的 json 用 std::map 存对象，顺序天然是字典序。
        // 注意：它的 begin()/end() 是给**数组**用的，对象要用 object_begin()。
        for (auto it = nodes.object_begin(); it != nodes.object_end(); ++it) {
            const String& type_id = it->first;
            const nlohmann::json& entry = it->second;
            if (!entry.is_object()) continue;

            NodeHealth h;
            h.tier  = entry.value("tier", String(""));
            h.note  = entry.value("note", String(""));
            h.score = entry.value("score", 0);
            if (h.tier.empty()) continue;   // 没档位 = 没结论，不放进表里

            // score 缺省或明显不对时按档位补上，别让报告里的数字自相矛盾
            if (h.score <= 0) h.score = node_health_score_for_tier(h.tier);

            parsed[type_id] = h;
        }
    } catch (const std::exception&) {
        // 解析失败 = 没体检过。fail-open，不抛也不返回半截数据。
        return false;
    } catch (...) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        table_.swap(parsed);
    }
    return true;
}

bool NodeHealthRegistry::load_file(const String& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return false;

    std::ostringstream buf;
    buf << in.rdbuf();
    const String text = buf.str();
    if (text.empty()) return false;

    if (!load_json(text)) return false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        source_ = path;
    }
    return true;
}

bool NodeHealthRegistry::load_default() {
    Vector<String> candidates;

    // 1. 环境变量最优先 —— 体检脚本和 CTest 用它指向刚生成的那份
    if (const char* env = std::getenv("OVF_NODE_HEALTH")) {
        if (*env) candidates.push_back(env);
    }

    // 2. 常见相对路径。二进制在 build/bin、报告在仓库根，cwd 不定。
    candidates.push_back("health/node_health.json");
    candidates.push_back("../health/node_health.json");
    candidates.push_back("../../health/node_health.json");

    // 3. 编译期记下的仓库根 —— 从任何地方启动都能找到（构建目录被删就失效）
#ifdef OVF_SOURCE_DIR
    candidates.push_back(String(OVF_SOURCE_DIR) + "/health/node_health.json");
#endif

    for (const auto& path : candidates) {
        if (load_file(path)) return true;
    }

    // 一个都没有：空表。**这不是错误** —— 没体检过的平台照样跑。
    std::lock_guard<std::mutex> lock(mutex_);
    table_.clear();
    source_.clear();
    return false;
}

NodeHealth NodeHealthRegistry::query(const String& type_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = table_.find(type_id);
    return it != table_.end() ? it->second : NodeHealth{};
}

size_t NodeHealthRegistry::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return table_.size();
}

bool NodeHealthRegistry::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return table_.empty();
}

Vector<String> NodeHealthRegistry::type_ids() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Vector<String> ids;
    ids.reserve(table_.size());
    for (const auto& kv : table_) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());
    return ids;
}

String NodeHealthRegistry::source() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return source_;
}

void NodeHealthRegistry::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    table_.clear();
    source_.clear();
}

} // namespace ovf
