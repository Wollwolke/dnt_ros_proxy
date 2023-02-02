#pragma once
#include <nlohmann/json.hpp>
#include <vector>

namespace data {

struct CreationTimestamp {
    uint64_t time;
    uint64_t sequenceNumber;
};

struct Ws2Dtn {
    std::string src;
    std::string dst;
    bool delivery_notification;
    uint64_t lifetime;
    std::vector<uint8_t> data;
};

struct Dtn2Ws {
    std::string bid;
    std::string src;
    std::string dst;
    CreationTimestamp cts;
    uint64_t lifetime;
    std::vector<uint8_t> data;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Ws2Dtn, src, dst, delivery_notification, lifetime, data)

inline void from_json(const nlohmann::json& j, CreationTimestamp& obj) {
    auto tmp = j.get<std::array<uint64_t, 2>>();
    obj.time = tmp[0];
    obj.sequenceNumber = tmp[1];
}

inline void from_json(const nlohmann::json& j, Dtn2Ws& obj) {
    j.at("bid").get_to(obj.bid);
    j.at("src").get_to(obj.src);
    j.at("dst").get_to(obj.dst);
    j.at("cts").get_to(obj.cts);
    j.at("lifetime").get_to(obj.lifetime);
    obj.data = j.at("data").get_binary();
}

}  // namespace data
