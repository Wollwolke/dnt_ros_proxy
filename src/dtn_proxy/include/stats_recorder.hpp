#pragma once

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy {

class StatsRecorder {
private:
    using Entry = struct Entry {
        using InternalEntry = struct InternalEntry {
            uint32_t size{0};
            uint32_t count{0};
            NLOHMANN_DEFINE_TYPE_INTRUSIVE(InternalEntry, size, count)
        };

        std::string msgType;
        InternalEntry rosEntry;
        InternalEntry dtnEntry;

        Entry(const std::string& msgType) : msgType(msgType) {}

        // ! Don't call this ctor
        // Need it for map initialisation?
        Entry() = default;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Entry, msgType, rosEntry, dtnEntry)
    };

    using serviceStats_t = std::map<std::string, std::map<std::string, Entry>>;
    static constexpr auto REQUEST = "request";
    static constexpr auto RESPONSE = "response";

    std::map<std::string, Entry> subStats;
    std::map<std::string, Entry> pubStats;
    serviceStats_t serviceStats;
    serviceStats_t clientStats;
    std::filesystem::path statsDir;

    void incrementEntry(Entry::InternalEntry& entry, uint32_t size);
    auto initServiceStats(serviceStats_t& stats, const std::string& topic,
                          const std::string& msgType);

public:
    StatsRecorder(const std::string& statsDir);
    ~StatsRecorder();

    void rosReceived(const std::string& topic, const std::string& msgType, uint32_t size,
                     ros::DtnMsgType rosType);
    void rosSent(const std::string& topic, const std::string& msgType, uint32_t size,
                 ros::DtnMsgType rosType);
    void dtnReceived(const std::string& endpoint, const std::string& msgType, uint32_t size,
                     ros::DtnMsgType rosType);
    void dtnSent(const std::string& endpoint, const std::string& msgType, uint32_t size,
                 ros::DtnMsgType rosType);
    void saveToDisk();

    friend void to_json(nlohmann::json& j, const StatsRecorder& obj);
};

}  // namespace dtnproxy

/*
Server:
- kriegt request über Ros
- verschickt request über Dtn
- kriegt response über Dtn
- verschickt response über Ros

Client:
- kriegt request über Dtn
- verschickt request über Ros
- kriegt response über Ros
- verschickt response über Dtn
*/
