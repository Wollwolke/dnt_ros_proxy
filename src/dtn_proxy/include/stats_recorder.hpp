#pragma once

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

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

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Entry, msgType, rosEntry, dtnEntry)
    };

    std::map<std::string, Entry> subStats;
    std::map<std::string, Entry> pubStats;
    std::filesystem::path statsDir;

    void incrementEntry(Entry::InternalEntry& entry, uint32_t size);

public:
    StatsRecorder(const std::string& statsDir);
    ~StatsRecorder();

    void rosReceived(const std::string& topic, const std::string& msgType, uint32_t size);
    void rosSent(const std::string& topic, const std::string& msgType, uint32_t size);
    void dtnReceived(const std::string& endpoint, const std::string& msgType, uint32_t size);
    void dtnSent(const std::string& endpoint, const std::string& msgType, uint32_t size);
    void saveToDisk();

    friend void to_json(nlohmann::json& j, const StatsRecorder& obj);
};
