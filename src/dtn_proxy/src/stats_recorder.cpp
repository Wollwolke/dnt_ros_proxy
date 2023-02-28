#include "stats_recorder.hpp"

#include <ctime>
#include <fstream>
#include <iostream>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy {

auto StatsRecorder::initServiceStats(serviceStats_t& stats, const std::string& topic,
                                     const std::string& msgType) {
    std::map<std::string, Entry> tmp = {{REQUEST, Entry(msgType)}, {RESPONSE, Entry(msgType)}};
    return stats.emplace(topic, tmp).first;
}

void StatsRecorder::incrementEntry(Entry::InternalEntry& entry, uint32_t size) {
    entry.count += 1;
    if (0 == entry.size) {
        entry.size = size;
    } else if (size != entry.size) {
        std::cout << "Stats: Entry size changed!!!" << std::endl;
    }
}

StatsRecorder::StatsRecorder(const std::string& statsDir) { this->statsDir.append(statsDir); }

StatsRecorder::~StatsRecorder() { saveToDisk(); }

// TODO: refactor this...

void StatsRecorder::rosReceived(const std::string& topic, const std::string& msgType, uint32_t size,
                                ros::DtnMsgType rosType) {
    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            auto result = subStats.emplace(topic, msgType);
            incrementEntry(result.first->second.rosEntry, size);
        } break;
        case ros::DtnMsgType::REQUEST: {
            auto result = initServiceStats(serviceStats, topic, msgType);
            incrementEntry(result->second[REQUEST].rosEntry, size);
        } break;
        case ros::DtnMsgType::RESPONSE: {
            auto result = initServiceStats(clientStats, topic, msgType);
            incrementEntry(result->second[RESPONSE].rosEntry, size);
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
}

void StatsRecorder::rosSent(const std::string& topic, const std::string& msgType, uint32_t size,
                            ros::DtnMsgType rosType) {
    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            auto result = pubStats.emplace(topic, msgType);
            incrementEntry(result.first->second.rosEntry, size);
        } break;
        case ros::DtnMsgType::REQUEST: {
            auto result = initServiceStats(clientStats, topic, msgType);
            incrementEntry(result->second[REQUEST].rosEntry, size);
        } break;
        case ros::DtnMsgType::RESPONSE: {
            auto result = initServiceStats(serviceStats, topic, msgType);
            incrementEntry(result->second[RESPONSE].rosEntry, size);
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
}

void StatsRecorder::dtnReceived(const std::string& endpoint, const std::string& msgType,
                                uint32_t size, ros::DtnMsgType rosType) {
    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            auto result = pubStats.emplace(endpoint, msgType);
            incrementEntry(result.first->second.dtnEntry, size);
        } break;
        case ros::DtnMsgType::REQUEST: {
            auto result = initServiceStats(clientStats, endpoint, msgType);
            incrementEntry(result->second[REQUEST].dtnEntry, size);
        } break;
        case ros::DtnMsgType::RESPONSE: {
            auto result = initServiceStats(serviceStats, endpoint, msgType);
            incrementEntry(result->second[RESPONSE].dtnEntry, size);
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
}

void StatsRecorder::dtnSent(const std::string& endpoint, const std::string& msgType, uint32_t size,
                            ros::DtnMsgType rosType) {
    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            auto result = subStats.emplace(endpoint, msgType);
            incrementEntry(result.first->second.dtnEntry, size);
        } break;
        case ros::DtnMsgType::REQUEST: {
            auto result = initServiceStats(serviceStats, endpoint, msgType);
            incrementEntry(result->second[REQUEST].dtnEntry, size);
        } break;
        case ros::DtnMsgType::RESPONSE: {
            auto result = initServiceStats(clientStats, endpoint, msgType);
            incrementEntry(result->second[RESPONSE].dtnEntry, size);
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
}

void StatsRecorder::saveToDisk() {
    nlohmann::json jStats = *this;

    auto time = std::time(nullptr);

    const auto lenOfDateString = (4 + 1 + 2 + 1 + 2) + 1 + (2 + 1 + 2 + 1 + 2) + 1;
    char fileName[lenOfDateString];
    std::strftime(fileName, lenOfDateString, "%Y-%m-%d_%H-%M-%S", std::localtime(&time));

    auto path = statsDir;
    path.append(fileName).replace_extension(".json");

    // TODO: handle file errors
    std::filesystem::create_directories(statsDir);
    std::ofstream ofs(path.string());
    ofs << std::setw(4) << jStats;
}

void to_json(nlohmann::json& j, const StatsRecorder& obj) {
    j = nlohmann::json{{"topics", {{"subscriber", obj.subStats}, {"publisher", obj.pubStats}}},
                       {"services", {{"client", obj.clientStats}, {"server", obj.serviceStats}}}};
}

}  // namespace dtnproxy
