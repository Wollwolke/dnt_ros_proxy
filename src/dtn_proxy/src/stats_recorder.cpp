#include "stats_recorder.hpp"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy {

std::string StatsRecorder::timestamp() {
    using Clock = std::chrono::system_clock;

    auto now = Clock::now();
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto fraction = now - seconds;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(fraction);

    time_t cnow = Clock::to_time_t(now);

    std::stringstream timeStream;
    timeStream << std::put_time(std::localtime(&cnow), "%FT%T.");
    timeStream << milliseconds.count();
    timeStream << std::put_time(std::localtime(&cnow), "%z") << ";";

    return timeStream.str();
}

StatsRecorder::StatsRecorder(const std::string& statsDir) {
    std::filesystem::path statsPath(statsDir);

    auto time = std::time(nullptr);

    const auto lenOfDateString = (4 + 1 + 2 + 1 + 2) + 1 + (2 + 1 + 2 + 1 + 2) + 1;
    char fileName[lenOfDateString];
    std::strftime(fileName, lenOfDateString, "%Y-%m-%d_%H-%M-%S", std::localtime(&time));

    auto filePath = statsPath;
    filePath.append(fileName).replace_extension(".log");

    // TODO: handle file errors
    std::filesystem::create_directories(statsPath);
    file.open(filePath);
}

StatsRecorder::~StatsRecorder() { file.close(); }

// TODO: refactor this...

void StatsRecorder::rosReceived(const std::string& topic, const std::string& msgType, uint32_t size,
                                ros::DtnMsgType rosType) {
    std::stringstream logMsg;
    logMsg << timestamp() << "ROS;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "SUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "SERVER;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "CLIENT;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << topic << ";" << msgType << ";" << size;
    file << logMsg.rdbuf() << std::endl;
}

void StatsRecorder::rosSent(const std::string& topic, const std::string& msgType, uint32_t size,
                            ros::DtnMsgType rosType) {
    std::stringstream logMsg;
    logMsg << timestamp() << "ROS;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "PUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "CLIENT;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "SERVER;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << topic << ";" << msgType << ";" << size;
    file << logMsg.rdbuf() << std::endl;
}

void StatsRecorder::dtnReceived(const std::string& endpoint, const std::string& msgType,
                                uint32_t size, ros::DtnMsgType rosType) {
    std::stringstream logMsg;
    logMsg << timestamp() << "DTN;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "PUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "CLIENT;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "SERVER;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << endpoint << ";" << msgType << ";" << size;
    file << logMsg.rdbuf() << std::endl;
}

void StatsRecorder::dtnSent(const std::string& endpoint, const std::string& msgType, uint32_t size,
                            ros::DtnMsgType rosType) {
    std::stringstream logMsg;
    logMsg << timestamp() << "DTN;";

    switch (rosType) {
        case ros::DtnMsgType::TOPIC: {
            logMsg << "TOPIC;"
                   << "SUB;";
        } break;
        case ros::DtnMsgType::REQUEST: {
            logMsg << "SERVER;"
                   << "REQUEST;";
        } break;
        case ros::DtnMsgType::RESPONSE: {
            logMsg << "CLIENT;"
                   << "RESPONSE;";
        } break;
        case ros::DtnMsgType::INVALID:
        default:
            return;
    }
    logMsg << endpoint << ";" << msgType << ";" << size;
    file << logMsg.rdbuf() << std::endl;
}

}  // namespace dtnproxy
