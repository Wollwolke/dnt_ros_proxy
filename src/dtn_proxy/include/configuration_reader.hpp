#pragma once

#include <exception>
#include <logger.hpp>
#include <string>
#include <vector>

#define TOML11_NO_ERROR_PREFIX
#include <toml.hpp>

namespace proxyConfig {

using RosConfig = struct RosConfig {
    using RosTopic = struct RosTopic {
        std::string name;
        std::string type;
        void from_toml(const toml::value& v);
    };
    std::vector<RosTopic> subTopics;
    std::vector<RosTopic> pubTopics;
};

using DtnConfig = struct DtnConfig {
    std::string remoteNodeId;
    uint32_t lifetime;
};

using Config = struct Config {
    DtnConfig dtn;
    RosConfig ros;
};

class ConfigurationReader {
private:
    static DtnConfig initDtnConfig(const toml::value& config, Logger& log);
    static RosConfig initRosConfig(const toml::value& config, Logger& log);

public:
    static Config readConfigFile(const std::string& filePath, const std::string& loggerName);
};

class ConfigException : public std::exception {
private:
    std::string message;

public:
    ConfigException(std::string msg = "Error while parsing configuration!") : message(msg) {}
    const char* what() { return message.c_str(); }
};

}  // namespace proxyConfig
