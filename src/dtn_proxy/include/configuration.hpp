#pragma once

#include <exception>
#include <string>
#include <vector>

#include "logger.hpp"
#include "pipeline/modules.hpp"

#define TOML11_NO_ERROR_PREFIX
#include <toml.hpp>

namespace dtnproxy::conf {

using RosConfig = struct RosConfig {
    using RosTopic = struct RosTopic {
        std::string name;
        std::string type;
        std::string profile;
        void from_toml(const toml::value& v);
    };
    using RosService = RosTopic;
    using Module = struct Module {
        pipeline::Module name;
        std::vector<std::string> params;
    };

    std::vector<RosTopic> subTopics;
    std::vector<RosTopic> pubTopics;
    std::vector<RosService> clients;
    std::vector<RosService> servers;

    std::map<std::string, std::vector<Module>> profiles;
};

using DtnConfig = struct DtnConfig {
    std::string address;
    uint16_t port;
    std::string remoteNodeId;
    uint32_t lifetime;
};

using Config = struct Config {
    std::string statsDir;
    DtnConfig dtn;
    RosConfig ros;
};

class ConfigurationReader {
private:
    static DtnConfig initDtnConfig(const toml::value& config, Logger& log);
    static RosConfig initRosConfig(const toml::value& config, Logger& log);
    static void initProfilesConfig(const toml::value& config, RosConfig& rosConfig);
    static pipeline::Module resolveStringModule(const std::string& moduleName);
    static std::vector<std::string> collectRequiredProfiles(const RosConfig& rosConfig);

public:
    static Config readConfigFile(const std::string& filePath);
};

class ConfigException : public std::exception {
private:
    std::string message;

public:
    ConfigException(std::string msg = "Error while parsing configuration!") : message(msg) {}
    const char* what() { return message.c_str(); }
};

}  // namespace dtnproxy::conf
