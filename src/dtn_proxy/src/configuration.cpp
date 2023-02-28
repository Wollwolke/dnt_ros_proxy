#include "configuration.hpp"

#include <iostream>

namespace dtnproxy::conf {

Config ConfigurationReader::readConfigFile(const std::string& filePath) {
    auto log = Logger("ConfigReader");

    toml::value config;
    try {
        config = toml::parse(filePath);
        log.DBG() << config;
    } catch (const std::exception& err) {
        log.FATAL() << err.what();
        throw ConfigException();
    }

    const DtnConfig dtn = initDtnConfig(config, log);
    const RosConfig ros = initRosConfig(config, log);
    const Config cfg{
        toml::find_or<std::string>(config, "statsDir",
                                   "/tmp/dtnproxy/stats/"),  // std::string statsPath;
        dtn,                                                 // DtnConfig dtn,
        ros,                                                 // RosConfig ros,
    };

    return cfg;
}

DtnConfig ConfigurationReader::initDtnConfig(const toml::value& config, Logger& log) {
    DtnConfig dtnConfig;
    if (config.contains("dtn")) {
        auto dtn = toml::find(config, "dtn");
        try {
            if (dtn.contains("remoteNodeId")) {
                dtnConfig.remoteNodeId = toml::find<std::string>(dtn, "remoteNodeId");
            } else {
                log.FATAL() << "Missing mandatory parameter: dtn.remoteNodeId !";
                throw ConfigException();
            }
            dtnConfig.lifetime = toml::find_or<uint32_t>(dtn, "lifetime", 5);
            dtnConfig.address = toml::find_or<std::string>(dtn, "dtndAddress", "127.0.0.1");
            dtnConfig.port = toml::find_or<uint16_t>(dtn, "dtndPort", 3000);
        } catch (const toml::exception& err) {
            log.ERR() << err.what();
            throw ConfigException();
        }
    }
    return dtnConfig;
}

RosConfig ConfigurationReader::initRosConfig(const toml::value& config, Logger& log) {
    RosConfig rosConfig;
    bool foundTopics = false;
    if (config.contains("ros")) {
        auto ros = toml::find(config, "ros");
        try {
            if (ros.contains("sub")) {
                rosConfig.subTopics = toml::find<std::vector<RosConfig::RosTopic>>(ros, "sub");
                foundTopics = true;
            }
            if (ros.contains("pub")) {
                rosConfig.pubTopics = toml::find<std::vector<RosConfig::RosTopic>>(ros, "pub");
                foundTopics = true;
            }
            if (ros.contains("servers")) {
                rosConfig.servers = toml::find<std::vector<RosConfig::RosTopic>>(ros, "servers");
                foundTopics = true;
            }
            if (ros.contains("clients")) {
                rosConfig.clients = toml::find<std::vector<RosConfig::RosTopic>>(ros, "clients");
                foundTopics = true;
            }
        } catch (const toml::exception& err) {
            log.ERR() << err.what();
            throw ConfigException();
        }
    }
    if (!foundTopics) log.WARN() << "No topics/services to forward found in config!";
    return rosConfig;
}

void RosConfig::RosTopic::from_toml(const toml::value& v) {
    try {
        auto tmp = toml::get<std::array<std::string, 2>>(v);
        this->name = tmp[0];
        this->type = tmp[1];
    } catch (std::exception& e) {
        throw ConfigException(e.what());
    }
}
}  // namespace dtnproxy::conf
