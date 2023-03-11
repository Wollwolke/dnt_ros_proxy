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
        try {
            const auto ros = toml::find(config, "ros");
            if (ros.contains("sub")) {
                rosConfig.subTopics = toml::find<std::vector<RosConfig::RosTopic>>(ros, "sub");
                foundTopics = true;
            }
            if (ros.contains("pub")) {
                rosConfig.pubTopics = toml::find<std::vector<RosConfig::RosTopic>>(ros, "pub");
                foundTopics = true;
            }
            if (ros.contains("servers")) {
                rosConfig.servers = toml::find<std::vector<RosConfig::RosService>>(ros, "servers");
                foundTopics = true;
            }
            if (ros.contains("clients")) {
                rosConfig.clients = toml::find<std::vector<RosConfig::RosService>>(ros, "clients");
                foundTopics = true;
            }
        } catch (const toml::exception& err) {
            log.ERR() << err.what();
            throw ConfigException();
        }
    }
    if (foundTopics) {
        // load profiles
        if (config.contains("profile")) {
            try {
                initProfilesConfig(config, rosConfig);
            } catch (const std::exception& err) {
                log.ERR() << err.what();
                throw ConfigException();
            }
        }
    } else {
        log.WARN() << "No topics/services to forward found in config!";
    }
    return rosConfig;
}

void ConfigurationReader::initProfilesConfig(const toml::value& config, RosConfig& rosConfig) {
    const auto profiles = toml::find<std::vector<toml::value>>(config, "profile");
    for (const auto& profile : profiles) {
        auto profileName = toml::find<std::string>(profile, "name");
        std::vector<RosConfig::Module> modules;
        const auto tomlModules = toml::find<std::vector<toml::value>>(profile, "module");
        for (const auto& module : tomlModules) {
            RosConfig::Module mod;
            mod.name = toml::find<std::string>(module, "name");
            mod.params = toml::find<std::vector<std::string>>(module, "params");
            modules.push_back(mod);
        }
        rosConfig.profiles.insert_or_assign(profileName, modules);
    }
}

void RosConfig::RosTopic::from_toml(const toml::value& v) {
    try {
        auto tmp = toml::get<std::vector<std::string>>(v);
        if (tmp.size() < 2) {
            // TODO: Fix exception msg
            throw ConfigException("Malformed Config in [ros]");
        }
        this->name = tmp[0];
        this->type = tmp[1];
        this->profile = (tmp.size() == 3) ? tmp[2] : "";
    } catch (std::exception& e) {
        throw ConfigException(e.what());
    }
}

}  // namespace dtnproxy::conf
