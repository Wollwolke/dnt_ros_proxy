#pragma once

#include <map>
#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <vector>

#include "configuration.hpp"
#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class Pipeline {
private:
    using PipelineConfig = std::map<std::string, std::vector<conf::RosConfig::Module>>;

    enum Module {
        INVALID,
        RATE_LIMIT,
    };

    std::vector<std::unique_ptr<IAction>> actions;

    Module resolveStringModule(const std::string& moduleName);

public:
    void initPipeline(const PipelineConfig& config, const std::string& profile);
    void appendActions(std::unique_ptr<IAction> actionPtr);
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg);
};

}  // namespace dtnproxy::pipeline
