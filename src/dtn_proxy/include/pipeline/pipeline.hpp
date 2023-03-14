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

    std::vector<std::unique_ptr<IAction>> actions;

public:
    void initPipeline(const PipelineConfig& config, const std::string& profile);
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg);
};

}  // namespace dtnproxy::pipeline
