#include "pipeline/pipeline.hpp"

#include <memory>

#include "pipeline/action_interface.hpp"
#include "pipeline/modules.hpp"
#include "pipeline/rate_limit.hpp"

namespace dtnproxy::pipeline {

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile) {
    if (profile.empty()) {
        return;
    }
    for (const auto& [module, params] : config.at(profile)) {
        switch (module) {
            case Module::RATE_LIMIT:
                // TODO: error handling for parameters
                actions.push_back(std::make_unique<RateLimitAction>(stoi(params.at(0))));
                break;
        }
    }
}

bool Pipeline::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    // TODO: sort actions before running

    for (auto& action : actions) {
        if (!action->run(msg)) {
            return false;
        }
    }
    return true;
}

}  // namespace dtnproxy::pipeline
