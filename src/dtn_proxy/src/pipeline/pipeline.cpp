#include "pipeline/pipeline.hpp"

#include <memory>

#include "pipeline/action_interface.hpp"
#include "pipeline/rate_limit.hpp"

namespace dtnproxy::pipeline {

Pipeline::Module Pipeline::resolveStringModule(const std::string& moduleName) {
    static const std::map<std::string, Module> moduleMapping{
        {"RateLimit", Module::RATE_LIMIT},
        // {"option2", Option2},
    };

    auto itr = moduleMapping.find(moduleName);
    if (itr != moduleMapping.end()) {
        return itr->second;
    }
    return Module::INVALID;
}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile) {
    // TODO: implement check [in configReader] that profile exists
    for (const auto& [moduleName, params] : config.at(profile)) {
        auto module = resolveStringModule(moduleName);
        switch (module) {
            case Module::RATE_LIMIT:
                // TODO: error handling for parameters
                appendActions(std::make_unique<RateLimitAction>(stoi(params.at(0))));
                break;
            case Module::INVALID:
                // TODO: do somehting
                break;
        }
    }
}

void Pipeline::appendActions(std::unique_ptr<IAction> actionPtr) {
    actions.push_back(std::move(actionPtr));
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
