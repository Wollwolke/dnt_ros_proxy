#include "pipeline/pipeline.hpp"

#include <memory>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

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
