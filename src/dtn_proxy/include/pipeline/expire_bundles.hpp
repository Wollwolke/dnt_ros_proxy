#pragma once

#include <chrono>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class ExpireBundlesAction : public IAction {
private:
    const Direction dir = Direction::IN;
    const uint SEQUENCE_NR = 90;

public:
    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
