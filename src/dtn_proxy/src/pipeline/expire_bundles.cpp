#include "pipeline/expire_bundles.hpp"

namespace dtnproxy::pipeline {

Direction ExpireBundlesAction::direction() { return dir; }

uint ExpireBundlesAction::order() { return SEQUENCE_NR; }

bool ExpireBundlesAction::run(PipelineMessage& pMsg) {
    pMsg.markExpired = true;
    return true;
}

}  // namespace dtnproxy::pipeline
