#include "pipeline/pipeline.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "pipeline/action_interface.hpp"
#include "pipeline/combine_topics.hpp"
#include "pipeline/expire_bundles.hpp"
#include "pipeline/image_compression.hpp"
#include "pipeline/image_decompression.hpp"
#include "pipeline/logging.hpp"
#include "pipeline/lzmh_decoding.hpp"
#include "pipeline/lzmh_encoding.hpp"
#include "pipeline/modules.hpp"
#include "pipeline/on_change.hpp"
#include "pipeline/pipeline_msg.hpp"
#include "pipeline/rate_limit.hpp"
#include "pipeline/split_topics.hpp"

namespace dtnproxy::pipeline {

Pipeline::Pipeline(Direction dir, std::string msgType, std::string topic)
    : msgType(std::move(msgType)), topic(std::move(topic)), direction(dir) {}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile) {
    if (profile.empty()) {
        return;
    }
    for (const auto& [module, params] : config.at(profile)) {
        std::unique_ptr<IAction> mod;
        switch (module) {
            case Module::RATE_LIMIT:
                // TODO: error handling for parameters
                mod = std::make_unique<RateLimitAction>(stoi(params.at(0)));
                break;
            case Module::LOGGING:
                mod = std::make_unique<LoggingAction>();
                break;
            case Module::COMPRESS:
                mod = std::make_unique<LzmhEncodingAction>(msgType);
                break;
            case Module::DECOMPRESS:
                mod = std::make_unique<LzmhDecodingAction>(msgType);
                break;
            case Module::IMG_COMPRESS:
                mod = std::make_unique<ImageCompressionAction>(msgType);
                break;
            case Module::IMG_DECOMPRESS:
                mod = std::make_unique<ImageDecompressionAction>(msgType);
                break;
            case Module::EXPIRE:
                mod = std::make_unique<ExpireBundlesAction>();
                break;
            case Module::ON_CHANGE:
                mod = std::make_unique<OnChangeAction>(msgType);
                break;
            case Module::COMBINE:
                mod = std::make_unique<CombineTopicsAction>(topic, params, msgStore);
                break;
            case Module::SPLIT:
                mod = std::make_unique<SplitTopicsAction>(topic, params, injectMsgCb);
                break;
        }
        // Check if module should run when msg enters / leaves proxy
        if ((mod->direction() & (this->direction | Direction::INOUT)) != 0) {
            actions.push_back(std::move(mod));
        }
    }

    std::sort(actions.begin(), actions.end(),
              [](auto& first, auto& second) { return first->order() < second->order(); });
}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile,
                            msgStorePtr_t msgStore) {
    this->msgStore = std::move(msgStore);
    initPipeline(config, profile);
}

void Pipeline::initPipeline(const PipelineConfig& config, const std::string& profile,
                            injectMsgCb_t injectMsgCb) {
    this->injectMsgCb = std::move(injectMsgCb);
    initPipeline(config, profile);
}

bool Pipeline::run(PipelineMessage& pMsg) {
    for (auto& action : actions) {
        if (!action->run(pMsg)) {
            return false;
        }
    }
    return true;
}

}  // namespace dtnproxy::pipeline
