#pragma once

#include <map>
#include <string>
namespace dtnproxy::pipeline {

enum Module {
    RATE_LIMIT,
    LOGGING,  // !Only properly works with timestamped msgs!
    COMPRESS,
    DECOMPRESS,
    IMG_COMPRESS,
    IMG_DECOMPRESS,
    EXPIRE,
    ON_CHANGE,
    COMBINE,
    SPLIT,
    COMBINE_TF,
    SPLIT_TF,
};

const std::map<std::string, Module> moduleMapping{{"RateLimit", Module::RATE_LIMIT},
                                                  {"Logging", Module::LOGGING},
                                                  {"Compress", Module::COMPRESS},
                                                  {"Decompress", Module::DECOMPRESS},
                                                  {"ImageCompress", Module::IMG_COMPRESS},
                                                  {"ImageDecompress", Module::IMG_DECOMPRESS},
                                                  {"Expire", Module::EXPIRE},
                                                  {"OnChange", Module::ON_CHANGE},
                                                  {"Combine", Module::COMBINE},
                                                  {"Split", Module::SPLIT},
                                                  {"CombineTF", Module::COMBINE_TF},
                                                  {"SplitTF", Module::SPLIT_TF}};

}  // namespace dtnproxy::pipeline
