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
};

const std::map<std::string, Module> moduleMapping{
    {"RateLimit", Module::RATE_LIMIT},       {"Logging", Module::LOGGING},
    {"Compress", Module::COMPRESS},          {"Decompress", Module::DECOMPRESS},
    {"ImageCompress", Module::IMG_COMPRESS}, {"ImageDecompress", Module::IMG_DECOMPRESS},
};

}  // namespace dtnproxy::pipeline
