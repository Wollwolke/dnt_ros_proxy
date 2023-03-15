#pragma once

#include <map>
#include <string>
namespace dtnproxy::pipeline {

enum Module {
    RATE_LIMIT,
    LOGGING,  // !Only properly works with timestamped msgs!
};

const std::map<std::string, Module> moduleMapping{
    {"RateLimit", Module::RATE_LIMIT},
    {"Logging", Module::LOGGING},

};

}  // namespace dtnproxy::pipeline
