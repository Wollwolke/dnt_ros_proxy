#pragma once

#include <map>
#include <string>
namespace dtnproxy::pipeline {

enum Module {
    RATE_LIMIT,
};

const std::map<std::string, Module> moduleMapping{
    {"RateLimit", Module::RATE_LIMIT},
    // {"option2", Option2},
};

}  // namespace dtnproxy::pipeline
