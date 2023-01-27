#include <logger.hpp>

using namespace logger;

Debug::~Debug() { std::cout << std::endl; }

void Logger::setLogger(rclcpp::Logger l) { rosLogger = l; }
