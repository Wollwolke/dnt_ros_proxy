#pragma once
#include <functional>
#include <iostream>

#ifdef ROS_LOGGER
#include "rclcpp/rclcpp.hpp"
#endif

namespace logger {

class Logger {
#ifdef ROS_LOGGER
protected:
    static rclcpp::Logger rosLogger;
    static const bool useRos = true;

public:
    static void setLogger(rclcpp::Logger l);
#elif
protected:
    static const bool useRos = false;
#endif
};

class Debug : Logger {
private:
public:
    ~Debug();

    template <class T>
    Debug& operator<<(const T& v) {
        RCLCPP_INFO_STREAM(rosLogger, "" << v);
        std::cout << v;
        return *this;
    }
};

}  // namespace logger
