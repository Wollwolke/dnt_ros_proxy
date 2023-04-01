#include "pipeline/image_decompression.hpp"

#include <lodepng.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <ros2_babel_fish/babel_fish.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "ros/ros_message.hpp"

namespace dtnproxy::pipeline {

ImageDecompressionAction::ImageDecompressionAction(const std::string& msgType) {
    active = (supportedMsgType == msgType);
}

Direction ImageDecompressionAction::direction() { return dir; }

uint ImageDecompressionAction::order() { return SEQUENCE_NR; }

bool ImageDecompressionAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    if (!active) {
        return true;
    }

    auto cdrMsg = msg->get_rcl_serialized_message();

    // setup decoder
    lodepng::State state;
    state.decoder.read_text_chunks = 1;
    state.decoder.remember_unknown_chunks = 0;
    state.info_raw.colortype = LCT_RGB;
    state.info_raw.bitdepth = BIT_DEPTH;

    // decoding
    std::vector<uint8_t> image;
    unsigned width;
    unsigned height;

    auto error = lodepng::decode(image, width, height, state, cdrMsg.buffer, cdrMsg.buffer_length);

    if (error != 0) {
        std::cout << "ImageDecompression: Encoder error " << error << ": "
                  << lodepng_error_text(error) << std::endl;
        return false;
    }

    auto strings = state.info_png.text_strings;
    auto keys = state.info_png.text_keys;
    auto numOfTexts = state.info_png.text_num;
    auto step = image.size() / height;

    // build ROS msg
    auto fish = ros2_babel_fish::BabelFish();
    auto fishMsg = fish.create_message(supportedMsgType);
    auto& compound = fishMsg;

    auto imageMsg = std::make_shared<sensor_msgs::msg::Image>();

    // Fill header
    for (size_t i = 0; i < numOfTexts; ++i) {
        if (std::strncmp("f", keys[i], 1) == 0) {
            imageMsg->header.frame_id = strings[i];
        } else if (std::strncmp("s", keys[i], 1) == 0) {
            imageMsg->header.stamp.sec = std::stoi(strings[i]);
        } else if (std::strncmp("n", keys[i], 1) == 0) {
            imageMsg->header.stamp.nanosec = std::stoul(strings[i]);
        }
    }

    compound["height"] = height;
    compound["width"] = width;
    compound["encoding"] = ENCODING;
    compound["is_bigendian"] = 0;
    compound["step"] = step;
    auto& test = compound["data"].as<ros2_babel_fish::ArrayMessageBase>();

    for (auto& b : image) {
        test.as<ros2_babel_fish::ArrayMessage<uint8_t>>().append(b);
    }

    auto test2 = fishMsg.type_erased_message();

    imageMsg->height = height;
    imageMsg->width = width;
    imageMsg->encoding = ENCODING;
    imageMsg->is_bigendian = 0;
    imageMsg->step = step;
    imageMsg->data = image;

    ros::RosMessage rosMsg(test2, supportedMsgType);
    // static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    // serializer.serialize_message(test2.get(), &serializedMsg);

    *msg = rosMsg.serializedMsg.get_rcl_serialized_message();
    // serializedMsg.get_rcl_serialized_message();

    return true;
}

}  // namespace dtnproxy::pipeline
