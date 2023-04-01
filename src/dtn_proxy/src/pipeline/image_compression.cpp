#include "pipeline/image_compression.hpp"

#include <lodepng.h>

#include <cstdint>
#include <iostream>
#include <string>

#include "ros/ros_message.hpp"

namespace dtnproxy::pipeline {

void ImageCompressionAction::getByteVector(const ros2_babel_fish::ArrayMessageBase &message,
                                           std::vector<uint8_t> &result) {
    for (size_t i = 0; i < message.size(); ++i) {
        result.push_back(message.as<ros2_babel_fish::ArrayMessage<uint8_t>>()[i]);
    }
}

ImageCompressionAction::ImageCompressionAction(const std::string &msgType) {
    active = (supportedMsgType == msgType);
}

Direction ImageCompressionAction::direction() { return dir; }

uint ImageCompressionAction::order() { return SEQUENCE_NR; }

bool ImageCompressionAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    if (!active) {
        return true;
    }

    ros::RosMessage rosMsg(msg, supportedMsgType);
    auto fishMsg = ros2_babel_fish::CompoundMessage(*rosMsg.msgTypeSupport, rosMsg.data);

    std::vector<uint8_t> data;
    getByteVector(fishMsg["data"].as<ros2_babel_fish::ArrayMessageBase>(), data);
    const auto encoding = fishMsg["encoding"].value<std::string>();
    const auto width = fishMsg["width"].value<uint32_t>();
    const auto height = fishMsg["height"].value<uint32_t>();
    const auto frame_id = fishMsg["header"]["frame_id"].value<std::string>();
    const auto stampSec = fishMsg["header"]["stamp"]["sec"].value<int32_t>();
    const auto stampNanosec = fishMsg["header"]["stamp"]["nanosec"].value<uint32_t>();

    // endiness is beeing ignored right now, as only RGB8 encoded images are supported
    // const auto isBigendian = fishMsg["is_bigendian"].value<uint8_t>();

    if ("rgb8" != encoding) {
        std::cout << "ImageCompression: Image not compressed, unsuported encoding " << encoding
                  << std::endl;
        return false;
    }

    std::vector<uint8_t> png;
    lodepng::State state;
    state.info_raw.colortype = LCT_RGB;
    state.info_raw.bitdepth = BIT_DEPTH;

    // add ros header as png text chunks
    lodepng_add_text(&state.info_png, "f", frame_id.c_str());
    lodepng_add_text(&state.info_png, "s", std::to_string(stampSec).c_str());
    lodepng_add_text(&state.info_png, "n", std::to_string(stampNanosec).c_str());

    // optimize for a tradeoff of compression & speed
    state.encoder.filter_palette_zero = 0;
    state.encoder.add_id = false;
    state.encoder.zlibsettings.nicematch = 258;  // stop searching if >= this length found. Set to
                                                 // 258 for best compression. Default: 128

    auto error = lodepng::encode(png, data, width, height, state);

    if (error != 0) {
        std::cout << "ImageCompression: Image not compressed, encoder error " << error << ": "
                  << lodepng_error_text(error) << std::endl;
        return false;
    }

    msg->reserve(png.size());
    std::move(png.begin(), png.end(), msg->get_rcl_serialized_message().buffer);

    return true;
}

}  // namespace dtnproxy::pipeline
