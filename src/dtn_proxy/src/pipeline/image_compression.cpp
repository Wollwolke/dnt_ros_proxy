#include "pipeline/image_compression.hpp"

#include <lodepng.h>

#include <cstdint>
#include <iostream>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace dtnproxy::pipeline {

ImageCompressionAction::ImageCompressionAction(const std::string &msgType) {
    active = (supportedMsgType == msgType);
}

Direction ImageCompressionAction::direction() { return dir; }

uint ImageCompressionAction::order() { return SEQUENCE_NR; }

bool ImageCompressionAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    if (!active) {
        return true;
    }

    using MessageT = sensor_msgs::msg::Image;
    MessageT imageMsg;
    auto serializer = rclcpp::Serialization<MessageT>();
    serializer.deserialize_message(msg.get(), &imageMsg);

    if ("rgb8" != imageMsg.encoding) {
        std::cout << "ImageCompression: Image not compressed, unsuported encoding "
                  << imageMsg.encoding << std::endl;
        return false;
    }

    std::vector<uint8_t> png;
    lodepng::State state;
    state.info_raw.colortype = LCT_RGB;
    state.info_raw.bitdepth = BIT_DEPTH;

    // add ros header as png text chunks
    lodepng_add_text(&state.info_png, "f", imageMsg.header.frame_id.c_str());
    lodepng_add_text(&state.info_png, "s", std::to_string(imageMsg.header.stamp.sec).c_str());
    lodepng_add_text(&state.info_png, "n", std::to_string(imageMsg.header.stamp.nanosec).c_str());

    // optimize for a tradeoff of compression & speed
    state.encoder.filter_palette_zero = 0;
    state.encoder.add_id = 0;
    state.encoder.zlibsettings.nicematch = 258;  // stop searching if >= this length found. Set to
                                                 // 258 for best compression. Default: 128

    auto error = lodepng::encode(png, imageMsg.data, imageMsg.width, imageMsg.height, state);

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
