#include "pipeline/image_processing.hpp"

#include <lodepng.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace dtnproxy::pipeline {

ImageProcessingAction::ImageProcessingAction(const std::string& msgType) {
    active = (supportedMsgType == msgType);
}

Direction ImageProcessingAction::direction() { return dir; }

uint ImageProcessingAction::order(const Direction& pipelineDir) {
    switch (pipelineDir) {
        case Direction::IN:
            return SEQUENCE_NR_IN;
            break;
        case Direction::OUT:
            return SEQUENCE_NR_OUT;
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
}

bool ImageProcessingAction::run(PipelineMessage& pMsg, const Direction& pipelineDir) {
    if (!active) {
        return true;
    }

    switch (pipelineDir) {
        case Direction::IN:
            return compress(pMsg);
            break;
        case Direction::OUT:
            return decompress(pMsg);
            break;
        case Direction::INOUT:
        default:
            throw new std::runtime_error("Pipeline should not have Direction INOUT");
    }
}

bool ImageProcessingAction::compress(PipelineMessage& pMsg) {
    using MessageT = sensor_msgs::msg::Image;
    MessageT imageMsg;
    auto serializer = rclcpp::Serialization<MessageT>();
    serializer.deserialize_message(pMsg.serializedMessage.get(), &imageMsg);

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

    pMsg.serializedMessage->reserve(png.size());
    auto* newCdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    newCdrMsg->buffer_length = png.size();
    std::move(png.begin(), png.end(), newCdrMsg->buffer);

    return true;
}

bool ImageProcessingAction::decompress(PipelineMessage& pMsg) {
    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

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
        std::cout << "ImageDecompression: Decoder error " << error << ": "
                  << lodepng_error_text(error) << std::endl;
        return false;
    }

    auto strings = state.info_png.text_strings;
    auto keys = state.info_png.text_keys;
    auto numOfTexts = state.info_png.text_num;
    auto step = image.size() / height;

    // build ROS msg
    sensor_msgs::msg::Image imageMsg;

    // Fill header
    for (size_t i = 0; i < numOfTexts; ++i) {
        if (std::strncmp("f", keys[i], 1) == 0) {
            imageMsg.header.frame_id = strings[i];
        } else if (std::strncmp("s", keys[i], 1) == 0) {
            imageMsg.header.stamp.sec = std::stoi(strings[i]);
        } else if (std::strncmp("n", keys[i], 1) == 0) {
            imageMsg.header.stamp.nanosec = std::stoul(strings[i]);
        }
    }

    imageMsg.height = height;
    imageMsg.width = width;
    imageMsg.encoding = ENCODING;
    imageMsg.is_bigendian = 0;
    imageMsg.step = step;
    imageMsg.data = image;

    auto serializedMsg = std::make_shared<rclcpp::SerializedMessage>();
    static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.serialize_message(&imageMsg, serializedMsg.get());

    pMsg.serializedMessage.swap(serializedMsg);
    return true;
}

}  // namespace dtnproxy::pipeline
