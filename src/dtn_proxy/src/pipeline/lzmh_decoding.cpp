#include "pipeline/lzmh_decoding.hpp"

#include <cstdint>
#include <iostream>
#include <vector>

extern "C" {
#include "bit_file_buffer.h"
#include "file_buffer.h"
#include "lzmh.h"
}

namespace dtnproxy::pipeline {

LzmhDecodingAction::LzmhDecodingAction(const std::string &msgType) {
    fbIn = AllocateFileBuffer();
    fbOut = AllocateFileBuffer();
    options.error_log_file = stderr;
    active = !(unSupportedMsgType == msgType);
}

LzmhDecodingAction::~LzmhDecodingAction() {
    FreeFileBuffer(fbIn);
    FreeFileBuffer(fbOut);
}

Direction LzmhDecodingAction::direction() { return dir; }

uint LzmhDecodingAction::order() { return SEQUENCE_NR; }

bool LzmhDecodingAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    if (!active) {
        return true;
    }

    auto cdrMsg = msg->get_rcl_serialized_message();

    InitFileBufferInMemory(fbIn, FBM_WRITING, cdrMsg.buffer_length);
    InitFileBufferInMemory(fbOut, FBM_WRITING, cdrMsg.buffer_length);

    WriteFileBuffer(fbIn, cdrMsg.buffer, cdrMsg.buffer_length);
    SetFileBufferMode(fbIn, FBM_READING);

    // Create & Init BitFileBuffer
    auto *bbIn = AllocateBitFileBuffer();
    auto *bbOut = AllocateBitFileBuffer();
    InitBitFileBuffer(bbIn, fbIn);
    InitBitFileBuffer(bbOut, fbOut);

    DecodeLZMH(bbIn, bbOut, &options);

    // Read decompressed data
    SetBitFileBufferMode(bbOut, FBM_READING);
    io_int_t bytes;
    uint8_t bits;
    GetActualBitFileSize(bbOut, &bytes, &bits);

    auto bitsToRead = bytes * BITS_IN_BYTE + bits;
    auto bytesToReserve = bytes + (bits == 0 ? 0 : 1);

    std::vector<uint8_t> buffer(bytesToReserve);

    auto ret = ReadBitFileBuffer(bbOut, &buffer.front(), bitsToRead);

    if (ret != bitsToRead) {
        std::cout << "LZMH Decompression: 💥 Error reading decompressed data." << std::endl;
        return false;
    }

    rcl_serialized_message_t newMsg{
        &buffer.front(),                      // buffer
        static_cast<size_t>(bytesToReserve),  // buffer_length
        static_cast<size_t>(bytesToReserve),  // buffer_capacity
        rcl_get_default_allocator()           // allocator
    };

    *msg = newMsg;

    // Destroy BitFileBuffer
    UninitBitFileBuffer(bbIn);
    UninitBitFileBuffer(bbOut);
    FreeBitFileBuffer(bbIn);
    FreeBitFileBuffer(bbOut);

    // Reset FileBuffer
    UninitFileBuffer(fbIn);
    UninitFileBuffer(fbOut);

    return true;
}

}  // namespace dtnproxy::pipeline
