#include "pipeline/lzmh_decoding.hpp"

#include <rclcpp/serialized_message.hpp>
#include <vector>

extern "C" {
#include "bit_file_buffer.h"
#include "file_buffer.h"
#include "lzmh.h"
}

namespace dtnproxy::pipeline {

LzmhDecodingAction::LzmhDecodingAction() {
    fbIn = AllocateFileBuffer();
    fbOut = AllocateFileBuffer();
    options.error_log_file = stderr;
}

LzmhDecodingAction::~LzmhDecodingAction() {
    FreeFileBuffer(fbIn);
    FreeFileBuffer(fbOut);
}

Direction LzmhDecodingAction::direction() { return dir; }

uint LzmhDecodingAction::order() { return SEQUENCE_NR; }

bool LzmhDecodingAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
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
    auto decompressedSize = GetActualFileSize(fbOut);
    std::vector<uint8_t> buffer(decompressedSize);

    ReadFileBuffer(fbOut, &buffer.front(), decompressedSize);

    rcl_serialized_message_t newMsg{
        &buffer.front(),                        // buffer
        static_cast<size_t>(decompressedSize),  // buffer_length
        static_cast<size_t>(decompressedSize),  // buffer_capacity
        rcl_get_default_allocator()             // allocator
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
