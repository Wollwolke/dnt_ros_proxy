#include "pipeline/lzmh_encoding.hpp"

extern "C" {
#include "bit_file_buffer.h"
#include "file_buffer.h"
#include "lzmh.h"
}

namespace dtnproxy::pipeline {

LzmhEncodingAction::LzmhEncodingAction() {
    fbIn = AllocateFileBuffer();
    fbOut = AllocateFileBuffer();
    options.error_log_file = stderr;
}

LzmhEncodingAction::~LzmhEncodingAction() {
    FreeFileBuffer(fbIn);
    FreeFileBuffer(fbOut);
}

Direction LzmhEncodingAction::direction() { return dir; }

uint LzmhEncodingAction::order() { return SEQUENCE_NR; }

bool LzmhEncodingAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
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

    EncodeLZMH(bbIn, bbOut, &options);

    // Read compressed data
    SetBitFileBufferMode(bbOut, FBM_READING);
    auto compressedSize = GetActualFileSize(fbOut);
    msg->reserve(compressedSize);
    ReadFileBuffer(fbOut, msg->get_rcl_serialized_message().buffer, compressedSize);

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