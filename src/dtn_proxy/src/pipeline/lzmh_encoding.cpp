#include "pipeline/lzmh_encoding.hpp"

#include <iostream>

extern "C" {
#include "bit_file_buffer.h"
#include "file_buffer.h"
#include "lzmh.h"
}

namespace dtnproxy::pipeline {

LzmhEncodingAction::LzmhEncodingAction(const std::string &msgType) {
    fbIn = AllocateFileBuffer();
    fbOut = AllocateFileBuffer();
    options.error_log_file = stderr;
    active = !(unSupportedMsgType == msgType);
}

LzmhEncodingAction::~LzmhEncodingAction() {
    FreeFileBuffer(fbIn);
    FreeFileBuffer(fbOut);
}

Direction LzmhEncodingAction::direction() { return dir; }

uint LzmhEncodingAction::order() { return SEQUENCE_NR; }

bool LzmhEncodingAction::run(PipelineMessage &pMsg) {
    if (!active) {
        return true;
    }

    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

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
    io_int_t bytes;
    uint8_t bits;
    GetActualBitFileSize(bbOut, &bytes, &bits);

    auto bitsToRead = bytes * BITS_IN_BYTE + bits;
    auto bytesToReserve = bytes + (bits == 0 ? 0 : 1);

    pMsg.serializedMessage->reserve(bytesToReserve);
    auto ret = ReadBitFileBuffer(bbOut, pMsg.serializedMessage->get_rcl_serialized_message().buffer,
                                 bitsToRead);

    if (ret != bitsToRead) {
        std::cout << "LZMH Decompression: 💥 Error reading compressed data." << std::endl;
        return false;
    }

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
