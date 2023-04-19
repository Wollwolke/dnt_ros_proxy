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

bool LzmhDecodingAction::run(PipelineMessage &pMsg) {
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

    DecodeLZMH(bbIn, bbOut, &options);

    // Read decompressed data
    SetBitFileBufferMode(bbOut, FBM_READING);
    io_int_t bytes;
    uint8_t bits;
    GetActualBitFileSize(bbOut, &bytes, &bits);

    auto bitsToRead = bytes * BITS_IN_BYTE + bits;
    auto bytesToReserve = bytes + (bits == 0 ? 0 : 1);

    pMsg.serializedMessage->reserve(bytesToReserve);
    auto *newCdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    newCdrMsg->buffer_length = bytesToReserve;

    auto ret = ReadBitFileBuffer(bbOut, newCdrMsg->buffer, bitsToRead);

    if (ret != bitsToRead) {
        std::cout << "LZMH Decompression: 💥 Error reading decompressed data." << std::endl;
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
