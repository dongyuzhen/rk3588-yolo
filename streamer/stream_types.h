#pragma once
#include <cstdint>
#include <cstddef>
#include <memory>

struct EncodedPacket{
    std::shared_ptr<uint8_t> data;
    size_t size;
    int64_t pts;
    int64_t dts;
};