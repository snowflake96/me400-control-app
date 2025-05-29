// Created by You, Jisang 2025/3/13
#pragma once

// Do not memory-align struct members
#pragma pack(push, 1)
#include <cstdint>

// Total size of the data packet is 65 bytes
struct DataPacket {
    enum class Type : uint8_t {
        COMMAND=0,
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        START,
        STOP,
        AUTONOMOUS
    } type;

    union Data {
        char text[64];
        int32_t integer;
        float floating_point;
        struct {
            double x;
            double y;
            double z;
        } vector3;
    } data;
};
#pragma pack(pop)