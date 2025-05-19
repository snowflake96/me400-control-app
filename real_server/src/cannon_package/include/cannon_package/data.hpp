// Created by You, Jisang 2025/3/13
#pragma once

// Do not memory-align struct members
#pragma pack(push, 1)
#include <cstdint>

// Total size of the data packet is 65 bytes
struct DataPacket {
    enum class Type : uint8_t {
        ServoCommand=0,
        TriggerCommand,
        EscCommand,
        TunePitch,
        TuneYaw,
        Start,
        Stop,
        SetAutonomous,
        SetManual,
        SetAutoAim,
        SetOffset,
        SetPitchIntegralLimit,
        SetYawIntegralLimit,
        SetLaunchThreshold,
        BboxPos,
        PitchAngle,
        Log,
        Query // Added
    } type;

    union Data {
        char text[64];
        double double_array[8];
        bool boolean;
        double w;
        struct Vec3 {
            double x;
            double y;
            double z;
        } vector3;
        struct Threshold{
            double EPS;
            uint8_t N;
        } threshold;
    } data;
};
#pragma pack(pop)