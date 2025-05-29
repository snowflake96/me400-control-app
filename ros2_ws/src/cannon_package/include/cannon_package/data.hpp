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
        Tilt,
        Log,
        Query,
        MotorOffset,
        FilteredBbox,
        SetCutoffFrequency,
        LaunchCounter, 
        SetStopThrottle,
        SetMaxConsecutiveNans,
        SetDefaultSpeed,
        CurrentState
    } type;

    union Data {
        char text[64];
        double double_array[8];
        uint32_t uint32;
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
        struct CurrentState{
            Type mode; // Auto, autoaim, manual
            Type motor_state; // either running or stopped
            uint32_t launch_counter;
            uint8_t N;
            double EPS;
            double target_x;
            double target_y;
            double stop_throttle;
            double motor_offset;
            double default_speed;
            double cutoff_freq;
        } state;
    } data;
};
#pragma pack(pop)