# ROS2 Server CurrentState Packet Format

## Problem
The Swift client was receiving garbage values for targetX and targetY:
```
Target offset mismatch - Server: (5.9019554118352744e+293, 4.06978633747e-312), Client: (-0.20600000000000002, 0.0)
```

## Root Cause
The Swift client was reading from incorrect byte offsets. The actual ROS2 server's `CurrentState` structure (from `data.hpp`) is:

```cpp
struct CurrentState{
    Type mode;              // offset 0 (1 byte) - driving mode
    Type motor_state;       // offset 1 (1 byte) - running/stopped
    uint32_t launch_counter;// offset 2 (4 bytes)
    uint8_t N;              // offset 6 (1 byte) - launch threshold N
    double EPS;             // offset 7 (8 bytes) - launch threshold epsilon
    double target_x;        // offset 15 (8 bytes)
    double target_y;        // offset 23 (8 bytes)
    double stop_throttle;   // offset 31 (8 bytes)
    double motor_offset;    // offset 39 (8 bytes)
    double default_speed;   // offset 47 (8 bytes)
    double cutoff_freq;     // offset 55 (8 bytes)
} state;
```

## Swift Client Was Reading Wrong Offsets
The client was reading:
- targetX from offset 10 (middle of EPS field!)
- targetY from offset 18 (middle of target_x field!)

This caused the garbage values.

## Solution
Updated the Swift client to read from correct offsets:
- targetX: offset 15
- targetY: offset 23

## Key Differences from Expected Format
1. Server doesn't send `maxConsecutiveNans` in CurrentState
2. Server includes launch threshold parameters (N and EPS) that client wasn't expecting
3. Total payload size is 63 bytes (excluding the packet type byte)

## Notes
- The server code is read-only on Raspberry Pi
- Client must adapt to server's packet format
- `#pragma pack(push, 1)` ensures no padding between struct members 