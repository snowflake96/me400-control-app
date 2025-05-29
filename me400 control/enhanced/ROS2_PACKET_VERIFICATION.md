# ROS2 Server Packet Structure Verification

## Overview
This document verifies that the Swift client correctly reads all packet types from the ROS2 server.

## Packet Structures from Server

### 1. ✅ BboxPos (Type 14)
**Server sends** (master.cpp line 398-405):
```cpp
packet.type = DataPacket::Type::BboxPos;
packet.data.double_array[0] = x1;  // offset 0
packet.data.double_array[1] = y1;  // offset 8
packet.data.double_array[2] = x2;  // offset 16
packet.data.double_array[3] = y2;  // offset 24
```

**Swift client reads** (NSystemStateManager.swift):
```swift
let x1 = PacketDecoder.readDouble(at: 0, from: payload)   // ✅ Correct
let y1 = PacketDecoder.readDouble(at: 8, from: payload)   // ✅ Correct
let x2 = PacketDecoder.readDouble(at: 16, from: payload)  // ✅ Correct
let y2 = PacketDecoder.readDouble(at: 24, from: payload)  // ✅ Correct
```

### 2. ✅ FilteredBbox (Type 19)
**Server sends** (master.cpp line 433-436):
```cpp
packet.type = DataPacket::Type::FilteredBbox;
packet.data.vector3.x = center_x;  // offset 0
packet.data.vector3.y = center_y;  // offset 8
// vector3.z not used              // offset 16
```

**Swift client reads**:
```swift
let centerX = PacketDecoder.readDouble(at: 0, from: payload)  // ✅ Correct
let centerY = PacketDecoder.readDouble(at: 8, from: payload)  // ✅ Correct
```

### 3. ✅ Tilt (Type 15)
**Server sends** (master.cpp line 474-478):
```cpp
packet.type = DataPacket::Type::Tilt;
packet.data.vector3.x = tilt_.roll;   // offset 0
packet.data.vector3.y = tilt_.pitch;  // offset 8
```

**Swift client reads**:
```swift
let roll = PacketDecoder.readDouble(at: 0, from: payload)   // ✅ Correct
let pitch = PacketDecoder.readDouble(at: 8, from: payload)  // ✅ Correct
```

### 4. ✅ LaunchCounter (Type 21)
**Server sends** (master.cpp line 503-506):
```cpp
packet.type = DataPacket::Type::LaunchCounter;
packet.data.uint32 = launch_counter_;  // offset 0 (4 bytes)
```

**Swift client reads**:
```swift
let counter = PacketDecoder.readUInt32(at: 0, from: payload)  // ✅ Correct
```

### 5. ✅ CurrentState (Type 25)
**Server sends** (master.cpp line 314-333):
```cpp
state_info.mode = DataPacket::Type::SetAutonomous/SetManual/SetAutoAim;  // offset 0
state_info.motor_state = DataPacket::Type::Start/Stop;                   // offset 1
state_info.launch_counter = launch_counter_;                             // offset 2
state_info.N = launch_threshold_.N;                                      // offset 6
state_info.EPS = launch_threshold_.EPS;                                  // offset 7
state_info.target_x = target_.x;                                         // offset 15
state_info.target_y = target_.y;                                         // offset 23
state_info.stop_throttle = stop_throttle_;                               // offset 31
state_info.motor_offset = motor_offset_;                                 // offset 39
state_info.default_speed = DEFAULT_SPEED_;                               // offset 47
state_info.cutoff_freq = lpf_x.getCutoffFrequency();                    // offset 55
```

**Swift client reads** (after fix):
All offsets now match correctly after fixing the targetX/targetY offset issue.

### 6. ✅ Log (Type 16)
**Server sends** (via server_.sendLogToClient):
Text data in char text[64]

**Swift client reads**:
```swift
let message = PacketDecoder.readString(from: payload)  // ✅ Correct
```

## Conclusion
All packet types are now correctly read by the Swift client. The main issue was with CurrentState packet where:
- targetX was being read from offset 10 instead of 15
- targetY was being read from offset 18 instead of 23

This has been fixed, and all other packet types were already correct. 