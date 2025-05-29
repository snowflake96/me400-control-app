# Server Target Offset Bug Fix

## Problem
The client was receiving garbage values for targetX and targetY from the server:
```
Target offset mismatch - Server: (-4.50116055520744e+229, 5.654086680502581e+76), Client: (0.18199999999999994, -0.225)
```

These extremely large values indicated uninitialized memory or incorrect data handling.

## Root Cause
The server (`bounding_box_server.py`) was not properly tracking the target offset values:

1. When creating CurrentState packets, it always set targetX and targetY to 0.0
2. It didn't store the values when receiving SetOffset commands from the client
3. Same issue existed for other parameters like maxConsecutiveNans, stopThrottle, etc.

## Solution
Modified the server to:

1. **Add state variables** to track all parameters:
   ```python
   target_x = 0.0
   target_y = 0.0
   max_consecutive_nans = 10
   stop_throttle = 0.0
   motor_offset = 0.0
   default_speed = 1.0
   cutoff_freq = 10.0
   launch_counter = 0
   ```

2. **Handle Set commands** and update the state:
   ```python
   if data[0] == 10:  # SetOffset
       target_x = struct.unpack('<d', data[1:9])[0]
       target_y = struct.unpack('<d', data[9:17])[0]
   ```

3. **Include current values** in CurrentState packets:
   ```python
   struct.pack_into('<d', current_state_packet, 11, target_x)
   struct.pack_into('<d', current_state_packet, 19, target_y)
   ```

## Additional Fixes
- Added handling for SetAutoAim mode (packet type 9)
- Added handlers for all parameter update commands
- Server now properly maintains state across client queries

## Testing
After applying these fixes:
1. Restart the server
2. Connect the iOS client
3. The target offset values should now sync properly
4. No more garbage values should appear in the debug output 