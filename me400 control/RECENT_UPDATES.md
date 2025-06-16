# Recent Updates

## 1. Launch Counter Data Type Change

### Change Made
- **SystemState**: Changed `launchCounter` from `UInt32` to `UInt8`
- **processLaunchCounter**: Updated to read UInt8 (1 byte) instead of UInt32 (4 bytes)
- **Payload size check**: Changed from `>= 4` to `>= 1` bytes

### Reason
Server now sends launch counter as UInt8 instead of UInt32.

## 2. Target Offset Synchronization Confirmation

### Question Answered
**Does the client sync target offset values when getting the first CurrentState packet?**

**Answer: YES**

### How it works:
1. `processCurrentState()` reads `targetX` and `targetY` from CurrentState packet
2. Values are stored in `systemState.targetX` and `systemState.targetY`
3. `NContentView.swift` copies these to SettingsStore when synchronized:
   ```swift
   settingsStore.targetOffsetX = coordinator.systemState.targetX
   settingsStore.targetOffsetY = coordinator.systemState.targetY
   ```

## 3. Offset Slider Range Expansion

### Change Made
Updated target offset controls from **-0.1 to 0.1** to **-0.2 to 0.2**

### Files Modified
- `NCameraView.swift`: 
  - YOffsetControlView: Updated buttons and slider range
  - XOffsetControlView: Updated buttons and slider range

### Components Updated
- Step buttons (+/-) now clamp to -0.2/+0.2
- Sliders now have range -0.2...0.2
- Step size remains 0.001 for precision

## Summary
All three requested changes have been implemented:
1. ✅ Launch counter now uses UInt8
2. ✅ Confirmed target offset synchronization behavior  
3. ✅ Offset sliders expanded to ±0.2 range 