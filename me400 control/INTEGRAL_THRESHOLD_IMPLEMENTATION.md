# Integral Threshold Implementation

## Date: Dec 2024

### Overview
Added support for two new packet types:
- `SetPitchIntegralThreshold` (type 26)
- `SetYawIntegralThreshold` (type 27)

Each packet contains one double value representing the integral threshold.

### Changes Made

#### 1. Protocol Updates

**Enhanced App:**
- Added packet types to `NProtocol.swift`:
  - `case setPitchIntegralThreshold = 26`
  - `case setYawIntegralThreshold = 27`
- Added factory methods:
  - `PacketFactory.setPitchIntegralThreshold(_:)`
  - `PacketFactory.setYawIntegralThreshold(_:)`
- Added coordinator methods in `NControlCoordinator.swift`:
  - `setPitchIntegralThreshold(_:)`
  - `setYawIntegralThreshold(_:)`

**Updated App:**
- Added packet types to `DataPacket.swift`:
  - `case setPitchIntegralThreshold = 26`
  - `case setYawIntegralThreshold = 27`
- Added factory methods:
  - `DataPacket.setPitchIntegralThreshold(_:)`
  - `DataPacket.setYawIntegralThreshold(_:)`

#### 2. Data Model Updates

**Enhanced App:**
- Added to `SettingsStore` in `NME400App.swift`:
  - `@Published var sharedPitchThreshold: Double = 0.025`
  - `@Published var sharedYawThreshold: Double = 0.025`
  - `@AppStorage pitchIntegralThresholdStepSize: Double = 0.1`
  - `@AppStorage yawIntegralThresholdStepSize: Double = 0.1`

**Updated App:**
- Added to `ParameterManager.swift`:
  - `@Published var pitchIntegralThreshold: Double = 0.025`
  - `@Published var yawIntegralThreshold: Double = 0.025`
  - `@Published var pitchIntegralThresholdStepSize: Double = 0.01`
  - `@Published var yawIntegralThresholdStepSize: Double = 0.01`

#### 3. UI Updates

**PID Control Views:**
- Added "I Thres" controls below "I Limit" in both apps
- Added "Send Thres" buttons next to "Send Limit" buttons
- Range: 0.0 to 1.0
- Default value: 0.025

**Settings Views:**
- Added step size options: [0.001, 0.01, 0.1, 1.0, 10.0]
- Added PID step size settings for integral thresholds:
  - "Pitch Integ Thres"
  - "Yaw Integ Thres"

**Label Changes:**
- Renamed "Pitch Limit" → "Pitch Integ Limit"
- Renamed "Yaw Limit" → "Yaw Integ Limit"
- Renamed button text to match shorter labels

#### 4. Packet Handling

**Updated App:**
Added handlers in `ServerCommunicationManager.swift`:
```swift
case 26: // SetPitchIntegralThreshold
    let threshold = readDouble(at: 0, from: payload)
    messageText = "Received SetPitchIntegralThreshold: \(threshold)"
case 27: // SetYawIntegralThreshold
    let threshold = readDouble(at: 0, from: payload)
    messageText = "Received SetYawIntegralThreshold: \(threshold)"
```

### Usage

The integral threshold works similarly to the integral limit:
- It sets a threshold below which the integral term is not accumulated
- This helps prevent integral windup for small errors
- The value is sent to the server when the "Send Thres" button is pressed
- Server should implement: `pid.setIntegralThreshold(value)`

### Testing

To test the implementation:
1. Connect to the server
2. Navigate to AutoAim or Autonomous mode
3. Adjust the "I Thres" value using the +/- buttons
4. Press "Send Thres" to send the value to the server
5. Verify the server receives the correct packet type and value 