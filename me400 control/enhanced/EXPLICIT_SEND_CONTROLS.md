# Explicit Send Controls

## Overview
Updated all controls to require explicit user action to send data to the server. Nothing is sent automatically on value changes.

## Changes Made

### 1. **Filtered Box Visibility**
- Filtered bounding box is already shown in ALL modes (including manual)
- No mode checks in NCameraView.swift - it displays whenever data is available

### 2. **PID Parameters**
- Removed individual "Send PI" and "Send Limit" buttons
- Added single "Send All PID Parameters" button that sends all 6 values:
  - Pitch P & I gains
  - Pitch integral limit
  - Yaw P & I gains  
  - Yaw integral limit
- 50ms delay between each send to avoid overwhelming the server
- Applied to both AutoAimControlsView and AutonomousControlsView

### 3. **Target Offset Control**
- Already had explicit send control
- Sliders don't send on change
- "Send" button sends the offset values to server
- "Reset" button only resets local values (does not send to server)

### 4. **ESC Control**
- Slider doesn't send on change
- "Send ESC" button sends the current value
- Applied to both ManualControlsView and AutoAimControlsView

### 5. **System Settings**
- All text fields don't send on change
- "Send Settings" button sends all values

### 6. **Start/Stop**
- Only sends Start or Stop packet
- No automatic configuration sending

### 7. **Mode Switching**
- Only sends the mode change packet
- No automatic parameter sending

## User Workflow
1. User adjusts values in UI (sliders, text fields, etc.)
2. Values are stored locally in the app
3. User presses the appropriate "Send" button
4. App sends the values to server
5. Server updates and responds with CurrentState
6. UI updates to reflect server's state

## Benefits
- Predictable behavior - user knows exactly when data is sent
- Reduced network traffic
- No accidental sends while adjusting values
- Clear separation between UI state and server communication
- Consistent pattern across all controls 