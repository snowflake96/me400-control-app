# Simplified Control Behavior

## Overview
Simplified the client-server interactions to be more explicit and predictable. Settings are only sent when explicitly requested by the user.

## Key Changes

### 1. **Start/Stop Buttons**
- Only send Start or Stop packet when pressed
- No automatic configuration sending on start
- Server handles all initialization

### 2. **Mode Switching**
- Only sends the mode change command
- No automatic ESC reset
- No automatic PID parameter sending
- No automatic target offset sending

### 3. **Target Offset Settings**
- Removed automatic sending on slider change
- Added explicit "Send to Server" button
- Reset button only resets local values
- Values still sync from server via CurrentState

### 4. **System Settings**
- Only sent when "Send Settings" button is pressed
- No automatic sending on any event

## New Behavior

### When Starting:
1. User presses Start button
2. Client sends Start packet
3. Server starts with its current configuration
4. Server sends CurrentState with its values

### When Changing Mode:
1. User selects new mode
2. Client sends SetMode packet
3. Server changes mode and responds with CurrentState

### When Adjusting Settings:
1. User adjusts values in UI
2. Values are stored locally
3. User presses "Send" button
4. Client sends the settings to server
5. Server updates and responds with CurrentState

## Benefits
- More predictable behavior
- User has explicit control over when settings are sent
- Reduces network traffic
- Clearer separation between local UI state and server state
- Server remains the authoritative source 