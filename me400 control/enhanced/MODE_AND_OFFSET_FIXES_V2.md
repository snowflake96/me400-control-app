# Mode and Target Offset Synchronization Fixes V2

## Critical Fix: Two-Phase Synchronization Model

### The Problem
1. Mode selector in client UI wasn't changing when user selected different modes
2. Previous fix incorrectly made server authoritative for all state after initial sync

### The Correct Model
As clarified by the user, synchronization works in two distinct phases:

#### Phase 1: Initial Sync (Server → Client)
- Client connects and sends Query packet
- Client waits for first CurrentState, ignoring all other packets
- When CurrentState arrives, client **adopts server's complete state**
- This ensures client starts with correct server state

#### Phase 2: Ongoing Operations (Client → Server)
- Client is now authoritative for user-controlled values (mode, target offset)
- Client sends Query every 1 second to get CurrentState
- If server's values don't match client's, **client updates server**
- This ensures server follows client's UI state

## Implementation Changes

### 1. SystemState Structure
Added `serverMode` to track server's reported mode separately:
```swift
struct SystemState {
    var drivingMode: DrivingMode     // Client's authoritative mode
    var serverMode: DrivingMode?     // Server's reported mode (for comparison)
    // ... other fields
}
```

### 2. CurrentState Processing
```swift
if isInitialSync {
    // Phase 1: Adopt server's state
    state.drivingMode = serverMode
    state.targetX = targetX
    state.targetY = targetY
    // ... adopt all values
} else {
    // Phase 2: Store server values for comparison only
    state.serverMode = serverMode  // Don't overwrite client's mode!
    state.targetX = targetX        // Store for offset comparison
    state.targetY = targetY
    // ... store all values for comparison
}
```

### 3. Mode Change Handling
```swift
func setMode(_ mode: DrivingMode) async throws {
    // Update client immediately (responsive UI)
    stateManager.updateDrivingMode(mode)
    
    // Send to server
    let packet = PacketFactory.setMode(mode)
    try await networkManager.send(packet)
}
```

### 4. Periodic Query Timer (Every 1 second)
```swift
// Send query
try? await sendQuery()

// Wait for response
try? await Task.sleep(nanoseconds: 100_000_000)

// Check mode sync
if let serverMode = systemState.serverMode,
   serverMode != systemState.drivingMode {
    // Server is out of sync, update it
    try? await sendModeToServer(systemState.drivingMode)
}

// Check target offset sync (AutoAim/Autonomous only)
if systemState.drivingMode == .autoAim || systemState.drivingMode == .autonomous {
    await checkAndSyncTargetOffset(...)
}
```

## Results
1. **Mode selector now works** - UI updates immediately when user changes mode
2. **Server stays in sync** - Periodic queries ensure server matches client
3. **Target offset sync fixed** - Only syncs once per second, not continuously
4. **Clear authority model** - No more confusion about who controls what

## Key Principle
After initial sync, the client UI is the source of truth for user-controllable values. The server must follow the client's state, not the other way around. 