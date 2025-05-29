# Client-Server Synchronization Flow

## Overview
The ME400 Control app uses a two-phase synchronization approach:
1. **Initial Sync**: Client adopts server state
2. **Ongoing Sync**: Client is authoritative and updates server if needed

## Phase 1: Initial Connection & Synchronization

### Steps:
1. User presses "Connect" button
2. Client establishes TCP connection to server
3. Client sends `Query` packet to request server state
4. Client ignores all other packets while waiting for `CurrentState`
5. When first `CurrentState` packet arrives:
   - Client adopts ALL server values (mode, parameters, etc.)
   - Client marks itself as "synchronized"
   - Synchronization callback triggers

### Code Flow:
```
connect() -> setupSynchronization() -> sendQuery() -> processCurrentState(isInitialSync=true)
```

## Phase 2: Ongoing Synchronization

### Periodic Query (Every 1 second):
1. Client sends `Query` packet
2. Server responds with `CurrentState` packet
3. Client stores server values separately (e.g., `serverMode`)
4. Client compares its state with server state
5. If mismatch detected, client sends its values to server

### Mode Changes:
1. User changes mode in UI
2. Client immediately updates its local `drivingMode`
3. Client sends `SetMode` packet to server
4. UI reflects change immediately (responsive)
5. Next periodic query will detect if server didn't update and resend

### Key Differences from Initial Sync:
- **Initial**: `state.drivingMode = serverMode` (adopt server value)
- **Ongoing**: `state.serverMode = serverMode` (store for comparison only)
- Client's `drivingMode` is never overwritten after initial sync

## State Management

### SystemState Structure:
```swift
struct SystemState {
    var drivingMode: DrivingMode      // Client's authoritative mode
    var serverMode: DrivingMode?      // Server's reported mode (for comparison)
    var targetX: Double              // Server's view of target offset
    var targetY: Double              // Server's view of target offset
    // ... other parameters
}
```

### Mode Synchronization Logic:
```swift
// In periodic query timer
if let serverMode = systemState.serverMode,
   serverMode != systemState.drivingMode {
    // Server is out of sync, update it
    sendModeToServer(systemState.drivingMode)
}
```

### Target Offset Synchronization:
- Only checked in AutoAim/Autonomous modes
- Compares client's offset (from SettingsStore) with server's (from SystemState)
- Updates server if difference > 0.001

## Timer Management

### Manual Mode:
- No periodic queries
- No target offset sync

### AutoAim/Autonomous Modes:
- Query every 1 second
- Check mode sync
- Check target offset sync

## Benefits
1. **Responsive UI**: Mode changes reflect immediately
2. **Reliable Sync**: Server eventually matches client state
3. **Efficient**: Only sends updates when needed
4. **Clear Authority**: Client controls UI state, server follows 