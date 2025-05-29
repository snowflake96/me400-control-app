# Client-Server Synchronization Protocol

## Overview

The enhanced ME400 Control client implements a synchronization protocol to ensure the client state matches the server state before allowing any control operations.

## Synchronization Flow

1. **Connection Established**
   - Client connects to server via TCP
   - Connection state changes to `.connected`

2. **Query Sent**
   - Client automatically sends a `Query` packet to the server
   - Client enters "synchronizing" state
   - All control inputs are disabled

3. **Waiting for CurrentState**
   - Client ignores all incoming packets except `CurrentState`
   - Other packets (like BboxPos, FilteredBbox) are dropped
   - UI shows "Synchronizing..." status

4. **CurrentState Received**
   - Server responds with `CurrentState` packet containing:
     - Current driving mode (Manual/AutoAim/Autonomous)
     - Running state (Running/Stopped)
     - Launch counter
     - PID parameters
     - Motor settings
     - Other configuration values

5. **Synchronization Complete**
   - Client updates its internal state to match server
   - `isSynchronized` flag is set to `true`
   - Control inputs are enabled
   - Client starts processing all incoming packets

## Implementation Details

### SystemStateManager
- Tracks synchronization state with `hasSynchronized` flag
- Ignores non-CurrentState packets until synchronized
- Provides callback mechanism for post-synchronization actions

### ControlCoordinator
- Sends query packet on connection
- Publishes `isSynchronized` state for UI
- Manages synchronization callback

### UI Components
- Show synchronization status in ConnectionStatusView
- Disable all controls until synchronized
- Display "Waiting for server synchronization..." message

## Benefits

1. **State Consistency**: Ensures client and server are in sync before operations
2. **Safety**: Prevents sending commands based on stale or incorrect state
3. **User Feedback**: Clear indication of synchronization status
4. **Robustness**: Handles reconnections properly by re-synchronizing

## Code Example

```swift
// In ControlCoordinator
private func setupSynchronization() {
    // Set up callback for when synchronization is complete
    stateManager.setSynchronizationCallback { [weak self] in
        print("Client synchronized with server")
        self?.isSynchronized = true
    }
    
    // Send initial query
    Task {
        try await sendQuery()
        print("Query packet sent, waiting for CurrentState response...")
    }
}
```

## Troubleshooting

If synchronization fails:
1. Check server is sending CurrentState in response to Query
2. Verify CurrentState packet format matches protocol
3. Check network connectivity
4. Look for errors in console output 