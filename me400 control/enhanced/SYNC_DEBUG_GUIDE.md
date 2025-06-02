# Synchronization Debug Guide

## Expected Flow

1. **Connection Established**
   ```
   NetworkManager: Connection state changed to: ready
   ControlCoordinator: Connection state changed to connected
   ControlCoordinator: Starting synchronization setup after delay
   ```

2. **Synchronization Setup**
   ```
   ControlCoordinator: Setting up synchronization
   ControlCoordinator: Callbacks set, sending initial query
   Initial sync query sent
   ```

3. **Server Response**
   ```
   Received packet: Current State
   SystemStateManager processing packet: Current State
   Processing CurrentState packet
   CurrentState packet size: 64 bytes
   SystemStateManager: Initial sync detected
   SystemStateManager: Marked as synchronized, calling callback
   SystemStateManager: Calling synchronization callback
   ```

4. **Synchronization Complete**
   ```
   ControlCoordinator: Synchronization callback triggered
   Client synchronized with server
   ```

## Common Issues

### Issue 1: Old Configuration Values
- **Symptom**: Logs show "2.0s" delay and "5" max attempts
- **Fix**: Update `SettingsStore.createNetworkConfiguration()` to use new values

### Issue 2: Synchronization Stuck
- **Symptom**: Client shows "Synchronizing..." despite receiving CurrentState
- **Possible Causes**:
  1. Synchronization callback not set before CurrentState arrives
  2. CurrentState packet arrives during connection setup
  3. Race condition between connection and packet processing

### Issue 3: No Sync Callback
- **Symptom**: "WARNING - No synchronization callback set!"
- **Fix**: Ensure setupSynchronization() is called after connection

## Debug Steps

1. Check connection flow in logs
2. Verify CurrentState packet is received
3. Look for "Initial sync detected" message
4. Check if synchronization callback is triggered
5. Verify client UI updates to show synchronized state 