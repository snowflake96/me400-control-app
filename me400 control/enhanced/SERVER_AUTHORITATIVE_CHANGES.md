# Server-Authoritative Architecture Changes

## Overview
Changed the client-server synchronization model from client-authoritative to server-authoritative. The server is now the single source of truth for all state.

## Key Changes

### 1. **SystemStateManager** 
- `processCurrentState()` now always updates client state from server
- Removed logic that prevented updating client-managed parameters after initial sync
- Both initial sync and ongoing queries work the same way - client adopts server state

### 2. **ControlCoordinator**
- Removed mode mismatch detection and correction logic
- Removed target offset synchronization checks
- Query timer now simply sends queries without any reconciliation logic
- `setMode()` no longer updates local state immediately - waits for server response

### 3. **SettingsView Target Offset**
- Added onChange handlers to send offset changes to server immediately
- Added onReceive to sync local values from server state
- Reset button now sends the reset command to server

### 4. **SystemSettingsView**
- Still prevents updates while user is editing (via @FocusState)
- Shows server values when not editing
- Sends all values to server when "Send Settings" is pressed

## Behavior
1. **Initial Connection**: Client sends query, receives CurrentState, adopts all values
2. **Ongoing Operation**: Client sends query every 1s, adopts server's CurrentState response
3. **User Changes**: 
   - Settings are sent to server
   - Client waits for server's CurrentState response to update UI
   - No immediate UI updates (except for user input fields)

## Benefits
- Simpler architecture - no reconciliation logic needed
- Server is single source of truth
- No possibility of client-server state divergence
- Consistent behavior between initial sync and ongoing operation 