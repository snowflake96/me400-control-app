# Connection Refused Fix

## Issue
When server is not running, client gets stuck in "waiting" state:
- `Connection refused` error (POSIXErrorCode 61)
- TCP RST packets from server
- No reconnection attempts after waiting state

## Root Cause
The `waiting` state in NWConnection wasn't handled properly:
- Only logged the error
- Didn't trigger reconnection
- Connection object remained in limbo

## Solution
1. **Handle waiting state as failure**
   - Cancel the connection
   - Clean up connection object
   - Trigger reconnection attempts

2. **Better error messages**
   - "Connection refused" → "Server not available"
   - More user-friendly error display

3. **Connection cleanup**
   - Always cancel existing connection before new attempt
   - Prevents stale connection issues

## Expected Behavior
```
NetworkManager: Connection state changed to: preparing
NetworkManager: Connection state changed to: waiting(Connection refused)
NetworkManager: Connection waiting with error: Connection refused
NetworkManager: Scheduling reconnection attempt 1/20 in 0.5s
NetworkManager: Attempting reconnection 1
[Continues every 0.5s until server is available]
```

## Result
- Client properly retries when server is not available
- No more stuck states
- Clear feedback about server availability 