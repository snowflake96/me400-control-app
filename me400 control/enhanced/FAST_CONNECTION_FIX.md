# Fast Connection Fix

## Issue
Connection timeout was too long (10 seconds), causing delays when server becomes available.

## Solution
Updated `NetworkConfiguration.default` with faster timings:
- **connectionTimeout**: 1.0 second (was 10.0)
- **reconnectDelay**: 0.5 seconds (was 2.0)
- **maxReconnectAttempts**: 20 (was 5)

## Behavior
1. Client tries to connect for 1 second
2. If timeout, waits 0.5 seconds before retrying
3. Continues retrying up to 20 times (10 seconds total)

## Result
- When server is available: Connection within 0.5 seconds
- When server comes online: Connection within 0.5-1.5 seconds
- Much more responsive connection experience

## Timeline Example
```
0.0s - Click connect
1.0s - Timeout, start reconnect timer
1.5s - Retry connection
2.5s - Timeout, start reconnect timer
3.0s - [Server starts here]
3.0s - Retry connection
3.2s - Connected! (only 0.2s after server available)
``` 