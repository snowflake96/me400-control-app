# Latency Calculation in ME400 Control App

## Overview
The latency displayed in the app represents the **round-trip time** for a Query-Response cycle between the client and server.

## How It Works

### 1. Query Sent (Start Timer)
When the client sends a `Query` packet to the server:
```swift
func sendQuery() async throws {
    // Record the current time when Query is sent
    latencyQueue.async {
        self.lastQueryTime = Date()
    }
    
    let packet = PacketFactory.query()
    try await networkManager.send(packet)
}
```

### 2. CurrentState Received (Calculate Latency)
When the server responds with a `CurrentState` packet:
```swift
// In NSystemStateManager
private func processCurrentState(_ payload: Data) {
    // ... process packet data ...
    
    // Notify coordinator that CurrentState was received
    if let callback = onCurrentStateReceived {
        DispatchQueue.main.async {
            callback()
        }
    }
}

// In NControlCoordinator
func updateLatency() {
    latencyQueue.async { [weak self] in
        guard let self = self,
              let queryTime = self.lastQueryTime else { return }
        
        // Calculate time difference
        let currentLatency = Date().timeIntervalSince(queryTime)
        
        DispatchQueue.main.async {
            self.latency = currentLatency  // In seconds
        }
    }
}
```

### 3. Display in UI
The latency is displayed in milliseconds:
```swift
Text("(\(Int(coordinator.latency * 1000))ms)")
```

## What the Latency Includes

The measured latency includes:

1. **Network Transit Time (Client → Server)**
   - Time for Query packet to travel from client to server
   - Includes network delays, routing, etc.

2. **Server Processing Time**
   - Time for server to receive the Query
   - Time to prepare CurrentState response
   - Time to send the response

3. **Network Transit Time (Server → Client)**
   - Time for CurrentState packet to travel back
   - Network delays on return path

4. **Client Processing Overhead**
   - Minimal time to process received packet
   - Time to trigger callback and calculate latency

## Interpretation

- **< 50ms (Green)**: Excellent connection, minimal delay
- **50-150ms (Orange)**: Good connection, acceptable for most operations
- **> 150ms (Red)**: High latency, may experience noticeable delays

## Important Notes

1. **Not Just Network Ping**: This includes server processing time, so it may be higher than a simple network ping.

2. **Query Frequency**: With queries sent every 200ms (5Hz), you get frequent latency updates.

3. **Accuracy**: Uses Swift's `Date()` which provides millisecond precision, suitable for this use case.

4. **Thread Safety**: Latency calculation uses a dedicated queue to ensure thread-safe access to timing data.

## Example Timeline
```
Time 0ms:    Client sends Query packet → [lastQueryTime = now]
Time 15ms:   Query arrives at server
Time 20ms:   Server processes and sends CurrentState
Time 35ms:   CurrentState arrives at client
Time 36ms:   Client calculates latency = 36ms
```

This 36ms latency represents the complete round-trip communication time. 