# ME400 Control - Enhanced Swift Client

## Overview

This is an enhanced version of the ME400 Control Swift client with improved architecture, better separation of concerns, and modern Swift patterns.

## Architecture

### Core Components

1. **NProtocol.swift**
   - Defines the communication protocol with type-safe packet structures
   - Includes all packet types, payloads, and encoding/decoding logic
   - Provides factory methods for creating packets

2. **NNetworkManager.swift**
   - Handles all network communication using Apple's Network framework
   - Implements automatic reconnection, keepalive, and timeout handling
   - Uses Combine for reactive state updates
   - Includes mock implementation for testing

3. **NSystemStateManager.swift**
   - Manages the application's state in a centralized, thread-safe manner
   - Processes incoming packets and updates state accordingly
   - Publishes state changes using Combine

4. **NControlCoordinator.swift**
   - Coordinates between network and state management
   - Provides high-level control methods with async/await
   - Handles connection lifecycle and error propagation

### UI Components

1. **NME400App.swift**
   - Main app entry point with dependency injection
   - Manages app-wide settings using `@AppStorage`

2. **NContentView.swift**
   - Main content view with clean layout
   - Header with connection status and system controls
   - Split view with control panel and visualization area

3. **NControlViews.swift**
   - Mode-specific control views (Manual, AutoAim, Autonomous)
   - Reusable control components
   - System information display

4. **NSettingsView.swift**
   - Configuration for connection, PID parameters, and scene settings
   - Persistent storage of user preferences

## Key Improvements

### 1. Protocol Definition
- Type-safe packet definitions with proper enums
- Consistent endianness handling (little-endian)
- Clear separation between packet types and payloads

### 2. Network Layer
- Proper error handling with typed errors
- Automatic reconnection with configurable attempts
- Connection timeout and keepalive support
- Async/await API for cleaner code

### 3. State Management
- Centralized state with thread-safe updates
- Reactive updates using Combine
- Clear separation between network state and system state

### 4. Architecture
- Protocol-oriented design for testability
- Dependency injection throughout
- Mock implementations for testing
- Clear separation of concerns

### 5. UI/UX
- Modern SwiftUI patterns
- Responsive layout
- Clear visual feedback for connection status
- Organized settings and controls

## Usage

### Basic Setup

```swift
// Create coordinator with default configuration
let coordinator = ControlCoordinatorFactory.create()

// Or with custom configuration
let config = NetworkConfiguration(
    host: "192.168.1.100",
    port: 12345,
    connectionTimeout: 10.0,
    reconnectDelay: 2.0,
    maxReconnectAttempts: 5,
    keepaliveInterval: 5
)
let coordinator = ControlCoordinatorFactory.create(configuration: config)
```

### Sending Commands

```swift
// All commands use async/await
Task {
    // Control commands
    try await coordinator.sendServoCommand(pitch: 0.5, yaw: -0.3)
    try await coordinator.sendESCCommand(0.7)
    try await coordinator.sendTriggerCommand(true)
    
    // System control
    try await coordinator.start()
    try await coordinator.stop()
    try await coordinator.setMode(.autoAim)
    
    // PID tuning
    try await coordinator.tunePitch(p: 10.0, i: 0.5)
    try await coordinator.setPitchIntegralLimit(1.0)
}
```

### Observing State

```swift
// In SwiftUI views
struct MyView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    
    var body: some View {
        VStack {
            // Connection state
            if coordinator.connectionState.isConnected {
                Text("Connected")
            }
            
            // System state
            Text("Mode: \(coordinator.systemState.drivingMode.displayName)")
            Text("Running: \(coordinator.systemState.isRunning ? "Yes" : "No")")
            
            // Bounding box
            if let bbox = coordinator.systemState.boundingBox {
                Text("Target at: \(bbox.centerX), \(bbox.centerY)")
            }
        }
    }
}
```

## Testing

The architecture supports easy testing with mock implementations:

```swift
// Create mock coordinator for testing
let mockCoordinator = ControlCoordinatorFactory.createMock()

// Mock network manager simulates connection and data
let mockNetwork = MockNetworkManager()
```

## Migration from Original Code

1. Replace `ServerCommunicationManager.shared` with `ControlCoordinator`
2. Replace `ParameterManager.shared` with `coordinator.systemState`
3. Update packet creation to use `PacketFactory` methods
4. Convert completion handlers to async/await
5. Use `@EnvironmentObject` for dependency injection

## Future Enhancements

1. Add unit tests for all components
2. Implement data persistence for offline mode
3. Add analytics and logging
4. Create reusable UI component library
5. Add protocol versioning support
6. Implement secure authentication 