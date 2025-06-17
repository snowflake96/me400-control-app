# ME400 Control - Enhanced Swift Client

## Overview

ME400 Control – Enhanced Swift Client is a modern, modular Swift application for managing and monitoring the ME400 system with robust networking, strict protocol handling, thread-safe state management, and a responsive SwiftUI interface.

## Architecture

### Core Components

1. **NProtocol.swift**
   - Defines the binary communication protocol in a type-safe manner
   - Implements all packet types, enums, payloads, and encoding/decoding logic
   - Ensures strict endianness requirements (little-endian)
   - Provides factory methods for constructing protocol packets

2. **NNetworkManager.swift**
   - Encapsulates all network I/O using Apple’s Network framework
   - Implements automatic reconnection, keepalive handling, and connection timeouts
   - Publishes connection and error state via Combine publishers
   - Includes a `MockNetworkManager` conforming to the same interface for testing

3. **NSystemStateManager.swift**
   - Maintains application system state in a central, thread-safe manner
   - Applies updates from incoming protocol packets
   - Publishes state changes using Combine for real-time UI and logic updates
   - Clearly separates network state (connection) from system state (model data)

4. **NControlCoordinator.swift**
   - Bridges the network and system state layers
   - Provides async/await APIs for all control commands and configuration
   - Manages connection life cycle and error propagation
   - Coordinates dependency injection for testability and modularity

### UI Components

1. **NME400App.swift**
   - SwiftUI app entry point
   - Sets up dependency injection and app storage using `@AppStorage`
   - Configures global state and mock/testing modes

2. **NContentView.swift**
   - Main UI dashboard
   - Displays connection status and system controls in a prominent header
   - Presents a flexible split view for control panels and live visualization

3. **NControlViews.swift**
   - Contains all mode-specific control panels (Manual, AutoAim, Autonomous)
   - Houses reusable UI logic and system diagnostics
   - Displays real-time system information and feedback

4. **NSettingsView.swift**
   - Presents configuration for connection parameters, PID values, and scene settings
   - Persists user preferences using app storage

## Key Improvements

### Protocol Layer
- All packets and payloads defined with precise Swift enums and structs
- Endianness and wire format strictly enforced
- Factory API for safe packet construction

### Network Layer
- Typed error reporting and propagation
- Configurable automatic reconnection and keepalive intervals
- Connection state published reactively with Combine
- Full async/await command support
- Mock network manager included

### State Management
- Centralized, thread-safe state with single source of truth
- Real-time updates via Combine publishers
- Strong separation of network/transport from system model

### Architecture
- Protocol-oriented for maximum testability
- Complete dependency injection for all major components
- Dedicated mocks for isolated testing
- UI and business logic are fully decoupled

### UI/UX
- Modular SwiftUI views with clean, responsive layouts
- Explicit feedback for connection and system state
- Persistent settings and mode switching
- Organized controls for each system mode

## Usage

### Initialization

```swift
// Default initialization
let coordinator = ControlCoordinatorFactory.create()

// Custom network configuration
let config = NetworkConfiguration(
    host: "192.168.1.100",
    port: 12345,
    connectionTimeout: 10.0,
    reconnectDelay: 2.0,
    maxReconnectAttempts: 5,
    keepaliveInterval: 5
)
let coordinator = ControlCoordinatorFactory.create(configuration: config)
