# New Communication Protocol Implementation

## Overview
Implemented a simplified communication protocol where the client maintains its own state and only syncs with the server once on initial connection.

## Key Changes

### 1. **Connection Flow**
- **Connect Button**: When pressed, sends Query packet and waits for CurrentState response
- **Initial Sync**: Client adopts server's state completely on first CurrentState
- **After Sync**: Client maintains its own state, server state is only for display

### 2. **State Management**
- **Client State**: Updated immediately when user interacts with UI
- **Server State**: Stored separately for display in ServerStateView
- **No Reconciliation**: Client doesn't try to keep in sync with server after initial connection

### 3. **Control Behavior**
- **Start/Stop**: Sends packet then updates client state immediately
- **Mode Change**: Sends packet then updates client mode immediately  
- **Settings**: Only sent when user explicitly presses Send button
- **Servo/ESC**: Sent on every slider change with values rounded to 2 decimal places

### 4. **New UI Layout**
- **Camera View**: Top-right corner
- **ServerStateView**: Left side showing 11 server state values
- **TargetOffsetControl**: Bottom spanning full width
- **L-shaped** layout for efficient space usage

### 5. **Component Structure**
- **NESCControlView**: Shared ESC control for Manual and AutoAim modes
- **PIDControlView**: Shared PID controls for AutoAim and Autonomous modes
- **StatusView**: Shared status display for AutoAim and Autonomous modes
- **ServerStateView**: Displays CurrentState data from periodic queries

### 6. **Communication Pattern**
```
1. Connect → Send Query → Receive CurrentState → Sync Once
2. User Action → Update Client State → Send Command to Server
3. Every 1s → Send Query → Receive CurrentState → Update ServerStateView Only
```

### 7. **Mode Values**
- Manual: 8 (SetManual)
- AutoAim: 9 (SetAutoAim)  
- Autonomous: 7 (SetAutonomous)

### 8. **Disabled State**
- All controls except Settings and Connect are disabled when not connected
- Controls are enabled only after successful synchronization

## Benefits
- Simpler state management
- Immediate UI response to user actions
- Clear separation between client state and server display
- Reduced complexity compared to continuous reconciliation 