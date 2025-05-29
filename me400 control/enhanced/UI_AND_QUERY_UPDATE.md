# UI and Query Update Summary

## Date: May 30, 2025

### 1. Server Terminal Messages
To diagnose the repeated messages from master_node in autonomous/autoaim mode, please share:
- The exact messages being shown in the server terminal
- Whether they appear only in specific modes
- Their frequency

This will help determine if it's a client issue (sending too many commands) or server issue.

### 2. Query Sending in All Modes ✅
**Change Made**: Removed mode-based query timer management

**Before**: Query packets were only sent in AutoAim and Autonomous modes
**After**: Query packets are sent continuously every 0.2 seconds (5Hz) regardless of mode

**Benefits**:
- ServerStateView updates in all modes including Manual
- Consistent latency monitoring
- Real-time server state visibility

**Code Change Location**: `NControlCoordinator.swift` lines 100-119

### 3. UI Improvements ✅

#### ServerStateView Width Reduction
**Change**: Reduced width from 30% to 20% of container width
- Less white space
- More room for camera view
- Still readable with reduced font sizes

#### Target Offset Y Control - Vertical Layout
**Change**: Implemented custom vertical slider for Y offset

**Features**:
- Vertical slider matches physical Y-axis movement
- Red line indicates zero position
- Value displayed at top
- Drag gesture for smooth control
- More intuitive: "up means up"

**Layout**:
```
[Title] [X Horizontal Slider ----------] [Y Vertical] [Buttons]
                                           Slider
```

### Visual Improvements
1. **ServerStateView**: Compact 2/3 size with better space utilization
2. **Y Offset Control**: Vertical orientation matching real-world movement
3. **Better Labels**: Clear instructions for X and Y adjustments

### Next Steps
1. Test the vertical Y slider functionality
2. Monitor Query packets in Manual mode
3. Share server terminal output for debugging 