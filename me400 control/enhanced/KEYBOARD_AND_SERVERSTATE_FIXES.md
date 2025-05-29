# Keyboard Overlay and ServerStateView Update Fixes

## Date: May 30, 2025

### 1. iPad Keyboard Overlay Fix ✅

**Problem**: When keyboard appeared on iPad, the visualization view would resize to fit the reduced screen space.

**Solution**: Added `.ignoresSafeArea(.keyboard)` to the main ContentView in NME400App.swift

```swift
WindowGroup {
    NContentView()
        .environmentObject(coordinator)
        .environmentObject(settingsStore)
        .ignoresSafeArea(.keyboard) // Prevents layout changes
}
```

**Result**: Keyboard now appears as an overlay without affecting the app layout.

### 2. ServerStateView Update Issue ✅

**Problem**: When system settings were changed and sent to server, the ServerStateView didn't reflect the updated values, even though the server received them correctly.

**Root Cause Analysis**:
1. After initial synchronization, the SystemStateManager was only updating `server*` variables (e.g., `serverCutoffFrequency`)
2. The ServerStateView was reading from the main state variables (e.g., `systemState.cutoffFrequency`)
3. This created a disconnect where the UI showed stale client values instead of current server values

**Solution Applied**:

#### A. Updated ServerStateView to use server-reported values:
```swift
// Before:
value: String(format: "%.1f Hz", coordinator.systemState.cutoffFrequency)

// After:
value: String(format: "%.1f Hz", coordinator.systemState.serverCutoffFrequency ?? coordinator.systemState.cutoffFrequency)
```

#### B. Fixed SystemStateManager to update both sets of values:
- Now updates both main state AND server state values from CurrentState packets
- Ensures consistency between what the client uses and what's displayed

### 3. How CurrentState Packet Processing Works

```
Initial Sync:
├── Receives first CurrentState packet
├── Updates all client state values
├── Updates all server state values
└── Marks as synchronized

After Sync:
├── Receives CurrentState packets (every 0.2s from Query)
├── Updates all client state values (to stay in sync)
├── Updates all server state values (for display)
└── ServerStateView shows server values with client fallback

```

### 4. Benefits of This Approach

1. **Accurate Display**: ServerStateView always shows what the server actually has
2. **Consistency**: Client state stays synchronized with server
3. **Fallback Support**: During initial connection, shows client values until server responds
4. **Real-time Updates**: Changes are reflected immediately when server updates values

### 5. Testing the Fix

To verify the fix works:
1. Connect to server
2. Change a system setting (e.g., Cutoff Frequency)
3. Click "Send Settings"
4. Observe ServerStateView updates to show new value
5. The update happens when the next CurrentState packet arrives (within 0.2s) 