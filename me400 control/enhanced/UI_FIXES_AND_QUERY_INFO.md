# UI Fixes and Query Frequency Information

## Date: May 30, 2025

### 1. Query Frequency Location ✅

**File**: `NControlCoordinator.swift`
**Line**: 286

```swift
// Current setting: 0.2 seconds (5Hz)
queryTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { _ in
```

**Also at Line**: 305
```swift
// Sync timer also uses 0.2 seconds
syncTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { _ in
```

**To Change Query Frequency**:
- Modify the `withTimeInterval` parameter value
- Example: `0.1` for 10Hz, `0.5` for 2Hz, `1.0` for 1Hz
- Note: Both timers should typically use the same interval

### 2. Latency Text Width Fix ✅

**Problem**: The latency text "(XXXms)" was changing width causing UI elements to shift

**Solution**: Added fixed width frame to the latency display
```swift
Text("(\(Int(coordinator.latency * 1000))ms)")
    .font(.caption2)
    .foregroundColor(latencyColor)
    .frame(width: 60, alignment: .leading) // Fixed width
```

**Result**: The "Connected" text and "Disconnect" button no longer move when latency changes

### 3. Keyboard Experience Improvements ✅

**Problem**: When keyboard appeared, the text being edited wasn't visible or focused

**Solutions Implemented**:

#### A. Keyboard Overlay (Already Done)
```swift
.ignoresSafeArea(.keyboard) // Prevents layout changes
```

#### B. Keyboard Toolbar Added
Each TextField now has a toolbar that shows:
- The current value being edited
- A "Done" button to dismiss the keyboard

**Example**:
```swift
.toolbar {
    ToolbarItemGroup(placement: .keyboard) {
        Spacer()
        Text("Cutoff: \(cutoffFrequency) Hz")
            .font(.system(.body, design: .monospaced))
            .foregroundColor(.blue)
        Spacer()
        Button("Done") {
            isEditing = false
        }
    }
}
```

**Benefits**:
- Always see what you're editing even if the TextField is hidden
- Quick way to dismiss keyboard with "Done" button
- Value displayed in monospaced font for clarity

### 4. Testing Instructions

1. **Query Frequency**: If you need to change it, modify line 286 and 305 in NControlCoordinator.swift
2. **Latency Display**: Connect and observe that UI doesn't shift as latency changes
3. **Keyboard**: 
   - Tap any system setting TextField
   - Notice the toolbar above keyboard showing current value
   - Tap "Done" to dismiss keyboard
   - Layout remains stable (no squishing) 