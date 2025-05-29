# Target Offset Initialization Fix

## Date: Dec 2024

### Problem
The target offset sliders were not being initialized with server values, even though the system settings were working correctly.

### Root Cause
The `targetOffsetX` and `targetOffsetY` properties in `SettingsStore` were using `@AppStorage`, which:
1. Persists values locally between app launches
2. Takes precedence over programmatic updates
3. Prevents proper initialization from server values

### Solution
1. **Changed from @AppStorage to @Published**:
   ```swift
   // Before:
   @AppStorage("targetOffsetX") var targetOffsetX: Double = 0.0
   @AppStorage("targetOffsetY") var targetOffsetY: Double = 0.0
   
   // After:
   @Published var targetOffsetX: Double = 0.0
   @Published var targetOffsetY: Double = 0.0
   ```

2. **Added connection state handler**:
   - Resets `hasInitializedFromServer` flag when disconnected
   - Ensures fresh initialization on each connection

3. **Removed automatic offset sending**:
   - Target offsets are only sent when user presses the "Send" button
   - No automatic synchronization to avoid unintended changes

### Benefits
- Target offsets now properly initialize from server values
- No local persistence interfering with server synchronization
- Clean state on each connection
- Consistent behavior with system settings
- User has full control over when offsets are sent to server

### Testing
1. Connect to server
2. Target offset sliders should update to server values after ~0.1s
3. Values are NOT automatically sent back to server
4. Press "Send" button to update server with current slider values
5. Disconnect and reconnect - sliders reinitialize from server values 