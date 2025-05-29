# Debugging System Settings Issue

## Problem
TextFields are showing user input but sending default values when "Send Settings" is pressed.

## Debug Steps Added

1. **onSubmit handlers**: Added to each TextField to print when user finishes editing
2. **Debug prints in sendAllSettings()**: Shows all values before parsing and sending
3. **Parse failure logging**: Shows if any value fails to parse

## What to Check

Run the app and:

1. Connect to server and wait for synchronization
2. Click on a TextField (e.g., Cutoff Frequency)
3. Change the value (e.g., from "10.0" to "20.0")
4. Press Enter or tap outside to commit the change
5. Click "Send Settings" button
6. Check the console output

## Expected Debug Output
```
Cutoff frequency set to: 20.0
=== Sending Settings ===
Cutoff Frequency: 20.0
Max Consecutive NANs: 10
...
=====================
Sending cutoff frequency: 20.0
```

## Possible Issues

1. **Decimal Separator**: If your locale uses comma (,) instead of dot (.), parsing might fail
2. **Whitespace**: Extra spaces in the text field can cause parsing to fail
3. **State Not Updating**: TextField might not be properly bound to the @State variable

## Quick Fix to Try

If the issue persists, try this workaround:

```swift
TextField("Hz", text: $cutoffFrequency)
    .onChange(of: cutoffFrequency) { _, newValue in
        // Force state update
        cutoffFrequency = newValue
    }
```

## Alternative Solution

Use a numeric keyboard and formatter:

```swift
TextField("Hz", value: $cutoffFrequencyDouble, format: .number)
    .keyboardType(.decimalPad)
```

Where `cutoffFrequencyDouble` is a `@State var cutoffFrequencyDouble: Double = 5.0` 