import SwiftUI

// MARK: - Manual Controls View
struct ManualControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @State private var pitchValue: Double = 0.0
    @State private var yawValue: Double = 0.0
    @State private var escValue: Double = 0.0
    @State private var isLaunching: Bool = false
    @State private var launchProgress: Double = 0.0
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Manual Control")
                .font(.headline)
            
            if !coordinator.isSynchronized {
                Text("Waiting for server synchronization...")
                    .font(.caption)
                    .foregroundColor(.orange)
                    .padding()
            }
            
            // Servo Controls
            GroupBox("Servo Control") {
                VStack(spacing: 16) {
                    // Pitch Control
                    VStack(alignment: .leading) {
                        Text("Pitch")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        
                        HStack {
                            Text("-1")
                                .font(.caption2)
                            
                            Slider(value: $pitchValue, in: -1...1, step: 0.01) { editing in
                                if !editing {
                                    pitchValue = 0
                                }
                            }
                            .onChange(of: pitchValue) { _, newValue in
                                Task {
                                    // Round to 2 decimal places before sending
                                    let roundedValue = round(newValue * 100) / 100
                                    try? await coordinator.sendServoCommand(
                                        pitch: roundedValue,
                                        yaw: round(yawValue * 100) / 100
                                    )
                                }
                            }
                            .disabled(!coordinator.systemState.isRunning)
                            
                            Text("1")
                                .font(.caption2)
                        }
                        
                        Text("Value: \(pitchValue, specifier: "%.2f")")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    
                    // Yaw Control
                    VStack(alignment: .leading) {
                        Text("Yaw")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        
                        HStack {
                            Text("-1")
                                .font(.caption2)
                            
                            Slider(value: $yawValue, in: -1...1, step: 0.01) { editing in
                                if !editing {
                                    yawValue = 0
                                }
                            }
                            .onChange(of: yawValue) { _, newValue in
                                Task {
                                    // Round to 2 decimal places before sending
                                    let roundedValue = round(newValue * 100) / 100
                                    try? await coordinator.sendServoCommand(
                                        pitch: round(pitchValue * 100) / 100,
                                        yaw: roundedValue
                                    )
                                }
                            }
                            .disabled(!coordinator.systemState.isRunning)
                            
                            Text("1")
                                .font(.caption2)
                        }
                        
                        Text("Value: \(yawValue, specifier: "%.2f")")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
            }
            
            // ESC Control
            GroupBox("ESC Control") {
                VStack(spacing: 16) {
                    HStack {
                        Slider(value: $escValue, in: 0...1)
                            .onChange(of: escValue) { _, newValue in
                                Task {
                                    try? await coordinator.sendESCCommand(newValue)
                                }
                            }
                            .disabled(!coordinator.systemState.isRunning)
                        
                        Text(String(format: "%.2f", escValue))
                            .frame(width: 50)
                            .font(.system(.body, design: .monospaced))
                    }
                    
                    // ESC Reset and Launch buttons
                    HStack(spacing: 12) {
                        Button("Reset") {
                            escValue = 0
                            Task {
                                try? await coordinator.sendESCCommand(0)
                            }
                        }
                        .buttonStyle(.bordered)
                        .frame(maxWidth: .infinity)
                        .frame(height: 44)
                        .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
                        
                        Button(action: {
                            guard !isLaunching else { return }
                            isLaunching = true
                            launchProgress = 0
                            
                            Task {
                                try? await coordinator.sendTriggerCommand(true)
                                
                                // Animate progress over 1 second
                                withAnimation(.linear(duration: 1.0)) {
                                    launchProgress = 1.0
                                }
                                
                                try? await Task.sleep(nanoseconds: 1_000_000_000)
                                try? await coordinator.sendTriggerCommand(false)
                                
                                isLaunching = false
                                launchProgress = 0
                            }
                        }) {
                            ZStack {
                                // Background progress
                                GeometryReader { geometry in
                                    Rectangle()
                                        .fill(Color.orange.opacity(0.3))
                                        .frame(width: geometry.size.width * launchProgress)
                                        .animation(.linear(duration: 1.0), value: launchProgress)
                                }
                                
                                Label("LAUNCH!", systemImage: "flame.fill")
                                    .frame(maxWidth: .infinity)
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        .tint(.orange)
                        .frame(maxWidth: .infinity, maxHeight: 44)
                        .frame(width: 200) // 3x the width of reset button
                        .disabled(isLaunching || !coordinator.systemState.isRunning)
                        .opacity(isLaunching ? 0.7 : 1.0)
                    }
                }
            }
        }
        .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
        .onReceive(coordinator.$systemState) { state in
            // Reset ESC value when stopped
            if !state.isRunning && escValue != 0 {
                escValue = 0
            }
        }
    }
}

// MARK: - Auto Aim Controls View
struct AutoAimControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    @State private var escValue: Double = 0.0
    @State private var isLaunching: Bool = false
    @State private var launchProgress: Double = 0.0
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Auto Aim Control")
                .font(.headline)
            
            // Pitch PID
            GroupBox("Pitch PID") {
                VStack(spacing: 12) {
                    NumericInputRow(
                        label: "P Gain",
                        value: $settingsStore.sharedPitchP,
                        range: -1000...1000,
                        step: settingsStore.pitchPStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Gain",
                        value: $settingsStore.sharedPitchI,
                        range: -1000...1000,
                        step: settingsStore.pitchIStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Limit",
                        value: $settingsStore.sharedPitchLimit,
                        range: 0...1000,
                        step: settingsStore.pitchIntegralLimitStepSize
                    )
                    
                    HStack {
                        Button("Send PI") {
                            Task {
                                try? await coordinator.tunePitch(p: settingsStore.sharedPitchP, i: settingsStore.sharedPitchI)
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        
                        Button("Send Limit") {
                            Task {
                                try? await coordinator.setPitchIntegralLimit(settingsStore.sharedPitchLimit)
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }
            }
            
            // Yaw PID
            GroupBox("Yaw PID") {
                VStack(spacing: 12) {
                    NumericInputRow(
                        label: "P Gain",
                        value: $settingsStore.sharedYawP,
                        range: -1000...1000,
                        step: settingsStore.yawPStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Gain",
                        value: $settingsStore.sharedYawI,
                        range: -1000...1000,
                        step: settingsStore.yawIStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Limit",
                        value: $settingsStore.sharedYawLimit,
                        range: 0...1000,
                        step: settingsStore.yawIntegralLimitStepSize
                    )
                    
                    HStack {
                        Button("Send PI") {
                            Task {
                                try? await coordinator.tuneYaw(p: settingsStore.sharedYawP, i: settingsStore.sharedYawI)
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        
                        Button("Send Limit") {
                            Task {
                                try? await coordinator.setYawIntegralLimit(settingsStore.sharedYawLimit)
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }
            }
            
            // ESC Control
            GroupBox("ESC Control") {
                VStack(spacing: 16) {
                    HStack {
                        Slider(value: $escValue, in: 0...1)
                            .onChange(of: escValue) { _, newValue in
                                Task {
                                    try? await coordinator.sendESCCommand(newValue)
                                }
                            }
                            .disabled(!coordinator.systemState.isRunning)
                        
                        Text(String(format: "%.2f", escValue))
                            .frame(width: 50)
                            .font(.system(.body, design: .monospaced))
                    }
                    
                    // ESC Reset and Launch buttons
                    HStack(spacing: 12) {
                        Button("Reset") {
                            escValue = 0
                            Task {
                                try? await coordinator.sendESCCommand(0)
                            }
                        }
                        .buttonStyle(.bordered)
                        .frame(maxWidth: .infinity)
                        .frame(height: 44)
                        .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
                        
                        Button(action: {
                            guard !isLaunching else { return }
                            isLaunching = true
                            launchProgress = 0
                            
                            Task {
                                try? await coordinator.sendTriggerCommand(true)
                                
                                // Animate progress over 1 second
                                withAnimation(.linear(duration: 1.0)) {
                                    launchProgress = 1.0
                                }
                                
                                try? await Task.sleep(nanoseconds: 1_000_000_000)
                                try? await coordinator.sendTriggerCommand(false)
                                
                                isLaunching = false
                                launchProgress = 0
                            }
                        }) {
                            ZStack {
                                // Background progress
                                GeometryReader { geometry in
                                    Rectangle()
                                        .fill(Color.orange.opacity(0.3))
                                        .frame(width: geometry.size.width * launchProgress)
                                        .animation(.linear(duration: 1.0), value: launchProgress)
                                }
                                
                                Label("LAUNCH!", systemImage: "flame.fill")
                                    .frame(maxWidth: .infinity)
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        .tint(.orange)
                        .frame(maxWidth: .infinity, maxHeight: 44)
                        .frame(width: 200) // 3x the width of reset button
                        .disabled(isLaunching || !coordinator.systemState.isRunning)
                        .opacity(isLaunching ? 0.7 : 1.0)
                    }
                }
            }
        }
        .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
        .onAppear {
            // Initialize from system state only if synchronized
            if coordinator.isSynchronized {
                // ESC value is tracked locally, not from system state
            }
        }
        .onReceive(coordinator.$systemState) { _ in
            // Check target offset synchronization when in AutoAim mode
            if coordinator.systemState.drivingMode == .autoAim && 
               coordinator.connectionState.isConnected && 
               coordinator.isSynchronized {
                Task {
                    await coordinator.checkAndSyncTargetOffset(
                        currentX: settingsStore.targetOffsetX,
                        currentY: settingsStore.targetOffsetY
                    )
                }
            }
        }
    }
}

// MARK: - Autonomous Controls View
struct AutonomousControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Autonomous Mode")
                .font(.headline)
            
            GroupBox("Status") {
                VStack(alignment: .leading, spacing: 8) {
                    // Target Detection Status
                    if coordinator.systemState.boundingBox != nil {
                        Label("Target Detected", systemImage: "viewfinder")
                            .foregroundColor(.green)
                    } else {
                        Label("No Target Detection", systemImage: "viewfinder")
                            .foregroundColor(.orange)
                    }
                    
                    // BBox Error
                    if let bbox = coordinator.systemState.boundingBox {
                        let bboxErrorX = bbox.centerX - settingsStore.targetOffsetX
                        let bboxErrorY = bbox.centerY - settingsStore.targetOffsetY
                        
                        Text("BBox Error: (\(bboxErrorX, specifier: "%.3f"), \(bboxErrorY, specifier: "%.3f"))")
                            .font(.caption)
                            .foregroundColor(.yellow)
                    } else {
                        Text("BBox Error: no detection")
                            .font(.caption)
                            .foregroundColor(.yellow.opacity(0.6))
                    }
                    
                    // Filtered Error
                    if let filtered = coordinator.systemState.filteredBoundingBox {
                        let filteredErrorX = filtered.centerX - settingsStore.targetOffsetX
                        let filteredErrorY = filtered.centerY - settingsStore.targetOffsetY
                        
                        Text("Filtered Error: (\(filteredErrorX, specifier: "%.3f"), \(filteredErrorY, specifier: "%.3f"))")
                            .font(.caption)
                            .foregroundColor(.green)
                    } else {
                        Text("Filtered Error: no detection")
                            .font(.caption)
                            .foregroundColor(.green.opacity(0.6))
                    }
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }
            
            // Pitch PID
            GroupBox("Pitch PID") {
                VStack(spacing: 12) {
                    NumericInputRow(
                        label: "P Gain",
                        value: $settingsStore.sharedPitchP,
                        range: -1000...1000,
                        step: settingsStore.pitchPStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Gain",
                        value: $settingsStore.sharedPitchI,
                        range: -1000...1000,
                        step: settingsStore.pitchIStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Limit",
                        value: $settingsStore.sharedPitchLimit,
                        range: 0...1000,
                        step: settingsStore.pitchIntegralLimitStepSize
                    )
                    
                    HStack {
                        Button("Send PI") {
                            Task {
                                try? await coordinator.tunePitch(p: settingsStore.sharedPitchP, i: settingsStore.sharedPitchI)
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        
                        Button("Send Limit") {
                            Task {
                                try? await coordinator.setPitchIntegralLimit(settingsStore.sharedPitchLimit)
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }
            }
            
            // Yaw PID
            GroupBox("Yaw PID") {
                VStack(spacing: 12) {
                    NumericInputRow(
                        label: "P Gain",
                        value: $settingsStore.sharedYawP,
                        range: -1000...1000,
                        step: settingsStore.yawPStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Gain",
                        value: $settingsStore.sharedYawI,
                        range: -1000...1000,
                        step: settingsStore.yawIStepSize
                    )
                    
                    NumericInputRow(
                        label: "I Limit",
                        value: $settingsStore.sharedYawLimit,
                        range: 0...1000,
                        step: settingsStore.yawIntegralLimitStepSize
                    )
                    
                    HStack {
                        Button("Send PI") {
                            Task {
                                try? await coordinator.tuneYaw(p: settingsStore.sharedYawP, i: settingsStore.sharedYawI)
                            }
                        }
                        .buttonStyle(.borderedProminent)
                        
                        Button("Send Limit") {
                            Task {
                                try? await coordinator.setYawIntegralLimit(settingsStore.sharedYawLimit)
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }
            }
        }
        .onReceive(coordinator.$systemState) { _ in
            // Check target offset synchronization when in Autonomous mode
            if coordinator.systemState.drivingMode == .autonomous && 
               coordinator.connectionState.isConnected && 
               coordinator.isSynchronized {
                Task {
                    await coordinator.checkAndSyncTargetOffset(
                        currentX: settingsStore.targetOffsetX,
                        currentY: settingsStore.targetOffsetY
                    )
                }
            }
        }
    }
}

// MARK: - System Settings View
struct SystemSettingsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @State private var cutoffFrequency: String = "5.0"
    @State private var maxConsecutiveNans: String = "10"
    @State private var launchThresholdEps: String = "0.005"
    @State private var launchThresholdN: String = "5"
    @State private var stopThrottle: String = "0.0"
    @State private var defaultSpeed: String = "0.0"
    @State private var motorOffset: String = "0.2"
    
    var body: some View {
        GroupBox("System Settings") {
            VStack(spacing: 12) {
                // Cutoff Frequency (Double)
                HStack {
                    Text("Cutoff Frequency")
                        .frame(width: 140, alignment: .leading)
                    TextField("Hz", text: $cutoffFrequency)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                    Text("Hz")
                        .foregroundColor(.secondary)
                }
                
                // Max Consecutive NANs (UInt32)
                HStack {
                    Text("Max Consecutive NANs")
                        .frame(width: 140, alignment: .leading)
                    TextField("Count", text: $maxConsecutiveNans)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                }
                
                // Launch Thresholds
                HStack {
                    Text("Launch Threshold")
                        .frame(width: 140, alignment: .leading)
                    VStack(alignment: .leading, spacing: 4) {
                        HStack {
                            Text("ε:")
                            TextField("Epsilon", text: $launchThresholdEps)
                                .textFieldStyle(.roundedBorder)
                                .frame(width: 60)
                        }
                        HStack {
                            Text("N:")
                            TextField("N", text: $launchThresholdN)
                                .textFieldStyle(.roundedBorder)
                                .frame(width: 60)
                        }
                    }
                }
                
                // Stop Throttle (Double)
                HStack {
                    Text("Stop Throttle")
                        .frame(width: 140, alignment: .leading)
                    TextField("Value", text: $stopThrottle)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                }
                
                // Default Speed (Double)
                HStack {
                    Text("Default Speed")
                        .frame(width: 140, alignment: .leading)
                    TextField("Value", text: $defaultSpeed)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                }
                
                // Motor Offset (Double)
                HStack {
                    Text("Motor Offset")
                        .frame(width: 140, alignment: .leading)
                    TextField("Value", text: $motorOffset)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                }
                
                // Send Settings Button
                Button("Send Settings") {
                    sendAllSettings()
                }
                .buttonStyle(.borderedProminent)
                .frame(maxWidth: .infinity)
                .padding(.top, 8)
            }
            .frame(maxWidth: .infinity, alignment: .leading)
        }
        .onAppear {
            // Initialize from current state if available
            if coordinator.isSynchronized {
                cutoffFrequency = String(format: "%.1f", coordinator.systemState.cutoffFrequency)
                maxConsecutiveNans = String(coordinator.systemState.maxConsecutiveNans)
                stopThrottle = String(format: "%.1f", coordinator.systemState.stopThrottle)
                defaultSpeed = String(format: "%.1f", coordinator.systemState.defaultSpeed)
                motorOffset = String(format: "%.1f", coordinator.systemState.motorOffset)
            }
        }
    }
    
    private func sendAllSettings() {
        Task {
            // Send Cutoff Frequency
            if let freq = Double(cutoffFrequency) {
                try? await coordinator.setCutoffFrequency(freq)
            }
            
            // Send Max Consecutive NANs
            if let maxNans = UInt32(maxConsecutiveNans) {
                try? await coordinator.setMaxConsecutiveNans(maxNans)
            }
            
            // Send Launch Threshold
            if let eps = Double(launchThresholdEps), let n = UInt8(launchThresholdN) {
                try? await coordinator.setLaunchThreshold(epsilon: eps, n: n)
            }
            
            // Send Stop Throttle
            if let throttle = Double(stopThrottle) {
                try? await coordinator.setStopThrottle(throttle)
            }
            
            // Send Default Speed
            if let speed = Double(defaultSpeed) {
                try? await coordinator.setDefaultSpeed(speed)
            }
            
            // Send Motor Offset
            if let offset = Double(motorOffset) {
                try? await coordinator.setMotorOffset(offset)
            }
        }
    }
}

// MARK: - Helper Views
struct NumericInputRow: View {
    let label: String
    @Binding var value: Double
    let range: ClosedRange<Double>
    let step: Double
    
    var body: some View {
        HStack {
            Text(label)
                .frame(width: 60, alignment: .leading)
            
            HStack(spacing: 4) {
                Button("-") {
                    value = max(value - step, range.lowerBound)
                }
                .buttonStyle(.bordered)
                .controlSize(.small)
                
                Text(String(format: "%.3f", value))
                    .frame(width: 80)
                    .font(.system(.body, design: .monospaced))
                
                Button("+") {
                    value = min(value + step, range.upperBound)
                }
                .buttonStyle(.bordered)
                .controlSize(.small)
            }
            
            Spacer()
        }
    }
} 