import SwiftUI

// MARK: - ESC Control View (Shared between Manual and AutoAim)
struct NESCControlView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @State private var escValue: Double = 0.0
    @State private var isLaunching: Bool = false
    @State private var launchProgress: Double = 0.0
    
    var body: some View {
        GroupBox("ESC Control") {
            VStack(spacing: 16) {
                HStack {
                    Slider(value: $escValue, in: 0...1)
                        .onChange(of: escValue) { _, newValue in
                            Task {
                                let roundedValue = round(newValue * 100) / 100
                                try? await coordinator.sendESCCommand(roundedValue)
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
                    
                    // Launch button
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
                    .disabled(isLaunching || !coordinator.systemState.isRunning)
                    .opacity(isLaunching ? 0.7 : 1.0)
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

// MARK: - Manual Controls View
struct ManualControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @State private var pitchValue: Double = 0.0
    @State private var yawValue: Double = 0.0
    
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
                                    // Spring back to zero when released
                                    withAnimation(.spring()) {
                                        pitchValue = 0
                                    }
                                    Task {
                                        try? await coordinator.sendServoCommand(pitch: 0, yaw: round(yawValue * 100) / 100)
                                    }
                                }
                            }
                            .onChange(of: pitchValue) { _, newValue in
                                Task {
                                    let roundedPitch = round(newValue * 100) / 100
                                    let roundedYaw = round(yawValue * 100) / 100
                                    try? await coordinator.sendServoCommand(pitch: roundedPitch, yaw: roundedYaw)
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
                                    // Spring back to zero when released
                                    withAnimation(.spring()) {
                                        yawValue = 0
                                    }
                                    Task {
                                        try? await coordinator.sendServoCommand(pitch: round(pitchValue * 100) / 100, yaw: 0)
                                    }
                                }
                            }
                            .onChange(of: yawValue) { _, newValue in
                                Task {
                                    let roundedPitch = round(pitchValue * 100) / 100
                                    let roundedYaw = round(newValue * 100) / 100
                                    try? await coordinator.sendServoCommand(pitch: roundedPitch, yaw: roundedYaw)
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
            NESCControlView()
        }
        .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
    }
}

// MARK: - PID Control View (Shared between AutoAim and Autonomous)
struct PIDControlView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        // PID Parameters
        GroupBox("PID Parameters") {
            VStack(spacing: 16) {
                // Pitch PID
                VStack(spacing: 12) {
                    Text("Pitch PID")
                        .font(.subheadline)
                        .frame(maxWidth: .infinity, alignment: .leading)
                    
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
                    
                    NumericInputRow(
                        label: "I Thres",
                        value: $settingsStore.sharedPitchThreshold,
                        range: 0...1,
                        step: settingsStore.pitchIntegralThresholdStepSize
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
                        
                        Button("Send Thres") {
                            Task {
                                try? await coordinator.setPitchIntegralThreshold(settingsStore.sharedPitchThreshold)
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }
                
                Divider()
                
                // Yaw PID
                VStack(spacing: 12) {
                    Text("Yaw PID")
                        .font(.subheadline)
                        .frame(maxWidth: .infinity, alignment: .leading)
                    
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
                    
                    NumericInputRow(
                        label: "I Thres",
                        value: $settingsStore.sharedYawThreshold,
                        range: 0...1,
                        step: settingsStore.yawIntegralThresholdStepSize
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
                        
                        Button("Send Thres") {
                            Task {
                                try? await coordinator.setYawIntegralThreshold(settingsStore.sharedYawThreshold)
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }
            }
        }
    }
}

// MARK: - Status View (Shared between AutoAim and Autonomous)
struct StatusView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
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
    }
}

// MARK: - Auto Aim Controls View
struct AutoAimControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Auto Aim Control")
                .font(.headline)
            
            // Status
            StatusView()
            
            // PID Controls
            PIDControlView()
            
            // ESC Control
            NESCControlView()
        }
        .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
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
            
            // Status
            StatusView()
            
            // PID Controls
            PIDControlView()
        }
    }
}

// MARK: - System Settings View
struct SystemSettingsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @State private var cutoffFrequency: String = ""
    @State private var maxConsecutiveNans: String = ""
    @State private var launchThresholdEps: String = ""
    @State private var launchThresholdN: String = ""
    @State private var stopThrottle: String = ""
    @State private var defaultSpeed: String = ""
    @State private var motorOffset: String = ""
    
    // Track which field is focused
    enum Field: Hashable {
        case cutoffFrequency
        case maxConsecutiveNans
        case launchThresholdEps
        case launchThresholdN
        case stopThrottle
        case defaultSpeed
        case motorOffset
    }
    
    @FocusState private var focusedField: Field?
    @State private var isSending: Bool = false
    @State private var lastSendTime: Date?
    @State private var hasInitialized: Bool = false
    
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
                        .focused($focusedField, equals: .cutoffFrequency)
                        .placeholder(when: cutoffFrequency.isEmpty) {
                            Text("--").foregroundColor(.gray)
                        }
                        .onChange(of: cutoffFrequency) { _, newValue in
                            print("Cutoff frequency changed to: \(newValue)")
                        }
                        .onSubmit {
                            print("Cutoff frequency set to: \(cutoffFrequency)")
                        }
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
                        .focused($focusedField, equals: .maxConsecutiveNans)
                        .placeholder(when: maxConsecutiveNans.isEmpty) {
                            Text("--").foregroundColor(.gray)
                        }
                        .onChange(of: maxConsecutiveNans) { _, newValue in
                            print("Max consecutive NANs changed to: \(newValue)")
                        }
                        .onSubmit {
                            print("Max consecutive NANs set to: \(maxConsecutiveNans)")
                        }
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
                                .focused($focusedField, equals: .launchThresholdEps)
                                .placeholder(when: launchThresholdEps.isEmpty) {
                                    Text("--").foregroundColor(.gray)
                                }
                                .onChange(of: launchThresholdEps) { _, newValue in
                                    print("Launch threshold epsilon changed to: \(newValue)")
                                }
                                .onSubmit {
                                    print("Launch threshold epsilon set to: \(launchThresholdEps)")
                                }
                        }
                        HStack {
                            Text("N:")
                            TextField("N", text: $launchThresholdN)
                                .textFieldStyle(.roundedBorder)
                                .frame(width: 60)
                                .focused($focusedField, equals: .launchThresholdN)
                                .placeholder(when: launchThresholdN.isEmpty) {
                                    Text("--").foregroundColor(.gray)
                                }
                                .onChange(of: launchThresholdN) { _, newValue in
                                    print("Launch threshold N changed to: \(newValue)")
                                }
                                .onSubmit {
                                    print("Launch threshold N set to: \(launchThresholdN)")
                                }
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
                        .focused($focusedField, equals: .stopThrottle)
                        .placeholder(when: stopThrottle.isEmpty) {
                            Text("--").foregroundColor(.gray)
                        }
                        .onChange(of: stopThrottle) { _, newValue in
                            print("Stop throttle changed to: \(newValue)")
                        }
                        .onSubmit {
                            print("Stop throttle set to: \(stopThrottle)")
                        }
                }
                
                // Default Speed (Double)
                HStack {
                    Text("Default Speed")
                        .frame(width: 140, alignment: .leading)
                    TextField("Value", text: $defaultSpeed)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                        .focused($focusedField, equals: .defaultSpeed)
                        .placeholder(when: defaultSpeed.isEmpty) {
                            Text("--").foregroundColor(.gray)
                        }
                        .onChange(of: defaultSpeed) { _, newValue in
                            print("Default speed changed to: \(newValue)")
                        }
                        .onSubmit {
                            print("Default speed set to: \(defaultSpeed)")
                        }
                }
                
                // Motor Offset (Double)
                HStack {
                    Text("Motor Offset")
                        .frame(width: 140, alignment: .leading)
                    TextField("Value", text: $motorOffset)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                        .focused($focusedField, equals: .motorOffset)
                        .placeholder(when: motorOffset.isEmpty) {
                            Text("--").foregroundColor(.gray)
                        }
                        .onChange(of: motorOffset) { _, newValue in
                            print("Motor offset changed to: \(newValue)")
                        }
                        .onSubmit {
                            print("Motor offset set to: \(motorOffset)")
                        }
                }
                
                // Send Settings Button
                HStack {
                    Button("Send Settings") {
                        sendAllSettings()
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(isSending || !coordinator.connectionState.isConnected || !coordinator.isSynchronized)
                    
                    if isSending {
                        ProgressView()
                            .scaleEffect(0.8)
                    } else if let lastSend = lastSendTime, Date().timeIntervalSince(lastSend) < 2 {
                        Image(systemName: "checkmark.circle.fill")
                            .foregroundColor(.green)
                            .transition(.scale.combined(with: .opacity))
                    }
                }
                .frame(maxWidth: .infinity)
                .padding(.top, 8)
            }
            .frame(maxWidth: .infinity, alignment: .leading)
        }
        .toolbar {
            ToolbarItemGroup(placement: .keyboard) {
                if let field = focusedField {
                    Spacer()
                    Text(toolbarText(for: field))
                        .font(.system(.body, design: .monospaced))
                        .foregroundColor(.blue)
                    Spacer()
                    Button("Done") {
                        focusedField = nil
                    }
                }
            }
        }
        .onReceive(coordinator.$isSynchronized) { synchronized in
            // Only initialize once when first synchronized
            if synchronized && !hasInitialized {
                // Small delay to ensure server values are populated
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.7) {
                    initializeFromSystemState()
                    hasInitialized = true
                }
            }
        }
        .onReceive(coordinator.$connectionState) { state in
            // Reset initialization flag when disconnected
            if !state.isConnected {
                hasInitialized = false
            }
        }
    }
    
    private func toolbarText(for field: Field) -> String {
        switch field {
        case .cutoffFrequency:
            return "Cutoff: \(cutoffFrequency) Hz"
        case .maxConsecutiveNans:
            return "Max NANs: \(maxConsecutiveNans)"
        case .launchThresholdEps:
            return "Epsilon: \(launchThresholdEps)"
        case .launchThresholdN:
            return "N: \(launchThresholdN)"
        case .stopThrottle:
            return "Stop Throttle: \(stopThrottle)"
        case .defaultSpeed:
            return "Speed: \(defaultSpeed)"
        case .motorOffset:
            return "Motor Offset: \(motorOffset)"
        }
    }
    
    private func initializeFromSystemState() {
        // Initialize with actual values from system state after synchronization
        cutoffFrequency = String(format: "%.1f", coordinator.systemState.cutoffFrequency)
        maxConsecutiveNans = String(coordinator.systemState.maxConsecutiveNans)
        launchThresholdEps = String(format: "%.3f", coordinator.systemState.launchThresholdEps)
        launchThresholdN = String(coordinator.systemState.launchThresholdN)
        stopThrottle = String(format: "%.1f", coordinator.systemState.stopThrottle)
        defaultSpeed = String(format: "%.1f", coordinator.systemState.defaultSpeed)
        motorOffset = String(format: "%.1f", coordinator.systemState.motorOffset)
    }
    
    private func sendAllSettings() {
        guard !isSending else { return }
        
        // Debug: Print current values before sending
        print("=== Sending Settings ===")
        print("Cutoff Frequency: \(cutoffFrequency)")
        print("Max Consecutive NANs: \(maxConsecutiveNans)")
        print("Launch Threshold Eps: \(launchThresholdEps)")
        print("Launch Threshold N: \(launchThresholdN)")
        print("Stop Throttle: \(stopThrottle)")
        print("Default Speed: \(defaultSpeed)")
        print("Motor Offset: \(motorOffset)")
        print("=====================")
        
        isSending = true
        focusedField = nil // Dismiss keyboard
        
        Task {
            defer { 
                isSending = false
                lastSendTime = Date()
            }
            
            var successCount = 0
            
            // Send Cutoff Frequency
            if let freq = Double(cutoffFrequency) {
                do {
                    print("Sending cutoff frequency: \(freq)")
                    try await coordinator.setCutoffFrequency(freq)
                    successCount += 1
                    // Small delay between sends
                    try? await Task.sleep(nanoseconds: 50_000_000) // 50ms
                } catch {
                    print("Failed to set cutoff frequency: \(error)")
                }
            } else {
                print("Failed to parse cutoff frequency: '\(cutoffFrequency)'")
            }
            
            // Send Max Consecutive NANs
            if let maxNans = UInt32(maxConsecutiveNans) {
                do {
                    print("Sending max consecutive NANs: \(maxNans)")
                    try await coordinator.setMaxConsecutiveNans(maxNans)
                    successCount += 1
                    try? await Task.sleep(nanoseconds: 50_000_000)
                } catch {
                    print("Failed to set max consecutive NANs: \(error)")
                }
            } else {
                print("Failed to parse max consecutive NANs: '\(maxConsecutiveNans)'")
            }
            
            // Send Launch Threshold
            if let eps = Double(launchThresholdEps), let n = UInt8(launchThresholdN) {
                do {
                    print("Sending launch threshold: eps=\(eps), n=\(n)")
                    try await coordinator.setLaunchThreshold(epsilon: eps, n: n)
                    successCount += 1
                    try? await Task.sleep(nanoseconds: 50_000_000)
                } catch {
                    print("Failed to set launch threshold: \(error)")
                }
            } else {
                print("Failed to parse launch threshold: eps='\(launchThresholdEps)', n='\(launchThresholdN)'")
            }
            
            // Send Stop Throttle
            if let throttle = Double(stopThrottle) {
                do {
                    print("Sending stop throttle: \(throttle)")
                    try await coordinator.setStopThrottle(throttle)
                    successCount += 1
                    try? await Task.sleep(nanoseconds: 50_000_000)
                } catch {
                    print("Failed to set stop throttle: \(error)")
                }
            } else {
                print("Failed to parse stop throttle: '\(stopThrottle)'")
            }
            
            // Send Default Speed
            if let speed = Double(defaultSpeed) {
                do {
                    print("Sending default speed: \(speed)")
                    try await coordinator.setDefaultSpeed(speed)
                    successCount += 1
                    try? await Task.sleep(nanoseconds: 50_000_000)
                } catch {
                    print("Failed to set default speed: \(error)")
                }
            } else {
                print("Failed to parse default speed: '\(defaultSpeed)'")
            }
            
            // Send Motor Offset
            if let offset = Double(motorOffset) {
                do {
                    print("Sending motor offset: \(offset)")
                    try await coordinator.setMotorOffset(offset)
                    successCount += 1
                } catch {
                    print("Failed to set motor offset: \(error)")
                }
            } else {
                print("Failed to parse motor offset: '\(motorOffset)'")
            }
            
            print("Successfully sent \(successCount) settings to server")
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

// MARK: - Extensions
extension View {
    func placeholder<Content: View>(
        when shouldShow: Bool,
        alignment: Alignment = .leading,
        @ViewBuilder placeholder: () -> Content) -> some View {
        
        ZStack(alignment: alignment) {
            placeholder().opacity(shouldShow ? 1 : 0)
            self
        }
    }
} 
