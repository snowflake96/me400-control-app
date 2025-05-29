import SwiftUI

// MARK: - Motor Settings Components
struct SimpleMotorOffsetRow: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        HStack {
            DoubleInputBox(
                title: "Motor Offset",
                value: Binding(
                    get: { parameterManager.getParameter("MotorOffset", defaultValue: 0.0) },
                    set: { parameterManager.setParameter("MotorOffset", value: $0) }
                ),
                minValue: -1000.0,
                maxValue: 1000.0,
                stepSize: 0.1,
                format: "%.3f",
                allowNegative: true
            )
            
            Spacer()
            
            Button("Send") {
                let value = parameterManager.getParameter("MotorOffset", defaultValue: 0.0)
                _ = ServerCommunicationManager.shared.send(DataPacket.motorOffset(value))
            }
            .buttonStyle(.bordered)
            .disabled(!parameterManager.serverConnected)
        }
        .padding(.horizontal)
    }
}

struct SimpleStopThrottleRow: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        HStack {
            DoubleInputBox(
                title: "Stop Throttle",
                value: Binding(
                    get: { parameterManager.getParameter("StopThrottle", defaultValue: 0.0) },
                    set: { parameterManager.setParameter("StopThrottle", value: $0) }
                ),
                minValue: -1000.0,
                maxValue: 1000.0,
                stepSize: 0.1,
                format: "%.3f",
                allowNegative: true
            )
            
            Spacer()
            
            Button("Send") {
                let value = parameterManager.getParameter("StopThrottle", defaultValue: 0.0)
                _ = ServerCommunicationManager.shared.send(DataPacket.setStopThrottle(value))
            }
            .buttonStyle(.bordered)
            .disabled(!parameterManager.serverConnected)
        }
        .padding(.horizontal)
    }
}

struct SimpleDefaultSpeedRow: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        HStack {
            DoubleInputBox(
                title: "Default Speed",
                value: Binding(
                    get: { parameterManager.getParameter("DefaultSpeed", defaultValue: 1.0) },
                    set: { parameterManager.setParameter("DefaultSpeed", value: $0) }
                ),
                minValue: -1000.0,
                maxValue: 1000.0,
                stepSize: 0.1,
                format: "%.3f",
                allowNegative: true
            )
            
            Spacer()
            
            Button("Send") {
                let value = parameterManager.getParameter("DefaultSpeed", defaultValue: 1.0)
                _ = ServerCommunicationManager.shared.send(DataPacket.setDefaultSpeed(value))
            }
            .buttonStyle(.bordered)
            .disabled(!parameterManager.serverConnected)
        }
        .padding(.horizontal)
    }
}

struct SimpleLaunchThresholdRow: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        VStack(spacing: 10) {
            HStack {
                UInt8InputBox(
                    title: "N (Integer)",
                    value: $parameterManager.launchThresholdN,
                    minValue: 1,
                    maxValue: 255,
                    stepSize: 1
                )
                
                DoubleInputBox(
                    title: "Epsilon",
                    value: Binding(
                        get: { parameterManager.getParameter("LaunchThresholdEpsilon", defaultValue: 0.005) },
                        set: { parameterManager.setParameter("LaunchThresholdEpsilon", value: $0) }
                    ),
                    minValue: 0.0,
                    maxValue: 1000.0,
                    stepSize: 0.001,
                    format: "%.4f",
                    allowNegative: false
                )
                
                Spacer()
                
                Button("Send") {
                    let epsilon = parameterManager.getParameter("LaunchThresholdEpsilon", defaultValue: 0.005)
                    let n = parameterManager.launchThresholdN
                    _ = ServerCommunicationManager.shared.send(DataPacket.setLaunchThreshold(eps: epsilon, n: n))
                }
                .buttonStyle(.bordered)
                .disabled(!parameterManager.serverConnected)
            }
        }
        .padding(.horizontal)
    }
}

// MARK: - Tree Settings Components
struct PipeLocationRow: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        DoubleInputBox(
            title: "Location",
            value: Binding(
                get: { parameterManager.getParameter("PipeLocation", defaultValue: 0.0) },
                set: { parameterManager.setParameter("PipeLocation", value: $0) }
            )
        )
        .padding(.horizontal)
    }
}

struct PipeSection: View {
    let pipeNumber: Int
    @ObservedObject var parameterManager = ParameterManager.shared
    
    private var defaultValues: (xOffset: Double, yLength: Double, zOffset: Double, diameter: Double) {
        switch pipeNumber {
        case 1: return (0.0, 1.0, 0.0, 0.12)
        case 2: return (0.0, 1.0, 0.0, 0.08)
        case 3: return (0.0, 0.8, 0.0, 0.08)
        default: return (0.0, 1.0, 0.0, 0.08)
        }
    }
    
    var body: some View {
        VStack(spacing: 10) {
            DoubleInputBox(
                title: "X Offset",
                value: Binding(
                    get: { parameterManager.getParameter("Pipe\(pipeNumber)XOffset", defaultValue: defaultValues.xOffset) },
                    set: { parameterManager.setParameter("Pipe\(pipeNumber)XOffset", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Y Length",
                value: Binding(
                    get: { parameterManager.getParameter("Pipe\(pipeNumber)YLength", defaultValue: defaultValues.yLength) },
                    set: { parameterManager.setParameter("Pipe\(pipeNumber)YLength", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Z Offset",
                value: Binding(
                    get: { parameterManager.getParameter("Pipe\(pipeNumber)ZOffset", defaultValue: defaultValues.zOffset) },
                    set: { parameterManager.setParameter("Pipe\(pipeNumber)ZOffset", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Diameter",
                value: Binding(
                    get: { parameterManager.getParameter("Pipe\(pipeNumber)Diameter", defaultValue: defaultValues.diameter) },
                    set: { parameterManager.setParameter("Pipe\(pipeNumber)Diameter", value: $0) }
                )
            )
            .padding(.horizontal)
        }
    }
}

struct BellSettingsSection: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        VStack(spacing: 10) {
            DoubleInputBox(
                title: "Neck Length",
                value: Binding(
                    get: { parameterManager.getParameter("BellNeckLength", defaultValue: 0.03) },
                    set: { parameterManager.setParameter("BellNeckLength", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Neck Diameter",
                value: Binding(
                    get: { parameterManager.getParameter("BellNeckDiameter", defaultValue: 0.01) },
                    set: { parameterManager.setParameter("BellNeckDiameter", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Trigger Placement",
                value: Binding(
                    get: { parameterManager.getParameter("TriggerPlacement", defaultValue: 0.05) },
                    set: { parameterManager.setParameter("TriggerPlacement", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Trigger Diameter",
                value: Binding(
                    get: { parameterManager.getParameter("TriggerDiameter", defaultValue: 0.03) },
                    set: { parameterManager.setParameter("TriggerDiameter", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Cone Height",
                value: Binding(
                    get: { parameterManager.getParameter("BellConeHeight", defaultValue: 0.1) },
                    set: { parameterManager.setParameter("BellConeHeight", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Cone Base Diameter",
                value: Binding(
                    get: { parameterManager.getParameter("BellConeBaseDiameter", defaultValue: 0.08) },
                    set: { parameterManager.setParameter("BellConeBaseDiameter", value: $0) }
                )
            )
            .padding(.horizontal)
        }
    }
}

struct AluminiumSettingsSection: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    
    var body: some View {
        VStack(spacing: 10) {
            DoubleInputBox(
                title: "X Offset",
                value: Binding(
                    get: { parameterManager.getParameter("AluminiumXOffset", defaultValue: 0.0) },
                    set: { parameterManager.setParameter("AluminiumXOffset", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Y Offset",
                value: Binding(
                    get: { parameterManager.getParameter("AluminiumYOffset", defaultValue: 0.0) },
                    set: { parameterManager.setParameter("AluminiumYOffset", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Z Length",
                value: Binding(
                    get: { parameterManager.getParameter("AluminiumZLength", defaultValue: 0.25) },
                    set: { parameterManager.setParameter("AluminiumZLength", value: $0) }
                )
            )
            .padding(.horizontal)
            
            DoubleInputBox(
                title: "Diameter",
                value: Binding(
                    get: { parameterManager.getParameter("AluminiumDiameter", defaultValue: 0.03) },
                    set: { parameterManager.setParameter("AluminiumDiameter", value: $0) }
                )
            )
            .padding(.horizontal)
        }
    }
}

struct TreeSettingsView: View {
    @Binding var showingResetAlert: Bool
    let saveAllParameters: () -> Void
    
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                HStack {
                    Text("Tree Settings")
                        .font(.title)
                    
                    Spacer()
                    
                    Button(action: { showingResetAlert = true }) {
                        Label("Reset All", systemImage: "arrow.counterclockwise")
                    }
                    .buttonStyle(.bordered)
                    .tint(.red)
                    
                    Button(action: saveAllParameters) {
                        Label("Save All", systemImage: "square.and.arrow.down")
                    }
                    .buttonStyle(.bordered)
                }
                .padding(.horizontal)
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Pipe Settings")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    PipeLocationRow()
                }
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Pipe 1")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    PipeSection(pipeNumber: 1)
                }
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Pipe 2")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    PipeSection(pipeNumber: 2)
                }
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Pipe 3")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    PipeSection(pipeNumber: 3)
                }
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Bell Settings")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    BellSettingsSection()
                }
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Aluminium Settings")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    AluminiumSettingsSection()
                }
            }
            .padding(.vertical)
        }
    }
}

// MARK: - View Settings Components
struct ViewSettingsView: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    @Binding var showingResetAlert: Bool
    let saveAllParameters: () -> Void
    
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                HStack {
                    Text("View Settings")
                        .font(.title)
                    
                    Spacer()
                    
                    Button(action: { showingResetAlert = true }) {
                        Label("Reset All", systemImage: "arrow.counterclockwise")
                    }
                    .buttonStyle(.bordered)
                    .tint(.red)
                    
                    Button(action: saveAllParameters) {
                        Label("Save All", systemImage: "square.and.arrow.down")
                    }
                    .buttonStyle(.bordered)
                }
                .padding(.horizontal)
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Display Settings")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    HStack {
                        DoubleInputBox(
                            title: "Set Max Consecutive NANs",
                            value: Binding(
                                get: { parameterManager.getParameter("MaxConsecutiveNans", defaultValue: 10.0) },
                                set: { 
                                    parameterManager.setParameter("MaxConsecutiveNans", value: $0)
                                    // Send the new value to the server
                                    _ = ServerCommunicationManager.shared.send(DataPacket.setMaxConsecutiveNans(UInt32($0)))
                                }
                            )
                        )
                        
                        Button("Send") {
                            let value = parameterManager.getParameter("MaxConsecutiveNans", defaultValue: 10.0)
                            _ = ServerCommunicationManager.shared.send(DataPacket.setMaxConsecutiveNans(UInt32(value)))
                        }
                        .buttonStyle(.bordered)
                        .disabled(!parameterManager.serverConnected)
                    }
                    .padding(.horizontal)
                }
            }
            .padding(.vertical)
        }
    }
}

struct MotorSettingsView: View {
    @Binding var showingResetAlert: Bool
    let saveAllParameters: () -> Void
    
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                HStack {
                    Text("Motor Settings")
                        .font(.title)
                    
                    Spacer()
                    
                    Button(action: { showingResetAlert = true }) {
                        Label("Reset All", systemImage: "arrow.counterclockwise")
                    }
                    .buttonStyle(.bordered)
                    .tint(.red)
                    
                    Button(action: saveAllParameters) {
                        Label("Save All", systemImage: "square.and.arrow.down")
                    }
                    .buttonStyle(.bordered)
                }
                .padding(.horizontal)
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Basic Motor Settings")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    SimpleMotorOffsetRow()
                    SimpleStopThrottleRow()
                    SimpleDefaultSpeedRow()
                }
                
                VStack(alignment: .leading, spacing: 15) {
                    Text("Launch Thresholds")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    SimpleLaunchThresholdRow()
                }
            }
            .padding(.vertical)
        }
    }
}

// MARK: - Main Settings View
struct SettingsView: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    @State private var selectedTab = 0
    @State private var showingSaveAlert = false
    @State private var showingResetAlert = false
    
    // List of frequently used server connections
    private let frequentConnections: [(id: String, host: String, port: String)] = [
        ("100.121.85.128:12345", "100.121.85.128", "12345"),
        ("100.121.85.128:12346", "100.121.85.128", "12346"),
        ("localhost:12345", "localhost", "12345"),
        ("100.102.243.9:12345", "100.102.243.9", "12345"),
        ("100.102.243.9:12346", "100.102.243.9", "12346")
    ]
    
    // List of all parameter keys with their default values
    private let parameterDefaults: [(key: String, defaultValue: Double)] = [
        ("MotorOffset", 0.0),
        ("MotorOffsetStepSize", 0.1),
        ("StopThrottle", 0.0),
        ("StopThrottleStepSize", 0.1),
        ("DefaultSpeed", 1.0),
        ("DefaultSpeedStepSize", 0.1),
        ("LaunchThresholdEpsilon", 0.005),
        ("LaunchThresholdEpsilonStepSize", 0.001),
        ("MaxConsecutiveNans", 10.0),
        ("PipeLocation", 0.0),
        ("Pipe1XOffset", 0.0),
        ("Pipe1YLength", 1.0),
        ("Pipe1ZOffset", 0.0),
        ("Pipe1Diameter", 0.12),
        ("Pipe2XOffset", 0.0),
        ("Pipe2YLength", 1.0),
        ("Pipe2ZOffset", 0.0),
        ("Pipe2Diameter", 0.08),
        ("Pipe3XOffset", 0.0),
        ("Pipe3YLength", 0.8),
        ("Pipe3ZOffset", 0.0),
        ("Pipe3Diameter", 0.08),
        ("BellNeckLength", 0.03),
        ("BellNeckDiameter", 0.01),
        ("TriggerPlacement", 0.05),
        ("TriggerDiameter", 0.03),
        ("BellConeHeight", 0.1),
        ("BellConeBaseDiameter", 0.08),
        ("AluminiumXOffset", 0.0),
        ("AluminiumYOffset", 0.0),
        ("AluminiumZLength", 0.25),
        ("AluminiumDiameter", 0.03)
    ]
    
    private func saveAllParameters() {
        for (key, _) in parameterDefaults {
            let value = parameterManager.getParameter(key, defaultValue: 0.0 as Double)
            UserDefaults.standard.set(value, forKey: key)
        }
        showingSaveAlert = true
    }
    
    private func resetAllParameters() {
        for (key, defaultValue) in parameterDefaults {
            parameterManager.setParameter(key, value: defaultValue)
            UserDefaults.standard.set(defaultValue, forKey: key)
        }
        // Reset UInt8 properties separately
        parameterManager.launchThresholdN = 5
    }
    
    var body: some View {
        NavigationView {
            VStack(spacing: 0) {
                // Header
                HStack {
                    Text("Settings")
                        .font(.largeTitle)
                        .fontWeight(.bold)
                        .padding()
                    
                    Spacer()
                    
                    HStack(spacing: 20) {
                        Button(action: {
                            showingResetAlert = true
                        }) {
                            Label("Reset", systemImage: "arrow.counterclockwise")
                                .foregroundColor(.red)
                        }
                        .alert("Reset All Settings?", isPresented: $showingResetAlert) {
                            Button("Cancel", role: .cancel) { }
                            Button("Reset", role: .destructive) {
                                resetAllParameters()
                            }
                        } message: {
                            Text("This will reset all parameters to their default values.")
                        }
                        
                        Button(action: {
                            // Save action (parameters are auto-saved via UserDefaults)
                            showingSaveAlert = true
                        }) {
                            Label("Save", systemImage: "checkmark.circle")
                        }
                        .alert("Settings Saved", isPresented: $showingSaveAlert) {
                            Button("OK", role: .cancel) { }
                        }
                    }
                    .padding(.trailing)
                }
                
                // Tab selection
                Picker("Settings Section", selection: $selectedTab) {
                    Text("Connection").tag(0)
                    Text("Motor").tag(1)
                    Text("Tree").tag(2)
                    Text("View").tag(3)
                    Text("PID Step Sizes").tag(4)
                }
                .pickerStyle(SegmentedPickerStyle())
                .padding()
                
                // Tab content
                ScrollView {
                    switch selectedTab {
                    case 0:
                        ConnectionSettings()
                    case 1:
                        MotorSettingsView(showingResetAlert: $showingResetAlert, saveAllParameters: saveAllParameters)
                    case 2:
                        TreeSettingsView(showingResetAlert: $showingResetAlert, saveAllParameters: saveAllParameters)
                    case 3:
                        ViewSettingsView(showingResetAlert: $showingResetAlert, saveAllParameters: saveAllParameters)
                    case 4:
                        PIDStepSizesSettings()
                    default:
                        ConnectionSettings()
                    }
                }
            }
        }
    }
}

// MARK: - Connection Settings
struct ConnectionSettings: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    
    // List of frequently used server connections
    private let frequentConnections: [(id: String, host: String, port: String)] = [
        ("100.121.85.128:12345", "100.121.85.128", "12345"),
        ("100.121.85.128:12346", "100.121.85.128", "12346"),
        ("localhost:12345", "localhost", "12345"),
        ("100.102.243.9:12345", "100.102.243.9", "12345"),
        ("100.102.243.9:12346", "100.102.243.9", "12346")
    ]
    
    var body: some View {
        VStack(spacing: 20) {
            ServerConnectingView()
            
            // Frequently Used Connections
            VStack(alignment: .leading, spacing: 8) {
                Text("Frequent Connections")
                    .font(.headline)
                    .padding(.horizontal)
                
                ForEach(frequentConnections, id: \.id) { connection in
                    Button(action: {
                        parameterManager.serverHost = connection.host
                        parameterManager.serverPort = connection.port
                    }) {
                        HStack {
                            Text(connection.host)
                                .foregroundColor(.primary)
                            Text(":")
                                .foregroundColor(.gray)
                            Text(connection.port)
                                .foregroundColor(.primary)
                        }
                        .padding(8)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(Color.gray.opacity(0.1))
                        .cornerRadius(6)
                    }
                    .disabled(parameterManager.serverConnected)
                    .padding(.horizontal)
                }
            }
            .padding(.vertical)
        }
    }
}

// MARK: - PID Step Sizes Settings
struct PIDStepSizesSettings: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    
    let stepOptions: [Double] = [0.001, 0.01, 0.1, 1.0, 10.0]
    
    var body: some View {
        VStack(spacing: 20) {
            Text("PID Tuning Step Sizes")
                .font(.title2)
                .fontWeight(.bold)
                .padding(.top)
            
            VStack(alignment: .leading, spacing: 15) {
                Text("Pitch Control Step Sizes")
                    .font(.headline)
                    .padding(.horizontal)
                
                StepSizePickerRow(
                    label: "Pitch P",
                    value: $parameterManager.pitchPStepSize,
                    options: stepOptions
                )
                
                StepSizePickerRow(
                    label: "Pitch I",
                    value: $parameterManager.pitchIStepSize,
                    options: stepOptions
                )
                
                StepSizePickerRow(
                    label: "Pitch Integ Limit",
                    value: $parameterManager.pitchIntegralLimitStepSize,
                    options: stepOptions
                )
                
                StepSizePickerRow(
                    label: "Pitch Integ Thres",
                    value: $parameterManager.pitchIntegralThresholdStepSize,
                    options: stepOptions
                )
                
                Divider()
                    .padding(.vertical)
                
                Text("Yaw Control Step Sizes")
                    .font(.headline)
                    .padding(.horizontal)
                
                StepSizePickerRow(
                    label: "Yaw P",
                    value: $parameterManager.yawPStepSize,
                    options: stepOptions
                )
                
                StepSizePickerRow(
                    label: "Yaw I",
                    value: $parameterManager.yawIStepSize,
                    options: stepOptions
                )
                
                StepSizePickerRow(
                    label: "Yaw Integ Limit",
                    value: $parameterManager.yawIntegralLimitStepSize,
                    options: stepOptions
                )
                
                StepSizePickerRow(
                    label: "Yaw Integ Thres",
                    value: $parameterManager.yawIntegralThresholdStepSize,
                    options: stepOptions
                )
            }
            .padding(.vertical)
            
            Spacer()
        }
    }
}

// MARK: - Step Size Picker Row
struct StepSizePickerRow: View {
    let label: String
    @Binding var value: Double
    let options: [Double]
    
    var body: some View {
        HStack {
            Text(label)
                .frame(width: 140, alignment: .leading)
                .padding(.leading)
            
            Picker("", selection: $value) {
                ForEach(options, id: \.self) { step in
                    Text(String(format: step < 1 ? "%.3f" : "%.0f", step))
                        .tag(step)
                }
            }
            .pickerStyle(.segmented)
            .padding(.trailing)
        }
    }
}

#Preview {
    SettingsView()
} 
 
