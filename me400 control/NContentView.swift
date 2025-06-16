import SwiftUI

struct NContentView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    @State private var showingSettings = false
    @State private var hasInitializedFromServer = false
    @State private var competitionMode = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Header
            HeaderView(showingSettings: $showingSettings, competitionMode: $competitionMode)
            
            // Main Content (Control panel on left, Camera on right)
            GeometryReader { geometry in
                HStack(spacing: 0) {
                    // Left side - Control panel
                    ControlPanelView(competitionMode: competitionMode)
                        .frame(width: geometry.size.width * 0.3)
                        .background(Color(.systemGray6))
                    
                    // Right side - Camera and visualization
                    VisualizationView(competitionMode: competitionMode)
                        .frame(width: geometry.size.width * 0.7)
                }
            }
        }
        .sheet(isPresented: $showingSettings) {
            NSettingsView()
                .environmentObject(coordinator)
                .environmentObject(settingsStore)
        }
        .onAppear {
            // Just update configuration, don't connect automatically
            let config = settingsStore.createNetworkConfiguration()
            coordinator.updateConfiguration(config)
        }
        .onReceive(coordinator.$isSynchronized) { synchronized in
            // Initialize values from system state only once when first synchronized
            if synchronized && !hasInitializedFromServer {
                // Small delay to ensure server values are populated
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.8) {
                    settingsStore.initializePIDFromSystemState(coordinator.systemState)
                    settingsStore.initializeTargetOffsetFromSystemState(coordinator.systemState)
                    hasInitializedFromServer = true
                    
                    // Do NOT send offset automatically - wait for user to press Send button
                }
            }
        }
        .onReceive(coordinator.$connectionState) { state in
            // Reset initialization flag when disconnected
            if !state.isConnected {
                hasInitializedFromServer = false
            }
        }
    }
}

// MARK: - Header View
struct HeaderView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @Binding var showingSettings: Bool
    @Binding var competitionMode: Bool
    
    var body: some View {
        VStack(spacing: 0) {
            HStack {
                // Settings Button
                Button(action: { showingSettings = true }) {
                    Image(systemName: "gear")
                }
                .buttonStyle(.bordered)
                .controlSize(.small)
                
                Spacer()
                
                // Connection Status
                ConnectionStatusView()
                
                Spacer()
                
                // System Controls
                SystemControlsView(competitionMode: $competitionMode)
            }
            .frame(height:50)
            .padding(.horizontal)
            
            // Log Message Display - Always visible
            HStack(spacing: 0) {
                Image(systemName: "info.circle")
                    .foregroundColor(.orange)
                    .font(.caption)
                    .padding(.trailing, 4)
                
                // Parse and display log messages with transparency gradient
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 0) {
                        // Use system state's lastLogMessage instead of logMessages array
                        if coordinator.systemState.lastLogMessage.isEmpty {
                            Text("No messages")
                                .font(.system(.headline, design: .monospaced))
                                .foregroundColor(.white)
                                .opacity(0.6)
                                .padding(.trailing, 8)
                        } else {
                            Text(coordinator.systemState.lastLogMessage)
                                .font(.system(.headline, design: .monospaced))
                                .foregroundColor(.white)
                                .opacity(1.0)
                                .padding(.trailing, 8)
                        }
                    }
                }
                .frame(height: 50)
            }
            .padding(.horizontal)
            .padding(.vertical, 4)
            .background(Color.black.opacity(0.8))
        }
    }
}

// MARK: - Connection Status View
struct ConnectionStatusView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    
    var body: some View {
        HStack(spacing: 12) {
            // Connection Indicator
            HStack(spacing: 8) {
                Circle()
                    .fill(connectionColor)
                    .frame(width: 8, height: 8)
                
                VStack(alignment: .leading, spacing: 2) {
                    Text(connectionText)
                        .font(.caption)
                        .foregroundColor(.secondary)
                    
                    if coordinator.connectionState.isConnected && !coordinator.isSynchronized {
                        Text("Synchronizing...")
                            .font(.caption2)
                            .foregroundColor(.orange)
                    } else if coordinator.isSynchronized {
                        HStack(spacing: 8) {
                            Text("Synchronized")
                                .font(.caption2)
                                .foregroundColor(.green)
                            
                            // Latency display with fixed width
                            Text("(\(Int(coordinator.latency * 1000))ms)")
                                .font(.caption2)
                                .foregroundColor(latencyColor)
                                .frame(width: 60, alignment: .leading) // Fixed width
                        }
                    }
                }
            }
            
            // Connect/Disconnect Button
            Button(action: toggleConnection) {
                Label(
                    coordinator.connectionState.isConnected ? "Disconnect" : "Connect",
                    systemImage: coordinator.connectionState.isConnected ? "wifi.slash" : "wifi"
                )
            }
            .buttonStyle(.bordered)
            .tint(coordinator.connectionState.isConnected ? .red : .blue)
            .disabled(isConnecting)
        }
    }
    
    private var connectionColor: Color {
        switch coordinator.connectionState {
        case .connected: return .green
        case .connecting: return .orange
        case .disconnected: return .gray
        case .failed: return .red
        }
    }
    
    private var connectionText: String {
        switch coordinator.connectionState {
        case .connected: return "Connected"
        case .connecting: return "Connecting..."
        case .disconnected: return "Disconnected"
        case .failed(let error): 
            // Show simplified error message
            if error.localizedDescription.contains("Server not available") {
                return "Server not available"
            }
            return error.localizedDescription
        }
    }
    
    private var latencyColor: Color {
        let ms = coordinator.latency * 1000
        if ms < 50 {
            return .green
        } else if ms < 150 {
            return .orange
        } else {
            return .red
        }
    }
    
    private func toggleConnection() {
        if coordinator.connectionState.isConnected {
            coordinator.disconnect()
        } else {
            coordinator.connect()
        }
    }
    
    private var isConnecting: Bool {
        if case .connecting = coordinator.connectionState {
            return true
        }
        return false
    }
}

// MARK: - System Controls View
struct SystemControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    @Binding var competitionMode: Bool
    
    var body: some View {
        HStack(spacing: 16) {
            // Mode Buttons - Better than picker for 3 options
            HStack(spacing: 0) {
                ForEach([DrivingMode.manual, .autoAim, .autonomous], id: \.self) { mode in
                    Button(action: {
                        Task {
                            try? await coordinator.setMode(mode)
                        }
                    }) {
                        Text(mode.displayName)
                            .font(.system(size: 14, weight: .medium))
                            .foregroundColor(coordinator.systemState.drivingMode == mode ? .white : .primary)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 8)
                            .background(
                                coordinator.systemState.drivingMode == mode ? 
                                Color.accentColor : Color.clear
                            )
                    }
                    .buttonStyle(.plain)
                    .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
                    
                    if mode != .autonomous {
                        Divider()
                            .frame(height: 30)
                    }
                }
            }
            .background(Color(.systemGray5))
            .cornerRadius(8)
            .overlay(
                RoundedRectangle(cornerRadius: 8)
                    .stroke(Color(.systemGray3), lineWidth: 1)
            )
            .frame(width: 300)
            
            // Competition Mode Toggle
            Toggle("Competition Mode", isOn: $competitionMode)
                .toggleStyle(SwitchToggleStyle(tint: .orange))
                .frame(width: 150)
            
            // Running Status - More visible
            HStack(spacing: 8) {
                Circle()
                    .fill(coordinator.systemState.isRunning ? Color.green : Color.red)
                    .frame(width: 12, height: 12)
                    .overlay(
                        Circle()
                            .stroke(coordinator.systemState.isRunning ? Color.green.opacity(0.5) : Color.red.opacity(0.5), lineWidth: 2)
                            .frame(width: 16, height: 16)
                    )
                
                Text(coordinator.systemState.isRunning ? "Running" : "Stopped")
                    .font(.system(.body, weight: .medium))
                    .foregroundColor(coordinator.systemState.isRunning ? .green : .red)
            }
            
            // Single Start/Stop Toggle Button - Much wider
            Button(action: {
                Task {
                    if coordinator.systemState.isRunning {
                        try? await coordinator.stop()
                    } else {
                        try? await coordinator.start()
                    }
                }
            }) {
                Label(
                    coordinator.systemState.isRunning ? "Stop" : "Start",
                    systemImage: coordinator.systemState.isRunning ? "stop.fill" : "play.fill"
                )
                .frame(width: 200) // Much wider
            }
            .buttonStyle(.borderedProminent)
            .tint(coordinator.systemState.isRunning ? .red : .green)
            .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
        }
    }
}

// MARK: - Control Panel View
struct ControlPanelView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    let competitionMode: Bool
    
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                // Mode-specific controls
                switch coordinator.systemState.drivingMode {
                case .manual:
                    ManualControlsView()
                case .autoAim:
                    AutoAimControlsView(competitionMode: competitionMode)
                case .autonomous:
                    AutonomousControlsView(competitionMode: competitionMode)
                }
                
                // Common settings
                SystemSettingsView(competitionMode: competitionMode)
            }
            .padding()
        }
    }
}

// MARK: - Visualization View
struct VisualizationView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    let competitionMode: Bool
    
    var body: some View {
        VStack(spacing: 12) {
            // Top section: ServerStateView and PID Settings side by side
            HStack(spacing: 12) {
                // ServerStateView (left)
                ServerStateView()
                    .frame(height: 90)
                    .frame(maxWidth: .infinity)
                
                // PID Settings (right)
//                PIDSettingsCompactView(competitionMode: competitionMode)
            }
            
            // Middle section: Y Offset Control (left) and CameraView (right)
            GeometryReader { geometry in
                HStack(spacing: 12) {
                    // Y Offset Control (left side)
                    YOffsetControlView(competitionMode: competitionMode)
                        .frame(width: 100)
                        .frame(maxHeight: .infinity)
                    
                    // Camera View (right side - takes remaining space)
                    ZStack {
                        NCameraView()
                            .frame(maxWidth: .infinity, maxHeight: .infinity)
                            .background(Color.black)
                            .cornerRadius(10)
                        
                        // Overlay information on camera view
                        VStack {
                            HStack {
                                VStack(alignment: .leading) {
                                    CameraInfoView()
                                    ErrorInfoView()
                                }
                                .padding(.leading, 8)
                                .padding(.top, 8)
                                
                                Spacer()
                                
                                // Tilt Information (top right)
                                VStack(alignment: .trailing, spacing: 4) {
                                    Label("Roll: \(coordinator.systemState.tiltRoll, specifier: "%.1f")°", systemImage: "rotate.left")
                                    Label("Pitch: \(coordinator.systemState.tiltPitch, specifier: "%.1f")°", systemImage: "rotate.right")
                                }
                                .font(.caption)
                                .foregroundColor(.white)
                                .padding(8)
                                .background(Color.black.opacity(0.7))
                                .cornerRadius(8)
                                .padding(.trailing, 8)
                                .padding(.top, 8)
                            }
                            
                            Spacer()
                            
                            // Launch Counter (bottom right)
                            HStack {
                                Spacer()
                                
                                if coordinator.systemState.launchCounter >= 0 {
                                    Label("Launches: \(coordinator.systemState.launchCounter)", systemImage: "flame")
                                        .font(.caption)
                                        .foregroundColor(.white)
                                        .padding(8)
                                        .background(Color.black.opacity(0.7))
                                        .cornerRadius(8)
                                        .padding(.trailing, 8)
                                        .padding(.bottom, 8)
                                }
                            }
                        }
                    }
                }
            }
            
            // Bottom section: X Offset Control
            XOffsetControlView(competitionMode: competitionMode)
                .frame(height: 100)
        }
        .padding()
    }
}

// MARK: - Compact PID Settings View
struct PIDSettingsCompactView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    let competitionMode: Bool
    
    var body: some View {
        VStack(spacing: 0) {
            Text("PID Settings")
                .font(.caption)
                .fontWeight(.medium)
                .padding(.vertical, 4)
            
            Divider()
            
            ScrollView {
                VStack(spacing: 16) {
                    // Show PID controls only in AutoAim or Autonomous mode
                    if coordinator.systemState.drivingMode == .autoAim || coordinator.systemState.drivingMode == .autonomous {
                        PIDControlView(competitionMode: competitionMode)
                            .padding()
                    } else {
                        Text("PID controls available in\nAutoAim or Autonomous mode")
                            .font(.caption)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                            .padding()
                    }
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

// MARK: - Preview
#if DEBUG
#Preview {
    NContentView()
        .environmentObject(ControlCoordinatorFactory.createMock())
        .environmentObject(SettingsStore())
}
#endif 

 
 
