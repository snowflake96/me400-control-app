import SwiftUI

struct NContentView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    @State private var showingSettings = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Header
            HeaderView(showingSettings: $showingSettings)
            
            // Main Content
            GeometryReader { geometry in
                HStack(spacing: 0) {
                    // Control Panel
                    ControlPanelView()
                        .frame(width: geometry.size.width * 0.3)
                        .background(Color(.systemGray6))
                    
                    // Visualization
                    VisualizationView()
                        .frame(maxWidth: .infinity)
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
            // Initialize PID values from system state when synchronized
            if synchronized {
                settingsStore.initializePIDFromSystemState(coordinator.systemState)
            }
        }
    }
}

// MARK: - Header View
struct HeaderView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @Binding var showingSettings: Bool
    
    var body: some View {
        VStack(spacing: 0) {
            HStack {
                // Settings Button
                Button(action: { showingSettings = true }) {
                    Label("Settings", systemImage: "gear")
                }
                .buttonStyle(.bordered)
                
                Spacer()
                
                // Connection Status
                ConnectionStatusView()
                
                Spacer()
                
                // System Controls
                SystemControlsView()
            }
            .padding(.horizontal)
            .padding(.vertical, 12)
            
            // Log Message Display
            if !coordinator.systemState.lastLogMessage.isEmpty {
                HStack {
                    Image(systemName: "info.circle")
                        .foregroundColor(.orange)
                    Text(coordinator.systemState.lastLogMessage)
                        .lineLimit(1)
                        .font(.caption)
                    Spacer()
                }
                .padding(.horizontal)
                .padding(.vertical, 6)
                .background(Color.orange.opacity(0.1))
            }
        }
        .background(Color(.systemBackground))
        .overlay(
            Rectangle()
                .fill(Color(.separator))
                .frame(height: 1),
            alignment: .bottom
        )
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
                        Text("Synchronized")
                            .font(.caption2)
                            .foregroundColor(.green)
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
        case .failed(let error): return error.localizedDescription
        }
    }
    
    private func toggleConnection() {
        if coordinator.connectionState.isConnected {
            coordinator.disconnect()
        } else {
            coordinator.connect()
        }
    }
}

// MARK: - System Controls View
struct SystemControlsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        HStack(spacing: 16) {
            // Mode Buttons - Better than picker for 3 options
            HStack(spacing: 0) {
                ForEach([DrivingMode.manual, .autoAim, .autonomous], id: \.self) { mode in
                    Button(action: {
                        Task {
                            try? await coordinator.setMode(mode)
                            
                            // Mode-specific actions
                            switch mode {
                            case .manual:
                                // Send ESC zero when switching to manual
                                try? await coordinator.sendESCCommand(0)
                                
                            case .autoAim, .autonomous:
                                // Send ESC zero first
                                try? await coordinator.sendESCCommand(0)
                                
                                // Send PID values and limits
                                try? await coordinator.tunePitch(p: settingsStore.sharedPitchP, i: settingsStore.sharedPitchI)
                                try? await coordinator.tuneYaw(p: settingsStore.sharedYawP, i: settingsStore.sharedYawI)
                                try? await coordinator.setPitchIntegralLimit(settingsStore.sharedPitchLimit)
                                try? await coordinator.setYawIntegralLimit(settingsStore.sharedYawLimit)
                                
                                // Send target offset
                                try? await coordinator.setOffset(
                                    x: settingsStore.targetOffsetX,
                                    y: settingsStore.targetOffsetY,
                                    z: 0
                                )
                            }
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
            .frame(width: 400)
            
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
                        // Send ESC zero first, then stop
                        try? await coordinator.sendESCCommand(0)
                        try? await coordinator.stop()
                    } else {
                        // Send configuration before starting
                        await sendStartConfiguration()
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
    
    private func sendStartConfiguration() async {
        // Send all configuration parameters
        try? await coordinator.setMaxConsecutiveNans(coordinator.systemState.maxConsecutiveNans)
        try? await coordinator.setOffset(
            x: settingsStore.targetOffsetX,
            y: settingsStore.targetOffsetY,
            z: 0
        )
        try? await coordinator.setStopThrottle(coordinator.systemState.stopThrottle)
        try? await coordinator.setMotorOffset(coordinator.systemState.motorOffset)
        try? await coordinator.setDefaultSpeed(coordinator.systemState.defaultSpeed)
        try? await coordinator.setCutoffFrequency(coordinator.systemState.cutoffFrequency)
        
        // Send PID parameters if in AutoAim or Autonomous mode
        if coordinator.systemState.drivingMode == .autoAim || coordinator.systemState.drivingMode == .autonomous {
            try? await coordinator.tunePitch(p: settingsStore.sharedPitchP, i: settingsStore.sharedPitchI)
            try? await coordinator.tuneYaw(p: settingsStore.sharedYawP, i: settingsStore.sharedYawI)
            try? await coordinator.setPitchIntegralLimit(settingsStore.sharedPitchLimit)
            try? await coordinator.setYawIntegralLimit(settingsStore.sharedYawLimit)
        }
    }
}

// MARK: - Control Panel View
struct ControlPanelView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                // Mode-specific controls
                switch coordinator.systemState.drivingMode {
                case .manual:
                    ManualControlsView()
                case .autoAim:
                    AutoAimControlsView()
                case .autonomous:
                    AutonomousControlsView()
                }
                
                // Common settings
                SystemSettingsView()
            }
            .padding()
        }
    }
}

// MARK: - Visualization View
struct VisualizationView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    
    var body: some View {
        VStack(spacing: 0) {
            // Camera View with overlays
            ZStack {
                NCameraView()
                
                // Overlay information on camera view
                VStack {
                    HStack {
                        Spacer()
                        
                        // Tilt Information (top right)
                        if coordinator.connectionState.isConnected {
                            VStack(alignment: .trailing, spacing: 4) {
                                Label("Roll: \(coordinator.systemState.tiltRoll, specifier: "%.1f")°", systemImage: "rotate.left")
                                Label("Pitch: \(coordinator.systemState.tiltPitch, specifier: "%.1f")°", systemImage: "rotate.right")
                            }
                            .font(.caption)
                            .foregroundColor(.white)
                            .padding(8)
                            .background(Color.black.opacity(0.7))
                            .cornerRadius(8)
                        }
                        
                        // Launch Counter
                        if coordinator.systemState.launchCounter > 0 {
                            Label("Launches: \(coordinator.systemState.launchCounter)", systemImage: "flame")
                                .font(.caption)
                                .foregroundColor(.white)
                                .padding(8)
                                .background(Color.black.opacity(0.7))
                                .cornerRadius(8)
                        }
                    }
                    .padding()
                    
                    Spacer()
                }
            }
            
            // Offset Control below camera view
            OffsetControlView()
                .frame(maxHeight: 200)
        }
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

