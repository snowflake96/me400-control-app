import SwiftUI

struct ServerStateView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    
    var body: some View {
        HStack(spacing: 10) {
            // Server State Section
            VStack(spacing: 0) {
                Text("Server State")
                    .font(.caption)
                    .fontWeight(.medium)
                    .padding(.vertical, 4)
                
                Divider()
                
                // Horizontal layout with equal spacing
                GeometryReader { geometry in
                    HStack(spacing: 0) {
                        // Mode
                        StateItemView(
                            title: "Mode",
                            value: serverModeText,
                            color: serverModeColor
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Motor State
                        StateItemView(
                            title: "Motor",
                            value: serverMotorStateText,
                            color: serverMotorStateColor
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Found Offset
                        StateItemView(
                            title: "Offset",
                            value: foundOffsetText,
                            color: foundOffsetColor
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Interpolation
                        StateItemView(
                            title: "Interp",
                            value: interpolationText,
                            color: .primary
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Max NANs
                        StateItemView(
                            title: "Max NANs",
                            value: maxNansText,
                            color: .primary
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Threshold (N, EPS)
                        GroupedStateItemView(
                            title: "Threshold",
                            value1: ("N", thresholdNText),
                            value2: ("EPS", thresholdEpsText)
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Target (X, Y)
                        GroupedStateItemView(
                            title: "Target",
                            value1: ("X", targetXText),
                            value2: ("Y", targetYText)
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Stop Throttle
                        StateItemView(
                            title: "Stop Thr",
                            value: stopThrottleText,
                            color: .primary
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Motor Offset
                        StateItemView(
                            title: "Motor Off",
                            value: motorOffsetText,
                            color: .primary
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Default Speed
                        StateItemView(
                            title: "Def Speed",
                            value: defaultSpeedText,
                            color: .primary
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                        
                        Divider()
                        
                        // Cutoff Frequency
                        StateItemView(
                            title: "Cutoff",
                            value: cutoffFrequencyText,
                            color: .primary
                        )
                        .frame(width: geometry.size.width * 0.091) // 1/11
                    }
                }
                .frame(maxHeight: .infinity)
            }
            .frame(maxWidth: .infinity, maxHeight: 90)
            .background(Color(.systemGray6))
            .cornerRadius(10)
            
            // PID Settings Section
            VStack(spacing: 0) {
                Text("PID Settings")
                    .font(.caption)
                    .fontWeight(.medium)
                    .padding(.vertical, 4)
                
                Divider()
                
                HStack(spacing: 0) {
                    // Pitch PID
                    PIDStateItemView(
                        title: "Pitch",
                        p: pitchPText,
                        i: pitchIText,
                        iLimit: pitchILimitText,
                        iThreshold: pitchIThresholdText
                    )
                    
                    Divider()
                    
                    // Yaw PID
                    PIDStateItemView(
                        title: "Yaw",
                        p: yawPText,
                        i: yawIText,
                        iLimit: yawILimitText,
                        iThreshold: yawIThresholdText
                    )
                }
                .frame(maxHeight: .infinity)
            }
            .frame(maxWidth: 240, maxHeight: 90)
            .background(Color(.systemGray6))
            .cornerRadius(10)
        }
    }
    
    // MARK: - Computed Properties
    
    private var isConnectedAndSynchronized: Bool {
        coordinator.connectionState.isConnected && coordinator.isSynchronized
    }
    
    private var serverModeText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverMode = coordinator.systemState.serverMode {
            switch serverMode {
            case .manual: return "Man"
            case .autoAim: return "AA"
            case .autonomous: return "Auto"
            }
        } else {
            return "N/A"
        }
    }
    
    private var serverModeColor: Color {
        guard isConnectedAndSynchronized else { return .secondary }
        
        if let serverMode = coordinator.systemState.serverMode {
            switch serverMode {
            case .manual: return .blue
            case .autoAim: return .orange
            case .autonomous: return .green
            }
        } else {
            return .secondary
        }
    }
    
    private var serverMotorStateText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverIsRunning = coordinator.systemState.serverIsRunning {
            return serverIsRunning ? "Run" : "Stop"
        } else {
            return "N/A"
        }
    }
    
    private var serverMotorStateColor: Color {
        guard isConnectedAndSynchronized else { return .secondary }
        
        if let serverIsRunning = coordinator.systemState.serverIsRunning {
            return serverIsRunning ? .green : .red
        } else {
            return .secondary
        }
    }
    
    private var thresholdNText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverLaunchThresholdN {
            return "\(serverValue)"
        } else {
            return "N/A"
        }
    }
    
    private var thresholdEpsText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverLaunchThresholdEps {
            return String(format: "%.3f", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var targetXText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverTargetX {
            return String(format: "%.3f", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var targetYText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverTargetY {
            return String(format: "%.3f", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var stopThrottleText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverStopThrottle {
            return String(format: "%.2f", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var motorOffsetText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverMotorOffset {
            return String(format: "%.3f", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var defaultSpeedText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverDefaultSpeed {
            return String(format: "%.2f", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var cutoffFrequencyText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverValue = coordinator.systemState.serverCutoffFrequency {
            return String(format: "%.1f Hz", serverValue)
        } else {
            return "N/A"
        }
    }
    
    private var foundOffsetText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let foundOffset = coordinator.systemState.serverFoundOffset {
            return foundOffset ? "Found" : "Not Found"
        } else {
            return "N/A"
        }
    }
    
    private var foundOffsetColor: Color {
        guard isConnectedAndSynchronized else { return .secondary }
        
        if let foundOffset = coordinator.systemState.serverFoundOffset {
            return foundOffset ? .green : .orange
        } else {
            return .secondary
        }
    }
    
    private var interpolationText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let useInterp = coordinator.systemState.serverUseInterpolation {
            return useInterp ? "On" : "Off"
        } else {
            return "N/A"
        }
    }
    
    private var maxNansText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let maxNans = coordinator.systemState.serverMaxNans {
            return "\(maxNans)"
        } else {
            return "N/A"
        }
    }
    
    // PID Settings Text Properties
    private var pitchPText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverPitchP {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var pitchIText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverPitchI {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var pitchILimitText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverPitchILimit {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var pitchIThresholdText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverPitchIThreshold {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var yawPText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverYawP {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var yawIText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverYawI {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var yawILimitText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverYawILimit {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
    
    private var yawIThresholdText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        if let value = coordinator.systemState.serverYawIThreshold {
            return String(format: "%.4f", value)
        }
        return "N/A"
    }
}

// Single value item view
struct StateItemView: View {
    let title: String
    let value: String
    let color: Color
    
    var body: some View {
        VStack(spacing: 4) {
            Text(title)
                .font(.system(size: 9))
                .fontWeight(.medium)
                .foregroundColor(.secondary)
            
            Text(value)
                .font(.system(size: 11))
                .fontWeight(.medium)
                .foregroundColor(color)
                .lineLimit(1)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .padding(.horizontal, 4)
    }
}

// Grouped values item view (for values that need to be shown vertically)
struct GroupedStateItemView: View {
    let title: String
    let value1: (label: String, value: String)
    let value2: (label: String, value: String)
    
    var body: some View {
        VStack(spacing: 2) {
            Text(title)
                .font(.system(size: 9))
                .fontWeight(.medium)
                .foregroundColor(.secondary)
            
            VStack(spacing: 1) {
                HStack(spacing: 2) {
                    Text("\(value1.label):")
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                    Text(value1.value)
                        .font(.system(size: 10))
                        .fontWeight(.medium)
                }
                
                HStack(spacing: 2) {
                    Text("\(value2.label):")
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                    Text(value2.value)
                        .font(.system(size: 10))
                        .fontWeight(.medium)
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .padding(.horizontal, 4)
    }
}


struct GroupedStateItemView2: View {
    let title: String
    let value1: (label: String, value: String)
    let value2: (label: String, value: String)
    let value3: (label: String, value: String)
    
    var body: some View {
        VStack(spacing: 2) {
            HStack(spacing: 2) {
                Text("\(value1.label):")
                    .font(.system(size: 9))
                    .foregroundColor(.secondary)
                Text(value1.value)
                    .font(.system(size: 10))
                    .fontWeight(.medium)
            }
            
            HStack(spacing: 2) {
                Text("\(value2.label):")
                    .font(.system(size: 9))
                    .foregroundColor(.secondary)
                Text(value2.value)
                    .font(.system(size: 10))
                    .fontWeight(.medium)
            }
            HStack(spacing: 2) {
                Text("\(value3.label):")
                    .font(.system(size: 9))
                    .foregroundColor(.secondary)
                Text(value3.value)
                    .font(.system(size: 10))
                    .fontWeight(.medium)
            }

        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .padding(.horizontal, 4)
    }
}

// PID Settings item view
struct PIDStateItemView: View {
    let title: String
    let p: String
    let i: String
    let iLimit: String
    let iThreshold: String
    
    var body: some View {
        VStack(spacing: 2) {
            Text(title)
                .font(.system(size: 10))
                .fontWeight(.medium)
                .foregroundColor(.primary)
            
            VStack(alignment: .leading, spacing: 1) {
                HStack(spacing: 4) {
                    Text("P:")
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                    Text(p)
                        .font(.system(size: 9))
                        .fontWeight(.medium)
                    
                    Text("I:")
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                    Text(i)
                        .font(.system(size: 9))
                        .fontWeight(.medium)
                }
                
                HStack(spacing: 4) {
                    Text("IL:")
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                    Text(iLimit)
                        .font(.system(size: 9))
                        .fontWeight(.medium)
                    
                    Text("IT:")
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                    Text(iThreshold)
                        .font(.system(size: 9))
                        .fontWeight(.medium)
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .padding(.horizontal, 8)
    }
}

// MARK: - Preview
#if DEBUG
#Preview {
    ServerStateView()
        .environmentObject(ControlCoordinatorFactory.createMock())
        .frame(height: 80)
        .padding()
}
#endif 
