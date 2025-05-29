import SwiftUI

struct ServerStateView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    
    var body: some View {
        VStack(spacing: 0) {
            Text("Server State")
                .font(.caption)
                .fontWeight(.medium)
                .padding(.vertical, 4)
            
            Divider()
            
            // Horizontal layout with equal spacing
            HStack(spacing: 0) {
                // Mode
                StateItemView(
                    title: "Mode",
                    value: serverModeText,
                    color: serverModeColor
                )
                
                Divider()
                
                // Motor State
                StateItemView(
                    title: "Motor",
                    value: serverMotorStateText,
                    color: serverMotorStateColor
                )
                
                Divider()
                
                // Launch Counter
                StateItemView(
                    title: "Launches",
                    value: launchCounterText,
                    color: .primary
                )
                
                Divider()
                
                // Threshold (N, EPS)
                GroupedStateItemView(
                    title: "Threshold",
                    value1: ("N", thresholdNText),
                    value2: ("EPS", thresholdEpsText)
                )
                
                Divider()
                
                // Target (X, Y)
                GroupedStateItemView(
                    title: "Target",
                    value1: ("X", targetXText),
                    value2: ("Y", targetYText)
                )
                
                Divider()
                
                // Stop Throttle
                StateItemView(
                    title: "Stop Thr",
                    value: stopThrottleText,
                    color: .primary
                )
                
                Divider()
                
                // Motor Offset
                StateItemView(
                    title: "Motor Off",
                    value: motorOffsetText,
                    color: .primary
                )
                
                Divider()
                
                // Default Speed
                StateItemView(
                    title: "Def Speed",
                    value: defaultSpeedText,
                    color: .primary
                )
                
                Divider()
                
                // Cutoff Frequency
                StateItemView(
                    title: "Cutoff",
                    value: cutoffFrequencyText,
                    color: .primary
                )
            }
            .frame(maxHeight: .infinity)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
    
    // MARK: - Computed Properties
    
    private var isConnectedAndSynchronized: Bool {
        coordinator.connectionState.isConnected && coordinator.isSynchronized
    }
    
    private var serverModeText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverMode = coordinator.systemState.serverMode {
            switch serverMode {
            case .manual: return "Manual"
            case .autoAim: return "AutoAim"
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
            return serverIsRunning ? "Running" : "Stopped"
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
    
    private var launchCounterText: String {
        guard isConnectedAndSynchronized else { return "N/A" }
        
        if let serverLaunchCounter = coordinator.systemState.serverLaunchCounter {
            return "\(serverLaunchCounter)"
        } else {
            return "\(coordinator.systemState.launchCounter)"
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
            return String(format: "%.2f", serverValue)
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

// MARK: - Preview
#if DEBUG
#Preview {
    ServerStateView()
        .environmentObject(ControlCoordinatorFactory.createMock())
        .frame(height: 80)
        .padding()
}
#endif 