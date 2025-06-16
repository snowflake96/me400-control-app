import SwiftUI

struct NSettingsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    @Environment(\.dismiss) var dismiss
    @State private var originalHost: String = ""
    @State private var originalPort: String = ""
    
    var body: some View {
        NavigationView {
            Form {
                // Connection Settings
                Section(content: {
                    connectionContent
                }, header: {
                    Text("Connection")
                })
                
                // PID Tuning Step Sizes
                Section(content: {
                    pidTuningContent
                }, header: {
                    Text("PID Tuning Step Sizes")
                })
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        // Only update configuration if server settings changed
                        if settingsStore.serverHost != originalHost || settingsStore.serverPort != originalPort {
                            let config = settingsStore.createNetworkConfiguration()
                            coordinator.updateConfiguration(config)
                        }
                        dismiss()
                    }
                }
            }
            .onAppear {
                // Store original values to detect changes
                originalHost = settingsStore.serverHost
                originalPort = settingsStore.serverPort
            }
        }
    }
    
    // MARK: - Section Content Views
    
    @ViewBuilder
    private var connectionContent: some View {
        // Server Presets
        VStack(alignment: .leading, spacing: 8) {
            Text("Server Presets")
                .font(.caption)
                .foregroundColor(.secondary)
            
            VStack(spacing: 8) {
                HStack(spacing: 8) {
                    makePresetButton("100.121.85.128", "12345")
                    makePresetButton("100.121.85.128", "12346")
                }
                HStack(spacing: 8) {
                    makePresetButton("100.102.243.9", "12345")
                    makePresetButton("100.102.243.9", "12346")
                }
                HStack(spacing: 8) {
                    makePresetButton("localhost", "12345")
                    Spacer()
                }
            }
        }
        .padding(.vertical, 4)
        
        HStack {
            Text("Host")
                .frame(width: 60, alignment: .leading)
            TextField("IP Address or hostname", text: $settingsStore.serverHost)
                .textFieldStyle(.roundedBorder)
                .disabled(coordinator.connectionState.isConnected)
                .foregroundColor(coordinator.connectionState.isConnected ? .secondary : .primary)
        }
        
        HStack {
            Text("Port")
                .frame(width: 60, alignment: .leading)
            TextField("Port number", text: $settingsStore.serverPort)
                .textFieldStyle(.roundedBorder)
                .keyboardType(.numberPad)
                .disabled(coordinator.connectionState.isConnected)
                .foregroundColor(coordinator.connectionState.isConnected ? .secondary : .primary)
        }
        
        if coordinator.connectionState.isConnected {
            HStack {
                Image(systemName: "info.circle")
                    .foregroundColor(.blue)
                Text("Disconnect to change server settings")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
    }
    
    @ViewBuilder
    private var pidTuningContent: some View {
        StepSizeRow(label: "Pitch P", value: $settingsStore.pitchPStepSize)
        StepSizeRow(label: "Pitch I", value: $settingsStore.pitchIStepSize)
        StepSizeRow(label: "Pitch Integ Limit", value: $settingsStore.pitchIntegralLimitStepSize)
        StepSizeRow(label: "Pitch Integ Thres", value: $settingsStore.pitchIntegralThresholdStepSize)
        StepSizeRow(label: "Yaw P", value: $settingsStore.yawPStepSize)
        StepSizeRow(label: "Yaw I", value: $settingsStore.yawIStepSize)
        StepSizeRow(label: "Yaw Integ Limit", value: $settingsStore.yawIntegralLimitStepSize)
        StepSizeRow(label: "Yaw Integ Thres", value: $settingsStore.yawIntegralThresholdStepSize)
    }
    
    // Helper function with simpler signature
    private func makePresetButton(_ host: String, _ port: String) -> some View {
        Button(action: {
            if !coordinator.connectionState.isConnected {
                settingsStore.serverHost = host
                settingsStore.serverPort = port
            }
        }) {
            VStack(spacing: 2) {
                Text(host)
                    .font(.caption)
                    .lineLimit(1)
                Text(":\(port)")
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
            .frame(maxWidth: .infinity)
            .padding(.vertical, 8)
            .padding(.horizontal, 4)
            .background(
                RoundedRectangle(cornerRadius: 6)
                    .fill(Color(.systemGray5))
            )
            .overlay(
                RoundedRectangle(cornerRadius: 6)
                    .stroke(
                        (settingsStore.serverHost == host && settingsStore.serverPort == port) ?
                        Color.accentColor : Color.clear,
                        lineWidth: 2
                    )
            )
        }
        .buttonStyle(.plain)
        .disabled(coordinator.connectionState.isConnected)
        .opacity(coordinator.connectionState.isConnected ? 0.6 : 1.0)
    }
}

// MARK: - Step Size Row
struct StepSizeRow: View {
    let label: String
    @Binding var value: Double
    
    let stepOptions: [Double] = [0.001, 0.01, 0.1, 1.0, 10.0]
    
    var body: some View {
        HStack {
            Text(label)
                .frame(width: 140, alignment: .leading)
            
            Picker("", selection: $value) {
                ForEach(stepOptions, id: \.self) { step in
                    Text(String(format: step < 1 ? "%.3f" : "%.0f", step))
                        .tag(step)
                }
            }
            .pickerStyle(.segmented)
        }
    }
}

// MARK: - Preview
#if DEBUG
#Preview {
    NSettingsView()
        .environmentObject(ControlCoordinatorFactory.createMock())
        .environmentObject(SettingsStore())
}
#endif 