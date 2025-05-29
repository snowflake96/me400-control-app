import SwiftUI

struct NSettingsView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    @Environment(\.dismiss) var dismiss
    
    var body: some View {
        NavigationView {
            Form {
                // Connection Settings
                Section(content: {
                    connectionContent
                }, header: {
                    Text("Connection")
                })
                
                // Target Offset Settings
                Section(content: {
                    targetOffsetContent
                }, header: {
                    Text("Target Offset")
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
                        // Update configuration when closing settings
                        let config = settingsStore.createNetworkConfiguration()
                        coordinator.updateConfiguration(config)
                        dismiss()
                    }
                }
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
        }
        
        HStack {
            Text("Port")
                .frame(width: 60, alignment: .leading)
            TextField("Port number", text: $settingsStore.serverPort)
                .textFieldStyle(.roundedBorder)
                .keyboardType(.numberPad)
        }
    }
    
    @ViewBuilder
    private var targetOffsetContent: some View {
        HStack {
            Text("X Offset")
                .frame(width: 80, alignment: .leading)
            Slider(value: $settingsStore.targetOffsetX, in: -1...1, step: 0.01)
            Text(String(format: "%.2f", settingsStore.targetOffsetX))
                .frame(width: 50)
                .font(.system(.body, design: .monospaced))
        }
        
        HStack {
            Text("Y Offset")
                .frame(width: 80, alignment: .leading)
            Slider(value: $settingsStore.targetOffsetY, in: -1...1, step: 0.01)
            Text(String(format: "%.2f", settingsStore.targetOffsetY))
                .frame(width: 50)
                .font(.system(.body, design: .monospaced))
        }
        
        Button("Reset Offsets") {
            settingsStore.targetOffsetX = 0
            settingsStore.targetOffsetY = 0
        }
    }
    
    @ViewBuilder
    private var pidTuningContent: some View {
        StepSizeRow(label: "Pitch P", value: $settingsStore.pitchPStepSize)
        StepSizeRow(label: "Pitch I", value: $settingsStore.pitchIStepSize)
        StepSizeRow(label: "Pitch Limit", value: $settingsStore.pitchIntegralLimitStepSize)
        StepSizeRow(label: "Yaw P", value: $settingsStore.yawPStepSize)
        StepSizeRow(label: "Yaw I", value: $settingsStore.yawIStepSize)
        StepSizeRow(label: "Yaw Limit", value: $settingsStore.yawIntegralLimitStepSize)
    }
    
    // Helper function with simpler signature
    private func makePresetButton(_ host: String, _ port: String) -> some View {
        Button(action: {
            settingsStore.serverHost = host
            settingsStore.serverPort = port
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
                .frame(width: 100, alignment: .leading)
            
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