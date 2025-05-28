import SwiftUI

struct ServerConnectingView: View {
    @ObservedObject var parameterManager = ParameterManager.shared
    @ObservedObject var serverManager = ServerCommunicationManager.shared
    
    var body: some View {
        VStack(spacing: 20) {
            // Connection Status
            HStack {
                Circle()
                    .fill(parameterManager.serverConnected ? Color.green : Color.red)
                    .frame(width: 12, height: 12)
                Text(parameterManager.serverConnected ? "Connected" : "Disconnected")
                    .foregroundColor(parameterManager.serverConnected ? .green : .red)
            }
            
            if !parameterManager.serverError.isEmpty {
                Text("Error: \(parameterManager.serverError)")
                    .foregroundColor(.red)
                    .font(.caption)
            }
            
            // Server Configuration
            VStack(spacing: 10) {
                StringInputBox(
                    label: "Host:",
                    placeholder: "localhost",
                    value: Binding(
                        get: { parameterManager.serverHost },
                        set: { parameterManager.serverHost = $0 }
                    ),
                    width: 60,
                    isDisabled: parameterManager.serverConnected
                )
                
                StringInputBox(
                    label: "Port:",
                    placeholder: "12345",
                    value: Binding(
                        get: { parameterManager.serverPort },
                        set: { parameterManager.serverPort = $0 }
                    ),
                    width: 60,
                    isDisabled: parameterManager.serverConnected,
                    keyboardType: .numberPad
                )
            }
            .padding(.horizontal)
            
            // Connection Controls
            HStack {
                Button(action: {
                    serverManager.connect()
                }) {
                    Text("Connect")
                        .frame(width: 100)
                }
                .buttonStyle(.bordered)
                .disabled(parameterManager.serverConnected)
                
                Button(action: {
                    serverManager.disconnect()
                }) {
                    Text("Disconnect")
                        .frame(width: 100)
                }
                .buttonStyle(.bordered)
                .disabled(!parameterManager.serverConnected)
            }
        }
        .padding()
    }
}

#Preview {
    ServerConnectingView()
}
