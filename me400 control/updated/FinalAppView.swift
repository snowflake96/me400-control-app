import SwiftUI

struct FinalAppView: View {
    private let serverManager = ServerCommunicationManager.shared
    @StateObject private var sceneViewModel = CombinedSceneViewModel()
    @StateObject private var parameterManager = ParameterManager.shared
    @State private var isSettingsPresented = false
    @State private var selectedMode: String = "Manual"
    
    var body: some View {
        ZStack {
            // Background
            Color.white.edgesIgnoringSafeArea(.all)
            
            VStack(spacing: 0) {
                // Top bar with settings button
                HStack {
                    Button(action: {
                        isSettingsPresented = true
                    }) {
                        Image(systemName: "gear")
                            .font(.title)
                            .foregroundColor(.blue)
                            .padding()
                    }
                    
                    Spacer()
                    
                    // Connect/Disconnect Button
                    Button(action: {
                        if parameterManager.serverConnected {
                            serverManager.disconnect()
                        } else {
                            serverManager.connect()
                        }
                    }) {
                        HStack {
                            Image(systemName: parameterManager.serverConnected ? "wifi.slash" : "wifi")
                            Text(parameterManager.serverConnected ? "Disconnect" : "Connect")
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(parameterManager.serverConnected ? Color.red.opacity(0.1) : Color.green.opacity(0.1))
                        .foregroundColor(parameterManager.serverConnected ? .red : .green)
                        .cornerRadius(8)
                    }
                    Spacer()
                    // Start Button
                    Button(action: {
                        serverManager.sendStart()
                    }) {
                        HStack {
                            Image(systemName: "play.fill")
                            Text("START")
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(Color.green.opacity(0.1))
                        .foregroundColor(.green)
                        .cornerRadius(8)
                    }
                    .disabled(!parameterManager.serverConnected)
                    
                    // Stop Button
                    Button(action: {
                        serverManager.sendStop()
                    }) {
                        HStack {
                            Image(systemName: "stop.fill")
                            Text("STOP")
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(Color.red.opacity(0.1))
                        .foregroundColor(.red)
                        .cornerRadius(8)
                    }
                    .disabled(!parameterManager.serverConnected)
                    Spacer()
                }
                .background(Color.gray.opacity(0.1))
                
                // 3D Scene view
                CombinedSceneView(viewModel: sceneViewModel)
                    .padding()
//                    .background(Color.gray.opacity(0.1))
                    .cornerRadius(12)
                
                // Mode Selector
                ModeSelector(selectedMode: $selectedMode)
                    .padding(.vertical, 5)
                
                ScrollView {
                    if selectedMode == "Manual" {
                        ManualControlView()
                    } else if selectedMode == "AutoAim" {
                        AutoAimView()
                    } else if selectedMode == "Autonomous" {
                        AutonomousView()
                    }
                }
                
                Spacer()
            }
            
        }
        
        .sheet(isPresented: $isSettingsPresented) {
            SettingsView()
                .frame(minWidth: 800, minHeight: 800)
                .presentationDetents([.large])
                .presentationDragIndicator(.visible)
        }
    }
}

#Preview {
    FinalAppView()
} 
 
