import SwiftUI

struct FinalAppView: View {
    private let serverManager = ServerCommunicationManager.shared
    @StateObject private var sceneViewModel = CombinedSceneViewModel()
    @StateObject private var parameterManager = ParameterManager.shared
    @State private var isSettingsPresented = false
    
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
                    
                    // Message Queue Size Indicator
                    if parameterManager.serverConnected {
                        Text("Queue: \(serverManager.messageQueueSize)")
                            .font(.caption)
                            .foregroundColor(.gray)
                            .padding(.horizontal, 12)
                            .padding(.vertical, 8)
                            .background(Color.gray.opacity(0.1))
                            .cornerRadius(8)
                    }
                    
                    // Running State Indicator
                    if parameterManager.serverConnected {
                        HStack {
                            Image(systemName: parameterManager.isRunning ? "circle.fill" : "circle")
                                .foregroundColor(parameterManager.isRunning ? .green : .red)
                            Text(parameterManager.isRunning ? "Running" : "Stopped")
                                .foregroundColor(parameterManager.isRunning ? .green : .red)
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(Color.gray.opacity(0.1))
                        .cornerRadius(8)
                    } else {
                        HStack {
                            Image(systemName: "wifi.slash")
                                .foregroundColor(.gray)
                            Text("Connect to server to start")
                                .foregroundColor(.gray)
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(Color.gray.opacity(0.1))
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
                    .cornerRadius(12)
                
                // Mode Selector
                ModeSelector()
                    .padding(.vertical, 5)
                
                ScrollView {
                    if parameterManager.selectedMode == "Manual" {
                        ManualControlView()
                    } else if parameterManager.selectedMode == "AutoAim" {
                        AutoAimView()
                    } else if parameterManager.selectedMode == "Autonomous" {
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
 
