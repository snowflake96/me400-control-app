import SwiftUI

struct MainMenuView: View {
    @StateObject private var tcpManager = TCPClientManager()
    @StateObject private var settingsManager = SettingsManager()
    @State private var showingConnectionSettings = false
    
    var body: some View {
        NavigationView {
            List {
                Section(header: Text("Connection Status")) {
                    HStack {
                        Text("Status:")
                        Spacer()
                        Text(tcpManager.isConnected ? "Connected" : "Disconnected")
                            .foregroundColor(tcpManager.isConnected ? .green : .red)
                    }
                    
                    if tcpManager.isConnected {
                        HStack {
                            Text("Server:")
                            Spacer()
                            Text("\(settingsManager.serverIP):\(settingsManager.serverPort)")
                        }
                        
                        if !tcpManager.serverStatus.isEmpty {
                            HStack {
                                Text("Server Status:")
                                Spacer()
                                Text(tcpManager.serverStatus)
                                    .foregroundColor(.blue)
                            }
                        }
                    }
                    
                    if let error = tcpManager.connectionError {
                        HStack {
                            Text("Error:")
                            Spacer()
                            Text(error)
                                .foregroundColor(.red)
                        }
                    }
                }
                
                Section(header: Text("Controls")) {
                    NavigationLink(destination: ControlView(tcpManager: tcpManager, settingsManager: settingsManager)) {
                        Label("Control Panel", systemImage: "gamecontroller")
                    }
                    .disabled(!tcpManager.isConnected)
                    
                    NavigationLink(destination: SettingsView(tcpManager: tcpManager, settingsManager: settingsManager)) {
                        Label("Connection Settings", systemImage: "network")
                    }
                }
            }
            .navigationTitle("ME400 1등조 컨트롤러")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button(action: {
                        if tcpManager.isConnected {
                            tcpManager.disconnect()
                        } else if let port = UInt16(settingsManager.serverPort) {
                            tcpManager.connect(to: settingsManager.serverIP, port: port)
                        }
                    }) {
                        Text(tcpManager.isConnected ? "Disconnect" : "Connect")
                            .foregroundColor(tcpManager.isConnected ? .red : .green)
                    }
                    .disabled(settingsManager.serverIP.isEmpty || settingsManager.serverPort.isEmpty)
                }
            }
        }
    }
}

#Preview {
    MainMenuView()
} 
