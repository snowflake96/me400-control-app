import SwiftUI

struct SettingsView: View {
    @ObservedObject var tcpManager: TCPClientManager
    @ObservedObject var settingsManager: SettingsManager
    @Environment(\.presentationMode) var presentationMode
    
    var body: some View {
        Form {
            Section(header: Text("Server Settings")) {
                TextField("Server IP", text: $settingsManager.serverIP)
                    .keyboardType(.numbersAndPunctuation)
                    .autocapitalization(.none)
                
                TextField("Port", text: $settingsManager.serverPort)
                    .keyboardType(.numberPad)
            }
            
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
                }
            }
            
            Section {
                Button(action: {
                    if tcpManager.isConnected {
                        tcpManager.disconnect()
                    } else if let port = UInt16(settingsManager.serverPort) {
                        tcpManager.connect(to: settingsManager.serverIP, port: port)
                    }
                }) {
                    HStack {
                        Spacer()
                        Text(tcpManager.isConnected ? "Disconnect" : "Connect")
                            .foregroundColor(tcpManager.isConnected ? .red : .green)
                        Spacer()
                    }
                }
                .disabled(settingsManager.serverIP.isEmpty || settingsManager.serverPort.isEmpty)
            }
            
            Section {
                Button(action: {
                    settingsManager.resetToDefaults()
                }) {
                    HStack {
                        Spacer()
                        Text("Reset to Defaults")
                            .foregroundColor(.red)
                        Spacer()
                    }
                }
            }
        }
        .navigationTitle("Connection Settings")
    }
} 