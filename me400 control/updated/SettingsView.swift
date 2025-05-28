import SwiftUI

struct SettingsView: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    @State private var selectedTab = 0
    @State private var showingSaveAlert = false
    @State private var showingResetAlert = false
    
    // List of frequently used server connections
    private let frequentConnections: [(id: String, host: String, port: String)] = [
        ("100.121.85.128:12345", "100.121.85.128", "12345"),
        ("100.121.85.128:12346", "100.121.85.128", "12346"),
        ("localhost:12345", "localhost", "12345"),
        ("100.102.243.9:12345", "100.102.243.9", "12345"),
        ("100.102.243.9:12346", "100.102.243.9", "12346")
    ]
    
    // List of all parameter keys with their default values
    private let parameterDefaults: [(key: String, defaultValue: Double)] = [
        ("MotorOffset", 0.0),
        ("StopThrottle", 0.0),
        ("PipeLocation", 0.0),
        ("Pipe1XOffset", 0.0),
        ("Pipe1YLength", 1.0),
        ("Pipe1ZOffset", 0.0),
        ("Pipe1Diameter", 0.12),
        ("Pipe2XOffset", 0.0),
        ("Pipe2YLength", 1.0),
        ("Pipe2ZOffset", 0.0),
        ("Pipe2Diameter", 0.08),
        ("Pipe3XOffset", 0.0),
        ("Pipe3YLength", 0.8),
        ("Pipe3ZOffset", 0.0),
        ("Pipe3Diameter", 0.08),
        ("BellNeckLength", 0.03),
        ("BellNeckDiameter", 0.01),
        ("TriggerPlacement", 0.05),
        ("TriggerDiameter", 0.03),
        ("BellConeHeight", 0.1),
        ("BellConeBaseDiameter", 0.08),
        ("AluminiumXOffset", 0.0),
        ("AluminiumYOffset", 0.0),
        ("AluminiumZLength", 0.25),
        ("AluminiumDiameter", 0.03)
    ]
    
    private func saveAllParameters() {
        for (key, _) in parameterDefaults {
            let value = parameterManager.getParameter(key, defaultValue: 0.0 as Double)
            UserDefaults.standard.set(value, forKey: key)
        }
        showingSaveAlert = true
    }
    
    private func resetAllParameters() {
        for (key, defaultValue) in parameterDefaults {
            parameterManager.setParameter(key, value: defaultValue)
            UserDefaults.standard.set(defaultValue, forKey: key)
        }
    }
    
    var body: some View {
        TabView(selection: $selectedTab) {
            // Server Connection Settings
            VStack(spacing: 20) {
                ServerConnectingView()
                
                // Frequently Used Connections
                VStack(alignment: .leading, spacing: 8) {
                    Text("Frequent Connections")
                        .font(.headline)
                        .padding(.horizontal)
                    
                    ForEach(frequentConnections, id: \.id) { connection in
                        Button(action: {
                            parameterManager.serverHost = connection.host
                            parameterManager.serverPort = connection.port
                        }) {
                            HStack {
                                Text(connection.host)
                                    .foregroundColor(.primary)
                                Text(":")
                                    .foregroundColor(.gray)
                                Text(connection.port)
                                    .foregroundColor(.primary)
                            }
                            .padding(8)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .background(Color.gray.opacity(0.1))
                            .cornerRadius(6)
                        }
                        .disabled(parameterManager.serverConnected)
                        .padding(.horizontal)
                    }
                }
                .padding(.vertical)
            }
            .tabItem {
                Label("Connection", systemImage: "network")
            }
            .tag(0)
            
            // Motor Settings
            ScrollView {
                VStack(alignment: .leading, spacing: 20) {
                    HStack {
                        Text("Motor Settings")
                            .font(.title)
                        
                        Spacer()
                        
                        Button(action: { showingResetAlert = true }) {
                            Label("Reset All", systemImage: "arrow.counterclockwise")
                        }
                        .buttonStyle(.bordered)
                        .tint(.red)
                        
                        Button(action: saveAllParameters) {
                            Label("Save All", systemImage: "square.and.arrow.down")
                        }
                        .buttonStyle(.bordered)
                    }
                    .padding(.horizontal)
                    
                    VStack(alignment: .leading, spacing: 15) {
                        DoubleInputBox(
                            title: "Motor Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("MotorOffset", defaultValue: 0.0) },
                                set: { 
                                    parameterManager.setParameter("MotorOffset", value: $0)
                                    // Send the new value to the server
                                    _ = ServerCommunicationManager.shared.send(DataPacket.motorOffset($0))
                                }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Stop Throttle",
                            value: Binding(
                                get: { parameterManager.getParameter("StopThrottle", defaultValue: 0.0) },
                                set: { 
                                    parameterManager.setParameter("StopThrottle", value: $0)
                                    // Send the new value to the server
                                    _ = ServerCommunicationManager.shared.send(DataPacket.setStopThrottle($0))
                                }
                            )
                        )
                        .padding(.horizontal)
                    }
                }
                .padding(.vertical)
            }
            .tabItem {
                Label("Motor", systemImage: "gearshape.2")
            }
            .tag(1)
            
            // Tree Settings (Combined Pipe, Bell, and Aluminium)
            ScrollView {
                VStack(alignment: .leading, spacing: 20) {
                    HStack {
                        Text("Tree Settings")
                            .font(.title)
                        
                        Spacer()
                        
                        Button(action: { showingResetAlert = true }) {
                            Label("Reset All", systemImage: "arrow.counterclockwise")
                        }
                        .buttonStyle(.bordered)
                        .tint(.red)
                        
                        Button(action: saveAllParameters) {
                            Label("Save All", systemImage: "square.and.arrow.down")
                        }
                        .buttonStyle(.bordered)
                    }
                    .padding(.horizontal)
                    
                    // Pipe Settings
                    VStack(alignment: .leading, spacing: 15) {
                        Text("Pipe Settings")
                            .font(.headline)
                            .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Location",
                            value: Binding(
                                get: { parameterManager.getParameter("PipeLocation", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("PipeLocation", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        Text("Pipe 1")
                            .font(.subheadline)
                            .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "X Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe1XOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("Pipe1XOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Y Length",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe1YLength", defaultValue: 1.0) },
                                set: { parameterManager.setParameter("Pipe1YLength", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Z Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe1ZOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("Pipe1ZOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe1Diameter", defaultValue: 0.12) },
                                set: { parameterManager.setParameter("Pipe1Diameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        Text("Pipe 2")
                            .font(.subheadline)
                            .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "X Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe2XOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("Pipe2XOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Y Length",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe2YLength", defaultValue: 1.0) },
                                set: { parameterManager.setParameter("Pipe2YLength", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Z Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe2ZOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("Pipe2ZOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe2Diameter", defaultValue: 0.08) },
                                set: { parameterManager.setParameter("Pipe2Diameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        Text("Pipe 3")
                            .font(.subheadline)
                            .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "X Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe3XOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("Pipe3XOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Y Length",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe3YLength", defaultValue: 0.8) },
                                set: { parameterManager.setParameter("Pipe3YLength", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Z Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe3ZOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("Pipe3ZOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("Pipe3Diameter", defaultValue: 0.08) },
                                set: { parameterManager.setParameter("Pipe3Diameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                    }
                    
                    Divider()
                        .padding(.vertical)
                    
                    // Bell Settings
                    VStack(alignment: .leading, spacing: 15) {
                        Text("Bell Settings")
                            .font(.headline)
                            .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Neck Length",
                            value: Binding(
                                get: { parameterManager.getParameter("BellNeckLength", defaultValue: 0.03) },
                                set: { parameterManager.setParameter("BellNeckLength", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Neck Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("BellNeckDiameter", defaultValue: 0.01) },
                                set: { parameterManager.setParameter("BellNeckDiameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Trigger Placement",
                            value: Binding(
                                get: { parameterManager.getParameter("TriggerPlacement", defaultValue: 0.05) },
                                set: { parameterManager.setParameter("TriggerPlacement", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Trigger Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("TriggerDiameter", defaultValue: 0.03) },
                                set: { parameterManager.setParameter("TriggerDiameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Cone Height",
                            value: Binding(
                                get: { parameterManager.getParameter("BellConeHeight", defaultValue: 0.1) },
                                set: { parameterManager.setParameter("BellConeHeight", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Cone Base Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("BellConeBaseDiameter", defaultValue: 0.08) },
                                set: { parameterManager.setParameter("BellConeBaseDiameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                    }
                    
                    Divider()
                        .padding(.vertical)
                    
                    // Aluminium Settings
                    VStack(alignment: .leading, spacing: 15) {
                        Text("Aluminium Settings")
                            .font(.headline)
                            .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "X Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("AluminiumXOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("AluminiumXOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Y Offset",
                            value: Binding(
                                get: { parameterManager.getParameter("AluminiumYOffset", defaultValue: 0.0) },
                                set: { parameterManager.setParameter("AluminiumYOffset", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Z Length",
                            value: Binding(
                                get: { parameterManager.getParameter("AluminiumZLength", defaultValue: 0.25) },
                                set: { parameterManager.setParameter("AluminiumZLength", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                        
                        DoubleInputBox(
                            title: "Diameter",
                            value: Binding(
                                get: { parameterManager.getParameter("AluminiumDiameter", defaultValue: 0.03) },
                                set: { parameterManager.setParameter("AluminiumDiameter", value: $0) }
                            )
                        )
                        .padding(.horizontal)
                    }
                }
                .padding(.vertical)
            }
            .tabItem {
                Label("Tree", systemImage: "leaf")
            }
            .tag(2)
        }
        .alert("Settings Saved", isPresented: $showingSaveAlert) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("All parameter values have been saved to app storage.")
        }
        .alert("Reset All Settings", isPresented: $showingResetAlert) {
            Button("Cancel", role: .cancel) { }
            Button("Reset", role: .destructive) {
                resetAllParameters()
            }
        } message: {
            Text("Are you sure you want to reset all settings to their default values? This action cannot be undone.")
        }
    }
}

#Preview {
    SettingsView()
} 
 