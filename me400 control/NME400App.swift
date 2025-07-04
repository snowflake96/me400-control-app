import SwiftUI

@main
@MainActor
struct NME400App: App {
    @StateObject private var coordinator = ControlCoordinatorFactory.create()
    @StateObject private var settingsStore = SettingsStore()
    
    init() {
        // This will be called after StateObject initialization
    }
    
    var body: some Scene {
        WindowGroup {
            NContentView()
                .environmentObject(coordinator)
                .environmentObject(settingsStore)
                .ignoresSafeArea(.keyboard)
                .onAppear {
                    // Set the settings store reference in coordinator
                    coordinator.setSettingsStore(settingsStore)
                }
        }
        #if os(macOS)
        .windowStyle(.hiddenTitleBar)
        .windowToolbarStyle(.unified)
        #endif
    }
}

// MARK: - Settings Store
final class SettingsStore: ObservableObject {
    @AppStorage("serverHost") var serverHost: String = "100.102.243.9"
    @AppStorage("serverPort") var serverPort: String = "12345"
    
    // PID Settings
    @AppStorage("pitchPStepSize") var pitchPStepSize: Double = 1.0
    @AppStorage("pitchIStepSize") var pitchIStepSize: Double = 0.1
    @AppStorage("pitchIntegralLimitStepSize") var pitchIntegralLimitStepSize: Double = 0.1
    @AppStorage("pitchIntegralThresholdStepSize") var pitchIntegralThresholdStepSize: Double = 0.1
    
    @AppStorage("yawPStepSize") var yawPStepSize: Double = 1.0
    @AppStorage("yawIStepSize") var yawIStepSize: Double = 0.1
    @AppStorage("yawIntegralLimitStepSize") var yawIntegralLimitStepSize: Double = 0.1
    @AppStorage("yawIntegralThresholdStepSize") var yawIntegralThresholdStepSize: Double = 0.1
    
    // Shared PID Values (between AutoAim and Autonomous)
    @Published var sharedPitchP: Double = 10.0
    @Published var sharedPitchI: Double = 0.0
    @Published var sharedPitchLimit: Double = 1.0
    @Published var sharedPitchThreshold: Double = 0.025
    @Published var sharedYawP: Double = 10.0
    @Published var sharedYawI: Double = 0.0
    @Published var sharedYawLimit: Double = 1.0
    @Published var sharedYawThreshold: Double = 0.025
    
    // Scene Settings
    @AppStorage("pipeLocation") var pipeLocation: Double = 1.5
    @AppStorage("cameraOffsetX") var cameraOffsetX: Double = 0.0
    @AppStorage("cameraOffsetY") var cameraOffsetY: Double = 0.0
    @AppStorage("cameraOffsetZ") var cameraOffsetZ: Double = 0.0
    
    // Target Offset Settings (for crosshair positioning)
    @Published var targetOffsetX: Double = 0.0
    @Published var targetOffsetY: Double = 0.0
    
    // Initialize PID values from system state
    func initializePIDFromSystemState(_ state: SystemState) {
        sharedPitchP = state.pitchP
        sharedPitchI = state.pitchI
        sharedPitchLimit = state.pitchIntegralLimit
        sharedYawP = state.yawP
        sharedYawI = state.yawI
        sharedYawLimit = state.yawIntegralLimit
    }
    
    // Initialize target offset values from system state
    func initializeTargetOffsetFromSystemState(_ state: SystemState) {
        targetOffsetX = state.targetX
        targetOffsetY = state.targetY
    }
    
    func createNetworkConfiguration() -> NetworkConfiguration {
        guard let port = UInt16(serverPort) else {
            return NetworkConfiguration(
                host: serverHost,
                port: 12345,
                connectionTimeout: 1.0,
                reconnectDelay: 0.5,
                maxReconnectAttempts: 20,
                keepaliveInterval: 5
            )
        }
        
        return NetworkConfiguration(
            host: serverHost,
            port: port,
            connectionTimeout: 1.0,
            reconnectDelay: 0.5,
            maxReconnectAttempts: 20,
            keepaliveInterval: 5
        )
    }
} 