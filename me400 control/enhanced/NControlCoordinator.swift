import Foundation
import Combine

// MARK: - Control Coordinator
@MainActor
final class ControlCoordinator: ObservableObject {
    // Managers
    private let networkManager: NetworkManagerProtocol
    private let stateManager: SystemStateManagerProtocol
    
    // Published State
    @Published private(set) var systemState = SystemState()
    @Published private(set) var connectionState: ConnectionState = .disconnected
    @Published private(set) var isSynchronized: Bool = false
    
    // Configuration
    private var configuration: NetworkConfiguration
    
    // Subscriptions
    private var cancellables = Set<AnyCancellable>()
    
    // Periodic query timer
    private var queryTimer: Timer?
    
    // Track target offset for comparison
    private var lastSentTargetOffset: (x: Double, y: Double)?
    
    // Weak reference to settings store for target offset sync
    private weak var settingsStore: SettingsStore?
    
    // MARK: - Initialization
    
    init(
        networkManager: NetworkManagerProtocol? = nil,
        stateManager: SystemStateManagerProtocol? = nil,
        configuration: NetworkConfiguration = .default
    ) {
        self.networkManager = networkManager ?? NetworkManagerFactory.create()
        self.stateManager = stateManager ?? SystemStateManagerFactory.create()
        self.configuration = configuration
        
        setupBindings()
    }
    
    // MARK: - Setup
    
    private func setupBindings() {
        // Bind network connection state
        networkManager.connectionState
            .receive(on: DispatchQueue.main)
            .sink { [weak self] state in
                self?.connectionState = state
                
                // Update state manager with connection status
                switch state {
                case .connected:
                    self?.stateManager.updateConnectionState(true, error: nil)
                    // Delay slightly to ensure connection is stable
                    DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                        self?.setupSynchronization()
                    }
                    
                case .failed(let error):
                    self?.stateManager.updateConnectionState(false, error: error.localizedDescription)
                    self?.isSynchronized = false
                    
                case .disconnected:
                    self?.stateManager.updateConnectionState(false, error: nil)
                    self?.isSynchronized = false
                    
                case .connecting:
                    self?.stateManager.updateConnectionState(false, error: nil)
                    self?.isSynchronized = false
                }
            }
            .store(in: &cancellables)
        
        // Process received packets
        networkManager.receivedPackets
            .receive(on: DispatchQueue.main)
            .sink { [weak self] packet in
                self?.stateManager.processPacket(packet)
            }
            .store(in: &cancellables)
        
        // Bind system state
        stateManager.state
            .receive(on: DispatchQueue.main)
            .assign(to: &$systemState)
        
        // Monitor driving mode changes to manage query timer
        $systemState
            .map { $0.drivingMode }
            .removeDuplicates()
            .dropFirst() // Skip initial value
            .sink { [weak self] mode in
                guard let self = self,
                      self.connectionState.isConnected,
                      self.isSynchronized else { return }
                
                // Manage query timer based on mode changes
                switch mode {
                case .manual:
                    self.stopQueryTimer()
                case .autoAim, .autonomous:
                    self.startQueryTimer()
                }
            }
            .store(in: &cancellables)
    }
    
    // MARK: - Synchronization
    
    private func setupSynchronization() {
        // Reset synchronization state
        isSynchronized = false
        
        // Set up callback for when synchronization is complete
        stateManager.setSynchronizationCallback { [weak self] in
            print("Client synchronized with server")
            self?.isSynchronized = true
            
            // Start query timer if already in AutoAim or Autonomous mode
            if let mode = self?.systemState.drivingMode,
               mode == .autoAim || mode == .autonomous {
                Task { @MainActor in
                    self?.startQueryTimer()
                }
            }
        }
        
        // Send initial query to request CurrentState
        Task {
            do {
                try await sendQuery()
                print("Query packet sent, waiting for CurrentState response...")
            } catch {
                print("Failed to send query packet: \(error)")
                // Retry after a delay
                DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
                    self.setupSynchronization()
                }
            }
        }
    }
    
    // MARK: - Connection Management
    
    func connect() {
        networkManager.connect(configuration: configuration)
    }
    
    func disconnect() {
        networkManager.disconnect()
        stateManager.reset()
        isSynchronized = false
        Task { @MainActor in
            stopQueryTimer() // Stop periodic queries
        }
    }
    
    func updateConfiguration(_ config: NetworkConfiguration) {
        self.configuration = config
        
        // Reconnect if currently connected
        if connectionState.isConnected {
            disconnect()
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                self.connect()
            }
        }
    }
    
    // MARK: - Control Commands
    
    func sendServoCommand(pitch: Double, yaw: Double) async throws {
        let packet = PacketFactory.servoCommand(x: pitch, y: yaw, z: 0)
        try await networkManager.send(packet)
    }
    
    func sendTriggerCommand(_ trigger: Bool) async throws {
        let packet = PacketFactory.triggerCommand(trigger)
        try await networkManager.send(packet)
    }
    
    func sendESCCommand(_ value: Double) async throws {
        let packet = PacketFactory.escCommand(value)
        try await networkManager.send(packet)
    }
    
    // MARK: - System Control
    
    func start() async throws {
        let packet = PacketFactory.start()
        try await networkManager.send(packet)
        
        // Update the state through the state manager
        stateManager.updateRunningState(true)
    }
    
    func stop() async throws {
        let packet = PacketFactory.stop()
        try await networkManager.send(packet)
        
        // Update the state through the state manager
        stateManager.updateRunningState(false)
    }
    
    func setMode(_ mode: DrivingMode) async throws {
        // Update client state immediately for responsive UI
        stateManager.updateDrivingMode(mode)
        
        // Send mode change to server
        let packet = PacketFactory.setMode(mode)
        try await networkManager.send(packet)
        
        // The periodic query will detect if server didn't update and resend if needed
    }
    
    // Internal method to send mode without updating local state
    private func sendModeToServer(_ mode: DrivingMode) async throws {
        let packet = PacketFactory.setMode(mode)
        try await networkManager.send(packet)
    }
    
    // MARK: - PID Tuning
    
    func tunePitch(p: Double, i: Double) async throws {
        let packet = PacketFactory.tunePitch(p: p, i: i)
        try await networkManager.send(packet)
    }
    
    func tuneYaw(p: Double, i: Double) async throws {
        let packet = PacketFactory.tuneYaw(p: p, i: i)
        try await networkManager.send(packet)
    }
    
    func setPitchIntegralLimit(_ limit: Double) async throws {
        let packet = PacketFactory.setPitchIntegralLimit(limit)
        try await networkManager.send(packet)
    }
    
    func setYawIntegralLimit(_ limit: Double) async throws {
        let packet = PacketFactory.setYawIntegralLimit(limit)
        try await networkManager.send(packet)
    }
    
    // MARK: - Configuration
    
    func setOffset(x: Double, y: Double, z: Double) async throws {
        let packet = PacketFactory.setOffset(x: x, y: y, z: z)
        try await networkManager.send(packet)
        
        // Track the last sent offset
        lastSentTargetOffset = (x: x, y: y)
    }
    
    func setLaunchThreshold(epsilon: Double, n: UInt8) async throws {
        let packet = PacketFactory.setLaunchThreshold(epsilon: epsilon, n: n)
        try await networkManager.send(packet)
    }
    
    func setCutoffFrequency(_ frequency: Double) async throws {
        let packet = PacketFactory.setCutoffFrequency(frequency)
        try await networkManager.send(packet)
    }
    
    func setStopThrottle(_ throttle: Double) async throws {
        let packet = PacketFactory.setStopThrottle(throttle)
        try await networkManager.send(packet)
    }
    
    func setMaxConsecutiveNans(_ maxNans: UInt32) async throws {
        let packet = PacketFactory.setMaxConsecutiveNans(maxNans)
        try await networkManager.send(packet)
    }
    
    func setDefaultSpeed(_ speed: Double) async throws {
        let packet = PacketFactory.setDefaultSpeed(speed)
        try await networkManager.send(packet)
    }
    
    func setMotorOffset(_ offset: Double) async throws {
        let packet = PacketFactory.motorOffset(offset)
        try await networkManager.send(packet)
    }
    
    // MARK: - Query
    
    func sendQuery() async throws {
        let packet = PacketFactory.query()
        try await networkManager.send(packet)
    }
    
    // MARK: - Query Timer Management
    
    @MainActor
    private func startQueryTimer() {
        stopQueryTimer() // Stop existing timer if any
        
        guard connectionState.isConnected && isSynchronized else { return }
        
        queryTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { _ in
            Task { @MainActor [weak self] in
                guard let self = self else { return }
                
                // Send query
                try? await self.sendQuery()
                
                // After receiving CurrentState, check for mismatches and update server if needed
                // Give server time to respond
                try? await Task.sleep(nanoseconds: 100_000_000) // 0.1 second
                
                // Check if server mode matches client mode
                if let serverMode = self.systemState.serverMode,
                   serverMode != self.systemState.drivingMode {
                    print("Mode mismatch detected - Client: \(self.systemState.drivingMode), Server: \(serverMode)")
                    // Resend client's mode to server
                    try? await self.sendModeToServer(self.systemState.drivingMode)
                }
                
                // Check target offset synchronization only in AutoAim/Autonomous modes
                if self.systemState.drivingMode == .autoAim || self.systemState.drivingMode == .autonomous {
                    // Get current target offset from settings store
                    if let settingsStore = self.settingsStore {
                        await self.checkAndSyncTargetOffset(
                            currentX: settingsStore.targetOffsetX,
                            currentY: settingsStore.targetOffsetY
                        )
                    }
                }
            }
        }
    }
    
    @MainActor
    private func stopQueryTimer() {
        queryTimer?.invalidate()
        queryTimer = nil
    }
    
    // MARK: - Target Offset Synchronization
    
    func checkAndSyncTargetOffset(currentX: Double, currentY: Double) async {
        // Check if server's target offset differs from our current values
        let serverX = systemState.targetX
        let serverY = systemState.targetY
        
        // Only send if values differ by more than 0.001
        if abs(serverX - currentX) > 0.001 || abs(serverY - currentY) > 0.001 {
            print("Target offset mismatch - Server: (\(serverX), \(serverY)), Client: (\(currentX), \(currentY))")
            
            // Send our target offset to server
            do {
                try await setOffset(x: currentX, y: currentY, z: 0)
                print("Updated server with client target offset")
            } catch {
                print("Failed to update server target offset: \(error)")
            }
        }
    }
    
    // MARK: - Settings Store
    
    func setSettingsStore(_ store: SettingsStore) {
        self.settingsStore = store
    }
}

// MARK: - Control Coordinator Factory
enum ControlCoordinatorFactory {
    @MainActor
    static func create(configuration: NetworkConfiguration = .default) -> ControlCoordinator {
        return ControlCoordinator(configuration: configuration)
    }
    
    #if DEBUG
    @MainActor
    static func createMock() -> ControlCoordinator {
        let mockNetwork = MockNetworkManager()
        return ControlCoordinator(networkManager: mockNetwork)
    }
    #endif
} 