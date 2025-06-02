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
    @Published private(set) var latency: TimeInterval = 0.0
    
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
    
    // Latency tracking
    private var lastQueryTime: Date?
    
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
                    print("ControlCoordinator: Connection state changed to connected")
                    self?.stateManager.updateConnectionState(true, error: nil)
                    // Delay slightly to ensure connection is stable
                    DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                        print("ControlCoordinator: Starting synchronization setup after delay")
                        self?.setupSynchronization()
                    }
                    
                case .failed(let error):
                    print("ControlCoordinator: Connection failed: \(error)")
                    self?.stateManager.updateConnectionState(false, error: error.localizedDescription)
                    self?.isSynchronized = false
                    self?.stopQueryTimer()
                    // Reset sync state to ensure clean sync on reconnect
                    self?.stateManager.resetSynchronization()
                    
                case .disconnected:
                    print("ControlCoordinator: Connection state changed to disconnected")
                    self?.stateManager.updateConnectionState(false, error: nil)
                    self?.isSynchronized = false
                    self?.stopQueryTimer()
                    // Reset sync state on disconnect
                    self?.stateManager.resetSynchronization()
                    
                case .connecting:
                    print("ControlCoordinator: Connection state changed to connecting")
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
        
        // Remove mode-based query timer management - Query should be sent in all modes
        // The timer will be started after synchronization and continue regardless of mode
    }
    
    // MARK: - Synchronization
    
    private func setupSynchronization() {
        print("ControlCoordinator: Setting up synchronization")
        // Reset synchronization state
        isSynchronized = false
        
        // Ensure state manager sync is reset
        stateManager.resetSynchronization()
        
        // Set up latency tracking callback - must be done after each connection
        stateManager.setCurrentStateCallback { [weak self] in
            self?.updateLatency()
        }
        
        // Set up callback for when synchronization is complete
        stateManager.setSynchronizationCallback { [weak self] in
            guard let self = self else { return }
            print("ControlCoordinator: Synchronization callback triggered")
            print("Client synchronized with server")
            
            // Set synchronized flag
            self.isSynchronized = true
            
            // Start regular query timer for periodic updates
            self.startQueryTimer()
        }
        
        print("ControlCoordinator: Callbacks set, sending initial query")
        
        // Send a single query to get initial state
        Task {
            do {
                try await sendQuery()
                print("Initial sync query sent")
            } catch {
                print("Failed to send initial sync query: \(error)")
                // Try again after a delay if failed
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
        
        // Reset latency tracking
        lastQueryTime = nil
        
        // Reset latency to 0 for clean UI
        latency = 0.0
        
        stopQueryTimer()
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
        // Send packet first
        let packet = PacketFactory.start()
        try await networkManager.send(packet)
        
        // Then update client state
        stateManager.updateRunningState(true)
    }
    
    func stop() async throws {
        // Send packet first
        let packet = PacketFactory.stop()
        try await networkManager.send(packet)
        
        // Then update client state
        stateManager.updateRunningState(false)
    }
    
    func setMode(_ mode: DrivingMode) async throws {
        // Send mode change to server
        let packet = PacketFactory.setMode(mode)
        try await networkManager.send(packet)
        
        // Update client state immediately
        stateManager.updateDrivingMode(mode)
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
    
    func setPitchIntegralThreshold(_ threshold: Double) async throws {
        let packet = PacketFactory.setPitchIntegralThreshold(threshold)
        try await networkManager.send(packet)
    }
    
    func setYawIntegralThreshold(_ threshold: Double) async throws {
        let packet = PacketFactory.setYawIntegralThreshold(threshold)
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
    
    func setMaxConsecutiveNans(_ maxNans: UInt8) async throws {
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
        // Track query time for latency calculation
        lastQueryTime = Date()
        
        let packet = PacketFactory.query()
        try await networkManager.send(packet)
    }
    
    // MARK: - Query Timer Management
    
    @MainActor
    private func startQueryTimer() {
        stopQueryTimer() // Stop existing timer if any
        
        guard connectionState.isConnected && isSynchronized else { return }
        
        // Send query every 0.2 seconds (5Hz) for ServerStateView updates
        queryTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { _ in
            Task { @MainActor [weak self] in
                guard let self = self else { return }
                
                // Simply send query to get current state from server
                try? await self.sendQuery()
            }
        }
    }
    
    @MainActor
    private func stopQueryTimer() {
        queryTimer?.invalidate()
        queryTimer = nil
    }
    
    // MARK: - Latency Update
    
    func updateLatency() {
        guard let queryTime = lastQueryTime else { 
            // If no query time, don't update latency (keep last value or 0)
            return 
        }
        
        let currentLatency = Date().timeIntervalSince(queryTime)
        latency = currentLatency
    }
    
    // MARK: - Target Offset Synchronization
    
    func checkAndSyncTargetOffset(currentX: Double, currentY: Double) async {
        // This method is no longer needed since server is authoritative
        // Keeping for backward compatibility but it does nothing
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
