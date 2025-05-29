import Foundation
import Combine

// MARK: - System State
struct SystemState {
    // Connection
    var isConnected: Bool = false
    var connectionError: String?
    
    // System Mode & Status
    var drivingMode: DrivingMode = .manual
    var isRunning: Bool = false
    
    // Server's reported mode (for comparison)
    var serverMode: DrivingMode? = nil
    
    // Parameters
    var launchCounter: UInt32 = 0
    var maxConsecutiveNans: UInt32 = 10
    var targetX: Double = 0.0
    var targetY: Double = 0.0
    var stopThrottle: Double = 0.0
    var motorOffset: Double = 0.0
    var defaultSpeed: Double = 1.0
    var cutoffFrequency: Double = 10.0
    
    // PID Parameters
    var pitchP: Double = 10.0
    var pitchI: Double = 0.0
    var pitchIntegralLimit: Double = 1.0
    var yawP: Double = 10.0
    var yawI: Double = 0.0
    var yawIntegralLimit: Double = 1.0
    
    // Sensor Data
    var tiltRoll: Double = 0.0
    var tiltPitch: Double = 0.0
    
    // Bounding Box
    var boundingBox: BoundingBox?
    var filteredBoundingBox: FilteredBoundingBox?
    
    // Log Messages
    var lastLogMessage: String = ""
}

// MARK: - Bounding Box Data
struct BoundingBox {
    let x1: Double
    let y1: Double
    let x2: Double
    let y2: Double
    let timestamp: Date
    
    var centerX: Double { (x1 + x2) / 2 }
    var centerY: Double { (y1 + y2) / 2 }
    var width: Double { x2 - x1 }
    var height: Double { y2 - y1 }
    
    var isValid: Bool {
        return !x1.isNaN && !y1.isNaN && !x2.isNaN && !y2.isNaN
    }
}

struct FilteredBoundingBox {
    let centerX: Double
    let centerY: Double
    let timestamp: Date
    
    var isValid: Bool {
        return !centerX.isNaN && !centerY.isNaN
    }
}

// MARK: - System State Manager Protocol
protocol SystemStateManagerProtocol: AnyObject {
    var state: AnyPublisher<SystemState, Never> { get }
    var currentState: SystemState { get }
    
    func processPacket(_ packet: ReceivedPacket)
    func updateConnectionState(_ isConnected: Bool, error: String?)
    func reset()
    func setSynchronizationCallback(_ callback: @escaping () -> Void)
    func isSynchronized() -> Bool
    func updateDrivingMode(_ mode: DrivingMode)
    func updateRunningState(_ isRunning: Bool)
}

// MARK: - System State Manager
final class SystemStateManager: SystemStateManagerProtocol {
    // State
    private let stateSubject = CurrentValueSubject<SystemState, Never>(SystemState())
    
    var state: AnyPublisher<SystemState, Never> {
        stateSubject.eraseToAnyPublisher()
    }
    
    var currentState: SystemState {
        stateSubject.value
    }
    
    // Synchronization
    private var hasSynchronized = false
    private let queue = DispatchQueue(label: "com.me400.statemanager", attributes: .concurrent)
    
    // Add synchronization callback
    private var onSynchronized: (() -> Void)?
    
    // MARK: - Public Methods
    
    func setSynchronizationCallback(_ callback: @escaping () -> Void) {
        queue.async(flags: .barrier) {
            self.onSynchronized = callback
        }
    }
    
    func isSynchronized() -> Bool {
        queue.sync {
            return hasSynchronized
        }
    }
    
    // MARK: - Packet Processing
    
    func processPacket(_ packet: ReceivedPacket) {
        queue.async(flags: .barrier) { [weak self] in
            guard let self = self else { return }
            
            // If not synchronized, only process CurrentState packets
            if !self.hasSynchronized && packet.type != .currentState {
                // Ignore other packets until synchronized
                return
            }
            
            switch packet.type {
            case .currentState:
                self.processCurrentState(packet.payload)
                
            case .bboxPos:
                self.processBoundingBox(packet.payload, timestamp: packet.timestamp)
                
            case .filteredBbox:
                self.processFilteredBoundingBox(packet.payload, timestamp: packet.timestamp)
                
            case .tilt:
                self.processTilt(packet.payload)
                
            case .log:
                self.processLog(packet.payload)
                
            case .launchCounter:
                self.processLaunchCounter(packet.payload)
                
            default:
                break
            }
        }
    }
    
    // MARK: - State Updates
    
    func updateConnectionState(_ isConnected: Bool, error: String?) {
        updateState { state in
            state.isConnected = isConnected
            state.connectionError = error
            
            if !isConnected {
                // Reset real-time data on disconnect
                state.boundingBox = nil
                state.filteredBoundingBox = nil
            }
        }
    }
    
    func reset() {
        queue.async(flags: .barrier) {
            self.hasSynchronized = false
            self.onSynchronized = nil
        }
        stateSubject.send(SystemState())
    }
    
    // MARK: - Private Processing Methods
    
    private func processCurrentState(_ payload: Data) {
        guard payload.count >= 58 else { return }
        
        let mode = PacketDecoder.readUInt8(at: 0, from: payload)
        let systemState = PacketDecoder.readUInt8(at: 1, from: payload)
        let launchCounter = PacketDecoder.readUInt32(at: 2, from: payload)
        let maxConsecutiveNans = PacketDecoder.readUInt32(at: 6, from: payload)
        let targetX = PacketDecoder.readDouble(at: 10, from: payload)
        let targetY = PacketDecoder.readDouble(at: 18, from: payload)
        let stopThrottle = PacketDecoder.readDouble(at: 26, from: payload)
        let motorOffset = PacketDecoder.readDouble(at: 34, from: payload)
        let defaultSpeed = PacketDecoder.readDouble(at: 42, from: payload)
        let cutoffFreq = PacketDecoder.readDouble(at: 50, from: payload)
        
        // Check if this is initial sync
        let isInitialSync = !hasSynchronized
        
        updateState { state in
            // Always store server's mode for comparison
            if let serverDrivingMode = DrivingMode(rawValue: mode) {
                state.serverMode = serverDrivingMode
            }
            
            if isInitialSync {
                // Initial sync: adopt server's state completely
                if let drivingMode = DrivingMode(rawValue: mode) {
                    state.drivingMode = drivingMode
                }
                
                // Update all parameters from server
                state.launchCounter = launchCounter
                state.maxConsecutiveNans = maxConsecutiveNans
                state.targetX = targetX
                state.targetY = targetY
                state.stopThrottle = stopThrottle
                state.motorOffset = motorOffset
                state.defaultSpeed = defaultSpeed
                state.cutoffFrequency = cutoffFreq
            } else {
                // After sync: only update server-controlled values
                // Don't update mode - client controls it
                // Only update values that server manages (like launch counter)
                state.launchCounter = launchCounter
                
                // Store server's view of parameters for comparison
                // These will be used by coordinator to detect mismatches
                state.targetX = targetX
                state.targetY = targetY
                state.maxConsecutiveNans = maxConsecutiveNans
                state.stopThrottle = stopThrottle
                state.motorOffset = motorOffset
                state.defaultSpeed = defaultSpeed
                state.cutoffFrequency = cutoffFreq
            }
            
            // Always update running state from server
            if let sysState = RunningState(rawValue: systemState) {
                state.isRunning = sysState.isRunning
            }
        }
        
        // Mark as synchronized and trigger callback
        let wasFirstSync = !hasSynchronized
        hasSynchronized = true
        
        if wasFirstSync {
            // Trigger synchronization callback on main thread
            if let callback = onSynchronized {
                DispatchQueue.main.async {
                    callback()
                }
            }
        }
    }
    
    private func processBoundingBox(_ payload: Data, timestamp: Date) {
        guard payload.count >= 32 else { return }
        
        let x1 = PacketDecoder.readDouble(at: 0, from: payload)
        let y1 = PacketDecoder.readDouble(at: 8, from: payload)
        let x2 = PacketDecoder.readDouble(at: 16, from: payload)
        let y2 = PacketDecoder.readDouble(at: 24, from: payload)
        
        let bbox = BoundingBox(x1: x1, y1: y1, x2: x2, y2: y2, timestamp: timestamp)
        
        updateState { state in
            state.boundingBox = bbox.isValid ? bbox : nil
        }
    }
    
    private func processFilteredBoundingBox(_ payload: Data, timestamp: Date) {
        guard payload.count >= 16 else { return }
        
        let centerX = PacketDecoder.readDouble(at: 0, from: payload)
        let centerY = PacketDecoder.readDouble(at: 8, from: payload)
        
        let filteredBbox = FilteredBoundingBox(
            centerX: centerX,
            centerY: centerY,
            timestamp: timestamp
        )
        
        updateState { state in
            state.filteredBoundingBox = filteredBbox.isValid ? filteredBbox : nil
        }
    }
    
    private func processTilt(_ payload: Data) {
        guard payload.count >= 16 else { return }
        
        let roll = PacketDecoder.readDouble(at: 0, from: payload)
        let pitch = PacketDecoder.readDouble(at: 8, from: payload)
        
        updateState { state in
            state.tiltRoll = roll
            state.tiltPitch = pitch
        }
    }
    
    private func processLog(_ payload: Data) {
        let message = PacketDecoder.readString(from: payload)
        
        updateState { state in
            state.lastLogMessage = message
        }
    }
    
    private func processLaunchCounter(_ payload: Data) {
        guard payload.count >= 4 else { return }
        
        let counter = PacketDecoder.readUInt32(at: 0, from: payload)
        
        updateState { state in
            state.launchCounter = counter
        }
    }
    
    // MARK: - Helper Methods
    
    private func updateState(_ update: @escaping (inout SystemState) -> Void) {
        queue.async(flags: .barrier) {
            var newState = self.stateSubject.value
            update(&newState)
            
            DispatchQueue.main.async {
                self.stateSubject.send(newState)
            }
        }
    }
    
    func updateRunningState(_ isRunning: Bool) {
        updateState { state in
            state.isRunning = isRunning
        }
    }
    
    func updateDrivingMode(_ mode: DrivingMode) {
        updateState { state in
            state.drivingMode = mode
        }
    }
}

// MARK: - System State Manager Factory
enum SystemStateManagerFactory {
    static func create() -> SystemStateManagerProtocol {
        return SystemStateManager()
    }
} 