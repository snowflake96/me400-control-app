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
    
    // Server's reported state (for display in ServerStateView)
    var serverMode: DrivingMode? = nil
    var serverIsRunning: Bool? = nil
    var serverFoundOffset: Bool? = nil
    var serverUseInterpolation: Bool? = nil
    var serverMaxNans: UInt8? = nil
    var serverLaunchThresholdN: UInt8? = nil
    var serverLaunchThresholdEps: Double? = nil
    var serverTargetX: Double? = nil
    var serverTargetY: Double? = nil
    var serverStopThrottle: Double? = nil
    var serverMotorOffset: Double? = nil
    var serverDefaultSpeed: Double? = nil
    var serverCutoffFrequency: Double? = nil
    
    // PID Settings from server
    var serverPitchP: Double? = nil
    var serverPitchI: Double? = nil
    var serverPitchILimit: Double? = nil
    var serverPitchIThreshold: Double? = nil
    var serverYawP: Double? = nil
    var serverYawI: Double? = nil
    var serverYawILimit: Double? = nil
    var serverYawIThreshold: Double? = nil
    
    // Parameters
    var launchCounter: UInt8 = 0
    var maxConsecutiveNans: UInt8 = 20  // Client-managed, not from server CurrentState
    var launchThresholdN: UInt8 = 10      // From server
    var launchThresholdEps: Double = 0.005 // From server
    var targetX: Double = 0.0
    var targetY: Double = 0.0
    var stopThrottle: Double = 0.0
    var motorOffset: Double = 0.0
    var defaultSpeed: Double = 0.0
    var cutoffFrequency: Double = 20.0
    
    // PID Parameters
    var pitchP: Double = 10.0
    var pitchI: Double = 0.0
    var pitchIntegralLimit: Double = 1.0
    var pitchIntegralThreshold: Double = 0.025
    var yawP: Double = 10.0
    var yawI: Double = 0.0
    var yawIntegralLimit: Double = 1.0
    var yawIntegralThreshold: Double = 0.025
    
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
    func resetSynchronization()
    func setSynchronizationCallback(_ callback: @escaping () -> Void)
    func isSynchronized() -> Bool
    func updateDrivingMode(_ mode: DrivingMode)
    func updateRunningState(_ isRunning: Bool)
    func setCurrentStateCallback(_ callback: @escaping () -> Void)
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
    private var onCurrentStateReceived: (() -> Void)?
    private var isProcessingSyncPacket = false
    
    // MARK: - Public Methods
    
    func setSynchronizationCallback(_ callback: @escaping () -> Void) {
        queue.async(flags: .barrier) {
            self.onSynchronized = callback
        }
    }
    
    func setCurrentStateCallback(_ callback: @escaping () -> Void) {
        queue.async(flags: .barrier) {
            self.onCurrentStateReceived = callback
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
            
            print("SystemStateManager processing packet: \(packet.type.description)")
            
            // If not synchronized, only process CurrentState packets
            if !self.hasSynchronized && packet.type != .currentState {
                print("Not synchronized yet, ignoring packet: \(packet.type.description)")
                // Ignore other packets until synchronized
                return
            }
            
            switch packet.type {
            case .currentState:
                print("Processing CurrentState packet")
                self.processCurrentState(packet.payload)
                
            case .pidSettings:
                self.processPIDSettings(packet.payload)
                
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
                // Clear log messages on disconnect
                state.lastLogMessage = ""
            }
        }
    }
    
    func reset() {
        queue.async(flags: .barrier) {
            self.hasSynchronized = false
            self.isProcessingSyncPacket = false
            self.onSynchronized = nil
            self.onCurrentStateReceived = nil
        }
        stateSubject.send(SystemState())
    }
    
    func resetSynchronization() {
        queue.async(flags: .barrier) {
            print("SystemStateManager: Resetting synchronization state")
            self.hasSynchronized = false
            self.isProcessingSyncPacket = false
        }
    }
    
    // MARK: - Private Processing Methods
    
    private func processCurrentState(_ payload: Data) {
        print("CurrentState packet size: \(payload.count) bytes")
        guard payload.count >= 62 else { 
            print("ERROR: CurrentState packet too small, expected >= 62 bytes")
            return 
        }  // Updated size (1+1+1+1+1+1+8*7 = 62)
        
        // Notify that CurrentState was received (for latency tracking)
        if let callback = onCurrentStateReceived {
            DispatchQueue.main.async {
                callback()
            }
        }
        
        let mode = PacketDecoder.readUInt8(at: 0, from: payload)
        let systemState = PacketDecoder.readUInt8(at: 1, from: payload)
        let foundOffset = PacketDecoder.readUInt8(at: 2, from: payload) != 0
        let useInterpolation = PacketDecoder.readUInt8(at: 3, from: payload) != 0
        let maxNans = PacketDecoder.readUInt8(at: 4, from: payload)
        let n = PacketDecoder.readUInt8(at: 5, from: payload)
        let eps = PacketDecoder.readDouble(at: 6, from: payload)
        let targetX = PacketDecoder.readDouble(at: 14, from: payload)
        let targetY = PacketDecoder.readDouble(at: 22, from: payload)
        let stopThrottle = PacketDecoder.readDouble(at: 30, from: payload)
        let motorOffset = PacketDecoder.readDouble(at: 38, from: payload)
        let defaultSpeed = PacketDecoder.readDouble(at: 46, from: payload)
        let cutoffFreq = PacketDecoder.readDouble(at: 54, from: payload)
        
        // Check if this is initial sync
        let isInitialSync = !hasSynchronized && !isProcessingSyncPacket
        
        if isInitialSync {
            print("SystemStateManager: Initial sync detected")
            // Prevent processing multiple sync packets
            isProcessingSyncPacket = true
            
            // Initial sync: adopt server's state completely
            updateState { state in
                // Update mode
                if let drivingMode = DrivingMode(rawValue: mode) {
                    state.drivingMode = drivingMode
                    state.serverMode = drivingMode
                }
                
                // Update all parameters from server
                state.launchCounter = 0 // Reset launch counter on sync
                state.launchThresholdN = n
                state.launchThresholdEps = eps
                state.targetX = targetX
                state.targetY = targetY
                state.stopThrottle = stopThrottle
                state.motorOffset = motorOffset
                state.defaultSpeed = defaultSpeed
                state.cutoffFrequency = cutoffFreq
                
                // ALSO store server's current state for ServerStateView display
                state.serverFoundOffset = foundOffset
                state.serverUseInterpolation = useInterpolation
                state.serverMaxNans = maxNans
                state.serverLaunchThresholdN = n
                state.serverLaunchThresholdEps = eps
                state.serverTargetX = targetX
                state.serverTargetY = targetY
                state.serverStopThrottle = stopThrottle
                state.serverMotorOffset = motorOffset
                state.serverDefaultSpeed = defaultSpeed
                state.serverCutoffFrequency = cutoffFreq
                
                // Update running state from server
                if let sysState = RunningState(rawValue: systemState) {
                    state.isRunning = sysState.isRunning
                    state.serverIsRunning = sysState.isRunning
                }
            }
            
            // Mark as synchronized and trigger callback
            hasSynchronized = true
            isProcessingSyncPacket = false  // Reset the flag after sync is complete
            
            print("SystemStateManager: Marked as synchronized, calling callback")
            
            // Trigger synchronization callback on main thread
            if let callback = onSynchronized {
                print("SystemStateManager: Calling synchronization callback")
                DispatchQueue.main.async {
                    callback()
                }
            } else {
                print("SystemStateManager: WARNING - No synchronization callback set!")
            }
        } else if hasSynchronized {
            print("SystemStateManager: Already synchronized, updating state")
            // After initial sync: update both server state AND main state
            updateState { state in
                // Update mode if changed by server
                if let drivingMode = DrivingMode(rawValue: mode) {
                    state.drivingMode = drivingMode
                    state.serverMode = drivingMode
                }
                
                // Update running state
                if let sysState = RunningState(rawValue: systemState) {
                    state.isRunning = sysState.isRunning
                    state.serverIsRunning = sysState.isRunning
                }
                
                // Update all parameters from server
                state.launchThresholdN = n
                state.launchThresholdEps = eps
                state.targetX = targetX
                state.targetY = targetY
                state.stopThrottle = stopThrottle
                state.motorOffset = motorOffset
                state.defaultSpeed = defaultSpeed
                state.cutoffFrequency = cutoffFreq
                
                // Also store server's current state for ServerStateView display
                state.serverFoundOffset = foundOffset
                state.serverUseInterpolation = useInterpolation
                state.serverMaxNans = maxNans
                state.serverLaunchThresholdN = n
                state.serverLaunchThresholdEps = eps
                state.serverTargetX = targetX
                state.serverTargetY = targetY
                state.serverStopThrottle = stopThrottle
                state.serverMotorOffset = motorOffset
                state.serverDefaultSpeed = defaultSpeed
                state.serverCutoffFrequency = cutoffFreq
            }
        }
    }
    
    private func processPIDSettings(_ payload: Data) {
        guard payload.count >= 64 else { return }
        
        // Pitch parameters
        let pitchP = PacketDecoder.readDouble(at: 0, from: payload)
        let pitchI = PacketDecoder.readDouble(at: 8, from: payload)
        let pitchILimit = PacketDecoder.readDouble(at: 16, from: payload)
        let pitchIThreshold = PacketDecoder.readDouble(at: 24, from: payload)
        
        // Yaw parameters
        let yawP = PacketDecoder.readDouble(at: 32, from: payload)
        let yawI = PacketDecoder.readDouble(at: 40, from: payload)
        let yawILimit = PacketDecoder.readDouble(at: 48, from: payload)
        let yawIThreshold = PacketDecoder.readDouble(at: 56, from: payload)
        
        updateState { state in
            // Update local PID parameters
            state.pitchP = pitchP
            state.pitchI = pitchI
            state.pitchIntegralLimit = pitchILimit
            state.pitchIntegralThreshold = pitchIThreshold
            state.yawP = yawP
            state.yawI = yawI
            state.yawIntegralLimit = yawILimit
            state.yawIntegralThreshold = yawIThreshold
            
            // Also store server's PID settings for display
            state.serverPitchP = pitchP
            state.serverPitchI = pitchI
            state.serverPitchILimit = pitchILimit
            state.serverPitchIThreshold = pitchIThreshold
            state.serverYawP = yawP
            state.serverYawI = yawI
            state.serverYawILimit = yawILimit
            state.serverYawIThreshold = yawIThreshold
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
            // If there's an existing message, concatenate with new message first
            if !state.lastLogMessage.isEmpty && !message.isEmpty {
                // Show newest message first, separated by " | "
                state.lastLogMessage = message + " | " + state.lastLogMessage
                
                // Limit the total length to prevent UI overflow (keep last ~200 chars)
                if state.lastLogMessage.count > 200 {
                    if let lastSeparatorIndex = state.lastLogMessage.lastIndex(of: "|") {
                        state.lastLogMessage = String(state.lastLogMessage[..<lastSeparatorIndex])
                    }
                }
            } else {
                state.lastLogMessage = message
            }
        }
    }
    
    private func processLaunchCounter(_ payload: Data) {
        guard payload.count >= 1 else { return }
        
        let counter = PacketDecoder.readUInt8(at: 0, from: payload)
        
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
