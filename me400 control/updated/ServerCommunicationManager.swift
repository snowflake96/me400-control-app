import Foundation
import Network
import SwiftUI
import Combine

class ServerCommunicationManager: ObservableObject {
    static let shared = ServerCommunicationManager()
    
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "com.me400.serverConnection")
    
    // Thread-safe message queue
    private let messageQueueLock = NSLock()
    private var _messageQueue: [(type: UInt8, payload: Data)] = []
    private var messageQueue: [(type: UInt8, payload: Data)] {
        get {
            messageQueueLock.lock()
            defer { messageQueueLock.unlock() }
            return _messageQueue
        }
        set {
            messageQueueLock.lock()
            defer { messageQueueLock.unlock() }
            _messageQueue = newValue
        }
    }
    
    // Public property to access message queue size
    var messageQueueSize: Int {
        messageQueueLock.lock()
        defer { messageQueueLock.unlock() }
        return _messageQueue.count
    }
    
    private var isProcessingMessage: Bool {
        get { ParameterManager.shared.getParameter("IsProcessingMessage", defaultValue: false) }
        set { ParameterManager.shared.setParameter("IsProcessingMessage", value: newValue) }
    }
    
    // Synchronization state managed by ParameterManager
    private var hasReceivedCurrentState: Bool {
        get { ParameterManager.shared.getParameter("HasReceivedCurrentState", defaultValue: false) }
        set { ParameterManager.shared.setParameter("HasReceivedCurrentState", value: newValue) }
    }
    
    private var isSynchronized: Bool {
        get { ParameterManager.shared.getParameter("IsSynchronized", defaultValue: false) }
        set { ParameterManager.shared.setParameter("IsSynchronized", value: newValue) }
    }
    
    // Published properties to notify observers of changes
    private var isServerConnected: Bool {
        didSet {
            ParameterManager.shared.setParameter("ServerConnected", value: isServerConnected)
        }
    }
    
    private var connectionError: String? {
        didSet {
            ParameterManager.shared.setParameter("ServerError", value: connectionError ?? "")
        }
    }
    
    // Properties managed by ParameterManager
    var receivedMessages: [String] {
        get { ParameterManager.shared.getParameter("ReceivedMessages", defaultValue: []) }
        set { ParameterManager.shared.setParameter("ReceivedMessages", value: newValue) }
    }
    
    enum DrivingMode {
        case manual
        case autonomous
        case autoAim
    }
    
    var currentMode: DrivingMode {
        get { 
            let modeString = ParameterManager.shared.getParameter("CurrentMode", defaultValue: "manual")
            switch modeString {
            case "autonomous": return .autonomous
            case "autoAim": return .autoAim
            default: return .manual
            }
        }
        set {
            let modeString: String
            switch newValue {
            case .autonomous: modeString = "autonomous"
            case .autoAim: modeString = "autoAim"
            case .manual: modeString = "manual"
            }
            ParameterManager.shared.setParameter("CurrentMode", value: modeString)
        }
    }
    
    var isRunning: Bool {
        get { ParameterManager.shared.getParameter("IsRunning", defaultValue: false) }
        set { ParameterManager.shared.setParameter("IsRunning", value: newValue) }
    }
    
    let boundingBoxHandler = BoundingBoxHandler.shared
    
    // Server configuration from ParameterManager
    var host: String {
        get { ParameterManager.shared.getParameter("ServerHost", defaultValue: "localhost") }
        set { ParameterManager.shared.setParameter("ServerHost", value: newValue) }
    }
    
    var port: String {
        get { ParameterManager.shared.getParameter("ServerPort", defaultValue: "12345") }
        set { ParameterManager.shared.setParameter("ServerPort", value: newValue) }
    }
    
    // Add timestamp tracking for latency measurement
    private var lastBboxTimestamp: Date?
    private var lastFilteredBboxTimestamp: Date?
    
    // Connection retry mechanism
    private var reconnectTimer: Timer?
    private let maxReconnectAttempts = 5
    private var reconnectAttempts = 0
    
    // Static initialization to ensure parameters exist
    static func initializeParameters() {
        // Initialize with default values if they don't exist
        if !ParameterManager.shared.hasParameter("ServerHost") {
            ParameterManager.shared.setParameter("ServerHost", value: "100.102.243.9") // change this for changing default
        }
        if !ParameterManager.shared.hasParameter("ServerPort") {
            ParameterManager.shared.setParameter("ServerPort", value: "12345")
        }
        if !ParameterManager.shared.hasParameter("ServerConnected") {
            ParameterManager.shared.setParameter("ServerConnected", value: false)
        }
        if !ParameterManager.shared.hasParameter("ServerError") {
            ParameterManager.shared.setParameter("ServerError", value: "")
        }
        if !ParameterManager.shared.hasParameter("ReceivedMessages") {
            ParameterManager.shared.setParameter("ReceivedMessages", value: [])
        }
        if !ParameterManager.shared.hasParameter("CurrentMode") {
            ParameterManager.shared.setParameter("CurrentMode", value: "manual")
        }
        if !ParameterManager.shared.hasParameter("IsRunning") {
            ParameterManager.shared.setParameter("IsRunning", value: false)
        }
        if !ParameterManager.shared.hasParameter("HasReceivedCurrentState") {
            ParameterManager.shared.setParameter("HasReceivedCurrentState", value: false)
        }
        if !ParameterManager.shared.hasParameter("IsSynchronized") {
            ParameterManager.shared.setParameter("IsSynchronized", value: false)
        }
        if !ParameterManager.shared.hasParameter("IsProcessingMessage") {
            ParameterManager.shared.setParameter("IsProcessingMessage", value: false)
        }
        
        // Initialize PID parameters with default values
        if !ParameterManager.shared.hasParameter("PitchP") {
            ParameterManager.shared.setParameter("PitchP", value: 10.0)
        }
        if !ParameterManager.shared.hasParameter("PitchPStepSize") {
            ParameterManager.shared.setParameter("PitchPStepSize", value: 1.0)
        }
        if !ParameterManager.shared.hasParameter("PitchI") {
            ParameterManager.shared.setParameter("PitchI", value: 0.0)
        }
        if !ParameterManager.shared.hasParameter("PitchIStepSize") {
            ParameterManager.shared.setParameter("PitchIStepSize", value: 0.1)
        }
        if !ParameterManager.shared.hasParameter("PitchIntegralLimit") {
            ParameterManager.shared.setParameter("PitchIntegralLimit", value: 1.0)
        }
        if !ParameterManager.shared.hasParameter("PitchIntegralLimitStepSize") {
            ParameterManager.shared.setParameter("PitchIntegralLimitStepSize", value: 0.1)
        }
        if !ParameterManager.shared.hasParameter("YawP") {
            ParameterManager.shared.setParameter("YawP", value: 10.0)
        }
        if !ParameterManager.shared.hasParameter("YawPStepSize") {
            ParameterManager.shared.setParameter("YawPStepSize", value: 1.0)
        }
        if !ParameterManager.shared.hasParameter("YawI") {
            ParameterManager.shared.setParameter("YawI", value: 0.0)
        }
        if !ParameterManager.shared.hasParameter("YawIStepSize") {
            ParameterManager.shared.setParameter("YawIStepSize", value: 0.1)
        }
        if !ParameterManager.shared.hasParameter("YawIntegralLimit") {
            ParameterManager.shared.setParameter("YawIntegralLimit", value: 1.0)
        }
        if !ParameterManager.shared.hasParameter("YawIntegralLimitStepSize") {
            ParameterManager.shared.setParameter("YawIntegralLimitStepSize", value: 0.1)
        }
        if !ParameterManager.shared.hasParameter("YawIntegralThreshold") {
            ParameterManager.shared.setParameter("YawIntegralThreshold", value: 0.025)
        }
        if !ParameterManager.shared.hasParameter("YawIntegralThresholdStepSize") {
            ParameterManager.shared.setParameter("YawIntegralThresholdStepSize", value: 0.01)
        }
        
        // Initialize client-side parameters with default values
        if !ParameterManager.shared.hasParameter("LaunchThresholdEpsilon") {
            ParameterManager.shared.setParameter("LaunchThresholdEpsilon", value: 0.005)
        }
    }
    
    // Call this in your App's init or first view
    static func initialize() {
        initializeParameters()
    }
    
    init() {
        // Ensure parameters are initialized when creating a new instance
        ServerCommunicationManager.initializeParameters()
        
        // Initialize stored properties
        self.isServerConnected = false
        self.connectionError = nil
    }
    
    func connect() {
        guard let portNumber = UInt16(port) else {
            connectionError = "Invalid port number"
            return
        }
        
        // Reset synchronization state
        hasReceivedCurrentState = false
        isSynchronized = false
        
        // Reset reconnect attempts
        reconnectAttempts = 0
        reconnectTimer?.invalidate()
        
        let params = NWParameters.tcp
        params.requiredInterfaceType = .other
        
        // Configure TCP options for keepalive
        if let tcpOptions = params.defaultProtocolStack.internetProtocol as? NWProtocolTCP.Options {
            tcpOptions.enableKeepalive = true
            tcpOptions.keepaliveIdle = 5
            tcpOptions.keepaliveInterval = 5
            tcpOptions.keepaliveCount = 3
        }
        
        let endpoint = NWEndpoint.hostPort(host: NWEndpoint.Host(host), port: NWEndpoint.Port(integerLiteral: portNumber))
        connection = NWConnection(to: endpoint, using: params)
        
        // Implement connection timeout with a timer
        var connectionTimeoutTimer: Timer?
        connectionTimeoutTimer = Timer.scheduledTimer(withTimeInterval: 10.0, repeats: false) { _ in
            if self.connection?.state != .ready {
                self.connectionError = "Connection timeout"
                self.connection?.cancel()
            }
        }
        
        connection?.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                guard let self = self else { return }
                
                // Cancel timeout timer if connection succeeds or fails
                connectionTimeoutTimer?.invalidate()
                
                switch state {
                case .ready:
                    self.isServerConnected = true
                    self.connectionError = nil
                    self.reconnectAttempts = 0
                    self.receiveNextMessage()
                    // Send Query packet to sync state
                    _ = self.send(DataPacket.query())
                case .failed(let error):
                    self.isServerConnected = false
                    self.connectionError = error.localizedDescription
                    self.connection?.cancel()
                    self.handleDisconnection()
                case .waiting(let error):
                    self.isServerConnected = false
                    self.connectionError = error.localizedDescription
                case .cancelled:
                    self.isServerConnected = false
                    self.connectionError = nil
                default:
                    break
                }
            }
        }
        
        connection?.start(queue: queue)
    }
    
    private func handleDisconnection() {
        guard reconnectAttempts < maxReconnectAttempts else {
            connectionError = "Max reconnection attempts reached"
            return
        }
        
        reconnectTimer?.invalidate()
        reconnectTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: false) { [weak self] _ in
            guard let self = self else { return }
            self.reconnectAttempts += 1
            self.connect()
        }
    }
    
    func disconnect() {
        // Cancel any reconnect timer
        reconnectTimer?.invalidate()
        reconnectTimer = nil
        
        // Cancel the connection
        connection?.cancel()
        
        // Reset connection state
        isServerConnected = false
        connectionError = nil
        
        // Reset synchronization state
        hasReceivedCurrentState = false
        isSynchronized = false
        
        // Clear message queue and processing state
        messageQueue.removeAll()
        isProcessingMessage = false
        
        // Reset other states
        currentMode = .manual
        isRunning = false
        receivedMessages.removeAll()
        
        // Clear log messages
        DispatchQueue.main.async {
            ParameterManager.shared.lastLogMessage = ""
        }
        
        // Set connection to nil after cleanup
        connection = nil
    }
    
    // MARK: – Helper functions for reading data
    private func readDouble(at offset: Int, from payload: Data) -> Double {
        let byteCount = MemoryLayout<UInt64>.size
        guard offset >= 0, offset + byteCount <= payload.count else {
            print("[ERROR] readDouble: offset \(offset) out of range.")
            return 0
        }

        // Copy the raw 8 bytes into a UInt64 bitPattern
        let bitPattern: UInt64 = payload.withUnsafeBytes { rawBuf in
            let src = rawBuf.baseAddress!.advanced(by: offset)
            var local: UInt64 = 0
            memcpy(&local, src, byteCount)
            return local
        }

        let hostBits = UInt64(littleEndian: bitPattern)
        return Double(bitPattern: hostBits)
    }

    private func readUInt8(at offset: Int, from payload: Data) -> UInt8 {
        guard offset >= 0, offset < payload.count else {
            print("[ERROR] readUInt8: offset \(offset) out of range.")
            return 0
        }
        return payload.withUnsafeBytes { rawBuf in
            let src = rawBuf.baseAddress!.advanced(by: offset)
            var local: UInt8 = 0
            memcpy(&local, src, 1)
            return local
        }
    }

    private func readUInt32(at offset: Int, from payload: Data) -> UInt32 {
        let byteCount = MemoryLayout<UInt32>.size
        guard offset >= 0, offset + byteCount <= payload.count else {
            print("[ERROR] readUInt32: offset \(offset) out of range.")
            return 0
        }

        // Copy the raw 4 bytes into a UInt32 bitPattern
        let bitPattern: UInt32 = payload.withUnsafeBytes { rawBuf in
            let src = rawBuf.baseAddress!.advanced(by: offset)
            var local: UInt32 = 0
            memcpy(&local, src, byteCount)
            return local
        }

        return UInt32(littleEndian: bitPattern)
    }

    private func receiveNextMessage() {
        connection?.receive(minimumIncompleteLength: 65, maximumLength: 65) { [weak self] packet, _, isComplete, error in
            guard let self = self else { return }
            
            // Check for connection errors or disconnection
            if let error = error {
                print("Connection error or disconnection detected: \(error.localizedDescription)")
                DispatchQueue.main.async {
                    self.disconnect()
                }
                return
            }
            
            // Check if connection is still valid
            guard let packet = packet, packet.count == 65 else {
                print("Failed to read 65-byte packet or connection closed.")
                DispatchQueue.main.async {
                    self.disconnect()
                }
                return
            }

            let type = packet[0]
            let payload = packet[1..<65]
            let now = Date()
            
            // Process bounding box and filtered box messages immediately
            if type == 14 { // BboxPos
                let x = self.readDouble(at: 0, from: payload)
                let y = self.readDouble(at: 8, from: payload)
                let z = self.readDouble(at: 16, from: payload)
                let w = self.readDouble(at: 24, from: payload)
                
                self.lastBboxTimestamp = now
                
                // Process on main thread for @Published property safety
                DispatchQueue.main.async {
                    self.boundingBoxHandler.processBoundingBoxData([x, y, z, w])
                }
                
                // Continue receiving
                self.receiveNextMessage()
                return
            }
            
            if type == 19 { // FilteredBbox
                let x = self.readDouble(at: 0, from: payload)
                let y = self.readDouble(at: 8, from: payload)
                let z = self.readDouble(at: 16, from: payload)
                
                self.lastFilteredBboxTimestamp = now
                
                // Process on main thread for @Published property safety
                DispatchQueue.main.async {
                    self.boundingBoxHandler.processFilteredBboxData([x, y, z])
                }
                
                // Continue receiving
                self.receiveNextMessage()
                return
            }
            
            // Add other messages to queue with thread safety
            self.messageQueueLock.lock()
            self._messageQueue.append((type: type, payload: payload))
            self.messageQueueLock.unlock()
            
            // Process next message if not already processing
            if !self.isProcessingMessage {
                self.processNextMessage()
            }
            
            // Continue receiving
            self.receiveNextMessage()
        }
    }
    
    private func processNextMessage() {
        // Get next message from queue with thread safety
        messageQueueLock.lock()
        guard !_messageQueue.isEmpty else {
            messageQueueLock.unlock()
            isProcessingMessage = false
            return
        }
        
        isProcessingMessage = true
        let message = _messageQueue.removeFirst()
        messageQueueLock.unlock()
        
        let type = message.type
        let payload = message.payload

        // Handle CurrentState packet for synchronization
        if type == 25 { // CurrentState
            // Debug: Print the raw payload in hex for LaunchCounter debugging
            let hexString = payload.map { String(format: "%02x", $0) }.joined(separator: " ")
            print("[DEBUG] CurrentState packet payload (hex): \(hexString)")
            
            let mode = readUInt8(at: 0, from: payload)
            let state = readUInt8(at: 1, from: payload)
            let launchCounter = readUInt32(at: 2, from: payload)
            let maxConsecutiveNans = readUInt32(at: 6, from: payload)
            let targetX = readDouble(at: 10, from: payload)
            let targetY = readDouble(at: 18, from: payload)
            let stopThrottle = readDouble(at: 26, from: payload)
            let motorOffset = readDouble(at: 34, from: payload)
            let defaultSpeed = readDouble(at: 42, from: payload)
            let cutoffFreq = readDouble(at: 50, from: payload)
            
            print("[DEBUG] Received CurrentState packet - Raw mode: \(mode), Raw state: \(state)")
            print("[DEBUG] CurrentState LaunchCounter: \(launchCounter) (from bytes 2-5: \(Array(payload[2..<6]).map { String(format: "%02x", $0) }.joined(separator: " ")))")
            
            DispatchQueue.main.async {
                // Update mode
                let modeString: String
                switch mode {
                case 7:
                    self.currentMode = .autonomous
                    modeString = "Autonomous"
                case 8:
                    self.currentMode = .manual
                    modeString = "Manual"
                case 9:
                    self.currentMode = .autoAim
                    modeString = "AutoAim"
                default:
                    self.currentMode = .manual
                    modeString = "Manual"
                }
                print("[DEBUG] Setting mode to: \(modeString) (from raw value: \(mode))")
                ParameterManager.shared.selectedMode = modeString
                
                // Update running state
                switch state {
                case 5: self.isRunning = true
                case 6: self.isRunning = false
                default: break
                }
                
                // Update other parameters
                ParameterManager.shared.setParameter("LaunchCounter", value: launchCounter)
                ParameterManager.shared.launchCounter = launchCounter
                ParameterManager.shared.setParameter("MaxConsecutiveNans", value: maxConsecutiveNans)
                ParameterManager.shared.setParameter("TargetX", value: targetX)
                ParameterManager.shared.setParameter("TargetY", value: targetY)
                ParameterManager.shared.setParameter("StopThrottle", value: stopThrottle)
                ParameterManager.shared.setParameter("MotorOffset", value: motorOffset)
                ParameterManager.shared.setParameter("DefaultSpeed", value: defaultSpeed)
                ParameterManager.shared.setParameter("CutoffFrequency", value: cutoffFreq)
                
                // Mark as synchronized
                self.hasReceivedCurrentState = true
                self.isSynchronized = true
                print("[DEBUG] Synchronization complete. Mode: \(self.currentMode), Running: \(self.isRunning)")
            }
            self.processNextMessage()
            return
        }

        var messageText = ""
        switch type {
        case 0: // ServoCommand
            let x = readDouble(at: 0, from: payload)
            let y = readDouble(at: 8, from: payload)
            let z = readDouble(at: 16, from: payload)
            messageText = "Received ServoCommand: x=\(x), y=\(y), z=\(z)"
        case 1: // TriggerCommand
            let flag = payload[0] != 0
            messageText = "Received TriggerCommand: \(flag)"
        case 2: // EscCommand
            let value = readDouble(at: 0, from: payload)
            messageText = "Received EscCommand: \(value)"
        case 3: // TunePitch
            let p = readDouble(at: 0, from: payload)
            let i = readDouble(at: 8, from: payload)
            let d = readDouble(at: 16, from: payload)
            messageText = "Received TunePitch: p=\(p), i=\(i), d=\(d)"
        case 4: // TuneYaw
            let p = readDouble(at: 0, from: payload)
            let i = readDouble(at: 8, from: payload)
            let d = readDouble(at: 16, from: payload)
            messageText = "Received TuneYaw: p=\(p), i=\(i), d=\(d)"
        case 10: // SetOffset
            let x = readDouble(at: 0, from: payload)
            let y = readDouble(at: 8, from: payload)
            let z = readDouble(at: 16, from: payload)
            messageText = "Received SetOffset: x=\(x), y=\(y), z=\(z)"
        case 11: // SetPitchIntegralLimit
            let limit = readDouble(at: 0, from: payload)
            messageText = "Received SetPitchIntegralLimit: \(limit)"
        case 12: // SetYawIntegralLimit
            let limit = readDouble(at: 0, from: payload)
            messageText = "Received SetYawIntegralLimit: \(limit)"
        case 13: // SetLaunchThreshold
            let eps = readDouble(at: 0, from: payload)
            let n = payload[8]
            messageText = "Received SetLaunchThreshold: eps=\(eps), n=\(n)"
        case 15: // Tilt
            let roll = readDouble(at: 0, from: payload)
            let pitch = readDouble(at: 8, from: payload)
            let _ = readDouble(at: 16, from: payload)  // unused z coordinate
            messageText = "Received Tilt: roll=\(roll), pitch=\(pitch)"
            
            // Store tilt data in ParameterManager
            DispatchQueue.main.async {
                ParameterManager.shared.tiltRoll = roll
                ParameterManager.shared.tiltPitch = pitch
            }
        case 16: // Log
            if let text = String(bytes: payload, encoding: .utf8) {
                messageText = "Received Log: \(text)"
                
                // Store log message in ParameterManager
                DispatchQueue.main.async {
                    // Get current log message
                    let currentLog = ParameterManager.shared.lastLogMessage
                    let newLogText = text.trimmingCharacters(in: .whitespacesAndNewlines)
                    
                    // Concatenate with existing log, newest first
                    if !currentLog.isEmpty && !newLogText.isEmpty {
                        ParameterManager.shared.lastLogMessage = newLogText + " | " + currentLog
                        
                        // Limit length to prevent overflow
                        if ParameterManager.shared.lastLogMessage.count > 200 {
                            if let lastSeparatorIndex = ParameterManager.shared.lastLogMessage.lastIndex(of: "|") {
                                ParameterManager.shared.lastLogMessage = String(ParameterManager.shared.lastLogMessage[..<lastSeparatorIndex])
                            }
                        }
                    } else {
                        ParameterManager.shared.lastLogMessage = newLogText
                    }
                }
            }
        case 17: // Query
            messageText = "Received Query"
        case 18: // MotorOffset
            let offset = readDouble(at: 0, from: payload)
            messageText = "Received MotorOffset: \(offset)"
        case 20: // SetCutoffFrequency
            let frequency = readDouble(at: 0, from: payload)
            messageText = "Received SetCutoffFrequency: \(frequency)"
        case 21: // LaunchCounter
            // Debug: Print the raw payload in hex
            let hexString = payload.map { String(format: "%02x", $0) }.joined(separator: " ")
            print("[DEBUG] LaunchCounter packet payload (hex): \(hexString)")
            
            let counter = readUInt32(at: 0, from: payload)
            print("[DEBUG] LaunchCounter parsed value: \(counter) (from bytes 0-3: \(Array(payload[0..<4]).map { String(format: "%02x", $0) }.joined(separator: " ")))")
            messageText = "Received LaunchCounter: \(counter)"
            
            // Store launch counter in ParameterManager
            DispatchQueue.main.async {
                ParameterManager.shared.launchCounter = counter
            }
        case 22: // SetStopThrottle
            let throttle = readDouble(at: 0, from: payload)
            messageText = "Received SetStopThrottle: \(throttle)"
        case 23: // SetMaxConsecutiveNans
            let maxNans = readUInt32(at: 0, from: payload)
            messageText = "Received SetMaxConsecutiveNans: \(maxNans)"
        case 24: // SetDefaultSpeed
            let speed = readDouble(at: 0, from: payload)
            messageText = "Received SetDefaultSpeed: \(speed)"
        case 26: // SetPitchIntegralThreshold
            let threshold = readDouble(at: 0, from: payload)
            messageText = "Received SetPitchIntegralThreshold: \(threshold)"
        case 27: // SetYawIntegralThreshold
            let threshold = readDouble(at: 0, from: payload)
            messageText = "Received SetYawIntegralThreshold: \(threshold)"
        default:
            messageText = "Received packet type unhandled: type=\(type)"
        }

        DispatchQueue.main.async {
            self.receivedMessages.append(messageText) // for debug
            // Keep only the last 100 messages
            if self.receivedMessages.count > 100 {
                self.receivedMessages.removeFirst()
            }
            self.processNextMessage()
        }
    }
    
    private func checkSynchronization() {
        if hasReceivedCurrentState {
            isSynchronized = true
            print("Synchronization complete. Mode: \(currentMode), Running: \(isRunning)")
        }
    }
    
    func send(_ packet: DataPacket) -> Bool {
        guard let connection = connection, isServerConnected else {
            return false
        }
        
        let data = packet.toBytes()
        connection.send(content: data, completion: .contentProcessed { [weak self] error in
            if let error = error {
            DispatchQueue.main.async {
                    self?.connectionError = error.localizedDescription
                }
            }
        })
        return true
    }
    
    func sendStart() {
        let packet = DataPacket.start()
        // Immediately update the client state
        DispatchQueue.main.async {
            self.isRunning = true
        }
        _ = send(packet)
    }
    
    func sendStop() {
        let packet = DataPacket.stop()
        // Immediately update the client state
        DispatchQueue.main.async {
            self.isRunning = false
        }
        _ = send(packet)
    }
    
    deinit {
        disconnect()
    }
} 
