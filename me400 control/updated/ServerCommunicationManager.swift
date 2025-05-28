import Foundation
import Network
import SwiftUI
import Combine

class ServerCommunicationManager: ObservableObject {
    static let shared = ServerCommunicationManager()
    
    @Published var isServerConnected: Bool = false
    @Published var connectionError: String? = nil
    @Published var isProcessingMessage: Bool = false
    @Published var hasReceivedCurrentState: Bool = false
    @Published var isSynchronized: Bool = false
    
    // Tilt data properties
    @Published var currentRoll: Double = 0.0
    @Published var currentPitch: Double = 0.0
    @Published var lastTiltUpdate: Date = Date()
    
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "ServerCommunicationManager", qos: .userInitiated)
    
    // Message queue for ordered processing
    private var messageQueue: [(type: UInt8, payload: Data)] {
        get { ParameterManager.shared.getParameter("MessageQueue", defaultValue: []) }
        set { ParameterManager.shared.setParameter("MessageQueue", value: newValue) }
    }
    
    // Public property to access message queue size
    var messageQueueSize: Int {
        messageQueue.count
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
        if !ParameterManager.shared.hasParameter("MessageQueue") {
            ParameterManager.shared.setParameter("MessageQueue", value: [])
        }
        if !ParameterManager.shared.hasParameter("IsProcessingMessage") {
            ParameterManager.shared.setParameter("IsProcessingMessage", value: false)
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
        
        let params = NWParameters.tcp
        params.requiredInterfaceType = .other
        let endpoint = NWEndpoint.hostPort(host: NWEndpoint.Host(host), port: NWEndpoint.Port(integerLiteral: portNumber))
        connection = NWConnection(to: endpoint, using: params)
        
        connection?.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                guard let self = self else { return }
                
                switch state {
                case .ready:
                    self.isServerConnected = true
                    self.connectionError = nil
                    self.receiveNextMessage()
                    // Send Query packet to sync state
                    _ = self.send(DataPacket.query())
                case .failed(let error):
                    self.isServerConnected = false
                    self.connectionError = error.localizedDescription
                    self.connection?.cancel()
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
    
    func disconnect() {
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
                
                // Measure latency
                if let lastTimestamp = self.lastBboxTimestamp {
                    let latency = now.timeIntervalSince(lastTimestamp)
                    print("[LATENCY] Bbox processing time: \(latency * 1000)ms")
                }
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
                
                // Measure latency
                if let lastTimestamp = self.lastFilteredBboxTimestamp {
                    let latency = now.timeIntervalSince(lastTimestamp)
                    print("[LATENCY] FilteredBbox processing time: \(latency * 1000)ms")
                }
                self.lastFilteredBboxTimestamp = now
                
                // Process on main thread for @Published property safety
                DispatchQueue.main.async {
                    self.boundingBoxHandler.processFilteredBboxData([x, y, z])
                }
                
                // Continue receiving
                self.receiveNextMessage()
                return
            }
            
            // Add other messages to queue
            self.messageQueue.append((type: type, payload: payload))
            
            // Process next message if not already processing
            if !self.isProcessingMessage {
                self.processNextMessage()
            }
            
            // Continue receiving
            self.receiveNextMessage()
        }
    }
    
    private func processNextMessage() {
        guard !messageQueue.isEmpty else {
            isProcessingMessage = false
            return
        }
        
        isProcessingMessage = true
        let message = messageQueue.removeFirst()
        let type = message.type
        let payload = message.payload

        // Handle CurrentState packet for synchronization
        if type == 25 { // CurrentState
            let mode = readUInt8(at: 0, from: payload)
            let state = readUInt8(at: 1, from: payload)
            let launchCounter = readUInt32(at: 3, from: payload)
            let maxConsecutiveNans = readUInt32(at: 7, from: payload)
            let targetX = readDouble(at: 11, from: payload)
            let targetY = readDouble(at: 19, from: payload)
            let stopThrottle = readDouble(at: 27, from: payload)
            let motorOffset = readDouble(at: 35, from: payload)
            let defaultSpeed = readDouble(at: 43, from: payload)
            let cutoffFreq = readDouble(at: 51, from: payload)
            
            print("[DEBUG] Received CurrentState packet - Raw mode: \(mode), Raw state: \(state)")
            
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
                ParameterManager.shared.setParameter("LaunchCounter", value: Int32(launchCounter))
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
            let z = readDouble(at: 16, from: payload)  // unused
            messageText = "Received Tilt: roll=\(roll), pitch=\(pitch)"
            
            // Update tilt data on main thread
            DispatchQueue.main.async {
                self.currentRoll = roll
                self.currentPitch = pitch
                self.lastTiltUpdate = Date()
            }
        case 16: // Log
            if let text = String(bytes: payload, encoding: .utf8) {
                messageText = "Received Log: \(text)"
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
            let counter = Int32(readDouble(at: 0, from: payload))
            messageText = "Received LaunchCounter: \(counter)"
        case 22: // SetStopThrottle
            let throttle = readDouble(at: 0, from: payload)
            messageText = "Received SetStopThrottle: \(throttle)"
        case 23: // SetMaxConsecutiveNans
            let maxNans = readUInt32(at: 0, from: payload)
            messageText = "Received SetMaxConsecutiveNans: \(maxNans)"
        case 24: // SetDefaultSpeed
            let speed = readDouble(at: 0, from: payload)
            messageText = "Received SetDefaultSpeed: \(speed)"
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
        send(packet)
    }
    
    func sendStop() {
        let packet = DataPacket.stop()
        // Immediately update the client state
        DispatchQueue.main.async {
            self.isRunning = false
        }
        send(packet)
    }
    
    deinit {
        disconnect()
    }
} 
