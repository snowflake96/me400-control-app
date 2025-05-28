import Foundation
import Network
import SwiftUI
import Combine

class ServerCommunicationManager: ObservableObject {
    static let shared = ServerCommunicationManager()
    
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "com.me400.serverConnection")
    
    // Message queue for ordered processing
    private var messageQueue: [(type: UInt8, payload: Data)] {
        get { ParameterManager.shared.getParameter("MessageQueue", defaultValue: []) }
        set { ParameterManager.shared.setParameter("MessageQueue", value: newValue) }
    }
    
    private var isProcessingMessage: Bool {
        get { ParameterManager.shared.getParameter("IsProcessingMessage", defaultValue: false) }
        set { ParameterManager.shared.setParameter("IsProcessingMessage", value: newValue) }
    }
    
    // Synchronization state managed by ParameterManager
    private var hasReceivedMode: Bool {
        get { ParameterManager.shared.getParameter("HasReceivedMode", defaultValue: false) }
        set { ParameterManager.shared.setParameter("HasReceivedMode", value: newValue) }
    }
    
    private var hasReceivedRunningState: Bool {
        get { ParameterManager.shared.getParameter("HasReceivedRunningState", defaultValue: false) }
        set { ParameterManager.shared.setParameter("HasReceivedRunningState", value: newValue) }
    }
    
    private var isSynchronized: Bool {
        get { ParameterManager.shared.getParameter("IsSynchronized", defaultValue: false) }
        set { ParameterManager.shared.setParameter("IsSynchronized", value: newValue) }
    }
    
    // Published properties to notify observers of changes
    // Remove these Published properties since we're using ParameterManager's published properties directly
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
        if !ParameterManager.shared.hasParameter("HasReceivedMode") {
            ParameterManager.shared.setParameter("HasReceivedMode", value: false)
        }
        if !ParameterManager.shared.hasParameter("HasReceivedRunningState") {
            ParameterManager.shared.setParameter("HasReceivedRunningState", value: false)
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
        hasReceivedMode = false
        hasReceivedRunningState = false
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
        connection = nil
        
        // Reset connection state
        isServerConnected = false
        connectionError = nil
        
        // Reset synchronization state
        hasReceivedMode = false
        hasReceivedRunningState = false
        isSynchronized = false
        
        // Clear message queue and processing state
        messageQueue.removeAll()
        isProcessingMessage = false
        
        // Reset other states
        currentMode = .manual
        isRunning = false
        receivedMessages.removeAll()
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
            
            // Add message to queue
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

            // MARK: – Safe Double reader (unaligned-safe + little-endian → host)
            func readDouble(at offset: Int) -> Double {
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

            // MARK: – Safe UInt32 reader (unaligned-safe + little-endian → host)
            func readUInt32(at offset: Int) -> UInt32 {
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

        // Handle mode and running state packets
        switch type {
        case 7: // SetAutonomous
            DispatchQueue.main.async {
                self.currentMode = .autonomous
                if !self.isSynchronized {
                    self.hasReceivedMode = true
                    self.checkSynchronization()
                }
                self.processNextMessage()
            }
            return
        case 8: // SetManual
            DispatchQueue.main.async {
                self.currentMode = .manual
                if !self.isSynchronized {
                    self.hasReceivedMode = true
                    self.checkSynchronization()
                }
                self.processNextMessage()
            }
            return
        case 9: // SetAutoAim
            DispatchQueue.main.async {
                self.currentMode = .autoAim
                if !self.isSynchronized {
                    self.hasReceivedMode = true
                    self.checkSynchronization()
                }
                self.processNextMessage()
            }
            return
        case 5: // Start
            DispatchQueue.main.async {
                self.isRunning = true
                if !self.isSynchronized {
                    self.hasReceivedRunningState = true
                    self.checkSynchronization()
                }
                self.processNextMessage()
            }
            return
        case 6: // Stop
            DispatchQueue.main.async {
                self.isRunning = false
                if !self.isSynchronized {
                    self.hasReceivedRunningState = true
                    self.checkSynchronization()
                }
                self.processNextMessage()
            }
            return
        default:
            // If not synchronized, ignore other packets
            if !self.isSynchronized {
                self.processNextMessage()
                return
            }
        }

        var messageText = ""
            switch type {
            case 0: // ServoCommand
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                messageText = "Received ServoCommand: x=\(x), y=\(y), z=\(z)"
            case 1: // TriggerCommand
                let flag = payload[0] != 0
                messageText = "Received TriggerCommand: \(flag)"
            case 2: // EscCommand
                let value = readDouble(at: 0)
                messageText = "Received EscCommand: \(value)"
            case 3: // TunePitch
                let p = readDouble(at: 0)
                let i = readDouble(at: 8)
                let d = readDouble(at: 16)
                messageText = "Received TunePitch: p=\(p), i=\(i), d=\(d)"
            case 4: // TuneYaw
                let p = readDouble(at: 0)
                let i = readDouble(at: 8)
                let d = readDouble(at: 16)
                messageText = "Received TuneYaw: p=\(p), i=\(i), d=\(d)"
            case 10: // SetOffset
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                messageText = "Received SetOffset: x=\(x), y=\(y), z=\(z)"
            case 11: // SetPitchIntegralLimit
                let limit = readDouble(at: 0)
                messageText = "Received SetPitchIntegralLimit: \(limit)"
            case 12: // SetYawIntegralLimit
                let limit = readDouble(at: 0)
                messageText = "Received SetYawIntegralLimit: \(limit)"
            case 13: // SetLaunchThreshold
                let eps = readDouble(at: 0)
                let n = payload[8]
                messageText = "Received SetLaunchThreshold: eps=\(eps), n=\(n)"
            case 14: // BboxPos
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                let w = readDouble(at: 24)
                messageText = "Received BboxPos: x=\(x), y=\(y), z=\(z), w=\(w)"

                // Process bounding box data
                self.boundingBoxHandler.processBoundingBoxData([x, y, z, w])
            case 15: // Tilt
                let roll = readDouble(at: 0)
                let pitch = readDouble(at: 8)
                let z = readDouble(at: 16)  // unused
                messageText = "Received Tilt: roll=\(roll), pitch=\(pitch)"
            case 16: // Log
                if let text = String(bytes: payload, encoding: .utf8) {
                    messageText = "Received Log: \(text)"
                }
            case 17: // Query
                messageText = "Received Query"
            case 18: // MotorOffset
                let offset = readDouble(at: 0)
                messageText = "Received MotorOffset: \(offset)"
            case 19: // FilteredBbox
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                messageText = "Received FilteredBbox: x=\(x), y=\(y), z=\(z)"
                
                // Process filtered bounding box data
                self.boundingBoxHandler.processFilteredBboxData([x, y, z])
            case 20: // SetCutoffFrequency
                let frequency = readDouble(at: 0)
                messageText = "Received SetCutoffFrequency: \(frequency)"
            case 21: // LaunchCounter
                let counter = Int32(readDouble(at: 0))
                messageText = "Received LaunchCounter: \(counter)"
            case 22: // SetStopThrottle
                let throttle = readDouble(at: 0)
                messageText = "Received SetStopThrottle: \(throttle)"
            case 23: // SetMaxConsecutiveNans
                let maxNans = readUInt32(at: 0)
                messageText = "Received SetMaxConsecutiveNans: \(maxNans)"
            case 24: // SetDefaultSpeed
                let speed = readDouble(at: 0)
                messageText = "Received SetDefaultSpeed: \(speed)"
            case 25: // CurrentState
                let mode = payload[0]
                let state = payload[1]
                let launchCounter = readUInt32(at: 2)
                let maxConsecutiveNans = readUInt32(at: 6)
                let targetX = readDouble(at: 10)
                let targetY = readDouble(at: 18)
                let stopThrottle = readDouble(at: 26)
                let motorOffset = readDouble(at: 34)
                let defaultSpeed = readDouble(at: 42)
                let cutoffFreq = readDouble(at: 50)
                
                messageText = "Received CurrentState: mode=\(mode), state=\(state), " +
                            "launchCounter=\(launchCounter), maxConsecutiveNans=\(maxConsecutiveNans), " +
                            "targetX=\(targetX), targetY=\(targetY), stopThrottle=\(stopThrottle), " +
                            "motorOffset=\(motorOffset), defaultSpeed=\(defaultSpeed), cutoffFreq=\(cutoffFreq)"
                
                // Update mode if needed
                DispatchQueue.main.async {
                    switch mode {
                    case 7: self.currentMode = .autonomous
                    case 8: self.currentMode = .manual
                    case 9: self.currentMode = .autoAim
                    default: break
                    }
                    
                    // Update running state
                    switch state {
                    case 5: self.isRunning = true
                    case 6: self.isRunning = false
                    default: break
                    }
                    
                    if !self.isSynchronized {
                        self.hasReceivedMode = true
                        self.hasReceivedRunningState = true
                        self.checkSynchronization()
                    }
                }
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
        if hasReceivedMode && hasReceivedRunningState {
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
        send(packet)
    }
    
    func sendStop() {
        let packet = DataPacket.stop()
        send(packet)
    }
    
    deinit {
        disconnect()
    }
} 
