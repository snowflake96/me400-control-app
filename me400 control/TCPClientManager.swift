import Foundation
import Network

class TCPClientManager: ObservableObject {
    private var connection: NWConnection?
    @Published var isConnected = false
    @Published var transmissionStatus: String = ""
    @Published var connectionError: String?
    @Published var serverStatus: String = ""
    @Published var receivedMessages: [String] = []
    
    func connect(to host: String, port: UInt16) {
        let endpoint = NWEndpoint.hostPort(host: NWEndpoint.Host(host), port: NWEndpoint.Port(integerLiteral: port))
        connection = NWConnection(to: endpoint, using: .tcp)
        
        connection?.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                switch state {
                case .ready:
                    self?.isConnected = true
                    self?.connectionError = nil
                    self?.transmissionStatus = "Connected"
                    self?.receiveNextMessage()
                case .failed(let error):
                    self?.isConnected = false
                    self?.connectionError = error.localizedDescription
                    self?.connection?.cancel()
                case .waiting(let error):
                    self?.isConnected = false
                    self?.connectionError = error.localizedDescription
                case .cancelled:
                    self?.isConnected = false
                    self?.connectionError = nil
                default:
                    break
                }
            }
        }
        
        connection?.start(queue: .main)
    }
    
    func disconnect() {
        connection?.cancel()
        connection = nil
        isConnected = false
        connectionError = nil
        serverStatus = ""
        transmissionStatus = "Disconnected"
    }
    
    private func receiveNextMessage() {
        connection?.receive(minimumIncompleteLength: 65, maximumLength: 65) { [weak self] packet, _, isComplete, error in
            guard let packet = packet, packet.count == 65 else {
                print("Failed to read 65-byte packet or connection closed.")
                return
            }

            let type = packet[0]
            let payload = packet[1..<65]

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

                // —— CHANGE HERE ——
                // Interpret bitPattern as little-endian (no swap needed on little-endian host)
                let hostBits = UInt64(littleEndian: bitPattern)

                return Double(bitPattern: hostBits)
            }

            var message = ""
            switch type {
            case 0: // ServoCommand (vector3)
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                message = "Received ServoCommand: x=\(x), y=\(y), z=\(z)"

            case 1: // TriggerCommand (boolean)
                let flag = payload[0] != 0
                message = "Received TriggerCommand: \(flag)"

            case 2: // EscCommand (double)
                let value = readDouble(at: 0)
                message = "Received EscCommand: \(value)"

            case 3: // TunePitch (vector3)
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                message = "Received TunePitch: x=\(x), y=\(y), z=\(z)"

            case 4: // TuneYaw (vector3)
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                message = "Received TuneYaw: x=\(x), y=\(y), z=\(z)"

            case 10: // SetOffset (vector3)
                let x = readDouble(at: 0)
                let y = readDouble(at: 8)
                let z = readDouble(at: 16)
                message = "Received SetOffset: x=\(x), y=\(y), z=\(z)"

            case 11: // SetPitchIntegralLimit (double)
                let limit = readDouble(at: 0)
                message = "Received SetPitchIntegralLimit: \(limit)"

            case 12: // SetYawIntegralLimit (double)
                let limit = readDouble(at: 0)
                message = "Received SetYawIntegralLimit: \(limit)"

            case 15: // PitchAngle (double)
                let angle = readDouble(at: 0)
                message = "Received PitchAngle: \(angle)"

            case 13: // SetLaunchThreshold (threshold)
                let eps = readDouble(at: 0)
                let n   = payload[8]
                message = "Received SetLaunchThreshold: eps=\(eps), n=\(n)"

            case 14: // BboxPos (vec4)
                let x1 = readDouble(at: 0)
                let y1 = readDouble(at: 8)
                let x2 = readDouble(at: 16)
                let y2 = readDouble(at: 24)
                message = "Received BboxPos: x1=\(x1), y1=\(y1), x2=\(x2), y2=\(y2)"

            case 16: // Log (text)
                let text = String(bytes: payload, encoding: .utf8) ?? "(invalid utf8)"
                message = "Received Log: \(text)"

            default:
                message = "Received type-only packet: type=\(type)"
            }

            DispatchQueue.main.async {
                self?.receivedMessages.append(message)
                // Keep only the last 10,000 messages
                if self?.receivedMessages.count ?? 0 > 10_000 {
                    self?.receivedMessages.removeFirst(self!.receivedMessages.count - 10_000)
                }
            }

            if let error = error {
                print("Receive error: \(error)")
                self?.handleConnectionError(error)
            } else if !isComplete {
                self?.receiveNextMessage()
            }
        }
    }
    
    private func handleConnectionError(_ error: Error) {
        DispatchQueue.main.async {
            self.isConnected = false
            self.connectionError = error.localizedDescription
            self.serverStatus = ""
            self.transmissionStatus = "Connection Error: \(error.localizedDescription)"
        }
        disconnect()
    }
    
    private func updateTransmissionStatus(_ packet: DataPacket, success: Bool) {
        let status = success ? "Success" : "Failed"
        var message: String = ""
        
        switch packet.type {
        case .servoCommand:
            switch packet.data {
            case .vector3(let x, let y, _):
                message = "PitchVel=\(String(format: "%.3f", x)), YawVel=\(String(format: "%.3f", y))"
            default:
                message = "Command"
            }
        case .tunePitch:
            if case .vector3(let p, let i, _) = packet.data {
                message = "Pitch PI: P=\(String(format: "%.2f", p)), I=\(String(format: "%.2f", i))"
            } else {
                message = "Pitch PI"
            }
        case .tuneYaw:
            if case .vector3(let p, let i, _) = packet.data {
                message = "Yaw PI: P=\(String(format: "%.2f", p)), I=\(String(format: "%.2f", i))"
            } else {
                message = "Yaw PI"
            }
        case .start:
            message = "Start"
        case .stop:
            message = "Stop"
        case .setAutonomous:
            message = "Set Autonomous"
        case .setManual:
            message = "Set Manual"
        case .setAutoAim:
            message = "Set Auto Aim"
        case .triggerCommand:
            if case .boolean(let value) = packet.data {
                message = value ? "Launch Start" : "Launch Stop"
            } else {
                message = "Trigger Command"
            }
        case .escCommand:
            if case .double(let value) = packet.data {
                message = "ESC Command: \(String(format: "%.3f", value))"
            } else {
                message = "ESC Command"
            }
        case .setPitchIntegralLimit:
            if case .double(let limit) = packet.data {
                message = "Set Pitch Integral Limit: \(String(format: "%.4f", limit))"
            } else {
                message = "Set Pitch Integral Limit"
            }
        case .setYawIntegralLimit:
            if case .double(let limit) = packet.data {
                message = "Set Yaw Integral Limit: \(String(format: "%.4f", limit))"
            } else {
                message = "Set Yaw Integral Limit"
            }
        case .setOffset:
            if case .vector3(let x, let y, _) = packet.data {
                message = "Set Offset: x=\(String(format: "%.2f", x)), y=\(String(format: "%.2f", y))"
            } else {
                message = "Set Offset"
            }
        case .bboxPos:
            if case .vector3(let x, let y, let z) = packet.data {
                message = "Bounding Box: x1=\(String(format: "%.2f", x)), y1=\(String(format: "%.2f", y)), x2=\(String(format: "%.2f", z))"
            } else {
                message = "Bounding Box Position"
            }
        @unknown default:
            message = "Unknown packet type"
        }
        
        transmissionStatus = "\(message) - \(status)"
    }
    
    func send(_ packet: DataPacket) -> Bool {
        guard isConnected else {
            updateTransmissionStatus(packet, success: false)
            return false
        }
        
        let data = packet.toBytes()
        
        // Start the send operation
        connection?.send(content: data, completion: .contentProcessed { [weak self] error in
            if let error = error {
                print("Send error: \(error)")
                self?.handleConnectionError(error)
                self?.updateTransmissionStatus(packet, success: false)
            } else {
                self?.updateTransmissionStatus(packet, success: true)
            }
        })
        
        return true
    }
    
//    // Convenience methods for different packet types
//    func sendservoCommand(_ text: String) -> Bool {
//        return send(DataPacket.servoCommand(x: 0, y: 0, z: 0))
//    }
//    
//    func sendTunePitch(_ value: Float) -> Bool {
//        return send(DataPacket.tunePitch(p: Double(value), i: 0, d: 0))
//    }
//    
//    func sendTuneYaw(_ value: Float) -> Bool {
//        return send(DataPacket.tuneYaw(p: Double(value), i: 0, d: 0))
//    }
//    
//    func sendVector3(x: Double, y: Double, z: Double) -> Bool {
//        return send(DataPacket.servoCommand(x: x, y: y, z: z))
//    }
//    
    deinit {
        disconnect()
    }
} 
