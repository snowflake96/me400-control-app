import Foundation

// MARK: - Protocol Constants
enum ProtocolConstants {
    static let packetSize = 65
    static let headerSize = 1
    static let payloadSize = 64
}

// MARK: - Packet Types
enum PacketType: UInt8, CaseIterable {
    // Control Commands
    case servoCommand = 0
    case triggerCommand = 1
    case escCommand = 2
    
    // PID Tuning
    case tunePitch = 3
    case tuneYaw = 4
    
    // System Control
    case start = 5
    case stop = 6
    
    // Mode Selection
    case setAutonomous = 7
    case setManual = 8
    case setAutoAim = 9
    
    // Configuration
    case setOffset = 10
    case setPitchIntegralLimit = 11
    case setYawIntegralLimit = 12
    case setLaunchThreshold = 13
    
    // Data Packets
    case bboxPos = 14
    case tilt = 15
    case log = 16
    case query = 17
    case motorOffset = 18
    case filteredBbox = 19
    case setCutoffFrequency = 20
    case launchCounter = 21
    case setStopThrottle = 22
    case setMaxConsecutiveNans = 23
    case setDefaultSpeed = 24
    case currentState = 25
    case setPitchIntegralThreshold = 26
    case setYawIntegralThreshold = 27
    case pidSettings = 28
    
    var description: String {
        switch self {
        case .servoCommand: return "Servo Command"
        case .triggerCommand: return "Trigger Command"
        case .escCommand: return "ESC Command"
        case .tunePitch: return "Tune Pitch"
        case .tuneYaw: return "Tune Yaw"
        case .start: return "Start"
        case .stop: return "Stop"
        case .setAutonomous: return "Set Autonomous"
        case .setManual: return "Set Manual"
        case .setAutoAim: return "Set Auto Aim"
        case .setOffset: return "Set Offset"
        case .setPitchIntegralLimit: return "Set Pitch Integral Limit"
        case .setYawIntegralLimit: return "Set Yaw Integral Limit"
        case .setLaunchThreshold: return "Set Launch Threshold"
        case .bboxPos: return "Bounding Box Position"
        case .tilt: return "Tilt"
        case .log: return "Log"
        case .query: return "Query"
        case .motorOffset: return "Motor Offset"
        case .filteredBbox: return "Filtered Bounding Box"
        case .setCutoffFrequency: return "Set Cutoff Frequency"
        case .launchCounter: return "Launch Counter"
        case .setStopThrottle: return "Set Stop Throttle"
        case .setMaxConsecutiveNans: return "Set Max Consecutive NANs"
        case .setDefaultSpeed: return "Set Default Speed"
        case .currentState: return "Current State"
        case .setPitchIntegralThreshold: return "Set Pitch Integral Threshold"
        case .setYawIntegralThreshold: return "Set Yaw Integral Threshold"
        case .pidSettings: return "PID Settings"
        }
    }
}

// MARK: - Mode Enums
enum DrivingMode: UInt8 {
    case autonomous = 7
    case manual = 8
    case autoAim = 9
    
    var displayName: String {
        switch self {
        case .autonomous: return "Autonomous"
        case .manual: return "Manual"
        case .autoAim: return "Auto Aim"
        }
    }
}

enum RunningState: UInt8 {
    case running = 5
    case stopped = 6
    
    var isRunning: Bool {
        return self == .running
    }
}

// MARK: - Packet Payloads
protocol PacketPayload {
    func encode() -> Data
}

struct Vector3Payload: PacketPayload {
    let x: Double
    let y: Double
    let z: Double
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        
        var xLE = x.bitPattern.littleEndian
        var yLE = y.bitPattern.littleEndian
        var zLE = z.bitPattern.littleEndian
        
        data.replaceSubrange(0..<8, with: Data(bytes: &xLE, count: 8))
        data.replaceSubrange(8..<16, with: Data(bytes: &yLE, count: 8))
        data.replaceSubrange(16..<24, with: Data(bytes: &zLE, count: 8))
        
        return data
    }
}

struct Vector4Payload: PacketPayload {
    let x: Double
    let y: Double
    let z: Double
    let w: Double
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        
        var xLE = x.bitPattern.littleEndian
        var yLE = y.bitPattern.littleEndian
        var zLE = z.bitPattern.littleEndian
        var wLE = w.bitPattern.littleEndian
        
        data.replaceSubrange(0..<8, with: Data(bytes: &xLE, count: 8))
        data.replaceSubrange(8..<16, with: Data(bytes: &yLE, count: 8))
        data.replaceSubrange(16..<24, with: Data(bytes: &zLE, count: 8))
        data.replaceSubrange(24..<32, with: Data(bytes: &wLE, count: 8))
        
        return data
    }
}

struct BooleanPayload: PacketPayload {
    let value: Bool
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        data[0] = value ? 1 : 0
        return data
    }
}

struct DoublePayload: PacketPayload {
    let value: Double
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        var littleEndianValue = value.bitPattern.littleEndian
        data.replaceSubrange(0..<8, with: Data(bytes: &littleEndianValue, count: 8))
        return data
    }
}

struct UInt32Payload: PacketPayload {
    let value: UInt32
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        var littleEndianValue = value.littleEndian
        data.replaceSubrange(0..<4, with: Data(bytes: &littleEndianValue, count: 4))
        return data
    }
}

struct UInt8Payload: PacketPayload {
    let value: UInt8
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        data[0] = value
        return data
    }
}

struct TextPayload: PacketPayload {
    let text: String
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        if let textData = text.data(using: .utf8) {
            let length = min(textData.count, ProtocolConstants.payloadSize - 1)
            data.replaceSubrange(0..<length, with: textData.prefix(length))
        }
        return data
    }
}

struct ThresholdPayload: PacketPayload {
    let epsilon: Double
    let n: UInt8
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        var epsLE = epsilon.bitPattern.littleEndian
        data.replaceSubrange(0..<8, with: Data(bytes: &epsLE, count: 8))
        data[8] = n
        return data
    }
}

struct CurrentStatePayload: PacketPayload {
    let mode: DrivingMode
    let state: RunningState
    let foundOffset: Bool
    let useInterpolation: Bool
    let maxNans: UInt8
    let n: UInt8
    let eps: Double
    let targetX: Double
    let targetY: Double
    let stopThrottle: Double
    let motorOffset: Double
    let defaultSpeed: Double
    let cutoffFreq: Double
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        
        data[0] = mode.rawValue
        data[1] = state.rawValue
        data[2] = foundOffset ? 1 : 0
        data[3] = useInterpolation ? 1 : 0
        data[4] = maxNans
        data[5] = n
        
        var epsLE = eps.bitPattern.littleEndian
        var txLE = targetX.bitPattern.littleEndian
        var tyLE = targetY.bitPattern.littleEndian
        var stLE = stopThrottle.bitPattern.littleEndian
        var moLE = motorOffset.bitPattern.littleEndian
        var dsLE = defaultSpeed.bitPattern.littleEndian
        var cfLE = cutoffFreq.bitPattern.littleEndian
        
        data.replaceSubrange(6..<14, with: Data(bytes: &epsLE, count: 8))
        data.replaceSubrange(14..<22, with: Data(bytes: &txLE, count: 8))
        data.replaceSubrange(22..<30, with: Data(bytes: &tyLE, count: 8))
        data.replaceSubrange(30..<38, with: Data(bytes: &stLE, count: 8))
        data.replaceSubrange(38..<46, with: Data(bytes: &moLE, count: 8))
        data.replaceSubrange(46..<54, with: Data(bytes: &dsLE, count: 8))
        data.replaceSubrange(54..<62, with: Data(bytes: &cfLE, count: 8))
        
        return data
    }
}

struct PIDSettingsPayload: PacketPayload {
    struct PIDParameters {
        let p: Double
        let i: Double
        let iLimit: Double
        let iThreshold: Double
    }
    
    let pitch: PIDParameters
    let yaw: PIDParameters
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.payloadSize)
        
        // Pitch parameters (32 bytes)
        var pitchP = pitch.p.bitPattern.littleEndian
        var pitchI = pitch.i.bitPattern.littleEndian
        var pitchILimit = pitch.iLimit.bitPattern.littleEndian
        var pitchIThreshold = pitch.iThreshold.bitPattern.littleEndian
        
        data.replaceSubrange(0..<8, with: Data(bytes: &pitchP, count: 8))
        data.replaceSubrange(8..<16, with: Data(bytes: &pitchI, count: 8))
        data.replaceSubrange(16..<24, with: Data(bytes: &pitchILimit, count: 8))
        data.replaceSubrange(24..<32, with: Data(bytes: &pitchIThreshold, count: 8))
        
        // Yaw parameters (32 bytes)
        var yawP = yaw.p.bitPattern.littleEndian
        var yawI = yaw.i.bitPattern.littleEndian
        var yawILimit = yaw.iLimit.bitPattern.littleEndian
        var yawIThreshold = yaw.iThreshold.bitPattern.littleEndian
        
        data.replaceSubrange(32..<40, with: Data(bytes: &yawP, count: 8))
        data.replaceSubrange(40..<48, with: Data(bytes: &yawI, count: 8))
        data.replaceSubrange(48..<56, with: Data(bytes: &yawILimit, count: 8))
        data.replaceSubrange(56..<64, with: Data(bytes: &yawIThreshold, count: 8))
        
        return data
    }
}

// MARK: - Packet Structure
struct Packet {
    let type: PacketType
    let payload: PacketPayload
    
    func encode() -> Data {
        var data = Data(count: ProtocolConstants.packetSize)
        data[0] = type.rawValue
        let payloadData = payload.encode()
        data.replaceSubrange(1..<ProtocolConstants.packetSize, with: payloadData)
        return data
    }
}

// MARK: - Packet Factory
enum PacketFactory {
    // Control Commands
    static func servoCommand(x: Double, y: Double, z: Double) -> Packet {
        Packet(type: .servoCommand, payload: Vector3Payload(x: x, y: y, z: z))
    }
    
    static func triggerCommand(_ value: Bool) -> Packet {
        Packet(type: .triggerCommand, payload: BooleanPayload(value: value))
    }
    
    static func escCommand(_ value: Double) -> Packet {
        Packet(type: .escCommand, payload: DoublePayload(value: value))
    }
    
    // PID Tuning
    static func tunePitch(p: Double, i: Double, d: Double = 0.0) -> Packet {
        Packet(type: .tunePitch, payload: Vector3Payload(x: p, y: i, z: d))
    }
    
    static func tuneYaw(p: Double, i: Double, d: Double = 0.0) -> Packet {
        Packet(type: .tuneYaw, payload: Vector3Payload(x: p, y: i, z: d))
    }
    
    // System Control
    static func start() -> Packet {
        Packet(type: .start, payload: TextPayload(text: ""))
    }
    
    static func stop() -> Packet {
        Packet(type: .stop, payload: TextPayload(text: ""))
    }
    
    // Mode Selection
    static func setMode(_ mode: DrivingMode) -> Packet {
        let type: PacketType
        switch mode {
        case .autonomous: type = .setAutonomous
        case .manual: type = .setManual
        case .autoAim: type = .setAutoAim
        }
        return Packet(type: type, payload: TextPayload(text: ""))
    }
    
    // Configuration
    static func setOffset(x: Double, y: Double, z: Double) -> Packet {
        Packet(type: .setOffset, payload: Vector3Payload(x: x, y: y, z: z))
    }
    
    static func setPitchIntegralLimit(_ value: Double) -> Packet {
        Packet(type: .setPitchIntegralLimit, payload: DoublePayload(value: value))
    }
    
    static func setYawIntegralLimit(_ value: Double) -> Packet {
        Packet(type: .setYawIntegralLimit, payload: DoublePayload(value: value))
    }
    
    static func setPitchIntegralThreshold(_ value: Double) -> Packet {
        Packet(type: .setPitchIntegralThreshold, payload: DoublePayload(value: value))
    }
    
    static func setYawIntegralThreshold(_ value: Double) -> Packet {
        Packet(type: .setYawIntegralThreshold, payload: DoublePayload(value: value))
    }
    
    static func setLaunchThreshold(epsilon: Double, n: UInt8) -> Packet {
        Packet(type: .setLaunchThreshold, payload: ThresholdPayload(epsilon: epsilon, n: n))
    }
    
    static func setCutoffFrequency(_ value: Double) -> Packet {
        Packet(type: .setCutoffFrequency, payload: DoublePayload(value: value))
    }
    
    static func setStopThrottle(_ value: Double) -> Packet {
        Packet(type: .setStopThrottle, payload: DoublePayload(value: value))
    }
    
    static func setMaxConsecutiveNans(_ value: UInt8) -> Packet {
        Packet(type: .setMaxConsecutiveNans, payload: UInt8Payload(value: value))
    }
    
    static func setDefaultSpeed(_ value: Double) -> Packet {
        Packet(type: .setDefaultSpeed, payload: DoublePayload(value: value))
    }
    
    static func motorOffset(_ value: Double) -> Packet {
        Packet(type: .motorOffset, payload: DoublePayload(value: value))
    }
    
    // Query
    static func query() -> Packet {
        Packet(type: .query, payload: TextPayload(text: ""))
    }
}

// MARK: - Packet Decoder
enum PacketDecoder {
    static func decode(from data: Data) -> (type: PacketType, payload: Data)? {
        guard data.count == ProtocolConstants.packetSize else { return nil }
        guard let type = PacketType(rawValue: data[0]) else { return nil }
        let payload = data[1..<ProtocolConstants.packetSize]
        return (type, Data(payload))
    }
    
    static func readDouble(at offset: Int, from data: Data) -> Double {
        let byteCount = MemoryLayout<UInt64>.size
        guard offset >= 0, offset + byteCount <= data.count else {
            return 0
        }

        // Use memcpy to safely copy bytes without alignment requirements
        let bitPattern: UInt64 = data.withUnsafeBytes { rawBuf in
            let src = rawBuf.baseAddress!.advanced(by: offset)
            var local: UInt64 = 0
            memcpy(&local, src, byteCount)
            return local
        }

        let hostBits = UInt64(littleEndian: bitPattern)
        return Double(bitPattern: hostBits)
    }
    
    static func readUInt32(at offset: Int, from data: Data) -> UInt32 {
        let byteCount = MemoryLayout<UInt32>.size
        guard offset >= 0, offset + byteCount <= data.count else {
            return 0
        }

        // Use memcpy to safely copy bytes without alignment requirements
        let bitPattern: UInt32 = data.withUnsafeBytes { rawBuf in
            let src = rawBuf.baseAddress!.advanced(by: offset)
            var local: UInt32 = 0
            memcpy(&local, src, byteCount)
            return local
        }

        return UInt32(littleEndian: bitPattern)
    }
    
    static func readUInt8(at offset: Int, from data: Data) -> UInt8 {
        guard offset >= 0, offset < data.count else {
            return 0
        }
        return data[offset]
    }
    
    static func readString(from data: Data) -> String {
        if let nullIndex = data.firstIndex(of: 0) {
            return String(data: data[..<nullIndex], encoding: .utf8) ?? ""
        }
        return String(data: data, encoding: .utf8) ?? ""
    }
} 