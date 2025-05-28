import Foundation

enum DataPacketType: UInt8 {
    case servoCommand = 0
    case triggerCommand = 1
    case escCommand = 2
    case tunePitch = 3
    case tuneYaw = 4
    case start = 5
    case stop = 6
    case setAutonomous = 7
    case setManual = 8
    case setAutoAim = 9
    case setOffset = 10
    case setPitchIntegralLimit = 11
    case setYawIntegralLimit = 12
    case setLaunchThreshold = 13
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
}

enum DataPacketData {
    case vector3(x: Double, y: Double, z: Double)
    case vector4(x: Double, y: Double, z: Double, w: Double)
    case boolean(Bool)
    case double(Double)
    case text(String)
    case threshold(eps: Double, n: UInt8)
    case uint32(UInt32)  // For SetMaxConsecutiveNans
    case currentState(mode: UInt8, state: UInt8, launchCounter: UInt32, maxConsecutiveNans: UInt32, 
                     targetX: Double, targetY: Double, stopThrottle: Double, motorOffset: Double,
                     defaultSpeed: Double, cutoffFreq: Double)
}

struct DataPacket {
    let type: DataPacketType
    let data: DataPacketData
    
    // Convert to raw bytes matching the C++ struct format
    func toBytes() -> Data {
        var data = Data(count: 65)  // Fixed size of 65 bytes
        data[0] = type.rawValue  // First byte is type
        
        switch self.data {
        case .text(let text):
            let textData = text.data(using: .utf8) ?? Data()
            let textLength = min(textData.count, 63)  // Leave space for null terminator
            data.replaceSubrange(1..<(1 + textLength), with: textData.prefix(textLength))
            
        case .boolean(let value):
            data[1] = value ? 1 : 0
            
        case .double(let value):
            var v = value
            let vData = Data(bytes: &v, count: MemoryLayout<Double>.size)
            data.replaceSubrange(1..<9, with: vData)
            
        case .vector3(let x, let y, let z):
            var xV = x, yV = y, zV = z
            let xData = Data(bytes: &xV, count: MemoryLayout<Double>.size)
            let yData = Data(bytes: &yV, count: MemoryLayout<Double>.size)
            let zData = Data(bytes: &zV, count: MemoryLayout<Double>.size)
            data.replaceSubrange(1..<9, with: xData)
            data.replaceSubrange(9..<17, with: yData)
            data.replaceSubrange(17..<25, with: zData)
            
        case .vector4(let x, let y, let z, let w):
            var xV = x, yV = y, zV = z, wV = w
            let xData = Data(bytes: &xV, count: MemoryLayout<Double>.size)
            let yData = Data(bytes: &yV, count: MemoryLayout<Double>.size)
            let zData = Data(bytes: &zV, count: MemoryLayout<Double>.size)
            let wData = Data(bytes: &wV, count: MemoryLayout<Double>.size)
            data.replaceSubrange(1..<9, with: xData)
            data.replaceSubrange(9..<17, with: yData)
            data.replaceSubrange(17..<25, with: zData)
            data.replaceSubrange(25..<33, with: wData)
            
        case .threshold(let eps, let n):
            var epsV = eps
            let epsData = Data(bytes: &epsV, count: MemoryLayout<Double>.size)
            data.replaceSubrange(1..<9, with: epsData)
            data[9] = n
            
        case .uint32(let value):
            var v = value
            let vData = Data(bytes: &v, count: MemoryLayout<UInt32>.size)
            data.replaceSubrange(1..<5, with: vData)
            
        case .currentState(let mode, let state, let launchCounter, let maxConsecutiveNans,
                         let targetX, let targetY, let stopThrottle, let motorOffset,
                         let defaultSpeed, let cutoffFreq):
            data[1] = mode
            data[2] = state
            
            var lc = launchCounter
            var mcn = maxConsecutiveNans
            var tx = targetX
            var ty = targetY
            var st = stopThrottle
            var mo = motorOffset
            var ds = defaultSpeed
            var cf = cutoffFreq
            
            let lcData = Data(bytes: &lc, count: MemoryLayout<UInt32>.size)
            let mcnData = Data(bytes: &mcn, count: MemoryLayout<UInt32>.size)
            let txData = Data(bytes: &tx, count: MemoryLayout<Double>.size)
            let tyData = Data(bytes: &ty, count: MemoryLayout<Double>.size)
            let stData = Data(bytes: &st, count: MemoryLayout<Double>.size)
            let moData = Data(bytes: &mo, count: MemoryLayout<Double>.size)
            let dsData = Data(bytes: &ds, count: MemoryLayout<Double>.size)
            let cfData = Data(bytes: &cf, count: MemoryLayout<Double>.size)
            
            data.replaceSubrange(3..<7, with: lcData)
            data.replaceSubrange(7..<11, with: mcnData)
            data.replaceSubrange(11..<19, with: txData)
            data.replaceSubrange(19..<27, with: tyData)
            data.replaceSubrange(27..<35, with: stData)
            data.replaceSubrange(35..<43, with: moData)
            data.replaceSubrange(43..<51, with: dsData)
            data.replaceSubrange(51..<59, with: cfData)
            
        default:
            break
        }
        
        return data
    }
    
    // Create a servo command packet
    static func servoCommand(x: Double, y: Double, z: Double) -> DataPacket {
        return DataPacket(type: .servoCommand, data: .vector3(x: x, y: y, z: z))
    }
    
    // Create a trigger command packet
    static func triggerCommand(_ value: Bool) -> DataPacket {
        return DataPacket(type: .triggerCommand, data: .boolean(value))
    }
    
    // Create an esc command packet
    static func escCommand(_ value: Double) -> DataPacket {
        return DataPacket(type: .escCommand, data: .double(value))
    }
    
    // Create a tune pitch packet
    static func tunePitch(p: Double, i: Double, d: Double) -> DataPacket {
        return DataPacket(type: .tunePitch, data: .vector3(x: p, y: i, z: d))
    }
    
    // Create a tune yaw packet
    static func tuneYaw(p: Double, i: Double, d: Double) -> DataPacket {
        return DataPacket(type: .tuneYaw, data: .vector3(x: p, y: i, z: d))
    }
    
    static func start() -> DataPacket {
        return DataPacket(type: .start, data: .text(""))
    }
    
    static func stop() -> DataPacket {
        return DataPacket(type: .stop, data: .text(""))
    }
    
    static func setAutonomous() -> DataPacket {
        return DataPacket(type: .setAutonomous, data: .text(""))
    }
    
    static func setManual() -> DataPacket {
        return DataPacket(type: .setManual, data: .text(""))
    }
    
    static func setAutoAim() -> DataPacket {
        return DataPacket(type: .setAutoAim, data: .text(""))
    }
    
    // Create a set offset packet
    static func setOffset(x: Double, y: Double, z: Double) -> DataPacket {
        return DataPacket(type: .setOffset, data: .vector3(x: x, y: y, z: z))
    }
    
    // Create a set pitch integral limit packet
    static func setPitchIntegralLimit(_ value: Double) -> DataPacket {
        return DataPacket(type: .setPitchIntegralLimit, data: .double(value))
    }
    
    // Create a set yaw integral limit packet
    static func setYawIntegralLimit(_ value: Double) -> DataPacket {
        return DataPacket(type: .setYawIntegralLimit, data: .double(value))
    }
    
    // Create a set launch threshold packet
    static func setLaunchThreshold(eps: Double, n: UInt8) -> DataPacket {
        return DataPacket(type: .setLaunchThreshold, data: .threshold(eps: eps, n: n))
    }
    
    // Create a bounding box position packet
    static func bboxPos(x: Double, y: Double, z: Double, w: Double) -> DataPacket {
        return DataPacket(type: .bboxPos, data: .vector4(x: x, y: y, z: z, w: w))
    }
    
    // Create a tilt packet (roll, pitch)
    static func tilt(roll: Double, pitch: Double) -> DataPacket {
        return DataPacket(type: .tilt, data: .vector3(x: roll, y: pitch, z: 0.0))
    }
    
    // Create a log packet
    static func log(_ text: String) -> DataPacket {
        return DataPacket(type: .log, data: .text(text))
    }
    
    // Create a query packet
    static func query() -> DataPacket {
        return DataPacket(type: .query, data: .text(""))
    }
    
    // Create a motor offset packet
    static func motorOffset(_ value: Double) -> DataPacket {
        return DataPacket(type: .motorOffset, data: .double(value))
    }
    
    // Create a filtered bounding box packet
    static func filteredBbox(x: Double, y: Double, z: Double) -> DataPacket {
        return DataPacket(type: .filteredBbox, data: .vector3(x: x, y: y, z: z))
    }
    
    // Create a set cutoff frequency packet
    static func setCutoffFrequency(_ value: Double) -> DataPacket {
        return DataPacket(type: .setCutoffFrequency, data: .double(value))
    }
    
    // Create a launch counter packet
    static func launchCounter(_ value: Int32) -> DataPacket {
        return DataPacket(type: .launchCounter, data: .double(Double(value)))
    }
    
    // Create a set stop throttle packet
    static func setStopThrottle(_ value: Double) -> DataPacket {
        return DataPacket(type: .setStopThrottle, data: .double(value))
    }
    
    // Create a set max consecutive NaNs packet
    static func setMaxConsecutiveNans(_ value: UInt32) -> DataPacket {
        return DataPacket(type: .setMaxConsecutiveNans, data: .uint32(value))
    }
    
    // Create a set default speed packet
    static func setDefaultSpeed(_ value: Double) -> DataPacket {
        return DataPacket(type: .setDefaultSpeed, data: .double(value))
    }
} 
 