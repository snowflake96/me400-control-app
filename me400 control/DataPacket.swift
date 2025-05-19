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
    case pitchAngle = 15
    case log = 16
}

enum DataPacketData {
    case text(String)
    case boolean(Bool)
    case double(Double)
    case vector3(x: Double, y: Double, z: Double)
    case vector4(x: Double, y: Double, z: Double, w: Double)
    case threshold(eps: Double, n: UInt8)
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
    
    static func setOffset(x: Double, y: Double) -> DataPacket {
        return DataPacket(type: .setOffset, data: .vector3(x: x, y: y, z: 0))
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
    
    // Create a pitch angle packet
    static func pitchAngle(_ value: Double) -> DataPacket {
        return DataPacket(type: .pitchAngle, data: .double(value))
    }
    
    // Create a log packet
    static func log(_ text: String) -> DataPacket {
        return DataPacket(type: .log, data: .text(text))
    }
} 
