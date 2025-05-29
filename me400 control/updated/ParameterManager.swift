import Foundation
import SwiftUI

// Add this struct at the top of the file
struct BoundingBoxData {
    let centerX: Double
    let centerY: Double
    let width: Double
    let height: Double
    let hasDetection: Bool
}

class ParameterManager: ObservableObject {
    static let shared = ParameterManager()
    
    // Camera parameters
    @Published var cameraOffsetX: Double = 0.0 {
        didSet { saveParameter("CameraOffsetX", value: cameraOffsetX) }
    }
    @Published var cameraOffsetY: Double = 0.0 {
        didSet { saveParameter("CameraOffsetY", value: cameraOffsetY) }
    }
    @Published var cameraOffsetZ: Double = 0.0 {
        didSet { saveParameter("CameraOffsetZ", value: cameraOffsetZ) }
    }
    @Published var cameraPitch: Double = 0.0 {
        didSet { saveParameter("CameraPitch", value: cameraPitch) }
    }
    @Published var cameraYaw: Double = 0.0 {
        didSet { saveParameter("CameraYaw", value: cameraYaw) }
    }
    
    // Mode parameter
    @Published var selectedMode: String = "Manual" {
        didSet { 
            saveParameter("SelectedMode", value: selectedMode)
            // Also update currentMode to maintain consistency
            currentMode = selectedMode.lowercased()
        }
    }
    
    // Pipe parameters
    @Published var pipeLocation: Double = 1.5 {
        didSet { saveParameter("PipeLocation", value: pipeLocation) }
    }
    
    // Bounding box parameters
    @Published private(set) var updateCounter: Int = 0
    @Published private(set) var hasDetection: Bool = false
    @Published private(set) var boundingBoxData: BoundingBoxData?
    
    // Tilt parameters (roll and pitch from server)
    @Published var tiltRoll: Double = 0.0 {
        didSet { saveParameter("TiltRoll", value: tiltRoll) }
    }
    @Published var tiltPitch: Double = 0.0 {
        didSet { saveParameter("TiltPitch", value: tiltPitch) }
    }
    
    // Launch counter from server
    @Published var launchCounter: UInt32? = nil {
        didSet { 
            if let counter = launchCounter {
                saveParameter("LaunchCounter", value: counter)
            }
        }
    }
    
    // Launch threshold N parameter (UInt8)
    @Published var launchThresholdN: UInt8 = 5 {
        didSet { saveParameter("LaunchThresholdN", value: launchThresholdN) }
    }
    
    // Last log message from server
    @Published var lastLogMessage: String = "" {
        didSet { saveParameter("LastLogMessage", value: lastLogMessage) }
    }
    
    // PID Control Parameters
    // Pitch Control
    @Published var pitchP: Double = 10.0 {
        didSet { saveParameter("PitchP", value: pitchP) }
    }
    @Published var pitchI: Double = 0.0 {
        didSet { saveParameter("PitchI", value: pitchI) }
    }
    @Published var pitchIntegralLimit: Double = 1.0 {
        didSet { saveParameter("PitchIntegralLimit", value: pitchIntegralLimit) }
    }
    @Published var pitchIntegralThreshold: Double = 0.025 {
        didSet { saveParameter("PitchIntegralThreshold", value: pitchIntegralThreshold) }
    }
    @Published var pitchPStepSize: Double = 1.0 {
        didSet { saveParameter("PitchPStepSize", value: pitchPStepSize) }
    }
    @Published var pitchIStepSize: Double = 0.1 {
        didSet { saveParameter("PitchIStepSize", value: pitchIStepSize) }
    }
    @Published var pitchIntegralLimitStepSize: Double = 0.1 {
        didSet { saveParameter("PitchIntegralLimitStepSize", value: pitchIntegralLimitStepSize) }
    }
    @Published var pitchIntegralThresholdStepSize: Double = 0.01 {
        didSet { saveParameter("PitchIntegralThresholdStepSize", value: pitchIntegralThresholdStepSize) }
    }
    
    // Yaw Control
    @Published var yawP: Double = 10.0 {
        didSet { saveParameter("YawP", value: yawP) }
    }
    @Published var yawI: Double = 0.0 {
        didSet { saveParameter("YawI", value: yawI) }
    }
    @Published var yawIntegralLimit: Double = 1.0 {
        didSet { saveParameter("YawIntegralLimit", value: yawIntegralLimit) }
    }
    @Published var yawIntegralThreshold: Double = 0.025 {
        didSet { saveParameter("YawIntegralThreshold", value: yawIntegralThreshold) }
    }
    @Published var yawPStepSize: Double = 1.0 {
        didSet { saveParameter("YawPStepSize", value: yawPStepSize) }
    }
    @Published var yawIStepSize: Double = 0.1 {
        didSet { saveParameter("YawIStepSize", value: yawIStepSize) }
    }
    @Published var yawIntegralLimitStepSize: Double = 0.1 {
        didSet { saveParameter("YawIntegralLimitStepSize", value: yawIntegralLimitStepSize) }
    }
    @Published var yawIntegralThresholdStepSize: Double = 0.01 {
        didSet { saveParameter("YawIntegralThresholdStepSize", value: yawIntegralThresholdStepSize) }
    }
    
    // Server parameters
    @Published var serverConnected: Bool = false {
        didSet { saveParameter("ServerConnected", value: serverConnected) }
    }
    @Published var serverError: String = "" {
        didSet { saveParameter("ServerError", value: serverError) }
    }
    @AppStorage("ServerHost") var serverHost: String = "localhost"
    @AppStorage("ServerPort") var serverPort: String = "12345"
    @Published var receivedMessages: [String] = [] {
        didSet { saveParameter("ReceivedMessages", value: receivedMessages) }
    }
    @Published var currentMode: String = "manual" {
        didSet { saveParameter("CurrentMode", value: currentMode) }
    }
    @Published var isRunning: Bool = false {
        didSet { saveParameter("IsRunning", value: isRunning) }
    }
    @Published var hasReceivedMode: Bool = false {
        didSet { saveParameter("HasReceivedMode", value: hasReceivedMode) }
    }
    @Published var hasReceivedRunningState: Bool = false {
        didSet { saveParameter("HasReceivedRunningState", value: hasReceivedRunningState) }
    }
    @Published var isSynchronized: Bool = false {
        didSet { saveParameter("IsSynchronized", value: isSynchronized) }
    }
    
    // Message queue handling - removed from here as it should be in ServerCommunicationManager
    @Published var isProcessingMessage: Bool = false {
        didSet { saveParameter("IsProcessingMessage", value: isProcessingMessage) }
    }
    
    private var parameters: [String: Any] = [:]
    // Changed to concurrent queue with proper label
    private let parametersQueue = DispatchQueue(label: "com.me400.parametermanager", attributes: .concurrent)
    
    private init() {
        loadAllParameters()
    }
    
    // MARK: - Persistence Methods
    
    private func saveParameter<T>(_ key: String, value: T) {
        // First update the in-memory parameters with barrier for thread safety
        parametersQueue.async(flags: .barrier) {
            self.parameters[key] = value
        }
        
        // Then save to UserDefaults on the main thread
        DispatchQueue.main.async {
            // Save to UserDefaults if the value is Codable
            if let codableValue = value as? (any Codable) {
                if let encoded = try? JSONEncoder().encode(codableValue) {
                    UserDefaults.standard.set(encoded, forKey: key)
                }
            } else if let stringValue = value as? String {
                UserDefaults.standard.set(stringValue, forKey: key)
            } else if let boolValue = value as? Bool {
                UserDefaults.standard.set(boolValue, forKey: key)
            } else if let doubleValue = value as? Double {
                UserDefaults.standard.set(doubleValue, forKey: key)
            } else if let intValue = value as? Int {
                UserDefaults.standard.set(intValue, forKey: key)
            } else if let uint32Value = value as? UInt32 {
                UserDefaults.standard.set(uint32Value, forKey: key)
            } else if let uint8Value = value as? UInt8 {
                UserDefaults.standard.set(uint8Value, forKey: key)
            } else if let arrayValue = value as? [String] {
                UserDefaults.standard.set(arrayValue, forKey: key)
            }
        }
    }
    
    private func loadParameter<T>(_ key: String, defaultValue: T) -> T {
        if let value = UserDefaults.standard.object(forKey: key) as? T {
            return value
        }
        return defaultValue
    }
    
    private func loadAllParameters() {
        // Load all parameters from UserDefaults
        cameraOffsetX = loadParameter("CameraOffsetX", defaultValue: 0.0)
        cameraOffsetY = loadParameter("CameraOffsetY", defaultValue: 0.0)
        cameraOffsetZ = loadParameter("CameraOffsetZ", defaultValue: 0.0)
        cameraPitch = loadParameter("CameraPitch", defaultValue: 0.0)
        cameraYaw = loadParameter("CameraYaw", defaultValue: 0.0)
        pipeLocation = loadParameter("PipeLocation", defaultValue: 1.5)
        serverHost = loadParameter("ServerHost", defaultValue: "localhost")
        serverPort = loadParameter("ServerPort", defaultValue: "12345")
        serverConnected = loadParameter("ServerConnected", defaultValue: false)
        serverError = loadParameter("ServerError", defaultValue: "")
        receivedMessages = loadParameter("ReceivedMessages", defaultValue: [])
        currentMode = loadParameter("CurrentMode", defaultValue: "manual")
        isRunning = loadParameter("IsRunning", defaultValue: false)
        hasReceivedMode = loadParameter("HasReceivedMode", defaultValue: false)
        hasReceivedRunningState = loadParameter("HasReceivedRunningState", defaultValue: false)
        isSynchronized = loadParameter("IsSynchronized", defaultValue: false)
        isProcessingMessage = loadParameter("IsProcessingMessage", defaultValue: false)
        selectedMode = loadParameter("SelectedMode", defaultValue: "Manual")
        tiltRoll = loadParameter("TiltRoll", defaultValue: 0.0)
        tiltPitch = loadParameter("TiltPitch", defaultValue: 0.0)
        launchCounter = loadParameter("LaunchCounter", defaultValue: nil)
        launchThresholdN = loadParameter("LaunchThresholdN", defaultValue: 5)
        lastLogMessage = loadParameter("LastLogMessage", defaultValue: "")
        pitchP = loadParameter("PitchP", defaultValue: 10.0)
        pitchI = loadParameter("PitchI", defaultValue: 0.0)
        pitchIntegralLimit = loadParameter("PitchIntegralLimit", defaultValue: 1.0)
        pitchIntegralThreshold = loadParameter("PitchIntegralThreshold", defaultValue: 0.025)
        pitchPStepSize = loadParameter("PitchPStepSize", defaultValue: 1.0)
        pitchIStepSize = loadParameter("PitchIStepSize", defaultValue: 0.1)
        pitchIntegralLimitStepSize = loadParameter("PitchIntegralLimitStepSize", defaultValue: 0.1)
        pitchIntegralThresholdStepSize = loadParameter("PitchIntegralThresholdStepSize", defaultValue: 0.01)
        yawP = loadParameter("YawP", defaultValue: 10.0)
        yawI = loadParameter("YawI", defaultValue: 0.0)
        yawIntegralLimit = loadParameter("YawIntegralLimit", defaultValue: 1.0)
        yawIntegralThreshold = loadParameter("YawIntegralThreshold", defaultValue: 0.025)
        yawPStepSize = loadParameter("YawPStepSize", defaultValue: 1.0)
        yawIStepSize = loadParameter("YawIStepSize", defaultValue: 0.1)
        yawIntegralLimitStepSize = loadParameter("YawIntegralLimitStepSize", defaultValue: 0.1)
        yawIntegralThresholdStepSize = loadParameter("YawIntegralThresholdStepSize", defaultValue: 0.01)
        
        // Load all custom parameters with thread safety
        for key in UserDefaults.standard.dictionaryRepresentation().keys {
            if let value = UserDefaults.standard.object(forKey: key) {
                parametersQueue.async(flags: .barrier) {
                    self.parameters[key] = value
                }
            }
        }
    }
    
    // MARK: - Parameter Management Methods
    
    func setParameter<T>(_ key: String, value: T) {
        // Store the value in the parameters dictionary with thread safety
        parametersQueue.async(flags: .barrier) {
            self.parameters[key] = value
        }
        
        // Save to UserDefaults
        saveParameter(key, value: value)
        
        // Update published properties on the main thread
        DispatchQueue.main.async {
            switch key {
            case "PipeLocation":
                if let newLocation = value as? Double {
                    self.pipeLocation = newLocation
                }
            case "ServerConnected":
                if let boolValue = value as? Bool {
                    self.serverConnected = boolValue
                }
            case "ServerError":
                if let stringValue = value as? String {
                    self.serverError = stringValue
                }
            case "ReceivedMessages":
                if let messages = value as? [String] {
                    self.receivedMessages = messages
                }
            case "CurrentMode":
                if let mode = value as? String {
                    self.currentMode = mode
                }
            case "IsRunning":
                if let running = value as? Bool {
                    self.isRunning = running
                }
            case "HasReceivedMode":
                if let received = value as? Bool {
                    self.hasReceivedMode = received
                }
            case "HasReceivedRunningState":
                if let received = value as? Bool {
                    self.hasReceivedRunningState = received
                }
            case "IsSynchronized":
                if let synced = value as? Bool {
                    self.isSynchronized = synced
                }
            case "IsProcessingMessage":
                if let processing = value as? Bool {
                    self.isProcessingMessage = processing
                }
            case "SelectedMode":
                if let mode = value as? String {
                    self.selectedMode = mode
                }
            case "TiltRoll":
                if let roll = value as? Double {
                    self.tiltRoll = roll
                }
            case "TiltPitch":
                if let pitch = value as? Double {
                    self.tiltPitch = pitch
                }
            case "LaunchCounter":
                if let counter = value as? UInt32? {
                    self.launchCounter = counter
                }
            case "LaunchThresholdN":
                if let threshold = value as? UInt8 {
                    self.launchThresholdN = threshold
                }
            case "LastLogMessage":
                if let message = value as? String {
                    self.lastLogMessage = message
                }
            case "PitchP":
                if let p = value as? Double {
                    self.pitchP = p
                }
            case "PitchI":
                if let i = value as? Double {
                    self.pitchI = i
                }
            case "PitchIntegralLimit":
                if let limit = value as? Double {
                    self.pitchIntegralLimit = limit
                }
            case "PitchIntegralThreshold":
                if let threshold = value as? Double {
                    self.pitchIntegralThreshold = threshold
                }
            case "PitchPStepSize":
                if let stepSize = value as? Double {
                    self.pitchPStepSize = stepSize
                }
            case "PitchIStepSize":
                if let stepSize = value as? Double {
                    self.pitchIStepSize = stepSize
                }
            case "PitchIntegralLimitStepSize":
                if let stepSize = value as? Double {
                    self.pitchIntegralLimitStepSize = stepSize
                }
            case "PitchIntegralThresholdStepSize":
                if let stepSize = value as? Double {
                    self.pitchIntegralThresholdStepSize = stepSize
                }
            case "YawP":
                if let p = value as? Double {
                    self.yawP = p
                }
            case "YawI":
                if let i = value as? Double {
                    self.yawI = i
                }
            case "YawIntegralLimit":
                if let limit = value as? Double {
                    self.yawIntegralLimit = limit
                }
            case "YawIntegralThreshold":
                if let threshold = value as? Double {
                    self.yawIntegralThreshold = threshold
                }
            case "YawPStepSize":
                if let stepSize = value as? Double {
                    self.yawPStepSize = stepSize
                }
            case "YawIStepSize":
                if let stepSize = value as? Double {
                    self.yawIStepSize = stepSize
                }
            case "YawIntegralLimitStepSize":
                if let stepSize = value as? Double {
                    self.yawIntegralLimitStepSize = stepSize
                }
            case "YawIntegralThresholdStepSize":
                if let stepSize = value as? Double {
                    self.yawIntegralThresholdStepSize = stepSize
                }
            default:
                break
            }
        }
    }
    
    func getParameter<T>(_ key: String, defaultValue: T) -> T {
        // Use sync to safely read from the concurrent queue
        parametersQueue.sync {
            return parameters[key] as? T ?? defaultValue
        }
    }
    
    func hasParameter(_ key: String) -> Bool {
        // Use sync to safely read from the concurrent queue
        parametersQueue.sync {
            return parameters[key] != nil
        }
    }
    
    // MARK: - Bounding Box Methods
    
    func updateBoundingBox(x1: Double, y1: Double, x2: Double, y2: Double) {
        // Calculate center and dimensions
        let centerX = (x1 + x2) / 2
        let centerY = (y1 + y2) / 2
        let width = x2 - x1
        let height = y2 - y1
        
        let newData = BoundingBoxData(
            centerX: centerX,
            centerY: centerY,
            width: width,
            height: height,
            hasDetection: true
        )
        
        DispatchQueue.main.async {
            self.boundingBoxData = newData
            self.hasDetection = true
            self.updateCounter += 1
        }
    }
    
    func resetBoundingBox() {
        DispatchQueue.main.async {
            self.boundingBoxData = nil
            self.hasDetection = false
            self.updateCounter += 1
        }
    }
    
    var boundingBoxCenterX: Double { boundingBoxData?.centerX ?? 0.0 }
    var boundingBoxCenterY: Double { boundingBoxData?.centerY ?? 0.0 }
    var boundingBoxWidth: Double { boundingBoxData?.width ?? 0.0 }
    var boundingBoxHeight: Double { boundingBoxData?.height ?? 0.0 }
} 