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
    
    // Pipe parameters
    @Published var pipeLocation: Double = 1.5 {
        didSet { saveParameter("PipeLocation", value: pipeLocation) }
    }
    
    // Bounding box parameters
    @Published private(set) var updateCounter: Int = 0
    @Published private(set) var hasDetection: Bool = false
    @Published private(set) var boundingBoxData: BoundingBoxData?
    
    // Server parameters
    @Published var serverHost: String = "localhost" {
        didSet { saveParameter("ServerHost", value: serverHost) }
    }
    @Published var serverPort: String = "12345" {
        didSet { saveParameter("ServerPort", value: serverPort) }
    }
    @Published var serverConnected: Bool = false {
        didSet { saveParameter("ServerConnected", value: serverConnected) }
    }
    @Published var serverError: String = "" {
        didSet { saveParameter("ServerError", value: serverError) }
    }
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
    
    // Message queue handling
    private var _messageQueue: [(type: UInt8, payload: Data)] = []
    var messageQueue: [(type: UInt8, payload: Data)] {
        get { queue.sync { _messageQueue } }
        set { 
            queue.sync { _messageQueue = newValue }
            // Don't save message queue to UserDefaults as it's temporary
        }
    }
    
    @Published var isProcessingMessage: Bool = false {
        didSet { saveParameter("IsProcessingMessage", value: isProcessingMessage) }
    }
    
    private var parameters: [String: Any] = [:]
    private let queue = DispatchQueue(label: "com.me400.parametermanager", qos: .userInitiated)
    
    private init() {
        loadAllParameters()
    }
    
    // MARK: - Persistence Methods
    
    private func saveParameter<T>(_ key: String, value: T) {
        // First update the in-memory parameters synchronously
        queue.sync {
            parameters[key] = value
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
        
        // Load all custom parameters
        for key in UserDefaults.standard.dictionaryRepresentation().keys {
            if let value = UserDefaults.standard.object(forKey: key) {
                queue.sync {
                    parameters[key] = value
                }
            }
        }
    }
    
    // MARK: - Parameter Management Methods
    
    func setParameter<T>(_ key: String, value: T) {
        queue.sync {
            // Store the value in the parameters dictionary
            parameters[key] = value
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
            default:
                break
            }
        }
    }
    
    func getParameter<T>(_ key: String, defaultValue: T) -> T {
        queue.sync {
            return parameters[key] as? T ?? defaultValue
        }
    }
    
    func hasParameter(_ key: String) -> Bool {
        queue.sync {
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