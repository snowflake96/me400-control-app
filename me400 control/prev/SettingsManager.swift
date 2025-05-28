import Foundation

class SettingsManager: ObservableObject {
    @Published var serverIP: String {
        didSet {
            UserDefaults.standard.set(serverIP, forKey: "serverIP")
        }
    }
    
    @Published var serverPort: String {
        didSet {
            UserDefaults.standard.set(serverPort, forKey: "serverPort")
        }
    }
    
    @Published var lastCommand: String {
        didSet {
            UserDefaults.standard.set(lastCommand, forKey: "lastCommand")
        }
    }
    
    @Published var lastPitchValue: Float {
        didSet {
            UserDefaults.standard.set(lastPitchValue, forKey: "lastPitchValue")
        }
    }
    
    @Published var lastYawValue: Float {
        didSet {
            UserDefaults.standard.set(lastYawValue, forKey: "lastYawValue")
        }
    }
    
    @Published var lastVectorX: Double {
        didSet {
            UserDefaults.standard.set(lastVectorX, forKey: "lastVectorX")
        }
    }
    
    @Published var lastVectorY: Double {
        didSet {
            UserDefaults.standard.set(lastVectorY, forKey: "lastVectorY")
        }
    }
    
    @Published var lastVectorZ: Double {
        didSet {
            UserDefaults.standard.set(lastVectorZ, forKey: "lastVectorZ")
        }
    }
    
    @Published var lastSelectedPacketType: Int {
        didSet {
            UserDefaults.standard.set(lastSelectedPacketType, forKey: "lastSelectedPacketType")
        }
    }
    
    init() {
        self.serverIP = UserDefaults.standard.string(forKey: "serverIP") ?? "192.168.0.1"
        self.serverPort = UserDefaults.standard.string(forKey: "serverPort") ?? "8080"
        self.lastCommand = UserDefaults.standard.string(forKey: "lastCommand") ?? ""
        self.lastPitchValue = UserDefaults.standard.float(forKey: "lastPitchValue")
        self.lastYawValue = UserDefaults.standard.float(forKey: "lastYawValue")
        self.lastVectorX = UserDefaults.standard.double(forKey: "lastVectorX")
        self.lastVectorY = UserDefaults.standard.double(forKey: "lastVectorY")
        self.lastVectorZ = UserDefaults.standard.double(forKey: "lastVectorZ")
        self.lastSelectedPacketType = UserDefaults.standard.integer(forKey: "lastSelectedPacketType")
    }
    
    func resetToDefaults() {
        serverIP = "192.168.0.1"
        serverPort = "8080"
        lastCommand = ""
        lastPitchValue = 0
        lastYawValue = 0
        lastVectorX = 0
        lastVectorY = 0
        lastVectorZ = 0
        lastSelectedPacketType = 0
    }
} 