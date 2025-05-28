import Foundation
import SwiftUI
import Combine

class CameraOffsetControl: ObservableObject {
    // MARK: - Properties
    @Published var xOffset: Double = 0.0 {
        didSet {
            if !isUpdating {
                isUpdating = true
//                print("DEBUG: xOffset didSet called with value: \(xOffset)")
                updateAndSendOffset()
                isUpdating = false
            }
        }
    }
    
    @Published var yOffset: Double = 0.0 {
        didSet {
            if !isUpdating {
                isUpdating = true
//                print("DEBUG: yOffset didSet called with value: \(yOffset)")
                updateAndSendOffset()
                isUpdating = false
            }
        }
    }
    
    @Published var zOffset: Double = 0.0 {
        didSet {
            if !isUpdating {
                isUpdating = true
//                print("DEBUG: zOffset didSet called with value: \(zOffset)")
                updateAndSendOffset()
                isUpdating = false
            }
        }
    }
    
    @Published var stepSize: Double = 0.01 {
        didSet { 
//            print("DEBUG: stepSize didSet called with value: \(stepSize)")
            ParameterManager.shared.setParameter("CameraOffsetStepSize", value: stepSize)
        }
    }
    
    // Constants
    let maxOffset: Double = 1.0
    let minOffset: Double = -1.0
    let maxStepSize: Double = 0.1
    let minStepSize: Double = 0.001
    
    // Server connection
    private let serverConnection: ServerCommunicationManager
    private var messageTimer: Timer?
    private var isUpdating: Bool = false
    
    // MARK: - Initialization
    init() {
        print("DEBUG: CameraOffsetControl init called")
        // Initialize values from ParameterManager
        xOffset = ParameterManager.shared.getParameter("CameraOffsetX", defaultValue: 0.0)
        yOffset = ParameterManager.shared.getParameter("CameraOffsetY", defaultValue: 0.0)
        zOffset = ParameterManager.shared.getParameter("CameraOffsetZ", defaultValue: 0.0)
        stepSize = ParameterManager.shared.getParameter("CameraOffsetStepSize", defaultValue: 0.01)
        
        // Initialize server connection
        serverConnection = ServerCommunicationManager.shared
        
        // Start observing messages
        startMessageObservation()
    }
    
    private func startMessageObservation() {
        // Create a timer to check for new messages periodically
        messageTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            guard let self = self else { return }
            let messages = self.serverConnection.receivedMessages
            if let lastMessage = messages.last {
                self.handleServerMessage(lastMessage)
            }
        }
    }
    
    // MARK: - Public Methods
    func setXOffset(_ value: Double) {
//        print("DEBUG: setXOffset called with value: \(value)")
        let boundedValue = min(max(value, minOffset), maxOffset)
        xOffset = boundedValue
    }
    
    func setYOffset(_ value: Double) {
//        print("DEBUG: setYOffset called with value: \(value)")
        let boundedValue = min(max(value, minOffset), maxOffset)
        yOffset = boundedValue
    }
    
    func setZOffset(_ value: Double) {
//        print("DEBUG: setZOffset called with value: \(value)")
        let boundedValue = min(max(value, minOffset), maxOffset)
        zOffset = boundedValue
    }
    
    func setStepSize(_ value: Double) {
//        print("DEBUG: setStepSize called with value: \(value)")
        let boundedValue = min(max(value, minStepSize), maxStepSize)
        stepSize = boundedValue
    }
    
    // MARK: - Private Methods
    private func updateAndSendOffset() {
//        print("DEBUG: updateAndSendOffset called with values - x: \(xOffset), y: \(yOffset), z: \(zOffset)")
        // Update parameters in ParameterManager
        ParameterManager.shared.setParameter("CameraOffsetX", value: xOffset)
        ParameterManager.shared.setParameter("CameraOffsetY", value: yOffset)
        ParameterManager.shared.setParameter("CameraOffsetZ", value: zOffset)
        
        // Send the offset to server
        let packet = DataPacket.setOffset(x: xOffset, y: yOffset, z: zOffset)
//        print("DEBUG: Sending packet to server")
        _ = serverConnection.send(packet)
    }
    
    private func handleServerMessage(_ message: String) {
//        print("DEBUG: handleServerMessage called with message: \(message)")
        // Handle incoming server messages
        // Example: "Received SetOffset: x=0.5, y=0.3, z=0.0"
        if message.contains("Received SetOffset:") {
            let components = message.split(separator: "=")
            guard components.count >= 3 else { return }
            
            // Extract x, y, and z values
            let xStr = components[1].split(separator: ",")[0]
            let yStr = components[2].split(separator: ",")[0]
            let zStr = components[3].split(separator: ",")[0]
            
            if let x = Double(xStr), let y = Double(yStr), let z = Double(zStr) {
                print("DEBUG: Parsed server message values - x: \(x), y: \(y), z: \(z)")
                DispatchQueue.main.async { [weak self] in
                    self?.xOffset = x
                    self?.yOffset = y
                    self?.zOffset = z
                }
            }
        }
    }
    
    deinit {
        messageTimer?.invalidate()
    }
}

// MARK: - SwiftUI View
struct CameraOffsetControlView: View {
    @ObservedObject var control: CameraOffsetControl
    
    var body: some View {
            VStack(spacing: 10) {
                // X Offset Control
                    DoubleInputBox(
                        title: "X Offset",
                        value: Binding(
                            get: { control.xOffset },
                            set: { control.setXOffset($0) }
                        ),
                        minValue: control.minOffset,
                        maxValue: control.maxOffset,
                        stepSize: control.stepSize,
                        format: "%.3f",
                        allowNegative: true,
                        frameWidth: 60
                    )
                
                // Y Offset Control
                    DoubleInputBox(
                        title: "Y Offset",
                        value: Binding(
                            get: { control.yOffset },
                            set: { control.setYOffset($0) }
                        ),
                        minValue: control.minOffset,
                    maxValue: control.maxOffset,
                    stepSize: control.stepSize,
                    format: "%.3f",
                    allowNegative: true,
                        frameWidth: 60
                    )
                
                // Step Size Control
                    DoubleInputBox(
                        title: "Step Size",
                        value: Binding(
                            get: { control.stepSize },
                            set: { control.setStepSize($0) }
                        ),
                        minValue: control.minStepSize,
                    maxValue: control.maxStepSize,
                    stepSize: control.stepSize,
                    format: "%.3f",
                    allowNegative: false,
                        frameWidth: 60
                    )
                
                // Reset Button
                Button(action: {
                    control.setXOffset(0.0)
                    control.setYOffset(0.0)
                    control.setStepSize(0.01)
                }) {
                    Text("Reset")
                        .foregroundColor(.white)
                        .padding(.horizontal, 20)
                        .padding(.vertical, 8)
                        .background(Color.blue.opacity(0.7))
                        .cornerRadius(8)
                }
            }
            .padding()
            .background(Color(.systemGray6))
            .cornerRadius(10)
    }
}

// MARK: - Preview
struct CameraOffsetControlView_Previews: PreviewProvider {
    static var previews: some View {
        CameraOffsetControlView(control: CameraOffsetControl())
            .padding()
    }
} 
