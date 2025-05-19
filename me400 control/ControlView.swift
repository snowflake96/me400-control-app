import SwiftUI

struct ControlView: View {
    @ObservedObject var tcpManager: TCPClientManager
    @ObservedObject var settingsManager: SettingsManager
    @State private var operationMode: OperationMode = .autoAim
    
    enum OperationMode: Int {
        case manual = 0
        case autoAim = 1
        case autonomous = 2
    }
    
    // PID values with persistence
    @AppStorage("pitchPID.x") private var pitchPIDX: Double = 1
    @AppStorage("pitchPID.y") private var pitchPIDY: Double = 0
    @AppStorage("pitchPID.z") private var pitchPIDZ: Double = 0
    @AppStorage("yawPID.x") private var yawPIDX: Double = 1
    @AppStorage("yawPID.y") private var yawPIDY: Double = 0
    @AppStorage("yawPID.z") private var yawPIDZ: Double = 0
    
    // Integral limits with persistence
    @AppStorage("pitchIntegralLimit") private var pitchIntegralLimit: Double = 1.0
    @AppStorage("yawIntegralLimit") private var yawIntegralLimit: Double = 1.0
    
    @State private var pitchVelocity: Double = 0
    @State private var yawVelocity: Double = 0
    @State private var lastSentPitchVelocity: Double = 0
    @State private var lastSentYawVelocity: Double = 0
    @State private var isTriggerPressed: Bool = false
    @State private var escValue: Double = 0.0
    @State private var lastSentEscValue: Double = 0.0
    
    // Constants for velocity limits
    private let minVelocity: Double = -100
    private let maxVelocity: Double = 100
    
    // Constants for normalized limits
    private let minNormalized: Double = -1.0
    private let maxNormalized: Double = 1.0
    
    // Add state variables for bounding box coordinates
    @State private var bboxX1: Double = 0
    @State private var bboxY1: Double = 0
    @State private var bboxX2: Double = 0
    @State private var bboxY2: Double = 0
    
    // Offset state variables
    @State private var offsetX: Double = 0.0
    @State private var offsetY: Double = 0.0
    private let offsetStep: Double = 0.01
    
    // Add state for bounding box color toggle
    @State private var bboxColorToggle: Bool = false
    
    // Computed properties for Vector3
    private var pitchPID: Vector3 {
        get { Vector3(x: pitchPIDX, y: pitchPIDY, z: pitchPIDZ) }
        set {
            pitchPIDX = newValue.x
            pitchPIDY = newValue.y
            pitchPIDZ = newValue.z
        }
    }
    
    private var yawPID: Vector3 {
        get { Vector3(x: yawPIDX, y: yawPIDY, z: yawPIDZ) }
        set {
            yawPIDX = newValue.x
            yawPIDY = newValue.y
            yawPIDZ = newValue.z
        }
    }
    
    // Helper function to round to 2 decimal places for Double
    private func roundToTwoDecimals(_ value: Double) -> Double {
        return (value * 100).rounded() / 100
    }
    
    // Helper function to round to 3 decimal places for Double
    private func roundToThreeDecimals(_ value: Double) -> Double {
        return (value * 1000).rounded() / 1000
    }
    
    // Helper function to round to 4 decimal places for Double
    private func roundToFourDecimals(_ value: Double) -> Double {
        return (value * 10000).rounded() / 10000
    }
    
    
    // Computed properties for rounded slider values
    private var roundedPitchVelocity: Binding<Double> {
        Binding(
            get: { roundToTwoDecimals(pitchVelocity) },
            set: { pitchVelocity = roundToThreeDecimals($0) }
        )
    }
    
    private var roundedYawVelocity: Binding<Double> {
        Binding(
            get: { roundToTwoDecimals(yawVelocity) },
            set: { yawVelocity = roundToThreeDecimals($0) }
        )
    }
    
    // Computed properties for rounded integral limits
    private var roundedPitchIntegralLimit: Binding<Double> {
        Binding(
            get: { roundToFourDecimals(pitchIntegralLimit) },
            set: { 
                pitchIntegralLimit = roundToFourDecimals($0)
                // Send the new value
                let packet = DataPacket.setPitchIntegralLimit(Double(pitchIntegralLimit))
                tcpManager.send(packet)
            }
        )
    }
    
    private var roundedYawIntegralLimit: Binding<Double> {
        Binding(
            get: { roundToFourDecimals(yawIntegralLimit) },
            set: { 
                yawIntegralLimit = roundToFourDecimals($0)
                // Send the new value
                let packet = DataPacket.setYawIntegralLimit(yawIntegralLimit)
                tcpManager.send(packet)
            }
        )
    }
    
    // Helper function to adjust PID values
    private func adjustPIDValue(_ value: inout Double, by amount: Double) {
        value = roundToTwoDecimals(value + amount)
    }
    
    var body: some View {
        VStack(spacing: 0) {
            // Set Offset Section at the very top
            VStack(alignment: .leading, spacing: 8) {
                Text("Set Offset")
                    .font(.headline)
                HStack(spacing: 16) {
                    VStack(alignment: .leading) {
                        Text("X:")
                        HStack(spacing: 4) {
                            Button("-") { offsetX -= offsetStep }
                                .buttonStyle(.bordered)
                            TextField("X Offset", value: $offsetX, format: .number.precision(.fractionLength(2)))
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                .frame(width: 60)
                            Button("+") { offsetX += offsetStep }
                                .buttonStyle(.bordered)
                        }
                    }
                    VStack(alignment: .leading) {
                        Text("Y:")
                        HStack(spacing: 4) {
                            Button("-") { offsetY -= offsetStep }
                                .buttonStyle(.bordered)
                            TextField("Y Offset", value: $offsetY, format: .number.precision(.fractionLength(2)))
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                .frame(width: 60)
                            Button("+") { offsetY += offsetStep }
                                .buttonStyle(.bordered)
                        }
                    }
                    Button("S") {
                        tcpManager.send(DataPacket.setOffset(x: offsetX, y: offsetY))
                    }
                    .buttonStyle(.borderedProminent)
                    .frame(width: 32)
                    Button("R") {
                        offsetX = 0.0
                        offsetY = 0.0
                        tcpManager.send(DataPacket.setOffset(x: 0.0, y: 0.0))
                    }
                    .buttonStyle(.bordered)
                    .frame(width: 32)
                }
            }
            .padding(.horizontal)
            .padding(.top, 10)
            // Middle: square box and start/stop buttons
            HStack(alignment: .top, spacing: 20) {
                ZStack {
                    Rectangle()
                        .stroke(Color.blue, lineWidth: 2)
                        .background(Rectangle().fill(Color.blue.opacity(0.08)))
                    if bboxX1.isNaN || bboxY1.isNaN || bboxX2.isNaN || bboxY2.isNaN {
                        Text("No YOLO Detection")
                            .foregroundColor(.gray)
                            .font(.system(size: 14))
                    } else {
                        let viewWidth = 156.0
                        let viewHeight = 156.0
                        let x1 = (bboxX1 + 1) * viewWidth / 2
                        let y1 = (1 - bboxY1) * viewHeight / 2
                        let x2 = (bboxX2 + 1) * viewWidth / 2
                        let y2 = (1 - bboxY2) * viewHeight / 2
                        let bboxColor = bboxColorToggle ? Color.green : Color.blue
                        Rectangle()
                            .stroke(bboxColor, lineWidth: 2)
                            .frame(width: abs(x2 - x1), height: abs(y2 - y1))
                            .position(x: (x1 + x2) / 2, y: (y1 + y2) / 2)
                    }
                    // Draw cross at offset as a '+' shape
                    let viewWidth = 156.0
                    let viewHeight = 156.0
                    let crossX = (offsetX + 1) * viewWidth / 2
                    let crossY = (1 - offsetY) * viewHeight / 2
                    Path { path in
                        // Vertical line
                        path.move(to: CGPoint(x: crossX, y: crossY - 8))
                        path.addLine(to: CGPoint(x: crossX, y: crossY + 8))
                        // Horizontal line
                        path.move(to: CGPoint(x: crossX - 8, y: crossY))
                        path.addLine(to: CGPoint(x: crossX + 8, y: crossY))
                    }
                    .stroke(Color(.darkGray), lineWidth: 3)
                }
                .frame(width: 156, height: 156)
                .padding(.top, 20)
                // Start/Stop buttons vertically
                VStack(spacing: 10) {
                    Button(action: {
                        tcpManager.send(DataPacket.start())
                    }) {
                        Text("Start")
                            .font(.headline)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.green)
                            .cornerRadius(10)
                    }
                    .buttonStyle(PlainButtonStyle())
                    .contentShape(Rectangle())
                    Button(action: {
                        tcpManager.send(DataPacket.stop())
                    }) {
                        Text("Stop")
                            .font(.headline)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.red)
                            .cornerRadius(10)
                    }
                    .buttonStyle(PlainButtonStyle())
                    .contentShape(Rectangle())
                }
                .frame(width: 120)
                .padding(.top, 20)
            }
            .padding(.bottom, 10)
            // Bottom: transmitted and received message boxes
            VStack(spacing: 6) {
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 10)
                        .fill(Color(.systemGray5))
                    Text(tcpManager.transmissionStatus)
                        .font(.system(size: 12, design: .monospaced))
                        .foregroundColor(tcpManager.transmissionStatus == "Connected" ? .green :
                                         tcpManager.transmissionStatus.contains("Success") ? .green : .red)
                        .padding(.vertical, 2)
                        .padding(.horizontal, 8)
                        .multilineTextAlignment(.leading)
                        .frame(maxWidth: .infinity, alignment: .leading)
                }
                .fixedSize(horizontal: false, vertical: true)
                .frame(maxWidth: 300, alignment: .leading)
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 10)
                        .fill(Color(.systemGray5))
                    Text(tcpManager.serverStatus)
                        .font(.system(size: 12, design: .monospaced))
                        .foregroundColor(.primary)
                        .padding(.vertical, 2)
                        .padding(.horizontal, 8)
                        .multilineTextAlignment(.leading)
                        .frame(maxWidth: .infinity, alignment: .leading)
                }
                .fixedSize(horizontal: false, vertical: true)
                .frame(maxWidth: 300, alignment: .leading)
                
            }
            .padding(.top, 6)
            
            // Scrollable control panel
            Form {
                // Operation Mode Selection
                Section(header: Text("Operation Mode")) {
                    Picker("Mode", selection: $operationMode) {
                        Text("Manual").tag(OperationMode.manual)
                        Text("Auto Aim").tag(OperationMode.autoAim)
                        Text("Autonomous").tag(OperationMode.autonomous)
                    }
                    .pickerStyle(SegmentedPickerStyle())
                    .onChange(of: operationMode) { newValue in
                        switch newValue {
                        case .manual:
                            tcpManager.send(DataPacket.setManual())
                        case .autoAim:
                            tcpManager.send(DataPacket(type: .setAutoAim, data: .text("")))
                        case .autonomous:
                            tcpManager.send(DataPacket.setAutonomous())
                        }
                    }
                }
                
                if operationMode == .autonomous {
                    // Autonomous Mode Controls (PID Tuning)
                    Section(header: Text("Pitch PID Tuning")) {
                        VStack(spacing: 15) {
                            HStack(spacing: 20) {
                                // Left side: PID Controls
                                VStack(alignment: .leading, spacing: 8) {
                                    HStack {
                                        Text("P:")
                                        TextField("P", value: $pitchPIDX, format: .number.precision(.fractionLength(2)))
                                            .textFieldStyle(RoundedBorderTextFieldStyle())
                                            .frame(width: 60)
                                        Button("-") { adjustPIDValue(&pitchPIDX, by: -0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                        Button("+") { adjustPIDValue(&pitchPIDX, by: 0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                    }
                                    HStack {
                                        Text("I:")
                                        TextField("I", value: $pitchPIDY, format: .number.precision(.fractionLength(2)))
                                            .textFieldStyle(RoundedBorderTextFieldStyle())
                                            .frame(width: 60)
                                        Button("-") { adjustPIDValue(&pitchPIDY, by: -0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                        Button("+") { adjustPIDValue(&pitchPIDY, by: 0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                    }
//                                    HStack {
//                                        Text("D:")
//                                        TextField("D", value: $pitchPIDZ, format: .number.precision(.fractionLength(2)))
//                                            .textFieldStyle(RoundedBorderTextFieldStyle())
//                                            .frame(width: 60)
//                                        Button("-") { adjustPIDValue(&pitchPIDZ, by: -0.1) }
//                                            .buttonStyle(.bordered)
//                                            .frame(width: 40)
//                                        Button("+") { adjustPIDValue(&pitchPIDZ, by: 0.1) }
//                                            .buttonStyle(.bordered)
//                                            .frame(width: 40)
//                                    }
                                }
                                
                                // Divider
                                Divider()
                                    .frame(height: 100)
                                
                                // Right side: Action Buttons
                                VStack(spacing: 10) {
                                    Button("Reset") {
                                        pitchPIDX = 1.0
                                        pitchPIDY = 0.0
                                        pitchPIDZ = 0.0
                                        pitchIntegralLimit = 1.0
                                        // Send the reset integral limit
                                        let packet = DataPacket.setPitchIntegralLimit(pitchIntegralLimit)
                                        tcpManager.send(packet)
                                    }
                                    .buttonStyle(.bordered)
                                    .frame(width: 100)
                                    
                                    Button("GO!") {
                                        let vector3Data = DataPacket.servoCommand(x: pitchPIDX, y: pitchPIDY, z: pitchPIDZ).data
                                        let packet = DataPacket(type: .tunePitch, data: vector3Data)
                                        tcpManager.send(packet)
                                    }
                                    .buttonStyle(.borderedProminent)
                                    .frame(width: 100)
                                }
                            }
                            
                            // Integral Limit Control
                            HStack {
                                Text("IL:")
                                TextField("Limit", value: roundedPitchIntegralLimit, format: .number.precision(.fractionLength(4)))
                                    .textFieldStyle(RoundedBorderTextFieldStyle())
                                    .frame(width: 100)
                                Button("-") { adjustPIDValue(&pitchIntegralLimit, by: -0.05) }
                                    .buttonStyle(.bordered)
                                    .frame(width: 40)
                                Button("+") { adjustPIDValue(&pitchIntegralLimit, by: 0.05) }
                                    .buttonStyle(.bordered)
                                    .frame(width: 40)
                                Button("Set") {
                                    let packet = DataPacket.setPitchIntegralLimit(Double(pitchIntegralLimit))
                                    tcpManager.send(packet)
                                }
                                .buttonStyle(.bordered)
                                .frame(width: 60)
                            }
                        }
                        .padding()
                        .background(Color(.systemGray6))
                        .cornerRadius(10)
                    }
                    
                    Section(header: Text("Yaw PID Tuning")) {
                        VStack(spacing: 15) {
                            HStack(spacing: 20) {
                                // Left side: PID Controls
                                VStack(alignment: .leading, spacing: 8) {
                                    HStack {
                                        Text("P:")
                                        TextField("P", value: $yawPIDX, format: .number.precision(.fractionLength(2)))
                                            .textFieldStyle(RoundedBorderTextFieldStyle())
                                            .frame(width: 60)
                                        Button("-") { adjustPIDValue(&yawPIDX, by: -0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                        Button("+") { adjustPIDValue(&yawPIDX, by: 0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                    }
                                    HStack {
                                        Text("I:")
                                        TextField("I", value: $yawPIDY, format: .number.precision(.fractionLength(2)))
                                            .textFieldStyle(RoundedBorderTextFieldStyle())
                                            .frame(width: 60)
                                        Button("-") { adjustPIDValue(&yawPIDY, by: -0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                        Button("+") { adjustPIDValue(&yawPIDY, by: 0.1) }
                                            .buttonStyle(.bordered)
                                            .frame(width: 40)
                                    }
//                                    HStack {
//                                        Text("D:")
//                                        TextField("D", value: $yawPIDZ, format: .number.precision(.fractionLength(2)))
//                                            .textFieldStyle(RoundedBorderTextFieldStyle())
//                                            .frame(width: 60)
//                                        Button("-") { adjustPIDValue(&yawPIDZ, by: -0.1) }
//                                            .buttonStyle(.bordered)
//                                            .frame(width: 40)
//                                        Button("+") { adjustPIDValue(&yawPIDZ, by: 0.1) }
//                                            .buttonStyle(.bordered)
//                                            .frame(width: 40)
//                                    }
                                }
                                
                                // Divider
                                Divider()
                                    .frame(height: 100)
                                
                                // Right side: Action Buttons
                                VStack(spacing: 10) {
                                    Button("Reset") {
                                        yawPIDX = 1.0
                                        yawPIDY = 0.0
                                        yawPIDZ = 0.0
                                        yawIntegralLimit = 1.0
                                        // Send the reset integral limit
                                        let packet = DataPacket.setYawIntegralLimit(yawIntegralLimit)
                                        tcpManager.send(packet)
                                    }
                                    .buttonStyle(.bordered)
                                    .frame(width: 100)
                                    
                                    Button("GO!") {
                                        let vector3Data = DataPacket.servoCommand(x: yawPIDX, y: yawPIDY, z: yawPIDZ).data
                                        let packet = DataPacket(type: .tuneYaw, data: vector3Data)
                                        tcpManager.send(packet)
                                    }
                                    .buttonStyle(.borderedProminent)
                                    .frame(width: 100)
                                }
                            }
                            
                            // Integral Limit Control
                            HStack {
                                Text("IL:")
                                TextField("Limit", value: roundedYawIntegralLimit, format: .number.precision(.fractionLength(4)))
                                    .textFieldStyle(RoundedBorderTextFieldStyle())
                                    .frame(width: 100)
                                Button("-") { adjustPIDValue(&yawIntegralLimit, by: -0.05) }
                                    .buttonStyle(.bordered)
                                    .frame(width: 40)
                                Button("+") { adjustPIDValue(&yawIntegralLimit, by: 0.05) }
                                    .buttonStyle(.bordered)
                                    .frame(width: 40)
                                Button("Set") {
                                    let packet = DataPacket.setYawIntegralLimit(Double(yawIntegralLimit))
                                    tcpManager.send(packet)
                                }
                                .buttonStyle(.bordered)
                                .frame(width: 60)
                            }
                        }
                        .padding()
                        .background(Color(.systemGray6))
                        .cornerRadius(10)
                    }
                } else {
                    // Manual Mode Controls (Normalized Value Setting)
                    Section(header: Text("Pitch Control")) {
                        VStack {
                            HStack {
                                Text("Pitch Value:")
                                Spacer()
                                Text(String(format: "%.3f", roundToThreeDecimals(pitchVelocity)))
                                    .frame(width: 80)
                                    .multilineTextAlignment(.trailing)
                            }
                            Slider(value: $pitchVelocity, in: minNormalized...maxNormalized)
                                .onChange(of: pitchVelocity) { newValue in
                                    let roundedPitch = roundToThreeDecimals(newValue)
                                    let packet = DataPacket(type: .servoCommand, data: .vector3(x: Double(roundedPitch), y: Double(roundToThreeDecimals(yawVelocity)), z: 0))
                                    tcpManager.send(packet)
                                }
                        }
                    }
                    
                    Section(header: Text("Yaw Control")) {
                        VStack {
                            HStack {
                                Text("Yaw Value:")
                                Spacer()
                                Text(String(format: "%.3f", roundToThreeDecimals(yawVelocity)))
                                    .frame(width: 80)
                                    .multilineTextAlignment(.trailing)
                            }
                            Slider(value: $yawVelocity, in: minNormalized...maxNormalized)
                                .onChange(of: yawVelocity) { newValue in
                                    let roundedYaw = roundToThreeDecimals(newValue)
                                    let packet = DataPacket(type: .servoCommand, data: .vector3(x: Double(roundToThreeDecimals(pitchVelocity)), y: Double(roundedYaw), z: 0))
                                    tcpManager.send(packet)
                                }
                        }
                    }
                    
                    Section(header: Text("Reset Controls")) {
                        Button(action: {
                            pitchVelocity = 0
                            yawVelocity = 0
                        }) {
                            Text("Reset Pitch & Yaw")
                                .frame(maxWidth: .infinity)
                                .padding()
                                .background(Color.blue)
                                .foregroundColor(.white)
                                .cornerRadius(10)
                        }
                    }
                    
                    Section(header: Text("Launch Control")) {
                        Toggle("Start Launch", isOn: $isTriggerPressed)
                            .onChange(of: isTriggerPressed) { newValue in
                                tcpManager.send(DataPacket(type: .triggerCommand, data: .boolean(newValue)))
                            }
                    }
                    
                    Section(header: Text("ESC Control")) {
                        VStack {
                            HStack {
                                Text("ESC Value:")
                                Spacer()
                                TextField("Value", value: Binding(get: { roundToThreeDecimals(escValue) }, set: { escValue = $0 }), format: .number.precision(.fractionLength(3)))
                                    .textFieldStyle(RoundedBorderTextFieldStyle())
                                    .frame(width: 80)
                                    .multilineTextAlignment(.trailing)
                            }
                            Slider(value: $escValue, in: 0...1.0)
                                .onChange(of: escValue) { newValue in
                                    let roundedEsc = roundToThreeDecimals(newValue)
                                    let packet = DataPacket(type: .escCommand, data: .double(roundedEsc))
                                    tcpManager.send(packet)
                                }
                        }
                    }
                }
            }
            .padding(.top, 10)
        }
        .navigationTitle("Control Panel")

    }
    
    // Helper function to parse bounding box data from server status
    private func parseBboxData(_ status: String) -> (x1: Double, y1: Double, x2: Double, y2: Double)? {
        guard status.contains("Bbox:") else { return nil }
        let pattern = #"Bbox: \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\)"#
        guard let regex = try? NSRegularExpression(pattern: pattern),
              let match = regex.firstMatch(in: status, range: NSRange(status.startIndex..., in: status)) else {
            return nil
        }
        let ranges = (1...4).map { match.range(at: $0) }
        let values = ranges.compactMap { range -> Double? in
            guard let stringRange = Range(range, in: status),
                  let value = Double(status[stringRange]) else { return nil }
            // Check if the value is a valid number (not NaN or infinity)
            return value.isFinite ? value : nil
        }
        guard values.count == 4 else { return nil }
        return (values[0], values[1], values[2], values[3])
    }
}

// Helper struct for PID values
struct Vector3 {
    var x: Double
    var y: Double
    var z: Double
} 



