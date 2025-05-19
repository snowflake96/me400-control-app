import SwiftUI

struct LeftDrawingView: View {
    var pitchAngle: Double? // in degrees
    var capsuleX: Double // normalized x for capsule center
    
    func normToPoint(_ x: CGFloat, _ y: CGFloat, in size: CGSize) -> CGPoint {
        // Map (-1,1) to (0,width) and (0,height), y is flipped (bottom is low y)
        let px = (x + 1) / 2 * size.width
        let py = size.height - ((y + 1) / 2 * size.height)
        return CGPoint(x: px, y: py)
    }
    
    var body: some View {
        GeometryReader { geo in
            let size = geo.size
            let capsuleNorm = CGPoint(x: capsuleX, y: -0.8)
            let capsuleCenter = normToPoint(capsuleNorm.x, capsuleNorm.y, in: size)
            let protractorRadius = min(size.width, size.height) * 0.18
            ZStack(alignment: .topLeading) {
                // Border
                Rectangle()
                    .stroke(Color.gray, lineWidth: 1)
                // Green line from (-1, -0.8) to (1, -0.8)
                Path { path in
                    path.move(to: normToPoint(-1, -0.8, in: size))
                    path.addLine(to: normToPoint(1, -0.8, in: size))
                }
                .stroke(Color.green, lineWidth: 3)
                // Thick gray line from (0.9, -0.8) to (0.9, 0.9)
                Path { path in
                    path.move(to: normToPoint(0.9, -0.8, in: size))
                    path.addLine(to: normToPoint(0.9, 0.9, in: size))
                }
                .stroke(Color.gray, lineWidth: 8)
                // Thick gray line from (0.75, 0.8) to (0.9, 0.8)
                Path { path in
                    path.move(to: normToPoint(0.75, 0.8, in: size))
                    path.addLine(to: normToPoint(0.9, 0.8, in: size))
                }
                .stroke(Color.gray, lineWidth: 8)
                // Black filled circle at (0.75, 0.75) with radius 0.05
                let circleCenter = normToPoint(0.75, 0.75, in: size)
                let circleRadius = 0.05 * min(size.width, size.height) / 2
                Circle()
                    .fill(Color.black)
                    .frame(width: circleRadius * 2, height: circleRadius * 2)
                    .position(circleCenter)
                // Laying capsule over the green line at capsuleX
                Capsule()
                    .fill(Color.blue)
                    .frame(width: 40, height: 16)
                    .position(capsuleCenter)
                // Protractor (arc with ticks and labels)
                ProtractorView(center: capsuleCenter, radius: protractorRadius, boxSize: size)
                // Red line for pitch angle
                if let angle = pitchAngle {
                    let rad = CGFloat(angle) * .pi / 180
                    // Find intersection with box edge
                    let dx = cos(rad)
                    let dy = -sin(rad) // y axis is down
                    // Find t for intersection with each edge
                    let tx = dx > 0 ? (size.width - capsuleCenter.x) / dx : (0 - capsuleCenter.x) / dx
                    let ty = dy > 0 ? (size.height - capsuleCenter.y) / dy : (0 - capsuleCenter.y) / dy
                    let t = min(tx, ty)
                    let end = CGPoint(
                        x: capsuleCenter.x + dx * t,
                        y: capsuleCenter.y + dy * t
                    )
                    Path { path in
                        path.move(to: capsuleCenter)
                        path.addLine(to: end)
                    }
                    .stroke(Color.red, lineWidth: 3)
                }
                // Pitch angle value in top-left
                if let angle = pitchAngle {
                    Text(String(format: "%.2f°", angle))
                        .font(.system(size: 16, weight: .bold, design: .monospaced))
                        .foregroundColor(.primary)
                        .padding(6)
                        .background(Color.white.opacity(0.7))
                        .cornerRadius(8)
                        .padding(8)
                }
            }
        }
        .aspectRatio(1, contentMode: .fit)
    }
}

struct ProtractorView: View {
    let center: CGPoint
    let radius: CGFloat
    let boxSize: CGSize
    var body: some View {
        Canvas { context, size in
            // Draw arc (0 to 180 deg)
            let arc = Path { path in
                path.addArc(center: center, radius: radius, startAngle: .degrees(0), endAngle: .degrees(180), clockwise: false)
            }
            context.stroke(arc, with: .color(.gray), lineWidth: 2)
            // Draw ticks every 10 deg, labels every 30 deg
            for deg in stride(from: 0, through: 180, by: 10) {
                let rad = CGFloat(deg) * .pi / 180
                let tickLen: CGFloat = (deg % 30 == 0) ? 12 : 6
                let tickStart = CGPoint(
                    x: center.x + cos(rad) * (radius - tickLen),
                    y: center.y - sin(rad) * (radius - tickLen)
                )
                let tickEnd = CGPoint(
                    x: center.x + cos(rad) * (radius + tickLen),
                    y: center.y - sin(rad) * (radius + tickLen)
                )
                var tickPath = Path()
                tickPath.move(to: tickStart)
                tickPath.addLine(to: tickEnd)
                context.stroke(tickPath, with: .color(.gray), lineWidth: 2)
                // Label every 30 deg
                if deg % 30 == 0 {
                    let label = Text("\(deg)")
                        .font(.system(size: 10))
                        .foregroundColor(.gray)
                    let labelPos = CGPoint(
                        x: center.x + cos(rad) * (radius + 22),
                        y: center.y - sin(rad) * (radius + 22)
                    )
                    context.draw(label, at: labelPos, anchor: .center)
                }
            }
        }
    }
}

struct RightDrawingView: View {
    let boundingBox: CGRect?
    let isNaN: Bool
    let showCrosshair: Bool
    let crosshairX: Double
    let crosshairY: Double
    let recentBboxCenters: [(x: Double, y: Double)]
    let launchThresholdN: Int
    let launchThresholdEpsilon: Double
    @State private var boxColor: Color = .blue
    @State private var selectedMode: Int = 0
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Background with custom border
                Rectangle()
                    .fill(Color(.systemBackground))
                Rectangle()
                    .stroke(Color.gray, lineWidth: 1)
                // Grid lines
                ForEach(-10...10, id: \.self) { i in
                    let position = CGFloat(i) / 10.0
                    // Vertical grid
                    Path { path in
                        let x = geometry.size.width * (position + 1) / 2
                        path.move(to: CGPoint(x: x, y: 0))
                        path.addLine(to: CGPoint(x: x, y: geometry.size.height))
                    }
                    .stroke(Color.gray.opacity(0.3), lineWidth: 0.5)
                    // Horizontal grid
                    Path { path in
                        let y = geometry.size.height * (position + 1) / 2
                        path.move(to: CGPoint(x: 0, y: y))
                        path.addLine(to: CGPoint(x: geometry.size.width, y: y))
                    }
                    .stroke(Color.gray.opacity(0.3), lineWidth: 0.5)
                }
                // Y axis (x=0) - dashed
                Path { path in
                    let x = geometry.size.width / 2
                    path.move(to: CGPoint(x: x, y: 0))
                    path.addLine(to: CGPoint(x: x, y: geometry.size.height))
                }
                .stroke(style: StrokeStyle(lineWidth: 2, dash: [8, 6]))
                .foregroundColor(.gray)
                // X axis (y=0) - dashed
                Path { path in
                    let y = geometry.size.height / 2
                    path.move(to: CGPoint(x: 0, y: y))
                    path.addLine(to: CGPoint(x: geometry.size.width, y: y))
                }
                .stroke(style: StrokeStyle(lineWidth: 2, dash: [8, 6]))
                .foregroundColor(.gray)
                // Crosshair
                if showCrosshair {
                    let centerX = geometry.size.width * CGFloat((crosshairX + 1) / 2)
                    let centerY = geometry.size.height * CGFloat((1 - crosshairY) / 2)
                    Path { path in
                        path.move(to: CGPoint(x: centerX - 20, y: centerY))
                        path.addLine(to: CGPoint(x: centerX + 20, y: centerY))
                        path.move(to: CGPoint(x: centerX, y: centerY - 20))
                        path.addLine(to: CGPoint(x: centerX, y: centerY + 20))
                    }
                    .stroke(Color.red, lineWidth: 2)
                }
                // Target lock-on feature (top right)
                GeometryReader { geo in
                    VStack(alignment: .leading, spacing: 4) {
                        HStack {
                            Spacer()
                            Text("Target Lock")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                        if recentBboxCenters.count < launchThresholdN {
                            HStack {
                                Spacer()
                                Text("\(launchThresholdN - recentBboxCenters.count) bounding box needed for target lock")
                                    .font(.caption2)
                                    .foregroundColor(.orange)
                            }
                            HStack(spacing: 8) {
                                Spacer()
                                Text("x: detect more!")
                                    .foregroundColor(.secondary)
                                    .font(.caption)
                                Text("y: detect more!")
                                    .foregroundColor(.secondary)
                                    .font(.caption)
                            }
                        } else {
                            // Find farthest two points
                            let pairs = recentBboxCenters.combinations(ofCount: 2)
                            let (maxPair, maxDist) = pairs.map { ($0, hypot($0[0].x - $0[1].x, $0[0].y - $0[1].y)) }
                                .max(by: { $0.1 < $1.1 }) ?? ([], 0)
                            if maxPair.count == 2 {
                                let dx = maxPair[0].x - maxPair[1].x
                                let dy = maxPair[0].y - maxPair[1].y
                                let absDx = abs(dx)
                                let absDy = abs(dy)
                                HStack(spacing: 8) {
                                    Spacer()
                                    Text("x: ")
                                    Text(String(format: "%.3f", dx))
                                        .foregroundColor(absDx < launchThresholdEpsilon ? .green : .primary)
                                    Text("y: ")
                                    Text(String(format: "%.3f", dy))
                                        .foregroundColor(absDy < launchThresholdEpsilon ? .green : .primary)
                                }
                            }
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .position(x: geo.size.width * 0.49, y: geo.size.height * 0.9)
                }
                if isNaN {
                    Text("YOLO no detection")
                        .foregroundColor(.red)
                        .font(.headline)
                } else if let box = boundingBox {
                    // Bounding box
                    Rectangle()
                        .fill(boxColor.opacity(0.3))
                        .frame(
                            width: abs(box.width) * geometry.size.width / 2,
                            height: abs(box.height) * geometry.size.height / 2
                        )
                        .position(
                            x: (box.midX + 1) * geometry.size.width / 2,
                            y: (box.midY + 1) * geometry.size.height / 2
                        )
                    // Show bounding box center coordinates
                    let centerX = box.midX
                    let centerY = box.midY
                    VStack {
                        Spacer().frame(height: 8)
                        HStack {
                            Text(String(format: "(%.2f, %.2f)", centerX, centerY))
                                .font(.system(size: 14, weight: .bold, design: .monospaced))
                                .foregroundColor(.primary)
                                .padding(5)
                                .background(Color.white.opacity(0.7))
                                .cornerRadius(6)
                            Spacer()
                        }
                    }
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
                }
            }
        }
        .aspectRatio(1, contentMode: .fit)
        .onChange(of: boundingBox) { _ in
            withAnimation {
                boxColor = boxColor == .blue ? .green : .blue
            }
        }
    }
}

struct MacAndiPadView: View {
    @StateObject private var tcpManager = TCPClientManager()
    @StateObject private var settingsManager = SettingsManager()
    @State private var isSidebarVisible = false
    @State private var boundingBox: CGRect?
    @State private var isBoundingBoxNaN = false
    @State private var latestPitchAngle: Double? = nil
    @State private var capsulePositionValue: Double = 1.0 // between 1 and 2
    @State private var xOffset: Double = 0.0
    @State private var yOffset: Double = 0.0
    @State private var selectedMode: Int = 0
    @State private var pitchValue: Double = 0.0
    @State private var yawValue: Double = 0.0
    @State private var escValue: Double = 0.0
    @GestureState private var pitchDragValue: Double? = nil
    @GestureState private var yawDragValue: Double? = nil
    @GestureState private var escDragValue: Double? = nil
    // PI controller states
    @State private var pitchP: Double = 1.0
    @State private var pitchI: Double = 0.0
    @State private var yawP: Double = 1.0
    @State private var yawI: Double = 0.0
    @State private var launchThresholdN: Int = 5
    @State private var launchThresholdEpsilon: Double = 0.01
    @State private var isLaunchPressed: Bool = false
    @State private var recentBboxCenters: [(x: Double, y: Double)] = []
    
    static let decimalFormatter: NumberFormatter = {
        let f = NumberFormatter()
        f.numberStyle = .decimal
        f.minimumFractionDigits = 2
        f.maximumFractionDigits = 2
        f.minimum = 0
        f.maximum = 100
        return f
    }()
    
    var capsuleX: Double {
        capsulePositionValue * (-0.5) + 0.2
    }
    
    var body: some View {
        HStack(spacing: 0) {
            // Main View
            VStack {
                // Status Bar
                HStack {
                    Text(tcpManager.isConnected ? "Connected" : "Disconnected")
                        .foregroundColor(tcpManager.isConnected ? .green : .red)
                    
                    Spacer()
                    
                    Button("Start") {
                        _ = tcpManager.send(DataPacket.start())
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(!tcpManager.isConnected)
                    
                    Button("Stop") {
                        _ = tcpManager.send(DataPacket.stop())
                    }
                    .buttonStyle(.bordered)
                    .disabled(!tcpManager.isConnected)
                    
                    Button(action: { isSidebarVisible.toggle() }) {
                        Image(systemName: "network")
                            .font(.title2)
                    }
                }
                .padding()
                
                // Recent server log message
                if let latestLog = tcpManager.receivedMessages.last(where: { $0.hasPrefix("Received Log:") }) {
                    HStack {
                        Text(latestLog.replacingOccurrences(of: "Received Log: ", with: ""))
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(.primary)
                            .padding(8)
                        Spacer()
                    }
                    .background(Color(.systemGray5))
                    .cornerRadius(10)
                    .padding(.horizontal)
                    .padding(.bottom, 4)
                }
                
                // Fixed Top Section
                HStack(spacing: 20) {
                    // Left square (custom drawing)
                    LeftDrawingView(pitchAngle: latestPitchAngle, capsuleX: capsuleX)
                    // Right square (coordinate system + crosshair)
                    RightDrawingView(
                        boundingBox: boundingBox,
                        isNaN: isBoundingBoxNaN,
                        showCrosshair: true,
                        crosshairX: xOffset,
                        crosshairY: yOffset,
                        recentBboxCenters: recentBboxCenters,
                        launchThresholdN: launchThresholdN,
                        launchThresholdEpsilon: launchThresholdEpsilon
                    )
                }
                .padding()
                
                // Controls under squares
                HStack(alignment: .top, spacing: 20) {
                    // Under left square: capsule position control
                    HStack(spacing: 8) {
                        Button("-") {
                            capsulePositionValue = max(1.0, (capsulePositionValue - 0.01).rounded(toPlaces: 2))
                        }
                        .buttonStyle(.bordered)
                        TextField("", value: $capsulePositionValue, formatter: Self.decimalFormatter)
                            .frame(width: 50)
                            .multilineTextAlignment(.center)
                            .textFieldStyle(RoundedBorderTextFieldStyle())
                        Button("+") {
                            capsulePositionValue = min(2.0, (capsulePositionValue + 0.01).rounded(toPlaces: 2))
                        }
                        .buttonStyle(.bordered)
                    }
                    // Under right square: xOffset/yOffset controls
                    VStack(spacing: 8) {
                        HStack(spacing: 8) {
                            Text("xOffset")
                            Button("-") {
                                xOffset = (xOffset - 0.01).rounded(toPlaces: 2)
                                sendOffset()
                            }
                            .buttonStyle(.bordered)
                            TextField("", value: $xOffset, formatter: Self.decimalFormatter)
                                .frame(width: 60)
                                .multilineTextAlignment(.center)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                            Button("+") {
                                xOffset = (xOffset + 0.01).rounded(toPlaces: 2)
                                sendOffset()
                            }
                            .buttonStyle(.bordered)
                        }
                        HStack(spacing: 8) {
                            Text("yOffset")
                            Button("-") {
                                yOffset = (yOffset - 0.01).rounded(toPlaces: 2)
                                sendOffset()
                            }
                            .buttonStyle(.bordered)
                            TextField("", value: $yOffset, formatter: Self.decimalFormatter)
                                .frame(width: 60)
                                .multilineTextAlignment(.center)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                            Button("+") {
                                yOffset = (yOffset + 0.01).rounded(toPlaces: 2)
                                sendOffset()
                            }
                            .buttonStyle(.bordered)
                        }
                    }
                }
                .padding(.horizontal)
                .onChange(of: xOffset) { _ in sendOffset() }
                .onChange(of: yOffset) { _ in sendOffset() }
                
                // Scrollable Bottom Section
                ScrollView {
                    VStack(spacing: 16) {
                        Text("Control Panel")
                            .font(.title2)
                            .fontWeight(.bold)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .padding([.top, .horizontal])
                        Picker("Mode", selection: $selectedMode) {
                            Text("Auto Aim").font(.title2).tag(0)
                            Text("Autonomous").font(.title2).tag(1)
                            Text("Manual").font(.title2).tag(2)
                        }
                        .pickerStyle(SegmentedPickerStyle())
                        .padding(.horizontal)
                        .font(.title2)
                        .onChange(of: selectedMode) { newValue in
                            switch newValue {
                            case 0:
                                _ = tcpManager.send(DataPacket.setAutoAim())
                            case 1:
                                _ = tcpManager.send(DataPacket.setAutonomous())
                            case 2:
                                _ = tcpManager.send(DataPacket.setManual())
                            default:
                                break
                            }
                        }
                        // Pitch and Yaw Control Sliders Side by Side
                        HStack(alignment: .top, spacing: 24) {
                            VStack(alignment: .leading, spacing: 8) {
                                HStack {
                                    Text("Pitch")
                                        .font(.headline)
                                    Spacer()
                                    Text(String(format: "%.2f", pitchDragValue ?? pitchValue))
                                        .font(.system(.body, design: .monospaced))
                                }
                                Slider(value: Binding(
                                    get: { pitchDragValue ?? pitchValue },
                                    set: { pitchValue = $0 }
                                ), in: -1.0...1.0, step: 0.01)
                                    .accentColor(.blue)
                                    .gesture(
                                        DragGesture(minimumDistance: 0)
                                            .updating($pitchDragValue) { value, state, _ in
                                                let width = UIScreen.main.bounds.width / 2 - 64 // estimate
                                                let percent = min(max(Double(value.location.x / width), 0), 1)
                                                let newValue = -1.0 + percent * 2.0
                                                let clamped = min(max(newValue, -1.0), 1.0)
                                                state = clamped
                                                _ = tcpManager.send(DataPacket.servoCommand(x: clamped, y: yawDragValue ?? yawValue, z: 0.0))
                                            }
                                            .onEnded { _ in
                                                withAnimation { pitchValue = 0.0 }
                                                _ = tcpManager.send(DataPacket.servoCommand(x: 0.0, y: yawDragValue ?? yawValue, z: 0.0))
                                            }
                                    )
                            }
                            VStack(alignment: .leading, spacing: 8) {
                                HStack {
                                    Text("Yaw")
                                        .font(.headline)
                                    Spacer()
                                    Text(String(format: "%.2f", yawDragValue ?? yawValue))
                                        .font(.system(.body, design: .monospaced))
                                }
                                Slider(value: Binding(
                                    get: { yawDragValue ?? yawValue },
                                    set: { yawValue = $0 }
                                ), in: -1.0...1.0, step: 0.01)
                                    .accentColor(.green)
                                    .gesture(
                                        DragGesture(minimumDistance: 0)
                                            .updating($yawDragValue) { value, state, _ in
                                                let width = UIScreen.main.bounds.width / 2 - 64 // estimate
                                                let percent = min(max(Double(value.location.x / width), 0), 1)
                                                let newValue = -1.0 + percent * 2.0
                                                let clamped = min(max(newValue, -1.0), 1.0)
                                                state = clamped
                                                _ = tcpManager.send(DataPacket.servoCommand(x: pitchDragValue ?? pitchValue, y: clamped, z: 0.0))
                                            }
                                            .onEnded { _ in
                                                withAnimation { yawValue = 0.0 }
                                                _ = tcpManager.send(DataPacket.servoCommand(x: pitchDragValue ?? pitchValue, y: 0.0, z: 0.0))
                                            }
                                    )
                            }
                        }
                        .padding(.horizontal)
                        // LAUNCH! and ESC Control Side by Side
                        HStack(alignment: .center, spacing: 16) {
                            Button(action: {
                                isLaunchPressed = true
                                _ = tcpManager.send(DataPacket.triggerCommand(true))
                                DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
                                    _ = tcpManager.send(DataPacket.triggerCommand(false))
                                    isLaunchPressed = false
                                }
                            }) {
                                Text("LAUNCH!")
                                    .font(.title)
                                    .fontWeight(.bold)
                                    .foregroundColor(.white)
                                    .frame(minWidth: 120)
                                    .padding()
                                    .background(isLaunchPressed ? Color.orange : Color.red)
                                    .cornerRadius(12)
                                    .shadow(radius: isLaunchPressed ? 0 : 4)
                                    .scaleEffect(isLaunchPressed ? 0.96 : 1.0)
                                    .animation(.easeInOut(duration: 0.1), value: isLaunchPressed)
                            }
                            Button(action: {
                                escValue = 0.0
                            }) {
                                Text("Reset")
                                    .font(.title)
                                    .fontWeight(.bold)
                                    .foregroundColor(.white)
                                    .frame(minWidth: 120)
                                    .padding()
                                    .background(Color.gray)
                                    .cornerRadius(12)
                                    .shadow(radius: 4)
                            }
                            VStack(alignment: .leading, spacing: 8) {
                                HStack {
                                    Text("ESC")
                                        .font(.headline)
                                    Spacer()
                                    Text(String(format: "%.2f", escValue))
                                        .font(.system(.body, design: .monospaced))
                                }
                                Slider(value: $escValue, in: 0.0...1.0, step: 0.01)
                                    .accentColor(.orange)
                            }
                            .frame(maxWidth: .infinity)
                        }
                        .padding(.horizontal)
                        // Pitch PI Controller
                        HStack(spacing: 16) {
                            Text("Pitch PI:")
                                .font(.headline)
                            // P
                            Text("P")
                            Stepper("", value: $pitchP, in: 0...10, step: 0.01)
                                .labelsHidden()
                            TextField("", value: $pitchP, formatter: Self.decimalFormatter)
                                .frame(width: 60)
                                .multilineTextAlignment(.center)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                            // I
                            Text("I")
                            Stepper("", value: $pitchI, in: 0...10, step: 0.01)
                                .labelsHidden()
                            TextField("", value: $pitchI, formatter: Self.decimalFormatter)
                                .frame(width: 60)
                                .multilineTextAlignment(.center)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                            Button("Reset") {
                                pitchP = 1.0; pitchI = 0.0
                            }
                            .buttonStyle(.bordered)
                            Button("Send") {
                                _ = tcpManager.send(DataPacket.tunePitch(p: pitchP, i: pitchI, d: 0.0))
                            }
                            .buttonStyle(.borderedProminent)
                        }
                        .padding(.horizontal)
                        // Yaw PI Controller
                        HStack(spacing: 16) {
                            Text("Yaw PI:")
                                .font(.headline)
                            // P
                            Text("P")
                            Stepper("", value: $yawP, in: 0...10, step: 0.01)
                                .labelsHidden()
                            TextField("", value: $yawP, formatter: Self.decimalFormatter)
                                .frame(width: 60)
                                .multilineTextAlignment(.center)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                            // I
                            Text("I")
                            Stepper("", value: $yawI, in: 0...10, step: 0.01)
                                .labelsHidden()
                            TextField("", value: $yawI, formatter: Self.decimalFormatter)
                                .frame(width: 60)
                                .multilineTextAlignment(.center)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                            Button("Reset") {
                                yawP = 1.0; yawI = 0.0
                            }
                            .buttonStyle(.bordered)
                            Button("Send") {
                                _ = tcpManager.send(DataPacket.tuneYaw(p: yawP, i: yawI, d: 0.0))
                            }
                            .buttonStyle(.borderedProminent)
                        }
                        .padding(.horizontal)
                        // Set Launch Threshold
                        HStack(spacing: 16) {
                            Text("Launch Threshold")
                                .font(.headline)
                                HStack(spacing: 4) {
                                    Text("N")
                                    Stepper("", value: $launchThresholdN, in: 1...100)
                                        .labelsHidden()
                                    TextField("", value: $launchThresholdN, formatter: NumberFormatter())
                                        .frame(width: 60)
                                        .multilineTextAlignment(.center)
                                        .textFieldStyle(RoundedBorderTextFieldStyle())
                                }
                                HStack(spacing: 4) {
                                    Text("Epsilon")
                                    Stepper("", value: $launchThresholdEpsilon, in: 0.0...1.0, step: 0.01)
                                        .labelsHidden()
                                    TextField("", value: $launchThresholdEpsilon, formatter: Self.decimalFormatter)
                                        .frame(width: 80)
                                        .multilineTextAlignment(.center)
                                        .textFieldStyle(RoundedBorderTextFieldStyle())
                                }
                                Button("Send") {
                                    _ = tcpManager.send(DataPacket.setLaunchThreshold(eps: launchThresholdEpsilon, n: UInt8(launchThresholdN)))
                                }
                                .buttonStyle(.borderedProminent)
                            }
                        .padding(.horizontal)
                        // Example text
                        Text("blablabla")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .padding(.horizontal)
                    }
                }
                .background(Color(.systemBackground))
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .opacity(tcpManager.isConnected ? 1.0 : 0.5)
            
            // Sidebar (Connection Settings)
            if isSidebarVisible {
                VStack {
                    List {
                        Section(header: Text("Connection Settings")) {
                            VStack(alignment: .leading, spacing: 10) {
                                Text("IP Address")
                                    .font(.caption)
                                TextField("IP Address", text: $settingsManager.serverIP)
                                    .textFieldStyle(RoundedBorderTextFieldStyle())
                                
                                Text("Port")
                                    .font(.caption)
                                TextField("Port", text: $settingsManager.serverPort)
                                    .textFieldStyle(RoundedBorderTextFieldStyle())
                            }
                            .padding(.vertical, 5)
                            if let error = tcpManager.connectionError, !error.isEmpty {
                                Text(error)
                                    .foregroundColor(.red)
                                    .font(.caption)
                                    .padding(.vertical, 2)
                            }
                            if tcpManager.isConnected {
                                Button("Disconnect") {
                                    tcpManager.disconnect()
                                }
                                .buttonStyle(.borderedProminent)
                                .tint(.red)
                            } else {
                                Button("Connect") {
                                    let rawPortString = settingsManager.serverPort
                                    guard !rawPortString.isEmpty else {
                                        print("⚠️ No port entered")
                                        return
                                    }
                                    guard let rawPortInt = Int(rawPortString) else {
                                        print("⚠️ Invalid port number: \(rawPortString)")
                                        return
                                    }
                                    guard (0...Int(UInt16.max)).contains(rawPortInt) else {
                                        print("⚠️ Port out of range: \(rawPortInt)")
                                        return
                                    }
                                    let port = UInt16(rawPortInt)
                                    tcpManager.connect(to: settingsManager.serverIP, port: port)
                                }
                                .buttonStyle(.borderedProminent)
                                .tint(.blue)
                            }
                        }
                        
                        Section(header: Text("Received Messages")) {
                            ScrollView {
                                LazyVStack(alignment: .leading, spacing: 8) {
                                    ForEach(tcpManager.receivedMessages, id: \.self) { message in
                                        Text(message)
                                            .font(.system(.body, design: .monospaced))
                                            .foregroundColor(.secondary)
                                    }
                                }
                                .padding(.vertical, 5)
                            }
                            .frame(height: 300)
                        }
                        Section(header: Text("Log Messages")) {
                            ScrollView {
                                LazyVStack(alignment: .leading, spacing: 8) {
                                    ForEach(tcpManager.receivedMessages.filter { $0.hasPrefix("Received Log:") }, id: \.self) { log in
                                        Text(log)
                                            .font(.system(.body, design: .monospaced))
                                            .foregroundColor(.orange)
                                    }
                                }
                                .padding(.vertical, 5)
                            }
                            .frame(height: 150)
                        }
                    }
                    .listStyle(SidebarListStyle())
                }
                .frame(width: 300)
                .background(Color(.systemBackground))
            }
        }
        .onChange(of: tcpManager.receivedMessages) { messages in
            // Update bounding box when new messages arrive
            if let lastMessage = messages.last {
                if lastMessage.contains("Received BboxPos:") {
                    let components = lastMessage.components(separatedBy: ": ")[1]
                        .components(separatedBy: ", ")
                    // Parse as Double and check for all-NaN
                    let x1 = Double(components[0].components(separatedBy: "=")[1])
                    let y1 = Double(components[1].components(separatedBy: "=")[1])
                    let x2 = Double(components[2].components(separatedBy: "=")[1])
                    let y2 = Double(components[3].components(separatedBy: "=")[1])
                    if [x1, y1, x2, y2].allSatisfy({ $0?.isNaN ?? true }) {
                        isBoundingBoxNaN = true
                        boundingBox = nil
                    } else if let x1 = x1, let y1 = y1, let x2 = x2, let y2 = y2 {
                        isBoundingBoxNaN = false
                        boundingBox = CGRect(
                            x: min(x1, x2),
                            y: min(y1, y2),
                            width: abs(x2 - x1),
                            height: abs(y2 - y1)
                        )
                        // Store recent N valid bbox centers for lock-on
                        let centerX = (x1 + x2) / 2
                        let centerY = (y1 + y2) / 2
                        if !centerX.isNaN && !centerY.isNaN {
                            recentBboxCenters.append((x: centerX, y: centerY))
                            if recentBboxCenters.count > launchThresholdN {
                                recentBboxCenters.removeFirst(recentBboxCenters.count - launchThresholdN)
                            }
                        }
                    } else {
                        isBoundingBoxNaN = true
                        boundingBox = nil
                    }
                } else if lastMessage.contains("Received PitchAngle:") {
                    // Parse pitch angle
                    if let angleStr = lastMessage.components(separatedBy: ": ").last,
                       let angle = Double(angleStr.trimmingCharacters(in: .whitespacesAndNewlines)) {
                        latestPitchAngle = angle
                    }
                }
            }
        }
        .onChange(of: escValue) { newValue in
            _ = tcpManager.send(DataPacket.escCommand(newValue))
        }
    }
    
    private func sendOffset() {
        _ = tcpManager.send(DataPacket.setOffset(x: xOffset, y: yOffset))
    }
}

extension Double {
    func rounded(toPlaces places: Int) -> Double {
        let divisor = pow(10.0, Double(places))
        return (self * divisor).rounded() / divisor
    }
}

// Helper for combinations
extension Array {
    func combinations(ofCount n: Int) -> [[Element]] {
        guard n > 0 else { return [[]] }
        guard let first = first else { return [] }
        let subcombos = Array(self.dropFirst()).combinations(ofCount: n - 1)
        var result = subcombos.map { [first] + $0 }
        result += Array(self.dropFirst()).combinations(ofCount: n)
        return result
    }
}

#Preview {
    MacAndiPadView()
}
