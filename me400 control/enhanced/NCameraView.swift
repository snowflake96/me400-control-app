import SwiftUI

// MARK: - Camera View with Bounding Box
struct NCameraView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    // Camera aspect ratio 16:9
    private let aspectRatio: CGFloat = 16.0 / 9.0
    
    // Zoomed view range
    private let viewRange: Double = 0.25 // Shows -0.25 to 0.25
    private let gridInterval: Double = 0.05
    
    var body: some View {
        GeometryReader { geometry in
            let viewSize = calculateViewSize(in: geometry.size)
            
            ZStack {
                // Background
                Rectangle()
                    .fill(Color.black.opacity(0.9))
                    .frame(width: viewSize.width, height: viewSize.height)
                    .overlay(
                        // Dense grid with coordinate labels
                        DenseGridOverlay(viewRange: viewRange, gridInterval: gridInterval)
                    )
                
                // Crosshair at offset position
                CrosshairView(
                    offsetX: settingsStore.targetOffsetX,
                    offsetY: settingsStore.targetOffsetY,
                    viewRange: viewRange
                )
                .frame(width: viewSize.width, height: viewSize.height)
                
                // Bounding Box (if available)
                if let bbox = coordinator.systemState.boundingBox {
                    ZoomedBoundingBoxView(
                        bbox: bbox,
                        viewSize: viewSize,
                        viewRange: viewRange,
                        color: .yellow.opacity(0.8)
                    )
                }
                
                // Filtered Bounding Box (if available)
                if let filteredBbox = coordinator.systemState.filteredBoundingBox {
                    ZoomedFilteredBoxView(
                        centerX: filteredBbox.centerX,
                        centerY: filteredBbox.centerY,
                        viewSize: viewSize,
                        viewRange: viewRange,
                        color: .green
                    )
                }
                
                // Info overlay
                VStack {
                    HStack {
                        CameraInfoView()
                        Spacer()
                    }
                    Spacer()
                    
                    // Error info at bottom left
                    HStack {
                        ErrorInfoView()
                        Spacer()
                    }
                }
                .frame(width: viewSize.width, height: viewSize.height)
                .padding()
            }
            .frame(width: viewSize.width, height: viewSize.height)
            .position(x: geometry.size.width / 2, y: geometry.size.height / 2)
            .clipped()
            .padding(.horizontal, 20) // Add horizontal padding
        }
    }
    
    private func calculateViewSize(in containerSize: CGSize) -> CGSize {
        // Account for padding
        let adjustedWidth = containerSize.width - 40 // 20 padding on each side
        let containerAspectRatio = adjustedWidth / containerSize.height
        
        if containerAspectRatio > aspectRatio {
            // Container is wider, fit by height
            let height = containerSize.height
            let width = height * aspectRatio
            return CGSize(width: width, height: height)
        } else {
            // Container is taller, fit by width
            let width = adjustedWidth
            let height = width / aspectRatio
            return CGSize(width: width, height: height)
        }
    }
}

// MARK: - Dense Grid Overlay with Coordinates
struct DenseGridOverlay: View {
    let viewRange: Double
    let gridInterval: Double
    
    // Specific coordinates to label
    let labeledCoordinates: [Double] = [-0.4, -0.3, -0.25, -0.2, -0.15, -0.1, 0, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4]
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Vertical lines with labels
                ForEach(Array(stride(from: -viewRange, through: viewRange, by: gridInterval)), id: \.self) { x in
                    let xPos = normalizedToView(x, range: viewRange, size: geometry.size.width)
                    
                    Path { path in
                        path.move(to: CGPoint(x: xPos, y: 0))
                        path.addLine(to: CGPoint(x: xPos, y: geometry.size.height))
                    }
                    .stroke(x == 0 ? Color.white.opacity(0.8) : Color.white.opacity(0.3), 
                           lineWidth: x == 0 ? 3 : 0.5) // Thicker axis line
                    
                    // Coordinate label - only show specific coordinates
                    if labeledCoordinates.contains(where: { abs($0 - x) < 0.001 }) {
                        Text(x == 0 ? "0" : String(format: "%.2f", x))
                            .font(.system(size: 10))
                            .foregroundColor(.white.opacity(0.7))
                            .position(x: xPos, y: geometry.size.height - 10)
                    }
                }
                
                // Horizontal lines with labels
                ForEach(Array(stride(from: -viewRange, through: viewRange, by: gridInterval)), id: \.self) { y in
                    let yPos = normalizedToView(-y, range: viewRange, size: geometry.size.height) // Inverted
                    
                    Path { path in
                        path.move(to: CGPoint(x: 0, y: yPos))
                        path.addLine(to: CGPoint(x: geometry.size.width, y: yPos))
                    }
                    .stroke(y == 0 ? Color.white.opacity(0.8) : Color.white.opacity(0.3), 
                           lineWidth: y == 0 ? 3 : 0.5) // Thicker axis line
                    
                    // Coordinate label - only show specific coordinates
                    if labeledCoordinates.contains(where: { abs($0 - y) < 0.001 }) {
                        Text(y == 0 ? "0" : String(format: "%.2f", y))
                            .font(.system(size: 10))
                            .foregroundColor(.white.opacity(0.7))
                            .position(x: 15, y: yPos)
                    }
                }
            }
        }
    }
    
    private func normalizedToView(_ value: Double, range: Double, size: CGFloat) -> CGFloat {
        return CGFloat((value + range) / (2 * range)) * size
    }
}

// MARK: - Crosshair View (Zoomed)
struct CrosshairView: View {
    let offsetX: Double
    let offsetY: Double
    let viewRange: Double
    
    var body: some View {
        GeometryReader { geometry in
            // Check if crosshair is within view
            if abs(offsetX) <= viewRange && abs(offsetY) <= viewRange {
                let targetX = normalizedToView(offsetX, range: viewRange, size: geometry.size.width)
                let targetY = normalizedToView(-offsetY, range: viewRange, size: geometry.size.height) // Inverted
                
                ZStack {
                    // Crosshair
                    Path { path in
                        // Horizontal line
                        path.move(to: CGPoint(x: targetX - 20, y: targetY))
                        path.addLine(to: CGPoint(x: targetX - 5, y: targetY))
                        path.move(to: CGPoint(x: targetX + 5, y: targetY))
                        path.addLine(to: CGPoint(x: targetX + 20, y: targetY))
                        
                        // Vertical line
                        path.move(to: CGPoint(x: targetX, y: targetY - 20))
                        path.addLine(to: CGPoint(x: targetX, y: targetY - 5))
                        path.move(to: CGPoint(x: targetX, y: targetY + 5))
                        path.addLine(to: CGPoint(x: targetX, y: targetY + 20))
                    }
                    .stroke(Color.red, lineWidth: 2)
                    
                    // Center circle
                    Circle()
                        .stroke(Color.red, lineWidth: 2)
                        .frame(width: 10, height: 10)
                        .position(x: targetX, y: targetY)
                }
            } else {
                // Draw arrow pointing to crosshair location
                DirectionalArrow(
                    targetX: offsetX,
                    targetY: offsetY,
                    viewRange: viewRange,
                    color: .red
                )
            }
        }
    }
    
    private func normalizedToView(_ value: Double, range: Double, size: CGFloat) -> CGFloat {
        return CGFloat((value + range) / (2 * range)) * size
    }
}

// MARK: - Zoomed Bounding Box View
struct ZoomedBoundingBoxView: View {
    let bbox: BoundingBox
    let viewSize: CGSize
    let viewRange: Double
    let color: Color
    
    var body: some View {
        GeometryReader { _ in
            let centerX = bbox.centerX
            let centerY = bbox.centerY
            
            // Show arrow if center is outside ±0.4 range
            if abs(centerX) > 0.4 || abs(centerY) > 0.4 {
                DirectionalArrow(
                    targetX: centerX,
                    targetY: centerY,
                    viewRange: viewRange,
                    color: color
                )
            } else {
                // Draw the box if center is within ±0.4
                let x1 = normalizedToView(bbox.x1, range: viewRange, size: viewSize.width)
                let y1 = normalizedToView(-bbox.y1, range: viewRange, size: viewSize.height)
                let x2 = normalizedToView(bbox.x2, range: viewRange, size: viewSize.width)
                let y2 = normalizedToView(-bbox.y2, range: viewRange, size: viewSize.height)
                
                let minX = min(x1, x2)
                let minY = min(y1, y2)
                let width = abs(x2 - x1)
                let height = abs(y2 - y1)
                
                Rectangle()
                    .stroke(color, lineWidth: 2)
                    .frame(width: width, height: height)
                    .position(x: minX + width/2, y: minY + height/2)
            }
        }
        .frame(width: viewSize.width, height: viewSize.height)
    }
    
    private func normalizedToView(_ value: Double, range: Double, size: CGFloat) -> CGFloat {
        return CGFloat((value + range) / (2 * range)) * size
    }
}

// MARK: - Zoomed Filtered Box View
struct ZoomedFilteredBoxView: View {
    let centerX: Double
    let centerY: Double
    let viewSize: CGSize
    let viewRange: Double
    let color: Color
    
    var body: some View {
        GeometryReader { _ in
            // Show arrow if center is outside ±0.4 range
            if abs(centerX) > 0.4 || abs(centerY) > 0.4 {
                DirectionalArrow(
                    targetX: centerX,
                    targetY: centerY,
                    viewRange: viewRange,
                    color: color
                )
            } else {
                // Draw the filtered position if within ±0.4
                let x = normalizedToView(centerX, range: viewRange, size: viewSize.width)
                let y = normalizedToView(-centerY, range: viewRange, size: viewSize.height)
                
                ZStack {
                    // Filtered position marker
                    Circle()
                        .fill(color.opacity(0.3))
                        .frame(width: 10, height: 10)
                        .position(x: x, y: y)
                    
                    Circle()
                        .stroke(color, lineWidth: 2)
                        .frame(width: 10, height: 10)
                        .position(x: x, y: y)
                    
                    // Small crosshair
                    Path { path in
                        path.move(to: CGPoint(x: x - 5, y: y))
                        path.addLine(to: CGPoint(x: x + 5, y: y))
                        path.move(to: CGPoint(x: x, y: y - 5))
                        path.addLine(to: CGPoint(x: x, y: y + 5))
                    }
                    .stroke(color, lineWidth: 1)
                }
            }
        }
        .frame(width: viewSize.width, height: viewSize.height)
    }
    
    private func normalizedToView(_ value: Double, range: Double, size: CGFloat) -> CGFloat {
        return CGFloat((value + range) / (2 * range)) * size
    }
}

// MARK: - Directional Arrow
struct DirectionalArrow: View {
    let targetX: Double
    let targetY: Double
    let viewRange: Double
    let color: Color
    
    var body: some View {
        GeometryReader { geometry in
            // Calculate angle from center to target
            let angle = atan2(-targetY, targetX) // Negative Y for inverted coordinates
            
            // Calculate where the line from origin to target intersects the view boundary
            let boundaryPoint = calculateBoundaryIntersection(
                targetX: targetX,
                targetY: targetY,
                viewRange: viewRange,
                viewSize: geometry.size
            )
            
            // Move arrow further from border
            let insetPoint = CGPoint(
                x: boundaryPoint.x + cos(angle + .pi) * 60, // Increased from 30
                y: boundaryPoint.y + sin(angle + .pi) * 60
            )
            
            Path { path in
                // Larger arrow shaft
                let shaftStart = CGPoint(
                    x: insetPoint.x + cos(angle + .pi) * 40, // Increased from 30
                    y: insetPoint.y + sin(angle + .pi) * 40
                )
                path.move(to: shaftStart)
                path.addLine(to: insetPoint)
                
                // Larger arrow head
                let arrowAngle1 = angle + .pi * 0.75
                let arrowAngle2 = angle - .pi * 0.75
                
                path.move(to: insetPoint)
                path.addLine(to: CGPoint(
                    x: insetPoint.x + cos(arrowAngle1) * 20, // Increased from 10
                    y: insetPoint.y + sin(arrowAngle1) * 20
                ))
                
                path.move(to: insetPoint)
                path.addLine(to: CGPoint(
                    x: insetPoint.x + cos(arrowAngle2) * 20, // Increased from 10
                    y: insetPoint.y + sin(arrowAngle2) * 20
                ))
            }
            .stroke(color, lineWidth: 4) // Increased from 3
            
            // Distance label
            Text(String(format: "%.2f", sqrt(targetX * targetX + targetY * targetY)))
                .font(.system(size: 12, weight: .bold)) // Increased from 10
                .foregroundColor(color)
                .padding(.horizontal, 4)
                .padding(.vertical, 2)
                .background(Color.black.opacity(0.8))
                .cornerRadius(4)
                .position(
                    x: insetPoint.x + cos(angle + .pi) * 70, // Adjusted position
                    y: insetPoint.y + sin(angle + .pi) * 70
                )
        }
    }
    
    private func calculateBoundaryIntersection(targetX: Double, targetY: Double, viewRange: Double, viewSize: CGSize) -> CGPoint {
        // Normalize target coordinates to view range
        let normalizedX = targetX / viewRange
        let normalizedY = -targetY / viewRange // Inverted Y
        
        // Find intersection with view boundary
        let absX = abs(normalizedX)
        let absY = abs(normalizedY)
        
        var intersectionX: Double
        var intersectionY: Double
        
        if absX > absY {
            // Intersects left or right edge
            intersectionX = normalizedX > 0 ? 1 : -1
            intersectionY = normalizedY / absX
        } else {
            // Intersects top or bottom edge
            intersectionY = normalizedY > 0 ? 1 : -1
            intersectionX = normalizedX / absY
        }
        
        // Convert back to view coordinates
        let viewX = (intersectionX + 1) * viewSize.width / 2
        let viewY = (intersectionY + 1) * viewSize.height / 2
        
        return CGPoint(x: viewX, y: viewY)
    }
}

// MARK: - Camera Info View
struct CameraInfoView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Camera View (Zoomed ±0.25)")
                .font(.caption)
                .fontWeight(.semibold)
            
            if let bbox = coordinator.systemState.boundingBox {
                Text("BBox: (\(bbox.centerX, specifier: "%.3f"), \(bbox.centerY, specifier: "%.3f"))")
                    .font(.caption2)
                    .foregroundColor(.yellow)
            } else {
                Text("BBox: no detection")
                    .font(.caption2)
                    .foregroundColor(.yellow.opacity(0.6))
            }
            
            if let filtered = coordinator.systemState.filteredBoundingBox {
                Text("Filtered: (\(filtered.centerX, specifier: "%.3f"), \(filtered.centerY, specifier: "%.3f"))")
                    .font(.caption2)
                    .foregroundColor(.green)
            } else {
                Text("Filtered: no detection")
                    .font(.caption2)
                    .foregroundColor(.green.opacity(0.6))
            }
            
            Text("Target: (\(settingsStore.targetOffsetX, specifier: "%.3f"), \(settingsStore.targetOffsetY, specifier: "%.3f"))")
                .font(.caption2)
                .foregroundColor(.red)
            
            Text("Cutoff: \(coordinator.systemState.cutoffFrequency, specifier: "%.1f") Hz")
                .font(.caption2)
                .foregroundColor(.secondary)
        }
        .padding(8)
        .background(Color.black.opacity(0.7))
        .cornerRadius(6)
    }
}

// MARK: - Y Offset Control View
struct YOffsetControlView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        VStack(spacing: 8) {
            Text("Y Offset")
                .font(.caption)
                .fontWeight(.medium)
            VStack(spacing:4){
                HStack(spacing: 20) {
                    Button("-") {
                        settingsStore.targetOffsetY = max(-0.2, settingsStore.targetOffsetY - 0.001)
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .disabled(!coordinator.connectionState.isConnected)
                    
//                    Text(String(format: "%.3f", settingsStore.targetOffsetY))
//                        .font(.system(size: 10, design: .monospaced))
//                        .frame(width: 40)
                    
                    Button("+") {
                        settingsStore.targetOffsetY = min(0.2, settingsStore.targetOffsetY + 0.001)
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .disabled(!coordinator.connectionState.isConnected)
                }
                Text(String(format: "%.3f", settingsStore.targetOffsetY))
                    .font(.system(size: 15, design: .monospaced))
                    .frame(width: 75)
            }
            
            // Rotated vertical slider
            GeometryReader { geometry in
                RotatedVerticalSlider(
                    value: $settingsStore.targetOffsetY,
                    in: -0.2...0.2,
                    step: 0.001
                )
                .frame(width: geometry.size.height, height: 40)
                .rotationEffect(.degrees(-90))
                .position(x: geometry.size.width / 2, y: geometry.size.height / 2)
            }
            .frame(width: 40)
            .clipped()
            
            Text("Y")
                .font(.caption)
                .foregroundColor(.secondary)
            
            Text("Move up if\nball goes\nabove")
                .font(.system(size: 10))
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .lineLimit(3)
        }
        .padding(10)
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        .opacity(coordinator.connectionState.isConnected ? 1.0 : 0.6)
    }
}

// MARK: - X Offset Control View
struct XOffsetControlView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        HStack(spacing: 20) {
            // Title
            Text("X Offset\nControl")
                .font(.system(size: 14, weight: .medium))
                .multilineTextAlignment(.center)
                .foregroundColor(.secondary)
                .frame(width: 60)
            
            // X Offset slider
            VStack(spacing: 8) {
                HStack {
                    Text("X")
                        .frame(width: 20)
                        .foregroundColor(.secondary)
                    
                    Button("-") {
                        settingsStore.targetOffsetX = max(-0.2, settingsStore.targetOffsetX - 0.001)
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .disabled(!coordinator.connectionState.isConnected)
                    
                    Slider(
                        value: $settingsStore.targetOffsetX,
                        in: -0.2...0.2,
                        step: 0.001
                    )
                    .disabled(!coordinator.connectionState.isConnected)
                    
                    Button("+") {
                        settingsStore.targetOffsetX = min(0.2, settingsStore.targetOffsetX + 0.001)
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .disabled(!coordinator.connectionState.isConnected)
                    
                    Text(String(format: "%.3f", settingsStore.targetOffsetX))
                        .frame(width: 80)
                        .font(.system(.body, design: .monospaced))
                }
                
                Text("X: Move right if ball goes right")
                    .font(.caption2)
                    .foregroundColor(.secondary)
                }
            
//                .frame(maxWidth: .infinity)
            
            // Buttons - horizontal layout
            HStack(spacing: 12) {
                Button("Reset") {
                    settingsStore.targetOffsetX = 0
                    settingsStore.targetOffsetY = 0
                }
                .buttonStyle(.bordered)
                .frame(width: 80, height: 36)
                .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
                
                Button("Send") {
                    Task {
                        try? await coordinator.setOffset(
                            x: settingsStore.targetOffsetX,
                            y: settingsStore.targetOffsetY,
                            z: 0
                        )
                    }
                }
                .buttonStyle(.borderedProminent)
                .frame(width: 80, height: 36)
                .disabled(!coordinator.connectionState.isConnected || !coordinator.isSynchronized)
            }
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        .opacity(coordinator.connectionState.isConnected ? 1.0 : 0.6)
    }
}

// MARK: - Rotated Vertical Slider
struct RotatedVerticalSlider: View {
    @Binding var value: Double
    let `in`: ClosedRange<Double>
    let step: Double
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Background track
                RoundedRectangle(cornerRadius: 3)
                    .fill(Color(.systemGray4))
                    .frame(width: geometry.size.width, height: 6)
                
                // Zero line indicator
                let zeroPosition = geometry.size.width * ((0 - `in`.lowerBound) / (`in`.upperBound - `in`.lowerBound))
                Rectangle()
                    .fill(Color.red.opacity(0.5))
                    .frame(width: 2, height: geometry.size.height)
                    .position(x: zeroPosition, y: geometry.size.height / 2)
                
                // Standard horizontal slider
                Slider(
                    value: $value,
                    in: `in`,
                    step: step
                )
                .accentColor(.blue)
            }
        }
    }
}

// MARK: - Offset Control View (keeping for backward compatibility)
struct OffsetControlView: View {
    var body: some View {
        EmptyView()
    }
}

// MARK: - Vertical Slider (keeping the original implementation)
struct VerticalSlider: View {
    @Binding var value: Double
    let `in`: ClosedRange<Double>
    let step: Double
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Background track
                RoundedRectangle(cornerRadius: 3)
                    .fill(Color(.systemGray4))
                    .frame(width: 6)
                    .position(x: geometry.size.width / 2, y: geometry.size.height / 2)
                
                // Value indicator
                let normalizedValue = (value - `in`.lowerBound) / (`in`.upperBound - `in`.lowerBound)
                let yPosition = geometry.size.height * (1 - normalizedValue)
                
                Circle()
                    .fill(Color.accentColor)
                    .frame(width: 20, height: 20)
                    .position(x: geometry.size.width / 2, y: yPosition)
                
                // Zero line indicator
                let zeroPosition = geometry.size.height * (1 - (0 - `in`.lowerBound) / (`in`.upperBound - `in`.lowerBound))
                Rectangle()
                    .fill(Color.red.opacity(0.5))
                    .frame(width: geometry.size.width, height: 1)
                    .position(x: geometry.size.width / 2, y: zeroPosition)
            }
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { drag in
                        let normalizedY = 1 - (drag.location.y / geometry.size.height)
                        let newValue = normalizedY * (`in`.upperBound - `in`.lowerBound) + `in`.lowerBound
                        
                        // Apply step
                        let steppedValue = round(newValue / step) * step
                        value = max(`in`.lowerBound, min(`in`.upperBound, steppedValue))
                    }
            )
        }
    }
}

// MARK: - Error Info View
struct ErrorInfoView: View {
    @EnvironmentObject var coordinator: ControlCoordinator
    @EnvironmentObject var settingsStore: SettingsStore
    
    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            if let bbox = coordinator.systemState.boundingBox {
                let bboxErrorX = bbox.centerX - settingsStore.targetOffsetX
                let bboxErrorY = bbox.centerY - settingsStore.targetOffsetY
                
                Text("BBox Error: (\(bboxErrorX, specifier: "%.3f"), \(bboxErrorY, specifier: "%.3f"))")
                    .font(.caption2)
                    .foregroundColor(.yellow)
            } else {
                Text("BBox Error: no detection")
                    .font(.caption2)
                    .foregroundColor(.yellow.opacity(0.6))
            }
            
            if let filtered = coordinator.systemState.filteredBoundingBox {
                let filteredErrorX = filtered.centerX - settingsStore.targetOffsetX
                let filteredErrorY = filtered.centerY - settingsStore.targetOffsetY
                
                Text("Filtered Error: (\(filteredErrorX, specifier: "%.3f"), \(filteredErrorY, specifier: "%.3f"))")
                    .font(.caption2)
                    .foregroundColor(.green)
            } else {
                Text("Filtered Error: no detection")
                    .font(.caption2)
                    .foregroundColor(.green.opacity(0.6))
            }
        }
        .padding(8)
        .background(Color.black.opacity(0.7))
        .cornerRadius(6)
    }
} 
