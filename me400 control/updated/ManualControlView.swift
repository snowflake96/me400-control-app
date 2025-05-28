import SwiftUI

struct PitchControlButton: View {
    let title: String
    let value: Double
    let isProminent: Bool
    @Binding var currentValue: Double
    let icon: String
    
    var body: some View {
        Button(title) {
            // No action needed
        }
        .buttonStyle(ControlButtonStyle(isProminent: isProminent, color: .blue, opacity: 0.1))
        .pressActions(
            onPress: {
                currentValue = value
            },
            onRelease: {
                withAnimation(.spring(response: 0.01, dampingFraction: 0.1)) {
                    currentValue = 0.0
                }
            }
        )
        .overlay(
            Image(systemName: icon)
                .font(.system(size: 40))
                .foregroundColor(.blue)
        )
    }
}

struct ControlButtonStyle: ButtonStyle {
    let isProminent: Bool
    let color: Color
    let opacity: Double
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .frame(width: 80, height: 80)
            .background(color.opacity(opacity))
            .foregroundColor(color)
            .cornerRadius(12)
            .scaleEffect(configuration.isPressed ? 0.95 : 1.0)
    }
}

extension View {
    func pressActions(onPress: @escaping () -> Void, onRelease: @escaping () -> Void) -> some View {
        self.simultaneousGesture(
            DragGesture(minimumDistance: 0)
                .onChanged { _ in
                    onPress()
                }
                .onEnded { _ in
                    onRelease()
                }
        )
    }
}

struct ManualESCControlView: View {
    @State private var escValue: Double = 0.0
    @State private var isLaunchPressed: Bool = false
    
    var body: some View {
        VStack(spacing: 20) {
            Text("ESC Control")
                .font(.headline)
            
            // ESC Slider
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("ESC")
                        .font(.headline)
                    Spacer()
                    Text(String(format: "%.2f", escValue))
                        .font(.system(.body, design: .monospaced))
                        .foregroundColor(.red)
                }
                Slider(value: $escValue, in: 0.0...1.0, step: 0.01)
                    .accentColor(.red)
                    .onChange(of: escValue) { _, newValue in
                        // Send ESC command to server
                        _ = ServerCommunicationManager.shared.send(DataPacket.escCommand(newValue))
                    }
            }
            
            // Reset Button
            Button("Reset") {
                withAnimation(.spring(response: 0.3, dampingFraction: 0.6)) {
                    escValue = 0.0
                }
            }
            .buttonStyle(.bordered)
            
            // LAUNCH! Button
            Button("LAUNCH!") {
                launchSequence()
            }
            .buttonStyle(.borderedProminent)
            .tint(isLaunchPressed ? .red : .blue)
            .scaleEffect(isLaunchPressed ? 1.1 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.6), value: isLaunchPressed)
        }
        .padding()
        .background(Color.gray.opacity(0.1))
        .cornerRadius(10)
    }
    
    private func launchSequence() {
        isLaunchPressed = true
        
        // Send trigger command true
        _ = ServerCommunicationManager.shared.send(DataPacket.triggerCommand(true))
        
        // After 1 second, send trigger command false
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            _ = ServerCommunicationManager.shared.send(DataPacket.triggerCommand(false))
            isLaunchPressed = false
        }
    }
}

struct ManualControlView: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    @State private var pitchValue: Double = 0.0
    @State private var yawValue: Double = 0.0
    
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                Text("Manual Control")
                    .font(.largeTitle)
                    .fontWeight(.bold)
                    .padding()
                
                // Pitch and Yaw Controls
                HStack(spacing: 20) {
                    // Pitch Control (Vertical)
                    VStack(alignment: .center, spacing: 10) {
                        Text("Pitch Control")
                            .font(.headline)
                        
                        Text("1.0")
                            .font(.caption)
                            .foregroundColor(.gray)
                        
                        HStack(spacing: 20) {
                            ZStack {
                                Slider(value: $pitchValue, in: -1...1, step: 0.01, onEditingChanged: { editing in
                                    if !editing {
                                        withAnimation(.spring(response: 0.01, dampingFraction: 0.1)) {
                                            pitchValue = 0.0
                                        }
                                    }
                                })
                                .rotationEffect(.degrees(-90))
                                .frame(width: 200, height: 20)
                                .onChange(of: pitchValue) { _, newValue in
                                    // Send servo command to server (pitch = x, yaw = y, z = 0)
                                    _ = ServerCommunicationManager.shared.send(DataPacket.servoCommand(x: newValue, y: yawValue, z: 0.0))
                                }
                            }
                            .frame(width: 20, height: 200)
                            
                            // Pitch buttons
                            VStack(spacing: 20) {
                                PitchControlButton(
                                    title: "",
                                    value: 0.5,
                                    isProminent: true,
                                    currentValue: $pitchValue,
                                    icon: "arrow.up.circle.fill"
                                )
                                
                                PitchControlButton(
                                    title: "",
                                    value: -0.5,
                                    isProminent: true,
                                    currentValue: $pitchValue,
                                    icon: "arrow.down.circle.fill"
                                )
                            }
                        }
                        
                        Text("-1.0")
                            .font(.caption)
                            .foregroundColor(.gray)
                        
                        Text("Pitch: \(pitchValue, specifier: "%.2f")")
                            .font(.caption)
                            .foregroundColor(.gray)
                    }
                    .padding()
                    .background(Color.gray.opacity(0.1))
                    .cornerRadius(10)
                    .onChange(of: pitchValue) { _, newValue in
                        // Send servo command when pitch changes
                        _ = ServerCommunicationManager.shared.send(DataPacket.servoCommand(x: newValue, y: yawValue, z: 0.0))
                    }
                    
                    // Yaw Control (Horizontal)
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Yaw Control")
                            .font(.headline)
                        
                        HStack {
                            Text("-1.0")
                                .font(.caption)
                                .foregroundColor(.gray)
                            
                            Slider(value: $yawValue, in: -1...1, step: 0.01, onEditingChanged: { editing in
                                if !editing {
                                    withAnimation(.spring(response: 0.01, dampingFraction: 0.1)) {
                                        yawValue = 0.0
                                    }
                                }
                            })
                            .onChange(of: yawValue) { _, newValue in
                                // Send servo command to server (pitch = x, yaw = y, z = 0)
                                _ = ServerCommunicationManager.shared.send(DataPacket.servoCommand(x: pitchValue, y: newValue, z: 0.0))
                            }
                            
                            Text("1.0")
                                .font(.caption)
                                .foregroundColor(.gray)
                        }
                        
                        Text("Current: \(yawValue, specifier: "%.2f")")
                            .font(.caption)
                            .foregroundColor(.gray)
                            
                        // Yaw buttons
                        HStack(spacing: 20) {
                            PitchControlButton(
                                title: "",
                                value: -0.5,
                                isProminent: true,
                                currentValue: $yawValue,
                                icon: "arrow.left.circle.fill"
                            )
                            
                            PitchControlButton(
                                title: "",
                                value: 0.5,
                                isProminent: true,
                                currentValue: $yawValue,
                                icon: "arrow.right.circle.fill"
                            )
                        }
                    }
                    .padding()
                    .background(Color.gray.opacity(0.1))
                    .cornerRadius(10)
                    .onChange(of: yawValue) { _, newValue in
                        // Send servo command when yaw changes
                        _ = ServerCommunicationManager.shared.send(DataPacket.servoCommand(x: pitchValue, y: newValue, z: 0.0))
                    }
                }
                
                // ESC Control
                ManualESCControlView()
            }
            .padding()
        }
    }
}

#Preview {
    ManualControlView()
} 
 
