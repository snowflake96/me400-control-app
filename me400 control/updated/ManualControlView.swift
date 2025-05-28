import SwiftUI

struct ManualControlView: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    @State private var pitchValue: Double = 0.0
    @State private var yawValue: Double = 0.0
    
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                // Pitch and Yaw Controls
                HStack(spacing: 20) {
                    // Pitch Control (Vertical)
                    VStack(alignment: .center, spacing: 10) {
                        Text("1.0")
                            .font(.caption)
                            .foregroundColor(.gray)
                        
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
                                // TODO: Send pitch value to server
                            }
                        }
                        .frame(width: 20, height: 200)
                        
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
                                // TODO: Send yaw value to server
                            }
                            
                            Text("1.0")
                                .font(.caption)
                                .foregroundColor(.gray)
                        }
                        
                        Text("Current: \(yawValue, specifier: "%.2f")")
                            .font(.caption)
                            .foregroundColor(.gray)
                    }
                    .padding()
                    .background(Color.gray.opacity(0.1))
                    .cornerRadius(10)
                }
                
                // ESC Control
                ESCControlView()
            }
            .padding()
        }
    }
}

#Preview {
    ManualControlView()
} 
 