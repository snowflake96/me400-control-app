import SwiftUI

struct ReactiveStepDoubleInputBox: View {
    let title: String
    @Binding var value: Double
    let minValue: Double
    let maxValue: Double
    let stepSizeKey: String
    let stepSizeDefault: Double
    let format: String
    
    @ObservedObject private var parameterManager = ParameterManager.shared
    
    var body: some View {
        let currentStepSize = parameterManager.getParameter(stepSizeKey, defaultValue: stepSizeDefault)
        
        return DoubleInputBox(
            title: title,
            value: $value,
            minValue: minValue,
            maxValue: maxValue,
            stepSize: currentStepSize,
            format: format
        )
        .id("\(stepSizeKey)-\(currentStepSize)")  // Force recreation when step size changes
    }
}

struct PitchControl: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    @ObservedObject private var serverManager = ServerCommunicationManager.shared
    
    var body: some View {
        VStack(spacing: 15) {
            Text("Pitch Control")
                .font(.headline)
                .fontWeight(.semibold)
            
            // PI Parameters
            VStack(spacing: 10) {
                HStack(spacing: 15) {
                    VStack(spacing: 5) {
                        DoubleInputBox(
                            title: "P",
                            value: $parameterManager.pitchP,
                            minValue: -1000.0,
                            maxValue: 1000.0,
                            stepSize: parameterManager.pitchPStepSize,
                            format: "%.1f"
                        )
                        .id("pitchP-\(parameterManager.pitchPStepSize)")
                        
                        DoubleInputBox(
                            title: "P Step",
                            value: $parameterManager.pitchPStepSize,
                            minValue: 0.001,
                            maxValue: 100.0,
                            stepSize: 0.1,
                            format: "%.3f"
                        )
                    }
                    
                    VStack(spacing: 5) {
                        DoubleInputBox(
                            title: "I",
                            value: $parameterManager.pitchI,
                            minValue: -1000.0,
                            maxValue: 1000.0,
                            stepSize: parameterManager.pitchIStepSize,
                            format: "%.3f"
                        )
                        .id("pitchI-\(parameterManager.pitchIStepSize)")
                        
                        DoubleInputBox(
                            title: "I Step",
                            value: $parameterManager.pitchIStepSize,
                            minValue: 0.001,
                            maxValue: 10.0,
                            stepSize: 0.01,
                            format: "%.3f"
                        )
                    }
                }
                
                Button(action: {
                    _ = serverManager.send(DataPacket.tunePitch(p: parameterManager.pitchP, i: parameterManager.pitchI, d: 0.0))
                }) {
                    Text("Send PI")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(parameterManager.serverConnected ? Color.blue : Color.gray)
                        .cornerRadius(10)
                }
                .disabled(!parameterManager.serverConnected)
            }
            
            // Integral Limit
            VStack(spacing: 10) {
                HStack(spacing: 15) {
                    DoubleInputBox(
                        title: "Integral Limit",
                        value: $parameterManager.pitchIntegralLimit,
                        minValue: 0.0,
                        maxValue: 1000.0,
                        stepSize: parameterManager.pitchIntegralLimitStepSize,
                        format: "%.3f"
                    )
                    .id("pitchIntegralLimit-\(parameterManager.pitchIntegralLimitStepSize)")
                    
                    DoubleInputBox(
                        title: "Limit Step",
                        value: $parameterManager.pitchIntegralLimitStepSize,
                        minValue: 0.001,
                        maxValue: 10.0,
                        stepSize: 0.01,
                        format: "%.3f"
                    )
                }
                
                Button(action: {
                    _ = serverManager.send(DataPacket.setPitchIntegralLimit(parameterManager.pitchIntegralLimit))
                }) {
                    Text("Send Integral Limit")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(parameterManager.serverConnected ? Color.green : Color.gray)
                        .cornerRadius(10)
                }
                .disabled(!parameterManager.serverConnected)
            }
        }
        .padding()
        .background(Color(.systemBackground))
        .cornerRadius(12)
        .shadow(radius: 2)
    }
}

struct YawControl: View {
    @ObservedObject private var parameterManager = ParameterManager.shared
    @ObservedObject private var serverManager = ServerCommunicationManager.shared
    
    var body: some View {
        VStack(spacing: 15) {
            Text("Yaw Control")
                .font(.headline)
                .fontWeight(.semibold)
            
            // PI Parameters
            VStack(spacing: 10) {
                HStack(spacing: 15) {
                    VStack(spacing: 5) {
                        DoubleInputBox(
                            title: "P",
                            value: $parameterManager.yawP,
                            minValue: -1000.0,
                            maxValue: 1000.0,
                            stepSize: parameterManager.yawPStepSize,
                            format: "%.1f"
                        )
                        .id("yawP-\(parameterManager.yawPStepSize)")
                        
                        DoubleInputBox(
                            title: "P Step",
                            value: $parameterManager.yawPStepSize,
                            minValue: 0.001,
                            maxValue: 100.0,
                            stepSize: 0.1,
                            format: "%.3f"
                        )
                    }
                    
                    VStack(spacing: 5) {
                        DoubleInputBox(
                            title: "I",
                            value: $parameterManager.yawI,
                            minValue: -1000.0,
                            maxValue: 1000.0,
                            stepSize: parameterManager.yawIStepSize,
                            format: "%.3f"
                        )
                        .id("yawI-\(parameterManager.yawIStepSize)")
                        
                        DoubleInputBox(
                            title: "I Step",
                            value: $parameterManager.yawIStepSize,
                            minValue: 0.001,
                            maxValue: 10.0,
                            stepSize: 0.01,
                            format: "%.3f"
                        )
                    }
                }
                
                Button(action: {
                    _ = serverManager.send(DataPacket.tuneYaw(p: parameterManager.yawP, i: parameterManager.yawI, d: 0.0))
                }) {
                    Text("Send PI")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(parameterManager.serverConnected ? Color.blue : Color.gray)
                        .cornerRadius(10)
                }
                .disabled(!parameterManager.serverConnected)
            }
            
            // Integral Limit
            VStack(spacing: 10) {
                HStack(spacing: 15) {
                    DoubleInputBox(
                        title: "Integral Limit",
                        value: $parameterManager.yawIntegralLimit,
                        minValue: 0.0,
                        maxValue: 1000.0,
                        stepSize: parameterManager.yawIntegralLimitStepSize,
                        format: "%.3f"
                    )
                    .id("yawIntegralLimit-\(parameterManager.yawIntegralLimitStepSize)")
                    
                    DoubleInputBox(
                        title: "Limit Step",
                        value: $parameterManager.yawIntegralLimitStepSize,
                        minValue: 0.001,
                        maxValue: 10.0,
                        stepSize: 0.01,
                        format: "%.3f"
                    )
                }
                
                Button(action: {
                    _ = serverManager.send(DataPacket.setYawIntegralLimit(parameterManager.yawIntegralLimit))
                }) {
                    Text("Send Integral Limit")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(parameterManager.serverConnected ? Color.green : Color.gray)
                        .cornerRadius(10)
                }
                .disabled(!parameterManager.serverConnected)
            }
        }
        .padding()
        .background(Color(.systemBackground))
        .cornerRadius(12)
        .shadow(radius: 2)
    }
}

struct AutoAimView: View {
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                Text("AutoAim Mode")
                    .font(.largeTitle)
                    .fontWeight(.bold)
                    .padding()
                
                Text("Configure PID parameters for automatic aiming")
                    .font(.title2)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
                    .padding(.horizontal)
                
                // Pitch Control
                PitchControl()
                
                Divider()
                    .padding(.vertical)
                
                // Yaw Control
                YawControl()
                
                Spacer()
            }
            .padding()
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemGroupedBackground))
    }
}

#Preview {
    AutoAimView()
} 