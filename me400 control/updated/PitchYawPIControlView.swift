import SwiftUI

struct PitchYawPIControlView: View {
    @State private var pitchP: Double = 10.0
    @State private var pitchI: Double = 0.0
    @State private var pitchIntegralLimit: Double = 1.0
    
    @State private var yawP: Double = 10.0
    @State private var yawI: Double = 0.0
    @State private var yawIntegralLimit: Double = 1.0
    
    var body: some View {
        VStack(spacing: 20) {
            Text("PI Control Parameters")
                .font(.headline)
            
            HStack(spacing: 40) {
                // Pitch PI Control
                VStack(alignment: .leading, spacing: 15) {
                    Text("Pitch Control")
                        .font(.headline)
                    
                    // P Value
                    DoubleInputBox(
                        title: "P",
                        value: $pitchP,
                        minValue: 0.0,
                        maxValue: 100.0,
                        stepSize: 0.1,
                        format: "%.1f",
                        allowNegative: false
                    )
                    
                    // I Value
                    DoubleInputBox(
                        title: "I",
                        value: $pitchI,
                        minValue: 0.0,
                        maxValue: 100.0,
                        stepSize: 0.1,
                        format: "%.1f",
                        allowNegative: false
                    )
                    
                    // Integral Limit
                    DoubleInputBox(
                        title: "Integral Limit",
                        value: $pitchIntegralLimit,
                        minValue: 0.0,
                        maxValue: 10.0,
                        stepSize: 0.1,
                        format: "%.1f",
                        allowNegative: false
                    )
                }
                
                // Yaw PI Control
                VStack(alignment: .leading, spacing: 15) {
                    Text("Yaw Control")
                        .font(.headline)
                    
                    // P Value
                    DoubleInputBox(
                        title: "P",
                        value: $yawP,
                        minValue: 0.0,
                        maxValue: 100.0,
                        stepSize: 0.1,
                        format: "%.1f",
                        allowNegative: false
                    )
                    
                    // I Value
                    DoubleInputBox(
                        title: "I",
                        value: $yawI,
                        minValue: 0.0,
                        maxValue: 100.0,
                        stepSize: 0.1,
                        format: "%.1f",
                        allowNegative: false
                    )
                    
                    // Integral Limit
                    DoubleInputBox(
                        title: "Integral Limit",
                        value: $yawIntegralLimit,
                        minValue: 0.0,
                        maxValue: 10.0,
                        stepSize: 0.1,
                        format: "%.1f",
                        allowNegative: false
                    )
                }
            }
            
            // Apply Button
            Button("Apply PI Settings") {
                // TODO: Send PI settings to server
            }
            .buttonStyle(.borderedProminent)
        }
        .padding()
        .background(Color.gray.opacity(0.1))
        .cornerRadius(10)
    }
}

#Preview {
    PitchYawPIControlView()
} 