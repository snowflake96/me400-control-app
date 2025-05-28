import SwiftUI

struct SetLaunchThresholdsView: View {
    @State private var nValue: Int = 5
    @State private var epsilonValue: Double = 0.01
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Launch Thresholds")
                .font(.headline)
            
            VStack(alignment: .leading, spacing: 15) {
                // N Value (Integer)
                IntegerInputBox(
                    title: "N",
                    value: $nValue,
                    minValue: 1,
                    maxValue: 100,
                    stepSize: 1,
                    allowNegative: false
                )
                
                // Epsilon Value (Double)
                DoubleInputBox(
                    title: "Epsilon",
                    value: $epsilonValue,
                    minValue: 0.001,
                    maxValue: 1.0,
                    stepSize: 0.001,
                    format: "%.3f",
                    allowNegative: false
                )
            }
            
            // Apply Button
            Button("Apply Thresholds") {
                // TODO: Send threshold settings to server
            }
            .buttonStyle(.borderedProminent)
        }
        .padding()
        .background(Color.gray.opacity(0.1))
        .cornerRadius(10)
    }
}

#Preview {
    SetLaunchThresholdsView()
} 