import SwiftUI

struct AutonomousView: View {
    var body: some View {
        VStack(spacing: 20) {
            Text("Autonomous Mode")
                .font(.largeTitle)
                .fontWeight(.bold)
                .padding()
            
            Text("Autonomous features are currently disabled.")
                .font(.title2)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding()
            
            Text("This section will contain fully autonomous control with PI parameters and launch threshold settings for complete automated operation when implemented.")
                .font(.body)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal)
            
            Spacer()
            
            // Commented out complex controls for future use
            /*
            PitchYawPIControlView()
            SetLaunchThresholdsView()
            */
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color(.systemGroupedBackground))
        .padding()
    }
}

#Preview {
    AutonomousView()
} 
 