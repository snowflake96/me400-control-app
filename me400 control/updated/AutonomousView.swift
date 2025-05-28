import SwiftUI

struct AutonomousView: View {
    var body: some View {
        VStack(spacing: 20) {
            Text("Autonomous Mode")
                .font(.largeTitle)
                .padding()
            
            PitchYawPIControlView()
            SetLaunchThresholdsView()
        }
        .padding()
    }
}

#Preview {
    AutonomousView()
} 
 