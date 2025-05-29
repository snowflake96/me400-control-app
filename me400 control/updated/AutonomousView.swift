import SwiftUI

struct AutonomousView: View {
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                Text("Autonomous Mode")
                    .font(.largeTitle)
                    .fontWeight(.bold)
                    .padding()
                
                Text("Configure PID parameters for autonomous operation")
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
    AutonomousView()
} 
 