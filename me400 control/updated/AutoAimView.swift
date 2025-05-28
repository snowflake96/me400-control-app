import SwiftUI

struct AutoAimView: View {
    var body: some View {
        VStack(spacing: 20) {
            Text("AutoAim Mode")
                .font(.largeTitle)
                .padding()
            
            PitchYawPIControlView()
            ESCControlView()
        }
        .padding()
    }
}

#Preview {
    AutoAimView()
} 