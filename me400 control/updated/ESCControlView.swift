import SwiftUI

struct ESCControlView: View {
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
                isLaunchPressed.toggle()
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
}

#Preview {
    ESCControlView()
} 