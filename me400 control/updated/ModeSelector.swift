import SwiftUI
import UIKit

struct ModeSelector: View {
    @Binding var selectedMode: String
    @ObservedObject private var parameterManager = ParameterManager.shared
    @ObservedObject private var serverManager = ServerCommunicationManager.shared
    
    init(selectedMode: Binding<String>) {
        UISegmentedControl.appearance().setTitleTextAttributes(
            [.font: UIFont.preferredFont(forTextStyle: .headline)], for: .normal)
        UISegmentedControl.appearance().setTitleTextAttributes(
            [.font: UIFont.preferredFont(forTextStyle: .headline)], for: .selected)
        _selectedMode = selectedMode
    }
    
    var body: some View {
        Picker("Mode", selection: $selectedMode) {
            Text("Manual").tag("Manual")
            Text("AutoAim").tag("AutoAim")
            Text("Autonomous").tag("Autonomous")
        }
        .pickerStyle(SegmentedPickerStyle())
        .frame(height: 40)
        .padding(.horizontal)
        .disabled(!parameterManager.serverConnected)
        .opacity(parameterManager.serverConnected ? 1.0 : 0.5)
    }
}

#Preview {
    ModeSelector(selectedMode: .constant("Manual"))
}
 