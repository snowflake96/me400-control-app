import SwiftUI
import UIKit

struct ModeSelector: View {
    @StateObject private var parameterManager = ParameterManager.shared
    @ObservedObject private var serverManager = ServerCommunicationManager.shared
    
    init() {
        UISegmentedControl.appearance().setTitleTextAttributes(
            [.font: UIFont.preferredFont(forTextStyle: .headline)], for: .normal)
        UISegmentedControl.appearance().setTitleTextAttributes(
            [.font: UIFont.preferredFont(forTextStyle: .headline)], for: .selected)
    }
    
    var body: some View {
        Picker("Mode", selection: $parameterManager.selectedMode) {
            Text("Manual").tag("Manual")
            Text("AutoAim").tag("AutoAim")
            Text("Autonomous").tag("Autonomous")
        }
        .pickerStyle(SegmentedPickerStyle())
        .frame(height: 40)
        .padding(.horizontal)
        .disabled(!parameterManager.serverConnected)
        .opacity(parameterManager.serverConnected ? 1.0 : 0.5)
        .onChange(of: parameterManager.selectedMode) { _, newMode in
            if parameterManager.serverConnected {
                switch newMode {
                case "Manual":
                    _ = serverManager.send(DataPacket.setManual())
                case "AutoAim":
                    _ = serverManager.send(DataPacket.setAutoAim())
                case "Autonomous":
                    _ = serverManager.send(DataPacket.setAutonomous())
                default:
                    break
                }
            }
        }
    }
}

#Preview {
    ModeSelector()
}
 