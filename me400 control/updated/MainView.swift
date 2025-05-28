import SwiftUI

struct MainView: View {
    @StateObject private var serverManager = ServerCommunicationManager.shared
    @StateObject private var settingsManager = SettingsManager()
    
    var body: some View {
        FinalAppView()
    }
}

#Preview {
    MainView()
} 
