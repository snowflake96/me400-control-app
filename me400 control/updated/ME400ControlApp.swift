import SwiftUI

@main
struct ME400ControlApp: App {
    @StateObject private var serverManager = ServerCommunicationManager()
    @StateObject private var settingsManager = SettingsManager()
    
    init() {
        // Initialize server connection parameters
        ServerCommunicationManager.initialize()
    }
    
    var body: some Scene {
        WindowGroup {
            #if os(macOS)
            FinalAppView()
            #else
            if UIDevice.current.userInterfaceIdiom == .pad {
                FinalAppView()
            }
            #endif
        }
        #if os(macOS)
        .windowStyle(.hiddenTitleBar)
        .windowToolbarStyle(.unified)
        #endif
    }
} 
