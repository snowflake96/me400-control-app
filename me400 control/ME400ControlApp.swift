import SwiftUI

@main
struct ME400ControlApp: App {
    @StateObject private var tcpManager = TCPClientManager()
    @StateObject private var settingsManager = SettingsManager()
    
    var body: some Scene {
        WindowGroup {
            #if os(macOS)
            MacAndiPadView()
            #else
            if UIDevice.current.userInterfaceIdiom == .pad {
                MacAndiPadView()
            } else {
                ControlView(tcpManager: tcpManager, settingsManager: settingsManager)
            }
            #endif
        }
        #if os(macOS)
        .windowStyle(.hiddenTitleBar)
        .windowToolbarStyle(.unified)
        #endif
    }
} 