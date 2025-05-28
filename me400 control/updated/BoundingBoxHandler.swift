import Foundation
import SwiftUI
import Combine

class BoundingBoxHandler: ObservableObject {
    static let shared = BoundingBoxHandler()
    
    @Published var hasDetection: Bool = false
    @Published var centerX: Double = 0.0
    @Published var centerY: Double = 0.0
    @Published var width: Double = 0.0
    @Published var height: Double = 0.0
    
    // Published properties for bounding box coordinates
    @Published var x1: Double = 0
    @Published var y1: Double = 0
    @Published var x2: Double = 0
    @Published var y2: Double = 0
    
    // Published properties for filtered bbox center coordinates
    @Published var filteredBboxCenterX: Double = 0
    @Published var filteredBboxCenterY: Double = 0
    
    // Counter for updates
    @Published private(set) var updateCounter: Int = 0
    
    private init() {
        reset()
    }
    
    func reset() {
        x1 = 0
        y1 = 0
        x2 = 0
        y2 = 0
        centerX = 0
        centerY = 0
        width = 0
        height = 0
        hasDetection = false
        filteredBboxCenterX = 0
        filteredBboxCenterY = 0
    }
    
    func processBoundingBoxData(_ data: [Double]) {
        guard data.count >= 4 else { return }
        
        // Check if all values are NaN
        let allNaN = data.allSatisfy { $0.isNaN }
        
        if allNaN {
            reset()
            return
        }
        
        // Extract and calculate values
        let newX1 = data[0]
        let newY1 = data[1]
        let newX2 = data[2]
        let newY2 = data[3]
        let newCenterX = (newX1 + newX2) / 2
        let newCenterY = (newY1 + newY2) / 2
        let newWidth = newX2 - newX1
        let newHeight = newY2 - newY1
        
        // Update properties directly
        x1 = newX1
        y1 = newY1
        x2 = newX2
        y2 = newY2
        centerX = newCenterX
        centerY = newCenterY
        width = newWidth
        height = newHeight
        hasDetection = true
        updateCounter += 1
    }
    
    func processFilteredBboxData(_ data: [Double]) {
        guard data.count >= 2 else { return }
        
        // Check for NaN values
        let hasNaN = data.contains { $0.isNaN }
        if hasNaN {
            return
        }
        
        // Update properties directly
        filteredBboxCenterX = data[0]
        filteredBboxCenterY = data[1]
        updateCounter += 1
    }
} 
