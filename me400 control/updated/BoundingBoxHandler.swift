import Foundation
import SwiftUI
import Combine

class BoundingBoxHandler: ObservableObject {
    static let shared = BoundingBoxHandler()
    
    // Serial queue for processing bounding box data
    private let processingQueue = DispatchQueue(label: "com.me400.bboxProcessing")
    
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
        // print("BoundingBoxHandler singleton instance created: \(Unmanaged.passUnretained(self).toOpaque())")
        // Initialize with default values
        reset()
    }
    
    func reset() {
        // Reset local values
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.centerX = 0
        self.centerY = 0
        self.width = 0
        self.height = 0
        self.hasDetection = false
        self.filteredBboxCenterX = 0
        self.filteredBboxCenterY = 0
    }
    
    func processBoundingBoxData(_ data: [Double]) {
        processingQueue.async { [weak self] in
            guard let self = self else { return }
            
            guard data.count >= 4 else { return }
            
            // Check if all values are NaN
            let allNaN = data.allSatisfy { $0.isNaN }
            
            if allNaN {
                // No detection - reset values
                // print("all NaN")
                DispatchQueue.main.async {
                    self.reset()
                }
                return
            }
            
            // Extract coordinates from data packet
            let x1 = data[0]  // Top-left X
            let y1 = data[1]  // Top-left Y
            let x2 = data[2]  // Bottom-right X
            let y2 = data[3]  // Bottom-right Y
            
            // Update values on main thread
            DispatchQueue.main.async {
                // Update local coordinates
                self.x1 = x1
                self.y1 = y1
                self.x2 = x2
                self.y2 = y2
                
                // Calculate center and dimensions
                self.centerX = (x1 + x2) / 2
                self.centerY = (y1 + y2) / 2
                self.width = x2 - x1
                self.height = y2 - y1
                self.hasDetection = true
                
                // Increment update counter
                self.updateCounter += 1
            }
        }
    }
    
    // Process filtered bbox center coordinates
    func processFilteredBboxData(_ data: [Double]) {
        processingQueue.async { [weak self] in
            guard let self = self else { return }
            guard data.count >= 2 else { return }
            
            // Update filtered coordinates on main thread
            DispatchQueue.main.async {
                self.filteredBboxCenterX = data[0]
                self.filteredBboxCenterY = data[1]
                self.updateCounter += 1
            }
        }
    }
} 
