//
//  Item.swift
//  me400 control
//
//  Created by Jiwoo Lee on 5/9/25.
//

import Foundation
import SwiftData

@Model
final class Item {
    var timestamp: Date
    
    init(timestamp: Date) {
        self.timestamp = timestamp
    }
}
