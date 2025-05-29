import SwiftUI

struct UInt8InputBox: View {
    let title: String
    @Binding var value: UInt8
    let minValue: UInt8
    let maxValue: UInt8
    let stepSize: UInt8
    
    @State private var textValue: String = ""
    @FocusState private var isFocused: Bool
    
    init(
        title: String,
        value: Binding<UInt8>,
        minValue: UInt8 = 0,
        maxValue: UInt8 = 255,
        stepSize: UInt8 = 1
    ) {
        self.title = title
        self._value = value
        self.minValue = minValue
        self.maxValue = maxValue
        self.stepSize = stepSize
    }
    
    var body: some View {
        HStack {
            VStack(spacing: 0) {
                Text(title)
                    .font(.subheadline)
                
                TextField(title, text: $textValue)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .frame(width: 60)
                    .keyboardType(.numberPad)
                    .focused($isFocused)
                    .onAppear {
                        textValue = String(value)
                    }
                    .onChange(of: value) { _, _ in
                        if !isFocused {
                            textValue = String(value)
                        }
                    }
                    .onChange(of: textValue) { _, newValue in
                        if let intValue = UInt8(newValue) {
                            let boundedValue = min(max(intValue, minValue), maxValue)
                            if boundedValue != value {
                                value = boundedValue
                            }
                        }
                    }
                    .onSubmit {
                        if let intValue = UInt8(textValue) {
                            value = min(max(intValue, minValue), maxValue)
                        }
                        textValue = String(value)
                        isFocused = false
                    }
            }
            
            VStack(spacing: 5) {
                Button(action: {
                    if value < maxValue {
                        let newValue = min(value + stepSize, maxValue)
                        value = newValue
                        textValue = String(value)
                    }
                }) {
                    Image(systemName: "chevron.up")
                        .font(.caption)
                        .frame(width: 20, height: 20)
                }
                .buttonStyle(.bordered)
                .disabled(value >= maxValue)
                
                Button(action: {
                    if value > minValue {
                        let newValue = max(value - stepSize, minValue)
                        value = newValue
                        textValue = String(value)
                    }
                }) {
                    Image(systemName: "chevron.down")
                        .font(.caption)
                        .frame(width: 20, height: 20)
                }
                .buttonStyle(.bordered)
                .disabled(value <= minValue)
            }
        }
    }
}

struct IntegerInputBox: View {
    let title: String
    @Binding var value: Int
    let minValue: Int
    let maxValue: Int
    let stepSize: Int
    let allowNegative: Bool
    
    @State private var textValue: String = ""
    @FocusState private var isFocused: Bool
    
    init(
        title: String,
        value: Binding<Int>,
        minValue: Int = 0,
        maxValue: Int = 100,
        stepSize: Int = 1,
        allowNegative: Bool = false
    ) {
        self.title = title
        self._value = value
        self.minValue = minValue
        self.maxValue = maxValue
        self.stepSize = stepSize
        self.allowNegative = allowNegative
    }
    
    var body: some View {
        HStack {
            VStack(spacing: 0) {
                Text(title)
                    .font(.subheadline)
                
                TextField(title, text: $textValue)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .frame(width: 60)
                    .keyboardType(.numberPad)
                    .focused($isFocused)
                    .onAppear {
                        textValue = String(value)
                    }
                    .onChange(of: value) { _, _ in
                        if !isFocused {
                            textValue = String(value)
                        }
                    }
                    .onChange(of: textValue) { _, newValue in
                        if let intValue = Int(newValue) {
                            let boundedValue = min(max(intValue, minValue), maxValue)
                            if boundedValue != value {
                                value = boundedValue
                            }
                        }
                    }
                    .onSubmit {
                        if let intValue = Int(textValue) {
                            value = min(max(intValue, minValue), maxValue)
                        }
                        textValue = String(value)
                        isFocused = false
                    }
            }
            
            VStack(spacing: 5) {
                Button(action: {
                    let newValue = value + stepSize
                    if newValue <= maxValue {
                        value = newValue
                        textValue = String(value)
                    }
                }) {
                    Image(systemName: "chevron.up")
                        .font(.caption)
                        .frame(width: 20, height: 20)
                }
                .buttonStyle(.bordered)
                .disabled(value >= maxValue)
                
                Button(action: {
                    let newValue = value - stepSize
                    if newValue >= minValue {
                        value = newValue
                        textValue = String(value)
                    }
                }) {
                    Image(systemName: "chevron.down")
                        .font(.caption)
                        .frame(width: 20, height: 20)
                }
                .buttonStyle(.bordered)
                .disabled(value <= minValue)
            }
        }
    }
}

#Preview {
    VStack {
        UInt8InputBox(
            title: "UInt8 Test",
            value: .constant(5),
            minValue: 0,
            maxValue: 255,
            stepSize: 1
        )
        
        IntegerInputBox(
            title: "Int Test",
            value: .constant(5),
            minValue: 0,
            maxValue: 10,
            stepSize: 1,
            allowNegative: false
        )
    }
} 