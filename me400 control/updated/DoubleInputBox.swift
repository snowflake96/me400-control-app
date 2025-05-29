import SwiftUI

struct DoubleInputBox: View {
    let title: String
    @Binding var value: Double
    let minValue: Double
    let maxValue: Double
    let stepSize: Double
    let format: String
    let allowNegative: Bool
    let frameWidth: CGFloat
    
    @State private var textValue: String
    @State private var isUpdatingFromUser: Bool = false
    @State private var lastValue: Double
    @FocusState private var isFocused: Bool
    
    init(
        title: String,
        value: Binding<Double>,
        minValue: Double = -0.9,
        maxValue: Double = 0.9,
        stepSize: Double = 0.1,
        format: String = "%.2f",
        allowNegative: Bool = true,
        frameWidth: CGFloat = 70
    ) {
        self.title = title
        self._value = value
        self.minValue = minValue
        self.maxValue = maxValue
        self.stepSize = stepSize
        self.format = format
        self.allowNegative = allowNegative
        self.frameWidth = frameWidth
        self._textValue = State(initialValue: String(format: format, value.wrappedValue))
        self._lastValue = State(initialValue: value.wrappedValue)
    }
    
    var body: some View {
        HStack{
            VStack(spacing: 0) {
                Text(title)
                    .font(.subheadline)
                
                HStack {
                    TextField(title, text: $textValue)
                        .textFieldStyle(RoundedBorderTextFieldStyle())
                        .frame(width: frameWidth)
                        .keyboardType(.decimalPad)
                        .focused($isFocused)
                        .onChange(of: textValue) { _, newValue in
                            // Handle empty string
                            if newValue.isEmpty {
                                return
                            }
                            
                            // Handle minus sign
                            if allowNegative && newValue == "-" {
                                return
                            }
                            
                            // Handle decimal point
                            if newValue == "." {
                                return
                            }
                            
                            // Handle partial decimal number
                            if newValue.hasSuffix(".") {
                                return
                            }
                            
                            // Validate input is a valid number
                            if let inputValue = Double(newValue) {
                                // Round to match the format's precision
                                let roundedValue = round(inputValue * pow(10, Double(format.count - 2))) / pow(10, Double(format.count - 2))
                                
                                // Only update if the value has actually changed
                                if abs(roundedValue - lastValue) > 0.000001 {
                                    isUpdatingFromUser = true
                                    let boundedValue = min(max(roundedValue, minValue), maxValue)
                                    value = boundedValue
                                    lastValue = boundedValue
                                    // Only update text if the value actually changed
                                    if abs(roundedValue - boundedValue) > 0.000001 {
                                        textValue = String(format: format, boundedValue)
                                    }
                                    isUpdatingFromUser = false
                                }
                            } else {
                                // If invalid, revert to previous valid value
                                textValue = String(format: format, value)
                            }
                        }
                        .onChange(of: value) { _, newValue in
                            if !isUpdatingFromUser {
                                // Round to match the format's precision
                                let roundedValue = round(newValue * pow(10, Double(format.count - 2))) / pow(10, Double(format.count - 2))
                                
                                // Only update if the value has actually changed
                                if abs(roundedValue - lastValue) > 0.000001 {
                                    textValue = String(format: format, roundedValue)
                                    lastValue = roundedValue
                                }
                            }
                        }
                        .onSubmit {
                            // Format the value when done editing
                            textValue = String(format: format, value)
                            isFocused = false
                        }
                    
                   
                }
                
            }
            
            VStack(spacing: 10) {
                Button(action: { stepValue(positive: true) }) {
                    Image(systemName: "chevron.up")
                        .frame(width: 25, height: 25)
                }
                Button(action: { stepValue(positive: false) }) {
                    Image(systemName: "chevron.down")
                        .frame(width: 25, height: 25)
                }
            }
        }
    }
    
    private func stepValue(positive: Bool) {
        isUpdatingFromUser = true
        let newValue = value + (positive ? stepSize : -stepSize)
        // Round to match the format's precision
        let roundedValue = round(newValue * pow(10, Double(format.count - 2))) / pow(10, Double(format.count - 2))
        value = min(max(roundedValue, minValue), maxValue)
        lastValue = value
        textValue = String(format: format, value)
        isUpdatingFromUser = false
    }
}

// example
struct DoubleInputBoxContainer: View {
    @FocusState private var isFocused: Bool
    
    var body: some View {
        VStack {
            DoubleInputBox(
                title: "X Offset",
                value: .constant(0.5)
            )
            
            DoubleInputBox(
                title: "Step Size",
                value: .constant(0.1),
                minValue: 0.01,
                maxValue: 0.5,
                stepSize: 0.01,
                format: "%.2f",
                allowNegative: false
            )
        }
        .padding()
        .toolbar {
            ToolbarItemGroup(placement: .keyboard) {
                Spacer()
                Button("Done") {
                    isFocused = false
                }
            }
        }
    }
}

#Preview {
    DoubleInputBoxContainer()
} 
