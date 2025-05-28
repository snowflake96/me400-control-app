import SwiftUI

struct IntegerInputBox: View {
    let title: String
    @Binding var value: Int
    let minValue: Int
    let maxValue: Int
    let stepSize: Int
    let allowNegative: Bool
    
    var body: some View {
        HStack {
            Text(title)
                .frame(width: 100, alignment: .leading)
            
            HStack(spacing: 0) {
                Button("-") {
                    if value > minValue {
                        value -= stepSize
                    }
                }
                .buttonStyle(.bordered)
                .disabled(value <= minValue)
                
                TextField("", value: $value, format: .number)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .frame(width: 60)
                    .multilineTextAlignment(.center)
                
                Button("+") {
                    if value < maxValue {
                        value += stepSize
                    }
                }
                .buttonStyle(.bordered)
                .disabled(value >= maxValue)
            }
        }
    }
}

#Preview {
    IntegerInputBox(
        title: "Test",
        value: .constant(5),
        minValue: 0,
        maxValue: 10,
        stepSize: 1,
        allowNegative: false
    )
} 