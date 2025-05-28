import SwiftUI

struct StringInputBox: View {
    let label: String
    let placeholder: String
    @Binding var value: String
    var width: CGFloat = 60
    var isDisabled: Bool = false
    var keyboardType: UIKeyboardType = .default
    var onCommit: (() -> Void)? = nil
    @FocusState private var isFocused: Bool
    
    var body: some View {
        HStack {
            Text(label)
                .frame(width: width, alignment: .leading)
            TextField(placeholder, text: $value)
                .textFieldStyle(.roundedBorder)
                .keyboardType(keyboardType)
                .autocapitalization(.none)
                .disableAutocorrection(true)
                .disabled(isDisabled)
                .focused($isFocused)
                .toolbar {
                    if isFocused {
                        ToolbarItemGroup(placement: .keyboard) {
                            Spacer()
                            Button("Done") {
                                isFocused = false
                                onCommit?()
                            }
                        }
                    }
                }
        }
    }
}

#Preview {
    VStack {
        StringInputBox(
            label: "Host:",
            placeholder: "localhost",
            value: .constant("127.0.0.1")
        )
        
        StringInputBox(
            label: "Port:",
            placeholder: "12345",
            value: .constant("8080"),
            keyboardType: .numberPad
        )
        
        StringInputBox(
            label: "Disabled:",
            placeholder: "Cannot edit",
            value: .constant("Locked"),
            isDisabled: true
        )
    }
    .padding()
} 