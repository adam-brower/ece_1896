import WatchKit

enum HapticFeedbackType {
    case click
    case notification(type: WKHapticType?)
}

class HapticFeedbackManager {
    static let shared = HapticFeedbackManager()
    
    private init() {}
    
    func trigger(_ type: HapticFeedbackType) {
        switch type {
        case .click:
            WKInterfaceDevice.current().play(.click)
        case .notification(let hapticType):
            if let unwrappedHapticType = hapticType {
                WKInterfaceDevice.current().play(unwrappedHapticType)
            }
        }
    }
    
    func triggerLongHardHaptic() {
        let hapticTypes: [WKHapticType] = [.success, .success, .success, .success, .success, .success]
        for hapticType in hapticTypes {
            WKInterfaceDevice.current().play(hapticType)
        }
    }
    
    func newHaptic() {
        let hapticTypes: [WKHapticType] = [.failure, .success, .failure]
        for hapticType in hapticTypes {
            WKInterfaceDevice.current().play(hapticType)
        }
    }

}
