import SwiftUI
import UIKit

struct ContentView: View {
    
    var body: some View {
        TabView {
            page1()
            page2()
            page3()
            page4()
            page5()
            page6()
            page7()
            page8()
            page9()
            page10()
        }
        .tabViewStyle(.page(indexDisplayMode: .always))
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}

struct page1: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    private let bleManager = BLEManager()
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            HStack {
                Image(imageName) // Switch between full and empty icons
                    .resizable()
                    .frame(width:70, height: 70)
                
                VStack{
                    Text("20:00 Min\n")
                    Text("2000 PSI")
                    Text("1200 LPM")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Text(timeNow)
                        .onReceive(timer) { _ in
                            self.timeNow = dateFormatter.string(from: Date())
                        }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                        self.bleManager.startScan()
                        print("test")
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
            }
        }
    }
}


struct page2: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            VStack {
                
                HStack{
                    Image(imageName) // Switch between full and empty icons
                        .resizable()
                        .frame(width:70, height: 70)
                    
                    Text("20min")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    HStack {
                        Image("battery")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(timeNow)
                            .onReceive(timer) { _ in
                                self.timeNow = dateFormatter.string(from: Date())
                            }
                            .font(.system(size: 13))

                    }
                    
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                    Text("1200 LPM")
                }
            }
        }
    }
}


struct page3: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var backgroundColor: Color {
        switch isFull % 3 {
        case 0:
            return Color.green
        case 1:
            return Color.yellow
        case 2:
            return Color.red
        default:
            return Color.red // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            VStack {
                
                HStack{
                    Image(imageName) // Switch between full and empty icons
                        .resizable()
                        .frame(width:70, height: 70)
                    
                    Text("\n\t20min\t\t\n")
                        .background(backgroundColor)
                        .cornerRadius(12)
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    HStack {
                        Image("battery")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(timeNow)
                            .onReceive(timer) { _ in
                                self.timeNow = dateFormatter.string(from: Date())
                            }
                            .font(.system(size: 13))

                    }
                    
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                    Text("1200 LPM")
                }
            }
        }
    }
}


struct page4: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var backgroundColor: Color {
        switch isFull % 3 {
        case 0:
            return Color.green
        case 1:
            return Color.yellow
        case 2:
            return Color.red
        default:
            return Color.red // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            Text("\n\t20min\t\t\n")
                .background(backgroundColor)
                .cornerRadius(12)
                .font(.system(size: 30))
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    HStack {
                        Image("battery")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(timeNow)
                            .onReceive(timer) { _ in
                                self.timeNow = dateFormatter.string(from: Date())
                            }
                            .font(.system(size: 13))

                    }
                    
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                    Text("1200 LPM")
                }
            }
        }
    }
}

struct page5: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            HStack {
                Image(imageName) // Switch between full and empty icons
                    .resizable()
                    .frame(width:70, height: 70)
                
                VStack{
                    Text("20:00 Min\n")
                    Text("2000 PSI\n")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Text(timeNow)
                        .onReceive(timer) { _ in
                            self.timeNow = dateFormatter.string(from: Date())
                        }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
            }
        }
    }
}

struct page6: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            HStack {
                Image(imageName) // Switch between full and empty icons
                    .resizable()
                    .frame(width:70, height: 70)
                
                VStack{
                    Text("20:00 Min")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Text(timeNow)
                        .onReceive(timer) { _ in
                            self.timeNow = dateFormatter.string(from: Date())
                        }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                }
            }
        }
    }
}

struct page7: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            HStack {
                Image(imageName) // Switch between full and empty icons
                    .resizable()
                    .frame(width:70, height: 70)
                
                VStack{
                    Text("20:00 Min\n")
                    Text("1200 LPM\n")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Text(timeNow)
                        .onReceive(timer) { _ in
                            self.timeNow = dateFormatter.string(from: Date())
                        }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
            }
        }
    }
}

struct page8: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "tank.full"
        case 1:
            return "tank.half"
        case 2:
            return "tank.empty"
        default:
            return "tank.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            HStack {
                Image(imageName) // Switch between full and empty icons
                    .resizable()
                    .frame(width:70, height: 70)
                
                VStack{
                    Text("20:00 Min")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Text(timeNow)
                        .onReceive(timer) { _ in
                            self.timeNow = dateFormatter.string(from: Date())
                        }
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("1200 LPM")
                }
            }
        }
    }
}

struct page9: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "base.full"
        case 1:
            return "base.half"
        case 2:
            return "base.empty"
        default:
            return "base.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var backgroundColor: Color {
        switch isFull % 3 {
        case 0:
            return Color.green
        case 1:
            return Color.yellow
        case 2:
            return Color.red
        default:
            return Color.red // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            VStack {
                
                HStack{
                    Image(imageName) // Switch between full and empty icons
                        .resizable()
                        .frame(width:70, height: 70)
                    
                    Text("\n\t20min\t\t\n")
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    HStack {
                        Image("battery")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(timeNow)
                            .onReceive(timer) { _ in
                                self.timeNow = dateFormatter.string(from: Date())
                            }
                            .font(.system(size: 13))

                    }
                    
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                    Text("1200 LPM")
                }
            }
        }
    }
}


struct page10: View {
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }
    
    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "base.full"
        case 1:
            return "base.half"
        case 2:
            return "base.empty"
        default:
            return "base.empty" // Default case if isFull is not divisible by 3
        }
    }
    
    var backgroundColor: Color {
        switch isFull % 3 {
        case 0:
            return Color.green
        case 1:
            return Color.yellow
        case 2:
            return Color.red
        default:
            return Color.red // Default case if isFull is not divisible by 3
        }
    }
    
    var body: some View {
        NavigationStack {
            VStack {
                
                HStack{
                    Image(imageName) // Switch between full and empty icons
                        .resizable()
                        .frame(width:70, height: 70)
                    
                    Text("\n\t20min\t\t\n")
                        .background(backgroundColor)
                        .cornerRadius(12)
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    HStack {
                        Image("battery")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(timeNow)
                            .onReceive(timer) { _ in
                                self.timeNow = dateFormatter.string(from: Date())
                            }
                            .font(.system(size: 13))

                    }
                    
                }
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                    Text("1200 LPM")
                }
            }
        }
    }
}
