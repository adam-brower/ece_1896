import SwiftUI
import UIKit

struct ContentView: View {
//    @EnvironmentObject private var bluetoothManager: BluetoothManager
    
    // color variables
    let green = Color(red: 0.349, green: 0.596, blue: 0.1058)
    let yellow = Color(red: 1, green: 0.831, blue: 0.145)
    let red = Color(red: 1, green: 0.192, blue: 0.192)
    let gray = Color(red: 0.650, green: 0.650, blue: 0.650)
    
    // vars
    @StateObject var bluetoothManager = BluetoothManager()
    @State private var timeNow = ""
    @State private var Rx: [String] = []
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    
    func hexToDecimal(hexString: String) -> UInt32? {
        let cleanedHexString = hexString.replacingOccurrences(of: "0x", with: "")
        return UInt32(cleanedHexString, radix: 16)
    }

    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()		
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }

    var imageName: String {
        if(Rx.count > 0){
            let time = hexToDecimal(hexString: Rx[0])
            switch time {
            case 0:
                return "test"
            case 1:
                HapticFeedbackManager.shared.newHaptic()
                return "yellow"
            case 2:
                HapticFeedbackManager.shared.newHaptic()
                return "red"
            default:
                return "no_conn"
            }
        }else{
            return "no_conn"
        }
    }

    var backgroundColor: Color {
        if(Rx.count > 0){
            let time = hexToDecimal(hexString: Rx[0])  // TODO: CHANGE TEH TIME VARIABLE TO BE PRESSURE INSTEAD, THATS WHAT EVERYTHING WILL BE BASED OFF OF MAYBE?
            switch time {
            case 0:
                return green
            case 1:
                return yellow
            case 2:
                return red
            default:
                return gray
            }
        }else{
            return gray
        }
        
//        if(Rx.count > 0){
//            let time = hexToDecimal(hexString: Rx[0])
//            
//            if (time! > 1000){
//                return green
//            }else if (time! <= 1000 && time! > hexToDecimal(hexString: Rx[5])!){
//                return yellow
//            }else if (time! <= hexToDecimal(hexString: Rx[5])! && time! >= 0){
//                return red
//            }else{
//                return gray
//            }
//            
//        }else{
//            return gray
//        }
        
    }
    
    var body: some View {
        NavigationStack {
            VStack {

                HStack{
                    Image(imageName)
                        .resizable()
                        .frame(width: 40, height: 120)
                    VStack {
                        
                        if(Rx.count > 0){
                            let time = hexToDecimal(hexString: Rx[0])
                            
                            let time_str = "\n\t" + String(time ?? 0) + "\t\t\n"
                            
                            Text(time_str)
                                .background(backgroundColor)
                                .cornerRadius(12)
                        }else{ // time remaining when disconnected
                            Text("\n\t-- min\t\t\n")
                                .background(Color.gray)
                                .cornerRadius(12)
                        }
                    }
                    
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
                            .onReceive(bluetoothManager.$Rx) { value in
                                print("Received value: \(value)")
                                self.Rx = value
                            }
                    }

                } // end of TBI
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
//                        ConnectionOptions()
                        bluetoothManager.startScanning()
                        
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                } // end of TBI
                ToolbarItemGroup(placement: .bottomBar){
                    
                    if(Rx.count > 0){
                        let concstr = Rx[1] + Rx[2]
                        let num = hexToDecimal(hexString: concstr)
                        Text(String(num ?? 0))
                        
                        let flow = hexToDecimal(hexString: Rx[3])
                        Text(String(flow ?? 0))
                    }else{ // metrics when disconnected
                        Text("---- PSI")
                        Text("-- LPM")
                    }
                    Button {
                        self.Rx.removeAll()
                        bluetoothManager.disconnectPeripheral()
                    } label: {
                    }
                }// end of TBI
            } // end of .toolbar
        } // end of nav stack
    }// end of view
}

//struct ConnectionOptions: View {
////    private let bluetoothManager = BluetoothManager()
//
//    var body: some View {
//        NavigationStack {
//            VStack {
//            }
//            .toolbar {
//                ToolbarItemGroup(placement: .bottomBar){
//                    Button {
//                        HapticFeedbackManager.shared.newHaptic()
//                        bluetoothManager.startScanning()
//                        
//                    } label: {
//                        Text("Search")
////                        Image(.bluetooth)
////                            .resizable()
////                            .frame(width:20, height:20)
//                    }
//                }
//            }
//        }
//    }
//}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}


