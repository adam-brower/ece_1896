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
    @State private var Rx: [String] = []  // [time] [pressure 1] [pressure 2] [flow] [thresh 1] [thresh 2]
    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()
    @State private var half_fill_state = false
    @State private var empty_state = false;
    
    func hexToDecimal(hexString: String) -> UInt32? {
        let cleanedHexString = hexString.replacingOccurrences(of: "0x", with: "")
        return UInt32(cleanedHexString, radix: 16)
    }

    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }

    var ox_tank_image: String {
        if(Rx.count > 0){
            let psi = hexToDecimal(hexString: Rx[1] + Rx[2])

            if (psi! > ((2000 - (hexToDecimal(hexString: Rx[4] + Rx[5]) ?? 500)) / 2)){ // if psi is between 2000psi and halfway from 2000 and lower threshold, display green
                half_fill_state = false;
                empty_state = false;
                return "green_tank"
            }else if (psi! <= ((2000 - (hexToDecimal(hexString: Rx[4] + Rx[5]) ?? 500)) / 2) && psi! > hexToDecimal(hexString: Rx[4] + Rx[5])!){
                empty_state = false;
                
                if(!half_fill_state){
                    HapticFeedbackManager.shared.newHaptic()
                    half_fill_state = true;
                }
                
                return "yellow_tank"
            }else if (psi! <= hexToDecimal(hexString: Rx[4] + Rx[5])! && psi! >= 0){
                half_fill_state = false;
                
                if(!empty_state){
                    HapticFeedbackManager.shared.newHaptic()
                    empty_state = true;
                }
                
                return "red_tank"
            }else{
                return "no_conn"
            }
        }else{
            return "no_conn"
        }
    }

    var text_background_color: Color {
        if(Rx.count > 0){
            let time = hexToDecimal(hexString: Rx[0])

            if (time! > 10){
                return green
            }else if (time! <= 10 && time! > hexToDecimal(hexString: Rx[5])!){
                return yellow
            }else if (time! <= hexToDecimal(hexString: Rx[5])! && time! >= 0){
                return red
            }else{
                return gray
            }
        }else{
            return gray
        }
        
    }
    
    var body: some View {
        NavigationStack {

            HStack{
                Image(ox_tank_image)
                    .resizable()
                    .frame(width: 40, height: 120)
                    
                if(Rx.count > 0){
                    let time_str = "\n\t" + String(hexToDecimal(hexString: Rx[0]) ?? 0) + " min\t\t\n"
                    
                    Text(time_str)
                        .background(text_background_color)
                        .cornerRadius(12)
                }else{ // time remaining palcehoder when disconnected
                    Text("\n\t-- min\t\t\n")
                        .background(Color.gray)
                        .cornerRadius(12)
                }
            } // end of HStack
            .toolbar {
                
                ToolbarItem(placement: .topBarLeading) {
                    Text(timeNow)
                        .onReceive(timer) { _ in
                            self.timeNow = dateFormatter.string(from: Date())
                        }
                        .font(.system(size: 13))
                        .onReceive(bluetoothManager.$Rx) { value in
                            print("Received value: \(value)")
                            self.Rx = value
                        }
                } // end of TBI
                
                ToolbarItem(placement: .topBarTrailing) {
                    NavigationLink(destination: ConnectionOptions().environmentObject(bluetoothManager)) {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                } // end of TBI
                
                ToolbarItemGroup(placement: .bottomBar){
                    if(Rx.count > 0){
//                        let concstr = Rx[1] + Rx[2]
//                        let num = hexToDecimal(hexString: concstr)
                        let psi = String(hexToDecimal(hexString: Rx[1] + Rx[2]) ?? 0) + " PSI"
                        Text(psi)
                        
//                        let flow = hexToDecimal(hexString: Rx[3])
                        let flow = String(hexToDecimal(hexString: Rx[3]) ?? 0) + " LPM"
                        Text(flow)
                    }else{ // meetrix placeholders when disconnected
                        Text("---- PSI")
                        Text("-- LPM")
                    }
                } // end of TBI
                
            } // end of .toolbar
        } // end of nav stack
    }// end of view
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
