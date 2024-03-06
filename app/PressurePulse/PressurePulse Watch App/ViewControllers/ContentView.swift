import SwiftUI
import UIKit
//import CoreBluetooth

struct ContentView: View {
//    @EnvironmentObject private var bluetoothManager: BluetoothManager
    private var bluetoothManager = BluetoothManager()
    
    @State private var isFull = 0 // State variable to track whether the icon should be full or empty
    @State private var timeNow = ""
    
//    var centralManager = CBCentralManager()
//    var peripheral: CBPeripheral?
    
    

    let timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect()

    var dateFormatter: DateFormatter {
        let fmtr = DateFormatter()		
        fmtr.dateFormat = "LLL d, h:mm:ss"
        return fmtr
    }

    var imageName: String {
        switch isFull % 3 {
        case 0:
            return "test"
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
                        .frame(width: 40, height: 120)

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
//                        ConnectionOptions()
                        self.isFull += 1
                        HapticFeedbackManager.shared.newHaptic()
                        bluetoothManager.startScanning()
                        
                    } label: {
                        Image(.bluetooth)
                            .resizable()
                            .frame(width:20, height:20)
                    }
                }
                ToolbarItemGroup(placement: .bottomBar){
                    Text("2000 PSI")
                    Text("1200 LPM")
                    Button {
                        HapticFeedbackManager.shared.newHaptic()
                        bluetoothManager.disconnectPeripheral()
                    } label: {
                    }
                }
            }
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
//            .environmentObject(BluetoothManager())
    }
}
