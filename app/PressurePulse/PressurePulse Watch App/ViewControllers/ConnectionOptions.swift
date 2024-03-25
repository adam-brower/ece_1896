//
//  ConnectionOptions.swift
//  PressurePulse Watch App
//
//  Created by Adam Brower on 3/4/24.
//

import SwiftUI

struct ConnectionOptions: View {
    @EnvironmentObject var bluetoothManager: BluetoothManager
    @State private var selectedPeripheral: Peripheral? = nil
    
    var body: some View {
        VStack {
            List {
                ForEach(bluetoothManager.discoveredPeripherals.indices, id: \.self) { index in
                    let peripheral = bluetoothManager.discoveredPeripherals[index]
                    Button(action: {
                        if peripheral.isConnected {
                            bluetoothManager.disconnectPeripheral(peripheral)
                            selectedPeripheral = nil // Deselect the peripheral
                        } else {
                            bluetoothManager.connectToPeripheral(peripheral)
                            selectedPeripheral = peripheral // Select the peripheral
                        }
                    }) {
                        Text("\(peripheral.peripheral.name ?? "Unknown") - \(peripheral.isConnected)")
                            .foregroundColor(selectedPeripheral == peripheral && peripheral.isConnected ? .blue : .primary)
                    }
                }
            }
            .toolbar {
                ToolbarItem(placement: .bottomBar) {
                    Button(action: {
                        bluetoothManager.startScanning()
                    }) {
                        Text("Search")
                    }
                }
            }
            .onDisappear {
                bluetoothManager.stopScanning()
            }
            
            // Disconnect button if a peripheral is selected
            if let selectedPeripheral = selectedPeripheral {
                Button(action: {
                    bluetoothManager.disconnectPeripheral(selectedPeripheral)
                    self.selectedPeripheral = nil
                }) {
                    Text("Disconnect \(selectedPeripheral.peripheral.name ?? "Peripheral")")
                        .foregroundColor(.red)
                }
            }
            
        }
    }
}

