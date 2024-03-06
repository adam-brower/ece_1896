//
//  BLESerialConnection.swift
//  BLEControl
//
//  Created by Dan Shepherd on 18/07/2017.
//  Copyright © 2017 Dan Shepherd. All rights reserved.
//

//import Foundation
import CoreBluetooth

class BluetoothManager: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate, ObservableObject {
    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var connectedPeripheral: CBPeripheral?
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            print("powered on")
        } else {
            print("not powered on")
        }
    }
    
    func startScanning() {
        if connectedPeripheral == nil{
            centralManager.scanForPeripherals(withServices: nil, options: nil)
            print("start scanning")
        }else{
            print("Device already Connected ")
        }
    }
    
    func stopScanning() {
        print("stopping scan")
        centralManager.stopScan()
    }
    
    func connectToPeripheral(_ peripheral: CBPeripheral) {
        print("connecting to peripheral")
        centralManager.connect(peripheral, options: nil)
        connectedPeripheral = peripheral // store reference to the connected peripheral
    }

    
    func disconnectPeripheral() {
        if let peripheral = connectedPeripheral { // Use the stored reference
            print("disconnecting peripheral: " + (connectedPeripheral?.name ??  "no value"))
            centralManager.cancelPeripheralConnection(peripheral)
            connectedPeripheral = nil // Reset the stored reference after disconnecting
        }else{
            print("No Peripheral to disconnect")
            stopScanning()
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print("didDiscover")
//        print(peripheral.name ?? "  ")
        
        // identifier: 14543F24-25B9-2B97-3E63-BE1E515E3E41
        
        if peripheral.name == "Adam"{
            print("Found Adam")
            stopScanning()
        }
        if peripheral.name == "P2PSRV1" || peripheral.name == "STM32WB"{
            print("P2PSRV1 or STM32WB")
            print(peripheral.name ?? "default")
            connectToPeripheral(peripheral)
            stopScanning()
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to peripheral")
        peripheral.delegate = self
        peripheral.discoverServices(nil)
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        print("didDiscoverServices")
        guard let services = peripheral.services else {
            return
        }
        for service in services {
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        print("didDiscoverCharacteristicsFor")
        guard let characteristics = service.characteristics else {
            return
        }
        for characteristic in characteristics {
            peripheral.setNotifyValue(true, for: characteristic)
            peripheral.readValue(for: characteristic)
            print("Characteristic: \(characteristic)")
            // Check if this is the characteristic you want to read
//            if characteristic.uuid == CBUUID(string: "0000FE41-8E22-4541-9D4C-21EDAE82ED19") {
//                // Read the value of this characteristic
//                peripheral.setNotifyValue(true, for: characteristic)
//                peripheral.readValue(for: characteristic)
//            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if let data = characteristic.value {
            print(data)
//            let dataString = String(data: data, encoding: .utf8)
//            print("Received data for characteristic \(characteristic.uuid): \(dataString ?? "Error decoding data")")
        }
    }
}
