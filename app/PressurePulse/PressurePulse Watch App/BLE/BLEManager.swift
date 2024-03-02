//
//  BLESerialConnection.swift
//  BLEControl
//
//  Created by Dan Shepherd on 18/07/2017.
//  Copyright © 2017 Dan Shepherd. All rights reserved.
//

import Foundation
import CoreBluetooth

class BluetoothManager: NSObject, CBCentralManagerDelegate {
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
        print("stop scanning")
        centralManager.stopScan()
    }
    
    func connectToPeripheral(_ peripheral: CBPeripheral) {
        print("connecting to peripheral")
        centralManager.connect(peripheral, options: nil)
        connectedPeripheral = peripheral // Store reference to the connected peripheral
    }

    
    func disconnectPeripheral() {
        if let peripheral = connectedPeripheral { // Use the stored reference
            print("disconnecting peripheral: " + (connectedPeripheral?.name ??  "no value"))
            centralManager.cancelPeripheralConnection(peripheral)
            connectedPeripheral = nil // Reset the stored reference after disconnecting
        }else{
            print("No Peripheral to disconnect")
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print("didDiscover")
//        print(peripheral.name ?? "  ")
        if peripheral.name == "P2PSRV1" || peripheral.name == "STM32WB"{
            print("P2PSRV1 or STM32WB")
            connectToPeripheral(peripheral)
            stopScanning()
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to peripheral")
        peripheral.discoverServices(nil)
    }
//    
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
//        print("didDiscoverServices")
//        guard let services = peripheral.services else {
//            return
//        }
//        for service in services {
//            peripheral.discoverCharacteristics(nil, for: service)
//        }
//    }
//    
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
//        print("didDiscoverCharacteristicsFor")
//        guard let characteristics = service.characteristics else {
//            return
//        }
//        for characteristic in characteristics {
//            if characteristic.uuid == targetCharacteristicUUID {
//                peripheral.readValue(for: characteristic) // Read the value of the characteristic
//                peripheral.setNotifyValue(true, for: characteristic) // Subscribe to notifications
//            }
//        }
//    }
//    
//    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
//        if let error = error {
//            print("Error updating value for characteristic: \(error.localizedDescription)")
//            return
//        }
//        
//        if let value = characteristic.value {
//            // Handle the received value from the characteristic
//            print("Received value: \(value)")
//        }
//    }
}


//////
//////  BLEManager.swift
//////  PressurePulse Watch App
//////
//////  Created by Adam Brower on 2/18/24.
//////
////
////import Foundation
////import CoreBluetooth
////
////class BLEManager: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
////    
////    private var centralManager: CBCentralManager!
////    private var connectedPeripheral: CBPeripheral!
////    private var txCharacteristic: CBCharacteristic!
////    private var rxCharacteristic: CBCharacteristic!
////    private var peripheralArray: [CBPeripheral] = []
////    private var rssiArray = [NSNumber]()
////    let UUIDdevice = CBUUID(string:"0523F722-C225-1B43-6ACA-472768DF6286")
////    
////    func centralManagerDidUpdateState(_ central: CBCentralManager) {
////        print("\n\ncentral manager")
////        switch central.state {
////            case .poweredOn:
////                print("powered on")
////            case .poweredOff:
////                print("powered off")
////            case .resetting:
////                print("resetting")
////            case .unauthorized:
////                print("unauth")
////            case .unsupported:
////                print("unsupportted")
////            case .unknown:
////                print("unknown state")
////        @unknown default:
////            fatalError()
////        }
////    }
////    
////    func viewDidLoad() {
////      // Manager
////      centralManager = CBCentralManager(delegate: self, queue: nil)
////    }
////    
////    func startScanning() -> Void {
////        print("in startScanning()")
////        print("Scanning for peripherals with service UUID: \(UUIDdevice)")
////        // Remove prior data
////        peripheralArray.removeAll()
////        rssiArray.removeAll()
////        // Start Scanning
////        centralManager?.scanForPeripherals(withServices: [UUIDdevice])
////    }
////    
////    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
////        print("Function: \(#function),Line: \(#line)")
////
////        connectedPeripheral = peripheral
////
////        if peripheralArray.contains(peripheral) {
////          print("Duplicate Found.")
////        } else {
////            peripheralArray.append(peripheral)
////            rssiArray.append(RSSI)
////        }
////        
////        print("Peripheral Discovered: \(peripheral)")
////
////    }
////    
////    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
////        centralManager?.stopScan()
////        connectedPeripheral.discoverServices([UUIDdevice])
////    }
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////    
////////    var centralManager: CBCentralManager!
////////    var connectedPeripheral: CBPeripheral?
//////    weak var delegate: BLEManagerDelegate?
//////
//////
//////    init(delegate: BLEManagerDelegate?) {
//////        super.init()
//////        self.delegate = delegate
//////        centralManager = CBCentralManager(delegate: self, queue: nil)
//////    }
//////
////////    func centralManagerDidUpdateState(_ central: CBCentralManager) {
////////        print("\n\ncentral manager")
////////        switch central.state {
////////            case .poweredOn:
////////                print("powered on")
////////                startScan()
////////            case .poweredOff:
////////                print("powered off")
////////            case .resetting:
////////                print("resetting")
////////            case .unauthorized:
////////                print("unauth")
////////            case .unsupported:
////////                print("unsupportted")
////////            case .unknown:
////////                print("unknown state")
////////        @unknown default:
////////            fatalError()
////////        }
////////    }
//////
//////    func startScan() {
////////        centralManager.scanForPeripherals(withServices: nil, options: nil)
//////        print("in startScan")
//////        
//////        let serviceUUID = CBUUID(string: "0523F722-C225-1B43-6ACA-472768DF6286")
//////            
//////        // scanning for peripherals advertising the specified service UUID
//////        self.centralManager.scanForPeripherals(withServices : [serviceUUID], options: nil)
//////        print("Scanning for peripherals with service UUID: \(serviceUUID)")
//////    }
//////
////////    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
////////        print("in didDiscover")
////////        print(peripheral.identifier)
//////////        if peripheral.identifier == UUIDdevice{
//////////            print("FOUND")
//////////        }
//////////        centralManager.connect(peripheral, options: nil)
////////        delegate?.didDiscoverPeripheral(peripheral)
////////        
////////        
//////////        print(peripheral.identifier)
////////        
////////    }
//////    
//////
//////    func connectPeripheral(_ peripheral: CBPeripheral) {
//////        print("in connectPeripheral")
//////        centralManager.connect(peripheral, options: nil)
//////    }
//////
////////    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
////////        print("in didConnect")
////////        connectedPeripheral = peripheral
////////        delegate?.didConnectPeripheral(peripheral)
////////    }
//////
//////    func disconnectPeripheral() {
//////        print("in disconnectPeripheral")
//////        if let peripheral = connectedPeripheral {
//////            centralManager.cancelPeripheralConnection(peripheral)
//////        }
//////    }
//////
//////    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
//////        connectedPeripheral = nil
//////        delegate?.didDisconnectPeripheral(peripheral)
//////    }
////}
////
//////protocol BLEManagerDelegate: AnyObject {
//////    func didDiscoverPeripheral(_ peripheral: CBPeripheral)
//////    func didConnectPeripheral(_ peripheral: CBPeripheral)
//////    func didDisconnectPeripheral(_ peripheral: CBPeripheral)
//////}
////
