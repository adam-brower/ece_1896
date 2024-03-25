//
//  BLEManager.swift
//
//  Created by Adam
//  PressurePulse App BLE
//

import CoreBluetooth

struct Peripheral: Identifiable, Equatable {
    let id = UUID()
    let peripheral: CBPeripheral
    var isConnected: Bool
    
    static func == (lhs: Peripheral, rhs: Peripheral) -> Bool {
        return lhs.id == rhs.id
    }
}

class BluetoothManager: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate, ObservableObject {
    private var centralManager: CBCentralManager!
//    private var peripheral: CBPeripheral?
    private var connectedPeripheral: CBPeripheral?
    @Published var Rx: [String] = []
    @Published var discoveredPeripherals: [Peripheral] = []
    
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
            print("Device already Connected")
        }
    }
    
    func stopScanning() {
        print("stopping scan")
        centralManager.stopScan()
    }
    
    func connectToPeripheral(_ peripheral: Peripheral) {
        print("connecting to peripheral")
        centralManager.connect(peripheral.peripheral, options: nil)
        connectedPeripheral = peripheral.peripheral // store reference to the connected peripheral
        
        if let index = discoveredPeripherals.firstIndex(where: { $0.peripheral == connectedPeripheral }) {
            discoveredPeripherals[index].isConnected = true
        }
    }
    
    func disconnectPeripheral(_ peripheral: Peripheral) {
        if let connectedPeripheral = connectedPeripheral {
            print("disconnecting peripheral: \(connectedPeripheral.name ?? "no value")")
            centralManager.cancelPeripheralConnection(connectedPeripheral)
            // update the isConnected property of the disconnected peripheral
            if let index = discoveredPeripherals.firstIndex(where: { $0.peripheral == connectedPeripheral }) {
                discoveredPeripherals[index].isConnected = false
            }
            self.Rx.removeAll()
            self.connectedPeripheral = nil
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        
        guard let peripheralName = peripheral.name else {
            return
        }
        
        print("Discovered peripheral: \(peripheralName)")
        
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            if !self.discoveredPeripherals.contains(where: { $0.peripheral == peripheral }) {
                self.discoveredPeripherals.append(Peripheral(peripheral: peripheral, isConnected: false))
            }
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
            print("Characteristic: \(characteristic)")
            peripheral.readValue(for: characteristic)
            peripheral.setNotifyValue(true, for: characteristic)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        self.Rx.removeAll()
        var tempRx: [String] = []
        
        if let count = characteristic.value?.count {
            for idx in 0..<count {
                if let value = characteristic.value?[idx] {
                    tempRx.append(String(format: "0x%02X", value))
                }
            }
        }
        
        self.Rx = tempRx
    }
}
