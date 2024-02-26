//
//  BLEManager.swift
//  PressurePulse Watch App
//
//  Created by Adam Brower on 2/18/24.
//

import Foundation
import CoreBluetooth

class BLEManager: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var centralManager: CBCentralManager!
    private var connectedPeripheral: CBPeripheral?
    weak var delegate: BLEManagerDelegate?

    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            startScan()
        } else {
            // Handle Bluetooth state not powered on
            print("Bluetooth is not available")
        }
    }

    func startScan() {
        centralManager.scanForPeripherals(withServices: nil, options: nil)
        print("in startScan")
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        delegate?.didDiscoverPeripheral(peripheral)
    }

    func connectPeripheral(_ peripheral: CBPeripheral) {
        centralManager.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        connectedPeripheral = peripheral
        delegate?.didConnectPeripheral(peripheral)
    }

    func disconnectPeripheral() {
        if let peripheral = connectedPeripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        connectedPeripheral = nil
        delegate?.didDisconnectPeripheral(peripheral)
    }
}

protocol BLEManagerDelegate: AnyObject {
    func didDiscoverPeripheral(_ peripheral: CBPeripheral)
    func didConnectPeripheral(_ peripheral: CBPeripheral)
    func didDisconnectPeripheral(_ peripheral: CBPeripheral)
}

