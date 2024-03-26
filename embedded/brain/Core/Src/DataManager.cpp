#include"DataManager.h"


DataManager::DataManager() {
    _timeRemaining_Minutes = 0;
    _pressure_PSI = 0;
    _flow_LPM = 0;
    _lowPressureThreshold_PSI = 0;

    for (int i = 0; i < ARR_SIZE; i++) {
        _dataArr[i] = 0;
    }
}

// -------------------------------------------------
// functions

uint8_t* DataManager::getDataArrPtr() {
    return _dataArr;
}

// -------------------------------------------------
// getters
uint8_t DataManager::getTimeRemaining_Minutes() {
    return _timeRemaining_Minutes;
}

uint16_t DataManager::getPressure_PSI() {
    return _pressure_PSI;
}

uint8_t DataManager::getFlow_LPM() {
    return _flow_LPM;
}

uint16_t DataManager::getLowPressureThreshold_PSI() {
    return _lowPressureThreshold_PSI;
}


// -------------------------------------------------
// setters
void DataManager::setTimeRemaining_Minutes(uint8_t timeRemaining_Minutes) {
    _timeRemaining_Minutes = timeRemaining_Minutes;
    _dataArr[0] = timeRemaining_Minutes;
}

void DataManager::setPressure_PSI(uint16_t pressure_PSI) {
    _pressure_PSI = pressure_PSI;
    _dataArr[1] = (pressure_PSI&0xFF00) >> 8;
    _dataArr[2] = (pressure_PSI&0x00FF);
}

void DataManager::setFlow_LPM(uint8_t flow_LPM) {
    _flow_LPM = flow_LPM;
    _dataArr[3] = flow_LPM;
}

void DataManager::setLowPressureThreshold_PSI(uint16_t lowPressureThreshold_PSI) {
    _lowPressureThreshold_PSI = lowPressureThreshold_PSI;
    _dataArr[4] = (lowPressureThreshold_PSI&0xFF00) >> 8;
    _dataArr[5] = (lowPressureThreshold_PSI&0x00FF);
}
