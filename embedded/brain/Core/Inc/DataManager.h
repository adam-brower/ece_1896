#include "main.h"

class DataManager {
public:
    DataManager();

    // getters
    uint8_t* getDataArrPtr();



    uint8_t getTimeRemaining_Minutes();
    uint16_t getPressure_PSI();
    uint8_t getFlow_LPM();
    uint16_t getLowPressureThreshold_PSI();

    // setters
    void setTimeRemaining_Minutes(uint8_t timeRemaining_Minutes);
    void setPressure_PSI(uint16_t pressure_PSI);
    void setFlow_LPM(uint8_t flow_LPM);
    void setLowPressureThreshold_PSI(uint16_t lowPressureThreshold_PSI);

    static const int ARR_SIZE = 6;

private:
    uint8_t  _timeRemaining_Minutes;
    uint16_t _pressure_PSI;
    uint8_t  _flow_LPM;
    uint16_t _lowPressureThreshold_PSI;

    // Index Chart
    // 0 = Time
    // 1 = Pressure bits 15-8
    // 2 = Pressure bits 7-0
    // 3 = Flow
    // 4 = Low Thresh bits 15-8
    // 5 = Low Thresh bits 7-0
    uint8_t _dataArr[ARR_SIZE];


};

//                 Time  Pressure 1 Pressure 2 Flow  Low Thresh 1 Low Thresh 2
// uint8_t data[6] = {0x00, 0x07,      0xD0,      0x0F, 0x00,        0xE1};
