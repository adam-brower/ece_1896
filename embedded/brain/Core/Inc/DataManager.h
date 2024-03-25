#include "main.h"


class DataManager {
    public:

    // getters
    uint8_t getTimeRemaining();
    void getPressure();
    uint8_t getFlow();
    void getLowPressureThreshold();

    // set



    // void setTimeRemaining



    private:
        uint8_t  _timeRemaining;
        uint16_t _Pressure;
        uint8_t  _flow;
        uint16_t _lowPressureThreshold;
};

//                 Time  Pressure 1 Pressure 2 Flow  Low Thresh 1 Low Thresh 2
// uint8_t data[6] = {0x00, 0x07,      0xD0,      0x0F, 0x00,        0xE1};
