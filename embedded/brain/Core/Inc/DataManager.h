#include "main.h"


class DataManager {
public:
    DataManager();

    // getters
    uint8_t getTimeRemaining();
    void getPressure();
    uint8_t getFlow();
    void getLowPressureThreshold();

    // setters
    void setTimeRemaining(uint8_t timeRemaining);
    void setPressure(uint16_t Pressure);
    void setFlow(uint8_t  flow);
    void setLowPressureThreshold(uint16_t lowPressureThreshold);


private:
    uint8_t  _timeRemaining;
    uint16_t _Pressure;
    uint8_t  _flow;
    uint16_t _lowPressureThreshold;
};

//                 Time  Pressure 1 Pressure 2 Flow  Low Thresh 1 Low Thresh 2
// uint8_t data[6] = {0x00, 0x07,      0xD0,      0x0F, 0x00,        0xE1};
