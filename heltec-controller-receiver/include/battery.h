#ifndef PUMP_BATTERY_H
#define PUMP_BATTERY_H

#include <Arduino.h>
#include "device-config.h"

class Battery {
public:
    Battery();
    void setup();
    int getPercentage();
    float getVoltage();
    bool isCharging();

    private:
    float mVoltage=0;
    unsigned long lastRead = -9999999;

    float readBatteryVoltage();
};

#endif // PUMP_BATTERY_H
