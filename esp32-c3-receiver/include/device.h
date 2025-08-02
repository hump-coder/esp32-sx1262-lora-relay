#ifndef PUMP_DEVICE_H
#define PUMP_DEVICE_H

class Device
{
    public:
    virtual void setup() = 0;
    virtual void loop() = 0;
};


#endif // PUMP_DEVICE_H
