#include "battery.h"
#include <math.h>

#define VBAT_Read 1
#define ADC_Ctrl 37


void Battery::setup()
{
    pinMode(CHARGE_STATUS_PIN, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
}

float Battery::getPercentage()
{
    const float min = 3.0f;
    const float max = 4.2f;

    readBatteryVoltage();

    float percent = ((mVoltage - min) / (max - min)) * 100.0f;
    // Round to one decimal place
    percent = roundf(percent * 10.0f) / 10.0f;

    if (percent < 0.0f)
        percent = 0.0f;
    if (percent > 100.0f)
        percent = 100.0f;
    return percent;
}

bool Battery::isCharging()
{
    return digitalRead(CHARGE_STATUS_PIN) == HIGH;
}


Battery::Battery()
{
    pinMode(ADC_Ctrl, OUTPUT);
    pinMode(VBAT_Read, INPUT);
    adcAttachPin(VBAT_Read);
    analogReadResolution(12);
    mVoltage = readBatteryVoltage();
}

float Battery::getVoltage()
{
    readBatteryVoltage();
    return mVoltage;
}

float Battery::readBatteryVoltage()
{
    // ADC resolution
    const int resolution = 12;
    const int adcMax = pow(2, resolution) - 1;
    const float adcMaxVoltage = 3.3;
    // On-board voltage divider
    const int R1 = 390;
    const int R2 = 100;
    // Calibration measurements
    const float measuredVoltage = 4.2;
    const float reportedVoltage = 3.88;
    // const float reportedVoltage = 4.095;
    // Calibration factor
    const float factor = (adcMaxVoltage / adcMax) * ((R1 + R2) / (float)R2) * (measuredVoltage / reportedVoltage);

    if (millis() - lastRead > 15000)
    {
        digitalWrite(ADC_Ctrl, HIGH);
        delay(100);
        int analogValue = analogRead(VBAT_Read);
        digitalWrite(ADC_Ctrl, LOW);

        float floatVoltage = factor * analogValue;
        // uint16_t voltage = (int)(floatVoltage * 1000.0);

        // // Serial.print("[readBatteryVoltage] ADC : ");
        // // Serial.println(analogValue);
        // // Serial.print("[readBatteryVoltage] Float : ");
        // // Serial.println(floatVoltage, 3);
        // Serial.print("[readBatteryVoltage] milliVolts : ");
        // Serial.println(voltage);

        //  delay(1000);

        mVoltage = floatVoltage;
        lastRead = millis();
    }

    return mVoltage;
}