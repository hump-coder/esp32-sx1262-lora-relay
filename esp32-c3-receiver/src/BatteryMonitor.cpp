#include "BatteryMonitor.h"
#include <esp32-hal-adc.h>
#include "device-config.h"

// Global instance
// Use a divider that keeps the ADC reading well away from the
// non-linear region near 0 and full scale (~100-3900 counts).
// 68k on the high side and 100k on the low side give about 2.5 V
// at 4.2 V battery voltage when using 11 dB attenuation.
// Last parameter is a calibration factor to account for ADC inaccuracies.
BatteryMonitor battery(BATTERY_VOLTAGE_PIN, 68000.0, 100000.0, 3.3, 20, 1.013);

struct VoltageSOC {
    float voltage;
    float soc;
};

static const VoltageSOC liIonCurve[] = {
    {4.20, 100}, {4.15, 95}, {4.10, 90}, {4.05, 85}, {4.00, 80},
    {3.95, 75},  {3.90, 70}, {3.85, 65}, {3.80, 60}, {3.75, 55},
    {3.70, 50},  {3.65, 45}, {3.60, 40}, {3.55, 35}, {3.50, 30},
    {3.45, 25},  {3.40, 20}, {3.35, 15}, {3.30, 10}, {3.20, 5},
    {3.00, 0}
};

BatteryMonitor::BatteryMonitor(uint8_t adcPin, float r1, float r2, float vRef, int numSamples, float calibrationFactor)
    : _adcPin(adcPin), _r1(r1), _r2(r2), _vRef(vRef), _numSamples(numSamples),
      _sampleIndex(0), _bufferFilled(false),
      _lowThreshold(3.2), _highThreshold(4.1),
      _vEmpty(3.0), _vFull(4.2),
      _chargeState(STABLE),
      _ucVoltage(true), _ucLinearPct(true), _ucCurvePct(true), _ucState(true),
      _dailyMinV(99.0), _dailyMaxV(0.0), _dailySumV(0.0),
      _dailyMinSOC(100.0), _dailyMaxSOC(0.0), _dailySumSOC(0.0),
      _dailyCount(0),
      _debug(false), _lastDebugPrint(0), _lastRawAvg(0.0f),
      _calibrationFactor(calibrationFactor) {
    _sampleBuffer = new float[_numSamples];
}

float BatteryMonitor::takeSingleReading() {
    long sumRaw = 0;
    long sumMilli = 0;
    const int quickSamples = 5;
    unsigned long now = millis();
    bool doPrint = _debug && (now - _lastDebugPrint >= 1000);
    for (int i = 0; i < quickSamples; i++) {
        int raw = analogRead(_adcPin);
        int mv = analogReadMilliVolts(_adcPin);
        sumRaw += raw;
        sumMilli += mv;
        if (doPrint) {
            Serial.printf("adc[%d]=%d\n", i, raw);
        }
        delay(3);
    }
    _lastRawAvg = sumRaw / (float)quickSamples;
    float vOut = (sumMilli / (float)quickSamples) / 1000.0;
    float vBat = vOut * ((_r1 + _r2) / _r2) * _calibrationFactor;
    if (doPrint) {
        Serial.printf("avgRaw: %.1f vOut: %.3f vBat: %.3f\n", _lastRawAvg, vOut, vBat);
    }
    return vBat;
}

void BatteryMonitor::begin() {
    pinMode(_adcPin, INPUT);
    analogReadResolution(12);
    adcAttachPin(_adcPin);
    analogSetPinAttenuation(_adcPin, ADC_11db);
    for (int i = 0; i < _numSamples; i++) {
        _sampleBuffer[i] = takeSingleReading();
    }
    _bufferFilled = true;
}

void BatteryMonitor::enableDebug(bool enable) {
    _debug = enable;
}

void BatteryMonitor::setCalibrationFactor(float factor) {
    _calibrationFactor = factor;
}

void BatteryMonitor::update() {
    float reading = takeSingleReading();
    _sampleBuffer[_sampleIndex] = reading;
    _sampleIndex = (_sampleIndex + 1) % _numSamples;
    if (_sampleIndex == 0) _bufferFilled = true;

    int lookback = _numSamples / 4;
    if (lookback < 2) lookback = 2;

    int oldIndex = (_sampleIndex - lookback + _numSamples) % _numSamples;
    float recentVoltage = _sampleBuffer[_sampleIndex == 0 ? _numSamples - 1 : _sampleIndex - 1];
    float olderVoltage = _sampleBuffer[oldIndex];

    float delta = recentVoltage - olderVoltage;
    const float hysteresis = 0.005;

    if (delta > hysteresis) {
        _chargeState = CHARGING;
    } else if (delta < -hysteresis) {
        _chargeState = DISCHARGING;
    } else {
        _chargeState = STABLE;
    }

    updateDailyStats();

    float voltage = getVoltage();
    float pctLinear = getPercentage();
    float pctCurve = getPercentageCurve();

    unsigned long now = millis();
    if (_debug && now - _lastDebugPrint >= 1000) {
        Serial.printf("rawAvg: %.1f reading: %.2f V, voltage: %.2f V, %%: %.1f, scaled %%: %.1f\n",
                      _lastRawAvg, reading, voltage, pctLinear, pctCurve);
        _lastDebugPrint = now;
    }
}

float BatteryMonitor::getVoltage() const {
    return getMovingAverage();
}

float BatteryMonitor::getMovingAverage() const {
    float sum = 0;
    int count = _bufferFilled ? _numSamples : _sampleIndex;
    for (int i = 0; i < count; i++) sum += _sampleBuffer[i];
    return sum / (float)count;
}

float BatteryMonitor::getPercentage() const {
    float v = getVoltage();
    if (v <= _vEmpty) return 0.0;
    if (v >= _vFull) return 100.0;
    return ((v - _vEmpty) / (_vFull - _vEmpty)) * 100.0;
}

float BatteryMonitor::getPercentageCurve() const {
    float v = getVoltage();
    if (v >= liIonCurve[0].voltage) return 100.0;
    if (v <= liIonCurve[sizeof(liIonCurve)/sizeof(liIonCurve[0]) - 1].voltage) return 0.0;

    for (int i = 0; i < (int)(sizeof(liIonCurve)/sizeof(liIonCurve[0])) - 1; i++) {
        if (v <= liIonCurve[i].voltage && v >= liIonCurve[i+1].voltage) {
            return interpolateCurve(v);
        }
    }
    return 0.0;
}

float BatteryMonitor::interpolateCurve(float voltage) const {
    for (int i = 0; i < (int)(sizeof(liIonCurve)/sizeof(liIonCurve[0])) - 1; i++) {
        if (voltage <= liIonCurve[i].voltage && voltage >= liIonCurve[i+1].voltage) {
            float v1 = liIonCurve[i].voltage;
            float v2 = liIonCurve[i+1].voltage;
            float s1 = liIonCurve[i].soc;
            float s2 = liIonCurve[i+1].soc;
            return s1 + (s2 - s1) * ((v1 - voltage) / (v1 - v2));
        }
    }
    return 0.0;
}

void BatteryMonitor::updateDailyStats() {
    float v = getVoltage();
    float soc = getPercentageCurve();

    if (v < _dailyMinV) _dailyMinV = v;
    if (v > _dailyMaxV) _dailyMaxV = v;
    _dailySumV += v;

    if (soc < _dailyMinSOC) _dailyMinSOC = soc;
    if (soc > _dailyMaxSOC) _dailyMaxSOC = soc;
    _dailySumSOC += soc;

    _dailyCount++;
}

void BatteryMonitor::resetDailyStats() {
    _dailyMinV = 99.0;
    _dailyMaxV = 0.0;
    _dailySumV = 0.0;

    _dailyMinSOC = 100.0;
    _dailyMaxSOC = 0.0;
    _dailySumSOC = 0.0;

    _dailyCount = 0;
}

float BatteryMonitor::getDailyMinVoltage() const {
    return (_dailyCount > 0) ? _dailyMinV : 0.0;
}

float BatteryMonitor::getDailyMaxVoltage() const {
    return (_dailyCount > 0) ? _dailyMaxV : 0.0;
}

float BatteryMonitor::getDailyAverageVoltage() const {
    return (_dailyCount > 0) ? (_dailySumV / _dailyCount) : 0.0;
}

float BatteryMonitor::getDailyMinSOC() const {
    return (_dailyCount > 0) ? _dailyMinSOC : 0.0;
}

float BatteryMonitor::getDailyMaxSOC() const {
    return (_dailyCount > 0) ? _dailyMaxSOC : 0.0;
}

float BatteryMonitor::getDailyAverageSOC() const {
    return (_dailyCount > 0) ? (_dailySumSOC / _dailyCount) : 0.0;
}

String BatteryMonitor::getDailyStatsJSON() const {
    String json = "{";
    json += "\"minV\":" + String(getDailyMinVoltage(), 2) + ",";
    json += "\"maxV\":" + String(getDailyMaxVoltage(), 2) + ",";
    json += "\"avgV\":" + String(getDailyAverageVoltage(), 2) + ",";
    json += "\"minSOC\":" + String(getDailyMinSOC(), 1) + ",";
    json += "\"maxSOC\":" + String(getDailyMaxSOC(), 1) + ",";
    json += "\"avgSOC\":" + String(getDailyAverageSOC(), 1);
    json += "}";
    return json;
}




String BatteryMonitor::trimFloat(float value, uint8_t decimals) const {
    String s = String(value, (int) decimals);
    while (s.endsWith("0")) s.remove(s.length() - 1);
    if (s.endsWith(".")) s.remove(s.length() - 1);
    return s;
}

String BatteryMonitor::getDailyStatsCompact() const {
    String msg;
    msg.reserve(48);
    msg += "m" + trimFloat(getDailyMinVoltage(), 2);
    msg += "M" + trimFloat(getDailyMaxVoltage(), 2);
    msg += "A" + trimFloat(getDailyAverageVoltage(), 2);
    msg += "s" + trimFloat(getDailyMinSOC(), 1);
    msg += "S" + trimFloat(getDailyMaxSOC(), 1);
    msg += "a" + trimFloat(getDailyAverageSOC(), 1);
    return msg;
}
