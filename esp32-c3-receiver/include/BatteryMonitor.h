#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

enum ChargeState {
    CHARGING,
    DISCHARGING,
    STABLE
};

class BatteryMonitor {
public:
    BatteryMonitor(uint8_t adcPin, float r1, float r2, float vRef, int numSamples, float calibrationFactor = 1.0);

    void begin();
    void update();
    float getVoltage() const;

    // Threshold setters
    void setLowThreshold(float voltage);
    void setHighThreshold(float voltage);

    // Threshold checks
    bool isLow() const;
    bool isFull() const;

    // Percentage calculation
    void setVoltageRange(float vEmpty, float vFull);
    float getPercentage() const;
    float getPercentageCurve() const;

    // Charge state
    ChargeState getChargeState() const;

    // Status output
    String getStatusJSON() const;
    String getStatusCompact() const;
    String getStatusUltraCompact() const;

    // Compact output config
    void setUltraCompactFields(bool voltage, bool linearPct, bool curvePct, bool state);

    // Daily stats
    void updateDailyStats();
    void resetDailyStats();
    float getDailyMinVoltage() const;
    float getDailyMaxVoltage() const;
    float getDailyAverageVoltage() const;
    float getDailyMinSOC() const;
    float getDailyMaxSOC() const;
    float getDailyAverageSOC() const;
    String getDailyStatsJSON() const;
    String getDailyStatsCompact() const;

    // Debug control
    void enableDebug(bool enable);

    // Calibration
    void setCalibrationFactor(float factor);

private:
    float takeSingleReading();
    float getMovingAverage() const;
    float interpolateCurve(float voltage) const;

    const char* chargeStateToStringShort() const;
    const char* chargeStateToStringLong() const;

    String trimFloat(float value, uint8_t decimals) const;

    uint8_t _adcPin;
    float _r1;
    float _r2;
    float _vRef;
    int _numSamples;

    float* _sampleBuffer;
    int _sampleIndex;
    bool _bufferFilled;

    float _lowThreshold;
    float _highThreshold;

    float _vEmpty;
    float _vFull;

    mutable ChargeState _chargeState;

    bool _ucVoltage;
    bool _ucLinearPct;
    bool _ucCurvePct;
    bool _ucState;

    // Daily voltage stats
    float _dailyMinV;
    float _dailyMaxV;
    float _dailySumV;

    // Daily SOC stats (curve-based)
    float _dailyMinSOC;
    float _dailyMaxSOC;
    float _dailySumSOC;

    unsigned long _dailyCount;

    // Debug helpers
    bool _debug;
    unsigned long _lastDebugPrint;
    float _lastRawAvg;
    float _calibrationFactor;
};

extern BatteryMonitor battery;

#endif
