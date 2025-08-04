#ifndef PUMP_RECEIVER_H
#define PUMP_RECEIVER_H

#include "device.h"
#include "config.h"
#include "settings.h"
#include "BatteryMonitor.h"


class Receiver : public Device
{
    public:
    Receiver();
    void setup() override;
    void loop() override;

    void OnRxDone(String payload, int16_t rssi, int8_t snr );
    void OnTxDone();
    void OnTxTimeout();

    int getTxPower() const { return txPower; }
    void setTxPower(int power);
    void setSendStatusFrequency(unsigned int freq);
    unsigned int getSendStatusFrequency() const { return statusSendFreqSec; }

    void sendStatus();
    void sendDailyStats();

    private:
    void sendHello();
    void updateDisplay();
    void setIdle();
    int getWifiState();
    void updateStatusCache();
    private:
    
    String mLastMessage;
    uint16_t mLastMessageSize;
    int16_t mLastRssi;
    int8_t mLastSnr;
    bool mRelayState;
    bool mPulseMode = false;
    unsigned long offTime = 0;
    // Duration to keep the relay on. This value is provided by the
    // controller in the ON message.
    unsigned int onTimeSec = 0;
    uint16_t lastCommandId = 0;
    int txPower = TX_OUTPUT_POWER;
    unsigned int statusSendFreqSec = DEFAULT_STATUS_SEND_FREQ_SEC;
    unsigned long lastStatusSend = 0;
    bool pendingDailyStats = false;
    bool mIsTransmitting = false;
    float lastBatteryPct = -1;
    ChargeState lastChargeState = DISCHARGING;
    int lastWifiState = WIFI_DISABLED;
    void sendAck(uint16_t id, const char *status);
    void setRelayState(bool newRelayState);
    void processReceived(char *rxpacket);
    void send(char *packet, size_t len);
    void actuateRelay(bool state);
    void connectWifi(const char *ssid, const char *pass);
    void disableWifi();
    bool otaEnabled = false;




};


#endif // PUMP_RECEIVER_H
