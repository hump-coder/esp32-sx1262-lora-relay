#ifndef PUMP_RECEIVER_H
#define PUMP_RECEIVER_H

#include "device.h"
#include "device-config.h"
#include "settings.h"


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

    private:
    void sendHello();
    void updateDisplay();
    void setIdle();
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
    int acksRemaining;
    uint16_t ackStateId;
    bool ackConfirmed;
    int txPower = TX_OUTPUT_POWER;
    unsigned int statusSendFreqSec = DEFAULT_STATUS_SEND_FREQ_SEC;
    unsigned long lastStatusSend = 0;
    bool mIsTransmitting = false;
    void sendAck(char *rxpacket);
    void setRelayState(bool newRelayState);
    void processReceived(char *rxpacket);
    void send(char *packet, size_t len);
    void actuateRelay(bool state);




};


#endif // PUMP_RECEIVER_H
