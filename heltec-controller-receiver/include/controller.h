#ifndef PUMP_CONTROLLER_H
#define PUMP_CONTROLLER_H

#include "LoRaWan_APP.h"
#include <WiFi.h>
#include <PubSubClient.h>

#include "device.h"
#include "display.h"
#include "device-config.h"
#include "settings.h"
#include <deque>

struct DailyStats;

enum RelayState
{
    UNKNOWN,
    ON,
    OFF
};

class Controller : public Device
{
public:
    Controller(Display &display);

    void setup() override;
    void loop() override;

    void mqttCallback(char *topic, byte *payload, unsigned int length);
    void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
    void OnTxDone();
    void OnTxTimeout();

    void processReceived(char *rxpacket);

    int getTxPower() const { return txPower; }
    void setTxPower(int power);

private:
    Display &mDisplay;
    String mLastMessage;
    bool mHasBattery = false;
    uint16_t mLastMessageSize;
    int16_t mLastRssi;
    int8_t mLastSnr;
    uint16_t mStateId;
    RelayState relayState;
    RelayState requestedRelayState;

    char txpacket[BUFFER_SIZE];
    char rxpacket[BUFFER_SIZE];

    double txNumber;
    int txPower = TX_OUTPUT_POWER;
    int receiverTxPower = TX_OUTPUT_POWER;

    bool lora_idle = true;

    RadioEvents_t RadioEvents;

    WiFiClient espClient;
    PubSubClient mqttClient; //(espClient);
    void ensureMqtt();
    void updateDisplay();
    void sendMessage(const char *msg);
    void enqueueMessage(const char *msg);
    void processQueue();
    void sendCurrentMessage();
    void setRelayState(bool pumpOn, unsigned int onTime = DEFAULT_ON_TIME_SEC, bool pulse = false);
    void pulseRelay(unsigned int onTime);

    void publishControllerStatus();
    void publishReceiverStatus(int power, int rssi, int snr, bool relay, bool pulse, float battery,
                               int chargeState, int wifi);
    void publishReceiverDailyStats(const struct DailyStats &stats);

    void setSendStatusFrequency(unsigned int freq);
    unsigned int getSendStatusFrequency() const { return statusSendFreqSec; }

    unsigned int onTimeSec = DEFAULT_ON_TIME_SEC;
    unsigned int statusSendFreqSec = DEFAULT_STATUS_SEND_FREQ_SEC;
    unsigned int receiverStatusFreqSec = DEFAULT_STATUS_SEND_FREQ_SEC;
    unsigned long autoOffTime = 0;
    unsigned long lastStatusPublish = 0;
    bool pulseMode = false;

    // Timestamp of the last packet received from the receiver
    unsigned long lastContactTime = 0;

    // Next time to refresh the ON command before receiver timeout
    unsigned long nextRelayRefresh = 0;

    // Outgoing message queue
    struct OutgoingMessage {
        String type;
        String payload;
        uint16_t id;
        uint8_t attempts;
    };
    std::deque<OutgoingMessage> outbox;
    bool awaitingAck = false;
    unsigned long lastSendAttempt = 0;
    static const unsigned long RETRY_INTERVAL_MS = 3000;
    static const uint8_t MAX_RETRIES = 4;

    // Flag set when a command is received on
    // pump_station/switch/set immediately after connecting
    bool initialSetReceived = false;

    // Tracks the retained pump_station/switch/state value at startup
    bool initialStateReceived = false;
    bool retainedStateOn = false;

    // unsigned int messageNumnber = 0;
    void publishState();
    void sendDiscovery();
};

#endif // PUMP_CONTROLLER_H
