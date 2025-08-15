#ifndef PUMP_CONTROLLER_H
#define PUMP_CONTROLLER_H

#include <WiFi.h>
#include <PubSubClient.h>
#include "device.h"
#include "config.h"
#include "settings.h"
#include "BatteryMonitor.h"
#include <deque>

struct DailyStats;

struct StatsWindow {
    struct Event {
        unsigned long time;
        size_t bytes;
        bool sent;
    };
    unsigned long period;
    std::deque<Event> events;
    unsigned long msgsSent = 0;
    unsigned long msgsReceived = 0;
    unsigned long bytesSent = 0;
    unsigned long bytesReceived = 0;

    explicit StatsWindow(unsigned long p = 0) : period(p) {}

    void add(unsigned long now, size_t bytes, bool sent) {
        events.push_back({now, bytes, sent});
        if (sent) {
            msgsSent++;
            bytesSent += bytes;
        } else {
            msgsReceived++;
            bytesReceived += bytes;
        }
        prune(now);
    }

    void prune(unsigned long now) {
        while (!events.empty() && now - events.front().time >= period) {
            const auto &e = events.front();
            if (e.sent) {
                msgsSent--;
                bytesSent -= e.bytes;
            } else {
                msgsReceived--;
                bytesReceived -= e.bytes;
            }
            events.pop_front();
        }
    }
};

struct RttStats {
    unsigned long last = 0;
    unsigned long min = 0;
    unsigned long max = 0;
    unsigned long total = 0;
    unsigned long count = 0;

    void add(unsigned long rtt) {
        last = rtt;
        if (count == 0 || rtt < min) {
            min = rtt;
        }
        if (rtt > max) {
            max = rtt;
        }
        total += rtt;
        count++;
    }

    unsigned long avg() const { return count ? total / count : 0; }
};

enum RelayState
{
    UNKNOWN,
    ON,
    OFF
};

class Controller : public Device
{
public:
    Controller();

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

    WiFiClient espClient;
    PubSubClient mqttClient;
    void ensureMqtt();
    void sendMessage(const char *msg);
    void enqueueMessage(const char *msg);
    void processQueue();
    void sendCurrentMessage();
    void setRelayState(bool pumpOn, unsigned int onTime = DEFAULT_ON_TIME_SEC, bool pulse = false);
    void pulseRelay(unsigned int onTime);

    void publishControllerStatus();
    void publishReceiverStatus(int power, int rssi, int snr, bool relay, bool pulse, float battery,
                               int chargeState, int wifi, unsigned long uptime);
    void publishReceiverDailyStats(const struct DailyStats &stats);

    void setSendStatusFrequency(unsigned int freq);
    unsigned int getSendStatusFrequency() const { return statusSendFreqSec; }

    unsigned int onTimeSec = DEFAULT_ON_TIME_SEC;
    unsigned int statusSendFreqSec = DEFAULT_CONTROLLER_STATUS_SEND_FREQ_SEC;
    unsigned int receiverStatusFreqSec = DEFAULT_RECEIVER_STATUS_SEND_FREQ_SEC;
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
        unsigned long sendTime;
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

    // Statistics helpers
    void publishStatistics();
    void updateStats(size_t bytes, bool sent);

    StatsWindow minuteStats{60000UL};
    StatsWindow hourStats{3600000UL};
    StatsWindow dayStats{86400000UL};
    unsigned long totalMsgsSent = 0;
    unsigned long totalMsgsReceived = 0;
    unsigned long bootTime = 0;
    unsigned long receiverUptimeSec = 0;
    unsigned long lastStatsPublish = 0;

    RttStats rttStats;

    void publishState();
    void sendDiscovery();
    void setIdle();
};

#endif // PUMP_CONTROLLER_H
