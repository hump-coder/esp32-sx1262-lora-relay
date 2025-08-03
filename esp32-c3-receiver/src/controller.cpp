#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <RadioLib.h>

#include "config.h"
#include "device-config.h"
#include "config-private.h"
#include "settings.h"
#include "BatteryMonitor.h"
#include "controller.h"
#include <ctype.h>
#include <string.h>

#define DEVICE_NAME "dam-pump-controller-c3-sx1262"

// XL1262 LoRa pin mapping to ESP32-C3 (same as receiver)
#define PIN_LORA_MOSI 6
#define PIN_LORA_MISO 5
#define PIN_LORA_SCK 10
#define PIN_LORA_CS 7
#define PIN_LORA_RESET 8
#define PIN_LORA_DIO1 4
#define PIN_LORA_BUSY 3

// radio configuration
#define RF_FREQUENCY                                915000000
#define TX_OUTPUT_POWER                             14
#define MIN_TX_OUTPUT_POWER                         2
#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       9
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

static SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);

Controller *controllerInstance;

static volatile bool activityFlag = false;
static int transmissionState = RADIOLIB_ERR_NONE;

ICACHE_RAM_ATTR static void setFlag(void) {
    activityFlag = true;
}

static void checkReceiveData();
static void checkTransmittedData();

struct DailyStats {
    float minV = 0;
    float maxV = 0;
    float avgV = 0;
    float minSOC = 0;
    float maxSOC = 0;
    float avgSOC = 0;
};

static bool parseDailyStats(const char *str, DailyStats &stats) {
    const char *p = str;
    while (*p) {
        char label = *p++;
        char buf[16];
        int i = 0;
        while (*p && (isdigit(*p) || *p == '.' || *p == '-')) {
            if (i < (int)sizeof(buf) - 1) buf[i++] = *p;
            p++;
        }
        buf[i] = '\0';
        float val = atof(buf);
        switch (label) {
            case 'm': stats.minV = val; break;
            case 'M': stats.maxV = val; break;
            case 'A': stats.avgV = val; break;
            case 's': stats.minSOC = val; break;
            case 'S': stats.maxSOC = val; break;
            case 'a': stats.avgSOC = val; break;
            default: break;
        }
    }
    return true;
}

Controller::Controller() : mqttClient(espClient) {
    controllerInstance = this;
    mLastMessage = "";
    mLastMessageSize = 0;
    mLastRssi = 0;
    mLastSnr = 0;
    mStateId = 1;
    relayState = RelayState::OFF;
    requestedRelayState = RelayState::UNKNOWN;
    pulseMode = false;
    lastStatusPublish = 0;
    initialSetReceived = false;
    initialStateReceived = false;
    retainedStateOn = false;
}

static String relayStateToString(RelayState state) {
    switch (state) {
        case RelayState::OFF: return "OFF";
        case RelayState::ON: return "ON";
        default: return "UNKNOWN";
    }
}

void Controller::publishState() {
    mqttClient.publish("pump_station/switch/state", relayStateToString(relayState).c_str(), true);
}

void Controller::publishControllerStatus() {
    char payload[128];
    int batt = (int)battery.getFilteredPercentage();
    snprintf(payload, sizeof(payload),
             "{\"power\":%d,\"rssi\":%d,\"snr\":%d,\"state\":\"%s\",\"mode\":\"%s\",\"battery\":%d}",
             txPower, mLastRssi, mLastSnr,
             relayStateToString(relayState).c_str(),
             pulseMode ? "pulse" : "normal", batt);
    mqttClient.publish("pump_station/status/controller", payload, true);
}

void Controller::publishReceiverStatus(int power, int rssi, int snr, bool relay, bool pulse, int batteryPct,
                                       int chargeState, int wifi) {
    char payload[200];
    const char *charge;
    switch (chargeState) {
        case 0: charge = "CHARGING"; break;
        case 1: charge = "DISCHARGING"; break;
        default: charge = "UNKNOWN"; break;
    }
    const char *wifiState;
    switch (wifi) {
        case WIFI_DISABLED: wifiState = "DISABLED"; break;
        case WIFI_CONNECTED: wifiState = "CONNECTED"; break;
        case WIFI_CONNECTING: wifiState = "CONNECTING"; break;
        case WIFI_ERROR: wifiState = "ERROR"; break;
        default: wifiState = "UNKNOWN"; break;
    }
    snprintf(payload, sizeof(payload),
             "{\"power\":%d,\"rssi\":%d,\"snr\":%d,\"state\":\"%s\",\"mode\":\"%s\",\"battery\":%d,\"charge\":\"%s\",\"wifi\":\"%s\"}",
             power, rssi, snr,
             relay ? "ON" : "OFF",
             pulse ? "pulse" : "normal", batteryPct,
             charge, wifiState);
    mqttClient.publish("pump_station/status/receiver", payload, true);
}

void Controller::publishReceiverDailyStats(const DailyStats &stats) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"minV\":%.2f,\"maxV\":%.2f,\"avgV\":%.2f,\"minSOC\":%.1f,\"maxSOC\":%.1f,\"avgSOC\":%.1f}",
             stats.minV, stats.maxV, stats.avgV, stats.minSOC, stats.maxSOC, stats.avgSOC);
    mqttClient.publish("pump_station/status/receiver/battery_daily", payload, true);
}

void controllerMqttCallback(char *topic, byte *payload, unsigned int length) {
    controllerInstance->mqttCallback(topic, payload, length);
}

void Controller::mqttCallback(char *topic, byte *payload, unsigned int length) {
    String cmd;
    for (unsigned int i = 0; i < length; i++) {
        cmd += (char)payload[i];
    }
    if(strcmp(topic, "pump_station/switch/set") == 0) {
        initialSetReceived = true;
        if(cmd == "ON") {
            setRelayState(true, DEFAULT_ON_TIME_SEC, false);
        } else if(cmd == "OFF") {
            setRelayState(false);
        }
    } else if(strcmp(topic, "pump_station/switch/pulse") == 0) {
        unsigned int dur = cmd.toInt();
        pulseRelay(dur);
    } else if(strcmp(topic, "pump_station/tx_power/controller/set") == 0) {
        int power = cmd.toInt();
        setTxPower(power);
        Settings::setInt(KEY_CTRL_TX_POWER, power);
    } else if(strcmp(topic, "pump_station/tx_power/receiver/set") == 0) {
        int power = cmd.toInt();
        receiverTxPower = power;
        Settings::setInt(KEY_RX_TX_POWER, power);
        char msg[16];
        ++mStateId;
        sprintf(msg, "PWR:%d", power);
        enqueueMessage(msg);
    } else if(strcmp(topic, "pump_station/status_freq/controller/set") == 0) {
        unsigned int freq = cmd.toInt();
        setSendStatusFrequency(freq);
        Settings::setInt(KEY_CTRL_STATUS_FREQ, freq);
    } else if(strcmp(topic, "pump_station/status_freq/receiver/set") == 0) {
        unsigned int freq = cmd.toInt();
        receiverStatusFreqSec = freq;
        Settings::setInt(KEY_RX_STATUS_FREQ, freq);
        char msg[16];
        ++mStateId;
        sprintf(msg, "FREQ:%u", freq);
        enqueueMessage(msg);
    }
}

void Controller::sendDiscovery() {
    const char *switchPayload = "{\"name\":\"Pump\",\"command_topic\":\"pump_station/switch/set\",\"state_topic\":\"pump_station/switch/state\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"unique_id\":\"pump_station\"}";
    mqttClient.publish("homeassistant/switch/pump_station/config", switchPayload, true);
    const char *pulsePayload = "{\"name\":\"Pump Pulse\",\"command_topic\":\"pump_station/switch/pulse\",\"min\":1,\"max\":3600,\"step\":1,\"unit_of_measurement\":\"s\",\"unique_id\":\"pump_station_pulse\"}";
    mqttClient.publish("homeassistant/number/pump_station_pulse/config", pulsePayload, true);
    const char *ctrlPowerPayload = "{\"name\":\"Controller Tx Power\",\"command_topic\":\"pump_station/tx_power/controller/set\",\"min\":0,\"max\":22,\"step\":1,\"unit_of_measurement\":\"dBm\",\"unique_id\":\"pump_station_ctrl_power\"}";
    mqttClient.publish("homeassistant/number/pump_station_ctrl_power/config", ctrlPowerPayload, true);
    const char *rxPowerPayload = "{\"name\":\"Receiver Tx Power\",\"command_topic\":\"pump_station/tx_power/receiver/set\",\"min\":0,\"max\":22,\"step\":1,\"unit_of_measurement\":\"dBm\",\"unique_id\":\"pump_station_rx_power\"}";
    mqttClient.publish("homeassistant/number/pump_station_rx_power/config", rxPowerPayload, true);
    const char *ctrlStatusPayload = "{\"name\":\"Controller Status\",\"state_topic\":\"pump_station/status/controller\",\"value_template\":\"{{ value_json.state }}\",\"json_attributes_topic\":\"pump_station/status/controller\",\"unique_id\":\"pump_station_ctrl_status\"}";
    mqttClient.publish("homeassistant/sensor/pump_station_ctrl_status/config", ctrlStatusPayload, true);
    const char *rxStatusPayload = "{\"name\":\"Receiver Status\",\"state_topic\":\"pump_station/status/receiver\",\"value_template\":\"{{ value_json.state }}\",\"json_attributes_topic\":\"pump_station/status/receiver\",\"unique_id\":\"pump_station_rx_status\"}";
    mqttClient.publish("homeassistant/sensor/pump_station_rx_status/config", rxStatusPayload, true);
    const char *rxBattDailyPayload = "{\"name\":\"Receiver Battery Daily\",\"state_topic\":\"pump_station/status/receiver/battery_daily\",\"value_template\":\"{{ value_json.avgV }}\",\"json_attributes_topic\":\"pump_station/status/receiver/battery_daily\",\"unique_id\":\"pump_station_batt_daily\"}";
    mqttClient.publish("homeassistant/sensor/pump_station_batt_daily/config", rxBattDailyPayload, true);
}

void Controller::ensureMqtt() {
    if (!mqttClient.connected()) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            unsigned long start = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
                delay(500);
            }
        }
        if (WiFi.status() == WL_CONNECTED) {
            mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
            mqttClient.setCallback(controllerMqttCallback);
            if (mqttClient.connect(DEVICE_NAME, MQTT_USER, MQTT_PASSWORD)) {
                mqttClient.subscribe("pump_station/switch/set");
                mqttClient.subscribe("pump_station/switch/pulse");
                mqttClient.subscribe("pump_station/tx_power/controller/set");
                mqttClient.subscribe("pump_station/tx_power/receiver/set");
                mqttClient.subscribe("pump_station/status_freq/controller/set");
                mqttClient.subscribe("pump_station/status_freq/receiver/set");
                sendDiscovery();
                publishState();
                publishControllerStatus();
            }
        }
    }
}

void Controller::setup() {
    Settings::begin();
    txPower = Settings::getInt(KEY_CTRL_TX_POWER, TX_OUTPUT_POWER);
    receiverTxPower = Settings::getInt(KEY_RX_TX_POWER, TX_OUTPUT_POWER);
    statusSendFreqSec = Settings::getInt(KEY_CTRL_STATUS_FREQ, DEFAULT_STATUS_SEND_FREQ_SEC);
    receiverStatusFreqSec = Settings::getInt(KEY_RX_STATUS_FREQ, DEFAULT_STATUS_SEND_FREQ_SEC);

    Serial.begin(115200);
    battery.begin();
    battery.enableDebug(false);

    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("radio init failed: %d\n", state);
        while (true) { delay(10); }
    }
    radio.setDio1Action(setFlag);
    radio.setFrequency(RF_FREQUENCY / 1000000.0);
    radio.setBandwidth(LORA_BANDWIDTH == 0 ? 125.0 : (LORA_BANDWIDTH == 1 ? 250.0 : 500.0));
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setCodingRate(LORA_CODINGRATE + 4);
    radio.setOutputPower(txPower);
    radio.setPreambleLength(LORA_PREAMBLE_LENGTH);
    radio.invertIQ(LORA_IQ_INVERSION_ON);
    if (LORA_FIX_LENGTH_PAYLOAD_ON) {
        radio.implicitHeader(LORA_SYMBOL_TIMEOUT);
    } else {
        radio.explicitHeader();
    }
    setIdle();

    ensureMqtt();
    lastStatusPublish = millis();
}

void Controller::setIdle() {
    radio.sleep();
    delay(100);
    int attempts = 10;
    int16_t st = RADIOLIB_ERR_NONE;
    while (--attempts >= 0 && (st = radio.startReceive()) != RADIOLIB_ERR_NONE) {
        delay(100);
    }
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("failed to enter receive mode: %d\n", st);
    } else {
        lora_idle = true;
    }
}

void Controller::enqueueMessage(const char *msg) {
    char tmp[BUFFER_SIZE];
    strncpy(tmp, msg, sizeof(tmp));
    char *type = strtok(tmp, ":");
    char *payload = strtok(nullptr, "");
    OutgoingMessage om;
    om.type = type ? type : "";
    om.payload = payload ? payload : "";
    om.id = mStateId++;
    om.attempts = 0;
    outbox.push_back(om);
    processQueue();
}

void Controller::sendCurrentMessage() {
    if (outbox.empty()) return;
    OutgoingMessage &om = outbox.front();
    char buffer[BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "%s:%u:%s", om.type.c_str(), om.id, om.payload.c_str());
    sendMessage(buffer);
    awaitingAck = true;
    om.attempts++;
    lastSendAttempt = millis();
}

void Controller::processQueue() {
    if (awaitingAck) {
        if (!outbox.empty() && millis() - lastSendAttempt > RETRY_INTERVAL_MS) {
            if (outbox.front().attempts < MAX_RETRIES) {
                sendCurrentMessage();
            } else {
                awaitingAck = false;
                outbox.pop_front();
                setIdle();
            }
        }
        return;
    }
    if (!outbox.empty()) {
        sendCurrentMessage();
    }
}

void Controller::sendMessage(const char *msg) {
    lora_idle = false;
    Serial.printf("Sending \"%s\"\n", msg);
    transmissionState = radio.startTransmit((uint8_t*)msg, strlen(msg));
}

void Controller::setTxPower(int power) {
    txPower = constrain(power, MIN_TX_OUTPUT_POWER, 22);
    radio.setOutputPower(txPower);
    publishControllerStatus();
}

void Controller::setSendStatusFrequency(unsigned int freq) {
    statusSendFreqSec = freq;
    publishControllerStatus();
}

void Controller::setRelayState(bool pumpOn, unsigned int onTime, bool pulse) {
    requestedRelayState = pumpOn ? RelayState::ON : RelayState::OFF;
    pulseMode = pulse;
    onTimeSec = onTime;
    char msg[32];
    if (pumpOn) {
        if (pulse) {
            snprintf(msg, sizeof(msg), "PULSE:%u", onTimeSec);
        } else {
            snprintf(msg, sizeof(msg), "ON:%u", onTimeSec);
        }
        nextRelayRefresh = millis() + ((unsigned long)onTimeSec * 1000UL / 2);
    } else {
        snprintf(msg, sizeof(msg), "OFF");
        nextRelayRefresh = 0;
    }
    enqueueMessage(msg);
    publishState();
}

void Controller::pulseRelay(unsigned int onTime) {
    setRelayState(true, onTime, true);
}

void Controller::loop() {
    if (lora_idle) {
        checkReceiveData();
    } else {
        checkTransmittedData();
    }

    if (!mqttClient.connected()) {
        ensureMqtt();
    }
    if(millis() - lastStatusPublish > statusSendFreqSec * 1000UL) {
        publishControllerStatus();
        lastStatusPublish = millis();
    }
    mqttClient.loop();
    processQueue();

    if(autoOffTime && millis() > autoOffTime) {
        setRelayState(false);
        autoOffTime = 0;
    }

    if(requestedRelayState == RelayState::ON && relayState == RelayState::ON &&
       lastContactTime && millis() - lastContactTime > COMMUNICATION_TIMEOUT_MS) {
        relayState = RelayState::UNKNOWN;
        publishState();
    }

    if(requestedRelayState == RelayState::ON && !pulseMode && !awaitingAck &&
       nextRelayRefresh && millis() > nextRelayRefresh) {
        char msg[32];
        sprintf(msg, "ON:%u", onTimeSec);
        enqueueMessage(msg);
        nextRelayRefresh = millis() + ((unsigned long)onTimeSec * 1000UL / 2);
    }
}

void Controller::processReceived(char *rxpacket) {
    char *strings[10];
    char *ptr = NULL;
    int index = 0;

    lastContactTime = millis();
    ptr = strtok(rxpacket, ":;");
    while (ptr != NULL) {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, ":;");
    }

    if (index >= 3) {
        if (strlen(strings[0]) == 1 && strings[0][0] == 'A') {
            uint16_t stateId = atoi(strings[1]);
            if (!outbox.empty() && outbox.front().id == stateId) {
                awaitingAck = false;
                outbox.pop_front();
            }
            if(strcasecmp(strings[2], "on") == 0 || strcasecmp(strings[2], "off") == 0 || strcasecmp(strings[2], "pulse") == 0) {
                RelayState confirmedRelayState = (strcasecmp(strings[2], "on") == 0 || strcasecmp(strings[2], "pulse") == 0) ? RelayState::ON : RelayState::OFF;
                if(confirmedRelayState == requestedRelayState) {
                    if(relayState != requestedRelayState) {
                        relayState = confirmedRelayState;
                        publishState();
                    }
                }
            } else if(strcasecmp(strings[2], "pwr") == 0 && index >= 4) {
                int power = atoi(strings[3]);
                receiverTxPower = power;
            } else if(strcasecmp(strings[2], "freq") == 0 && index >= 4) {
                unsigned int freq = atoi(strings[3]);
                receiverStatusFreqSec = freq;
            } else if(strcasecmp(strings[2], "status") == 0) {
                // no-op
            } else if(strcasecmp(strings[2], "wifi") == 0) {
                // no-op
            }
        } else if(strlen(strings[0]) == 1 && strings[0][0] == 'H') {
            if(index >= 3 && strcasecmp(strings[1], "pwr") == 0) {
                int power = atoi(strings[2]);
                if(power != receiverTxPower) {
                    char msg[16];
                    ++mStateId;
                    sprintf(msg, "PWR:%d", receiverTxPower);
                    sendMessage(msg);
                }
            }
        } else if(strlen(strings[0]) == 1 && strings[0][0] == 'S') {
            if(index >= 7) {
                int power = atoi(strings[1]);
                int rssi = atoi(strings[2]);
                int snr = atoi(strings[3]);
                bool state = atoi(strings[4]);
                bool pulse = atoi(strings[5]);
                int batt = atoi(strings[6]);
                int cstate = -1;
                int wifi = WIFI_DISABLED;
                if(index >= 9) {
                    cstate = atoi(strings[7]);
                    wifi = atoi(strings[8]);
                } else if(index >= 8) {
                    wifi = atoi(strings[7]);
                }
                publishReceiverStatus(power, rssi, snr, state, pulse, batt, cstate, wifi);
            }
        } else if(strlen(strings[0]) == 1 && strings[0][0] == 'D') {
            if(index >= 2) {
                DailyStats stats; memset(&stats, 0, sizeof(stats));
                if(parseDailyStats(strings[1], stats)) {
                    publishReceiverDailyStats(stats);
                }
            }
        }
    }
}

void Controller::OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size >= BUFFER_SIZE) {
        size = BUFFER_SIZE - 1;
    }
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    mLastMessage = (char *)rxpacket;
    mLastMessageSize = size;
    mLastRssi = rssi;
    mLastSnr = snr;
    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n", rxpacket, rssi, size);
    setIdle();
    processReceived(rxpacket);
}

void Controller::OnTxDone() {
    setIdle();
}

void Controller::OnTxTimeout() {
    setIdle();
}

static void checkReceiveData() {
  if(activityFlag) {
    activityFlag = false;
    String str;
    int state = radio.readData(str);
    if (state == RADIOLIB_ERR_NONE) {
      controllerInstance->OnRxDone((uint8_t*)str.c_str(), str.length(), radio.getRSSI(), radio.getSNR());
    } else if (state != RADIOLIB_ERR_NONE) {
      // ignore other errors
    }
  }
}

static void checkTransmittedData() {
  if(activityFlag) {
    activityFlag = false;
    radio.finishTransmit();
    controllerInstance->OnTxDone();
  }
}
