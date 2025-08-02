
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"

#include "config.h"
#include "display.h"
#include "device-config.h"
#include "settings.h"
#include "battery.h"
#include "controller.h"

// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

Controller *controllerInstance;
extern Battery battery;

void COnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    controllerInstance->OnRxDone(payload, size, rssi, snr);
}

void COnTxDone()
{
    controllerInstance->OnTxDone();
}
void COnTxTimeout()
{
    controllerInstance->OnTxTimeout();
}


Controller::Controller(Display &display) : mDisplay(display), mqttClient(espClient)
{
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

String relayStateToString(RelayState state)
{
  switch (state)
  {
  case RelayState::OFF:
    return "OFF";
  case RelayState::ON:
    return "ON";  
  }
  
  return "UNKNOWN";  
}

void Controller::updateDisplay()
{
    mDisplay.display.clearDisplay();
    mDisplay.display.setTextSize(1); // Draw 2X-scale text
    mDisplay.display.setTextColor(SSD1306_WHITE);
    mDisplay.display.setCursor(10, 0);
    mDisplay.display.printf("Controller: %s", relayStateToString(relayState));
    mDisplay.display.setTextSize(1); // Draw 2X-scale text

    mDisplay.display.setCursor(10, 22);
    if(requestedRelayState == RelayState::ON) {
        mDisplay.display.printf("Dur: %us", onTimeSec);
    } else {
        mDisplay.display.printf("Pending: %s", relayStateToString(requestedRelayState));
    }

    bool awaitingAck = relayState != requestedRelayState;
    const char *status = lora_idle ? "IDLE" : "TX";
    if (awaitingAck) {
        status = "WAIT ACK";
    }
    mDisplay.display.setCursor(10, 36);
    mDisplay.display.printf("Status: %s", status);
        
    mDisplay.display.setCursor(10, 50);
    mDisplay.display.printf("PW:%d RS:%d SR:%d", txPower, mLastRssi, mLastSnr);

    if (mHasBattery)
    {
        int batt = battery.getPercentage();
        bool chg = battery.isCharging();
        mDisplay.display.setCursor(10, 58);
        mDisplay.display.printf("BAT:%d%% %s", batt, chg ? "CHG" : " ");
    }
    mDisplay.display.display();
}

void Controller::publishState() {
    mqttClient.publish("pump_station/switch/state", relayStateToString(relayState).c_str(), true);
    updateDisplay();
}

void Controller::publishControllerStatus() {
    char payload[128];
    int batt = battery.getPercentage();
    snprintf(payload, sizeof(payload),
             "{\"power\":%d,\"rssi\":%d,\"snr\":%d,\"state\":\"%s\",\"mode\":\"%s\",\"battery\":%d}",
             txPower, mLastRssi, mLastSnr,
             relayStateToString(relayState).c_str(),
             pulseMode ? "pulse" : "normal", batt);
    mqttClient.publish("pump_station/status/controller", payload, true);
}

void Controller::publishReceiverStatus(int power, int rssi, int snr, bool relay, bool pulse, int battery) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"power\":%d,\"rssi\":%d,\"snr\":%d,\"state\":\"%s\",\"mode\":\"%s\",\"battery\":%d}",
             power, rssi, snr,
             relay ? "ON" : "OFF",
             pulse ? "pulse" : "normal", battery);
    mqttClient.publish("pump_station/status/receiver", payload, true);
}

void controllerMqttCallback(char *topic, byte *payload, unsigned int length)
{
  controllerInstance->mqttCallback(topic, payload, length);
}

void Controller::mqttCallback(char *topic, byte *payload, unsigned int length) {
    String cmd;

    for (unsigned int i = 0; i < length; i++) {
        cmd += (char)payload[i];
    }

    if(strcmp(topic, "pump_station/switch/set") == 0) {
        initialSetReceived = true;
        if (cmd.startsWith("ON")) {
            int idx = cmd.indexOf(":");
            unsigned int t = DEFAULT_ON_TIME_SEC;
            if(idx > 0) {
                t = cmd.substring(idx+1).toInt();
                if(t == 0) t = DEFAULT_ON_TIME_SEC;
            }
            heartbeatEnabled = true;
            setRelayState(true, t);
        } else if (cmd.startsWith("OFF")) {
            heartbeatEnabled = false;
            setRelayState(false);
        }
        publishState();
    } else if(strcmp(topic, "pump_station/switch/pulse") == 0) {
        unsigned int t = cmd.toInt();
        if(t == 0) t = DEFAULT_ON_TIME_SEC;
        pulseRelay(t);
        publishState();
    } else if(strcmp(topic, "pump_station/tx_power/controller/set") == 0) {
        int pwr = cmd.toInt();
        setTxPower(pwr);
    } else if(strcmp(topic, "pump_station/tx_power/receiver/set") == 0) {
        int pwr = cmd.toInt();
        if(pwr < MIN_TX_OUTPUT_POWER) pwr = MIN_TX_OUTPUT_POWER;
        char msg[16];
        ++mStateId;
        sprintf(msg, "PWR:%d", pwr);
        sendMessage(msg);
        receiverTxPower = pwr;
        Settings::setInt(KEY_RX_TX_POWER, receiverTxPower);
    } else if(strcmp(topic, "pump_station/status_freq/controller/set") == 0) {
        unsigned int f = cmd.toInt();
        if(f == 0) f = DEFAULT_STATUS_SEND_FREQ_SEC;
        setSendStatusFrequency(f);
    } else if(strcmp(topic, "pump_station/status_freq/receiver/set") == 0) {
        unsigned int f = cmd.toInt();
        if(f == 0) f = DEFAULT_STATUS_SEND_FREQ_SEC;
        char msg[16];
        ++mStateId;
        sprintf(msg, "FREQ:%u", f);
        sendMessage(msg);
        receiverStatusFreqSec = f;
        Settings::setInt(KEY_RX_STATUS_FREQ, receiverStatusFreqSec);
    } else if(strcmp(topic, "pump_station/switch/state") == 0) {
        initialStateReceived = true;
        retainedStateOn = cmd.startsWith("ON");
    }
}

#define BASE_TOPIC "esp32-lora-controller"

#define MQTT_DEVICE \
    "\"device\": {" \
      "\"identifiers\": [\"esp32-lora-controller\"]," \
      "\"name\": \"ESP32 LoRa Controller\"," \
      "\"manufacturer\": \"HumpTech\"," \
      "\"model\": \"ESP32-LORA-PC\"," \
      "\"sw_version\": \"1.0\"" \
    "}"

void Controller::sendDiscovery() {
    const char *switchTopic = "homeassistant/switch/pump_station/config";
    const char *switchPayload = "{\"name\":\"Pump\",\"command_topic\":\"pump_station/switch/set\",\"state_topic\":\"pump_station/switch/state\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"unique_id\":\"pump_station\",\"device\":{\"identifiers\":[\"pump_station\"],\"name\":\"Pump Controller\",\"model\":\"Heltec WiFi LoRa 32 V3\",\"manufacturer\":\"Heltec\"}}";
    mqttClient.publish(switchTopic, switchPayload, true);

    const char *pulseTopic = "homeassistant/number/pump_station_pulse/config";
    const char *pulsePayload = "{\"name\":\"Pump Pulse\",\"command_topic\":\"pump_station/switch/pulse\",\"min\":1,\"max\":3600,\"step\":1,\"unit_of_measurement\":\"s\",\"unique_id\":\"pump_station_pulse\",\"device\":{\"identifiers\":[\"pump_station\"],\"name\":\"Pump Controller\",\"model\":\"Heltec WiFi LoRa 32 V3\",\"manufacturer\":\"Heltec\"}}";
    mqttClient.publish(pulseTopic, pulsePayload, true);

    const char *ctrlPowerTopic = "homeassistant/number/pump_station_ctrl_power/config";
    const char *ctrlPowerPayload = "{\"name\":\"Controller Tx Power\",\"command_topic\":\"pump_station/tx_power/controller/set\",\"min\":0,\"max\":22,\"step\":1,\"unit_of_measurement\":\"dBm\",\"unique_id\":\"pump_station_ctrl_power\",\"device\":{\"identifiers\":[\"pump_station\"],\"name\":\"Pump Controller\",\"model\":\"Heltec WiFi LoRa 32 V3\",\"manufacturer\":\"Heltec\"}}";
    mqttClient.publish(ctrlPowerTopic, ctrlPowerPayload, true);

    const char *rxPowerTopic = "homeassistant/number/pump_station_rx_power/config";
    const char *rxPowerPayload = "{\"name\":\"Receiver Tx Power\",\"command_topic\":\"pump_station/tx_power/receiver/set\",\"min\":0,\"max\":22,\"step\":1,\"unit_of_measurement\":\"dBm\",\"unique_id\":\"pump_station_rx_power\",\"device\":{\"identifiers\":[\"pump_station\"],\"name\":\"Pump Controller\",\"model\":\"Heltec WiFi LoRa 32 V3\",\"manufacturer\":\"Heltec\"}}";
    mqttClient.publish(rxPowerTopic, rxPowerPayload, true);

    const char *ctrlStatusTopic = "homeassistant/sensor/pump_station_ctrl_status/config";
    const char *ctrlStatusPayload = "{\"name\":\"Controller Status\",\"state_topic\":\"pump_station/status/controller\",\"value_template\":\"{{ value_json.state }}\",\"json_attributes_topic\":\"pump_station/status/controller\",\"unique_id\":\"pump_station_ctrl_status\",\"device\":{\"identifiers\":[\"pump_station\"],\"name\":\"Pump Controller\",\"model\":\"Heltec WiFi LoRa 32 V3\",\"manufacturer\":\"Heltec\"}}";
    mqttClient.publish(ctrlStatusTopic, ctrlStatusPayload, true);

    const char *rxStatusTopic = "homeassistant/sensor/pump_station_rx_status/config";
    const char *rxStatusPayload = "{\"name\":\"Receiver Status\",\"state_topic\":\"pump_station/status/receiver\",\"value_template\":\"{{ value_json.state }}\",\"json_attributes_topic\":\"pump_station/status/receiver\",\"unique_id\":\"pump_station_rx_status\",\"device\":{\"identifiers\":[\"pump_station\"],\"name\":\"Pump Controller\",\"model\":\"Heltec WiFi LoRa 32 V3\",\"manufacturer\":\"Heltec\"}}";
    mqttClient.publish(rxStatusTopic, rxStatusPayload, true);
}



// void Controller::sendDiscovery()
// {

//   char topic[128];
//   char payload[2048];

//   const char *THING_NAME = "pump_station";

//   snprintf(topic, sizeof(topic), "homeassistant/switch/%s/config", THING_NAME);
//   snprintf(payload, sizeof(payload),
//            "{"
//            "%s,"
//            "\"name\": \"%s\","
//            "\"unique_id\": \"%s\","
//            "\"command_topic\": \"%s/switch/set\","
//            "\"state_topic\": \"%s/switch/state\","
//         //    "\"availability_topic\": \"%s/status\","
//            "\"payload_on\": \"ON\","
//            "\"payload_off\": \"OFF\""
//            "}",
//            MQTT_DEVICE,
//            THING_NAME,           
//            THING_NAME,
//            BASE_TOPIC,
//            BASE_TOPIC,
//            BASE_TOPIC);

//            mqttClient.publish(topic, payload, true);
// }

void Controller::ensureMqtt() {
    while (!mqttClient.connected()) {
        mqttClient.connect("pump_station", MQTT_USER, MQTT_PASS);
        if (!mqttClient.connected()) {
            delay(500);
        }
    }
    mqttClient.subscribe("pump_station/switch/set");
    mqttClient.subscribe("pump_station/switch/pulse");
    mqttClient.subscribe("pump_station/tx_power/controller/set");
    mqttClient.subscribe("pump_station/tx_power/receiver/set");
    mqttClient.subscribe("pump_station/status_freq/controller/set");
    mqttClient.subscribe("pump_station/status_freq/receiver/set");
    mqttClient.subscribe("pump_station/switch/state");

    // Process any retained messages (such as the last set command or state)
    initialSetReceived = false;
    initialStateReceived = false;
    retainedStateOn = false;
    unsigned long start = millis();
    while (millis() - start < 2000 && !initialSetReceived && !initialStateReceived) {
        mqttClient.loop();
        delay(10);
    }

    mqttClient.unsubscribe("pump_station/switch/state");

    if(!initialSetReceived && initialStateReceived && retainedStateOn) {
        // No retained command but last known state was ON
        heartbeatEnabled = true;
        setRelayState(true);
    }
}

void Controller::setup() {
    Settings::begin();
    txPower = Settings::getInt(KEY_CTRL_TX_POWER, TX_OUTPUT_POWER);
    receiverTxPower = Settings::getInt(KEY_RX_TX_POWER, TX_OUTPUT_POWER);
    statusSendFreqSec = Settings::getInt(KEY_CTRL_STATUS_FREQ, DEFAULT_STATUS_SEND_FREQ_SEC);
    receiverStatusFreqSec = Settings::getInt(KEY_RX_STATUS_FREQ, DEFAULT_STATUS_SEND_FREQ_SEC);

    Wire.begin(17, 18);

    Serial.println("Init Wifi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    Serial.println("Init Wifi - complete");

    //
    // Setup radio.
    //

    Mcu.begin();
    txNumber=0;

    RadioEvents.TxDone = ::COnTxDone;
    RadioEvents.TxTimeout = ::COnTxTimeout;
    RadioEvents.RxDone = ::COnRxDone;
    Serial.println("Init Radio.");

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, txPower, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );


    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    // Begin in continuous receive mode so incoming packets are processed
    Radio.Rx( 0 );

    Serial.println("Init Radio - complete");


    Serial.println("Init MQTT");
    mqttClient.setServer(MQTT_SERVER, 1883);
    mqttClient.setCallback(controllerMqttCallback);
    

    ensureMqtt();
    sendDiscovery();
    publishState();
    {
        char msg[16];
        ++mStateId;
        sprintf(msg, "FREQ:%u", receiverStatusFreqSec);
        sendMessage(msg);
    }
    sendMessage("STATUS");

    Serial.println("Init MQTT - complete");
}

void Controller::sendMessage(const char *msg)
{
        //snprintf(txpacket, sizeof(txpacket), msg.c_str());
 		sprintf(txpacket,"C:%ld:%s",mStateId, msg);  //start a package
   
		Serial.printf("Sending packet \"%s\", length %d\r\n",txpacket, strlen(txpacket));

        lora_idle = false;
		Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
       
        Serial.println("packet sent.");   
}

void Controller::sendAckReceived(uint16_t stateId)
{
        sprintf(txpacket, "A:%u", stateId);

        Serial.printf("Sending ack receipt \"%s\", length %d\r\n", txpacket, strlen(txpacket));

        lora_idle = false;
        Radio.Send((uint8_t *)txpacket, strlen(txpacket));

        Serial.println("ack receipt sent.");
}

void Controller::setTxPower(int power)
{
    if(power < MIN_TX_OUTPUT_POWER)
        power = MIN_TX_OUTPUT_POWER;
    if(power > 22)
        power = 22;

    txPower = power;
    Radio.SetTxConfig( MODEM_LORA, txPower, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Settings::setInt(KEY_CTRL_TX_POWER, txPower);

    updateDisplay();
}

void Controller::setSendStatusFrequency(unsigned int freq)
{
    statusSendFreqSec = freq;
    Settings::setInt(KEY_CTRL_STATUS_FREQ, statusSendFreqSec);
}


void Controller::setRelayState(bool pumpOn, unsigned int onTime, bool pulse)
{
  ++mStateId;
  requestedRelayState = pumpOn ? RelayState::ON : RelayState::OFF;
  if(pumpOn) {
      onTimeSec = onTime;
  }
  pulseMode = pumpOn && pulse;
  relayState = RelayState::UNKNOWN;
  lastCommandTime = millis();
  if(pumpOn) {
      offRetriesRemaining = 0;
  } else {
      offRetriesRemaining = OFF_RETRY_COUNT;
  }

  char msg[32];
  if(pumpOn) {
      if(pulse) {
          sprintf(msg, "PULSE:%u", onTimeSec);
      } else {
          sprintf(msg, "ON:%u", onTimeSec);
          nextOnSend = millis();
      }
  } else {
      sprintf(msg, "OFF");
  }

  sendMessage(msg);
}

void Controller::pulseRelay(unsigned int onTime)
{
    heartbeatEnabled = false;
    autoOffTime = millis() + (unsigned long)onTime * 1000UL;
    setRelayState(true, onTime, true);
}


void Controller::loop() {
    if(autoOffTime && millis() > autoOffTime) {
        autoOffTime = 0;
        heartbeatEnabled = false;
        setRelayState(false);
    }

    // When ON is requested continue to periodically resend the ON command even
    // if the receiver is not acknowledging.
    if(heartbeatEnabled && lora_idle && requestedRelayState == RelayState::ON) {
        unsigned long interval = (onTimeSec * 1000UL) / 20;
        unsigned long minimumInterval = 5000;
        unsigned long maximumInterval = 30000;
        
        if(interval < minimumInterval)
        {
            interval = minimumInterval;
        }
        
        if(millis() - nextOnSend >= interval) {
            char msg[32];
            sprintf(msg, "ON:%u", onTimeSec);
            sendMessage(msg);
            nextOnSend = millis();
        }
    }

    // Detect loss of communication while the relay should be ON
    if(requestedRelayState == RelayState::ON && relayState == RelayState::ON &&
       lastContactTime && millis() - lastContactTime > COMMUNICATION_TIMEOUT_MS) {
        relayState = RelayState::UNKNOWN;
        publishState();
    }

    // Retry OFF commands a few times before giving up
    if(requestedRelayState == RelayState::OFF && relayState != RelayState::OFF) {
        if(offRetriesRemaining > 0 && millis() - lastCommandTime >= OFF_RETRY_INTERVAL_MS && lora_idle) {
            sendMessage("OFF");
            lastCommandTime = millis();
            offRetriesRemaining--;
        } else if(offRetriesRemaining == 0 && millis() - lastCommandTime >= OFF_RETRY_INTERVAL_MS) {
            if(relayState != RelayState::UNKNOWN) {
                relayState = RelayState::UNKNOWN;
                publishState();
            }
        }
    }
    


	// if(lora_idle == true)
	// {
    //     delay(1000);
	// 	txNumber += 0.01;
	// 	sprintf(txpacket,"Hello world number %0.2f",txNumber);  //start a package
   
	// 	Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

	// 	Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	

    //     Serial.println("packet sent.");

    //     lora_idle = false;
	// }
    

    if(lora_idle)
    {
      Radio.Rx( 0 );
    }


    // if(lora_idle == false)
    {
      Radio.IrqProcess( );
    }
    
    if (!mqttClient.connected()) {
        ensureMqtt();
    }
    if(millis() - lastStatusPublish > statusSendFreqSec * 1000UL) {
        publishControllerStatus();
        lastStatusPublish = millis();
    }
    mqttClient.loop();
}



void Controller::processReceived(char *rxpacket)
{
    char *strings[10];
    char *ptr = NULL;
    int index = 0;

    lastContactTime = millis();
    ptr = strtok(rxpacket, ":;"); // takes a list of delimiters
    while (ptr != NULL)
    {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, ":;"); // takes a list of delimiters
    }

    
    if (index >= 3)
    {
        if (strlen(strings[0]) == 1 && strings[0][0] == 'R')
        {
            uint16_t stateId = atoi(strings[1]);

            if (stateId != mStateId)
            {
                Serial.printf("Skipping - likely an old ack expected stateid: %d, got %d.\n", mStateId, stateId);
                return;
            }

            if(strcasecmp(strings[2], "on") == 0 || strcasecmp(strings[2], "off") == 0 || strcasecmp(strings[2], "pulse") == 0)
            {
                RelayState confirmedRelayState = (strcasecmp(strings[2], "on") == 0 || strcasecmp(strings[2], "pulse") == 0) ? RelayState::ON : RelayState::OFF;

                if(confirmedRelayState == requestedRelayState)
                {
                    if(relayState != requestedRelayState)
                    {
                        Serial.printf("Relay state confirmed - publishing new state: %s\n", confirmedRelayState == RelayState::ON ? "ON" : "OFF");
                        relayState = confirmedRelayState;
                        ++mStateId; // do not increment for heartbeat
                        offRetriesRemaining = 0;
                        publishState();
                    }
                    else
                    {
                        Serial.println("Heartbeat ack received.");
                    }
                }
                else
                {
                    Serial.printf("Skipping - ack state mismatch expected: %s got: %s\n", relayStateToString(requestedRelayState).c_str(), strings[2]);
                }
            }
            else if(strcasecmp(strings[2], "pwr") == 0 && index >= 4)
            {
                int power = atoi(strings[3]);
                Serial.printf("Receiver power acknowledged: %d dBm\n", power);
                receiverTxPower = power;
                ++mStateId;
            }
            else if(strcasecmp(strings[2], "freq") == 0 && index >= 4)
            {
                unsigned int freq = atoi(strings[3]);
                Serial.printf("Receiver status frequency acknowledged: %u sec\n", freq);
                receiverStatusFreqSec = freq;
                ++mStateId;
            }
            else if(strcasecmp(strings[2], "status") == 0)
            {
                Serial.println("Status command acknowledged");
                ++mStateId;
            }
            else
            {
                Serial.printf("Unknown R ack for command: %s\n", strings[2]);
                ++mStateId;
            }

            sendAckReceived(stateId);
        }
        else if(strlen(strings[0]) == 1 && strings[0][0] == 'H')
        {
            if(index >= 3 && strcasecmp(strings[1], "pwr") == 0)
            {
                int power = atoi(strings[2]);
                if(power != receiverTxPower)
                {
                    Serial.printf("Receiver tx power is %d but should be %d - sending config messasge to receiver.\n", power, receiverTxPower);
                    char msg[16];
                    ++mStateId;
                    sprintf(msg, "PWR:%d", receiverTxPower);
                    sendMessage(msg);
                }
                else
                {
                    Serial.printf("Receiver configuration matches expected configuration.");
                }
            }
        }
        else if(strlen(strings[0]) == 1 && strings[0][0] == 'S')
        {
            if(index >= 7) {
                int power = atoi(strings[1]);
                int rssi = atoi(strings[2]);
                int snr = atoi(strings[3]);
                bool state = atoi(strings[4]);
                bool pulse = atoi(strings[5]);
                int battery = atoi(strings[6]);
                publishReceiverStatus(power, rssi, snr, state, pulse, battery);
            }
        }
    }
}

void Controller::OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    if (size >= BUFFER_SIZE)
    {
        //
        // We can only process packets up to BUFFER_SIZE - 1 or we'll buffer overflow.
        // Just truncate - this allows for forward compatability with larger messages.
        //
        size = BUFFER_SIZE - 1;
    }

    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Radio.Sleep();

    mLastRssi = rssi;
    mLastSnr = snr;

    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n", rxpacket, rssi, size);
    Serial.println("wait to send next packet");

    lora_idle = true;
    // Resume listening for the next packet
    Radio.Rx( 0 );

    processReceived((char *) rxpacket);
}

void Controller::OnTxDone( void )
{
        Serial.println("TX done......");
        lora_idle = true;
        // After transmitting, return to receive mode
        Radio.Rx( 0 );
    //state=STATE_RX;
}

void Controller::OnTxTimeout( void )
{
    Radio.Sleep();
    Serial.println("TX Timeout......");
    lora_idle = true;
    // Return to receive mode after a timeout
    Radio.Rx( 0 );
     //state=STATE_RX;
}

