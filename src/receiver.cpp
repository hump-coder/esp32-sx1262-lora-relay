#include <WiFi.h>
#include <SPI.h>
#include "Arduino.h"

#include "config.h"
#include "config.h"
#include "settings.h"
#include "receiver.h"

#include <RadioLib.h>
#include "BatteryMonitor.h"
#include <ArduinoOTA.h>
#include <math.h>

#define RELAY_PIN 1
#define DEVICE_NAME "dam-pump-receiver-c3-sx1262"


// // XL1262 LoRa pin mapping to ESP32-C3
// #define PIN_LORA_MOSI 23   // purple gpio 8
// #define PIN_LORA_MISO 19   // orange gpio 7
// #define PIN_LORA_SCK 18    // white gpio 6
// #define PIN_LORA_CS 5    // blue (NSS) gpio 16
// #define PIN_LORA_RESET 14 // green gpio 15
// #define PIN_LORA_DIO1 26   // yellow gpio  4
// #define PIN_LORA_BUSY 27    // brown gpio  2

// XL1262 LoRa pin mapping to ESP32-C3
#define PIN_LORA_MOSI 6   
#define PIN_LORA_MISO 5   
#define PIN_LORA_SCK 10    
#define PIN_LORA_CS 7    // blue (NSS)
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

// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// initialize the RadioLib SX1262 instance using ESP32 hardware SPI
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);

Receiver *instance;
// extern Battery battery;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle = true;


//void OnTxDone(void);
//void OnTxTimeout(void);
//void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

// // flag to indicate that a packet was received
// volatile bool receivedFlag = false;

// // this function is called when a complete packet
// // is received by the module
// // IMPORTANT: this function MUST be 'void' type
// //            and MUST NOT have any arguments!
// // #if defined(ESP8266) || defined(ESP32)
// //   ICACHE_RAM_ATTR
// // #endif
// ICACHE_RAM_ATTR
// void setReceivedFlag(void) {
//   // we got a packet, set the flag
//   receivedFlag = true;
// }


// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool activityFlag = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
// #if defined(ESP8266) || defined(ESP32)
//   ICACHE_RAM_ATTR
// #endif
ICACHE_RAM_ATTR
void setFlag(void) {
  // we sent a packet, set the flag
  activityFlag = true;
}



Receiver::Receiver()
{
    ::instance = this;

    mLastMessage = "";
    mLastMessageSize = 0;
    mLastRssi = 0;
    mLastSnr = 0;
    mRelayState = false;
    mPulseMode = false;
    lastStatusSend = 0;
    pendingDailyStats = false;
    otaEnabled = false;
}
void Receiver::updateDisplay()
{

}

// void Receiver::updateDisplay()
// {
//     int x = 5;
//     int step = 12;
//     int y = 0;
//     mDisplay.display.clearDisplay();
//     mDisplay.display.setTextSize(1); // Draw 2X-scale text
//     mDisplay.display.setTextColor(SSD1306_WHITE);
//     mDisplay.display.setCursor(x, y);
//     mDisplay.display.printf("Receiver: ");
//     mDisplay.display.println(mRelayState ? "ON" : "OFF");
//     mDisplay.display.setTextSize(1); // Draw 2X-scale text

//     y+=step;
//     mDisplay.display.setCursor(x, y);
    
//     y+=step;
//     mDisplay.display.setCursor(x, y);
//     if(mRelayState) {
//         unsigned long remaining = 0;
//         if(offTime > millis()) {
//             remaining = (offTime - millis()) / 1000;
//         }
//         mDisplay.display.printf("Time: %lus", remaining);
//     } else {
//     }

//     y+=step;
//     mDisplay.display.setCursor(x, y);
//     mDisplay.display.printf("PW:%d RS:%d SR:%d", txPower, mLastRssi, mLastSnr);

//     y+=step;
//     mDisplay.display.setCursor(x, y);

//     float voltage = mBattery.getVoltage();
//     int batt = mBattery.getPercentage();
//     bool chg = mBattery.isCharging();

//     mDisplay.display.printf("BAT: %.2fv  %d%% %s", voltage, batt, chg ? "CHG" : " ");

//     mDisplay.display.display();
// }


void Receiver::setup()
{
    Settings::begin();
    txPower = Settings::getInt(KEY_RX_TX_POWER, TX_OUTPUT_POWER);
    statusSendFreqSec = Settings::getInt(KEY_RX_STATUS_FREQ, DEFAULT_RECEIVER_STATUS_SEND_FREQ_SEC);

    Serial.println("Setting up");
    Serial.begin(115200);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);
   
    // Mcu.begin();

    //    if(firstrun)
    //    {
    // LoRaWAN.displayMcuInit();

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    //   display.display.display();
    //   delay(1000); // Pause for 2 seconds

    // Clear the buffer
    //     display.clearDisplay();

    //   firstrun = false;
    //  }

    // Draw a single pixel in white
    // display.drawPixel(10, 10, SSD1306_WHITE);

    // testscrolltext();
    //  Show the display buffer on the screen. You MUST call display() after
    //  drawing commands to make them visible on screen!
    // display.display();
    // delay(2000);

    txNumber = 0;

    // RadioEvents.TxDone = ::OnTxDone;
    // RadioEvents.TxTimeout = ::OnTxTimeout;
    // RadioEvents.RxDone = ::OnRxDone;
    // Serial.println("Init Radio.");

    // Radio.Init(&RadioEvents);

    Serial.println("Init Radio.");

    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
        {
            delay(10);
        }
    }

    // set the function that will be called
    // when new packet is received
    //radio.setPacketReceivedAction(setReceivedFlag);
    radio.setDio1Action(setFlag);
    // set the function that will be called
    // when packet transmission is finished
    // radio.setPacketSentAction(setTransmittedFlag);

    // apply requested configuration
    radio.setFrequency(RF_FREQUENCY / 1000000.0);
    radio.setBandwidth(LORA_BANDWIDTH == 0 ? 125.0 : (LORA_BANDWIDTH == 1 ? 250.0 : 500.0));
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setCodingRate(LORA_CODINGRATE + 4);
    radio.setOutputPower(TX_OUTPUT_POWER);
    radio.setPreambleLength(LORA_PREAMBLE_LENGTH);
    radio.invertIQ(LORA_IQ_INVERSION_ON);
    radio.setSyncWord(LORA_SYNC_WORD);
    if (LORA_FIX_LENGTH_PAYLOAD_ON)
    {
        radio.implicitHeader(LORA_SYMBOL_TIMEOUT);
    }
    else
    {
        radio.explicitHeader();
    }

    setIdle();


    Serial.println("Init Radio - complete");

    // Notify controller that we're online
    delay(200);
    sendHello();
}

void Receiver::setIdle()
{
    // delay(100);
    radio.sleep();

    Serial.println("Radio - attempting to enter read mode.");
    //
    // Begin in continuous receive mode so incoming packets are processed
    //
    //    Radio.Rx(0);

    delay(100);

    int attempts = 10;

    int16_t state = RADIOLIB_ERR_NONE;
    while (--attempts >= 0 && (state = radio.startReceive()) != RADIOLIB_ERR_NONE)
    {  
        Serial.printf("Failed to startReceive(), code %d, remainig attempts: %d\n", state, attempts);        

        delay(100);
    }

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("Radio - in read mode: waiting for incoming messages");

        // Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed to enter read more, code "));
        Serial.println(state);
        //  while (true) { delay(10); }
    }

    //
    // We're idle if we're waiting.
    //
    lora_idle = true;
}

void checkReceiveData() {
  // check if the flag is set
  if(activityFlag) {
    // reset flag
    activityFlag = false;
   // Serial.println(F("[SX1262] Received flag set!"));

    // you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
    //   Serial.println(F("[SX1262] Received packet!"));

    //   // print data of the packet
    //   Serial.print(F("[SX1262] Data:\t\t"));
    //   Serial.println(str);

    //   // print RSSI (Received Signal Strength Indicator)
    //   Serial.print(F("[SX1262] RSSI:\t\t"));
    //   Serial.print(radio.getRSSI());
    //   Serial.println(F(" dBm"));

    //   // print SNR (Signal-to-Noise Ratio)
    //   Serial.print(F("[SX1262] SNR:\t\t"));
    //   Serial.print(radio.getSNR());
    //   Serial.println(F(" dB"));

    //   // print frequency error
    //   Serial.print(F("[SX1262] Frequency error:\t"));
    //   Serial.print(radio.getFrequencyError());
    //   Serial.println(F(" Hz"));

      instance->OnRxDone(str, radio.getRSSI(),radio.getSNR());

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("Receive: CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("Receive: failed, code "));
      Serial.println(state);

    }
  }
}

void checkTransmittedData() {
  // check if the previous transmission finished
  if(activityFlag) {
    // reset flag
    activityFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
    //   Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio.finishTransmit();
    instance->OnTxDone();
  }
}

void Receiver::send(char *packet, size_t len)
{
    lora_idle = false;
    // Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    // you can transmit C-string or Arduino string up to
    // 256 characters long
    Serial.printf("Sending \"%s\", length %d\r\n", packet, len);
    transmissionState = radio.startTransmit((uint8_t *)packet, strlen(packet));
}

void Receiver::sendHello()
{
    sprintf(txpacket, "H:PWR:%d", txPower);
    Serial.printf("Sending hello \"%s\", length %d\r\n", txpacket, strlen(txpacket));
    
    send(txpacket, strlen(txpacket));

    Serial.println("hello sent.");
}

void Receiver::sendStatus()
{
    updateStatusCache();
    sprintf(txpacket, "S:%d:%d:%d:%d:%d:%.1f:%d:%d:%lu", txPower, mLastRssi, mLastSnr, mRelayState ? 1 : 0, mPulseMode ? 1 : 0, lastBatteryPct, static_cast<int>(lastChargeState), lastWifiState, millis() / 1000UL);

    pendingDailyStats = true;
    send(txpacket, strlen(txpacket));
    Serial.printf("Sent status \"%s\", length %d\r\n", txpacket, strlen(txpacket));

    
    // lora_idle = false;
    // // Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    // transmissionState = radio.startTransmit((uint8_t *)txpacket, strlen(txpacket));

    // Serial.println("status sent.");
}

int Receiver::getWifiState()
{
    // If the WiFi subsystem is not started, report disabled regardless of status
    if (WiFi.getMode() == WIFI_MODE_NULL) {
        return WIFI_DISABLED;
    }

    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
        return WIFI_CONNECTED;
    }
    if (st == WL_DISCONNECTED || st == WL_IDLE_STATUS) {
        return WIFI_CONNECTING;
    }
    if (st != WL_NO_SHIELD) {
        return WIFI_ERROR;
    }
    return WIFI_DISABLED;
}

void Receiver::updateStatusCache()
{
    lastBatteryPct = battery.getFilteredPercentage();
    lastChargeState = battery.getChargeState();
    lastWifiState = getWifiState();
}

void Receiver::sendDailyStats()
{
    String stats = battery.getDailyStatsCompact();
    snprintf(txpacket, sizeof(txpacket), "D:%s", stats.c_str());
    send(txpacket, strlen(txpacket));
    Serial.printf("Sent daily stats \"%s\", length %d\r\n", txpacket, strlen(txpacket));
}

unsigned long lastServiceUpdate = 0;
unsigned long serviceUpdateFrequency = 1000;



void Receiver::loop()
{
    if(otaEnabled) {
        ArduinoOTA.handle();
    }

    if(mRelayState && offTime && millis() > offTime) {
        setRelayState(false);
    }

    if (millis() - lastServiceUpdate > serviceUpdateFrequency)
    {
        float b = battery.getFilteredPercentage();
        ChargeState cs = battery.getChargeState();
        int wifi = getWifiState();
        // Only trigger an immediate status send if battery percent changed
        // significantly to avoid oscillations causing message spam.
        if (fabs(b - lastBatteryPct) >= BATTERY_PERCENT_CHANGE_THRESHOLD ||
            cs != lastChargeState || wifi != lastWifiState)
        {
            if (lora_idle)
            {
                sendStatus();
                lastStatusSend = millis();
            }
        }
        updateDisplay();
        lastServiceUpdate = millis();
    }
    
    if(lora_idle && millis() - lastStatusSend > statusSendFreqSec * 1000UL)
    {
        sendStatus();
        lastStatusSend = millis();
    }

    if (lora_idle)
    {
        checkReceiveData();
    }
    else
    {
        checkTransmittedData();
    }
}

void Receiver::sendAck(uint16_t id, const char *status)
{
    snprintf(txpacket, sizeof(txpacket), "A:%u:%s", id, status);
    send(txpacket, strlen(txpacket));
}

void Receiver::actuateRelay(bool state)
{
    digitalWrite(RELAY_PIN, state ? LOW : HIGH);
}

void Receiver::setRelayState(bool newRelayState)
{
    mRelayState = newRelayState;
    if(newRelayState) {
        offTime = millis() + (unsigned long)onTimeSec * 1000UL;
    } else {
        offTime = 0;
    }
    actuateRelay(mRelayState);
}

void Receiver::setTxPower(int power)
{
    if(power < MIN_TX_OUTPUT_POWER)
        power = MIN_TX_OUTPUT_POWER;
    if(power > 22)
        power = 22;

    txPower = power;

    radio.setOutputPower(txPower);

    Settings::setInt(KEY_RX_TX_POWER, txPower);
    updateDisplay();
}

void Receiver::setSendStatusFrequency(unsigned int freq)
{
    statusSendFreqSec = freq;
    Settings::setInt(KEY_RX_STATUS_FREQ, statusSendFreqSec);
}

void Receiver::connectWifi(const char *ssid, const char *pass)
{
    Serial.println("Connecting to WiFi for OTA");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected, starting OTA");
        ArduinoOTA.setHostname(DEVICE_NAME);
        ArduinoOTA.begin();
        otaEnabled = true;
    } else {
        Serial.println("WiFi connection failed");
    }
}

void Receiver::disableWifi()
{
    Serial.println("Disabling WiFi and OTA");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_MODE_NULL);
    if (otaEnabled)
    {
        ArduinoOTA.end();
        otaEnabled = false;
    }
}

void Receiver::processReceived(char *rxpacket)
{
    char *strings[10];
    char *ptr = NULL;
    int index = 0;

    ptr = strtok(rxpacket, ":;"); // takes a list of delimiters
    while (ptr != NULL)
    {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, ":;"); // takes a list of delimiters
    }

    if (index >= 3 && strlen(strings[0]) == 1 && strings[0][0] == 'C')
    {
        uint16_t stateId = atoi(strings[1]);
        const char *resp = NULL;

        if(strcasecmp(strings[2], "reboot") == 0) {
            delay(200);
            sendAck(stateId, "reboot");
            delay(100);
            ESP.restart();
            return;
        } else if(strcasecmp(strings[2], "sync") == 0) {
            resp = "sync";
        } else if(strcasecmp(strings[2], "status") == 0) {
            sendAck(stateId, "status");
            pendingStatus = true;
            return;
        } else if(strcasecmp(strings[2], "freq") == 0 && index >= 4) {
            int freq = atoi(strings[3]);
            setSendStatusFrequency(freq);
            resp = "freq";
        } else if(strcasecmp(strings[2], "pwr") == 0 && index >= 4) {
            int power = atoi(strings[3]);
            setTxPower(power);
            resp = "pwr";
        } else if(strcasecmp(strings[2], "wifi") == 0) {
            if (index >= 4 && strcasecmp(strings[3], "off") == 0) {
                disableWifi();
            } else {
                const char *ssid = (index >= 4) ? strings[3] : WIFI_SSID;
                const char *pass = (index >= 5) ? strings[4] : WIFI_PASS;
                connectWifi(ssid, pass);
            }
            resp = "wifi";
        } else {
            bool newRelayState = false;
            if(strcasecmp(strings[2], "on") == 0 || strcasecmp(strings[2], "pulse") == 0) {
                newRelayState = true;
            }
            if(newRelayState && index >= 4) {
                onTimeSec = atoi(strings[3]);
            } else if(newRelayState) {
                onTimeSec = 0;
            }
            mPulseMode = strcasecmp(strings[2], "pulse") == 0;
            setRelayState(newRelayState);
            resp = newRelayState ? (mPulseMode ? "pulse" : "on") : "off";
        }
        delay(200);
        sendAck(stateId, resp ? resp : "ok");
    }
}

void Receiver::OnRxDone(String payload, int16_t rssi, int8_t snr)
{
    size_t size = payload.length();

    if (size >= BUFFER_SIZE)
    {
        //
        // We can only process packets up to BUFFER_SIZE - 1 or we'll buffer overflow.
        // Just truncate - this allows for forward compatability with larger messages.
        //
        size = BUFFER_SIZE - 1;
    }

    memcpy(rxpacket, payload.c_str(), size);
    rxpacket[size] = '\0';

   
    mLastMessage = (char *)rxpacket;
    mLastMessageSize = size;
    mLastRssi = rssi;
    mLastSnr = snr;

    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n", rxpacket, rssi, size);
 
    // radio.sleep();
    setIdle();

    processReceived(rxpacket);
}

void Receiver::OnTxDone(void)
{
    // radio.sleep();
    Serial.println("TX done......");
    if (pendingStatus) {
        pendingStatus = false;
        sendStatus();
    } else if (pendingDailyStats) {
        pendingDailyStats = false;
        sendDailyStats();
    } else {
        setIdle();
    }
}

void Receiver::OnTxTimeout()
{
    // radio.sleep();
    Serial.println("TX Timeout......");
    setIdle();
    // lora_idle = true;
    // // Return to receive mode after a timeout
    // Radio.Rx( 0 );
    // state=STATE_RX;
}
