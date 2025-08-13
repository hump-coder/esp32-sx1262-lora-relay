#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <ArduinoOTA.h>

#include "config.h"
#include "display.h"
#include "device-config.h"
#include "settings.h"
#include "receiver.h"
#include "battery.h"

// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

Receiver *instance;
// extern Battery battery;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle = true;

static RadioEvents_t RadioEvents;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

Receiver::Receiver(Display &display, Battery &battery, bool enableWifi) : mDisplay(display), mBattery(battery), mWifiEnabled(enableWifi)
{
    ::instance = this;

    mLastMessage = "";
    mLastMessageSize = 0;
    mLastRssi = 0;
    mLastSnr = 0;
    mRelayState = false;
    mPulseMode = false;
    lastStatusSend = 0;
    otaEnabled = false;
}

void Receiver::updateDisplay()
{
    int x = 5;
    int step = 12;
    int y = 0;
    mDisplay.display.clearDisplay();
    mDisplay.display.setTextSize(1); // Draw 2X-scale text
    mDisplay.display.setTextColor(SSD1306_WHITE);
    mDisplay.display.setCursor(x, y);
    mDisplay.display.printf("Receiver: ");
    mDisplay.display.println(mRelayState ? "ON" : "OFF");
    mDisplay.display.setTextSize(1); // Draw 2X-scale text

    y+=step;
    mDisplay.display.setCursor(x, y);
    unsigned long remaining = 0;
    if(mRelayState && offTime > millis()) {
        remaining = (offTime - millis()) / 1000;
    }
    mDisplay.display.setCursor(x, y + step);
    if(mRelayState) {
        mDisplay.display.printf("Time: %lus", remaining);
    }

    y += step * 2;
    mDisplay.display.setCursor(x, y);
    mDisplay.display.printf("PW:%d RS:%d SR:%d", txPower, mLastRssi, mLastSnr);

    y+=step;
    mDisplay.display.setCursor(x, y);

    float voltage = mBattery.getVoltage();
    float batt = mBattery.getPercentage();
    bool chg = mBattery.isCharging();

    mDisplay.display.printf("BAT: %.2fv  %.1f%% %s", voltage, batt, chg ? "CHG" : " ");

    mDisplay.display.display();
}

void Receiver::setup()
{
    Settings::begin();
    txPower = Settings::getInt(KEY_RX_TX_POWER, TX_OUTPUT_POWER);
    statusSendFreqSec = Settings::getInt(KEY_RX_STATUS_FREQ, DEFAULT_STATUS_SEND_FREQ_SEC);

    Serial.println("Setting up");
    Serial.begin(115200);

    mDisplay.display.clearDisplay();
    mDisplay.display.display();

    if (mWifiEnabled)
    {
        connectWifi(WIFI_SSID, WIFI_PASS);
    }
    Mcu.begin();

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

    RadioEvents.TxDone = ::OnTxDone;
    RadioEvents.TxTimeout = ::OnTxTimeout;
    RadioEvents.RxDone = ::OnRxDone;
    Serial.println("Init Radio.");

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, txPower, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetSyncWord(LORA_SYNC_WORD);

    setIdle();

    Serial.println("Init Radio - complete");

    // Notify controller that we're online
    delay(200);
    sendHello();
}

void Receiver::setIdle()
{
    Serial.println("Radio - in read mode: waiting for incoming mesages");
    //
    // Begin in continuous receive mode so incoming packets are processed
    //
    Radio.Rx(0);

    //
    // We're idle if we're waiting.
    //
    lora_idle = true;
}

void Receiver::sendHello()
{
    sprintf(txpacket, "H:PWR:%d", txPower);
    Serial.printf("Sending hello \"%s\", length %d\r\n", txpacket, strlen(txpacket));
    lora_idle = false;
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    Serial.println("hello sent.");
}

void Receiver::sendStatus()
{
    float b = mBattery.getPercentage();
    int state = mBattery.isCharging() ? 0 : 1;
    int wifi = WIFI_DISABLED;
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED)
    {
        wifi = WIFI_CONNECTED;
    }
    else if (st == WL_DISCONNECTED || st == WL_IDLE_STATUS)
    {
        wifi = WIFI_CONNECTING;
    }
    else if (st != WL_NO_SHIELD)
    {
        wifi = WIFI_ERROR;
    }
    sprintf(txpacket, "S:%d:%d:%d:%d:%d:%.1f:%d:%d:%lu", txPower, mLastRssi, mLastSnr, mRelayState ? 1 : 0, mPulseMode ? 1 : 0, b, state, wifi, millis() / 1000UL);
    Serial.printf("Sending status \"%s\", length %d\r\n", txpacket, strlen(txpacket));
    lora_idle = false;
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    Serial.println("status sent.");
}

unsigned long lastScreenUpdate = 0;

void Receiver::loop()
{
    if (otaEnabled)
    {
        ArduinoOTA.handle();
    }
    if(mRelayState && offTime && millis() > offTime) {
        setRelayState(false);
    }
    if (millis() - lastScreenUpdate > 1000)
    {
        updateDisplay();

        lastScreenUpdate = millis();
    }

    if(lora_idle && millis() - lastStatusSend > statusSendFreqSec * 1000UL)
    {
        sendStatus();
        lastStatusSend = millis();
    }

    // if(lora_idle == false)
    {
        Radio.IrqProcess();
    }
}

void Receiver::sendAck(uint16_t id, const char *status)
{
    snprintf(txpacket, sizeof(txpacket), "A:%u:%s", id, status);
    Serial.printf("Sending ack \"%s\", length %d\r\n", txpacket, strlen(txpacket));
    lora_idle = false;
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    Serial.println("ack packet sent.");
}

void Receiver::setRelayState(bool newRelayState)
{
    mRelayState = newRelayState;
    if(newRelayState) {
        offTime = millis() + (unsigned long)onTimeSec * 1000UL;
    } else {
        offTime = 0;
    }
}

void Receiver::setTxPower(int power)
{
    if(power < MIN_TX_OUTPUT_POWER)
        power = MIN_TX_OUTPUT_POWER;
    if(power > 22)
        power = 22;

    txPower = power;
    Radio.SetTxConfig(MODEM_LORA, txPower, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Settings::setInt(KEY_RX_TX_POWER, txPower);
    updateDisplay();
}

void Receiver::setSendStatusFrequency(unsigned int freq)
{
    statusSendFreqSec = freq;
    Settings::setInt(KEY_RX_STATUS_FREQ, statusSendFreqSec);
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
            sendStatus();
            resp = "status";
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

void Receiver::connectWifi(const char *ssid, const char *pass)
{
    if(!mWifiEnabled)
    {
        return;
    }
    Serial.println("Connecting to WiFi for OTA");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected, starting OTA");
        ArduinoOTA.setHostname("heltec-pump-receiver");
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

void Receiver::OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
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

    mLastMessage = (char *)rxpacket;
    mLastMessageSize = size;
    mLastRssi = rssi;
    mLastSnr = snr;

    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n", rxpacket, rssi, size);
    processReceived(rxpacket);
    setIdle();
}

void Receiver::OnTxDone(void)
{
    Serial.println("TX done......");
    setIdle();
}

void Receiver::OnTxTimeout()
{
    Radio.Sleep();
    Serial.println("TX Timeout......");
    setIdle();
    // lora_idle = true;
    // // Return to receive mode after a timeout
    // Radio.Rx( 0 );
    // state=STATE_RX;
}

//
//
//

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    instance->OnRxDone(payload, size, rssi, snr);
}

void OnTxDone()
{
    instance->OnTxDone();
}
void OnTxTimeout()
{
    instance->OnTxTimeout();
}
