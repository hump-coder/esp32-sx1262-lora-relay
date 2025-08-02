#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"

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
char ackpacket[BUFFER_SIZE];

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
    acksRemaining = 0;
    ackStateId = 0;
    ackConfirmed = true;
    lastStatusSend = 0;
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
    mDisplay.display.printf("Ack: %s", ackConfirmed ? "OK" : "WAIT");
    
    y+=step;
    mDisplay.display.setCursor(x, y);
    if(mRelayState) {
        unsigned long remaining = 0;
        if(offTime > millis()) {
            remaining = (offTime - millis()) / 1000;
        }
        mDisplay.display.printf("Time: %lus", remaining);
    } else {
        mDisplay.display.printf("Pend Acks: %d", acksRemaining);
    }

    y+=step;
    mDisplay.display.setCursor(x, y);
    mDisplay.display.printf("PW:%d RS:%d SR:%d", txPower, mLastRssi, mLastSnr);

    y+=step;
    mDisplay.display.setCursor(x, y);

    float voltage = mBattery.getVoltage();
    int batt = mBattery.getPercentage();
    bool chg = mBattery.isCharging();

    mDisplay.display.printf("BAT: %.2fv  %d%% %s", voltage, batt, chg ? "CHG" : " ");

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
        Serial.println("Init Wifi");
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
        }
        Serial.println("Init Wifi - complete");
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
    int battery = mBattery.getPercentage();
    sprintf(txpacket, "S:%d:%d:%d:%d:%d:%d", txPower, mLastRssi, mLastSnr, mRelayState ? 1 : 0, mPulseMode ? 1 : 0, battery);
    Serial.printf("Sending status \"%s\", length %d\r\n", txpacket, strlen(txpacket));
    lora_idle = false;
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    Serial.println("status sent.");
}

unsigned long lastScreenUpdate = 0;

void Receiver::loop()
{
    if(mRelayState && offTime && millis() > offTime) {
        setRelayState(false);
    }
    if (millis() - lastScreenUpdate > 1000)
    {
        updateDisplay();

        if (!ackConfirmed && acksRemaining)
        {
            --acksRemaining;
            delay(100);
            sendAck(ackpacket);
        }

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

void Receiver::sendAck(char *packet)
{
    //
    // Mark the packet as from the receiver.
    //
    packet[0] = 'R';

    Serial.printf("Sending ack \"%s\", length %d\r\n", packet, strlen(packet));

    lora_idle = false;
    Radio.Send((uint8_t *)packet, strlen(packet)); // send the package out

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

    ackpacket[0] = 0; 
    strcpy(ackpacket, rxpacket);

    ptr = strtok(rxpacket, ":;"); // takes a list of delimiters
    while (ptr != NULL)
    {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, ":;"); // takes a list of delimiters
    }

    if (index >= 2 && strlen(strings[0]) == 1 && strings[0][0] == 'A')
    {
        uint16_t stateId = atoi(strings[1]);
        if(stateId == ackStateId)
        {
            ackConfirmed = true;
            acksRemaining = 0;
        }
    }
    else if (index >= 3 && strlen(strings[0]) == 1 && strings[0][0] == 'C')
    {
        uint16_t stateId = atoi(strings[1]);
        if(strcasecmp(strings[2], "status") == 0) {
            sendStatus();
            return;
        } else if(strcasecmp(strings[2], "freq") == 0 && index >= 4) {
            int freq = atoi(strings[3]);
            setSendStatusFrequency(freq);
            return;
        } else if(strcasecmp(strings[2], "pwr") == 0 && index >= 4) {
            int power = atoi(strings[3]);
            setTxPower(power);
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
        }
        delay(200);
        ackStateId = stateId;
        ackConfirmed = false;
        acksRemaining = 4;
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
