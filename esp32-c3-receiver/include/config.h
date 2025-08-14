#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include "config-private.h"


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     21 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             14       // dBm
#define MIN_TX_OUTPUT_POWER                         2        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12] // was 7
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_SYNC_WORD                              0x12
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 128 // Define the payload size here

// Default number of seconds the receiver will remain ON if
// no further ON commands are received.
#define DEFAULT_ON_TIME_SEC                         300

// Default frequency in seconds to send a status update
#define DEFAULT_STATUS_SEND_FREQ_SEC                60

// Minimum percent change in battery level before sending an immediate status
// update. Smaller fluctuations are ignored to reduce chatter.
#define BATTERY_PERCENT_CHANGE_THRESHOLD            2

// Milliseconds to wait without receiving any packets from the receiver
// before marking the relay state as unknown.
#define COMMUNICATION_TIMEOUT_MS                    60000

// Interval in milliseconds between OFF command retries
#define OFF_RETRY_INTERVAL_MS                       5000

// Number of retries to attempt when sending an OFF command
#define OFF_RETRY_COUNT                             3

// Analog pin used to measure battery voltage
#define BATTERY_VOLTAGE_PIN 2


enum WifiStatus {
    WIFI_DISABLED,
    WIFI_CONNECTED,
    WIFI_CONNECTING,
    WIFI_ERROR
};



#endif // DEVICE_CONFIG_H
