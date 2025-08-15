#ifndef PUMP_SETTINGS_H
#define PUMP_SETTINGS_H

#include <Preferences.h>

namespace Settings {
    void begin();
    int getInt(const char *key, int defaultValue);
    void setInt(const char *key, int value);
}

#define KEY_CTRL_TX_POWER "ctrl_tx_pwr"
#define KEY_RX_TX_POWER "rx_tx_pwr"
#define KEY_CTRL_STATUS_FREQ "ctrl_status_freq"
#define KEY_RX_STATUS_FREQ "rx_status_freq"

#endif // PUMP_SETTINGS_H
