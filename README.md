# esp32-sx1262-lora-relay

A LoRa-based pump control system built around a generic ESP32-C3 board paired with an SX1262 module. The firmware can operate in two roles controlled by the `isController` flag in `src/main.cpp`:

- **Controller** – connects to WiFi and MQTT, publishes Home Assistant discovery messages and sends control commands to the receiver.
- **Receiver** – listens for LoRa commands, toggles a relay and reports battery charging information back to the controller.

## Building

1. Copy `include/config-example.h` to `include/config-private.h` and update it with your WiFi and MQTT settings.
2. Edit `src/main.cpp` and set `isController` to `true` for the controller firmware or `false` for the receiver.
3. Build the firmware with PlatformIO:

```bash
pio run
```

## Usage

Flash the compiled firmware to your ESP32-C3 boards. One board should be built with `isController = true`, the other with `false`. The controller connects to WiFi/MQTT and exchanges LoRa messages with the receiver. The receiver runs offline by default but can be asked to join WiFi for OTA updates.

## MQTT Topics

- `pump_station/switch/set` – payload `ON[:seconds]` or `OFF` to control the relay. If `seconds` is omitted the controller uses `DEFAULT_ON_TIME_SEC`.
- `pump_station/switch/pulse` – payload is the number of seconds to turn the relay on once.
- `pump_station/tx_power/controller/set` – integer transmit power in dBm for the controller.
- `pump_station/tx_power/receiver/set` – integer transmit power in dBm for the receiver.
- `pump_station/status_freq/controller/set` – controller status interval in seconds.
- `pump_station/status_freq/receiver/set` – receiver status interval in seconds.
- `pump_station/wifi/connect` – ask the receiver to join the default WiFi network and enable OTA updates.
- `pump_station/wifi/connect_custom` – payload `SSID:PASSWORD` to join a specific network for OTA.
- `pump_station/wifi/disable` – disconnect the receiver from WiFi and disable OTA updates.
- `pump_station/reboot` – instruct the receiver to reboot.
- `pump_station/status/request` – ask the receiver to immediately send status and battery info.
- `pump_station/switch/state` – retained state topic used by the controller to resume the last relay state.

The controller publishes status updates to:

- `pump_station/status/controller`
- `pump_station/status/receiver`
- `pump_station/status/receiver/battery_daily`
- `pump_station/status/stats`

## Home Assistant Discovery

When the controller connects to MQTT it publishes discovery messages so Home Assistant can automatically create entities. These include:

- `switch` – pump on/off control.
- `number` – `Pump Pulse`, `Controller Tx Power` and `Receiver Tx Power`.
- `sensor` – `Controller Status`, `Receiver Status`, `Receiver Battery Daily` and `Pump Stats`.
- `button` – request receiver status updates on demand.

Each discovery message references the MQTT topics listed above.
