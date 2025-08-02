# esp32-c3-SX1262-LoRa-Receiver
LoRa SX1262 receiver component tailored for the ESP32 C3 with an 18650 battery management board.

## Battery voltage divider

The ADC on the ESP32 is not very linear near 0 or full scale. To keep
measurements in the reliable range it uses a voltage divider on the
battery input. The code assumes ~68&nbsp;kΩ from the battery positive
to the ADC pin and 100&nbsp;kΩ from the pin to ground. With a fully
charged cell (~4.2&nbsp;V) the ADC sees about 2.5&nbsp;V which is well inside
the 11&nbsp;dB attenuation range.
