#include "config.h"
#include "Arduino.h"

#include "receiver.h"
#include "BatteryMonitor.h"


Device *device = 0;
// Display display;

void setup() {

    
    Serial.begin(115200);
    Serial.println("-------------------------------------------");
    Serial.println("Setting up");

    // display.setup();
    battery.begin();
    battery.enableDebug(false);


    device = new Receiver();

    
    device->setup();
    Serial.println("Setup complete");
    Serial.println("-------------------------------------------");
}


void loop() {
    
    device->loop();
    battery.update();
    // display.loop();
}

