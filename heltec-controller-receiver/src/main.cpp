#include "config.h"
#include "Arduino.h"

#include "controller.h"
#include "receiver.h"
#include "display.h"
#include "battery.h"

bool isController = false;
bool enableWifi = isController;

Device *device = 0;
Display display;
Battery battery;

void setup() {

    
    Serial.begin(115200);
    Serial.println("-------------------------------------------");
    Serial.println("Setting up");

    display.setup();
    battery.setup();

    if(isController)
    {
        device = new Controller(display);        
    }
    else
    {
        device = new Receiver(display, battery, enableWifi);
    }

    
    device->setup();
    Serial.println("Setup complete");
    Serial.println("-------------------------------------------");
}


void loop() {
    
    device->loop();
    // display.loop();
}

