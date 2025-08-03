#include "config.h"
#include "Arduino.h"

#include "receiver.h"
#include "controller.h"
#include "BatteryMonitor.h"


bool isController = true;
bool enableWifi = isController;

Device *device = 0;

void setup() {

    
    Serial.begin(115200);
    Serial.println("-------------------------------------------");
    Serial.println("Setting up");

    battery.begin();
    battery.enableDebug(false);

    if(isController)
    {
        device = new Controller();
    }
    else
    {
        device = new Receiver();
    }

    device->setup();
    Serial.println("Setup complete");
    Serial.println("-------------------------------------------");
}


void loop() {
    
    device->loop();
    battery.update();
}

