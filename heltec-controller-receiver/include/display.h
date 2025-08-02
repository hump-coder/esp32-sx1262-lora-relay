#ifndef PUMP_DISPLAY_H
#define PUMP_DISPLAY_H

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "device-config.h"

class Display 
{
    public:    
    Display() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}
    void setup();        
    Adafruit_SSD1306 display;
};


#endif // PUMP_DISPLAY_H
