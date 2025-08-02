#include "display.h"
//#include "LoRaWan_APP.h"
//#include "config.h"
//#include "Arduino.h"

//#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "device-config.h"


void Display::setup() {
    Serial.println("Init display.");
    Wire.begin(17, 18);

    //Light on
//    pinMode(35, OUTPUT);
//    digitalWrite(35, HIGH);


    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);
    // Heltec delays for 100ms in their example
    delay(100);
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
    //    for(;;); // Don't proceed, loop forever
    }

    //Mcu.begin();
      // Show initial display buffer contents on the screen --
      // the library initializes this with an Adafruit splash screen.
      display.display();
      delay(1000); // Pause for 2 seconds

      // Clear the buffer
 //     display.clearDisplay();

    // Draw a single pixel in white
    //display.drawPixel(10, 10, SSD1306_WHITE);

    //testscrolltext();
    // Show the display buffer on the screen. You MUST call display() after
    // drawing commands to make them visible on screen!
    //display.display();
    //delay(2000);

    Serial.println("Init display - complete.");
}

