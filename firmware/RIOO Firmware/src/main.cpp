#include <Arduino.h>
#include "sensors/ultrasonic.h"
#include <utility.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup() {
  initializeLEDs();
  Serial.begin(115200);
  tft.init();
    tft.setRotation(1); // Landscape
    tft.fillScreen(TFT_BLACK);
    
    // Static Header
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("RIOO DIAGNOSTICS", 10, 10, 4);
    tft.drawFastHLine(0, 40, 320, TFT_DARKGREY);
}

unsigned long lastPrintTime = 0;

void loop() {

  heartbeatLED();

  // updateCollisionSensors();

  // unsigned long loopTime = millis();
  // if (loopTime - lastPrintTime > 100) {
  //   Serial.print("LEFT: "); Serial.println(collisionSensorDistances[0]);
  //   Serial.print("RIGHT: "); Serial.println(collisionSensorDistances[1]);
  //   lastPrintTime = loopTime;
  // }
}
