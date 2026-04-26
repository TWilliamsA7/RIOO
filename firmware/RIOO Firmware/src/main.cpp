#include <Arduino.h>
#include "sensors/ultrasonic.h"
#include <utility.h>

void setup() {
  initializeLEDs();
  Serial.begin(115200);
}

unsigned long lastPrintTime = 0;

void loop() {

  heartbeatLED();

  updateCollisionSensors();

  unsigned long loopTime = millis();
  if (loopTime - lastPrintTime > 100) {
    Serial.print("LEFT: "); Serial.println(collisionSensorDistances[0]);
    Serial.print("RIGHT: "); Serial.println(collisionSensorDistances[1]);
    lastPrintTime = loopTime;
  }
}
