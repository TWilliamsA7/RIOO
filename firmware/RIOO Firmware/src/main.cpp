#include <Arduino.h>
#include "sensors/ultrasonic.h"
#include <utility.h>

void setup() {
  Serial.begin(115200);
  initializeBuiltinLED();
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
