#include <Arduino.h>
#include "sensors/ultrasonic.h"
#include <utility.h>
#include "rioo.h"

void setup() {
  initializeRIOO();
}

void loop() {
  runRIOO();
}
