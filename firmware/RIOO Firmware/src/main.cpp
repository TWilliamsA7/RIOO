#include <Arduino.h>
#include <ESP32Servo.h>
#include <utility.h>

Servo myServo;

void setup() {

  initializeBuiltinLED();

    myServo.attach(18);  // GPIO pin
}

void loop() {

  heartbeatLED();

    myServo.write(0);
    delay(1000);

    myServo.write(90);
    delay(1000);

    myServo.write(180);
    delay(1000);
}
