#include <Arduino.h>
#include <ESP32Servo.h>
#include <utility.h>
#include <ultrasonic.h>

Servo myServo;

void setup() {

  Serial.begin(115200);
  initializeBuiltinLED();
  initializeUltrasonics();

    // myServo.attach(18);  // GPIO pin
}

void loop() {

  heartbeatLED();

  float dist = getDistanceCM();
  Serial.print("Distance: ");
  Serial.println(dist);
  delay(50);  

    // myServo.write(0);
    // delay(1000);

    // myServo.write(90);
    // delay(1000);

    // myServo.write(180);
    // delay(1000);
}
