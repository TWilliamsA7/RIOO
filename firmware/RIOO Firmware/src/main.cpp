#include <Arduino.h>
#include <ESP32Servo.h>

const int POT_PIN = 34;
const int SERVO_PIN = 13;

Servo servo;

void setup() {
  Serial.begin(115200);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  analogReadResolution(12);
}

void loop() {

  int potValue = analogRead(POT_PIN);

  int angle = map(potValue, 0, 4095, 0, 180);
  servo.write(angle);

  Serial.print("Pot: ");
  Serial.print(potValue);
  Serial.print(" | Angle: ");
  Serial.println(angle);

  delay(100); // small delay is fine here

}