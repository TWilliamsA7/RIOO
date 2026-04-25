#include "ultrasonic.h"
#include "constants.h"

void initializeUltrasonics() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

float getDistanceCM() {
    // Ensure trigger is LOW
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10 µs pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo time
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout = 30ms

    // Convert to distance
    float distance = duration * 0.0343 / 2;

    return distance;
}