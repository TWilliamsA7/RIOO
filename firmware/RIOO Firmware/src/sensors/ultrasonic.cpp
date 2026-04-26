#include "sensors/ultrasonic.h"
#include "systems.h"

uint8_t currentSensor = 0;
unsigned long nextPingTime = 0;
float collisionSensorDistances[ULTRASONIC_NUM];


void updateCollisionSensors() {
    if (millis() >= nextPingTime) {
        // 1. Get the distance from the previous sensor
        collisionSensorDistances[currentSensor] = sonars[currentSensor].ping_cm();

        // 2. Move to the next sensor
        currentSensor++;
        if (currentSensor >= ULTRASONIC_NUM) currentSensor = 0;

        // 3. Schedule the next check
        nextPingTime = millis() + PING_INTERVAL;
    }
}
    