#include "sensors/ultrasonic.h"
#include "systems.h"
#include "constants.h"

uint8_t currentSensor = 0;
unsigned long nextPingTime = 0;
float sensorDistances[ULTRASONIC_NUM];


void updateCollisionSensors() {
    if (millis() >= nextPingTime) {
        // 1. Get the distance from the previous sensor
        sensorDistances[currentSensor] = sonars[currentSensor].ping_cm();

        // 2. Move to the next sensor
        currentSensor++;
        if (currentSensor >= ULTRASONIC_NUM) currentSensor = 0;

        // 3. Schedule the next check
        nextPingTime = millis() + PING_INTERVAL;
    }
}
    