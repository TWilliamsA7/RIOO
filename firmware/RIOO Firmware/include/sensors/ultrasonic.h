#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include "constants.h"

extern float collisionSensorDistances[ULTRASONIC_NUM];

void updateCollisionSensors();

#endif // ULTRASONIC_H