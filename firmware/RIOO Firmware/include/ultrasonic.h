#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

void initializeUltrasonics();

float getDistanceCM();

#endif // ULTRASONIC_H