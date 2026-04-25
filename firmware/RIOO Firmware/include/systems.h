#ifndef SYSTEMS_H
#define SYSTEMS_H

#include "Adafruit_VL6180X.h"
#include "ESP32Servo.h"

// === SERVOS === //

extern Servo BaseServo;
extern Servo ShoulderServo;
extern Servo ElbowServo;
extern Servo GripperServo;

// === TOF SENSORS === //

extern Adafruit_VL6180X gripperTOF;

#endif // SYSTEMS_H