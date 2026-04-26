#ifndef SYSTEMS_H
#define SYSTEMS_H

#include "Adafruit_VL6180X.h"
#include "ESP32Servo.h"
#include "NewPing.h"
#include "constants.h"

// === SERVOS === //

extern Servo BaseServo;
extern Servo ShoulderServo;
extern Servo ElbowServo;
extern Servo GripperServo;

// === TOF SENSORS === //

extern Adafruit_VL6180X gripperTOF;

// === ULTRASONIC SENSORS === //

extern NewPing leftSonar;
extern NewPing rightSonar;
extern NewPing sonars[ULTRASONIC_NUM];

#endif // SYSTEMS_H