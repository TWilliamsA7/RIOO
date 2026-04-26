#include "systems.h"
#include "constants.h"

Servo BaseServo;
Servo ShoulderServo;
Servo ElbowServo;
Servo GripperServo;

Adafruit_VL6180X gripperTOF;

NewPing leftSonar(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_ULTRASONIC_DISTANCE);
NewPing rightSonar(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_ULTRASONIC_DISTANCE);
NewPing sonars[ULTRASONIC_NUM] = {
    leftSonar,
    rightSonar
};