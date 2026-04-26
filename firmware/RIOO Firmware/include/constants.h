#ifndef CONSTANTS_H
#define CONSTANTS_H

// LEDS

#define BUILTIN_LED_PIN 2

// ULTRASONIC SENSORS

#define TRIG_PIN 5
#define ECHO_PIN 12

// === SERVOS === //

#define BASE_SERVO_PIN 13
#define SHOULDER_SERVO_PIN 14
#define ELBOW_SERVO_PIN 18
#define GRIPPER_SERVO_PIN 19

// === ARM MEASUREMENTS & CONSTRAINTS === //

const float SHOULDER_LENGTH = 127.0;
const float FOREARM_LENGTH = 101.6;
const float GRIPPER_REACH = 15.0;

const float MIN_JOINT = 0.0;
const float MAX_JOINT = 90.0;

const float MIN_REACH = 0.0;
const float MAX_REACH = 300.0;

#endif // CONSTANTS_H