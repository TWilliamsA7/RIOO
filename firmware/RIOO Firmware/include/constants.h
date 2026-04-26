#ifndef CONSTANTS_H
#define CONSTANTS_H

// LEDS

#define BUILTIN_LED_PIN 2
#define WARNING_LED_PIN 4
#define CALIBRATION_LED_PIN 17

// ULTRASONIC SENSORS

#define LEFT_ECHO_PIN 5
#define LEFT_TRIG_PIN 12

#define RIGHT_ECHO_PIN 26
#define RIGHT_TRIG_PIN 27

#define MAX_ULTRASONIC_DISTANCE 200 // (cm)
#define ULTRASONIC_NUM 2
#define PING_INTERVAL 33 // (ms)

constexpr float COLLISION_WARNING_DISTANCE = 8.0;

// === SERVOS === //

#define BASE_SERVO_PIN 13
#define SHOULDER_SERVO_PIN 14
#define ELBOW_SERVO_PIN 18
#define GRIPPER_SERVO_PIN 19

// === SERVO CONFIGURATION === //

constexpr float SERVO_STEP_SIZE = 1.5;
constexpr float SERVO_DEADBAND = 0.5;
constexpr float REACH_SPEED = 2.5;

// === ARM MEASUREMENTS & CONSTRAINTS === //

const float SHOULDER_LENGTH = 127.0; // (mm)
const float FOREARM_LENGTH = 101.6; // (mm)
const float GRIPPER_REACH = 15.0; // (mm)

const float MIN_JOINT = 0.0;
const float MAX_JOINT = 90.0;

const float MIN_REACH = 0.0;
const float MAX_REACH = 300.0; // (mm)

constexpr int GRIP_OPEN = 5; // (deg)
constexpr int GRIP_CLOSED = 9; // (deg)
constexpr float GRIP_SPEED = 0.5; 
constexpr float RELEASE_SPEED = 0.5;

// === ARM RESPONSIVENESS PARAMETERS === //

constexpr int LOOP_INTERVAL = 20;
constexpr float PLANAR_ALPHA = 0.25;

#endif // CONSTANTS_H