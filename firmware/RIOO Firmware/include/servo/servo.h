#ifndef SERVO_H
#define SERVO_H

#include "ikinematics.h"

struct ServoAngles {
    float base, shoulder, elbow;
};

enum ServoPlacement { BASE, ELBOW, SHOULDER };

extern ServoAngles servoAngles;

// Initialize all servos
void initializeServos();

// Updates one servo per cycle { BASE, ELBOW, SHOULDER }
void updateServos(JointAngles ja);

// Incremental Movement of a single servo towards angle
void updateServo(ServoPlacement plc, float angle);

// Execute Grip and Release
void toggleGrip(bool grab);

#endif // SERVO_H