#ifndef IKINEMATICS_H
#define IKINEMATICS_H

#include "gaze.h"

struct Point {
    float x, y, z;
};

struct JointAngles {
    float base, shoulder, elbow;
};

Point target;
GazeTracker gaze;

const float REACH_SPEED = 2.5;

JointAngles calculateIK();
void computeTargetZ(float x, float y);


#endif // IKINEMATICS_H