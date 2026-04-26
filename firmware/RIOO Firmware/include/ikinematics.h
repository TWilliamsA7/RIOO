#ifndef IKINEMATICS_H
#define IKINEMATICS_H

#include "gaze.h"

struct Point {
    float x, y, z;
};

struct JointAngles {
    float base, shoulder, elbow;
};

extern Point target;
extern GazeTracker gaze;

JointAngles calculateIK();
void computeTargetZ(float x, float y);

#endif
