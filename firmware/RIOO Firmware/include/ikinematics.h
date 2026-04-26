#ifndef IKINEMATICS_H
#define IKINEMATICS_H

#include "gaze.h"

struct Point {
    float x, y, z;
};

struct JointAngles {
    float base, shoulder, elbow;
};

extern GazeTracker gaze;

JointAngles calculateIK(Point& target);
void computeTargetZ(Point& target);

#endif
