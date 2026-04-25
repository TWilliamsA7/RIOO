#ifndef IKINEMATICS_H
#define IKINEMATICS_H

struct Point {
    float x, y, z;
};

struct JointAngles {
    float base, shoulder, elbow;
};

JointAngles calculateIK(Point target);

#endif // IKINEMATICS_H