#ifndef IKINEMATICS_H
#define IKINEMATICS_H

typedef struct Point {
    float x, y, z;
};

typedef struct JointAngles {
    float base, shoulder, elbow;
};

JointAngles calculateIK(Point target);

#endif // IKINEMATICS_H