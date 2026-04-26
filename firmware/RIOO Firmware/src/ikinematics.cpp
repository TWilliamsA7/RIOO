#include "ikinematics.h"
#include "constants.h"
#include "sensors/tof.h"
#include "Arduino.h"

GazeTracker gaze;

JointAngles calculateIK(Point& target) {
    JointAngles angles;

    // 1. Base Angle
    angles.base = atan2(target.x, target.z) * 180.0 / M_PI;

    // 2. Reach and Space Distance
    float r = sqrt(sq(target.x) + sq(target.z));
    float s = sqrt(sq(r) + sq(target.y));

    if (s < 1e-6f) return angles;

    // 3. Elbow Angle (Law of Cosines)
    float cosElbow = (sq(SHOULDER_LENGTH) + sq(FOREARM_LENGTH) - sq(s)) / (2 * SHOULDER_LENGTH * FOREARM_LENGTH);
    cosElbow = constrain(cosElbow, -1.0, 1.0); 
    angles.elbow = acos(cosElbow) * 180.0 / M_PI;

    // 4. Shoulder Angle
    float angleA = atan2(target.y, r);
    float cosShoulder = (sq(SHOULDER_LENGTH) + sq(s) - sq(FOREARM_LENGTH)) / (2 * SHOULDER_LENGTH * s);
    cosShoulder = constrain(cosShoulder, -1.0, 1.0);
    angles.shoulder = (angleA + acos(cosShoulder)) * 180.0 / M_PI;

    return angles;
}

void computeTargetZ(Point& target) {
    if (gaze.update(target.x, target.y)) {
        float gripperDist = getGripperDistance();

        if (gripperDist == TOF_ERROR) {
            target.z += REACH_SPEED;
        } else {
            target.z += (gripperDist - GRIPPER_REACH);
        }
    } else {
        target.z -= REACH_SPEED;
        if (target.z < 0.0) {
            target.z = 0.0;
        }
    }
}