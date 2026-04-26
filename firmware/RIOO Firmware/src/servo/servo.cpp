#include "servo/servo.h"
#include "systems.h"
#include "constants.h"

ServoAngles servoAngles = { 0.0f, 0.0f, 0.0f };

void initializeServos() {
    servoAngles = {0, 0, 0};

    // Attach servos to pins
    BaseServo.attach(BASE_SERVO_PIN);
    ShoulderServo.attach(SHOULDER_SERVO_PIN);
    ElbowServo.attach(ELBOW_SERVO_PIN);

    // Move each servo sequentially to 0
    BaseServo.write(0);
    delay(1000);
    ShoulderServo.write(0);
    delay(1000);
    ElbowServo.write(0);
    delay(1000);
}

void updateServos(JointAngles ja) {
    static ServoPlacement activeJoint = BASE;
    switch (activeJoint) {
        case BASE:
            updateServo(BASE, ja.base);
            activeJoint = SHOULDER;
            break;
        case SHOULDER:
            updateServo(SHOULDER, ja.shoulder);
            activeJoint = ELBOW;
            break;
        case ELBOW:
            updateServo(ELBOW, ja.elbow);
            activeJoint = BASE;
            break;
    }
} 

void updateServo(ServoPlacement plc, float angle) {
    float currentAngle = 0.0;
    switch (plc) {
        case BASE:
            currentAngle = servoAngles.base;
            break;
        case ELBOW:
            currentAngle = servoAngles.elbow;
            break;
        case SHOULDER:
            currentAngle = servoAngles.shoulder;
            break;
    }

    float diff = angle - currentAngle;

    if (abs(diff) < SERVO_DEADBAND) return;

    if (diff > 0) {
        currentAngle += min(diff, SERVO_STEP_SIZE);
    } else {
        currentAngle -= min(abs(diff), SERVO_STEP_SIZE);
    }

    switch (plc) {
        case BASE:
            BaseServo.write(currentAngle);
            break;
        case ELBOW:
            ElbowServo.write(currentAngle);
            break;
        case SHOULDER:
            ShoulderServo.write(currentAngle);
            break;
    }
}

void toggleGrip(bool grab) {
    static float currentGripPos = GRIP_OPEN;
    
    if (grab) {
        if (currentGripPos < GRIP_CLOSED) {
            currentGripPos += GRIP_SPEED;
        }
    }else {
        if (currentGripPos > GRIP_OPEN) {
            currentGripPos -= RELEASE_SPEED;
        }
    }

    GripperServo.write(currentGripPos);
}


