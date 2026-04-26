#include "rioo.h"
#include "uart.h"
#include "servo/servo.h"
#include "sensors/tof.h"
#include "sensors/ultrasonic.h"
#include "constants.h"
#include "utility.h"

float min_working_x;
float max_working_x;
float min_working_y;
float max_working_y;

void initializeRIOO() {
    initializeLEDs();
    initializeUART();
    initializeServos();
    initializeTOF();
    Serial.println("Finished Initialization!");

    /* 
        TODO: Calibration Step 
        Have the user look at specific spots marked on
        the table to set the bounds of the robot. 
        We should have this in setup
    */
}

unsigned long lastLoopTime = 0;

void runRIOO() {
    // Parse Coordinate Data from over UART
    parseUART();

    unsigned long currentTime = millis();


    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {

        // Check collision sensors
        updateCollisionSensors();

        // Iterate over distances
        for (int i = 0; i < ULTRASONIC_NUM; i++) {
            if (collisionSensorDistances[i] < COLLISION_WARNING_DISTANCE) {
                flashWarningLED();
            }
        }

        // TODO: Grip

        // Compute new target based on UART data
        Point target = computeTargetPoint(gazeCommand.xPos, gazeCommand.yPos);
    
        // Use IK to calculate expected angles of joints
        JointAngles ja = calculateIK(target);
    
        // Update servos based on computed angles
        updateServos(ja);
    }
}

float prevTargetX = 0.0f;
float prevTargetY = 0.0f;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Point computeTargetPoint(float rawX, float rawY) {
    float targetX = mapFloat(rawX, -1.0, 1.0, min_working_x, max_working_x);
    float targetY = mapFloat(rawY, -1.0, 1.0, min_working_y, max_working_y);

    static bool firstRun = true;
    if (firstRun) {
        prevTargetX = targetX;
        prevTargetY = targetY;
        firstRun = false;
    }

    targetX = (PLANAR_ALPHA * targetX) + ((1.0 - PLANAR_ALPHA) * prevTargetX);
    targetY = (PLANAR_ALPHA * targetY) + ((1.0 - PLANAR_ALPHA) * prevTargetY);

    Point newTarget;
    newTarget.x = targetX;
    newTarget.y = targetY;
    prevTargetX = targetX;
    prevTargetY = targetY;

    computeTargetZ(newTarget);
    return newTarget;
}
