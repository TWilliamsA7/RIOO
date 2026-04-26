#include "rioo.h"
#include "uart.h"
#include "servo/servo.h"
#include "sensors/tof.h"
#include "sensors/ultrasonic.h"
#include "constants.h"
#include "utility.h"
#include "display/tft.h"

void calculateEvasiveManeuver(JointAngles &ja);

void initializeRIOO() {
    initializeLEDs();
    initializeUART();
    initializeServos();
    initializeTOF();
    Serial.println("Finished Initialization!");
}

unsigned long lastLoopTime = 0;

void runRIOO() {
    // Parse Coordinate Data from over UART
    parseUART();

    unsigned long currentTime = millis();

    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {

        heartbeatLED();

        // Check collision sensors
        updateCollisionSensors();

        // Iterate over distances
        for (int i = 0; i < ULTRASONIC_NUM; i++) {
            if (collisionSensorDistances[i] < COLLISION_WARNING_DISTANCE) {
                flashWarningLED();
            }
        }

        // Update the gripper to desired state

        toggleGrip(gazeCommand.grab);

        updateDebugScreen(gazeCommand.xPos, gazeCommand.yPos, 0, collisionSensorDistances[0], collisionSensorDistances[1]);

        // Only move ARM when the claw is not moving
        if (clawActionDone) {
            // Compute new target based on UART data
            Point target = computeTargetPoint(gazeCommand.xPos, gazeCommand.yPos);
        
            // Use IK to calculate expected angles of joints
            JointAngles ja = calculateIK(target);

            calculateEvasiveManeuver(ja);
        
            // Update servos based on computed angles
            updateServos(ja);
        }
    }
}

float prevTargetX = 0.0f;
float prevTargetY = 0.0f;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float avoidanceBias = 0.0f; 

void calculateEvasiveManeuver(JointAngles &ja) {
    float left = collisionSensorDistances[0];
    float right = collisionSensorDistances[1];
    
    float nudgeStrength = 3.0; // Degrees to move per update

    // If Left sensor sees something, nudge the base to the Right (Negative)
    if (left > 0 && left < COLLISION_AVOIDANCE_DISTANCE) {
        avoidanceBias -= nudgeStrength;
    } 
    // If Right sensor sees something, nudge the base to the Left (Positive)
    else if (right > 0 && right < COLLISION_AVOIDANCE_DISTANCE) {
        avoidanceBias += nudgeStrength;
    } 
    else {
        // "Centering Force": Slowly bring bias back to 0 when path is clear
        avoidanceBias *= 0.85; 
    }

    // Clamp the bias so the arm doesn't spin wildly
    avoidanceBias = constrain(avoidanceBias, -30.0, 30.0);

    // Apply to the Base Joint
    ja.base += avoidanceBias;
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
