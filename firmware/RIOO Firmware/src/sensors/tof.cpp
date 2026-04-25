#include "sensors/tof.h"
#include "systems.h"

bool initializeTOF() {
    return gripperTOF.begin();
}

uint8_t getGripperDistance() {
    uint8_t range = gripperTOF.readRange();
    uint8_t status = gripperTOF.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
        return range;
    } else {
        return TOF_ERROR;
    }
}