#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>

// Initialize On-Board LED
void initializeLEDs(void);

// Blink On-Board LED
void heartbeatLED(void);

// Indicate a potential collision
void flashWarningLED(void);

// Indicate a successful calibration
void enableCalibrationLED(void);

#endif // UTILITY_H