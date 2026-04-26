#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>

// Initialize On-Board LED
void initializeLEDs(void);

// Blink On-Board LED
void heartbeatLED(void);

void flashWarningLED(void);

#endif // UTILITY_H