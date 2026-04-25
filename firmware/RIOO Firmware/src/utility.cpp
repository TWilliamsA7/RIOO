#include "utility.h"
#include "constants.h"

void initializeBuiltinLED(void) {
    pinMode(BUILTIN_LED_PIN, OUTPUT);
}

void heartbeatLED(void) {
    static unsigned long lastToggle = 0;
    static bool ledState = false;

    unsigned long currentMillis = millis();
    
    // 500 ms interval
    if (currentMillis - lastToggle >= 500) { 
        lastToggle = currentMillis;
        ledState = !ledState;
        digitalWrite(BUILTIN_LED_PIN, ledState);
    }
}

