#include "utility.h"
#include "constants.h"

void initializeLEDs(void) {
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

void flashWarningLED() {
    static unsigned long lastFlash = 0;
    static bool ledState = false;
    
    if (millis() - lastFlash > 200) { // Blink every 200ms
        ledState = !ledState;
        digitalWrite(WARNING_LED_PIN, ledState);
        lastFlash = millis();
    }
}

