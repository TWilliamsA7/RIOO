#include "uart.h"
#include <HardwareSerial.h>
#include <Arduino.h>

GazeCommand currentCommand = {0.0f, 0.0f, false};
HardwareSerial SerialPi(2);

void initializeUART() {

    Serial.begin(115200);

    // Physical Pin Connection
    SerialPi.begin(115200, SERIAL_8N1, 16, 17);
}

void parseUART() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        int xPos = input.indexOf('X');
        int yPos = input.indexOf('Y');
        int gPos = input.indexOf('G');

        if (xPos != -1 && yPos != -1 && gPos != -1) {
            String xVal = input.substring(xPos + 1, yPos);
            String yVal = input.substring(yPos + 1, gPos);
            String gVal = input.substring(gPos + 1);

            currentCommand.xPos = xVal.toFloat();
            currentCommand.yPos = yVal.toFloat();
            currentCommand.grab = (gVal.toInt() == 1);

            // Debugging Print
            Serial.print("X: "); Serial.print(currentCommand.xPos);
            Serial.print(" | Y: "); Serial.print(currentCommand.yPos);
            Serial.print(" | Grab: "); Serial.println(currentCommand.grab);
        }
    }
}