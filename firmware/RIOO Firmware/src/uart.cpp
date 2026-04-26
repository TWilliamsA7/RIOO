#include "uart.h"
#include <HardwareSerial.h>
#include <Arduino.h>
#include "utility.h"

GazeCommand gazeCommand = {0.0f, 0.0f, false};
HardwareSerial SerialPi(2);

float min_working_x = -1.0f;
float max_working_x = 1.0f;
float min_working_y =  -1.0f;
float max_working_y = 1.0f;

void initializeUART() {

    Serial.begin(115200);

    // Physical Pin Connection
    SerialPi.begin(115200, SERIAL_8N1, 16, 17);
}

void parseUART() {
    if (Serial.available() > 0) {
        String input = SerialPi.readStringUntil('\n');
        Serial.print("Raw: "); Serial.println(input);
        input.trim();

        if (input.length() == 0) return;

        int xPos = input.indexOf('X');
        int zPos = input.indexOf('Z');
        int gPos = input.indexOf('G');

        if (xPos != -1 && zPos != -1 && gPos != -1) {
            String xVal = input.substring(xPos + 1, zPos);
            String zVal = input.substring(zPos + 1, gPos);
            String gVal = input.substring(gPos + 1);

            gazeCommand.xPos = xVal.toFloat();
            gazeCommand.zPos = zVal.toFloat();
            gazeCommand.grab = (gVal.toInt() == 1);

            // Debugging Print
            Serial.print("X: "); Serial.print(gazeCommand.xPos);
            Serial.print(" | Z: "); Serial.print(gazeCommand.zPos);
            Serial.print(" | Grab: "); Serial.println(gazeCommand.grab);
        }


        
    }
}

