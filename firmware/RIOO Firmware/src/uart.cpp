#include "uart.h"
#include <HardwareSerial.h>
#include <Arduino.h>

GazeCommand currentCommand = {0.0f, 0.0f, false};
HardwareSerial SerialPi(2);

float min_working_x;
float max_working_x;
float min_working_y;
float max_working_y;

void parseCalibration(String data);
void parseTracking(String input);

void initializeUART() {

    Serial.begin(115200);

    // Physical Pin Connection
    SerialPi.begin(115200, SERIAL_8N1, 16, 17);
}

void parseUART() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) return;

        char header = input.charAt(0);
        String data = input.substring(1);

        // Tracking
        if (header == 'T') {
            parseTracking(data);
        } else if (header == 'C') { // Calibration
            
        }


        
    }
}

void parseTracking(String input) {
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

void parseCalibration(String data) {

    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);

    if (c1 != -1 && c2 != -1 && c3 != -1) {
        min_working_x = data.substring(0, c1).toFloat();
        max_working_x = data.substring(c1 + 1, c2).toFloat();
        min_working_y = data.substring(c2 + 1, c3).toFloat();
        max_working_y = data.substring(c3 + 1).toFloat();
        
        Serial.println("Workspace Calibrated!");
    }
}