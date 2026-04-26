#include "display/tft.h"

TFT_eSPI tft = TFT_eSPI();

void initializeDebugScreen() {
    tft.init();
    tft.setRotation(1); // Landscape
    tft.fillScreen(TFT_BLACK);
    
    // Static Header
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("RIOO DIAGNOSTICS", 10, 10, 4);
    tft.drawFastHLine(0, 40, 320, TFT_DARKGREY);
}

void updateDebugScreen(float x, float y, float z, float leftDist, float rightDist) {
    tft.setTextSize(1);
    
    // Label Setup - Shifted slightly for better margins
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("GAZE X:", 10, 60, 4);
    tft.drawString("GAZE Y:", 10, 90, 4);
    tft.drawString("GAZE Z:", 10, 120, 4);
    tft.drawString("L-SONAR:", 10, 150, 4); // Shortened to prevent overlap
    tft.drawString("R-SONAR:", 10, 180, 4);

    // Dynamic Values - Moved X to 160 to give labels breathing room
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawFloat(x, 3, 160, 60, 4);
    tft.drawFloat(y, 3, 160, 90, 4);
    tft.drawFloat(z, 3, 160, 120, 4);
    
    // Left Distance
    uint16_t LColor = (leftDist < 15.0 && leftDist > 0) ? TFT_RED : TFT_GREEN;
    tft.setTextColor(LColor, TFT_BLACK);
    tft.drawFloat(leftDist, 1, 160, 150, 4);
    tft.drawString("cm  ", 250, 150, 4); // Pushed 'cm' further right

    // Right Distance
    uint16_t RColor = (rightDist < 15.0 && rightDist > 0) ? TFT_RED : TFT_GREEN;
    tft.setTextColor(RColor, TFT_BLACK);
    tft.drawFloat(rightDist, 1, 160, 180, 4);
    tft.drawString("cm  ", 250, 180, 4);
}