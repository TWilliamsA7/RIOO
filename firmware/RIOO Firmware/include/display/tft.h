#ifndef TFT_H
#define TFT_H

#include <TFT_eSPI.h>

extern TFT_eSPI tft;

void initializeDebugScreen();
void updateDebugScreen(float x, float y, float z, float leftDist, float rightDist);

#endif TFT_H