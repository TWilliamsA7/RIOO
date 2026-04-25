#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

#define TOF_ERROR 0xFF

bool initializeTOF();
uint8_t getGripperDistance();

#endif // TOF_H