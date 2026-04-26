#ifndef RIOO_H
#define RIOO_H

#include "ikinematics.h"

void initializeRIOO();
void runRIOO();

Point computeTargetPoint(float rawX, float rawY);

#endif // RIOO_H