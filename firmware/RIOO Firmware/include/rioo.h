#ifndef RIOO_H
#define RIOO_H

#include "ikinematics.h"

extern float min_working_x;
extern float max_working_x;
extern float min_working_y;
extern float max_working_y;


void initializeRIOO();
void runRIOO();

Point computeTargetPoint(float rawX, float rawY);

#endif // RIOO_H