#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <Arduino.h>

void initializeStepper();
void handleStepperCommand(int rpm, int distance_mm, int flag);
void runStepper();
bool isStepperActive();
#endif