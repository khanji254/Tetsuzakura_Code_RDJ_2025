/***************************************************************
   Motor driver function definitions for TB6612 - by [Your Name]
   *************************************************************/

#ifndef MOTOR_DRIVER_TB6612_H
#define MOTOR_DRIVER_TB6612_H

#include "config.h"

// Motor index mapping for TB6612
#define MOTOR_FRONT_LEFT   0
#define MOTOR_FRONT_RIGHT  1
#define MOTOR_REAR_LEFT    2
#define MOTOR_REAR_RIGHT   3

void initMotorControllerTB6612();
void setMotorSpeedTB6612(int motor, int speed);
void setMotorSpeedsTB6612(int flSpeed, int frSpeed, int rlSpeed, int rrSpeed);

#endif // MOTOR_DRIVER_TB6612_H