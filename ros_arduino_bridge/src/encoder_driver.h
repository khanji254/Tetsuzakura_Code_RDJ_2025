#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include "config.h"

// Encoder count variables
extern volatile long encCount1, encCount2, encCount3, encCount4;

// Initialization and ISR prototypes
void initializeEncoders();
void ISR_enc1();
void ISR_enc2();
void ISR_enc3();
void ISR_enc4();

// Encoder reading functions (cumulative)
long getM1Encoder();
long getM2Encoder();
long getM3Encoder();
long getM4Encoder();

// Reset all encoder counts to zero
void resetEncoders();

#endif // ENCODER_DRIVER_H