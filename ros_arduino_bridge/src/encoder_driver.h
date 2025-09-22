/*************************************************************
   Encoder driver function definitions for 4-motor TB6612 config
   *************************************************************/

#include "config.h"

// Encoder pin mapping for 4 motors (from config.h)
#define FRONT_LEFT_ENC_A   ENC1_A_PIN
#define FRONT_LEFT_ENC_B   ENC1_B_PIN

#define FRONT_RIGHT_ENC_A  ENC2_A_PIN
#define FRONT_RIGHT_ENC_B  ENC2_B_PIN

#define REAR_LEFT_ENC_A    ENC3_A_PIN
#define REAR_LEFT_ENC_B    ENC3_B_PIN

#define REAR_RIGHT_ENC_A   ENC4_A_PIN
#define REAR_RIGHT_ENC_B   ENC4_B_PIN

// Function prototypes
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
