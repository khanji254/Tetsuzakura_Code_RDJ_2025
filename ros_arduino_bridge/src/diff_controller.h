/* Functions and type-defs for PID control (4-motor version).

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#include "config.h"
#include "encoder_driver.h"  // <-- Ensure you include the new encoder driver

#define FRONT_LEFT   0
#define FRONT_RIGHT  1
#define REAR_LEFT    2
#define REAR_RIGHT   3

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevInput;                 // last input
  int ITerm;                     // integrated term
  long output;                   // last motor setting
} SetPointInfo;

SetPointInfo flPID, frPID, rlPID, rrPID;

/* PID Parameters - Tuned for fast-responding motors with 285 ticks/rev
 * These values were determined through auto-tuning with step response testing
 * Conservative gains to prevent oscillation: Kp=0.3*Ku, Ki=0.1*Kp, Kd=0.05*Kp
 * Ko is the scaling factor for integer math (multiply float gains by Ko)
 */
int Kp = 30;  // Proportional gain (scaled by Ko=100)
int Kd = 1;   // Derivative gain (scaled by Ko=100) 
int Ki = 3;   // Integral gain (scaled by Ko=100)
int Ko = 100; // Scaling factor for integer PID math

// Anti-windup limits
#define MAX_INTEGRAL 5000  // Maximum integral term to prevent windup
#define MIN_ERROR_THRESHOLD 2  // Dead zone to prevent jitter when close to target

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   flPID.TargetTicksPerFrame = 0.0;
   flPID.Encoder = getM1Encoder();
   flPID.PrevEnc = flPID.Encoder;
   flPID.output = 0;
   flPID.PrevInput = 0;
   flPID.ITerm = 0;

   frPID.TargetTicksPerFrame = 0.0;
   frPID.Encoder = getM2Encoder();
   frPID.PrevEnc = frPID.Encoder;
   frPID.output = 0;
   frPID.PrevInput = 0;
   frPID.ITerm = 0;

   rlPID.TargetTicksPerFrame = 0.0;
   rlPID.Encoder = getM3Encoder();
   rlPID.PrevEnc = rlPID.Encoder;
   rlPID.output = 0;
   rlPID.PrevInput = 0;
   rlPID.ITerm = 0;

   rrPID.TargetTicksPerFrame = 0.0;
   rrPID.Encoder = getM4Encoder();
   rrPID.PrevEnc = rrPID.Encoder;
   rrPID.output = 0;
   rrPID.PrevInput = 0;
   rrPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID_4motor(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  // Dead zone: if error is very small, set it to zero to prevent jitter
  if (abs(Perror) < MIN_ERROR_THRESHOLD) {
    Perror = 0;
  }

  // Calculate PID output
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  // Add previous output for incremental control
  output += p->output;
  
  // Clamp output to motor limits
  if (output >= MOTOR_MAX_SPEED)
    output = MOTOR_MAX_SPEED;
  else if (output <= -MOTOR_MAX_SPEED)
    output = -MOTOR_MAX_SPEED;
  else {
    // Only accumulate integral when not saturated (anti-windup)
    p->ITerm += Ki * Perror;
    
    // Clamp integral term to prevent windup
    if (p->ITerm > MAX_INTEGRAL)
      p->ITerm = MAX_INTEGRAL;
    else if (p->ITerm < -MAX_INTEGRAL)
      p->ITerm = -MAX_INTEGRAL;
  }

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  flPID.Encoder = getM1Encoder();
  frPID.Encoder = getM2Encoder();
  rlPID.Encoder = getM3Encoder();
  rrPID.Encoder = getM4Encoder();

  if (!moving){
    if (flPID.PrevInput != 0 || frPID.PrevInput != 0 || rlPID.PrevInput != 0 || rrPID.PrevInput != 0) resetPID();
    return;
  }

  doPID_4motor(&flPID);
  doPID_4motor(&frPID);
  doPID_4motor(&rlPID);
  doPID_4motor(&rrPID);

  setMotorSpeedsTB6612(flPID.output, frPID.output, rlPID.output, rrPID.output);
}