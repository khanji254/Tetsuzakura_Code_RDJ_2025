/* Functions and type-defs for PID control (4-motor version).

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#include "config.h"
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

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID_4motor(){
   flPID.TargetTicksPerFrame = 0.0;
   flPID.Encoder = readEncoder(FRONT_LEFT);
   flPID.PrevEnc = flPID.Encoder;
   flPID.output = 0;
   flPID.PrevInput = 0;
   flPID.ITerm = 0;

   frPID.TargetTicksPerFrame = 0.0;
   frPID.Encoder = readEncoder(FRONT_RIGHT);
   frPID.PrevEnc = frPID.Encoder;
   frPID.output = 0;
   frPID.PrevInput = 0;
   frPID.ITerm = 0;

   rlPID.TargetTicksPerFrame = 0.0;
   rlPID.Encoder = readEncoder(REAR_LEFT);
   rlPID.PrevEnc = rlPID.Encoder;
   rlPID.output = 0;
   rlPID.PrevInput = 0;
   rlPID.ITerm = 0;

   rrPID.TargetTicksPerFrame = 0.0;
   rrPID.Encoder = readEncoder(REAR_RIGHT);
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

  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  if (output >= MOTOR_MAX_SPEED)
    output = MOTOR_MAX_SPEED;
  else if (output <= -MOTOR_MAX_SPEED)
    output = -MOTOR_MAX_SPEED;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID_4motor() {
  flPID.Encoder = readEncoder(FRONT_LEFT);
  frPID.Encoder = readEncoder(FRONT_RIGHT);
  rlPID.Encoder = readEncoder(REAR_LEFT);
  rrPID.Encoder = readEncoder(REAR_RIGHT);

  if (!moving){
    if (flPID.PrevInput != 0 || frPID.PrevInput != 0 || rlPID.PrevInput != 0 || rrPID.PrevInput != 0) resetPID_4motor();
    return;
  }

  doPID_4motor(&flPID);
  doPID_4motor(&frPID);
  doPID_4motor(&rlPID);
  doPID_4motor(&rrPID);

  setMotorSpeedsTB6612(flPID.output, frPID.output, rlPID.output, rrPID.output);
}