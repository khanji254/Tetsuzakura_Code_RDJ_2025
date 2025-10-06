/*********************************************************************
 * PID Auto-Tuner v2 - WITH VALIDATION
 * 
 * Features:
 * - Step response tuning
 * - PID validation with real closed-loop control
 * - Performance scoring
 * 
 * Commands:
 * - D         Quick diagnostics
 * - T<1-4>    Quick motor test (PWM sweep)
 * - E         Read encoders
 * - S         Start auto-tuning
 * - V         Validate PID values (after tuning)
 * - C         Custom PID test (enter your own values)
 * - s         Stop
 * - H         Help
 *********************************************************************/

#include <Arduino.h>

#define BAUDRATE 57600

// ========== PIN DEFINITIONS ==========
#define M1_PWM   7
#define M1_IN1   22
#define M1_IN2   23

#define M2_PWM   5
#define M2_IN1   25
#define M2_IN2   24

#define M3_PWM   6
#define M3_IN1   26
#define M3_IN2   27

#define M4_PWM   9
#define M4_IN1   29
#define M4_IN2   28

#define MOTOR_STBY 40

#define ENC1_A_PIN 2
#define ENC1_B_PIN 30
#define ENC2_A_PIN 18
#define ENC2_B_PIN 31
#define ENC3_A_PIN 19
#define ENC3_B_PIN 32
#define ENC4_A_PIN 3
#define ENC4_B_PIN 34

// ========== CONFIGURATION ==========
#define TICKS_PER_REV 285
#define UPDATE_INTERVAL 50
#define TEST_SPEEDS 3
#define TEST_DURATION 8000

// ========== ENCODER VARIABLES ==========
volatile long encCount1 = 0;
volatile long encCount2 = 0;
volatile long encCount3 = 0;
volatile long encCount4 = 0;

// ========== ENCODER ISRs ==========
void ISR_enc1() { 
  bool b = digitalRead(ENC1_B_PIN); 
  if (b) encCount1++; 
  else encCount1--;
}

void ISR_enc2() { 
  bool b = digitalRead(ENC2_B_PIN); 
  if (b) encCount2++; 
  else encCount2--;
}

void ISR_enc3() { 
  bool b = digitalRead(ENC3_B_PIN); 
  if (b) encCount3++; 
  else encCount3--;
}

void ISR_enc4() { 
  bool b = digitalRead(ENC4_B_PIN); 
  if (b) encCount4++; 
  else encCount4--;
}

// ========== ENCODER FUNCTIONS ==========
void initializeEncoders() {
  pinMode(ENC1_A_PIN, INPUT_PULLUP); 
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP); 
  pinMode(ENC2_B_PIN, INPUT_PULLUP);
  pinMode(ENC3_A_PIN, INPUT_PULLUP); 
  pinMode(ENC3_B_PIN, INPUT_PULLUP);
  pinMode(ENC4_A_PIN, INPUT_PULLUP); 
  pinMode(ENC4_B_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), ISR_enc1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), ISR_enc2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3_A_PIN), ISR_enc3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC4_A_PIN), ISR_enc4, RISING);
}

long getEncoder(int motor) {
  long temp;
  noInterrupts();
  switch(motor) {
    case 0: temp = encCount1; break;
    case 1: temp = -encCount2; break;
    case 2: temp = encCount3; break;
    case 3: temp = -encCount4; break;
    default: temp = 0;
  }
  interrupts();
  return temp;
}

void resetEncoders() {
  noInterrupts();
  encCount1 = encCount2 = encCount3 = encCount4 = 0;
  interrupts();
}

// ========== MOTOR DRIVER ==========
void initMotorControllerTB6612() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);
}

void setMotorSpeed(int motor, int speed) {
  int pwmPin, in1Pin, in2Pin;
  
  switch(motor) {
    case 0: pwmPin = M1_PWM; in1Pin = M1_IN1; in2Pin = M1_IN2; break;
    case 1: pwmPin = M2_PWM; in1Pin = M2_IN1; in2Pin = M2_IN2; break;
    case 2: pwmPin = M3_PWM; in1Pin = M3_IN1; in2Pin = M3_IN2; break;
    case 3: pwmPin = M4_PWM; in1Pin = M4_IN1; in2Pin = M4_IN2; break;
    default: return;
  }
  
  if (speed >= 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  
  analogWrite(pwmPin, min(abs(speed), 255));
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) setMotorSpeed(i, 0);
}

// ========== UTILITY FUNCTIONS ==========
char readChar() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c != ' ' && c != '\n' && c != '\r' && c != '\t') {
        return c;
      }
    }
  }
}

void clearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

float readFloat() {
  clearSerialBuffer();
  while (!Serial.available());
  return Serial.parseFloat();
}

// ========== PID TUNING STRUCTURE ==========
typedef struct {
  float targetSpeed;
  float currentSpeed;
  long lastEncoder;
  unsigned long lastTime;
  
  // PID variables
  float Kp, Ki, Kd;
  float integral;
  float lastError;
  int output;
  
  // Tuning data
  int testPWM;
  float avgSpeed;
  int samplesCount;
  float totalSpeed;
  float maxSpeed;
  float minSpeed;
  float settleTime;
  bool hasSettled;
  
  // Validation metrics
  float totalAbsError;
  float maxError;
  float overshoot;
  int oscillationCount;
  float steadyStateError;
  
  float bestScore;
} MotorTuning;

MotorTuning motors[4];

bool autoTuning = false;
bool validating = false;
int currentMotor = 0;
int currentSpeedTest = 0;
unsigned long testStartTime = 0;
const float testSpeeds[TEST_SPEEDS] = {50, 100, 150};

// ========== PID CONTROLLER ==========
int computePID(int motor, float targetSpeed, float currentSpeed, float dt) {
  float error = targetSpeed - currentSpeed;
  
  // Proportional
  float P = motors[motor].Kp * error;
  
  // Integral (with anti-windup)
  motors[motor].integral += error * dt;
  motors[motor].integral = constrain(motors[motor].integral, -100, 100);
  float I = motors[motor].Ki * motors[motor].integral;
  
  // Derivative
  float D = 0;
  if (dt > 0) {
    D = motors[motor].Kd * (error - motors[motor].lastError) / dt;
  }
  
  motors[motor].lastError = error;
  
  // Compute output
  float output = P + I + D;
  return (int)constrain(output, -255, 255);
}

void resetPID(int motor) {
  motors[motor].integral = 0;
  motors[motor].lastError = 0;
  motors[motor].output = 0;
}

// ========== STEP RESPONSE TEST ==========
void updateSpeed(int motor) {
  unsigned long currentTime = millis();
  long currentEncoder = getEncoder(motor);
  
  if (motors[motor].lastTime == 0) {
    motors[motor].lastTime = currentTime;
    motors[motor].lastEncoder = currentEncoder;
    return;
  }
  
  unsigned long dt = currentTime - motors[motor].lastTime;
  if (dt < UPDATE_INTERVAL) return;
  
  long encoderDiff = currentEncoder - motors[motor].lastEncoder;
  motors[motor].currentSpeed = (encoderDiff * 1000.0) / dt;
  
  motors[motor].lastTime = currentTime;
  motors[motor].lastEncoder = currentEncoder;
}

void initMotorTest(int motor) {
  motors[motor].targetSpeed = 0;
  motors[motor].currentSpeed = 0;
  motors[motor].lastEncoder = getEncoder(motor);
  motors[motor].lastTime = 0;
  motors[motor].testPWM = 0;
  motors[motor].avgSpeed = 0;
  motors[motor].samplesCount = 0;
  motors[motor].totalSpeed = 0;
  motors[motor].maxSpeed = 0;
  motors[motor].minSpeed = 9999;
  motors[motor].settleTime = 0;
  motors[motor].hasSettled = false;
  motors[motor].bestScore = 999999;
  
  // Validation metrics
  motors[motor].totalAbsError = 0;
  motors[motor].maxError = 0;
  motors[motor].overshoot = 0;
  motors[motor].oscillationCount = 0;
  motors[motor].steadyStateError = 0;
  
  resetPID(motor);
}

void startStepTest(int motor, float targetSpeed) {
  Serial.println(F("\n========================================"));
  Serial.print(F("Testing Motor "));
  Serial.print(motor + 1);
  Serial.print(F(" - Target: "));
  Serial.print(targetSpeed);
  Serial.println(F(" ticks/sec"));
  Serial.println(F("========================================"));
  
  initMotorTest(motor);
  motors[motor].targetSpeed = targetSpeed;
  
  motors[motor].testPWM = (int)((targetSpeed / 240.0) * 255.0);
  motors[motor].testPWM = constrain(motors[motor].testPWM, 30, 255);
  
  Serial.print(F("Starting PWM: "));
  Serial.println(motors[motor].testPWM);
  
  setMotorSpeed(motor, motors[motor].testPWM);
  testStartTime = millis();
}

void processStepTest(int motor) {
  updateSpeed(motor);
  
  float speed = motors[motor].currentSpeed;
  float target = motors[motor].targetSpeed;
  
  motors[motor].samplesCount++;
  motors[motor].totalSpeed += speed;
  
  if (speed > motors[motor].maxSpeed) {
    motors[motor].maxSpeed = speed;
  }
  if (speed < motors[motor].minSpeed && speed > 0) {
    motors[motor].minSpeed = speed;
  }
  
  float error = abs(speed - target);
  float errorPercent = (error / target) * 100.0;
  
  if (!motors[motor].hasSettled && errorPercent < 10.0) {
    if (motors[motor].settleTime == 0) {
      motors[motor].settleTime = millis() - testStartTime;
      motors[motor].hasSettled = true;
    }
  } else if (errorPercent >= 15.0) {
    motors[motor].settleTime = 0;
    motors[motor].hasSettled = false;
  }
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print(F("Speed: "));
    Serial.print(speed, 1);
    Serial.print(F(" / "));
    Serial.print(target, 1);
    Serial.print(F(" | PWM: "));
    Serial.print(motors[motor].testPWM);
    Serial.print(F(" | Error: "));
    Serial.print(errorPercent, 1);
    Serial.print(F("%"));
    Serial.println(motors[motor].hasSettled ? " [SETTLED]" : "");
    lastPrint = millis();
  }
}

void finishStepTest(int motor) {
  setMotorSpeed(motor, 0);
  
  motors[motor].avgSpeed = motors[motor].totalSpeed / motors[motor].samplesCount;
  
  Serial.println(F("\n--- Test Results ---"));
  Serial.print(F("PWM Used:      ")); Serial.println(motors[motor].testPWM);
  Serial.print(F("Target Speed:  ")); Serial.println(motors[motor].targetSpeed, 1);
  Serial.print(F("Avg Speed:     ")); Serial.println(motors[motor].avgSpeed, 1);
  Serial.print(F("Max Speed:     ")); Serial.println(motors[motor].maxSpeed, 1);
  Serial.print(F("Settled:       ")); Serial.println(motors[motor].hasSettled ? "YES" : "NO");
  
  if (motors[motor].hasSettled) {
    Serial.print(F("Settle Time:   ")); 
    Serial.print(motors[motor].settleTime);
    Serial.println(F(" ms"));
  }
  
  float speedError = abs(motors[motor].avgSpeed - motors[motor].targetSpeed);
  float overshoot = max(0.0f, motors[motor].maxSpeed - motors[motor].targetSpeed);
  
  Serial.print(F("Speed Error:   ")); Serial.println(speedError, 1);
  Serial.print(F("Overshoot:     ")); Serial.println(overshoot, 1);
  Serial.println();
}

void calculatePIDFromStepResponse(int motor) {
  float target = motors[motor].targetSpeed;
  float avgSpeed = motors[motor].avgSpeed;
  float overshoot = max(0.0f, motors[motor].maxSpeed - target);
  float steadyStateError = abs(avgSpeed - target);
  
  float Ku = motors[motor].testPWM / target;
  float tau = motors[motor].hasSettled ? (motors[motor].settleTime / 1000.0) : 0.5;
  
  motors[motor].Kp = 0.6 * Ku;
  motors[motor].Ki = 1.2 * Ku / tau;
  motors[motor].Kd = 0.075 * Ku * tau;
  
  motors[motor].bestScore = steadyStateError + (overshoot * 2.0) + (tau * 10.0);
  
  Serial.println(F("--- Calculated PID ---"));
  Serial.print(F("Kp = ")); Serial.println(motors[motor].Kp, 2);
  Serial.print(F("Ki = ")); Serial.println(motors[motor].Ki, 2);
  Serial.print(F("Kd = ")); Serial.println(motors[motor].Kd, 2);
  Serial.print(F("Score: ")); Serial.println(motors[motor].bestScore, 2);
  Serial.println();
}

void processAutoTuning() {
  processStepTest(currentMotor);
  
  if (millis() - testStartTime >= TEST_DURATION) {
    finishStepTest(currentMotor);
    calculatePIDFromStepResponse(currentMotor);
    
    delay(2000);
    
    currentSpeedTest++;
    
    if (currentSpeedTest >= TEST_SPEEDS) {
      currentSpeedTest = 0;
      currentMotor++;
      
      if (currentMotor >= 4) {
        finishAutoTuning();
        return;
      }
    }
    
    startStepTest(currentMotor, testSpeeds[currentSpeedTest]);
  }
}

void finishAutoTuning() {
  autoTuning = false;
  
  Serial.println(F("\n========================================"));
  Serial.println(F("AUTO-TUNING COMPLETE!"));
  Serial.println(F("========================================\n"));
  
  for (int i = 0; i < 4; i++) {
    Serial.print(F("Motor "));
    Serial.print(i + 1);
    Serial.print(F(": Kp="));
    Serial.print(motors[i].Kp, 2);
    Serial.print(F(" Ki="));
    Serial.print(motors[i].Ki, 2);
    Serial.print(F(" Kd="));
    Serial.print(motors[i].Kd, 2);
    Serial.print(F(" (Score="));
    Serial.print(motors[i].bestScore, 2);
    Serial.println(F(")"));
  }
  
  float avgKp = 0, avgKi = 0, avgKd = 0;
  for (int i = 0; i < 4; i++) {
    avgKp += motors[i].Kp;
    avgKi += motors[i].Ki;
    avgKd += motors[i].Kd;
  }
  avgKp /= 4.0;
  avgKi /= 4.0;
  avgKd /= 4.0;
  
  Serial.println(F("\n========================================"));
  Serial.println(F("Recommended values for diff_controller.h:"));
  Serial.println(F("========================================"));
  Serial.print(F("int Kp = ")); Serial.print((int)(avgKp * 10)); Serial.println(F(";"));
  Serial.print(F("int Ki = ")); Serial.print((int)(avgKi * 10)); Serial.println(F(";"));
  Serial.print(F("int Kd = ")); Serial.print((int)(avgKd * 10)); Serial.println(F(";"));
  Serial.println(F("int Ko = 10;  // Note: Values are scaled by 10x"));
  Serial.println(F("========================================"));
  Serial.println(F("\n>>> Press 'V' to VALIDATE these PID values <<<\n"));
}

// ========== PID VALIDATION ==========
void startPIDValidation(int motor, float targetSpeed) {
  Serial.println(F("\n========================================"));
  Serial.print(F("VALIDATING Motor "));
  Serial.print(motor + 1);
  Serial.print(F(" with PID Control"));
  Serial.println(F("\n========================================"));
  Serial.print(F("Target Speed: ")); Serial.print(targetSpeed); Serial.println(F(" ticks/sec"));
  Serial.print(F("PID: Kp=")); Serial.print(motors[motor].Kp, 2);
  Serial.print(F(" Ki=")); Serial.print(motors[motor].Ki, 2);
  Serial.print(F(" Kd=")); Serial.println(motors[motor].Kd, 2);
  Serial.println(F("========================================\n"));
  
  initMotorTest(motor);
  motors[motor].targetSpeed = targetSpeed;
  testStartTime = millis();
}

void processPIDValidation(int motor) {
  updateSpeed(motor);
  
  float dt = UPDATE_INTERVAL / 1000.0;  // Convert to seconds
  
  // Compute PID control
  int pwmOutput = computePID(motor, motors[motor].targetSpeed, motors[motor].currentSpeed, dt);
  motors[motor].output = pwmOutput;
  setMotorSpeed(motor, pwmOutput);
  
  // Collect metrics
  float error = motors[motor].targetSpeed - motors[motor].currentSpeed;
  float absError = abs(error);
  
  motors[motor].samplesCount++;
  motors[motor].totalAbsError += absError;
  motors[motor].totalSpeed += motors[motor].currentSpeed;
  
  if (absError > motors[motor].maxError) {
    motors[motor].maxError = absError;
  }
  
  if (motors[motor].currentSpeed > motors[motor].maxSpeed) {
    motors[motor].maxSpeed = motors[motor].currentSpeed;
  }
  
  // Check for oscillations
  static float lastErr = 0;
  if ((lastErr > 0 && error < 0) || (lastErr < 0 && error > 0)) {
    motors[motor].oscillationCount++;
  }
  lastErr = error;
  
  // Check settling
  float errorPercent = (absError / motors[motor].targetSpeed) * 100.0;
  if (!motors[motor].hasSettled && errorPercent < 5.0) {  // Tighter tolerance for validation
    if (motors[motor].settleTime == 0) {
      motors[motor].settleTime = millis() - testStartTime;
      motors[motor].hasSettled = true;
    }
  }
  
  // Print status
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print(F("Speed: "));
    Serial.print(motors[motor].currentSpeed, 1);
    Serial.print(F(" / "));
    Serial.print(motors[motor].targetSpeed, 1);
    Serial.print(F(" | PWM: "));
    Serial.print(pwmOutput);
    Serial.print(F(" | Error: "));
    Serial.print(error, 1);
    Serial.print(F(" ("));
    Serial.print(errorPercent, 1);
    Serial.print(F("%)"));
    
    if (motors[motor].hasSettled) {
      Serial.print(F(" [SETTLED]"));
    }
    Serial.println();
    
    lastPrint = millis();
  }
}

void finishPIDValidation(int motor) {
  setMotorSpeed(motor, 0);
  
  float avgSpeed = motors[motor].totalSpeed / motors[motor].samplesCount;
  float avgError = motors[motor].totalAbsError / motors[motor].samplesCount;
  motors[motor].steadyStateError = abs(avgSpeed - motors[motor].targetSpeed);
  motors[motor].overshoot = max(0.0f, motors[motor].maxSpeed - motors[motor].targetSpeed);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("VALIDATION RESULTS"));
  Serial.println(F("========================================"));
  Serial.print(F("Target Speed:       ")); Serial.println(motors[motor].targetSpeed, 1);
  Serial.print(F("Average Speed:      ")); Serial.println(avgSpeed, 1);
  Serial.print(F("Steady-State Error: ")); Serial.println(motors[motor].steadyStateError, 1);
  Serial.print(F("Max Error:          ")); Serial.println(motors[motor].maxError, 1);
  Serial.print(F("Avg Error:          ")); Serial.println(avgError, 1);
  Serial.print(F("Overshoot:          ")); Serial.println(motors[motor].overshoot, 1);
  Serial.print(F("Oscillations:       ")); Serial.println(motors[motor].oscillationCount);
  
  if (motors[motor].hasSettled) {
    Serial.print(F("Settle Time:        ")); 
    Serial.print(motors[motor].settleTime);
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("Settle Time:        DID NOT SETTLE"));
  }
  
  // Performance grading
  Serial.println(F("\n--- Performance Analysis ---"));
  
  float errorPercent = (motors[motor].steadyStateError / motors[motor].targetSpeed) * 100.0;
  Serial.print(F("Accuracy: "));
  if (errorPercent < 5.0) {
    Serial.println(F("EXCELLENT (< 5% error)"));
  } else if (errorPercent < 10.0) {
    Serial.println(F("GOOD (5-10% error)"));
  } else if (errorPercent < 20.0) {
    Serial.println(F("FAIR (10-20% error)"));
  } else {
    Serial.println(F("POOR (> 20% error) - Consider re-tuning"));
  }
  
  Serial.print(F("Stability: "));
  if (motors[motor].oscillationCount < 5) {
    Serial.println(F("STABLE (minimal oscillations)"));
  } else if (motors[motor].oscillationCount < 15) {
    Serial.println(F("MODERATE (some oscillations)"));
  } else {
    Serial.println(F("UNSTABLE (excessive oscillations) - Reduce Ki/Kd"));
  }
  
  Serial.print(F("Response: "));
  if (motors[motor].hasSettled) {
    if (motors[motor].settleTime < 1000) {
      Serial.println(F("FAST (< 1 sec)"));
    } else if (motors[motor].settleTime < 2000) {
      Serial.println(F("GOOD (1-2 sec)"));
    } else {
      Serial.println(F("SLOW (> 2 sec) - Increase Kp"));
    }
  } else {
    Serial.println(F("FAILED TO SETTLE - Adjust PID gains"));
  }
  
  Serial.print(F("Overshoot: "));
  float overshootPercent = (motors[motor].overshoot / motors[motor].targetSpeed) * 100.0;
  if (overshootPercent < 10.0) {
    Serial.println(F("LOW (< 10%) - Excellent"));
  } else if (overshootPercent < 25.0) {
    Serial.println(F("MODERATE (10-25%) - Acceptable"));
  } else {
    Serial.println(F("HIGH (> 25%) - Reduce Kp, increase Kd"));
  }
  
  // Overall score
  float score = avgError + (motors[motor].overshoot * 2.0) + (motors[motor].oscillationCount * 5.0);
  Serial.print(F("\nOverall Score: "));
  Serial.print(score, 2);
  Serial.println(F(" (lower is better)"));
  
  Serial.println(F("========================================\n"));
}

void runFullValidation() {
  Serial.println(F("\n========================================"));
  Serial.println(F("FULL PID VALIDATION"));
  Serial.println(F("========================================"));
  Serial.println(F("Testing all 4 motors at 3 speeds each"));
  Serial.println(F("This will take ~2 minutes"));
  Serial.println(F("========================================\n"));
  
  validating = true;
  currentMotor = 0;
  currentSpeedTest = 0;
  
  startPIDValidation(0, testSpeeds[0]);
}

void processValidation() {
  processPIDValidation(currentMotor);
  
  if (millis() - testStartTime >= TEST_DURATION) {
    finishPIDValidation(currentMotor);
    
    delay(2000);
    
    currentSpeedTest++;
    
    if (currentSpeedTest >= TEST_SPEEDS) {
      currentSpeedTest = 0;
      currentMotor++;
      
      if (currentMotor >= 4) {
        finishValidation();
        return;
      }
    }
    
    startPIDValidation(currentMotor, testSpeeds[currentSpeedTest]);
  }
}

void finishValidation() {
  validating = false;
  
  Serial.println(F("\n========================================"));
  Serial.println(F("VALIDATION COMPLETE!"));
  Serial.println(F("========================================\n"));
  
  Serial.println(F("Summary of all motors:"));
  for (int i = 0; i < 4; i++) {
    Serial.print(F("Motor ")); Serial.print(i + 1);
    Serial.print(F(": PID("));
    Serial.print(motors[i].Kp, 2); Serial.print(F(", "));
    Serial.print(motors[i].Ki, 2); Serial.print(F(", "));
    Serial.print(motors[i].Kd, 2); Serial.println(F(")"));
  }
  
  Serial.println(F("\nIf results are good, use these values in diff_controller.h"));
  Serial.println(F("If not, press 'C' to try custom PID values"));
  Serial.println(F("========================================\n"));
}

// ========== CUSTOM PID TEST ==========
void runCustomPIDTest() {
  Serial.println(F("\n========================================"));
  Serial.println(F("CUSTOM PID TEST"));
  Serial.println(F("========================================"));
  
  Serial.println(F("Enter motor number (1-4):"));
  clearSerialBuffer();
  char c = readChar();
  int motor = c - '0' - 1;
  
  if (motor < 0 || motor > 3) {
    Serial.println(F("Invalid motor"));
    return;
  }
  
  Serial.println(F("Enter Kp:"));
  float kp = readFloat();
  
  Serial.println(F("Enter Ki:"));
  float ki = readFloat();
  
  Serial.println(F("Enter Kd:"));
  float kd = readFloat();
  
  Serial.println(F("Enter target speed (ticks/sec):"));
  float target = readFloat();
  
  motors[motor].Kp = kp;
  motors[motor].Ki = ki;
  motors[motor].Kd = kd;
  
  Serial.print(F("\nTesting Motor ")); Serial.print(motor + 1);
  Serial.print(F(" with Kp=")); Serial.print(kp, 2);
  Serial.print(F(" Ki=")); Serial.print(ki, 2);
  Serial.print(F(" Kd=")); Serial.println(kd, 2);
  
  validating = true;
  currentMotor = motor;
  currentSpeedTest = 0;
  
  startPIDValidation(motor, target);
  
  // Run for 8 seconds
  while (millis() - testStartTime < TEST_DURATION) {
    processPIDValidation(motor);
  }
  
  finishPIDValidation(motor);
  validating = false;
}

// ========== QUICK MOTOR TEST ==========
void quickMotorTest(int motor) {
  Serial.println(F("\n========================================"));
  Serial.print(F("Quick Motor Test - Motor "));
  Serial.print(motor + 1);
  Serial.println(F("\n========================================"));
  
  resetEncoders();
  
  const int testPWMs[] = {80, 120, 160, 200, 255};
  const int numTests = 5;
  
  for (int i = 0; i < numTests; i++) {
    int pwm = testPWMs[i];
    Serial.print(F("\nTesting PWM "));
    Serial.println(pwm);
    
    setMotorSpeed(motor, pwm);
    long startEnc = getEncoder(motor);
    unsigned long startTime = millis();
    
    delay(2000);
    
    long endEnc = getEncoder(motor);
    unsigned long endTime = millis();
    
    float dt = (endTime - startTime) / 1000.0;
    float ticksPerSec = (endEnc - startEnc) / dt;
    
    Serial.print(F("  Speed: "));
    Serial.print(ticksPerSec, 1);
    Serial.println(F(" ticks/sec"));
    
    setMotorSpeed(motor, 0);
    delay(500);
  }
  
  Serial.println(F("\n========================================\n"));
}

// ========== COMMAND PROCESSING ==========
void processCommand(char cmd) {
  switch(cmd) {
    case 'D':
    case 'd':
      Serial.println(F("\nQuick diagnostics..."));
      Serial.print(F("STBY: "));
      Serial.println(digitalRead(MOTOR_STBY) ? "OK" : "FAIL");
      break;
      
    case 'T':
    case 't':
      {
        Serial.println(F("Enter motor number (1-4):"));
        clearSerialBuffer();
        char c = readChar();
        int motor = c - '0' - 1;
        
        if (motor < 0 || motor > 3) {
          Serial.println(F("Invalid motor"));
          return;
        }
        
        quickMotorTest(motor);
      }
      break;
      
    case 'E':
    case 'e':
      Serial.println(F("\nEncoder Values:"));
      for (int i = 0; i < 4; i++) {
        Serial.print(F("M")); Serial.print(i + 1);
        Serial.print(F(": ")); Serial.println(getEncoder(i));
      }
      Serial.println();
      break;
      
    case 'S':
      if (autoTuning || validating) {
        Serial.println(F("Already running!"));
        return;
      }
      
      Serial.println(F("\nStarting auto-tuning..."));
      resetEncoders();
      autoTuning = true;
      currentMotor = 0;
      currentSpeedTest = 0;
      startStepTest(0, testSpeeds[0]);
      break;
      
    case 'V':
    case 'v':
      if (autoTuning || validating) {
        Serial.println(F("Already running!"));
        return;
      }
      
      resetEncoders();
      runFullValidation();
      break;
      
    case 'C':
    case 'c':
      if (autoTuning || validating) {
        Serial.println(F("Already running!"));
        return;
      }
      
      resetEncoders();
      runCustomPIDTest();
      break;
      
    case 's':
      autoTuning = false;
      validating = false;
      stopAllMotors();
      Serial.println(F("Stopped"));
      break;
      
    case 'H':
    case 'h':
      Serial.println(F("\n========================================"));
      Serial.println(F("PID Auto-Tuner v2 - Commands"));
      Serial.println(F("========================================"));
      Serial.println(F("D - Quick diagnostics"));
      Serial.println(F("T - Quick motor test (PWM sweep)"));
      Serial.println(F("E - Read encoders"));
      Serial.println(F("S - Start auto-tuning"));
      Serial.println(F("V - Validate PID (after tuning)"));
      Serial.println(F("C - Custom PID test"));
      Serial.println(F("s - Stop/Emergency stop"));
      Serial.println(F("H - Help"));
      Serial.println(F("========================================\n"));
      break;
      
    default:
      if (!autoTuning && !validating) {
        Serial.println(F("Unknown. Press 'H' for help"));
      }
      break;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(BAUDRATE);
  delay(500);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("PID Auto-Tuner v2 - WITH VALIDATION"));
  Serial.println(F("========================================"));
  
  initMotorControllerTB6612();
  initializeEncoders();
  resetEncoders();
  
  for (int i = 0; i < 4; i++) {
    initMotorTest(i);
  }
  
  Serial.println(F("\nWorkflow:"));
  Serial.println(F("  1. Press 'S' to auto-tune"));
  Serial.println(F("  2. Press 'V' to validate"));
  Serial.println(F("  3. Press 'C' to fine-tune"));
  Serial.println(F("\nPress 'H' for all commands"));
  Serial.println(F("========================================\n"));
}

// ========== MAIN LOOP ==========
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != ' ' && cmd != '\n' && cmd != '\r' && cmd != '\t') {
      processCommand(cmd);
    }
  }
  
  if (autoTuning) {
    processAutoTuning();
  } else if (validating) {
    processValidation();
  }
}