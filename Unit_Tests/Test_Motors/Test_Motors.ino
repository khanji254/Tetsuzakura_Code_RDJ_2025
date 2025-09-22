// === 4WD Robot Motor Test ===
// Arduino Mega + 2x TB6612FNG + 4x 25GA370 DC Motors

// --- Motor driver pins (from wiring plan) ---
// Motor 1 (Front Left) - TB6612 c
#define M1_PWM   7
#define M1_IN1   22
#define M1_IN2   23

// Motor 2 (Front Right) - TB6612  d
#define M2_PWM   5
#define M2_IN1   24
#define M2_IN2   25

// Motor 3 (Rear Left - Motor A) 
#define M3_PWM   6    // ENA pin (speed control)
#define M3_IN1   26   // IN1 pin -> OUT1
#define M3_IN2   27   // IN2 pin -> OUT2

// Motor 4 (Rear Right - Motor B)
#define M4_PWM   9    // ENB pin (speed control)
#define M4_IN1   28   // IN3 pin -> OUT3
#define M4_IN2   29   // IN4 pin -> OUT4

#define MOTOR_STBY 40   // HIGH = enable TB6612 

// === Motor helper functions ===
void setMotor(int pwmPin, int in1, int in2, int speed) {
  // speed > 0 = forward, speed < 0 = reverse, speed = 0 = stop
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// Set all 4 motors at once
void setAllMotors(int m1, int m2, int m3, int m4) {
  setMotor(M1_PWM, M1_IN1, M1_IN2, m1);
  setMotor(M2_PWM, M2_IN1, M2_IN2, m2);
  setMotor(M3_PWM, M3_IN1, M3_IN2, m3);
  setMotor(M4_PWM, M4_IN1, M4_IN2, m4);
}

void setup() {
  // Setup all motor control pins
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT); pinMode(M1_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT); pinMode(M2_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT); pinMode(M3_PWM, OUTPUT);
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT); pinMode(M4_PWM, OUTPUT);

  // Setup STBY and enable drivers
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH); // enable all TB6612s

  Serial.begin(9600);
  Serial.println("4WD Robot Motor Test Start");
}

void loop() {
  int testSpeed = 180; // 0-255 PWM value

  // Forward
  Serial.println("Forward");
  setAllMotors(testSpeed, testSpeed, testSpeed, testSpeed);
  delay(2000);

  // Backward
  Serial.println("Backward");
  setAllMotors(-testSpeed, -testSpeed, -testSpeed, -testSpeed);
  delay(2000);

  // Left turn (left motors stop, right motors forward)
  Serial.println("Left turn");
  setAllMotors(0, testSpeed, 0, testSpeed);
  delay(2000);

  // Right turn (right motors stop, left motors forward)
  Serial.println("Right turn");
  setAllMotors(testSpeed, 0, testSpeed, 0);
  delay(2000);

  // Stop
  Serial.println("Stop");
  setAllMotors(0, 0, 0, 0);
  delay(2000);
}
