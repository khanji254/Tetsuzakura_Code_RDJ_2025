#include <Servo.h>

#define N_SERVOS 2
#define CAMERA_SERVO_PIN 52
#define TIPPER_SERVO_PIN 44

Servo servos[N_SERVOS];
int currentAngles[N_SERVOS] = {90, 90};
int stepDelayMs[N_SERVOS] = {10, 10};
int targetAngles[N_SERVOS] = {90, 90};
unsigned long lastMoveTime[N_SERVOS] = {0, 0};

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  servos[0].attach(CAMERA_SERVO_PIN);
  servos[1].attach(TIPPER_SERVO_PIN);
  servos[0].write(currentAngles[0]);
  servos[1].write(currentAngles[1]);
  Serial.println("Servo test: send 'w <index> <angle> <speed>' (e.g. w 0 120 10)");
}

void loop() {
  // Handle serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else if (isPrintable(c)) {
      inputBuffer += c;
    }
  }

  // Sweep servos toward their targets at the set speed
  for (int i = 0; i < N_SERVOS; i++) {
    if (currentAngles[i] != targetAngles[i]) {
      unsigned long now = millis();
      if (now - lastMoveTime[i] >= stepDelayMs[i]) {
        if (currentAngles[i] < targetAngles[i]) currentAngles[i]++;
        else if (currentAngles[i] > targetAngles[i]) currentAngles[i]--;
        servos[i].write(currentAngles[i]);
        lastMoveTime[i] = now;
      }
    }
  }
}

void processCommand(const String& cmd) {
  // Command format: w <index> <angle> <speed>
  if (cmd.charAt(0) == 'w') {
    int idx = -1, angle = 90, speed = 10;
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int thirdSpace = cmd.indexOf(' ', secondSpace + 1);

    if (firstSpace > 0 && secondSpace > firstSpace && thirdSpace > secondSpace) {
      idx = cmd.substring(firstSpace + 1, secondSpace).toInt();
      angle = cmd.substring(secondSpace + 1, thirdSpace).toInt();
      speed = cmd.substring(thirdSpace + 1).toInt();
    } else if (firstSpace > 0 && secondSpace > firstSpace) {
      idx = cmd.substring(firstSpace + 1, secondSpace).toInt();
      angle = cmd.substring(secondSpace + 1).toInt();
      speed = 10;
    }

    if (idx >= 0 && idx < N_SERVOS && angle >= 0 && angle <= 180 && speed > 0) {
      targetAngles[idx] = angle;
      stepDelayMs[idx] = speed;
      Serial.print("Servo "); Serial.print(idx);
      Serial.print(" moving to "); Serial.print(angle);
      Serial.print(" deg at "); Serial.print(speed); Serial.println(" ms/deg");
    } else {
      Serial.println("Invalid command. Use: w <index> <angle> <speed>");
    }
  } else {
    Serial.println("Unknown command.");
  }
}