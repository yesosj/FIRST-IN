/*
  DanVI Jetson UART motor control test

  Command format from Jetson:
    M <left_pwm> <right_pwm>\n

  Examples:
    M 120 120    forward
    M -120 -120  backward
    M -80 80     turn left in place
    M 80 -80     turn right in place
    M 0 0        stop

  Arduino Leonardo:
    Serial  = USB debug serial
    Serial1 = hardware UART on D0(RX), D1(TX)
*/

#include "motor_pwm_setting.h"

const long UART_BAUD = 115200;
const int MAX_PWM = 255;
const unsigned long COMMAND_TIMEOUT_MS = 100;

char line[24];
byte lineIndex = 0;
unsigned long lastCommandTime = 0;

void setup() {
  Serial1.begin(UART_BAUD);

  motor_output();
  stopAllMotors();
}

void loop() {
  readUart();

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopAllMotors();
  }
}

void readUart() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      line[lineIndex] = '\0';
      handleCommand(line);
      lineIndex = 0;
    } else if (c != '\r' && lineIndex < sizeof(line) - 1) {
      line[lineIndex++] = c;
    } else if (lineIndex >= sizeof(line) - 1) {
      lineIndex = 0;
    }
  }
}

void handleCommand(char *cmd) {
  int left;
  int right;

  if (sscanf(cmd, "M %d %d", &left, &right) != 2) {
    return;
  }

  left = constrain(left, -MAX_PWM, MAX_PWM);
  right = constrain(right, -MAX_PWM, MAX_PWM);

  setLeftRightMotors(left, right);
  lastCommandTime = millis();
}

void setLeftRightMotors(int left, int right) {
  setMotor(A_F_pin, A_B_pin, F_left_PWM_pin, left);
  setMotor(C_F_pin, C_B_pin, B_left_PWM_pin, left);

  setMotor(B_F_pin, B_B_pin, F_right_PWM_pin, right);
  setMotor(D_F_pin, D_B_pin, B_right_PWM_pin, right);
}

void setMotor(int forwardPin, int backwardPin, int pwmPin, int value) {
  if (value > 0) {
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
    analogWrite(pwmPin, value);
  } else if (value < 0) {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
    analogWrite(pwmPin, -value);
  } else {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopAllMotors() {
  setLeftRightMotors(0, 0);
}