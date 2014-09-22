#include <AFMotor.h>

AF_Stepper motorLeft(200, 1);
AF_Stepper motorRight(200, 2);

int MIN = -100;
int MAX = 100;
int X = MIN;
int Y = MIN;

void setup() {
  Serial.begin(9600); // set up Serial library at 9600 bps
  motorRight.setSpeed(10); // 10 rpm
  motorLeft.setSpeed(10); // 10 rpm
}

void right() {
  if (X >= MAX) {
    return;
  }
  motorRight.step(1, FORWARD, MICROSTEP);
  motorLeft.step(1, BACKWARD, MICROSTEP);
  X = X + 1;
}

void left() {
  if (X <= MIN) {
    return;
  }
  motorRight.step(1, BACKWARD, MICROSTEP);
  motorLeft.step(1, FORWARD, MICROSTEP);
  X = X - 1;
}

void up() {
  if (Y <= MIN) {
    return;
  }
  motorRight.step(1, BACKWARD, MICROSTEP); // INTERLEAVE
  motorLeft.step(1, BACKWARD, MICROSTEP);
  Y = Y - 1;
}

void down() {
  if (Y >= MAX) {
    return;
  }
  motorRight.step(1, FORWARD, MICROSTEP);
  motorLeft.step(1, FORWARD, MICROSTEP);
  Y = Y + 1;
}

void loop() {
  if (X >= MAX && Y < MAX) {
    down();
  } else if (X <= MIN && Y > MIN) {
    up();
  } else if (Y >= MAX && X > MIN) {
    left();
  } else if (Y <= MIN && X < MAX) {
    right();
  }
  Serial.print("X=");
  Serial.print(X);
  Serial.print(", Y=");
  Serial.println(Y);
}

