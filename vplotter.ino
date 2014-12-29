/*
 * Vplotter
 *
 * Based on https://github.com/tinkerlog/Kritzler
 */

#include <AFMotor.h>
#include <Servo.h>
#include <stdlib.h>

// All these should be set via a configuration command
#define AXIS_DISTANCE_XY 5120 // Distance between the center of both motors 51.2 cm
#define START_X 2560 // 25.6 cm
#define START_Y 2000 // 20 cm
#define MIN_X 1500 // 15 cm
#define MAX_X 4100 // 41 cm
#define MIN_Y 2000 // 20 cm
#define MAX_Y 4500 // 45 cm
#define PULLEY_R 50 // Pulley radius 5 mm
#define STEPS_PER_ROT 200 // 200 steps per rotation

// Fixed constants
#define MAX_BUFFER_SIZE 50
#define PI 3.14159 // Circumference 2*PI*r = 50.2 mm

// Command chars
#define CMD_CHAR_LINE_A 'L' // Draw an absolute line
#define CMD_CHAR_LINE_R 'l' // Draw a relative line
#define CMD_CHAR_MOVE_A 'M' // Move an absolute line
#define CMD_CHAR_MOVE_R 'm' // Move a relative line
#define CMD_CHAR_MOVE_H 'h' // Move to home

AF_Stepper M1(STEPS_PER_ROT, 2); // Left
AF_Stepper M2(STEPS_PER_ROT, 1); // Right
Servo PEN; // Pen lifter

// Computed constants
float m2s = (2 * PI * PULLEY_R) / STEPS_PER_ROT; // Compute mm to steps
char line[MAX_BUFFER_SIZE];

// Computed globals
long currentX = 0;
long currentY = 0;
long stepsM1 = 0;
long stepsM2 = 0;

void setup() {

  while (!Serial) {
    ; // If there is no serial do nothing
  }
  Serial.begin(57600);
  Serial.println("#startup");

  // Compute starting position
  currentX = START_X;
  currentY = START_Y;
  stepsM1 = computeLeftPosition(START_X, START_Y) / m2s;
  stepsM2 = computeRightPosition(START_X, START_Y) / m2s;

  Serial.print("#cmd h, x:"); Serial.print(currentX); Serial.print(", y:"); Serial.println(currentY);
  
  M1.setSpeed(20); // 20 rpm - Any faster an the steps seem to get out of sync
  M2.setSpeed(20); // 20 rpm

  PEN.attach(10);
  liftPen();

  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("OK");
}

void moveTo(long x, long y, long tM1, long tM2) {

  long targetM1 = 0;
  long targetM2 = 0;
  int dirM1, dirM2;
  int dsM1 = 0;
  int dsM2 = 0;
  int dM1 = 0;
  int dM2 = 0;
  int err = 0;
  int e2 = 0;

  // Compute deltas
  dM1 = abs(tM1 - stepsM1);
  dM2 = abs(tM2 - stepsM2);
  err = dM1 - dM2;
  // Set directions
  dsM1 = (tM1 > stepsM1) ? +1 : -1;
  dsM2 = (tM2 > stepsM2) ? +1 : -1;
  dirM1 = (tM1 > stepsM1) ? FORWARD : BACKWARD;
  dirM2 = (tM2 > stepsM2) ? BACKWARD : FORWARD;
  targetM1 = tM1;
  targetM2 = tM2;

  while ((stepsM1 != targetM1) && (stepsM2 != targetM2)) {
    e2 = err * 2;
    if (e2 > -dM2) {
      err = err - dM2;
      M1.step(1, dirM1, SINGLE); // SINGLE, DOUBLE, INTERLEAVE, MICROSTEP
      stepsM1 += dsM1;
    }
    if (e2 < dM1) {
      err = err + dM1;
      M2.step(1, dirM2, SINGLE);
      stepsM2 += dsM2;
    }
  }
  
  Serial.println("OK");
}

int computeLeftPosition(long x, long y) {
  return sqrt(x * x + y * y);
}
  
int computeRightPosition(long x, long y) {
  long distanceX = AXIS_DISTANCE_XY - x;
  return sqrt((distanceX * distanceX) + y * y);
}

char *readToken(char *str, char *buf, char delimiter) {
  uint8_t c = 0;
  while (true) {
    c = *str++;
    if ((c == delimiter) || (c == '\0')) {
      break;
    }
    else if (c != ' ') {
      *buf++ = c;
    }
  }
  *buf = '\0';
  return str;
}

void liftPen() {
  // PEN.attach(10);
  PEN.write(0);
  // PEN.detach();
}

void dropPen() {
  // PEN.attach(10);
  PEN.write(90);
  // PEN.detach();
}

byte parseLine(char *line) {

  char tcmd;
  long tx = 0, ty = 0;
  long a, b;
  char buf[10];

  tcmd = line[0];
  line += 2; // Skip command and space

  switch (tcmd) {
  case CMD_CHAR_MOVE_R:
  case CMD_CHAR_LINE_R: // Relative position
    line = readToken(line, buf, ' ');
    tx = atol(buf) + currentX;
    line = readToken(line, buf, ' ');
    ty = atol(buf) + currentY;
    break;
  case CMD_CHAR_MOVE_A:
  case CMD_CHAR_LINE_A: // Absolute position
    line = readToken(line, buf, ' ');
    tx = atol(buf);
    line = readToken(line, buf, ' ');
    ty = atol(buf);
    break;
  case CMD_CHAR_MOVE_H: // Move to the home position
    tx = START_X;
    ty = START_Y;
    break;
  default:
    Serial.print("#unknown command: ");
    Serial.println(line[0]);
    return 1;
  }

  if (tcmd == CMD_CHAR_LINE_A || tcmd == CMD_CHAR_LINE_R) {
    dropPen();
  } else {
    liftPen();
  }

  if (tx < MIN_X) tx = MIN_X;
  if (tx > MAX_X) tx = MAX_X;
  if (ty < MIN_Y) ty = MIN_Y;
  if (ty > MAX_Y) ty = MAX_Y;

  Serial.print("#cmd: ");
  Serial.print(tcmd);
  Serial.print(", x:" );
  Serial.print(tx);
  Serial.print(", y:");
  Serial.println(ty);

  // Compute a and b from x and y
  a = computeLeftPosition(tx, ty);
  b = computeRightPosition(tx, ty);

  currentX = tx;
  currentY = ty;

  // Command [mMlLoOh], Target X, Target Y, ?, ? 
  moveTo(tx, ty, a / m2s, b / m2s);

  return 0;
}

byte readLine(char *line, byte size) {
  byte length = 0;
  char c;
  while (length < size) {
    if (Serial.available()) {
      c = Serial.read();
      length++;
      if ((c == '\r') || (c == '\n')) {
	      *line = '\0';
        break;
      }
      *line++ = c;
    }
  }
  return length;
}

void loop() {
  byte length;
  byte error;
  length = readLine(line, MAX_BUFFER_SIZE);
  if (length > 0) {
    error = parseLine(line);
    if (error) {
//      Serial.println("errored, stopped!");
//      while (true);
    }
  }
}
