/*
 * Kritzelbot
 *
 */

#include <AFMotor.h>
#include <stdlib.h>

// distance between both motors (axis) 51 cm
#define AXIS_DISTANCE_X 5120
#define AXIS_DISTANCE_Y 5120

// starting position
// a = b = 10607 --> 1060.7mm
// m2s = 0.7853982

#define START_X 2560
#define START_Y 2000
#define MIN_X 1500
#define MAX_X 4100
#define MIN_Y 2000
#define MAX_Y 4500

// pulley radius 5mm
#define PULLEY_R 50 // This should be set via a command
#define PI 3.14159
// circumference 2*PI*r = 50.2 mm

// 200 steps per rotation
#define STEPS_PER_ROT 200

// command chars
#define CMD_CHAR_LINE_A 'L' // draw an absolute line
#define CMD_CHAR_LINE_R 'l' // move an absolute line
#define CMD_CHAR_MOVE_A 'M' // draw a reletive line
#define CMD_CHAR_MOVE_R 'm' // move a reletive line
#define CMD_CHAR_MOVE_H 'h' // move to home

#define MAX_BUFFER_SIZE 50

float m2s;

AF_Stepper M1(STEPS_PER_ROT, 2); // left
AF_Stepper M2(STEPS_PER_ROT, 1); // right

long currentX = 0;
long currentY = 0;
long stepsM1 = 0;
long stepsM2 = 0;
long targetM1 = 0;
long targetM2 = 0;

char line[MAX_BUFFER_SIZE];

typedef struct {
  char cmd;
  long x;
  long y;
  long targetM1;
  long targetM2;
} command;


void setup() {

  while (!Serial) {
    ; // If there is no serial do nothing.
  }
  Serial.begin(57600);
  Serial.println("#startup");

  // compute mm to steps
  m2s = (2 * PI * PULLEY_R) / STEPS_PER_ROT;

  // compute starting pos
  currentX = START_X;
  currentY = START_Y;
  stepsM1 = computeA(START_X, START_Y) / m2s;
  stepsM2 = computeB(START_X, START_Y) / m2s;
  targetM1 = stepsM1;
  targetM2 = stepsM2;
  Serial.print("#cmd h, x:"); Serial.print(currentX); Serial.print(", y:"); Serial.println(currentY);
  
  M1.setSpeed(20); // 20 rpm - Any faster an the steps seem to get out of sync
  M2.setSpeed(20); // 20 rpm                                   

  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("OK");
}

int dsM1 = 0;
int dsM2 = 0;
int dM1 = 0;
int dM2 = 0;
int err = 0;
int e2 = 0;

void execCmd(char cmd, long x, long y, long tM1, long tM2) {

  int dirM1, dirM2;
  
  // compute deltas
  dM1 = abs(tM1 - stepsM1);
  dM2 = abs(tM2 - stepsM2);
  err = dM1 - dM2;
  // set directions
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

int computeA(long x, long y) {
  return sqrt(x * x + y * y);
}
  
int computeB(long x, long y) {
  long distanceX = AXIS_DISTANCE_X - x;
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

byte parseLine(char *line) {

  byte newWritePtr;
  char tcmd;
  long tx = 0, ty = 0;
  long a, b;
  char buf[10];

  switch (line[0]) {
  case 'm':
  case 'l':
    tcmd = line[0];
    line += 2; // skip command and space
    line = readToken(line, buf, ' ');
    tx = atol(buf) + currentX;
    line = readToken(line, buf, ' ');
    ty = atol(buf) + currentY;
    break;
  case 'M':
  case 'L':
    tcmd = line[0];
    line += 2; // skip command and space
    line = readToken(line, buf, ' ');
    tx = atol(buf);
    line = readToken(line, buf, ' ');
    ty = atol(buf);
    break;
  case 'O':
  case 'o':
    tcmd = line[0];
    tx = currentX;
    ty = currentY;
    break;
  case 'h':
    tcmd = line[0];
    tx = START_X;
    ty = START_Y;
    break;
  default:
    Serial.print("#unknown command: ");
    Serial.println(line[0]);
    return 1;
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

  // compute a and b from x and y
  a = computeA(tx, ty);
  b = computeB(tx, ty);

  currentX = tx;
  currentY = ty;

  execCmd(tcmd, tx, ty, a / m2s, b / m2s);

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

