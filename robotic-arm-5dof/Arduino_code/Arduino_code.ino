#include <Servo.h>
#include <Stepper.h>

// ---------- Pins (from your schematic) ----------
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 5;
const int FORCE_PIN  = A0;

Servo s1, s2;
Stepper pen(2048, 6, 8, 7, 9);          // IN1, IN3, IN2, IN4

// ---------- Settings to tune ----------
const int   PEN_DIR       = 1;          // 1 = positive steps lower the pen, -1 = reverse
const int   FORCE_TARGET  = 250;        // pressure to hold (ADC 0-1023)
const int   FORCE_CONTACT = 60;         // reading that means "pen touched paper"
const int   MAX_PEN_STEPS = 600;        // safety limit of pen travel
const float Kp = 0.02, Ki = 0.01, Kd = 0.0;
const int   MAX_STEP_PER_LOOP = 4;      // limits how fast the PID can move the pen
const int   DEADBAND      = 15;         // ignore small force errors
const float MAX_SERVO_STEP = 1.0;       // degrees per loop (smooth motion)
const int   LOOP_MS       = 10;

float cur1 = 90, cur2 = 90;
int   penPos = 0;                       // steps below the raised position
bool  penDown = false;
float integ = 0, lastErr = 0;
unsigned long lastT = 0;

float readForce() {
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(FORCE_PIN);
  return sum / 4.0;
}

void writeServo(Servo &s, float deg) {
  deg = constrain(deg, 0, 180);
  s.writeMicroseconds(500 + deg * (2000.0 / 180.0));
}

void stepPen(int steps) {               // steps > 0 means pen goes down
  pen.step(steps * PEN_DIR);
  penPos += steps;
}

void forcePID() {
  if (!penDown) return;
  unsigned long now = millis();
  float dt = (now - lastT) / 1000.0;
  if (dt <= 0) return;
  lastT = now;

  float err = FORCE_TARGET - readForce();      // positive: pressing too lightly
  if (abs(err) < DEADBAND) { lastErr = err; return; }

  integ += err * dt;
  integ = constrain(integ, -200, 200);         // anti-windup
  float d = (err - lastErr) / dt;
  lastErr = err;

  int steps = (int)(Kp * err + Ki * integ + Kd * d);
  steps = constrain(steps, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP);
  if (penPos + steps > MAX_PEN_STEPS) steps = MAX_PEN_STEPS - penPos;
  if (penPos + steps < 0)             steps = -penPos;
  if (steps != 0) stepPen(steps);
}

void lowerPen() {
  if (penDown) return;
  while (readForce() < FORCE_CONTACT && penPos < MAX_PEN_STEPS) stepPen(2);
  penDown = true;
  integ = 0; lastErr = 0; lastT = millis();
}

void raisePen() {
  penDown = false;
  while (penPos > 0) stepPen(-min(20, penPos));
}

void moveServos(float t1, float t2) {
  while (fabs(t1 - cur1) > 0.05 || fabs(t2 - cur2) > 0.05) {
    cur1 += constrain(t1 - cur1, -MAX_SERVO_STEP, MAX_SERVO_STEP);
    cur2 += constrain(t2 - cur2, -MAX_SERVO_STEP, MAX_SERVO_STEP);
    writeServo(s1, cur1);
    writeServo(s2, cur2);
    forcePID();                                // pressure stays controlled while moving
    delay(LOOP_MS);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  s1.attach(SERVO1_PIN);
  s2.attach(SERVO2_PIN);
  writeServo(s1, cur1);
  writeServo(s2, cur2);
  pen.setSpeed(12);
}

void loop() {
  if (Serial.available()) {
    if (Serial.peek() == 'F') {                // calibration: type F to read the force
      Serial.read();
      while (Serial.available()) Serial.read();
      Serial.println(readForce());
      return;
    }
    float q1 = Serial.parseFloat();
    float q2 = Serial.parseFloat();
    int   p  = Serial.parseInt();
    while (Serial.available()) Serial.read();

    if (p == 1) lowerPen(); else raisePen();
    moveServos(q1, q2);
    Serial.println("ok");
  }
  forcePID();                                  // keeps pressure steady when idle
  delay(LOOP_MS);
}