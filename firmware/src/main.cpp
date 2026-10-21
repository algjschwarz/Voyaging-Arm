#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>

// Protocol (115200 baud, one command per line):
//   P <base> <shoulder> <elbow> <wristpitch> <wristroll> <gripper>
//       Servo commands in degrees, clamped to 0..180. "-" leaves that servo alone.
//   R          release every servo (stop sending pulses, so it goes limp and stops straining)
//   R <0..5>   release one servo
// Silent on success, "ERR <reason>" on a bad line.
// Servos stay released until their first command, so nothing jumps at power-on.
// Prints "READY" once setup is done so the host knows the board has finished resetting.

constexpr int kJoints = 6;
constexpr uint8_t kPins[kJoints] = {3, 5, 6, 9, 10, 11};  // base, shoulder, elbow, wrist pitch, wrist roll, gripper

Servo servos[kJoints];

char line[64];
size_t len = 0;
bool overflow = false;

char* skipSpaces(char* p) {
  while (*p == ' ') ++p;
  return p;
}

void handlePose(char* p) {
  float deg[kJoints];
  bool set[kJoints];
  for (int i = 0; i < kJoints; ++i) {
    p = skipSpaces(p);
    if (*p == '-' && (p[1] == ' ' || p[1] == '\0')) { set[i] = false; ++p; continue; }
    char* end;
    deg[i] = strtod(p, &end);
    if (end == p) { Serial.println("ERR expected 6 angles"); return; }
    set[i] = true;
    p = end;
  }

  for (int i = 0; i < kJoints; ++i) {
    if (!set[i]) continue;
    servos[i].write(constrain(deg[i], 0, 180));  // set the pulse first, so attaching goes straight there
    if (!servos[i].attached()) servos[i].attach(kPins[i]);
  }
}

void handleRelease(char* p) {
  p = skipSpaces(p);
  if (*p == '\0') {
    for (int i = 0; i < kJoints; ++i) servos[i].detach();
    return;
  }
  char* end;
  long i = strtol(p, &end, 10);
  if (end == p || i < 0 || i >= kJoints) { Serial.println("ERR bad servo index"); return; }
  servos[i].detach();
}

void handleLine(char* s) {
  if (s[0] == 'P') handlePose(s + 1);
  else if (s[0] == 'R') handleRelease(s + 1);
  else Serial.println("ERR unknown command");
}

void setup() {
  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  // Only read what has already arrived, so loop() never waits on the host.
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[len] = '\0';
      if (overflow) Serial.println("ERR line too long");
      else if (len > 0) handleLine(line);
      len = 0;
      overflow = false;
    } else if (len < sizeof line - 1) {
      line[len++] = ch;
    } else {
      overflow = true;
    }
  }
}
