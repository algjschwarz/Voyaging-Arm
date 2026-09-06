#include <Arduino.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_pitch;
Servo wrist_roll;
Servo gripper;

void setup() {
  Servo arr[] =
  {base, shoulder, elbow, wrist_pitch, wrist_roll, gripper};
  base.attach(3);
  shoulder.attach(5);
  elbow.attach(6);
  wrist_pitch.attach(9);
  wrist_roll.attach(10);
  gripper.attach(11);
  for (int i = 0; i < 6; i++) {
    arr[i].write(90);
  }
}


void loop() {
}