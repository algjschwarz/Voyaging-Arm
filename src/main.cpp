#include <Arduino.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_pitch;
Servo wrist_roll;
Servo gripper;

void setup() {
  Serial.begin(115200);
  base.attach(3);
  shoulder.attach(5);
  elbow.attach(6);
  wrist_pitch.attach(9);
  wrist_roll.attach(10);
  gripper.attach(11);
}


void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    int space = line.indexOf(' ');
    if (space > 0) {
        String name = line.substring(0, space);
        float angle = line.substring(space + 1).toFloat();
        angle = constrain(angle, 0, 180);

        if (name == "base")        { Serial.println("base"); base.write(angle);}
        else if (name == "shoulder") { Serial.println("shoulder"); shoulder.write(angle);}
        else if (name == "elbow")    { Serial.println("elbow"); elbow.write(angle);}
        else if (name == "wristpitch") { Serial.println("wristpitch"); wrist_pitch.write(angle);}
        else if (name == "wristroll")  { Serial.println("wristroll"); wrist_roll.write(angle);}
        else if (name == "gripper")    { Serial.println("gripper"); gripper.write(angle);}
        else {Serial.println("unknown joint"); Serial.println(name); Serial.println(name.length()); return;}
        Serial.print("-> "); Serial.println(angle);
    }
}
}