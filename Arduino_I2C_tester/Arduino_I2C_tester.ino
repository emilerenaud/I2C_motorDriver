#include "Wire.h"

enum I2Creg
{
  drive_version = 0,
  drive_mode = 1,

  cmd_speed = 10,
  cmd_pos = 11,

  status_cpu = 20,
  status_drive = 21

};

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    int cmd = Serial.readString().toInt();
    Serial.println("Reg = " + String(cmd_speed) + " Data = " + String(cmd));
    Wire.beginTransmission(10);
    Wire.write(cmd_speed);
    Wire.write(cmd);
    Wire.endTransmission();
    Serial.println("Data Sent");

  }
}
