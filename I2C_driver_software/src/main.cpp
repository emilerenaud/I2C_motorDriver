#include <Arduino.h>
#include "encoder_wrapper.h"
#include "motor_wrapper.h"
#include <Wire.h>
#include "I2C_driver_register.h"

#define DRIVER_VERSION 001

#define motorPinA PD6
#define motorPinB PD5
#define encoderPinA PD4
#define encoderPinB PD3

#define led1 PB0
#define led2 PB1

#define SDA PC4
#define SCL PC5


// Prototypes
void I2C_receive(int data);
void I2C_request(void);

// Data
MotorWrapper motor;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");

  motor.init(motorPinA, motorPinB, encoderPinA, encoderPinB);
  motor.setSpd(0);

  // setup I2C slave
  Wire.begin(10);
  Wire.onReceive(I2C_receive);
  Wire.onRequest(I2C_request);
}

void loop() {
  motor.update();
}

void I2C_receive(int data)
{
  int reg = Wire.read();
  int data = Wire.read();
  Serial.println("Reg = " + String(reg) + " Data = " + String(data));

  switch(reg)
  {
    case drive_mode:
      motor.setMode(MotorWrapper::eMotorMode::openLoop);
      break;
    case cmd_speed:
      motor.setSpd(data);
      break;
  }
}

void I2C_request()
{
  int reg = Wire.read();

  switch(reg)
  {
    case drive_version:
      Wire.write(DRIVER_VERSION);
      break;
    case drive_mode:
      Wire.write(1);
      break;
    
  }
}


