#include <Arduino.h>
#include "encoder_wrapper.h"
#include "motor_wrapper.h"
#include <Wire.h>

#define motorPinA PD6
#define motorPinB PD5
#define encoderPinA PD4
#define encoderPinB PD3

#define led1 PB0
#define led2 PB1

#define SDA PC4
#define SCL PC5


// Prototypes
void I2C_callback(int data);

// Data
MotorWrapper motor;

struct I2Creg
{
  uint8_t cmd;
  uint8_t data;
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");

  motor.init(motorPinA, motorPinB, encoderPinA, encoderPinB);
  motor.setSpd(0);

  // setup I2C slave
  Wire.begin(10);
  Wire.onReceive(I2C_callback);

}

void loop() {
  motor.update();
}

void I2C_callback(int data)
{
  byte cmd = Wire.read();
  motor.setSpd(cmd);
  Serial.println(cmd);
}


