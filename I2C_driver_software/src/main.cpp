#include <Arduino.h>
#include "encoder_wrapper.h"
#include "motor_wrapper.h"
#include <Wire.h>
#include "I2C_driver_register.h"

#define DRIVER_VERSION 001

#define motorPinA 6 // PD6
#define motorPinB 5 // PD5
#define encoderPinA 4 // PD4
#define encoderPinB 3 // PD3

#define led1 8 // PB0
#define led2 9 // PB1

#define SDA A4 // PC4
#define SCL A5 // PC5

#define addrPin1 13 // PB5
#define addrPin2 12 // PB4
#define addrPin3 11 // PB3


// Prototypes
void I2C_receive(int test);
void I2C_request(void);
void I2C_setup(void);

// Data
union Buffer
{
   unsigned long longNumber;
   byte longBytes[4];
};
Buffer i2cbuffer;

MotorWrapper motor;

uint8_t i2c_read_reg = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, 0);
  digitalWrite(led2, 0);

  motor.init(motorPinA, motorPinB, encoderPinA, encoderPinB);
  motor.setSpd(0);

  // setup I2C slave
  I2C_setup();

  Wire.begin(10);
  Wire.onReceive(I2C_receive);
  Wire.onRequest(I2C_request);
}

void loop() {
  static unsigned long lastMillis = 0;
  if(millis() - lastMillis > 250)
  {
    lastMillis = millis();
    digitalWrite(led1,!digitalRead(led1));
  }
  motor.update();

}

void I2C_receive(int test)
{
  int8_t reg = Wire.read();
  int8_t data = Wire.read();
  Serial.println("Reg = " + String(reg) + " Data = " + String(data));

  switch(reg)
  {
    case drive_mode:
      // motor.setMode(MotorWrapper::eMotorMode::openLoop);
      Serial.println("Set Mode");
      motor.setMode(MotorWrapper::eMotorMode(data));
      break;
    case cmd_speed:
      Serial.println("Set Speed");
      motor.setSpd(data);
      break;
    case cmd_pos:
      Serial.println("Set Pos");
      motor.setPos(data);
      break;
    case i2c_read:
      i2c_read_reg = data;
    break;

  }
}

// Not validated
void I2C_request()
{
  // Serial.println("On Request");
  switch(i2c_read_reg)
  {
    case position:
      // Serial.println("Position");
      i2cbuffer.longNumber = motor.getPos();
      Wire.write(i2cbuffer.longBytes, 4); // send requested bytes
      break;
    case drive_version:
      Wire.write(DRIVER_VERSION);
      break;
  }
  
  // switch(reg)
  // {
  //   case drive_version:
  //     Wire.write(DRIVER_VERSION);
  //     break;
  //   case drive_mode:
  //     Wire.write(1);
  //     break;
  //   case status_drive:
  //     Wire.write(2);
  //     break;
  // }
}

// Not working. PB5 read 1, PB4 read 0, PB3 read 0.
void I2C_setup()
{
  pinMode(addrPin1,INPUT_PULLUP);
  pinMode(addrPin2,INPUT_PULLUP);
  pinMode(addrPin3,INPUT_PULLUP);
  delay(100);
  uint8_t addr = 0;
  if(digitalRead(addrPin1) == 0)
  {
    Serial.print("PB5 = 0");
    addr += 1;
  }
  if(digitalRead(addrPin2) == 0)
  {
    Serial.print("  PB4 = 0");
    addr += 2;
  }
  if(digitalRead(addrPin3) == 0)
  {
    Serial.print("PB3 = 0");
    addr += 4;
  }

  Serial.println("  Addr = " + String(addr));

}


