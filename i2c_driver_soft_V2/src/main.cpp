#include <Arduino.h>
#include <Wire.h>

#define DRIVER_VERSION 002

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

// Variables
uint8_t slaveAdress = 0;

int32_t encoderCount = 0;
int32_t speed = 0;

uint8_t motorSpeed = 0;

uint16_t watchdog = 0;

union Buffer
{
   unsigned long longNumber;
   byte longBytes[4];
} i2cbuffer;

// Prototypes
void IO_setup(void);
void ISR_encoder(void);
void controlMotor(int8_t speed);

uint8_t I2C_setup(void);
void I2C_receiveEvent(int howMany);
void I2C_requestEvent(void);


void setup()
{
  Serial.begin(9600);
  Serial.println("Start I2C driver");

  IO_setup();

  slaveAdress = I2C_setup();
  Wire.begin(slaveAdress);
  Wire.onReceive(I2C_receiveEvent);
  Wire.onRequest(I2C_requestEvent);

}

void loop()
{
  static unsigned long lastMillis = 0;
  static unsigned long watchdogMillis = 0;
  if(millis() - watchdogMillis > 1)
  {
    watchdogMillis = millis();
    watchdog ++;
    if(watchdog > 250)
    {
      controlMotor(0);
      digitalWrite(led2, 0); // LED 2 on.
      Serial.println("Watchdog hit : " + String(millis()));
    }
    else
    {
      digitalWrite(led2, 1);
    }
  }

  if(millis() - lastMillis > 250)
  {
    lastMillis = millis();
    digitalWrite(led1,!digitalRead(led1));
    // Serial.println("Encoder: " + String(encoderCount));
    // controlMotor(0);
  }
}

void I2C_requestEvent(void)
{
  i2cbuffer.longNumber = encoderCount;
  Wire.write(i2cbuffer.longBytes, 4); // send requested bytes
}

void I2C_receiveEvent(int howMany)
{
  watchdog = 0;
  // Serial.println("I2C_receiveEvent");
  while(Wire.available())
  {
    int8_t speed =  int8_t(Wire.read());
    controlMotor(speed);
  }
}

void IO_setup()
{
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, 1);
  digitalWrite(led2, 1);

  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  analogWrite(motorPinA, 255);
  analogWrite(motorPinB, 255);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoder, CHANGE);
}

void ISR_encoder(void)
{
  static uint8_t lastA = 0;
  static uint8_t lastB = 0;
  uint8_t newA = digitalRead(encoderPinA);
  uint8_t newB = digitalRead(encoderPinB);

  if(newA != lastA)
  {
    lastA = newA;
    if(newA == newB)
    {
      encoderCount ++;
    }
    else
    {
      encoderCount --;
    }
  }
  else
  {
    if(newB != lastB)
    {
      lastB = newB;
      if(newA == newB)
      {
        encoderCount --;
      }
      else
      {
        encoderCount ++;
      }
    }
  }
}

void controlMotor(int8_t speed)
{
  int8_t speedAbs = int8_t(map(constrain(abs(speed), 0, 100), 0, 100, 0, 255));
  if(speed > 0)
  {
    analogWrite(motorPinA, 255 - speedAbs);
    analogWrite(motorPinB, 255);
  }
  else if(speed < 0)
  {
    analogWrite(motorPinA, 255);
    analogWrite(motorPinB, 255 - speedAbs);
  }
  else
  {
    analogWrite(motorPinA, 255);
    analogWrite(motorPinB, 255);
  }
}


uint8_t I2C_setup(void)
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
    return addr;
}


