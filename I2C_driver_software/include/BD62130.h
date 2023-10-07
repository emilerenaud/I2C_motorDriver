#ifndef __BD62130_H__
#define __BD62130_H__

#include <Arduino.h>

class BD62130{

  public:
    BD62130();
    void init(int pin1, int pin2);
    void setSpd(int spd);
    void moveMotor(int spd);
    void moveFwd(void);
    void moveRev(void);
    void moveStop(void);
    void enableMot(void);
    void disableMot(void);
    void resetMot(void);
    void setPins(uint8_t pin_A, uint8_t pin_B);
    bool isMoving(void);

  private:
    uint8_t pwm_A;
    uint8_t pwm_B;
    bool motEnable;
    int16_t motSpd; 
    bool _moving = 0;
    bool fwd;
};

#endif