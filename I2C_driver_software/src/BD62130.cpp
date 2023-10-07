#include "BD62130.h"

BD62130::BD62130(){
    this->motSpd = 0;
}
void BD62130::init(int pin_A, int pin_B){

    this->pwm_A = pin_A;
    this->pwm_B = pin_B;

    pinMode(this->pwm_A, OUTPUT);
    pinMode(this->pwm_B, OUTPUT);

    // Necessaire? pas sur. probablement etant donner qu'on utilise analogwrite direct.
    // analogWriteResolution(8);
    // analogWriteFrequency(this->pwm_A, 20000);
    // analogWriteFrequency(this->pwm_B, 20000);
    
    analogWrite(this->pwm_A, 255);
    analogWrite(this->pwm_B, 255);

    this->enableMot();
}
void BD62130::setSpd(int spd){
    this->motSpd = int16_t(map(constrain(spd, 0, 100), 0, 100, 0, 255));
}
void BD62130::moveFwd(void){
    if(this->motEnable){
        if(this->_moving && !this->fwd)
        {
            this->moveStop();
        }
        analogWrite(this->pwm_B, 255);
        analogWrite(this->pwm_A, 255 - this->motSpd);
        this->_moving = 1;
        this->fwd = 1;
    }
}
void BD62130::moveRev(void){
    if(this->motEnable){
        if(this->_moving && this->fwd)
        {
            this->moveStop();
        }
        analogWrite(this->pwm_A, 255);
        analogWrite(this->pwm_B, 255 - this->motSpd);
        this->_moving = 1;
        this->fwd = 0;
    }
}
void BD62130::moveStop(void){
    analogWrite(this->pwm_A, 0);
    analogWrite(this->pwm_B, 0);
    if (this->_moving)
    {
        this->_moving = 0;
        delay(50);
    }
}
void BD62130::enableMot(void){
    this->motEnable = true;
}
void BD62130::disableMot(void){
    this->motEnable = false;
    analogWrite(this->pwm_A, 0);
    analogWrite(this->pwm_B, 0);
}
void BD62130::resetMot(void){

}
bool BD62130::isMoving(){
    return this->_moving;
}

void BD62130::moveMotor(int spd)
{
    if (spd > 0)
  {
    setSpd(spd);
    moveFwd();
  }
  else if (spd == 0)
  {
    setSpd(spd);
    moveStop();
  }
  else if (spd < 0)
  {
    setSpd(abs(spd));
    moveRev();
  }
}