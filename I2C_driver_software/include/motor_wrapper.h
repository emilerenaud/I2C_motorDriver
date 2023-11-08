#ifndef _MOTOR_WRAPPER_H
#define _MOTOR_WRAPPER_H

#include "Arduino.h"
#include "encoder_wrapper.h"
#include "PID_V2.h"
#include "BD62130.h"


class MotorWrapper
{
private:
    BD62130 *_motor = nullptr;
    encoderWrapper *_encoder = nullptr;
    PIDV2 _pid;

    bool _closedLoop = 0;
    double _pos = 0;
    double _spd = 0;
    double _lastPos = 0;
    double _goalPos = 0;
    uint8_t _goalSpd = 0;
    double _lastCmd = 0;

    unsigned long _lastMillis = 0;


public:

    enum eMotorMode
    {
        pos,
        speed,
        openLoop
    }_mode = openLoop;

   

    MotorWrapper();
    ~MotorWrapper();
    void init(uint8_t pinA, uint8_t pinB, uint8_t encoderPinA, uint8_t encoderPinB);

    void sendCmd(int8_t cmd);
    void setPos(double pos);
    void setSpd(int8_t spd);

    void update(void);

    void setMode(eMotorMode mode);
    void setPID(double kp, double ki, double kd, double limite);

    long getPos();

    double mmToStep(double mm);

};

#endif
