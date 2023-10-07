#include "motor_wrapper.h"

MotorWrapper::MotorWrapper()
{

}

MotorWrapper::~MotorWrapper()
{

}


void MotorWrapper::init(uint8_t pinA, uint8_t pinB, uint8_t encoderPinA, uint8_t encoderPinB)
{
    this->_motor = new BD62130();
    this->_motor->init(pinA, pinB);
    this->_motor->enableMot();
    this->_motor->setSpd(0);
    _encoder = new encoderWrapper(encoderPinA, encoderPinB);
    setMode(openLoop);
}

void MotorWrapper::update()
{
    if(_encoder != nullptr)
    {
        // read pos
        _encoder->read();
        _pos = _encoder->getCount(); // pos in step
        _spd = _encoder->getSpeed();

        // get speed
        if(millis() - _lastMillis > 10)
        {
            _lastMillis = millis();
            if(_mode == pos)
            {
                if(_pos != _goalPos)
                {
                    // recompute PID + set new command
                    double error = _goalPos - _pos;
                    double cmd = this->_pid.computeCommand(error);
                    _lastCmd += cmd;
                    sendCmd(_lastCmd);
                }
            }
            else if(_mode == speed) // speed mode
            {
                if(_spd != _goalSpd)
                {
                    double error = _goalSpd - _spd;
                    double cmd = this->_pid.computeCommand(error);
                    _lastCmd += cmd;
                    sendCmd(_lastCmd);
                }
            }
            else // open loop
            {
                sendCmd(_goalSpd);
            }
        }
    }
    else // open loop
    {
        sendCmd(_goalSpd);
    }
}

void MotorWrapper::setPos(double pos)
{
    _goalPos = pos;
}

void MotorWrapper::setSpd(double spd)
{
    _goalSpd = spd;
}

// cmd to motor DC[-100 to 100]. BLDC[0 to 100]
void MotorWrapper::sendCmd(uint8_t cmd)
{
    if (cmd < -100 || cmd > 100)
    {
        // WARNING("Cmd out of bound");
        cmd = (cmd < -100) ? -100 : 100;  
    }

    _motor->moveMotor(cmd);
}

void MotorWrapper::setMode(eMotorMode mode)
{
    _mode = mode;
}

void MotorWrapper::setPID(double kp, double ki, double kd, double limite)
{
    this->_pid.setGains(kp,ki,kd);
    this->_pid.seteIntLimit(limite);
}

double MotorWrapper::mmToStep(double mm)
{
    return mm;
}

