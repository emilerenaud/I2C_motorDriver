
#include "PID_V2.h"

PIDV2::PIDV2()
{
    this->kp = 0;
    this->ki = 0;
    this->kd = 0;
    this->eIntegral = 0;
    this->ePrevious = 0;
    this->eIntegralLimit = 0;
    this->lastMeasureTime = micros();
}

PIDV2::~PIDV2()
{
}

void PIDV2::setGains(double kp_, double ki_, double kd_)
{
    this->kp = kp_;
    this->ki = ki_;
    this->kd = kd_;
}

void PIDV2::seteIntLimit(double limit_)
{
    this->eIntegralLimit = limit_;
}

void PIDV2::setID(int ID_)
{
    this->ID = ID_;
}

double PIDV2::computeCommand(double error)
{
    if (isnan(error)){error = 0;}
    if (isinf(error)){error = 0;}

    double cmd;
    double currentTime = micros();
    double dt = currentTime - lastMeasureTime;

    this->eIntegral += error*dt;

    double P = this->kp*error;
    double I = this->ki*this->eIntegral;
    double D = this->kd*(error - this->ePrevious)/dt;

    if (I > this->eIntegralLimit)
    {
        I = this->eIntegralLimit;
    }
    if (I < -this->eIntegralLimit)
    {
        I = -this->eIntegralLimit;
    }

    cmd = P + I + D;

    this-> ePrevious = error;
    this-> lastMeasureTime = currentTime;

    return cmd;
}

void PIDV2::reset()
{
    this->eIntegralLimit = 0;
    this->ePrevious = 0;
    this-> lastMeasureTime = micros();
}
