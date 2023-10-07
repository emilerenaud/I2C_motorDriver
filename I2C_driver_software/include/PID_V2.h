
#ifndef PIDV2_H_
#define PIDV2_H_

#include <Arduino.h>

class PIDV2
{
  public:
 
    PIDV2();
    ~PIDV2();
    void setGains(double kp_, double ki_, double kd_);
    void seteIntLimit(double limit_);
    double computeCommand(double error);
    void setID(int ID_);
    void reset();

  private:

    double kp = 1;
    double ki = 0;
    double kd = 0;

    double eIntegral;
    double eIntegralLimit = 0;
    double ePrevious;
    double lastMeasureTime;

    int ID;
};
#endif //PID
