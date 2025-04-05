#ifndef pidcontroller_h
#define pidcontroller_h

#include "Arduino.h"

class PIDController
{
  public:
    PIDController(float, float, float, float);
    double compute(double, float);
  private:
    float kp;
    float ki;
    float kd;
    double setpoint;
    double previous_error;
    double integral;
};

#endif
