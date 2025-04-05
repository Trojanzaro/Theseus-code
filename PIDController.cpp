#include "PIDController.h"

PIDController::PIDController(float Kp, float Ki, float Kd, float Setpoint) {
  setpoint = Setpoint;
  integral = 0;
  previous_error = 0;
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

double PIDController::compute(double process_variable, float dt) {
  double error = setpoint - process_variable;
            
  // Proportional term
  double P_out = kp * error;
  
  // Integral term
  integral += error * dt;
  double I_out = ki * integral;
  
  // Derivative term
  double derivative = (error - previous_error) / dt;
  double D_out = kd * derivative;
  
  // Update previous error
  previous_error = error;
  
  // Compute and return total output
  return P_out + I_out + D_out;
}
