#include "pid.h"
#include "Arduino.h"

PID::PID() {
  // Variables initiazed to 0 on creation
  prev_error = 0.0;
  total_error = 0.0;
  curr_error = 0.0;

  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
}

void PID::compute_pid_corr(double errorVal) {
  update_variables();

  curr_error = errorVal;

  // PID formula
  pid_corr_val = Kp*curr_error + Ki*total_error + Kd*(curr_error - prev_error);
}

void PID::update_variables() {
  prev_error = curr_error;
  total_error += curr_error;
}

void PID::reset_variables() {
  curr_error = 0.0;
  prev_error = 0.0;
  total_error = 0.0;
}
