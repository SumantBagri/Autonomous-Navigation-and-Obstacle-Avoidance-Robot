#ifndef pid_H
#define pid_H

#include "Arduino.h"
#include <pins_arduino.h>

class PID {
  public:
    PID();

    void compute_pid_corr(double errorVal);
    void update_variables();
    void reset_variables();

    // PARAMETERS
    double Kp;
    double Ki;
    double Kd;

    double curr_error; // Input from corrective_B1/corrective_B2
    double pid_corr_val;
  private:
    // VARIABLES
    double prev_error;
    double total_error;
};

#endif
