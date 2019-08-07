#ifndef motors_H
#define motors_H

#include "pid.h"
#include "sensors.h"
#include "Arduino.h"
#include <pins_arduino.h>

class Motors {
  public:
    // Constructors and Initializers
    Motors();
    void pinMode_init();

    // Heading is gonna be an input to drive motors
    int heading;
    int torq_fact;
    float Normalized_motor_torque;

    int should_turn; // Execution flag

    // Actuation handlers
    //void heading_update(int* feasible_curr); // Requires feasible_curr from Sensors
    // Requires corrective_B1 and corrective_B3 output from Sensors
    void orientation_update (Sensors* mySensors);
    void dir_update_PID(PID* B1_pid, PID* B3_pid, float* corrective_B1, float* corrective_B3);
    void get_torques(PID* B2_pid, float corrective_B2);
    void drive_motors();
    void drive_motors_move();
    void apply_brakes();
    void rotate_bot(int angle_to_rotate);
    void rotate_bot_90(int angle_to_rotate);
    void drive_motors_slow(int dir_fact);
	void get_torques_slow(PID* B2_pid, float corrective_B2);
    // Serial Monitor writers
    void print_heading_val();
    void print_torque_vals();
  private:
    // Motor pins
    // 3*i = enable; 3*i+1 = in1; 3*i+2 = in2;
    int motor_pins[12];

    // PARAMETERS
    float max_allowed_torque; // Torque parameters
    float min_torque;

    //VARIABLES
    int opposite_heading;
    float dir[2];
    float motor_torques[4];
    float motor_torques_move[4];
    float max_normalized_motor_torque;
    float motor_torques_sum;
    float normalized_scale;
    float capped_scale;
    int time_to_move;
    int time_to_rotate_90;
	int torque_offset;
};
#endif
