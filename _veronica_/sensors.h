#ifndef sensors_H
#define sensors_H

#include "pid.h"
#include "Arduino.h"
#include <pins_arduino.h>

class Sensors {
  public:
    // VARIABLES
    double dist[8]; // Stores data from all eight sensors

    float corrective_B1[4]; // "i" is the face which is too close to an obstacle
                            // B1[i] stores the corrective error value
                            // 0.0 if no error
    float corrective_B2;    // B2 stores the sensor readings difference
                            // + implies ACW rotation is required for alignment
                            // -  implies CW rotation is required for alignment
    float corrective_B3[2]; // B3[0] stores the face number
                            // B3[1] stores the corrective error value for maintaining wall-distance
                            // 0.0 if no error or if there is no wall
    int heading_inp;        // Heading input received from Localization

    // system output is an array storing feasible directions
    int feasible_curr[4];
    int prev_face;
    float avg_sensor_dist[4];
    float max_wall_dist;              // B2 correction parameter

    // Constructors and Initializers
    Sensors();
    void pinMode_init();
    void io_init();
    void pid_init();

    // Sensor_Data handlers
    void read_ultrasonic(int i);
    void return_sel(int i);
    void print_sensor_data();

    // Obstacle_Avoidance handlers
    void find_feasible_directions(); // Computes the feasible directions
    void update_corrective_factors(int heading, PID* B1_pid, PID* B2_pid, PID* B3_pid); // Updates the corrective PID factors
    void print_feasible_directions(); // Use only for Debugging;
  private:
    // system input is the sensor readings
    int noInput; // Flag to check retrival of input
    int bool_check;

    int u_s[4]; //Select pins of mux
    int u_sig;
    int sel[4]; // Stores the select pin state for selecting a particular line out of 16X1 mux

    // Obstacle avoidance
    // PARAMETERS
    float upper_threshold_value;
    float min_allowable_dist;         // B1 correction parameter
    float eps;                        // B1 correction band
    float dist_to_maintain;           // B3 correction parameter

};

#endif
