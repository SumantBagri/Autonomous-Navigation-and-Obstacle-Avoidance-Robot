#include "sensors.h"
#include "pid.h"
#include "Arduino.h"
#include "HardwareSerial.h"

Sensors::Sensors() {

  eps = 0.75;

  noInput = 1; // Initialized to TRUE as no input is received
  heading_inp = 0; // Input heading initialized in the direction of the gripper
  bool_check = -1;

  u_s[0] = 46; // Select pin: 0  is connected to Pin 46
  u_s[1] = 48; // Select pin: 1  is connected to Pin 48
  u_s[2] = 50; // Select pin: 2  is connected to Pin 50
  u_s[3] = 52; // Select pin: 3  is connected to Pin 52

  u_sig = 2; // SIG pin on the MUX is connected to Pin 2

  // Initializing as none of the directions being feasible
  feasible_curr[0] = 0;
  feasible_curr[1] = 0;
  feasible_curr[2] = 0;
  feasible_curr[3] = 0;
  upper_threshold_value = 12; // Value in cms

  corrective_B1[0] = 0.0;
  corrective_B1[1] = 0.0;
  corrective_B1[2] = 0.0;
  corrective_B1[3] = 0.0;
  min_allowable_dist = 4; // Value in cms

  corrective_B2 = 0.0; // Value in cms
  max_wall_dist = 10;

  corrective_B3[0] = 0.0;
  corrective_B3[0] = 0.0;
  corrective_B3[0] = 0.0;
  corrective_B3[0] = 0.0;
  dist_to_maintain = 5.5;
}

void Sensors::pinMode_init() {

  for(int i=0;i<4;i++){
     pinMode(u_s[i],OUTPUT);
  }
  pinMode(u_sig,OUTPUT); //Initially setting it to output, though it will switch dynamically between output and input

}

void Sensors::return_sel(int i) {
  int quotient_prev=i;
  int quotient_current;
  for (int j=0;j<4;j++){
    quotient_current=quotient_prev/2;
    sel[j]=quotient_prev-quotient_current*2;
    quotient_prev=quotient_current;
  }
}

void Sensors::read_ultrasonic(int i) {
  pinMode(u_sig,OUTPUT);
    digitalWrite(u_sig,LOW);
    return_sel(i+8);
    for (int j=0;j<4;j++){
      if (sel[j]==0){
        digitalWrite(u_s[j],LOW);
      }
      else{
        digitalWrite(u_s[j],HIGH);
      }
    }
    delayMicroseconds(2);
    digitalWrite(u_sig, HIGH);
    delayMicroseconds(10);
    digitalWrite(u_sig,LOW);
    pinMode(u_sig,INPUT);
    return_sel(i);
    for (int j=0;j<4;j++){
      if (sel[j]==0){
        digitalWrite(u_s[j],LOW);
      }
      else{
        digitalWrite(u_s[j],HIGH);
      }
    }
    dist[i] = double(pulseIn(u_sig, HIGH, 60000)/58.31);
    if(dist[i] == 0.0) dist[i] = 3000.0; // With timeout pulseIn might return 0.0 indicating no wall/infinite distance
}

void Sensors::find_feasible_directions() {
  //---------------------//
  for (int i=0;i<4;i++){
    float avg_sensor_dist = (dist[2*i]+dist[2*i+1])/2;
    /// Check for feasible directions
    if (avg_sensor_dist > upper_threshold_value){
      feasible_curr[i]=1;
    }
    else if(avg_sensor_dist < upper_threshold_value){
      feasible_curr[i]=0;
    }
    //*/
  }
}

void Sensors::update_corrective_factors(int heading, PID* B1_pid, PID* B2_pid, PID* B3_pid) {
  int reset_var = 1;
  int left_face  = (heading+1)%4;
  int right_face = (heading+3)%4;
  /// Check for distance from "wall" i
  if ((dist[2*left_face]<max_wall_dist)&&(dist[2*left_face+1]<max_wall_dist)) {
    if (prev_face != left_face) {
      (*B3_pid).reset_variables();
    }
    corrective_B3[0] = left_face;
    float avg_sensor_dist = (dist[2*left_face] + dist[2*left_face+1])/2;
    corrective_B3[1] = dist_to_maintain - avg_sensor_dist;
    prev_face = left_face;
  }
  else if ((dist[2*right_face]<max_wall_dist)&&(dist[2*right_face+1]<max_wall_dist)){
    if (prev_face != right_face) {
      (*B3_pid).reset_variables();
    }
    corrective_B3[0] = right_face;
    float avg_sensor_dist = (dist[2*right_face] + dist[2*right_face+1])/2;
    corrective_B3[1] = dist_to_maintain - avg_sensor_dist;
    prev_face = right_face;
  }
  else {
    corrective_B3[0] = -1;
    (*B3_pid).reset_variables();
  }
  //*/

  for (int i=0;i<4;i++){
    float min_sensor_dist = min(dist[2*i],dist[2*i+1]);
    /// Check for misalignment
    if ((dist[2*i]<max_wall_dist)&&(dist[2*i+1]<max_wall_dist)){
      double sensor_val_diff = dist[2*i] - dist[2*i+1];
      if(bool_check == i) {
        corrective_B2 = sensor_val_diff;
        reset_var = 0;
      }
      else if ((bool_check != i) && (reset_var == 1)) {
        (*B2_pid).reset_variables();
        corrective_B2 = sensor_val_diff;
        bool_check = i;
        reset_var = 0;
      }
    }
    else if ((i == 3) && (reset_var)){
      corrective_B2 = 0.0;
      (*B2_pid).reset_variables();
    }//*/

    /// Check for wall closeness
    if (min_sensor_dist < min_allowable_dist-eps) {
      corrective_B1[i] = min_allowable_dist-min(dist[2*i],dist[2*i+1]);
    }
    else if (min_sensor_dist > min_allowable_dist+eps) {
      corrective_B1[i] = 0.0;
      B1_pid[i].reset_variables();
    }
    //*/
  }
}

void Sensors::print_sensor_data() {
  for (int i = 0; i < 8; i++) {
    Serial.print(dist[i]);
    if (i!=7) {
      Serial.print(",");
    }
  }
  Serial.println();
}

//!!NOT WORKING!!//
void Sensors::io_init() {
  for(int i=0; i < 8; i++){
    read_ultrasonic(i);
  }
  print_sensor_data();

  while(noInput){
    if(millis() > 5000) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      noInput = 0;
    }

    while(Serial.available() > 0) {
      heading_inp = Serial.read() - 48;
      noInput = 0;
      Serial.read();
    }
  }
}

void Sensors::print_feasible_directions() {
  for(int i =0; i<4; i++){
    Serial.print(feasible_curr[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("B1:(");
  Serial.print(corrective_B1[0]);
  Serial.print(",");
  Serial.print(corrective_B1[1]);
  Serial.print(",");
  Serial.print(corrective_B1[2]);
  Serial.print(",");
  Serial.print(corrective_B1[3]);
  Serial.print(",");
  Serial.print(corrective_B1[1]);
  Serial.print(") B2:");
  Serial.println(corrective_B2);
}
