#include "motors.h"
#include "sensors.h"
#include "pid.h"
#include "Arduino.h"
#include "HardwareSerial.h"

Motors::Motors() {
  // Motor pins INITIALIZATION
  motor_pins[0] = 3;
  motor_pins[1] = 40;
  motor_pins[2] = 42;
  motor_pins[3] = 4;
  motor_pins[4] = 41;
  motor_pins[5] = 43;
  motor_pins[6] = 5;
  motor_pins[7] = 36;
  motor_pins[8] = 38;
  motor_pins[9] = 6;
  motor_pins[10] = 37;
  motor_pins[11] = 39;
  time_to_move=760;
  time_to_rotate_90=610;
  dir[0] = 0.0;
  dir[1] = 0.0;

  heading = 1; // Initialized to move in the direction of the gripper

  should_turn = 1;

  torq_fact = 150;
  torque_offset = 25;
  max_allowed_torque = 254;
  min_torque = 67;
  Normalized_motor_torque = 550;
  for (int i=0;i<4;i++){
    motor_torques_move[i]=torq_fact;
  }
  motor_torques_move[1] += torque_offset;
  motor_torques_move[3] += torque_offset;
}

void Motors::pinMode_init() {
  for (int i = 0; i < 12; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }
}
/*
void Motors::heading_update(int* feasible_curr) {
  opposite_heading = ((heading+1)%2)*(2-heading) + (heading%2)*(4-heading);
  for (int i=0;i<4;i++){
    if((feasible_curr[i]==1) && i!=opposite_heading){
      should_turn=1;
      heading=i;
      break;
    }
  }
  if (feasible_curr[heading] == 0){
      if(feasible_curr[opposite_heading] == 1){
        heading == opposite_heading;
        should_turn = 1;
      }
      else{
        // Applying brakes to all the motors
        for (int i = 0; i < 4; i++) {
          digitalWrite(motor_pins[3*i+1],LOW);
          digitalWrite(motor_pins[3*i+2],LOW);
        }
        should_turn = 0;
        Serial.println("No where to go");
      }
  }
}
*/
void Motors::orientation_update(Sensors* mySensors){
  (*mySensors).print_feasible_directions();
  if ((*mySensors).feasible_curr[0]){
    rotate_bot(-90);
    }
    else if ((*mySensors).feasible_curr[2]){
      rotate_bot(90);
    }
    else if((*mySensors).feasible_curr[3]){
      rotate_bot(90);
      delay(1000);
      rotate_bot(90);
    }
    else {
      apply_brakes();
    }
  (*mySensors).find_feasible_directions();
}
void Motors::rotate_bot_90(int angle_to_rotate){
  motor_torques[0]=angle_to_rotate/abs(angle_to_rotate)*torq_fact;
  motor_torques[1]=angle_to_rotate/abs(angle_to_rotate)*torq_fact;
  motor_torques[2]=-angle_to_rotate/abs(angle_to_rotate)*torq_fact;
  motor_torques[3]=-angle_to_rotate/abs(angle_to_rotate)*torq_fact;
  drive_motors();

  delay(time_to_rotate_90*int(abs(angle_to_rotate)/90));
  apply_brakes();
}
void Motors::rotate_bot(int angle_to_rotate){
  if(abs(angle_to_rotate)==90){
    rotate_bot_90(angle_to_rotate);
  }
  else if(abs(angle_to_rotate)==180){
    rotate_bot_90(90);
    delay(1500);
    rotate_bot_90(90);
  }
  else if(angle_to_rotate==270){
    rotate_bot_90(-90);
  }
  else if(angle_to_rotate==-270){
    rotate_bot_90(90);
  }
}
void Motors::dir_update_PID(PID* B1_pid, PID* B3_pid, float* corrective_B1, float* corrective_B3) {
  //*/
  dir[0]=0;
  dir[1]=0;
  // B3 Implementation
  // Implementing PID for maintaining distance from wall
  // Prefers wall left to heading if it exists
  ///
  (*B3_pid).compute_pid_corr(corrective_B3[1]);
  if (corrective_B3[0] != -1) {
    if (int(corrective_B3[0])%2 == 0) {
      dir[0] = dir[0] - pow(-1, 4%(int(corrective_B3[0])+1))*(*B3_pid).pid_corr_val;
    }
    else {
      dir[1] = dir[1] - pow(-1, 4%(int(corrective_B3[0])))*(*B3_pid).pid_corr_val;
    }
  }
  //*/

  // If it is too close to a wall, adding a component to the direction which would make it move away, only B1 is implimented yet
  // Implementing PID for direction correction
  ///
  for (int i = 0; i < 4; i++) {
    B1_pid[i].compute_pid_corr(corrective_B1[i]);
    if(i%2 == 0) {
      dir[0]=dir[0]-pow(-1, 4%(i+1))*B1_pid[i].pid_corr_val;
    }
    else if(i%2==1) {
      dir[1]=dir[1]-pow(-1, 4%i)*B1_pid[i].pid_corr_val;
    }
  }//*/
}

void Motors::get_torques(PID* B2_pid, float corrective_B2) {
  ///
  motor_torques[0]=dir[0]+dir[1];
  motor_torques[1]=-dir[0]+dir[1];
  motor_torques[2]=dir[0]+dir[1];
  motor_torques[3]=-dir[0]+dir[1];
  //*/

  ///
  (*B2_pid).compute_pid_corr(corrective_B2);
  motor_torques[0] += (*B2_pid).pid_corr_val;
  motor_torques[1] += (*B2_pid).pid_corr_val;
  motor_torques[2] -= (*B2_pid).pid_corr_val;
  motor_torques[3] -= (*B2_pid).pid_corr_val;
  //*/

  ///
  motor_torques_sum=abs(motor_torques[0])+abs(motor_torques[1])+abs(motor_torques[2])+abs(motor_torques[3]);
  if(motor_torques_sum != 0.0) {
    normalized_scale=(Normalized_motor_torque-4*min_torque)/motor_torques_sum;
    motor_torques[0]=motor_torques[0]*normalized_scale;
    motor_torques[1]=motor_torques[1]*normalized_scale;
    motor_torques[2]=motor_torques[2]*normalized_scale;
    motor_torques[3]=motor_torques[3]*normalized_scale;
  }
  //*/

  max_normalized_motor_torque = motor_torques[0];
  for (int i = 1; i < 4; i++) {
    if(motor_torques[i] > max_normalized_motor_torque) max_normalized_motor_torque = motor_torques[i];
  }
  if (max_normalized_motor_torque>max_allowed_torque-min_torque){
    capped_scale=max_normalized_motor_torque/(max_allowed_torque-min_torque);
  }

  if (capped_scale != 0.0) {
    motor_torques[0]=(motor_torques[0]/capped_scale)+min_torque;
    motor_torques[1]=(motor_torques[1]/capped_scale)+min_torque;
    motor_torques[2]=(motor_torques[2]/capped_scale)+min_torque;
    motor_torques[3]=(motor_torques[3]/capped_scale)+min_torque;
  }
  capped_scale = 0.0;
}

void Motors::drive_motors() {
  for (int i = 0; i < 4; i++) {
    int dir_fact = (motor_torques[i] > 0 ) ? 1 : -1;
    int in1_state = (dir_fact+2)%3;

    analogWrite(motor_pins[3*i], abs(motor_torques[i]));
    digitalWrite(motor_pins[3*i+1], in1_state);
    digitalWrite(motor_pins[3*i+2], 1-in1_state);
  }
}

void Motors::apply_brakes() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motor_pins[3*i+1], LOW);
    digitalWrite(motor_pins[3*i+2], LOW);
  }
}

void Motors::drive_motors_move(){
  for (int i = 0; i < 4; i++) {
    int dir_fact = (motor_torques_move[i] > 0 ) ? 1 : -1;
    int in1_state = (dir_fact+2)%3;
    analogWrite(motor_pins[3*i], abs(motor_torques_move[i]));
    digitalWrite(motor_pins[3*i+1], in1_state); // In1 pin for motor i
    digitalWrite(motor_pins[3*i+2], 1-in1_state); // In2 pin for motor i
  }
  delay(time_to_move);
  apply_brakes();
  delay(2000);
}


void Motors::get_torques_slow(PID* B2_pid, float corrective_B2) {
  ///
  motor_torques[0]=dir[0]+dir[1];
  motor_torques[1]=-dir[0]+dir[1];
  motor_torques[2]=dir[0]+dir[1];
  motor_torques[3]=-dir[0]+dir[1];
  //*/

  ///
  (*B2_pid).compute_pid_corr(corrective_B2);
  motor_torques[0] += (*B2_pid).pid_corr_val;
  motor_torques[1] += (*B2_pid).pid_corr_val;
  motor_torques[2] -= (*B2_pid).pid_corr_val;
  motor_torques[3] -= (*B2_pid).pid_corr_val;
  //*/

  ///
  motor_torques_sum=abs(motor_torques[0])+abs(motor_torques[1])+abs(motor_torques[2])+abs(motor_torques[3]);
  if(motor_torques_sum != 0.0) {
    normalized_scale=(Normalized_motor_torque-4*min_torque)/motor_torques_sum;
    motor_torques[0]=motor_torques[0]*normalized_scale;
    motor_torques[1]=motor_torques[1]*normalized_scale;
    motor_torques[2]=motor_torques[2]*normalized_scale;
    motor_torques[3]=motor_torques[3]*normalized_scale;
  }
  //*/

  max_normalized_motor_torque = motor_torques[0];
  for (int i = 1; i < 4; i++) {
    if(motor_torques[i] > max_normalized_motor_torque) max_normalized_motor_torque = motor_torques[i];
  }
  if (max_normalized_motor_torque>max_allowed_torque-min_torque){
    capped_scale=max_normalized_motor_torque/(max_allowed_torque-min_torque);
  }

  if (capped_scale != 0.0) {
    motor_torques[0]=(motor_torques[0]/capped_scale)+min_torque;
    motor_torques[1]=(motor_torques[1]/capped_scale)+min_torque;
    motor_torques[2]=(motor_torques[2]/capped_scale)+min_torque;
    motor_torques[3]=(motor_torques[3]/capped_scale)+min_torque;
  }
  
  //added slow calc
  for (int i = 0; i < 4; i++) {
	motor_torques[i] = motor_torques[i];  
  }
  
  capped_scale = 0.0;
  
}

void Motors::drive_motors_slow(int dir_fact){
  for (int i = 0; i < 4; i++) {
    int in1_state = (dir_fact+2)%3;
    analogWrite(motor_pins[3*i], 80);
    digitalWrite(motor_pins[3*i+1], in1_state); // In1 pin for motor i
    digitalWrite(motor_pins[3*i+2], 1-in1_state); // In2 pin for motor i
  }
}

void Motors::print_heading_val() {
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print(" Opposite Heading: ");
  Serial.println(opposite_heading);
}

void Motors::print_torque_vals() {
  for (int i = 0; i < 4; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(":");
    Serial.print(motor_torques[i]);
    Serial.print(" ");
  }
  Serial.println();
}
