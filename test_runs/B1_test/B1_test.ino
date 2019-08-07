#include <motors.h>
#include <sensors.h>
#include <pid.h>

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;

String command;

void parseCommand(PID* B1_pid, String com);

void setup() {
  Serial.begin(115200);
  myMotors.pinMode_init();
  mySensors.pinMode_init();

  for (int i = 0; i < 4; i++) {
    B1_pid[i].Kp = 40;
    B1_pid[i].Ki = 0;
    B1_pid[i].Kd = 60;
  }

}

void loop() {
  while(Serial.available() > 0){
      char val = Serial.read();

      if(val == '\n') {
        parseCommand(B1_pid, command);
        command = "";
      }
      else {
        command += val;
      }
  }

  for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }
  mySensors.find_feasible_directions(B1_pid, &B2_pid);
  mySensors.print_sensor_data();

  if (mySensors.feasible_curr[myMotors.heading] == 0) {
    myMotors.heading_update(mySensors.feasible_curr);
  }
  if (myMotors.should_turn) {
    myMotors.direction_update(B1_pid, mySensors.corrective_B1);
    Serial.println(B1_pid[3].pid_corr_val);
    myMotors.get_torques(&B2_pid, mySensors.corrective_B2);
    myMotors.drive_motors();
  }
}

void parseCommand(PID* B1_pid, String com) {
  String part1;
  String part2;
  String part3;

  part1 = com.substring(0, com.indexOf(","));
  part2 = com.substring(com.indexOf(",")+1, com.indexOf(";"));
  part3 = com.substring(com.indexOf(";")+1);

  for (int i = 0; i < 4; i++) {
    B1_pid[i].Kp = part1.toFloat();
    B1_pid[i].Ki = part2.toFloat();
    B1_pid[i].Kd = part3.toFloat();
  }
}
