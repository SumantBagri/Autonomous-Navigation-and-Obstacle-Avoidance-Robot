#include <motors.h>
#include <sensors.h>
#include <pid.h>

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;
PID B3_pid;

String command;

void parseCommand(PID* B3_pid, String com);

void setup() {
  Serial.begin(115200);
  myMotors.pinMode_init();
  mySensors.pinMode_init();
}

void loop() {
  while(Serial.available() > 0){
      char val = Serial.read();

      if(val == '\n') {
        parseCommand(&B3_pid, command);
        command = "";
      }
      else {
        command += val;
      }
  }

  for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }
  mySensors.find_feasible_directions(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
  myMotors.prev_heading = myMotors.heading;
  mySensors.print_sensor_data();

  if (myMotors.should_turn) {
    myMotors.direction_update(B1_pid, &B3_pid, mySensors.corrective_B1, mySensors.corrective_B3);
    Serial.println(B3_pid.pid_corr_val);
    myMotors.get_torques(&B2_pid, mySensors.corrective_B2);
    myMotors.drive_motors();
  }
}

void parseCommand(PID* B3_pid, String com) {
  String part1;
  String part2;
  String part3;

  part1 = com.substring(0, com.indexOf(","));
  part2 = com.substring(com.indexOf(",")+1, com.indexOf(";"));
  part3 = com.substring(com.indexOf(";")+1);

  for (int i = 0; i < 4; i++) {
    (*B3_pid).Kp = part1.toFloat();
    (*B3_pid).Ki = part2.toFloat();
    (*B3_pid).Kd = part3.toFloat();
  }
}
