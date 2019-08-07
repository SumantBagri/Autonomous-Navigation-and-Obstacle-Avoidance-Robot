#include <motors.h>
#include <sensors.h>
#include <pid.h>

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;
PID B3_pid;

String command;

void parseCommand(Motors* myMotors, String com);

void setup() {
  Serial.begin(115200);
  myMotors.pinMode_init();
  mySensors.pinMode_init();

  for (int i = 0; i < 4; i++) {
    B1_pid[i].Kp = 40;
    B1_pid[i].Ki = 0;
    B1_pid[i].Kd = 60;
  }

  // From PID tuning
  B2_pid.Kp = 40;
  B2_pid.Ki = 0.1;
  B2_pid.Kd = 2;

  // From PID tuning
  B3_pid.Kp = 10;
  B3_pid.Ki = 0.05;
  B3_pid.Kd = 15;
}

void loop() {
  while(Serial.available() > 0){
      char val = Serial.read();

      if(val == '\n') {
        parseCommand(&myMotors, command);
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
  mySensors.print_sensor_data();

  Serial.println(mySensors.feasible_curr[myMotors.heading]);
  if (mySensors.feasible_curr[myMotors.heading] == 0) {
    myMotors.heading_update(mySensors.feasible_curr);
  }

  if (myMotors.should_turn) {
    myMotors.direction_update(B1_pid, &B3_pid, mySensors.corrective_B1, mySensors.corrective_B3);
    myMotors.get_torques(&B2_pid, mySensors.corrective_B2);
    myMotors.print_torque_vals();
    myMotors.drive_motors();
  }
}

void parseCommand(Motors* myMotors, String com) {
  String part1;
  String part2;

  part1 = com.substring(0, com.indexOf(","));
  part2 = com.substring(com.indexOf(",")+1, com.indexOf(";"));

  (*myMotors).Normalized_motor_torque = part1.toFloat();
  (*myMotors).torq_fact = part2.toInt();
}
