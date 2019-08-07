#include <motors.h>
#include <sensors.h>
#include <pid.h>

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;

String command;

void parseCommand(PID* B2_pid, String com);

void setup() {
  Serial.begin(115200);
  myMotors.pinMode_init();
  mySensors.pinMode_init();

  // From PID tuning
  B2_pid.Kp = 40;
  B2_pid.Ki = 0.1;
  B2_pid.Kd = 2;
}

void loop() {
  while(Serial.available() > 0){
      char val = Serial.read();

      if(val == '\n') {
        parseCommand(&B2_pid, command);
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

  if (myMotors.should_turn) {
    myMotors.get_torques(&B2_pid, mySensors.corrective_B2);
    //Serial.println(mySensors.corrective_B2);
    Serial.println(B2_pid.pid_corr_val);
    //myMotors.print_torque_vals();
    myMotors.drive_motors();
  }
}

void parseCommand(PID* B2_pid, String com) {
  String part1;
  String part2;
  String part3;

  part1 = com.substring(0, com.indexOf(","));
  part2 = com.substring(com.indexOf(",")+1, com.indexOf(";"));
  part3 = com.substring(com.indexOf(";")+1);

  (*B2_pid).Kp = part1.toFloat();
  (*B2_pid).Ki = part2.toFloat();
  (*B2_pid).Kd = part3.toFloat();
}
