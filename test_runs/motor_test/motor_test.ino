#include <motors.h>
#include <sensors.h>
#include <pid.h>

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;

String command;

void setup() {
  Serial.begin(115200);
  myMotors.pinMode_init();
  mySensors.pinMode_init();
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  // From B1 PID tuning
  for (int i = 0; i < 4; i++) {
    B1_pid[i].Kp = 40;
    B1_pid[i].Ki = 0;
    B1_pid[i].Kd = 60;
  }

  // From B2 PID tuning
  B2_pid.Kp = 40;
  B2_pid.Ki = 0.1;
  B2_pid.Kd = 2;
}

void loop() {
  for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }

  while(Serial.available() > 0){
    char val = Serial.read();

    if(val == '\n') {
      mySensors.heading_inp = command.toInt();
      command = "";
    }
    else {
      command += val;
    }
  }
  //mySensors.print_sensor_data();

  mySensors.find_feasible_directions(B1_pid, &B2_pid, B3_pid);

  if(mySensors.feasible_curr[mySensors.heading_inp] != 0) {
    digitalWrite(LED_BUILTIN, LOW);
    myMotors.heading = mySensors.heading_inp;
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (mySensors.feasible_curr[myMotors.heading] == 0) {
    myMotors.heading_update(mySensors.feasible_curr);
  }

  if (myMotors.should_turn) {
    myMotors.direction_update(B1_pid, mySensors.corrective_B1);
    myMotors.get_torques(&B2_pid, mySensors.corrective_B2);
    myMotors.print_torque_vals();
    myMotors.drive_motors();
  }
}
