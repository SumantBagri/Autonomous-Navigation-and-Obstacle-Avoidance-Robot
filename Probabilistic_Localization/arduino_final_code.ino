#include <motors.h>
#include <sensors.h>
#include <pid.h>

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;
PID B3_pid;

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

  // From PID tuning
  B3_pid.Kp = 10;
  B3_pid.Ki = 0.05;
  B3_pid.Kd = 15;

  while (millis() < 5000) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }
}
int check=0;
unsigned long timer=5000;
void loop() {
  check=0;
  if (millis() > timer) {
    timer=timer+600;
    mySensors.print_sensor_data();
    Serial.print(",");
    Serial.print(myMotors.heading);

    for(int i=0; i < 8; i++){
      mySensors.read_ultrasonic(i);
    }

    while(Serial.available() > 0){
      check=1;
      char val = Serial.read();
      if(val == '\n') {
        myMotors.heading = command.toInt();
        Serial.print(",");
        Serial.println(99);
        command = "";
      }
      else {
        command += val;
      }
    }
    if(check!=1){
      Serial.print(",");
      Serial.println("00");
    }
    if (myMotors.heading == -1) {
      myMotors.apply_brakes();
    }
    else {
      mySensors.find_feasible_directions(myMotors.heading, B1_pid, &B2_pid, &B3_pid);

      if (mySensors.feasible_curr[myMotors.heading] == 0) {
          myMotors.heading_update(mySensors.feasible_curr);
      }

      if (myMotors.should_turn) {
        myMotors.direction_update(B1_pid, &B3_pid, mySensors.corrective_B1, mySensors.corrective_B3);
        myMotors.get_torques(&B2_pid, mySensors.corrective_B2);
        myMotors.drive_motors();
      }
    }

    delay(200);
  }
  else {
    myMotors.apply_brakes();
    delay(300);
  }
}
