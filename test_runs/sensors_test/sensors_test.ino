#include <sensors.h>
#include <pid.h>

Sensors mySensors;
PID B1_pid_ctrl[4];
PID B2_pid_ctrl;

void setup() {
  Serial.begin(115200);
  mySensors.pinMode_init();
}

void loop() {
  for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }
  mySensors.find_feasible_directions();
  mySensors.print_sensor_data();
  mySensors.print_feasible_directions();

  Serial.print("B1_pid: ");
  for (int i = 0; i < 4; i++) {
    B1_pid_ctrl[i].compute_pid_corr(mySensors.corrective_B1[i]);

    Serial.print(B1_pid_ctrl[i].pid_corr_val);
    Serial.print(" ");
  }
  Serial.println();

  B2_pid_ctrl.compute_pid_corr(mySensors.corrective_B2);
  Serial.print("B2_pid: ");
  Serial.println(B2_pid_ctrl.pid_corr_val);

  for (int i = 0; i < 4; i++) mySensors.corrective_B1[i] = 0.0;
  mySensors.corrective_B2 = 0.0;
  delay(1000);
}
