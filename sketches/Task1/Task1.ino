#include <single_driver.h>
single_driver R_motor, L_motor;

void setup() {
  int R_motor_pin = 13;
  int R_direction_pin = 12;
  int L_motor_pin = 11;
  int L_direction_pin = 10;
  int R_measure_pin = int_1;
  int L_measure_pin = int_0;
  int control_interval_time = 200; //in ms

  Serial.begin(9600);
  R_motor.set_time_intervals(250);
  L_motor.set_time_intervals(250);
  
  R_motor.setup_motor(R_motor_pin, R_direction_pin);
  L_motor.setup_motor(L_motor_pin, L_direction_pin);
  
  R_motor.setup_speed_measure(control_interval_time);
  L_motor.setup_speed_measure(control_interval_time);
}

void loop() {
  L_motor.system_execute();
  R_motor.system_execute();

}
