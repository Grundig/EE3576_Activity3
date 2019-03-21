#include <double_driver.h>
double_driver motor;

void setup() {
  int R_motor_pin = 13;
  int R_direction_pin = 12;
  int L_motor_pin = 11;
  int L_direction_pin = 10;
  int R_measure_pin = int_0;
  int L_measure_pin = int_1;
  int control_interval_time = 250; //in ms
  double task1_speed = 75; //rpm
  
  Serial.begin(9600);
  motor.set_time_intervals(control_interval_time);
  motor.setup_motor(R_motor_pin, R_direction_pin, L_motor_pin, L_direction_pin);
  motor.setup_speed_measure(R_measure_pin, L_measure_pin);
  motor.set_target_speed_L(task1_speed);
  motor.set_target_speed_R(task1_speed);
  
}

void loop() {
  motor.execute_task1();
  
}
