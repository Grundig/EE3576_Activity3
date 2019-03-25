#include <double_driver.h>
#include <RangeSensor.h>
double_driver motor;
RangeSensor range_sensor;

int task3_distance[10] = {20, 25, 20, 20, 15, 15, 25, 25, 20, 15};
bool task3_turns[10] = {0,1,0,1,0,0,0,0,1,1};
float task1_distance = 20; //cm
  
void setup() {
  int R_motor_pin = 9;
  int R_direction_pin = 12;
  int L_motor_pin = 11;
  int L_direction_pin = 10;
  int R_measure_pin = int_0;
  int L_measure_pin = int_1;
  int control_interval_time = 250; //in ms
  double task1_speed = 100; //rpm
  
  
  
  Serial.begin(9600);
  motor.set_time_intervals(control_interval_time);
  motor.setup_motor(R_motor_pin, R_direction_pin, L_motor_pin, L_direction_pin);
  motor.setup_speed_measure(R_measure_pin, L_measure_pin);
  motor.set_target_speed_L(task1_speed);
  motor.set_target_speed_R(task1_speed);
  motor.setup_pid_R();
  motor.setup_pid_L();
  range_sensor = RangeSensor(8, 7, 20);
}

void loop() {
  if (range_sensor.safe()) {
//    motor.execute_task1(task1_distance, true);
    motor.execute_task3(task3_distance, task3_turns);
//    motor.execute_task4(30,20,1);
  } else {
    motor.motor_speed_input(stop_R);
    motor.motor_speed_input(stop_L);
    }
}
