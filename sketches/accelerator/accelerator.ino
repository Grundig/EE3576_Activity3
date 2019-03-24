#include <double_driver.h>
#include <Accelarator.h>
#include <SpeedMeasure.h>
#include <RangeSensor.h>

double_driver motor;
Accelarator accelarator;
RangeSensor range_sensor;

void setup() {
  int R_motor_pin = 9;
  int R_direction_pin = 12;
  int L_motor_pin = 11;
  int L_direction_pin = 10;
  int R_measure_pin = int_0;
  int L_measure_pin = int_1;
  int control_interval_time = 250; //in ms

  Serial.begin(9600);
  motor.set_time_intervals(control_interval_time);
  motor.setup_motor(R_motor_pin, R_direction_pin, L_motor_pin, L_direction_pin);
  motor.setup_speed_measure(R_measure_pin, L_measure_pin);

  motor.setup_pid_R();
  motor.setup_pid_L();

  accelarator = Accelarator(2000, 10, 500, 500, 4000, &motor);

  range_sensor = RangeSensor(8, 7, 50);

  Serial.println("SETUP FINISHED");
}

void loop() {
  if (range_sensor.safe()) {
    accelarator.apply_desired_speed();
  } else {
    motor.motor_speed_input(stop_R);
    motor.motor_speed_input(stop_L);
    Serial.println("There is an obstacle in the path");
  }
}
