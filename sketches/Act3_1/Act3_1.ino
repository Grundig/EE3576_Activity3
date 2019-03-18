#include<Act3_1.h>

Act3_1 our_sys;

void setup() {
  // put your setup code here, to run once:
  int start_pin = 4;
  int stop_pin = 5;
  int reverse_pin = 6;
  int analog_pin = A0;
  int motor_pin = 9;
  int direction_pin = 8;
  int speed_pin = int_0;   // Pin 3
  
  int check_time = 100;
  
  our_sys.setup_pushbuttons(start_pin, stop_pin, reverse_pin);
  our_sys.setup_potentiometer(analog_pin);
  our_sys.set_time_between_input_checks(check_time);
  our_sys.setup_motor(motor_pin, direction_pin);
  our_sys.setup_speed_measure(speed_pin);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  our_sys.system_execute();
}
