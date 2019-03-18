#ifndef Act3_1_h
#define Act3_1_h

#include<Arduino.h>
#include<Basic_Input.h> 
#include<PushButton.h>
#include<IntervalCheckTimer.h>
#include<DCmotor.h>
#include<InterruptBasedSpeedMeasure.h>
#include<InterruptBasedInputs.h>
double RPM;
class Act3_1{
	
	protected:
		
		// Objects
		in_analog motor_potentiometer;
		inputs motor_pushbuttons;
		HBridgeDCmotor motor;
		InterruptSpeedMeasure rotation_counter;
		IntervalCheckTimer check_inp_time;
		IntervalCheckTimer speed_check;
		
		// Values
		int map_min_val = 0;
		int map_max_val = 100;
		int speed_control_ms = 200; 
		
		// Enabled
		bool pushbuttons_enabled = false, potentiometer_enabled = false, time_enabled = false, motor_enabled = false, speed_enabled = false;
		bool enabled;
		
		
	public:
		
		// Constructer, set to default
		Act3_1()
		{
			enabled = false;
		}
		
		// Check if system is fully set up
		bool isEnabled()
		{
			// All setups complete
			if (pushbuttons_enabled && potentiometer_enabled && time_enabled && motor_enabled && speed_enabled)
			{
				enabled = true;
			}
			
			return enabled;
		}
		
		// Set up all three pushbuttons
		void setup_pushbuttons(int start_pin, int stop_pin, int reverse_pin)
		{
			if (!isEnabled())
			{
				// Set long push
				unsigned long int mininterval_ms=2000;
			
				// Labelling and assigning each pin
  				in_push_button start_but(start_pin, start, mininterval_ms);
  				in_push_button stop_but(stop_pin, stop, mininterval_ms);
  				in_push_button reverse_but(reverse_pin, reverse, mininterval_ms);
  			
  				// Add push button into system
  				motor_pushbuttons.add_in_push_button(start_but);
 				motor_pushbuttons.add_in_push_button(stop_but);
  				motor_pushbuttons.add_in_push_button(reverse_but);
  			
  				pushbuttons_enabled = true;
  			}
		}
		
		
		// Set up potentiometer
		void setup_potentiometer(int analog_pin)
		{
			if(!isEnabled())
			{
				// setup potentiometer
				motor_potentiometer.setup_in_analog(analog_pin);
			
				potentiometer_enabled = true;
			}
		}
		
		// Assign the time between checks of input variables
		void set_time_between_input_checks(int check_time)
		{
			if(!isEnabled())
			{
				//buttons
  				check_inp_time.setInterCheck(check_time);
  			
  				time_enabled = true;
  			}
		}
		
		
		// Set up motor
		void setup_motor(int motor_pin, int direction_pin)
		{
			if(!isEnabled())
			{
				motor.setup_HBridgeDCmotor(motor_pin, direction_pin);
			
				motor_enabled = true;
			}
		}
		
		// Set up hall effect sensor to measure speed
		void setup_speed_measure(ArduinoInterruptNames speed_pin)
		{
			if(!isEnabled())
			{
				rotation_counter.setupSpeedMeasure(speed_pin);
				
				// Set time between speed measurements
				speed_check.setInterCheck(speed_control_ms);
				
				speed_enabled = true;
			}
		}
		
		// Determine motor command (start, stop, reverse) from pushbuttons
		void motor_direction(command_list_enum in_smpl_cmd)
		{
			switch (in_smpl_cmd)
			{
        		case start:
        		Serial.println(" Start button pressed");
        		motor.start();
        		break;
        		
        		case stop:
        		Serial.println("    Stop button pressed");  
        		motor.stop();
        		break;
        		
        		case reverse:
        		Serial.println("        Reverse button pressed");
        		motor.changedir();
        		break;
        		
        		default:
          		Serial.println("Unknown button pressed");
          		break;
			}
		}
		
		// Set motor speed from potentiometer value
		void motor_speed(int val)
		{
			motor.setSpeedPWM(val);
		}
		
		// Read motor speed (rpm) from hall effect sensor
		void read_motor_speed()
		{
			RPM=rotation_counter.getRPMandUpdate();
    			if(RPM>0)
    			{
    				//Serial.print("revs per min = ");
    				//Serial.println(RPM);
    			}
    					
    			else
    			{
    				//Serial.println("Reading speed failed");
    			}
		}
		
		// Execute the system task
		system_execute()
		{
			if (isEnabled())
			{
				// Check inputs 
  				if(check_inp_time.isMinChekTimeElapsedAndUpdate()) 
  				{
  					command_list_enum in_smpl_cmd;
    				bool success_command, success_val;
    				int val, mapped_val, plot_map;
  					
  					// Get motor command and potentiometer value
  					success_command = motor_pushbuttons.check_n_get_command(in_smpl_cmd);
  					success_val = motor_potentiometer.read_input(val); 
  					
  					// Map potentiometer value
  					mapped_val = map(val, 0, 1023, map_min_val, map_max_val);
					//plot_map = map(val,0,1023,0,1000);
					// Output motor direction and speed
					if(success_command)
					{
  						motor_direction(in_smpl_cmd);
  					}
  					
  					if(success_val)
  					{
  						motor_speed(mapped_val);
  					}
  					
  					// Display motor speed
  					if (speed_check.isMinChekTimeElapsedAndUpdate())
  					{
  						read_motor_speed();
  						Serial.print(plot_map);
  						Serial.print(" ");
  						Serial.print(0);
  						Serial.print(" ");
						Serial.print(1023);
						Serial.print(" ");
						Serial.println(RPM);
  					}
  				}
  			}
		}
};


#endif
