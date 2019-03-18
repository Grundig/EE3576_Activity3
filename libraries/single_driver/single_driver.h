#ifndef single_driver_h
#define single_driver_h


#include<Arduino.h>
#include<Basic_Input.h> 
#include<PushButton.h>
#include<IntervalCheckTimer.h>
#include<DCmotor.h>
#include<InterruptBasedSpeedMeasure.h>
#include<InterruptBasedInputs.h>
#include<SerialPrinterPlotter.h>
#include<basic_speed_PID.h>


bool verbose = true; //TURN ON FOR DEBUGGING

command_list_enum command;
double target_speed = 200;
double pid_out;

class single_driver{
	
	protected:

		// Objects
		basic_speed_PID pid;
		HBridgeDCmotor HBmotor;
		InterruptSpeedMeasure rotate_count;
		IntervalCheckTimer target_speed_check;		
		
	public:	
	
		// Set up motor
		void setup_motor(int pwm_motor_pin, int direction_pin)
		{
			if(verbose)
				Serial.println("Motor setup");
			
			HBmotor.setup_HBridgeDCmotor(pwm_motor_pin, direction_pin);
		}
		
		// Set up speed measuring unit, hall effect sensor
		void setup_speed_measure(ArduinoInterruptNames measure_pin)  // int_0, pin 2
		{
			if(verbose)
				Serial.println("Speed measure setup");
				
			rotate_count.setupSpeedMeasure(measure_pin);
		}
		
		// Set times between checks for components and system
		void set_time_intervals(int target_speed_time)
		{
			if(verbose)
				Serial.println("Time interval setup");
			target_speed_check.setInterCheck(target_speed_time);
		}
		        
		// Determine motor speed command (low, mid, high) from pushbuttons
		void motor_speed_input(command_list_enum command)
		{	
			switch (command)
			{
				case start:
				if(verbose)
					Serial.println("	Started");
				
				HBmotor.start();	
        		break;
        		
        		case stop:
        		if(verbose)
					Serial.println("	Stopped");  
        		HBmotor.stop();
        		target_speed=0; 
        		break;
        		
        		case reverse:
        		if(verbose)
					Serial.println("	Reversing");
        		HBmotor.changedir();
        		break;
        
        		default:
          		break;
			}
			
		}
		
		// Get motor speed from Hall effect sensor (RPM)
		double read_motor_speed()
		{
			double RPM=rotate_count.getRPMandUpdate();
    		Serial.print("@@@@@@@");
    		Serial.println(RPM);
			return RPM;
    		
		}
		
		// Execute the system task
		void system_execute()
		{		
			double curr_speed;
			
				if(!HBmotor.isStarted())
					motor_speed_input(start);
				
				// PID controller to adjust speed to set point, use target speed check
				if(target_speed_check.isMinChekTimeElapsedAndUpdate())
				{
					curr_speed = read_motor_speed();
					
					if(HBmotor.isStarted())
					{
						pid_out = pid.ComputePID_output(target_speed, curr_speed);
						HBmotor.setSpeedPWM(pid_out);
						Serial.print(target_speed);
						Serial.print(" Hello World ");
						Serial.println(curr_speed);
						
					}
					else
						pid.reset_pidcontrol();
				}
			
		}
};


#endif
