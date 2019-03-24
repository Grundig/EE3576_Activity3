#ifndef single_driver_h
#define single_driver_h


#include<Arduino.h>
#include<Basic_Input.h> 
#include<IntervalCheckTimer.h>
#include<DCmotor.h>
#include<InterruptBasedSpeedMeasure.h>
#include<InterruptBasedInputs.h>
#include<basic_speed_PID.h>
#include<SpeedMeasure.h>


bool verbose = false; //TURN ON FOR DEBUGGING
bool printed = false;
enum command_list {start_R, start_L, stop_R, stop_L, reverse_R, reverse_L};

double target_speed_R;
double target_speed_L;
double pid_out_R;
double pid_out_L;

class double_driver{

	protected:

		// Objects
		basic_speed_PID pid_R, pid_L;
		HBridgeDCmotor motor_R, motor_L;
		InterruptSpeedMeasure rotation_counter_R, rotation_counter_L;
		IntervalCheckTimer speed_check;
		SpeedMeasure speed_measure;

	public:

		// Constructor
		double_driver(){}

		// Set up motor
		void setup_motor(int motorpin_R, int directionpin_R, int motorpin_L, int directionpin_L)
		{
			if(verbose)
				Serial.println("Motor setup");

			motor_R.setup_HBridgeDCmotor(motorpin_R,directionpin_R);
			motor_L.setup_HBridgeDCmotor(motorpin_L,directionpin_L);
		}

		// Set up speed measuring unit, hall effect sensor
		void setup_speed_measure(ArduinoInterruptNames measure_pin_R, ArduinoInterruptNames measure_pin_L)  // int_0 = pin 2, int_1 = pin 3
		{
			if(verbose)
				Serial.println("Speed measure setup");

			rotation_counter_R.setupSpeedMeasure(measure_pin_R);
			rotation_counter_L.setupSpeedMeasure(measure_pin_L);

			speed_measure = SpeedMeasure(57, &rotation_counter_L, &rotation_counter_R);
		}

		//Set up PID values
		void setup_pid_R(){
			double kp=1; 
			double ki=0.1; 
			double kd=0;
			double PIDoutMin=0;
			double PIDoutMax=255;
			pid_R= basic_speed_PID(kp, ki, kd, PIDoutMin, PIDoutMax);

		}

		void setup_pid_L(){
			double kp=1; 
			double ki=0.1; 
			double kd=0;
			double PIDoutMin=0;
			double PIDoutMax=255;
			pid_L = basic_speed_PID(kp, ki, kd, PIDoutMin, PIDoutMax);

		}

		// Set times between checks for components and system
		void set_time_intervals(int target_speed_time)
		{
			if(verbose)
				Serial.println("Time interval setup");
			speed_check.setInterCheck(target_speed_time);
		}

		void set_target_speed_R(int speed_R){
			target_speed_R = speed_R;
		}

		void set_target_speed_L(int speed_L){
			target_speed_L = speed_L;
		}

		// Determine motor speed command (low, mid, high) from pushbuttons
		void motor_speed_input(command_list command)
		{	
			switch (command)
			{
				case start_R:
					if(verbose)
						Serial.println("	Started RIGHT");

					motor_R.start();	
					break;

				case start_L:
					if(verbose)
						Serial.println("	Started LEFT");

					motor_L.start();	
					break;

				case stop_R:
					if(verbose)
						Serial.println("	Stopped RIGHT");  

					motor_R.stop();
					target_speed_R=0; 
					break;

				case stop_L:
					if(verbose)
						Serial.println("	Stopped LEFT");  

					motor_L.stop();
					target_speed_L=0; 
					break;

				case reverse_R:
					if(verbose)
						Serial.println("	Reversing RIGHT");
					motor_R.changedir();

					break;

				case reverse_L:
					if(verbose)
						Serial.println("	Reversing LEFT");
					motor_L.changedir();
					break;

				default:
					break;
			}

		}

		// Get motor speed from Hall effect sensor (RPM)
		double read_motor_speed(int side)
		{
			double RPM;
			if(side == 0){
				RPM=rotation_counter_R.getRPMandUpdate();
				//    			Serial.print("Right: ");
				//				Serial.print(RPM);
			}
			if(side == 1){
				RPM=rotation_counter_L.getRPMandUpdate();
				//    			Serial.print(" Left: ");
				//    			Serial.println(RPM);
			}
			return RPM;

		}

		// Execute the system task
		void execute_task1()
		{		
			if(verbose && printed == false){
				printed = true;
				Serial.println("Running task_1");
			}

			double curr_speed_R;
			double curr_speed_L;


			if(!motor_R.isStarted())
				motor_speed_input(start_R);
			if(!motor_L.isStarted())
				motor_speed_input(start_L);

			// PID controller to adjust speed to set point, use target speed check
			if(speed_check.isMinChekTimeElapsedAndUpdate()){
				curr_speed_R = read_motor_speed(0);
				curr_speed_L = read_motor_speed(1);



				if(motor_R.isStarted())
				{
					Serial.print("RIGHT: ");
					Serial.print(curr_speed_R);
					Serial.print(" ");
					Serial.print(target_speed_R);
					Serial.print(" PWM right: ");
					Serial.print(pid_out_R);
					Serial.print(" ");
					pid_out_R = pid_R.ComputePID_output(target_speed_R, curr_speed_R);
					motor_R.setSpeedPWM(pid_out_R);



				}
				if(motor_L.isStarted())
				{
					Serial.print("LEFT: ");
					Serial.print(curr_speed_L);
					Serial.print(" ");
					Serial.print(target_speed_L);

					Serial.print(" PWM left: ");
					Serial.print(pid_out_L);
					Serial.print(" ");
					pid_out_L = pid_L.ComputePID_output(target_speed_L, curr_speed_L);
					motor_L.setSpeedPWM(pid_out_L);



					Serial.println("-------------------");
				}

				Serial.println(rotation_counter_R.GetkDistanceCount());
				Serial.println(rotation_counter_L.GetkDistanceCount());

				//				else
				//					pid_L.reset_pidcontrol();
				//					pid_R.reset_pidcontrol();
			}

			Serial.println(speed_measure.get_speed_in_cm_per_m());
		}
};

#endif
