#ifndef single_driver_h
#define single_driver_h


#include<Arduino.h>
#include<Basic_Input.h> 
#include<IntervalCheckTimer.h>
#include<DCmotor.h>
#include<InterruptBasedSpeedMeasure.h>
#include<InterruptBasedInputs.h>
#include<basic_speed_PID.h>
#include<DistanceMeasure.h>

bool verbose = true; //TURN ON FOR DEBUGGING
bool printed = false;
enum command_list {start_R, start_L, stop_R, stop_L, reverse_R, reverse_L, spin_R, spin_L};

double target_speed_R;
double target_speed_L;
double pid_out_R;
double pid_out_L;
int spin_distance = 80;
bool finished = false;
int i = 0;

			
class double_driver{
	
	protected:

		// Objects
		basic_speed_PID pid_R, pid_L;
		HBridgeDCmotor motor_R, motor_L;
		InterruptSpeedMeasure rotation_counter_R, rotation_counter_L;
		IntervalCheckTimer speed_check;	
		DistanceMeasure distance;	
		
	public:	

		double curr_speed_R;
		double curr_speed_L;

	
		// Constructor
		double_driver(){	
			distance = DistanceMeasure(57, &rotation_counter_R, &rotation_counter_L);
		}
		
		
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
	        		break;
	        		
        		case stop_L:
	        		if(verbose)
						Serial.println("	Stopped LEFT");  
	        		
					motor_L.stop();
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
	        		
	        	case spin_R:
	        		if(verbose)
	        			Serial.println("	Spinning RIGHT");
	        		rotation_counter_L.reset_distancecount();
					motor_speed_input(stop_R);
					motor_speed_input(start_L);
					motor_L.setSpeedPWM(150); //for stable turning 					
					while(!rotation_counter_L.checkDistanceMet(spin_distance)){
						continue;
					}
					motor_speed_input(stop_L);
					rotation_counter_L.reset_distancecount();
					break;
	        
        		case spin_L:
	        		if(verbose)
	        			Serial.println("	Spinning LEFT");
	        		rotation_counter_R.reset_distancecount();
					motor_speed_input(stop_L);
					motor_speed_input(start_R);
					motor_R.setSpeedPWM(150); //for stable turning
	        			while(!rotation_counter_R.checkDistanceMet(spin_distance)){
						continue;
					}
					motor_speed_input(stop_R);
					rotation_counter_R.reset_distancecount();
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
		
		void turn(float turn_radius){
			float inner_radius = turn_radius - 5;
			float outer_radius = turn_radius + 5;
			double turn_speed_R = target_speed_L*(inner_radius/outer_radius);
			
			motor_speed_input(start_R);
			motor_speed_input(start_L);
			if(speed_check.isMinChekTimeElapsedAndUpdate()){
				curr_speed_R = read_motor_speed(0);
				curr_speed_L = read_motor_speed(1);
			
				pid_out_R = pid_R.ComputePID_output(turn_speed_R, curr_speed_R);
				motor_R.setSpeedPWM(pid_out_R);	
				pid_out_L = pid_L.ComputePID_output(target_speed_L, curr_speed_L);
				motor_L.setSpeedPWM(pid_out_L);	
			}
			
		}
		
		// Execute the system task
		void execute_task1(float goal_distance, bool reverse){		
			finished = false;
			
			if(verbose && printed == false){
			printed = true;
			Serial.println("Running task_1");
			}
			
			
			if(!motor_R.isStarted()){
				rotation_counter_R.reset_distancecount();
				motor_speed_input(start_R);
			}
			
			if(!motor_L.isStarted()){
				rotation_counter_L.reset_distancecount();
				motor_speed_input(start_L);
			}
			
			// PID controller to adjust speed to set point, use target speed check
			if(speed_check.isMinChekTimeElapsedAndUpdate()){
				curr_speed_R = read_motor_speed(0);
				curr_speed_L = read_motor_speed(1);
				
				if(distance.get_distance_travelled_mm() >= goal_distance*10){
					rotation_counter_R.reset_distancecount();
					rotation_counter_L.reset_distancecount();
					if(reverse){
						motor_speed_input(reverse_R);
						motor_speed_input(reverse_L);		
					} else{
						motor_speed_input(stop_R);
						motor_speed_input(stop_L);
					}
					finished = true;
				}
			
			
				if(motor_R.isStarted()){
//					Serial.print("RIGHT: ");
//					Serial.print(curr_speed_R);
//					Serial.print(" ");
//					Serial.print(target_speed_R);
//					Serial.print(" PWM right: ");
//					Serial.print(pid_out_R);
//					Serial.print(" ");

					pid_out_R = pid_R.ComputePID_output(target_speed_R, curr_speed_R);
					motor_R.setSpeedPWM(pid_out_R);	
				}
				
				if(motor_L.isStarted()){
//					Serial.print("LEFT: ");
//					Serial.print(curr_speed_L);
//					Serial.print(" ");
//					Serial.print(target_speed_L);			
//					Serial.print(" PWM left: ");
//					Serial.print(pid_out_L);
//					Serial.print(" ");
//					Serial.println("-------------------");

					pid_out_L = pid_L.ComputePID_output(target_speed_L, curr_speed_L);
					motor_L.setSpeedPWM(pid_out_L);
				}
			}			
		}
		
		void execute_task3(int distance_array[], bool turns_array[]){
			if(i<10){
				while (finished == false){
	//				Serial.println("finished false");
					execute_task1(distance_array[i], false);
				}
				finished = false;
					
				if (turns_array[i]){
					motor_speed_input(spin_R);
				} else{
					motor_speed_input(spin_L);
				}
				i++;
			}
		}

		enum state_list {pre_turn_straight, turning, post_turn_straight, finish};
		state_list state = pre_turn_straight;
		int round = 1;
		
		void execute_task4(int straight, int turn_radius, int laps){
			
			if (state == pre_turn_straight) {
				Serial.println("hello world!");
				execute_task1(straight/2, false);
					if(finished){
						state = turning;
						pid_R.reset_pidcontrol();
						pid_L.reset_pidcontrol();
					}
				
			}
			
			if (state == turning ) {
				Serial.println("turning");
				turn(turn_radius);
				if (distance.get_distance_travelled_mm() > turn_radius*3.1416*13.4) {
					state = post_turn_straight;
					rotation_counter_R.reset_distancecount();
					rotation_counter_L.reset_distancecount();
					pid_R.reset_pidcontrol();
					pid_L.reset_pidcontrol();
				}
			}
			if (state == post_turn_straight) {
				execute_task1(straight/2, false);
				Serial.println("post");
				if (finished) {
					state = pre_turn_straight;
					pid_R.reset_pidcontrol();
					pid_L.reset_pidcontrol();
					round += 1;
				}
			}	
			
			if (round>laps*2){
				motor_speed_input(stop_R);
				motor_speed_input(stop_L);
				state = finish;
			}
				
		}
		
};


#endif
