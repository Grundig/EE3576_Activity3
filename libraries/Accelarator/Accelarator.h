#ifndef ACCELARATOR_H
#define ACCELARATOR_H

#include <SpeedMeasure.h>
#include <double_driver.h>
#include <Arduino.h>

/**
 * Author: Keeyan Nejad (51552189)
 * Implements the task where the car drives straight with a set target speed (input parameter);
 * The car has maintains this speed (±Δ%, where Δ is an input parameter) for a certain amount of time (input parameter);
 * The target speed increases by a certain fraction (input parameter);
 * This process repeats until the car has reached a maximum target speed (another input parameter)
 */
class Accelarator {
	private:
		SpeedMeasure speed_measure;
		double_driver * motor;
		float desired_speed;
		float delta;
		float delta_time_ms;
		float speed_increase_cm_per_m;
		float maximum_speed_cm_per_m;
		unsigned long time_within_limits = 0;
		unsigned long previous_check_time;

		void update_motor_speed() {
			float desired_speed_in_rpm = speed_measure.convert_speed_in_cm_per_m_to_rpm(desired_speed);
			motor->set_target_speed_L(desired_speed_in_rpm);
			motor->set_target_speed_R(desired_speed_in_rpm);
		}

		bool speed_is_within_limits() {

			float real_speed = speed_measure.get_speed_in_cm_per_m();
			return abs(real_speed - desired_speed) < delta;
		}
	public:
		Accelarator() {}
		Accelarator(float start_speed_cm_per_m, float delta, float delta_time_ms, float speed_increase_cm_per_m, float maximum_speed_cm_per_m, double_driver * motor) :
			desired_speed(start_speed_cm_per_m), delta(delta), delta_time_ms(delta_time_ms), speed_increase_cm_per_m(speed_increase_cm_per_m), maximum_speed_cm_per_m(maximum_speed_cm_per_m), motor(motor)
	{
		speed_measure = SpeedMeasure(57, motor);

		update_motor_speed();


	}

		void apply_desired_speed() {
			if (speed_is_within_limits()) {
				unsigned long time_since_previous_check = millis() - previous_check_time;
				time_within_limits = time_within_limits + time_since_previous_check;
			} else {
				time_within_limits = 0;
			}

			previous_check_time = millis();

			if (time_within_limits > delta_time_ms) {
				time_within_limits = 0;
				desired_speed = min(desired_speed + speed_increase_cm_per_m, maximum_speed_cm_per_m);
				update_motor_speed();
			}

			motor->execute_task1();
		}
};

#endif
