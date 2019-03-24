#ifndef SPEEDMEASURE_H
#define SPEEDMEASURE_H

#include <InterruptBasedSpeedMeasure.h>

class SpeedMeasure {
	private:
		double_driver * motor;
		float wheel_circumference_mm;
		float last_average_rpm;

		float get_average_rpm()
		{
			float left_rpm = motor->curr_speed_L;
			float right_rpm = motor->curr_speed_R;
			return (left_rpm+right_rpm) / 2.0;
		}

	public:
		SpeedMeasure() {};
		SpeedMeasure(float wheel_diameter_mm, double_driver * motor)
			: motor(motor)
		{
			wheel_circumference_mm = wheel_diameter_mm * 3.1416;
		}

		float get_speed_in_cm_per_m()
		{
			float rpm = get_average_rpm();
			return rpm * (wheel_circumference_mm / 10.0);
		}

		float convert_speed_in_cm_per_m_to_rpm(float speed) {
			return speed/(wheel_circumference_mm / 10.0);
		}
};

#endif
