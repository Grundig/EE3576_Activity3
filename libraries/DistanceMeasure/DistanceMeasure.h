#ifndef DISTANCEMEASURE
#define DISTANCEMEASURE
#include <InterruptBasedSpeedMeasure.h>

class DistanceMeasure {
	private:
		InterruptSpeedMeasure * left_wheel;
		InterruptSpeedMeasure * right_wheel;
		float total_interrupts_on_wheel;
		float wheel_circumference_mm;

		float get_average_number_of_wheel_rotations()
		{
			float left_wheel_rotations = ((float)left_wheel->GetkDistanceCount())/total_interrupts_on_wheel;
			float right_wheel_rotations = ((float)right_wheel->GetkDistanceCount())/total_interrupts_on_wheel;

			return (left_wheel_rotations+right_wheel_rotations)/2.0;
		}

	public:
		DistanceMeasure() {};
		DistanceMeasure(float wheel_diameter_mm, InterruptSpeedMeasure * left_wheel, InterruptSpeedMeasure * right_wheel)
			: left_wheel(left_wheel), right_wheel(right_wheel)
		{
			total_interrupts_on_wheel = right_wheel->get_tot_interr_on_circle();
			wheel_circumference_mm = wheel_diameter_mm * 3.1416;
		}

		float get_distance_travelled_mm()
		{
			float number_of_wheel_rotations = get_average_number_of_wheel_rotations();
			return number_of_wheel_rotations * wheel_circumference_mm;
		}
};
#endif
