#include <InterruptBasedSpeedMeasure.h>

class SpeedMeasure {
	private:
		InterruptSpeedMeasure * left_wheel;
		InterruptSpeedMeasure * right_wheel;
		float wheel_circumference_mm;

		float get_average_rpm()
		{
			float left_rpm = left_wheel->getRPMandUpdate();
			float right_rpm = right_wheel->getRPMandUpdate();

			return (left_rpm+right_rpm)/2.0;
		}

	public:
		SpeedMeasure() {};
		SpeedMeasure(float wheel_diameter_mm, InterruptSpeedMeasure * left_wheel, InterruptSpeedMeasure * right_wheel)
			: left_wheel(left_wheel), right_wheel(right_wheel)
		{
			wheel_circumference_mm = wheel_diameter_mm * 3.1416;
		}

		float get_speed_in_cm_per_m()
		{
			float rpm = get_average_rpm();
			float speed_in_mm_per_m = rpm * wheel_circumference_mm;
			return speed_in_mm_per_m / 10;
		}
};
