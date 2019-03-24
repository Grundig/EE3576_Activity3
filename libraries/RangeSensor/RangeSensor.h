#ifndef RANGE_SENSOR_H
#define RANGE_SENSOR_H

#include <Arduino.h>

class RangeSensor {
	private:
		int trigger_pin;
		int echo_pin;
		int safe_distance;
	public:
		RangeSensor() {}
		RangeSensor(int trigger_pin, int echo_pin, int safe_distance) : trigger_pin(trigger_pin), echo_pin(echo_pin), safe_distance(safe_distance) {
			pinMode(trigger_pin, OUTPUT);
			pinMode(echo_pin, INPUT);
		}

		bool safe() {
			// Ensure the trigger is low
			digitalWrite(trigger_pin, LOW);
			delayMicroseconds(2);

			// Sets the trigger_pin on HIGH state for 10 micro seconds
			digitalWrite(trigger_pin, HIGH);
			delayMicroseconds(10);
			digitalWrite(trigger_pin, LOW);

			// Reads the echo_pin, returns the sound wave travel time in microseconds
			float duration = pulseIn(echo_pin, HIGH);

			// Multiply by the speed of sound (340 m/s or 0.034 cm/µs)
			// The signal bounces of an obstacle and then must return.
			// So divide the result by 2 and we have our distance in MS
			float distance = duration*0.034/2;

			return distance > safe_distance;
		}
};

#endif
