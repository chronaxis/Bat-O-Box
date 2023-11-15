/*
 * TemperatureSensor.cpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Dennis Chen and Tristan
 */

#include "TemperatureSensor.hpp"
#include <cmath>

TemperatureSensor::TemperatureSensor() {
	float temperature{0};
	const float heat_capacity_ratio{1.4};
	const float molar_gas_constant{8.3145};
	const float molecular_mass{28.96};
}

void TemperatureSensor::sense_temperature() {

}

float TemperatureSensor::get_speed_of_sound() {
	float speed_of_sound = math.sqrt((heat_capacity_ratio * temperature * molar_gas_constant)/molercular_mass));
	return speed_of_sound;
}




