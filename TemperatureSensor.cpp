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
}

void TemperatureSensor::sense_temperature() {

}

float TemperatureSensor::get_speed_of_sound() {
	float speed_of_sound = sqrt((heat_capacity_ratio * temperature * molar_gas_constant)/molecular_mass);
	return speed_of_sound;
}




