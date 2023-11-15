/*
 * TemperatureSensor.hpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Dennis Chen and Tristan
 */

#ifndef INC_TEMPERATURESENSOR_HPP_
#define INC_TEMPERATURESENSOR_HPP_

class TemperatureSensor() {
	private:
		double temperature{0};
		const double heat_capacity_ratio{1.4};
		const double molar_gas_constant{8.3145};
		const double molecular_mass{28.96};

	public:
		void sense_temperature();
		double get_temperature();
}



#endif /* INC_TEMPERATURESENSOR_HPP_ */
