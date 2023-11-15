/*
 * TemperatureSensor.hpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Dennis Chen and Tristan
 */

#ifndef INC_TEMPERATURESENSOR_HPP_
#define INC_TEMPERATURESENSOR_HPP_

class TemperatureSensor {
	private:
		float temperature;
		const float heat_capacity_ratio{1.4};
		const float molar_gas_constant{8.3145};
		const float molecular_mass{28.96};

		TemperatureSensor();

	public:
		void sense_temperature();
		float get_speed_of_sound();
};



#endif /* INC_TEMPERATURESENSOR_HPP_ */
