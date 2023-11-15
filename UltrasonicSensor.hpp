/*
 * UltrasonicSensor.hpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Dennis Chen and Tristan Lok
 */

#ifndef INC_ULTRASONICSENSOR_HPP_
#define INC_ULTRASONICSENSOR_HPP_

class UltrasonicSensor {
	private:
		UltrasonicSensor();

	public:
		float compute_distance(float time, float speed);
};

#endif /* INC_ULTRASONICSENSOR_HPP_ */
