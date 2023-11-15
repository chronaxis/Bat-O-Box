/*
 * EventLoop.hpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Dennis Chen and Tristan
 */

#ifndef EVENTLOOP_HPP_
#define EVENTLOOP_HPP_

#include <stdint.h>
#include <stdio.h>

// Cpp function to call into main event loop
void EventLoopCpp();

#ifdef __cplusplus
extern "C" {
#endif
	// function to call into Cpp event loop from main
	void EventLoopC();
#ifdef __cplusplus
}
#endif

#endif /* INC_EVENTLOOP_HPP_ */
