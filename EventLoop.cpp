/*
 * EventLoop.cpp
 *
 *  Created on: Nov 14, 2023
 *      Author: Dennis Chen and Tristan
 */

#include "EventLoop.hpp"
#include "CppLedBlink.hpp"

//Main Cpp event loop
void EventLoopCpp() {
	CppLedBlink testing;
}

// Define all C function calls from main.c below
extern "C" {
	void EventLoopC() {
		EventLoopCpp();
	}
}
