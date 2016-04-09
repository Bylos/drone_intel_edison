/*
 * ControllerData.cpp
 *
 *  Created on: 1 janv. 2016
 *      Author: Bylos
 */

#include "FlightData.h"

namespace std {

JoystickData::JoystickData() {
	left_x = 0;
	left_y = 0;
	right_x = 0;
	right_y = 0;
}

JoystickData::~JoystickData() {
	// TODO Auto-generated destructor stub
}

void JoystickData::Set(float LeftX, float LeftY, float RightX, float RightY) {
	left_x = LeftX;
	left_y = LeftY;
	right_x = RightX;
	right_y = RightY;
}

float JoystickData::GetLeftX(void) {
	return left_x;
}

float JoystickData::GetLeftY(void) {
	return left_y;
}

float JoystickData::GetRightX(void) {
	return right_x;
}

float JoystickData::GetRightY(void) {
	return right_y;
}

} /* namespace std */
