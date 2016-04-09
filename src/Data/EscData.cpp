/*
 * EscData.cpp
 *
 *  Created on: 1 janv. 2016
 *      Author: Bylos
 */

#include "FlightData.h"

namespace std {

EscData::EscData() {
	front_left = front_right = rear_right = rear_left = 0;
}

EscData::~EscData() {
	// TODO Auto-generated destructor stub
}

void EscData::Set(float FrontLeft, float FrontRight, float RearRight, float RearLeft) {
	front_left = FrontLeft;
	front_right = FrontRight;
	rear_right = RearRight;
	rear_left = RearLeft;
}

float EscData::GetFrontLeft() {
	return front_left;
}
float EscData::GetFrontRight() {
	return front_right;
}

float EscData::GetRearRight() {
	return rear_right;
}

float EscData::GetRearLeft() {
	return rear_left;
}

} /* namespace std */
