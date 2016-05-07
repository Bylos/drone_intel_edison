/*
 * SensorData.cpp
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#include "FlightData.h"

namespace std {

SensorData::SensorData() {
	x = 0;
	y = 0;
	z = 0;
}

SensorData::~SensorData() {
}

void SensorData::Set(float X, float Y, float Z) {
	x = X;
	y = Y;
	z = Z;
}

float SensorData::GetX(void) {
	return x;
}

float SensorData::GetY(void) {
	return y;
}

float SensorData::GetZ(void) {
	return z;
}

} /* namespace std */
