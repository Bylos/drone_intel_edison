/*
 * AttitudeData.cpp
 *
 *  Created on: 11 janv. 2016
 *      Author: Adrien
 */

#include "FlightData.h"

namespace std {

AttitudeData::AttitudeData() {
	roll = 0;
	pitch = 0;
	yaw = 0;
}

AttitudeData::~AttitudeData() {
}

void AttitudeData::Set(float r, float p, float y) {
	roll = r;
	pitch = p;
	yaw = y;
}

float AttitudeData::GetRoll(void) {
	return roll;
}

float AttitudeData::GetPitch(void) {
	return pitch;
}

float AttitudeData::GetYaw(void) {
	return yaw;
}

} /* namespace std */
