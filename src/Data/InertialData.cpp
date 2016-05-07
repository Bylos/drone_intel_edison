/*
 * InertialData.cpp
 *
 *  Created on: 1 janv. 2016
 *      Author: Bylos
 */

#include "FlightData.h"

namespace std {

InertialData::InertialData() {
}

InertialData::~InertialData() {
}

void InertialData::Set(const AccelerometerSensorData &Accelerometer, const GyroscopeSensorData &Gyroscope, const MagnetometerSensorData &Magnetometer) {
	accelerometer = Accelerometer;
	gyroscope = Gyroscope;
	magnetometer = Magnetometer;
}

AccelerometerSensorData InertialData::GetAccelerometer() {
	return accelerometer;
}

GyroscopeSensorData InertialData::GetGyroscope() {
	return gyroscope;
}

MagnetometerSensorData InertialData::GetMagnetometer() {
	return magnetometer;
}

} /* namespace std */
