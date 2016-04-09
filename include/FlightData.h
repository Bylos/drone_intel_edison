/*
 * FlightData.h
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#ifndef FLIGHTDATA_H_
#define FLIGHTDATA_H_

namespace std {

class FlightData {
public:
	FlightData();
	virtual ~FlightData();
};


class SensorData {
public:
	SensorData();
	virtual ~SensorData();
	void Set(float X, float Y, float Z);
	float GetX(void);
	float GetY(void);
	float GetZ(void);
private:
	float x, y, z;
};


class AccelerometerSensorData: public SensorData {
public:
	AccelerometerSensorData();
	virtual ~AccelerometerSensorData();
};


class GyroscopeSensorData: public SensorData {
public:
	GyroscopeSensorData();
	virtual ~GyroscopeSensorData();
};


class MagnetometerSensorData: public SensorData {
public:
	MagnetometerSensorData();
	virtual ~MagnetometerSensorData();
};

class InertialData {
public:
	InertialData();
	virtual ~InertialData();
	void Set(const AccelerometerSensorData &Accelerometer, const GyroscopeSensorData &Gyroscope, const MagnetometerSensorData &Magnetometer);
	AccelerometerSensorData GetAccelerometer();
	GyroscopeSensorData GetGyroscope();
	MagnetometerSensorData GetMagnetometer();
private:
	AccelerometerSensorData accelerometer;
	GyroscopeSensorData gyroscope;
	MagnetometerSensorData magnetometer;
};

class JoystickData {
public:
	JoystickData();
	virtual ~JoystickData();
	void Set(float LeftX, float LeftY, float RightX, float RightY);
	float GetLeftX(void);
	float GetLeftY(void);
	float GetRightX(void);
	float GetRightY(void);
private:
	float left_x, left_y, right_x, right_y;
};


class AttitudeData {
public:
	AttitudeData();
	virtual ~AttitudeData();
	void Set(float roll, float pitch, float yaw);
	float GetRoll(void);
	float GetPitch(void);
	float GetYaw(void);
private:
	float roll, pitch, yaw;
};

class EscData {
public:
	EscData();
	virtual ~EscData();
	void Set(float FrontLeft, float FrontRight, float RearRight, float RearLeft);
	float GetFrontLeft();
	float GetFrontRight();
	float GetRearRight();
	float GetRearLeft();
private:
	float front_left, front_right, rear_right, rear_left;
};

} /* namespace std */

#endif /* FLIGHTDATA_H_ */
