/*
 * AccelCalibrationMode.cpp
 *
 *  Created on: 10 janv. 2016
 *      Author: Adrien
 */

#include "FlightModes.h"
#include "math.h"

namespace std {

AccelCalibrationMode::AccelCalibrationMode(McuInterface *mcu_int): FlightMode(mcu_int) {

	id = MCU_CMD_MODE_ACCELCALIBRATION;

}

AccelCalibrationMode::~AccelCalibrationMode() {
	// TODO Auto-generated destructor stub
}

void AccelCalibrationMode::init() {


}

int AccelCalibrationMode::RunMode() {
	cout << "Accelerometer calibration running, the quadcopter must be horizontal and must not move\n" ;

	//Time variables
	float sdeltat				= 0.0f; //time without variation of gyro values
	float sdeltatTarget			= 2.0f; //period for getting gyro values
	float initialTime			= mcu_interface->TimeElapsed()/1000.0f; //Initial time
	float currentTime			= initialTime; //Current time

	//gyroscope values
	GyroscopeSensorData avgGyroData;
	GyroscopeSensorData newGyroData;
	//accelerometer values
	AccelerometerSensorData avgAccelData;
	AccelerometerSensorData newAccelData;
	int nVal = 0 ; //number of values used for calibration

	//reset current calibration values
	fvector_t aCalibb ;
	aCalibb.x = 0.0f ; aCalibb.y = 0.0f ; aCalibb.z = 0.0f ;
	fvector_t aCalibs ;
	aCalibs.x = 1.0f ; aCalibs.y = 1.0f ; aCalibs.z = 1.0f ;
	mcu_interface->SetAccelCalib(aCalibb,aCalibs);

	while(!quit) {
		nanosleep((const struct timespec[]){{0, 1000000L}}, NULL); //Free the CPU for 1ms

		//Update of time variables
		currentTime = mcu_interface->TimeElapsed()/1000.0f;
		sdeltat = currentTime - initialTime;
		if (sdeltat >= sdeltatTarget) quit = 1;

		//Handle IMU data sent by MCU
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			newGyroData = inertialData.GetGyroscope();
			newAccelData = inertialData.GetAccelerometer();
			if(nVal ==0) {
				avgGyroData = newGyroData;
				avgAccelData = newAccelData;
			}

			//check if values have changed significantly
			if(fabs(newGyroData.GetX()-avgGyroData.GetX())>10.0f) {
				nVal = 0;
				initialTime = currentTime;
			} else { //otherwise add new value in the average
				nVal ++;
				avgAccelData.Set((avgAccelData.GetX()*nVal+avgAccelData.GetX())/(nVal+1),
						(avgAccelData.GetY()*nVal+avgAccelData.GetY())/(nVal+1),
						(avgAccelData.GetZ()*nVal+avgAccelData.GetZ())/(nVal+1));
			}
		}
	}

	//Set new calibration values
	aCalibb.x = avgAccelData.GetX() ; aCalibb.y = avgAccelData.GetY() ; aCalibb.z = avgAccelData.GetZ()-1.0f ;
	mcu_interface->SetAccelCalib(aCalibb, aCalibs);
	cout << "New accelerometer calibration values: " << endl;
	cout << aCalibb.x << " " << aCalibb.y << " " << aCalibb.z << endl;
	cout << aCalibs.x << " " << aCalibs.y << " " << aCalibs.z << endl;

	quit = 0;
	AccelerometerSensorData accelData ;
	while(!quit) {
		nanosleep((const struct timespec[]){{0, 100000000L}}, NULL);
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			accelData = inertialData.GetAccelerometer();
			cout << accelData.GetX() << " " << accelData.GetY() << " " << accelData.GetZ() << endl;
		}
	}
	return 0;
}

} /* namespace std */
