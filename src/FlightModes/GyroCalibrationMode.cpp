/*
 * GyroCalibrationMode.cpp
 *
 *  Created on: 10 janv. 2016
 *      Author: Adrien
 */

#include "FlightModes.h"
#include "math.h"

namespace std {

GyroCalibrationMode::GyroCalibrationMode(McuInterface *mcu_int): FlightMode(mcu_int) {

	id = MCU_CMD_MODE_GYROCALIBRATION;

}

GyroCalibrationMode::~GyroCalibrationMode() {
}

void GyroCalibrationMode::init() {


}

int GyroCalibrationMode::RunMode() {

	cout << "Gyroscope calibration running, do not move the quadcopter\n" ;

	//Time variables
	float sdeltat				= 0.0f; //time without variation of gyro values
	float sdeltatTarget			= 2.0f; //period for getting gyro values
	float initialTime			= mcu_interface->TimeElapsed()/1000.0f; //Initial time
	float currentTime			= initialTime; //Current time

	//gyroscope values
	GyroscopeSensorData avgGyroData;
	GyroscopeSensorData newGyroData;
	int nVal = 0 ; //number of values used for calibration

	//reset current calibration values
	fvector_t gCalib ;
	gCalib.x = 0.0f ; gCalib.y = 0.0f ; gCalib.z = 0.0f ;
	mcu_interface->SetGyroCalib(gCalib);

	while(!quit) {
		nanosleep((const struct timespec[]){{0, 100000L}}, NULL); //Free the CPU for 1ms

		//Update of time variables
		currentTime = mcu_interface->TimeElapsed()/1000.0f;
		sdeltat = currentTime - initialTime;
		if (sdeltat >= sdeltatTarget) quit = 1;

		//Handle IMU data sent by MCU
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			newGyroData = inertialData.GetGyroscope();
			if(nVal ==0) avgGyroData = newGyroData;

			//check if values have changed significantly
			if(fabs(newGyroData.GetX()-avgGyroData.GetX())>10.0f) {
				nVal = 0;
				initialTime = currentTime;
			} else if(fabs(newGyroData.GetY()-avgGyroData.GetY())>10.0f) {
				nVal = 0;
				initialTime = currentTime;
			} else if(fabs(newGyroData.GetZ()-avgGyroData.GetZ())>10.0f) {
				nVal = 0;
				initialTime = currentTime;
			} else { //otherwise add new value in the average
				nVal ++;
				avgGyroData.Set((avgGyroData.GetX()*nVal+newGyroData.GetX())/(nVal+1),
						(avgGyroData.GetY()*nVal+newGyroData.GetY())/(nVal+1),
						(avgGyroData.GetZ()*nVal+newGyroData.GetZ())/(nVal+1));
			}

		}
	}

	//Set new calibration values
	gCalib.x = avgGyroData.GetX() ; gCalib.y = avgGyroData.GetY() ; gCalib.z = avgGyroData.GetZ();
	mcu_interface->SetGyroCalib(gCalib);
	cout << "New gyro calibration values: " << avgGyroData.GetX() << " " << avgGyroData.GetY() << " " << avgGyroData.GetZ() << endl;

	quit = 0;
	GyroscopeSensorData gyroData ;
	FILE* gyrolog = fopen("/etc/drone/log_gyro.csv", "w");
	while(!quit) {
		nanosleep((const struct timespec[]){{0, 100000L}}, NULL);
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			gyroData = inertialData.GetGyroscope();
			//cout << gyroData.GetX() << " " << gyroData.GetY() << " " << gyroData.GetZ() << endl;
			fprintf(gyrolog, "%08u; %+08.3f; %+08.3f; %08.3f\n", mcu_interface->TimeElapsed(), gyroData.GetX(), gyroData.GetY(), gyroData.GetZ());
		}
	}
	fclose(gyrolog);
	return 0;
}

} /* namespace std */



