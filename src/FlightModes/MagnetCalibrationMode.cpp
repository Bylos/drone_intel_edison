/*
 * MagnetCalibrationMode.cpp
 *
 *  Created on: 10 janv. 2016
 *      Author: Adrien
 */

#include "FlightModes.h"
#include "ellipseCalc.h"

namespace std {

MagnetCalibrationMode::MagnetCalibrationMode(McuInterface *mcu_int): FlightMode(mcu_int) {

	id = MCU_CMD_MODE_MAGNCALIBRATION;

}

MagnetCalibrationMode::~MagnetCalibrationMode() {
}

void MagnetCalibrationMode::init() {


}

int MagnetCalibrationMode::RunMode() {
	cout << "Magnetometer calibration running, move the quadcopter until message says OK\n" ;

	//Time variables
	float sdeltat				= 0.0f; //time without variation of gyro values
	float maxDuration			= 120.0f; //max duration of data acquisition
	float initialTime			= mcu_interface->TimeElapsed()/1000.0f; //Initial time
	float currentTime			= initialTime; //Current time

	//Variable for checking that all directions have been pointed (xp, xm, yp, ym, zp, zm)
	int sampleComplete = 0;

	//Magnetometer values
	MagnetometerSensorData magnetDataTable[4000]; //table for storing the magneto data
	int nValTot = 0 ; //total number of values used for calibration
	int minValNb = 100; //min number of values to be recorded in each octant
	int nValOctmmm = 0, nValOctmmp = 0, // one counter for each octant
			nValOctmpm = 0, nValOctmpp = 0,
			nValOctpmm = 0, nValOctpmp = 0,
			nValOctppm = 0, nValOctppp = 0;

	//reset current calibration values
	fvector_t mCalibb; fvector_t mCalibx; fvector_t mCaliby; fvector_t mCalibz;
	mCalibb.x = 0.0f ; mCalibb.y = 0.0f ; mCalibb.z = 0.0f ;
	mCalibx.x = 1.0f ; mCalibx.y = 0.0f ; mCalibx.z = 0.0f ;
	mCaliby.x = 0.0f ; mCaliby.y = 1.0f ; mCaliby.z = 0.0f ;
	mCalibz.x = 0.0f ; mCalibz.y = 0.0f ; mCalibz.z = 1.0f ;
	mcu_interface->SetMagnetoCalib(mCalibb,mCalibx,mCaliby,mCalibz);

	while(!sampleComplete && sdeltat<maxDuration) {
		nanosleep((const struct timespec[]){{0, 100000L}}, NULL); //Free the CPU for 50ms

		//Update of time variables
		currentTime = mcu_interface->TimeElapsed()/1000.0f;
		sdeltat = currentTime - initialTime;

		//Handle IMU data sent by MCU
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			magnetDataTable[nValTot] = inertialData.GetMagnetometer();
			cout << nValOctmmm << " " << nValOctmmp << " " << nValOctmpm << " " << nValOctmpp << " " <<
					nValOctpmm << " " << nValOctpmp << " " << nValOctppm << " " << nValOctppp << " " << endl;

			//Determine the octant contatining the data point
			if(magnetDataTable[nValTot].GetX()< 0.0 && magnetDataTable[nValTot].GetY()< 0.0 && magnetDataTable[nValTot].GetZ()< 0.0) nValOctmmm++;
			if(magnetDataTable[nValTot].GetX()< 0.0 && magnetDataTable[nValTot].GetY()< 0.0 && magnetDataTable[nValTot].GetZ()> 0.0) nValOctmmp++;
			if(magnetDataTable[nValTot].GetX()< 0.0 && magnetDataTable[nValTot].GetY()> 0.0 && magnetDataTable[nValTot].GetZ()< 0.0) nValOctmpm++;
			if(magnetDataTable[nValTot].GetX()< 0.0 && magnetDataTable[nValTot].GetY()> 0.0 && magnetDataTable[nValTot].GetZ()> 0.0) nValOctmpp++;
			if(magnetDataTable[nValTot].GetX()> 0.0 && magnetDataTable[nValTot].GetY()< 0.0 && magnetDataTable[nValTot].GetZ()< 0.0) nValOctpmm++;
			if(magnetDataTable[nValTot].GetX()> 0.0 && magnetDataTable[nValTot].GetY()< 0.0 && magnetDataTable[nValTot].GetZ()> 0.0) nValOctpmp++;
			if(magnetDataTable[nValTot].GetX()> 0.0 && magnetDataTable[nValTot].GetY()> 0.0 && magnetDataTable[nValTot].GetZ()< 0.0) nValOctppm++;
			if(magnetDataTable[nValTot].GetX()> 0.0 && magnetDataTable[nValTot].GetY()> 0.0 && magnetDataTable[nValTot].GetZ()> 0.0) nValOctppp++;

			if(nValOctmmm>minValNb && nValOctmmp>minValNb && nValOctmpm>minValNb && nValOctmpp>minValNb&&
					nValOctpmm>minValNb && nValOctpmp>minValNb && nValOctppm>minValNb && nValOctppp>minValNb ) sampleComplete = 1 ;
			nValTot ++;
		}
	}

	if(!sampleComplete) { //If time elapsed without proper data collected
		cout << "Magnetometer calibration failure: all the directions could not be recorded" << endl;
	}

	else {
		//Calculate new calibration values
		gyroCalib_t calibData = ellipseCalc(magnetDataTable, nValTot);
		mCalibb.x = calibData.b.x  ; mCalibb.y = calibData.b.y  ; mCalibb.z = calibData.b.z  ;
		mCalibx.x = calibData.a1.x ; mCalibx.y = calibData.a1.y ; mCalibx.z = calibData.a1.z ;
		mCaliby.x = calibData.a2.x ; mCaliby.y = calibData.a2.y ; mCaliby.z = calibData.a2.z  ;
		mCalibz.x = calibData.a3.x ; mCalibz.y = calibData.a3.y ; mCalibz.z = calibData.a3.z ;

		//Set the new calibration values
		mcu_interface->SetMagnetoCalib(mCalibb,mCalibx,mCaliby,mCalibz);
		cout << "New magneto calibration values: " << endl;
		cout << "Bias:   " <<  mCalibb.x << " " << mCalibb.y << " " << mCalibb.z << endl;
		cout << "X axis: " <<  mCalibx.x << " " << mCalibx.y << " " << mCalibx.z << endl;
		cout << "Y axis: " <<  mCaliby.x << " " << mCaliby.y << " " << mCaliby.z << endl;
		cout << "Z axis: " <<  mCalibz.x << " " << mCalibz.y << " " << mCalibz.z << endl;
	}


	MagnetometerSensorData magData ;
	while(!quit) {
		nanosleep((const struct timespec[]){{0, 100000000L}}, NULL);
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			magData = inertialData.GetMagnetometer();
			cout << magData.GetX() << " " << magData.GetY() << " " << magData.GetZ() << endl;
		}
	}

	return 0;
}

} /* namespace std */





