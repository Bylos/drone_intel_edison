/*
 * StabilizedFlightMode.cpp
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#include "FlightModes.h"
#include "McuInterface.h"
#include "AHRS.h"

namespace std {

StabilizedFlightMode::StabilizedFlightMode(McuInterface *mcu_int): FlightMode(mcu_int) {

	init() ;

	id = MCU_CMD_MODE_STABILIZED;

}

StabilizedFlightMode::~StabilizedFlightMode() {
	//
}

void StabilizedFlightMode::init() {


}

int StabilizedFlightMode::RunMode() {

	cout << "Stabilized FlightMode running\n" ;

	//Time variables
	float sdeltat					= 0.0f; //Time increment
	float lastTime					= mcu_interface->TimeElapsed()/1000.0f; //Last time
	float currentTime				= lastTime; //New time

	//Global regulation variables
	float commonPower           	= 0.0f ;

	//TODO !!GET PID VARIABLES FROM MODE CONFIGURATION FILE !!

	ahrs_filter_t IMUfilter = AHRS_MADGWICK_2015;

	//Regulation variables for Yaw rate
	float yawDifferentialPower   	= 0.0f ;
	PIDController yawRatePID   		= PIDController(0.5f, 0.25f, 0.0f, 2.0f) ; //TIPTOP

	//Regulation variables for Pitch rate
	float pitchDifferentialPower 	= 0.0f ;
	PIDController pitchRatePID 		= PIDController(0.13f, 0.3f, 0.003f, 2.0f) ; //TIPTOP
	float targetPitchRate 			= 0.0f ;
	PIDController pitchAnglePID 	= PIDController(3.5f, 0.0f, 0.0f, 0.0f) ;


	//Regulation variables for Roll rate
	float rollDifferentialPower  	= 0.0f ;
	PIDController rollRatePID  		= PIDController(0.18f, 0.3f, 0.003f, 2.0f) ; //TIPTOP
	float targetRollRate 			= 0.0f ;
	PIDController rollAnglePID 		= PIDController(3.5f, 0.0f, 0.0f, 0.0f) ;

	//ESC data
	EscData escData;

	//Gyroscope data
	AccelerometerSensorData accel;
	GyroscopeSensorData gyro;
	MagnetometerSensorData magnet;

	//initialization of ahrs
	cout << "AHRS initializing..." ;
	while(ahrs_init(inertialData, IMUfilter) == 0) {
		nanosleep((const struct timespec[]){{0, (int)(AHRS_INIT_UPDATE_PERIOD*1000000000)}}, NULL);
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
		}
	}
	cout << " OK\n" ;

	//Main loop of the mode
	while(!quit) {
		nanosleep((const struct timespec[]){{0, 1000000L}}, NULL);

		//Update of time variables
		currentTime = mcu_interface->TimeElapsed()/1000.0f;
		sdeltat = currentTime - lastTime;

		//Handle events sent by MCU
		if (mcu_interface->GetEventFlag()) {
			//int event = mcu_interface->GetEvent(); //No event management so far
		}

		//Handle joystick/RC data sent by MCU
		if (mcu_interface->GetJoystickDataFlag()) {
			joystickData = mcu_interface->GetJoystickData();
		}

		//Handle IMU data sent by MCU and update PID ouputs
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
			gyro = inertialData.GetGyroscope();

			//AHRS update
			attitudeData = ahrs_orientation_update(inertialData,IMUfilter,sdeltat);
			//cout << attitudeData.GetRoll() << " " << attitudeData.GetPitch() << " " << attitudeData.GetYaw() << "\n";

			//Common power update
			commonPower = 0.8*joystickData.GetRightY();

			//roll PID update
			targetRollRate = rollAnglePID.getControllerOutput(attitudeData.GetRoll(),joystickData.GetRightX(), sdeltat);
			rollDifferentialPower 	= rollRatePID.getControllerOutput	( gyro.GetX(), targetRollRate, sdeltat);

			//pitch PID update
			targetPitchRate = pitchAnglePID.getControllerOutput(attitudeData.GetPitch(),-joystickData.GetLeftY(), sdeltat);
			pitchDifferentialPower 	= pitchRatePID.getControllerOutput	( gyro.GetY(), targetPitchRate, sdeltat);

			//yaw PID update
			yawDifferentialPower 	= yawRatePID.getControllerOutput	(  gyro.GetZ(), joystickData.GetLeftX(),sdeltat);

			/*
			cout << min(max(commonPower+rollDifferentialPower+pitchDifferentialPower-yawDifferentialPower,10.0f),100.0f) << " " //Front left
				 <<	min(max(commonPower-rollDifferentialPower+pitchDifferentialPower+yawDifferentialPower,10.0f),100.0f) << " " //Front right
				 <<	min(max(commonPower-rollDifferentialPower-pitchDifferentialPower-yawDifferentialPower,10.0f),100.0f) << " " //Back right
				 <<	min(max(commonPower+rollDifferentialPower-pitchDifferentialPower+yawDifferentialPower,10.0f),100.0f) << "\n";  //Back left
			*/

			//Set esc data variable and send values to MCU
			escData.Set(
					min(max(commonPower+rollDifferentialPower+pitchDifferentialPower-yawDifferentialPower,10.0f),100.0f), //Front left
					min(max(commonPower-rollDifferentialPower+pitchDifferentialPower+yawDifferentialPower,10.0f),100.0f), //Front right
					min(max(commonPower-rollDifferentialPower-pitchDifferentialPower-yawDifferentialPower,10.0f),100.0f), //Back right
					min(max(commonPower+rollDifferentialPower-pitchDifferentialPower+yawDifferentialPower,10.0f),100.0f)  //Back left
					);

			//TODO remettre l envoi vers les ESC
			mcu_interface->SetEsc(escData,1);

			//last time is the current time
			lastTime = currentTime;

		}
	}

	return 0;
}

} /* namespace std */
