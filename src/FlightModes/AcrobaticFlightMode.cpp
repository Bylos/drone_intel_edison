/*
 * AcrobaticFlightMode.cpp
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#include "FlightModes.h"
#include "McuInterface.h"

namespace std {

AcrobaticFlightMode::AcrobaticFlightMode(McuInterface *mcu_int): FlightMode(mcu_int) {

	//cout << "Entering AcrobaticFlightMode constructor\n" ;

	init() ;

	id = MCU_CMD_MODE_ACROBATIC;

	cout << "Acrobatic flight mode object created\n" ;
}

AcrobaticFlightMode::~AcrobaticFlightMode() {
	// TODO Auto-generated destructor stub
}

void AcrobaticFlightMode::init() {

	cout << "Acrobatic flight mode initialization\n" ;

}

int AcrobaticFlightMode::RunMode() {

	cout << "Acrobatic flight mode running\n" ;

	//Time variables
	float sdeltat				= 0.0f; //Time increment
	float lastTime				= mcu_interface->TimeElapsed()/1000.0f; //Last time
	float currentTime			= lastTime; //New time

	//Global regulation variables
	float commonPower           = 0.0f ;

	//TODO !!GET PID VARIABLES FROM MODE CONFIGURATION FILE !!

	//Regulation variables for Yaw rate
	float yawRateDifferentialPower   = 0.0f ;
	PIDController yawRatePID   = PIDController(0.5f, 0.5f, 0.0f, 2.0f) ; //TIPTOP

	//Regulation variables for Pitch rate
	float pitchRateDifferentialPower = 0.0f ;
	PIDController pitchRatePID = PIDController(0.13f, 0.3f, 0.003f, 2.0f) ; //TIPTOP


	//Regulation variables for Roll rate
	float rollRateDifferentialPower  = 0.0f ;
	PIDController rollRatePID  = PIDController(0.18f, 0.3f, 0.003f, 2.0f) ; //TIPTOP

	//ESC data
	EscData escData;

	//Gyroscope data
	GyroscopeSensorData gyro;

	//Main loop of the mode
	while(!quit) {
		nanosleep((const struct timespec[]){{0, 1000000L}}, NULL); //Free the CPU for 1ms

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
			//cout << "JS Data reiceived: " << joystickData.GetLeftX() << " " << joystickData.GetLeftY() << " " << joystickData.GetRightX() << " " << joystickData.GetRightY() << "\n";
		}

		//Handle IMU data sent by MCU and update PID ouputs
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();

			gyro = inertialData.GetGyroscope();

			//Common power update
			commonPower = 0.8*joystickData.GetRightY();

			//roll PID update
			rollRateDifferentialPower 	= rollRatePID.getControllerOutput	(  gyro.GetX(), joystickData.GetRightX(),sdeltat);

			//pitch PID update
			pitchRateDifferentialPower 	= pitchRatePID.getControllerOutput	(  gyro.GetY(), -joystickData.GetLeftY(),sdeltat);

			//yaw PID update
			yawRateDifferentialPower 	= yawRatePID.getControllerOutput	(  gyro.GetZ(), joystickData.GetLeftX(),sdeltat);

			/*
			cout << (gyro.GetX()) << " " << (gyro.GetY()) << " " << (gyro.GetZ()) << " ";
			cout << rollRateDifferentialPower << " " << pitchRateDifferentialPower << " " << yawRateDifferentialPower << " ";
			cout << min(max(commonPower+rollRateDifferentialPower+pitchRateDifferentialPower-yawRateDifferentialPower,10.0f),100.0f) << " " //Front left
				 <<	min(max(commonPower-rollRateDifferentialPower+pitchRateDifferentialPower+yawRateDifferentialPower,10.0f),100.0f) << " " //Front right
				 <<	min(max(commonPower-rollRateDifferentialPower-pitchRateDifferentialPower-yawRateDifferentialPower,10.0f),100.0f) << " " //Back right
				 <<	min(max(commonPower+rollRateDifferentialPower-pitchRateDifferentialPower+yawRateDifferentialPower,10.0f),100.0f) << "\n";  //Back left
			*/

			//Set esc data variable and send values to MCU
			escData.Set(
					min(max(commonPower+rollRateDifferentialPower+pitchRateDifferentialPower-yawRateDifferentialPower,10.0f),100.0f), //Front left
					min(max(commonPower-rollRateDifferentialPower+pitchRateDifferentialPower+yawRateDifferentialPower,10.0f),100.0f), //Front right
					min(max(commonPower-rollRateDifferentialPower-pitchRateDifferentialPower-yawRateDifferentialPower,10.0f),100.0f), //Back right
					min(max(commonPower+rollRateDifferentialPower-pitchRateDifferentialPower+yawRateDifferentialPower,10.0f),100.0f)  //Back left
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
