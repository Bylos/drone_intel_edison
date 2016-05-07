/*
 * AlexKidd2FlightMode.cpp
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#include "FlightModes.h"
#include "McuInterface.h"

namespace std {

AlexKidd2FlightMode::AlexKidd2FlightMode(McuInterface *mcu_int): FlightMode(mcu_int) {

	init() ;

	id = MCU_CMD_MODE_ALEXKIDD2;

	cout << "AlexKidd 2 flight mode object created\n" ;
}

AlexKidd2FlightMode::~AlexKidd2FlightMode() {
}

void AlexKidd2FlightMode::init() {

	cout << "AlexKidd 2 flight mode initialization\n" ;

}

int AlexKidd2FlightMode::RunMode() {

	cout << "AlexKidd 2 flight mode running\n" ;

	//ESC data
	EscData escData;

	//Main loop of the mode
	while(!quit) {
		nanosleep((const struct timespec[]){{0, 000100000L}}, NULL); // 1us seems already enough

		//Handle events sent by MCU
		if (mcu_interface->GetEventFlag()) {
			mcu_interface->GetEvent(); //No event management so far
		}

		//Handle joystick/RC data sent by MCU
		if (mcu_interface->GetJoystickDataFlag()) {
			joystickData = mcu_interface->GetJoystickData();
			float front_diff  = joystickData.GetLeftY()/8.0f;
			float sides_diff  = joystickData.GetLeftX()/8.0f;
			escData.Set(
					min(max(front_diff - sides_diff, 0.0f), 25.0f),
					min(max(front_diff + sides_diff, 0.0f), 25.0f),
					min(max(- front_diff + sides_diff, 0.0f), 25.0f),
					min(max(- front_diff - sides_diff, 0.0f), 25.0f)
					);
			mcu_interface->SetEsc(escData,1);
		}

		//IMU data sent by MCU
		if (mcu_interface->GetInertialDataFlag()) {
			mcu_interface->GetInertialData(); // Nothing to do with IMU data
		}
	}
	return 0;
}

} /* namespace std */
