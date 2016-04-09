/*
 * StandbyFlightMode.cpp
 *
 *  Created on: 10 janv. 2016
 *      Author: Adrien
 */

#include "FlightModes.h"

namespace std {

StandbyFlightMode::StandbyFlightMode(McuInterface* mcu_int): FlightMode(mcu_int) {

	init() ;

	id = MCU_CMD_MODE_STANDBY;

}

StandbyFlightMode::~StandbyFlightMode() {
	// TODO Auto-generated destructor stub
}

void StandbyFlightMode::init() {


}

int StandbyFlightMode::RunMode() {

	cout << "Standby FlightMode running\n" ;

	//ESC data
	EscData escData;

	while(!quit) {
		nanosleep((const struct timespec[]){{0, 1000000L}}, NULL);

		//Handle joystick/RC data sent by MCU
		if (mcu_interface->GetJoystickDataFlag()) {
			joystickData = mcu_interface->GetJoystickData();
		}

		//Handle IMU data sent by MCU
		if (mcu_interface->GetInertialDataFlag()) {
			inertialData = mcu_interface->GetInertialData();
		}


	}

	return 0;
}

} /* namespace std */





