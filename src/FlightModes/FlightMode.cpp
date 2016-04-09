/*
 * FlightMode.cpp
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#include "FlightModes.h"

namespace std {

void* FlightModeCallback(void* obj)
{
	static_cast<FlightMode*>(obj)->RunMode();
	static_cast<FlightMode*>(obj)->deactivate();
	return(0);
} // callback

FlightMode::FlightMode() {

	//cout << "Entering FlightMode constructor (without arguments)\n" ;
	mcu_interface = NULL;
	thread = 0;
	quit = 1;
	active = 0;
	ret = NULL;
	id = 0;
	JoystickData joystickData;
	//cout << "Leaving FlightMode constructor (without arguments)\n" ;
}

FlightMode::FlightMode(McuInterface *mcu_int) {

	//cout << "Entering FlightMode constructor (with arguments)\n" ;
	//Initialization
	init();
	mcu_interface = mcu_int;
	active = 0;
	id = 0;
	//cout << "Leaving FlightMode constructor (with arguments)\n" ;


}

FlightMode::~FlightMode() {
	// Deinit functions
	quit = 1;
	if (active) pthread_join(thread,ret);
}

void FlightMode::activate() {
	//thread creation
	quit = 0;
	pthread_create(&thread, 0, &FlightModeCallback, this);
	active = 1 ;
	switch (id) {
	case MCU_CMD_MODE_STANDBY: 			cout << "Standby flight mode activated\n" ; 			break;
	case MCU_CMD_MODE_ALEXKIDD: 		cout << "AlexKidd flight mode activated\n" ; 			break;
	case MCU_CMD_MODE_GYROCALIBRATION: 	cout << "Gyro calibration flight mode activated\n" ; 	break;
	case MCU_CMD_MODE_ACCELCALIBRATION: cout << "Accelero calibration flight mode activated\n" ;break;
	case MCU_CMD_MODE_MAGNCALIBRATION: 	cout << "Magneto calibration flight mode activated\n" ;	break;
	case MCU_CMD_MODE_STABILIZED: 		cout << "Stabilized flight mode activated\n" ; 			break;
	case MCU_CMD_MODE_ACROBATIC: 		cout << "Acrobatic flight mode activated\n" ; 			break;
	case MCU_CMD_MODE_ALEXKIDD2:		cout << "AlexKidd 2 flight mode activated\n" ;			break;
	}
}

void FlightMode::deactivate(){
	quit = 1;
	if (active) {
		pthread_join(thread,NULL);
	}
	active = 0 ;
	/*switch (id) {
	case MCU_CMD_MODE_STANDBY: 			cout << "Standby flight mode deactivated\n" ; 				break;
	case MCU_CMD_MODE_ALEXKIDD: 		cout << "AlexKidd flight mode deactivated\n" ; 				break;
	case MCU_CMD_MODE_GYROCALIBRATION: 	cout << "Gyro calibration flight mode deactivated\n" ; 		break;
	case MCU_CMD_MODE_ACCELCALIBRATION: cout << "Accelero calibration flight mode deactivated\n" ;	break;
	case MCU_CMD_MODE_MAGNCALIBRATION: 	cout << "Magneto calibration flight mode deactivated\n" ;	break;
	case MCU_CMD_MODE_STABILIZED: 		cout << "Stabilized flight mode deactivated\n" ; 			break;
	case MCU_CMD_MODE_ACROBATIC: 		cout << "Acrobatic flight mode deactivated\n" ; 			break;
	case MCU_CMD_MODE_ALEXKIDD2:		cout << "AlexKidd 2 flight mode deactivated\n" ;			break;
	}*/
}
int FlightMode::isActive(){
	return active ;
}

int FlightMode::getId(){
	return id ;
}

McuInterface* FlightMode::getMcuInterface() {
	return mcu_interface;
}

void FlightMode::init() {
	// TODO common init for all modes


}

int FlightMode::RunMode() {

	cout << "FlightMode running\n" ;

	while(!quit) {

	}

	return 0;
}

} /* namespace std */


