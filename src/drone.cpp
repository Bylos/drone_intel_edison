#include "drone.h"

using namespace std;

FlightMode * changeFlightMode(int newModeId, FlightMode *currentFlightMode) {

	FlightMode* newFlightMode;
	McuInterface* mcuInterface = currentFlightMode->getMcuInterface();
	currentFlightMode->deactivate();
	try {
		switch (newModeId) {
		case MCU_CMD_MODE_STANDBY: 			newFlightMode = new StandbyFlightMode(mcuInterface); 	break;
		case MCU_CMD_MODE_ALEXKIDD: 		newFlightMode = new AlexKiddFlightMode(mcuInterface); 	break;
		case MCU_CMD_MODE_GYROCALIBRATION: 	newFlightMode = new GyroCalibrationMode(mcuInterface); 	break;
		case MCU_CMD_MODE_ACCELCALIBRATION: newFlightMode = new AccelCalibrationMode(mcuInterface);	break;
		case MCU_CMD_MODE_MAGNCALIBRATION: 	newFlightMode = new MagnetCalibrationMode(mcuInterface);break;
		case MCU_CMD_MODE_STABILIZED: 		newFlightMode = new StabilizedFlightMode(mcuInterface); break;
		case MCU_CMD_MODE_ACROBATIC: 		newFlightMode = new AcrobaticFlightMode(mcuInterface); 	break;
		case MCU_CMD_MODE_ALEXKIDD2:		newFlightMode = new AlexKidd2FlightMode(mcuInterface);	break;
		}
	}
	catch (const std::exception & e) {
		currentFlightMode->activate();
		return currentFlightMode;
	}
	delete currentFlightMode;
	currentFlightMode = newFlightMode;
	currentFlightMode->activate();

	mcuInterface->SetMode(newModeId, 1);
	return currentFlightMode;
}

int main() {
	cout << "*** Drone Application Starts ***\n" ;

	//Initialisation of MCU interface
	McuInterface *mcu_interface = new McuInterface();
	mcu_interface->Run();
	nanosleep((const struct timespec[]){{0, 100000000L}}, NULL); // Wait for communication to be started

	// Instantiation of Standby mode at start
	FlightMode *flightMode = new StandbyFlightMode(mcu_interface);
	flightMode->activate();
	mcu_interface->SetMode(MCU_CMD_MODE_STANDBY, 1);
	nanosleep((const struct timespec[]){{2, 000000000L}}, NULL);


	FILE* modelog = fopen("/etc/drone/log_mode.txt", "a");
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main Loop
	for(;;) {
		nanosleep((const struct timespec[]){{0, 1000000L}}, NULL);

		// Gestion des changements de modes
		if (mcu_interface->GetCommandFlag()) {
			int command = mcu_interface->GetCommand() ;
			fprintf(modelog, "%8u command : %u\n", mcu_interface->TimeElapsed(), command);
			// cout << "Drone : MCU Command " << command << endl;
			switch (command) {

			// Switch mode commands
			case MCU_CMD_MODE_STANDBY: case MCU_CMD_MODE_ALEXKIDD: case MCU_CMD_MODE_GYROCALIBRATION: case MCU_CMD_MODE_ACCELCALIBRATION:
			case MCU_CMD_MODE_MAGNCALIBRATION: case MCU_CMD_MODE_STABILIZED: case MCU_CMD_MODE_ACROBATIC:
			case MCU_CMD_MODE_ALEXKIDD2:
				if (flightMode->getId() == command) break;
				flightMode = changeFlightMode(command,flightMode);
				break;

			}

		}

	}


	mcu_interface->Stop();
	mcu_interface->Join();
	return 0;
	cout << "*** Drone Application Stops ***\n" ;
}


