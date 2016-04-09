/*
 * FlightMode.h
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#ifndef FLIGHTMODES_H_
#define FLIGHTMODES_H_

#include <iostream>
#include "McuInterface.h"
#include "Controllers.h"

namespace std {



class FlightMode {
public:
	FlightMode();
	FlightMode(McuInterface *mcu_int);
	virtual ~FlightMode();
	void activate();
	void deactivate();
	int isActive();
	virtual int RunMode(); //Unse virtual so that callback refers to children RunMode() function
	int getId();
	McuInterface* getMcuInterface();

protected:
	void init();
	McuInterface *mcu_interface;
	InertialData inertialData;
	JoystickData joystickData;

	pthread_t thread;
	int quit;
	void** ret;

	int active;
	int id ;
};





class StandbyFlightMode: public FlightMode {
public:
	StandbyFlightMode(McuInterface *mcu_int);
	virtual ~StandbyFlightMode();
	void activate();
	int RunMode();

protected:
	void init();
	AttitudeData attitudeData;
};





class GyroCalibrationMode: public FlightMode {
public:
	GyroCalibrationMode(McuInterface *mcu_int);
	virtual ~GyroCalibrationMode();
	int RunMode();

protected:
	void init();
};





class AccelCalibrationMode: public FlightMode {
public:
	AccelCalibrationMode(McuInterface *mcu_int);
	virtual ~AccelCalibrationMode();
	int RunMode();

protected:
	void init();
};



class MagnetCalibrationMode: public FlightMode {
public:
	MagnetCalibrationMode(McuInterface *mcu_int);
	virtual ~MagnetCalibrationMode();
	int RunMode();

protected:
	void init();
};




class AcrobaticFlightMode: public FlightMode {
public:
	AcrobaticFlightMode(McuInterface *mcu_int);
	virtual ~AcrobaticFlightMode();
	int RunMode();

protected:
	void init();
};





class StabilizedFlightMode: public FlightMode {
public:
	StabilizedFlightMode(McuInterface *mcu_int);
	virtual ~StabilizedFlightMode();
	int RunMode();

protected:
	void init();
	AttitudeData attitudeData;
};



class AlexKidd2FlightMode: public FlightMode {
public:
	AlexKidd2FlightMode(McuInterface *mcu_int);
	virtual ~AlexKidd2FlightMode();
	int RunMode();

protected:
	void init();
};

class AlexKiddFlightMode: public FlightMode {
public:
	AlexKiddFlightMode(McuInterface *mcu_int);
	virtual ~AlexKiddFlightMode();
	void activate();
	int RunMode();

protected:
	void init();
	AttitudeData attitudeData;
};

} /* namespace std */

#endif /* FLIGHTMODES_H_ */
