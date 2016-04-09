/*
 * McuInterface.h
 *
 *  Created on: 24 déc. 2015
 *      Author: Bylos
 */

// INFO: Link with rt library for proper time functions

#ifndef MCUINTERFACE_H_
#define MCUINTERFACE_H_

#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "McuTypes.h"
#include "FlightData.h"

namespace std {

#define MCU_FRAME_JOYSTICK	0x00
#define MCU_FRAME_INERTIAL	0x01
#define MCU_FRAME_COMMAND	0x02
#define MCU_FRAME_EVENT		0x03

#define CPU_FRAME_MODE	1
#define CPU_FRAME_ESC	2

#define CPU_ADVERT_PERIOD	500	// ms

enum {
	MCU_CMD_MODE_STANDBY			= 0,
	MCU_CMD_MODE_ALEXKIDD			= 1,
	MCU_CMD_MODE_ACCELCALIBRATION	= 2,
	MCU_CMD_MODE_GYROCALIBRATION	= 3,
	MCU_CMD_MODE_MAGNCALIBRATION	= 4,
	MCU_CMD_MODE_ALEXKIDD2			= 5,
	MCU_CMD_MODE_ACROBATIC			= 6,
	MCU_CMD_MODE_STABILIZED			= 7,
};
class McuInterface {
public:
	/* constructor */
	McuInterface();
	/* start interface thread */
	int Run();
	/* stop interface thread */
	void Stop();
	/* set internal cpu mode value and possibly send it if send is non-zero */
	void SetMode(int Mode, int Send);
	/* set internal esc values and possibly send it if send is non-zero */
	void SetEsc(EscData &Esc, int Send);

	/*functions for setting the calibration values for sensors*/
	void SetAccelCalib(fvector_t aCalibb, fvector_t aCalibs);
	void SetGyroCalib(fvector_t gCalib);
	void SetMagnetoCalib(fvector_t mCalibb,fvector_t mCalibx,fvector_t mCaliby,fvector_t mCalibz);

	/* return non-zero if new joystick data were received */
	int GetJoystickDataFlag();
	JoystickData GetJoystickData();
	/* return non-zero if new inertial data were received */
	int GetInertialDataFlag();
	InertialData GetInertialData();
	/* return non-zero if new command was received */
	int GetCommandFlag();
	int GetCommand();
	/* return non-zero if new event was received */
	int GetEventFlag();
	int GetEvent();

	/* return time elapsed in millisecond since thread started */
	unsigned long TimeElapsed();

	/* called automatically when thread starts DO NOT CALL MANUALLY */
	int Main();
	/* can be used to wait for end of thread */
	int Join();


private:
	/* return number of bytes in put buffer */
	int BytesAvailable();
	/* set start time in milliseconds */
	void StartTime();
	/* send frames to mcu */
	void SendMode(void);
	void SendEsc(void);
	/* thread control variables */
	pthread_t thread;
	void** ret;
	int quit;
	unsigned long start_time;
	/* mcu interface file descriptor */
	//int mcu_port;
	int cpu2mcu_port;
	int mcu2cpu_port;
	/* input data flags and variables */
	int joystick_data_flag;
	joystick_t joystick;
	int inertial_data_flag;
	inertial_t inertial;
	int command_flag;
	int command;
	int event_flag;
	int event;
	/* output data variables */
	int mode;
	esc_pwm_t esc_pwm;

	/*Vectors for sensors calibration*/
	fvector_t accelCalibb; fvector_t accelCalibs;
	fvector_t gyroCalib;
	fvector_t magnetoCalibb; fvector_t magnetoCalibX; fvector_t magnetoCalibY; fvector_t magnetoCalibZ;
};

} /* namespace std */

#endif /* MCUINTERFACE_H_ */
