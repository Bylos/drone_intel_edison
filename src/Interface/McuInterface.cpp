/*
 * McuInterface.cpp
 *
 *  Created on: 24 déc. 2015
 *      Author: Bylos
 */

#include "McuInterface.h"

namespace std {

void crc_init(void);
unsigned char crc_fast(unsigned char const message[], int nBytes);

void* McuInterfaceCallback(void* obj)
{
	static_cast<McuInterface*>(obj)->Main();
	return(0);
} // callback

McuInterface::McuInterface() {
	/* thread variables initialization */
	thread = 0;
	ret = 0;
	quit = 0;
	mode = 0;
	start_time = 0;
	/* interface variables initialization */
	joystick_data_flag = 0;
	inertial_data_flag = 0;
	command_flag = 0;
	command = 0;
	event_flag = 0;
	event = 0;
	/* crc table initialization */
	crc_init();
	/* mcu serial port initialization */
	//mcu_port = -1;
	/*mcu_port = open("/dev/ttymcu0", O_RDWR | O_NOCTTY);
	if (mcu_port == -1) {
		cout << "MCU Interface : Unable to open interface" << endl;
	} else {
		cout << "MCU Interface : Interface successfully opened" << endl;
	}*/
	mcu2cpu_port = -1;
	char const * mcu2cpu_file = "/tmp/mcu2cpu";
	cpu2mcu_port = -1;
	char const * cpu2mcu_file = "/tmp/cpu2mcu";
	/* create the mcu to cpu FIFO (named pipe) */
	while(mcu2cpu_port==-1){
		nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
		mcu2cpu_port = open(mcu2cpu_file, O_RDONLY|O_NONBLOCK);
	}
	cout << "MCU 2 CPU Interface : Interface successfully opened" << endl;

	/* create the cpu to mcu FIFO (named pipe) */
	while(cpu2mcu_port==-1){
		nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
		cpu2mcu_port = open(cpu2mcu_file, O_WRONLY);
	}
	cout << "CPU 2 MCU Interface : Interface successfully opened" << endl;

	/* Load calibration values for accelerometer */
	std::ifstream accelCalibFile;
	accelCalibFile.open("/etc/drone/accelerometerCalibration.txt");
	if (!accelCalibFile.is_open()) {
	    std::cout << "Failed to open accelerometer calibration file\n";
	    accelCalibb.x = 0.0f ; accelCalibb.y = 0.0f ; accelCalibb.z = 0.0f ;
	    accelCalibs.x = 1.0f ; accelCalibs.y = 1.0f ; accelCalibs.z = 1.0f ;
	    std::cout << accelCalibb.x << " " << accelCalibb.y << " " << accelCalibb.z << endl;
	    std::cout << accelCalibs.x << " " << accelCalibs.y << " " << accelCalibs.z << endl;
	} else {
		accelCalibFile >> accelCalibb.x >> accelCalibb.y >> accelCalibb.z;
		accelCalibFile >> accelCalibs.x >> accelCalibs.y >> accelCalibs.z;
	}

	/* Load calibration values for gyroscope */
	std::ifstream gyroCalibFile;
	gyroCalibFile.open("/etc/drone/gyroscopeCalibration.txt");
	if (!gyroCalibFile.is_open()) {
	    std::cout << "Failed to open gyroscope calibration file\n";
	    gyroCalib.x = 0.0f ; gyroCalib.y = 0.0f ; gyroCalib.z = 0.0f ;
	} else {
		gyroCalibFile >> gyroCalib.x >> gyroCalib.y >> gyroCalib.z;
	}

	/* Load calibration values for magnetometer */
	std::ifstream magnCalibFile;
	magnCalibFile.open("/etc/drone/magnetometerCalibration.txt");
	if (!magnCalibFile.is_open()) {
	    std::cout << "Failed to open magnetometer calibration file\n";
	    magnetoCalibb.x = 0.0f ; magnetoCalibb.y = 0.0f ; magnetoCalibb.z = 0.0f ;
	    magnetoCalibX.x = 0.0f ; magnetoCalibX.y = 0.0f ; magnetoCalibX.z = 0.0f ;
	    magnetoCalibY.x = 0.0f ; magnetoCalibY.y = 0.0f ; magnetoCalibY.z = 0.0f ;
	    magnetoCalibZ.x = 0.0f ; magnetoCalibZ.y = 0.0f ; magnetoCalibZ.z = 0.0f ;
	} else {
		magnCalibFile >> magnetoCalibb.x >> magnetoCalibb.y >> magnetoCalibb.z;
		magnCalibFile >> magnetoCalibX.x >> magnetoCalibX.y >> magnetoCalibX.z;
		magnCalibFile >> magnetoCalibY.x >> magnetoCalibY.y >> magnetoCalibY.z;
		magnCalibFile >> magnetoCalibZ.x >> magnetoCalibZ.y >> magnetoCalibZ.z;
	}

	/* Load position of sensors relative to gravity center */
	std::ifstream sensorsToGravityCenterFile;
	sensorsToGravityCenterFile.open("/etc/drone/sensorsPosition.txt");
	if (!sensorsToGravityCenterFile.is_open()) {
	    std::cout << "Failed to open sensors position file\n";
	    sensorsPosition.x = 0.0f ; sensorsPosition.y = 0.0f ; sensorsPosition.z = 0.0f ;
	} else {
		sensorsToGravityCenterFile >> sensorsPosition.x >> sensorsPosition.y >> sensorsPosition.z;
	}


}

int McuInterface::Run() {
	//if (mcu_port == -1) {
	if ((cpu2mcu_port == -1)|(mcu2cpu_port == -1)) {
		cout << "Interface is not opened" << endl;
		return -1;
	} else {
		pthread_create(&thread, 0, &McuInterfaceCallback, this);
		return 0;
	}
}

int McuInterface::Join() {
	return pthread_join(thread, ret);
}

void McuInterface::Stop() {
	quit = 1;
}

int McuInterface::BytesAvailable() {
	int bytes = 0;
	ioctl(mcu2cpu_port, FIONREAD, &bytes);
	return bytes;
}
void McuInterface::StartTime() {
	struct timespec time;
	clock_gettime(CLOCK_MONOTONIC_RAW, &time);
	start_time = time.tv_sec*1000 + time.tv_nsec*0.000001;
}

unsigned long McuInterface::TimeElapsed() {
	struct timespec time;
	clock_gettime(CLOCK_MONOTONIC_RAW, &time);
	return time.tv_sec*1000 + time.tv_nsec*0.000001 - start_time;
}

int McuInterface::Main() {
	int i, n;
	unsigned char buffer[8192];
	unsigned long time, cpu_advert_timer = 0;

	cout << "MCU Interface : Communication is started" << endl;
	//tcflush(mcu_port, TCIOFLUSH);
	tcflush(mcu2cpu_port, TCIOFLUSH);
	n = BytesAvailable();
	if (n > 0) {
		read(mcu2cpu_port, buffer, n);
	}
	StartTime();

	while(!quit) {
		/* wait for 1ms to release thread from system */
		nanosleep((const struct timespec[]){{0, 100000L}}, NULL);
		time = TimeElapsed();
		/* get data from input buffer */
		n = BytesAvailable();
		if (n > 0) {
			//read(mcu_port, buffer, n);
			read(mcu2cpu_port, buffer, n);
			/* look for a preamble */
			i = 0;
			while (i < n-3) {
				if (buffer[i] == 0xFF && buffer[i+1] == 0xFF) {	// we have a preamble
					int length = buffer[i+2];
					if (i+length+3 < n) {	// we may have a complete frame
						if (buffer[i+length+3] == crc_fast(buffer+i+3, length)) { // we have a crc
							/* check frame type */
							switch (buffer[i+3]) {
							case MCU_FRAME_JOYSTICK:
								joystick = *(joystick_t*)(buffer+i+4);
								joystick_data_flag = 1;
								break;
							case MCU_FRAME_INERTIAL:
								inertial = *(inertial_t*)(buffer+i+4);
								inertial_data_flag = 1;
								break;
							case MCU_FRAME_COMMAND:
								command = buffer[i+4];
								command_flag = 1;
								break;
							case MCU_FRAME_EVENT:
								event = buffer[i+4];
								event_flag = 1;
								break;
							default: // Should never be there anyhow
								break;
							}
							i += 3+length; // Set cursor to last byte of frame
						} else {
							/* There is a crc error */
							cout << "MCU Interface : CRC Error" << endl;

							for(int j=i;j<i+length+2;j++) std::cout << +buffer[j] <<" ";
							std::cout << endl ;

						}
					} else {
						/* There is an uncomplete frame */
						cout << "MCU Interface : Incomplete Frame" << endl;
					}
				}
				i++;
			}
		}
		if (time - cpu_advert_timer > CPU_ADVERT_PERIOD) {
			cpu_advert_timer = time;
			SendMode();
		}
	}
	//close(mcu_port);
	close(mcu2cpu_port);
	close(cpu2mcu_port);

	cout << "MCU Interface : Time of thread: " << TimeElapsed() << endl;
	cout << "MCU Interface : Communication is stopped" << endl;
	return(0);
}

void McuInterface::SetMode(int Mode, int Send) {
	mode = Mode;
	if (Send) {
		SendMode();
	}
}

void McuInterface::SendMode(void) {
	unsigned char m = static_cast<unsigned char>(mode);
	unsigned char buffer[6] = { 0xFF, 0xFF, 0x02, CPU_FRAME_MODE,  m, 0x00 };
	buffer[5] = crc_fast(buffer+3, 2);
	write(cpu2mcu_port, buffer, 6);
}

#define MCU_ESC_SCALE	864.0
#define CPU_ESC_SCALE	100.0

void McuInterface::SetEsc(EscData &Esc, int Send) {
	esc_pwm.front_left = (unsigned short)(Esc.GetFrontLeft()/CPU_ESC_SCALE*MCU_ESC_SCALE);
	esc_pwm.front_right = (unsigned short)(Esc.GetFrontRight()/CPU_ESC_SCALE*MCU_ESC_SCALE);
	esc_pwm.rear_right = (unsigned short)(Esc.GetRearRight()/CPU_ESC_SCALE*MCU_ESC_SCALE);
	esc_pwm.rear_left = (unsigned short)(Esc.GetRearLeft()/CPU_ESC_SCALE*MCU_ESC_SCALE);
	if (Send) {
		SendEsc();
	}
}

void McuInterface::SendEsc(void) {
	unsigned char buffer[5+sizeof(esc_pwm_t)];
	buffer[0] = buffer[1] = 0xFF;
	buffer[2] = 1+sizeof(esc_pwm_t);
	buffer[3] = CPU_FRAME_ESC;
	esc_pwm_t *esc_pwm_ptr = (esc_pwm_t*)&buffer[4];
	*esc_pwm_ptr = esc_pwm;
	buffer[4+sizeof(esc_pwm_t)] = crc_fast(buffer+3, 1+sizeof(esc_pwm_t));
	//write(mcu_port, buffer, 5+sizeof(esc_pwm_t));
	write(cpu2mcu_port, buffer, 5+sizeof(esc_pwm_t));
}

int McuInterface::GetJoystickDataFlag() {
	if (joystick_data_flag) {
		joystick_data_flag = 0;
		return 1;
	}
	return 0;
}

#define MCU_JOYSTICK_SCALE 2048.0
#define CPU_JOYSTICK_SCALE 100.0

JoystickData McuInterface::GetJoystickData() {
	JoystickData Joystick;
	Joystick.Set(
			(float)joystick.left.x/MCU_JOYSTICK_SCALE*CPU_JOYSTICK_SCALE,
			(float)joystick.left.y/MCU_JOYSTICK_SCALE*CPU_JOYSTICK_SCALE,
			(float)joystick.right.x/MCU_JOYSTICK_SCALE*CPU_JOYSTICK_SCALE,
			(float)joystick.right.y/MCU_JOYSTICK_SCALE*CPU_JOYSTICK_SCALE);
	return Joystick;
}

int McuInterface::GetInertialDataFlag() {
	if (inertial_data_flag) {
		inertial_data_flag = 0;
		return 1;
	}
	return 0;
}

#define MCU_A_SCALE		32768.0
#define MCU_G_SCALE		32768.0
#define MCU_M_SCALE		32768.0
#define CPU_A_SCALE		4.0
#define CPU_G_SCALE		500.0
#define CPU_M_SCALE		2.0

InertialData McuInterface::GetInertialData() {
	InertialData Inertial;
	AccelerometerSensorData Accelerometer;
	GyroscopeSensorData Gyroscope;
	MagnetometerSensorData Magnetometer;
	Accelerometer.Set(
			( (float)inertial.accel.x/MCU_A_SCALE*CPU_A_SCALE-accelCalibb.x)/accelCalibs.x,
			( (float)inertial.accel.y/MCU_A_SCALE*CPU_A_SCALE-accelCalibb.y)/accelCalibs.y,
			(-(float)inertial.accel.z/MCU_A_SCALE*CPU_A_SCALE-accelCalibb.z)/accelCalibs.z);
	Gyroscope.Set(
			(-(float)inertial.gyro.x/MCU_G_SCALE*CPU_G_SCALE-gyroCalib.x),
			(-(float)inertial.gyro.y/MCU_G_SCALE*CPU_G_SCALE-gyroCalib.y),
			( (float)inertial.gyro.z/MCU_G_SCALE*CPU_G_SCALE-gyroCalib.z));

	float magnTempx = -(float)inertial.magn.x/MCU_M_SCALE*CPU_M_SCALE - magnetoCalibb.x;
	float magnTempy = -(float)inertial.magn.y/MCU_M_SCALE*CPU_M_SCALE - magnetoCalibb.y;
	float magnTempz = -(float)inertial.magn.z/MCU_M_SCALE*CPU_M_SCALE - magnetoCalibb.z;
	Magnetometer.Set(
			magnTempx * magnetoCalibX.x + magnTempy * magnetoCalibX.y + magnTempz * magnetoCalibX.z,
			magnTempx * magnetoCalibY.x + magnTempy * magnetoCalibY.y + magnTempz * magnetoCalibY.z,
			magnTempx * magnetoCalibZ.x + magnTempy * magnetoCalibZ.y + magnTempz * magnetoCalibZ.z);

	/* exponential smoothing */
	static AccelerometerSensorData newAcc;
	newAcc.Set(
			0.4*Accelerometer.GetX()+0.6*newAcc.GetX(),
			0.4*Accelerometer.GetY()+0.6*newAcc.GetY(),
			0.4*Accelerometer.GetZ()+0.6*newAcc.GetZ());
	static GyroscopeSensorData newGyr;
	newGyr.Set(
			0.4*Gyroscope.GetX()+0.6*newGyr.GetX(),
			0.4*Gyroscope.GetY()+0.6*newGyr.GetY(),
			0.4*Gyroscope.GetZ()+0.6*newGyr.GetZ());
	static MagnetometerSensorData newMag;
	newMag.Set(
			0.4*Magnetometer.GetX()+0.6*newMag.GetX(),
			0.4*Magnetometer.GetY()+0.6*newMag.GetY(),
			0.4*Magnetometer.GetZ()+0.6*newMag.GetZ());

	Inertial.Set(newAcc, newGyr, newMag);

	return Inertial;
}

int McuInterface::GetCommandFlag() {
	if (command_flag) {
		command_flag = 0;
		return 1;
	}
	return 0;
}

int McuInterface::GetCommand() {
	return command;
}

int McuInterface::GetEventFlag() {
	if (event_flag) {
		event_flag = 0;
		return 1;
	}
	return 0;
}

int McuInterface::GetEvent() {
	return event;
}
void McuInterface::SetAccelCalib(fvector_t aCalibb, fvector_t aCalibs){

	//Write the new calibration values in a file
	using std::fstream;
	ofstream accelCalibFile;
	accelCalibFile.open("/etc/drone/accelerometerCalibration.txt", std::ofstream::out | std::ofstream::trunc);
	accelCalibFile << aCalibb.x << " " << aCalibb.y << " " << aCalibb.z << endl;
	accelCalibFile << aCalibs.x << " " << aCalibs.y << " " << aCalibs.z << endl;
	accelCalibFile.close();

	//update the  calibration variables
	accelCalibb = aCalibb ;
	accelCalibs = aCalibs ;
}
void McuInterface::SetGyroCalib(fvector_t gCalib){

	//Write the new calibration values in a file
	using std::fstream;
	ofstream gyroCalibFile;
	gyroCalibFile.open("/etc/drone/gyroscopeCalibration.txt", std::ofstream::out | std::ofstream::trunc);
	gyroCalibFile << gCalib.x << " " << gCalib.y << " " << gCalib.z << endl;
	gyroCalibFile.close();

	//update the  calibration variables
	gyroCalib = gCalib ;
}
void McuInterface::SetMagnetoCalib(fvector_t mCalibb,fvector_t mCalibx,fvector_t mCaliby,fvector_t mCalibz){

	//Write the new calibration values in a file
	using std::fstream;
	ofstream magCalibFile;
	magCalibFile.open("/etc/drone/magnetometerCalibration.txt", std::ofstream::out | std::ofstream::trunc);
	magCalibFile << mCalibb.x << " " << mCalibb.y << " " << mCalibb.z << endl;
	magCalibFile << mCalibx.x << " " << mCalibx.y << " " << mCalibx.z << endl;
	magCalibFile << mCaliby.x << " " << mCaliby.y << " " << mCaliby.z << endl;
	magCalibFile << mCalibz.x << " " << mCalibz.y << " " << mCalibz.z << endl;
	magCalibFile.close();

	//update the  calibration variables
	magnetoCalibb = mCalibb ;
	magnetoCalibX = mCalibx ;
	magnetoCalibY = mCaliby ;
	magnetoCalibZ = mCalibz ;
}

/*****************************************************************************
 * Local 8-bit CRC fast calculation                                          *
 *****************************************************************************/

#define POLYNOMIAL 0xD8
#define TOPBIT (1 << 7)
unsigned char crcTable[256];

void crc_init(void) {
	unsigned char remainder;
	int dividend;
	for (dividend = 0; dividend < 256; ++dividend) {
		remainder = dividend;
		unsigned char bit;
		for (bit = 8; bit > 0; --bit) {
			if (remainder & TOPBIT) {
				remainder = (remainder << 1) ^ POLYNOMIAL;
			} else {
				remainder = (remainder << 1);
			}
		}
		crcTable[dividend] = remainder;
	}
}   /* crcInit() */

unsigned char crc_fast(unsigned char const message[], int nBytes) {
	unsigned char data;
	unsigned char remainder = 0;
	int byte;
	for (byte = 0; byte < nBytes; ++byte) {
		data = message[byte] ^ remainder;
		remainder = crcTable[data] ^ (remainder << 8);
	}
	return (remainder);
}   /* crcFast() */

} /* namespace std */
