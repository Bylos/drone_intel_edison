/*
 * McuTypes.h
 *
 *  Created on: 1 janv. 2016
 *      Author: Bylos
 */

#ifndef MCUTYPES_H_
#define MCUTYPES_H_


/* Convenient structure types for inertial data, size is 18 bytes */
typedef struct {
	short x;
	short y;
	short z;
} vector_t;

typedef struct {
	float x;
	float y;
	float z;
} fvector_t;

typedef struct {
	vector_t gyro;
	vector_t accel;
	vector_t magn;
} inertial_t;

/* Convenient structure types for esc pwm data, size is 8 bytes */
typedef struct {
	unsigned short front_left;
	unsigned short front_right;
	unsigned short rear_right;
	unsigned short rear_left;
} esc_pwm_t;

/* Convenient structure types for joystick data, size is 8 bytes */
typedef struct {
	signed short x;
	signed short y;
} axe_t;

typedef struct {
	axe_t left;
	axe_t right;
} joystick_t;


#endif /* MCUTYPES_H_ */
