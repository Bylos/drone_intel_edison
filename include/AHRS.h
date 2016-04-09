/*
 * ahrs.h
 * Implements orientation filters based on Madgwick's quaternion
 * algorithm
 *
 * Copyright (C) 2015  Bylos & Korky
 * Thanks to Seb Madgwick, Jim Lindblom, Kris Winer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _AHRS_H_
#define _AHRS_H_

#include <math.h>
#include "McuTypes.h"
#include "FlightData.h"


namespace std {

// Defines integration time in orientation filters
// Calls to filters should be synchronized using this value
#define AHRS_INIT_UPDATE_PERIOD	0.005f

// Defines start (max) and final (min) feedback gain for fast convergence
// High values gives fast convergence but poor stability
// Low values gives good stability but induces drifts during fast changes
#define BETA_MIN	0.02f
#define BETA_MAX	30.0f
#define ALPHA		0.5f

// Enumerates implemented orientation filters
typedef enum {
	AHRS_MADGWICK_2015,
	AHRS_MADGWICK_IMU
} ahrs_filter_t;

void ahrs_BetaUpdate(GyroscopeSensorData gyro);

/* ahrs_Madgwick2015
 * Updated implementation of Madgwick's orientation filter
 * Thanks to Jeroen van de Mortel <http://diydrones.com/profile/JeroenvandeMortel>
 */
void ahrs_Madgwick2015(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float sdeltat);

/* ahrs_MadgwickIMU
 * 6dof version of Madgwick orientation filter
 */
void ahrs_MadgwickIMU(float ax, float ay, float az, float gx, float gy, float gz, float sdeltat);

/* ahrs_Quaternion2Euler
 * Converts quaternion to Trait-Brian angles (z-y-x)
 * Yaw, Pitch and Roll values are updated internally
 */
AttitudeData ahrs_Quaternion2Euler(void);

/* ahrs_init
 * initialize internal beta and quaternion values
 */
int ahrs_init(InertialData data, ahrs_filter_t filter);

/* ahrs_orientation_update
 * update quaternion and euler angle from new interial sensors values using a chosen filter
 */
AttitudeData ahrs_orientation_update(InertialData data, ahrs_filter_t filter, float sdeltat);


} /* namespace std */

#endif /* _AHRS_H_ */
