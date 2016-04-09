/*
 * PIDController.cpp
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#include "Controllers.h"
#include <algorithm>
#include <stdio.h>

namespace std {

/* Constructor for standard PID, with integral limitation */
PIDController::PIDController(float Kp, float Ki, float Kd, float maxI) {
	Kp_   = Kp ;
	Ki_   = Ki ;
	Kd_   = Kd ;
	maxI_ = maxI ;

	lastErr_    = 0.0f ;
	derivative_ = 0.0f ;
	integral_   = 0.0f ;

}

PIDController::~PIDController() {

}

float PIDController::getKp() {
	return Kp_;
}
void PIDController::setKp(float Kp) {
	Kp_ = Kp ;
}
float PIDController::getKi() {
	return Ki_;
}
void PIDController::setKi(float Ki) {
	Ki_=Ki;
}
float PIDController::getKd() {
	return Kd_;
}
void PIDController::setKd(float Kd) {
	Kd_=Kd;
}
float PIDController::getMaxI() {
	return maxI_;
}
void PIDController::setMaxI(float maxI) {
	maxI_=maxI;
}
float PIDController::getIntegral() {
	return integral_;
}
void PIDController::setIntegral(float integral) {
	integral_=integral;
}
float PIDController::getLastErr() {
	return lastErr_;
}
void PIDController::setLastErr(float lastErr) {
	lastErr_=lastErr;
}
float PIDController::getDerivative() {
	return derivative_;
}
void PIDController::setDerivative(float derivative) {
	derivative_=derivative;
}


float PIDController::getControllerOutput(float currentValue, float targetValue, float deltat) {
	float outputValue = 0.0f ;
	float err = targetValue -currentValue ;

	integral_ = integral_ + err*deltat ;

	if(integral_<=0) integral_=max(integral_,-maxI_) ;
	else integral_ = min(integral_,maxI_) ;

	derivative_ = (err - lastErr_)/deltat ;

	outputValue = Kp_*err+Ki_*integral_+Kd_*derivative_ ;

	lastErr_ = err ;

	return outputValue ;
}


} /* namespace std */
