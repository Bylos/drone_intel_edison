/*
 * controller.h
 *
 *  Created on: 23 août 2015
 *      Author: Adrien
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

namespace std {

class Controller {
public:
	Controller();
	virtual ~Controller();

	float getControllerOutput();
};


class PIDController: public Controller {
public:
	PIDController(float Kp, float Ki, float Kd, float maxI);
	PIDController(float Kp, float Ki, float Kd, float maxI, float maxThComp);
	virtual ~PIDController();

	float getKp();
	void setKp(float Kp);
	float getKi();
	void setKi(float Ki);
	float getKd();
	void setKd(float Kd);
	float getMaxI();
	void setMaxI(float maxI);
	float getIntegral();
	void setIntegral(float integral);
	float getLastErr();
	void setLastErr(float lastErr);
	float getDerivative();
	void setDerivative(float derivative);

	float getControllerOutput(float currentValue, float targetValue, float deltat);

private:
	float Kp_, Ki_, Kd_, maxI_;
	float integral_, lastErr_, derivative_;

};

} /* namespace std */

#endif /* CONTROLLER_H_ */
