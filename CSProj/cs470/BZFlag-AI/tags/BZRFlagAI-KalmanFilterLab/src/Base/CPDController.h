/*
 * CPDController.h
 *
 *  Created on: Sep 30, 2012
 *      Author: walter
 */

#ifndef CPDCONTROLLER_H_
#define CPDCONTROLLER_H_

#include "CController.h"
#include <vector>

const int STATIC_TIME_MAX = 10;

class CPDController : public CController
{
public:
	CPDController(double P, double D);
	virtual ~CPDController();

	void setCtrlStepTime(double stepTime);
	void setParamP(double P) { mParamP = P; };
	void setParamD(double D) { mParamD = D; };

	double getCtrlStepTime(void) { return mCtrlStepTime; };
	double getParamP(void) { return mParamP; };
	double getParamD(void) { return mParamD; };

	ControlParams calcControlParams(double deltaX, double deltaY, double deltaOrientation, double stepTime);

private:
	double mParamP;
	double mParamD;

	double mLastDeltaX;
	double mLastDeltaY;
	double mLastDeltaOrientation;

	double mCtrlStepTime;

	double mStaticTimeCnt;
};

#endif /* CPDCONTROLLER_H_ */
