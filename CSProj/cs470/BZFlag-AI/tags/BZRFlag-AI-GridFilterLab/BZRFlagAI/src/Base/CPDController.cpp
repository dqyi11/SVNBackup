/*
 * CPDController.cpp
 *
 *  Created on: Sep 30, 2012
 *      Author: walter
 */

#include <math.h>
#include <stdlib.h>
#include "CPDController.h"
#include "../ControlParams.h"
#include <iostream>

using namespace std;

const double THRESHOLD = 2;
const double ERROR_THRESHOLD = 30;

CPDController::CPDController(double P, double D)
{
	// TODO Auto-generated constructor stub
	mParamP = P;
	mParamD = D;
	// mCtrlStepTime = stepTime;
	mStaticTimeCnt = 0;

}

CPDController::~CPDController()
{
	// TODO Auto-generated destructor stub

}

void CPDController::setCtrlStepTime(double stepTime)
{
	mCtrlStepTime = stepTime;
}

ControlParams CPDController::calcControlParams(double deltaX, double deltaY, double deltaOrientation, double stepTime)
{
	ControlParams params;
	params.angleVelocity = 0.0;
	params.speed = 0.0;

	double deltaLength = sqrt(pow(deltaX, 2)+ pow(deltaY, 2));
	double lastDeltaLength = sqrt(pow(mLastDeltaX,2)+pow(mLastDeltaY,2));

	cout << " calc ctrl: delta orientation " << deltaOrientation << " " ;
	cout << deltaOrientation-mLastDeltaOrientation;
	cout << " delta length " << deltaLength << endl;
	params.angleVelocity = mParamP * deltaOrientation;
			              // + mParamD * (deltaOrientation - mLastDeltaOrientation) / stepTime;
	params.speed = // CONSTANT_LINEAR_SPEED; // * exp(-params.angleVelocity * 10); //mParamP * deltaLength
			ANGLE_SPEED_VELOCITY_RATIO * (mParamP * deltaLength +
			        mParamD * (deltaLength - lastDeltaLength) / stepTime);

	if(params.speed < 0)
	{
		params.speed = -params.speed;
	}

	if(params.speed > CONSTANT_LINEAR_SPEED)
	{
		params.speed = CONSTANT_LINEAR_SPEED;
	}

	mLastDeltaX = deltaX;
	mLastDeltaY = deltaY;
	mLastDeltaOrientation = deltaOrientation;

	if((mLastDeltaX < THRESHOLD)
		&& (mLastDeltaY < THRESHOLD)
		&& (mLastDeltaOrientation < THRESHOLD))
	{
		mStaticTimeCnt++;
	}

	if (mStaticTimeCnt > STATIC_TIME_MAX)
	{
		params.speed = 5;// * rand()/RAND_MAX;
		params.angleVelocity = 5;
		mStaticTimeCnt = 0;
	}

	cout << "CTRL OUTPUT speed " << params.speed;
	cout << " angleVel " << params.angleVelocity << endl;

	return params;
}
