/*
 * CMyAgent.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: walter
 */

#include <iostream>
#include "CMyAgent.h"
#include "CWorldController.h"
#include "../ControlParams.h"

CMyAgent::CMyAgent(int index, string callSign)
    : CAgent(callSign)
{
	// TODO Auto-generated constructor stub
	mIndex = index;

	mVelocityX = 0.0;
	mVelocityY = 0.0;
	mAngleVelocity = 0.0;
	mShotsAvailable = 0;
	mTimeToReload = 0;

	mpController = new CPDController(PD_PROPORTIONAL, PD_DERIVATIVE);
}

CMyAgent::~CMyAgent() {
	// TODO Auto-generated destructor stub
	if(mpController)
	{
		delete mpController;
		mpController = NULL;
	}
}

void CMyAgent::print()
{
	cout << "The my agent state is : " <<endl;
	cout << "[INDEX]" << mIndex;
	cout << "[COLOR]" << mColor;
	cout << "[CALL SIGN]" << mCallSign;
	cout << "[STATUS]" << mStatus;
	cout << "[X POS]" << mPosX;
	cout << "[Y POS]" << mPosY;
	cout << "[ORIENTATION]" << mOrientation;
	cout << "[VEL X]" << mVelocityX;
	cout << "[VEL Y" << mVelocityY;
	cout << "[VEL A]" << mAngleVelocity;
	cout << "[SHOT AVAIL]" << mShotsAvailable;
	cout << "[TIME RELOAD]" << mTimeToReload;
	cout << "[FLAG]" << mFlag << endl;
}

void CMyAgent::setCtrlTimeStepLength(double T)
{
	if(mpController)
	{
		mpController->setCtrlStepTime(T);
	}

}
void CMyAgent::setCtrlParamP(double P)
{
	if(mpController)
	{
		mpController->setParamP(P);
	}

}
void CMyAgent::setCtrlParamD(double D)
{
	if(mpController)
	{
		mpController->setParamD(D);
	}
}

double CMyAgent::getCtrlTimeStepLength(void)
{
	if(mpController)
	{
		return mpController->getCtrlStepTime();
	}

	return 0;
}

double CMyAgent::getCtrlParamP(void)
{
	if(mpController)
	{
		return mpController->getParamP();
	}

	return 0;
}

double CMyAgent::getCtrlParamD(void)
{
	if(mpController)
	{
		return mpController->getParamD();
	}

	return 0;
}

ControlParams CMyAgent::getControlParams()
{
	ControlParams params;
	params.angleVelocity = 0;
	params.speed = 0;

	//double maxSpeed = GET_WORLD()->tankspeed;
    //double maxAngVel = GET_WORLD()->tankangvel;

    /*
	if(mEnableRandomWalk)
	{
		params.angleVelocity = maxAngVel * rand()/RAND_MAX;
		params.speed = maxSpeed * rand()/RAND_MAX;

		//out << "ctrl rand " << params.angleVelocity << " " << params.speed << endl;
	}
	else
	{
	*/
		if(mpController)
		{
			double targetOrientation = atan2(mTargetY-mPosY,mTargetX-mPosX);
			//cout << "target orient " << targetOrientation;
			//cout << " orient "  << mOrientation << endl;

		    params = mpController->calcControlParams(mTargetX-mPosX, mTargetY-mPosY, targetOrientation-mOrientation, CTRL_STEP_TIME);

		    // cout << "ctrl non-rand " << params.angleVelocity << " " << params.speed << endl;
		}
	//}

	return params;
}
