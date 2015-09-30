/*
 * CMyAgent.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: walter
 */

#include "CMyAgent.h"
#include <iostream>

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
}

CMyAgent::~CMyAgent() {
	// TODO Auto-generated destructor stub
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
