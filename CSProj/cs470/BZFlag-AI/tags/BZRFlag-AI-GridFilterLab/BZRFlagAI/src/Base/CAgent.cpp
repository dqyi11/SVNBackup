/*
 * CAgent.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#include "CAgent.h"
#include "CWorldController.h"

#include <iostream>

CAgent::CAgent(string callSign)
{
	// TODO Auto-generated constructor stub
	mCallSign = callSign;

	mStatus = "";
	mPosX = 0;
	mPosY = 0;
	mOrientation = 0;
	mColor = "";
	mFlag = "";

}

CAgent::~CAgent()
{
	// TODO Auto-generated destructor stub
}

void CAgent::print()
{
	cout << "The agent state is : " <<endl;
	cout << "[COLOR]" << mColor;
	cout << "[CALL SIGN]" << mCallSign;
	cout << "[STATUS]" << mStatus;
	cout << "[X POS]" << mPosX;
	cout << "[Y POS]" << mPosY;
	cout << "[ORIENTATION]" << mOrientation;
	cout << "[FLAG]" << mFlag << endl;
}

AgentMotionStatus CAgent::getAgentMotionStatus(void)
{
	double threshold = GET_WORLD()->tankwidth;

	if(mPosX<(-(GET_WORLD()->mSize)/2+threshold))
	{
		return ON_THE_LEFTMOST;
	}

	if(mPosX>((GET_WORLD()->mSize)/2-threshold))
	{
		return ON_THE_RIGHTMOST;
	}

	if(mPosY<(-(GET_WORLD()->mSize)/2+threshold))
	{
		return ON_THE_BOTTOM;
	}

	if(mPosY>((GET_WORLD()->mSize)/2-threshold))
	{
		return ON_THE_UPPER;
	}

	if(pow(mPosY-mLastPosY,2)<4 && pow(mPosX-mLastPosX,2)<4)
	{
		return POS_UNCHANGE;
	}

	return POS_CHANGE;
}
