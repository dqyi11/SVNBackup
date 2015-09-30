/*
 * CRandomTargetGridFilterAgentController.cpp
 *
 *  Created on: Nov 9, 2012
 *      Author: walter
 */

#include "CRandomTargetGridFilterAgentController.h"
#include "../Base/CWorldController.h"

CRandomTargetGridFilterAgentController::CRandomTargetGridFilterAgentController(int num)
    : CGridFilterAgentController(num)
{
	// TODO Auto-generated constructor stub
	mNearTargetThreshold = 40;
	mpRegenerateTarget = new bool[num];
	for(int i=0;i<num;i++)
	{
		mpRegenerateTarget[i]=true;
	}
}

CRandomTargetGridFilterAgentController::~CRandomTargetGridFilterAgentController() {
	// TODO Auto-generated destructor stub
	if(mpRegenerateTarget)
	{
		delete [] mpRegenerateTarget;
		mpRegenerateTarget = NULL;
	}
}

void CRandomTargetGridFilterAgentController::moveAgent(int index)
{
	ControlParams ctrlParams;
	double targetX = 0, targetY = 0;
	double currentX = 0, currentY = 0;
	int worldSize = GET_WORLD()->mSize;
	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	CMyAgent * pAgent = getAgent(index);
	if(NULL==pAgent)
	{
		return;
	}

	currentX = pAgent->getPosX();
	currentY = pAgent->getPosY();

	if(mpRegenerateTarget[index])
	{
		targetX = worldSize * rand()/RAND_MAX - worldSize/2;
		targetY = worldSize * rand()/RAND_MAX - worldSize/2;
		pAgent->setTargetPosition(targetX, targetY);
		mpRegenerateTarget[index] = false;
	}


	if(pow(currentX-targetX,2)+pow(currentY-targetY,2)>mNearTargetThreshold)
	{
		mpRegenerateTarget[index] = true;
	}

	AgentMotionStatus motionStatus = pAgent->getAgentMotionStatus();
	if(motionStatus==POS_UNCHANGE)
	{
		pAgent->setRandomWalkMode(true);
	}

	ctrlParams = pAgent->getControlParams();

	if(motionStatus==POS_UNCHANGE)
	{
		pAgent->setRandomWalkMode(false);
	}


	pCommunicator->angvel(index, ctrlParams.angleVelocity);
	pCommunicator->speed(index, ctrlParams.speed);

}

