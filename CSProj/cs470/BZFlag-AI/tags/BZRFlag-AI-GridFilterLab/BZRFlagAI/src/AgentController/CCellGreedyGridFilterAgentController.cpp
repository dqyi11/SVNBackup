/*
 * CCellGreedyGridFilterAgentController.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: walter
 */

#include "CCellGreedyGridFilterAgentController.h"
#include "../Base/CWorldController.h"

CCellGreedyGridFilterAgentController::CCellGreedyGridFilterAgentController(int num, int grid_size)
    : CCellBasedGridFilterAgentController(num, grid_size)
{
	// TODO Auto-generated constructor stub
	mpInTargetRegion = new bool(num);

	for(int i=0;i<num;i++)
	{
		mpInTargetRegion[i]=false;
	}

}

CCellGreedyGridFilterAgentController::~CCellGreedyGridFilterAgentController() {
	// TODO Auto-generated destructor stub
	if(mpInTargetRegion)
	{
		delete mpInTargetRegion;
		mpInTargetRegion = NULL;
	}
}

void CCellGreedyGridFilterAgentController::moveAgent(int index)
{
	int gridIndex = 0;
	double targetPosX = 0, targetPosY = 0;

	ControlParams ctrlParams;
	double currentX = 0, currentY = 0;
	int worldSize = GET_WORLD()->mSize;
	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	CMyAgent * agent = getAgent(index);
	if(NULL==agent)
	{
		return;
	}

	gridIndex = getNextGridIndex(index);

	//cout << "grid index " << gridIndex << endl;

	getGridCenterPos(gridIndex, targetPosX, targetPosY);


	CMyAgent * pAgent = getAgent(index);
	if(NULL==agent)
	{
		return;
	}

	currentX = agent->getPosX();
	currentY = agent->getPosY();

	agent->setTargetPosition(targetPosX, targetPosY);

	cout << " index " << index;
	cout << " tar index " << gridIndex;
	cout << " tar x " << targetPosX;
	cout << " tar y " << targetPosY;
	cout << " cur x " << currentX;
	cout << " cur y " << currentY << endl;
	getGridIndexByPos(currentX, currentY, gridIndex);

	AgentMotionStatus motionStatus = pAgent->getAgentMotionStatus();
	if(motionStatus==POS_UNCHANGE)
	{
		agent->setRandomWalkMode(true);
		// cout << "pos unchanged " << endl;
	}
	else
	{
		agent->setRandomWalkMode(false);
	}

	//agent->setRandomWalkMode(false);

	//cout << "target " << targetPosX << " " << targetPosY << endl;
	ctrlParams = pAgent->getControlParams();

	if(motionStatus==POS_UNCHANGE)
	{
		agent->setRandomWalkMode(false);
	}

	//cout << "ctrl params " << ctrlParams.angleVelocity << " " << ctrlParams.speed << endl;

	pCommunicator->angvel(index, ctrlParams.angleVelocity);
	pCommunicator->speed(index, ctrlParams.speed);

}

int CCellGreedyGridFilterAgentController::getNextGridIndex(int index)
{
	int targetGridIndex = 0;

	//index = 3;
	if(false==mpInTargetRegion[index])
	{
		switch(index)
		{
		case 0:
			getGridIndexByGridPos((int)mWidthNum/4, (int)mHeightNum/4, targetGridIndex);
			break;
		case 1:
			getGridIndexByGridPos((int)3*mWidthNum/4, (int)mHeightNum/4, targetGridIndex);
			break;
		case 2:
			getGridIndexByGridPos((int)mWidthNum/4, (int)3*mHeightNum/4, targetGridIndex);
			break;
		case 3:
			getGridIndexByGridPos((int)3*mWidthNum/4, (int)3*mHeightNum/4, targetGridIndex);
			break;
		case 4:
			getGridIndexByGridPos((int)mWidthNum/4, (int)mHeightNum/4, targetGridIndex);
			break;
		case 5:
			getGridIndexByGridPos((int)3*mWidthNum/4, (int)mHeightNum/4, targetGridIndex);
			break;
		case 6:
			getGridIndexByGridPos((int)mWidthNum/4, (int)3*mHeightNum/4, targetGridIndex);
			break;
		case 7:
			getGridIndexByGridPos((int)3*mWidthNum/4, (int)3*mHeightNum/4, targetGridIndex);
			break;

		}
	}

	return targetGridIndex;

}

