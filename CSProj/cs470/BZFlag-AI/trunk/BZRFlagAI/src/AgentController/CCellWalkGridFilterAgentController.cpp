/*
 * GridWalkGridFilterAgentController.cpp
 *
 *  Created on: Nov 7, 2012
 *      Author: walter
 */

#include "CCellWalkGridFilterAgentController.h"
#include "../Base/CWorldController.h"


CCellWalkGridFilterAgentController::CCellWalkGridFilterAgentController(int num, int grid_size)
    : CCellBasedGridFilterAgentController(num, grid_size)
{
	// TODO Auto-generated constructor stub

}

CCellWalkGridFilterAgentController::~CCellWalkGridFilterAgentController() {
	// TODO Auto-generated destructor stub
}

void CCellWalkGridFilterAgentController::moveAgent(int index)
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

	if(index>6)
	{
		// for index > 6
		// only random walk
		agent->setRandomWalkMode(true);
		ctrlParams = agent->getControlParams();
		pCommunicator->angvel(index, ctrlParams.angleVelocity);
		pCommunicator->speed(index, ctrlParams.speed);
		return;
	}


	currentX = agent->getPosX();
	currentY = agent->getPosY();
    agent->getTargetPosition(targetPosX, targetPosY);


	if(insideTargetGrid(targetPosX, targetPosY, currentX, currentY))
	{
		getGridIndexByPos(targetPosX, targetPosY, gridIndex);
		cout << " Index " << index << " get into target " << endl;
        getGridIndexByPos(currentX, currentY, gridIndex);
        markGridVisited(gridIndex);
		gridIndex = getNextGridIndex(index);
		mAgentInnerCounter[index]=0;

		//cout << "grid index " << gridIndex << endl;
		if(gridIndex < 0)
		{
			agent->setRandomWalkMode(true);
			ctrlParams = agent->getControlParams();
			pCommunicator->angvel(index, ctrlParams.angleVelocity);
			pCommunicator->speed(index, ctrlParams.speed);
			agent->setRandomWalkMode(false);
			return;
		}

		getGridCenterPos(gridIndex, targetPosX, targetPosY);
		agent->setTargetPosition(targetPosX, targetPosY);
	}


    // when counter > 100, it will also give up the current target and go to next one
    if(mAgentInnerCounter[index]>30)
    {
    	cout << " index " << index << " over try, get a new one " << endl;
    	gridIndex = getNextGridIndex(index);
		mAgentInnerCounter[index]=0;

		getGridCenterPos(gridIndex, targetPosX, targetPosY);
		agent->setTargetPosition(targetPosX, targetPosY);
    }

    /*
	cout << " index " << index;
	cout << " tar index " << gridIndex;
	cout << " tar x " << targetPosX;
	cout << " tar y " << targetPosY;
	cout << " cur x " << currentX;
	cout << " cur y " << currentY << endl;
	*/
	getGridIndexByPos(currentX, currentY, gridIndex);


	AgentMotionStatus motionStatus = agent->getAgentMotionStatus();
	if(motionStatus==POS_UNCHANGE)
	{
		agent->setRandomWalkMode(true);
		//cout << "pos unchanged " << endl;
	}
	else
	{
		agent->setRandomWalkMode(false);
	}

	//agent->setRandomWalkMode(false);

	//cout << "target " << targetPosX << " " << targetPosY << endl;
	ctrlParams = agent->getControlParams();

	mAgentInnerCounter[index]++;

	if(motionStatus==POS_UNCHANGE)
	{
		agent->setRandomWalkMode(false);
	}

	//cout << "ctrl params " << ctrlParams.angleVelocity << " " << ctrlParams.speed << endl;

	pCommunicator->angvel(index, ctrlParams.angleVelocity);
	pCommunicator->speed(index, ctrlParams.speed);
}

int CCellWalkGridFilterAgentController::getNextGridIndex(int index)
{
	// TASK Allocation by index
	// 0,1,2,3 will go as preassigned path
	// 4,5,6 will explore nearest unvisited node
	// 7,8,9 will go randomly
	int gridIndex = -1;

	if(index>6)
	{
		return gridIndex;
	}

	if(index>=4&&index<=6)
	{
		return getNearestUnvisited(index);
	}

	if(mCellSequences[index]->mSequence.size()>0)
	{
		CPathCell * cell = mCellSequences[index]->mSequence.front();

		if(cell)
		{
			cout << "get index " << index << " x " << cell->mCellX << " y " << cell->mCellY << endl;

			getGridIndexByGridPos(cell->mCellX, cell->mCellY, gridIndex);

			mCellSequences[index]->mSequence.pop_front();
		}

	}
	else
	{
		// TODO : find unvisited cell
		cout << " find unvisited cell " << endl;
		int YBegin = 0, YEnd = 0, XBegin = 0, XEnd = 0;
		if(index==0)
		{
			YBegin = 0;
			YEnd = mHeightNum/2 - 1;
			XBegin = 0;
			XEnd = mWidthNum/2 - 1;

		}else if(index==1)
		{
			YBegin = 0;
			YEnd = mHeightNum/2 - 1;
			XBegin = mWidthNum/2;
			XEnd = mWidthNum - 1;
		}else if(index==2)
		{
			YBegin = mHeightNum/2;
			YEnd = mHeightNum - 1;
			XBegin = 0;
			XEnd = mWidthNum/2 - 1;
		}else if(index==3)
		{
			YBegin = mHeightNum/2;
			YEnd = mHeightNum - 1;
			XBegin = mWidthNum/2;
			XEnd = mWidthNum - 1;
		}

		gridIndex = findUnvisitedCellInRegion(XBegin, YBegin, XEnd-XBegin, YEnd-YBegin);

		cout << " find " << gridIndex << endl;

	}

	return gridIndex;
}

void CCellWalkGridFilterAgentController::init()
{
	CGridFilterAgentController::init();

	int agentNum = agents.size();
	int YBegin = 0, YEnd = 0;
	int XBegin = 0, XEnd = 0;
	bool Xorder = false, Yorder = false;

	for(int i=0;i<4;i++)
	{
		if(i==0)
		{
			YBegin = 0;
			YEnd = mHeightNum/2 - 1;
			XBegin = 0;
			XEnd = mWidthNum/2 - 1;
			Yorder = true;

		}else if(i==1)
		{
			YBegin = 0;
			YEnd = mHeightNum/2 - 1;
			XBegin = mWidthNum/2;
			XEnd = mWidthNum - 1;
			Yorder = true;
		}else if(i==2)
		{
			YBegin = mHeightNum/2;
			YEnd = mHeightNum - 1;
			XBegin = 0;
			XEnd = mWidthNum/2 - 1;
			Yorder = false;
		}else if(i==3)
		{
			YBegin = mHeightNum/2;
			YEnd = mHeightNum - 1;
			XBegin = mWidthNum/2;
			XEnd = mWidthNum - 1;
			Yorder = false;
		}

		if(Yorder)
		{

			Xorder = true;

			for(int y=YBegin; y<=YEnd; y++)
			{

				if(Xorder)
				{
					for(int x=XBegin;x<=XEnd;x++)
					{
						CPathCell * pCell = new CPathCell(x,y);
						mCellSequences[i]->mSequence.push_back(pCell);
					}
					Xorder = false;
				}
				else
				{
					for(int x=XEnd;x>=XBegin;x--)
					{
						CPathCell * pCell = new CPathCell(x,y);
						mCellSequences[i]->mSequence.push_back(pCell);
					}
					Xorder = true;
				}

			}

		}
		else
		{
			Xorder = true;

			for(int y=YEnd; y>=YBegin; y--)
			{

				if(Xorder)
				{
					for(int x=XBegin;x<=XEnd;x++)
					{
						CPathCell * pCell = new CPathCell(x,y);
						mCellSequences[i]->mSequence.push_back(pCell);
					}
					Xorder = false;
				}
				else
				{
					for(int x=XEnd;x>=XBegin;x--)
					{
						CPathCell * pCell = new CPathCell(x,y);
						mCellSequences[i]->mSequence.push_back(pCell);
					}
					Xorder = true;
				}

			}
		}

		// set the first target
		CPathCell * cell = mCellSequences[i]->mSequence.front();
		int targetIndex = 0;
		double targetX = 0, targetY = 0;
		getGridIndexByGridPos(cell->mCellX, cell->mCellY, targetIndex);
		getGridCenterPos(targetIndex, targetX, targetY);
		agents[i]->setTargetPosition(targetX, targetY);

		if(cell)
		{
			delete cell;
			cell = NULL;
		}
		mCellSequences[i]->mSequence.pop_front();

	}

	for(int ii=4;ii<7;ii++)
	{
		int targetIndex = 0;
		double targetX = 0, targetY = 0;
		targetIndex = getNearestUnvisited(ii);
		getGridCenterPos(targetIndex, targetX, targetY);
		agents[ii]->setTargetPosition(targetX, targetY);
	}

}

int CCellWalkGridFilterAgentController::findUnvisitedCellInRegion(int x, int y, int w, int h)
{
	int index = 0;
	for(int i=x;i<x+w;i++)
	{
		for(int j=y;j<y+h;j++)
		{
			getGridIndexByGridPos(i,j,index);
			if(mpGridStatusMap[index]==0)
			{
				return index;
			}
		}
	}

	return -1;

}
