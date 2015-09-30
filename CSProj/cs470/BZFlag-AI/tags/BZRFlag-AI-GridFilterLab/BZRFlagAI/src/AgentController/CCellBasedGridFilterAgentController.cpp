/*
 * CCellBasedGridFilterAgentController.cpp
 *
 *  Created on: Nov 10, 2012
 *      Author: walter
 */

#include <string.h>
#include "CCellBasedGridFilterAgentController.h"
#include "../Base/CWorldController.h"

CCellBasedGridFilterAgentController::CCellBasedGridFilterAgentController(int num, int grid_size)
    : CGridFilterAgentController(num)
{
	// TODO Auto-generated constructor stub
	mGridSize = grid_size;
	mWidthNum = GET_WORLD()->mSize/mGridSize;
    mHeightNum = GET_WORLD()->mSize/mGridSize;
    mGridNum = mWidthNum * mHeightNum;

	mpGridStatusMap = new int[mGridNum];

	memset(mpGridStatusMap,0,mGridNum);

	vector<CMyAgent*>::iterator it;
	for(it=agents.begin();it!=agents.end();it++)
	{
		int index = (*it)->getIndex();
		CCellSequence * pSeq = new CCellSequence(index);
		mCellSequences.push_back(pSeq);
	}
}

CCellBasedGridFilterAgentController::~CCellBasedGridFilterAgentController()
{
	// TODO Auto-generated destructor stub
	vector<CCellSequence*>::iterator it;
	for(it=mCellSequences.begin();it!=mCellSequences.end();it++)
	{
		CCellSequence * pSeq = (*it);
		delete pSeq;
		pSeq=NULL;
	}
	mCellSequences.clear();

	if(mpGridStatusMap)
	{
		delete [] mpGridStatusMap;
		mpGridStatusMap = NULL;
	}
}

bool CCellBasedGridFilterAgentController::getGridCenterPos(int gridIndex, double & posX, double & posY)
{
	int gridX = 0, gridY = 0;
	double dimSize = GET_WORLD()->mSize;

	if(!convertGridIndexToGridPos(gridIndex, gridX, gridY))
	{
		return false;
	}

	// calc the position by grid X and grid Y
    posX = mGridSize * gridX  + mGridSize/2 - dimSize/2;
    posY = mGridSize * gridY  + mGridSize/2 - dimSize/2;

    //cout << " so the gridX is " << gridX << " gridY ";
    //cout << gridY << " posX " << posX << " posY " << posY << endl;
	return true;
}

bool CCellBasedGridFilterAgentController::getGridIndexByPos(double posX, double posY, int & gridIndex)
{
	int gridX = 0, gridY = 0;
	int dimSize = GET_WORLD()->mSize;

	gridX = (int)floor((posX + dimSize/2)/mGridSize);
	gridY = (int)floor((posY + dimSize/2)/mGridSize);

	//cout << "CALC " << " GX " << gridX << " GY " << gridY;
	//cout << " X " << posX << " Y " << posY << endl;

	return getGridIndexByGridPos(gridX, gridY, gridIndex);
}


bool CCellBasedGridFilterAgentController::getGridIndexByGridPos(int gridX, int gridY, int & gridIndex)
{
	if((gridX<0)||(gridX>=mWidthNum)||(gridY<0)||(gridY>=mHeightNum))
	{
		return false;
	}

	//cout << "conv gridx " << gridX << " gridy " << gridY << endl;

	gridIndex = gridX + gridY * mWidthNum;

	return true;
}

bool CCellBasedGridFilterAgentController::convertGridIndexToGridPos(int gridIndex, int & gridX, int & gridY)
{
	if((gridIndex<0)||(gridIndex>mGridNum))
	{
		return false;
	}

	//cout << "convert " << "gridIndex " << gridIndex;
	//cout << " mWidth " << mWidthNum << endl;
	gridY = (int)floor(gridIndex/mWidthNum);
	gridX = gridIndex - gridY * mWidthNum;

	return true;
}

bool CCellBasedGridFilterAgentController::hasGridBeenVisited(int index)
{
	if((index < 0)||(index > mGridNum))
	{
		return true;
	}

	if(mpGridStatusMap[index]>0)
	{
		return true;
	}

	return false;
}

void CCellBasedGridFilterAgentController::markGridVisited(int index)
{
	mpGridStatusMap[index]++;
}

bool CCellBasedGridFilterAgentController::insideTargetGrid(double targetX, double targetY, double currentX, double currentY)
{
	double dist = sqrt(pow(targetX-currentX,2)+pow(targetY-currentY,2));

	if(dist<=mGridSize)
	{
		return true;
	}

	return false;
}

bool CCellBasedGridFilterAgentController::hasGridBeenVisited(int gridX, int gridY)
{
	int gridIndex = 0;
	if(getGridIndexByGridPos(gridX, gridY, gridIndex))
	{
		return hasGridBeenVisited(gridIndex);
	}

	return true;
}

int CCellBasedGridFilterAgentController::getNearestUnvisited(int index)
{
	int currentIndex = -1;
	int currentGridX = 0, currentGridY = 0;
	int findIndex = -1;
	CMyAgent * agent = getAgent(index);
	if(NULL==agent)
	{
		return -1;
	}

	getGridIndexByPos(agent->getPosX(),agent->getPosY(),currentIndex);
	convertGridIndexToGridPos(currentIndex, currentGridX, currentGridY);

	int max_depth = 6;
	for(int depth=1;depth<=max_depth;depth++)
	{
		for(int i=-depth;i<=depth;i++)
		{
			cout << "find nearest unvisited , trying depth " << i << endl;
			if(!hasGridBeenVisited(currentGridX+i,currentGridY+depth))
			{
				getGridIndexByGridPos(currentGridX+i,currentGridY+depth,findIndex);
				return findIndex;
			}

			if(!hasGridBeenVisited(currentGridX+i,currentGridY-depth))
			{
				getGridIndexByGridPos(currentGridX+i,currentGridY-depth,findIndex);
				return findIndex;
			}
		}

		for(int j=-depth;j<=depth;j++)
		{
			if(!hasGridBeenVisited(currentGridX+depth,currentGridY+j))
			{
				getGridIndexByGridPos(currentGridX+depth,currentGridY+j,findIndex);
				return findIndex;
			}

			if(!hasGridBeenVisited(currentGridX-depth,currentGridY+j))
			{
				getGridIndexByGridPos(currentGridX+j,currentGridY-depth,findIndex);
				return findIndex;
			}
		}
	}

	cout << " did not find nearest unvisited " << endl;
	return -1;
}
