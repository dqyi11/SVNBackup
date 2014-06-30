/*
 * CHexagonDiscretizedMap.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#include "CHexagonDiscretizedMap.h"
#include <iostream>

using namespace std;

AgentMotion motions[7] = {STAY, EAST, WEST, NORTHEAST, NORTHWEST,
		                  SOUTHEAST, SOUTHWEST,
};

CHexagonDiscretizedMap::CHexagonDiscretizedMap()
    : CDiscretizedMap(0,0)
{

}

CHexagonDiscretizedMap::CHexagonDiscretizedMap(int w, int h, int hexSize)
    : CDiscretizedMap(2*w, h)
{
	// TODO Auto-generated constructor stub
	mHexagonSize = hexSize;
}

CHexagonDiscretizedMap::~CHexagonDiscretizedMap()
{
	// TODO Auto-generated destructor stub
}

void CHexagonDiscretizedMap::init()
{
	CDiscretizedMap::init();

	for(int x = 0; x < mWidth; x++){
		for(int y = 0; y < mHeight; y++){
			mpGridLattice[x][y].mProbabilityValue = 0.5;
		}
	}

}

void CHexagonDiscretizedMap::updatePosByMotion(AgentMotion motion, int & x, int & y)
{
	//cout << "motion is " << motion << endl;
	switch(motion)
	{
	case EAST:
		x += 2;
		break;
	case WEST:
		x -= 2;
		break;
	case NORTHEAST:
		x += 1;
		y += 1;
		break;
	case NORTHWEST:
		x -= 1;
		y += 1;
		break;
	case SOUTHEAST:
		x += 1;
		y -= 1;
		break;
	case SOUTHWEST:
		x -= 1;
		y -= 1;
		break;
	case STAY:
	default:
		break;
	}

	if(x<0) x = 0;
	if(x>mWidth-1) x= mWidth - 1;
	if(y<0) y = 0;
	if(y>mHeight-1) y = mHeight - 1;

}

CGridSet CHexagonDiscretizedMap::getGridSet(int posX, int posY, int radius, bool includeCenter)
{
	CGridSet set;
	CGridSet nextSet;
	CGridSet finalSet;

	set.clear();
	nextSet.clear();
	finalSet.clear();

	// add itself
	set.addGrid(getGrid(posX, posY));

	list<CGrid*>::iterator it;
	int cellX = 0, cellY = 0;

	if(radius <= 0)
	{
		finalSet.append(set);
		return finalSet;
	}

	finalSet.append(set);

	for(int i=0;i<radius;i++)
	{

		nextSet.clear();

		for(it=set.mSet.begin();it!=set.mSet.end();it++)
		{
			CGrid * pGrid = (*it);
			if(0==pGrid)
			{
				cout << 'error ' << endl;
			}

    		if(pGrid)
			{
				for(int j=0;j<7;j++)
				{
					cellX = pGrid->mX;
					cellY = pGrid->mY;
					updatePosByMotion(motions[j], cellX, cellY);
					nextSet.addGrid(getGrid(cellX, cellY));

				 }

			}
		}

		set.clear();
		set.append(nextSet);
		finalSet.append(nextSet);

	}

	if(false==includeCenter)
	{
		//remove self
		set.removeGrid(getGrid(posX, posY));
	}

	return finalSet;
}

CGrid * CHexagonDiscretizedMap::getGrid(int x, int y)
{
	if(x<0 || x>mWidth-1 || y<0 || y>mHeight-1)
	{
		return 0;
	}

	//cout << " get Grid " << x << " and " << y << endl;

	return (CGrid *)&(mpGridLattice[x][y]);
}
