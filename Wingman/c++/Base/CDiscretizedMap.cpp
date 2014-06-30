/*
 * CDiscretizedMap.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#include "CDiscretizedMap.h"

#include <iostream>

using namespace std;

CDiscretizedMap::CDiscretizedMap(int w, int h) {
	// TODO Auto-generated constructor stub
	mHeight = h;
	mWidth = w;

	mpGridLattice = new CGrid * [mWidth];
	for(int i = 0; i < mWidth; i++){
		mpGridLattice[i] = new CGrid[mHeight];
	}

}

CDiscretizedMap::~CDiscretizedMap() {
	// TODO Auto-generated destructor stub
    if(mpGridLattice)
    {
    	for(int i=0;i<mHeight;i++)
    	{
    		delete [] mpGridLattice[i];
    		mpGridLattice[i] = 0;
    	}

    	delete [] mpGridLattice;
    	mpGridLattice = 0;
    }
}

void CDiscretizedMap::init()
{
	for(int x = 0; x < mWidth; x++){
		for(int y = 0; y < mHeight; y++){
			mpGridLattice[x][y].mX = x;
			mpGridLattice[x][y].mY = y;
			mpGridLattice[x][y].mIndexId = y * mWidth + x;

		}
	}
}

void CDiscretizedMap::moveAgent(CAgent * agent, int t)
{
	int posX = 0, posY = 0;
	if(0==agent)
	{
		return;
	}

	AgentMotion motion = agent->getMotion(t);
	posX = agent->getPosX();
	posY = agent->getPosY();

	updatePosByMotion(motion, posX, posY);

	agent->setPosX(posX);
	agent->setPosY(posY);

	CGrid * pGrid = getGrid(posX, posY);

	agent->addVistedGridHist(*pGrid);

	//cout << " new pos is " << agent->getPosX() << " and " << agent->getPosY() <<endl;

}

void CDiscretizedMap::update(int t)
{
	vector<CAgent*>::iterator it;
	CGridSet grids;
	for(it=mAgents.begin();it!=mAgents.end();it++)
	{
		moveAgent(*it, t);
		grids = getGridSet((*it)->getPosX(), (*it)->getPosY(), (*it)->getObservationRange());
		//grids.print();
		(*it)->updateByObservation(grids);
	}
}

void CDiscretizedMap::run(int T)
{
	// set current position to history before moving
	CGridSet grids;
	vector<CAgent*>::iterator it;
	for(it=mAgents.begin();it!=mAgents.end();it++)
	{
		int posX = (*it)->getPosX();
		int posY = (*it)->getPosY();

		CGrid * pGrid = getGrid(posX, posY);
		(*it)->addVistedGridHist(*pGrid);

		grids = getGridSet((*it)->getPosX(), (*it)->getPosY(), (*it)->getObservationRange());
		(*it)->updateByObservation(grids);
	}

	for(int t=0;t<T;t++)
	{
		//cout << " run at time " << t << endl;
		update(t);
	}

}

bool CDiscretizedMap::setGridValue(int x, int y, double value)
{
	if(x<0 || x>=mWidth || y<0 || y>=mHeight)
	{
		return false;
	}


	mpGridLattice[x][y].mProbabilityValue = value;

	return true;
}
