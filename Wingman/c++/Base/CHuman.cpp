/*
 * CHuman.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#include "CHuman.h"
#include <iostream>
using namespace std;

CHuman::CHuman() {
	// TODO Auto-generated constructor stub
	mAgentType = HUMAN;

	mObservationLikelihood = 0.4;

	mWingmanRadius = 1;
}

CHuman::~CHuman() {
	// TODO Auto-generated destructor stub
}

void CHuman::updateByObservation(CGridSet grids)
{
	if(grids.mSet.size()==0)
	{
		return;
	}

	//cout << " updateByObservation " << endl;
	//grids.print();

	list<CGrid*>::iterator it;
	for(it=grids.mSet.begin();it!=grids.mSet.end();it++)
	{
		CGrid * pGrid = (*it);
		if(pGrid->mX == mPosX
				&& pGrid->mY==mPosY)
		{
			//cout << " same position " <<endl;
			pGrid->mProbabilityValue = 0;
		}
		else
		{
			pGrid->mProbabilityValue =
					(1 - mObservationLikelihood)
					* (pGrid)->mProbabilityValue;
		}
	}


}
