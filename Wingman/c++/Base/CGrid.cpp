/*
 * CGrid.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#include "CGrid.h"
#include <iostream>

using namespace std;

CGrid::CGrid() {
	// TODO Auto-generated constructor stub
	mX = 0;
	mY = 0;

	mIndexId = 0;
	mProbabilityValue = 0;

	mType = EMPTY;

}

CGrid::~CGrid() {
	// TODO Auto-generated destructor stub
}

CGridSet::CGridSet()
{
	mSet.clear();

}

CGridSet::~CGridSet()
{
    mSet.clear();
}

bool CGridSet::hasGrid(CGrid * pGrid)
{
	list<CGrid*>::iterator it;
	if(pGrid)
	{
		for(it=mSet.begin();it!=mSet.end();it++)
		{
			if(*it==pGrid)
			{
				return true;
			}
		}
	}

	return false;
}

void CGridSet::addGrid(CGrid * pGrid)
{
	if(pGrid==0)
	{
		return;
	}

	//cout << " adding " << pGrid->mX << " and " << pGrid->mY << endl;

	if(false==hasGrid(pGrid))
	{
		mSet.push_back(pGrid);
	}
	else
	{
		// cout << " existed " << endl;
 	}
}

void CGridSet::print()
{
	list<CGrid*>::iterator it;
	cout << " printing the grids in the grid set "  << endl;
	for(it=mSet.begin();it!=mSet.end();it++)
	{
		cout << " grid " << (*it)->mX << " and " << (*it)->mY << endl;
	}

	cout << endl;

}

void CGridSet::append(CGridSet otherSet)
{
	list<CGrid*>::iterator it;
	for(it=otherSet.mSet.begin();it!=otherSet.mSet.end();it++)
	{
		addGrid(*it);
	}
}

void CGridSet::removeGrid(CGrid * pGrid)
{
	list<CGrid*>::iterator it;
	if(pGrid)
	{
		for(it=mSet.begin();it!=mSet.end();it++)
		{
			if(*it==pGrid)
			{
				it = mSet.erase(it);
			}
		}
	}
}
