/*
 * CAgent.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#include "CAgent.h"

CAgent::CAgent() {
	// TODO Auto-generated constructor stub

	mObservationRange = 1;

	mGridsVisited.clear();
	mMotionSequence.clear();

}

CAgent::~CAgent() {
	// TODO Auto-generated destructor stub
	mGridsVisited.clear();
	mMotionSequence.clear();
}

AgentMotion CAgent::getMotion(int t)
{
	if(t<0 || t>getMotionSequenceSize()-1)
	{
		return STAY;
	}

	return mMotionSequence[t];
}

void CAgent::loadMotion(AgentMotion * pMotionSeq, int num)
{
	mMotionSequence.clear();

	for(int i=0;i<num;i++)
	{
		addMotionToSequence(pMotionSeq[i]);
	}
}

void CAgent::addVistedGridHist(CGrid grid)
{
	mGridsVisited.push_back(grid);

}
