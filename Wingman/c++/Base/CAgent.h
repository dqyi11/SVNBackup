/*
 * CAgent.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CAGENT_H_
#define CAGENT_H_

#include <vector>
#include "CGrid.h"

enum AgentType
{
	HUMAN = 0,
	ROBOT = 1,
};

enum AgentMotion
{
	STAY = 0,
	EAST,
	SOUTH,
	WEST,
	NORTH,
	SOUTHEAST,
	SOUTHWEST,
	NORTHEAST,
	NORTHWEST,
};

using namespace std;

class CAgent {
public:
	CAgent();
	virtual ~CAgent();

	AgentType getAgentType() { return mAgentType; };

	void setPosX(int x) { mPosX = x; };
	void setPosY(int y) { mPosY = y; };
	int getPosX() { return mPosX; };
	int getPosY() { return mPosY; };

	void setObservationRange(int range) { mObservationRange = range; };
	int getObservationRange() { return mObservationRange; };

	double getObservationLikelihood() { return mObservationLikelihood; };

	void loadMotion(AgentMotion * pMotionSeq, int num);


	void addMotionToSequence(AgentMotion motion) { mMotionSequence.push_back(motion); };
	int getMotionSequenceSize() { return mMotionSequence.size(); };
	AgentMotion getMotion(int t);

	virtual void updateByObservation(CGridSet grids) = 0;

	void addVistedGridHist(CGrid grid);

	vector<CGrid> mGridsVisited;

protected:
	int mPosX;
	int mPosY;
	AgentType mAgentType;
	vector<AgentMotion> mMotionSequence;

	double mObservationLikelihood;

	int mObservationRange;
};

#endif /* CAGENT_H_ */
