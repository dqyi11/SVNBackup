/*
 * CGreedySearchPathPlanner.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#include "CGreedySearchPathPlanner.h"

#include <iostream>

using namespace std;

CGreedySearchPathPlanner::CGreedySearchPathPlanner(CPathPlanningGraph * graph, CAgent * agent)
    : CRobotPathPlanner(graph, agent)
{
	// TODO Auto-generated constructor stub
	mGeneratedPath.clear();
}

CGreedySearchPathPlanner::~CGreedySearchPathPlanner() {
	// TODO Auto-generated destructor stub
	mGeneratedPath.clear();
}

bool CGreedySearchPathPlanner::doGreedySearch(CLevelVertex * startVertex)
{
	cout << "doGreedySearch " << startVertex->mId << endl;
	CLevelVertex * pCurrentVertex = startVertex;
	mGeneratedPath.clear();

	for(int i=1;i<=mPlanStep;i++)
	{
		if(NULL==pCurrentVertex)
		{
			return false;
		}

		mGeneratedPath.addVertex(*pCurrentVertex);

		pCurrentVertex = findNextMaxStep(pCurrentVertex);

	}

	cout << "GREEDY : " << endl;
	mGeneratedPath.printPath();

	return true;
}

bool CGreedySearchPathPlanner::doGreedySearch()
{
	CLevelVertexSet levelSet = mpGraph->mpLevelSets[0];

	CLevelVertex * startVertex = NULL;

	startVertex = levelSet.mSet.front();

	doGreedySearch(startVertex);
}

double CGreedySearchPathPlanner::scoreVertex(CLevelVertex * vertex)
{
	CPath prevPath = mGeneratedPath;
	CPath newPath = mGeneratedPath;

	newPath.addVertex(*vertex);

	initObservedCells(prevPath);
	initObservedCells(newPath);

	double prevScore = scorePath(prevPath);
	double newScore = scorePath(newPath);
	return newScore-prevScore;
}

CLevelVertex * CGreedySearchPathPlanner::findNextMaxStep(CLevelVertex * vertex)
{
	CLevelVertex * pNextVertex = NULL;
	int currentLevel = -1;

	if(NULL==vertex || mpGraph==NULL)
	{
		return NULL;
	}

	currentLevel = vertex->mLevel+1;

	if(currentLevel>=mpGraph->getPlanningLength())
	{
		return NULL;
	}

	CLevelVertexSet levelSet = mpGraph->mpLevelSets[currentLevel];

	double maxReward = -1;
	CLevelVertex * pMaxRewardVertex = NULL;

	list<CLevelVertex*>::iterator it;
	for(it=levelSet.mSet.begin();it!=levelSet.mSet.end();it++)
	{
		CLevelVertex * pVertex = (*it);

		//cout << "At step " << currentLevel << " , " << (*it)->mId << endl;

		if(mpGraph->isConnected((CVertex*)vertex,(CVertex*)pVertex))
		{
			double reward = scoreVertex(pVertex);
			if(reward > maxReward)
			{
				pMaxRewardVertex = pVertex;
				maxReward = reward;
			}
		}
	}

	return pMaxRewardVertex;
}
