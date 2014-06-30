/*
 * CRecedingHorizonPathPlanner.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#include "CRecedingHorizonPathPlanner.h"

#include <iostream>

using namespace std;

CRecedingHorizonPathPlanner::CRecedingHorizonPathPlanner(CPathPlanningGraph * graph, CAgent * agent)
    : CRobotPathPlanner(graph, agent)
{
	// TODO Auto-generated constructor stub
	mGeneratedPath.clear();
	mRecedingHorizonLength = 1;

}

CRecedingHorizonPathPlanner::~CRecedingHorizonPathPlanner() {
	// TODO Auto-generated destructor stub
	mGeneratedPath.clear();
}

bool CRecedingHorizonPathPlanner::doRecedingHorizonSearch()
{
	mBenchmark.setStartTime();

	CLevelVertexSet levelSet = mpGraph->mpLevelSets[0];

	CLevelVertex * startVertex = NULL;

	startVertex = levelSet.mSet.front();

	doRecedingHorizonSearch(startVertex);

	mBenchmark.setEndTime();

	cout << " RECEDING " << mPlanStep << " Time spent " << mBenchmark.getTimeElasped() << endl;

}

bool CRecedingHorizonPathPlanner::doRecedingHorizonSearch(CLevelVertex * startVertex)
{
	CLevelVertex * pCurrentVertex = startVertex;

	if(NULL==startVertex)
	{
		return false;
	}

	mGeneratedPath.clear();
	mGeneratedPath.addVertex(*pCurrentVertex);


	for(int i=1;i<mPlanStep;i++)
	{
		CStepBatch stepBatch = findNextMaxStepBatch(pCurrentVertex);

		/*
		cout << " recursion @ " << i << " : ";
		stepBatch.print();
		cout << endl;
		*/

		if(stepBatch.size()==0)
		{
			return false;
		}

		pCurrentVertex = stepBatch.getFirstStep();
		mGeneratedPath.addVertex(*pCurrentVertex);

	}

	cout << "RECEDING: " << endl;
	mGeneratedPath.printPath();
}

CStepBatch CRecedingHorizonPathPlanner::findNextMaxStepBatch(CLevelVertex * vertex)
{
	mAllPossibleBatches.clear();
	// using DFL to find all possible batch
	CStepBatch batch;
	int currentLevel = vertex->mLevel;
	depthFirstLimitedSearch(vertex, batch, 1);

	// score all the batch to find the max one
	scoreAllPossibleBatches();

	return getMaxStepBatch();
}

void CRecedingHorizonPathPlanner::depthFirstLimitedSearch(CLevelVertex * start, CStepBatch batch, int level)
{
	if(NULL==start)
	{
		return;
	}

	if(level>1)
	{
		batch.addVertex(start);
	}

	int currentLevel = start->mLevel+1;

	if(level>mRecedingHorizonLength
			|| currentLevel>=mpGraph->getPlanningLength())
	{
		mAllPossibleBatches.push_back(batch);
		return;
	}


	CLevelVertexSet levelSet = mpGraph->mpLevelSets[currentLevel];

	list<CLevelVertex*>::iterator itV;
	for(itV=levelSet.mSet.begin();itV!=levelSet.mSet.end();itV++)
	{
		CLevelVertex * pVertex = (*itV);

		if(mpGraph->isConnected((CVertex*)start,(CVertex*)pVertex))
		{
			depthFirstLimitedSearch(pVertex, batch, level+1);
		}
	}

}

double CRecedingHorizonPathPlanner::scoreBatch(CStepBatch & batch)
{
	CPath prevPath = mGeneratedPath;
	CPath newPath = mGeneratedPath;

	list<CLevelVertex*>::iterator it;
	for(it=batch.mBatch.begin();it!=batch.mBatch.end();it++)
	{
		CLevelVertex * vertex = (*it);
		newPath.addVertex(*vertex);
	}

	initObservedCells(prevPath);
	initObservedCells(newPath);

	double prevScore = scorePath(prevPath);
	double newScore = scorePath(newPath);
	batch.mReward = newScore-prevScore;

	return batch.mReward;
}

void CRecedingHorizonPathPlanner::scoreAllPossibleBatches()
{
	list<CStepBatch>::iterator it;

	for(it=mAllPossibleBatches.begin();it!=mAllPossibleBatches.end();it++)
	{
		scoreBatch(*it);
	}
}

CStepBatch CRecedingHorizonPathPlanner::getMaxStepBatch()
{
	list<CStepBatch>::iterator it;
	CStepBatch maxBatch;
	double maxBatchReward = -1;

	for(it=mAllPossibleBatches.begin();it!=mAllPossibleBatches.end();it++)
	{
		if((*it).mReward > maxBatchReward)
		{
			maxBatchReward = (*it).mReward;
			maxBatch = (*it);
		}
	}

	return maxBatch;
}
