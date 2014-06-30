/*
 * CBatchSearchPathPlanner.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#include "CBatchSearchPathPlanner.h"

#include <iostream>

using namespace std;

CBatchSearchPathPlanner::CBatchSearchPathPlanner(CPathPlanningGraph * graph, CAgent * agent)
    : CRobotPathPlanner(graph, agent)
{
	// TODO Auto-generated constructor stub
	mGeneratedPath.clear();
	mBatchCnt = 1;
}

CBatchSearchPathPlanner::~CBatchSearchPathPlanner() {
	// TODO Auto-generated destructor stub
	mGeneratedPath.clear();
}

bool CBatchSearchPathPlanner::doBatchSearch()
{
	mBenchmark.setStartTime();

	CLevelVertexSet levelSet = mpGraph->mpLevelSets[0];

	CLevelVertex * startVertex = NULL;

	startVertex = levelSet.mSet.front();

	doBatchSearch(startVertex);

	mBenchmark.setEndTime();

	cout << " BATCH " << mPlanStep << " Time spent " << mBenchmark.getTimeElasped() << endl;

}

bool CBatchSearchPathPlanner::doBatchSearch(CLevelVertex * startVertex)
{
	CLevelVertex * pCurrentVertex = startVertex;

	if(NULL==startVertex)
	{
		return false;
	}

	mGeneratedPath.clear();
	mGeneratedPath.addVertex(*pCurrentVertex);

	int currentCnt = mPlanStep-1;
	int i = 0;

	while(currentCnt>0)
	{
		CStepBatch stepBatch = findNextMaxStepBatch(pCurrentVertex);

		/*
		cout << " batch @" << i << " : ";
		stepBatch.print();
		cout << endl;
		*/

		if(stepBatch.size()==0)
		{
			return false;
		}

		pCurrentVertex = stepBatch.getLastStep();

		list<CLevelVertex*>::iterator it;
		for(it=stepBatch.mBatch.begin();it!=stepBatch.mBatch.end();it++)
		{
			CLevelVertex * pVertex = (*it);
			mGeneratedPath.addVertex(*pVertex);
		}

		currentCnt -= mBatchCnt;
		i++;
	}

	cout << "BATCH: " << endl;
	mGeneratedPath.printPath();
}

CStepBatch CBatchSearchPathPlanner::findNextMaxStepBatch(CLevelVertex * vertex)
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

void CBatchSearchPathPlanner::depthFirstLimitedSearch(CLevelVertex * start, CStepBatch batch, int level)
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

	if(level>mBatchCnt || currentLevel>=mpGraph->getPlanningLength())
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

double CBatchSearchPathPlanner::scoreBatch(CStepBatch & batch)
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

void CBatchSearchPathPlanner::scoreAllPossibleBatches()
{
	list<CStepBatch>::iterator it;

	for(it=mAllPossibleBatches.begin();it!=mAllPossibleBatches.end();it++)
	{
		scoreBatch(*it);
	}
}

CStepBatch CBatchSearchPathPlanner::getMaxStepBatch()
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
