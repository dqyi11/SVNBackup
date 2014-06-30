/*
 * CExhaustiveSearchPathPlanner.cpp
 *
 *  Created on: Jan 20, 2013
 *      Author: walter
 */

#include "CExhaustiveSearchPathPlanner.h"
#include <iostream>

using namespace std;

CExhaustiveSearchPathPlanner::CExhaustiveSearchPathPlanner(CPathPlanningGraph * graph, CAgent * agent)
    : CRobotPathPlanner(graph, agent)
{
	// TODO Auto-generated constructor stub

}

CExhaustiveSearchPathPlanner::~CExhaustiveSearchPathPlanner() {
	// TODO Auto-generated destructor stub

}

void CExhaustiveSearchPathPlanner::generateAllPossiblePath()
{
	if(mpGraph)
	{
		cout << "generateAllPossiblePath " << mPlanStep << endl;
		CLevelVertexSet levelSet = mpGraph->mpLevelSets[0];
		list<CLevelVertex*>::iterator it;
		for(it=levelSet.mSet.begin();it!=levelSet.mSet.end();it++)
		{
			CPath path;
			CLevelVertex * pVertex = (*it);
			//path.mVertexSeq.push_back(*pVertex);
			depthFirstSearch(*it, path, 1);
		}

	}

}

void CExhaustiveSearchPathPlanner::depthFirstSearch(CLevelVertex * start, CPath path, int level)
{
	if(NULL==mpGraph)
	{
		cout << " ERROR: graph is empty " << endl;
		return;
	}

	path.mVertexSeq.push_back(*start);

	if(level>=mPlanStep)
	{
		// cout << " at finish level " << endl;
		// path.print();
		mAllPathList.push_back(path);
		return;
	}

	CLevelVertexSet levelSet = mpGraph->mpLevelSets[level];

	// cout << "DFS at level " << level << endl;

	list<CLevelVertex*>::iterator itV;
	for(itV=levelSet.mSet.begin();itV!=levelSet.mSet.end();itV++)
	{
		CLevelVertex * pVertex = (*itV);

		if(mpGraph->isConnected((CVertex*)start,(CVertex*)pVertex))
		{
			//cout << " find " << pVertex->mId << endl;

			//path.print();
			depthFirstSearch(pVertex, path, level+1);
		}
	}

	return;
}

void CExhaustiveSearchPathPlanner::scoreAllPossiblePath()
{
	list<CPath>::iterator it;
	double score = 0;
	int length = mAllPathList.size();
	int i = 0;
	for(it=mAllPathList.begin();it!=mAllPathList.end();it++)
	{
		initObservedCells(*it);
		/*
        cout << "scoring " << i << "/" << length << endl;
		i++;
		*/
		//(*it).printCellObsHist();
		score = scorePath(*it);
		//cout << " score " << score << endl;
	}

}
