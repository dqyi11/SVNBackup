/*
 * CPathPlanningGraphPruner.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: walter
 */

#include "CPathPlanningGraphPruner.h"
#include <iostream>

using namespace std;

CPathPlanningGraphPruner::CPathPlanningGraphPruner(CPathPlanningGraph * graph) {
	// TODO Auto-generated constructor stub
	mpPathPlanningGraph = graph;
	mStartVertexName = "";
	mpVisualizer = NULL;
	mpMap = NULL;
}

CPathPlanningGraphPruner::~CPathPlanningGraphPruner() {
	// TODO Auto-generated destructor stub
	mpPathPlanningGraph = NULL;
	mpMap = NULL;

	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}
}

bool CPathPlanningGraphPruner::pruneGraph()
{
	if(false==pruneByStartVertex())
	{
		cout << "pruneByStartVertex" << endl;
		return false;
	}

	if(false==pruneByObstacle())
	{
		cout << "pruneByObstacle" << endl;
		return false;
	}

	if(false==pruneForward())
	{
		cout << "pruneForward" << endl;
		return false;
	}

	if(false==pruneBackward())
	{
		cout << "pruneBackward" << endl;
		return false;
	}

	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}

	mpVisualizer = new CPathPlanningGraphVisualizer(mpPathPlanningGraph);

	cout << " finish prunning " << endl;
}

bool CPathPlanningGraphPruner::pruneByStartVertex()
{
	if(NULL==mpPathPlanningGraph)
	{
		return false;
	}

	CLevelVertex * pVertex = mpPathPlanningGraph->findVertex(mStartVertexName, 0);
	if(pVertex)
	{
		mpPathPlanningGraph->removeAllOtherVertex(mStartVertexName, 0);
	}

	return true;
}


bool CPathPlanningGraphPruner::pruneByObstacle()
{
	if(NULL==mpPathPlanningGraph || NULL==mpMap)
	{
		return false;
	}

    int planLength = mpPathPlanningGraph->getPlanningLength();
    for(int i=1;i<planLength;i++)
    {
    	// cout << "doing " << i << endl;

		CLevelVertexSet levelSet = mpPathPlanningGraph->mpLevelSets[i];

		list<CLevelVertex*>::iterator itV;
		for(itV=levelSet.mSet.begin();itV!=levelSet.mSet.end();itV++)
		{
			CLevelVertex * pVertex = (*itV);



 			int posX = pVertex->mpHexaVertex->mPosX;
			int posY = pVertex->mpHexaVertex->mPosY;

			// cout << " checking " << posX << "+" << posY << endl;

			CGrid * grid = mpMap->getGrid(posX, posY);
			if(grid && grid->mType==OBSTACLE)
			{
				mpPathPlanningGraph->removeVertex(pVertex->mName, pVertex->mLevel);
			}

		}


    }

    return true;
}

bool CPathPlanningGraphPruner::pruneForward()
{
	/* any vertex cannot be reached from a vertex in last level
	 * will be pruned.
	 */
	if(NULL==mpPathPlanningGraph)
	{
		return false;
	}

    int planLength = mpPathPlanningGraph->getPlanningLength();
    for(int i=1;i<planLength;i++)
    {
    	list<CLevelVertex*>::iterator itV;
    	{
    		CLevelVertexSet levelSet = mpPathPlanningGraph->mpLevelSets[i];

    		for(itV=levelSet.mSet.begin();itV!=levelSet.mSet.end();itV++)
    		{
    			CLevelVertex * pVertex = (*itV);

    			CEdgeSet edgeSet = mpPathPlanningGraph->findEdgeByEnd(pVertex);
    			if(edgeSet.mSet.size()==0)
    			{
    				mpPathPlanningGraph->removeVertex(pVertex->mName, pVertex->mLevel);
    			}

    		}
    	}

    }

    //cout << " finish forward prune " << endl;

    return true;
}


bool CPathPlanningGraphPruner::pruneBackward()
{
	/* any vertex cannot reach to a vertex in next level
	 * will be pruned.
	 */
	if(NULL==mpPathPlanningGraph)
	{
		return false;
	}

	int planLength = mpPathPlanningGraph->getPlanningLength();
	for(int i=planLength-2;i>=0;i--)
	{
    	list<CLevelVertex*>::iterator itV;
    	{
    		for(itV=mpPathPlanningGraph->mpLevelSets[i].mSet.begin();itV!=mpPathPlanningGraph->mpLevelSets[i].mSet.end();)
    		{
    			CLevelVertex * pVertex = (*itV);

    			CEdgeSet edgeSet = mpPathPlanningGraph->findEdgeByStart(pVertex);
    			if(edgeSet.mSet.size()==0)
    			{
    				itV++;
    				mpPathPlanningGraph->removeVertex(pVertex->mName, pVertex->mLevel);
    			}
    			else
    			{
    				itV++;
    			}
    		}
    	}
	}

	cout << " finish back ward prune " << endl;

	return true;
}

void CPathPlanningGraphPruner::visualizeGraph(const char * filename)
{
	if(mpVisualizer)
	{
		cout << "drawing " << filename << endl;
		mpVisualizer->draw(filename);
	}
}
