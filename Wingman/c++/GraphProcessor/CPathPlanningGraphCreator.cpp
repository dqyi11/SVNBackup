/*
 * CPathPlanningGraphCreator.cpp
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#include "CPathPlanningGraphCreator.h"
#include <sstream>
#include <iostream>

using namespace std;

CPathPlanningGraphCreator::CPathPlanningGraphCreator()
{
	// TODO Auto-generated constructor stub
	mpMap = NULL;
	mpGridGraph = NULL;
	mpPathPlanningGraph = NULL;
	mpHuman = NULL;

	mpVisualizer = NULL;

}

CPathPlanningGraphCreator::~CPathPlanningGraphCreator()
{
	// TODO Auto-generated destructor stub
	mpMap = NULL;
	mpGridGraph = NULL;
	mpHuman = NULL;

	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}


	if(mpPathPlanningGraph)
	{
		delete mpPathPlanningGraph;
		mpPathPlanningGraph = NULL;
	}

	mPlanningTimeLength = 0;
}

bool CPathPlanningGraphCreator::init()
{
	if(mpMap==NULL || mpGridGraph==NULL || mpHuman==NULL)
	{
		return false;
	}

	vector<CGrid> visitedGrids = mpHuman->mGridsVisited;

	if(visitedGrids.size()==0)
	{
		return false;
	}

	mPlanningTimeLength = visitedGrids.size();

	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}

	if(mpPathPlanningGraph)
	{
		delete mpPathPlanningGraph;
		mpPathPlanningGraph = NULL;
	}

	mpPathPlanningGraph = new CPathPlanningGraph(mPlanningTimeLength);

	vector<CGridSet> levelSet;

	vector<CGrid>::iterator it;
	for(it=visitedGrids.begin();it!=visitedGrids.end();it++)
	{
		//cout << " doing " << (*it).mX <<"," << (*it).mY << endl;
 		CGridSet set = mpMap->getGridSet((*it).mX,(*it).mY,mpHuman->mWingmanRadius, false);
		// set.print();
		levelSet.push_back(set);
	}

	stringstream sstream;
	int count = 0;
	vector<CGridSet>::iterator itS;
	list<CGrid*>::iterator itG;
	for(itS=levelSet.begin();itS!=levelSet.end();itS++)
	{
		CGridSet set = (*itS);
		for(itG=set.mSet.begin();itG!=set.mSet.end();itG++)
		{
			CVisualHexagon * hexagon = &(mpMap->mpVisualHexagons[(*itG)->mX][(*itG)->mY]);

			int index = hexagon->getIndex();
			if(index!=-1)
			{
				string name = "";
				sstream << hexagon->getIndex(); //<< "@"<<count;
				sstream >> name;
				sstream.clear();

				//cout << " creating " << name << endl;

				CLevelVertex * vertex = mpPathPlanningGraph->createVertex(name, count);
				if(vertex)
				{
					CHexaVertex * hexa = mpGridGraph->findVertex(name);
					if(hexa==NULL)
					{
						cout << " pos is " << (*itG)->mX << " and " << (*itG)->mY << endl;
						cout << "init error " << name << endl;
					}
					vertex->mpHexaVertex = hexa;
				}
			}
		}

		count++;
	}


	for(int i=1;i<mPlanningTimeLength;i++)
	{
		CLevelVertexSet lastLevel = mpPathPlanningGraph->mpLevelSets[i-1];
		CLevelVertexSet currentLevel = mpPathPlanningGraph->mpLevelSets[i];

		list<CLevelVertex*>::iterator itLast;
		list<CLevelVertex*>::iterator itCurrent;

		for(itLast=lastLevel.mSet.begin();itLast!=lastLevel.mSet.end();itLast++)
		{
			for(itCurrent=currentLevel.mSet.begin();itCurrent!=currentLevel.mSet.end();itCurrent++)
			{
				CHexaVertex * from = (*itLast)->mpHexaVertex;
				CHexaVertex * to = (*itCurrent)->mpHexaVertex;
				// cout << " checking connection for " << from->mName << " and " << to->mName << endl;
				if(mpGridGraph->isConnected(from, to))
				{
					// cout << "adding edge for " << from->mName << " and " << to->mName << endl;
					CDirectedEdge * edge = mpPathPlanningGraph->addEdge((*itLast), (*itCurrent));
				}
			}
		}

	}

	mpVisualizer = new CPathPlanningGraphVisualizer(mpPathPlanningGraph);

}

void CPathPlanningGraphCreator::visualizeGraph(const char * filename)
{
	if(mpVisualizer)
	{
		mpVisualizer->draw(filename);
	}
}
