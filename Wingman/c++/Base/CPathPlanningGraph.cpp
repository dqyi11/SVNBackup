/*
 * CPathPlanningGraph.cpp
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#include "CPathPlanningGraph.h"

#include <iostream>
#include <sstream>

using namespace std;

CLevelVertex::CLevelVertex()
{
	mLevel = -1;
	mpHexaVertex = NULL;
}

CLevelVertex::~CLevelVertex()
{
	mLevel = -1;
	mpHexaVertex = NULL;
}

CLevelVertexSet::CLevelVertexSet()
{
	mLevel = -1;
	mSet.clear();
}

CLevelVertexSet::~CLevelVertexSet()
{
	mLevel = -1;
	mSet.clear();
}

void CLevelVertexSet::addLevelVertex(CLevelVertex * levelVertex)
{
	if(hasLevelVertex(levelVertex))
	{
		return;
	}

	mSet.push_back(levelVertex);
}

bool CLevelVertexSet::hasLevelVertex(CLevelVertex * levelVertex)
{
	list<CLevelVertex*>::iterator it;
	for(it=mSet.begin();it!=mSet.end();it++)
	{
		if((*it)==levelVertex)
		{
			return true;
		}
	}

	return false;
}

CPathPlanningGraph::CPathPlanningGraph(int planningLength)
   : CDirectedGraph()
{
	// TODO Auto-generated constructor stub
	mPlanningLength = planningLength;
	mpLevelSets = new CLevelVertexSet [mPlanningLength];
	for(int i=0;i<mPlanningLength;i++)
	{
		mpLevelSets[i].setLevel(i);
	}
}

CPathPlanningGraph::~CPathPlanningGraph()
{
	// TODO Auto-generated destructor stub
	if(mpLevelSets)
	{
		delete [] mpLevelSets;
		mpLevelSets = NULL;
	}
}

bool CPathPlanningGraph::addVertex(CLevelVertex * vertex, int level)
{
	if(level>=mPlanningLength)
	{
		return false;
	}

	mpLevelSets[level].addLevelVertex(vertex);

	CDirectedGraph::addVertex((CVertex*)vertex);

	return true;
}

CLevelVertex * CPathPlanningGraph::createVertex(string name, int level)
{
	stringstream sstream;
	sstream << name << "@" << level;
	CLevelVertex * vertex = new CLevelVertex();
	vertex->mName = name;
	sstream >> vertex->mId;
	vertex->mLevel = level;

	sstream.clear();

	if(false==addVertex(vertex, level))
	{
		delete vertex;
		vertex = NULL;

		return NULL;
	}

	return vertex;
}

CDirectedEdge * CPathPlanningGraph::addEdge(CLevelVertex * vertexStart, CLevelVertex * vertexEnd)
{
	if(vertexStart==NULL || vertexEnd==NULL)
	{
		return NULL;
	}

	return CDirectedGraph::addEdge((CVertex*)vertexStart, (CVertex*)vertexEnd);
}

CLevelVertex * CPathPlanningGraph::findVertex(string name, int level)
{
	CLevelVertex * pVertex = NULL;
	if(level<0 || level>=mPlanningLength)
	{
		return NULL;
	}

	CLevelVertexSet vertexSet = mpLevelSets[level];

	list<CLevelVertex*>::iterator itV;
	for(itV=vertexSet.mSet.begin();itV!=vertexSet.mSet.end();itV++)
	{
		if(0==(*itV)->mName.compare(name))
		{
			pVertex = (*itV);
			return pVertex;
		}
	}

	return pVertex;
}

bool CPathPlanningGraph::removeVertex(string name, int level)
{
	// cout << "CPathPlanningGraph::removeVertex " << name << "@" << level << endl;
	CLevelVertex * pVertex = findVertex(name, level);

	if(NULL == pVertex)
	{
		return false;
	}

	list<CLevelVertex*>::iterator itV;
	for(itV=mpLevelSets[level].mSet.begin();itV!=mpLevelSets[level].mSet.end();)
	{
		if(0==(*itV)->mName.compare(name))
		{
			itV = mpLevelSets[level].mSet.erase(itV);
		}
		else
		{
			itV++;
		}
	}

	CDirectedGraph::removeVertex(pVertex->mId);

	return true;
}

bool CPathPlanningGraph::removeAllOtherVertex(string name, int level)
{
	// cout << "CPathPlanningGraph::removeAllOtherVertex " << name << "@" << level << endl;
	CLevelVertex * pVertex = findVertex(name, level);
	CLevelVertex * pVertexToRemove = NULL;

	if(NULL == pVertex)
	{
		return false;
	}

	list<CLevelVertex*>::iterator itV;
	for(itV=mpLevelSets[level].mSet.begin();itV!=mpLevelSets[level].mSet.end();)
	{
		if(0!=(*itV)->mName.compare(name))
		{
			pVertexToRemove = (*itV);
			string vertexToRemoveId = pVertexToRemove->mId;

			itV = mpLevelSets[level].mSet.erase(itV);

			CDirectedGraph::removeVertex(vertexToRemoveId);

		}
		else
		{
			itV++;
		}
	}

	return true;
}

CEdgeSet CPathPlanningGraph::findEdgeByStart(CLevelVertex * vertexStart)
{
	CDirectedEdge * pEdge = NULL;
	CEdgeSet edgeSet;
	list<CEdge*>::iterator itE;

	for(itE=mEdgeList.begin();itE!=mEdgeList.end();itE++)
	{
		pEdge = (CDirectedEdge*)(*itE);

		if((pEdge->mpVertexA==(CVertex*)vertexStart && pEdge->mFromAToB==true)
				||
				(pEdge->mpVertexB==(CVertex*)vertexStart && pEdge->mFromAToB==false))
		{
			edgeSet.addEdge(*itE);
		}
	}

	return edgeSet;
}

CEdgeSet CPathPlanningGraph::findEdgeByEnd(CLevelVertex * vertexEnd)
{
	CDirectedEdge * pEdge = NULL;
	CEdgeSet edgeSet;
	list<CEdge*>::iterator itE;

	for(itE=mEdgeList.begin();itE!=mEdgeList.end();itE++)
	{
		pEdge = (CDirectedEdge*)(*itE);

		if((pEdge->mpVertexA==(CVertex*)vertexEnd && pEdge->mFromAToB==false)
				||
				(pEdge->mpVertexB==(CVertex*)vertexEnd && pEdge->mFromAToB==true))
		{
			edgeSet.addEdge(*itE);
		}
	}

    return edgeSet;
}

void CPathPlanningGraph::printLevelVertex()
{
	cout << " LISTING vertex ... " << endl;
	list<CLevelVertex*>::iterator itV;

	cout << " planning length " << mPlanningLength << endl;

	for(int i=0;i<mPlanningLength;i++)
	{
		cout << "LEVEL " << mpLevelSets[i].mLevel << " size " << mpLevelSets[i].mSet.size() << endl;

		for(itV=mpLevelSets[i].mSet.begin();itV!=mpLevelSets[i].mSet.end();itV++)
		{
			if(i==0)
			cout << (*itV)->mName << "@" << (*itV)->mLevel << endl;
		}

	}

}
