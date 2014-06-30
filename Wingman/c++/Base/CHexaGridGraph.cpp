/*
 * CHexaGridGraph.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: walter
 */

#include "CHexaGridGraph.h"
#include <iostream>

using namespace std;

CHexaVertex::CHexaVertex()
   : CVertex()
{
	mPosX = 0;
	mPosY = 0;
	mIndex = 0;
}

CHexaVertex::~CHexaVertex()
{

}

CHexaGridGraph::CHexaGridGraph()
   : CGraph()
{
	// TODO Auto-generated constructor stub

}

CHexaGridGraph::~CHexaGridGraph()
{
	// TODO Auto-generated destructor stub
}

CHexaVertex * CHexaGridGraph::createVertex(string name)
{
	CVertex * pVertex = NULL;
	pVertex = CGraph::findVertex(name);

	if(pVertex)
	{
		return (CHexaVertex *)pVertex;
	}

	pVertex = new CHexaVertex();
	pVertex->mName = name;

	addVertex(pVertex);

	return (CHexaVertex *)pVertex;
}

CHexaVertex * CHexaGridGraph::findVertex(int posX, int posY)
{
	list<CVertex*>::iterator it;

	for(it=mVertexList.begin(); it!=mVertexList.end(); it++)
	{
		CHexaVertex * pVertex = (CHexaVertex *)(*it);
		if(pVertex->mPosX==posX && pVertex->mPosY==posY)
		{
			return pVertex;
		}
	}

	return NULL;
}

CHexaVertex * CHexaGridGraph::findVertex(string name)
{
	list<CVertex*>::iterator it;

	for(it=mVertexList.begin(); it!=mVertexList.end(); it++)
	{
		CHexaVertex * pVertex = (CHexaVertex *)(*it);
		if(pVertex->mName.compare(name)==0)
		{
			return pVertex;
		}
	}

	return NULL;
}

bool CHexaGridGraph::isConnected(CHexaVertex * a, CHexaVertex * b)
{
	if(a==NULL || b==NULL)
	{
		return false;
	}

	CVertex * pA = (CVertex*)a;
	CVertex * pB = (CVertex*)b;

	// assume each node is connected with itself
	if(pA==pB)
	{
		return true;
	}

    list<CEdge*>::iterator it;
    for(it=mEdgeList.begin();it!=mEdgeList.end();it++)
    {
    	if( ((*it)->mpVertexA==pA && (*it)->mpVertexB==pB)
    			||
    			((*it)->mpVertexA==pB && (*it)->mpVertexB==pA) )
    	{
    		return true;
    	}
    }

    return false;
}
