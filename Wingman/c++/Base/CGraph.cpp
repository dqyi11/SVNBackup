/*
 * CGraph.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: walter
 */

#include "CGraph.h"
#include <iostream>

using namespace std;

CVertex::CVertex()
{
	mName = "";
}

CVertex::~CVertex()
{
	mName = "";
}

CEdge::CEdge()
{
	mpVertexA = NULL;
	mpVertexB = NULL;
}

CEdge::~CEdge()
{
	mpVertexA = NULL;
	mpVertexB = NULL;
}

CEdgeSet::CEdgeSet()
{
	mSet.clear();
}

CEdgeSet::~CEdgeSet()
{
	mSet.clear();
}

void CEdgeSet::addEdge(CEdge * pEdge)
{
	if(false==hasEdge(pEdge))
	{
		mSet.push_back(pEdge);
	}

}

bool CEdgeSet::hasEdge(CEdge * pEdge)
{
	list<CEdge*>::iterator itE;
	for(itE=mSet.begin();itE!=mSet.end();itE++)
	{
		if((*itE)==pEdge)
		{
			return true;
		}
	}

	return false;
}

CGraph::CGraph() {
	// TODO Auto-generated constructor stub

	mVertexList.clear();
	mEdgeList.clear();

}

CGraph::~CGraph() {
	// TODO Auto-generated destructor stub
	list<CEdge *>::iterator it;
	for(it=mEdgeList.begin(); it!=mEdgeList.end(); it++)
	{
		delete (*it);
		(*it) = NULL;
	}

	mEdgeList.clear();

	list<CVertex *>::iterator it2;
	for(it2=mVertexList.begin(); it2!=mVertexList.end(); it2++)
	{
		delete (*it2);
		(*it2) = NULL;
	}

	mVertexList.clear();
}

CVertex * CGraph::createVertex(string name)
{
	CVertex * pVertex = NULL;
	// assume name is identical with id
	pVertex = findVertex(name);

	if(pVertex)
	{
		return pVertex;
	}

	pVertex = new CVertex();
	pVertex->mName = name;
	pVertex->mId = name;

	addVertex(pVertex);

	return pVertex;
}

CVertex * CGraph::findVertex(string id)
{
	list<CVertex*>::iterator it;

	for(it=mVertexList.begin(); it!=mVertexList.end(); it++)
	{
		if((*it)->mId.compare(id)==0)
		{
			return (*it);
		}
	}

	return NULL;
}

CEdge * CGraph::addEdge(string idA, string idB)
{
	CVertex * pVertexA = findVertex(idA);
	CVertex * pVertexB = findVertex(idB);

	return addEdge(pVertexA, pVertexB);
}

CEdge * CGraph::addEdge(CVertex * vertexA, CVertex * vertexB)
{
	if(vertexA==NULL || vertexB==NULL)
	{
		return NULL;
	}

	CEdge * pEdge = NULL;

	pEdge = findEdge(vertexA, vertexB);

	if(pEdge==NULL)
	{
		pEdge = new CEdge();
		pEdge->mpVertexA = vertexA;
		pEdge->mpVertexB = vertexB;

		addEdge(pEdge);
	}

	return pEdge;
}

CEdge * CGraph::findEdge(CVertex * vertexA, CVertex * vertexB)
{
	list<CEdge*>::iterator it;

	for(it=mEdgeList.begin(); it!=mEdgeList.end(); it++)
	{
		if( ((*it)->mpVertexA==vertexA && (*it)->mpVertexB==vertexB)
				||
				((*it)->mpVertexA==vertexB && (*it)->mpVertexB==vertexA) )
		{
			return (*it);
		}
	}

	return NULL;
}

void CGraph::printVertex()
{
	list<CVertex *>::iterator it;
	cout << "PRINTING VERTEX ... " << endl;
	for(it=mVertexList.begin();it!=mVertexList.end();it++)
	{
		cout << "(" << (*it)->mName << ")" << " ";
	}

}

void CGraph::printEdge()
{
	list<CEdge *>::iterator it;
	cout << "PRINTING EDGE ... " << endl;
	for(it=mEdgeList.begin();it!=mEdgeList.end();it++)
	{
		cout << " LINK FROM " << (*it)->mpVertexA->mName << " TO " << (*it)->mpVertexB->mName << endl;
	}
}
