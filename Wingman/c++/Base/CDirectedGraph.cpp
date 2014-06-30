/*
 * CDirectedGraph.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: walter
 */

#include "CDirectedGraph.h"

#include <iostream>

using namespace std;

CDirectedEdge::CDirectedEdge() {
	mFromAToB = true;
}

CDirectedEdge::~CDirectedEdge()
{
	mFromAToB = true;
}

CDirectedGraph::CDirectedGraph() {
	// TODO Auto-generated constructor stub

}

CDirectedGraph::~CDirectedGraph() {
	// TODO Auto-generated destructor stub
}

CDirectedEdge * CDirectedGraph::findEdge(CVertex * vertexStart, CVertex * vertexEnd)
{
	list<CEdge*>::iterator it;
	CDirectedEdge * pDEdge = NULL;

	for(it=mEdgeList.begin(); it!=mEdgeList.end(); it++)
	{
		pDEdge = (CDirectedEdge *)(*it);

		if( (pDEdge->mpVertexA==vertexStart && pDEdge->mpVertexB==vertexEnd && pDEdge->mFromAToB==true)
				||
				(pDEdge->mpVertexA==vertexEnd && pDEdge->mpVertexB==vertexStart && pDEdge->mFromAToB==false) )
		{
			return pDEdge;
		}
	}

	return NULL;
}

CDirectedEdge * CDirectedGraph::addEdge(string idStart, string idEnd)
{
	CVertex * pVertexStart = findVertex(idStart);
	CVertex * pVertexEnd = findVertex(idEnd);

	return addEdge(pVertexStart, pVertexEnd);
}

CDirectedEdge * CDirectedGraph::addEdge(CVertex * vertexStart, CVertex * vertexEnd)
{
	if(vertexStart==NULL || vertexEnd==NULL)
	{
		return NULL;
	}

	CDirectedEdge * pDEdge = NULL;

	pDEdge = findEdge(vertexStart, vertexEnd);

	if(pDEdge==NULL)
	{
		pDEdge = new CDirectedEdge();
		pDEdge->mpVertexA = vertexStart;
		pDEdge->mpVertexB = vertexEnd;

		CGraph::addEdge((CEdge *)pDEdge);
	}

	return pDEdge;
}

bool CDirectedGraph::removeVertex(string id)
{
	CVertex * pVertex = findVertex(id);

	if(NULL==pVertex)
	{
		cout << id << " not found " << endl;
 		return false;
	}

	// remove edges connect to pVertex
	list<CEdge*>::iterator itE;
	CEdge * pDEdge = NULL;

	for(itE=mEdgeList.begin(); itE!=mEdgeList.end();)
	{
		pDEdge = (*itE);

		if((pDEdge->mpVertexA==pVertex)
			||
		   (pDEdge->mpVertexB==pVertex))
		{
			itE = mEdgeList.erase(itE);
			delete pDEdge;
			pDEdge = NULL;
		}
		else
		{
			itE++;
		}
	}

	// cout << " mEdgeList num is " << mEdgeList.size() << endl;

	// remove vertex from list
	list<CVertex*>::iterator itV;
	for(itV=mVertexList.begin();itV!=mVertexList.end();)
	{
		if((*itV)==pVertex)
		{
			itV = mVertexList.erase(itV);
			delete pVertex;
			pVertex = NULL;
		}
		else
		{
			itV++;
		}

	}

	// cout << " mVertexList num is " << mVertexList.size() << endl;

	return true;
}

bool CDirectedGraph::isConnected(CVertex * vertexStart, CVertex * vertexEnd)
{
	if(NULL==vertexStart || NULL==vertexEnd)
	{
		return false;
	}

	list<CEdge*>::iterator itE;
	CDirectedEdge * pDEdge = NULL;

	//cout << " checking connection " << vertexStart->mName << " and " << vertexEnd->mName << endl;

	for(itE=mEdgeList.begin(); itE!=mEdgeList.end();itE++)
	{
		pDEdge = (CDirectedEdge * )(*itE);

		if((pDEdge->mpVertexA==vertexStart &&
				pDEdge->mpVertexB==vertexEnd &&
				true==pDEdge->mFromAToB)
				||
			(pDEdge->mpVertexA==vertexEnd &&
				pDEdge->mpVertexB==vertexStart &&
				false==pDEdge->mFromAToB))
		{
			return true;
		}
	}

	return false;
}
